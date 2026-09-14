//! GDB 远程调试服务器（调试平台 P2-3）。
//!
//! 自研 GDB Remote Serial Protocol（RSP）TCP 服务端子集——使模拟器可被
//! `arm-none-eabi-gdb` 直接连接调试（类 Renode/Renegade）：
//! - 寄存器读写（`g`/`G`，Cortex-M4 23 寄存器布局）
//! - 内存读写（`m`/`M`，hex 编码）
//! - 单步/继续（`s`/`c`，继续跑到断点或预算上限）
//! - 软断点（`Z0`/`z0`，PC 命中停 → SIGTRAP）
//! - 目标特性协商（`qSupported`/`qC`/`qAttached`）、中断原因查询（`?`）
//!
//! 用法：
//! ```rust
//! let handle = GdbServer::new().spawn(3333, || {
//!     let mut m = Machine::new_m4f().unwrap();
//!     m.map_stm32f407_layout().unwrap();
//!     m.load_elf(&elf).unwrap();
//!     m
//! })?;
//! // arm-none-eabi-gdb firmware.elf
//! // (gdb) target remote :3333
//! ```
//!
//! 线程模型：Machine 含 Unicorn（`Rc` 内部）非 Send，不能跨线程——`spawn` 的
//! 机器工厂在**服务线程内**执行，每连接一个独立 Machine 实例。
//!
//! 限制（fidelity 边界，见 docs）：`c` 在无断点时跑固定预算后停（单线程无法
//! 响应 Ctrl-C 中断）；不做内存/寄存器差分同步（GDB 本地直接读符号表）。

use std::collections::HashSet;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};

use unicorn_engine::RegisterARM;

use crate::machine::Machine;

/// 单步预算（退休字节；~500 ≈ 微秒级）。
const STEP_BUDGET: usize = 500;
/// 继续无断点时最多跑的段数（防死循环；返回 SIGINT）。
/// 实际调试中正常 `c` 都靠断点停；预算上限只是护栏（每段 ~500 退休）。
const CONTINUE_MAX_SEG: u64 = 5_000;

/// GDB 服务器：按 RSP 命令驱动仿真（断点集；Machine 由调用方/服务线程持有）。
pub struct GdbServer {
    /// 软断点地址集合（PC 命中即停）。
    breaks: HashSet<u32>,
}

impl GdbServer {
    pub fn new() -> Self {
        Self {
            breaks: HashSet::new(),
        }
    }

    /// 后台启动监听（每连接在服务线程内建一个 Machine 实例并服务；返回线程句柄）。
    ///
    /// `factory` 在服务线程内执行（Machine 非 Send，见模块注释）。
    pub fn spawn(
        port: u16,
        machine_factory: impl Fn() -> Machine + Send + 'static,
    ) -> std::io::Result<std::thread::JoinHandle<()>> {
        let listener = TcpListener::bind(("127.0.0.1", port))?;
        log::info!("GDB server 监听 127.0.0.1:{port}");
        Ok(std::thread::spawn(move || {
            for conn in listener.incoming() {
                match conn {
                    Ok(stream) => {
                        log::info!("GDB 连接建立");
                        let mut machine = machine_factory();
                        let mut srv = GdbServer::new();
                        srv.serve(stream, &mut machine);
                        log::info!("GDB 连接关闭");
                    }
                    Err(e) => log::warn!("GDB 连接错误: {e}"),
                }
            }
        }))
    }

    /// 单连接 RSP 循环（调用者线程内驱动 `machine`）。
    pub fn serve(&mut self, mut stream: TcpStream, machine: &mut Machine) {
        let mut byte = [0u8; 1];
        let mut pkt: Vec<u8> = Vec::new();
        loop {
            match stream.read(&mut byte) {
                Ok(0) | Err(_) => break, // 断开
                Ok(_) => {}
            }
            match byte[0] {
                b'+' | b'-' => {} // 对端 ACK/NAK（响应不等待，直接发下一条）
                b'$' => {
                    // 收数据到 '#'，校验和忽略（仿真环境宽松）
                    pkt.clear();
                    loop {
                        if stream.read(&mut byte).is_err() {
                            return;
                        }
                        if byte[0] == b'#' {
                            break;
                        }
                        pkt.push(byte[0]);
                    }
                    // 吃掉 2 位校验和
                    let mut cs = [0u8; 2];
                    if stream.read(&mut cs).is_err() {
                        return;
                    }
                    let cmd = String::from_utf8_lossy(&pkt).into_owned();
                    let resp = self.handle_packet(&cmd, machine);
                    // 发响应（ACK 由对端发；我们忽略）
                    let mut out = Vec::with_capacity(resp.len() + 4);
                    out.push(b'$');
                    out.extend_from_slice(resp.as_bytes());
                    out.push(b'#');
                    let sum: u8 = resp.bytes().fold(0, |a, b| a.wrapping_add(b));
                    out.extend_from_slice(format!("{sum:02x}").as_bytes());
                    if stream.write_all(&out).is_err() {
                        return;
                    }
                }
                _ => {}
            }
        }
    }

    /// 处理单条 RSP 命令，返回响应（不含 $# 包裹）。
    fn handle_packet(&mut self, cmd: &str, machine: &mut Machine) -> String {
        let c = cmd.as_bytes();
        if c.is_empty() {
            return String::new();
        }
        match c[0] {
            b'?' => "S05".into(), // 停止原因：SIGTRAP
            b'g' => self.cmd_read_regs(machine),
            b'G' => self.cmd_write_regs(&cmd[1..], machine),
            b'm' => self.cmd_read_mem(&cmd[1..], machine),
            b'M' => self.cmd_write_mem(&cmd[1..], machine),
            b'c' => {
                // c [addr]：跑到断点或预算上限
                let target = if cmd.len() > 1 {
                    hex_to_u32(&cmd[1..])
                } else {
                    None
                };
                self.cmd_continue(target, machine)
            }
            b's' => {
                machine.run(STEP_BUDGET).unwrap_or_default();
                "S05".into()
            }
            b'Z' => {
                // Z0,addr,kind
                self.set_break(&cmd[1..], true);
                "OK".into()
            }
            b'z' => {
                self.set_break(&cmd[1..], false);
                "OK".into()
            }
            b'k' => String::new(), // kill：空响应 + 对端断开
            b'q' => self.cmd_query(&cmd[1..]),
            _ => String::new(), // 未知命令：空响应
        }
    }

    // ---------------- 命令实现 ----------------

    fn cmd_read_regs(&mut self, m: &mut Machine) -> String {
        let mut out = String::with_capacity(23 * 8);
        for &r in cortex_m_regs() {
            let v = m.cpu.reg_read_u32(r).unwrap_or(0);
            for b in v.to_le_bytes() {
                out.push_str(&format!("{b:02x}"));
            }
        }
        out
    }

    fn cmd_write_regs(&mut self, hex: &str, m: &mut Machine) -> String {
        let regs = cortex_m_regs();
        let bytes = match hex_to_bytes(hex) {
            Some(b) => b,
            None => return "E01".into(),
        };
        // 每 4 字节一个寄存器（小端）
        for (i, r) in regs.iter().enumerate() {
            let off = i * 4;
            if off + 4 > bytes.len() {
                break;
            }
            let v = u32::from_le_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]]);
            let _ = m.cpu.reg_write(*r, v as u64);
        }
        "OK".into()
    }

    fn cmd_read_mem(&mut self, args: &str, m: &mut Machine) -> String {
        // m addr,len
        let (addr, len) = match parse_addr_len(args) {
            Some(v) => v,
            None => return "E01".into(),
        };
        match m.cpu.mem_read(addr as u64, len as usize) {
            Ok(buf) => buf.iter().map(|b| format!("{b:02x}")).collect(),
            Err(_) => "E02".into(),
        }
    }

    fn cmd_write_mem(&mut self, args: &str, m: &mut Machine) -> String {
        // M addr,len:data
        let (head, data_hex) = match args.split_once(':') {
            Some(v) => v,
            None => return "E01".into(),
        };
        let (addr, len) = match parse_addr_len(head) {
            Some(v) => v,
            None => return "E01".into(),
        };
        let data = match hex_to_bytes(data_hex) {
            Some(d) if d.len() == len as usize => d,
            _ => return "E01".into(),
        };
        match m.cpu.mem_write(addr as u64, &data) {
            Ok(()) => "OK".into(),
            Err(_) => "E02".into(),
        }
    }

    fn cmd_continue(&mut self, target: Option<u32>, m: &mut Machine) -> String {
        if let Some(addr) = target {
            // c addr：跳转后继续（GDB step 到地址）
            let _ = m.cpu.reg_write(RegisterARM::PC, addr as u64);
        }
        let mut segs = 0u64;
        loop {
            let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap_or(0);
            if self.breaks.contains(&pc) {
                return "S05".into(); // 断点命中：SIGTRAP
            }
            // 跑一段（小预算，段边界查断点）
            if m.run(STEP_BUDGET).is_err() {
                return "S05".into();
            }
            segs += 1;
            if segs >= CONTINUE_MAX_SEG {
                return "S02".into(); // 预算上限：SIGINT
            }
        }
    }

    fn set_break(&mut self, args: &str, add: bool) {
        // Z0,addr,kind
        if let Some((_, rest)) = args.split_once(',') {
            if let Some(addr) = hex_to_u32(rest.split(',').next().unwrap_or("")) {
                if add {
                    self.breaks.insert(addr);
                } else {
                    self.breaks.remove(&addr);
                }
                log::info!("GDB {}断点 {:#010x}", if add { "设置" } else { "清除" }, addr);
            }
        }
    }

    fn cmd_query(&mut self, q: &str) -> String {
        if q.starts_with("Supported") {
            "PacketSize=1024;qXfer:memory-map:read-;qXfer:features:read-".into()
        } else if q.starts_with('C') {
            "QC1".into() // 当前线程 1
        } else if q.starts_with("Attached") {
            "1".into()
        } else {
            String::new()
        }
    }
}

/// Cortex-M4 GDB 寄存器布局（顺序 = GDB 寄存器号）：
/// r0-r12, sp, lr, pc, xpsr, msp, psp, primask, basepri, faultmask, control。
fn cortex_m_regs() -> &'static [RegisterARM] {
    &[
        RegisterARM::R0, RegisterARM::R1, RegisterARM::R2, RegisterARM::R3,
        RegisterARM::R4, RegisterARM::R5, RegisterARM::R6, RegisterARM::R7,
        RegisterARM::R8, RegisterARM::R9, RegisterARM::R10, RegisterARM::R11,
        RegisterARM::R12, RegisterARM::SP, RegisterARM::LR, RegisterARM::PC,
        RegisterARM::XPSR, RegisterARM::MSP, RegisterARM::PSP, RegisterARM::PRIMASK,
        RegisterARM::BASEPRI, RegisterARM::FAULTMASK, RegisterARM::CONTROL,
    ]
}

/// 解析 `addr,len`（均 hex）。
fn parse_addr_len(s: &str) -> Option<(u32, u32)> {
    let (a, l) = s.split_once(',')?;
    Some((hex_to_u32(a)?, hex_to_u32(l)?))
}

fn hex_to_u32(s: &str) -> Option<u32> {
    u32::from_str_radix(s.trim(), 16).ok()
}

fn hex_to_bytes(s: &str) -> Option<Vec<u8>> {
    if s.len() % 2 != 0 {
        return None;
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).ok())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn server() -> (GdbServer, Machine) {
        let m = Machine::new_m4f().unwrap();
        (GdbServer::new(), m)
    }

    #[test]
    fn hex_helpers() {
        assert_eq!(hex_to_u32("08000100"), Some(0x0800_0100));
        assert_eq!(hex_to_u32("ff"), Some(0xFF));
        assert_eq!(hex_to_bytes("0102ff"), Some(vec![1, 2, 0xFF]));
        assert!(hex_to_bytes("010").is_none());
        assert_eq!(parse_addr_len("20000000,4"), Some((0x2000_0000, 4)));
    }

    #[test]
    fn read_regs_returns_184_hex() {
        let (mut s, mut m) = server();
        let g = s.handle_packet("g", &mut m);
        assert_eq!(g.len(), 23 * 8, "23 寄存器 × 4 字节 × 2 hex");
    }

    #[test]
    fn query_and_halt_reason() {
        let (mut s, mut m) = server();
        assert!(s.handle_packet("qSupported", &mut m).contains("PacketSize"));
        assert_eq!(s.handle_packet("?", &mut m), "S05");
        assert_eq!(s.handle_packet("qAttached", &mut m), "1");
        assert_eq!(s.handle_packet("qC", &mut m), "QC1");
    }

    #[test]
    fn memory_read_write_roundtrip() {
        let (mut s, mut m) = server();
        m.map_stm32f407_layout().unwrap();
        // 写 → 读回
        assert_eq!(s.handle_packet("M20000000,4:deadbeef", &mut m), "OK");
        let r = s.handle_packet("m20000000,4", &mut m);
        assert_eq!(r, "deadbeef");
        // 坏参数
        assert_eq!(s.handle_packet("m20000000,x", &mut m), "E01");
    }

    #[test]
    fn breakpoint_set_and_hit() {
        let (mut s, mut m) = server();
        // 设置断点在当前 PC（复位后 PC=entry|1）
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        assert_eq!(s.handle_packet(&format!("Z0,{:x},2", pc & !1), &mut m), "OK");
        // continue：第一段就命中（PC 即断点）
        let r = s.handle_packet("c", &mut m);
        assert_eq!(r, "S05", "断点应立即命中");
        // 清除
        assert_eq!(s.handle_packet(&format!("z0,{:x},2", pc & !1), &mut m), "OK");
    }

    #[test]
    fn write_regs_roundtrip() {
        let (mut s, mut m) = server();
        let g = s.handle_packet("g", &mut m);
        // 改 r0（前 8 hex）为 0x11223344 LE = 44332211
        let mut modified = g.clone().into_bytes();
        modified[0..8].copy_from_slice(b"44332211");
        let g2 = String::from_utf8(modified).unwrap();
        assert_eq!(s.handle_packet(&format!("G{g2}"), &mut m), "OK");
        let g3 = s.handle_packet("g", &mut m);
        assert!(g3.starts_with("44332211"), "r0 应被写为 0x11223344：{g3}");
    }
}
