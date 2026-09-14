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

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

use unicorn_engine::RegisterARM;

use crate::machine::Machine;

/// 单步预算（退休字节；2 ≈ 1 条 Thumb 指令）。
const STEP_BUDGET: usize = 2;
/// 继续预算（退休字节；大预算一次跑，靠 block hook 指令级断点停）。
const CONTINUE_BUDGET: usize = 2_000_000;

/// GDB 服务器：按 RSP 命令驱动仿真（断点集存于 Machine，block hook 指令级检查；
/// Machine 由调用方/服务线程持有）。
pub struct GdbServer {}

impl GdbServer {
    pub fn new() -> Self {
        Self {}
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
                    // RSP 双向 ACK：确认收到 GDB 的包（否则 GDB 等待超时）
                    if stream.write_all(b"+").is_err() {
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
                // c [addr]：跑到断点（block hook 精确停）或预算上限
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
                self.set_break(&cmd[1..], true, machine);
                "OK".into()
            }
            b'z' => {
                self.set_break(&cmd[1..], false, machine);
                "OK".into()
            }
            b'D' => "OK".into(), // detach
            b'k' => String::new(), // kill：空响应 + 对端断开
            b'q' => self.cmd_query(&cmd[1..]),
            _ => String::new(), // 未知命令：空响应
        }
    }

    // ---------------- 命令实现 ----------------

    fn cmd_read_regs(&mut self, m: &mut Machine) -> String {
        let mut out = String::new();
        for &(r, w) in cortex_m_regs().iter() {
            let v = match r {
                Some(r) => m.cpu.reg_read(r).unwrap_or(0),
                None => 0, // f0-f7 无映射
            };
            // 小端：低 `w` 字节（w 可 >8，如 FPA 12B；u64 之外补 0）
            for i in 0..w {
                let byte = if i < 8 { (v >> (i * 8)) & 0xFF } else { 0 };
                out.push_str(&format!("{byte:02x}"));
            }
        }
        out
    }

    fn cmd_write_regs(&mut self, hex: &str, m: &mut Machine) -> String {
        let bytes = match hex_to_bytes(hex) {
            Some(b) => b,
            None => return "E01".into(),
        };
        // 按布局逐寄存器写回（小端）
        let mut off = 0usize;
        for &(r, w) in cortex_m_regs().iter() {
            if off + w > bytes.len() {
                break;
            }
            let mut v: u64 = 0;
            for i in 0..w.min(8) {
                v |= (bytes[off + i] as u64) << (i * 8);
            }
            if let Some(r) = r {
                let _ = m.cpu.reg_write(r, v);
            }
            off += w;
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
        // 大预算一次跑；block hook 在断点地址精确停（StopReason::Breakpoint）
        let _ = m.run(CONTINUE_BUDGET);
        if m.gdb_break_hit_take() {
            "S05".into() // 断点命中：SIGTRAP
        } else {
            "S02".into() // 预算上限：SIGINT
        }
    }

    fn set_break(&mut self, args: &str, add: bool, m: &mut Machine) {
        // Z0,addr,kind
        if let Some((_, rest)) = args.split_once(',') {
            if let Some(addr) = hex_to_u32(rest.split(',').next().unwrap_or("")) {
                if add {
                    m.gdb_set_break(addr);
                } else {
                    m.gdb_clear_break(addr);
                }
                log::info!("GDB {}断点 {:#010x}", if add { "设置" } else { "清除" }, addr);
            }
        }
    }

    fn cmd_query(&mut self, q: &str) -> String {
        if q.starts_with("Supported") {
            // 声明 qXfer:features 支持（tdesc：armv7e-m + VFPv4-D16）
            "PacketSize=16384;qXfer:memory-map:read-;qXfer:features:read-".into()
        } else if q.starts_with("Xfer:features:read") {
            self.cmd_qxfer_features(q)
        } else if q.starts_with("fThreadInfo") {
            "m1".into() // 线程列表开始：只有线程 1
        } else if q.starts_with("sThreadInfo") {
            "l".into() // 线程列表结束
        } else if q.starts_with("ThreadExtraInfo") {
            // hex 编码的 "Thread 1"
            "5468726561642031".into()
        } else if q.starts_with('C') {
            "QC1".into() // 当前线程 1
        } else if q.starts_with("Attached") {
            "1".into()
        } else if q.starts_with("Offsets") {
            String::new()
        } else if q.starts_with("Symbol") {
            String::new()
        } else {
            String::new()
        }
    }

    /// qXfer:features:read:target.xml:<offset>,<length> → hex 编码分块。
    ///
    /// GDB 的 qXfer **offset/length 单位是 hex 编码字符**（把 XML 的 hex 串当
    /// "对象"分块传输，GDB 端按 hex 偏移拼接后统一解码）。因此数据源用整个
    /// XML 的 hex 串，off/len 直接作为 hex 偏移/长度；响应每块 ≤ len 字符
    /// （含 m/l 前缀），最后一块 l。
    fn cmd_qxfer_features(&mut self, q: &str) -> String {
        // 格式：Xfer:features:read:target.xml:0,1000
        let parts: Vec<&str> = q.splitn(5, ':').collect();
        if parts.len() < 5 {
            return String::new();
        }
        let (target, range) = (parts[3], parts[4]);
        if target != "target.xml" {
            return String::new();
        }
        let (off, len) = match range.split_once(',') {
            Some((o, l)) => match (usize::from_str_radix(o, 16), usize::from_str_radix(l, 16)) {
                (Ok(o), Ok(l)) => (o, l),
                _ => return "E01".into(),
            },
            None => return "E01".into(),
        };
        let hex_all = qxfer_hex_cache();
        if off >= hex_all.len() {
            return "l".into(); // 空末块
        }
        // 每块响应（含 m/l 前缀）≤ len hex 字符；hex 数据必须偶数长度
        let max_chars = (len.saturating_sub(1)) & !1;
        let end = (off + max_chars).min(hex_all.len());
        // end 需与 off 同奇偶（保证切片偶数长）
        let end = end & !1;
        let chunk = &hex_all[off..end];
        let prefix = if end >= hex_all.len() { "l" } else { "m" };
        format!("{prefix}{chunk}")
    }
}

/// TARGET_XML 的 hex 编码缓存（qXfer 以 hex 偏移分块）。
fn qxfer_hex_cache() -> &'static str {
    use std::sync::OnceLock;
    static CACHE: OnceLock<String> = OnceLock::new();
    CACHE.get_or_init(|| {
        TARGET_XML.as_bytes().iter().map(|b| format!("{b:02x}")).collect()
    })
}

/// armv7e-m + VFPv4-D16 目标描述（GDB tdesc）。
/// 寄存器顺序（= g 包布局，352 字节）：
/// r0-r15, xpsr, msp, psp, primask, basepri, faultmask, control,
/// fpscr, d0-d15(8B), s0-s31。
const TARGET_XML: &str = r#"<?xml version="1.0"?>
<target>
  <architecture>arm</architecture>
  <feature name="org.gnu.gdb.arm.m-profile">
    <reg name="r0" bitsize="32"/>
    <reg name="r1" bitsize="32"/>
    <reg name="r2" bitsize="32"/>
    <reg name="r3" bitsize="32"/>
    <reg name="r4" bitsize="32"/>
    <reg name="r5" bitsize="32"/>
    <reg name="r6" bitsize="32"/>
    <reg name="r7" bitsize="32"/>
    <reg name="r8" bitsize="32"/>
    <reg name="r9" bitsize="32"/>
    <reg name="r10" bitsize="32"/>
    <reg name="r11" bitsize="32"/>
    <reg name="r12" bitsize="32"/>
    <reg name="sp" bitsize="32" type="data_ptr"/>
    <reg name="lr" bitsize="32"/>
    <reg name="pc" bitsize="32" type="code_ptr"/>
    <reg name="xpsr" bitsize="32"/>
    <reg name="msp" bitsize="32"/>
    <reg name="psp" bitsize="32"/>
    <reg name="primask" bitsize="32"/>
    <reg name="basepri" bitsize="32"/>
    <reg name="faultmask" bitsize="32"/>
    <reg name="control" bitsize="32"/>
  </feature>
  <feature name="org.gnu.gdb.arm.vfp">
    <reg name="fpscr" bitsize="32" type="int" group="float"/>
    <reg name="d0" bitsize="64" type="float" group="float"/>
    <reg name="d1" bitsize="64" type="float" group="float"/>
    <reg name="d2" bitsize="64" type="float" group="float"/>
    <reg name="d3" bitsize="64" type="float" group="float"/>
    <reg name="d4" bitsize="64" type="float" group="float"/>
    <reg name="d5" bitsize="64" type="float" group="float"/>
    <reg name="d6" bitsize="64" type="float" group="float"/>
    <reg name="d7" bitsize="64" type="float" group="float"/>
    <reg name="d8" bitsize="64" type="float" group="float"/>
    <reg name="d9" bitsize="64" type="float" group="float"/>
    <reg name="d10" bitsize="64" type="float" group="float"/>
    <reg name="d11" bitsize="64" type="float" group="float"/>
    <reg name="d12" bitsize="64" type="float" group="float"/>
    <reg name="d13" bitsize="64" type="float" group="float"/>
    <reg name="d14" bitsize="64" type="float" group="float"/>
    <reg name="d15" bitsize="64" type="float" group="float"/>
    <reg name="s0" bitsize="32" type="float" group="float"/>
    <reg name="s1" bitsize="32" type="float" group="float"/>
    <reg name="s2" bitsize="32" type="float" group="float"/>
    <reg name="s3" bitsize="32" type="float" group="float"/>
    <reg name="s4" bitsize="32" type="float" group="float"/>
    <reg name="s5" bitsize="32" type="float" group="float"/>
    <reg name="s6" bitsize="32" type="float" group="float"/>
    <reg name="s7" bitsize="32" type="float" group="float"/>
    <reg name="s8" bitsize="32" type="float" group="float"/>
    <reg name="s9" bitsize="32" type="float" group="float"/>
    <reg name="s10" bitsize="32" type="float" group="float"/>
    <reg name="s11" bitsize="32" type="float" group="float"/>
    <reg name="s12" bitsize="32" type="float" group="float"/>
    <reg name="s13" bitsize="32" type="float" group="float"/>
    <reg name="s14" bitsize="32" type="float" group="float"/>
    <reg name="s15" bitsize="32" type="float" group="float"/>
    <reg name="s16" bitsize="32" type="float" group="float"/>
    <reg name="s17" bitsize="32" type="float" group="float"/>
    <reg name="s18" bitsize="32" type="float" group="float"/>
    <reg name="s19" bitsize="32" type="float" group="float"/>
    <reg name="s20" bitsize="32" type="float" group="float"/>
    <reg name="s21" bitsize="32" type="float" group="float"/>
    <reg name="s22" bitsize="32" type="float" group="float"/>
    <reg name="s23" bitsize="32" type="float" group="float"/>
    <reg name="s24" bitsize="32" type="float" group="float"/>
    <reg name="s25" bitsize="32" type="float" group="float"/>
    <reg name="s26" bitsize="32" type="float" group="float"/>
    <reg name="s27" bitsize="32" type="float" group="float"/>
    <reg name="s28" bitsize="32" type="float" group="float"/>
    <reg name="s29" bitsize="32" type="float" group="float"/>
    <reg name="s30" bitsize="32" type="float" group="float"/>
    <reg name="s31" bitsize="32" type="float" group="float"/>
  </feature>
</target>"#;

/// GDB **默认 ARM 布局**（无 tdesc 时 GDB 用 A-profile 传统布局，168 字节）：
/// r0-r15(4B×16), f0-f7(12B×8, FPA 扩展→0), fps(4B), cpsr(4B)。
///
/// 不提供 tdesc 的原因：GDB 13 的 qXfer 分块拼接会按 hex 字符偏移写入缓冲，
/// 多块 tdesc 易产生空洞（syntax error）；默认布局单块 168B 无此问题。
/// M-profile 的 xpsr 映射到 cpsr 槽位（GDB info registers 正常显示）；
/// FPU 寄存器（d0-d15/s0-s31）不暴露——固件浮点逻辑不受影响（调试器不读写
/// FPU 寄存器即可调试控制流/内存/断点）。
fn cortex_m_regs() -> Vec<(Option<RegisterARM>, usize)> {
    let mut v: Vec<(Option<RegisterARM>, usize)> = vec![
        (Some(RegisterARM::R0), 4), (Some(RegisterARM::R1), 4),
        (Some(RegisterARM::R2), 4), (Some(RegisterARM::R3), 4),
        (Some(RegisterARM::R4), 4), (Some(RegisterARM::R5), 4),
        (Some(RegisterARM::R6), 4), (Some(RegisterARM::R7), 4),
        (Some(RegisterARM::R8), 4), (Some(RegisterARM::R9), 4),
        (Some(RegisterARM::R10), 4), (Some(RegisterARM::R11), 4),
        (Some(RegisterARM::R12), 4), (Some(RegisterARM::SP), 4),
        (Some(RegisterARM::LR), 4), (Some(RegisterARM::PC), 4),
    ];
    // f0-f7：FPA 扩展（12B），本平台无 → 0
    for _ in 0..8 {
        v.push((None, 12));
    }
    // fps（FPSCR）、cpsr（xpsr 映射）
    v.push((Some(RegisterARM::FPSCR), 4));
    v.push((Some(RegisterARM::XPSR), 4));
    v
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
    fn read_regs_returns_336_hex() {
        let (mut s, mut m) = server();
        let g = s.handle_packet("g", &mut m);
        assert_eq!(g.len(), 336, "168 字节 = r0-r15(4B) + f0-f7(12B) + fps + cpsr，hex ×2");
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
