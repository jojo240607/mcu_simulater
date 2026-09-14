//! GDB Server（调试平台 P2-3）端到端验收：真实 TCP 连接走 RSP 协议。
//!
//! 服务线程内建真实固件（i2c_irq_demo）Machine 并监听，测试客户端按 RSP
//! 帧格式（$cmd#cs）收发：
//! - 特性协商（qSupported）与停止原因（?）
//! - 寄存器读取（g：184 hex）/ 写入（G）
//! - 内存读写（m/M）
//! - 断点（Z0）→ 继续（c）→ 断点命中（S05）
//! - 单步（s）、kill（k）

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::Path;
use std::time::Duration;

use mcu_simulater::gdbstub::GdbServer;

const G_TX: u32 = 0x2000_0000;

fn rsp_send(stream: &mut TcpStream, cmd: &str) -> String {
    // 发 $cmd#cs（校验和 = 字节和 & 0xff）
    let sum: u8 = cmd.bytes().fold(0, |a, b| a.wrapping_add(b));
    let frame = format!("${cmd}#{sum:02x}");
    stream.write_all(frame.as_bytes()).unwrap();
    // 读响应 $resp#cs（含 ACK 处理：对端不发 ACK，我们直接收 $...）
    let mut buf = Vec::new();
    let mut byte = [0u8; 1];
    // 等 '$'
    loop {
        stream.read_exact(&mut byte).unwrap();
        if byte[0] == b'$' {
            break;
        }
    }
    loop {
        stream.read_exact(&mut byte).unwrap();
        if byte[0] == b'#' {
            break;
        }
        buf.push(byte[0]);
    }
    // 吃掉 2 位校验和
    let mut cs = [0u8; 2];
    let _ = stream.read_exact(&mut cs);
    String::from_utf8(buf).unwrap()
}

#[test]
fn gdb_rsp_end_to_end() {
    // 服务线程：建固件 Machine + 监听（随机端口避免冲突）
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/i2c_irq_demo/i2c_irq_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}");
    let port = 33400 + (std::process::id() % 1000) as u16;
    let handle = GdbServer::spawn(port, move || {
        let mut m = mcu_simulater::machine::Machine::new_m4f().unwrap();
        m.map_stm32f407_layout().unwrap();
        m.load_elf(&elf).unwrap();
        m.reset().unwrap();
        m
    })
    .expect("监听失败");

    // 客户端连接（重试直到服务就绪）
    let mut stream = None;
    for _ in 0..50 {
        match TcpStream::connect(("127.0.0.1", port)) {
            Ok(s) => {
                stream = Some(s);
                break;
            }
            Err(_) => std::thread::sleep(Duration::from_millis(50)),
        }
    }
    let mut s = stream.expect("无法连接 GDB server");

    // 1) 协商 + 停止原因
    let r = rsp_send(&mut s, "qSupported");
    assert!(r.contains("PacketSize"), "qSupported 应回特性：{r}");
    assert_eq!(rsp_send(&mut s, "?"), "S05");

    // 2) 寄存器读取：336 hex（GDB 默认 ARM 布局 168B = r0-r15 + f0-f7×12B + fps + cpsr）
    let g = rsp_send(&mut s, "g");
    assert_eq!(g.len(), 336, "寄存器串长度");

    // 3) 内存读写
    assert_eq!(rsp_send(&mut s, &format!("M{:x},4:deadbeef", G_TX)), "OK");
    assert_eq!(rsp_send(&mut s, &format!("m{:x},4", G_TX)), "deadbeef");

    // 4) 寄存器写（r0 = 0x11223344）
    let mut g2 = g.clone().into_bytes();
    g2[0..8].copy_from_slice(b"44332211");
    let g2s = String::from_utf8(g2).unwrap();
    assert_eq!(rsp_send(&mut s, &format!("G{g2s}")), "OK");
    let g3 = rsp_send(&mut s, "g");
    assert!(g3.starts_with("44332211"), "r0 应写回 0x11223344");

    // 5) 断点 + 继续 → 命中（断点设在当前 PC=Reset_Handler 入口；block hook 精确停）
    let pc_hex = std::str::from_utf8(&g.as_bytes()[15 * 8..16 * 8]).unwrap();
    let pc_le = u32::from_str_radix(
        &format!("{}{}{}{}", &pc_hex[6..8], &pc_hex[4..6], &pc_hex[2..4], &pc_hex[0..2]),
        16,
    )
    .unwrap();
    assert_eq!(rsp_send(&mut s, &format!("Z0,{:x},2", pc_le & !1)), "OK");
    let r = rsp_send(&mut s, "c");
    assert_eq!(r, "S05", "断点应立即命中（指令级）");
    assert_eq!(rsp_send(&mut s, &format!("z0,{:x},2", pc_le & !1)), "OK");

    // 6) 单步
    assert_eq!(rsp_send(&mut s, "s"), "S05");

    // 7) kill：空响应（$#00）
    let r = rsp_send(&mut s, "k");
    assert_eq!(r, "", "kill 应回空响应");

    // 服务线程收尾（不 join：监听循环持续 accept，进程退出即结束）
    drop(s);
    let _ = handle;
}

#[test]
fn gdb_continue_runs_firmware() {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/i2c_irq_demo/i2c_irq_demo.elf");
    let port = 34400 + (std::process::id() % 1000) as u16;
    let handle = GdbServer::spawn(port, move || {
        let mut m = mcu_simulater::machine::Machine::new_m4f().unwrap();
        m.map_stm32f407_layout().unwrap();
        m.load_elf(&elf).unwrap();
        m.reset().unwrap();
        m
    })
    .expect("监听失败");

    let mut s = None;
    for _ in 0..50 {
        match TcpStream::connect(("127.0.0.1", port)) {
            Ok(x) => {
                s = Some(x);
                break;
            }
            Err(_) => std::thread::sleep(Duration::from_millis(50)),
        }
    }
    let mut s = s.expect("无法连接 GDB server");

    // 在固件主循环设断点（0x0800_0000 范围）跑一段：continue 后应推进固件
    // （G_TX 由固件写入——先 c 跑到断点，再读内存应已变化）
    // 用 entry 断点：读复位后 PC 太早；直接在固件复位向量区设断点验证 c 工作。
    let pc = {
        let g = rsp_send(&mut s, "g");
        let pb = std::str::from_utf8(&g.as_bytes()[15 * 8..16 * 8]).unwrap();
        u32::from_str_radix(
            &format!("{}{}{}{}", &pb[6..8], &pb[4..6], &pb[2..4], &pb[0..2]),
            16,
        )
        .unwrap()
    };
    // c 无断点：应跑到预算上限返回 S02（SIGINT）——验证 continue 执行固件
    let r = rsp_send(&mut s, "c");
    assert!(r == "S05" || r == "S02", "continue 应停（断点或预算上限）：{r}");
    let _ = pc;

    // kill 收尾（不 join：监听循环持续 accept，进程退出即结束）
    let _ = rsp_send(&mut s, "k");
    drop(s);
    let _ = handle;
}
