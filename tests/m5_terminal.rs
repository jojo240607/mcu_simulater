//! M5 验收测试：UART4-6 挂载 + 虚拟终端接线（终端回环）端到端。
//!
//! 复用 firmware/terminal_demo 固件：
//! 1. 固件使能 UART4(IRQ52)/UART5/USART6，UART4 UE+TE+RE+RXNEIE；
//! 2. 发问候：UART4→'T'、UART5→'5'、USART6→'6'；
//!    - 默认接线：全部 UART TX → Console（捕获 'T56'）；
//!    - connect(uart.tx, terminal.rx) port=4 → 'T' 进虚拟终端显示；
//! 3. 测试经虚拟终端键盘 `terminal.type_char(4, c)` 发布 UartRx → UART4 feed_rx
//!    → RXNE + 挂起 IRQ52 → handler 读 DR + 回显 → 'a'..'d' 进 Console 与终端显示；
//! 4. 主线等待 G_RX4==4 → G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_TX4   = 5（问候 1 + 回显 4）
//!   0x20000004 G_RX4   = 4（回显次数）
//!   0x20000008 G_TX56  = 2（UART5 + USART6 问候）
//!   0x2000000C G_DONE  = 0xAAAAAAAA
//!   0x20000010 G_ORE   = 0
//!   Terminal 显示 = "Tabcd"（UART4 TX → 终端）
//!   Console 输出  = "T56abcd"（UART4/5/6 TX → Console）

use std::path::Path;

use mcu_simulater::events::Event;
use mcu_simulater::machine::{ConnectSource, ConnectTarget, Machine};

const G_TX4: u32 = 0x2000_0000;
const G_RX4: u32 = 0x2000_0004;
const G_TX56: u32 = 0x2000_0008;
const G_DONE: u32 = 0x2000_000C;
const G_ORE: u32 = 0x2000_0010;

/// 终端键盘注入的回显字节（UART4 终端回环）
const RX_BYTES: [u8; 4] = [b'a', b'b', b'c', b'd'];

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/terminal_demo/terminal_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    // 虚拟终端接线：UART4 TX → 终端显示（类 Renode connect uart.tx -> terminal.rx）
    m.connect(ConnectSource::UartTx(4), ConnectTarget::TerminalRx)
        .unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

fn read_u32(m: &mut Machine, addr: u32) -> u32 {
    let out = m.cpu.mem_read(addr as u64, 4).unwrap();
    u32::from_le_bytes(out.try_into().unwrap())
}

#[test]
fn m5_uart46_terminal_loopback_end_to_end() {
    let mut m = load_machine();

    // 固件使能 UART4/5/USART6 + 发问候，随后进入等待循环
    m.run(10_000).unwrap();

    // 经虚拟终端键盘逐字符输入：type_char 发布 UartRx → UART4 RX → 中断回显
    for &b in &RX_BYTES {
        m.terminal.lock().unwrap().type_char(4, b);
        m.run(50_000).unwrap();
    }
    // 主线退出循环，写 G_DONE
    m.run(50_000).unwrap();

    // 1) 结果区
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_RX4), 4, "UART4 中断应回显 4 次");
    assert_eq!(read_u32(&mut m, G_TX4), 5, "UART4 应发送 问候1 + 回显4 = 5 字节");
    assert_eq!(read_u32(&mut m, G_TX56), 2, "UART5 + USART6 应各发 1 字节问候");
    assert_eq!(read_u32(&mut m, G_ORE), 0, "逐字符输入不应产生过载");

    // 2) 虚拟终端显示：UART4 TX → 终端（问候 'T' + 回显 "abcd"）
    let mut expected_term = vec![b'T'];
    expected_term.extend_from_slice(&RX_BYTES);
    let term = m.terminal.lock().unwrap().display().to_vec();
    assert_eq!(term, expected_term, "虚拟终端应显示 UART4 的 'T' + 回显字节");

    // 3) Console 默认接线：UART4/5/6 全部 TX（'T' '5' '6' + 回显 "abcd"）
    let mut expected_con = vec![b'T', b'5', b'6'];
    expected_con.extend_from_slice(&RX_BYTES);
    let con = m.console.lock().unwrap().output().to_vec();
    assert_eq!(con, expected_con, "Console 应收到 'T56' + 回显字节");
}
