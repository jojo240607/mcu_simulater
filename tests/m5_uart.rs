//! M5 验收测试：USART 串口仿真模块端到端（RX 中断接收 + 回显 + TX→Console）。
//!
//! 复用 firmware/uart_demo 固件（USART1 @ 0x40011000 + IRQ37）：
//! 1. 固件使能 USART1 UE+TE+RE + RXNEIE，NVIC IRQ37 优先级 15 + 使能；
//! 2. 轮询 TXE 发送问候 'U' → Console 捕获；
//! 3. 测试逐字节发布 Event::UartRx{port=1} → USART feed_rx：锁存 DR + 置 RXNE + 挂起 IRQ37；
//! 4. USART1_IRQHandler：读 DR（清 RXNE）→ 回显写 DR（TE+UE → Console）→ G_RX++；
//! 5. 主线等待 G_RX==4 → 写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_TX   = 5（问候 1 + 回显 4）
//!   0x20000004 G_RX   = 4（回显次数）
//!   0x20000008 G_ORE  = 0（逐字节注入，无过载）
//!   0x2000000C G_DONE = 0xAAAAAAAA（主线完成）
//!   Console 输出 = "U" + "abcd"

use std::path::Path;

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const G_TX: u32 = 0x2000_0000;
const G_RX: u32 = 0x2000_0004;
const G_ORE: u32 = 0x2000_0008;
const G_DONE: u32 = 0x2000_000C;

/// 测试注入的 RX 字节（固件逐字节回显到 Console）
const RX_BYTES: [u8; 4] = [b'a', b'b', b'c', b'd'];

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/uart_demo/uart_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

fn read_u32(m: &mut Machine, addr: u32) -> u32 {
    let out = m.cpu.mem_read(addr as u64, 4).unwrap();
    u32::from_le_bytes(out.try_into().unwrap())
}

#[test]
fn m5_usart_rx_interrupt_echo_end_to_end() {
    let mut m = load_machine();

    // 固件配置 USART1 + NVIC + 发问候 'U'，随后进入等待循环（run 按指令预算返回）
    m.run(10_000).unwrap();

    // 逐字节注入 RX：每注入 1 字节 → 运行让 IRQ37 handler 读 DR + 回显
    for &b in &RX_BYTES {
        m.events
            .lock()
            .unwrap()
            .publish(&Event::UartRx { port: 1, byte: b });
        m.run(50_000).unwrap();
    }
    // 主线退出循环，写 G_DONE
    m.run(50_000).unwrap();

    // 1) 主线完成 + 回显计数 + 无过载
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_RX), 4, "RX 中断应回显 4 次");
    assert_eq!(read_u32(&mut m, G_ORE), 0, "逐字节注入不应产生过载");
    assert_eq!(read_u32(&mut m, G_TX), 5, "应发送 问候1 + 回显4 = 5 字节");

    // 2) Console 捕获：问候 'U' + 回显 "abcd"（RX 中断 → TX 全链路）
    let mut expected = vec![b'U'];
    expected.extend_from_slice(&RX_BYTES);
    let got = m.console.lock().unwrap().output().to_vec();
    assert_eq!(got, expected, "Console 应收到 'U' + 回显字节");
}

