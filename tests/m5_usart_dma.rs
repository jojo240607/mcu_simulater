//! M5 验收测试：USART1 ↔ DMA2 外设方向传输端到端（HAL 默认流 TX/RX）。
//!
//! 复用 firmware/usart_dma_demo 固件（场景见其 main.c 注释）：
//! 1. 固件配置 USART1 UE+TE+RE + CR3.DMAT|DMAR，接 DMA2 默认流
//!    （TX=DMA2_Stream7_Ch4、RX=DMA2_Stream2_Ch4），NVIC 使能 IRQ70/IRQ58；
//! 2. 写 CR3 时 TXE 已置位 → 发布 TX DMA 请求 → 路由 Stream7 → run 间隙
//!    process 一次搬完 TX_BUF（每字节 → USART DR → Console）→ TCIF + IRQ70；
//! 3. 测试逐字节注入 Event::UartRx{port=1} → feed_rx 置 RXNE → DMAR →
//!    发布 RX DMA 请求 → 路由 Stream2 → 每字节搬 1 次 → RX_BUF；
//! 4. 两条流完成后各自 TC 中断 handler：G_TX_TC++ / G_RX_TC++（校验 RX_BUF）；
//! 5. 主线等待两者完成 → 写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_TX_TC = 1（TX 完成中断次数）
//!   0x20000004 G_RX_TC = 1（RX 完成中断次数，handler 内校验 RX_BUF == "abcd"）
//!   0x20000008 G_DONE  = 0xAAAAAAAA（主线完成）
//!   0x20000200 RX_BUF  = "abcd"（DMA RX 搬运结果，仿真器直接校验）
//!   Console 输出 = "M5DMA!"（DMA TX 发送）

use std::path::Path;

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const G_TX_TC: u32 = 0x2000_0000;
const G_RX_TC: u32 = 0x2000_0004;
const G_DONE: u32 = 0x2000_0008;
const RX_BUF: u32 = 0x2000_0200;
const RX_LEN: usize = 4;

/// 测试注入的 RX 字节（固件 DMA RX 搬运到 RX_BUF）
const RX_BYTES: [u8; RX_LEN] = [b'a', b'b', b'c', b'd'];
/// 固件预置的 TX 源（DMA TX → Console）
const TX_BYTES: [u8; 6] = [b'M', b'5', b'D', b'M', b'A', b'!'];

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/usart_dma_demo/usart_dma_demo.elf");
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
fn m5_usart_dma_tx_rx_end_to_end() {
    let mut m = load_machine();

    // 固件配置 USART1 + DMA2 双流 + NVIC，写 CR3 触发 TX DMA → run 搬运 TX_BUF → IRQ70
    m.run(10_000).unwrap();

    // 逐字节注入 RX：每注入 1 字节 → DMA2_Stream2 搬 1 字节到 RX_BUF
    for &b in RX_BYTES.iter() {
        m.events
            .lock()
            .unwrap()
            .publish(&Event::UartRx { port: 1, byte: b });
        m.run(50_000).unwrap();
    }
    // 主线退出循环，写 G_DONE
    m.run(50_000).unwrap();

    // 1) 主线完成 + TX/RX 完成中断各 1 次
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_TX_TC), 1, "DMA2_Stream7 TX 完成中断应执行 1 次");
    assert_eq!(read_u32(&mut m, G_RX_TC), 1, "DMA2_Stream2 RX 完成中断应执行 1 次");

    // 2) DMA RX 搬运结果：RX_BUF == "abcd"
    for (i, b) in RX_BYTES.iter().enumerate() {
        let out = m.cpu.mem_read((RX_BUF + i as u32) as u64, 1).unwrap();
        assert_eq!(out[0], *b, "RX_BUF[{i}] 应为 {b:?}（DMA RX 搬运结果）");
    }

    // 3) DMA TX 发送结果：Console 收到 "M5DMA!"
    let got = m.console.lock().unwrap().output().to_vec();
    assert_eq!(got, TX_BYTES.to_vec(), "Console 应收到 DMA TX 发送的字节");
}
