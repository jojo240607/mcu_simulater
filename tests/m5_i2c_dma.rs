//! M5 验收测试：I2C1 ↔ DMA1 外设方向传输端到端（HAL 默认流 TX/RX）。
//!
//! 复用 firmware/i2c_dma_demo 固件（场景见其 main.c 注释）：
//! 1. 固件配置 I2C1 PE + CR2.DMAEN，接 DMA1 默认流
//!    （TX=DMA1_Stream6_Ch1、RX=DMA1_Stream0_Ch1），NVIC 使能 IRQ17/IRQ11；
//! 2. 写 CR2 时 TxE 已置位 → 发布 TX DMA 请求 → 路由 Stream6 → run 间隙
//!    process 一次搬完 TX_BUF（每字节 → I2C1_DR → I2cByte 事件）→ TCIF + IRQ17；
//! 3. 测试逐字节注入 Event::I2cRx{port=1} → feed_rx 置 RxNE → CR2.DMAEN →
//!    直接路由 Stream0 → 每字节搬 1 次 → RX_BUF；
//! 4. 两条流完成后各自 TC 中断 handler：G_TX_TC++ / G_RX_TC++（校验 RX_BUF）；
//! 5. 主线等待两者完成 → 写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_TX_TC = 1（TX 完成中断次数）
//!   0x20000004 G_RX_TC = 1（RX 完成中断次数，handler 内校验 RX_BUF == "abcd"）
//!   0x20000008 G_DONE  = 0xAAAAAAAA（主线完成）
//!   0x20000200 RX_BUF  = "abcd"（DMA RX 搬运结果，仿真器直接校验）
//!   订阅 I2cByte 收集 = "I2CDMA"（DMA TX 发送）

use std::path::Path;
use std::sync::{Arc, Mutex};

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const G_TX_TC: u32 = 0x2000_0000;
const G_RX_TC: u32 = 0x2000_0004;
const G_DONE: u32 = 0x2000_0008;
const RX_BUF: u32 = 0x2000_0200;
const RX_LEN: usize = 4;

/// 测试注入的 RX 字节（固件 DMA RX 搬运到 RX_BUF）
const RX_BYTES: [u8; RX_LEN] = [b'a', b'b', b'c', b'd'];
/// 固件预置的 TX 源（DMA TX → I2cByte 事件，测试订阅收集）
const TX_BYTES: [u8; 6] = [b'I', b'2', b'C', b'D', b'M', b'A'];

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/i2c_dma_demo/i2c_dma_demo.elf");
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
fn m5_i2c_dma_tx_rx_end_to_end() {
    let mut m = load_machine();

    // 订阅 I2C1 TX 事件：收集固件经 DMA TX 发送的字节
    let got: Arc<Mutex<Vec<u8>>> = Arc::new(Mutex::new(Vec::new()));
    let got2 = got.clone();
    m.events.lock().unwrap().subscribe(Arc::new(Mutex::new(
        move |ev: &Event| {
            if let Event::I2cByte { port: 1, byte } = ev {
                got2.lock().unwrap().push(*byte);
            }
        },
    )));

    // 固件配置 I2C1 + DMA1 双流 + NVIC，写 CR2 触发 TX DMA → run 搬运 TX_BUF → IRQ17
    m.run(10_000).unwrap();

    // 逐字节注入 RX：每注入 1 字节 → DMA1_Stream0 搬 1 字节到 RX_BUF
    for &b in RX_BYTES.iter() {
        m.events
            .lock()
            .unwrap()
            .publish(&Event::I2cRx { port: 1, byte: b });
        m.run(50_000).unwrap();
    }
    // 主线退出循环，写 G_DONE
    m.run(50_000).unwrap();

    // 1) 主线完成 + TX/RX 完成中断各 1 次
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_TX_TC), 1, "DMA1_Stream6 TX 完成中断应执行 1 次");
    assert_eq!(read_u32(&mut m, G_RX_TC), 1, "DMA1_Stream0 RX 完成中断应执行 1 次");

    // 2) DMA RX 搬运结果：RX_BUF == "abcd"
    for (i, b) in RX_BYTES.iter().enumerate() {
        let out = m.cpu.mem_read((RX_BUF + i as u32) as u64, 1).unwrap();
        assert_eq!(out[0], *b, "RX_BUF[{i}] 应为 {b:?}（DMA RX 搬运结果）");
    }

    // 3) DMA TX 发送结果：I2cByte 事件收到 "I2CDMA"
    assert_eq!(
        got.lock().unwrap().as_slice(),
        TX_BYTES.as_slice(),
        "I2cByte 事件应收到 DMA TX 发送的字节"
    );
}
