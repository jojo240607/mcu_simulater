//! M5 验收测试：ADC1 ↔ DMA2 外设→内存传输端到端（HAL 默认流）。
//!
//! 复用 firmware/adc_dma_demo 固件（场景见其 main.c 注释）：
//! 1. 固件配置 ADC1 CR2.ADON + CR1.EOCIE + CR2.DMA，接 DMA2_Stream0_Ch0
//!    （RX：外设→内存，半字），NVIC 使能 IRQ56（DMA2_Stream0）；
//! 2. 测试逐次发布 Event::AdcValue{port=1} → feed_value 锁存采样值 + 置 EOC →
//!    CR2.DMA 且 EOC 置位 → 直接路由 Stream0 → run 间隙 process 搬 1 个半字
//!    （ADC1_DR → ADC_BUF，读 DR 清 EOC）；
//! 3. 4 个采样值搬完后 NDTR 归零 → EN 自清 + TCIF + IRQ56 →
//!    DMA2_Stream0_IRQHandler 校验 ADC_BUF == {0x0123,0x0456,0x0789,0x0ABC} →
//!    G_DMA_TC++；
//! 4. 主线轮询 G_DMA_TC 达 1 → 写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_DMA_TC = 1（DMA2_Stream0 完成中断执行次数）
//!   0x20000004 G_DONE  = 0xAAAAAAAA（主线完成）
//!   0x20000200 ADC_BUF = {0x0123,0x0456,0x0789,0x0ABC}（DMA 半字搬运结果，仿真器直接校验）

use std::path::Path;

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const G_DMA_TC: u32 = 0x2000_0000;
const G_DONE: u32 = 0x2000_0004;
const ADC_BUF: u32 = 0x2000_0200;
const ADC_LEN: usize = 4;

/// 测试注入的采样值（12 位，固件 DMA 半字搬运到 ADC_BUF）
const SAMPLES: [u16; ADC_LEN] = [0x0123, 0x0456, 0x0789, 0x0ABC];

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/adc_dma_demo/adc_dma_demo.elf");
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

fn read_u16(m: &mut Machine, addr: u32) -> u16 {
    let out = m.cpu.mem_read(addr as u64, 2).unwrap();
    u16::from_le_bytes(out.try_into().unwrap())
}

#[test]
fn m5_adc_dma_end_to_end() {
    let mut m = load_machine();

    // 固件配置 ADC1 + DMA2_Stream0 + NVIC，随后进入主循环等待 DMA 完成
    m.run(10_000).unwrap();

    // 逐次注入采样值：每次 → feed_value 置 EOC → 路由 Stream0 → run 间隙搬 1 半字
    for &v in SAMPLES.iter() {
        m.events
            .lock()
            .unwrap()
            .publish(&Event::AdcValue { port: 1, channel: 0, value: v });
        m.run(50_000).unwrap();
    }
    // 主线退出循环，写 G_DONE
    m.run(50_000).unwrap();

    // 1) DMA 完成中断执行 1 次 + 主线完成
    assert_eq!(read_u32(&mut m, G_DMA_TC), 1, "DMA2_Stream0 完成中断应执行 1 次");
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");

    // 2) DMA 半字搬运结果：ADC_BUF == 注入采样序列
    for (i, v) in SAMPLES.iter().enumerate() {
        let got = read_u16(&mut m, ADC_BUF + (i as u32) * 2);
        assert_eq!(got, *v, "ADC_BUF[{i}] 应为 0x{v:04X}（DMA 半字搬运结果）");
    }
}
