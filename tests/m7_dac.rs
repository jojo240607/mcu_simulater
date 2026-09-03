//! M7 验收测试：DAC1 软件触发 + 定时器触发 DMA（内存→外设）端到端。
//!
//! 复用 firmware/dac_dma_demo 固件（场景见其 main.c 注释）：
//! 1. 固件 Phase A：CR=EN1|TEN1（软件触发），写 DHR12R1=0x0321 → SWTRIGR.SWTRIG1
//!    → DHR→DOR 锁存 + 发布 DacLevel{0x0321}；固件回读 DOR1 校验 → G_SW_OK=1；
//! 2. 固件 Phase B：CR=EN1|TEN1|DMAEN1（TSEL 高 2 位=00 → TIM6 TRGO），配置
//!    DMA1_Stream5_Ch7（TX：内存→外设，半字，MINC，TCIE）+ NVIC IRQ16，TIM6
//!    ARR=1 + UDE + CEN 启动；
//! 3. TIM6 溢出 → TimUpdate{port=6} → DAC 锁存 DHR→DOR + 路由 Stream5 →
//!    run 间隙 Dma::process 从 DAC_BUF 搬 4 个半字 → dma_write_dr 写 DHR12R1
//!    并转换（各发布 DacLevel）→ NDTR 归零 → EN 自清 + TCIF + IRQ16 →
//!    DMA1_Stream5_IRQHandler 回读 DOR1==DAC_BUF[3] → G_DMA_TC++；
//! 4. 主线轮询 G_DMA_TC 达 1 → 写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_SW_OK  = 1（Phase A 软件触发 DHR→DOR 回读校验）
//!   0x20000004 G_DMA_TC = 1（DMA1_Stream5 完成中断执行次数）
//!   0x20000008 G_DONE   = 0xAAAAAAAA（主线完成）
//!   0x20000200 DAC_BUF  = {0x0111,0x0222,0x0333,0x0DDD}（测试预装载，DMA 搬运源）
//!   事件总线 DacLevel 序列：含 0x0321（软件触发）且尾随 DAC_BUF 四值（DMA 写 DHR 转换）

use std::path::Path;
use std::sync::{Arc, Mutex};

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const G_SW_OK: u32 = 0x2000_0000;
const G_DMA_TC: u32 = 0x2000_0004;
const G_DONE: u32 = 0x2000_0008;
const DAC_BUF: u32 = 0x2000_0200;

/// 测试预装载的 DAC 发送源（固件 DMA 半字搬运到 DHR12R1，再转换发布）
const DAC_BUF_EXPECT: [u16; 4] = [0x0111, 0x0222, 0x0333, 0x0DDD];

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/dac_dma_demo/dac_dma_demo.elf");
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
fn m7_dac_end_to_end() {
    let mut m = load_machine();

    // 预装载 DAC 发送源（DMA 内存→外设搬运，固件不初始化，由测试写入 SRAM）
    for (i, v) in DAC_BUF_EXPECT.iter().enumerate() {
        m.cpu
            .mem_write(DAC_BUF as u64 + (i as u64) * 2, &v.to_le_bytes())
            .unwrap();
    }

    // 订阅 DAC1 输出电平事件（ch1，按序记录 DacLevel 电平值）
    let trace = Arc::new(Mutex::new(Vec::new()));
    let tr = trace.clone();
    m.events.lock().unwrap().subscribe(Arc::new(Mutex::new(
        move |ev: &Event| {
            if let Event::DacLevel { port: 1, channel: 1, level } = ev {
                tr.lock().unwrap().push(*level);
            }
        },
    )));

    // 阶段一：固件初始化 + Phase A 软件触发 + Phase B 定时器/DMA 配置
    m.run(10_000).unwrap();
    // 1) 软件触发：DHR→DOR 锁存回读校验通过
    assert_eq!(read_u32(&mut m, G_SW_OK), 1, "Phase A 软件触发 DOR1 回读校验应通过");

    // 阶段二：TIM6 溢出 → DAC 触发 DMA → run 间隙搬运 + 中断投递，直到主线写 G_DONE
    for _ in 0..10 {
        m.run(200_000).unwrap();
        if read_u32(&mut m, G_DONE) == 0xAAAA_AAAA {
            break;
        }
    }

    // 2) DMA 完成中断执行 1 次 + 主线完成
    assert_eq!(read_u32(&mut m, G_DMA_TC), 1, "DMA1_Stream5 完成中断应执行 1 次");
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");

    // 3) 事件总线 DacLevel 序列：软件触发发布 0x0321；DMA 写 DHR 逐次转换发布 DAC_BUF 四值。
    //    注：TIM6 自 Phase B 启动到 DMA 完成（run 间隙）前以 ARR=1 持续溢出，逐次锁存当前
    //    DHR（DMA 前为陈旧 0x0321、DMA 后到固件停 TIM6 前为陈旧 0x0DDD），产生大量冗余
    //    DacLevel——属批量仿真时序伪影，非 DMA 重复搬运（DMA 完成即 EN 自清）。故不要求
    //    事件序列精确相等，只断言"过滤出 DAC_BUF 值后前四个按序等于 DAC_BUF"。
    let evs = trace.lock().unwrap();
    assert!(
        evs.contains(&0x0321),
        "软件触发应发布 DacLevel{{0x0321}}（实际 {evs:?}）"
    );
    // 过滤出 DAC_BUF 集合内电平（DMA 写 DHR 转换产生；陈旧锁存也可能混入相同值）
    let dma_levels: Vec<u16> = evs
        .iter()
        .copied()
        .filter(|l| DAC_BUF_EXPECT.contains(l))
        .collect();
    assert!(
        dma_levels.len() >= DAC_BUF_EXPECT.len(),
        "DMA 写 DHR 转换发布不足（实际 {evs:?}）"
    );
    assert_eq!(
        &dma_levels[..DAC_BUF_EXPECT.len()],
        &DAC_BUF_EXPECT[..],
        "DMA 写 DHR 转换发布前四值应按序等于 DAC_BUF（事件序列 {evs:?}）"
    );
}
