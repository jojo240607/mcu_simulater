//! M15 验收测试：CAN1/2 控制器局域网（STM32F407）端到端 + 总线错误/互联链路。
//!
//! 复用 firmware/can_demo 固件（场景见其 main.c 注释）：
//!   Phase A（CAN1 TX → CAN2 RX，标准帧）：CAN1 邮箱0 发送 ID=0x123 DLC=2
//!         [0x11,0x22] → 总线发布 → CAN2 过滤器全收命中 FIFO0 → 读回校验
//!         （RI/RDT/RDL/RDH）→ G_TX2RX_OK=1；
//!   Phase B（CAN2 TX → CAN1 RX，扩展帧）：CAN2 发送 ID=0x1FFEDCBA DLC=8 →
//!         CAN1 过滤器 F1 命中 FIFO0 → 读回校验 → G_RX2TX_OK=1；
//!   Phase C（FIFO 排队 + RFOM）：CAN2 连发两帧 → CAN1 FIFO0 FMP=2 → 读第 1
//!         帧 → RFOM 释放 → FMP=1 → 读第 2 帧 → 释放 → FMP=0 → G_FIFO_OK=1。
//!   发送在写 TIRx 时同步完成并发布/路由，单次 run 即可完成；固件使能
//!   IER.TMEIE/FMPIE0 → 发送/接收同步挂起对应 NVIC IRQ。
//!
//! 期望结果区：
//!   0x20000000 G_TX2RX_OK = 1（Phase A 标准帧互联校验）
//!   0x20000004 G_RX2TX_OK = 1（Phase B 扩展帧互联校验）
//!   0x20000008 G_FIFO_OK  = 1（Phase C FIFO 排队/RFOM 释放校验）
//!   0x2000000C G_DONE     = 0xAAAAAAAA（主线完成）

use std::path::Path;

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::can::{
    CanFrame, CAN1_BASE, CAN1_IRQ_RX0, CAN1_IRQ_SCE, CAN1_IRQ_TX, CAN2_BASE, CAN2_IRQ_RX0,
    CAN2_IRQ_TX,
};

const G_TX2RX_OK: u32 = 0x2000_0000;
const G_RX2TX_OK: u32 = 0x2000_0004;
const G_FIFO_OK: u32 = 0x2000_0008;
const G_DONE: u32 = 0x2000_000C;

/// CAN 寄存器偏移（与固件/外设一致）
const OFF_RF0R: u32 = 0x0C;
const OFF_IER: u32 = 0x14;
const OFF_ESR: u32 = 0x18;

/// 位定义（与 src/peripheral/can.rs 一致）
const IER_ERRIE: u32 = 1 << 7;
const ESR_EWGF: u32 = 1 << 0;
const ESR_EPVF: u32 = 1 << 1;
const ESR_BOFF: u32 = 1 << 2;
const ESR_LEC_MASK: u32 = 0x7 << 3;
const RF_FMP_MASK: u32 = 0x3;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/can_demo/can_demo.elf");
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
fn m15_can_end_to_end() {
    let mut m = load_machine();

    // 发送在写 TIRx 时同步完成并发布/路由，单次 run 即可完成（循环仅保险）。
    for _ in 0..100 {
        m.run(500_000).unwrap();
        if read_u32(&mut m, G_DONE) == 0xAAAA_AAAA {
            break;
        }
    }
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_TX2RX_OK), 1, "Phase A CAN1→CAN2 标准帧校验应通过");
    assert_eq!(read_u32(&mut m, G_RX2TX_OK), 1, "Phase B CAN2→CAN1 扩展帧校验应通过");
    assert_eq!(read_u32(&mut m, G_FIFO_OK), 1, "Phase C FIFO 排队/RFOM 释放校验应通过");

    // 中断挂起：固件使能 IER.TMEIE/FMPIE0，发送/接收同步挂起对应 NVIC IRQ。
    // 固件未使能 NVIC（不写 ISER），pending 位保持置位供校验。
    let nvic = m.nvic.lock().unwrap();
    assert!(nvic.is_pending(CAN1_IRQ_TX), "CAN1 TX IRQ 应挂起（发送）");
    assert!(nvic.is_pending(CAN2_IRQ_TX), "CAN2 TX IRQ 应挂起（Phase B/C 发送）");
    assert!(nvic.is_pending(CAN1_IRQ_RX0), "CAN1 RX0 IRQ 应挂起（Phase B/C 接收）");
    assert!(nvic.is_pending(CAN2_IRQ_RX0), "CAN2 RX0 IRQ 应挂起（Phase A 接收）");
}

/// 事件级互联：CanFrame 事件（port=1/2）经 machine 订阅路由到对端 CAN FIFO。
/// 不经固件，直接验证 machine/mod.rs 的事件互联接线。
#[test]
fn m15_can_event_interconnect() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    let bus = m.bus.clone();

    // 使能两端外设级接收中断（IER.FMPIE0），帧路由入 FIFO 时应挂起 RX0 IRQ
    const IER_FMPIE0: u32 = 1 << 1;
    bus.lock()
        .unwrap()
        .write(CAN1_BASE + OFF_IER, 4, IER_FMPIE0)
        .unwrap();
    bus.lock()
        .unwrap()
        .write(CAN2_BASE + OFF_IER, 4, IER_FMPIE0)
        .unwrap();

    // CAN1 发布（port=1）→ 路由到 CAN2 FIFO0（CAN2 全收：FA1R=0）
    let frame = CanFrame::new(1, 0x123, false, false, 2, [0x11, 0x22, 0, 0, 0, 0, 0, 0]);
    m.events
        .lock()
        .unwrap()
        .publish(&Event::CanFrame { frame: Box::new(frame) });
    let fmp = bus.lock().unwrap().read(CAN2_BASE + OFF_RF0R, 4).unwrap() & RF_FMP_MASK;
    assert_eq!(fmp, 1, "port=1 帧应路由到 CAN2 FIFO0");
    assert!(m.nvic.lock().unwrap().is_pending(CAN2_IRQ_RX0), "CAN2 RX0 IRQ 应挂起");

    // 反向：CAN2 发布（port=2）→ 路由到 CAN1 FIFO0
    let frame = CanFrame::new(2, 0x456, false, false, 1, [0xAA, 0, 0, 0, 0, 0, 0, 0]);
    m.events
        .lock()
        .unwrap()
        .publish(&Event::CanFrame { frame: Box::new(frame) });
    let fmp = bus.lock().unwrap().read(CAN1_BASE + OFF_RF0R, 4).unwrap() & RF_FMP_MASK;
    assert_eq!(fmp, 1, "port=2 帧应路由到 CAN1 FIFO0");
    assert!(m.nvic.lock().unwrap().is_pending(CAN1_IRQ_RX0), "CAN1 RX0 IRQ 应挂起");
}

/// 错误管理：inject_error 置 ESR 标志 + LEC，ERRIE 使能时挂起 SCE IRQ。
#[test]
fn m15_can_error_injection() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    let bus = m.bus.clone();

    // 使能错误中断（IER.ERRIE）
    bus.lock()
        .unwrap()
        .write(CAN1_BASE + OFF_IER, 4, IER_ERRIE)
        .unwrap();
    assert_eq!(
        bus.lock().unwrap().read(CAN1_BASE + OFF_IER, 4).unwrap(),
        IER_ERRIE,
        "IER 应回读"
    );

    // 错误警告：EWGF + LEC=1（位填充）
    m.can1.lock().unwrap().inject_error(true, false, false);
    let esr = bus.lock().unwrap().read(CAN1_BASE + OFF_ESR, 4).unwrap();
    assert_ne!(esr & ESR_EWGF, 0, "EWGF 应置位");
    assert_eq!((esr & ESR_LEC_MASK) >> 3, 1, "LEC 应为 1（位填充）");
    assert!(m.nvic.lock().unwrap().is_pending(CAN1_IRQ_SCE), "SCE IRQ 应挂起");

    // 总线关闭：BOFF + LEC=5（位错误），EWGF/EPVF 互斥清除
    m.can1.lock().unwrap().inject_error(false, false, true);
    let esr = bus.lock().unwrap().read(CAN1_BASE + OFF_ESR, 4).unwrap();
    assert_ne!(esr & ESR_BOFF, 0, "BOFF 应置位");
    assert_eq!(esr & (ESR_EWGF | ESR_EPVF), 0, "EWGF/EPVF 应互斥清除");
    assert_eq!((esr & ESR_LEC_MASK) >> 3, 5, "LEC 应为 5（位错误）");

    // 清除错误注入：ESR 全清（错误计数不复用，标志注入模型）
    m.can1.lock().unwrap().inject_error(false, false, false);
    let esr = bus.lock().unwrap().read(CAN1_BASE + OFF_ESR, 4).unwrap();
    assert_eq!(esr & (ESR_EWGF | ESR_EPVF | ESR_BOFF | ESR_LEC_MASK), 0, "错误标志应清除");
}
