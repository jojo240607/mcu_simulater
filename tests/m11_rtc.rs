//! M11 验收测试：RTC 实时时钟 + 备份寄存器（STM32F407）端到端 + 寄存器级。
//!
//! 复用 firmware/rtc_demo 固件（场景见其 main.c 注释）：
//!   Phase A：RTC 初始化（WPR 解锁 → INIT/INITF → 写 PRER/DR/TR → INIT=0 等 RSF）
//!     → 校验 INITS → G_RTC_INIT_OK=1；
//!   Phase B：备份寄存器——未置 PWR_CR.DBP 时写 BKP0R 被忽略（保持 0）→ 置 DBP
//!     后写 BKP0R/BKP1R 读回一致 → G_BKP_NODBP=1、G_BKP_OK=1；
//!   Phase C：闹钟 A（23:59:57）+ 唤醒定时器（ck_spre 1Hz）→ 开 NVIC IRQ3/IRQ41；
//!     时间推进到匹配点后 RTC_Alarm_IRQHandler / RTC_WKUP_IRQHandler 各置计数
//!     → 主线停掉 RTC 中断源（避免中断风暴阻止 run() 返回）→ G_DONE。
//! 期望结果区：
//!   0x20000000 G_RTC_INIT_OK = 1（初始化序列完成、INITS 置位）
//!   0x20000004 G_BKP_NODBP   = 1（未解锁备份域时 BKP 写被忽略）
//!   0x20000008 G_BKP_OK      = 1（解锁后 BKP 写读一致）
//!   0x2000000C G_ALARM_IRQ   ≥ 1（RTC_Alarm IRQ41 handler 触发次数）
//!   0x20000010 G_WAKEUP_IRQ  ≥ 1（RTC_WKUP IRQ3 handler 触发次数）
//!   0x20000014 G_TR          = 0x00235955（轮询开始时 TR）
//!   0x20000018 G_DONE        = 0xAAAAAAAA（主线完成）
//!
//! 另含总线直写测试：不经固件验证寄存器级语义（写保护、初始化序列、日历推进、
//! 闹钟匹配+中断挂起、唤醒定时器周期、BKP 的 DBP 门控与掉电保持）。

use std::path::Path;

use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::rtc::{RTC_ALARM_IRQ, RTC_WKUP_IRQ};
use mcu_simulater::peripheral::Peripheral;

/* RTC @ 0x40002800 */
const RTC_BASE: u32 = 0x4000_2800;
const RTC_TR: u32 = RTC_BASE + 0x00;
const RTC_DR: u32 = RTC_BASE + 0x04;
const RTC_CR: u32 = RTC_BASE + 0x08;
const RTC_ISR: u32 = RTC_BASE + 0x0C;
const RTC_PRER: u32 = RTC_BASE + 0x10;
const RTC_WUTR: u32 = RTC_BASE + 0x14;
const RTC_ALRMAR: u32 = RTC_BASE + 0x1C;
const RTC_WPR: u32 = RTC_BASE + 0x24;
const RTC_SSR: u32 = RTC_BASE + 0x28;
const RTC_BKP0R: u32 = RTC_BASE + 0x50;
const RTC_BKP19R: u32 = RTC_BASE + 0x9C;

/* PWR @ 0x40007000 */
const PWR_CR: u32 = 0x4000_7000;
const CR_DBP: u32 = 1 << 8;

/* ISR 位 */
const ISR_INITS: u32 = 1 << 4;
const ISR_RSF: u32 = 1 << 5;
const ISR_INITF: u32 = 1 << 6;
const ISR_INIT: u32 = 1 << 7;
const ISR_ALRAF: u32 = 1 << 8;
const ISR_WUTF: u32 = 1 << 10;

/* CR 位 */
const CR_WUCKSEL_CKSPRE: u32 = 4 << 0;
const CR_ALRAE: u32 = 1 << 8;
const CR_WUTE: u32 = 1 << 10;
const CR_ALRAIE: u32 = 1 << 12;
const CR_WUTIE: u32 = 1 << 14;

/* ALRMAR：MSK4=1 屏蔽日期 */
const ALR_MSK4: u32 = 1 << 31;

/* 结果区（与固件 main.c 一致） */
const G_RTC_INIT_OK: u32 = 0x2000_0000;
const G_BKP_NODBP: u32 = 0x2000_0004;
const G_BKP_OK: u32 = 0x2000_0008;
const G_ALARM_IRQ: u32 = 0x2000_000C;
const G_WAKEUP_IRQ: u32 = 0x2000_0010;
const G_TR: u32 = 0x2000_0014;
const G_DONE: u32 = 0x2000_0018;

/// 默认 PRER 周期（PREDIV_A=127, PREDIV_S=255 → 每 ck_spre 32768 个 RTCCLK 周期）
const RTC_PERIOD: u64 = 32768;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/rtc_demo/rtc_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

fn make_machine() -> Machine {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m
}

fn read_u32(m: &mut Machine, addr: u32) -> u32 {
    let out = m.cpu.mem_read(addr as u64, 4).unwrap();
    u32::from_le_bytes(out.try_into().unwrap())
}

/// 解锁 RTC 写保护（WPR 依次写 0xCA、0x53）。
fn rtc_unlock(m: &mut Machine) {
    m.bus.lock().unwrap().write(RTC_WPR, 1, 0xCA).unwrap();
    m.bus.lock().unwrap().write(RTC_WPR, 1, 0x53).unwrap();
}

/// 进入初始化模式：写 ISR.INIT=1 并等待 INITF。
fn rtc_enter_init(m: &mut Machine) {
    let bus = m.bus.clone();
    let b = bus.lock().unwrap();
    b.write(RTC_ISR, 4, ISR_INIT).unwrap();
    assert_ne!(b.read(RTC_ISR, 4).unwrap() & ISR_INITF, 0, "应进入初始化模式（INITF=1）");
}

/// 端到端：固件走完整 RTC 初始化 → BKP → 闹钟/唤醒中断链路。
#[test]
fn m11_rtc_end_to_end() {
    let mut m = load_machine();
    m.run(1_000_000).unwrap();

    assert_eq!(read_u32(&mut m, G_RTC_INIT_OK), 1, "RTC 初始化序列应完成（INITS 置位）");
    assert_eq!(read_u32(&mut m, G_BKP_NODBP), 1, "未解锁备份域时 BKP 写应被忽略");
    assert_eq!(read_u32(&mut m, G_BKP_OK), 1, "解锁后 BKP 写读应一致");
    assert_eq!(read_u32(&mut m, G_ALARM_IRQ), 1, "RTC_Alarm(IRQ41) handler 应触发一次");
    assert!(
        read_u32(&mut m, G_WAKEUP_IRQ) >= 1,
        "RTC_WKUP(IRQ3) handler 应至少触发一次"
    );
    assert_eq!(read_u32(&mut m, G_TR), 0x0023_5955, "TR 应为写入的 23:59:55");
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
}

/// 寄存器级：写保护（WPR 0xCA→0x53 解锁；错键重新上锁；上锁期写被忽略）。
#[test]
fn m11_rtc_write_protection() {
    let mut m = make_machine();

    // 未解锁：写 ISR.INIT 无效（仍为 0，未进入初始化）
    m.bus.lock().unwrap().write(RTC_ISR, 4, ISR_INIT).unwrap();
    let isr = m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap();
    assert_eq!(isr & ISR_INITF, 0, "未解锁写 INIT 应被忽略");

    // 错键（0xCA → 0xAA）：保持上锁
    m.bus.lock().unwrap().write(RTC_WPR, 1, 0xCA).unwrap();
    m.bus.lock().unwrap().write(RTC_WPR, 1, 0xAA).unwrap();
    m.bus.lock().unwrap().write(RTC_ISR, 4, ISR_INIT).unwrap();
    let isr = m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap();
    assert_eq!(isr & ISR_INITF, 0, "错键后应仍上锁，写 INIT 被忽略");

    // 正确序列：0xCA → 0x53 解锁
    rtc_unlock(&mut m);
    m.bus.lock().unwrap().write(RTC_ISR, 4, ISR_INIT).unwrap();
    let isr = m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap();
    assert_ne!(isr & ISR_INITF, 0, "解锁后写 INIT 应进入初始化模式");
}

/// 寄存器级：初始化序列（INIT/INITF → 写 PRER/DR/TR → INIT=0 等 RSF/INITS）与 TR/DR 读回。
#[test]
fn m11_rtc_init_sequence() {
    let mut m = make_machine();
    rtc_unlock(&mut m);

    // 进入初始化 → INITF=1；INITS 尚未置位
    rtc_enter_init(&mut m);
    let isr = m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap();
    assert_eq!(isr & ISR_INITS, 0, "初始化完成前 INITS 应为 0");

    // 写 PRER / DR / TR（DR 先写：write_dr 保留时分秒并置秒=0；TR 后写保留日期）
    m.bus.lock().unwrap().write(RTC_PRER, 4, 0x007F_00FF).unwrap();
    m.bus.lock().unwrap().write(RTC_DR, 4, 0x0024_C601).unwrap(); // 2024-06-01
    m.bus.lock().unwrap().write(RTC_TR, 4, 0x0023_5955).unwrap(); // 23:59:55

    // 退出初始化 → RSF=1、INITS=1
    m.bus.lock().unwrap().write(RTC_ISR, 4, 0).unwrap(); // INIT=0
    let isr = m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap();
    assert_ne!(isr & ISR_RSF, 0, "退出初始化后 RSF 应为 1");
    assert_ne!(isr & ISR_INITS, 0, "写日期后 INITS 应为 1");

    // TR/DR 读回一致（BCD；星期由历法自动计算：2024-06-01 = 周六 = 6）
    assert_eq!(m.bus.lock().unwrap().read(RTC_TR, 4).unwrap(), 0x0023_5955);
    let dr = m.bus.lock().unwrap().read(RTC_DR, 4).unwrap();
    assert_eq!(dr & 0x0000_003F, 0x01, "日应为 01");
    assert_eq!(dr & 0x0000_3F00, 0x0000_0600, "月应为 06");
    assert_eq!((dr >> 13) & 0x7, 6, "2024-06-01 星期应为周六(6)");
    assert_eq!(dr >> 16, 0x24, "年应为 24");
}

/// 寄存器级：日历推进（每 32768 周期推进 1 秒；TR/SSR 更新）。
#[test]
fn m11_rtc_calendar_advance() {
    let mut m = make_machine();
    rtc_unlock(&mut m);

    // 复位默认时间 2021-01-01 00:00:00（RTC::new 复位值）
    assert_eq!(m.bus.lock().unwrap().read(RTC_TR, 4).unwrap(), 0x0000_0000);
    // 复位默认 PRER=0x007F00FF（PREDIV_A=127 / PREDIV_S=255），SSR=PREDIV_S=0xFF
    assert_eq!(m.bus.lock().unwrap().read(RTC_PRER, 4).unwrap(), 0x007F_00FF);
    assert_eq!(m.bus.lock().unwrap().read(RTC_SSR, 4).unwrap(), 0xFF);

    // 推进 1 秒
    m.rtc.lock().unwrap().tick(RTC_PERIOD);
    let tr_dbg = m.bus.lock().unwrap().read(RTC_TR, 4).unwrap();
    assert_eq!(tr_dbg, 0x0000_0001);

    // 推进 59 秒 → 00:01:00
    m.rtc.lock().unwrap().tick(RTC_PERIOD * 59);
    assert_eq!(m.bus.lock().unwrap().read(RTC_TR, 4).unwrap(), 0x0000_0100);

    // 推进 3600 秒 → 01:01:00（时分秒 BCD；00:01:00 + 3600s）
    m.rtc.lock().unwrap().tick(RTC_PERIOD * 3600);
    assert_eq!(m.bus.lock().unwrap().read(RTC_TR, 4).unwrap(), 0x0001_0100);
}

/// 寄存器级：闹钟 A 匹配 → ALRAF + NVIC 挂起 IRQ41；清除后不再重触发。
#[test]
fn m11_rtc_alarm() {
    let mut m = make_machine();
    rtc_unlock(&mut m);

    // 初始化到 23:59:55（DR 先写）
    rtc_enter_init(&mut m);
    m.bus.lock().unwrap().write(RTC_DR, 4, 0x0024_C601).unwrap();
    m.bus.lock().unwrap().write(RTC_TR, 4, 0x0023_5955).unwrap();
    m.bus.lock().unwrap().write(RTC_ISR, 4, 0).unwrap();

    // 闹钟 23:59:57（MSK4 屏蔽日期）+ 使能 + 中断使能
    m.bus
        .lock()
        .unwrap()
        .write(RTC_ALRMAR, 4, ALR_MSK4 | (0x23 << 16) | (0x59 << 8) | 0x57)
        .unwrap();
    m.bus
        .lock()
        .unwrap()
        .write(RTC_CR, 4, CR_ALRAE | CR_ALRAIE)
        .unwrap();

    assert_eq!(
        m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap() & ISR_ALRAF,
        0,
        "未到匹配秒不应置 ALRAF"
    );
    assert!(!m.nvic.lock().unwrap().is_pending(RTC_ALARM_IRQ), "未匹配不应挂起中断");

    // 推进 2 秒 → 23:59:57 匹配
    m.rtc.lock().unwrap().tick(RTC_PERIOD * 2);
    let isr = m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap();
    assert_ne!(isr & ISR_ALRAF, 0, "匹配秒应置 ALRAF");
    assert!(m.nvic.lock().unwrap().is_pending(RTC_ALARM_IRQ), "应挂起 RTC_Alarm(IRQ41)");

    // 清除 ALRAF → 推进到 58 秒不重触发
    m.bus.lock().unwrap().write(RTC_ISR, 4, isr & !ISR_ALRAF).unwrap();
    m.rtc.lock().unwrap().tick(RTC_PERIOD);
    assert_eq!(
        m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap() & ISR_ALRAF,
        0,
        "清除后不匹配新秒不应重触发"
    );
}

/// 寄存器级：唤醒定时器（ck_spre 1Hz）周期触发 → WUTF + NVIC 挂起 IRQ3。
#[test]
fn m11_rtc_wakeup() {
    let mut m = make_machine();
    rtc_unlock(&mut m);

    // WUCKSEL=ck_spre（WUTE=0 时可写）→ WUTR=0（1 秒周期）→ 使能 WUTE|WUTIE
    m.bus
        .lock()
        .unwrap()
        .write(RTC_CR, 4, CR_WUCKSEL_CKSPRE)
        .unwrap();
    m.bus.lock().unwrap().write(RTC_WUTR, 4, 0).unwrap();
    m.bus
        .lock()
        .unwrap()
        .write(RTC_CR, 4, CR_WUCKSEL_CKSPRE | CR_WUTE | CR_WUTIE)
        .unwrap();

    assert_eq!(
        m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap() & ISR_WUTF,
        0,
        "使能瞬间不应置 WUTF"
    );

    // 1 秒后首次唤醒
    m.rtc.lock().unwrap().tick(RTC_PERIOD);
    let isr = m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap();
    assert_ne!(isr & ISR_WUTF, 0, "1 秒后应置 WUTF");
    assert!(m.nvic.lock().unwrap().is_pending(RTC_WKUP_IRQ), "应挂起 RTC_WKUP(IRQ3)");

    // 清除 WUTF → 再 1 秒 → 周期再次触发
    m.bus.lock().unwrap().write(RTC_ISR, 4, isr & !ISR_WUTF).unwrap();
    m.rtc.lock().unwrap().tick(RTC_PERIOD);
    let isr2 = m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap();
    assert_ne!(isr2 & ISR_WUTF, 0, "唤醒定时器应周期触发");

    // 清除 WUTF 并停用 WUTE → 不再触发
    m.bus.lock().unwrap().write(RTC_ISR, 4, isr2 & !ISR_WUTF).unwrap();
    m.bus
        .lock()
        .unwrap()
        .write(RTC_CR, 4, CR_WUCKSEL_CKSPRE)
        .unwrap(); // WUTE=0
    m.rtc.lock().unwrap().tick(RTC_PERIOD * 3);
    assert_eq!(
        m.bus.lock().unwrap().read(RTC_ISR, 4).unwrap() & ISR_WUTF,
        0,
        "WUTE=0 后不应再触发"
    );
}

/// 寄存器级：备份寄存器（BKP）——写访问需 PWR_CR.DBP=1；外设复位保留数据。
#[test]
fn m11_rtc_bkp() {
    let m = make_machine();

    // 未置 DBP：写被忽略（读恒 0）
    m.bus.lock().unwrap().write(RTC_BKP0R, 4, 0xDEAD_BEEF).unwrap();
    assert_eq!(
        m.bus.lock().unwrap().read(RTC_BKP0R, 4).unwrap(),
        0,
        "未解锁备份域写 BKP0R 应被忽略"
    );

    // 置 DBP（PWR_CR bit8）→ 写读一致（首尾寄存器）
    let b = m.bus.lock().unwrap();
    b.write(PWR_CR, 4, CR_DBP).unwrap();
    b.write(RTC_BKP0R, 4, 0xCAFE_BABE).unwrap();
    b.write(RTC_BKP19R, 4, 0x1234_5678).unwrap();
    assert_eq!(b.read(RTC_BKP0R, 4).unwrap(), 0xCAFE_BABE, "BKP0R 写读一致");
    assert_eq!(b.read(RTC_BKP19R, 4).unwrap(), 0x1234_5678, "BKP19R 写读一致");

    // 外设复位：BKP 掉电保持（VBAT 域），其余寄存器复位
    b.reset();
    assert_eq!(b.read(RTC_BKP0R, 4).unwrap(), 0xCAFE_BABE, "外设复位 BKP 应保持");
    assert_eq!(b.read(RTC_BKP19R, 4).unwrap(), 0x1234_5678, "外设复位 BKP19R 应保持");

    // 复位后写保护重新上锁：TR 恢复默认、且写 INIT 被忽略
    assert_eq!(b.read(RTC_TR, 4).unwrap(), 0x0000_0000, "复位后 TR 应回默认 00:00:00");
    b.write(RTC_ISR, 4, ISR_INIT).unwrap();
    assert_eq!(b.read(RTC_ISR, 4).unwrap() & ISR_INITF, 0, "复位后写保护应重新上锁");
}
