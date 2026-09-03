//! M9 验收测试：RNG 真随机数发生器（STM32F407）端到端 + 寄存器级。
//!
//! 复用 firmware/rng_demo 固件（场景见其 main.c 注释）：
//!   Phase A：CR.RNGEN=1 使能 → 轮询 SR.DRDY → 读 DR 得 v1 → 轮询 DRDY 再次置位
//!            （连续生成）→ 读 DR 得 v2 → v1 != v2 且 v2 != 0 → G_RNG_OK=1；
//!   主线写 G_DONE。
//! 期望结果区：
//!   0x20000000 G_RNG_OK = 1（两次读取随机值不同且非零）
//!   0x20000004 G_DONE   = 0xAAAAAAAA（主线完成）
//!
//! 另含总线直写测试：不经固件验证寄存器级语义（DRDY 置位/清除、连续生成、
//! CECS/SECS 错误注入 + RNG IRQ80 挂起、RESET 清零），错误经 [`Machine::rng`]
//! 句柄注入（模拟外部时钟/种子异常，测试种子可控可复现）。

use std::path::Path;

use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::rng::RNG_IRQ;

const G_RNG_OK: u32 = 0x2000_0000;
const G_DONE: u32 = 0x2000_0004;

/// RNG 寄存器地址
const RNG_CR: u32 = 0x5006_0800;
const RNG_SR: u32 = 0x5006_0804;
const RNG_DR: u32 = 0x5006_0808;

/// 位定义（与 src/peripheral/rng.rs 一致）
const CR_RNGEN: u32 = 1 << 2;
const CR_IE: u32 = 1 << 3;
const SR_DRDY: u32 = 1 << 0;
const SR_CECS: u32 = 1 << 1;
const SR_SECS: u32 = 1 << 2;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/rng_demo/rng_demo.elf");
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
fn m9_rng_end_to_end() {
    let mut m = load_machine();
    m.run(100_000).unwrap();

    // Phase A：使能后两次读取随机值不同且非零
    assert_eq!(read_u32(&mut m, G_RNG_OK), 1, "两次读取随机值应不同且非零");
    // 主线完成
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
}

/// 总线直写：不经固件，验证 RNG 寄存器级语义 + 错误注入 + 中断。
#[test]
fn m9_rng_bus_direct() {
    let m = load_machine();
    let nvic = m.nvic.clone();

    // 使能 → SR.DRDY 置位
    m.bus.lock().unwrap().write(RNG_CR, 4, CR_RNGEN).unwrap();
    let sr = m.bus.lock().unwrap().read(RNG_SR, 4).unwrap();
    assert_ne!(sr & SR_DRDY, 0, "使能后 SR.DRDY 应置位");

    // 连续读 DR 两次 → 值不同（连续生成）
    let v1 = m.bus.lock().unwrap().read(RNG_DR, 4).unwrap();
    let v2 = m.bus.lock().unwrap().read(RNG_DR, 4).unwrap();
    assert_ne!(v1, v2, "连续读 DR 应返回不同随机值");
    assert_ne!(v2, 0);

    // 注入时钟错误 → SR.CECS 置位；未使能 IE → 不挂起中断
    m.rng.lock().unwrap().inject_clock_error(true);
    let sr = m.bus.lock().unwrap().read(RNG_SR, 4).unwrap();
    assert_ne!(sr & SR_CECS, 0, "注入时钟错误应置 CECS");
    assert!(
        !nvic.lock().unwrap().is_pending(RNG_IRQ),
        "未使能 IE 不应挂起 RNG 中断"
    );

    // 使能 IE 后注入种子错误 → SR.SECS 置位且挂起 RNG 中断
    m.bus.lock().unwrap().write(RNG_CR, 4, CR_RNGEN | CR_IE).unwrap();
    m.rng.lock().unwrap().inject_seed_error(true);
    let sr = m.bus.lock().unwrap().read(RNG_SR, 4).unwrap();
    assert_ne!(sr & SR_SECS, 0, "注入种子错误应置 SECS");
    assert!(
        nvic.lock().unwrap().is_pending(RNG_IRQ),
        "IE + 种子错误应挂起 RNG 中断"
    );

    // 清除注入 → 错误位清除
    m.rng.lock().unwrap().inject_seed_error(false);
    let sr = m.bus.lock().unwrap().read(RNG_SR, 4).unwrap();
    assert_eq!(sr & SR_SECS, 0, "清除注入应清 SECS");

    // 外设复位 → SR 清零
    m.bus.lock().unwrap().reset();
    assert_eq!(
        m.bus.lock().unwrap().read(RNG_SR, 4).unwrap(),
        0,
        "复位后 SR 应清零"
    );
}
