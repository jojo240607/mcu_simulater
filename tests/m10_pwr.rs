//! M10 验收测试：PWR 电源控制（STM32F407）端到端 + 寄存器级。
//!
//! 复用 firmware/pwr_demo 固件（场景见其 main.c 注释）：
//!   Phase A：写 CR 低功耗位（LPDS|PDDS）→ 读回校验一致 → G_PWR_CR_OK=1；
//!   Phase B：写 CSR 可写位（EWUP|BRE）→ 读回校验一致 → G_PWR_CSR_OK=1；
//!   写 G_PWR_READY=1（等待测试注入 WKUP 唤醒→待机唤醒复位）；
//!   测试经 [`Machine::pwr`] 注入 enter_standby（模拟 WFI/WFE 进待机，置 CSR.SBF）
//!   再 inject_wakeup（WKUP 唤醒）→ 置 CSR.WUF 并发出 LowPower 复位请求；
//!   run() 消费请求执行系统复位（RCC_CSR.LPWRRSTF 置位）→ 固件再进 Reset_Handler：
//!   检测到 LPWRRSTF → G_PWR_WOKE=1、G_RESET_FLAG=CSR、PWR_CSR.SBF 保持置位
//!   → G_PWR_SBF=1；主线写 G_DONE。
//! 期望结果区：
//!   0x20000000 G_PWR_CR_OK  = 1（低功耗位写读一致）
//!   0x20000004 G_PWR_CSR_OK = 1（CSR 可写位写读一致）
//!   0x20000008 G_PWR_READY  = 1（低功耗位已设置）
//!   0x2000000C G_PWR_WOKE   = 1（待机唤醒复位后检测到 LPWRRSTF）
//!   0x20000010 G_PWR_SBF    = 1（待机唤醒复位后 PWR_CSR.SBF 保持）
//!   0x20000014 G_RESET_FLAG 复位后 RCC_CSR（含 LPWRRSTF=bit31）
//!   0x20000018 G_DONE       = 0xAAAAAAAA（主线完成）
//!
//! 另含总线直写测试：不经固件验证寄存器级语义（CR/CSR 写读、rc_w1 清标志、
//! PVD 注入、待机唤醒复位请求链路）。

use std::path::Path;

use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::wdog::ResetReason;

const G_PWR_CR_OK: u32 = 0x2000_0000;
const G_PWR_CSR_OK: u32 = 0x2000_0004;
const G_PWR_READY: u32 = 0x2000_0008;
const G_PWR_WOKE: u32 = 0x2000_000C;
const G_PWR_SBF: u32 = 0x2000_0010;
const G_RESET_FLAG: u32 = 0x2000_0014;
const G_DONE: u32 = 0x2000_0018;

/// PWR 寄存器地址
const PWR_CR: u32 = 0x4000_7000;
const PWR_CSR: u32 = 0x4000_7004;
/// RCC_CSR @ 0x40023874
const RCC_CSR: u32 = 0x4002_3874;

/// 位定义（与 src/peripheral/pwr.rs 一致）
const CR_LPDS: u32 = 1 << 0;
const CR_PDDS: u32 = 1 << 1;
const CR_CWUF: u32 = 1 << 2;
const CR_CSBF: u32 = 1 << 3;
const CSR_WUF: u32 = 1 << 0;
const CSR_SBF: u32 = 1 << 1;
const CSR_PVDO: u32 = 1 << 2;
const CSR_EWUP: u32 = 1 << 8;
const CSR_BRE: u32 = 1 << 9;
const CSR_LPWRRSTF: u32 = 1 << 31;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/pwr_demo/pwr_demo.elf");
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

/// 端到端：固件 Phase A/B → 测试注入待机+唤醒 → 待机唤醒复位 → 固件检测 LPWRRSTF。
#[test]
fn m10_pwr_end_to_end() {
    let mut m = load_machine();
    m.run(100_000).unwrap();

    // Phase A/B：低功耗位与 CSR 可写位写读一致
    assert_eq!(read_u32(&mut m, G_PWR_CR_OK), 1, "CR 低功耗位写读应一致");
    assert_eq!(read_u32(&mut m, G_PWR_CSR_OK), 1, "CSR 可写位写读应一致");
    assert_eq!(read_u32(&mut m, G_PWR_READY), 1, "低功耗位应已设置、等待唤醒");
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");

    // 注入待机 + 唤醒 → 触发 LowPower 复位请求
    m.pwr.lock().unwrap().enter_standby();
    m.pwr.lock().unwrap().inject_wakeup();

    // run() 消费复位请求 → 系统复位 → 固件再进 Reset_Handler 检测 LPWRRSTF
    m.run(100_000).unwrap();

    assert_eq!(read_u32(&mut m, G_PWR_WOKE), 1, "待机唤醒复位后应检测到 LPWRRSTF");
    assert_eq!(read_u32(&mut m, G_PWR_SBF), 1, "待机唤醒复位后 PWR_CSR.SBF 应保持");
    let flag = read_u32(&mut m, G_RESET_FLAG);
    assert_ne!(flag & CSR_LPWRRSTF, 0, "RCC_CSR 应含 LPWRRSTF（bit31），实际 0x{flag:08X}");
    // RCC_CSR 寄存器直读一致（复位标志经总线可见）
    let rcc_csr = m.bus.lock().unwrap().read(RCC_CSR, 4).unwrap();
    assert_ne!(rcc_csr & CSR_LPWRRSTF, 0, "总线读 RCC_CSR 应含 LPWRRSTF");
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
}

/// 总线直写：不经固件，验证 PWR 寄存器级语义 + 注入 + 复位请求链路。
#[test]
fn m10_pwr_bus_direct() {
    let m = load_machine();

    // CR 低功耗位写读一致
    m.bus.lock().unwrap().write(PWR_CR, 4, CR_LPDS | CR_PDDS).unwrap();
    let cr = m.bus.lock().unwrap().read(PWR_CR, 4).unwrap();
    assert_eq!(cr & (CR_LPDS | CR_PDDS), CR_LPDS | CR_PDDS, "CR 低功耗位应写读一致");

    // CSR 可写位写读一致；只读标志（WUF/SBF/PVDO）不受写影响
    m.bus.lock().unwrap().write(PWR_CSR, 4, CSR_EWUP | CSR_BRE).unwrap();
    let csr = m.bus.lock().unwrap().read(PWR_CSR, 4).unwrap();
    assert_eq!(csr & (CSR_EWUP | CSR_BRE), CSR_EWUP | CSR_BRE, "CSR 可写位应写读一致");
    assert_eq!(csr & (CSR_WUF | CSR_SBF | CSR_PVDO), 0, "只读标志不应被写置位");

    // 注入唤醒（未进待机）→ 仅置 CSR.WUF，不触发复位
    m.pwr.lock().unwrap().inject_wakeup();
    let csr = m.bus.lock().unwrap().read(PWR_CSR, 4).unwrap();
    assert_ne!(csr & CSR_WUF, 0, "注入唤醒应置 CSR.WUF");
    assert!(!m.wdog_req.is_pending(), "未进待机的唤醒不应触发复位请求");

    // CWUF 写 1 清除 WUF（rc_w1）
    m.bus.lock().unwrap().write(PWR_CR, 4, CR_CWUF).unwrap();
    let csr = m.bus.lock().unwrap().read(PWR_CSR, 4).unwrap();
    assert_eq!(csr & CSR_WUF, 0, "CWUF 写 1 应清除 CSR.WUF");

    // 注入 PVD 事件 → PVDO 置位；关闭 → 清除
    m.pwr.lock().unwrap().inject_pvd(true);
    let csr = m.bus.lock().unwrap().read(PWR_CSR, 4).unwrap();
    assert_ne!(csr & CSR_PVDO, 0, "注入 PVD 应置 CSR.PVDO");
    m.pwr.lock().unwrap().inject_pvd(false);
    let csr = m.bus.lock().unwrap().read(PWR_CSR, 4).unwrap();
    assert_eq!(csr & CSR_PVDO, 0, "关闭 PVD 应清除 CSR.PVDO");

    // 进待机 + 唤醒 → 触发 LowPower 复位请求（复用看门狗复位链路）
    m.pwr.lock().unwrap().enter_standby();
    let csr = m.bus.lock().unwrap().read(PWR_CSR, 4).unwrap();
    assert_ne!(csr & CSR_SBF, 0, "进待机应置 CSR.SBF");
    m.pwr.lock().unwrap().inject_wakeup();
    assert!(
        m.wdog_req.is_pending(),
        "待机唤醒应触发 LowPower 复位请求"
    );
    assert_eq!(
        m.wdog_req.take(),
        Some(ResetReason::LowPower),
        "复位请求原因应为 LowPower"
    );

    // CSBF 写 1 清除 SBF（rc_w1；注入前已 take 复位请求，此处仅验清除语义）
    m.pwr.lock().unwrap().inject_wakeup(); // 置 WUF（standby 仍 true → 又请求复位，忽略）
    let _ = m.wdog_req.take();
    m.bus.lock().unwrap().write(PWR_CR, 4, CR_CSBF).unwrap();
    let csr = m.bus.lock().unwrap().read(PWR_CSR, 4).unwrap();
    assert_eq!(csr & CSR_SBF, 0, "CSBF 写 1 应清除 CSR.SBF");

    // 外设复位 → CR/CSR 清零
    m.bus.lock().unwrap().reset();
    assert_eq!(m.bus.lock().unwrap().read(PWR_CR, 4).unwrap(), 0);
    assert_eq!(m.bus.lock().unwrap().read(PWR_CSR, 4).unwrap(), 0);
}
