//! M4 验收测试：MPU 完善——重叠 region 优先级（最高编号优先，ARMv7-M B3.5.4）。
//!
//! 复用 M0 的 fp_acceptance 固件（含浮点运算，末尾向 0x20000000 写结果）。
//! 两条链路对比证明"最高编号 region 优先"（而非首条命中）：
//! 1. region0=可写(rw)、region7=只读(ro) 覆盖同一地址 → 写 0x20000000 触发 DACCVIOL（region7 胜）；
//! 2. region0=只读(ro)、region7=可写(rw) 覆盖同一地址 → 写 0x20000000 放行（region7 胜），固件跑完。
//!
//! MPU 寄存器经内存总线写入（地址为 STM32F407 SCB 内绝对地址）。

use std::path::Path;

use mcu_simulater::core::CoreError;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::mpu::MemManageKind;

const MPU_CTRL: u32 = 0xE000_ED94;
const MPU_RBAR: u32 = 0xE000_ED9C;
const MPU_RASR: u32 = 0xE000_EDA0;
const MMFSR: u32 = 0xE000_ED28;
const MMFAR: u32 = 0xE000_ED34;

/// 加载并复位 M0 固件
fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/fp_acceptance/fp_acceptance.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

/// 配置两个重叠 region：[0x20000000, +64B)，分别指定 AP。
/// RBAR 经 VALID=1 + REGION 字段选中对应 region。
fn setup_overlap_regions(
    m: &mut Machine,
    region0_ap: u32,
    region7_ap: u32,
) {
    let bus = m.bus.lock().unwrap();
    // region0
    bus.write(MPU_RBAR, 4, 0x2000_0000 | (1 << 4)).unwrap();
    // SIZE=5 → 64B；AP 由调用方给定；ENABLE
    bus.write(MPU_RASR, 4, (5 << 1) | (region0_ap << 24) | 1).unwrap();
    // region7（VALID + REGION=7，同时更新 RNR=7）
    bus.write(MPU_RBAR, 4, 0x2000_0000 | (1 << 4) | 7).unwrap();
    bus.write(MPU_RASR, 4, (5 << 1) | (region7_ap << 24) | 1).unwrap();
    // ENABLE + PRIVDEFENA：区域外特权访问走后台 region 放行
    bus.write(MPU_CTRL, 4, 0x5).unwrap();
}

#[test]
fn m4_mpu_overlap_high_region_restrictive_blocks() {
    let mut m = load_machine();
    // region0=AP011(全 rw)，region7=AP101(特权 ro)：写应被 region7 拒绝
    setup_overlap_regions(&mut m, 0b011, 0b101);

    // 固件末尾写 0x20000000 应触发 DACCVIOL（region7 优先于 region0）
    let err = m.run(200_000).expect_err("高编号只读 region 应拒绝写访问");
    match err {
        CoreError::MemManageFault { addr, kind } => {
            assert_eq!(addr, 0x2000_0000, "故障地址应为被写地址");
            assert_eq!(kind, MemManageKind::DataAccess);
        }
        other => panic!("期望 MemManageFault，得到 {other:?}"),
    }

    // 故障状态寄存器：DACCVIOL(bit1) + MMARVALID(bit7)，MMFAR 记录地址
    let bus = m.bus.lock().unwrap();
    assert_ne!(bus.read(MMFSR, 4).unwrap() & 0x2, 0, "MMFSR.DACCVIOL 应置位");
    assert_ne!(bus.read(MMFSR, 4).unwrap() & 0x80, 0, "MMFSR.MMARVALID 应置位");
    assert_eq!(bus.read(MMFAR, 4).unwrap(), 0x2000_0000);
}

#[test]
fn m4_mpu_overlap_high_region_permissive_allows() {
    let mut m = load_machine();
    // region0=AP101(特权 ro)，region7=AP011(全 rw)：写应被 region7 放行
    setup_overlap_regions(&mut m, 0b101, 0b011);

    m.run(200_000).unwrap();

    // 固件正常跑完，结果仍为 12，且无 fault 记录
    let out = m.cpu.mem_read(0x2000_0000, 4).unwrap();
    assert_eq!(u32::from_le_bytes(out.try_into().unwrap()), 0x0000_000C);
    assert!(
        m.mpu.lock().unwrap().pending_fault().is_none(),
        "高编号可写 region 下写访问应放行，不应记录故障"
    );
}
