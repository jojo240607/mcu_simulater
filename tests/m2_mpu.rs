//! M2 验收测试：MPU 全强制访问控制 + MemManage fault。
//!
//! 复用 M0 的 fp_acceptance 固件（含浮点运算，末尾向 0x20000000 写结果）。
//! 覆盖四条链路：
//! 1. 数据访问违规：SRAM 区域只读，写 0x20000000 → DACCVIOL + 停止执行；
//! 2. 取指违规：FLASH 区域 XN，复位后首条取指 → IACCVIOL；
//! 3. MPU 使能 + PRIVDEFENA：无 region 命中时特权访问放行，固件正常跑完；
//! 4. MPU 未使能（默认）：行为与 M0/M1 一致（回归）。
//!
//! MPU 寄存器经内存总线写入（地址为 STM32F407 SCB 内绝对地址）。

use std::path::Path;

use mcu_simulater::core::CoreError;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::mpu::MemManageKind;

const CPACR: u32 = 0xE000_ED88;
const MPU_CTRL: u32 = 0xE000_ED94;
const MPU_RBAR: u32 = 0xE000_ED9C;
const MPU_RASR: u32 = 0xE000_EDA0;
const MMFSR: u32 = 0xE000_ED28;
const MMFAR: u32 = 0xE000_ED34;

/// 加载并复位 M0 固件（未使能 MPU）
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

#[test]
fn m2_write_protect_sram_faults() {
    let mut m = load_machine();
    {
        let bus = m.bus.lock().unwrap();
        // region0 = [0x20000000, +64B)，AP=101（特权只读），ENABLE
        bus.write(MPU_RBAR, 4, 0x2000_0000 | (1 << 4)).unwrap();
        // SIZE=5 → 64B；AP=101；ENABLE
        bus.write(MPU_RASR, 4, (5 << 1) | (0b101 << 24) | 1).unwrap();
        // ENABLE + PRIVDEFENA：非命中区特权访问走后台 region 放行
        bus.write(MPU_CTRL, 4, 0x1 | 0x4).unwrap();
    }

    // 固件最后写 0x20000000 应触发 DACCVIOL 并停止执行
    let err = m.run(200_000).expect_err("写只读区域应触发 MemManage fault");
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
fn m2_xn_flash_fetch_faults() {
    let mut m = load_machine();
    {
        let bus = m.bus.lock().unwrap();
        // region0 = FLASH 0x08000000 512KB，XN（禁止执行）
        bus.write(MPU_RBAR, 4, 0x0800_0000 | (1 << 4)).unwrap();
        // SIZE=18 → 2^19=512KB；XN(bit28)；ENABLE
        bus.write(MPU_RASR, 4, (18 << 1) | (1 << 28) | 1).unwrap();
        // ENABLE + PRIVDEFENA：后台数据访问放行，使 FLASH 区取指 XN 成为唯一故障源
        bus.write(MPU_CTRL, 4, 0x1 | 0x4).unwrap();
    }

    // FLASH 区被置 XN → 取指触发 IACCVIOL，执行停止
    let err = m.run(200_000).expect_err("XN 区域取指应触发 MemManage fault");
    match err {
        CoreError::MemManageFault { addr, kind } => {
            assert_eq!(kind, MemManageKind::InstructionAccess);
            // 故障地址应在 FLASH 区（0x08000000..0x08080000）
            assert!(
                (0x0800_0000..0x0808_0000).contains(&addr),
                "故障地址应落在 FLASH 区，得到 0x{addr:08X}"
            );
        }
        other => panic!("期望 MemManageFault，得到 {other:?}"),
    }

    let bus = m.bus.lock().unwrap();
    assert_ne!(bus.read(MMFSR, 4).unwrap() & 0x1, 0, "MMFSR.IACCVIOL 应置位");
}

#[test]
fn m2_enabled_privdefena_background_allows() {
    let mut m = load_machine();
    {
        let bus = m.bus.lock().unwrap();
        // ENABLE + PRIVDEFENA，无任何 region：全部走后台 region，特权放行
        bus.write(MPU_CTRL, 4, 0x1 | 0x4).unwrap();
    }

    m.run(200_000).unwrap();

    // 固件正常跑完，结果仍为 12，且无 fault 记录
    let out = m.cpu.mem_read(0x2000_0000, 4).unwrap();
    assert_eq!(u32::from_le_bytes(out.try_into().unwrap()), 0x0000_000C);
    assert!(
        m.mpu.lock().unwrap().pending_fault().is_none(),
        "PRIVDEFENA 下不应记录故障"
    );
}

#[test]
fn m2_mpu_disabled_regression() {
    let mut m = load_machine();
    m.run(200_000).unwrap();

    // M0 回归：浮点结果正确
    let out = m.cpu.mem_read(0x2000_0000, 4).unwrap();
    assert_eq!(u32::from_le_bytes(out.try_into().unwrap()), 0x0000_000C);
    // M1 回归：SCB 仍经总线转发
    let bus_val = m.bus.lock().unwrap().read(CPACR, 4).unwrap();
    assert_eq!(bus_val, 0x00F0_0000);
}
