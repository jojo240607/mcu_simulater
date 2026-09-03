//! M8 验收测试：CRC 计算单元（CRC-32/MPEG-2 风格）端到端。
//!
//! 复用 firmware/crc_demo 固件（场景见其 main.c 注释）：
//!   Phase A：CR.RESET 写 1 → 读 DR = 0xFFFFFFFF（初始值）→ G_INIT_OK=1；
//!   Phase B：按字节（8 位访问）写 DR 喂 "123456789" → 读 DR = 0x0376E6E7
//!            （CRC-32/MPEG-2 标准 check 值，独立锚点）→ G_CRC_OK=1；
//!   Phase C：CR.RESET 写 1 → 读 DR 回 0xFFFFFFFF（计算单元复位）→ G_RESET_OK=1；
//!   Phase D：IDR 写 0xAB 读回保持且不影响 DR（IDR 不参与计算）→ G_IDR_OK=1；
//!   主线写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_INIT_OK  = 1（复位后 DR 读回初始值）
//!   0x20000004 G_CRC_OK   = 1（"123456789" CRC == 0x0376E6E7）
//!   0x20000008 G_RESET_OK = 1（CR.RESET 后 DR 回 0xFFFFFFFF）
//!   0x2000000C G_IDR_OK   = 1（IDR 写读回保持且 CRC 不变）
//!   0x20000010 G_DONE     = 0xAAAAAAAA（主线完成）
//!
//! 另含一个总线直写测试：不经固件，直接经 Machine 总线验证 DR/IDR/CR 寄存器语义
//! （32 位字写推进、IDR 独立、RESET 自清零）。

use std::path::Path;

use mcu_simulater::machine::Machine;

const G_INIT_OK: u32 = 0x2000_0000;
const G_CRC_OK: u32 = 0x2000_0004;
const G_RESET_OK: u32 = 0x2000_0008;
const G_IDR_OK: u32 = 0x2000_000C;
const G_DONE: u32 = 0x2000_0010;

/// CRC 寄存器地址
const CRC_DR: u32 = 0x4002_3000;
const CRC_IDR: u32 = 0x4002_3004;
const CRC_CR: u32 = 0x4002_3008;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/crc_demo/crc_demo.elf");
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
fn m8_crc_end_to_end() {
    let mut m = load_machine();
    m.run(100_000).unwrap();

    // Phase A：复位后 DR 读回初始值 0xFFFFFFFF
    assert_eq!(read_u32(&mut m, G_INIT_OK), 1, "复位后 DR 应读回 0xFFFFFFFF");
    // Phase B："123456789" 按字节喂入 → CRC-32/MPEG-2 check 0x0376E6E7
    assert_eq!(read_u32(&mut m, G_CRC_OK), 1, "\"123456789\" CRC 应等于 0x0376E6E7");
    // Phase C：CR.RESET 后 DR 回 0xFFFFFFFF
    assert_eq!(read_u32(&mut m, G_RESET_OK), 1, "CR.RESET 后 DR 应回 0xFFFFFFFF");
    // Phase D：IDR 写读回保持且 CRC 不变
    assert_eq!(read_u32(&mut m, G_IDR_OK), 1, "IDR 写读回应保持且不影响 CRC");
    // 主线完成
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
}

/// 总线直写：不经固件，验证 CRC 寄存器级语义。
#[test]
fn m8_crc_bus_direct() {
    let m = load_machine();
    let bus = m.bus.lock().unwrap();

    // 复位外设 → CRC 计算单元回初始值（读 DR = 0xFFFFFFFF）
    bus.reset();
    assert_eq!(bus.read(CRC_DR, 4).unwrap(), 0xFFFF_FFFF, "复位后 DR 应为初始值");

    // 32 位字写 0x31323334 → DR 回读应返回当前 CRC（不复位前 ≠ 0xFFFFFFFF）
    bus.write(CRC_DR, 4, 0x3132_3334).unwrap();
    let after_word = bus.read(CRC_DR, 4).unwrap();
    assert_ne!(after_word, 0xFFFF_FFFF, "写入数据后 CRC 应已推进");

    // IDR 独立：写 0xAB 读回保持，且不影响 DR（CRC 不变）
    bus.write(CRC_IDR, 4, 0xAB).unwrap();
    assert_eq!(bus.read(CRC_IDR, 4).unwrap(), 0xAB, "IDR 应读回 0xAB");
    assert_eq!(
        bus.read(CRC_DR, 4).unwrap(),
        after_word,
        "IDR 写不应影响 DR/CRC"
    );

    // CR.RESET 写 1 → 计算单元复位（DR 回初始值），RESET 位自清零（读 CR = 0）
    bus.write(CRC_CR, 4, 1).unwrap();
    assert_eq!(bus.read(CRC_DR, 4).unwrap(), 0xFFFF_FFFF, "RESET 后 DR 应回初始值");
    assert_eq!(bus.read(CRC_CR, 4).unwrap(), 0, "RESET 位应自清零");
}
