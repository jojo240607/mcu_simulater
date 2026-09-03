//! M13 验收测试：FSMC 外部存储器控制器（STM32F407）端到端 + 片选窗口门控。
//!
//! 复用 firmware/fsmc_demo 固件（场景见其 main.c 注释）：
//!   Phase A（Bank1 窗口读写）：BCR1 = MBKEN|WREN|SRAM|16 位 → 写/读
//!         0x60000000 16 字模式 + 窗口末尾 0x6000FFFC + BCR1 回读校验 → G_READ_OK=1；
//!   Phase B（片选门控）：Bank2 未使能时写 0x64000000 读回应 0（写被丢弃）→ G_GATE_OK=1；
//!   Phase C（Bank2 使能后）：BCR2 = MBKEN|WREN → 写/读 0x64000000 校验 → G_BANK2_OK=1；
//!   主线写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_READ_OK  = 1（Phase A Bank1 窗口读写校验）
//!   0x20000004 G_GATE_OK  = 1（Phase B 未使能窗口访问被忽略）
//!   0x20000008 G_BANK2_OK = 1（Phase C Bank2 使能后读写校验）
//!   0x2000000C G_DONE     = 0xAAAAAAAA（主线完成）

use std::path::Path;

use mcu_simulater::machine::Machine;

const G_READ_OK: u32 = 0x2000_0000;
const G_GATE_OK: u32 = 0x2000_0004;
const G_BANK2_OK: u32 = 0x2000_0008;
const G_DONE: u32 = 0x2000_000C;

/// FSMC 寄存器（与固件一致）
const FSMC_BASE: u32 = 0xA000_0000;
const OFF_BCR1: u32 = 0x00;
const OFF_BCR2: u32 = 0x08;
const BCR_MBKEN: u32 = 1 << 0;
const BCR_WREN: u32 = 1 << 12;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/fsmc_demo/fsmc_demo.elf");
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
fn m13_fsmc_end_to_end() {
    let mut m = load_machine();

    // 运行直至主线完成（FSMC 全流程自包含，无外部注入）
    m.run(5_000_000).unwrap();
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_READ_OK), 1, "Phase A Bank1 窗口读写校验应通过");
    assert_eq!(read_u32(&mut m, G_GATE_OK), 1, "Phase B 未使能窗口访问应被忽略");
    assert_eq!(read_u32(&mut m, G_BANK2_OK), 1, "Phase C Bank2 使能后读写校验应通过");
}

#[test]
fn fsmc_registers_via_bus() {
    // 独立机器：不经固件，经总线直接访问 FSMC 寄存器文件（窗口/hook 路径由
    // 上述固件端到端测试覆盖——程序化 mem_read 不触发 MMIO hook，仅见 RAM 镜像）。
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    let bus = m.bus.clone();

    // BCR1 = MBKEN|WREN → 回读一致；非可写位（bit1）被掩码清 0
    bus.lock()
        .unwrap()
        .write(FSMC_BASE + OFF_BCR1, 4, BCR_MBKEN | BCR_WREN | 0x2)
        .unwrap();
    assert_eq!(
        bus.lock().unwrap().read(FSMC_BASE + OFF_BCR1, 4).unwrap(),
        BCR_MBKEN | BCR_WREN,
        "BCR1 应回读可写位且非法位被掩码"
    );

    // BCR2 独立：MBKEN → 回读一致，不影响 BCR1
    bus.lock()
        .unwrap()
        .write(FSMC_BASE + OFF_BCR2, 4, BCR_MBKEN)
        .unwrap();
    assert_eq!(
        bus.lock().unwrap().read(FSMC_BASE + OFF_BCR2, 4).unwrap(),
        BCR_MBKEN
    );
    assert_eq!(
        bus.lock().unwrap().read(FSMC_BASE + OFF_BCR1, 4).unwrap(),
        BCR_MBKEN | BCR_WREN,
        "BCR1 不受 BCR2 写入影响"
    );

    // BTR 时序位全位可回读
    bus.lock()
        .unwrap()
        .write(FSMC_BASE + 4, 4, 0x1234_5678)
        .unwrap();
    assert_eq!(
        bus.lock().unwrap().read(FSMC_BASE + 4, 4).unwrap(),
        0x1234_5678
    );
}
