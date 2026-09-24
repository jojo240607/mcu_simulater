//! M1 验收测试：MMIO 转发链路（CPU 访问 → mem hook → 内存总线 → Rust 外设）。
//!
//! 复用 M0 的 fp_acceptance 固件：其启动时对 CPACR（0xE000ED88，SCB 区间内）
//! 做读-改-写。SCB 占位外设为全零镜像 RAM，故读返回 0，写值为 0x00F00000。
//! 若 M1 链路工作正常，运行固件后总线上的 SCB 外设与 Unicorn 的 RAM 视图
//! 应都记录到该值；否则外设区间内的访问不会落到总线，断言失败。

use std::path::Path;

use unicorn_engine::RegisterARM;

use mcu_simulater::machine::Machine;

const CPACR: u32 = 0xE000_ED88;
const EXPECTED: u32 = 0x00F0_0000; // 读回 0 → 写入 (0 & ~0xF00000) | 0xF00000

#[test]
fn m1_mmio_scb_forwarding() {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/fp_acceptance/fp_acceptance.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap(); // 内部完成 SCB 挂载 + hook 注册
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();

    let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
    m.cpu.emu_start(pc as u64, 0, 0, 200_000).unwrap();

    // 总线侧：SCB 外设应记录到 CPACR 写值（证明 CPU 访问经 hook 到达总线）
    let bus_val = m.bus.lock().unwrap().read(CPACR, 4).unwrap();
    assert_eq!(bus_val, EXPECTED, "总线上的 SCB 外设应记录 CPACR 写值");

    // RAM 视图侧：hook 放行后 Unicorn 照常写 RAM，两处应一致
    let ram = m.cpu.mem_read(CPACR as u64, 4).unwrap();
    assert_eq!(u32::from_le_bytes(ram.try_into().unwrap()), EXPECTED);

    // 回归：FPU 计算结果仍正确（M0 验收不受 SCB 挂载影响）
    let out = m.cpu.mem_read(0x2000_0000, 4).unwrap();
    assert_eq!(u32::from_le_bytes(out.try_into().unwrap()), 0x0000_000C);
}
