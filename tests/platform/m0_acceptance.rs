//! M0 验收测试：加载含浮点运算的裸机 ELF 并跑通。
//!
//! 固件源文件位于 `firmware/fp_acceptance/`，用 arm-none-eabi-gcc 编译：
//! ```text
//! arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
//!   -O0 -ffreestanding -nostdlib -nostartfiles -Wl,-T,linker.ld \
//!   -o fp_acceptance.elf main.c
//! ```
//! 期望：0x20000000 处写入 12.0f = 0x41400000。

use std::path::Path;

use unicorn_engine::RegisterARM;

use mcu_simulater::machine::Machine;

#[test]
fn m0_acceptance_fp_baremetal() {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/fp_acceptance/fp_acceptance.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();

    // 从复位后的 PC 运行固定指令数（固件末尾为死循环，靠 count 上限停住）
    let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
    m.cpu.emu_start(pc as u64, 0, 0, 200_000).unwrap();

    // 固件将 d=12.0f 经 vcvt.u32.f32 转整数后存入 0x20000000，故期望整数 12（0x0000000C）
    let out = m.cpu.mem_read(0x2000_0000, 4).unwrap();
    let val = u32::from_le_bytes(out.try_into().unwrap());
    assert_eq!(val, 0x0000_000C, "FPU 计算结果应为 12");
}
