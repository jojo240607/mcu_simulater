//! M4 验收测试：IWDG 独立看门狗 + WWDG 窗口看门狗端到端（超时 → 系统复位 → CSR 复位标志）。
//!
//! 复用 firmware/iwdg_demo 与 firmware/wwdg_demo（场景见各自 main.c 注释）：
//! - iwdg_demo：首次启动 IWDG（PR=0 ÷4、RLR=0x10）后不喂狗 → 超时 → 系统复位
//!   （置 CSR.IWDGRSTF + 复位看门狗 + 回复位向量）→ 二次进入读 CSR → G_DONE。
//! - wwdg_demo：启动 WWDG + EWI（IRQ0），计数器 0x41→0x40 触发 EWI（handler 置 G_EWI），
//!   再 0x40→0x3F 超时 → 系统复位（置 CSR.WWDGRSTF）→ 二次进入读 CSR → G_DONE。
//!
//! 期望结果区：
//!   iwdg_demo：0x20000000 G_BOOT=2（两次进入 Reset_Handler）
//!              0x20000004 G_RESET_FLAG 含 IWDGRSTF(bit28)
//!              0x20000008 G_DONE=0xAAAAAAAA
//!   wwdg_demo：0x20000000 G_BOOT=2
//!              0x20000004 G_RESET_FLAG 含 WWDGRSTF(bit27)
//!              0x20000008 G_DONE=0xAAAAAAAA
//!              0x2000000C G_EWI=1（EWI 中断执行 1 次）

use std::path::Path;

use mcu_simulater::machine::Machine;

const G_BOOT: u32 = 0x2000_0000;
const G_RESET_FLAG: u32 = 0x2000_0004;
const G_DONE: u32 = 0x2000_0008;
const G_EWI: u32 = 0x2000_000C;

const CSR_IWDGRSTF: u32 = 1 << 28;
const CSR_WWDGRSTF: u32 = 1 << 27;

fn load_machine(firmware: &str) -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("firmware/{firmware}/{firmware}.elf"));
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
fn m4_wdog_iwdg_timeout_system_reset_end_to_end() {
    let mut m = load_machine("iwdg_demo");

    m.run(200_000).unwrap();

    // 1) 两次进入 Reset_Handler（首次启动看门狗 → 超时复位 → 二次进入）
    assert_eq!(read_u32(&mut m, G_BOOT), 2, "看门狗复位后应二次进入 Reset_Handler");
    // 2) 复位后读 RCC_CSR：应含 IWDGRSTF（bit28）
    let csr = read_u32(&mut m, G_RESET_FLAG);
    assert_eq!(csr & CSR_IWDGRSTF, CSR_IWDGRSTF, "CSR.IWDGRSTF 应置位（csr=0x{csr:08X}）");
    // 3) 完成标记
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "复位后主线应写 G_DONE");
}

#[test]
fn m4_wdog_wwdg_ewi_and_timeout_system_reset_end_to_end() {
    let mut m = load_machine("wwdg_demo");

    m.run(300_000).unwrap();

    // 1) EWI 早期唤醒中断应执行 1 次（计数器跨 0x40）
    assert_eq!(read_u32(&mut m, G_EWI), 1, "WWDG EWI 中断应执行 1 次");
    // 2) 两次进入 Reset_Handler（首次启动看门狗 → 超时复位 → 二次进入）
    assert_eq!(read_u32(&mut m, G_BOOT), 2, "看门狗复位后应二次进入 Reset_Handler");
    // 3) 复位后读 RCC_CSR：应含 WWDGRSTF（bit27）
    let csr = read_u32(&mut m, G_RESET_FLAG);
    assert_eq!(csr & CSR_WWDGRSTF, CSR_WWDGRSTF, "CSR.WWDGRSTF 应置位（csr=0x{csr:08X}）");
    // 4) 完成标记
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "复位后主线应写 G_DONE");
}
