//! 探针：Unicorn Cortex-M4F 执行 VFP 指令后，CONTROL.FPCA（bit2）是否被自动置位？
//!
//! QEMU 翻译层 full_vfp_access_check：FPCCR.ASPEN=1（内部默认）且 FPCA=0 时，
//! 首次 VFP 指令会把 CONTROL.M_REG_S |= FPCA。本探针验证此行为在
//! Unicorn 2.1.5（M4F）上确实生效 —— 这是 EXC_RETURN.FTYPE 修复的前提。

use unicorn_engine::unicorn_const::{Arch, Mode, Prot};
use unicorn_engine::{ArmCpuModel, RegisterARM, Unicorn};

const BASE: u64 = 0x0806_0000;
const CPACR: u64 = 0xE000_ED88;

fn new_m4f() -> Unicorn<'static, ()> {
    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB | Mode::MCLASS).unwrap();
    uc.ctl_set_cpu_model(ArmCpuModel::CORTEX_M4 as i32).unwrap();
    uc.mem_map(0xE000_0000, 0x10000, Prot::ALL).unwrap();
    uc.mem_write(CPACR, &[0x00, 0x00, 0xF0, 0x00]).unwrap();
    uc
}

/// 执行一段代码，返回执行后 CONTROL 值
fn run_and_read_control(code: &[u8], lr_thumb: u64) -> u32 {
    let mut uc = new_m4f();
    uc.mem_map(BASE, 0x1000, Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x1000, Prot::ALL).unwrap();
    uc.mem_write(BASE, code).unwrap();
    uc.reg_write(RegisterARM::LR, lr_thumb).unwrap();
    uc.mem_write(lr_thumb & !1, &[0x00, 0xBF]).unwrap(); // nop
    let res = uc.emu_start(BASE | 1, 0, 0, 64);
    assert_eq!(res, Ok(()), "emu_start 失败: {res:?}");
    uc.reg_read(RegisterARM::CONTROL).unwrap() as u32
}

/// 1) 仅整数代码（movs/nop/bx lr）→ FPCA 应保持 0
#[test]
fn no_vfp_no_fpca() {
    let control = run_and_read_control(
        &[0x00, 0x20, // movs r0, #0
          0x00, 0xBF, // nop
          0x70, 0x47], // bx lr
        0x2000_0001,
    );
    eprintln!("[no_vfp] CONTROL=0x{control:08x}");
    assert_eq!(control & 0x4, 0, "无 VFP 指令时 FPCA 不应置位");
}

/// 2) vmov.f32 s0, #112（1.0）→ 首次 VFP 指令后 FPCA 应置位
#[test]
fn vfp_sets_fpca() {
    let control = run_and_read_control(
        &[0xb7, 0xee, 0x00, 0x0a, // vmov.f32 s0, #112 (1.0)
          0x00, 0xBF,             // nop
          0x70, 0x47],            // bx lr
        0x2000_0001,
    );
    eprintln!("[vfp] CONTROL=0x{control:08x}");
    assert_eq!(control & 0x4, 0x4, "VFP 指令后 FPCA 应置位（ASPEN 自动上下文创建）");
}

/// 3) 显式写 CONTROL 清除 FPCA 后，再执行 VFP → FPCA 重新置位
#[test]
fn fpca_cleared_then_vfp_resets() {
    let mut uc = new_m4f();
    uc.mem_map(BASE, 0x1000, Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x1000, Prot::ALL).unwrap();
    // 先执行一次 VFP 置位
    uc.mem_write(BASE, &[0xb7, 0xee, 0x00, 0x0a, 0x00, 0xBF, 0x70, 0x47]).unwrap();
    uc.reg_write(RegisterARM::LR, 0x2000_0001u64).unwrap();
    uc.mem_write(0x2000_0000, &[0x00, 0xBF]).unwrap();
    let _ = uc.emu_start(BASE | 1, 0, 0, 64);
    let c = uc.reg_read(RegisterARM::CONTROL).unwrap() as u32;
    eprintln!("[clear] 第一次 VFP 后 CONTROL=0x{c:08x}");
    assert_eq!(c & 0x4, 0x4);

    // 显式清 FPCA
    uc.reg_write(RegisterARM::CONTROL, (c & !0x4) as u64).unwrap();
    let c2 = uc.reg_read(RegisterARM::CONTROL).unwrap() as u32;
    eprintln!("[clear] 清除后 CONTROL=0x{c2:08x}");
    assert_eq!(c2 & 0x4, 0, "显式清 FPCA 应生效");

    // 再次执行 VFP → FPCA 重新置位
    let _ = uc.emu_start(BASE | 1, 0, 0, 64);
    let c3 = uc.reg_read(RegisterARM::CONTROL).unwrap() as u32;
    eprintln!("[clear] 再次 VFP 后 CONTROL=0x{c3:08x}");
    assert_eq!(c3 & 0x4, 0x4, "FPCA 清除后再次 VFP 应重新置位");
}
