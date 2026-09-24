//! 最小化复现：clamp_bits 返回 2⁻¹⁴ (0x38800000) 而非 1.0 的 VFP 执行缺陷定位。
//!
//! 固件 clamp_bits(dbg_pre, 0.1, 1.0) 编译为（字节取自 /tmp/flyctrl_clean.bin）：
//!   vmov r0, s22               ; 1b ee 10 0a
//!   cmp.w r0, #0x3f800000      ; b0 f1 7e 5f
//!   it hi                      ; 88 bf
//!   vmovhi.f32 s22, s24        ; b0 ee 4c ba   (s24 = 1.0)
//!   cmp r0, r2                 ; 90 42         (r2 = bits(0.1) = 0x3DCCCCCD)
//!   it cc                      ; 38 bf
//!   vmovcc.f32 s22, s4         ; b0 ee 42 ba   (s4 = 0.1)
//!   vstr s22, [r0]             ; 80 ed 00 ba
//!
//! 探针观察：DBG_PRE=1.12（正确）→ DBG_THR=2⁻¹⁴（错误）。1.12>1.0 应走
//! vmovhi 分支取 s24=1.0。0x38800000 不可能由任何 imm8 产生（已数学证明），
//! 故怀疑 IT 块 + 条件 VFP 移动执行错误，或 s24 初始化为错误值。
//!
//! 结论（2026-09-12 验证）：本文件中 4 个用例全部通过 —— 隔离执行下
//!   vmov.f32 s24,#112 → 0x3f800000（1.0）
//!   clamp_hi（1.12）  → 0x3f800000（1.0，vmovhi 生效）
//!   clamp_lo（0.05）  → 0x3dcccccd（0.1，vmovcc 生效）
//!   clamp_mid（0.5）  → 0x3f000000（0.5，原样通过）
//! 即 Unicorn 2.1.5 对 IT 块 + 条件 VFP 移动 + vmov.f32 立即数 的执行是**正确**的。
//! 因此 HIL 中 DBG_THR=2⁻¹⁴ 的缺陷不在指令翻译层，而在 RTOS 级 FPU 状态：
//!   ① 惰性 FPU 保存/恢复（LSPACT/FPCCR）在任务切换间损坏 s24/s4；
//!   ② s22 在 dbg_pre 落点与 clamp_bits 调用间被重载自过期任务帧；
//!   ③ clamp_bits 运行时 FPU 未启用（CPACR/FPCCR 不一致），VFP 寄存器陈旧。
//! 后续需在完整 HIL 中检查任务切换点的 FPCCR/LSPACT 与 VFP 寄存器现场。
//!
//! 运行: cargo test --test m_vfp_clamp_repro -- --nocapture

use unicorn_engine::unicorn_const::{Arch, Mode, Prot};
use unicorn_engine::{ArmCpuModel, RegisterARM, Unicorn};

const BASE: u64 = 0x0806_0000;
// SCB CPACR（固件 system_stm32f4xx.c 启动时置 CP10/CP11 Full Access）
const CPACR: u64 = 0xE000_ED88;

fn new_m4f() -> Unicorn<'static, ()> {
    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB | Mode::MCLASS).unwrap();
    uc.ctl_set_cpu_model(ArmCpuModel::CORTEX_M4 as i32).unwrap();
    // 映射 SCB 并启用 FPU（CP10/CP11 Full Access = 0xF00000），否则 VFP 指令触发
    // INSN_INVALID（UC_ERR_INSN_INVALID），与真实固件启动代码等价。
    uc.mem_map(0xE000_0000, 0x10000, Prot::ALL).unwrap();
    uc.mem_write(CPACR, &[0x00, 0x00, 0xF0, 0x00]).unwrap();
    uc
}

/// 执行 code 并返回。关键：Unicorn 的 uc_emu_start(uc.c:1137) 对 ARM 总会用 begin
/// 覆写 PC，故必须传 BASE|1（带 Thumb 位）；传 0 会从地址 0 取指触发 EXCEPTION。
/// LR 必须带 Thumb 位（|1），否则 bx lr 切到 ARM 态触发 INSN_INVALID。
fn run_code(uc: &mut Unicorn<'static, ()>, code: &[u8], lr_thumb: u64) {
    uc.mem_write(BASE, code).unwrap();
    uc.reg_write(RegisterARM::LR, lr_thumb).unwrap();
    // 返回点放 nop（bx lr 后靠 count 上限停下）
    uc.mem_write(lr_thumb & !1, &[0x00, 0xBF]).unwrap();
    let res = uc.emu_start(BASE | 1, 0, 0, 64);
    assert_eq!(res, Ok(()), "emu_start 失败: {res:?}");
}

fn as_f32(v: u64) -> f32 {
    f32::from_bits(v as u32)
}

fn bits(f: f32) -> u64 {
    f.to_bits() as u64
}

/// 步骤 1：vmov.f32 s24, #112 是否产出 1.0？
#[test]
fn vmov_imm_s24() {
    let mut uc = new_m4f();
    uc.mem_map(BASE, 0x1000, unicorn_engine::Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x1000, unicorn_engine::Prot::ALL).unwrap();
    // vmov.f32 s24, #112 (1.0)  ; bx lr
    // 固件实际字节: b7 ee 00 ca (eeb7 ca00)
    run_code(&mut uc, &[0xb7, 0xee, 0x00, 0xca, 0x70, 0x47], 0x2000_0001);

    let s24 = uc.reg_read(RegisterARM::S24).unwrap();
    eprintln!("[vmov_imm] s24=0x{s24:08x} ({})", as_f32(s24));
    assert_eq!(s24, 0x3f80_0000, "vmov.f32 s24,#112 应得 1.0");
}

/// 步骤 2：完整 clamp 模式（1.12 应 clamp 到 1.0）。
/// 用 reg_write 预置 s22/s24/s4，避免手编 vldr。
#[test]
fn clamp_hi_path() {
    let mut uc = new_m4f();
    uc.mem_map(BASE, 0x1000, unicorn_engine::Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x1000, unicorn_engine::Prot::ALL).unwrap();

    // 预置 VFP 与 GPR
    uc.reg_write(RegisterARM::S22, bits(1.12f32)).unwrap(); // 待 clamp 值
    uc.reg_write(RegisterARM::S24, bits(1.0f32)).unwrap();  // hi = 1.0
    uc.reg_write(RegisterARM::S4, bits(0.1f32)).unwrap();   // lo = 0.1
    uc.reg_write(RegisterARM::R2, bits(0.1f32)).unwrap();   // bits(0.1)

    // 精确固件字节：vmov r0,s22 / cmp.w / it hi / vmovhi / cmp / it cc / vmovcc / bx lr
    // （真实固件 vstr 前会从 literal 重新装载 r0 输出指针，此处省略，直接检查 s22）
    let code: &[u8] = &[
        0x1b, 0xee, 0x10, 0x0a, // vmov r0, s22
        0xb0, 0xf1, 0x7e, 0x5f, // cmp.w r0, #0x3f800000
        0x88, 0xbf,             // it hi
        0xb0, 0xee, 0x4c, 0xba, // vmovhi.f32 s22, s24
        0x90, 0x42,             // cmp r0, r2
        0x38, 0xbf,             // it cc
        0xb0, 0xee, 0x42, 0xba, // vmovcc.f32 s22, s4
        0x70, 0x47,             // bx lr
    ];
    run_code(&mut uc, code, 0x2000_0001);

    let s22 = uc.reg_read(RegisterARM::S22).unwrap();
    eprintln!(
        "[clamp_hi] s22=0x{s22:08x} ({})",
        as_f32(s22)
    );
    assert_eq!(s22, 0x3f80_0000, "1.12 clamp 到 1.0 失败");
}

/// 步骤 3：lo 路径（0.05 应 clamp 到 0.1）。
#[test]
fn clamp_lo_path() {
    let mut uc = new_m4f();
    uc.mem_map(BASE, 0x1000, unicorn_engine::Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x1000, unicorn_engine::Prot::ALL).unwrap();

    uc.reg_write(RegisterARM::S22, bits(0.05f32)).unwrap();
    uc.reg_write(RegisterARM::S24, bits(1.0f32)).unwrap();
    uc.reg_write(RegisterARM::S4, bits(0.1f32)).unwrap();
    uc.reg_write(RegisterARM::R2, bits(0.1f32)).unwrap();

    let code: &[u8] = &[
        0x1b, 0xee, 0x10, 0x0a, // vmov r0, s22
        0xb0, 0xf1, 0x7e, 0x5f, // cmp.w r0, #0x3f800000
        0x88, 0xbf,             // it hi
        0xb0, 0xee, 0x4c, 0xba, // vmovhi.f32 s22, s24
        0x90, 0x42,             // cmp r0, r2
        0x38, 0xbf,             // it cc
        0xb0, 0xee, 0x42, 0xba, // vmovcc.f32 s22, s4
        0x70, 0x47,             // bx lr
    ];
    run_code(&mut uc, code, 0x2000_0001);

    let s22 = uc.reg_read(RegisterARM::S22).unwrap();
    eprintln!("[clamp_lo] s22=0x{s22:08x} ({})", as_f32(s22));
    assert_eq!(s22, 0x3dcc_cccd, "0.05 clamp 到 0.1 失败");
}

/// 步骤 4：中间路径（0.5 应原样通过）。
#[test]
fn clamp_mid_path() {
    let mut uc = new_m4f();
    uc.mem_map(BASE, 0x1000, unicorn_engine::Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x1000, unicorn_engine::Prot::ALL).unwrap();

    uc.reg_write(RegisterARM::S22, bits(0.5f32)).unwrap();
    uc.reg_write(RegisterARM::S24, bits(1.0f32)).unwrap();
    uc.reg_write(RegisterARM::S4, bits(0.1f32)).unwrap();
    uc.reg_write(RegisterARM::R2, bits(0.1f32)).unwrap();

    let code: &[u8] = &[
        0x1b, 0xee, 0x10, 0x0a, // vmov r0, s22
        0xb0, 0xf1, 0x7e, 0x5f, // cmp.w r0, #0x3f800000
        0x88, 0xbf,             // it hi
        0xb0, 0xee, 0x4c, 0xba, // vmovhi.f32 s22, s24
        0x90, 0x42,             // cmp r0, r2
        0x38, 0xbf,             // it cc
        0xb0, 0xee, 0x42, 0xba, // vmovcc.f32 s22, s4
        0x70, 0x47,             // bx lr
    ];
    run_code(&mut uc, code, 0x2000_0001);

    let s22 = uc.reg_read(RegisterARM::S22).unwrap();
    eprintln!("[clamp_mid] s22=0x{s22:08x} ({})", as_f32(s22));
    assert_eq!(s22, 0x3f00_0000, "0.5 应原样通过");
}
