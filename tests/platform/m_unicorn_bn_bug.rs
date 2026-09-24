//! 最小化复现：Unicorn 2.1.5 ARM Thumb b.n 跳转目标计算错误
//!
//! 指令: b.n 0x0801113A (编码 0xE7AF) 位于 0x080111D8
//! 预期: 跳转到 0x0801113A
//! 实际: 跳转到 0x0801113E (off by +4)
//!
//! 策略 v1: 每次重建 Unicorn 实例 (已验证 100% 正确，bug 需要上下文)
//! 策略 v2: 同一实例，先预执行 0x1113E 创建 TB 缓存，再执行 b.n
//! 策略 v3: 写入真实 32-bit Thumb 指令模拟原始场景
//!
//! 运行: cargo test test_bn -- --nocapture

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use unicorn_engine::unicorn_const::{Arch, Mode};
use unicorn_engine::{Prot, RegisterARM, Unicorn};

const ADDR_BASE: u64 = 0x0801_0000;
const ADDR_BRANCH: u64 = 0x0801_11D8; // b.n 0x0801113A
const ADDR_RETURN: u64 = 0x0801_113A; // 预期跳转目标 (ldmia.w)
const ADDR_MAIN: u64 = 0x0801_113E; // 错误跳转目标 (ldr.w)
const N_RUNS: u32 = 200;

// 真实固件中的 32-bit Thumb 指令
// 0x0801113A: ldmia.w sp!, {r4, r5, r6, r7, pc}  → 0xE8BD 0x00F0
// 0x0801113E: ldr.w   r3, [r3, #0x20]            → 0xF8D3 0x3020
const LDMIAW_SP: [u8; 4] = [0xBD, 0xE8, 0xF0, 0x00]; // little-endian
const LDRW_R3: [u8; 4] = [0xD3, 0xF8, 0x20, 0x30]; // little-endian

fn setup_memory(uc: &mut Unicorn<()>) {
    uc.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();
    // 映射栈空间
    uc.mem_map(0x2000_0000, 0x1000, Prot::ALL).unwrap();
}

fn write_branch(uc: &mut Unicorn<()>) {
    uc.mem_write(ADDR_BRANCH, &[0xAF, 0xE7]).unwrap();
}

fn write_real_instructions(uc: &mut Unicorn<()>) {
    uc.mem_write(ADDR_RETURN, &LDMIAW_SP).unwrap();
    uc.mem_write(ADDR_MAIN, &LDRW_R3).unwrap();
}

/// 设置基本寄存器状态 (SP, R3 等)
fn setup_regs(uc: &mut Unicorn<()>) {
    // 设置栈指针 (ldmia.w 需要合法栈)
    uc.reg_write(RegisterARM::SP, 0x2000_0F00).unwrap();
    // 在栈上放置一些值，使 ldmia.w 弹栈后 PC 回到安全位置
    let stack_data: [u8; 20] = [
        0x00, 0x00, 0x00, 0x00, // r4
        0x00, 0x00, 0x00, 0x00, // r5
        0x00, 0x00, 0x00, 0x00, // r6
        0x00, 0x00, 0x00, 0x00, // r7
        0x00, 0x00, 0x00, 0x00, // pc (will be patched per test)
    ];
    uc.mem_write(0x2000_0F00, &stack_data).unwrap();
    // R3 需要指向可读内存 (ldr.w r3, [r3, #0x20] 会读取)
    uc.reg_write(RegisterARM::R3, 0x2000_0000).unwrap();
}

/// 策略 v2: 同一实例，先预执行 0x1113E 创建 TB 缓存
#[test]
fn test_bn_v2_pre_execute_tb() {
    let mut correct = 0u32;
    let mut wrong = 0u32;
    let mut other = 0u32;

    for i in 0..N_RUNS {
        let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();
        setup_memory(&mut uc);
        setup_regs(&mut uc);
        write_branch(&mut uc);
        write_real_instructions(&mut uc);

        // 步骤 1: 预执行 0x1113E 处的代码，创建 TB 缓存
        // 在 0x1113E 写入一个会停止的指令序列
        uc.mem_write(ADDR_MAIN, &[0x00, 0xBF]).unwrap(); // nop
        uc.mem_write(ADDR_MAIN + 2, &[0x00, 0xBE]).unwrap(); // bkpt

        let pre_hook = {
            let pre_hit = Arc::new(AtomicU64::new(0));
            let pre_hit_clone = pre_hit.clone();
            let hook = uc
                .add_code_hook(ADDR_MAIN, ADDR_MAIN + 4, move |uc, addr, _size| {
                    pre_hit_clone.store(addr, Ordering::Relaxed);
                    uc.emu_stop().unwrap();
                })
                .unwrap();
            (hook, pre_hit)
        };

        uc.reg_write(RegisterARM::PC, ADDR_MAIN | 1).unwrap();
        let r = uc.emu_start(ADDR_MAIN | 1, 0, 0, 0);
        // 可能因 bkpt 返回错误，忽略
        let _ = r;
        drop(pre_hook.0);

        assert!(pre_hook.1.load(Ordering::Relaxed) == ADDR_MAIN,
                "预执行未命中 0x{:08X}", ADDR_MAIN);

        // 恢复真实指令
        write_real_instructions(&mut uc);

        // 步骤 2: 执行 b.n 0x1113A
        let last_addr = Arc::new(AtomicU64::new(0));
        let target = Arc::new(AtomicU64::new(0));

        let _hook = uc
            .add_code_hook(0, u64::MAX, {
                let last_addr = last_addr.clone();
                let target = target.clone();
                move |uc, addr, _size| {
                    if last_addr.load(Ordering::Relaxed) == ADDR_BRANCH {
                        target.store(addr, Ordering::Relaxed);
                        uc.emu_stop().unwrap();
                    }
                    last_addr.store(addr, Ordering::Relaxed);
                }
            })
            .unwrap();

        // 在栈上写入安全返回地址
        let safe_pc: [u8; 4] = ((ADDR_MAIN | 1) as u32).to_le_bytes();
        uc.mem_write(0x2000_0F00 + 16, &safe_pc).unwrap();

        uc.reg_write(RegisterARM::PC, ADDR_BRANCH | 1).unwrap();
        uc.emu_start(ADDR_BRANCH | 1, 0, 0, 0).ok();

        let t = target.load(Ordering::Relaxed);
        if t == ADDR_RETURN {
            correct += 1;
        } else if t == ADDR_MAIN {
            wrong += 1;
        } else {
            other += 1;
            println!("  [#{}] unexpected: 0x{:08X}", i, t);
        }
    }

    print_results("v2 (预执行 0x1113E TB)", correct, wrong, other);
}

/// 策略 v3: 同一实例，多次 b.n 不重建 (累积 TB 状态)
#[test]
fn test_bn_v3_same_instance() {
    let mut correct = 0u32;
    let mut wrong = 0u32;
    let mut other = 0u32;

    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();
    setup_memory(&mut uc);
    setup_regs(&mut uc);
    write_branch(&mut uc);
    write_real_instructions(&mut uc);

    let last_addr = Arc::new(AtomicU64::new(0));
    let target = Arc::new(AtomicU64::new(0));

    let _hook = uc
        .add_code_hook(0, u64::MAX, {
            let last_addr = last_addr.clone();
            let target = target.clone();
            move |uc, addr, _size| {
                if last_addr.load(Ordering::Relaxed) == ADDR_BRANCH {
                    target.store(addr, Ordering::Relaxed);
                    uc.emu_stop().unwrap();
                }
                last_addr.store(addr, Ordering::Relaxed);
            }
        })
        .unwrap();

    for i in 0..N_RUNS {
        // 每次重置 last_addr 和堆栈
        last_addr.store(0, Ordering::Relaxed);
        target.store(0, Ordering::Relaxed);

        let safe_pc: [u8; 4] = ((ADDR_MAIN | 1) as u32).to_le_bytes();
        uc.mem_write(0x2000_0F00 + 16, &safe_pc).unwrap();

        uc.reg_write(RegisterARM::PC, ADDR_BRANCH | 1).unwrap();
        uc.emu_start(ADDR_BRANCH | 1, 0, 0, 0).ok();

        let t = target.load(Ordering::Relaxed);
        if t == ADDR_RETURN {
            correct += 1;
        } else if t == ADDR_MAIN {
            wrong += 1;
        } else {
            other += 1;
            println!("  [#{}] unexpected: 0x{:08X}", i, t);
        }
    }

    print_results("v3 (同一实例 100 次)", correct, wrong, other);
}

/// 策略 v4: 模拟完整固件执行流程
///
/// 从 _malloc_r 函数入口 (0x08011118) 开始执行，让 Unicorn 自然创建 TB 链：
/// 0x11118 (push) → 0x11132 (bls.n→0x1113E) → 分配路径 → 0x111D8 (b.n→0x1113A)
///
/// 关键区别: v2 中 TB 被 code hook 中断后未完整执行与链式连接；
/// v4 让 TB 在完整执行路径中自然创建和链接。
#[test]
fn test_bn_v4_full_function() {
    let mut correct = 0u32;
    let mut wrong = 0u32;
    let mut other = 0u32;

    // _malloc_r 函数二进制 blob (从固件 ELF 提取)
    // 0x08011118 ~ 0x0801122D (含 __malloc_lock / __malloc_unlock)
    const MALLOC_R_BLOB: &[u8] = &[
        0x2D, 0xE9, 0xF8, 0x43, // 0x11118: stmdb sp!,{r3,r4,r5,r6,r7,r8,r9,lr}
        0xCD, 0x1C, // 0x1111C: adds r5,r1,#3
        0x25, 0xF0, 0x03, 0x05, // 0x1111E: bic.w r5,r5,#3
        0x08, 0x35, // 0x11122: adds r5,#8
        0x0C, 0x2D, // 0x11124: cmp r5,#12
        0x38, 0xBF, // 0x11126: it cc
        0x0C, 0x25, // 0x11128: movcc r5,#12
        0x00, 0x2D, // 0x1112A: cmp r5,#0
        0x06, 0x46, // 0x1112C: mov r6,r0
        0x01, 0xDB, // 0x1112E: blt.n 0x11134
        0xA9, 0x42, // 0x11130: cmp r1,r5
        0x04, 0xD9, // 0x11132: bls.n 0x1113E  ← 跳转到分配路径
        0x0C, 0x23, // 0x11134: movs r3,#12
        0x33, 0x60, // 0x11136: str r3,[r6,#0]
        0x00, 0x20, // 0x11138: movs r0,#0
        0xBD, 0xE8, 0xF8, 0x83, // 0x1113A: ldmia.w sp!,{r3,r4,r5,r6,r7,r8,r9,pc} ← 返回
        0xDF, 0xF8, 0xD4, 0x80, // 0x1113E: ldr.w r8,[pc,#212] → 0x11216
        0x00, 0xF0, 0x69, 0xF8, // 0x11142: bl __malloc_lock
        0xD8, 0xF8, 0x00, 0x30, // 0x11146: ldr.w r3,[r8]
        0x1C, 0x46, // 0x1114A: mov r4,r3
        0x44, 0xBB, // 0x1114C: cbnz r4,0x111A0
        0x29, 0x46, // 0x1114E: mov r1,r5
        0x30, 0x46, // 0x11150: mov r0,r6
        0xFF, 0xF7, 0xBF, 0xFF, // 0x11152: bl sbrk_aligned
        0x43, 0x1C, // 0x11156: adds r3,r0,#1
        0x04, 0x46, // 0x11158: mov r4,r0
        0x58, 0xD1, // 0x1115A: bne.n 0x1120E
        0xD8, 0xF8, 0x00, 0x40, // 0x1115C: ldr.w r4,[r8]
        0x27, 0x46, // 0x11160: mov r7,r4
        0x00, 0x2F, // 0x11162: cmp r7,#0
        0x43, 0xD1, // 0x11164: bne.n 0x111EE
        0x00, 0x2C, // 0x11166: cmp r4,#0
        0x4B, 0xD0, // 0x11168: beq.n 0x11202
        0x23, 0x68, // 0x1116A: ldr r3,[r4,#0]
        0x39, 0x46, // 0x1116C: mov r1,r7
        0x30, 0x46, // 0x1116E: mov r0,r6
        0x04, 0xEB, 0x03, 0x09, // 0x11170: add.w r9,r4,r3
        0x00, 0xF0, 0xD8, 0xFB, // 0x11174: bl _sbrk_r
        0x81, 0x45, // 0x11178: cmp r9,r0
        0x42, 0xD1, // 0x1117A: bne.n 0x11202
        0x21, 0x68, // 0x1117C: ldr r1,[r4,#0]
        0x6D, 0x1A, // 0x1117E: subs r5,r5,r1
        0x29, 0x46, // 0x11180: mov r1,r5
        0x30, 0x46, // 0x11182: mov r0,r6
        0xFF, 0xF7, 0xA6, 0xFF, // 0x11184: bl sbrk_aligned
        0x01, 0x30, // 0x11188: adds r0,#1
        0x3A, 0xD0, // 0x1118A: beq.n 0x11202
        0x23, 0x68, // 0x1118C: ldr r3,[r4,#0]
        0x2B, 0x44, // 0x1118E: add r3,r5
        0x23, 0x60, // 0x11190: str r3,[r4,#0]
        0xD8, 0xF8, 0x00, 0x30, // 0x11192: ldr.w r3,[r8]
        0x5A, 0x68, // 0x11196: ldr r2,[r3,#4]
        0x62, 0xBB, // 0x11198: cbnz r2,0x111F4
        0xC8, 0xF8, 0x00, 0x70, // 0x1119A: str.w r7,[r8]
        0x0F, 0xE0, // 0x1119E: b.n 0x111C0
        // 0x111A0: 分配路径 (free list 非空时进入)
        0x22, 0x68, // 0x111A0: ldr r2,[r4,#0]
        0x52, 0x1B, // 0x111A2: subs r2,r2,r5
        0x20, 0xD4, // 0x111A4: bmi.n 0x111E8
        0x0B, 0x2A, // 0x111A6: cmp r2,#11
        0x17, 0xD9, // 0x111A8: bls.n 0x111DA
        0x61, 0x19, // 0x111AA: adds r1,r4,r5
        0xA3, 0x42, // 0x111AC: cmp r3,r4
        0x25, 0x60, // 0x111AE: str r5,[r4,#0]
        0x18, 0xBF, // 0x111B0: it ne
        0x59, 0x60, // 0x111B2: strne r1,[r3,#4]
        0x63, 0x68, // 0x111B4: ldr r3,[r4,#4]
        0x08, 0xBF, // 0x111B6: it eq
        0xC8, 0xF8, 0x00, 0x10, // 0x111B8: streq.w r1,[r8]
        0x62, 0x51, // 0x111BC: str r2,[r4,r5]
        0x4B, 0x60, // 0x111BE: str r3,[r1,#4]
        // 0x111C0: unlock 并返回
        0x30, 0x46, // 0x111C0: mov r0,r6
        0x00, 0xF0, 0x2F, 0xF8, // 0x111C2: bl __malloc_unlock
        0x04, 0xF1, 0x0B, 0x00, // 0x111C6: add.w r0,r4,#11
        0x23, 0x1D, // 0x111CA: adds r3,r4,#4
        0x20, 0xF0, 0x07, 0x00, // 0x111CC: bic.w r0,r0,#7
        0xC2, 0x1A, // 0x111D0: subs r2,r0,r3
        0x1C, 0xBF, // 0x111D2: itt ne
        0x1B, 0x1A, // 0x111D4: subne r3,r3,r0
        0xA3, 0x50, // 0x111D6: strne r3,[r4,r2]
        0xAF, 0xE7, // 0x111D8: b.n 0x1113A  ← 关键跳转!
        // 0x111DA: 小块分配路径
        0x62, 0x68, // 0x111DA: ldr r2,[r4,#4]
        0xA3, 0x42, // 0x111DC: cmp r3,r4
        0x0C, 0xBF, // 0x111DE: ite eq
        0xC8, 0xF8, 0x00, 0x20, // 0x111E0: streq.w r2,[r8]
        0x5A, 0x60, // 0x111E4: strne r2,[r3,#4]
        0xEB, 0xE7, // 0x111E6: b.n 0x111C0
        // 0x111E8: 遍历 free list
        0x23, 0x46, // 0x111E8: mov r3,r4
        0x64, 0x68, // 0x111EA: ldr r4,[r4,#4]
        0xAE, 0xE7, // 0x111EC: b.n 0x1114C
        0x3C, 0x46, // 0x111EE: mov r4,r7
        0x7F, 0x68, // 0x111F0: ldr r7,[r7,#4]
        0xB6, 0xE7, // 0x111F2: b.n 0x11162
        0x1A, 0x46, // 0x111F4: mov r2,r3
        0x5B, 0x68, // 0x111F6: ldr r3,[r3,#4]
        0xA3, 0x42, // 0x111F8: cmp r3,r4
        0xFB, 0xD1, // 0x111FA: bne.n 0x111F4
        0x00, 0x23, // 0x111FC: movs r3,#0
        0x53, 0x60, // 0x111FE: str r3,[r2,#4]
        0xDE, 0xE7, // 0x11200: b.n 0x111C0
        0x0C, 0x23, // 0x11202: movs r3,#12
        0x33, 0x60, // 0x11204: str r3,[r6,#0]
        0x30, 0x46, // 0x11206: mov r0,r6
        0x00, 0xF0, 0x0C, 0xF8, // 0x11208: bl __malloc_unlock
        0x94, 0xE7, // 0x1120C: b.n 0x11138
        0x05, 0x60, // 0x1120E: str r5,[r0,#0]
        0xD6, 0xE7, // 0x11210: b.n 0x111C0
        0x00, 0xBF, // 0x11212: nop
        0x00, 0x01, 0x00, 0x20, // 0x11214: .word 0x20000100 (global ptr)
        0x00, 0xBF, // 0x11218: nop (bl target)
        0x09, 0xE0, // 0x1121A: b.n 0x11230 (__malloc_lock → bx lr stub)
        0x00, 0xBF, 0x00, 0xBF, // 0x1121C-0x1121F: nop padding
        0x00, 0xBF, 0x00, 0xBF, // 0x11220-0x11223: nop padding
        0x00, 0xBF, // 0x11224: nop (__malloc_unlock entry)
        0x03, 0xE0, // 0x11226: b.n 0x11230 (__malloc_unlock → bx lr stub)
        0x00, 0xBF, 0x00, 0xBF, // 0x11228-0x1122B: nop padding
        0x00, 0xBF, 0x00, 0xBF, // 0x1122C-0x1122F: nop padding
        0x70, 0x47, // 0x11230: bx lr
        0x00, 0xBF, // 0x11232: nop padding
    ];

    // 使用单个 Unicorn 实例，保持 TB 缓存跨运行
    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();
    uc.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x10000, Prot::ALL).unwrap();
    uc.mem_write(ADDR_BASE + 0x1118, MALLOC_R_BLOB).unwrap();

    let sp = 0x2000_0F00u32;
    let safe_ret: [u8; 4] = ((ADDR_MAIN | 1) as u32).to_le_bytes();

    let last_addr = Arc::new(AtomicU64::new(0));
    let target = Arc::new(AtomicU64::new(0));
    let reached_bn = Arc::new(AtomicU64::new(0));

    let _hook = uc
        .add_code_hook(0, u64::MAX, {
            let last_addr = last_addr.clone();
            let target = target.clone();
            let reached_bn = reached_bn.clone();
            move |uc, addr, _size| {
                if last_addr.load(Ordering::Relaxed) == ADDR_BRANCH {
                    target.store(addr, Ordering::Relaxed);
                    reached_bn.store(1, Ordering::Relaxed);
                    uc.emu_stop().unwrap();
                }
                last_addr.store(addr, Ordering::Relaxed);
            }
        })
        .unwrap();

    for i in 0..N_RUNS {
        // 重置 free list
        let head: [u8; 4] = 0x20000200u32.to_le_bytes();
        uc.mem_write(0x20000100, &head).unwrap();
        let chunk_size: [u8; 4] = 12u32.to_le_bytes();
        uc.mem_write(0x20000200, &chunk_size).unwrap();
        let chunk_next: [u8; 4] = 0u32.to_le_bytes();
        uc.mem_write(0x20000204, &chunk_next).unwrap();

        // 重置寄存器
        uc.reg_write(RegisterARM::SP, sp as u64).unwrap();
        uc.reg_write(RegisterARM::R0, 0x20000300).unwrap();
        uc.reg_write(RegisterARM::R1, 0).unwrap();
        uc.reg_write(RegisterARM::R2, 0).unwrap();
        uc.reg_write(RegisterARM::R3, 0).unwrap();
        uc.reg_write(RegisterARM::R4, 0).unwrap();
        uc.reg_write(RegisterARM::R5, 0).unwrap();
        uc.reg_write(RegisterARM::R6, 0).unwrap();
        uc.reg_write(RegisterARM::R7, 0).unwrap();
        uc.reg_write(RegisterARM::R8, 0).unwrap();
        uc.reg_write(RegisterARM::R9, 0).unwrap();
        uc.reg_write(RegisterARM::R10, 0).unwrap();
        uc.reg_write(RegisterARM::R11, 0).unwrap();
        uc.reg_write(RegisterARM::R12, 0).unwrap();
        uc.reg_write(RegisterARM::LR, 0).unwrap();

        uc.mem_write((sp - 4) as u64, &safe_ret).unwrap();

        last_addr.store(0, Ordering::Relaxed);
        target.store(0, Ordering::Relaxed);
        reached_bn.store(0, Ordering::Relaxed);

        uc.reg_write(RegisterARM::PC, (ADDR_BASE + 0x1118) | 1).unwrap();
        uc.emu_start((ADDR_BASE + 0x1118) | 1, 0, 0, 10000).ok();

        if reached_bn.load(Ordering::Relaxed) == 1 {
            let t = target.load(Ordering::Relaxed);
            if t == ADDR_RETURN {
                correct += 1;
            } else if t == ADDR_MAIN {
                wrong += 1;
                println!("  [#{}] BUG! 跳转到 0x{:08X}", i, t);
            } else {
                other += 1;
                println!("  [#{}] unexpected: 0x{:08X}", i, t);
            }
        } else {
            other += 1;
            println!("  [#{}] 未到达 b.n, 最后地址: 0x{:08X}", i, last_addr.load(Ordering::Relaxed));
        }
    }

    print_results("v4 (完整 _malloc_r 执行)", correct, wrong, other);
}

/// 策略 v5: TB 失效 + 重建
///
/// 原始模拟器中，外设 MMIO 写会触发 TB 失效，导致 TB 频繁重建。
/// v5 在每次运行后写入代码内存来强制 TB 失效，模拟这种扰动。
///
/// 关键假设: b.n 跳转目标错误(0x1113A→0x1113E)发生在 TB 缓存被扰动后，
/// 跳转缓存(tb_jmp_cache)或哈希表返回了错误的 TB 条目。
/// 0x1113A 和 0x1113E 仅差 4 字节，在 TB 频繁失效重建的场景下容易出错。
#[test]
fn test_bn_v5_tb_invalidate() {
    let mut correct = 0u32;
    let mut wrong = 0u32;
    let mut other = 0u32;

    // _malloc_r 函数二进制 blob (从固件 ELF 提取)
    const MALLOC_R_BLOB: &[u8] = &[
        0x2D, 0xE9, 0xF8, 0x43, 0xCD, 0x1C, 0x25, 0xF0, 0x03, 0x05, 0x08, 0x35,
        0x0C, 0x2D, 0x38, 0xBF, 0x0C, 0x25, 0x00, 0x2D, 0x06, 0x46, 0x01, 0xDB,
        0xA9, 0x42, 0x04, 0xD9, 0x0C, 0x23, 0x33, 0x60, 0x00, 0x20, 0xBD, 0xE8,
        0xF8, 0x83, 0xDF, 0xF8, 0xD4, 0x80, 0x00, 0xF0, 0x69, 0xF8, 0xD8, 0xF8,
        0x00, 0x30, 0x1C, 0x46, 0x44, 0xBB, 0x29, 0x46, 0x30, 0x46, 0xFF, 0xF7,
        0xBF, 0xFF, 0x43, 0x1C, 0x04, 0x46, 0x58, 0xD1, 0xD8, 0xF8, 0x00, 0x40,
        0x27, 0x46, 0x00, 0x2F, 0x43, 0xD1, 0x00, 0x2C, 0x4B, 0xD0, 0x23, 0x68,
        0x39, 0x46, 0x30, 0x46, 0x04, 0xEB, 0x03, 0x09, 0x00, 0xF0, 0xD8, 0xFB,
        0x81, 0x45, 0x42, 0xD1, 0x21, 0x68, 0x6D, 0x1A, 0x29, 0x46, 0x30, 0x46,
        0xFF, 0xF7, 0xA6, 0xFF, 0x01, 0x30, 0x3A, 0xD0, 0x23, 0x68, 0x2B, 0x44,
        0x23, 0x60, 0xD8, 0xF8, 0x00, 0x30, 0x5A, 0x68, 0x62, 0xBB, 0xC8, 0xF8,
        0x00, 0x70, 0x0F, 0xE0, 0x22, 0x68, 0x52, 0x1B, 0x20, 0xD4, 0x0B, 0x2A,
        0x17, 0xD9, 0x61, 0x19, 0xA3, 0x42, 0x25, 0x60, 0x18, 0xBF, 0x59, 0x60,
        0x63, 0x68, 0x08, 0xBF, 0xC8, 0xF8, 0x00, 0x10, 0x62, 0x51, 0x4B, 0x60,
        0x30, 0x46, 0x00, 0xF0, 0x2F, 0xF8, 0x04, 0xF1, 0x0B, 0x00, 0x23, 0x1D,
        0x20, 0xF0, 0x07, 0x00, 0xC2, 0x1A, 0x1C, 0xBF, 0x1B, 0x1A, 0xA3, 0x50,
        0xAF, 0xE7, 0x62, 0x68, 0xA3, 0x42, 0x0C, 0xBF, 0xC8, 0xF8, 0x00, 0x20,
        0x5A, 0x60, 0xEB, 0xE7, 0x23, 0x46, 0x64, 0x68, 0xAE, 0xE7, 0x3C, 0x46,
        0x7F, 0x68, 0xB6, 0xE7, 0x1A, 0x46, 0x5B, 0x68, 0xA3, 0x42, 0xFB, 0xD1,
        0x00, 0x23, 0x53, 0x60, 0xDE, 0xE7, 0x0C, 0x23, 0x33, 0x60, 0x30, 0x46,
        0x00, 0xF0, 0x0C, 0xF8, 0x94, 0xE7, 0x05, 0x60, 0xD6, 0xE7, 0x00, 0xBF,
        0x00, 0x01, 0x00, 0x20, 0x00, 0xBF, 0x09, 0xE0, 0x00, 0xBF, 0x00, 0xBF,
        0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x03, 0xE0, 0x00, 0xBF, 0x00, 0xBF,
        0x00, 0xBF, 0x00, 0xBF, 0x70, 0x47, 0x00, 0xBF,
    ];

    const N_RUNS_V5: u32 = 5000;

    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();
    uc.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x10000, Prot::ALL).unwrap();
    uc.mem_write(ADDR_BASE + 0x1118, MALLOC_R_BLOB).unwrap();

    let sp = 0x2000_0F00u32;
    let safe_ret: [u8; 4] = ((ADDR_MAIN | 1) as u32).to_le_bytes();

    let last_addr = Arc::new(AtomicU64::new(0));
    let target = Arc::new(AtomicU64::new(0));
    let reached_bn = Arc::new(AtomicU64::new(0));

    let _hook = uc
        .add_code_hook(0, u64::MAX, {
            let last_addr = last_addr.clone();
            let target = target.clone();
            let reached_bn = reached_bn.clone();
            move |uc, addr, _size| {
                if last_addr.load(Ordering::Relaxed) == ADDR_BRANCH {
                    target.store(addr, Ordering::Relaxed);
                    reached_bn.store(1, Ordering::Relaxed);
                    uc.emu_stop().unwrap();
                }
                last_addr.store(addr, Ordering::Relaxed);
            }
        })
        .unwrap();

    for i in 0..N_RUNS_V5 {
        // 重置 free list
        let head: [u8; 4] = 0x20000200u32.to_le_bytes();
        uc.mem_write(0x20000100, &head).unwrap();
        let chunk_size: [u8; 4] = 12u32.to_le_bytes();
        uc.mem_write(0x20000200, &chunk_size).unwrap();
        let chunk_next: [u8; 4] = 0u32.to_le_bytes();
        uc.mem_write(0x20000204, &chunk_next).unwrap();

        // 重置寄存器
        uc.reg_write(RegisterARM::SP, sp as u64).unwrap();
        uc.reg_write(RegisterARM::R0, 0x20000300).unwrap();
        uc.reg_write(RegisterARM::R1, 0).unwrap();
        uc.reg_write(RegisterARM::R2, 0).unwrap();
        uc.reg_write(RegisterARM::R3, 0).unwrap();
        uc.reg_write(RegisterARM::R4, 0).unwrap();
        uc.reg_write(RegisterARM::R5, 0).unwrap();
        uc.reg_write(RegisterARM::R6, 0).unwrap();
        uc.reg_write(RegisterARM::R7, 0).unwrap();
        uc.reg_write(RegisterARM::R8, 0).unwrap();
        uc.reg_write(RegisterARM::R9, 0).unwrap();
        uc.reg_write(RegisterARM::R10, 0).unwrap();
        uc.reg_write(RegisterARM::R11, 0).unwrap();
        uc.reg_write(RegisterARM::R12, 0).unwrap();
        uc.reg_write(RegisterARM::LR, 0).unwrap();

        uc.mem_write((sp - 4) as u64, &safe_ret).unwrap();

        last_addr.store(0, Ordering::Relaxed);
        target.store(0, Ordering::Relaxed);
        reached_bn.store(0, Ordering::Relaxed);

        uc.reg_write(RegisterARM::PC, (ADDR_BASE + 0x1118) | 1).unwrap();
        uc.emu_start((ADDR_BASE + 0x1118) | 1, 0, 0, 10000).ok();

        if reached_bn.load(Ordering::Relaxed) == 1 {
            let t = target.load(Ordering::Relaxed);
            if t == ADDR_RETURN {
                correct += 1;
            } else if t == ADDR_MAIN {
                wrong += 1;
                println!("  [#{}] BUG! 跳转到 0x{:08X}", i, t);
            } else {
                other += 1;
                println!("  [#{}] unexpected: 0x{:08X}", i, t);
            }
        } else {
            other += 1;
            println!("  [#{}] 未到达 b.n, 最后地址: 0x{:08X}", i, last_addr.load(Ordering::Relaxed));
        }

        // 关键: 写入代码内存强制 TB 失效，模拟外设 MMIO 写导致的 TB 扰动
        // 交替失效 0x1113A 和 0x1113E 附近的 TB
        let invalidate_addr = if i % 3 == 0 {
            ADDR_RETURN // 0x1113A - 预期目标
        } else if i % 3 == 1 {
            ADDR_MAIN // 0x1113E - 错误目标
        } else {
            ADDR_BRANCH // 0x111D8 - b.n 指令本身
        };
        // 读取当前值，写回相同值（触发 TB 失效但不改变代码）
        let mut buf = [0u8; 4];
        uc.mem_read(invalidate_addr, &mut buf).unwrap();
        uc.mem_write(invalidate_addr, &buf).unwrap();
    }

    let total = correct + wrong + other;
    println!();
    println!("========== 结果 [v5 (TB 失效重建)] ==========");
    println!("指令: b.n 0x{:08X} (编码 0xE7AF) @ 0x{:08X}", ADDR_RETURN, ADDR_BRANCH);
    println!("运行次数: {} (总: {})", N_RUNS_V5, total);
    println!("  正确 (0x{:08X}): {:5} ({:.1}%)", ADDR_RETURN, correct, correct as f64 / N_RUNS_V5 as f64 * 100.0);
    println!("  错误 (0x{:08X}): {:5} ({:.1}%)", ADDR_MAIN, wrong, wrong as f64 / N_RUNS_V5 as f64 * 100.0);
    println!("  其他:             {:5}", other);

    if wrong > 0 {
        println!(">>> BUG 确认! 错误率 {}/{} ({:.1}%)", wrong, N_RUNS_V5, wrong as f64 / N_RUNS_V5 as f64 * 100.0);
    } else {
        println!("未复现 bug");
    }
}

/// 策略 v6: 填充 TB 缓存制造哈希碰撞，然后执行 b.n 检查链接错误
///
/// 在完整模拟器中，TB 缓存有大量条目。0x1113A 和 0x1113E 虽然哈希值不同
/// (0x47E vs 0x47A)，但在 TB 密集的场景下，tb_jmp_cache 的 4096 个槽位
/// 可能被大量填充，导致 tb_htable_lookup 的二级查找路径出现异常。
///
/// v6 策略: 在多个地址创建 TB 条目填充缓存，然后在缓存压力下执行 b.n。
#[test]
fn test_bn_v6_cache_pressure() {
    let mut correct = 0u32;
    let mut wrong = 0u32;
    let mut other = 0u32;

    // _malloc_r 函数二进制 blob
    const MALLOC_R_BLOB: &[u8] = &[
        0x2D, 0xE9, 0xF8, 0x43, 0xCD, 0x1C, 0x25, 0xF0, 0x03, 0x05, 0x08, 0x35,
        0x0C, 0x2D, 0x38, 0xBF, 0x0C, 0x25, 0x00, 0x2D, 0x06, 0x46, 0x01, 0xDB,
        0xA9, 0x42, 0x04, 0xD9, 0x0C, 0x23, 0x33, 0x60, 0x00, 0x20, 0xBD, 0xE8,
        0xF8, 0x83, 0xDF, 0xF8, 0xD4, 0x80, 0x00, 0xF0, 0x69, 0xF8, 0xD8, 0xF8,
        0x00, 0x30, 0x1C, 0x46, 0x44, 0xBB, 0x29, 0x46, 0x30, 0x46, 0xFF, 0xF7,
        0xBF, 0xFF, 0x43, 0x1C, 0x04, 0x46, 0x58, 0xD1, 0xD8, 0xF8, 0x00, 0x40,
        0x27, 0x46, 0x00, 0x2F, 0x43, 0xD1, 0x00, 0x2C, 0x4B, 0xD0, 0x23, 0x68,
        0x39, 0x46, 0x30, 0x46, 0x04, 0xEB, 0x03, 0x09, 0x00, 0xF0, 0xD8, 0xFB,
        0x81, 0x45, 0x42, 0xD1, 0x21, 0x68, 0x6D, 0x1A, 0x29, 0x46, 0x30, 0x46,
        0xFF, 0xF7, 0xA6, 0xFF, 0x01, 0x30, 0x3A, 0xD0, 0x23, 0x68, 0x2B, 0x44,
        0x23, 0x60, 0xD8, 0xF8, 0x00, 0x30, 0x5A, 0x68, 0x62, 0xBB, 0xC8, 0xF8,
        0x00, 0x70, 0x0F, 0xE0, 0x22, 0x68, 0x52, 0x1B, 0x20, 0xD4, 0x0B, 0x2A,
        0x17, 0xD9, 0x61, 0x19, 0xA3, 0x42, 0x25, 0x60, 0x18, 0xBF, 0x59, 0x60,
        0x63, 0x68, 0x08, 0xBF, 0xC8, 0xF8, 0x00, 0x10, 0x62, 0x51, 0x4B, 0x60,
        0x30, 0x46, 0x00, 0xF0, 0x2F, 0xF8, 0x04, 0xF1, 0x0B, 0x00, 0x23, 0x1D,
        0x20, 0xF0, 0x07, 0x00, 0xC2, 0x1A, 0x1C, 0xBF, 0x1B, 0x1A, 0xA3, 0x50,
        0xAF, 0xE7, 0x62, 0x68, 0xA3, 0x42, 0x0C, 0xBF, 0xC8, 0xF8, 0x00, 0x20,
        0x5A, 0x60, 0xEB, 0xE7, 0x23, 0x46, 0x64, 0x68, 0xAE, 0xE7, 0x3C, 0x46,
        0x7F, 0x68, 0xB6, 0xE7, 0x1A, 0x46, 0x5B, 0x68, 0xA3, 0x42, 0xFB, 0xD1,
        0x00, 0x23, 0x53, 0x60, 0xDE, 0xE7, 0x0C, 0x23, 0x33, 0x60, 0x30, 0x46,
        0x00, 0xF0, 0x0C, 0xF8, 0x94, 0xE7, 0x05, 0x60, 0xD6, 0xE7, 0x00, 0xBF,
        0x00, 0x01, 0x00, 0x20, 0x00, 0xBF, 0x09, 0xE0, 0x00, 0xBF, 0x00, 0xBF,
        0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x03, 0xE0, 0x00, 0xBF, 0x00, 0xBF,
        0x00, 0xBF, 0x00, 0xBF, 0x70, 0x47, 0x00, 0xBF,
    ];

    const N_RUNS_V6: u32 = 5000;

    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();
    uc.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x10000, Prot::ALL).unwrap();
    uc.mem_write(ADDR_BASE + 0x1118, MALLOC_R_BLOB).unwrap();

    let sp = 0x2000_0F00u32;

    let last_addr = Arc::new(AtomicU64::new(0));
    let target = Arc::new(AtomicU64::new(0));
    let reached_bn = Arc::new(AtomicU64::new(0));

    let _hook = uc
        .add_code_hook(0, u64::MAX, {
            let last_addr = last_addr.clone();
            let target = target.clone();
            let reached_bn = reached_bn.clone();
            move |uc, addr, _size| {
                if last_addr.load(Ordering::Relaxed) == ADDR_BRANCH {
                    target.store(addr, Ordering::Relaxed);
                    reached_bn.store(1, Ordering::Relaxed);
                    uc.emu_stop().unwrap();
                }
                last_addr.store(addr, Ordering::Relaxed);
            }
        })
        .unwrap();

    // 第一阶段: 填充 TB 缓存 — 在多个地址创建 bx lr 的 TB
    // 这会占据 tb_jmp_cache 的多个槽位，模拟完整模拟器的缓存压力
    let fill_range = ADDR_BASE + 0x2000;
    let fill_count = 2000u64;
    for j in 0..fill_count {
        let addr = fill_range + j * 4;
        // bx lr (0x4770) — 2 字节指令，立即返回
        uc.mem_write(addr, &[0x70, 0x47]).unwrap();
        // 设置 lr 为安全的返回地址
        uc.reg_write(RegisterARM::LR, (addr + 4) | 1).unwrap();
        uc.reg_write(RegisterARM::PC, addr | 1).unwrap();
        uc.emu_start(addr | 1, addr + 2, 0, 2).ok();
    }
    println!("TB 缓存填充: {} 个 bx lr TB 已创建", fill_count);

    // 第二阶段: 反复执行完整 _malloc_r，检查 b.n 跳转目标
    // 不同于 v4: 此时 TB 缓存已被 2000 个 bx lr 占据，模拟缓存压力
    let safe_ret: [u8; 4] = ((ADDR_MAIN | 1) as u32).to_le_bytes();

    for i in 0..N_RUNS_V6 {
        // 重置 free list
        let head: [u8; 4] = 0x20000200u32.to_le_bytes();
        uc.mem_write(0x20000100, &head).unwrap();
        let chunk_size: [u8; 4] = 12u32.to_le_bytes();
        uc.mem_write(0x20000200, &chunk_size).unwrap();
        let chunk_next: [u8; 4] = 0u32.to_le_bytes();
        uc.mem_write(0x20000204, &chunk_next).unwrap();

        // 重置寄存器
        uc.reg_write(RegisterARM::SP, sp as u64).unwrap();
        uc.reg_write(RegisterARM::R0, 0x20000300).unwrap();
        uc.reg_write(RegisterARM::R1, 0).unwrap();
        uc.reg_write(RegisterARM::R2, 0).unwrap();
        uc.reg_write(RegisterARM::R3, 0).unwrap();
        uc.reg_write(RegisterARM::R4, 0).unwrap();
        uc.reg_write(RegisterARM::R5, 12).unwrap();
        uc.reg_write(RegisterARM::R6, 0x20000300).unwrap();
        uc.reg_write(RegisterARM::R7, 0).unwrap();
        uc.reg_write(RegisterARM::R8, 0).unwrap();
        uc.reg_write(RegisterARM::R9, 0).unwrap();
        uc.reg_write(RegisterARM::R10, 0).unwrap();
        uc.reg_write(RegisterARM::R11, 0).unwrap();
        uc.reg_write(RegisterARM::R12, 0).unwrap();
        uc.reg_write(RegisterARM::LR, 0).unwrap();

        uc.mem_write((sp - 4) as u64, &safe_ret).unwrap();

        last_addr.store(0, Ordering::Relaxed);
        target.store(0, Ordering::Relaxed);
        reached_bn.store(0, Ordering::Relaxed);

        uc.reg_write(RegisterARM::PC, (ADDR_BASE + 0x1118) | 1).unwrap();
        uc.emu_start((ADDR_BASE + 0x1118) | 1, 0, 0, 10000).ok();

        if reached_bn.load(Ordering::Relaxed) == 1 {
            let t = target.load(Ordering::Relaxed);
            if t == ADDR_RETURN {
                correct += 1;
            } else if t == ADDR_MAIN {
                wrong += 1;
                println!("  [#{}] BUG! b.n → 0x{:08X} (应为 0x{:08X})", i, t, ADDR_RETURN);
            } else {
                other += 1;
                println!("  [#{}] unexpected: 0x{:08X}", i, t);
            }
        } else {
            other += 1;
            if i < 10 {
                println!("  [#{}] 未到达 b.n, last=0x{:08X}", i, last_addr.load(Ordering::Relaxed));
            }
        }

        // 每次迭代后失效 b.n 目标区域的 TB，模拟持续扰动
        let mut buf = [0u8; 4];
        uc.mem_read(ADDR_RETURN, &mut buf).unwrap();
        uc.mem_write(ADDR_RETURN, &buf).unwrap();
        uc.mem_read(ADDR_MAIN, &mut buf).unwrap();
        uc.mem_write(ADDR_MAIN, &buf).unwrap();
    }

    let total = correct + wrong + other;
    println!();
    println!("========== 结果 [v6 (缓存压力)] ==========");
    println!("指令: b.n 0x{:08X} (编码 0xE7AF) @ 0x{:08X}", ADDR_RETURN, ADDR_BRANCH);
    println!("TB 缓存填充: {} 条, 运行次数: {} (总: {})", fill_count, N_RUNS_V6, total);
    println!("  正确 (0x{:08X}): {:5} ({:.1}%)", ADDR_RETURN, correct, correct as f64 / N_RUNS_V6 as f64 * 100.0);
    println!("  错误 (0x{:08X}): {:5} ({:.1}%)", ADDR_MAIN, wrong, wrong as f64 / N_RUNS_V6 as f64 * 100.0);
    println!("  其他:             {:5}", other);

    if wrong > 0 {
        println!(">>> BUG 确认! 错误率 {}/{} ({:.1}%)", wrong, N_RUNS_V6, wrong as f64 / N_RUNS_V6 as f64 * 100.0);
    } else {
        println!("未复现 bug");
    }
}

/// 验证 bx lr 在 Unicorn 中是否正常工作
#[test]
fn test_bx_lr() {
    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();
    uc.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();

    // 测试1: bx lr (0x4770)
    uc.mem_write(ADDR_BASE + 0x100, &[0x70, 0x47]).unwrap(); // bx lr
    uc.reg_write(RegisterARM::LR, (ADDR_MAIN | 1) as u64).unwrap();
    uc.reg_write(RegisterARM::PC, (ADDR_BASE + 0x100) | 1).unwrap();

    let hit = Arc::new(AtomicU64::new(0));
    let _hook = uc.add_code_hook(ADDR_MAIN, ADDR_MAIN + 2, {
        let hit = hit.clone();
        move |_uc, addr, _size| {
            hit.store(addr, Ordering::Relaxed);
        }
    }).unwrap();

    uc.emu_start((ADDR_BASE + 0x100) | 1, 0, 0, 10).ok();
    println!("bx lr (0x4770): hit=0x{:08X}", hit.load(Ordering::Relaxed));

    // 测试2: mov pc, lr (0x46F7)
    let mut uc2 = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();
    uc2.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();
    uc2.mem_write(ADDR_BASE + 0x100, &[0xF7, 0x46]).unwrap(); // mov pc, lr
    uc2.reg_write(RegisterARM::LR, (ADDR_MAIN | 1) as u64).unwrap();
    uc2.reg_write(RegisterARM::PC, (ADDR_BASE + 0x100) | 1).unwrap();

    let hit2 = Arc::new(AtomicU64::new(0));
    let _hook2 = uc2.add_code_hook(ADDR_MAIN, ADDR_MAIN + 2, {
        let hit2 = hit2.clone();
        move |_uc, addr, _size| {
            hit2.store(addr, Ordering::Relaxed);
        }
    }).unwrap();

    uc2.emu_start((ADDR_BASE + 0x100) | 1, 0, 0, 10).ok();
    println!("mov pc,lr (0x46F7): hit=0x{:08X}", hit2.load(Ordering::Relaxed));

    // 测试3: pop {pc} (0xBD00)
    let mut uc3 = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();
    uc3.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();
    uc3.mem_map(0x2000_0000, 0x1000, Prot::ALL).unwrap();
    uc3.mem_write(ADDR_BASE + 0x100, &[0x00, 0xBD]).unwrap(); // pop {pc}
    uc3.reg_write(RegisterARM::SP, 0x2000_0100).unwrap();
    let ret_addr: [u8; 4] = ((ADDR_MAIN | 1) as u32).to_le_bytes();
    uc3.mem_write(0x2000_0100, &ret_addr).unwrap();
    uc3.reg_write(RegisterARM::PC, (ADDR_BASE + 0x100) | 1).unwrap();

    let hit3 = Arc::new(AtomicU64::new(0));
    let _hook3 = uc3.add_code_hook(ADDR_MAIN, ADDR_MAIN + 2, {
        let hit3 = hit3.clone();
        move |_uc, addr, _size| {
            hit3.store(addr, Ordering::Relaxed);
        }
    }).unwrap();

    uc3.emu_start((ADDR_BASE + 0x100) | 1, 0, 0, 10).ok();
    println!("pop {{pc}} (0xBD00): hit=0x{:08X}", hit3.load(Ordering::Relaxed));
}

/// 验证 bl 后 lr 的值
#[test]
fn test_bl_lr_value() {
    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();
    uc.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x1000, Prot::ALL).unwrap();

    // 在 0x100 放一个 bl 到 0x200
    // F000 F07E: bl 0x200 (offset = 0x200 - (0x100 + 4) = 0xFC)
    // But we need to encode the bl correctly.
    // Let's compute: offset = 0x200 - 0x104 = 0xFC
    // S = 0, I1 = 0, I2 = 0, imm10 = 0, imm11 = 0xFC >> 1 = 0x7E, bit0 = 0
    // J1 = 1, J2 = 1
    // H1 = 11110 S imm10 = 11110 0 0000000000 = 0xF000
    // H2 = 11 J1 1 J2 imm11 = 11 1 1 1 00001111110 = 0b1111100001111110 = 0xF87E

    // 0x100: bl 0x200
    uc.mem_write(ADDR_BASE + 0x100, &[0x00, 0xF0, 0x7E, 0xF8]).unwrap();

    // 在 0x200: 放个 bkpt 或 nop->bkpt 停住
    uc.mem_write(ADDR_BASE + 0x200, &[0x00, 0xBE]).unwrap(); // bkpt

    let ret_addr = Arc::new(AtomicU64::new(0));
    let _hook = uc
        .add_code_hook(ADDR_BASE + 0x200, ADDR_BASE + 0x202, {
            let ret_addr = ret_addr.clone();
            move |uc, _addr, _size| {
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                ret_addr.store(lr, Ordering::Relaxed);
                uc.emu_stop().unwrap();
            }
        })
        .unwrap();

    uc.reg_write(RegisterARM::PC, (ADDR_BASE + 0x100) | 1).unwrap();
    uc.emu_start((ADDR_BASE + 0x100) | 1, 0, 0, 10).ok();

    let lr = ret_addr.load(Ordering::Relaxed);
    println!("bl 后 lr = 0x{:08X} (bit0={})", lr, lr & 1);
    println!("预期: 0x{:08X} (bl 下一条指令地址)", ADDR_BASE + 0x100 + 4);
}

/// 调试 v4: 追踪指令执行路径，找出 FETCH_UNMAPPED 发生位置
#[test]
fn test_bn_v4_debug_trace() {
    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();

    uc.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x10000, Prot::ALL).unwrap();

    const MALLOC_R_BLOB: &[u8] = &[
        0x2D, 0xE9, 0xF8, 0x43, 0xCD, 0x1C, 0x25, 0xF0, 0x03, 0x05, 0x08, 0x35,
        0x0C, 0x2D, 0x38, 0xBF, 0x0C, 0x25, 0x00, 0x2D, 0x06, 0x46, 0x01, 0xDB,
        0xA9, 0x42, 0x04, 0xD9, 0x0C, 0x23, 0x33, 0x60, 0x00, 0x20, 0xBD, 0xE8,
        0xF8, 0x83, 0xDF, 0xF8, 0xD4, 0x80, 0x00, 0xF0, 0x69, 0xF8, 0xD8, 0xF8,
        0x00, 0x30, 0x1C, 0x46, 0x44, 0xBB, 0x29, 0x46, 0x30, 0x46, 0xFF, 0xF7,
        0xBF, 0xFF, 0x43, 0x1C, 0x04, 0x46, 0x58, 0xD1, 0xD8, 0xF8, 0x00, 0x40,
        0x27, 0x46, 0x00, 0x2F, 0x43, 0xD1, 0x00, 0x2C, 0x4B, 0xD0, 0x23, 0x68,
        0x39, 0x46, 0x30, 0x46, 0x04, 0xEB, 0x03, 0x09, 0x00, 0xF0, 0xD8, 0xFB,
        0x81, 0x45, 0x42, 0xD1, 0x21, 0x68, 0x6D, 0x1A, 0x29, 0x46, 0x30, 0x46,
        0xFF, 0xF7, 0xA6, 0xFF, 0x01, 0x30, 0x3A, 0xD0, 0x23, 0x68, 0x2B, 0x44,
        0x23, 0x60, 0xD8, 0xF8, 0x00, 0x30, 0x5A, 0x68, 0x62, 0xBB, 0xC8, 0xF8,
        0x00, 0x70, 0x0F, 0xE0, 0x22, 0x68, 0x52, 0x1B, 0x20, 0xD4, 0x0B, 0x2A,
        0x17, 0xD9, 0x61, 0x19, 0xA3, 0x42, 0x25, 0x60, 0x18, 0xBF, 0x59, 0x60,
        0x63, 0x68, 0x08, 0xBF, 0xC8, 0xF8, 0x00, 0x10, 0x62, 0x51, 0x4B, 0x60,
        0x30, 0x46, 0x00, 0xF0, 0x2F, 0xF8, 0x04, 0xF1, 0x0B, 0x00, 0x23, 0x1D,
        0x20, 0xF0, 0x07, 0x00, 0xC2, 0x1A, 0x1C, 0xBF, 0x1B, 0x1A, 0xA3, 0x50,
        0xAF, 0xE7, 0x62, 0x68, 0xA3, 0x42, 0x0C, 0xBF, 0xC8, 0xF8, 0x00, 0x20,
        0x5A, 0x60, 0xEB, 0xE7, 0x23, 0x46, 0x64, 0x68, 0xAE, 0xE7, 0x3C, 0x46,
        0x7F, 0x68, 0xB6, 0xE7, 0x1A, 0x46, 0x5B, 0x68, 0xA3, 0x42, 0xFB, 0xD1,
        0x00, 0x23, 0x53, 0x60, 0xDE, 0xE7, 0x0C, 0x23, 0x33, 0x60, 0x30, 0x46,
        0x00, 0xF0, 0x0C, 0xF8, 0x94, 0xE7, 0x05, 0x60, 0xD6, 0xE7, 0x00, 0xBF,
        0x00, 0x01, 0x00, 0x20, // 0x11214: .word 0x20000100 (global ptr)
        0x00, 0xBF, // 0x11218: nop (padding to avoid bx lr at bl target)
        0x70, 0x47, // 0x1121A: bx lr (__malloc_lock stub)
        0x00, 0xBF, 0x00, 0xBF, // 0x1121C-0x1121F: nop padding
        0x00, 0xBF, 0x00, 0xBF, // 0x11220-0x11223: nop padding
        0x70, 0x47, // 0x11224: bx lr (__malloc_unlock stub)
        0x00, 0xBF, 0x00, 0xBF, // 0x11226-0x11229: nop padding
        0x00, 0xBF, 0x00, 0xBF, // 0x1122A-0x1122D: nop padding
    ];

    uc.mem_write(ADDR_BASE + 0x1118, MALLOC_R_BLOB).unwrap();
    // global_ptr / __malloc_lock / __malloc_unlock 已嵌入 blob 中

    // free list: head = 0x20000200, chunk.size = 12, chunk.next = 0
    let head: [u8; 4] = 0x20000200u32.to_le_bytes();
    uc.mem_write(0x20000100, &head).unwrap();
    let chunk_size: [u8; 4] = 12u32.to_le_bytes();
    uc.mem_write(0x20000200, &chunk_size).unwrap();
    let chunk_next: [u8; 4] = 0u32.to_le_bytes();
    uc.mem_write(0x20000204, &chunk_next).unwrap();

    let sp = 0x2000_0F00u32;
    uc.reg_write(RegisterARM::SP, sp as u64).unwrap();
    uc.reg_write(RegisterARM::R0, 0x20000300).unwrap();
    uc.reg_write(RegisterARM::R1, 0).unwrap();

    let safe_ret: [u8; 4] = ((ADDR_MAIN | 1) as u32).to_le_bytes();
    uc.mem_write((sp - 4) as u64, &safe_ret).unwrap();

    // 追踪每条指令，找出 FETCH_UNMAPPED 发生位置
    let last_pc = Arc::new(AtomicU64::new(0));
    let trace_count = Arc::new(AtomicU64::new(0));
    let _hook = uc
        .add_code_hook(0, u64::MAX, {
            let last_pc = last_pc.clone();
            let trace_count = trace_count.clone();
            move |_uc, addr, _size| {
                let count = trace_count.fetch_add(1, Ordering::Relaxed);
                if count < 50 {
                    println!("  [{:3}] 0x{:08X}", count, addr);
                }
                last_pc.store(addr, Ordering::Relaxed);
            }
        })
        .unwrap();

    // 在 bx lr 存根处读取 lr 值
    let lr_at_bx = Arc::new(AtomicU64::new(0));
    let _hook_bx = uc
                .add_code_hook(ADDR_BASE + 0x121A, ADDR_BASE + 0x121C, {
            let lr_at_bx = lr_at_bx.clone();
            move |uc, _addr, _size| {
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                lr_at_bx.store(lr, Ordering::Relaxed);
                println!("  >>> bx lr: lr=0x{:08X} (bit0={})", lr, lr & 1);
            }
        })
        .unwrap();

    uc.reg_write(RegisterARM::PC, (ADDR_BASE + 0x1118) | 1).unwrap();
    let result = uc.emu_start((ADDR_BASE + 0x1118) | 1, 0, 0, 200);
    println!("emu_start 结果: {:?}", result);
    println!("最后执行地址: 0x{:08X}", last_pc.load(Ordering::Relaxed));
    println!("总指令数: {}", trace_count.load(Ordering::Relaxed));
    println!("bx lr 时 lr = 0x{:08X}", lr_at_bx.load(Ordering::Relaxed));
}

/// 验证 v4 上下文中 bl __malloc_lock 后的 LR 值和返回行为
#[test]
fn test_bn_v4_lr_and_return() {
    let mut uc = Unicorn::new(Arch::ARM, Mode::THUMB).unwrap();

    uc.mem_map(ADDR_BASE, 0x10000, Prot::ALL).unwrap();
    uc.mem_map(0x2000_0000, 0x10000, Prot::ALL).unwrap();

    // 简化场景: 只放 bl __malloc_lock 和后续指令
    // 0x11142: bl __malloc_lock (4 bytes)
    uc.mem_write(ADDR_BASE + 0x1142, &[0x00, 0xF0, 0x69, 0xF8]).unwrap();
    // 0x11146: nop (作为返回点标记)
    uc.mem_write(ADDR_BASE + 0x1146, &[0x00, 0xBF]).unwrap();
    // 0x11148: bkpt (停止)
    uc.mem_write(ADDR_BASE + 0x1148, &[0x00, 0xBE]).unwrap();

    // __malloc_lock stub: bx lr
    uc.mem_write(ADDR_BASE + 0x1218, &[0x70, 0x47]).unwrap();

    // 设置栈
    uc.reg_write(RegisterARM::SP, 0x2000_0F00).unwrap();

    // 在 bl 处设置 hook，读取 LR
    let lr_after_bl = Arc::new(AtomicU64::new(0));
    let ret_hit = Arc::new(AtomicU64::new(0));

    let _hook = uc
        .add_code_hook(ADDR_BASE + 0x1142, ADDR_BASE + 0x1146, {
            let lr_after_bl = lr_after_bl.clone();
            move |uc, _addr, _size| {
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                lr_after_bl.store(lr, Ordering::Relaxed);
            }
        })
        .unwrap();

    let _hook2 = uc
        .add_code_hook(ADDR_BASE + 0x1146, ADDR_BASE + 0x1148, {
            let ret_hit = ret_hit.clone();
            move |_uc, addr, _size| {
                ret_hit.store(addr, Ordering::Relaxed);
            }
        })
        .unwrap();

    uc.reg_write(RegisterARM::PC, (ADDR_BASE + 0x1142) | 1).unwrap();
    let result = uc.emu_start((ADDR_BASE + 0x1142) | 1, 0, 0, 100);
    println!("emu_start 结果: {:?}", result);

    let lr = lr_after_bl.load(Ordering::Relaxed);
    println!("bl 后 lr = 0x{:08X} (bit0={})", lr, lr & 1);
    println!("预期返回地址: 0x{:08X}", ADDR_BASE + 0x1146);
    println!("ret_hit = 0x{:08X} (0 表示未返回)", ret_hit.load(Ordering::Relaxed));
}

fn print_results(label: &str, correct: u32, wrong: u32, other: u32) {
    println!();
    println!("========== 结果 [{}] ==========", label);
    println!(
        "指令: b.n 0x{:08X} (编码 0xE7AF) @ 0x{:08X}",
        ADDR_RETURN, ADDR_BRANCH
    );
    println!("预期跳转目标: 0x{:08X}", ADDR_RETURN);
    println!("错误跳转目标: 0x{:08X}", ADDR_MAIN);
    println!("运行次数: {}", N_RUNS);
    println!();
    println!(
        "  正确 (0x{:08X}): {:4} ({:.1}%)",
        ADDR_RETURN,
        correct,
        correct as f64 / N_RUNS as f64 * 100.0
    );
    println!(
        "  错误 (0x{:08X}): {:4} ({:.1}%)",
        ADDR_MAIN,
        wrong,
        wrong as f64 / N_RUNS as f64 * 100.0
    );
    println!("  其他:             {:4}", other);

    if wrong > 0 {
        println!();
        println!(
            ">>> BUG 确认: b.n 跳转到 0x{:08X} (off by +4), 错误率 {}/{} ({:.1}%)",
            ADDR_MAIN,
            wrong,
            N_RUNS,
            wrong as f64 / N_RUNS as f64 * 100.0
        );
    } else {
        println!("\n未复现 bug (需要更多上下文)");
    }
}