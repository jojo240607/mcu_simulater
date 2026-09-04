//! MIPS 基准：测量模拟器吞吐（百万指令/秒）。
//!
//! 加载纯计算固件 fp_acceptance（浮点运算 + 末尾死循环，无外设等待/无中断），
//! 用 run() 大指令预算测量墙钟吞吐。指令数取 `run(budget)` 的预算本身——
//! 校准测试（bench_calibrate_count）已用 code hook 精确验证：
//! run(budget) 恰好执行 budget 条指令。
//!
//! 注意：block hook 的 `size` 是块字节数（Thumb-16 平均 2 字节/条），
//! `clock.cycles` 以 `size * AVG_CYCLES_PER_INS` 推进，因此 clock/3 约等于
//! 2× 指令数，不能直接用作 MIPS 的指令计数。
//!
//! 运行方式：`cargo test --release --test bench_mips -- --nocapture`
//! （debug 构建下 Rust 层开销会显著拉低 MIPS，请用 release 测量）

use std::path::Path;
use std::time::Instant;

use mcu_simulater::machine::Machine;

fn load(fp: &str) -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join(fp);
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

#[test]
fn bench_mips_compute() {
    let mut m = load("firmware/fp_acceptance/fp_acceptance.elf");

    // 预热：JIT/翻译缓存 + 块 hook 建立
    m.run(2_000_000).unwrap();

    // 计时窗口：大预算连续执行（run(budget) 精确执行 budget 条指令，见校准测试）
    let budget = 100_000_000usize;
    let t0 = Instant::now();
    m.run(budget).unwrap();
    let elapsed = t0.elapsed().as_secs_f64();

    let mips = budget as f64 / elapsed / 1e6;
    println!("[bench] 纯计算：{budget} 指令 / {elapsed:.3}s → {mips:.1} MIPS");

    // 仅报告，不做硬断言（不同机器吞吐不同）
    assert!(mips > 0.0);
}

/// 校准：code hook 精确数指令，验证 run(budget) 执行预算条指令，
/// 并揭示 block hook 的 `size` 为字节数（clock/3 ≈ 2× 指令数）。
#[test]
fn bench_calibrate_count() {
    let mut m = load("firmware/fp_acceptance/fp_acceptance.elf");
    m.run(1_000_000).unwrap(); // 预热

    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;

    // code hook：每条指令回调一次；同时累计字节数验证 block hook 的 size 语义
    let actual = Arc::new(AtomicU64::new(0));
    let bytes = Arc::new(AtomicU64::new(0));
    let a = actual.clone();
    let b = bytes.clone();
    m.cpu.add_code_hook(1, 0, move |_uc, _addr, size| {
        a.fetch_add(1, Ordering::Relaxed);
        b.fetch_add(size as u64, Ordering::Relaxed);
    })
    .unwrap();

    let budget = 2_000_000usize;
    let c0 = m.clock.count();
    m.run(budget).unwrap();
    let c1 = m.clock.count();
    let n = actual.load(Ordering::Relaxed);
    let nb = bytes.load(Ordering::Relaxed);
    println!(
        "[bench] 校准：budget={budget} codehook指令={n} codehook字节={nb} \
         clock增量={} clock/3={}",
        c1 - c0,
        (c1 - c0) / 3
    );
    assert_eq!(n, budget as u64, "run(budget) 应精确执行 budget 条指令");
}
