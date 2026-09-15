//! 性能基准：仿真飞控代码（jOS minimal + drvtest app + 虚拟外设）实际吞吐 MIPS。
//!
//! 口径：退役指令数按 Thumb 字节计数（≈2 字节/指令）→ MIPS = Δretired/2/1e6/墙钟秒。
//! 跑真实负载（固件启动 + 用例调度），不含任何测试断言。

use std::time::Instant;

use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;

#[test]
#[ignore]
fn flight_controller_throughput() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&artifact::joc_base_elf()).unwrap();
    m.load_app_partition(&artifact::drvtest_app_bin()).unwrap();
    m.reset().unwrap();

    // 预热（跳过启动峰值：复位/内存清零段）
    for _ in 0..100 {
        let _ = m.run(400_000);
    }
    let r0 = m.retired_count();
    let t0 = Instant::now();

    // 测 5 秒稳态吞吐
    let mut n = 0u32;
    while t0.elapsed().as_secs() < 5 {
        let _ = m.run(400_000);
        n += 1;
    }
    let dt = t0.elapsed().as_secs_f64();
    let r1 = m.retired_count();
    let bytes = (r1 - r0) as f64;
    let insns = bytes / 2.0; // Thumb ≈2 字节/指令
    let mips = insns / dt / 1e6;
    eprintln!(
        ">>> [BENCH] 仿真飞控代码稳态吞吐：退役 {:.0} 字节 / {dt:.2}s / {n} 段 run(400K)",
        bytes
    );
    eprintln!(">>> [BENCH] 估算指令数 ≈ {insns:.0}（字节÷2）→ **{mips:.1} MIPS**");
    assert!(mips > 0.0);
}

/// 对照：仅内核（无 app，无外设推流/中断风暴）的吞吐——接近裸热路径上限。
#[test]
#[ignore]
fn kernel_only_throughput() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&artifact::joc_base_elf()).unwrap();
    m.reset().unwrap();

    for _ in 0..50 {
        let _ = m.run(400_000);
    }
    let r0 = m.retired_count();
    let t0 = Instant::now();
    let mut n = 0u32;
    while t0.elapsed().as_secs() < 4 {
        let _ = m.run(400_000);
        n += 1;
    }
    let dt = t0.elapsed().as_secs_f64();
    let bytes = (m.retired_count() - r0) as f64;
    let mips = bytes / 2.0 / dt / 1e6;
    eprintln!(">>> [BENCH] 仅内核（无 app）吞吐：{bytes:.0} 字节 / {dt:.2}s / {n} 段 → **{mips:.1} MIPS**");
    assert!(mips > 0.0);
}
