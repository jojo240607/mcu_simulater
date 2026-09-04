//! 临时探测：分解 block/code hook 在纯计算负载下的开销占比。
//! 完成后删除。运行：cargo test --release --test bench_probe -- --nocapture
use std::path::Path;
use std::time::Instant;

use mcu_simulater::machine::Machine;
use unicorn_engine::Prot;

fn elf_path() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/fp_acceptance/fp_acceptance.elf")
}

/// 裸机（无任何 hook）：手动 mem_map + load_elf + reset，不调用 map_stm32f407_layout
fn bare() -> Machine {
    let elf = elf_path();
    assert!(elf.exists(), "固件未编译：{elf:?}");
    let mut m = Machine::new_m4f().unwrap();
    m.cpu
        .mem_map(0x0800_0000, 0x0008_0000, Prot::ALL)
        .unwrap();
    m.cpu
        .mem_map(0x2000_0000, 0x0002_0000, Prot::ALL)
        .unwrap();
    m.cpu
        .mem_map(0x1000_0000, 0x0001_0000, Prot::ALL)
        .unwrap();
    m.cpu
        .mem_map(0xE000_E000, 0x1000, Prot::ALL)
        .unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

/// 每指令平均翻译块数（探测 TB 是否碎片化）
fn blocks_per_ins(m: &mut Machine) -> f64 {
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;
    let n = Arc::new(AtomicU64::new(0));
    let n2 = n.clone();
    m.cpu
        .add_block_hook(1, 0, move |_uc, _a, _s| {
            n2.fetch_add(1, Ordering::Relaxed);
        })
        .unwrap();
    let warm = 2_000_000usize;
    m.run(warm).unwrap();
    let budget = 5_000_000usize;
    let before = n.load(Ordering::Relaxed);
    m.run(budget).unwrap();
    let after = n.load(Ordering::Relaxed);
    (after - before) as f64 / budget as f64
}

fn run_bench(name: &str, mut m: Machine) {
    // 预热后重复 3 次取最小值：抑制 CPU 频率/热漂移噪声（min-of-3）
    m.run(2_000_000).unwrap(); // 预热
    let pc = m.cpu.reg_read_u32(unicorn_engine::RegisterARM::PC).unwrap();
    let ipsr = m
        .cpu
        .reg_read_u32(unicorn_engine::RegisterARM::IPSR)
        .unwrap();
    println!("[probe] {name} 预热后: pc=0x{pc:08X} ipsr={ipsr}");
    let budget = 100_000_000usize;
    let mut best = f64::MAX;
    for _ in 0..3 {
        let t0 = Instant::now();
        m.run(budget).unwrap();
        let elapsed = t0.elapsed().as_secs_f64();
        best = best.min(budget as f64 / elapsed / 1e6);
    }
    println!(
        "[probe] {name}: {budget} 指令 ×3 → {:.1} MIPS (min), run_iterations={}",
        best,
        m.run_iterations()
    );
}

#[test]
fn probe_hook_cost() {
    // A. 无任何 hook（纯 Unicorn 上限）
    run_bench("A 无 hook", bare());

    // B. 仅空 block hook（每块一次 FFI）
    {
        let mut m = bare();
        m.cpu
            .add_block_hook(1, 0, |_uc, _a, _s| {})
            .unwrap();
        run_bench("B 仅 block hook", m);
    }

    // C. 仅空 code hook（每指令一次 FFI，强制单指令 TB）
    {
        let mut m = bare();
        m.cpu
            .add_code_hook(1, 0, |_uc, _a, _s| {})
            .unwrap();
        run_bench("C 仅 code hook", m);
    }

    // D. block + code 双空 hook（近似机器当前 hook 数量）
    {
        let mut m = bare();
        m.cpu
            .add_block_hook(1, 0, |_uc, _a, _s| {})
            .unwrap();
        m.cpu
            .add_code_hook(1, 0, |_uc, _a, _s| {})
            .unwrap();
        run_bench("D block+code 空 hook", m);
    }

    // F. 仅空 mem hook（READ|WRITE，与机器同区间 0x0800_0000-0x2002_0000）
    //    → 隔离 mem hook 每次内存访问的 FFI 成本
    {
        let mut m = bare();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        run_bench("F 仅 mem hook", m);
    }

    // G. block + mem 双空 hook → hook FFI 组合成本（不含机器逻辑）
    {
        let mut m = bare();
        m.cpu
            .add_block_hook(1, 0, |_uc, _a, _s| {})
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        run_bench("G block+mem 空 hook", m);
    }

    // H1. 复刻机器 block hook 原子部分：clock.advance + mpu_enabled.load
    {
        use std::sync::atomic::{AtomicU64, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let clock = Arc::new(AtomicU64::new(0));
        let mpu_enabled = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查（此处不执行，仅模拟跳转）
                }
                c.fetch_add(size as u64 * 3, Ordering::Relaxed);
            })
            .unwrap();
        run_bench("H1 block(clock+mpu)", m);
    }

    // H2. H1 + any_active/tick_actives(7) + wdog + nvic_pending 完整复刻
    {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let clock = Arc::new(AtomicU64::new(0));
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let any_active = Arc::new(AtomicBool::new(false));
        let actives: Vec<Arc<AtomicBool>> =
            (0..7).map(|_| Arc::new(AtomicBool::new(false))).collect();
        let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
        let wdog_req = Arc::new(AtomicBool::new(false));
        let nvic_pending = Arc::new(AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        let aa = any_active.clone();
        let ac = actives.clone();
        let wr = wdog_req.clone();
        let np = nvic_pending.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查跳过
                }
                let cycles = size as u64 * 3;
                c.fetch_add(cycles, Ordering::Relaxed);
                if !aa.load(Ordering::Relaxed)
                    && ac.iter().any(|a| a.load(Ordering::Relaxed))
                {
                    aa.store(true, Ordering::Relaxed);
                }
                if aa.load(Ordering::Relaxed) {
                    // tick 循环（未激活时跳过）
                }
                if wr.load(Ordering::Relaxed) {
                    return;
                }
                if np.load(Ordering::Relaxed) {
                    // 中断检查跳过
                }
            })
            .unwrap();
        run_bench("H2 block(完整逻辑复刻)", m);
    }

    // H1c. H1 的 clock.advance 改用 Cell<u64>（无 lock xadd）→ 验证原子 RMW 是否大头
    {
        use std::cell::Cell;
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let clock = Arc::new(Cell::new(0u64));
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查跳过
                }
                c.set(c.get() + size as u64 * 3);
            })
            .unwrap();
        run_bench("H1c block(clock Cell + mpu)", m);
    }

    // H12. 单原子状态字（bit0=mpu,bit1=any_active,bit2=wdog,bit3=nvic）+ Cell 时钟 + 精简捕获
    // → 验证把 E 的 5 次独立原子判读合并为 1 次 load 能否逼近/突破 100 MIPS
    {
        use std::cell::Cell;
        use std::sync::atomic::{AtomicU8, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let status = Arc::new(AtomicU8::new(0));
        let clock = Arc::new(Cell::new(0u64));
        let st = status.clone();
        let c = clock.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                let s = st.load(Ordering::Relaxed);
                if s & 1 != 0 {
                    // XN 检查（不触发）
                }
                c.set(c.get() + size as u64 * 3);
                if s & 2 != 0 {
                    // tick 循环（不触发）
                }
                if s & 4 != 0 {
                    return;
                }
                if s & 8 != 0 {
                    // 中断检查（不触发）
                }
            })
            .unwrap();
        run_bench("H12 block(单状态字+Cell+精简捕获)", m);
    }

    // H13. H12 快速 block + E 全套 hook/映射（数据 mem hook + SCB/外设 MMIO + 4 映射区 MMIO
    //      + intr + 额外 mem_map，block 逻辑仍为快速版、精简捕获）
    //      → 定位 E(87.1) 与 H12(111.6) 之间 ~24 MIPS：hooks/mappings 在快速 block 上的边际开销
    {
        use std::cell::Cell;
        use std::sync::atomic::{AtomicU8, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let status = Arc::new(AtomicU8::new(0));
        let clock = Arc::new(Cell::new(0u64));
        let st = status.clone();
        let c = clock.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                let s = st.load(Ordering::Relaxed);
                if s & 1 != 0 {
                    // XN 检查（不触发）
                }
                c.set(c.get() + size as u64 * 3);
                if s & 2 != 0 {
                    // tick 循环（不触发）
                }
                if s & 4 != 0 {
                    return;
                }
                if s & 8 != 0 {
                    // 中断检查（不触发）
                }
            })
            .unwrap();
        // 数据区 mem hook（复刻 E 的 attach_data_access_hook 快路径）
        let st2 = status.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if st2.load(Ordering::Relaxed) & 1 == 0 {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        // SCB MMIO + 外设区 MMIO（复刻 E 的 attach_system_control / attach_t1_peripherals）
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0xE000_E000,
                0xE000_F000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x4000_0000,
                0x4004_0000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        // 复刻 E 的额外内存映射
        for (base, size) in [
            (0x4000_0000u64, 0x0004_0000u64),
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
            (0x6400_0000u64, 0x0001_0000u64),
            (0x6800_0000u64, 0x0001_0000u64),
            (0x6C00_0000u64, 0x0001_0000u64),
        ] {
            m.cpu.mem_map(base, size, Prot::ALL).unwrap();
        }
        // 映射区 MMIO hook
        for (base, size) in [
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
        ] {
            m.cpu
                .add_mmio_hook(base, base + size, |_uc, _ty, _addr, _size, _value| false)
                .unwrap();
        }
        m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
        run_bench("H13 E全套hook/映射+快速block", m);
    }

    // H13b. H13 去掉数据区 mem hook（其余 hook/映射照旧）→ 隔离数据 mem hook 的开销
    {
        use std::cell::Cell;
        use std::sync::atomic::{AtomicU8, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let status = Arc::new(AtomicU8::new(0));
        let clock = Arc::new(Cell::new(0u64));
        let st = status.clone();
        let c = clock.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                let s = st.load(Ordering::Relaxed);
                if s & 1 != 0 {
                }
                c.set(c.get() + size as u64 * 3);
                if s & 2 != 0 {
                }
                if s & 4 != 0 {
                    return;
                }
                if s & 8 != 0 {
                }
            })
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0xE000_E000,
                0xE000_F000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x4000_0000,
                0x4004_0000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        for (base, size) in [
            (0x4000_0000u64, 0x0004_0000u64),
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
            (0x6400_0000u64, 0x0001_0000u64),
            (0x6800_0000u64, 0x0001_0000u64),
            (0x6C00_0000u64, 0x0001_0000u64),
        ] {
            m.cpu.mem_map(base, size, Prot::ALL).unwrap();
        }
        for (base, size) in [
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
        ] {
            m.cpu
                .add_mmio_hook(base, base + size, |_uc, _ty, _addr, _size, _value| false)
                .unwrap();
        }
        m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
        run_bench("H13b 无数据memhook", m);
    }

    // H13c. H12 快速 block + 仅数据 mem hook（无 SCB/外设 MMIO、无额外映射、无 intr）
    //      → 隔离数据 mem hook 在快速 block 上的单独开销
    {
        use std::cell::Cell;
        use std::sync::atomic::{AtomicU8, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let status = Arc::new(AtomicU8::new(0));
        let clock = Arc::new(Cell::new(0u64));
        let st = status.clone();
        let c = clock.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                let s = st.load(Ordering::Relaxed);
                if s & 1 != 0 {
                }
                c.set(c.get() + size as u64 * 3);
                if s & 2 != 0 {
                }
                if s & 4 != 0 {
                    return;
                }
                if s & 8 != 0 {
                }
            })
            .unwrap();
        let st2 = status.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if st2.load(Ordering::Relaxed) & 1 == 0 {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        run_bench("H13c 仅数据memhook", m);
    }

    // H13d. H13 + 复刻 E 的 6 捕获 block 闭包（status/clock + timers/tick_actives/mpu/nvic 4 个
    //      dummy Arc，慢分支不触发）→ 验证 E(88) 与 H13(110) 的差距是否来自闭包捕获形态
    {
        use std::cell::Cell;
        use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
        use std::sync::{Arc, Mutex};
        let mut m = bare();
        let status = Arc::new(AtomicU8::new(0));
        let clock = Arc::new(Cell::new(0u64));
        let timers: Arc<Vec<Arc<Mutex<u8>>>> = Arc::new(Vec::new());
        let tick_actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(Vec::new());
        let mpu = Arc::new(Mutex::new(0u8));
        let nvic = Arc::new(Mutex::new(0u8));
        let st = status.clone();
        let c = clock.clone();
        let tm = timers.clone();
        let ta = tick_actives.clone();
        let mp = mpu.clone();
        let nv = nvic.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                let s = st.load(Ordering::Relaxed);
                if s & 1 != 0 {
                    let _g = mp.lock();
                }
                c.set(c.get() + size as u64 * 3);
                if s & 2 != 0 {
                    for t in tm.iter() {
                        let _g = t.lock();
                        let _ = ta.len();
                    }
                }
                if s & 4 != 0 {
                    let _g = nv.lock();
                    return;
                }
                if s & 8 != 0 {
                    let _g = nv.lock();
                }
            })
            .unwrap();
        // E 全套 hook/映射（同 H13）
        let st2 = status.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if st2.load(Ordering::Relaxed) & 1 == 0 {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0xE000_E000,
                0xE000_F000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x4000_0000,
                0x4004_0000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        for (base, size) in [
            (0x4000_0000u64, 0x0004_0000u64),
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
            (0x6400_0000u64, 0x0001_0000u64),
            (0x6800_0000u64, 0x0001_0000u64),
            (0x6C00_0000u64, 0x0001_0000u64),
        ] {
            m.cpu.mem_map(base, size, Prot::ALL).unwrap();
        }
        for (base, size) in [
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
        ] {
            m.cpu
                .add_mmio_hook(base, base + size, |_uc, _ty, _addr, _size, _value| false)
                .unwrap();
        }
        m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
        run_bench("H13d E6捕获+全套hook", m);
    }

    // H13e. H13d 但把 4 个冷 Arc 捆成一个 Arc<Cold>（status/clock 仍单独捕获，共 3 字段）
    //      → 量化「6 字段 → 3 字段」闭包瘦身收益
    {
        use std::cell::Cell;
        use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
        use std::sync::{Arc, Mutex};
        struct Cold {
            timers: Vec<Arc<Mutex<u8>>>,
            actives: Vec<Arc<AtomicBool>>,
            mpu: Arc<Mutex<u8>>,
            nvic: Arc<Mutex<u8>>,
        }
        let mut m = bare();
        let status = Arc::new(AtomicU8::new(0));
        let clock = Arc::new(Cell::new(0u64));
        let cold = Arc::new(Cold {
            timers: Vec::new(),
            actives: Vec::new(),
            mpu: Arc::new(Mutex::new(0u8)),
            nvic: Arc::new(Mutex::new(0u8)),
        });
        let st = status.clone();
        let c = clock.clone();
        let cd = cold.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                let s = st.load(Ordering::Relaxed);
                if s & 1 != 0 {
                    let _g = cd.mpu.lock();
                }
                c.set(c.get() + size as u64 * 3);
                if s & 2 != 0 {
                    for (t, a) in cd.timers.iter().zip(cd.actives.iter()) {
                        let _g = t.lock();
                        let _ = a.load(Ordering::Relaxed);
                    }
                }
                if s & 4 != 0 {
                    let _g = cd.nvic.lock();
                    return;
                }
                if s & 8 != 0 {
                    let _g = cd.nvic.lock();
                }
            })
            .unwrap();
        // E 全套 hook/映射（同 H13）
        let st2 = status.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if st2.load(Ordering::Relaxed) & 1 == 0 {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0xE000_E000,
                0xE000_F000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x4000_0000,
                0x4004_0000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        for (base, size) in [
            (0x4000_0000u64, 0x0004_0000u64),
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
            (0x6400_0000u64, 0x0001_0000u64),
            (0x6800_0000u64, 0x0001_0000u64),
            (0x6C00_0000u64, 0x0001_0000u64),
        ] {
            m.cpu.mem_map(base, size, Prot::ALL).unwrap();
        }
        for (base, size) in [
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
        ] {
            m.cpu
                .add_mmio_hook(base, base + size, |_uc, _ty, _addr, _size, _value| false)
                .unwrap();
        }
        m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
        run_bench("H13e 3字段(捆4冷Arc)", m);
    }

    // H13f. 单字段：status+clock+cold 全捆进一个 Arc<Ctx>，闭包仅捕获 1 个指针
    //      → 量化极限瘦身收益（与机器侧最终形态对应）
    {
        use std::cell::Cell;
        use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
        use std::sync::{Arc, Mutex};
        struct Ctx {
            status: AtomicU8,
            clock: Cell<u64>,
            timers: Vec<Arc<Mutex<u8>>>,
            actives: Vec<Arc<AtomicBool>>,
            mpu: Arc<Mutex<u8>>,
            nvic: Arc<Mutex<u8>>,
        }
        let mut m = bare();
        let ctx = Arc::new(Ctx {
            status: AtomicU8::new(0),
            clock: Cell::new(0),
            timers: Vec::new(),
            actives: Vec::new(),
            mpu: Arc::new(Mutex::new(0u8)),
            nvic: Arc::new(Mutex::new(0u8)),
        });
        let c2 = ctx.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                let s = c2.status.load(Ordering::Relaxed);
                if s & 1 != 0 {
                    let _g = c2.mpu.lock();
                }
                c2.clock.set(c2.clock.get() + size as u64 * 3);
                if s & 2 != 0 {
                    for (t, a) in c2.timers.iter().zip(c2.actives.iter()) {
                        let _g = t.lock();
                        let _ = a.load(Ordering::Relaxed);
                    }
                }
                if s & 4 != 0 {
                    let _g = c2.nvic.lock();
                    return;
                }
                if s & 8 != 0 {
                    let _g = c2.nvic.lock();
                }
            })
            .unwrap();
        // E 全套 hook/映射（同 H13）
        let st2 = ctx.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if st2.status.load(Ordering::Relaxed) & 1 == 0 {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0xE000_E000,
                0xE000_F000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x4000_0000,
                0x4004_0000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        for (base, size) in [
            (0x4000_0000u64, 0x0004_0000u64),
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
            (0x6400_0000u64, 0x0001_0000u64),
            (0x6800_0000u64, 0x0001_0000u64),
            (0x6C00_0000u64, 0x0001_0000u64),
        ] {
            m.cpu.mem_map(base, size, Prot::ALL).unwrap();
        }
        for (base, size) in [
            (0x5005_0000u64, 0x0000_1000u64),
            (0x5006_0000u64, 0x0000_1000u64),
            (0x5000_0000u64, 0x0000_5000u64),
            (0x6000_0000u64, 0x0000_1000u64),
        ] {
            m.cpu
                .add_mmio_hook(base, base + size, |_uc, _ty, _addr, _size, _value| false)
                .unwrap();
        }
        m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
        run_bench("H13f 1字段(全捆Ctx)", m);
    }

    // F2. 数据区间 mem hook，带 mpu_enabled 原子判读快路径（复刻机器 mem hook 主体）
    {
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let me = mpu_enabled.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if !me.load(Ordering::Relaxed) {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        run_bench("F2 mem(原子判读)", m);
    }

    // H3a. H2 block(原子完整逻辑) + 数据 mem hook → 隔离 mem hook 与 block 的组合
    {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let clock = Arc::new(AtomicU64::new(0));
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let any_active = Arc::new(AtomicBool::new(false));
        let actives: Vec<Arc<AtomicBool>> =
            (0..7).map(|_| Arc::new(AtomicBool::new(false))).collect();
        let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
        let wdog_req = Arc::new(AtomicBool::new(false));
        let nvic_pending = Arc::new(AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        let aa = any_active.clone();
        let ac = actives.clone();
        let wr = wdog_req.clone();
        let np = nvic_pending.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查跳过
                }
                let cycles = size as u64 * 3;
                c.fetch_add(cycles, Ordering::Relaxed);
                if !aa.load(Ordering::Relaxed)
                    && ac.iter().any(|a| a.load(Ordering::Relaxed))
                {
                    aa.store(true, Ordering::Relaxed);
                }
                if aa.load(Ordering::Relaxed) {
                    // tick 循环（未激活时跳过）
                }
                if wr.load(Ordering::Relaxed) {
                    return;
                }
                if np.load(Ordering::Relaxed) {
                    // 中断检查跳过
                }
            })
            .unwrap();
        let me2 = mpu_enabled.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if !me2.load(Ordering::Relaxed) {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        run_bench("H3a H2block+数据mem", m);
    }

    // H3b. H3a + 12 个不触发的 MMIO mem hook（复刻 E 的 hook 数量，验证边界检查循环）
    {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let clock = Arc::new(AtomicU64::new(0));
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let any_active = Arc::new(AtomicBool::new(false));
        let actives: Vec<Arc<AtomicBool>> =
            (0..7).map(|_| Arc::new(AtomicBool::new(false))).collect();
        let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
        let wdog_req = Arc::new(AtomicBool::new(false));
        let nvic_pending = Arc::new(AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        let aa = any_active.clone();
        let ac = actives.clone();
        let wr = wdog_req.clone();
        let np = nvic_pending.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查跳过
                }
                let cycles = size as u64 * 3;
                c.fetch_add(cycles, Ordering::Relaxed);
                if !aa.load(Ordering::Relaxed)
                    && ac.iter().any(|a| a.load(Ordering::Relaxed))
                {
                    aa.store(true, Ordering::Relaxed);
                }
                if aa.load(Ordering::Relaxed) {
                    // tick 循环（未激活时跳过）
                }
                if wr.load(Ordering::Relaxed) {
                    return;
                }
                if np.load(Ordering::Relaxed) {
                    // 中断检查跳过
                }
            })
            .unwrap();
        let me2 = mpu_enabled.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if !me2.load(Ordering::Relaxed) {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        // 12 个 MMIO 区域（0x40000000 起，负载不会访问 → 不触发，只吃 C 层边界检查）
        for i in 0..12u64 {
            let base = 0x4000_0000 + i * 0x400;
            m.cpu
                .add_mem_hook(
                    unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                    base,
                    base + 0x100,
                    |_uc, _ty, _addr, _size, _value| false,
                )
                .unwrap();
        }
        run_bench("H3b H3a+12个MMIO", m);
    }

    // E. 完整机器（当前基准，含 mem/mpu hook 快路径）→ 对照
    {
        let elf = elf_path();
        let mut m = Machine::new_m4f().unwrap();
        m.map_stm32f407_layout().unwrap();
        m.load_elf(&elf).unwrap();
        m.reset().unwrap();
        // 探测：外设是否激活（决定是否走 tick 慢路径）、挂起中断、MPU
        println!(
            "[probe] E 探测: any_active={} active_cnt={} nvic_pending={} mpu_enabled={}",
            m.peripheral_any_active(),
            m.peripheral_active_count(),
            m.nvic_pending(),
            m.mpu_enabled(),
        );
        // 用独立机器数 TB（避免计数 hook 污染主测速机的单 hook 快路径）
        let mut m2 = Machine::new_m4f().unwrap();
        m2.map_stm32f407_layout().unwrap();
        m2.load_elf(&elf).unwrap();
        m2.reset().unwrap();
        println!("[probe] E blocks/ins={:.3}", blocks_per_ins(&mut m2));
        // 自测：min-of-3 + run 迭代计数 + 运行前后状态（定位 emu_start 提前返回）
        m.run(2_000_000).unwrap();
        let pc = m.cpu.reg_read_u32(unicorn_engine::RegisterARM::PC).unwrap();
        let ipsr = m.cpu.reg_read_u32(unicorn_engine::RegisterARM::IPSR).unwrap();
        println!(
            "[probe] E 预热后: nvic_pending={} mpu_enabled={} run_iterations={} pc=0x{pc:08X} ipsr={ipsr}",
            m.nvic_pending(),
            m.mpu_enabled(),
            m.run_iterations()
        );
        let budget = 100_000_000usize;
        let mut best = f64::MAX;
        for _ in 0..3 {
            let t0 = std::time::Instant::now();
            m.run(budget).unwrap();
            let dt = t0.elapsed().as_secs_f64();
            best = best.min(budget as f64 / dt / 1e6);
        }
        println!(
            "[probe] E 完整机器(基准): {budget} 指令 ×3 → {best:.1} MIPS (min), run_iterations={}",
            m.run_iterations()
        );
        println!(
            "[probe] E 运行后: nvic_pending={} mpu_enabled={}",
            m.nvic_pending(),
            m.mpu_enabled()
        );
    }

    // H4. E 的 block hook 但外设激活（any_active=true，真实 Mutex tick 循环）→ 量化慢路径
    {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::sync::{Arc, Mutex};
        let mut m = bare();
        let clock = Arc::new(AtomicU64::new(0));
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let any_active = Arc::new(AtomicBool::new(true)); // 强制激活 → 走 tick 循环
        let actives: Vec<Arc<AtomicBool>> =
            (0..7).map(|_| Arc::new(AtomicBool::new(false))).collect();
        let timers: Arc<Mutex<Vec<Arc<Mutex<u64>>>>> = Arc::new(Mutex::new(
            (0..7).map(|_| Arc::new(Mutex::new(0u64))).collect(),
        ));
        let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
        let wdog_req = Arc::new(AtomicBool::new(false));
        let nvic_pending = Arc::new(AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        let aa = any_active.clone();
        let ac = actives.clone();
        let tm = timers.clone();
        let wr = wdog_req.clone();
        let np = nvic_pending.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查跳过
                }
                let cycles = size as u64 * 3;
                c.fetch_add(cycles, Ordering::Relaxed);
                if !aa.load(Ordering::Relaxed)
                    && ac.iter().any(|a| a.load(Ordering::Relaxed))
                {
                    aa.store(true, Ordering::Relaxed);
                }
                if aa.load(Ordering::Relaxed) {
                    let timers = tm.lock().unwrap();
                    for (t, a) in timers.iter().zip(ac.iter()) {
                        if a.load(Ordering::Relaxed) {
                            *t.lock().unwrap() += cycles;
                        }
                    }
                }
                if wr.load(Ordering::Relaxed) {
                    return;
                }
                if np.load(Ordering::Relaxed) {
                    // 中断检查跳过
                }
            })
            .unwrap();
        run_bench("H4 block(外设激活+tick锁)", m);
    }

    // H5. 同 H4 但 timers 冻结为 Arc<Vec>（去掉每块 timers.lock()）→ 验证 Vec 锁是否大头
    {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::sync::{Arc, Mutex};
        let mut m = bare();
        let clock = Arc::new(AtomicU64::new(0));
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let any_active = Arc::new(AtomicBool::new(true)); // 强制激活 → 走 tick 循环
        let actives: Vec<Arc<AtomicBool>> =
            (0..7).map(|_| Arc::new(AtomicBool::new(false))).collect();
        let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
        // 冻结：外层 Vec 无锁，仅内层单个 timer 的 Mutex
        let timers: Arc<Vec<Arc<Mutex<u64>>>> = Arc::new(
            (0..7).map(|_| Arc::new(Mutex::new(0u64))).collect(),
        );
        let wdog_req = Arc::new(AtomicBool::new(false));
        let nvic_pending = Arc::new(AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        let aa = any_active.clone();
        let ac = actives.clone();
        let tm = timers.clone();
        let wr = wdog_req.clone();
        let np = nvic_pending.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查跳过
                }
                let cycles = size as u64 * 3;
                c.fetch_add(cycles, Ordering::Relaxed);
                if !aa.load(Ordering::Relaxed)
                    && ac.iter().any(|a| a.load(Ordering::Relaxed))
                {
                    aa.store(true, Ordering::Relaxed);
                }
                if aa.load(Ordering::Relaxed) {
                    for (t, a) in tm.iter().zip(ac.iter()) {
                        if a.load(Ordering::Relaxed) {
                            *t.lock().unwrap() += cycles;
                        }
                    }
                }
                if wr.load(Ordering::Relaxed) {
                    return;
                }
                if np.load(Ordering::Relaxed) {
                    // 中断检查跳过
                }
            })
            .unwrap();
        run_bench("H5 block(冻结Vec+无锁)", m);
    }

    // H6. H3a + 空 intr hook → 验证 UC_HOOK_INTR 是否拖慢公共路径
    {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let clock = Arc::new(AtomicU64::new(0));
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let any_active = Arc::new(AtomicBool::new(false));
        let actives: Vec<Arc<AtomicBool>> =
            (0..7).map(|_| Arc::new(AtomicBool::new(false))).collect();
        let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
        let wdog_req = Arc::new(AtomicBool::new(false));
        let nvic_pending = Arc::new(AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        let aa = any_active.clone();
        let ac = actives.clone();
        let wr = wdog_req.clone();
        let np = nvic_pending.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查跳过
                }
                let cycles = size as u64 * 3;
                c.fetch_add(cycles, Ordering::Relaxed);
                if !aa.load(Ordering::Relaxed)
                    && ac.iter().any(|a| a.load(Ordering::Relaxed))
                {
                    aa.store(true, Ordering::Relaxed);
                }
                if aa.load(Ordering::Relaxed) {
                    // tick 循环（未激活时跳过）
                }
                if wr.load(Ordering::Relaxed) {
                    return;
                }
                if np.load(Ordering::Relaxed) {
                    // 中断检查跳过
                }
            })
            .unwrap();
        let me2 = mpu_enabled.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if !me2.load(Ordering::Relaxed) {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
        run_bench("H6 H3a+intr", m);
    }

    // H7. H6 + SCB MMIO + 外设区 MMIO（复刻 E 的 3 个 mem hook 全套）→ 验证 MMIO hook 是否拖慢
    {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let clock = Arc::new(AtomicU64::new(0));
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let any_active = Arc::new(AtomicBool::new(false));
        let actives: Vec<Arc<AtomicBool>> =
            (0..7).map(|_| Arc::new(AtomicBool::new(false))).collect();
        let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
        let wdog_req = Arc::new(AtomicBool::new(false));
        let nvic_pending = Arc::new(AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        let aa = any_active.clone();
        let ac = actives.clone();
        let wr = wdog_req.clone();
        let np = nvic_pending.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查跳过
                }
                let cycles = size as u64 * 3;
                c.fetch_add(cycles, Ordering::Relaxed);
                if !aa.load(Ordering::Relaxed)
                    && ac.iter().any(|a| a.load(Ordering::Relaxed))
                {
                    aa.store(true, Ordering::Relaxed);
                }
                if aa.load(Ordering::Relaxed) {
                    // tick 循环（未激活时跳过）
                }
                if wr.load(Ordering::Relaxed) {
                    return;
                }
                if np.load(Ordering::Relaxed) {
                    // 中断检查跳过
                }
            })
            .unwrap();
        let me2 = mpu_enabled.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if !me2.load(Ordering::Relaxed) {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        // SCB MMIO + 外设区 MMIO（不触发，但影响 TLB/hook 边界检查）
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0xE000_E000,
                0xE000_F000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x4000_0000,
                0x4004_0000,
                |_uc, _ty, _addr, _size, _value| false,
            )
            .unwrap();
        m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
        run_bench("H7 H6+SCB+外设MMIO", m);
    }

    // H8. H7 + 复刻 E 的额外内存映射（periph/RNG/DCMI/USB/FSMC/CAN，无 hook）
    //     → 验证 mem_map 区域数量是否拖慢 TCG（代码取指 TLB/区段分发）
    {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::sync::Arc;
        let mut m = bare();
        let clock = Arc::new(AtomicU64::new(0));
        let mpu_enabled = Arc::new(AtomicBool::new(false));
        let any_active = Arc::new(AtomicBool::new(false));
        let actives: Vec<Arc<AtomicBool>> =
            (0..7).map(|_| Arc::new(AtomicBool::new(false))).collect();
        let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
        let wdog_req = Arc::new(AtomicBool::new(false));
        let nvic_pending = Arc::new(AtomicBool::new(false));
        let c = clock.clone();
        let me = mpu_enabled.clone();
        let aa = any_active.clone();
        let ac = actives.clone();
        let wr = wdog_req.clone();
        let np = nvic_pending.clone();
        m.cpu
            .add_block_hook(1, 0, move |_uc, _addr, size| {
                if me.load(Ordering::Relaxed) {
                    // XN 检查跳过
                }
                let cycles = size as u64 * 3;
                c.fetch_add(cycles, Ordering::Relaxed);
                if !aa.load(Ordering::Relaxed)
                    && ac.iter().any(|a| a.load(Ordering::Relaxed))
                {
                    aa.store(true, Ordering::Relaxed);
                }
                if aa.load(Ordering::Relaxed) {
                    // tick 循环（未激活时跳过）
                }
                if wr.load(Ordering::Relaxed) {
                    return;
                }
                if np.load(Ordering::Relaxed) {
                    // 中断检查跳过
                }
            })
            .unwrap();
        let me2 = mpu_enabled.clone();
        m.cpu
            .add_mem_hook(
                unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                0x0800_0000,
                0x2002_0000,
                move |_uc, _ty, _addr, _size, _value| {
                    if !me2.load(Ordering::Relaxed) {
                        return false;
                    }
                    false
                },
            )
            .unwrap();
        // 复刻 E 的额外内存映射（仅映射，不挂 hook/外设）
        for (base, size) in [
            (0x4000_0000u64, 0x0004_0000u64), // 外设区
            (0x5005_0000u64, 0x0000_1000u64), // DCMI
            (0x5006_0000u64, 0x0000_1000u64), // RNG
            (0x5000_0000u64, 0x0000_5000u64), // USB OTG
            (0x6000_0000u64, 0x0000_1000u64), // FSMC 寄存器块
            (0x6400_0000u64, 0x0001_0000u64), // FSMC Bank2
            (0x6800_0000u64, 0x0001_0000u64), // FSMC Bank3
            (0x6C00_0000u64, 0x0001_0000u64), // FSMC Bank4
        ] {
            m.cpu.mem_map(base, size, Prot::ALL).unwrap();
        }
        // H9 变体：在映射区上额外挂 RNG/DCMI/USB/FSMC 空 hook（复刻 E 的 4 个 MMIO hook）
        for (base, size) in [
            (0x5005_0000u64, 0x0000_1000u64), // DCMI
            (0x5006_0000u64, 0x0000_1000u64), // RNG
            (0x5000_0000u64, 0x0000_5000u64), // USB OTG
            (0x6000_0000u64, 0x0000_1000u64), // FSMC 寄存器块
        ] {
            m.cpu
                .add_mmio_hook(base, base + size, |_uc, _ty, _addr, _size, _value| false)
                .unwrap();
        }
        m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
        run_bench("H9 H8+映射区hook", m);
    }

    // H10. H9 + 复刻 E 的闭包捕获环境（Cell 时钟 + 大体积 dummy 捕获）
    //      → 验证"闭包捕获体积/类型"是否拖慢 block hook（E 40.6 vs H2 57）
    // H11. H9 + Cell 时钟（无大体积捕获）→ 对照
    {
        use std::cell::Cell;
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::{Arc, Mutex};

        // H10：大体积捕获
        {
            let mut m = bare();
            let clock = Arc::new(Cell::new(0u64));
            let mpu_enabled = Arc::new(AtomicBool::new(false));
            let any_active = Arc::new(AtomicBool::new(false));
            // H10x：actives 扩到 20（复刻 E 的真实 TIM1-14+DMA/DAC/RTC/IWDG/WWDG 数量）
            let actives: Vec<Arc<AtomicBool>> =
                (0..20).map(|_| Arc::new(AtomicBool::new(false))).collect();
            let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
            let wdog_req = Arc::new(AtomicBool::new(false));
            let nvic_pending = Arc::new(AtomicBool::new(false));
            let nvic_dummy = Arc::new(Mutex::new(0u64));
            let mpu_dummy = Arc::new(Mutex::new(0u64));
            let timers_dummy: Arc<Vec<Arc<Mutex<u64>>>> =
                Arc::new((0..20).map(|_| Arc::new(Mutex::new(0u64))).collect());
            let c = clock.clone();
            let me = mpu_enabled.clone();
            let aa = any_active.clone();
            let ac = actives.clone();
            let wr = wdog_req.clone();
            let np = nvic_pending.clone();
            let nd = nvic_dummy.clone();
            let md = mpu_dummy.clone();
            let td = timers_dummy.clone();
            m.cpu
                .add_block_hook(1, 0, move |uc, _addr, size| {
                    if me.load(Ordering::Relaxed) {
                        let _g = nd.lock().unwrap(); // XN 分支（不触发）
                    }
                    let cycles = size as u64 * 3;
                    c.set(c.get() + cycles); // Cell 时钟
                    if !aa.load(Ordering::Relaxed)
                        && ac.iter().any(|a| a.load(Ordering::Relaxed))
                    {
                        aa.store(true, Ordering::Relaxed);
                    }
                    if aa.load(Ordering::Relaxed) {
                        for (t, a) in td.iter().zip(ac.iter()) {
                            if a.load(Ordering::Relaxed) {
                                let _g = t.lock().unwrap();
                            }
                        }
                    }
                    if wr.load(Ordering::Relaxed) {
                        return;
                    }
                    if np.load(Ordering::Relaxed) {
                        let _g = md.lock().unwrap(); // 中断分支（不触发）
                        let _ = uc;
                    }
                })
                .unwrap();
            let me2 = mpu_enabled.clone();
            m.cpu
                .add_mem_hook(
                    unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                    0x0800_0000,
                    0x2002_0000,
                    move |_uc, _ty, _addr, _size, _value| {
                        if !me2.load(Ordering::Relaxed) {
                            return false;
                        }
                        false
                    },
                )
                .unwrap();
            for (base, size) in [
                (0x4000_0000u64, 0x0004_0000u64),
                (0x5005_0000u64, 0x0000_1000u64),
                (0x5006_0000u64, 0x0000_1000u64),
                (0x5000_0000u64, 0x0000_5000u64),
                (0x6000_0000u64, 0x0000_1000u64),
                (0x6400_0000u64, 0x0001_0000u64),
                (0x6800_0000u64, 0x0001_0000u64),
                (0x6C00_0000u64, 0x0001_0000u64),
            ] {
                m.cpu.mem_map(base, size, Prot::ALL).unwrap();
            }
            for (base, size) in [
                (0x5005_0000u64, 0x0000_1000u64),
                (0x5006_0000u64, 0x0000_1000u64),
                (0x5000_0000u64, 0x0000_5000u64),
                (0x6000_0000u64, 0x0000_1000u64),
            ] {
                m.cpu
                    .add_mmio_hook(base, base + size, |_uc, _ty, _addr, _size, _value| false)
                    .unwrap();
            }
            m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
            run_bench("H10x H9+Cell时钟+大捕获+actives=20", m);
        }

        // H11：无大体积捕获（小捕获 + Cell 时钟）
        {
            let mut m = bare();
            let clock = Arc::new(Cell::new(0u64));
            let mpu_enabled = Arc::new(AtomicBool::new(false));
            let any_active = Arc::new(AtomicBool::new(false));
            let actives: Vec<Arc<AtomicBool>> =
                (0..7).map(|_| Arc::new(AtomicBool::new(false))).collect();
            let actives: Arc<Vec<Arc<AtomicBool>>> = Arc::new(actives);
            let wdog_req = Arc::new(AtomicBool::new(false));
            let nvic_pending = Arc::new(AtomicBool::new(false));
            let c = clock.clone();
            let me = mpu_enabled.clone();
            let aa = any_active.clone();
            let ac = actives.clone();
            let wr = wdog_req.clone();
            let np = nvic_pending.clone();
            m.cpu
                .add_block_hook(1, 0, move |_uc, _addr, size| {
                    if me.load(Ordering::Relaxed) {
                        // XN 分支（不触发）
                    }
                    let cycles = size as u64 * 3;
                    c.set(c.get() + cycles); // Cell 时钟
                    if !aa.load(Ordering::Relaxed)
                        && ac.iter().any(|a| a.load(Ordering::Relaxed))
                    {
                        aa.store(true, Ordering::Relaxed);
                    }
                    if aa.load(Ordering::Relaxed) {
                        // tick 循环（未激活跳过）
                    }
                    if wr.load(Ordering::Relaxed) {
                        return;
                    }
                    if np.load(Ordering::Relaxed) {
                        // 中断检查跳过
                    }
                })
                .unwrap();
            let me2 = mpu_enabled.clone();
            m.cpu
                .add_mem_hook(
                    unicorn_engine::HookType::MEM_READ | unicorn_engine::HookType::MEM_WRITE,
                    0x0800_0000,
                    0x2002_0000,
                    move |_uc, _ty, _addr, _size, _value| {
                        if !me2.load(Ordering::Relaxed) {
                            return false;
                        }
                        false
                    },
                )
                .unwrap();
            for (base, size) in [
                (0x4000_0000u64, 0x0004_0000u64),
                (0x5005_0000u64, 0x0000_1000u64),
                (0x5006_0000u64, 0x0000_1000u64),
                (0x5000_0000u64, 0x0000_5000u64),
                (0x6000_0000u64, 0x0000_1000u64),
                (0x6400_0000u64, 0x0001_0000u64),
                (0x6800_0000u64, 0x0001_0000u64),
                (0x6C00_0000u64, 0x0001_0000u64),
            ] {
                m.cpu.mem_map(base, size, Prot::ALL).unwrap();
            }
            for (base, size) in [
                (0x5005_0000u64, 0x0000_1000u64),
                (0x5006_0000u64, 0x0000_1000u64),
                (0x5000_0000u64, 0x0000_5000u64),
                (0x6000_0000u64, 0x0000_1000u64),
            ] {
                m.cpu
                    .add_mmio_hook(base, base + size, |_uc, _ty, _addr, _size, _value| false)
                    .unwrap();
            }
            m.cpu.add_intr_hook(|_uc, _no| {}).unwrap();
            run_bench("H11 H9+Cell时钟", m);
        }
    }
}
