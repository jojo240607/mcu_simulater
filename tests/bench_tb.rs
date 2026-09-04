//! 临时 bisect：定位是哪个 hook 导致 E 机器 blocks/ins=1.000（每指令一个 TB）。
//! 完成后删除。运行：cargo test --release --test bench_tb -- --nocapture
use std::path::Path;

use mcu_simulater::machine::Machine;
use unicorn_engine::{HookType, Prot};

fn elf_path() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/fp_acceptance/fp_acceptance.elf")
}

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
    m.run(2_000_000).unwrap();
    let budget = 5_000_000usize;
    let before = n.load(Ordering::Relaxed);
    m.run(budget).unwrap();
    let after = n.load(Ordering::Relaxed);
    (after - before) as f64 / budget as f64
}

#[test]
fn tb_bisect() {
    // 1. 基线：仅计数 block hook
    let mut a = bare();
    println!(
        "[tb] bare + counter:          blocks/ins = {:.3}",
        blocks_per_ins(&mut a)
    );

    // 2. + 数据区 mem hook（FLASH/CCM/SRAM，覆盖代码区）
    let mut b = bare();
    b.cpu
        .add_mem_hook(
            HookType::MEM_READ | HookType::MEM_WRITE,
            0x0800_0000,
            0x2002_0000,
            |_uc, _ty, _addr, _size, _v| false,
        )
        .unwrap();
    println!(
        "[tb] + data memhook:          blocks/ins = {:.3}",
        blocks_per_ins(&mut b)
    );

    // 3. + intr hook
    let mut c = bare();
    c.cpu.add_intr_hook(|_uc, _intno| {}).unwrap();
    println!(
        "[tb] + intr hook:             blocks/ins = {:.3}",
        blocks_per_ins(&mut c)
    );

    // 4. + SCB MMIO hook（0xE000E000..0xE000F000）
    let mut d = bare();
    d.cpu
        .add_mem_hook(
            HookType::MEM_READ | HookType::MEM_WRITE,
            0xE000_E000,
            0xE000_F000,
            |_uc, _ty, _addr, _size, _v| false,
        )
        .unwrap();
    println!(
        "[tb] + SCB mmio:              blocks/ins = {:.3}",
        blocks_per_ins(&mut d)
    );

    // 5. + 外设区 MMIO hook（0x40000000..0x40024000）
    let mut e = bare();
    e.cpu
        .add_mem_hook(
            HookType::MEM_READ | HookType::MEM_WRITE,
            0x4000_0000,
            0x4002_4000,
            |_uc, _ty, _addr, _size, _v| false,
        )
        .unwrap();
    println!(
        "[tb] + periph mmio:           blocks/ins = {:.3}",
        blocks_per_ins(&mut e)
    );

    // 6. 全 hook 组合
    let mut f = bare();
    f.cpu
        .add_mem_hook(
            HookType::MEM_READ | HookType::MEM_WRITE,
            0x0800_0000,
            0x2002_0000,
            |_uc, _ty, _addr, _size, _v| false,
        )
        .unwrap();
    f.cpu
        .add_mem_hook(
            HookType::MEM_READ | HookType::MEM_WRITE,
            0xE000_E000,
            0xE000_F000,
            |_uc, _ty, _addr, _size, _v| false,
        )
        .unwrap();
    f.cpu
        .add_mem_hook(
            HookType::MEM_READ | HookType::MEM_WRITE,
            0x4000_0000,
            0x4002_4000,
            |_uc, _ty, _addr, _size, _v| false,
        )
        .unwrap();
    f.cpu.add_intr_hook(|_uc, _intno| {}).unwrap();
    println!(
        "[tb] + all hooks:             blocks/ins = {:.3}",
        blocks_per_ins(&mut f)
    );
}
