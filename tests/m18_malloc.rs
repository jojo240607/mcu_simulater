//! M18 验收测试：验证模拟器对 newlib-nano malloc/calloc/free 的支持是否正常。
//!
//! 背景：jOS 启动诊断发现 board_init 阶段堆被快速耗尽（sbrk 145 次 / 累计 0x34E8B），
//! 需要隔离"是模拟器问题还是 jOS 问题"。本测试加载独立固件 `firmware/malloc_demo`，
//! 该固件用与 jOS 完全相同的技术栈（newlib-nano malloc/calloc/free + 静态堆 `_sbrk`，
//! 8KB 堆）跑确定性测试序列，把结果写入固定 SRAM 地址。
//!
//! 固件编译（arm-none-eabi-gcc，与 jOS 相同 CPU_FLAGS + nano.specs）：
//! ```text
//! arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
//!   -O1 -ffreestanding -nostartfiles -specs=nano.specs \
//!   -Wl,-e,Reset_Handler -Wl,-T,linker.ld -o malloc_demo.elf main.c
//! ```
//!
//! 结果区（0x20000000 起 0x20 字节）：
//!   0x20000000 R_BASIC            = 1（malloc 可写性）
//!   0x20000004 R_CALLOC           = 1（calloc 清零）
//!   0x20000008 R_FREE_REUSE       = 1（free 后地址复用）
//!   0x2000000C R_HEAP_TOTAL       = 填充循环成功分配总字节（8KB 堆应约 7.4KB）
//!   0x20000010 R_HEAP_EXHAUST     = 1（耗尽时 malloc 正确返回 NULL）
//!   0x20000014 R_REUSE_EXHAUST    = 1（耗尽后 free 首块可复用且可写）
//!   0x20000018 R_DONE             = 0xA5A5A5A5（完成标记）
//!
//! 若全部断言通过，则证明模拟器对 newlib-nano malloc 的支持正确，jOS 的堆耗尽
//! 属于 jOS 自身堆尺寸/使用量问题，而非模拟器缺陷。

use std::path::Path;

use mcu_simulater::machine::Machine;

const R_BASIC: u32 = 0x2000_0000;
const R_CALLOC: u32 = 0x2000_0004;
const R_FREE_REUSE: u32 = 0x2000_0008;
const R_HEAP_TOTAL: u32 = 0x2000_000C;
const R_HEAP_EXHAUST: u32 = 0x2000_0010;
const R_REUSE_EXHAUST: u32 = 0x2000_0014;
const R_DONE: u32 = 0x2000_0018;

const DONE_MAGIC: u32 = 0xA5A5_A5A5;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/malloc_demo/malloc_demo.elf");
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
fn m18_malloc_end_to_end() {
    let mut m = load_machine();

    // 分次运行直到固件写入完成标记（防止一次性预算不足/超时）
    let mut done = false;
    for _ in 0..60 {
        m.run(50_000).unwrap();
        if read_u32(&mut m, R_DONE) == DONE_MAGIC {
            done = true;
            break;
        }
    }
    assert!(done, "固件未在预算内完成 malloc 测试序列");

    let basic = read_u32(&mut m, R_BASIC);
    let calloc = read_u32(&mut m, R_CALLOC);
    let free_reuse = read_u32(&mut m, R_FREE_REUSE);
    let heap_total = read_u32(&mut m, R_HEAP_TOTAL);
    let heap_exhaust = read_u32(&mut m, R_HEAP_EXHAUST);
    let reuse_exhaust = read_u32(&mut m, R_REUSE_EXHAUST);

    eprintln!("[result] R_BASIC        = {basic} (期望 1)");
    eprintln!("[result] R_CALLOC       = {calloc} (期望 1)");
    eprintln!("[result] R_FREE_REUSE   = {free_reuse} (期望 1)");
    eprintln!("[result] R_HEAP_TOTAL   = {heap_total} (8KB 静态堆应约 7040~8000)");
    eprintln!("[result] R_HEAP_EXHAUST = {heap_exhaust} (期望 1)");
    eprintln!("[result] R_REUSE_EXHAUST= {reuse_exhaust} (期望 1)");

    assert_eq!(basic, 1, "基本 malloc 可写性应通过");
    assert_eq!(calloc, 1, "calloc 清零应通过");
    assert_eq!(free_reuse, 1, "free 后地址复用应通过");

    // 8KB 静态堆：newlib-nano 每块 8 字节头 + 8 对齐，malloc(80) 实际占 88 字节，
    // 8KB / 88 ≈ 93 块 → 累计 7440 字节。给合理容差（80 块~100 块）。
    assert!(
        (7040..=8000).contains(&heap_total),
        "8KB 堆填充总量应在 7040~8000 字节，实际 {heap_total}"
    );
    assert!(heap_total % 80 == 0, "填充总量应为 80 的整数倍，实际 {heap_total}");
    assert_eq!(heap_exhaust, 1, "堆耗尽时 malloc 应正确返回 NULL");
    assert_eq!(reuse_exhaust, 1, "耗尽后 free 首块应可被复用且可写");

    eprintln!("[PASS] malloc/calloc/free 在模拟器中工作正常，jOS 堆耗尽非模拟器缺陷");
}
