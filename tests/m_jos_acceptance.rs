//! jOS 自研 RTOS 验收测试：加载 joc-base 发布的 jOS 固件并在本模拟器中运行。
//!
//! 固件位于 d:\project\mcu\oop\joc-base\build_rel\stm32f407_minimal.elf
//! （RTOS_SELFTEST=OFF 发布版，USART1@115200 为控制台，TX 走 DMA2_Stream7_CH4）。
//!
//! 目标：观察 jOS 能否启动到控制台（打印 jOS RTOS ready / READY. Commands: ...）。

use std::path::Path;

use object::Object;
use object::ObjectSymbol;
use unicorn_engine::RegisterARM;

use mcu_simulater::machine::Machine;

const JOS_ELF: &str = r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf";

fn load_jos() -> Machine {
    let elf = Path::new(JOS_ELF);
    assert!(elf.exists(), "jOS 固件未编译：{JOS_ELF}");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

#[test]
fn m_jos_boot_diagnose() {
    let mut m = load_jos();

    // 校验关键节是否被 load_elf 写入内存
    for (name, addr) in [
        ("init_array", 0x0801_698Cusize),
        ("fini_array", 0x0801_6990),
        ("_rtos_tasks", 0x0801_6994),
    ] {
        let b = m.cpu.mem_read(addr as u64, 4).unwrap();
        eprintln!("[mem] {name} @0x{addr:08X} = 0x{:08X}", u32::from_le_bytes(b.try_into().unwrap()));
    }
    let b = m.cpu.mem_read(0x0801_698C, 8).unwrap();
    eprintln!("[mem] init_array 8B = {:02X?}", b);
    for (name, addr) in [
        ("data@20000020(heap.0)", 0x2000_0020usize),
        ("data@080169CC(LMA)", 0x0801_69CC),
        ("data@080169D0(LMA+4)", 0x0801_69D0),
    ] {
        let b = m.cpu.mem_read(addr as u64, 4).unwrap();
        eprintln!("[mem] {name} = 0x{:08X}", u32::from_le_bytes(b.try_into().unwrap()));
    }

    // 分多次小预算运行，逐段观察卡点
    use std::io::Write;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;
    let counter = Arc::new(AtomicU32::new(0));
    let c2 = counter.clone();
    m.cpu
        .add_code_hook(1, 0, move |uc, addr, _size| {
            let n = c2.fetch_add(1, Ordering::Relaxed);
            if addr >= 0x0801_1200 && addr <= 0x0801_1214 {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r5 = uc.reg_read(RegisterARM::R5).unwrap();
                let sp = uc.reg_read(RegisterARM::SP).unwrap();
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                eprintln!("[code] n={n} pc=0x{addr:08X} r0=0x{r0:08X} r5=0x{r5:08X} sp=0x{sp:08X} lr=0x{lr:08X}");
            }
            if addr == 0x0800_9B04 || addr == 0x0800_9B06 {
                let sp = uc.reg_read(RegisterARM::SP).unwrap();
                let msp = uc.reg_read(RegisterARM::MSP).unwrap();
                let psp = uc.reg_read(RegisterARM::PSP).unwrap();
                let control = uc.reg_read(RegisterARM::CONTROL).unwrap();
                eprintln!("[DBG] pc=0x{addr:08X} sp=0x{sp:08X} msp=0x{msp:08X} psp=0x{psp:08X} control=0x{control:08X}");
            }
            if addr == 0x0800_9AC4 {
                let sp = uc.reg_read(RegisterARM::SP).unwrap();
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                eprintln!("[DBG-ENTRY] board_tick_init n={n} sp=0x{sp:08X} lr=0x{lr:08X}");
            }
            if addr == 0x0800_14CE {
                let sp = uc.reg_read(RegisterARM::SP).unwrap();
                let r3 = uc.reg_read(RegisterARM::R3).unwrap();
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                eprintln!("[DBG-RET] system_early_init after board_tick_init n={n} sp=0x{sp:08X} r3=0x{r3:08X} lr=0x{lr:08X}");
            }
        })
        .unwrap();
    for step in 0..400 {
        let pc_before = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        eprintln!("[step {step}] enter run pc=0x{pc_before:08X}");
        let r = m.run(100_000);
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let reason = m.nvic.lock().unwrap().take_stop_reason();
        let out = m.console.lock().unwrap().output().to_vec();
        eprintln!(
            "[step {step}] pc0x{pc_before:08X}->0x{pc:08X} reason={reason:?} run={r:?} out_len={}",
            out.len()
        );
        if r.is_err() {
            // 转储 CCM RAM 设备管理器/哈希表区域
            let base = 0x1000_4000u64;
            let chunk = m.cpu.mem_read(base, 0x300).unwrap();
            eprintln!("[ccm] dump 0x10004000..0x10004300:");
            for (i, b) in chunk.chunks(16).enumerate() {
                let addr = base + (i * 16) as u64;
                let hex: Vec<String> = b.iter().map(|x| format!("{x:02X}")).collect();
                let ascii: String = b
                    .iter()
                    .map(|&x| if (0x20..0x7f).contains(&x) { x as char } else { '.' })
                    .collect();
                eprintln!("[ccm] {addr:08X}: {}  {}", hex.join(" "), ascii);
            }
            for (addr, nm) in [
                (0x1000_4000u64, "dm_init_flag"),
                (0x1000_413c, "dm_mgr_ptr"),
                (0x1000_4134, "dm_vtable"),
                (0x0800_1488, "app_id0"),
                (0x0800_148c, "app_id1"),
                (0x0800_1490, "app_id2"),
                (0x0800_1494, "app_id3"),
                (0x0800_1498, "app_id4"),
            ] {
                match m.cpu.mem_read(addr, 4) {
                    Ok(b) => eprintln!("[mem] {nm} @0x{addr:08X} = 0x{:08X}", u32::from_le_bytes(b.try_into().unwrap())),
                    Err(e) => eprintln!("[mem] {nm} @0x{addr:08X} = 读失败 {e:?}"),
                }
            }
            for (nm, reg) in [
                ("R0", RegisterARM::R0),
                ("R1", RegisterARM::R1),
                ("R2", RegisterARM::R2),
                ("R3", RegisterARM::R3),
                ("R4", RegisterARM::R4),
                ("R5", RegisterARM::R5),
                ("R6", RegisterARM::R6),
                ("R7", RegisterARM::R7),
                ("R8", RegisterARM::R8),
                ("R9", RegisterARM::R9),
                ("R12", RegisterARM::R12),
                ("SP", RegisterARM::SP),
                ("MSP", RegisterARM::MSP),
                ("PSP", RegisterARM::PSP),
                ("LR", RegisterARM::LR),
                ("PC", RegisterARM::PC),
                ("IPSR", RegisterARM::IPSR),
                ("CONTROL", RegisterARM::CONTROL),
                ("PRIMASK", RegisterARM::PRIMASK),
                ("BASEPRI", RegisterARM::BASEPRI),
            ] {
                eprintln!("[reg] {nm} = 0x{:08X}", m.cpu.reg_read(reg).unwrap());
            }
        }
        if !out.is_empty() {
            println!("[step {step}] 控制台有输出：{:?}", String::from_utf8_lossy(&out));
            break;
        }
        if pc == pc_before && r.is_err() {
            println!("[step {step}] PC 未前进且出错，疑似停机");
            break;
        }
    }

    let out = m.console.lock().unwrap().output().to_vec();
    let text = String::from_utf8_lossy(&out);
    println!("=== 最终控制台输出 ===\n{text}");
    assert!(!text.is_empty(), "jOS 应产生控制台输出");
}

/// 追踪 board_init 期间各 create/malloc/register 调用序列，定位 uart0 未注册的原因。
/// 使用 object crate 从 release build ELF 动态解析符号地址。
#[test]
fn m_jos_board_init_trace() {
    let mut m = load_jos();

    // 从 ELF 动态解析 release build 符号地址
    let elf_data = std::fs::read(JOS_ELF).unwrap();
    let elf = object::read::elf::ElfFile32::<object::Endianness>::parse(elf_data.as_slice()).unwrap();

    fn lookup<Elf: object::read::elf::FileHeader>(
        elf: &object::read::elf::ElfFile<Elf>,
        name: &str,
    ) -> Option<u64> {
        for sym in elf.symbols() {
            if let Ok(n) = sym.name() {
                if n == name {
                    return Some(sym.address());
                }
            }
        }
        None
    }

    // 打印关键堆符号
    for name in &[
        "g_sys_heap", "_end", "__HeapLimit", "__HeapBase",
        "_sbss", "_ebss", "_sdata", "_edata",
    ] {
        if let Some(addr) = lookup(&elf, name) {
            eprintln!("[sym] {name} = 0x{addr:08X}");
        }
    }

    let addr_board_init = lookup(&elf, "board_init").expect("board_init not found") & !1;
    let addr_uart_create = lookup(&elf, "uart_create").expect("uart_create not found") & !1;
    let addr_uart_hal_create = lookup(&elf, "uart_hal_create").expect("uart_hal_create not found") & !1;
    let addr_register = lookup(&elf, "device_manager_register").expect("register not found") & !1;
    let addr_malloc = lookup(&elf, "malloc").expect("malloc not found") & !1;
    let addr_malloc_r = lookup(&elf, "_malloc_r").expect("_malloc_r not found") & !1;
    let addr_calloc = lookup(&elf, "calloc").expect("calloc not found") & !1;
    let addr_calloc_r = lookup(&elf, "_calloc_r").expect("_calloc_r not found") & !1;
    let addr_sbrk = lookup(&elf, "_sbrk").expect("_sbrk not found") & !1;
    let addr_sbrk_r = lookup(&elf, "_sbrk_r").expect("_sbrk_r not found") & !1;

    eprintln!("[symbols] board_init=0x{addr_board_init:08X} uart_create=0x{addr_uart_create:08X} uart_hal_create=0x{addr_uart_hal_create:08X} register=0x{addr_register:08X} malloc=0x{addr_malloc:08X} _malloc_r=0x{addr_malloc_r:08X} calloc=0x{addr_calloc:08X} _calloc_r=0x{addr_calloc_r:08X} _sbrk=0x{addr_sbrk:08X} _sbrk_r=0x{addr_sbrk_r:08X}");

    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;

    // 追踪 calloc 返回（从 uart_create 调用的 calloc）
    let calloc_lr = Arc::new(AtomicU64::new(0));   // calloc 入口时的 LR
    let sbrk_lr = Arc::new(AtomicU64::new(0));      // _sbrk 入口时的 LR
    let in_calloc = Arc::new(AtomicU64::new(0));    // 是否在 calloc 内部
    let calloc_call_count = Arc::new(AtomicU64::new(0));
    let cl = calloc_lr.clone();
    let sl = sbrk_lr.clone();
    let ic = in_calloc.clone();
    let ccc = calloc_call_count.clone();

    m.cpu
        .add_code_hook(1, 0, move |uc, addr, _size| {
            fn cstr(uc: &unicorn_engine::Unicorn<'_, ()>, addr: u64) -> String {
                let mut s = String::new();
                let mut a = addr;
                for _ in 0..32 {
                    let mut b = [0u8; 1];
                    if uc.mem_read(a, &mut b).is_ok() && b[0] != 0 {
                        s.push(b[0] as char);
                        a += 1;
                    } else {
                        break;
                    }
                }
                s
            }

            match addr {
                a if a == addr_board_init => {
                    eprintln!("[trace] board_init entry");
                }
                a if a == addr_uart_create => {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    let lr = uc.reg_read(RegisterARM::LR).unwrap();
                    eprintln!("[trace] uart_create entry r0(config)=0x{r0:08X} lr=0x{lr:08X}");
                }
                a if a == addr_uart_hal_create => {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    eprintln!("[trace] uart_hal_create entry r0=0x{r0:08X}");
                }
                a if a == addr_register => {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                    eprintln!(
                        "[trace] register name=\"{}\" dev=0x{r1:08X}",
                        cstr(uc, r0)
                    );
                }
                // === calloc 追踪 ===
                a if a == addr_calloc => {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                    let lr = uc.reg_read(RegisterARM::LR).unwrap();
                    let n = ccc.fetch_add(1, Ordering::Relaxed);
                    eprintln!("[trace] calloc#{n} entry count=0x{r0:X} size=0x{r1:X} lr=0x{lr:08X}");
                    cl.store(lr, Ordering::Relaxed);
                    ic.store(1, Ordering::Relaxed);
                }
                a if a == addr_calloc_r => {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                    let r2 = uc.reg_read(RegisterARM::R2).unwrap();
                    let lr = uc.reg_read(RegisterARM::LR).unwrap();
                    eprintln!("[trace] _calloc_r entry r0=0x{r0:08X} count=0x{r1:X} size=0x{r2:X} lr=0x{lr:08X}");
                }
                // === _sbrk 追踪 ===
                a if a == addr_sbrk => {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    let lr = uc.reg_read(RegisterARM::LR).unwrap();
                    eprintln!("[trace] _sbrk entry incr=0x{r0:X} lr=0x{lr:08X}");
                    sl.store(lr, Ordering::Relaxed);
                }
                a if a == addr_sbrk_r => {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                    let lr = uc.reg_read(RegisterARM::LR).unwrap();
                    eprintln!("[trace] _sbrk_r entry reent=0x{r0:08X} incr=0x{r1:X} lr=0x{lr:08X}");
                    sl.store(lr, Ordering::Relaxed);
                }
                _ => {}
            }

            // 检测 _sbrk 返回：打印返回值 R0
            let saved_sl = sl.load(Ordering::Relaxed) & !1;
            if saved_sl != 0 && addr == saved_sl {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                eprintln!("[trace] _sbrk RETURNS r0=0x{r0:08X} (caller=0x{saved_sl:08X})");
                sl.store(0, Ordering::Relaxed);
            }

            // 检测 calloc 返回：打印返回值 R0
            let saved_cl = cl.load(Ordering::Relaxed) & !1;
            if saved_cl != 0 && addr == saved_cl {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                eprintln!("[trace] calloc RETURNS r0=0x{r0:08X} (caller=0x{saved_cl:08X})");
                ic.store(0, Ordering::Relaxed);
                cl.store(0, Ordering::Relaxed);
            }
        })
        .unwrap();

    let _ = m.run(2_000_000);
    eprintln!("[trace] board_init 后 PC=0x{:08X}", m.cpu.reg_read_u32(RegisterARM::PC).unwrap());
}

/// 追踪 _malloc_r 内部循环：检测 calloc#1 为何触发 86 次 _sbrk 调用。
/// 打印 _malloc_r → _sbrk_r 调用链中每次 _sbrk 的 LR 以及 _sbrk 返回后的指令序列。
#[test]
fn m_jos_malloc_loop_trace() {
    let mut m = load_jos();

    let elf_data = std::fs::read(JOS_ELF).unwrap();
    let elf = object::read::elf::ElfFile32::<object::Endianness>::parse(elf_data.as_slice()).unwrap();

    fn lookup<Elf: object::read::elf::FileHeader>(
        elf: &object::read::elf::ElfFile<Elf>,
        name: &str,
    ) -> Option<u64> {
        for sym in elf.symbols() {
            if let Ok(n) = sym.name() {
                if n == name {
                    return Some(sym.address());
                }
            }
        }
        None
    }

    let addr_sbrk = lookup(&elf, "_sbrk").expect("_sbrk not found") & !1;
    let addr_sbrk_r = lookup(&elf, "_sbrk_r").expect("_sbrk_r not found") & !1;
    let addr_malloc_r = lookup(&elf, "_malloc_r").expect("_malloc_r not found") & !1;
    let addr_calloc = lookup(&elf, "calloc").expect("calloc not found") & !1;

    eprintln!("[sym] _malloc_r=0x{addr_malloc_r:08X} _sbrk_r=0x{addr_sbrk_r:08X} _sbrk=0x{addr_sbrk:08X} calloc=0x{addr_calloc:08X}");

    // 固件内关键地址（从 objdump 反汇编获取）
    const ADDR_SBRK_ALIGNED: u64 = 0x0801_10D4; // sbrk_aligned 入口
    const ADDR_SBRK_ALIGNED_CALL1: u64 = 0x0801_10E0; // sbrk_aligned 内第一次 _sbrk_r 调用
    const ADDR_SBRK_ALIGNED_CALL2: u64 = 0x0801_10EA; // sbrk_aligned 内第二次 _sbrk_r 调用
    const ADDR_SBRK_ALIGNED_RET_CHECK: u64 = 0x0801_10EE; // sbrk_aligned 内 _sbrk_r 返回后检查
    const ADDR_MALLOC_R_EXTEND_CHECK: u64 = 0x0801_1170; // _malloc_r 内 add.w r9, r4, r3
    const ADDR_MALLOC_R_EXTEND_CALL: u64 = 0x0801_1174; // _malloc_r 内 bl _sbrk_r (扩展块)
    const ADDR_MALLOC_R_EXTEND_CMP: u64 = 0x0801_1178; // _malloc_r 内 cmp r9, r0
    const ADDR_MALLOC_R_ERROR: u64 = 0x0801_1202; // _malloc_r 错误返回路径

    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;

    let total_sbrk = Arc::new(AtomicU64::new(0));         // 全局 _sbrk 调用计数
    let total_calloc = Arc::new(AtomicU64::new(0));       // 全局 calloc 调用计数
    let total_malloc_r = Arc::new(AtomicU64::new(0));     // 全局 _malloc_r 调用计数
    let in_malloc_r = Arc::new(AtomicU64::new(0));        // 当前在 _malloc_r 内
    let sbrk_aligned_count = Arc::new(AtomicU64::new(0)); // sbrk_aligned 调用次数
    let sbrk_in_sbrk_aligned = Arc::new(AtomicU64::new(0)); // 在 sbrk_aligned 内的 _sbrk_r 调用计数

    let ts = total_sbrk.clone();
    let tc = total_calloc.clone();
    let tm = total_malloc_r.clone();
    let im = in_malloc_r.clone();
    let sac = sbrk_aligned_count.clone();
    let sisa = sbrk_in_sbrk_aligned.clone();

    // ── 追踪 b.n 0xE7AF 跳转目标 ──
    // _malloc_r 末尾 0x080111D8: b.n 0x0801113A (return), 编码 0xE7AF
    // 如果 Unicorn 有 b.n bug，可能跳到 0x0801113E (ldr.w r8, ...)，导致循环
    let b_n_seen = Arc::new(AtomicU64::new(0));
    let b_n_bug_count = Arc::new(AtomicU64::new(0));
    let bns = b_n_seen.clone();
    let bnbc = b_n_bug_count.clone();

    m.cpu
        .add_code_hook(1, 0, move |uc, addr, _size| {
            // ── 检测 b.n 0xE7AF 跳转 ──
            if bns.load(Ordering::Relaxed) == 1 {
                bns.store(0, Ordering::Relaxed);
                if addr == 0x0801_113A {
                    // 正确目标
                } else if addr == 0x0801_113E {
                    let n = bnbc.fetch_add(1, Ordering::Relaxed);
                    eprintln!("!!! B.N BUG #{n}: b.n 0xE7AF jumped to 0x{addr:08X} (expected 0x0801113A) !!!");
                } else {
                    eprintln!("??? b.n 0xE7AF jumped to unexpected 0x{addr:08X} ???");
                }
            }
            if addr == 0x0801_11D8 {
                bns.store(1, Ordering::Relaxed);
            }

            // ── _sbrk 入口：打印当前堆状态 ──
            if addr == addr_sbrk {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap(); // incr
                let n = ts.fetch_add(1, Ordering::Relaxed);
                // 读取 heap_end 和 heap_limit
                let mut heap_end_buf = [0u8; 4];
                let mut heap_limit_buf = [0u8; 4];
                let heap_end = if uc.mem_read(0x2000_0020, &mut heap_end_buf).is_ok() {
                    u32::from_le_bytes(heap_end_buf)
                } else { 0 };
                let heap_limit = if uc.mem_read(0x800_1548, &mut heap_limit_buf).is_ok() {
                    u32::from_le_bytes(heap_limit_buf)
                } else { 0 };
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                eprintln!("[sbrk#{n}] incr=0x{r0:X} heap_end=0x{heap_end:08X} limit=0x{heap_limit:08X} lr=0x{lr:08X}");
            }

            // ── _sbrk 返回点1（成功路径 0x08001534） ──
            if addr == 0x0800_1534 {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let mut heap_end_buf = [0u8; 4];
                let heap_end = if uc.mem_read(0x2000_0020, &mut heap_end_buf).is_ok() {
                    u32::from_le_bytes(heap_end_buf)
                } else { 0 };
                eprintln!("[sbrk] SUCCESS return old=0x{r0:08X} new_heap_end=0x{heap_end:08X}");
            }
            // ── _sbrk 返回点2（失败路径 0x08001542） ──
            if addr == 0x0800_1542 {
                eprintln!("[sbrk] FAILED return -1 (ENOMEM)");
            }

            // ── calloc 入口 ──
            if addr == addr_calloc {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                let n = tc.fetch_add(1, Ordering::Relaxed);
                eprintln!("[calloc#{n}] count=0x{r0:X} size=0x{r1:X} total=0x{:X} lr=0x{lr:08X}", r0 * r1);
            }

            // ── _malloc_r 入口 ──
            if addr == addr_malloc_r {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                let n = tm.fetch_add(1, Ordering::Relaxed);
                im.store(1, Ordering::Relaxed);
                eprintln!("[_malloc_r#{n}] size=0x{r1:X} reent=0x{r0:08X} lr=0x{lr:08X}");
            }

            // ── sbrk_aligned 入口 ──
            if addr == ADDR_SBRK_ALIGNED {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                let n = sac.fetch_add(1, Ordering::Relaxed);
                // 读取缓存的堆指针
                let mut cache_buf = [0u8; 4];
                let cached = if uc.mem_read(0x2000_3248, &mut cache_buf).is_ok() {
                    u32::from_le_bytes(cache_buf)
                } else { 0 };
                eprintln!("[sbrk_aligned#{n}] reent=0x{r0:08X} size=0x{r1:X} cached=0x{cached:08X}");
            }

            // ── sbrk_aligned 内第一次 _sbrk_r 调用（获取初始指针） ──
            if addr == ADDR_SBRK_ALIGNED_CALL1 {
                let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                eprintln!("[sbrk_aligned] call#1 _sbrk_r(r1=0x{r1:X}) ← 获取初始堆指针");
            }

            // ── sbrk_aligned 内第二次 _sbrk_r 调用（实际分配） ──
            if addr == ADDR_SBRK_ALIGNED_CALL2 {
                let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                let n = sisa.fetch_add(1, Ordering::Relaxed);
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                eprintln!("[sbrk_aligned] call#2 _sbrk_r#{n}(r0=0x{r0:08X} r1=0x{r1:X}) ← 实际分配");
            }

            // ── sbrk_aligned 内 _sbrk_r 返回后检查 ──
            if addr == ADDR_SBRK_ALIGNED_RET_CHECK {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r3 = uc.reg_read(RegisterARM::R3).unwrap();
                // r3 = r0 + 1, 如果 r0 == -1 则 r3 == 0 (Z=1)
                let is_err = r3 == 0;
                if is_err {
                    eprintln!("[sbrk_aligned] _sbrk_r returned -1 (ENOMEM)!");
                } else {
                    eprintln!("[sbrk_aligned] _sbrk_r returned 0x{r0:08X} (OK)");
                }
            }

            // ── _malloc_r 扩展块路径：计算 r9 = r4 + r3 ──
            if addr == ADDR_MALLOC_R_EXTEND_CHECK {
                let r4 = uc.reg_read(RegisterARM::R4).unwrap();
                let r3 = uc.reg_read(RegisterARM::R3).unwrap();
                let r9 = r4 + r3;
                eprintln!("[_malloc_r] extend-block: r4(block)=0x{r4:08X} r3(size)=0x{r3:X} r9(expected)=0x{r9:08X}");
            }

            // ── _malloc_r 扩展块 _sbrk_r 调用 ──
            if addr == ADDR_MALLOC_R_EXTEND_CALL {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                eprintln!("[_malloc_r] extend-block call _sbrk_r(reent=0x{r0:08X} incr=0x{r1:X})");
            }

            // ── _malloc_r 扩展块 cmp r9, r0 ──
            if addr == ADDR_MALLOC_R_EXTEND_CMP {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r9 = uc.reg_read(RegisterARM::R9).unwrap();
                let ok = r9 == r0;
                eprintln!("[_malloc_r] extend-block cmp r9=0x{r9:08X} vs r0=0x{r0:08X} → {}", if ok { "OK" } else { "MISMATCH!" });
                if !ok {
                    // 读取更多上下文
                    let r4 = uc.reg_read(RegisterARM::R4).unwrap();
                    let r5 = uc.reg_read(RegisterARM::R5).unwrap();
                    let r6 = uc.reg_read(RegisterARM::R6).unwrap();
                    let r7 = uc.reg_read(RegisterARM::R7).unwrap();
                    let sp = uc.reg_read(RegisterARM::SP).unwrap();
                    eprintln!("[_malloc_r]   r4=0x{r4:08X} r5=0x{r5:08X} r6=0x{r6:08X} r7=0x{r7:08X} sp=0x{sp:08X}");
                    // 读取 r4(block) 内容
                    let mut block_buf = [0u8; 8];
                    if uc.mem_read(r4, &mut block_buf).is_ok() {
                        let sz = u32::from_le_bytes([block_buf[0], block_buf[1], block_buf[2], block_buf[3]]);
                        let next = u32::from_le_bytes([block_buf[4], block_buf[5], block_buf[6], block_buf[7]]);
                        eprintln!("[_malloc_r]   block[0]=size=0x{sz:X} block[4]=next=0x{next:08X}");
                    }
                }
            }

            // ── _malloc_r 错误返回路径 ──
            if addr == ADDR_MALLOC_R_ERROR {
                eprintln!("[_malloc_r] → ERROR (errno=12 ENOMEM), returning NULL");
                im.store(0, Ordering::Relaxed);
            }

            // ── _malloc_r 返回（通过 ldmia.w pc 弹出） ──
            // 检测 _malloc_r 的返回指令: 0x0801113A
            if addr == 0x0801_113A {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                eprintln!("[_malloc_r] RETURN r0=0x{r0:08X} {}",
                    if r0 == 0 { "(NULL)" } else { "" });
                im.store(0, Ordering::Relaxed);
            }

            // ── _sbrk_r 入口 ──
            if addr == addr_sbrk_r && im.load(Ordering::Relaxed) == 1 {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                eprintln!("[_sbrk_r] reent=0x{r0:08X} incr=0x{r1:X} lr=0x{lr:08X}");
            }
        })
        .unwrap();

    let _ = m.run(2_000_000);
    let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
    eprintln!("[trace] done PC=0x{pc:08X} total_sbrk={} total_calloc={} total_malloc_r={}",
        total_sbrk.load(Ordering::Relaxed),
        total_calloc.load(Ordering::Relaxed),
        total_malloc_r.load(Ordering::Relaxed));
}

/// 在 _malloc_r 函数范围内逐指令打印，重点验证 b.n 0xE7AF 跳转目标。
/// 每次到达 b.n 指令时读取原始字节编码、计算预期目标、核对实际跳转地址。
#[test]
fn m_jos_bn_instruction_trace() {
    let mut m = load_jos();

    // _malloc_r 函数范围 (从 objdump 反汇编获取)
    const MALLOC_R_START: u64 = 0x0801_10D0;
    const MALLOC_R_END: u64 = 0x0801_1218;
    // b.n 0xE7AF 所在地址
    const ADDR_BN_E7AF: u64 = 0x0801_11D8;
    // 预期跳转目标
    const EXPECTED_TARGET: u64 = 0x0801_113A;

    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;

    let insn_count = Arc::new(AtomicU64::new(0));
    let prev_pc = Arc::new(AtomicU64::new(0));
    let in_malloc_range = Arc::new(AtomicU64::new(0));
    let bn_bug_total = Arc::new(AtomicU64::new(0));

    let ic = insn_count.clone();
    let pp = prev_pc.clone();
    let imr = in_malloc_range.clone();
    let bbt = bn_bug_total.clone();

    m.cpu
        .add_code_hook(1, 0, move |uc, addr, _size| {
            let in_range = addr >= MALLOC_R_START && addr < MALLOC_R_END;
            if in_range {
                imr.store(1, Ordering::Relaxed);
            }

            // ── 检测 b.n 0xE7AF 跳转 ──
            // 前一条指令的 PC 是 0x080111D8（b.n 指令），当前 addr 是跳转后的目标
            let prev = pp.load(Ordering::Relaxed);
            if prev == ADDR_BN_E7AF {
                // 读取原始指令字节（在 b.n 指令地址处）
                let mut raw = [0u8; 2];
                if uc.mem_read(ADDR_BN_E7AF, &mut raw).is_ok() {
                    let encoding = u16::from_le_bytes(raw);
                    let imm11 = (encoding & 0x07FF) as i16;
                    // 符号扩展 11 位
                    let signed_offset = if imm11 & 0x0400 != 0 {
                        ((imm11 as u16) | 0xF800u16) as i16 as i32
                    } else {
                        imm11 as i32
                    };
                    let computed_target = (ADDR_BN_E7AF as i32 + 4 + signed_offset * 2) as u64;
                    let correct = addr == EXPECTED_TARGET;
                    let match_computed = addr == computed_target;

                    let n = ic.fetch_add(1, Ordering::Relaxed);
                    if !correct {
                        let bug_n = bbt.fetch_add(1, Ordering::Relaxed);
                        eprintln!(
                            "!!! B.N BUG #{bug_n} at insn#{n}: \
                            b.n @0x{ADDR_BN_E7AF:08X} raw=0x{encoding:04X} \
                            imm11=0x{imm11:03X} signed_off={signed_offset} \
                            computed=0x{computed_target:08X} expected=0x{EXPECTED_TARGET:08X} \
                            actual=0x{addr:08X} correct={correct} match_computed={match_computed} !!!"
                        );
                    } else if n <= 10 {
                        eprintln!(
                            "[b.n OK #{n}] @0x{ADDR_BN_E7AF:08X} raw=0x{encoding:04X} \
                            imm11=0x{imm11:03X} -> actual=0x{addr:08X} (correct)"
                        );
                    }
                }
            }

            // ── 在 _malloc_r 范围内，打印前 200 条指令 ──
            if in_range && ic.load(Ordering::Relaxed) < 200 {
                let n = ic.fetch_add(1, Ordering::Relaxed);
                // 读取原始指令字节
                let mut raw = [0u8; 4];
                let sz = if uc.mem_read(addr, &mut raw).is_ok() {
                    let hw = u16::from_le_bytes([raw[0], raw[1]]);
                    // 判断是否为 32 位 Thumb 指令
                    if (hw & 0xF800) >= 0xE800 { 4 } else { 2 }
                } else { 2 };
                let encoding = if sz == 2 {
                    u16::from_le_bytes([raw[0], raw[1]]) as u32
                } else {
                    u32::from_le_bytes(raw)
                };

                // 判断是否为 b.n 指令（Thumb-16: 11100xxx xxxxxxxx）
                let is_bn = (encoding & 0xF800) == 0xE000;
                let tag = if addr == ADDR_BN_E7AF { " *** B.N 0xE7AF ***" } else { "" };

                if is_bn {
                    let imm11 = (encoding & 0x07FF) as i16;
                    let signed_offset = if imm11 & 0x0400 != 0 {
                        ((imm11 as u16) | 0xF800u16) as i16 as i32
                    } else {
                        imm11 as i32
                    };
                    let target = (addr as i32 + 4 + signed_offset * 2) as u64;
                    eprintln!(
                        "[insn#{n}] 0x{addr:08X} | 0x{encoding:04X} | b.n -> 0x{target:08X}{tag}"
                    );
                } else {
                    eprintln!(
                        "[insn#{n}] 0x{addr:08X} | 0x{encoding:08X} ({sz}B){tag}"
                    );
                }
            } else if in_range {
                ic.fetch_add(1, Ordering::Relaxed);
            }

            pp.store(addr, Ordering::Relaxed);
        })
        .unwrap();

    let _ = m.run(2_000_000);
    let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
    eprintln!(
        "[bn_trace] done PC=0x{pc:08X} total_insns={} bn_bugs={}",
        insn_count.load(Ordering::Relaxed),
        bn_bug_total.load(Ordering::Relaxed),
    );
}

/// 使用 release 固件的符号地址，追踪执行流并检测死循环位置。
/// 关键地址来自 build_rel 的 nm 输出。
#[test]
fn m_jos_release_flow_trace() {
    let mut m = load_jos();

    // === release build 关键符号地址 ===
    const ADDR_MAIN: u64 = 0x0800_07B8;
    const ADDR_SYSTEM_EARLY_INIT: u64 = 0x0800_14C4;
    const ADDR_BOARD_INIT: u64 = 0x0800_9B1C;
    const ADDR_BOARD_TICK_INIT: u64 = 0x0800_9AC4;
    const ADDR_CLOCK_HAL_CONFIGURE: u64 = 0x0800_9E84;
    const ADDR_RTOS_INIT: u64 = 0x0800_EBF8;
    const ADDR_RTOS_START: u64 = 0x0800_F48C;
    const ADDR_UART_HAL_INIT: u64 = 0x0800_9F10;
    const ADDR_UART_HAL_PUTC: u64 = 0x0800_9FA0;
    const ADDR_UART_CONSOLE_PUTC: u64 = 0x0800_3C38;
    const ADDR_APP_MAIN_TASK: u64 = 0x0800_13A4;
    const ADDR_HARD_FAULT: u64 = 0x0800_02C0;
    const ADDR_BUS_FAULT: u64 = 0x0800_02E8;
    const ADDR_USAGE_FAULT: u64 = 0x0800_02D4;
    const ADDR_MEM_MANAGE: u64 = 0x0800_02AC;
    const ADDR_DMA_REMAINING: u64 = 0x0800_8F88;
    const ADDR_DMA_HAL_REMAINING: u64 = 0x0800_C874;

    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;

    let last_pc = Arc::new(AtomicU64::new(0));
    let same_pc_count = Arc::new(AtomicU64::new(0));
    let lp = last_pc.clone();
    let spc = same_pc_count.clone();

    m.cpu
        .add_code_hook(1, 0, move |uc, addr, _size| {
            // 检测死循环：同一个 PC 出现超过 100 次
            let prev = lp.swap(addr, Ordering::Relaxed);
            if prev == addr {
                let c = spc.fetch_add(1, Ordering::Relaxed);
                if c == 100 {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                    let r2 = uc.reg_read(RegisterARM::R2).unwrap();
                    let sp = uc.reg_read(RegisterARM::SP).unwrap();
                    let lr = uc.reg_read(RegisterARM::LR).unwrap();
                    eprintln!(
                        "!!! 检测到死循环: PC=0x{addr:08X} (出现 {c}+ 次) r0=0x{r0:08X} r1=0x{r1:08X} r2=0x{r2:08X} sp=0x{sp:08X} lr=0x{lr:08X}"
                    );
                    // 读取死循环附近的指令（前后 16 字节）
                    let mut inst_buf = [0u8; 32];
                    if uc.mem_read(addr & !1, &mut inst_buf).is_ok() {
                        eprintln!("  附近指令: {:02X?}", &inst_buf);
                    }
                    // 读取 RCC CR 寄存器
                    let mut rcc_buf = [0u8; 4];
                    if uc.mem_read(0x4002_3800, &mut rcc_buf).is_ok() {
                        let cr = u32::from_le_bytes(rcc_buf);
                        eprintln!("  RCC_CR = 0x{cr:08X} (HSEON={} HSERDY={} PLLON={} PLLRDY={})",
                            cr & (1<<16) != 0, cr & (1<<17) != 0, cr & (1<<24) != 0, cr & (1<<25) != 0);
                    }
                    // 读取 USART1 SR
                    let mut usart_buf = [0u8; 4];
                    if uc.mem_read(0x4001_1000, &mut usart_buf).is_ok() {
                        let sr = u32::from_le_bytes(usart_buf);
                        eprintln!("  USART1_SR = 0x{sr:08X} (TXE={} TC={} RXNE={})",
                            sr & (1<<7) != 0, sr & (1<<6) != 0, sr & (1<<5) != 0);
                    }
                }
            } else {
                spc.store(0, Ordering::Relaxed);
            }

            // 追踪关键函数入口
            match addr {
                ADDR_MAIN => eprintln!("[flow] -> main"),
                ADDR_SYSTEM_EARLY_INIT => eprintln!("[flow] -> system_early_init"),
                ADDR_BOARD_INIT => eprintln!("[flow] -> board_init"),
                ADDR_BOARD_TICK_INIT => eprintln!("[flow] -> board_tick_init"),
                ADDR_CLOCK_HAL_CONFIGURE => eprintln!("[flow] -> clock_hal_configure"),
                ADDR_RTOS_INIT => eprintln!("[flow] -> rtos_init"),
                ADDR_RTOS_START => eprintln!("[flow] -> rtos_start"),
                ADDR_UART_HAL_INIT => eprintln!("[flow] -> uart_hal_init"),
                ADDR_UART_HAL_PUTC => {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                    eprintln!("[flow] -> uart_hal_putc r0(hal)=0x{r0:08X} r1(c)=0x{r1:02X}('{}')",
                        (r1 as u8) as char);
                }
                ADDR_UART_CONSOLE_PUTC => {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    eprintln!("[flow] -> uart_console_putc r0(c)=0x{r0:02X}('{}')",
                        (r0 as u8) as char);
                }
                ADDR_APP_MAIN_TASK => eprintln!("[flow] -> app_main_task"),
                ADDR_HARD_FAULT => {
                    eprintln!("!!! HARD FAULT !!!");
                    dump_fault_regs(uc);
                }
                ADDR_BUS_FAULT => {
                    eprintln!("!!! BUS FAULT !!!");
                    dump_fault_regs(uc);
                }
                ADDR_USAGE_FAULT => {
                    eprintln!("!!! USAGE FAULT !!!");
                    dump_fault_regs(uc);
                }
                ADDR_MEM_MANAGE => {
                    eprintln!("!!! MEM MANAGE FAULT !!!");
                    dump_fault_regs(uc);
                }
                _ => {}
            }
        })
        .unwrap();

    fn dump_fault_regs(uc: &unicorn_engine::Unicorn<'_, ()>) {
        for (nm, reg) in [
            ("R0", RegisterARM::R0), ("R1", RegisterARM::R1), ("R2", RegisterARM::R2),
            ("R3", RegisterARM::R3), ("R12", RegisterARM::R12),
            ("SP", RegisterARM::SP), ("LR", RegisterARM::LR), ("PC", RegisterARM::PC),
            ("MSP", RegisterARM::MSP), ("PSP", RegisterARM::PSP),
        ] {
            if let Ok(v) = uc.reg_read(reg) {
                eprintln!("  {nm} = 0x{v:08X}");
            }
        }
        // 读取 fault 状态寄存器
        for (addr, name) in [
            (0xE000_ED28u64, "CFSR"), (0xE000_ED2C, "HFSR"),
            (0xE000_ED34, "MMFAR"), (0xE000_ED38, "BFAR"),
        ] {
            let mut buf = [0u8; 4];
            if uc.mem_read(addr, &mut buf).is_ok() {
                let v = u32::from_le_bytes(buf);
                if v != 0 {
                    eprintln!("  {name} = 0x{v:08X}");
                }
            }
        }
    }

    // 分步运行，观察执行流
    for step in 0..500 {
        let pc_before = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let r = m.run(50_000);
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let out = m.console.lock().unwrap().output().to_vec();

        if !out.is_empty() {
            eprintln!("[step {step}] 控制台输出 ({}B): {:?}", out.len(), String::from_utf8_lossy(&out));
        }

        if let Err(ref e) = r {
            let reason = m.nvic.lock().unwrap().take_stop_reason();
            let r0 = m.cpu.reg_read_u32(RegisterARM::R0).unwrap_or(0);
            let r1 = m.cpu.reg_read_u32(RegisterARM::R1).unwrap_or(0);
            let r5 = m.cpu.reg_read_u32(RegisterARM::R5).unwrap_or(0);
            let sp = m.cpu.reg_read_u32(RegisterARM::SP).unwrap_or(0);
            let lr = m.cpu.reg_read_u32(RegisterARM::LR).unwrap_or(0);
            eprintln!("[step {step}] pc=0x{pc_before:08X}->0x{pc:08X} ERROR={e:?} reason={reason:?}");
            eprintln!("  寄存器: r0=0x{r0:08X} r1=0x{r1:08X} r5=0x{r5:08X} sp=0x{sp:08X} lr=0x{lr:08X}");
            // 读取 fault 状态寄存器
            for (addr, name) in [
                (0xE000_ED28u64, "CFSR"), (0xE000_ED2C, "HFSR"),
                (0xE000_ED34, "MMFAR"), (0xE000_ED38, "BFAR"),
            ] {
                if let Ok(buf) = m.cpu.mem_read(addr, 4) {
                    let v = u32::from_le_bytes(buf.try_into().unwrap());
                    if v != 0 {
                        eprintln!("  {name} = 0x{v:08X}");
                    }
                }
            }
            break;
        }

        if pc == pc_before {
            eprintln!("[step {step}] pc=0x{pc:08X} 未前进（可能死循环）");
            // 再给一次机会
            let r2 = m.run(100_000);
            let pc2 = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
            if pc2 == pc && r2.is_err() {
                eprintln!("[step {step}] 确认停机");
                break;
            }
        }

        if step % 50 == 49 {
            eprintln!("[step {step}] pc=0x{pc:08X} ... 继续运行中");
        }
    }

    let out = m.console.lock().unwrap().output().to_vec();
    let text = String::from_utf8_lossy(&out);
    println!("=== 最终控制台输出 ({}B) ===\n{text}", out.len());
}

/// 逐指令追踪 _sbrk_r 函数，打印每一步的 r0/r1/LR，找出 _sbrk 返回 0x20000220 后
/// _sbrk_r 却返回 -1 的根因。
#[test]
fn m_jos_trace_sbrk_r_flow() {
    let mut m = load_jos();

    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;

    // _sbrk_r 函数地址范围
    const SBRK_R_START: u64 = 0x0801_1928;
    const SBRK_R_END: u64 = 0x0801_1948;

    let insn_count = Arc::new(AtomicU64::new(0));
    let prev_pc = Arc::new(AtomicU64::new(0));
    let depth = Arc::new(AtomicU64::new(0)); // 递归深度

    let ic = insn_count.clone();
    let pp = prev_pc.clone();
    let dp = depth.clone();

    m.cpu
        .add_code_hook(1, 0, move |uc, addr, _size| {
            if addr == SBRK_R_START {
                let d = dp.fetch_add(1, Ordering::Relaxed);
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                let sp = uc.reg_read(RegisterARM::SP).unwrap();
                eprintln!(">>> _sbrk_r ENTRY depth={d} r0=0x{r0:08X} r1=0x{r1:08X} lr=0x{lr:08X} sp=0x{sp:08X}");
            }

            if addr >= SBRK_R_START && addr < SBRK_R_END {
                let n = ic.fetch_add(1, Ordering::Relaxed);
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                let sp = uc.reg_read(RegisterARM::SP).unwrap();

                // 读取指令字节
                let mut raw = [0u8; 4];
                let sz = if uc.mem_read(addr, &mut raw).is_ok() {
                    let hw = u16::from_le_bytes([raw[0], raw[1]]);
                    if (hw & 0xF800) >= 0xE800 { 4 } else { 2 }
                } else { 2 };

                let is_bl = (sz == 4) && (raw[0] & 0xF0) == 0xF0 && (raw[1] & 0xF0) == 0xF0
                    && (raw[2] & 0xF0) == 0xF0 && (raw[3] & 0xF0) == 0xF0;
                let is_pop_pc = (sz == 2) && (raw[0] & 0xFE) == 0xBC && (raw[1] & 0x01) == 0x01;

                let tag = if is_bl { " <-- BL" } else if is_pop_pc { " <-- POP{PC}" } else { "" };

                eprintln!(
                    "[_sbrk_r insn#{n}] 0x{addr:08X} | r0=0x{r0:08X} r1=0x{r1:08X} lr=0x{lr:08X} sp=0x{sp:08X}{tag}"
                );

                if is_pop_pc {
                    dp.fetch_sub(1, Ordering::Relaxed);
                    eprintln!("<<< _sbrk_r EXIT r0=0x{r0:08X} (pop pc)");
                }
            }

            // 追踪 _sbrk 内部的返回
            if addr == 0x0800_1534 {
                let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                eprintln!("    [_sbrk SUCCESS] r0=0x{r0:08X}");
            }
            if addr == 0x0800_1542 {
                eprintln!("    [_sbrk FAIL] ENOMEM");
            }

            pp.store(addr, Ordering::Relaxed);
        })
        .unwrap();

    // 加一个钩子在 sbrk_aligned 的返回点检查 r0
    m.cpu
        .add_code_hook(0x0801_10EE, 0x0801_10EE, move |uc, addr, _size| {
            let r0 = uc.reg_read(RegisterARM::R0).unwrap();
            let r3 = uc.reg_read(RegisterARM::R3).unwrap();
            let sp = uc.reg_read(RegisterARM::SP).unwrap();
            eprintln!("!!! [sbrk_aligned check] 0x{addr:08X} r0=0x{r0:08X} r3=0x{r3:08X} sp=0x{sp:08X} (r3==0 → ENOMEM)");
        })
        .unwrap();

    let _ = m.run(1_000_000);
    let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
    eprintln!(
        "[sbrk_r_trace] done PC=0x{pc:08X} total_insns_in_sbrk_r={}",
        insn_count.load(Ordering::Relaxed)
    );
}
