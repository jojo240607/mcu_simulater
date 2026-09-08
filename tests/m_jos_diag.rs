//! 临时探测：诊断 jOS board_init 阶段 _sbrk 洪泛 / 堆耗尽问题。完成后删除。
//!
//! 关键符号地址（取自 build-sim ELF）：
//!   _sbrk                  = 0x0800_151C
//!   _sbrk_r (newlib 包装)  = 0x0801_1928 .. 0x0801_1948 (bl _sbrk 后返回地址 0x08011939)
//!   board_init             = 0x0800_9B1C
//!   device_manager_register= 0x0800_9938
//!   uart_create            = 0x0800_3BB4

use std::io::Write;
use std::path::Path;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;

use unicorn_engine::RegisterARM;

use mcu_simulater::machine::Machine;

const JOS_ELF: &str = r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf";
const OUT: &str = r"/home/ubuntu/work/joc-base/build_rel/jos_diag_out.txt";

const ADDR_SBRK: u64 = 0x0800_151C; // _sbrk 本体入口: R0 = incr
const ADDR_SBRK_FAIL: u64 = 0x0800_1542; // _sbrk 失败返回点 (R0=0xFFFFFFFF)
const ADDR_SBRK_SUCCESS: u64 = 0x0800_1532; // _sbrk 成功推进堆指针 (str r1,[r2,#0]) 指令执行前：R1=新堆顶
const G_SYS_HEAP: u32 = 0x2000_0220; // build_rel 符号 g_sys_heap（堆基址）
const PTR_SBRK_BREAK: u32 = 0x2000_0020; // _sbrk 静态 heap 指针变量地址（读此值-基址=真实堆消耗）
const ADDR_BOARD_INIT: u64 = 0x0800_9B1C;
const ADDR_REGISTER: u64 = 0x0800_9938;
const ADDR_UART_CREATE: u64 = 0x0800_3BB4;
// malloc/calloc/free 包装器 + 各 create 入口（取自 build_rel ELF 符号表）
const ADDR_MALLOC: u64 = 0x0801_10B4;
const ADDR_CALLOC: u64 = 0x0801_107C;
const ADDR_FREE: u64 = 0x0801_10C4;
const ADDR_MALLOC_R: u64 = 0x0801_1118; // _malloc_r: 直接调用者可能绕过公共包装器
const ADDR_CALLOC_R: u64 = 0x0801_108C; // _calloc_r
const ADDR_SBRK_ALIGNED: u64 = 0x0801_10D4; // jOS 自定义对齐分配器 sbrk_aligned
const ADDR_MAIN: u64 = 0x0800_07B8;
const ADDR_PINMUX_CREATE: u64 = 0x0800_404C;
const ADDR_CLOCK_CREATE: u64 = 0x0800_2DC0;
const ADDR_SYSTICK_CREATE: u64 = 0x0800_4144;
const ADDR_GPIO_PIN_CREATE: u64 = 0x0800_2C68;
// uart_create 内部路径（反汇编 0x08003BB4）：
//   0x08003C18 = pop 返回点（R0=r4=返回值）
//   0x08003C20 = calloc(1,72) 失败路径（返回 NULL）
//   0x08003C1A = uart_hal_create 失败路径（free + 返回 NULL）
const ADDR_UART_CREATE_RET: u64 = 0x0800_3C18;
const ADDR_UART_CALLOC_FAIL: u64 = 0x0800_3C20;
const ADDR_UART_HAL_FAIL: u64 = 0x0800_3C1A;
// board_init 循环内每次 create 返回后的判定点（cbz r0, skip_register），R0 = dev
const ADDR_BOARD_INIT_CBZ: u64 = 0x0800_9B2C;

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
fn diag_sbrk_flood() {
    let mut m = load_jos();
    let f = Arc::new(std::sync::Mutex::new(std::fs::File::create(OUT).unwrap()));

    // 计数
    let n_sbrk = Arc::new(AtomicU32::new(0));
    let sum_incr = Arc::new(AtomicU64::new(0)); // 请求总量（含失败的调用）
    let n_ok = Arc::new(AtomicU32::new(0)); // _sbrk 成功推进次数
    let sum_ok = Arc::new(AtomicU64::new(0)); // 成功推进的累计字节（真实堆消耗）
    let last_incr = Arc::new(AtomicU64::new(0)); // 最近一次 _sbrk 的 incr
    let peak_break = Arc::new(AtomicU64::new(0)); // 观测到的最高堆顶
    let n_neg1 = Arc::new(AtomicU32::new(0)); // sbrk 返回 -1 的次数
    let n_register = Arc::new(AtomicU32::new(0));
    let n_uart_create = Arc::new(AtomicU32::new(0));
    // uart_create 结果细分 + board_init 循环 create 结果
    let n_uart_ok = Arc::new(AtomicU32::new(0));
    let n_uart_null = Arc::new(AtomicU32::new(0));
    let n_calloc_fail = Arc::new(AtomicU32::new(0));
    let n_hal_fail = Arc::new(AtomicU32::new(0));
    let n_create_ok = Arc::new(AtomicU32::new(0));
    let n_create_fail = Arc::new(AtomicU32::new(0));

    // malloc/calloc 调用点(LR) -> (次数, 累计请求字节)；free 调用点(LR) -> 次数
    let alloc_sites = Arc::new(std::sync::Mutex::new(std::collections::BTreeMap::<u64, (u32, u64)>::new()));
    let free_sites = Arc::new(std::sync::Mutex::new(std::collections::BTreeMap::<u64, u32>::new()));
    let n_free = Arc::new(AtomicU32::new(0));
    // malloc / calloc 包装器被调用次数（分别统计）
    let n_malloc = Arc::new(AtomicU32::new(0));
    let n_calloc = Arc::new(AtomicU32::new(0));
    // sbrk_aligned 入口的调用者(LR) -> (次数, R1=请求字节累计)
    let sa_sites = Arc::new(std::sync::Mutex::new(std::collections::BTreeMap::<u64, (u32, u64)>::new()));
    // sbrk_aligned 两个调用点计数（区分 _malloc_r 内 0x11152 / 0x11184）
    let n_sa_1152 = Arc::new(AtomicU32::new(0));
    let n_sa_1184 = Arc::new(AtomicU32::new(0));
    // sbrk_aligned 入口 [SP+28] = _malloc_r 用 stmdb {r3-r9,lr} 保存的 LR
    // （= malloc/calloc 尾调用后保留的固件真实调用点）-> (次数, 累计请求字节)
    let sa_origin = Arc::new(std::sync::Mutex::new(std::collections::BTreeMap::<u64, (u32, u64)>::new()));
    // 逐条记录 (序号, 请求字节, origin LR)，用于定位 88 次 0x50 洪泛来源
    let sa_seq = Arc::new(std::sync::Mutex::new(Vec::<(u64, u64, u64)>::new()));

    // 直接调用 _malloc_r/_calloc_r 的调用点(LR) -> (次数, 累计请求字节)
    let direct_sites = Arc::new(std::sync::Mutex::new(std::collections::BTreeMap::<u64, (u32, u64)>::new()));
    // _malloc_r / _calloc_r 入口被进入的总次数（独立计数，与 sbrk_aligned 对比）
    let n_malloc_r = Arc::new(AtomicU32::new(0));
    let n_calloc_r = Arc::new(AtomicU32::new(0));
    // 独立最小钩子计数（最小闭包，排除 direct_sites 闭包可能的干扰）
    let n_malloc_r_v = Arc::new(AtomicU32::new(0));
    let n_114c_v = Arc::new(AtomicU32::new(0));
    let n_1152_v = Arc::new(AtomicU32::new(0));
    // _malloc_r 内部细粒度路径追踪
    let n_114a_v = Arc::new(AtomicU32::new(0)); // 0x1114a: mov r4, r3（在0x1114c之前）
    let n_114e_v = Arc::new(AtomicU32::new(0)); // 0x1114e: mov r1, r5（0x1114c之后，0x11152之前）
    let n_11a0_v = Arc::new(AtomicU32::new(0)); // 0x111a0: 空闲链表处理入口（r4 != 0 时跳入）
    let n_11ec_v = Arc::new(AtomicU32::new(0)); // 0x111ec: 循环回边 b.n 0x1114c
    let n_115c_v = Arc::new(AtomicU32::new(0)); // 0x1115c: sbrk_aligned 失败后重载空闲链表
    let n_113a_v = Arc::new(AtomicU32::new(0)); // 0x1113a: _malloc_r 返回点
    let n_11c0_v = Arc::new(AtomicU32::new(0)); // 0x111c0: 成功路径 __malloc_unlock
    let n_1202_v = Arc::new(AtomicU32::new(0)); // 0x11202: 失败返回点
    let n_11c6_v = Arc::new(AtomicU32::new(0)); // 0x111c6: 成功路径 __malloc_unlock 返回后
    let n_11d8_v = Arc::new(AtomicU32::new(0)); // 0x111d8: 成功路径 b.n 0x1113a
    let n_1138_v = Arc::new(AtomicU32::new(0)); // 0x11138: 失败路径 movs r0,#0 (在0x1113a之前)
    let n_120c_v = Arc::new(AtomicU32::new(0)); // 0x1120c: 失败路径 b.n 0x11138
    // _calloc_r 内部：调用 _malloc_r 的 bl 指令地址
    let n_1096_v = Arc::new(AtomicU32::new(0)); // 0x11096: _calloc_r 内 bl _malloc_r
    // 关键追踪点：验证是否有隐藏入口跳到 0x1113e 或 0x11132
    let n_113e_v = Arc::new(AtomicU32::new(0)); // 0x1113e: ldr.w r8（主流程起点，紧接 0x1113a 返回之后）
    let n_1132_v = Arc::new(AtomicU32::new(0)); // 0x11132: bls.n 0x1113e（入口校验通过后跳转）
    let n_1142_v = Arc::new(AtomicU32::new(0)); // 0x11142: bl __malloc_lock
    let n_1146_v = Arc::new(AtomicU32::new(0)); // 0x11146: ldr.w r3, [r8]（lock 返回后）
    // 完整命中矩阵：0x11118-0x1113f 每条指令（含省略的中间地址）
    let n_1111c_v = Arc::new(AtomicU32::new(0)); // 0x1111c: adds r5, r1, #3
    let n_1111e_v = Arc::new(AtomicU32::new(0)); // 0x1111e: bic.w r5, r5, #3
    let n_11122_v = Arc::new(AtomicU32::new(0)); // 0x11122: adds r5, #8
    let n_11124_v = Arc::new(AtomicU32::new(0)); // 0x11124: cmp r5, #12
    let n_11128_v = Arc::new(AtomicU32::new(0)); // 0x11128: movcc r5, #12
    let n_1112a_v = Arc::new(AtomicU32::new(0)); // 0x1112a: cmp r5, #0
    let n_1112c_v = Arc::new(AtomicU32::new(0)); // 0x1112c: mov r6, r0
    let n_1112e_v = Arc::new(AtomicU32::new(0)); // 0x1112e: blt.n 0x11134
    let n_11130_v = Arc::new(AtomicU32::new(0)); // 0x11130: cmp r1, r5 （关键！如果命中>57说明有隐藏入口）
    let n_11134_v = Arc::new(AtomicU32::new(0)); // 0x11134: movs r3, #12
    let n_11136_v = Arc::new(AtomicU32::new(0)); // 0x11136: str r3, [r6, #0]
    // 探测 Unicorn 是否在 ldmia.w(0x1113a-0x1113d) 中间误触发钩子
    let n_1113c_v = Arc::new(AtomicU32::new(0)); // 0x1113c: ldmia.w 中间字节
    let n_1113d_v = Arc::new(AtomicU32::new(0)); // 0x1113d: ldmia.w 中间字节
    // 记录 0x1113e 处的 LR 分布，用于追踪隐藏入口来源
    let lr_at_113e = Arc::new(std::sync::Mutex::new(std::collections::BTreeMap::<u64, u32>::new()));
    // 记录 0x1113a (ldmia.w) 处栈顶的 PC 值（即返回目标地址），用于追踪是否弹栈回 0x1113e
    let ret_target_at_113a = Arc::new(std::sync::Mutex::new(std::collections::BTreeMap::<u64, u32>::new()));
    // 转移追踪：从 0x111D8 跳转到 0x1113A 还是 0x1113E
    let last_was_11d8 = Arc::new(AtomicU32::new(0)); // 0=无, 1=刚经过0x111D8
    let n_11d8_to_113a = Arc::new(AtomicU32::new(0)); // 0x111D8→0x1113A 次数
    let n_11d8_to_113e = Arc::new(AtomicU32::new(0)); // 0x111D8→0x1113E 次数

    // board_init 循环内每个设备 create 完成时的 sbrk 序号序列
    let cbz_sbrk_idx = Arc::new(std::sync::Mutex::new(Vec::<(u64, u64)>::new())); // (cbz执行序, sbrk序号)

    // 各 create 入口处的 sbrk 快照（调用序号 + 累计字节），用于判断 88 次 0x50 分配发生在哪个阶段
    let snapshots = Arc::new(std::sync::Mutex::new(Vec::<(u64, u64, u64)>::new())); // (addr, sbrk_idx, sum_incr)

    // 记录每个 mallocr 调用点(LR)的出现次数
    let sites = Arc::new(std::sync::Mutex::new(std::collections::BTreeMap::<u64, u32>::new()));
    // 逐条记录 (序号, incr, caller LR)，用于分析首失败前分配序列
    let calls = Arc::new(std::sync::Mutex::new(Vec::<(u64, u64, u64)>::new())); // (idx, incr, alloc_caller)
    // 分配请求者(_sbrk_r 的调用者 LR, 即 malloc 内部 bl _sbrk_r 的返回地址) -> (次数, 累计incr)
    let alloc_callers = Arc::new(std::sync::Mutex::new(std::collections::BTreeMap::<u64, (u32, u64)>::new()));
    let fail_idx = Arc::new(std::sync::Mutex::new(None::<usize>));

    {
        let n_sbrk = n_sbrk.clone();
        let sum_incr = sum_incr.clone();
        let last_incr = last_incr.clone();
        let sites = sites.clone();
        let calls = calls.clone();
        let alloc_callers = alloc_callers.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                // 0x0800151C = _sbrk 本体入口：R0 = incr，LR = _sbrk_r 内返回点(0x08011939)
                if addr == ADDR_SBRK {
                    let incr = uc.reg_read(RegisterARM::R0).unwrap();
                    let lr = uc.reg_read(RegisterARM::LR).unwrap();
                    let idx = n_sbrk.fetch_add(1, Ordering::Relaxed);
                    sum_incr.fetch_add(incr as u64, Ordering::Relaxed); // 请求总量（含失败）
                    last_incr.store(incr as u64, Ordering::Relaxed);
                    // 栈回溯：code hook 在指令执行前触发，故进入 _sbrk 时 SP 仍是
                    // _sbrk_r 的 SP(其 push{r3,r4,r5,lr} 已完成)：SP+0=r3, SP+4=r4, SP+8=r5,
                    // SP+12=LR(=malloc 内 bl _sbrk_r 的返回点，即真正的分配请求者)。
                    let mut alloc_caller = 0u64;
                    if let Ok(sp) = uc.reg_read(RegisterARM::SP) {
                        let mut b = [0u8; 4];
                        if uc.mem_read(sp + 12, &mut b).is_ok() {
                            alloc_caller = u32::from_le_bytes(b) as u64;
                        }
                    }
                    let mut g = sites.lock().unwrap();
                    *g.entry(lr).or_insert(0) += 1;
                    drop(g);
                    if alloc_caller != 0 {
                        let mut g = alloc_callers.lock().unwrap();
                        let e = g.entry(alloc_caller).or_insert((0, 0));
                        e.0 += 1;
                        e.1 += incr as u64;
                    }
                    calls.lock().unwrap().push((idx as u64, incr as u64, alloc_caller));
                }
            })
            .unwrap();
    }
    {
        // 0x08001532 = _sbrk 成功路径 `str r1,[r2,#0]`（推进堆指针）指令执行前：
        // R1 = 新堆顶。此指令只在 incr<=0 或 (heap+incr)<=limit 时执行，故命中即成功。
        let n_ok = n_ok.clone();
        let sum_ok = sum_ok.clone();
        let last_incr = last_incr.clone();
        let peak_break = peak_break.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                if addr == ADDR_SBRK_SUCCESS {
                    n_ok.fetch_add(1, Ordering::Relaxed);
                    sum_ok.fetch_add(last_incr.load(Ordering::Relaxed), Ordering::Relaxed);
                    if let Ok(r1) = uc.reg_read(RegisterARM::R1) {
                        peak_break.fetch_max(r1, Ordering::Relaxed);
                    }
                }
            })
            .unwrap();
    }
    {
        let n_neg1 = n_neg1.clone();
        let fail_idx = fail_idx.clone();
        let calls = calls.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                // 0x08001542 = _sbrk 失败返回点（R0 被置 0xFFFFFFFF）
                if addr == ADDR_SBRK_FAIL {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    if r0 == u64::from(u32::MAX) {
                        let n = n_neg1.fetch_add(1, Ordering::Relaxed);
                        if n == 0 {
                            // 记录首次失败对应的 sbrk 调用序号（即当前调用数-1）
                            let cur = calls.lock().unwrap().len();
                            *fail_idx.lock().unwrap() = Some(cur);
                        }
                    }
                }
            })
            .unwrap();
    }
    {
        let n_register = n_register.clone();
        let f = f.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                if addr == ADDR_REGISTER {
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    let r1 = uc.reg_read(RegisterARM::R1).unwrap();
                    n_register.fetch_add(1, Ordering::Relaxed);
                    let mut name = String::new();
                    let mut a = r0;
                    for _ in 0..32 {
                        let mut b = [0u8; 1];
                        if uc.mem_read(a, &mut b).is_ok() && b[0] != 0 {
                            name.push(b[0] as char);
                            a += 1;
                        } else {
                            break;
                        }
                    }
                    writeln!(f.lock().unwrap(), "[register] \"{}\" dev=0x{r1:08X}", name).ok();
                }
            })
            .unwrap();
    }
    {
        let n_uart_create = n_uart_create.clone();
        let n_uart_ok = n_uart_ok.clone();
        let n_uart_null = n_uart_null.clone();
        let n_calloc_fail = n_calloc_fail.clone();
        let n_hal_fail = n_hal_fail.clone();
        let f = f.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                if addr == ADDR_UART_CREATE {
                    n_uart_create.fetch_add(1, Ordering::Relaxed);
                } else if addr == ADDR_UART_CREATE_RET {
                    // 返回点：R0 = 返回值（r4）。区分 成功 / 返回NULL
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    if r0 == 0 {
                        let n = n_uart_null.fetch_add(1, Ordering::Relaxed);
                        if n == 0 {
                            let lr = uc.reg_read(RegisterARM::LR).unwrap();
                            writeln!(f.lock().unwrap(),
                                     "[uart_create] 第{}次返回 NULL (lr=0x{:08X})", n_uart_create.load(Ordering::Relaxed), lr).ok();
                        }
                    } else {
                        n_uart_ok.fetch_add(1, Ordering::Relaxed);
                    }
                } else if addr == ADDR_UART_CALLOC_FAIL {
                    n_calloc_fail.fetch_add(1, Ordering::Relaxed);
                } else if addr == ADDR_UART_HAL_FAIL {
                    n_hal_fail.fetch_add(1, Ordering::Relaxed);
                }
            })
            .unwrap();
    }
    {
        let n_create_ok = n_create_ok.clone();
        let n_create_fail = n_create_fail.clone();
        let cbz_sbrk_idx = cbz_sbrk_idx.clone();
        let n_sbrk = n_sbrk.clone();
        let n_cbzx = Arc::new(AtomicU32::new(0));
        let n_cbzx = n_cbzx.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                // board_init 循环内 create 返回后的判定点（cbz r0）：R0 = dev
                if addr == ADDR_BOARD_INIT_CBZ {
                    let idx = n_cbzx.fetch_add(1, Ordering::Relaxed) as u64;
                    let r0 = uc.reg_read(RegisterARM::R0).unwrap();
                    if r0 == 0 {
                        n_create_fail.fetch_add(1, Ordering::Relaxed);
                    } else {
                        n_create_ok.fetch_add(1, Ordering::Relaxed);
                    }
                    cbz_sbrk_idx.lock().unwrap().push((idx, n_sbrk.load(Ordering::Relaxed) as u64));
                }
            })
            .unwrap();
    }
    {
        // malloc/calloc 包装器入口：LR = 固件真实调用点（bl 之后的返回地址），R0 = 请求字节数
        let alloc_sites = alloc_sites.clone();
        let n_malloc = n_malloc.clone();
        let n_calloc = n_calloc.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                let (is_malloc, size) = if addr == ADDR_MALLOC {
                    (true, uc.reg_read(RegisterARM::R0).unwrap())
                } else if addr == ADDR_CALLOC {
                    let nm = uc.reg_read(RegisterARM::R0).unwrap();
                    let sz = uc.reg_read(RegisterARM::R1).unwrap();
                    (false, nm.wrapping_mul(sz))
                } else {
                    return;
                };
                if is_malloc {
                    n_malloc.fetch_add(1, Ordering::Relaxed);
                } else {
                    n_calloc.fetch_add(1, Ordering::Relaxed);
                }
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                let mut g = alloc_sites.lock().unwrap();
                let e = g.entry(lr).or_insert((0, 0));
                e.0 += 1;
                e.1 += size;
            })
            .unwrap();
    }
    {
        // sbrk_aligned (0x080110D4) 入口：LR = 调用者，R1 = 请求字节。
        // sbrk_aligned 仅被 _malloc_r 内部两处调用(0x8011152/0x8011184)，
        // _malloc_r prologue 为 stmdb {r3-r9,lr}(32字节)，body 不改 SP，
        // 故 [SP+28] = _malloc_r 保存的 LR = 固件真实的 malloc/calloc 调用点。
        let sa_sites = sa_sites.clone();
        let sa_origin = sa_origin.clone();
        let sa_seq = sa_seq.clone();
        let sa_idx = Arc::new(AtomicU64::new(0));
        let n_sa_1152 = n_sa_1152.clone();
        let n_sa_1184 = n_sa_1184.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                if addr == ADDR_SBRK_ALIGNED {
                    // 记录从哪个调用点进来：LR=返回地址，0x8011157=第一处, 0x8011185=第二处
                    let caller = uc.reg_read(RegisterARM::LR).unwrap();
                    if caller == 0x0801_1157 {
                        n_sa_1152.fetch_add(1, Ordering::Relaxed);
                    } else if caller == 0x0801_1185 {
                        n_sa_1184.fetch_add(1, Ordering::Relaxed);
                    }
                    let size = uc.reg_read(RegisterARM::R1).unwrap();
                    let lr = uc.reg_read(RegisterARM::LR).unwrap();
                    let idx = sa_idx.fetch_add(1, Ordering::Relaxed);

                    // 栈回溯：SP+28 = _malloc_r 保存的 LR（最终请求者）
                    let mut origin_lr = 0u64;
                    if let Ok(sp) = uc.reg_read(RegisterARM::SP) {
                        let mut b = [0u8; 4];
                        if uc.mem_read(sp + 28, &mut b).is_ok() {
                            origin_lr = u32::from_le_bytes(b) as u64;
                        }
                    }

                    // 直接调用者（_malloc_r 内两处 bl 的返回地址）
                    let mut g = sa_sites.lock().unwrap();
                    let e = g.entry(lr).or_insert((0, 0));
                    e.0 += 1;
                    e.1 += size;
                    drop(g);

                    // 最终请求者（malloc/calloc 的固件真实调用点）
                    if origin_lr != 0 {
                        let mut g = sa_origin.lock().unwrap();
                        let e = g.entry(origin_lr).or_insert((0, 0));
                        e.0 += 1;
                        e.1 += size;
                        drop(g);
                    }

                    sa_seq.lock().unwrap().push((idx, size, origin_lr));
                }
            })
            .unwrap();
    }
    {
        // 直接调用 _malloc_r/_calloc_r 的调用点(LR)：进入时 R0=reent, R1=size(_malloc_r)；
        // _calloc_r: R0=reent, R1=nm, R2=sz
        let direct_sites = direct_sites.clone();
        let n_malloc_r = n_malloc_r.clone();
        let n_calloc_r = n_calloc_r.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                let size = if addr == ADDR_MALLOC_R {
                    n_malloc_r.fetch_add(1, Ordering::Relaxed);
                    uc.reg_read(RegisterARM::R1).unwrap()
                } else if addr == ADDR_CALLOC_R {
                    n_calloc_r.fetch_add(1, Ordering::Relaxed);
                    let nm = uc.reg_read(RegisterARM::R1).unwrap();
                    let sz = uc.reg_read(RegisterARM::R2).unwrap();
                    nm.wrapping_mul(sz)
                } else {
                    return;
                };
                let lr = uc.reg_read(RegisterARM::LR).unwrap();
                let mut g = direct_sites.lock().unwrap();
                let e = g.entry(lr).or_insert((0, 0));
                e.0 += 1;
                e.1 += size;
            })
            .unwrap();
    }
    {
        // 独立验证钩子注册（在 run 之前注册，运行期间累计计数，结束后统一打印）
        // 最小闭包排除 direct_sites 闭包可能存在的问题
        let n_malloc_r_v = n_malloc_r_v.clone();
        let n_114c_v = n_114c_v.clone();
        let n_1152_v = n_1152_v.clone();
        let n_114a_v = n_114a_v.clone();
        let n_114e_v = n_114e_v.clone();
        let n_11a0_v = n_11a0_v.clone();
        let n_11ec_v = n_11ec_v.clone();
        let n_115c_v = n_115c_v.clone();
        let n_113a_v = n_113a_v.clone();
        let n_11c0_v = n_11c0_v.clone();
        let n_1202_v = n_1202_v.clone();
        let n_11c6_v = n_11c6_v.clone();
        let n_11d8_v = n_11d8_v.clone();
        let n_1138_v = n_1138_v.clone();
        let n_120c_v = n_120c_v.clone();
        let n_1096_v = n_1096_v.clone();
        let n_113e_v = n_113e_v.clone();
        let n_1132_v = n_1132_v.clone();
        let n_1142_v = n_1142_v.clone();
        let n_1146_v = n_1146_v.clone();
        let n_1111c_v = n_1111c_v.clone();
        let n_1111e_v = n_1111e_v.clone();
        let n_11122_v = n_11122_v.clone();
        let n_11124_v = n_11124_v.clone();
        let n_11128_v = n_11128_v.clone();
        let n_1112a_v = n_1112a_v.clone();
        let n_1112c_v = n_1112c_v.clone();
        let n_1112e_v = n_1112e_v.clone();
        let n_11130_v = n_11130_v.clone();
        let n_11134_v = n_11134_v.clone();
        let n_11136_v = n_11136_v.clone();
        let n_1113c_v = n_1113c_v.clone();
        let n_1113d_v = n_1113d_v.clone();
        let lr_at_113e = lr_at_113e.clone();
        let ret_target_at_113a = ret_target_at_113a.clone();
        let last_was_11d8 = last_was_11d8.clone();
        let n_11d8_to_113a = n_11d8_to_113a.clone();
        let n_11d8_to_113e = n_11d8_to_113e.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                match addr {
                    0x0801_1118 => { n_malloc_r_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_111c => { n_1111c_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_111e => { n_1111e_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1122 => { n_11122_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1124 => { n_11124_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1128 => { n_11128_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_112a => { n_1112a_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_112c => { n_1112c_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_112e => { n_1112e_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1130 => { n_11130_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1132 => { n_1132_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1134 => { n_11134_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1136 => { n_11136_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_113c => { n_1113c_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_113d => { n_1113d_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_113e => {
                        n_113e_v.fetch_add(1, Ordering::Relaxed);
                        // 转移追踪：如果上一条指令是 0x111D8(b.n 0x1113a)，则实际跳到了 0x1113E
                        if last_was_11d8.load(Ordering::Relaxed) == 1 {
                            n_11d8_to_113e.fetch_add(1, Ordering::Relaxed);
                            last_was_11d8.store(0, Ordering::Relaxed);
                        }
                        if let Ok(lr) = uc.reg_read(RegisterARM::LR) {
                            *lr_at_113e.lock().unwrap().entry(lr as u64).or_insert(0) += 1;
                        }
                    }
                    0x0801_1142 => { n_1142_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1146 => { n_1146_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_114a => { n_114a_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_114c => { n_114c_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_114e => { n_114e_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1152 => { n_1152_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_11a0 => { n_11a0_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_11ec => { n_11ec_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_115c => { n_115c_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_113a => {
                        n_113a_v.fetch_add(1, Ordering::Relaxed);
                        // 转移追踪：如果上一条指令是 0x111D8，则 b.n 正确跳到了 0x1113A
                        if last_was_11d8.load(Ordering::Relaxed) == 1 {
                            n_11d8_to_113a.fetch_add(1, Ordering::Relaxed);
                            last_was_11d8.store(0, Ordering::Relaxed);
                        }
                        // 读取栈顶 ldmia.w 弹给 PC 的值（SP+28 是保存的 LR）
                        if let Ok(sp) = uc.reg_read(RegisterARM::SP) {
                            let mut b = [0u8; 4];
                            if uc.mem_read(sp + 28, &mut b).is_ok() {
                                let target = u32::from_le_bytes(b) as u64;
                                *ret_target_at_113a.lock().unwrap().entry(target).or_insert(0) += 1;
                            }
                        }
                    }
                    0x0801_11c0 => { n_11c0_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1202 => { n_1202_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_11c6 => { n_11c6_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_11d8 => {
                        n_11d8_v.fetch_add(1, Ordering::Relaxed);
                        // 标记：刚经过 b.n 0x1113a，下一条指令应到 0x1113A
                        last_was_11d8.store(1, Ordering::Relaxed);
                    }
                    0x0801_1138 => { n_1138_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_120c => { n_120c_v.fetch_add(1, Ordering::Relaxed); }
                    0x0801_1096 => { n_1096_v.fetch_add(1, Ordering::Relaxed); }
                    _ => {}
                }
            })
            .unwrap();
    }
    {
        // free 调用点(LR) -> 次数
        let free_sites = free_sites.clone();
        let n_free = n_free.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                if addr == ADDR_FREE {
                    n_free.fetch_add(1, Ordering::Relaxed);
                    let lr = uc.reg_read(RegisterARM::LR).unwrap();
                    let mut g = free_sites.lock().unwrap();
                    *g.entry(lr).or_insert(0) += 1;
                }
            })
            .unwrap();
    }
    {
        // main / board_init / 各 create 入口：记录当时的 sbrk 状态快照
        let snapshots = snapshots.clone();
        let n_sbrk = n_sbrk.clone();
        let sum_incr = sum_incr.clone();
        m.cpu
            .add_code_hook(1, 0, move |uc, addr, _size| {
                if matches!(addr, ADDR_MAIN | ADDR_BOARD_INIT | ADDR_PINMUX_CREATE | ADDR_CLOCK_CREATE | ADDR_SYSTICK_CREATE | ADDR_GPIO_PIN_CREATE) {
                    snapshots.lock().unwrap().push((
                        addr,
                        n_sbrk.load(Ordering::Relaxed) as u64,
                        sum_incr.load(Ordering::Relaxed),
                    ));
                }
            })
            .unwrap();
    }

    writeln!(f.lock().unwrap(), "[diag] board_init 前 g_sys_heap? 直接跑。").ok();

    // 分小步运行，直至出错或 8M 指令
    let mut budget = 0u64;
    let mut err = None;
    let mut pc_last = 0u64;
    let mut stall = 0u32;
    for step in 0..40 {
        let r = m.run(200_000);
        budget += 200_000;
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap() as u64;
        if pc == pc_last {
            stall += 1;
        } else {
            stall = 0;
            pc_last = pc;
        }
        if r.is_err() {
            err = Some(format!("{:?}", r.as_ref().unwrap_err()));
            writeln!(f.lock().unwrap(), "[run] step={step} budget={budget} ERR={:?} pc=0x{pc:08X}", r.as_ref().unwrap_err()).ok();
            break;
        }
        if stall >= 8 {
            writeln!(f.lock().unwrap(), "[run] step={step} budget={budget} PC 停滞 pc=0x{pc:08X}").ok();
            break;
        }
        if step % 8 == 7 {
            writeln!(f.lock().unwrap(), "[run] step={step} budget={budget} pc=0x{pc:08X} sbrk={} incr_sum={} neg1={} reg={} uart_create={}",
                     n_sbrk.load(Ordering::Relaxed),
                     sum_incr.load(Ordering::Relaxed),
                     n_neg1.load(Ordering::Relaxed),
                     n_register.load(Ordering::Relaxed),
                     n_uart_create.load(Ordering::Relaxed)).ok();
        }
    }

    let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
    writeln!(f.lock().unwrap(), "\n[result] 结束 pc=0x{pc:08X} err={:?} budget={}", err, budget).ok();

    // 独立最小钩子与主钩子的计数对照，验证 _malloc_r 入口计数是否漏数
    writeln!(f.lock().unwrap(), "\n[verify] 独立最小钩子: _malloc_r入口(0x11118)={} 0x1132(bls.n)={} 0x113e(ldr.w)={} 0x1142(bl_lock)={} 0x1146(ldr_r3)={}", 
             n_malloc_r_v.load(Ordering::Relaxed),
             n_1132_v.load(Ordering::Relaxed),
             n_113e_v.load(Ordering::Relaxed),
             n_1142_v.load(Ordering::Relaxed),
             n_1146_v.load(Ordering::Relaxed)).ok();
    // 完整命中矩阵：0x11118-0x1113f 每条指令
    writeln!(f.lock().unwrap(), "[verify] 完整命中矩阵 0x11118-0x1113f:").ok();
    writeln!(f.lock().unwrap(), "  0x11118(entry)={} 0x1111c(adds)={} 0x1111e(bic.w)={} 0x11122(adds#8)={}",
             n_malloc_r_v.load(Ordering::Relaxed),
             n_1111c_v.load(Ordering::Relaxed),
             n_1111e_v.load(Ordering::Relaxed),
             n_11122_v.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "  0x11124(cmp#12)={} 0x11128(movcc)={} 0x1112a(cmp#0)={} 0x1112c(mov_r6)={}",
             n_11124_v.load(Ordering::Relaxed),
             n_11128_v.load(Ordering::Relaxed),
             n_1112a_v.load(Ordering::Relaxed),
             n_1112c_v.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "  0x1112e(blt.n)={} 0x11130(cmp_r1_r5)={} 0x11132(bls.n)={} 0x11134(movs#12)={}",
             n_1112e_v.load(Ordering::Relaxed),
             n_11130_v.load(Ordering::Relaxed),
             n_1132_v.load(Ordering::Relaxed),
             n_11134_v.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "  0x11136(str)={} 0x1138(movs#0)={} 0x113a(return)={} 0x113e(ldr.w)={}",
             n_11136_v.load(Ordering::Relaxed),
             n_1138_v.load(Ordering::Relaxed),
             n_113a_v.load(Ordering::Relaxed),
             n_113e_v.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "[verify] 0x1113c(mid_ldmia)={} 0x1113d(mid_ldmia)={} (非零=Unicorn钩子异常)",
             n_1113c_v.load(Ordering::Relaxed),
             n_1113d_v.load(Ordering::Relaxed)).ok();
    {
        let lr_map = lr_at_113e.lock().unwrap();
        writeln!(f.lock().unwrap(), "[verify] 0x1113e 处 LR 分布 (共{}次):", lr_map.values().sum::<u32>()).ok();
        // 按次数降序排列
        let mut lr_vec: Vec<_> = lr_map.iter().collect();
        lr_vec.sort_by(|a, b| b.1.cmp(a.1));
        for (lr, cnt) in lr_vec.iter().take(10) {
            writeln!(f.lock().unwrap(), "  LR=0x{:08X} 次数={}", lr, cnt).ok();
        }
    }
    {
        let ret_map = ret_target_at_113a.lock().unwrap();
        writeln!(f.lock().unwrap(), "[verify] 0x1113a ldmia.w 弹栈目标(PC)分布 (共{}次):", ret_map.values().sum::<u32>()).ok();
        let mut ret_vec: Vec<_> = ret_map.iter().collect();
        ret_vec.sort_by(|a, b| b.1.cmp(a.1));
        for (target, cnt) in ret_vec.iter().take(10) {
            writeln!(f.lock().unwrap(), "  弹栈PC=0x{:08X} 次数={}", target, cnt).ok();
        }
    }
    writeln!(f.lock().unwrap(), "[verify] _malloc_r 续: 0x114a(mov)={} 0x114c(cbnz)={} 0x114e(mov)={} 0x1152(sbrk_aligned)={}", 
             n_114a_v.load(Ordering::Relaxed),
             n_114c_v.load(Ordering::Relaxed),
             n_114e_v.load(Ordering::Relaxed),
             n_1152_v.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "[verify] _malloc_r 内部路径追踪:").ok();
    writeln!(f.lock().unwrap(), "  0x111a0(空闲链表处理)={} 0x111ec(循环回边)={} 0x115c(失败后重载)={}",
             n_11a0_v.load(Ordering::Relaxed),
             n_11ec_v.load(Ordering::Relaxed),
             n_115c_v.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "  0x111c0(成功unlock)={} 0x111c6(解锁返回后)={} 0x111d8(b.n返回)={} 0x113a(lr返回)={}",
             n_11c0_v.load(Ordering::Relaxed),
             n_11c6_v.load(Ordering::Relaxed),
             n_11d8_v.load(Ordering::Relaxed),
             n_113a_v.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "  0x11202(失败入口)={} 0x1120c(失败b.n 0x11138)={} 0x1138(失败mov r0,#0)={}",
             n_1202_v.load(Ordering::Relaxed),
             n_120c_v.load(Ordering::Relaxed),
             n_1138_v.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "  0x11096(_calloc_r内bl_malloc_r)={}",
             n_1096_v.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "[verify] 主钩子对照: _malloc_r入口={} _calloc_r入口={} sa_1152={} sa_1184={} sa_合计={}",
             n_malloc_r.load(Ordering::Relaxed),
             n_calloc_r.load(Ordering::Relaxed),
             n_sa_1152.load(Ordering::Relaxed),
             n_sa_1184.load(Ordering::Relaxed),
             n_sa_1152.load(Ordering::Relaxed) + n_sa_1184.load(Ordering::Relaxed)).ok();

    // 直接读 _sbrk 静态堆指针（0x20000020）：真实堆消耗铁证，不受钩子计数影响
    let heap_break = if let Ok(b) = m.cpu.mem_read(PTR_SBRK_BREAK as u64, 4) {
        u32::from_le_bytes(b.try_into().unwrap())
    } else {
        0
    };
    let consumed = heap_break.wrapping_sub(G_SYS_HEAP);
    let peak = peak_break.load(Ordering::Relaxed).wrapping_sub(G_SYS_HEAP as u64) as u32;

    writeln!(f.lock().unwrap(), "[result] sbrk: 总调用={} 请求总量(含失败)=0x{:X} 成功次数={} 成功累计(真实消耗)=0x{:X} 失败次数={}",
             n_sbrk.load(Ordering::Relaxed),
             sum_incr.load(Ordering::Relaxed),
             n_ok.load(Ordering::Relaxed),
             sum_ok.load(Ordering::Relaxed),
             n_neg1.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "[result] sbrk_aligned 两调用点: 0x11152={} 0x11184={} (合计={})",
             n_sa_1152.load(Ordering::Relaxed),
             n_sa_1184.load(Ordering::Relaxed),
             n_sa_1152.load(Ordering::Relaxed) + n_sa_1184.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "[result] 真实堆消耗(读内存0x20000020): 堆顶=0x{heap_break:08X} - g_sys_heap=0x{G_SYS_HEAP:08X} = 0x{consumed:X} ({consumed}字节/8192={:.2}%)",
             consumed as f64 / 8192.0 * 100.0).ok();
    writeln!(f.lock().unwrap(), "[result] 峰值堆消耗(钩子观测最高堆顶R1) = 0x{peak:X} ({peak} 字节)").ok();
    writeln!(f.lock().unwrap(), "[result] register次数={} uart_create次数={}",
             n_register.load(Ordering::Relaxed),
             n_uart_create.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "[result] uart_create: 成功={} 返回NULL={} (其中 calloc失败={} hal失败={})",
             n_uart_ok.load(Ordering::Relaxed),
             n_uart_null.load(Ordering::Relaxed),
             n_calloc_fail.load(Ordering::Relaxed),
             n_hal_fail.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "[result] board_init 循环: create成功={} create失败={} register={}",
             n_create_ok.load(Ordering::Relaxed),
             n_create_fail.load(Ordering::Relaxed),
             n_register.load(Ordering::Relaxed)).ok();
    writeln!(f.lock().unwrap(), "[result] free 调用次数={}", n_free.load(Ordering::Relaxed)).ok();

    writeln!(f.lock().unwrap(), "\n[result] 阶段快照 (addr, sbrk调用数, 累计字节):").ok();
    {
        let ss = snapshots.lock().unwrap();
        for (addr, idx, sum) in ss.iter() {
            let name = match *addr {
                a if a == ADDR_MAIN => "main".to_string(),
                a if a == ADDR_BOARD_INIT => "board_init".to_string(),
                a if a == ADDR_PINMUX_CREATE => "pinmux_create".to_string(),
                a if a == ADDR_CLOCK_CREATE => "clock_create".to_string(),
                a if a == ADDR_SYSTICK_CREATE => "systick_create".to_string(),
                a if a == ADDR_GPIO_PIN_CREATE => "gpio_pin_create".to_string(),
                _ => format!("0x{addr:08X}"),
            };
            writeln!(f.lock().unwrap(), "  {name:<16} sbrk={idx} sum=0x{sum:X} ({sum})").ok();
        }
    }

    writeln!(f.lock().unwrap(), "\n[result] malloc/calloc 调用点(LR) -> (次数, 累计请求字节):").ok();
    writeln!(f.lock().unwrap(), "  (malloc 包装器调用 {} 次, calloc 包装器调用 {} 次)",
             n_malloc.load(Ordering::Relaxed), n_calloc.load(Ordering::Relaxed)).ok();
    {
        let g = alloc_sites.lock().unwrap();
        for (lr, (cnt, sz)) in g.iter() {
            writeln!(f.lock().unwrap(), "  lr=0x{lr:08X} : {cnt} 次, 累计 0x{sz:X} ({sz})").ok();
        }
    }
    writeln!(f.lock().unwrap(), "\n[result] sbrk_aligned 入口调用者(LR) -> (次数, R1=请求字节累计):").ok();
    writeln!(f.lock().unwrap(), "  (_malloc_r 入口进入 {} 次, _calloc_r 入口进入 {} 次)",
             n_malloc_r.load(Ordering::Relaxed), n_calloc_r.load(Ordering::Relaxed)).ok();
    {
        let g = sa_sites.lock().unwrap();
        for (lr, (cnt, sz)) in g.iter() {
            writeln!(f.lock().unwrap(), "  lr=0x{lr:08X} : {cnt} 次, 累计 0x{sz:X} ({sz})").ok();
        }
    }
    writeln!(f.lock().unwrap(), "\n[result] 直接调用 _malloc_r/_calloc_r 的调用点(LR) -> (次数, 累计请求字节):").ok();
    {
        let g = direct_sites.lock().unwrap();
        for (lr, (cnt, sz)) in g.iter() {
            writeln!(f.lock().unwrap(), "  lr=0x{lr:08X} : {cnt} 次, 累计 0x{sz:X} ({sz})").ok();
        }
    }
    writeln!(f.lock().unwrap(), "\n[result] sbrk_aligned 最终请求者(malloc/calloc 固件真实调用点 LR) -> (次数, 累计请求字节):").ok();
    {
        let g = sa_origin.lock().unwrap();
        for (lr, (cnt, sz)) in g.iter() {
            writeln!(f.lock().unwrap(), "  origin=0x{lr:08X} : {cnt} 次, 累计 0x{sz:X} ({sz})").ok();
        }
    }
    writeln!(f.lock().unwrap(), "\n[result] sbrk_aligned 调用序列 (序号, 请求字节, origin LR) 前 60 条 + 88次洪泛段:").ok();
    {
        let g = sa_seq.lock().unwrap();
        for (idx, sz, o) in g.iter().take(60) {
            writeln!(f.lock().unwrap(), "  #{idx:3} size=0x{sz:X} ({sz}) origin=0x{o:08X}").ok();
        }
        // 输出连续相同 origin 的长段（首段 origin 重复 >= 8 次的片段）
        if g.len() > 60 {
            writeln!(f.lock().unwrap(), "  ... (共 {} 条)", g.len()).ok();
            let mut start = 0usize;
            while start < g.len() {
                let o = g[start].2;
                let mut end = start;
                while end < g.len() && g[end].2 == o {
                    end += 1;
                }
                if end - start >= 8 {
                    writeln!(f.lock().unwrap(), "  origin=0x{o:08X} : #{start}..#{end} 共 {} 条", end - start).ok();
                }
                start = end;
            }
        }
    }
    writeln!(f.lock().unwrap(), "\n[result] board_init 每设备 create 后的 sbrk 序号 (cbz序, sbrk数):").ok();
    {
        let g = cbz_sbrk_idx.lock().unwrap();
        let mut prev = 0u64;
        for (idx, n) in g.iter() {
            let delta = *n as i64 - prev as i64;
            writeln!(f.lock().unwrap(), "  cbz#{idx:3} sbrk={n:3} (+{delta})").ok();
            prev = *n;
        }
    }
    writeln!(f.lock().unwrap(), "\n[result] free 调用点(LR) -> 次数:").ok();
    {
        let g = free_sites.lock().unwrap();
        for (lr, cnt) in g.iter() {
            writeln!(f.lock().unwrap(), "  lr=0x{lr:08X} : {cnt}").ok();
        }
    }
    writeln!(f.lock().unwrap(), "[result] 寄存器:").ok();
    for (nm, reg) in [
        ("R0", RegisterARM::R0),
        ("R1", RegisterARM::R1),
        ("R4", RegisterARM::R4),
        ("R5", RegisterARM::R5),
        ("SP", RegisterARM::SP),
        ("MSP", RegisterARM::MSP),
        ("PSP", RegisterARM::PSP),
        ("LR", RegisterARM::LR),
        ("PC", RegisterARM::PC),
        ("IPSR", RegisterARM::IPSR),
    ] {
        writeln!(f.lock().unwrap(), "  {nm} = 0x{:08X}", m.cpu.reg_read(reg).unwrap()).ok();
    }
    writeln!(f.lock().unwrap(), "\n[result] mallocr 调用点(LR)分布:").ok();
    let g = sites.lock().unwrap();
    for (lr, cnt) in g.iter() {
        writeln!(f.lock().unwrap(), "  lr=0x{lr:08X} : {cnt}").ok();
    }
    drop(g);

    let fl = *fail_idx.lock().unwrap();
    writeln!(f.lock().unwrap(), "\n[result] 首次 sbrk 失败 = 第 {:?} 次调用", fl).ok();
    writeln!(f.lock().unwrap(), "\n[result] 分配请求者(_sbrk_r 调用者 LR) -> (次数, 累计incr):").ok();
    {
        let g = alloc_callers.lock().unwrap();
        for (lr, (cnt, sz)) in g.iter() {
            writeln!(f.lock().unwrap(), "  lr=0x{lr:08X} : {cnt} 次, 累计 0x{sz:X} ({sz})").ok();
        }
    }
    writeln!(f.lock().unwrap(), "\n[result] 前 40 次 sbrk 调用 (idx, incr, alloc_caller):").ok();
    let cl = calls.lock().unwrap();
    for (idx, incr, ac) in cl.iter().take(40) {
        writeln!(f.lock().unwrap(), "  #{idx:3} incr=0x{incr:X} ({incr}) caller=0x{ac:08X}").ok();
    }
    if let Some(fl) = fl {
        writeln!(f.lock().unwrap(), "[result] 首失败附近 sbrk 调用:").ok();
        let lo = fl.saturating_sub(3);
        let hi = (fl + 2).min(cl.len());
        for (idx, incr, ac) in cl.iter().take(hi).skip(lo) {
            writeln!(f.lock().unwrap(), "  #{idx:3} incr=0x{incr:X} ({incr}) caller=0x{ac:08X}").ok();
        }
    }
    drop(cl);

    // 验证 0x111D8 处的原始指令字节：b.n 0x1113a 的正确编码是 e7af
    // 如果内存中是 e7b1 则跳转目标变为 0x1113e，解释 87 次隐藏入口
    let inst_bytes = m.cpu.mem_read(0x0801_11D8, 2).unwrap_or_default();
    let inst_val = u16::from_le_bytes([inst_bytes[0], inst_bytes[1]]);
    // 手动解码 b.n 的 imm11 并计算目标
    let imm11 = (inst_val & 0x7FF) as i16;
    let imm11_signed = if imm11 & 0x400 != 0 { imm11 | (0xF800u16 as i16) } else { imm11 };
    let target = (0x0801_11DCu64).wrapping_add_signed((imm11_signed as i32 * 2) as i64);
    writeln!(f.lock().unwrap(), "\n[verify] 0x111D8 内存字节: 0x{:02X} {:02X} (指令=0x{:04X}, imm11=0x{:03X} signed={}, 解码目标=0x{:08X})",
             inst_bytes[0], inst_bytes[1], inst_val, inst_val & 0x7FF, imm11_signed, target).ok();
    writeln!(f.lock().unwrap(), "[verify] 期望: 0xE7AF -> 0x1113A | 若为: 0xE7B1 -> 0x1113E").ok();
    // 同时验证 0x11132 (bls.n 0x1113e) 的字节
    let bls_bytes = m.cpu.mem_read(0x0801_1132, 2).unwrap_or_default();
    writeln!(f.lock().unwrap(), "[verify] 0x11132 内存字节: 0x{:02X} {:02X} (bls.n 0x1113e, 期望 0xD904)",
             bls_bytes[0], bls_bytes[1]).ok();

    // 转移追踪结果：0x111D8(b.n 0x1113A) → 下一条指令是 0x1113A 还是 0x1113E？
    let to_113a = n_11d8_to_113a.load(Ordering::Relaxed);
    let to_113e = n_11d8_to_113e.load(Ordering::Relaxed);
    let total_11d8 = n_11d8_v.load(Ordering::Relaxed);
    writeln!(f.lock().unwrap(), "\n[verify] 转移追踪: 0x111D8(b.n 0x1113A) 共执行 {} 次", total_11d8).ok();
    writeln!(f.lock().unwrap(), "[verify]   0x111D8 → 0x1113A(正确) = {} 次", to_113a).ok();
    writeln!(f.lock().unwrap(), "[verify]   0x111D8 → 0x1113E(异常) = {} 次", to_113e).ok();
    writeln!(f.lock().unwrap(), "[verify]   未追踪到(到其他地址) = {} 次", total_11d8.saturating_sub(to_113a + to_113e)).ok();
    if to_113e > 0 {
        writeln!(f.lock().unwrap(), "[verify] >>> 结论: b.n 0x1113A(编码0xE7AF) 实际跳转到 0x1113E，Unicorn 模拟器存在跳转目标计算错误!").ok();
    }
    if to_113a + to_113e == 0 && total_11d8 > 0 {
        writeln!(f.lock().unwrap(), "[verify] >>> 注意: b.n 后未命中 0x1113A 或 0x1113E，可能跳到了其他地址(需进一步排查)").ok();
    }

    // 显式 flush 确保文件完整落盘
    f.lock().unwrap().flush().ok();
    println!("diag 完成 -> {OUT}");
}
