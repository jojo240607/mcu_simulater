//! 第一次 ready_add 双挂出现时，把 g_sched_bad_tcb(其 sched_next/prev !=0)的字段、
//! g_sleep_head、以及被错误残留的链归属 dump 出来，判断它是"误以为在睡眠链"还是
//! "重复挂在就绪链"。
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;

const SYS: &str = r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf";
const APP: &str = r"/home/ubuntu/work/joc-drvtest-app/app.bin";
const CCM_POOL_LO: u32 = 0x1000_6000;
const CCM_POOL_HI: u32 = 0x1000_8b00;

fn rd(m: &mut Machine, addr: u64, n: usize) -> Vec<u8> {
    match m.cpu.mem_read(addr, n) {
        Ok(b) => b,
        Err(_) => Vec::new(),
    }
}
fn u32at(b: &[u8], o: usize) -> u32 {
    let mut a = [0u8; 4];
    a.copy_from_slice(&b[o..o + 4]);
    u32::from_le_bytes(a)
}
fn cstr(b: &[u8]) -> String {
    let e = b.iter().position(|&x| x == 0).unwrap_or(b.len());
    b[..e].iter().map(|&c| c as char).collect()
}

// 在 task pool 的可打印区里找该任务名（TA 名字在 TCB 开头某个字串区）。宽搜名字区。
fn pool_name(m: &mut Machine, ptr: u32) -> String {
    // 直接读 ptr 附近 96B 找 ASCII 名段（Task 结构前部一般存 name）
    for off in [0usize, 16, 32, 48] {
        let b = rd(m, ptr as u64 + off as u64, 16);
        let s = b
            .iter()
            .take_while(|&&c| (c as char).is_ascii_graphic() || c == b' ')
            .copied()
            .collect::<Vec<u8>>();
        let sstr = String::from_utf8_lossy(&s).into_owned();
        if sstr.len() >= 3 {
            return sstr;
        }
    }
    String::new()
}
fn inpool(p: u32) -> bool {
    p >= CCM_POOL_LO && p < CCM_POOL_HI
}

#[test]
fn first_double_add_snapshot() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&Path::new(SYS)).unwrap();
    m.load_app_partition(&Path::new(APP)).unwrap();
    m.reset().unwrap();

    // 每次进入 rtos_sched_assert_fail(0x800e644) 置 marks，供判定"首次"位置
    let af = Arc::new(AtomicU32::new(0));
    let af2 = af.clone();
    m.cpu
        .raw()
        .add_code_hook(0x0800_e644, 0x0800_e644, move |_, _a, _s| {
            af2.fetch_add(1, Ordering::Relaxed);
        })
        .unwrap();
    let _ = m.cpu.raw().ctl_flush_tb();

    // 捕获对 main TCB(0x100060A4) sched_next/prev(+28/+32→0x100060C0/0x100060C4) 的每次写
    // 者 PC 与写入值——判定谁把"自身地址"写进双字段。
    let wp = Arc::new(AtomicU32::new(0));
    let wn = Arc::new(AtomicU32::new(0));
    let wv = Arc::new(AtomicU32::new(0));
    let (p2, n2, v2) = (wp.clone(), wn.clone(), wv.clone());
    m.cpu
        .raw()
        .add_mem_hook(
            unicorn_engine::HookType::MEM_WRITE,
            0x1000_60A4 + 28,
            0x1000_60A4 + 32 + 4,
            move |uc, _ty, _a, _s, val| {
                let pc = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
                p2.store(pc, Ordering::Relaxed);
                v2.store(val as u32, Ordering::Relaxed);
                n2.fetch_add(1, Ordering::Relaxed);
                false
            },
        )
        .unwrap();

    let t0 = std::time::Instant::now();
    for step in 0..120u32 {
        if t0.elapsed().as_secs() > 140 {
            eprintln!(">>> 超时(assert 未提前出现)");
            break;
        }
        let _ = m.run(400_000);
        if af.load(Ordering::Relaxed) > 0 && step < 2 {
            let wpc = wp.load(Ordering::Relaxed);
            let wv2 = wv.load(Ordering::Relaxed);
            eprintln!(
                ">> 最近写 main TCB sched_next/prev: pc=0x{wpc:08X} val=0x{wv2:08X} total_writes={}",
                wn.load(Ordering::Relaxed)
            );
            // 取 assert 现场：g_sched_bad_tcb 首见于 0x10005f1c；g_running/g_sleep_head
            let bad_tcb = u32at(&rd(&mut m, 0x1000_5f1c, 4), 0);
            let g_running = u32at(&rd(&mut m, 0x1000_6040, 4), 0);
            let sleep_head = u32at(&rd(&mut m, 0x1000_5f28, 4), 0);
            // 重点任务即 bad_tcb
            let t = if inpool(bad_tcb) { bad_tcb } else if inpool(g_running) { g_running } else { bad_tcb };
            eprintln!("\n== first double-add snapshot (step {step}) assert_count={} ==", af.load(Ordering::Relaxed));
            eprintln!("g_running=0x{g_running:08X} bad_tcb=0x{bad_tcb:08X} sleep_head=0x{sleep_head:08X}");
            let raws = rd(&mut m, t as u64, 64);
            eprintln!(
                "task@{t:08X} raw[0..64] = {}",
                raws.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().chunks(1).map(|c| format!("{}", c[0])).collect::<String>()
            );
            // name 区（若前 12B ASCII 可读）
            let nameb = rd(&mut m, t as u64, 24);
            eprintln!("name-region(24B): {:?}", String::from_utf8_lossy(&nameb).into_owned());
            // 关键字段：sched_next=+28, sched_prev=+32, prio 近 +8, state 近 +11
            let nxt = u32at(&raws.clone(), 28);
            let prv = u32at(&raws, 32);
            let raw_upto = rd(&mut m, t as u64 + 6, 8); // +6..+13 覆盖 prio(应该+8)/state(≈+11)
            eprintln!(
                "field prio/state bytes@+6..+13 = {}",
                raw_upto.iter().map(|b| format!("{b:02X}")).collect::<String>()
            );
            eprintln!(
                "sched_next=0x{nxt:08X} (inpool={}) sched_prev=0x{prv:08X} (inpool={})",
                inpool(nxt),
                inpool(prv)
            );
            // 判断 next 指向谁：若可达另一个 task，读其 name
            if inpool(nxt) {
                eprintln!("  -> next 是另一 TCB，其 name-region≈ {:?}", pool_name(&mut m, nxt));
            }
            // 是否在睡眠链上：从 sleep_head 走看是否含 t
            {
                let mut cur = sleep_head;
                let mut found = false;
                for _ in 0..8 {
                    if cur == 0 || !inpool(cur) {
                        break;
                    }
                    if cur == bad_tcb {
                        found = true;
                        break;
                    }
                    let b = rd(&mut m, cur as u64 + 28, 4);
                    if b.len() != 4 {
                        break;
                    }
                    cur = u32at(&b, 0);
                }
                eprintln!("  bad_tcb 在睡眠链(g_sleep_head)上？ {found}");
            }
            break;
        }
    }
    let out = {
        let outv = m.console.lock().unwrap().output().to_vec();
        String::from_utf8_lossy(&outv).into_owned()
    };
    println!("=== console ({}B) tail ===\n{}", out.len(), &out[out.len().saturating_sub(700)..]);
}
