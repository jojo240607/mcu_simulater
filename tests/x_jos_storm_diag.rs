//! 诊断：App 各任务进入 msleep 周期睡眠后、idle 停滞（无心跳/无新日志）：
//! 采样每步被抢占进入的向量分布、被抢 PC、SysTick/PendSV 挂起与异常栈。
//! 用于归因"任务睡下后由 SysTick 周期唤醒推进停滞"是哪个环节断了
//! (SysTick 没触发 / handler 没推进 tick / PendSV 没切回 / 或其它中断霸占)。
use std::path::Path;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;

fn vecname(v: u32) -> String {
    match v {
        3 => "HardFault".into(),
        6 => "UsageFault".into(),
        11 => "SVC".into(),
        14 => "PendSV".into(),
        15 => "SysTick".into(),
        16 => "IRQ0".into(),
        37 => "USART1".into(),
        38 => "USART2".into(),
        40 => "TIM4".into(),
        45 => "DMA2_S1".into(),
        54 => "TIM6".into(),
        56 => "TIM8".into(),
        67 => "OTG_FS".into(),
        63 => "TIM1_UP".into(),
        n => format!("IRQ{}", n - 16),
    }
}

#[test]
fn storm_diag() {
    let elf = Path::new(r"D:\project\mcu\oop\joc-base\build_rel\stm32f407_minimal.elf");
    let app = Path::new(r"D:\project\mcu\oop\joc-app-rust\app.bin");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();

    let t_start = std::time::Instant::now();
    let mut reached_stall = false;
    // 先跑到 App 挂载+首轮完成（进入各任务 sleep 的状态）
    for _ in 0..200u32 {
        if t_start.elapsed().as_secs() > 120 {
            eprintln!(">>> 挂载阶段超时");
            break;
        }
        let r = m.run(200_000);
        let text = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        let mounted = text.contains("RUST app mounted");
        let tasks = text.contains("task started");
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        if r.is_err() {
            eprintln!("ERR {r:?}");
            break;
        }
        if mounted && tasks {
            eprintln!(
                ">>> App 挂载+首轮完成（进入睡眠状态）pc=0x{pc:08X} iters={} el={:?}",
                m.run_iterations(),
                t_start.elapsed()
            );
            reached_stall = true;
            break;
        }
    }

    if !reached_stall {
        eprintln!("未进入停滞态，中止");
        let out = m.console.lock().unwrap().output().to_vec();
        println!("=== console ===\n{}", String::from_utf8_lossy(&out));
        return;
    }

    // 停滞态：再做几次护栏受限的 run，逐步采样向量分布与抢占点
    for i in 0..3 {
        let t0 = std::time::Instant::now();
        let base_iter = m.run_iterations();
        let g_tick_r = |m: &mut Machine| -> u32 {
            if let Ok(b) = m.cpu.mem_read(0x1000_603C, 4) {
                u32::from_le_bytes(b.try_into().unwrap())
            } else {
                u32::MAX
            }
        };
        let tk0 = g_tick_r(&mut m);
        let pc0 = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let r = m.run(400_000);
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let tk1 = g_tick_r(&mut m);
        let iters = m.run_iterations() - base_iter;
        let out_len = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).len()
        };
        let (exc, syspend, extpend) = {
            let n = m.nvic.lock().unwrap();
            (
                n.current_exception(),
                n.is_sys_pending(14) || n.is_sys_pending(15),
                (0..82).any(|q| n.is_pending(q) && n.is_enabled(q)),
            )
        };
        eprintln!(
            "\n[stall-run {i}] pc0=0x{pc0:08X}->0x{pc:08X} g_tick {tk0}->{tk1} r={r:?} iters={iters} exc={exc} syspend={syspend} ext_pend={extpend} out={out_len} dt={:.1}s",
            t0.elapsed().as_secs_f64()
        );
        let entries = m.vec_entries();
        eprintln!("  向量入场累计：");
        for (v, n) in entries.iter() {
            eprintln!("    vec={:>3} {:14} n={}", v, vecname(*v), n);
        }
        eprintln!("  最近抢占前 PC=0x{:08X}", m.last_switch_pc());
    }

    let out = m.console.lock().unwrap().output().to_vec();
    println!("=== console ({}B) ===\n{}", out.len(), String::from_utf8_lossy(&out));
}
