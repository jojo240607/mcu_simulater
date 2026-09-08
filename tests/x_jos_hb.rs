//! 第二段验收：icount 节拍下，应用分区周期任务能否真正被周期唤醒并打出
//! `hb seq=`（flyctrl）/ `alive seq=`（demo-app）心跳；同时应无 [SCHED_ASSERT]。
//! （PendSV 高密度风暴修复后该链路已打通，见 CONTRIBUTING-run.md §二.4）
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;

const SYS: &str = r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf";
const APP: &str = r"/home/ubuntu/work/joc-rtos-app-sdk/app.bin";

#[test]
fn hb_periodic() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&Path::new(SYS)).unwrap();
    m.load_app_partition(&Path::new(APP)).unwrap();
    m.reset().unwrap();

    let inv = Arc::new(AtomicBool::new(false));
    let ipc = Arc::new(AtomicU32::new(0));
    let (i1, i2) = (inv.clone(), ipc.clone());
    m.cpu
        .raw()
        .add_insn_invalid_hook(move |uc| {
            let pc = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
            i2.store(pc, Ordering::Relaxed);
            i1.store(true, Ordering::Relaxed);
            false
        })
        .unwrap();

    let t0 = std::time::Instant::now();
    let mut hb = false;
    let mut schd = false;
    for step in 0..600u32 {
        if t0.elapsed().as_secs() > 300 {
            eprintln!(">>> 超时(300s) 无 hb");
            break;
        }
        let r = m.run(400_000);
        let text = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        if text.contains("hb seq=") || text.contains("alive seq=") {
            eprintln!("\n>>> 心跳出现（step {step}）");
            hb = true;
        }
        if text.contains("[SCHED_ASSERT]") {
            schd = true;
            eprintln!("!!! [SCHED_ASSERT] 出现");
        }
        if inv.load(Ordering::Relaxed) {
            eprintln!("!!! INSN_INVALID @0x{:08X}", ipc.load(Ordering::Relaxed));
            break;
        }
        if let Err(e) = r {
            eprintln!("ERR {e:?}"); break;
        }
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let marks = m.vec_entries();
        let st = marks.iter().find(|(v, _)| *v == 14).map(|(_, n)| *n).unwrap_or(0);
        let ss = marks.iter().find(|(v, _)| *v == 15).map(|(_, n)| *n).unwrap_or(0);
        if step % 8 == 0 || hb {
            eprintln!(
                "[step {step}] pc=0x{pc:08X} pendSV={st} sysTick={ss} out={} hb={hb} dt={:.0}s",
                text.len(),
                t0.elapsed().as_secs_f64()
            );
        }
        if hb {
            break;
        }
        if step > 580 {
            break;
        }
    }
    let out = {
        let outv = m.console.lock().unwrap().output().to_vec();
        String::from_utf8_lossy(&outv).into_owned()
    };
    println!("=== console ({}B) tail ===\n{}", out.len(), &out[out.len().saturating_sub(2200)..]);
    eprintln!("\nRESULT: hb={hb} sched_assert={schd}");
    assert!(hb, "周期心跳 hb 未出现");
    assert!(!schd, "出现 SCHED_ASSERT");
    assert!(!inv.load(Ordering::Relaxed), "INSN_INVALID");
}
