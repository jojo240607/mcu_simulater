//! 校准判据：量"正常启动段（bringup done 之前，非风暴）里每 SysTick 平均退休指令数”。
//! 供对齐 QEMU 周期口径（真机 vs 每块×3 幻数）。(只读诊断探针)
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use mcu_simulater::machine::Machine;

const SYS: &str = r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf";
const APP: &str = r"/home/ubuntu/work/drv-bringup-app-rust/app.bin";

#[test]
fn sys_retire_calib() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&Path::new(SYS)).unwrap();
    m.load_app_partition(&Path::new(APP)).unwrap();
    m.reset().unwrap();

    // 全局每指令计数器（code hook）。仅用于校准，运行较短段可接受。
    let retire = Arc::new(AtomicU64::new(0));
    let r2 = retire.clone();
    m.cpu
        .raw()
        .add_code_hook(1, 0, move |_uc, _a, _s| {
            r2.fetch_add(1, Ordering::Relaxed);
        })
        .unwrap();

    let t0 = std::time::Instant::now();
    let mut sys_before = 0u64;
    for step in 0..40u32 {
        if t0.elapsed().as_secs() > 150 {
            break;
        }
        // 记 sysTick 计数（vec_entries vector15）
        let tick_n = |m: &Machine| -> u64 {
            m.vec_entries()
                .iter()
                .find(|(v, _)| *v == 15)
                .map(|(_, n)| *n)
                .unwrap_or(0)
        };
        let sy0 = tick_n(&m);
        let ret0 = retire.load(Ordering::Relaxed);
        let _ = m.run(400_000);
        let sy1 = tick_n(&m);
        let ret1 = retire.load(Ordering::Relaxed);
        eprintln!(
            "[step {step}] dt={:4.1}s retire={} (+{}) sysTick {}->{} (+{})  per-sysTick_insn={}",
            t0.elapsed().as_secs_f64(),
            ret1,
            ret1 - ret0,
            sy0,
            sy1,
            sy1 - sy0,
            if sy1 - sy0 > 0 { (ret1 - ret0) / (sy1 - sy0) } else { 0 }
        );
        sys_before = sy1;
        // 跑到 bringup done 即完成校准窗口（之后才进风暴）
        let out = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        if out.contains("bringup done") && (sy1 - sy0) > 0 && (ret1 - ret0) > 1000 {
            eprintln!(
                ">>> 校准窗口(upto bringup done): per-SysTick ≈ {} 退休指令",
                (ret1 - ret0) / (sy1 - sy0)
            );
            break;
        }
        let _ = sys_before;
    }
}
