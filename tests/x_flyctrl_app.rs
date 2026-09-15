//! 系统分区 + flyctrl App 分区（轨 B 双分区）整机验收：
//! 系统 ELF + flyctrl/app.bin 双镜像加载，验证 App 分区被系统自举挂载，
//! 且正式飞控多任务（control/sensors/telem/uplink）成功拉起并跑出周期心跳。
//!
//! 里程碑（控制台日志）：
//!   ready    = "READY. Commands" + "mounting app layer"
//!   mounted  = "RUST app mounted"        （SDK rust_app_start 自报，App 分区挂载成功）
//!   tasks    = 任一 "task started"       （ctrl/sensor/telem/uplink 业务任务被调度）
//!   hb       = "hb seq="                 （control 4ms 周期、每 250 拍一条心跳 ≈ 1s）
//! 断言到 hb：周期心跳依赖 SysTick 唤醒推进（PendSV 风暴修复后已打通）。
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;

#[test]
fn flyctrl_app_boot() {
    let elf = artifact::joc_base_elf();
    let app = artifact::flyctrl_app_bin();
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();

    // INSN_INVALID 兜底：任何非法指令直接判失败（回归守卫）。
    let bad_pc = Arc::new(AtomicU32::new(0));
    let got_invalid = Arc::new(AtomicBool::new(false));
    let (b2, g2) = (bad_pc.clone(), got_invalid.clone());
    m.cpu
        .raw()
        .add_insn_invalid_hook(move |uc| {
            let pc = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
            b2.store(pc, Ordering::Relaxed);
            g2.store(true, Ordering::Relaxed);
            eprintln!("[INSN_INVALID] pc=0x{pc:08X}");
            false
        })
        .unwrap();

    let t_start = std::time::Instant::now();
    let mut ready = false;
    let mut mounted = false;
    let mut tasks = false;
    let mut hb = false;
    let mut mount_reported = false;
    for step in 0..800u32 {
        if t_start.elapsed().as_secs() > 300 {
            eprintln!(">>> 超时（300s）终止");
            break;
        }
        let r = m.run(400_000);
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let text = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        if text.contains("READY. Commands") && text.contains("mounting app layer") {
            ready = true;
        }
        if text.contains("RUST app mounted") {
            mounted = true;
        }
        if text.contains("task started") {
            tasks = true;
        }
        if text.contains("hb seq=") {
            hb = true;
        }
        if step % 5 == 0 || (!tasks && mounted) {
            eprintln!(
                "[step {step}] pc=0x{pc:08X} ready={ready} mounted={mounted} tasks={tasks} hb={hb} console={} iters={}",
                text.len(),
                m.run_iterations()
            );
        }
        if got_invalid.load(Ordering::Relaxed) {
            eprintln!("!!! INSN_INVALID 出现 @0x{:08X}", bad_pc.load(Ordering::Relaxed));
            break;
        }
        match r {
            Err(e) => {
                eprintln!("ERR {e:?} pc=0x{pc:08X}");
                break;
            }
            Ok(()) => {}
        }
        if mounted && tasks && !mount_reported {
            eprintln!(">>> App 挂载 + 业务任务拉起（step {step}; hb={hb}）");
            mount_reported = true;
        }
        if mounted && tasks && hb {
            break;
        }
    }
    let out = m.console.lock().unwrap().output().to_vec();
    println!("=== console ({}B) ===\n{}", out.len(), String::from_utf8_lossy(&out));
    println!("=== end ===");
    eprintln!("RESULT: ready={ready} mounted={mounted} tasks={tasks} hb={hb}");
    assert!(ready, "jOS 未到达 READY");
    assert!(mounted, "App 分区未挂载（无 RUST app mounted）");
    assert!(tasks, "App 业务任务未启动（无 task started 日志）");
    assert!(hb, "周期心跳未出现（hb seq=）：SysTick 唤醒推进链路异常（PendSV 风暴回归？）");
}
