//! jOS 系统分区 + joc-app-rust 应用分区（轨 B 双分区）整机验收：
//! 系统 ELF + app.bin 双镜像加载，验证 App 分区被系统自举挂载，
//! 且 Rust 应用层（flyctrl 飞控任务）成功拉起并运行。
//!
//! 里程碑（控制台日志）：
//!   ready    = "READY. Commands" + "mounting app layer"
//!   mounted  = "RUST app mounted"        （rust_app_start 自报，App 分区挂载成功）
//!   tasks    = 任一 "task started"       （ctrl/sensor/telem/uplink 业务任务被调度）
//!   hb       = "hb seq=" / "alive seq=" （周期心跳，≈1s 一条；demo-app 用 alive）
//! 断言到 hb：周期心跳依赖「任务睡眠后 SysTick 唤醒推进」——该能力随
//! PendSV 高密度风暴修复（joc-base rtos_yield 空切防护 + 本仓库 run() 预算递减）
//! 已打通，故 hb 从加分项升级为必需判据（回归守卫）。
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;

#[test]
fn app_partition_boot() {
    let elf = Path::new(r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf");
    let app = Path::new(r"/home/ubuntu/work/joc-rtos-app-sdk/app.bin");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();

    // INSN_INVALID 兜底：任何非法指令直接判失败（① 修复的回归守卫）
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
    for step in 0..700u32 {
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
        if text.contains("hb seq=") || text.contains("alive seq=") {
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
            // App 分区已挂载 + 业务任务拉起。继续跑到周期心跳出现（demo 任务
            // 每 50ms 醒一次、每 20 醒打一条 alive ≈ 1 虚拟秒；窗口留 ~700 步），
            // 验证 SysTick 周期唤醒推进链路（本次修复目标）；超窗则 hb 断言失败。
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
    assert!(hb, "周期心跳未出现（alive/hb seq=）：SysTick 唤醒推进链路异常（PendSV 风暴回归？）");
}
