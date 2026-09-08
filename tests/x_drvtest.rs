//! joc-drvtest-app 整机验收：系统分区 + 驱动验证 App（双分区）。
//!
//! 验证目标（App 内部 21 个用例，覆盖 15 组驱动）：
//!   k_sdk（tick/msleep/信号量/互斥量/设备表）、d_uart、d_gpio、d_adc、d_temp、
//!   d_rng、d_crc（与软件模型逐值比对）、d_rtc、d_timer（溢出 + App ISR）、
//!   d_exti（共享线）、d_pwm、d_dac、d_spi_i2c、d_dma。
//!
//! 里程碑（控制台日志）：
//!   mounted = "RUST app mounted"   App 分区挂载成功
//!   hb      = "hb n="              心跳任务推进（SysTick 周期唤醒链路）
//!   report  = "DRVTEST REPORT total=.. pass=.. fail=.. skip=.."  用例跑完
//! 断言：fail == 0（任何驱动用例失败即验收失败）；INSN_INVALID 兜底判失败。
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;

#[test]
fn drvtest_all_drivers_pass() {
    let elf = Path::new(r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf");
    let app = Path::new(r"/home/ubuntu/work/joc-drvtest-app/app.bin");
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
    let mut mounted = false;
    let mut hb = false;
    let mut report: Option<(u32, u32, u32, u32)> = None; // (total, pass, fail, skip)
    // 21 个用例含若干 msleep（timer 溢出 120ms、exti 死线轮询等）→ 虚拟时间预算
    // 需 ~2.5s；每步 400K 字节 ≈ 2.4ms 虚拟，留 5000 步（~12s 虚拟）充足余量。
    for step in 0..5000u32 {
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
        if text.contains("RUST app mounted") {
            mounted = true;
        }
        if text.contains("hb n=") {
            hb = true;
        }
        // 解析 "DRVTEST REPORT total=21 pass=21 fail=0 skip=0"
        if report.is_none() {
            for line in text.lines() {
                if let Some(rest) = line.strip_prefix("R/I/drvtest: DRVTEST REPORT") {
                    let mut t = 0u32;
                    let mut p = 0u32;
                    let mut f = 0u32;
                    let mut s = 0u32;
                    for kv in rest.split_whitespace() {
                        let mut it = kv.split('=');
                        if let (Some(k), Some(v)) = (it.next(), it.next()) {
                            let n = v.parse::<u32>().unwrap_or(0);
                            match k {
                                "total" => t = n,
                                "pass" => p = n,
                                "fail" => f = n,
                                "skip" => s = n,
                                _ => {}
                            }
                        }
                    }
                    if t > 0 {
                        report = Some((t, p, f, s));
                    }
                }
            }
        }
        if step % 5 == 0 || (report.is_none() && mounted) {
            eprintln!(
                "[step {step}] pc=0x{pc:08X} mounted={mounted} hb={hb} report={report:?} console={} iters={}",
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
        if mounted && hb && report.is_some() {
            break;
        }
    }
    let out = m.console.lock().unwrap().output().to_vec();
    println!("=== console ({}B) ===\n{}", out.len(), String::from_utf8_lossy(&out));
    println!("=== end ===");
    eprintln!("RESULT: mounted={mounted} hb={hb} report={report:?}");
    assert!(mounted, "App 分区未挂载（无 RUST app mounted）");
    assert!(hb, "心跳未出现（hb n=）：SysTick 周期唤醒链路异常？");
    let (total, pass, fail, skip) = report.expect("未出现 DRVTEST REPORT（用例卡死？）");
    assert!(total >= 20, "用例总数异常（total={total}）");
    assert_eq!(fail, 0, "存在驱动用例失败（fail={fail}，pass={pass}，skip={skip}）");
    assert_eq!(pass, total - skip, "pass 数不吻合（total={total} pass={pass} skip={skip}）");
    eprintln!(">>> 验收通过：{pass}/{total} 通过，{skip} 跳过，0 失败");
}
