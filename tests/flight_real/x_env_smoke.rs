//! 冒烟：静态悬停 —— 验证「虚拟设备直接模拟」全链路。
//!
//! 链路：EnvScenario(Hover) → FlySimState → 虚拟外设 I2C/UART → real-sensors
//! 固件真实驱动 → EKF。断言：
//! - hb 心跳持续（imu/baro/gps 健康全 true，任务不冻结）；
//! - EKF 收敛：悬停静止时速度估计 ≈ 0、姿态 ≈ 水平、高度 ≈ 真值；
//! - SENSOR_SEQ 持续推进（sensors 任务不冻结）；
//! - 无非法指令 / 无 panic。

#[path = "../common/mod.rs"]
mod common;

use common::{EnvHarness, EstReadout};
use mcu_simulater::env::scenario::{EnvScenario, Motion};

/// 布局探针 + 全链路冒烟：任务推进、EKF 输出可读、health/armed 值域正确。
#[test]
fn est_layout_probe() {
    let scn = EnvScenario::new(Motion::Hover, mcu_simulater::env::scenario::Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    // 跑 400 步（~2.7s 虚拟时间），EKF 应已稳定。
    h.run_for_ms(400 as f64 * 13.0);
    let e = h.read_est();
    assert!(
        e.att_wxyz[0].abs() > 0.9,
        "姿态四元数 w 应 ≈1（单位四元数，布局/读取可能错位），得 {}",
        e.att_wxyz[0]
    );
    assert!(
        e.health <= 2,
        "health 应在 0..=2（Nominal/Degraded/Critical），得 {}（EST 布局偏移可能变了）",
        e.health
    );
    assert!(
        e.armed == 0 || e.armed == 1,
        "armed 应为 0/1，得 {}（EST 布局偏移可能变了）",
        e.armed
    );
    // EKF 高度收敛（Hover 真值 0m）：初始 ~2m，应收敛到 <1m
    assert!(
        e.pos[2].abs() < 1.0,
        "EKF 高度应收敛到 ~0m，得 {}",
        e.pos[2]
    );
    // SENSOR_SEQ 持续推进
    let s1 = h.read_sensor_seq();
    h.run_for_ms(50 as f64 * 13.0);
    let s2 = h.read_sensor_seq();
    assert!(s2 > s1, "SENSOR_SEQ 应持续推进（sensors 任务冻结？）：{s1} → {s2}");
    eprintln!("RESULT: layout ok, pos[2]={:.2} health={} sensor_seq {s1}→{s2}", e.pos[2], e.health);
}

/// 静态悬停收敛：速度≈0、姿态≈水平、高度≈真值、无漂移发散。
#[test]
fn hover_converges_stable() {
    let scn = EnvScenario::new(Motion::Hover, mcu_simulater::env::scenario::Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);

    // 预热（解锁不必要，估计器独立于 armed；跑足让 EKF 收敛）
    // ★★★相位剖分（固件侧自计 STAGE_CYC/STAGE_N ✓，ELFSYM 直读 ✓）
    {
        let names = ["0 step_hil入口","1 IMU预处理","2 姿态/位置门控","3 EKF预测+更新完",
                     "4 外部观测注入完","5","6","7","8 ESKF step进入","9 ESKF predict完",
                     "10 重力完","11 GPS位前","12 GPS位后","13 GPS速后","14 空速后",
                     "15 state组装前","16 气压前","17 气压后","18 磁前","19","20","21","22","23"];
        eprintln!("[ph] elf={:?}", mcu_simulater::elfsym::flyctrl_app_elf());
        eprintln!("[ph] sym STAGE_CYC={:?} STAGE_N={:?} ESKF_PRED={:?}",
                  mcu_simulater::elfsym::try_app_sym("STAGE_CYC"),
                  mcu_simulater::elfsym::try_app_sym("STAGE_N"),
                  mcu_simulater::elfsym::try_app_sym("ESKF_PRED"));
        if let (Some(a), Some(b)) = (mcu_simulater::elfsym::try_app_sym("STAGE_CYC"),
                                     mcu_simulater::elfsym::try_app_sym("STAGE_N")) {
            if let (Ok(c), Ok(n)) = (h.m.cpu.mem_read(a as u64, 24*4), h.m.cpu.mem_read(b as u64, 24*4)) {
                eprintln!("[ph] ---- 相位剖分（累计 DWT 周期 @168MHz ⇒ ms）----");
                for i in 0..20usize {
                    let cy = u32::from_le_bytes([c[i*4],c[i*4+1],c[i*4+2],c[i*4+3]]);
                    let nn = u32::from_le_bytes([n[i*4],n[i*4+1],n[i*4+2],n[i*4+3]]);
                    if nn > 0 {
                        eprintln!("[ph] {:<18} cyc={:>12} (n={:>5}) ⇒ {:>8.3} ms/次",
                                  names[i], cy, nn, cy as f32 / 168_000.0 / nn as f32);
                    }
                }
            }
        }
    }
    // ★★★时间基准实测：固件自己的 tick 计数 vs harness 步进/虚拟时间 ✓
    {
        let gt = mcu_simulater::elfsym::try_app_sym("g_tick");
        let sq0 = h.read_sensor_seq();
        let st0 = h.steps;
        eprintln!("[tb0] g_tick sym={:?} steps={} seq={}", gt, st0, sq0);
        if let Some(a) = gt {
            let a = a as u64;
            let t0 = match h.m.cpu.mem_read(a, 4) {
                Ok(b) => u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f32,
                Err(_) => -1.0,
            };
            let s0 = h.steps;
            h.run_for_ms(1000.0);
            let t1 = match h.m.cpu.mem_read(a, 4) {
                Ok(b) => u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f32,
                Err(_) => -1.0,
            };
            let s1 = h.steps;
            let sq1 = h.read_sensor_seq();
            eprintln!("[tb] run_for_ms(1000) ⇒ g_tick {t0:.0} → {t1:.0} (Δ{:.0} tick), steps {s0} → {s1} (Δ{}), seq={}",
                      t1 - t0, s1 - s0, sq1);
        } else {
            eprintln!("[tb] g_tick 不在 app ELF 符号表 ✗");
        }
    }
    // ★先把固件 console 里的 RT 统计打出来（exec_us/jit_us/超时 ✓）
    {
        let c = h.console_all();
        let mut n = 0;
        for ln in c.lines() {
            if ln.contains("exec_us") || ln.contains("overrun") || ln.contains("deadline")
               || ln.contains("wq") || ln.contains("rt") {
                eprintln!("[rt] {ln}");
                n += 1;
                if n > 40 { break; }
            }
        }
        eprintln!("[rt] (console 共 {} 字节, 命中 {n} 行)", c.len());
    }
    // ★预热期逐段仪表（发散发生在预热期内 ⇒ 必须看这里 ✓）
    for j in 0..13 {
        h.run_for_ms(450.0);
        let e: EstReadout = h.read_est();
        let eu = e.euler();
        {
            // ★L2 estimator item 内部实际耗时（cycles @168MHz ⇒ ms）
            for nm in ["IT_EXEC_EKF", "IT_EXEC_ATT", "IT_EXEC_SENS", "WQ_EXEC_L2"] {
                if let Some(a) = mcu_simulater::elfsym::try_app_sym(nm) {
                    if let Ok(b) = h.m.cpu.mem_read(a as u64, 4) {
                        let v = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                        eprintln!("[exec] {nm} = {v} cyc = {:.3} ms", v as f32 / 168_000.0);
                    }
                }
            }
        }
        {
            let a = mcu_simulater::elfsym::app_sym("ESKF_PRED") as u64;
            if let Ok(b) = h.m.cpu.mem_read(a, 8 * 4) {
                let g = |i: usize| f32::from_le_bytes([b[i*4], b[i*4+1], b[i*4+2], b[i*4+3]]);
                eprintln!("[pred] N={:.0} dtt_a={:.6} dtt_v={:.6} dv=({:.2},{:.2},{:.2})",
                          g(0), g(1), g(2), g(3), g(4), g(5));
            }
        }
        {
            let a = mcu_simulater::elfsym::app_sym("ESKF_VZ") as u64;
            if let Ok(b) = h.m.cpu.mem_read(a, 12 * 4) {
                let g = |i: usize| f32::from_le_bytes([b[i*4], b[i*4+1], b[i*4+2], b[i*4+3]]);
                eprintln!("[vz] obs={:8.3} pz={:8.3} vz={:8.3} ok={:.0}", g(0), g(1), g(2), g(3));
            }
        }
        eprintln!(
            "[warm] j={j:2} pos=({:10.3},{:10.3},{:10.3}) vel=({:8.3},{:8.3},{:8.3}) rpy=({:7.3},{:7.3},{:7.3})",
            e.pos[0], e.pos[1], e.pos[2], e.vel[0], e.vel[1], e.vel[2], eu[0], eu[1], eu[2]
        );
    }
    // ★初始化瞬间的姿态 + 比力（固件静态直读 ✓，绕开日志 ring ✓）
    {
        let a = mcu_simulater::elfsym::app_sym("ESKF_INIT_Q") as u64;
        if let Ok(b) = h.m.cpu.mem_read(a, 8 * 4) {
            let g = |i: usize| f32::from_le_bytes([b[i*4], b[i*4+1], b[i*4+2], b[i*4+3]]);
            eprintln!("[initq] q0=({:.4},{:.4},{:.4},{:.4}) acc0=({:.3},{:.3},{:.3})",
                      g(0), g(1), g(2), g(3), g(4), g(5), g(6));
        } else { eprintln!("[initq] READ FAIL"); }
    }

    // 收敛后连续采样 500 步（~3.4s）：断言误差全程有界、健康保持 Nominal、任务不冻结
    let mut worst_vel = 0.0f32;
    let mut worst_tilt = 0.0f32;
    let mut worst_alt_err = 0.0f32;
    let mut health_ok = true;
    let mut est_ok = true;
    let mut last_seq = h.read_sensor_seq();
    let mut last_adv = 0;
    for k in 0..500 {
        h.step();
        let e: EstReadout = h.read_est();
        let eu = e.euler();
        // ★逐段仪表（Hover 真值恒 0 ⇒ 任何偏离都是纯误差 ✓，一眼看出哪一量先偏 ✓）
        if k % 25 == 0 {
            eprintln!(
                "[hov] k={k:4} pos=({:9.3},{:9.3},{:9.3}) vel=({:8.3},{:8.3},{:8.3})                  rpy=({:7.3},{:7.3},{:7.3}) ab=({:7.3},{:7.3},{:7.3})",
                e.pos[0], e.pos[1], e.pos[2],
                e.vel[0], e.vel[1], e.vel[2],
                eu[0], eu[1], eu[2],
                e.accel_bias[0], e.accel_bias[1], e.accel_bias[2]
            );
        }
        // 悬停真值：vel=0、att=0、pos[2]=0（Hover 参考 0m）
        worst_vel = worst_vel.max(e.vel.iter().map(|v| v.abs()).fold(0.0, f32::max));
        worst_tilt = worst_tilt.max(eu[0].abs()).max(eu[1].abs());
        worst_alt_err = worst_alt_err.max(e.pos[2].abs());
        if e.health != 0 {
            health_ok = false; // FDIR 应保持 Nominal（全传感器正常）
        }
        let sq = h.read_sensor_seq();
        if sq > last_seq {
            last_seq = sq;
            last_adv = 0; // 重置"未推进计数"
        } else {
            last_adv += 1;
            if last_adv > 20 {
                // 连续 >20 步（~270ms 虚拟）SENSOR_SEQ 未推进 → sensors 任务冻结
                est_ok = false;
            }
        }
        if h.got_invalid.load(std::sync::atomic::Ordering::Relaxed) {
            est_ok = false;
            break;
        }
    }
    assert!(est_ok, "任务冻结/非法指令：console tail:\n{}", h.console_all());
    assert!(health_ok, "FDIR 应保持 Nominal（全传感器正常）");
    assert!(
        worst_vel < 0.5,
        "悬停速度估计应收敛到 ~0，worst|vel|={worst_vel:.3} m/s"
    );
    assert!(
        worst_tilt < 0.12,
        "悬停姿态应接近水平，worst tilt={worst_tilt:.3} rad"
    );
    assert!(
        worst_alt_err < 1.0,
        "悬停高度估计应接近真值 0m，worst|alt_err|={worst_alt_err:.3} m"
    );
    eprintln!(
        "RESULT: hover stable worst vel={worst_vel:.3} tilt={worst_tilt:.3} alt_err={worst_alt_err:.3} health_ok={health_ok} (steps={})",
        h.steps
    );
}
