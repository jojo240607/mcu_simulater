//! 冒烟：静态悬停 —— 验证「虚拟设备直接模拟」全链路。
//!
//! 链路：EnvScenario(Hover) → FlySimState → 虚拟外设 I2C/UART → real-sensors
//! 固件真实驱动 → EKF。断言：
//! - hb 心跳持续（imu/baro/gps 健康全 true，任务不冻结）；
//! - EKF 收敛：悬停静止时速度估计 ≈ 0、姿态 ≈ 水平、高度 ≈ 真值；
//! - SENSOR_SEQ 持续推进（sensors 任务不冻结）；
//! - 无非法指令 / 无 panic。

mod common;

use common::{EnvHarness, EstReadout};
use mcu_simulater::env::scenario::{EnvScenario, Motion};

/// 布局探针 + 全链路冒烟：任务推进、EKF 输出可读、health/armed 值域正确。
#[test]
fn est_layout_probe() {
    let scn = EnvScenario::new(Motion::Hover, mcu_simulater::env::scenario::Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    // 跑 400 步（~2.7s 虚拟时间），EKF 应已稳定。
    h.run_steps(400);
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
    h.run_steps(50);
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
    h.run_steps(450); // ~3s

    // 收敛后连续采样 500 步（~3.4s）：断言误差全程有界、健康保持 Nominal、任务不冻结
    let mut worst_vel = 0.0f32;
    let mut worst_tilt = 0.0f32;
    let mut worst_alt_err = 0.0f32;
    let mut health_ok = true;
    let mut est_ok = true;
    let mut last_seq = h.read_sensor_seq();
    let mut last_adv = 0;
    for _ in 0..500 {
        h.step();
        let e: EstReadout = h.read_est();
        let eu = e.euler();
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
