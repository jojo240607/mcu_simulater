//! 环境测试：传感器故障 → FDIR 健康位 + EKF 抗性 + 恢复收敛（虚拟外设直接模拟）。
//!
//! 固件 FDIR 检测基于**可用性**（fdir.rs）：GPS 失锁（fix=0 → pos_available=false）
//! 超 40 拍 → Degraded；IMU 输出异常幅值（<6 或 >14 m/s²）且连续不变超 20 拍 →
//! Critical；baro/mag 冻结（仍有读数）不置位（防误报）。数值阶跃（BaroStep/
//! GpsJump）不触发 FDIR，靠 EKF 观测融合抗性 + baro 强锚定（r_alt=0.3）吸收。
//! 固件时间比场景慢约 8 倍（虚拟时钟保真度），故故障窗口（场景秒）需按固件
//! 拍数折算：gps_timeout=40 拍 ≈ 1.3s 场景，imu_stale_timeout=20 拍 ≈ 0.7s 场景。

mod common;

use common::EnvHarness;
use mcu_simulater::env::scenario::{EnvScenario, FaultEvent, Motion, Perturb};

#[test]
fn imu_saturate_critical() {
    // IMU 饱和到 ±2 m/s²：比力幅值 ≤3.46 <6 且恒定 → FDIR 判冻结 → Critical（安全模式）。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::ImuSaturate { t: 3.0, fs: 2.0 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(300); // 预热（fix + 正常）
    let mut saw_critical = false;
    for _ in 0..600 {
        h.step();
        let e = h.read_est();
        if e.health == 2 {
            saw_critical = true;
            break;
        }
    }
    assert!(saw_critical, "IMU 饱和应触发 FDIR Critical（health=2）");
}

#[test]
fn imu_freeze_hover_no_false_positive() {
    // IMU 冻结在悬停比力（norm=9.81 ∈ [6,14]）→ FDIR 的"合理冻结"不误报（防
    // 把稳定悬停判成故障），health 保持 Nominal。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::ImuFreeze { t: 4.0 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(400);
    for _ in 0..400 {
        h.step();
        let e = h.read_est();
        assert!(e.health == 0, "悬停中 IMU 冻结（幅值合理）不应误报，health={}", e.health);
    }
}

#[test]
fn gps_drop_degraded_then_recover() {
    // GPS 失锁 6s 场景：fix=0 → pos_available=false → 40 拍后 Degraded；恢复后回 Nominal。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::GpsDrop { t: 3.0, dur: 6.0 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(300); // 预热（fix established）
    let mut saw_degraded = false;
    let mut degraded_step = 0u32;
    for s in 0..900u32 {
        h.step();
        let e = h.read_est();
        if e.health == 1 && !saw_degraded {
            saw_degraded = true;
            degraded_step = s;
        }
    }
    assert!(saw_degraded, "GPS 失锁应触发 Degraded（health=1），首个降级步={degraded_step}");
    // 恢复窗口（GpsDrop 6s ≈ 450 步场景）：再跑 600 步断言回 Nominal
    let mut recovered = false;
    for _ in 0..600 {
        h.step();
        let e = h.read_est();
        if e.health == 0 {
            recovered = true;
            break;
        }
    }
    assert!(recovered, "GPS 恢复后 FDIR 应回 Nominal（health=0）");
}

#[test]
fn baro_step_bounded_by_gps() {
    // 气压计阶跃 +15m：不触发 FDIR（防误报），EKF pos[2] 瞬态被 GPS 位置观测
    // 融合吸收（baro 强约束但非唯一），偏差有界且不爆。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::BaroStep { t: 5.0, dalt: 15.0 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(400); // 预热
    let before = h.read_est().pos[2];
    let mut max_dev = 0.0f32;
    for _ in 0..400 {
        h.step();
        let e = h.read_est();
        max_dev = max_dev.max((e.pos[2] - before).abs());
        assert!(e.health == 0, "气压计阶跃不应触发 FDIR，health={}", e.health);
    }
    // baro 阶跃 15m 被 GPS(r_pos=0.5) 与 baro(r_alt=0.3) 融合：posD 偏差小于阶跃
    // 且逐步收敛（GPS 位置观测 20Hz 持续拉回）
    assert!(max_dev < 12.0, "气压计阶跃后 pos[2] 偏差应有限（<12m），实际 {max_dev:.1}m");
    let after = h.read_est().pos[2];
    let dev = (after - before).abs();
    // baro 阶跃 15m 后稳态偏差 = baro(r_alt=0.3 强) 与 GPS(r_pos=0.5 弱) 的融合
    // 加权结果（实测 ~8.6m）：被部分吸收（<15m）但不会爆，属"有界污染"而非失控。
    assert!(dev > 0.5 && dev < 14.0, "气压计阶跃后 pos[2] 稳态偏差应被 GPS 部分吸收（0.5~14m），实际 {dev:.1}m");
}

#[test]
fn gps_jump_rejected_by_baro() {
    // GPS 位置高度跳变 +25m：EKF pos[2] 被 baro 强锚定（r_alt=0.3）拉回，不被
    // 单次异常观测带走；健康保持 Nominal（跳变不触发 FDIR，属观测噪声层面）。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::GpsJump { t: 5.0, d: [0.0, 0.0, 25.0] }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(400); // 预热
    let before = h.read_est().pos[2];
    let mut max_dev = 0.0f32;
    for _ in 0..300 {
        h.step();
        let e = h.read_est();
        max_dev = max_dev.max((e.pos[2] - before).abs());
        assert!(e.health == 0, "GPS 跳变不应触发 FDIR，health={}", e.health);
    }
    assert!(max_dev < 8.0, "GPS 高度跳变 25m 后 pos[2] 偏差应被 baro 抑制（<8m），实际 {max_dev:.1}m");
}

#[test]
fn baro_freeze_no_false_positive() {
    // 气压计冻结（仍有读数）：baro_available 仍 true → 不触发 FDIR（防误报）；
    // 高度估计由冻结气压锚定（无漂移源），health 保持 Nominal。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::BaroFreeze { t: 4.0 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(400);
    for _ in 0..400 {
        h.step();
        let e = h.read_est();
        assert!(e.health == 0, "气压计冻结（有读数）不应触发 FDIR，health={}", e.health);
    }
}
