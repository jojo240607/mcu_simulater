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
        vec![FaultEvent::ImuSaturate { t: 1.0, fs: 2.0 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(70); // 预热 0.93s（fault t=1.0 前）
    let mut saw_critical = false;
    for _ in 0..300 {
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
        vec![FaultEvent::ImuFreeze { t: 1.2 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(85); // 预热 1.13s（fault t=1.2 前）
    for _ in 0..300 {
        h.step();
        let e = h.read_est();
        assert!(e.health == 0, "悬停中 IMU 冻结（幅值合理）不应误报，health={}", e.health);
    }
}

#[test]
fn gps_drop_degraded_then_recover() {
    // GPS 失锁 1.5s 场景（时钟校准后场景时间=固件时间）：fix=0 → pos_available=false
    // → 40 拍后 Degraded；失锁结束恢复 fix → 回 Nominal。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::GpsDrop { t: 1.0, dur: 1.5 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(60); // 预热 0.8s（fault t=1.0 前，fix established）
    let mut saw_degraded = false;
    let mut degraded_step = 0u32;
    for s in 0..260u32 {
        h.step();
        let e = h.read_est();
        if e.health == 1 && !saw_degraded {
            saw_degraded = true;
            degraded_step = s;
        }
    }
    assert!(saw_degraded, "GPS 失锁应触发 Degraded（health=1），首个降级步={degraded_step}");
    // 恢复窗口（fault 1.0-2.5s；GPS 样本保持 500ms 延迟 + FDIR 40 拍 → 降级约
    // 1.7s，恢复约 2.7s）：检测循环 260 步（3.46s）内应已见恢复，额外 150 步兜底。
    let mut recovered = false;
    for _ in 0..150 {
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
    // 气压计阶跃 +15m：不触发 FDIR（单传感器突变防误报），EKF 垂直跟随 baro
    // （架构：r_alt=0.3 强约束，GPS 垂直弱——信息量 baro:GPS ≈ 69:1，见 ekf.rs
    // default_quad 注释），水平位置由 GPS 位置观测约束（不扰动）。
    // 【校准语义】虚拟时钟校准（VIRTUAL=172M，场景时间=固件时间）后检测窗口
    // 按固件秒计：2.8s 内 baro 阶跃收敛到稳态 dev≈14.7m（baro 主导，GPS 仅
    // 微弱拉回）——这是真实 EKF 行为，而非"GPS 完全吸收"（校准前窗口 0.93s
    // 场景=0.16s 固件，pos 尚处瞬态 dev=8.6m 侥幸 PASS，掩盖了真实稳态）。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::BaroStep { t: 1.2, dalt: 15.0 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(80); // 预热 1.06s（fault t=1.2 前）
    let before = h.read_est().pos;
    let mut max_dev = 0.0f32;
    let mut worst_hz = 0.0f32;
    // 检测 210 步（2.8s 场景=固件，13.3ms/步）。
    for _ in 0..210u32 {
        h.step();
        let e = h.read_est();
        max_dev = max_dev.max((e.pos[2] - before[2]).abs());
        worst_hz = worst_hz.max(e.pos[0].abs().max(e.pos[1].abs()));
        assert!(e.health == 0, "气压计阶跃不应触发 FDIR，health={}", e.health);
    }
    // 瞬态有界：pos[2] 峰值不超过阶跃 +10%（不振荡/不超调污染）。
    assert!(max_dev < 16.5, "气压计阶跃后 pos[2] 瞬态偏差应有界（<16.5m），实际 {max_dev:.1}m");
    // 稳态被 GPS 部分吸收：sensors 保持 GPS 样本后（见 sensors_task.rs），EKF
    // 每拍都有位置观测（r_pos=0.5 与 baro r_alt=0.3 同量级），15m 阶跃稳态收敛到
    // 两观测加权平衡（实测 ~8.7m，远小于阶跃、明显大于 0——GPS 约束生效但 baro
    // 仍占优）。校准前（观测稀疏+窗口 0.16s 固件）dev=8.6m 是瞬态侥幸，语义不同。
    let dev = (h.read_est().pos[2] - before[2]).abs();
    assert!(dev > 0.5 && dev < 15.0,
        "baro 阶跃 15m 后稳态 dev 应被 GPS 部分吸收（0.5..15m，baro/GPS 加权），实际 {dev:.1}m");
    // 水平不被扰动：GPS 水平位置观测保持 EKF 水平约束。
    assert!(worst_hz < 1.0, "baro 阶跃不应扰动水平位置（GPS 约束），worst|pos_h|={worst_hz:.2}m");
}

#[test]
fn gps_jump_rejected_by_baro() {
    // GPS 位置高度跳变 +25m：EKF pos[2] 由 baro 强锚定（r_alt=0.3）拉回，不被
    // 单次异常观测带走；健康保持 Nominal（跳变不触发 FDIR，属观测噪声层面）。
    // 【校准语义】fault t 按固件秒（场景=固件）；GPS 样本保持后位置观测每拍
    // 注入，跳变被 baro/GPS 融合吸收，pos[2] 偏差显著小于 25m 跳变。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::GpsJump { t: 1.2, d: [0.0, 0.0, 25.0] }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(80); // 预热 1.06s（fault t=1.2 前）
    let before = h.read_est().pos[2];
    let mut max_dev = 0.0f32;
    for _ in 0..210 {
        h.step();
        let e = h.read_est();
        max_dev = max_dev.max((e.pos[2] - before).abs());
        assert!(e.health == 0, "GPS 跳变不应触发 FDIR，health={}", e.health);
    }
    // baro 强锚定：25m 跳变稳态被压到远小于跳变（实测 ~9m，baro/GPS 加权）。
    assert!(max_dev < 15.0, "GPS 高度跳变 25m 后 pos[2] 偏差应被 baro 抑制（<15m），实际 {max_dev:.1}m");
}

#[test]
fn baro_freeze_no_false_positive() {
    // 气压计冻结（仍有读数）：baro_available 仍 true → 不触发 FDIR（防误报）；
    // 高度估计由冻结气压锚定（无漂移源），health 保持 Nominal。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::BaroFreeze { t: 1.2 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(80); // 预热 1.06s（fault t=1.2 前）
    for _ in 0..210 {
        h.step();
        let e = h.read_est();
        assert!(e.health == 0, "气压计冻结（有读数）不应触发 FDIR，health={}", e.health);
    }
}

#[test]
fn mag_disturb_keeps_attitude() {
    // 磁干扰（硬铁偏置与地磁场同量级 [0.2,0.2,0.1]G）：数据持续"正常"（有读数）
    // → FDIR 不误报（mag 冻结/异常按可用性判据不置位）；EKF 磁观测被拉偏（yaw），
    // 但 roll/pitch 由 IMU+GPS 主导——验证姿态不发散（真机磁干扰最常见故障）。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::MagDisturb { t: 2.0, bias: [0.2, 0.2, 0.1] }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(120); // 预热 1.6s（fault t=2.0 前，EKF 收敛）
    let mut max_rp = 0.0f32;
    let mut health_ok = true;
    for _ in 0..250 {
        h.step();
        let e = h.read_est();
        let rp = e.euler();
        max_rp = max_rp.max(rp[0].abs()).max(rp[1].abs());
        if e.health == 2 {
            health_ok = false;
        }
    }
    assert!(health_ok, "磁干扰（有读数）不应触发 FDIR Critical（health=2）");
    assert!(
        max_rp.to_degrees() < 15.0,
        "磁干扰下 roll/pitch 应保持有界（稳定性），max={:.1}°",
        max_rp.to_degrees()
    );
}

#[test]
fn mag_freeze_keeps_attitude() {
    // 磁力计冻结（数据恒定）：与 baro 冻结同理 FDIR 不误报；yaw 转陀螺积分，
    // roll/pitch 不受影响——姿态稳定不发散。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::MagFreeze { t: 2.0 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(120);
    let mut max_rp = 0.0f32;
    let mut health_ok = true;
    for _ in 0..250 {
        h.step();
        let e = h.read_est();
        let rp = e.euler();
        max_rp = max_rp.max(rp[0].abs()).max(rp[1].abs());
        if e.health == 2 {
            health_ok = false;
        }
    }
    assert!(health_ok, "磁力计冻结不应触发 FDIR Critical，health={}", if health_ok { 0 } else { 2 });
    assert!(
        max_rp.to_degrees() < 15.0,
        "磁力计冻结下 roll/pitch 应保持有界，max={:.1}°",
        max_rp.to_degrees()
    );
}
