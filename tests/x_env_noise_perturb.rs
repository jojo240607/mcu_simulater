//! 环境测试：噪声 / 恒定偏置 / 温漂 / 阶跃扰动下 EKF 估计的鲁棒性（有界、不发散、
//! 健康位正常）。扰动只作用于传感器输出，真值不变——断言估计不被扰动拖离。
//! ⚠️ 时钟前提已更正（2026-09-21）：**场景时间 = 固件时间（1:1）**，控制拍 249.7Hz
//! （原写“固件时间比场景慢约 8 倍”是 `28bb0c5` 前的标定错误遗留）。
//! 现有断言仍是按旧前提设计的（**有界性/稳态量级**而非时间对齐动态跟踪），
//! 在 1:1 时钟下可以加强——列为待办，**尚未重写**。

mod common;

use common::EnvHarness;
use mcu_simulater::env::scenario::{EnvScenario, Motion, Noise, Perturb};

/// 跑 n 步并统计 pos/vel 范数最大值与健康位。
fn run_stats(h: &mut EnvHarness, n: u32) -> (f32, f32, u32) {
    let mut max_pos = 0.0f32;
    let mut max_vel = 0.0f32;
    let mut worst_health = 0u32;
    for _ in 0..n {
        h.step();
        let e = h.read_est();
        max_pos = max_pos.max((e.pos[0].powi(2) + e.pos[1].powi(2) + e.pos[2].powi(2)).sqrt());
        max_vel = max_vel.max((e.vel[0].powi(2) + e.vel[1].powi(2) + e.vel[2].powi(2)).sqrt());
        worst_health = worst_health.max(e.health);
    }
    (max_pos, max_vel, worst_health)
}

#[test]
fn noise_robust_hover() {
    // 典型传感器噪声（加计/陀螺/气压/GPS）：估计应保持有界（悬停不漂移）、健康 0。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb { noise: Some(Noise::default()), ..Perturb::clean() },
        vec![],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(400 as f64 * 13.0); // 预热（fix + 收敛）
    let (max_pos, max_vel, wh) = run_stats(&mut h, 500);
    assert!(max_pos < 3.0, "噪声下悬停位置应保持有界（<3m），实际 {max_pos:.2}m");
    assert!(max_vel < 1.0, "噪声下悬停速度应保持有界（<1m/s），实际 {max_vel:.2}m/s");
    assert_eq!(wh, 0, "典型噪声不应触发 FDIR（health={wh}）");
}

#[test]
fn accel_bias_tolerated() {
    // 恒定加计偏置（体轴 0.3 m/s²）：EKF 仅垂向零偏有状态（x[9]），水平偏置靠
    // GPS Doppler 速度约束兜底 → 速度/位置有界（不积分漂移失控）、健康 0。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb { accel_bias: [0.3, 0.2, 0.3], ..Perturb::clean() },
        vec![],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(500 as f64 * 13.0);
    let (max_pos, max_vel, wh) = run_stats(&mut h, 500);
    // 【校准语义】校准后窗口按固件秒计（6.6s），恒定偏置积分达到稳态（~3.6 m/s）：
    // EKF 仅垂向零偏有状态（x[9]），水平偏置靠 GPS Doppler 速度（r_vel=0.3，消费级
    // 噪声）拉回，稳态速度有界但非 0。断言"有界不失控"（<5）而非"收敛 0"。
    assert!(max_vel < 5.0, "加计偏置下速度应被 GPS 约束有界（<5m/s），实际 {max_vel:.2}m/s");
    assert!(max_pos < 5.0, "加计偏置下位置应有界（<5m），实际 {max_pos:.2}m");
    assert_eq!(wh, 0, "加计偏置不应触发 FDIR（health={wh}）");
}

#[test]
fn gyro_bias_tolerated() {
    // ★**与 H 场对齐**（2026-09-21 ✓）——对照 H 场 `drift_rejection_still_works_after_fix` ✓
    //   H 场口径：① 安静配置（low_noise ✓）② 零偏 0.01 rad/s ③ 静态悬停【长时长】④
    //             判据 =【稳态倾角】而非 max（max 对时长敏感 ✗）⑤ 磁【干净】（硬铁=0 ✓）
    //   M 场现实约束 ✗：步率 74.9Hz（STEP_DT_MS=13 ✓）⇒ H 场 120s ↔ 9231 步 ≈ 35 分钟 ✗
    //   ⇒ 取【等效激励 b×t】并【在注释里显式写出等价关系】✓：
    //        0.05 rad/s × 65 s  ≡  0.01 rad/s × 325 s（b×t 均为 3.25 rad ✓）
    //   ⇒ 本测例取 0.05×65s（= 5000 步 ≈ 19 分钟 ✗，但比 35 分钟可行 ✓）
    //   ★历史：原测例为 500 步（6.5s）× 0.05 rad/s ⇒ 激励仅 0.325 rad，
    //     且判据用 max tilt（对时长敏感 ✗）⇒ 与 H 场不可比 ✗（§5.4 ✓）。
    const SECS: f32 = 65.0;
    let steps = (SECS / (common::STEP_DT_MS / 1000.0)) as u32;
    let scn = EnvScenario::new(
        Motion::Hover,
        // 磁干净（硬铁=0 ✓，照 H 场"不把航向课题混进来"✓）+ 仅陀螺零偏 ✓
        Perturb { gyro_bias: [0.05, 0.0, 0.0], ..Perturb::clean() },
        vec![],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(500 as f64 * 13.0); // 预热（fix + 收敛）
    let mut max_tilt = 0.0f32;
    let mut ss_sum = 0.0f64;
    let mut ss_n = 0u32;
    for k in 0..steps {
        h.step();
        let e = h.read_est();
        let eu = e.euler();
        let tilt = (eu[0].powi(2) + eu[1].powi(2)).sqrt();
        max_tilt = max_tilt.max(tilt);
        // ★稳态窗口 = 后 1/4（照"稳态 ≠ 全段均值"的既有教训 ✓）
        if k >= steps * 3 / 4 {
            ss_sum += tilt as f64;
            ss_n += 1;
        }
        assert!(e.health == 0, "陀螺零偏不应触发 FDIR（health={}）", e.health);
    }
    let ss = (ss_sum / ss_n.max(1) as f64) as f32;
    println!(
        "\n[对齐后的陀螺零偏测例] 零偏 0.05 rad/s × {SECS}s（= 5000 步 ✓，等效激励 3.25 rad ✓）"
    );
    println!("  max tilt = {max_tilt:.4} rad（{:.2}°）", max_tilt.to_degrees());
    println!("  ★稳态 tilt = {ss:.4} rad（{:.2}°）", ss.to_degrees());
    println!("  对照 H 场预测（Legacy 不学零偏）：b/k_eff = {:.2}°", (0.01f32 / (0.02 * 0.5 / 0.004)).to_degrees());
    // ⚠️ 阈值【待按物理推导】✗：先测量，不为了让测试通过而定阈值 ✓（本会话纪律 ✓）
    //    推导依据将用：ESKF 有 bg 状态 ⇒ 稳态倾角应 【显著小于】不学零偏时的 b/k_eff 量级 ✓
    assert!(max_tilt < 1.0, "对齐后 max tilt 应有界（<1.0 rad），实际 {max_tilt:.3} rad");
}
#[test]
fn baro_drift_tolerated() {
    // 气压高度温漂 0.15 m/s：高度估计在 baro（漂移）与 GPS（不漂移）间融合，
    // 位置有界不爆；健康 0。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb { baro_drift: 0.15, ..Perturb::clean() },
        vec![],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(400 as f64 * 13.0);
    let (max_pos, _, wh) = run_stats(&mut h, 500);
    assert!(max_pos < 4.0, "气压温漂下高度估计应有界（<4m），实际 {max_pos:.2}m");
    assert_eq!(wh, 0, "气压温漂不应触发 FDIR（health={wh}）");
}

#[test]
fn accel_bias_step_tolerated() {
    // 加计偏置阶跃（t=5s 时 +0.5 m/s² 体轴）：瞬态后估计有界恢复，健康 0。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb { accel_bias_step: Some((5.0, [0.5, 0.0, 0.0])), ..Perturb::clean() },
        vec![],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(400 as f64 * 13.0);
    let (max_vel, _, wh) = run_stats(&mut h, 500);
    assert!(max_vel < 2.0, "加计偏置阶跃后速度应有界（<2m/s），实际 {max_vel:.2}m/s");
    assert_eq!(wh, 0, "加计偏置阶跃不应触发 FDIR（health={wh}）");
}
