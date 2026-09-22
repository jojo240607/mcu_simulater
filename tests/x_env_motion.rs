//! 环境测试：运动场景下 EKF 估计的稳态收敛与健康性（虚拟外设直接模拟）。
//!
//! ## ⚠️ 时钟前提已更正（2026-09-21）
//!
//! 本文件曾写「控制任务实际仅 ~30Hz（名义 250Hz），固件时间比场景时间慢约 8 倍」——
//! **该说法已过时**。它源自 `28bb0c5` 之前的虚拟时钟标定错误（VIRTUAL 30M 指令口径
//! → 172M 字节口径），后被多个测试头注沿用。
//!
//! **实测现状**：`zz_ctlprof::ctl_period_and_tick_cost` → 控制拍 **249.7Hz**
//! （周期 4.005ms），**场景时间 = 固件时间（1:1）**；计算仅占 26%（余量 74%）。
//! 又：`zz_ctlprof` 当时报的“单拍 5.897ms / CPU 147.2%”也是工具常数错误所致（见该测试）。
//!
//! **现状**：本测试的断言**仍是按旧前提设计的**（只断言稳态收敛/单调性/量级/健康位，
//! 刻意不做相位对齐的动态跟踪）。在 1:1 时钟下这些断言**可以且应当加强**
//! （相位/幅值对齐）——列为待办，**尚未重写**。

mod common;

use common::EnvHarness;
use mcu_simulater::env::scenario::{EnvScenario, Motion, Perturb};

/// 采样窗口内统计 EKF 输出（用于稳态断言）。
fn steady_roll(h: &mut EnvHarness, n: u32) -> f32 {
    let mut max_roll = 0.0f32;
    for _ in 0..n {
        h.step();
        let e = h.read_est();
        let eu = e.euler();
        max_roll = max_roll.max(eu[0].abs());
    }
    max_roll
}

#[test]
fn climb_height_tracks() {
    // 匀速爬升（vel_up=2.0）：垂向速度估计是 EKF 设计局限（位置观测不清零垂向
    // 速度增益、垂向速度纯 IMU 积分，见 EKF 注释），但高度（baro 绝对 + GPS 相对）
    // 跟踪应单调正确。断言高度单调下降（NED 向下）且健康位正常。
    let scn = EnvScenario::new(Motion::Vertical { vel_up: 2.0 }, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(400 as f64 * 13.0); // 预热（GPS fix + EKF 收敛）
    let mut prev_pz = f32::NAN;
    let mut mon_dec = true;
    let mut alt_gain = 0.0f32;
    let start_pz = {
        let e = h.read_est();
        e.pos[2]
    };
    let _t0 = h.scn.t() as f64;
        while (h.scn.t() as f64) - _t0 < (300 as f64 * 0.013) {
        h.step();
        let e = h.read_est();
        if prev_pz.is_finite() {
            // pos[2]（NED 向下）应单调减小（高度上升）
            if e.pos[2] > prev_pz + 0.05 {
                mon_dec = false;
            }
        }
        prev_pz = e.pos[2];
        if e.health > 0 {
            panic!("爬升中 FDIR health={}（应 0）", e.health);
        }
    }
    let end_pz = prev_pz;
    alt_gain = start_pz - end_pz; // NED 向下，减 = 高度增
    assert!(mon_dec, "爬升高度应单调上升（pos[2] 单调减），start={start_pz:.2} end={end_pz:.2}");
    assert!(alt_gain > 0.5, "爬升高度增益应显著（>0.5m），实际 {alt_gain:.2}m（start {start_pz:.2} end {end_pz:.2}）");
    assert!(end_pz < -0.5, "爬升后高度应为负（NED 向上为正高度），end={end_pz:.2}");
}

#[test]
fn cruise_velocity_tracks() {
    // 匀速巡航（vel_n=3.0）：GPS RMC Doppler 速度观测约束水平速度。断言稳态
    // 北向速度收敛到真值附近（GPS 观测生效），位置向北增长。
    let scn = EnvScenario::new(Motion::Cruise { vel_n: 3.0 }, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(500 as f64 * 13.0); // 预热（GPS fix + Doppler 收敛）
    let mut vmax = 0.0f32;
    let mut vmin = 1e9f32;
    let mut moved = false;
    let _t0 = h.scn.t() as f64;
        while (h.scn.t() as f64) - _t0 < (200 as f64 * 0.013) {
        h.step();
        let e = h.read_est();
        vmax = vmax.max(e.vel[0]);
        vmin = vmin.min(e.vel[0]);
        if e.pos[0] > 3.0 {
            moved = true;
        }
        if e.health > 0 {
            panic!("巡航中 FDIR health={}（应 0）", e.health);
        }
    }
    // GPS Doppler 生效 → 北向速度应在真值 3.0 附近（收敛期波动容忍）
    assert!(vmin > 1.0, "北向速度估计最小值过低 {vmin:.2}（GPS Doppler 应约束到 ~3）");
    assert!(vmax < 5.0, "北向速度估计最大值过高 {vmax:.2}");
    assert!(moved, "巡航 200 采样步后位置应向北推进（>3m）");
}

#[test]
fn oscillate_attitude_responds() {
    // 绕 x 轴摆动（amp=0.3 rad ≈ ±17°）：姿态估计应响应摆动（陀螺积分 + 机动
    // 时锚定关闭）。固件时间比场景慢，相位不可比；断言 roll 估计幅度显著出现
    // （响应摆动而非冻结在水平）且不发散。
    let scn = EnvScenario::new(Motion::Oscillate { axis: 0, amp: 0.3, freq: 0.5 }, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(500 as f64 * 13.0);
    let mut max_roll = 0.0f32;
    let _t0 = h.scn.t() as f64;
        while (h.scn.t() as f64) - _t0 < (400 as f64 * 0.013) {
        h.step();
        let e = h.read_est();
        let eu = e.euler();
        max_roll = max_roll.max(eu[0].abs());
        if e.health > 0 {
            panic!("摆动中 FDIR health={}（应 0）", e.health);
        }
    }
    assert!(max_roll > 0.06, "roll 估计应显著响应摆动（max {max_roll:.3} rad，应 >0.06）");
    assert!(max_roll < 1.0, "roll 估计应不发散（max {max_roll:.3} rad）");
}

#[test]
fn turn_yaw_rate_tracks() {
    // 协调转弯（r=20, w=0.5）：断言**机体角速率**跟踪真值、姿态有界、yaw 在转、位置绕圆推进。
    //
    // ⚠️ 本测试**不再断言 Euler yaw 的累计旋转量**（原为 >1 rad）。原因（2026-09-20 定位，
    // 见 `docs/stage1-attitude-findings.md` F6）：
    //   协调转弯下比力沿机体 −z（`a_body = [0,0,−g/cosφ]`），**加速度计看不到 bank**；
    //   `step_hil` 用加速度计做倾角对准 → EKF 初值为水平；重力锚定的**方向门**在
    //   `bank < 25.8°` 时开启 → 把估计 roll 往水平拉 → bank 永远长不到门限 →
    //   **门永不关闭，形成死锁**。估计姿态因此停在“水平偏航”，与带 29° bank 的真值
    //   **不可比**：Euler yaw 变化率 `ψ̇=(q·sinφ+r·cosφ)/cosθ` 在 φ 不同时连符号都会反。
    //   实测（诊断打印）：est=(roll 2.33°, pitch 1.95°) vs truth=(roll 29.20°, 0°)，
    //   而 `est.omega ≡ truth.omega`（0.000,0.244,0.436）逐位一致 —— 即陀螺通路完美，
    //   不可观的是 bank，不是积分。
    //
    //   该断言此前“通过”只因 `EkfEstimator` 缺 `Estimator::update_mag` 的 trait 委托、
    //   磁锚定静默失效（F5）——那时估计纯陀螺积分、Euler yaw 恰好正向累积。
    //
    // 真正要守的（均为可观测量）：
    //   1) `est.omega` 跟踪真值（容差覆盖注入时序）
    //   2) 姿态有界不发散 + FDIR 健康
    //   3) yaw **在转**（只看 |累计旋转| 有量级；方向不作为判据，理由见上）
    //   4) 位置沿圆周推进
    let scn = EnvScenario::new(Motion::Turn { radius: 20.0, rate: 0.5 }, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(500 as f64 * 13.0);
    let mut omz_min = 1e9f32;
    let mut omz_max = 0.0f32;
    let mut prev_yaw = f32::NAN;
    let mut yaw_total = 0.0f32;
    let mut pos_norm = 0.0f32;
    let mut max_roll = 0.0f32;
    let mut max_om_err = 0.0f32;
    let _t0 = h.scn.t() as f64;
        while (h.scn.t() as f64) - _t0 < (200 as f64 * 0.013) {
        h.step();
        let e = h.read_est();
        let eu = e.euler();
        omz_min = omz_min.min(e.omega[2]);
        omz_max = omz_max.max(e.omega[2]);
        // 机体角速率与真值的偏差（本轮加入：这才是可观测量）
        let tr_om = h.scn.truth().omega;
        for k in 0..3 {
            max_om_err = max_om_err.max((e.omega[k] - tr_om[k]).abs());
        }
        // yaw 累计按 unwrap（euler() 的 yaw 在 ±π wrap，直接比较会误判）。
        if prev_yaw.is_finite() {
            let mut d = eu[2] - prev_yaw;
            if d > core::f32::consts::PI { d -= 2.0 * core::f32::consts::PI; }
            else if d < -core::f32::consts::PI { d += 2.0 * core::f32::consts::PI; }
            yaw_total += d;
        }
        prev_yaw = eu[2];
        pos_norm = pos_norm.max((e.pos[0] * e.pos[0] + e.pos[1] * e.pos[1]).sqrt());
        if e.health > 0 {
            panic!("转弯中 FDIR health={}（应 0）", e.health);
        }
        max_roll = max_roll.max(eu[0].abs());
    }
    eprintln!(
        "[turn] est_omz=[{omz_min:.3},{omz_max:.3}] max|Δω|={max_om_err:.4} |Δyaw|={:.2} rad max|roll|={:.1}° pos_norm={pos_norm:.1}m",
        yaw_total.abs(),
        max_roll.to_degrees()
    );
    // 1) 机体角速率跟踪（可观测量）
    assert!(omz_min > 0.3 && omz_max < 0.7, "yaw 角速率应跟踪真值 0.5 rad/s（min={omz_min:.2} max={omz_max:.2}）");
    assert!(max_om_err < 0.15, "机体角速率应贴近真值（max|Δω|={max_om_err:.4} rad/s）");
    // 2) 姿态有界
    assert!(max_roll < 1.0, "roll 估计应不发散（{:.3} rad）", max_roll);
    // 3) yaw 在转（方向不作为判据，见函数头 F6 说明）
    assert!(yaw_total.abs() > 0.5, "转弯中 yaw 应有旋转（|累计| > 0.5 rad，实际 {yaw_total:.2} rad）");
    // 4) 位置推进
    assert!(pos_norm > 3.0, "转弯中位置应沿圆周推进（>3m，实际 {pos_norm:.1}）");
}
