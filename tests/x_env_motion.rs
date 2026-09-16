//! 环境测试：运动场景下 EKF 估计的稳态收敛与健康性（虚拟外设直接模拟）。
//!
//! 重要背景：模拟器虚拟时钟下固件 EKF 每拍执行时间超过 4ms 预算，控制任务实际
//! 仅 ~30Hz（名义 250Hz），固件时间比场景时间慢约 8 倍（虚拟时钟保真度限制，
//! 见 docs/virtual_direct_mode.md）。因此本测试【不做场景时间相位对齐】的动态
//! 跟踪断言，改断言稳态收敛 / 单调性 / 量级 / 健康位——这些在传感器数据正确
//! （场景 write_state 驱动真实固件驱动）的前提下不依赖时间基准。

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
    h.run_steps(400); // 预热（GPS fix + EKF 收敛）
    let mut prev_pz = f32::NAN;
    let mut mon_dec = true;
    let mut alt_gain = 0.0f32;
    let start_pz = {
        let e = h.read_est();
        e.pos[2]
    };
    for _ in 0..300 {
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
    h.run_steps(500); // 预热（GPS fix + Doppler 收敛）
    let mut vmax = 0.0f32;
    let mut vmin = 1e9f32;
    let mut moved = false;
    for _ in 0..200 {
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
    h.run_steps(500);
    let mut max_roll = 0.0f32;
    for _ in 0..400 {
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
    // 协调转弯（r=20, w=0.5）：yaw 角速率估计应跟踪陀螺（稳态），yaw 单调增长，
    // 位置绕圆周推进。协调转弯的 roll 稳态由 bank 建立过程（陀螺积分）决定；
    // 本场景从 t=0 即恒定 bank 27°（无建立过程）→ roll 不可由陀螺建立（omega[0]=0），
    // 且比力方向竖直使重力锚定无法提供参考——此为场景构造局限，不断言 roll 精确值，
    // 只断言不发散。
    let scn = EnvScenario::new(Motion::Turn { radius: 20.0, rate: 0.5 }, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(500);
    let mut omz_min = 1e9f32;
    let mut omz_max = 0.0f32;
    let mut prev_yaw = f32::NAN;
    let mut yaw_total = 0.0f32;
    let mut pos_norm = 0.0f32;
    for _ in 0..200 {
        h.step();
        let e = h.read_est();
        let eu = e.euler();
        omz_min = omz_min.min(e.omega[2]);
        omz_max = omz_max.max(e.omega[2]);
        // yaw 单调性按 unwrap 累计（euler() 的 yaw 在 ±π wrap，跨边界直接比较
        // 会误判"减小"）：单步增量 wrap 到 [-π, π]，累计净旋转 > 0 即单调推进。
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
        let r = e.euler()[0];
        assert!(r.abs() < 1.0, "roll 估计应不发散（{r:.3} rad）");
    }
    assert!(omz_min > 0.3 && omz_max < 0.7, "yaw 角速率应跟踪真值 0.5 rad/s（min={omz_min:.2} max={omz_max:.2}）");
    assert!(yaw_total > 1.0, "转弯中 yaw 应单调增长（累计净旋转 >1 rad，实际 {yaw_total:.2} rad）");
    assert!(pos_norm > 3.0, "转弯中位置应沿圆周推进（>3m，实际 {pos_norm:.1}）");
}
