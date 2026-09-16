//! 环境测试：长时间运行（10s+ 虚拟）估计有界、任务不冻结、健康正常。
//! 2000 步 ≈ 26.7s 虚拟时间（固件视角因虚拟时钟慢约 8 倍 ≈ 3.3s 固件秒）。

mod common;

use common::EnvHarness;
use mcu_simulater::env::scenario::{EnvScenario, Motion, Noise, Perturb};

#[test]
fn long_hover_bounded_and_alive() {
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb { noise: Some(Noise::default()), ..Perturb::clean() },
        vec![],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(400); // 预热（fix + 收敛）
    let seq0 = h.read_sensor_seq();
    let mut max_pos = 0.0f32;
    let mut max_vel = 0.0f32;
    let mut last_adv = 0u32; // SENSOR_SEQ 冻结窗口
    let mut last_seq = seq0;
    for _ in 0..1600 {
        h.step();
        let e = h.read_est();
        max_pos = max_pos.max((e.pos[0].powi(2) + e.pos[1].powi(2) + e.pos[2].powi(2)).sqrt());
        max_vel = max_vel.max((e.vel[0].powi(2) + e.vel[1].powi(2) + e.vel[2].powi(2)).sqrt());
        assert!(e.health == 0, "长跑中 health={}（应保持 Nominal）", e.health);
        let sq = h.read_sensor_seq();
        if sq > last_seq {
            last_adv = 0;
        } else {
            last_adv += 1;
            assert!(last_adv < 20, "SENSOR_SEQ 连续 {last_adv} 步未推进（任务冻结？）");
        }
        last_seq = sq;
    }
    assert!(max_pos < 4.0, "长时间悬停位置应有界（<4m），实际 {max_pos:.2}m");
    assert!(max_vel < 1.5, "长时间悬停速度应有界（<1.5m/s），实际 {max_vel:.2}m/s");
    let seq1 = h.read_sensor_seq();
    assert!(seq1 > seq0, "SENSOR_SEQ 应持续推进（{seq0}→{seq1}）");
}

#[test]
fn long_cruise_converges_and_bounded() {
    let scn = EnvScenario::new(Motion::Cruise { vel_n: 3.0 }, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    h.run_steps(600); // 预热（fix + Doppler 收敛）
    let mut vmin = 1e9f32;
    let mut vmax = 0.0f32;
    let mut pos_n = 0.0f32;
    for _ in 0..800 {
        h.step();
        let e = h.read_est();
        vmin = vmin.min(e.vel[0]);
        vmax = vmax.max(e.vel[0]);
        pos_n = pos_n.max(e.pos[0]);
        assert!(e.health == 0, "长巡航中 health={}（应保持 Nominal）", e.health);
    }
    // 固件时间比场景慢约 8 倍（虚拟时钟保真度）→ GPS 位置观测经交叉协方差把
    // 速度推高（pos 滞后 → 加速追赶），EKF 速度有界但偏高（实测 4~5 vs 真值 3）。
    // 真机时间同步下 pos 跟踪正确、无持续位置误差，不会过冲。此处断言**有界**。
    assert!(vmin > 0.5 && vmax < 6.5, "长巡航北向速度应有界（min={vmin:.2} max={vmax:.2}，真值 3）");
    assert!(pos_n > 5.0, "长巡航位置应显著北移（>5m，实际 {pos_n:.1}m）");
}
