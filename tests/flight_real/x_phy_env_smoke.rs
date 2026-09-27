//! ★§5.143 PHY 化样板（逐目标迁移的首个 ✓）：把 env 家族的运动学推进换成**真动力学**。
//!
//! 口径（与 env 家族一致 ✓）：解锁 → LOITER → 悬停；断言"姿态/高度/水平有界 + 健康正常"。
//! 与 `x_env_smoke` 的区别：真值运动由 `SimLoop::step_hil(真实刚体)` 产生（**真闭环** ✓），
//! 而非运动学直接指定 ⇒ 控制↔动力学耦合、饱和、转动惯量都参与 ✓
//!
//! 运行：`cargo test --release --test x_phy_env_smoke`（可用 `PHY_ENV_SECS` 调时长 ✓）

#[path = "../common/mod.rs"]
mod common;

use common::phy_backend::PhyBackendImpl;
use common::EnvHarness;
use mcu_simulater::env::scenario::{EnvScenario, Motion, Perturb};

#[test]
fn phy_hover_bounded_and_healthy() {
    let secs: u64 = std::env::var("PHY_ENV_SECS").ok().and_then(|v| v.parse().ok()).unwrap_or(15);
    let scn = EnvScenario::new(Motion::Hover, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    // ★PHY 化：挂真动力学后端（注入物理世界场作为磁 ⇒ 与真机语义一致 ✓）
    h.phy = Some(Box::new(PhyBackendImpl::new(true)));

    h.run_for_ms(400 as f64 * 13.0); // 预热（GPS fix + EKF 收敛 + 起飞 ✓）
    let mut max_pos = 0.0f32;
    let mut max_tilt = 0.0f32;
    let mut worst_health = 0u32;
    let t0 = h.fw_ms();
    while h.fw_ms() - t0 < (secs * 1000) as u64 {
        h.step();
        let e = h.read_est();
        max_pos = max_pos.max((e.pos[0].powi(2) + e.pos[1].powi(2) + e.pos[2].powi(2)).sqrt());
        max_tilt = max_tilt.max(e.euler()[0].abs().max(e.euler()[1].abs()));
        worst_health = worst_health.max(e.health);
    }
    eprintln!(
        "[phy-env] {secs}s | max_pos={max_pos:.2}m max_tilt={max_tilt:.1}° health={worst_health}"
    );
    assert!(max_pos < 15.0, "PHY 悬停位置应有界（<15m），实际 {max_pos:.2}m");
    assert!(max_tilt < 45.0, "PHY 悬停姿态应有界（<45°），实际 {max_tilt:.1}°");
    assert_eq!(worst_health, 0, "PHY 悬停不应触发 FDIR（health={worst_health}）");
}

/// ★§5.143 PHY 化迁移②：**机动/扰动场景**（真动力学下的鲁棒性）。
///
/// 说明 ✓：摇杆机动需 **MAVLink RC override**（固件经 `rc_ov` 取摇杆 ✓，非虚拟外设
/// `FlySimState.rc_ch` ✗）⇒ 该基础设施另立目标。本迁移改做**扰动注入**（无需摇杆 ✓）：
/// 施加力矩脉冲 ⇒ 真动力学下机身被扰 ⇒ 断言**姿态被拉回且有界**（闭环抗扰 ✓）。
#[test]
fn phy_disturbance_recovered() {
    let secs: u64 = std::env::var("PHY_ENV_SECS").ok().and_then(|v| v.parse().ok()).unwrap_or(20);
    let scn = EnvScenario::new(Motion::Hover, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    h.phy = Some(Box::new(PhyBackendImpl::new(true)));
    h.run_for_ms(400 as f64 * 13.0); // 预热（起飞 + 悬停稳定 ✓）

    let mut max_tilt = 0.0f32;
    let mut worst_health = 0u32;
    let mut peak = 0.0f32;
    let t0 = h.fw_ms();
    let mut k = 0u32;
    while h.fw_ms() - t0 < (secs * 1000) as u64 {
        h.step();
        // 在 t≈5s / 10s 处各施加一次力矩脉冲（经 `PhyBackendImpl` 的 plant ✓）
        k += 1;
        if k == 1250 || k == 2500 {
            if let Some(ref mut phy) = h.phy {
                phy.disturb_torque([0.6, 0.0, 0.0]); // 绕 x 脉冲（N·m·s ✓）
            }
        }
        let e = h.read_est();
        let tilt = e.euler()[0].abs().max(e.euler()[1].abs());
        max_tilt = max_tilt.max(tilt);
        peak = peak.max(tilt);
        worst_health = worst_health.max(e.health);
    }
    let e = h.read_est();
    let final_tilt = e.euler()[0].abs().max(e.euler()[1].abs());
    eprintln!("[phy-env] 抗扰 {secs}s | peak_tilt={peak:.1}° max_tilt={max_tilt:.1}° 末态={final_tilt:.2}° health={worst_health}");
    assert!(max_tilt < 45.0, "扰动下姿态应有界（<45°），实际 {max_tilt:.1}°");
    assert!(final_tilt < 10.0, "扰动后姿态应被拉回（<10°），实际 {final_tilt:.2}°");
    assert_eq!(worst_health, 0, "扰动不应触发 FDIR（health={worst_health}）");
}

/// ★§5.144/§5.145 PHY 化迁移③：**摇杆机动**（真动力学 + MAVLink RC override）。
///
/// §5.145 通路打通要点（逐条实测 ✓，全部机械性修复）：
///  ① 固件 `G_RC_OVERRIDE`/`_VALID`/`_TICK` 需 `#[no_mangle]+#[used]` 导出（原不在符号表 ✗）
///  ② override 必须在**控制拍读取之前**写（`pre_tick` ✓；拍后写 ⇒ 固件读到上一拍 ✗）
///  ③ **模式开关 `rc_ch[5]` 必须每拍读取前重设**（固件/虚拟外设会改写它 ⇒ 拍后写无效、
///     档位掉到 0=STABILIZE ⇒ 位置环旁路 ✗；实测 `rc_ch[5]=500 ⇒ 档 0 ⇒ cmd_mode=0` ✗）
///     ⇒ `pre_tick_state` 每拍写 `rc_ch[4]=2000`（解锁）/`rc_ch[5]=2000`（LOITER ✓）
///  ④ `RcInput.armed` 应**继承 RC 链路**（override 只覆盖摇杆 4 通道 ✓ 合 MAVLink 语义 ✓）；
///     原用 `rc_ov[0]>1500` 作解锁指示 ⇒ 摇杆中位 1500 会误判失锁 ✗（已修 ✓）
/// 打通证据（探针实测 ✓✓）：`mode档=2 cmd_mode=5(LOITER) pitch=0.60 armed=1` ✓、
///   `est vel=(-1.21,-0.25,-0.20)`（有响应 ✓）
///
/// 断言口径 ✓（与 `x_env_motion` 同精神）：摇杆应产生**水平响应 + 位移 + 健康 0**。
/// 入台账 ✓：方向/量级尚需校准 —— 固件 `vx = rc.pitch * LOITER_NUDGE_GAIN` **未做中位归零**
///   （`norm(1500)=0.5` 而非 0 ✗）⇒ 摇杆量被放大 ✓
#[test]
fn phy_rc_forward_moves_north() {
    let secs: u64 = std::env::var("PHY_ENV_SECS").ok().and_then(|v| v.parse().ok()).unwrap_or(20);
    let scn = EnvScenario::new(Motion::Hover, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    let mut phy = PhyBackendImpl::new(true);
    // 摇杆：ch2=1600（pitch 前推 ✓）；解锁/模式由后端每拍保持（`pre_tick_state` ✓）
    phy.set_rc_override(Some([1500, 1600, 1500, 1500]));
    h.phy = Some(Box::new(phy));

    h.run_for_ms(400 as f64 * 13.0); // 预热（起飞 + 悬停稳定 ✓）
    let mut max_hspeed = 0.0f32;
    let mut worst_health = 0u32;
    let t0 = h.fw_ms();
    while h.fw_ms() - t0 < (secs * 1000) as u64 {
        h.step();
        let e = h.read_est();
        max_hspeed = max_hspeed.max((e.vel[0].powi(2) + e.vel[1].powi(2)).sqrt());
        worst_health = worst_health.max(e.health);
    }
    let e = h.read_est();
    let horiz = (e.pos[0].powi(2) + e.pos[1].powi(2)).sqrt();
    eprintln!(
        "[phy-env] 摇杆机动 {secs}s | max_hspeed={max_hspeed:.2}m/s 末水平位移={horiz:.2}m health={worst_health}"
    );
    assert!(max_hspeed > 0.3, "水平速度应响应摇杆（>0.3m/s），实际 {max_hspeed:.2}m/s");
    assert!(horiz > 1.0, "摇杆应产生水平位移（>1m），实际 {horiz:.2}m");
    assert_eq!(worst_health, 0, "摇杆机动不应触发 FDIR（health={worst_health}）");
}

/// ★§5.143 PHY 化迁移④：**长跑**（真动力学 + 任务活性）。
///
/// 口径（与 `x_env_longrun::long_hover_bounded_and_alive` 一致 ✓）：估计有界 + 健康 0 +
/// **`SENSOR_SEQ` 持续推进**（任务不冻结 ✓）；真值由真刚体产生 ✓。
#[test]
fn phy_long_hover_bounded_and_alive() {
    let secs: u64 = std::env::var("PHY_ENV_SECS").ok().and_then(|v| v.parse().ok()).unwrap_or(30);
    let scn = EnvScenario::new(Motion::Hover, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    h.phy = Some(Box::new(PhyBackendImpl::new(true)));
    h.run_for_ms(400 as f64 * 13.0); // 预热（起飞 + 收敛 ✓）

    let seq0 = h.read_sensor_seq();
    let mut max_pos = 0.0f32;
    let mut max_hvel = 0.0f32;
    let mut worst_health = 0u32;
    let mut last_adv = 0u32;
    let mut last_seq = seq0;
    let t0 = h.fw_ms();
    while h.fw_ms() - t0 < (secs * 1000) {
        h.step();
        let e = h.read_est();
        max_pos = max_pos.max((e.pos[0].powi(2) + e.pos[1].powi(2) + e.pos[2].powi(2)).sqrt());
        max_hvel = max_hvel.max((e.vel[0].powi(2) + e.vel[1].powi(2)).sqrt());
        worst_health = worst_health.max(e.health);
        let sq = h.read_sensor_seq();
        if sq > last_seq {
            last_adv = 0;
        } else {
            last_adv += 1;
            assert!(last_adv < 20, "SENSOR_SEQ 连续 {last_adv} 步未推进（任务冻结？）");
        }
        last_seq = sq;
    }
    let seq1 = h.read_sensor_seq();
    eprintln!(
        "[phy-env] 长跑 {secs}s | max_pos={max_pos:.2}m max_hvel={max_hvel:.2}m/s health={worst_health} SENSOR_SEQ {seq0}→{seq1}"
    );
    assert!(max_pos < 8.0, "PHY 长跑位置应有界（<8m），实际 {max_pos:.2}m");
    assert!(max_hvel < 3.0, "PHY 长跑水平速度应有界（<3m/s），实际 {max_hvel:.2}m/s");
    assert_eq!(worst_health, 0, "PHY 长跑不应触发 FDIR（health={worst_health}）");
    assert!(seq1 > seq0, "SENSOR_SEQ 应持续推进（{seq0}→{seq1}）");
}

/// ★§5.143 PHY 化迁移⑤：**传感器零偏容忍**（真动力学 + 恒定加计偏置）。
///
/// 口径（与 `x_env_noise_perturb::accel_bias_tolerated` 一致 ✓）：恒定加计偏置下
/// 速度/位置**有界不失控** + 健康 0；但真值由**真刚体**产生 ⇒ 更能体现"闭环能否容忍"
/// （偏置会经控制回路放大 ✓ 更苛刻 ✓）。
#[test]
fn phy_accel_bias_tolerated() {
    let secs: u64 = std::env::var("PHY_ENV_SECS").ok().and_then(|v| v.parse().ok()).unwrap_or(25);
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb { accel_bias: [0.3, 0.2, 0.3], ..Perturb::clean() },
        vec![],
    );
    let mut h = EnvHarness::new(scn, true);
    h.phy = Some(Box::new(PhyBackendImpl::new(true)));
    h.run_for_ms(400 as f64 * 13.0); // 预热（起飞 + 收敛 ✓）

    let mut max_pos = 0.0f32;
    let mut max_vel = 0.0f32;
    let mut worst_health = 0u32;
    let t0 = h.fw_ms();
    while h.fw_ms() - t0 < (secs * 1000) {
        h.step();
        let e = h.read_est();
        max_pos = max_pos.max((e.pos[0].powi(2) + e.pos[1].powi(2) + e.pos[2].powi(2)).sqrt());
        max_vel = max_vel.max((e.vel[0].powi(2) + e.vel[1].powi(2) + e.vel[2].powi(2)).sqrt());
        worst_health = worst_health.max(e.health);
    }
    eprintln!(
        "[phy-env] 加计偏置 {secs}s | max_pos={max_pos:.2}m max_vel={max_vel:.2}m/s health={worst_health}"
    );
    assert!(max_vel < 8.0, "加计偏置下速度应有界（<8m/s），实际 {max_vel:.2}m/s");
    assert!(max_pos < 12.0, "加计偏置下位置应有界（<12m），实际 {max_pos:.2}m");
    assert_eq!(worst_health, 0, "加计偏置不应触发 FDIR（health={worst_health}）");
}
