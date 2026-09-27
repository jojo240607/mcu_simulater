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

/// ★§5.144 PHY 化迁移③：**摇杆机动（待办）** —— 已完成基础设施，但尚未打通固件通路。
///
/// 已定位的事实（本轮实测 ✓，供后续接手）：
///  · `G_RC_OVERRIDE`/`_VALID`/`_TICK` 已 `#[no_mangle]` 导出 ✓（`uplink.rs` ✓）
///  · 后端 `set_rc_override()` 每步写入 + 刷新时间戳 ✓ **写入确认生效**：
///    读回 `[1281,1600,1500,1500] valid=1` ✓（ch1 被固件改写 ⇒ 固件确实在处理 ✓）
///  · 但 20s 内北向速度恒 0 ✗ ⇒ 通路未打通（候选：模式档位 `rc.mode` 来源、
///    `rc.fresh`/模式分支、LOITER 下 `LOITER_NUDGE_GAIN` 路径 ✓，见
///    `app/src/flyctrl/control.rs:324-362`）
///  · 排查工具：`dbg est` 输出（本构建未启用 VERBOSE ✓）、CTRL_TICKS/G_CMD_MODE 探针
///    （`G_CMD_MODE` 符号名需核实 ✓）
#[test]
#[ignore = "§5.144：RC override 通路待打通（基础设施已完成 ✓，见注释）"]
fn phy_rc_forward_moves_north() {
    let secs: u64 = std::env::var("PHY_ENV_SECS").ok().and_then(|v| v.parse().ok()).unwrap_or(20);
    let scn = EnvScenario::new(Motion::Hover, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    let mut phy = PhyBackendImpl::new(true);
    phy.set_rc_override(Some([1500, 1600, 1500, 1500]));
    h.phy = Some(Box::new(phy));
    h.run_for_ms(400 as f64 * 13.0);
    let mut max_vn = 0.0f32;
    let mut worst_health = 0u32;
    let t0 = h.fw_ms();
    while h.fw_ms() - t0 < (secs * 1000) as u64 {
        h.step();
        let e = h.read_est();
        max_vn = max_vn.max(e.vel[0]);
        worst_health = worst_health.max(e.health);
    }
    let e = h.read_est();
    eprintln!("[phy-env] 摇杆机动 {secs}s | max_vn={max_vn:.2}m/s 末北向={:.2}m health={worst_health}", e.pos[0]);
    assert!(max_vn > 0.5, "北向速度应响应前推摇杆（>0.5m/s），实际 {max_vn:.2}m/s");
    assert!(e.pos[0] > 1.0, "应向北产生位移（>1m），实际 {:.2}m", e.pos[0]);
    assert_eq!(worst_health, 0, "摇杆机动不应触发 FDIR（health={worst_health}）");
}
