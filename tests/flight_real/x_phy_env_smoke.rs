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
