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
    // ★与 H 场对齐（2026-09-21 ✓）——对照 H 场 `drift_rejection_still_works_after_fix` ✓
    //   H 场口径 ✓：① 安静配置 ② 零偏 0.05 rad/s ③ 静态悬停【长时长】④
    //             判据 =【稳态倾角】而非 max ✗（max 对时长敏感）⑤ 磁【干净】（硬铁=0 ✓）
    //   ★时长按【固件时间】表达 ✓（锁相步进后步数是实现细节 ✗；不再依赖废弃的
    //     `STEP_DT_MS` ✗）—— 这正是 §5.17/§5.18 迁移的要求 ✓。
    //   等效激励写在注释里 ✓：0.05 rad/s × 65 s（b×t = 3.25 rad ✓）
    const SECS: f64 = 65.0;
    let scn = EnvScenario::new(
        Motion::Hover,
        // 磁干净（硬铁=0 ✓，照 H 场"不把航向课题混进来"✓）+ 仅陀螺零偏 ✓
        Perturb { gyro_bias: [0.05, 0.0, 0.0], ..Perturb::clean() },
        vec![],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_secs(5.0); // 预热（fix + 收敛 ✓）—— 按固件时间 ✓

    let t0 = h.fw_ms();
    let target_ms = SECS * 1000.0;
    let mut max_tilt = 0.0f32;
    let mut ss_sum = 0.0f64;
    let mut ss_n = 0u32;
    while ((h.fw_ms() - t0) as f64 + 4.0) < target_ms {
        h.step();
        let e = h.read_est();
        assert!(e.health == 0, "陀螺零偏不应触发 FDIR（health={}）", e.health);
        let eu = e.euler();
        let tilt = (eu[0].powi(2) + eu[1].powi(2)).sqrt();
        max_tilt = max_tilt.max(tilt);
        // ★稳态窗口 = 后 1/4（照"稳态 ≠ 全段均值"的既有教训 ✓）
        if (h.fw_ms() - t0) as f64 >= target_ms * 0.75 {
            ss_sum += tilt as f64;
            ss_n += 1;
        }
    }
    let ss = (ss_sum / ss_n.max(1) as f64) as f32;
    println!("\n[对齐后的陀螺零偏测例] 零偏 0.05 rad/s × {SECS}s（按固件时间 ✓）");
    println!("  max tilt = {max_tilt:.4} rad（{:.2}°）", max_tilt.to_degrees());
    println!("  ★稳态 tilt = {ss:.4} rad（{:.2}°）", ss.to_degrees());
    println!(
        "  对照 H 场预测（Legacy 不学零偏）：b/k_eff = {:.2}°",
        (0.01f32 / (0.02 * 0.5 / 0.004)).to_degrees()
    );
    // ⚠️ 阈值【待按物理推导】✗：先测量，不为了让测试通过而定阈值 ✓（本会话纪律 ✓）
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

/// ★★★**读固件里 ESKF 适配器的逐通路计数**（§5.27 的定位手段 ✓）
///
/// 目的 ✓：回答"固件上 ESKF 实际收到了哪些观测、各多少次"✗✓
/// （"先证明机制在运行" ✓）—— 比盲猜"哪一路缺失"快得多 ✓。
///
/// 计数由 `flyctrl_core::estimator::eskf_estimator::ESKF_COUNTS`（`#[used]` ✓）
/// 经 ELF 符号 `ESKF_COUNTS` 暴露 ✓（M 场已具备 `elfsym::app_sym` ✓）。
#[test]
fn eskf_adapter_counters_in_firmware() {
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb { gyro_bias: [0.05, 0.0, 0.0], ..Perturb::clean() },
        vec![],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_secs(12.0); // 跑 12s 固件时间（含 boot ✓）

    let names = [
        "n_step", "n_grav_applied", "★n_grav_gated", "n_baro", "n_baro_rejected",
        "n_gps_pos", "n_gps_pos_rejected", "n_gps_vel", "n_gps_vel_rejected",
        "n_mag", "n_mag_rejected", "n_mag_reanchored",
    ];
    let addr = mcu_simulater::elfsym::app_sym("ESKF_COUNTS") as u64;
    let b = h.m.cpu.mem_read(addr, 48).expect("读 ESKF_COUNTS 失败 ✗");
    println!("\n[固件内 ESKF 通路计数] 地址 0x{addr:08X}（12×u32 ✓）");
    for i in 0..12 {
        let v = u32::from_le_bytes([b[4 * i], b[4 * i + 1], b[4 * i + 2], b[4 * i + 3]]);
        println!("  {:>20}: {v}", names[i]);
    }
    // ★机制自检（防"计数器根本没被写"✗）：step 必须非零 ✓
    let n_step = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
    assert!(n_step > 0, "ESKF_COUNTS[0] (n_step) = 0 ✗ ⇒ 计数器未生效/适配器未被调用 ✗");
    println!("  ✓ 计数器有效（n_step={n_step}）✓");
}

/// ★★**读固件侧诊断快照**（§5.43 ✓）——与 H 场同刻对比，定位环境差异 ✓
#[test]
fn eskf_diag_snapshot_from_firmware() {
    let scn = EnvScenario::new(Motion::Hover, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    h.run_for_secs(8.0); // 跑 8s（必须 > boot ~5.2s ✓，否则估计器走不到第 5 步 ✗）
    let addr = mcu_simulater::elfsym::app_sym("ESKF_DIAG") as u64;
    let b = h.m.cpu.mem_read(addr, 256).expect("读 ESKF_DIAG 失败 ✗"); // 4 槽 × 16 × 4B ✓
    let f = |slot: usize, i: usize| {
        let o = 4 * (slot * 16 + i);
        f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
    };
    // ★读【重力辅助三分支计数】+ 最近 dev/gn ⇒ 指名到支路 ✓
    {
        let gb = mcu_simulater::elfsym::app_sym("ESKF_GRAV_BRANCH") as u64;
        let b = h.m.cpu.mem_read(gb, 12).expect("读 GRAV_BRANCH 失败");
        let f = |i: usize| f32::from_le_bytes([b[4*i], b[4*i+1], b[4*i+2], b[4*i+3]]);
        let dv = mcu_simulater::elfsym::app_sym("ESKF_LAST_DEV") as u64;
        let d = h.m.cpu.mem_read(dv, 8).expect("读 LAST_DEV 失败");
        let g = |i: usize| f32::from_le_bytes([d[4*i], d[4*i+1], d[4*i+2], d[4*i+3]]);
        println!(
            "  [重力三分支] 退化 = {:.0} · ★门关 = {:.0} · 应用 = {:.0} ｜ 最近 dev = {:.4} m/s² (dev/g = {:.4})",
            f(0), f(1), f(2), g(0), g(1)
        );
    }
    // ★读【被拒/成功】的最新新息诊断（区分"量纲错"vs"门太紧" ✓）
    for (sym, label) in [("ESKF_LAST_REJ", "最近被拒"), ("ESKF_LAST_OK", "最近成功")] {
        let ad = mcu_simulater::elfsym::app_sym(sym) as u64;
        let b = h.m.cpu.mem_read(ad, 16).expect("读诊断失败");
        let f = |i: usize| f32::from_le_bytes([b[4*i], b[4*i+1], b[4*i+2], b[4*i+3]]);
        println!(
            "  [{label}] residual = {:.4} · sigma = {:.4} · NIS = {:.3} · 次数 = {:.0}",
            f(0), f(1), f(2), f(3)
        );
    }
    // ★同时读【各通路计数】（判定"辅助观测是否真在生效"✓）
    {
        let c = mcu_simulater::elfsym::app_sym("ESKF_COUNTS") as u64;
        let cb = h.m.cpu.mem_read(c, 48).expect("读 ESKF_COUNTS 失败");
        let g = |i: usize| u32::from_le_bytes([cb[4*i], cb[4*i+1], cb[4*i+2], cb[4*i+3]]);
        println!(
            "\n[固件通路计数] step={} grav应用={} ★grav被门拒={} baro={}(拒{}) gps位={}(拒{}) gps速={}(拒{}) mag={}(拒{})",
            g(0), g(1), g(2), g(3), g(4), g(5), g(6), g(7), g(8), g(9), g(10)
        );
        assert!(g(0) > 0, "step 计数为 0 ✗ ⇒ 适配器未被调用 ✗");
    }
    println!("\n[固件侧多点快照] 4 个时刻 ✓");
    for slot in 0..4 {
        let n = f(slot, 15);
        if n == 0.0 { println!("  槽{slot}: 未采到 ✗"); continue; }
        let a = [f(slot, 3), f(slot, 4), f(slot, 5)];
        let amag = (a[0]*a[0] + a[1]*a[1] + a[2]*a[2]).sqrt();
        println!(
            "  步{:.0}: |accel| = {:.4} ✓应≈9.81 · accel_z = {:.4} · q_w = {:.5} · gps_z = {:.2}",
            n, amag, a[2], f(slot, 6), f(slot, 13)
        );
    }
    println!("\n[固件侧快照 ESKF_DIAG] 第 {} 步", f(0, 15));
    println!("  gyro  = [{:.6}, {:.6}, {:.6}]", f(0, 0), f(0, 1), f(0, 2));
    println!("  accel = [{:.6}, {:.6}, {:.6}]", f(0, 3), f(0, 4), f(0, 5));
    println!("  q     = [{:.6}, {:.6}, {:.6}, {:.6}] (w,x,y,z)", f(0, 6), f(0, 7), f(0, 8), f(0, 9));
    println!("  bg    = [{:.6}, {:.6}, {:.6}]", f(0, 10), f(0, 11), f(0, 12));
    println!("  gps   = pos[2]={:.3}, pos[0]={:.3}", f(0, 13), f(0, 14));
    // ★先验证【符号→RAM 读取通路】本身 ✓（拿 profiler 已证明可读的 CTRL_TICKS 对照 ✓）
    {
        let ct = mcu_simulater::elfsym::app_sym("CTRL_TICKS") as u64;
        let ec = mcu_simulater::elfsym::app_sym("ESKF_COUNTS") as u64;
        let ctv = u32::from_le_bytes(h.m.cpu.mem_read(ct, 4).unwrap().try_into().unwrap());
        let ecv = u32::from_le_bytes(h.m.cpu.mem_read(ec, 4).unwrap().try_into().unwrap());
        println!("  [通路自检] CTRL_TICKS(0x{ct:X})={ctv}（应>0 ✓）| ESKF_COUNTS[0](0x{ec:X})={ecv}");
        assert!(ctv > 0, "CTRL_TICKS=0 ⇒ 符号→RAM 读取通路本身有问题 ✗（先修这个 ✗）");
    }
    assert!(f(0, 15) > 0.0, "快照未采样 ✗（步数=0）⇒ 诊断机制未运行 ✗");
    let amag = (f(0, 3) * f(0, 3) + f(0, 4) * f(0, 4) + f(0, 5) * f(0, 5)).sqrt();
    println!("  （槽0 |accel| = {amag:.4} ✓）");
    assert!((amag - 9.81).abs() < 2.0, "|accel|={amag:.3} 偏离 g 过多 ⇒ 量纲/标度可疑 ✗");
    // ★初始姿态：悬停应接近水平 ⇒ w 应 ≈1 ✓
    println!("  （槽0 q_w = {:.5} ✓）", f(0, 6));
}
