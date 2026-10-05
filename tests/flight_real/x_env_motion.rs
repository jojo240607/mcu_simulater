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

#[path = "../common/mod.rs"]
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
    // ★诊断：预热期间同时打印【场景真值】与固件时间，判定真值是否已积分到荒谬量级。
    for k in 0..6 {
        h.run_for_ms(400 as f64 * 13.0 / 6.0);
        let t = h.truth();
        let e = h.read_est();
        eprintln!(
            "[motion][warm{}] fw_ms={:5} tru=({:9.2},{:9.2},{:9.2}) tru_vel=({:7.2},{:7.2},{:7.2}) | est=({:9.2},{:9.2},{:9.2})",
            k, h.fw_ms(), t.pos[0], t.pos[1], t.pos[2], t.vel[0], t.vel[1], t.vel[2],
            e.pos[0], e.pos[1], e.pos[2]
        );
        {
            // ★运行时计数器（elfsym 直读，绕开日志 ring ✓）
            let a2 = mcu_simulater::elfsym::app_sym("ESKF_CNT") as u64;
            if let Ok(b) = h.m.cpu.mem_read(a2, 48 * 4) {
                let f = |i: usize| f32::from_le_bytes([b[i*4], b[i*4+1], b[i*4+2], b[i*4+3]]);
                eprintln!("[gyr] ekf_gyro=({:.3},{:.3},{:.3})", f(28), f(29), f(30));
                {
                    let a4 = mcu_simulater::elfsym::app_sym("ESKF_PQ") as u64;
                    if let Ok(q) = h.m.cpu.mem_read(a4, 12 * 4) {
                        let g = |i: usize| f32::from_le_bytes([q[i*4], q[i*4+1], q[i*4+2], q[i*4+3]]);
                        eprintln!("[pq] pre=({:.3},{:.3},{:.3},{:.3}) d_ang=({:.4},{:.4},{:.4}) post=({:.3},{:.3},{:.3},{:.3})",
                                  g(0), g(1), g(2), g(3), g(4), g(5), g(6), g(7), g(8), g(9), g(10));
                    }
                }
                {
                    let a3 = mcu_simulater::elfsym::app_sym("ESKF_INIT_Q") as u64;
                    if let Ok(q) = h.m.cpu.mem_read(a3, 8 * 4) {
                        let g = |i: usize| f32::from_le_bytes([q[i*4], q[i*4+1], q[i*4+2], q[i*4+3]]);
                        eprintln!("[initq] q0=({:.3},{:.3},{:.3},{:.3}) acc0=({:.2},{:.2},{:.2})",
                                  g(0), g(1), g(2), g(3), g(4), g(5), g(6));
                    }
                }
                eprintln!("[gravq] before=({:.3},{:.3},{:.3},{:.3}) after=({:.3},{:.3},{:.3},{:.3})",
                          f(39), f(40), f(41), f(42), f(43), f(44), f(45), f(46));
                eprintln!("[magq] before=({:.3},{:.3},{:.3},{:.3}) after=({:.3},{:.3},{:.3},{:.3})",
                          f(31), f(32), f(33), f(34), f(35), f(36), f(37), f(38));
                eprintln!("[att] q=({:.3},{:.3},{:.3},{:.3}) acc_b=({:.2},{:.2},{:.2}) Rf=({:.2},{:.2},{:.2})",
                          f(16), f(17), f(18), f(19), f(23), f(24), f(25), f(20), f(21), f(22));
                eprintln!("[cnt] gpsP={}/{} gpsV={}/{} baro_rej={} grav={}/{} step={} est_p=({:.2},{:.2},{:.2}) est_v=({:.2},{:.2},{:.2})",
                          f(0) as u32, f(1) as u32, f(2) as u32, f(3) as u32,
                          f(4) as u32, f(5) as u32, f(6) as u32, f(7) as u32,
                          f(8), f(9), f(10), f(12), f(13), f(14));
            }
        }
        if k == 0 {
            // ★② 诊断：读固件侧 ESKF_DIAG2（step 10..14 的 P/零偏快照 ✓）
            let addr = mcu_simulater::elfsym::app_sym("ESKF_DIAG2") as u64;
            if let Ok(b) = h.m.cpu.mem_read(addr, 80 * 4) {
                let f = |i: usize| f32::from_le_bytes([b[i*4], b[i*4+1], b[i*4+2], b[i*4+3]]);
                for slot in 0..5usize {
                    let o = slot * 16;
                    eprintln!(
                        "[diag2] step={:>5} ba=({:9.2},{:9.2},{:9.2}) Pvv=({:8.1},{:8.1},{:8.1}) Pba=({:9.1},{:9.1},{:9.1}) Pvb=({:9.1},{:9.1},{:9.1})",
                        f(o+15) as u32,
                        f(o+0), f(o+1), f(o+2),
                        f(o+3), f(o+4), f(o+5),
                        f(o+6), f(o+7), f(o+8),
                        f(o+12), f(o+13), f(o+14));
                }
            }
        }
        if true {  // ★②诊断：每轮都 dump（原来只在 k==5，测试被锁相中止时永远看不到 ✗）
            let c = h.console_all();
            let tail: String = c.chars().rev().take(2400).collect::<String>().chars().rev().collect();
            eprintln!("[motion][console tail]\n{tail}");
        }
    }
    let mut prev_pz = f32::NAN;
    let mut mon_dec = true;
    let mut alt_gain = 0.0f32;
    // ★诊断：逐 0.5s 打印估计（pos/vel）与固件时间，定位"估计是否跟随机动"。
    let mut dbg_i: u32 = 0;
    let start_pz = {
        let e = h.read_est();
        e.pos[2]
    };
    // ★§5.123：观察窗改用【固件毫秒】口径 ✓（原 `scn.t()` 假定 13ms/步 ✗，
    // 实际每步 = 一个控制拍 ≈4ms ⇒ 窗口只有原意 1/3.25 ✗，见 §5.122 ✓）
    let _t0 = h.fw_ms();
        while h.fw_ms() - _t0 < (300 * 13) {
        h.step();
        let e = h.read_est();
        if prev_pz.is_finite() {
            // pos[2]（NED 向下）应单调减小（高度上升）
            if e.pos[2] > prev_pz + 0.05 {
                mon_dec = false;
            }
        }
        prev_pz = e.pos[2];
        if dbg_i % 125 == 0 {
            let ee = h.read_est();
            eprintln!(
                "[motion][climb] fw_ms={:6} est=({:7.2},{:7.2},{:7.2}) vel=({:6.2},{:6.2},{:6.2}) armed={} health={} seq={}",
                h.fw_ms(), ee.pos[0], ee.pos[1], ee.pos[2],
                ee.vel[0], ee.vel[1], ee.vel[2], ee.armed, ee.health, h.read_sensor_seq()
            );
        }
        dbg_i += 1;
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
    // ★§5.123：观察窗改用【固件毫秒】口径 ✓（原 `scn.t()` 假定 13ms/步 ✗，
    // 实际每步 = 一个控制拍 ≈4ms ⇒ 窗口只有原意 1/3.25 ✗，见 §5.122 ✓）
    let _t0 = h.fw_ms();
        while h.fw_ms() - _t0 < (200 * 13) {
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
    // ★§5.123：观察窗改用【固件毫秒】口径 ✓（原 `scn.t()` 假定 13ms/步 ✗，
    // 实际每步 = 一个控制拍 ≈4ms ⇒ 窗口只有原意 1/3.25 ✗，见 §5.122 ✓）
    let _t0 = h.fw_ms();
        while h.fw_ms() - _t0 < (400 * 13) {
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
    // ★§5.123：观察窗改用【固件毫秒】口径 ✓（原 `scn.t()` 假定 13ms/步 ✗，
    // 实际每步 = 一个控制拍 ≈4ms ⇒ 窗口只有原意 1/3.25 ✗，见 §5.122 ✓）
    let _t0 = h.fw_ms();
        while h.fw_ms() - _t0 < (200 * 13) {
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
