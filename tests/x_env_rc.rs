//! 环境测试：RC 解锁 / 掉链保护（虚拟外设直接模拟，SBUS→uart2）。
//!
//! ⚠️ 时钟前提已更正（2026-09-21）：**场景时间 = 固件时间（1:1）**，控制拍 249.7Hz
//! （原写“固件时间比场景慢约 8 倍”是 `28bb0c5` 前的标定错误遗留）。
//!
//! 固件解锁语义：SBUS ch4 raw >1700 → 解锁（armed=1）；掉链（RcDrop 全通道
//! 回中性 1500）→ 解锁位清零（armed=0）。场景用 RcStuck 在 t=0 强置 ch4=2000
//! 模拟遥控器解锁开关拨到高位。

mod common;

use common::EnvHarness;
use mcu_simulater::env::scenario::{EnvScenario, FaultEvent, Motion, Perturb};

#[test]
fn unlock_via_rc_armed() {
    // ch4=2000（>1700）→ 固件解锁，armed 置位。
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![FaultEvent::RcStuck { t: 0.0, ch: 4, raw: 2000.0, dur: 1.0e9 }],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(400 as f64 * 13.0); // boot + RC 帧同步（固件 seq ~150）
    let mut saw_armed = false;
    // 既有 x_flyctrl_unlock_flight 实测 armed 在固件 seq≈250（SBUS 20Hz 帧 + 锁存）
    // 置位。⚠️ 时钟前提已更正（2026-09-21：场景=固件 1:1，控制 249.7Hz）→
    // 250 拍 × 4ms = **1s 固件 = 1s 场景 ≈ 75 场景步**（原写“需要 ~800 场景步”
    // 是按错误的“慢 8 倍”估的）。本循环仍留 600 步（余量充足）。
    for _ in 0..600 {
        h.step();
        let e = h.read_est();
        if e.armed == 1 {
            saw_armed = true;
            break;
        }
    }
    assert!(saw_armed, "ch4=2000 应解锁（armed=1）");
}

#[test]
fn rc_drop_disarms() {
    // 解锁后掉链（全通道回 1500）→ 解锁位清零（防失控保护 ✓）。
    // 解锁先发生（`RcStuck` 持续置 ch4=2000 ✓），掉链在场景 t=10s 触发 ✓。
    // ★判据窗口按【固件时间】重述 ✓（锁相后步数是实现细节 ✗）：
    //   旧写法"600 步 + 700 步"按【13ms/步】标定 ✗（≈16.9s ✓，够到 t=10s ✓）；
    //   锁相后 ≈4ms/步 ⇒ 1300 步仅 ≈5.2s ✗ ⇒ 够不到掉链窗口 ✗✓
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![
            // ★两个故障都要有 ✓（少了 RcStuck 就不会解锁 ✗ —— 我此前漏掉过 ✓）
            FaultEvent::RcStuck { t: 0.0, ch: 4, raw: 2000.0, dur: 1.0e9 },
            FaultEvent::RcDrop { t: 10.0, dur: 30.0 },
        ],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(400 as f64 * 13.0); // boot（保持原时长 ✓，按固件时间 ✓）
    let t0 = h.fw_ms();
    println!("  [基准] t0 时刻：场景 t={:.3}s；此后我的 ms = fw_ms()-t0 ✓", h.scn.t());
    let mut armed_seen = false;
    while (h.fw_ms() - t0) < 8_000 {
        h.step();
        if h.read_est().armed == 1 {
            armed_seen = true;
            break;
        }
    }
    assert!(armed_seen, "解锁应生效（armed=1）");
    // ★判据（含宽限期 ✓）：掉链在场景 t=10s 触发 ✓，但固件**检测需要若干帧** ✓
    //   ⇒ 不能从 t=10s 就要求 armed=0 ✗（那是"要求瞬时检测"✗，物理上做不到 ✓）
    //   正确表述 ✓：① 宽限期内（10s → 11s ✓）必须转为 0 ✓；
    //              ② 此后到故障结束前（→ 39.5s ✓）必须【持续】为 0 ✓
    // ★★**时间基准 = 场景时间** ✓（不是 fw_ms()-t0 ✗）：
    //   `FaultEvent` 的 t/dur 都是【场景时间】✓；而 t0 取在 boot 之后 ✗
    //   ⇒ 用 fw_ms()-t0 会偏移一个 boot 时长（实测 t0 对应场景 t=5.198s ✓）
    //   ⇒ 实测教训 ✗：曾据此把"故障结束后 RC 恢复⇒重新解锁（正确 ✓）"误判为"回跳缺陷"✗✓
    //   （本会话第 14 次"把不同量当成同一个"✗ —— 这次是【两条时间轴】✓）
    const T_DROP: f64 = 10_000.0;
    const T_GRACE: f64 = 11_000.0;
    const T_END: f64 = 39_500.0;
    let mut became_zero_ms: Option<f64> = None;
    let mut rebounce_ms: Option<f64> = None;
    let mut in_window = 0u32;
    while (h.scn.t() as f64) < 44.0 {
        h.step();
        let ms = (h.scn.t() as f64) * 1000.0; // ★场景时间基 ✓
        if ms >= T_DROP && ms <= T_END {
            in_window += 1;
            let a = h.read_est().armed;
            if a == 0 && became_zero_ms.is_none() {
                became_zero_ms = Some(ms);
            }
            if became_zero_ms.is_some() && a == 1 && rebounce_ms.is_none() {
                rebounce_ms = Some(ms); // 掉链期间回跳（疑似缺陷 ✗）
                // ★双时间轴诊断 ✓（分辨"真缺陷"✗ vs"我的时间基准错位"✗）：
                println!(
                    "  [回跳] 我的 t={ms:.0}ms | 场景 t={:.3}s | 我的 t0 对应场景 t={:.3}s",
                    h.scn.t(),
                    h.scn.t() - (ms / 1000.0) as f32
                );
            }
        }
    }
    println!(
        "  掉链检测：转为 0 于 {:?} ms（宽限至 {T_GRACE:.0}ms ✓）；回跳于 {:?}；窗口采样 {in_window} ✓",
        became_zero_ms, rebounce_ms
    );
    assert!(in_window > 100, "掉链窗口采样过少（{in_window}）—— 判据可能空洞 ✗");
    let bz = became_zero_ms.expect("掉链窗口内从未出现 armed=0 ✗ ⇒ 固件未检测到掉链 ✗");
    assert!(
        bz <= T_GRACE,
        "掉链检测过慢 ✗：转为 0 于 {bz:.0}ms，宽限 {T_GRACE:.0}ms"
    );
    assert!(
        rebounce_ms.is_none(),
        "掉链期间解锁位【回跳】✗（于 {:?} ms）—— 防失控保护未持续生效 ✗",
        rebounce_ms
    );
}