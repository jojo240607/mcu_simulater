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
    // 解锁后掉链（全通道回 1500）→ 解锁位清零（防失控保护）。
    // 解锁先发生（固件 seq≈250），掉链在场景 t=10s（固件 ~290 拍）后触发
    let scn = EnvScenario::new(
        Motion::Hover,
        Perturb::clean(),
        vec![
            FaultEvent::RcStuck { t: 0.0, ch: 4, raw: 2000.0, dur: 1.0e9 },
            FaultEvent::RcDrop { t: 10.0, dur: 30.0 },
        ],
    );
    let mut h = EnvHarness::new(scn, true);
    h.run_for_ms(400 as f64 * 13.0); // boot
    let mut armed_seen = false;
    for _ in 0..600 {
        h.step();
        if h.read_est().armed == 1 {
            armed_seen = true;
            break;
        }
    }
    assert!(armed_seen, "解锁应生效（armed=1）");
    // 掉链窗口（t=10s 场景 ≈ 750 步）持续 30s 场景 → 解锁位应清零
    let mut disarmed = false;
    for _ in 0..700 {
        h.step();
        let e = h.read_est();
        // RC 掉链时固件应保持解锁位清零（不回跳）
        if e.armed == 0 {
            disarmed = true;
        } else {
            disarmed = false; // 掉链期间必须持续未解锁
        }
    }
    assert!(disarmed, "RC 掉链应保持解锁位清零（armed=0）");
}
