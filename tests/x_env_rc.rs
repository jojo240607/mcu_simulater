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
    let mut armed_seen = false;
    while (h.fw_ms() - t0) < 8_000 {
        h.step();
        if h.read_est().armed == 1 {
            armed_seen = true;
            break;
        }
    }
    assert!(armed_seen, "解锁应生效（armed=1）");
    // ★判据只在【掉链窗口内】采信 ✓（t∈[10s,40s] ✓）：
    //   故障在 t=40s 结束 ⇒ RC 恢复 ⇒ 固件会【重新解锁】✓（那是正确行为 ✓）
    //   ⇒ 若把窗口跑到 45s 并在末次取值，会因"已恢复解锁"而误判为失败 ✗✓
    let mut in_window = 0u32;
    let mut disarmed_all = true;
    let mut still = 0u32;
    while (h.fw_ms() - t0) < 44_000 {
        h.step();
        let ms = (h.fw_ms() - t0) as f64;
        if ms >= 10_000.0 && ms <= 40_000.0 {
            in_window += 1;
            if h.read_est().armed != 0 {
                disarmed_all = false; // 掉链窗口内必须【始终】为 0 ✓
            }
        }
        if ms > 40_500.0 && h.read_est().armed == 1 {
            still += 1; // 窗口结束后应恢复解锁（记录，不作主判据 ✓）
        }
    }
    assert!(in_window > 100, "掉链窗口采样过少（{in_window}）—— 判据可能空洞 ✗");
    assert!(
        disarmed_all,
        "RC 掉链窗口 [10s,40s] 内解锁位必须【持续】为 0 ✗（采样 {in_window} 次）"
    );
    println!("  ✓ 掉链窗口采样 {in_window} 次全部 armed=0 ✓；窗口后恢复解锁计数 {still} ✓");
}