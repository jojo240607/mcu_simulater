//! 任务停滞回归：长跑下控制/传感器任务必须**持续**推进。
//!
//! 背景：实测控制任务会在运行 8~10s 后**永久阻塞**（`CTRL_TICKS` 停增，而
//! `SENSOR_SEQ`/`g_tick`/遥测继续），表现与"性能不够"相似但性质不同——是阻塞，
//! 不是变慢。该缺陷曾让 `x_env_motion::turn_yaw_rate_tracks` 失败（它的测量窗口
//! 落在卡死之后 → 累计 yaw 不足）。
//!
//! 触发条件与固件的 USB 通路相关：固件一旦经 usb0 收发（无论有无主机）就会卡；
//! 一期固件不带机载电脑、已用 `usb-link` feature 关掉 usb0 收发（默认关闭），
//! 本测试即为该状态的回归守护。后续重新打开 `usb-link` 时必须先解决该阻塞。
//!
//! 断言用**每 50 步的推进量**而非绝对拍数：只要任一区间推进为 0，即判定停滞。

mod common;
use common::EnvHarness;
use mcu_simulater::env::scenario::{EnvScenario, Motion, Perturb};

fn rd(m: &mut mcu_simulater::machine::Machine, a: u64) -> u32 {
    let b = m.cpu.mem_read(a, 4).unwrap_or_else(|_| vec![0; 4]);
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

#[test]
fn tasks_keep_running_long_run() {
    const STEPS: u32 = 900; // 900 × 13ms ≈ 11.7s 场景时间
    let scn = EnvScenario::new(Motion::Turn { radius: 20.0, rate: 0.5 }, Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    let ctrl = mcu_simulater::elfsym::app_sym("CTRL_TICKS");
    let seq = mcu_simulater::elfsym::app_sym("SENSOR_SEQ");

    let mut c_prev = rd(&mut h.m, ctrl as u64);
    let mut q_prev = rd(&mut h.m, seq as u64);
    let (mut c_total, mut q_total) = (0u32, 0u32);
    for k in 1..=STEPS {
        h.step();
        if k % 50 != 0 {
            continue;
        }
        let c = rd(&mut h.m, ctrl as u64);
        let q = rd(&mut h.m, seq as u64);
        let dc = c.wrapping_sub(c_prev);
        let dq = q.wrapping_sub(q_prev);
        assert!(
            dc > 0,
            "[{k} 步] 控制任务停滞：CTRL_TICKS 在最近 50 步内没有推进（{c_prev} -> {c}）"
        );
        assert!(
            dq > 0,
            "[{k} 步] 传感器任务停滞：SENSOR_SEQ 在最近 50 步内没有推进（{q_prev} -> {q}）"
        );
        c_total = c_total.wrapping_add(dc);
        q_total = q_total.wrapping_add(dq);
        c_prev = c;
        q_prev = q;
    }
    // ★锁相步进下（§5.17 ✓）：**一步 = 恰好一拍控制** ✓（时基就是控制拍 ✓）
    //   旧断言的"每步约 3.25 拍"✗ 是【13ms/步】时代的产物 ✗ ⇒ 在锁相下永不成立 ✗
    //   —— 而"是否被饿死"其实已由上面的 `dc > 0`（每 50 步内 CTRL_TICKS 必须推进 ✓）
    //      直接覆盖 ✓ ⇒ 这里改为【与锁相语义一致】的总量校验 ✓
    assert!(
        c_total + 2 >= STEPS,
        "控制任务总推进与步数不符（{c_total} 拍 / {STEPS} 步）—— 锁相下应恰好一步一拍 ✓"
    );
    // 传感器任务：名义 2ms/轮 ⇒ 每步（≈4ms）应推进 ≥1 轮 ✓
    assert!(
        q_total + 2 >= STEPS,
        "传感器任务总推进偏低（{q_total} 轮 / {STEPS} 步）—— 名义 2ms/轮 ⇒ 每步应 ≥1 轮 ✓"
    );
    assert!(
        q_total > 2400,
        "传感器任务总推进偏低（{q_total} 计数 / {STEPS} 步），疑似被饿死"
    );
    println!("[stall] OK: {STEPS} 步内控制推进 {c_total} 拍、sensors 计数推进 {q_total}");
}
