//! [临时剖析] 控制任务：实际环路周期 / 单拍成本 / CPU 占用 / 分段耗时归属。
//!
//! 目标量（用户口径）：控制回路是否回到标称 4ms(250Hz) 执行周期。
//! `ctl_period_and_tick_cost` 无 hook，测量不扰动被测对象；
//! `ctl_phase_breakdown` 装一个 block hook，按固件探针分段，只看相对分布。
//!
//! 探针（固定 VMA，见各固件侧声明）：
//!   CTRL_PHASE 0x2001F000 / CTRL_TICKS 0x2001F004  —— 控制循环外层分段
//!   HIL_PROBE  0x2001F100  —— step_hil 内层分段（[0]=段号 [1]=调用计数）

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::vperiph::data_source::FlySimState;

const CTRL_TICKS: u64 = 0x2001F004;
const CTRL_PHASE: u64 = 0x2001F000;
const HIL_PROBE: u64 = 0x2001F100;
const RETIRED_BYTES_PER_MS: f64 = 92_000.0;

/// 外层相位名（CTRL_PHASE 取值）。
const OUTER: [(usize, &str); 5] = [
    (0, "0 循环顶 -> 读完帧"),
    (1, "(1 读帧后，见内层)"),
    (2, "2 step_hil完 -> 设定点/PWM"),
    (3, "3 PWM写 -> PWM完"),
    (4, "4 PWM完 -> 拍末(含 delay_until)"),
];

/// 内层相位名（PROBE[0] 段号），键 = 10 + 段号。
/// 段号 0..=7 来自 `hil.rs`（step_hil 外层），8..=11 来自 `ekf.rs`（EKF::step 内层）。
const INNER: [(usize, &str); 12] = [
    (10, "1.0 进入 step_hil -> IMU预处理完"),
    (11, "1.1 IMU预处理 -> 初始化门控完"),
    (12, "1.2 初始化门控 -> EKF::step 返回"),
    (13, "1.3 EKF -> 外部观测(baro/vio/rtk/mag)"),
    (14, "1.4 外部观测 -> FDIR 完"),
    (15, "1.5 FDIR -> 健康闸+控制律"),
    (16, "1.6 控制律 -> 执行器限幅完"),
    (17, "1.7 限幅完 -> 退出 step_hil"),
    (18, "  └2.0 EKF入口 -> 姿态积分+重力锚定完"),
    (19, "  └2.1 重力锚定 -> 协方差预测完"),
    (20, "  └2.2 协方差预测 -> GPS观测更新完"),
    (21, "  └2.3 GPS观测 -> EKF 返回"),
];

fn u32at(m: &mut Machine, a: u64) -> u32 {
    let b = m.cpu.mem_read(a, 4).unwrap_or_else(|_| vec![0; 4]);
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn systick(m: &Machine) -> u64 {
    m.vec_entries()
        .iter()
        .find(|(v, _)| *v == 15)
        .map(|(_, n)| *n)
        .unwrap_or(0)
}

fn build() -> Machine {
    let sys = artifact::joc_base_elf();
    let app = artifact::flyctrl_real_app_bin();
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    let state = Arc::new(Mutex::new(FlySimState::default()));
    m.attach_flysim_sensors(state.clone());
    m.attach_flysim_uart_slaves(state.clone());
    {
        let mut st = state.lock().unwrap();
        st.imu_acc = [0.0, 0.0, -9.81];
        st.baro_pa = 101_325.0;
        st.gps_lat = 31.2304;
        st.gps_lon = 121.4737;
        st.gps_alt = 9.0;
        st.gps_fix = 3.0;
        st.rc_ch = [1500.0; 16];
    }
    m.load_elf(&sys).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();
    m
}

fn boot(m: &mut Machine) {
    for _ in 0..12 {
        m.run_budget(1_000_000).unwrap();
    }
    for _ in 0..60 {
        m.run_budget(1_000_000).unwrap();
    }
}

#[test]
fn ctl_period_and_tick_cost() {
    let mut m = build();
    boot(&mut m);

    const STEPS: u32 = 750; // 3s 场景
    let t0 = systick(&m);
    let c0 = u32at(&mut m, CTRL_TICKS);
    let r0 = m.retired_count();
    for _ in 0..STEPS {
        m.run_ms(4.0).unwrap();
    }
    let t1 = systick(&m);
    let c1 = u32at(&mut m, CTRL_TICKS);
    let r1 = m.retired_count();

    let fw_ms = (t1 - t0) as f64;
    let ticks = c1.wrapping_sub(c0) as f64;
    let retired = (r1 - r0) as f64;

    println!(
        "[ctl] 场景 {}ms / 固件 {:.0}ms | 控制拍 {} | 周期 {:.3}ms → {:.1}Hz (标称 4.000ms/250Hz)",
        STEPS * 4,
        fw_ms,
        ticks,
        fw_ms / ticks,
        ticks * 1000.0 / fw_ms
    );
    println!(
        "[ctl] 单拍平均退休 {:.0} 字节 → 全拍耗时 {:.3}ms | CPU 总占用 {:.1}%",
        retired / ticks,
        retired / ticks / RETIRED_BYTES_PER_MS,
        retired * 100.0 / RETIRED_BYTES_PER_MS / fw_ms
    );
}

/// 分段耗时归属。外层 `CTRL_PHASE`，内层 `HIL_PROBE`（仅在 `CTRL_PHASE == 1` 期间有效，
/// 故合成键 = 10 + 内层段号）。有 hook 开销，绝对值偏大，只看相对分布。
#[test]
fn ctl_phase_breakdown() {
    const N: usize = 22;
    #[derive(Default)]
    struct Acc {
        last_r: u64,
        last_key: usize,
        last_ticks: u32,
        ticks: u64,
        buckets: [u64; N],
    }

    let mut m = build();
    boot(&mut m);

    let retired = m.retired_handle();
    let acc = Arc::new(Mutex::new(Acc {
        last_r: retired.load(Ordering::Relaxed),
        ..Default::default()
    }));
    {
        let a = acc.clone();
        let r = retired.clone();
        m.cpu
            .raw()
            .add_block_hook(2, 0, move |uc, _addr, _size| {
                let Ok(b) = uc.mem_read_as_vec(CTRL_PHASE, 8) else {
                    return;
                };
                let ctrl_phase = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                let ticks = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
                let Ok(h) = uc.mem_read_as_vec(HIL_PROBE, 4) else {
                    return;
                };
                let hil_phase = u32::from_le_bytes([h[0], h[1], h[2], h[3]]);
                // CTRL_PHASE==1 期间（即 step_hil 内）用内层段号，否则用外层段号
                let key = if ctrl_phase == 1 {
                    (10 + hil_phase.min(11)) as usize
                } else {
                    ctrl_phase as usize
                };
                let cur = r.load(Ordering::Relaxed);
                let mut g = a.lock().unwrap();
                if ticks != g.last_ticks {
                    // 跨拍：先结算上一拍末段，再重置
                    if g.last_key < N {
                        let k = g.last_key;
                        g.buckets[k] += cur.saturating_sub(g.last_r);
                    }
                    g.last_ticks = ticks;
                    g.ticks += 1;
                    g.last_key = key;
                    g.last_r = cur;
                    return;
                }
                if key != g.last_key {
                    if g.last_key < N {
                        let k = g.last_key;
                        g.buckets[k] += cur.saturating_sub(g.last_r);
                    }
                    g.last_key = key;
                    g.last_r = cur;
                }
            })
            .unwrap();
    }

    const STEPS: u32 = 500;
    for _ in 0..STEPS {
        m.run_ms(4.0).unwrap();
    }

    let g = acc.lock().unwrap();
    let n = g.ticks.max(1) as f64;
    let ms = |i: usize| g.buckets[i] as f64 / n / RETIRED_BYTES_PER_MS;
    let mut total = 0.0;
    for (i, name) in OUTER {
        println!("[phase] 外层 {name:<34} {:7.3} ms/拍", ms(i));
        total += ms(i);
    }
    let mut inner_total = 0.0;
    for (i, name) in INNER {
        println!("[phase]   step_hil {name:<32} {:7.3} ms/拍", ms(i));
        inner_total += ms(i);
    }
    println!(
        "[phase] step_hil 内层合计 {inner_total:.3} ms/拍 | 外层合计 {total:.3} ms/拍 | 采样 {} 拍",
        g.ticks
    );
}
