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

fn build_with_uart(uart: bool) -> Machine {
    let sys = artifact::joc_base_elf();
    let app = artifact::flyctrl_real_app_bin();
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    let state = Arc::new(Mutex::new(FlySimState::default()));
    m.attach_flysim_sensors(state.clone());
    if uart {
        m.attach_flysim_uart_slaves(state.clone());
    }
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

fn build() -> Machine {
    build_with_uart(true)
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

/// 固件 RTOS 毫秒计数器（joc-base `g_tick`，RTOS_TICK_HZ=1000）。
const G_TICK: u64 = 0x100063B4;
const SENSOR_SEQ: u64 = 0x200116DC;

/// 控制/传感器任务的**真实运行频率**。
///
/// 零扰动：只在宿主侧读固件计数器（每 1ms 调一次 `run_ms(1.0)` 后采样），
/// 不装任何 hook —— 实测 block hook 每 TB 三次 `mem_read_as_vec`（带堆分配）
/// 会让固件轨迹显著偏移（同窗口 249.7Hz -> 57.3Hz），故不可用于测频率。
///
/// 标称：控制 `CONTROL_PERIOD_TICKS`=4 拍 => 250Hz；sensors `sample_dt`=0.002s
/// => 2 拍 => 500Hz。
#[test]
fn task_rates_and_periods() {
    let mut m = build();
    boot(&mut m);

    // 预热窗口（让两条任务都进入稳态）
    for _ in 0..250u32 {
        m.run_ms(4.0).unwrap();
    }

    const MS: u32 = 3000;
    let t0 = systick(&m);
    let r0 = m.retired_count();
    let c0 = u32at(&mut m, CTRL_TICKS);
    let q0 = u32at(&mut m, SENSOR_SEQ);

    // 1ms 粒度采样：记录每次 CTRL_TICKS / SENSOR_SEQ(每轮+2) 变化时经过的固件毫秒
    let mut hist_ctrl = [0u32; 24];
    let mut hist_sen = [0u32; 64];
    let mut last_c = c0;
    let mut last_q = q0;
    let mut last_ct = u32at(&mut m, G_TICK);
    let mut last_qt = last_ct;
    while ((systick(&m) - t0) as u32) < MS {
        m.run_ms(1.0).unwrap();
        let t = u32at(&mut m, G_TICK);
        let c = u32at(&mut m, CTRL_TICKS);
        if c != last_c {
            let d = t.wrapping_sub(last_ct) as usize;
            if d < hist_ctrl.len() {
                hist_ctrl[d] += 1;
            }
            last_c = c;
            last_ct = t;
        }
        let q = u32at(&mut m, SENSOR_SEQ);
        if q.wrapping_sub(last_q) >= 2 {
            let d = t.wrapping_sub(last_qt) as usize;
            if d < hist_sen.len() {
                hist_sen[d] += 1;
            }
            last_q = q;
            last_qt = t;
        }
    }
    let fw_ms = (systick(&m) - t0) as f64;
    let retired = (m.retired_count() - r0) as f64;
    let ctrl = last_c.wrapping_sub(c0) as f64;
    let sen = u32at(&mut m, SENSOR_SEQ).wrapping_sub(q0) as f64 / 2.0;

    println!(
        "[rates] 窗口 {fw_ms:.0}ms | 控制 {ctrl:.0} 拍 -> {:.2} Hz（标称 250.0，周期 {:.3}ms）",
        ctrl * 1000.0 / fw_ms,
        fw_ms / ctrl
    );
    println!(
        "[rates]                  | sensors {sen:.0} 轮 -> {:.2} Hz（标称 500.0，周期 {:.3}ms）",
        sen * 1000.0 / fw_ms,
        fw_ms / sen
    );
    println!(
        "[rates] 单拍(轮)成本：控制 {:.0} 字节 = {:.3}ms/拍 | sensors {:.0} 字节 = {:.3}ms/轮",
        retired / ctrl,
        retired / ctrl / RETIRED_BYTES_PER_MS,
        retired / sen,
        retired / sen / RETIRED_BYTES_PER_MS
    );
    println!(
        "[rates] CPU 总占用 {:.1}%（退休 {:.0} 字节/固件ms）",
        retired * 100.0 / RETIRED_BYTES_PER_MS / fw_ms,
        retired / fw_ms
    );
    let show = |name: &str, h: &[u32]| {
        let n: u32 = h.iter().sum();
        let mut parts = String::new();
        for (d, c) in h.iter().enumerate() {
            if *c > 0 {
                parts.push_str(&format!(" {d}ms:{:.1}%", *c as f64 * 100.0 / n as f64));
            }
        }
        println!("[rates] {name} 周期分布（1ms 采样粒度）{parts}");
    };
    show("控制  ", &hist_ctrl);
    show("sensors", &hist_sen);
}

/// 轻载对照：只挂 flysim 传感器、不挂 UART 从设备。
///
/// 用于判定 sensors 达不到 500Hz 是'CPU 被抢'还是'周期设定本身不对'。
/// 注意两种配置的差别**只在 UART**（USART2 NMEA GPS + USART3 SBUS）；USB 路径
/// 两者相同——固件 telemetry/uplink 任务都无条件跑，而本路径没有虚拟 USB 主机
/// （见 task_rates_with_usb_host）。
#[test]
fn task_rates_light() {
    let mut m = build_with_uart(false);
    boot(&mut m);
    for _ in 0..250u32 {
        m.run_ms(4.0).unwrap();
    }
    const MS: u32 = 3000;
    let t0 = systick(&m);
    let r0 = m.retired_count();
    let c0 = u32at(&mut m, CTRL_TICKS);
    let q0 = u32at(&mut m, SENSOR_SEQ);
    while ((systick(&m) - t0) as u32) < MS {
        m.run_ms(1.0).unwrap();
    }
    let fw = (systick(&m) - t0) as f64;
    let retired = (m.retired_count() - r0) as f64;
    let c = u32at(&mut m, CTRL_TICKS).wrapping_sub(c0) as f64;
    let q = u32at(&mut m, SENSOR_SEQ).wrapping_sub(q0) as f64 / 2.0;
    println!(
        "[rates-light] 窗口 {fw:.0}ms | 控制 {} 拍 -> {:.2}Hz（周期 {:.3}ms） | sensors {} 轮 -> {:.2}Hz（周期 {:.3}ms） | CPU {:.1}%",
        c as u32,
        c * 1000.0 / fw,
        fw / c,
        q as u32,
        q * 1000.0 / fw,
        fw / q,
        retired * 100.0 / RETIRED_BYTES_PER_MS / fw
    );
}

/// 重载 + **建模真实 USB 主机**（同 x_hil_mcusim：枚举 + 每 ms 取走下行）。
/// 用于判定：flysim 路径下"无 USB 主机→IN 永不完成→usb_tx_pump 忙等"这个脚手架产物
/// 对 CPU 占用与任务频率的影响有多大。
#[test]
fn task_rates_with_usb_host() {
    use mcu_simulater::events::Event;

    let mut m = build_with_uart(true);
    boot(&mut m);

    // USB 总线枚举：复位 + 4 个标准 SETUP
    let setup = |m: &mut Machine, data: [u8; 8]| {
        m.events.lock().unwrap().publish(&Event::UsbSetup { data });
        m.run_budget(60_000).unwrap();
    };
    m.usb_otg.lock().unwrap().inject_usb_reset();
    m.run_budget(60_000).unwrap();
    setup(&mut m, [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]);
    setup(&mut m, [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x20, 0x00]);
    setup(&mut m, [0x00, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00]);
    setup(&mut m, [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
    m.run_budget(200_000).unwrap();

    const MS: u32 = 3000;
    let t0 = systick(&m);
    let r0 = m.retired_count();
    let c0 = u32at(&mut m, CTRL_TICKS);
    let q0 = u32at(&mut m, SENSOR_SEQ);
    let mut drained_total = 0usize;
    while ((systick(&m) - t0) as u32) < MS {
        m.run_ms(1.0).unwrap();
        // 主机侧取走下行（CDC 数据端点 EP1 IN）
        let data = m.usb_otg.lock().unwrap().host_take_in(1);
        drained_total += data.len();
    }
    let fw = (systick(&m) - t0) as f64;
    let retired = (m.retired_count() - r0) as f64;
    let c = u32at(&mut m, CTRL_TICKS).wrapping_sub(c0) as f64;
    let q = u32at(&mut m, SENSOR_SEQ).wrapping_sub(q0) as f64 / 2.0;
    println!(
        "[rates-usbhost] 窗口 {fw:.0}ms | 控制 {} 拍 -> {:.2}Hz（{:.3}ms） | sensors {} 轮 -> {:.2}Hz（{:.3}ms） | CPU {:.1}% | 下行取走 {} 字节",
        c as u32,
        c * 1000.0 / fw,
        fw / c,
        q as u32,
        q * 1000.0 / fw,
        fw / q,
        retired * 100.0 / RETIRED_BYTES_PER_MS / fw,
        drained_total
    );
}
