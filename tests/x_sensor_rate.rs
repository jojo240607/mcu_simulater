//! [闭环时钟口径] 固件侧推进与物理步长的对齐校验 + sensors 采样率量测。
//!
//! 背景（闭环测试真实性问题）：`run(count)` 的 `count` 是"退休字节"，与物理步长
//! 无约定关系；固件内 EKF 的积分步长却是**编译期常量 dt=4ms**（与真机 250Hz
//! 标称一致）。若每步物理推进 4ms、而固件侧只推进 3.26ms（旧测试预算
//! `run(300_000)` 的实测值），则固件任务周期与 EKF 的 dt 都相对物理步长系统性
//! 失配 1.23×——EKF 加速度积分少算 → 垂向速度估计滞后 → 定高环阻尼相位偏移 →
//! 悬停慢漂（SIL 同控制律下 est≡真值，反证根因在时钟口径而非控制律）。
//!
//! 本测试以传感器任务（prio=5）的 `SENSOR_SEQ` seqlock 计数与固件 SysTick
//! （vector 15，1ms/拍）为标尺，断言 `Machine::run_ms(4.0)` 下：
//!   1) 固件自身时钟推进与场景时间 **1:1**（比值 1.0 ± 5%）；
//!   2) sensors 采样率落在实测区间（当前 ~255Hz，名义 500Hz 受 CPU 总负载限制，
//!      见 `src/machine/mod.rs` DMA 待搬运检查处的实测记录）。
//!
//! 注：采样率上限由 CPU 总负载（5 路驱动状态机 + control 任务 EKF 同核抢占）
//! 决定，与 DMA 待搬运检查间隔无关（256/16/4 三档实测均 254.5Hz）。
use std::sync::{Arc, Mutex};

use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::vperiph::data_source::FlySimState;

/// 固件 `SENSOR_SEQ` 地址——从 app.elf 符号表解析（见 mcu_simulater::elfsym）。
/// 不硬编码：`.app_globals` 段内符号顺序随固件代码变化，硬编码会在固件改动后
/// 静默读到垃圾（表现为"采样率 0Hz"这类假失败）。
fn sensor_seq_addr() -> u64 {
    mcu_simulater::elfsym::app_sym("SENSOR_SEQ") as u64
}

fn systick(m: &Machine) -> u64 {
    m.vec_entries()
        .iter()
        .find(|(v, _)| *v == 15)
        .map(|(_, n)| *n)
        .unwrap_or(0)
}

fn seq(m: &mut Machine) -> u32 {
    let b = m.cpu.mem_read(sensor_seq_addr(), 4).unwrap_or_else(|_| vec![0; 4]);
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

#[test]
fn sensor_rate_and_fw_clock_alignment() {
    let sys = artifact::joc_base_elf();
    let app = artifact::flyctrl_real_app_bin();
    assert!(sys.exists() && app.exists());
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
    // boot + 收敛（与闭环测试同口径）
    for _ in 0..12 {
        m.run_budget(1_000_000).unwrap();
    }
    for _ in 0..60 {
        m.run_budget(1_000_000).unwrap();
    }

    // 采样窗口：40 步 × 4ms = 160ms 场景
    let s0 = seq(&mut m);
    let r0 = m.retired_count();
    let t0 = systick(&m);
    const STEPS: u32 = 120;
    for _ in 0..STEPS {
        m.run_ms(4.0).unwrap();
    }
    let s1 = seq(&mut m);
    let r1 = m.retired_count();
    let t1 = systick(&m);
    let loops = (s1.wrapping_sub(s0)) as f64 / 2.0; // SENSOR_SEQ 每 loop +2
    let scene_s = STEPS as f64 * 0.004;
    let fw_ms = (r1 - r0) as f64 / 92_000.0;
    let dtick = (t1 - t0) as f64;
    println!(
        "[rate] {STEPS} 步 ×4ms: sensor loops={loops:.1} → {:.1} Hz(场景) {:.1} Hz(固件) | 固件时钟 {dtick:.0}ms/场景 {scene_ms:.0}ms (比值 {ratio:.3}) | retired/step={rps} | retired/SysTick={per_tick:.0}",
        loops / scene_s,
        loops / (dtick / 1000.0),
        scene_ms = scene_s * 1000.0,
        ratio = dtick / (scene_s * 1000.0),
        rps = (r1 - r0) / STEPS as u64,
        per_tick = (r1 - r0) as f64 / dtick.max(1.0),
    );

    // 1) 固件时钟必须与场景 1:1（run_ms 的 SysTick 对齐不变量）
    let ratio = dtick / (scene_s * 1000.0);
    assert!(
        (0.95..=1.05).contains(&ratio),
        "固件时钟与场景时间失配：{dtick:.0}ms 固件 / {:.0}ms 场景（比值 {ratio:.3}，期望 1.0±5%）——         检查 run_ms 的 SysTick 对齐是否被破坏",
        scene_s * 1000.0
    );
    // 2) 采样率落在实测区间（宽带下限防回归；上限由 CPU 总负载决定）
    let rate = loops / scene_s;
    assert!(
        (200.0..=500.0).contains(&rate),
        "sensors 采样率异常：{rate:.1}Hz（期望 200~500Hz，实测稳态 ~255Hz）"
    );
}
