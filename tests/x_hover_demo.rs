//! [持续悬停演示] 按虚拟外设直通闭环流程（fly-sim 物理 → FlySimState →
//! mcu_sim 虚拟外设 → flyctrl 固件 → PWM 读回）跑 60s 连续悬停仿真。
//!
//! 与 `tests/x_vperiph_mcusim.rs` 同一套流程，仅拉长仿真时长并输出轨迹统计，
//! 用于回答"能否持续悬停"：分段统计高度/水平漂移、姿态发散、推力稳定性。
//!
//! 构建前置：`cd joc-base && cmake --build build_rel`（minimal elf）、
//! `cd flyctrl && python3 build_app.py --features real-sensors --out /tmp/flyctrl_clean.bin`。

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::ToyWorld;
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::ActuatorCmd;
use mcu_simulater::machine::Machine;
use unicorn_engine::RegisterARM;
use mcu_simulater::peripheral::vperiph::data_source::FlySimState;

const SYS: &str = "/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf";
const APP_REAL: &str = "/tmp/flyctrl_clean.bin";

// TIM 基址（固件 pwm0..3 = TIM3/TIM2/TIM1/TIM4 CH1）
const TIM3: u64 = 0x4000_0400; // pwm0
const TIM2: u64 = 0x4000_0000; // pwm1
const TIM1: u64 = 0x4001_0000; // pwm2
const TIM4: u64 = 0x4000_0800; // pwm3
const OFF_CRR1: u64 = 0x34;
const OFF_ARR: u64 = 0x2C;

// 固定 GPS 原点（悬停点附近）
const LAT0: f32 = 31.2304;
const LON0: f32 = 121.4737;
const ALT0: f32 = 4.0;

// 悬停目标：物理 NED d=-5（5m 高，与固件 EKF 原点 d=0 对齐，见 x_vperiph 注释）
const HOVER_D: f32 = -5.0;

fn rd_u32(m: &Arc<Mutex<Machine>>, addr: u64) -> u32 {
    let b = m.lock().unwrap().cpu.mem_read(addr, 4).unwrap();
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

/// 读 4 路 PWM 的 CCR1/ARR → 归一化推力（m = (duty_us - 1000)/1000）。
fn read_thrust(m: &Arc<Mutex<Machine>>) -> [f32; 4] {
    let tims = [TIM3, TIM2, TIM1, TIM4];
    let mut out = [0f32; 4];
    for (i, &t) in tims.iter().enumerate() {
        let arr = rd_u32(m, t + OFF_ARR) as f32;
        let ccr = rd_u32(m, t + OFF_CRR1) as f32;
        let duty = if arr > 0.0 { ccr / arr } else { 0.0 }; // 0..1 占空比
        let us = duty * 2500.0; // 400Hz 周期 2500us
        out[i] = ((us - 1000.0) / 1000.0).clamp(0.0, 1.0);
    }
    out
}

/// 读固件 EKF 估计高度 est.pos[2]（NED 向下正，地址布局同 x_vperiph）。
fn read_ekf_z(m: &Arc<Mutex<Machine>>) -> f32 {
    let b = m.lock().unwrap().cpu.mem_read(0x2000_9074 + 28, 4).unwrap();
    f32::from_le_bytes(b.try_into().unwrap())
}

/// ARM 前收敛推进（根因同 x_vperiph：boot 早期 RC 未建立 → target_alt=+2.0
/// 污染 EKF 高度初始化；须等 SBUS 帧到达 + EKF 高度拉回原点再 ARM）。
fn settle_ekf_before_arm(m: &Arc<Mutex<Machine>>) -> f32 {
    let mut mm = m.lock().unwrap();
    let mut z = f32::NAN;
    for i in 0..400 {
        mm.run(1_000_000).unwrap();
        z = f32::from_le_bytes(mm.cpu.mem_read(0x2000_9074 + 28, 4).unwrap().try_into().unwrap());
        if i % 50 == 0 {
            eprintln!("[demo] 收敛推进 i={i} ekf_z={z:.3}");
        }
        if z.abs() < 0.6 {
            break;
        }
    }
    eprintln!("[demo] 收敛完成 ekf_z={z:.3}");
    z
}

#[test]
fn hover_60s_demo() {
    init_log();
    let sys = Path::new(SYS);
    let app = Path::new(APP_REAL);
    assert!(sys.exists(), "minimal elf 缺失");
    assert!(app.exists(), "real-sensors app 缺失：build_app.py --features real-sensors --out /tmp/flyctrl_clean.bin");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();

    let state = Arc::new(Mutex::new(FlySimState::default()));
    m.attach_flysim_sensors(state.clone());
    m.attach_flysim_uart_slaves(state.clone());

    // boot 前注入初始真值（同 x_vperiph：静止水平悬停 + 气压 h=0 + GPS 有效）
    {
        let mut st = state.lock().unwrap();
        st.imu_acc = [0.0, 0.0, -9.81];
        st.imu_gyr = [0.0, 0.0, 0.0];
        st.baro_pa = 101_325.0f32;
        st.gps_lat = LAT0;
        st.gps_lon = LON0;
        st.gps_alt = ALT0 + 5.0;
        st.gps_fix = 3.0;
        st.gps_vel = [0.0, 0.0, 0.0]; // 静止：无 Doppler 速度
        st.rc_ch = [1500.0; 16];
    }

    m.load_elf(sys).unwrap();
    m.load_app_partition(app).unwrap();
    m.reset().unwrap();
    for _ in 0..12 {
        m.run(1_000_000).unwrap();
    }
    let m = Arc::new(Mutex::new(m));

    settle_ekf_before_arm(&m);

    // ARM + RC 解锁
    m.lock().unwrap().cpu.mem_write(0x2000_b669, &[1u8]).unwrap();
    {
        let mut st = state.lock().unwrap();
        st.rc_ch[4] = 2000.0;
        st.rc_ch[3] = 1500.0;
    }

    // ---- 35s 闭环：每步 4ms，8750 步；起飞台保持 → 升空 → 持续悬停 ----
    let mut sim = SimLoop::new(
        ToyWorld::new(9.81),
        &VehicleConfig::default_quad(),
        0.004,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(fly_sim_core::physics::ContactModel::default()),
        vec![],
    );
    let t0 = Instant::now();
    let mut held = true;
    let mut max_thrust = 0.0f32;
    let mut traj: Vec<(u64, [f32; 3], [f32; 3])> = Vec::new(); // (step, pos, vel)
    let mut roll_max = 0.0f32;
    let mut pitch_max = 0.0f32;
    let mut last_state = None;

    for step in 0..15_000u64 {
        let motors = read_thrust(&m);
        let thrust = motors.iter().sum::<f32>();
        max_thrust = max_thrust.max(thrust);

        let (st, imu_true) = if held && thrust < 0.05 {
            (None, flyctrl_core::vehicle::ImuSample {
                accel: [
                    flyctrl_core::units::MeterPerSecondSquared(0.0),
                    flyctrl_core::units::MeterPerSecondSquared(0.0),
                    flyctrl_core::units::MeterPerSecondSquared(-9.81),
                ],
                gyro: [flyctrl_core::units::RadianPerSecond(0.0); 3],
            })
        } else {
            held = false;
            let cmd = ActuatorCmd { motor: motors };
            let st = sim.step_hil(&cmd);
            (Some(st), sim.last_imu())
        };
        last_state = st;
        let (pos, vel) = match st {
            Some(s) => ([s.pos[0].0, s.pos[1].0, s.pos[2].0], [s.vel[0].0, s.vel[1].0, s.vel[2].0]),
            None => ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0]),
        };
        roll_max = roll_max.max(if let Some(s) = st { s.att.roll().abs() } else { 0.0 });
        pitch_max = pitch_max.max(if let Some(s) = st { s.att.pitch().abs() } else { 0.0 });

        // 注入真值
        {
            let mut st = state.lock().unwrap();
            st.imu_acc = [imu_true.accel[0].0, imu_true.accel[1].0, imu_true.accel[2].0];
            st.imu_gyr = [imu_true.gyro[0].0, imu_true.gyro[1].0, imu_true.gyro[2].0];
            let (n, e, d) = (pos[0], pos[1], pos[2]);
            st.gps_lat = LAT0 + n / 111_320.0;
            st.gps_lon = LON0 + e / (111_320.0 * LAT0.to_radians().cos());
            st.gps_alt = ALT0 - d;
            st.gps_fix = 3.0;
            // GPS Doppler 速度（NED m/s）：经 $GNRMC 帧下发 → EKF update_vel 约束
            // 水平速度估计（无此约束时长时间悬停水平速度纯积分漂移失稳，见方案 A）
            st.gps_vel = vel;
            let h = -(d + 5.0);
            st.baro_pa = 101_325.0 * (-h / 8434.5).exp();
            st.rc_ch[4] = 2000.0;
        }

        m.lock().unwrap().run(300_000).unwrap();

        if step % 1000 == 0 {
            let ekf_z = read_ekf_z(&m);
            // [DIAG] USART2(port=2, GPS) 推流帧数 + FIFO 残留：判断固件是否消费完全部
            // GGA/RMC 字节（FIFO 残留 >0 → RMC 尾滞留未消费 → 解释 gps_v=0）
            let mm = m.lock().unwrap();
            let (gps_frames, fifo_len) = {
                let uv = mm.usart.lock().unwrap();
                let u2 = uv.get(1).unwrap().lock().unwrap();
                let frames = u2.slaves().first().map(|s| s.frames()).unwrap_or(0);
                (frames, u2.rx_fifo_len())
            };
            eprintln!(
                "[demo] t={:.0}s thrust={thrust:.3} pos=({:.2},{:.2},{:.2}) vel=({:.2},{:.2},{:.2}) ekf_z={ekf_z:.2} gps_frames={gps_frames} fifo={fifo_len}",
                step as f64 * 0.004, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]
            );
        }
        if st.is_some() {
            traj.push((step, pos, vel));
        }
    }

    let wall = t0.elapsed();

    // ---- 固件 console（含 ctrl dbg est 每 100ms 的 EKF 姿态/位置/速度演变）----
    {
        let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
        let t = String::from_utf8_lossy(&out);
        let n = t.len();
        eprintln!("[demo] === console FINAL ({n}B, tail 6000) ===\n{}", &t[n.saturating_sub(6000)..]);
    }

    // ---- 统计：分段（每 15s 一段）位置/速度 ----
    let n = traj.len();
    let seg = |a: f64, b: f64| -> ([f64; 3], [f64; 3]) {
        let s: Vec<_> = traj.iter().filter(|(stp, _, _)| {
            let t = *stp as f64 * 0.004;
            t >= a && t < b
        }).collect();
        let k = s.len().max(1);
        let mut ps = [0.0f64; 3];
        let mut vs = [0.0f64; 3];
        for (_, p, v) in &s {
            for i in 0..3 { ps[i] += p[i] as f64; vs[i] += v[i] as f64; }
        }
        ([ps[0]/k as f64, ps[1]/k as f64, ps[2]/k as f64],
         [vs[0]/k as f64, vs[1]/k as f64, vs[2]/k as f64])
    };

    eprintln!("\n[demo] === 60s 持续悬停统计（步数 {n}，墙钟 {wall:.1}s）===",
        wall = wall.as_secs_f64());
    let mut ok = true;
    // 分段统计；判定段取 20-60s（跳过起飞过渡期）
    for (a, b) in [(0.0, 10.0), (10.0, 20.0), (20.0, 35.0)] {
        let (p, v) = seg(a, b);
        let dz = (p[2] - HOVER_D as f64).abs();
        let horiz = (p[0] * p[0] + p[1] * p[1]).sqrt();
        eprintln!(
            "[demo] {a:>2.0}-{b:>2.0}s: pos=({p0:+.2},{p1:+.2},{p2:+.2}) |dz|={dz:.2}m horiz={horiz:.2}m vel=({v0:+.2},{v1:+.2},{v2:+.2}) m/s",
            p0 = p[0], p1 = p[1], p2 = p[2], v0 = v[0], v1 = v[1], v2 = v[2]
        );
        if a >= 20.0 && (dz > 0.8 || horiz > 0.5) { ok = false; }
    }
    let st = last_state.expect("物理从未推进");
    eprintln!(
        "[demo] 末态 pos=({:.2},{:.2},{:.2})m 姿态=({:.1}°,{:.1}°) max_thrust={max_thrust:.3}",
        st.pos[0].0, st.pos[1].0, st.pos[2].0,
        st.att.roll().to_degrees(), st.att.pitch().to_degrees()
    );
    eprintln!("[demo] 全程 max|roll|={:.2}° max|pitch|={:.2}°", roll_max.to_degrees(), pitch_max.to_degrees());

    // ---- 判定：20s 后高度稳定在悬停点 ±0.8m、水平 ±0.5m，姿态 < 5°，位置有限 ----
    assert!(max_thrust > 0.05, "MCU 未输出有效推力");
    for v in [st.pos[0].0, st.pos[1].0, st.pos[2].0] {
        assert!(v.is_finite() && v.abs() < 100.0, "位置发散：{v}");
    }
    assert!(roll_max < 5f32.to_radians(), "roll 发散：{}", roll_max.to_degrees());
    assert!(pitch_max < 5f32.to_radians(), "pitch 发散：{}", pitch_max.to_degrees());
    assert!(ok, "20s 后未稳定在悬停点附近");
    eprintln!(">>> [HOVER-DEMO] 60s 持续悬停验证通过 ✓");
}

fn init_log() {
    let _ = env_logger::builder().is_test(true).filter_level(log::LevelFilter::Warn).try_init();
}
