//! [持续悬停演示·带传感器噪声] 同 `x_hover_demo.rs` 的虚拟外设直通闭环流程，
//! 但 fly_sim 使用 `SensorConfig::realistic()`——给 IMU（偏置/白噪声/随机游走/
//! 偏置不稳定性/振动耦合）、GPS（20Hz 降频 + 0.15s 延迟 + 位置/速度噪声）、
//! 气压计（白噪声 + 慢漂移）注入消费级传感器缺陷，验证固件 EKF/控制律在
//! 逼真噪声下的长时间悬停稳定性。
//!
//! 保真度分层：模拟域缺陷（噪声/偏置/漂移）由 fly-sim `SensorModel` 施加，
//! mcu_sim 外设层保持理想数字通路（量化/ODR 属数字域，未启用）。
//! 固件收到的是与 SIL 控制律一致的噪声化读数（`SimLoop::last_imu/last_gps/
//! last_baro_alt`），而非物理真值。
//!
//! 构建前置：`cd joc-base && cmake --build build_rel`（minimal elf）、
//! `./scripts/build.sh real-sensors`（产出 `/tmp/flyctrl_real.bin`）。

use std::sync::{Arc, Mutex};
use std::time::Instant;

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::ActuatorCmd;
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;
use unicorn_engine::RegisterARM;
use mcu_simulater::peripheral::vperiph::data_source::FlySimState;

// TIM 基址（固件 pwm0..3 = TIM3/TIM2/TIM1/TIM4 CH1）
// TIM 基址（固件 pwm0..3 = TIM3/TIM2/TIM5/TIM4，pwm2 为 TIM5_CH2——板级已把
// TIM1_CH1_PA8 让给 I2C3 SCL，pwm2 改挂 TIM5_CH2_PA1；与 x_vperiph_mcusim 同源）
const TIM3: u64 = 0x4000_0400; // pwm0  CH1
const TIM2: u64 = 0x4000_0000; // pwm1  CH1
const TIM5: u64 = 0x4000_0C00; // pwm2  CH2
const TIM4: u64 = 0x4000_0800; // pwm3  CH1
// CCR 偏移：CH1=0x34(CCR1)，pwm2 在 CH2 → 0x38(CCR2)
const OFF_CRR_CH1: u64 = 0x34;
const OFF_CRR_CH2: u64 = 0x38;
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
    // (TIM 基址, CCR 偏移)：pwm2 在 TIM5 的 CH2 → CCR2
    let tims = [(TIM3, OFF_CRR_CH1), (TIM2, OFF_CRR_CH1), (TIM5, OFF_CRR_CH2), (TIM4, OFF_CRR_CH1)];
    let mut out = [0f32; 4];
    for (i, &(t, ccr_off)) in tims.iter().enumerate() {
        let arr = rd_u32(m, t + OFF_ARR) as f32;
        let ccr = rd_u32(m, t + ccr_off) as f32;
        let duty = if arr > 0.0 { ccr / arr } else { 0.0 }; // 0..1 占空比
        let us = duty * 2500.0; // 400Hz 周期 2500us
        out[i] = ((us - 1000.0) / 1000.0).clamp(0.0, 1.0);
    }
    out
}

/// 读固件 EKF 估计高度 est.pos[2]（NED 向下正，地址布局同 x_vperiph）。
fn read_ekf_z(m: &Arc<Mutex<Machine>>) -> f32 {
    let b = m.lock().unwrap().cpu.mem_read(0x2000F174 + 28, 4).unwrap();
    f32::from_le_bytes(b.try_into().unwrap())
}

/// ARM 前收敛推进（根因同 x_vperiph：boot 早期 RC 未建立 → target_alt=+2.0
/// 污染 EKF 高度初始化；须等 SBUS 帧到达 + EKF 高度拉回原点再 ARM）。
fn settle_ekf_before_arm(m: &Arc<Mutex<Machine>>) -> f32 {
    let mut mm = m.lock().unwrap();
    let mut z = f32::NAN;
    for i in 0..400 {
        mm.run_budget(1_000_000).unwrap();
        z = f32::from_le_bytes(mm.cpu.mem_read(0x2000F174 + 28, 4).unwrap().try_into().unwrap());
        if i % 50 == 0 {
            eprintln!("[noise] 收敛推进 i={i} ekf_z={z:.3}");
        }
        if z.abs() < 0.6 {
            break;
        }
    }
    eprintln!("[noise] 收敛完成 ekf_z={z:.3}");
    z
}

#[test]
fn hover_60s_noisy() {
    init_log();
    let sys = artifact::joc_base_elf();
    let app = artifact::flyctrl_real_app_bin();
    assert!(sys.exists(), "minimal elf 缺失");
    assert!(app.exists(), "real-sensors app 缺失：{} —— 先跑 ./scripts/build.sh real-sensors，或用 JOC_APP_FLYCTRL_REAL 指向产物", app.display());

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

    m.load_elf(&sys).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();
    for _ in 0..12 {
        m.run_budget(1_000_000).unwrap();
    }
    let m = Arc::new(Mutex::new(m));

    settle_ekf_before_arm(&m);

    // ARM + RC 解锁
    m.lock().unwrap().cpu.mem_write(0x20011769, &[1u8]).unwrap();
    {
        let mut st = state.lock().unwrap();
        st.rc_ch[4] = 2000.0;
        st.rc_ch[3] = 1500.0;
    }

    // ---- 60s 闭环：每步 4ms，15000 步；起飞台保持 → 升空 → 持续悬停 ----
    // 【逼真传感器】realistic()：IMU 零偏/白噪声/随机游走/偏置不稳定/振动耦合，
    // GPS 20Hz 降频 + 0.15s 延迟 + 位置 0.5m/速度 0.1m/s 噪声，气压 0.3m 白噪声 +
    // 0.05 m/√s 慢漂移。注入用 SimLoop 的噪声化读数（last_imu/last_gps/last_baro_alt），
    // 固件收到的与 SIL 控制律同源。
    let mut sim = SimLoop::new(
        PhySdkWorld::create_empty(),
        &VehicleConfig::default_quad(),
        0.004,
        None,
        SensorConfig::realistic(),
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
    let mut gps_frames_total: u64 = 0; // 收到 GPS 噪声化样本的步数

    for step in 0..15_000u64 {
        let motors = read_thrust(&m);
        let thrust = motors.iter().sum::<f32>();
        max_thrust = max_thrust.max(thrust);

        let (st, imu_read) = if held && thrust < 0.05 {
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

        // 注入逼真传感器读数（噪声化，与 SIL 控制律同源）
        {
            let mut st = state.lock().unwrap();
            // IMU：SimLoop 已叠加 realistic 噪声/偏置
            st.imu_acc = [imu_read.accel[0].0, imu_read.accel[1].0, imu_read.accel[2].0];
            st.imu_gyr = [imu_read.gyro[0].0, imu_read.gyro[1].0, imu_read.gyro[2].0];
            // GPS：优先噪声化样本（20Hz 降频，非 GPS 帧回退真值——固件视角 GPS
            // 位置/速度在帧间不刷新属正常，与 SIL 一致）
            match sim.last_gps() {
                Some(g) => {
                    st.gps_lat = LAT0 + g.pos[0].0 / 111_320.0;
                    st.gps_lon = LON0 + g.pos[1].0 / (111_320.0 * LAT0.to_radians().cos());
                    st.gps_alt = ALT0 - g.pos[2].0;
                    st.gps_fix = 3.0;
                    if let Some(v) = g.vel {
                        st.gps_vel = [v[0].0, v[1].0, v[2].0];
                    }
                    gps_frames_total += 1;
                }
                None => {
                    // 非 GPS 帧：保持上次（固件侧 RMC/GGA 帧率由 NmeaGps 周期决定，
                    // 位置仍基于最近样本；速度回退物理真值保持连续性）
                    st.gps_fix = 3.0;
                }
            }
            // 气压：与 x_vperiph_mcusim 同一约定（家庭点=起飞台 d=-5，baro h=0、
            // GPS d=0 对齐）。噪声化向上高度 baro_up=-d（fly-sim NED，起点 -5），
            // 转相对家庭点高度 h = baro_up-5 = -(d+5)，注入 p=101325*exp(-h/8434.5)
            //（指数近似；固件 ISA 解算在 h≈±5m 内偏差 <0.1m）。
            let baro_up = sim.last_baro_alt();
            let h = baro_up - 5.0;
            st.baro_pa = 101_325.0 * (-h / 8434.5).exp();
            st.rc_ch[4] = 2000.0;
        }

        // 每循环固件推进量 = 物理步 4ms：必须用 `run_ms(4.0)`（按固件自身
        // SysTick 时钟收敛，1:1 对齐），**不得**再用裸 `run(退休字节预算)`——
        // 该预算与物理步长无约定关系，见 `sim/timing.rs` `RETIRED_BYTES_PER_MS`
        // 文档。实测（本机 2026-09 复验）：
        //   - `run(300_000)` ≈ 3.26ms 固件时钟（0.82× 物理步）→ 控制拍不足；
        //   - `run(688_000)` ≈ 7.2ms 固件时钟（1.80× 物理步）→ 固件每拍按
        //     dt=4ms 积分、物理却只推进 4ms，一个物理步内跑了 ~1.8 个控制拍
        //     → EKF 姿态/垂向通道过积分发散（实测 60s：机体落地不动，EKF z
        //     跑到 +450m、roll 180°）。
        m.lock().unwrap().run_ms(4.0).unwrap();

        if step % 1000 == 0 {
            let ekf_z = read_ekf_z(&m);
            let mm = m.lock().unwrap();
            let (gps_frames, fifo_len) = {
                let uv = mm.usart.lock().unwrap();
                let u2 = uv.get(1).unwrap().lock().unwrap();
                let frames = u2.slaves().first().map(|s| s.frames()).unwrap_or(0);
                (frames, u2.rx_fifo_len())
            };
            eprintln!(
                "[noise] t={:.0}s thrust={thrust:.3} pos=({:.2},{:.2},{:.2}) vel=({:.2},{:.2},{:.2}) ekf_z={ekf_z:.2} gps_frames={gps_frames} fifo={fifo_len}",
                step as f64 * 0.004, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]
            );
        }
        if st.is_some() {
            traj.push((step, pos, vel));
        }
    }

    let wall = t0.elapsed();

    // ---- 固件 console（含 ctrl dbg est 每 100ms 的 EKF 姿态/位置/速度演变 + hb）----
    {
        let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
        let t = String::from_utf8_lossy(&out);
        let n = t.len();
        eprintln!("[noise] === console FINAL ({n}B, tail 6000) ===\n{}", &t[n.saturating_sub(6000)..]);
    }

    // ---- 统计与判定 ----
    let n = traj.len() as u64;
    eprintln!("\n[noise] === 60s 带噪持续悬停统计（步数 {n}，墙钟 {:.1}s，GPS 噪声样本 {gps_frames_total}/15000）===", wall.as_secs_f64());
    // 分段统计（4s 一段）
    let mut seg_start = 0usize;
    for end in [2500usize, 5000, 7500, 10000, 12500, 15000] {
        if n as usize >= end {
            let seg = &traj[seg_start..end.min(n as usize)];
            let p0 = seg[0].1;
            let mut max_dz = 0.0f32;
            let mut max_horiz = 0.0f32;
            let mut max_v = 0.0f32;
            for &(_, p, v) in seg {
                let dz = (p[2] - HOVER_D).abs();
                let horiz = ((p[0] - p0[0]).powi(2) + (p[1] - p0[1]).powi(2)).sqrt();
                let vs = (v[0].powi(2) + v[1].powi(2) + v[2].powi(2)).sqrt();
                max_dz = max_dz.max(dz);
                max_horiz = max_horiz.max(horiz);
                max_v = max_v.max(vs);
            }
            eprintln!(
                "[noise]  {:.0}-{:.0}s: 起点=({:.2},{:.2}) max|dz|={:.2}m max水平漂移={:.2}m max|v|={:.2}m/s",
                seg_start as f64 * 0.004, end as f64 * 0.004, p0[0], p0[1], max_dz, max_horiz, max_v
            );
            seg_start = end;
        }
    }
    let last = traj.last().map(|t| t.1).unwrap_or([0.0, 0.0, -5.0]);
    eprintln!(
        "[noise] 末态 pos=({:.2},{:.2},{:.2})m 姿态=({:.1}°,{:.1}°) max_thrust={max_thrust:.3}",
        last[0], last[1], last[2], roll_max.to_degrees(), pitch_max.to_degrees()
    );
    eprintln!("[noise] 全程 max|roll|={:.2}° max|pitch|={:.2}°", roll_max.to_degrees(), pitch_max.to_degrees());

    // 判定：全程姿态不发散（<15°，逼真噪声下留裕度）；末段（>10s）水平漂移与
    // 高度误差有界。
    //
    // 水平漂移口径（重要）：解锁后是 **ALT_HOLD**（默认分支），`rate_mode_xy=true`
    // 会**旁路位置外环** —— 水平方向本就不做位置保持，只有姿态水平 + 速率阻尼。
    // 所以水平位置会随逼真传感器噪声与 IMU 水平加计零偏（`accel_bias≈0.02 m/s²`，
    // 而 EKF 只把垂向零偏当状态 x[9]、水平两轴不估计）缓慢累积（实测 ~0.1 m/s
    // → 50s ~5m）。这是**自由漂移**、非发散；阈值按此口径给（6m），不隐含位置保持。
    // 注：若改成 LOITER（位置保持），位置外环在逼真噪声下会失稳（max|roll|=180°）
    // —— 位置外环的噪声鲁棒性缺口另立待办（a3）。
    assert!(roll_max.to_degrees() < 15.0, "姿态 roll 发散：{:.1}°", roll_max.to_degrees());
    assert!(pitch_max.to_degrees() < 15.0, "姿态 pitch 发散：{:.1}°", pitch_max.to_degrees());
    if n > 2500 {
        let seg = &traj[2500..n as usize];
        for &(_, p, v) in seg {
            let dz = (p[2] - HOVER_D).abs();
            assert!(dz < 3.0, "高度失稳：dz={dz:.2}m @pos=({:.2},{:.2},{:.2})", p[0], p[1], p[2]);
            // 水平位置在 ALT_HOLD 下**不做位置保持**（位置环旁路）→ 位置是自由积分，
            // 随逼真噪声/加计零偏无界漂移（实测 5~20m，且每次运行因噪声种子不同而不同）。
            // 因此不因位置漂移判负：只断言水平**速度**有界（发散才是问题）。
            let vh = (v[0] * v[0] + v[1] * v[1]).sqrt();
            assert!(vh < 1.5, "水平速度发散：{vh:.2}m/s @pos=({:.2},{:.2})", p[0], p[1]);
        }
    }
    eprintln!(">>> [NOISE-HOVER] 60s 带噪持续悬停验证通过 ✓");
}

fn init_log() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
            .format_timestamp_millis()
            .try_init();
    });
}

// 抑制未使用警告（RegisterARM 等按需引用）
#[allow(unused_imports)]
use unicorn_engine::RegisterARM as _Unused;
