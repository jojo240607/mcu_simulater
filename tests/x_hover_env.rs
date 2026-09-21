//! [持续悬停演示·环境逼真版] 同 `x_hover_noise.rs` 的虚拟外设直通闭环流程，
//! 叠加**全部可用真实化因素**，逼近真实户外使用场景：
//!
//! 1. 传感器噪声/偏置/漂移（`SensorConfig::realistic()`）：
//!    IMU 零偏/白噪声/随机游走/偏置不稳定性（≈温漂慢变）/振动耦合；
//!    GPS 20Hz 降频 + 0.15s 延迟 + 位置 0.5m/速度 0.1m/s 噪声 + **丢星 1%**；
//!    气压 0.3m 白噪声 + 0.05m/√s 慢漂移（≈气压温漂）。
//! 2. 风场（`WindConfig`）：恒定侧风 2.5m/s + 阵风 1.2m/s@0.12Hz + Dryden
//!    湍流 0.3m/s + 风切变（α=0.2 @10m）+ 空间相关风。
//! 3. 温和温漂（`SensorFault::GyroDrift/AccelDrift`）：陀螺/加计偏置缓变
//!    漂移率（60s 累计 <0.01 rad/s、<0.015 m/s²，模拟器件温漂慢变）。
//! 4. 电池压降（`battery_r` 内阻）已内置：悬停负载下电压缓降、推力略降。
//!
//! 目标：验证固件 EKF/控制律在**全场景真实化**下的长时间悬停稳定性——
//! 姿态不发散、位置不漂移失控（风下允许恒定偏置漂移，但需有界）。
//!
//! 保真度分层：模拟域缺陷（噪声/偏置/漂移/风）由 fly-sim `SensorModel`/
//! `WindField` 施加，mcu_sim 外设层保持理想数字通路。
//! 固件收到与 SIL 控制律同源的噪声化读数（`SimLoop::last_*`）。
//!
//! 构建前置：`cd joc-base && cmake --build build_rel`（minimal elf）、
//! `./scripts/build.sh real-sensors`（产出 `/tmp/flyctrl_real.bin`）。

use std::sync::{Arc, Mutex};
use std::time::Instant;

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sensor::{SensorConfig, SensorFault};
use fly_sim_core::sim::SimLoop;
use fly_sim_core::wind::{WindConfig, WindField};
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::ActuatorCmd;
use mcu_simulater::artifact;
use mcu_simulater::clock::run_one_control_tick;
use mcu_simulater::machine::Machine;

/// 从 app.elf 符号表解析固件全局地址（不硬编码：`.app_globals` 段内符号顺序随固件
/// 代码变化，硬编码会在固件改动后静默读错 → "假失败"）。见 `mcu_simulater::elfsym`。
fn sym(name: &str) -> u64 {
    mcu_simulater::elfsym::app_sym(name) as u64
}
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
    let b = m.lock().unwrap().cpu.mem_read(sym("EST_STATE") - 16 + 28, 4).unwrap();
    f32::from_le_bytes(b.try_into().unwrap())
}

/// ARM 前收敛推进（根因同 x_vperiph：boot 早期 RC 未建立 → target_alt=+2.0
/// 污染 EKF 高度初始化；须等 SBUS 帧到达 + EKF 高度拉回原点再 ARM）。
fn settle_ekf_before_arm(m: &Arc<Mutex<Machine>>) -> f32 {
    let mut mm = m.lock().unwrap();
    let mut z = f32::NAN;
    for i in 0..400 {
        mm.run_budget(1_000_000).unwrap();
        z = f32::from_le_bytes(mm.cpu.mem_read(sym("EST_STATE") - 16 + 28, 4).unwrap().try_into().unwrap());
        if i % 50 == 0 {
            eprintln!("[env] 收敛推进 i={i} ekf_z={z:.3}");
        }
        if z.abs() < 0.6 {
            break;
        }
    }
    eprintln!("[env] 收敛完成 ekf_z={z:.3}");
    z
}

#[test]
fn hover_60s_env() {
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

    // 【一期】解锁只走 RC（原"地面站 ARM 注入"依赖 USB 上行，一期已用 usb-link 关闭）
    {
        let mut st = state.lock().unwrap();
        st.rc_ch[4] = 2000.0;
        st.rc_ch[3] = 1500.0;
    }

    // ---- 60s 闭环：每步 4ms，15000 步；起飞台保持 → 升空 → 持续悬停 ----
    // 【全场景真实化】
    // - 传感器：realistic() + GPS 丢星 1%（gps_drop_prob）——IMU 零偏/白噪声/
    //   随机游走/偏置不稳定/振动耦合，GPS 20Hz 降频 + 0.15s 延迟 + 位置 0.5m/
    //   速度 0.1m/s 噪声，气压 0.3m 白噪声 + 0.05 m/√s 慢漂移
    // - 风场：恒定侧风 + 阵风 + Dryden 湍流 + 风切变 + 空间相关
    // - 温漂：陀螺/加计偏置缓变漂移（SensorFault 软故障）
    // - 电池压降：battery_r 内阻（VehicleConfig 内置，悬停负载自动压降）
    let mut sensor_cfg = SensorConfig::realistic();
    sensor_cfg.gps_drop_prob = 0.01; // 1% 丢星：偶发 GPS 无样本，考验 FDIR/位置环
    let wind = WindField::new(WindConfig {
        // NED 风 (n,e,d) → UP (x,y,z) = [n, -d, -e]：北 2.5 + 东 1.0 m/s 斜向侧风
        base: [2.5, 0.0, -1.0],
        // 阵风：北向 1.2 m/s 峰值 @ 0.12Hz（约 8s 周期）
        gust_amp: [1.2, 0.0, 0.0],
        gust_freq: 0.12,
        // Dryden 湍流：水平 0.3 m/s σ，垂直 0.1 m/s σ，τ=0.5s
        turb_sigma: [0.3, 0.1, -0.3],
        turb_tau: 0.5,
        seed: 0x1234_5678,
        // 风切变：幂律 α=0.2 @10m 参考（5m 悬停处 ≈ 0.87×base）
        shear_exponent: 0.2,
        shear_ref_height: 10.0,
        // 空间相关风：尺度 2m（机身不同部位风速差异）
        spatial_scale: 2.0,
        ..WindConfig::default()
    });
    let mut sim = SimLoop::new(
        PhySdkWorld::create_empty(),
        &VehicleConfig::default_quad(),
        0.004,
        Some(wind),
        sensor_cfg,
        ControllerKind::Pid,
        Some(fly_sim_core::physics::ContactModel::default()),
        vec![],
    );
    // 温和温漂（缓变漂移率）：60s 累计陀螺偏置 ~0.006 rad/s、加计偏置 ~0.012 m/s²
    sim.inject_sensor_fault(SensorFault::GyroDrift([0.0001, -0.00005, 0.0001]));
    sim.inject_sensor_fault(SensorFault::AccelDrift([0.0002, 0.0, 0.0002]));
    let t0 = Instant::now();
    let mut held = true;
    let mut max_thrust = 0.0f32;
    let mut traj: Vec<(u64, [f32; 3], [f32; 3])> = Vec::new(); // (step, pos, vel)
    let mut fw_ms0: Option<u64> = None;
    let mut roll_max = 0.0f32;
    let mut pitch_max = 0.0f32;
    let mut last_state = None;
    let mut gps_frames_total: u64 = 0; // 收到 GPS 噪声化样本的步数

    for step in 0..15_000u64 {
        // ---- 锁相步进（2026-09-21）----
        // 先推进固件**直到恰好完成一拍控制**，再读 PWM。
        // 为何：控制拍周期实测抖动 std 0.84ms（均值恰好 4.0000ms，根因是
        // `delay_until` 唤醒量化）；固定 `run_ms(4.0)` 推进会让 **PWM 回读的
        // 相位自由漂移** ⇒ 任何代码改动（哪怕加一条不执行的支路）都会改变
        // 量化图案 → 改变相位 → 改变结果。这就是 M 场“代码布局灵敏度”的根因，
        // **与算力余量无关**（实测计算仅占 26%）。详见 `clock::run_one_control_tick`。
        {
            let mut mm = m.lock().unwrap();
            run_one_control_tick(&mut mm).unwrap();
            // 锁相基准：取开机后的**第一拍末**为 0 点（开机到首拍有 ~200ms 初始化）。
            if fw_ms0.is_none() {
                fw_ms0 = Some(mm.systick_ms());
            }
        }
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

        // 固件推进已移至上方的 `run_one_control_tick`（锁相）。
        // 旧写法：`m.lock().unwrap().run_ms(4.0)` —— 固定 4ms 推进，相位自由漂移；
        // 更早是裸 `run(300_000)`（实测仅 ≈3.3ms 固件时间 < 4ms 物理步 → 0.83× 失配）。

        if step % 1000 == 0 {
            let ekf_z = read_ekf_z(&m);
            let mm = m.lock().unwrap();
            // 锁相对齐（**增量**口径）：从首拍末起，固件时钟应与“物理已推进的
            // 名义时间” 1:1（实测均值拍 = 4.0000ms）。漂移只会来自“实测均值 ≠ 名义 dt”，
            // 本断言把它变成可见信号。
            let fw_ms = mm.systick_ms();
            let phys_ms = (step as f64) * 4.0;
            let drift = (fw_ms - fw_ms0.unwrap_or(fw_ms)) as f64 - phys_ms;
            let (gps_frames, fifo_len) = {
                let uv = mm.usart.lock().unwrap();
                let u2 = uv.get(1).unwrap().lock().unwrap();
                let frames = u2.slaves().first().map(|s| s.frames()).unwrap_or(0);
                (frames, u2.rx_fifo_len())
            };
            eprintln!(
                "[env] t={:.0}s thrust={thrust:.3} pos=({:.2},{:.2},{:.2}) vel=({:.2},{:.2},{:.2}) ekf_z={ekf_z:.2} gps_frames={gps_frames} fifo={fifo_len} | 锁相漂移={drift:+.1}ms",
                step as f64 * 0.004, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]
            );
            assert!(
                drift.abs() < 100.0,
                "锁相漂移过大（{drift:+.1}ms @ step {step}）：固件拍均值与名义 4ms 不匹配，\
                 双时基又开始漂了"
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
        eprintln!("[env] === console FINAL ({n}B, tail 6000) ===\n{}", &t[n.saturating_sub(6000)..]);
    }

    // ---- 统计与判定 ----
    let n = traj.len() as u64;
    eprintln!("\n[env] === 60s 全场景真实化持续悬停统计（步数 {n}，墙钟 {:.1}s，GPS 噪声样本 {gps_frames_total}/15000）===", wall.as_secs_f64());
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
                "[env]  {:.0}-{:.0}s: 起点=({:.2},{:.2}) max|dz|={:.2}m max水平漂移={:.2}m max|v|={:.2}m/s",
                seg_start as f64 * 0.004, end as f64 * 0.004, p0[0], p0[1], max_dz, max_horiz, max_v
            );
            seg_start = end;
        }
    }
    let last = traj.last().map(|t| t.1).unwrap_or([0.0, 0.0, -5.0]);
    eprintln!(
        "[env] 末态 pos=({:.2},{:.2},{:.2})m 姿态=({:.1}°,{:.1}°) max_thrust={max_thrust:.3}",
        last[0], last[1], last[2], roll_max.to_degrees(), pitch_max.to_degrees()
    );
    eprintln!("[env] 全程 max|roll|={:.2}° max|pitch|={:.2}°", roll_max.to_degrees(), pitch_max.to_degrees());

    // 判定：全场景真实化（噪声 + 2.5m/s 侧风 + 阵风/湍流 + 温漂 + 丢星）下——
    // - 姿态不发散：对抗侧风需持续倾斜（~5-8°），湍流叠加摆动，阈值 25° 留裕度
    // - 高度有界：GPS 位置噪声 0.5m + 气压噪声 0.3m + 垂向湍流，dz<3m
    // - 水平漂移有界：风 2.5m/s 下位置环若失效 60s 会漂 >100m；<8m 证明位置环
    //   有效对抗（允许恒定偏置漂移，但必须有界——"稳定悬停"的工程定义）
    assert!(roll_max.to_degrees() < 25.0, "姿态 roll 发散：{:.1}°", roll_max.to_degrees());
    assert!(pitch_max.to_degrees() < 25.0, "姿态 pitch 发散：{:.1}°", pitch_max.to_degrees());
    if n > 2500 {
        let seg = &traj[2500..n as usize];
        let p0 = seg[0].1;
        for &(_, p, _) in seg {
            let dz = (p[2] - HOVER_D).abs();
            assert!(dz < 3.0, "高度失稳：dz={dz:.2}m @pos=({:.2},{:.2},{:.2})", p[0], p[1], p[2]);
            let horiz = ((p[0] - p0[0]).powi(2) + (p[1] - p0[1]).powi(2)).sqrt();
            assert!(horiz < 8.0, "水平漂移过大（风下失稳）：{horiz:.2}m @pos=({:.2},{:.2})", p[0], p[1]);
        }
    }
    eprintln!(">>> [ENV-HOVER] 60s 全场景真实化持续悬停验证通过 ✓");
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
