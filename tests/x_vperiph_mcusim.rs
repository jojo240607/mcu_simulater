//! [虚拟外设直通] fly_simulater（PC 物理）↔ mcu_simulater 虚拟总线外设直通闭环。
//!
//! 架构（用户确认方案）：
//!   - fly_sim 每 4ms 物理步把真值写入共享 `FlySimState`（Arc<Mutex>）；
//!   - mcu_sim 虚拟外设（I2C1 mpu6050/bmp280、UART2 gps、UART3 sbus）经
//!     `FlySimSource` 即时读到该状态，动态寄存器随之更新；
//!   - 固件 flyctrl（real-sensors 特性）走标准驱动：sensors_task → SensorStack
//!     → ImuMpu6050(i2c0,0x68)/BaroBmp280(i2c0,0x76)/GpsUblox(uart1)/RcSbus(uart2)；
//!   - control 输出 → 标准 PWM 驱动（pwm0..3 = TIM3/2/1/4 CH1，400Hz），
//!     PC 读 TIM CCR1 得占空比 → 电机推力 → 更新物理。
//! 全程无 USB / 无 MAVLink 帧连接飞控。
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

fn init_log() {
    let _ = env_logger::builder().is_test(true).filter_level(log::LevelFilter::Warn).try_init();
}

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

#[test]
fn vperiph_closed_loop() {
    init_log();
    let sys = Path::new(SYS);
    let app = Path::new(APP_REAL);
    assert!(sys.exists(), "minimal elf 缺失");
    assert!(app.exists(), "real-sensors app 缺失：build_app.py --features real-sensors --out /tmp/flyctrl_clean.bin");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();

    // 共享传感器/RC 状态（fly_sim 注入目标）
    let state = Arc::new(Mutex::new(FlySimState::default()));
    m.attach_flysim_sensors(state.clone());
    m.attach_flysim_uart_slaves(state.clone());

    m.load_elf(sys).unwrap();
    m.load_app_partition(app).unwrap();
    m.reset().unwrap();
    for _ in 0..12 {
        m.run(1_000_000).unwrap();
    }
    let m = Arc::new(Mutex::new(m));

    // 地面站 ARM 等效注入：直接置 G_CMD_ARMED（AtomicBool）。
    // 地址随固件构建变化：`arm-none-eabi-nm app.elf | grep G_CMD_ARMED` 获取，
    // 重建固件后需同步（当前 clean 基线 = 0x2000b669）。
    m.lock().unwrap().cpu.mem_write(0x2000_b669, &[1u8]).unwrap();

    // 注入初始真值：悬停（静止水平，FRD accel z=-9.81）+ RC 通道
    {
        let mut st = state.lock().unwrap();
        st.imu_acc = [0.0, 0.0, 9.81];
        st.imu_gyr = [0.0, 0.0, 0.0];
        st.baro_pa = 101325.0;
        st.gps_lat = LAT0;
        st.gps_lon = LON0;
        st.gps_alt = ALT0;
        st.gps_fix = 3.0;
        st.rc_ch = [1500.0; 16];
        // SBUS 通道编码：raw = 992 + (us-1500)/500*819.5。armed 阈值 raw>1700 ≈ us>1932，
        // 故 1800us 不足（raw=1484<1700），须给足 2000us（raw=1811>1700）。
        st.rc_ch[4] = 2000.0; // armed（SBUS raw 1811 > 1700）
        st.rc_ch[3] = 1500.0; // 油门中位（raw 992 → throttle 0.5）
    }

    // 物理仿真
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
    let mut final_state = None;

    for step in 0..60u64 {
        // ---- 读回 PWM 推力（上一拍固件输出）----
        let motors = read_thrust(&m);
        let thrust = motors.iter().sum::<f32>();
        max_thrust = max_thrust.max(thrust);

        // ---- 物理推进 / 起飞台保持 ----
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
        final_state = st;
        let (pos, vel) = match st {
            Some(s) => ([s.pos[0].0, s.pos[1].0, s.pos[2].0], [s.vel[0].0, s.vel[1].0, s.vel[2].0]),
            None => ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0]),
        };

        // ---- 注入物理真值到共享状态（设备约定换算）----
        {
            let mut st = state.lock().unwrap();
            // IMU：FRD 比力直通（悬停 (0,0,-9.81)，hil.rs tilt alignment 期望此约定）
            st.imu_acc = [imu_true.accel[0].0, imu_true.accel[1].0, imu_true.accel[2].0];
            st.imu_gyr = [imu_true.gyro[0].0, imu_true.gyro[1].0, imu_true.gyro[2].0];
            // 位置：NED (n,e,d 向下正) → GPS 度/米 + 气压
            let (n, e, d) = (pos[0], pos[1], pos[2]);
            st.gps_lat = LAT0 + n / 111_320.0;
            st.gps_lon = LON0 + e / (111_320.0 * LAT0.to_radians().cos());
            st.gps_alt = ALT0 - d; // d 向下正 → 越低 alt 越小
            st.gps_fix = 3.0;
            // 气压：标准大气（h 向上正 = -d）
            let h = -d;
            st.baro_pa = 101_325.0 * (-h / 8434.5).exp();
            // RC 保持解锁（SBUS raw 1811 > 1700）
            st.rc_ch[4] = 2000.0;
        }

        // ---- MCU 推进（sensors 2ms 采样 + control 4ms + PWM 输出）----
        // 注意：不可在 if-let 条件里直接 m.lock().unwrap().run(...)：临时 MutexGuard
        // 存活到整个 if-let 语句结束（含 Err 分支），分支内再 m.lock() 会重入自死锁。
        // 先用块语句结束 guard 生命周期，再判断结果。
        let run_res = { m.lock().unwrap().run(300_000) };
        if let Err(e) = run_res {
            let mut mm = m.lock().unwrap();
            let pc = mm.cpu.reg_read_u32(RegisterARM::PC).unwrap_or(0);
            let sp = mm.cpu.reg_read_u32(RegisterARM::SP).unwrap_or(0);
            let lr = mm.cpu.reg_read_u32(RegisterARM::LR).unwrap_or(0);
            panic!("[vperiph] run ERR at step={step}: {e:?} PC=0x{pc:08x} SP=0x{sp:08x} LR=0x{lr:08x}");
        }

        if step % 10 == 0 {
            eprintln!(
                "[vperiph] step={step} t={:.2}s thrust={thrust:.3} m=[{:.3},{:.3},{:.3},{:.3}] pos=({:.2},{:.2},{:.2})",
                step as f64 * 0.004, motors[0], motors[1], motors[2], motors[3],
                pos[0], pos[1], pos[2],
            );
        }
    }
    let wall = t0.elapsed();

    // ---- 断言 ----
    {
        let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
        let t = String::from_utf8_lossy(&out);
        let n = t.len();
        eprintln!("[vperiph] === console FINAL ({n}B) ===\n{}", &t[n.saturating_sub(3000)..]);
    }
    {
        let mut mm = m.lock().unwrap();
        let est_armed = mm.cpu.mem_read(0x2000_9074 + 116, 1).unwrap()[0];
        let fr_armed = mm.cpu.mem_read(0x2000_9018 + 0x58, 1).unwrap()[0];
        let fr_fresh = mm.cpu.mem_read(0x2000_9018 + 0x52, 1).unwrap()[0];
        let fr_thr = f32::from_le_bytes(mm.cpu.mem_read(0x2000_9018 + 0x4C, 4).unwrap().try_into().unwrap());
        let cmd_armed = mm.cpu.mem_read(0x2000_b669, 1).unwrap()[0];
        let gates = u32::from_le_bytes(mm.cpu.mem_read(0x2000_b62c, 4).unwrap().try_into().unwrap());
        eprintln!("[DIAG] est.armed={est_armed} G_CMD_ARMED={cmd_armed} frame.armed={fr_armed} frame.fresh={fr_fresh} frame.throttle={fr_thr}");
        eprintln!("[DIAG] HIL_GATES=0x{gates:02x} armed={} rc={} health_ok={} est={} sp={} att_i={} pos_i={}",
                  (gates>>0)&1, (gates>>1)&1, (gates>>2)&1, (gates>>3)&1, (gates>>4)&1, (gates>>5)&1, (gates>>6)&1);
        let dm: Vec<String> = (0..4).map(|k| {
            let off = 0x2000_b61c + 4*k;
            f32::from_le_bytes(mm.cpu.mem_read(off, 4).unwrap().try_into().unwrap()).to_string()
        }).collect();
        eprintln!("[DIAG] DBG_MOTOR=[{}]", dm.join(","));
        let pre = f32::from_le_bytes(mm.cpu.mem_read(0x2000_b660, 4).unwrap().try_into().unwrap());
        let thr = f32::from_le_bytes(mm.cpu.mem_read(0x2000_b664, 4).unwrap().try_into().unwrap());
        eprintln!("[DIAG] DBG_PRE={pre} DBG_THR={thr}");
    }
    assert!(max_thrust > 0.05, "MCU 未回传有效推力（PWM CCR 未通？sensors 链路未通？未解锁？）");
    let st = final_state.expect("物理从未推进（起飞台一直保持？）");
    for v in [st.pos[0].0, st.pos[1].0, st.pos[2].0] {
        assert!(v.is_finite() && v.abs() < 100.0, "位置发散：{v}");
    }
    assert!(st.att.roll().is_finite() && st.att.roll().abs() < 1.0, "roll 发散：{}", st.att.roll());
    assert!(st.att.pitch().is_finite() && st.att.pitch().abs() < 1.0, "pitch 发散：{}", st.att.pitch());
    eprintln!(
        "[vperiph] 闭环完成：墙钟 {:.1}s，max_thrust={max_thrust:.3}，末态 pos=({:.2},{:.2},{:.2}) roll={:.3}° pitch={:.3}°",
        wall.as_secs_f64(), st.pos[0].0, st.pos[1].0, st.pos[2].0,
        st.att.roll().to_degrees(), st.att.pitch().to_degrees(),
    );
    eprintln!(">>> [VPERIPH-MCUSIM] 虚拟外设直通闭环验证通过 ✓");
}
