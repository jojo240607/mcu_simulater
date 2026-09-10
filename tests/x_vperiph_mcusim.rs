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
//! 构建前置：`cd joc-base && cmake --build build_hil`（minimal elf）、
//! `cd flyctrl && python3 build_app.py --features real-sensors --out /tmp/app_real.bin`。

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

const SYS: &str = "/home/ubuntu/work/joc-base/build_hil/stm32f407_minimal.elf";
const APP_REAL: &str = "/tmp/app_real.bin";

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
    assert!(app.exists(), "real-sensors app 缺失：build_app.py --features real-sensors --out /tmp/app_real.bin");

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

    // [临时] 地面站 ARM 等效注入：直接置 G_CMD_ARMED（AtomicBool @0x2000c825）
    m.lock().unwrap().cpu.mem_write(0x2000_c825, &[1u8]).unwrap();

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
        st.rc_ch[4] = 1800.0; // armed
        st.rc_ch[3] = 1500.0; // 油门中位
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
            // RC 保持解锁
            st.rc_ch[4] = 1800.0;
        }
        if step % 20 == 0 && step >= 20 {
            eprintln!("[vperiph] inject imu_z={} baro_pa={}", state.lock().unwrap().imu_acc[2], state.lock().unwrap().baro_pa);
        }

        // ---- MCU 推进（sensors 2ms 采样 + control 4ms + PWM 输出）----
        if let Err(e) = m.lock().unwrap().run(300_000) {
            let mut mm = m.lock().unwrap();
            let pc = mm.cpu.reg_read_u32(RegisterARM::PC).unwrap_or(0);
            let sp = mm.cpu.reg_read_u32(RegisterARM::SP).unwrap_or(0);
            let lr = mm.cpu.reg_read_u32(RegisterARM::LR).unwrap_or(0);
            panic!("[vperiph] run ERR at step={step}: {e:?} PC=0x{pc:08x} SP=0x{sp:08x} LR=0x{lr:08x}");
        }

        if step % 50 == 0 {
            eprintln!(
                "[vperiph] step={step} t={:.2}s thrust={thrust:.3} m=[{:.3},{:.3},{:.3},{:.3}] pos=({:.2},{:.2},{:.2})",
                step as f64 * 0.004, motors[0], motors[1], motors[2], motors[3],
                pos[0], pos[1], pos[2],
            );
        }
        if step % 20 == 0 && step >= 20 {
            let rd = |off: u64| -> u32 {
                let b = m.lock().unwrap().cpu.mem_read(0x2002_0000 + off, 4).unwrap();
                u32::from_le_bytes([b[0], b[1], b[2], b[3]])
            };
            eprintln!("[vperiph] shm-diag: gates=0x{:08x} est=({:.1},{:.1},{:.1}) vz={:.2} armed={} spv={} rcf={} rcarmed={} thr={:.2} accz={:.2} gyrz={:.2} rpy=({:.1},{:.1},{:.1})",
                rd(0), f32::from_bits(rd(36)), f32::from_bits(rd(40)), f32::from_bits(rd(4)),
                f32::from_bits(rd(44)), f32::from_bits(rd(8)) > 0.5,
                f32::from_bits(rd(20)) > 0.5, f32::from_bits(rd(24)) > 0.5,
                f32::from_bits(rd(28)) > 0.5, f32::from_bits(rd(32)),
                f32::from_bits(rd(52)), f32::from_bits(rd(56)),
                f32::from_bits(rd(12)).to_degrees(), f32::from_bits(rd(16)).to_degrees(),
                f32::from_bits(rd(48)).to_degrees());
            eprintln!("[vperiph] armed_flag={} cmd0={:.3} ticks0={:.0} pwmioc={:.0} pwmok={}",
                f32::from_bits(rd(60)), f32::from_bits(rd(64)), f32::from_bits(rd(68)),
                f32::from_bits(rd(72)), f32::from_bits(rd(76)) > 0.5);
            eprintln!("[vperiph] imu=({:.2},{:.2},{:.2}) baro={:.2}",
                f32::from_bits(rd(80)), f32::from_bits(rd(84)), f32::from_bits(rd(88)),
                f32::from_bits(rd(92)));
            {
                use mcu_simulater::peripheral::vperiph::I2cDir;
                let mm = m.lock().unwrap();
                let iv = mm.i2c.lock().unwrap();
                let mut i0 = iv[0].lock().unwrap();
                let az = {
                    let sl = i0.slaves_mut().iter_mut().find(|s| s.addr7() == 0x68).unwrap();
                    sl.on_start(I2cDir::Write); sl.on_write(0x3B);
                    sl.on_start(I2cDir::Read);
                    let _ = (sl.on_read(), sl.on_read(), sl.on_read(), sl.on_read());
                    (sl.on_read().unwrap(), sl.on_read().unwrap())
                };
                let bp = {
                    let sl = i0.slaves_mut().iter_mut().find(|s| s.addr7() == 0x76).unwrap();
                    sl.on_start(I2cDir::Write); sl.on_write(0xF7);
                    sl.on_start(I2cDir::Read);
                    (sl.on_read().unwrap(), sl.on_read().unwrap())
                };
                eprintln!("[vperiph] mpu sim az=0x{:02x}{:02x} baro sim p=0x{:02x}{:02x}",
                    az.0, az.1, bp.0, bp.1);
            }
            let rd2 = |off: u64| -> u32 {
                let b = m.lock().unwrap().cpu.mem_read(0x2002_0100 + off, 4).unwrap();
                u32::from_le_bytes([b[0], b[1], b[2], b[3]])
            };
            let (ufifo, udr, sr1, cr1a, cr3a, sr2, cr1b, cr3b) = {
                let mm = m.lock().unwrap();
                let us = mm.usart.lock().unwrap();
                let u2 = us[2].lock().unwrap();
                let u1 = us[1].lock().unwrap();
                (u2.rx_fifo_len(), u2.n_cpu_dr_reads(),
                 u2.dbg_state().0, u2.dbg_state().1, u2.dbg_state().2,
                 u1.dbg_state().0, u1.dbg_state().1, u1.dbg_state().2)
            };
            let sframes = {
                let mm = m.lock().unwrap();
                let us = mm.usart.lock().unwrap();
                let u2 = us[2].lock().unwrap();
                u2.slaves().iter().map(|s| s.frames()).sum::<u64>()
            };
            eprintln!("[vperiph] uart2: fifo={ufifo} dr={udr} SR=0x{sr1:08x} CR1=0x{cr1a:08x} CR3=0x{cr3a:08x} sbus_frames={sframes}");
            eprintln!("[vperiph] uart1: SR=0x{sr2:08x} CR1=0x{cr1b:08x} CR3=0x{cr3b:08x}");
            eprintln!("[vperiph] rcsbus: n={} fill={} fresh={} ch4={} buf0={}",
                f32::from_bits(rd2(0)), f32::from_bits(rd2(4)) as u32,
                f32::from_bits(rd2(8)) > 0.5, f32::from_bits(rd2(12)), f32::from_bits(rd2(16)) as u32);
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
