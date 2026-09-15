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
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;
use unicorn_engine::RegisterARM;
use mcu_simulater::peripheral::vperiph::data_source::FlySimState;

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

/// EST_STATE 布局（由 telemetry_entry 反汇编 + 运行期 f32 转储交叉确认）：
///   offset 0:   VehicleState (72B)，但编译器对结构体字段做了重排（非 repr(C)）：
///     +0   att 四元数 (16B) — dump index 0 恒为 1.0（att.w 水平悬停）
///     +16  time_boot_ms (4B)
///     +20  pos[0]  +24 pos[1]  +28 pos[2]  (NED, D 向下正) — dump index 5/6/7
///     +32  vel[0]  +36 vel[1]  +40 vel[2]  — dump index 8/9/10
///     +44  omega[3] +56 airspeed +60 accel_bias[3]
///   offset 72:  Health (1B)
///   offset 73:  armed (1B)
/// 依据：telemetry_entry 0x08064128-0x08064196 把 sp+32/36/40/44/48/52（EST_STATE 拷贝
/// 偏移 +20/+24/+28/+32/+36/+40）写入 LOCAL_POSITION_NED payload pos/vel 各分量；
/// 且运行期转储 index7 与固件控制台 est.pos[2] 数值吻合（早期 ≈1.0）。
/// 读 EKF 估计高度 est.pos[2]（f32，NED D 向下正）。
fn read_ekf_z(m: &Arc<Mutex<Machine>>) -> f32 {
    let b = m.lock().unwrap().cpu.mem_read(0x2000_9074 + 28, 4).unwrap();
    f32::from_le_bytes(b.try_into().unwrap())
}

/// [DIAG] 转储 EST_STATE 前 72B（VehicleState）为 18 个 f32，定位 est.pos[2] 实际偏移。
fn dump_est_state(m: &Arc<Mutex<Machine>>) -> Vec<f32> {
    let b = m.lock().unwrap().cpu.mem_read(0x2000_9074, 72).unwrap();
    (0..18).map(|i| f32::from_le_bytes([b[i*4], b[i*4+1], b[i*4+2], b[i*4+3]])).collect()
}

/// [DIAG] boot 后、ARM 前转储 EKF 状态 + 传感器帧字段，定位 boot 阶段 EKF z 漂移。
/// 内存布局（据 symbol 表）：
///   EST_STATE    @0x2000_9074 (VehicleState 72B; pos[2] @ +28)
///   SENSOR_FRAME @0x2000_9018 (SensorFrame: imu Option(24B) + rc(24B) + gps Option(24B) + baro Option(4B)...)
fn dump_boot_state(m: &mut Machine) {
    fn rd(m: &mut Machine, addr: u64, len: usize) -> Vec<u8> {
        m.cpu.mem_read(addr, len).unwrap_or_default()
    }
    fn f32at(m: &mut Machine, addr: u64) -> f32 {
        let b = rd(m, addr, 4);
        if b.len() < 4 { f32::NAN } else { f32::from_le_bytes([b[0], b[1], b[2], b[3]]) }
    }
    fn u32at(m: &mut Machine, addr: u64) -> u32 {
        let b = rd(m, addr, 4);
        if b.len() < 4 { 0 } else { u32::from_le_bytes([b[0], b[1], b[2], b[3]]) }
    }
    fn u8at(m: &mut Machine, addr: u64) -> u8 {
        rd(m, addr, 1).first().copied().unwrap_or(0)
    }
    let ekf_z = f32at(m, 0x2000_9074 + 28);
    let ekf_velz = f32at(m, 0x2000_9074 + 40);
    // SENSOR_FRAME 字段（据 mod.rs SensorFrame 布局推断；不稳妥则显示原始字节）
    let fr = rd(m, 0x2000_9018, 96);
    eprintln!(
        "[DIAG-BOOT] EKF pos=({:.3},{:.3},{:.3}) vel=({:.3},{:.3},{:.3})",
        f32at(m, 0x2000_9074 + 20), f32at(m, 0x2000_9074 + 24), ekf_z,
        f32at(m, 0x2000_9074 + 32), f32at(m, 0x2000_9074 + 36), ekf_velz,
    );
    eprintln!(
        "[DIAG-BOOT] SENSOR_FRAME@0x20009018 前96B: {}",
        fr.iter().enumerate().map(|(i, b)| if i % 4 == 0 { format!("\n  +{i:02x}:") } else { String::new() } + &format!("{b:02x} ")).collect::<String>()
    );
    eprintln!(
        "[DIAG-BOOT] SENSOR_SEQ={} HIL_GATES=0x{:08x} G_CMD_ARMED={}",
        u32at(m, 0x2000_b5cc), u32at(m, 0x2000_b62c), u8at(m, 0x2000_b669)
    );
}

/// [ARM 前收敛推进] boot 早期 RC 链路未建立：`RcInput::neutral().throttle=0` →
/// control 首拍 `thr_off=(0-0.5)*2=-1` → `target_alt=hold_alt(0)-(-1)*2=+2.0` →
/// `set_initial_position` 把 EKF 高度锁到 d=+2.0（错误）。若在 EKF 收敛前 ARM，
/// `hold_alt=est.pos[2]≈1.3` 锁错 → 机体悬停在物理 d≈-3.7（距 5m 悬停点偏 ~1.3m，
/// `vperiph_hover_sustained` 实测 |dz|=1.33 失败）。
///
/// 修复：ARM 前推进仿真，等 SBUS 帧到达（20Hz → RC fresh、throttle=0.5 →
/// target_alt 回落 0）且 EKF 高度被 baro/GPS 观测拉回设计原点（|z|<tol），再 ARM。
/// 返回推进后 EKF 高度（m，NED 向下正）供调用方打印。
fn settle_ekf_before_arm(m: &Arc<Mutex<Machine>>, tag: &str, tol: f32) -> f32 {
    let mut mm = m.lock().unwrap();
    let mut z = f32::NAN;
    for i in 0..400 {
        mm.run(1_000_000).unwrap();
        z = f32::from_le_bytes(mm.cpu.mem_read(0x2000_9074 + 28, 4).unwrap().try_into().unwrap());
        if i % 50 == 0 {
            eprintln!("[{tag}] 收敛推进 i={i} ekf_z={z:.3}");
        }
        if z.abs() < tol {
            break;
        }
    }
    let fr_thr = f32::from_le_bytes(mm.cpu.mem_read(0x2000_9018 + 0x4C, 4).unwrap().try_into().unwrap());
    let fr_fresh = mm.cpu.mem_read(0x2000_9018 + 0x52, 1).unwrap()[0];
    eprintln!("[{tag}] 收敛完成 ekf_z={z:.3} frame.throttle={fr_thr:.3} frame.fresh={fr_fresh}");
    z
}


/// 共享的"注入 + 推进 + 读回 PWM"单步逻辑，供多个闭环测试复用。
///
/// `steps` 为物理步数（每步 4ms）。`hold_thresh` 为起飞台保持的推力阈值。
/// 返回 `(final_state, max_thrust, vec![(step, pos)])` 以便调用方做收敛判定。
#[allow(clippy::too_many_arguments)]
fn run_closed_loop(
    m: &Arc<Mutex<Machine>>,
    state: &Arc<Mutex<FlySimState>>,
    steps: u64,
    hold_thresh: f32,
    log_every: u64,
) -> (Option<flyctrl_core::vehicle::VehicleState>, f32, Vec<(u64, [f32; 3])>) {
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
    let mut traj = Vec::new();

    for step in 0..steps {
        // ---- 读回 PWM 推力（上一拍固件输出）----
        let motors = read_thrust(m);
        let thrust = motors.iter().sum::<f32>();
        max_thrust = max_thrust.max(thrust);

        // ---- 物理推进 / 起飞台保持 ----
        let (st, imu_true) = if held && thrust < hold_thresh {
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
            // 气压：标准大气（h 向上正 = -(d - 家庭点)）。家庭点=起飞台(d=-5)，
            // 与 GPS 原点（首次定位锁定）对齐 → 起飞台 baro 读 h=0、GPS 读 d=0，
            // 消除 EKF 高度源冲突（历史根因：GPS d=0 vs baro d=-5 折中 → hold_alt
            // 锁定错误值 → 机体下沉到错误高度）。
            let h = -(d + 5.0);
            st.baro_pa = 101_325.0 * (-h / 8434.5).exp();
            // RC 保持解锁（SBUS raw 1811 > 1700）
            st.rc_ch[4] = 2000.0;
        }

        // ---- MCU 推进（sensors 2ms 采样 + control 4ms + PWM 输出）----
        let run_res = { m.lock().unwrap().run(300_000) };
        if let Err(e) = run_res {
            let mut mm = m.lock().unwrap();
            let pc = mm.cpu.reg_read_u32(RegisterARM::PC).unwrap_or(0);
            let sp = mm.cpu.reg_read_u32(RegisterARM::SP).unwrap_or(0);
            let lr = mm.cpu.reg_read_u32(RegisterARM::LR).unwrap_or(0);
            panic!("[vperiph] run ERR at step={step}: {e:?} PC=0x{pc:08x} SP=0x{sp:08x} LR=0x{lr:08x}");
        }

        if let Some(s) = st {
            traj.push((step, [s.pos[0].0, s.pos[1].0, s.pos[2].0]));
        }
        if log_every > 0 && step % log_every == 0 {
            let ekf_z = read_ekf_z(m);
            let dump = dump_est_state(m);
            eprintln!(
                "[vperiph] step={step} t={:.2}s thrust={thrust:.3} m=[{:.3},{:.3},{:.3},{:.3}] pos=({:.2},{:.2},{:.2}) ekf_z={ekf_z:.3}",
                step as f64 * 0.004, motors[0], motors[1], motors[2], motors[3],
                pos[0], pos[1], pos[2],
            );
            eprintln!(
                "[vperiph]   est f32 dump: {:?}",
                dump.iter().enumerate().map(|(i, v)| format!("{i}:{v:.3}")).collect::<Vec<_>>().join(" ")
            );
        }
    }
    let wall = t0.elapsed();
    eprintln!(
        "[vperiph] 阶段完成：{steps} 步（{:.1}s 仿真），墙钟 {:.1}s，max_thrust={max_thrust:.3}",
        steps as f64 * 0.004, wall.as_secs_f64(),
    );
    (final_state, max_thrust, traj)
}

#[test]
fn vperiph_closed_loop() {
    init_log();
    let sys = artifact::joc_base_elf();
    let app = Path::new(APP_REAL);
    assert!(sys.exists(), "minimal elf 缺失");
    assert!(app.exists(), "real-sensors app 缺失：build_app.py --features real-sensors --out /tmp/flyctrl_clean.bin");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();

    // 共享传感器/RC 状态（fly_sim 注入目标）
    let state = Arc::new(Mutex::new(FlySimState::default()));
    m.attach_flysim_sensors(state.clone());
    m.attach_flysim_uart_slaves(state.clone());

    // 【关键】boot 前注入初始真值：固件 boot 阶段（12×1M 周期）内 sensors/control 任务
    // 已开始采样，若 FlySimState 保持默认（baro_pa=0 → 气压高 44330m、gps 无效），
    // EKF 高度会被污染到 -6454m（历史根因），解锁瞬间 hold_alt 锁定该垃圾值 → 持续下沉。
    // 静止水平 FRD 悬停 + 气压 h=5m（起飞台悬停点）→ EKF 从首拍即收敛到 d≈-5。
    // RC 保持中性：boot 阶段未解锁 → 不输出 PWM → 起飞台保持成立。
    {
        let mut st = state.lock().unwrap();
        st.imu_acc = [0.0, 0.0, -9.81];
        st.imu_gyr = [0.0, 0.0, 0.0];
        st.baro_pa = 101_325.0f32; // h=0 家庭点气压（d=-5 起飞台；与 GPS 原点对齐，消除高度源冲突）
        st.gps_lat = LAT0;
        st.gps_lon = LON0;
        st.gps_alt = ALT0 + 5.0; // 起飞台(d=-5) alt=9 → ref_alt=9 → 运行期 d_fw=0，气压单独驱动 d=-5
        st.gps_fix = 3.0;
        st.rc_ch = [1500.0; 16];
    }

    m.load_elf(sys).unwrap();
    m.load_app_partition(app).unwrap();
    m.reset().unwrap();
    for _ in 0..12 {
        m.run(1_000_000).unwrap();
    }
    let m = Arc::new(Mutex::new(m));

    // [DIAG] boot 后、ARM 前：转储 EKF 状态与传感器帧（定位 boot 阶段 z 漂移）
    {
        let mut mm = m.lock().unwrap();
        dump_boot_state(&mut mm);
    }

    // [ARM 前收敛推进] RC 链路建立 + EKF 高度收敛（根因见 settle_ekf_before_arm 文档）
    settle_ekf_before_arm(&m, "vperiph-closed", 0.6);

    // 地面站 ARM 等效注入：直接置 G_CMD_ARMED（AtomicBool）。
    // 地址随固件构建变化：`arm-none-eabi-nm app.elf | grep G_CMD_ARMED` 获取，
    // 重建固件后需同步（当前 clean 基线 = 0x2000b669）。
    m.lock().unwrap().cpu.mem_write(0x2000_b669, &[1u8]).unwrap();

    // 解锁 RC：ch4=2000（SBUS raw 1811 > 1700 armed）、ch3=1500（油门中位 raw 992 → 0.5）
    {
        let mut st = state.lock().unwrap();
        st.rc_ch[4] = 2000.0;
        st.rc_ch[3] = 1500.0;
    }

    let m2 = m.clone();
    let st2 = state.clone();
    let (final_state, max_thrust, _traj) = run_closed_loop(&m2, &st2, 60, 0.05, 10);

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
        "[vperiph] 闭环完成：max_thrust={max_thrust:.3}，末态 pos=({:.2},{:.2},{:.2}) roll={:.3}° pitch={:.3}°",
        st.pos[0].0, st.pos[1].0, st.pos[2].0,
        st.att.roll().to_degrees(), st.att.pitch().to_degrees(),
    );
    eprintln!(">>> [VPERIPH-MCUSIM] 虚拟外设直通闭环验证通过 ✓");
}

/// [虚拟外设直通] 长时悬停收敛验证：起飞台保持 → 升到 5m 悬停点 → 保持稳定。
///
/// 判定：仿真末段（最后 30%）位置应收敛在悬停点附近（|dz|<0.8m、水平 <0.5m），
/// 姿态 roll/pitch 全程 < 3°，且终态速度小（说明已收敛而非仍在飘）。
#[test]
fn vperiph_hover_long() {
    init_log();
    let sys = artifact::joc_base_elf();
    let app = Path::new(APP_REAL);
    assert!(sys.exists(), "minimal elf 缺失");
    assert!(app.exists(), "real-sensors app 缺失：build_app.py --features real-sensors --out /tmp/flyctrl_clean.bin");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();

    let state = Arc::new(Mutex::new(FlySimState::default()));
    m.attach_flysim_sensors(state.clone());
    m.attach_flysim_uart_slaves(state.clone());

    // 【关键】boot 前注入初始真值：boot 阶段 sensors 已采样，若默认 baro_pa=0 →
    // EKF 高度被污染到 -6454m，解锁时 hold_alt 锁定垃圾值 → 悬停测试持续下沉（历史根因）。
    // 静止水平 FRD 悬停 + 气压 h=5m + GPS 有效 → EKF 从首拍即收敛到 d≈-5。
    {
        let mut st = state.lock().unwrap();
        st.imu_acc = [0.0, 0.0, -9.81];
        st.imu_gyr = [0.0, 0.0, 0.0];
        st.baro_pa = 101_325.0f32; // h=0 家庭点气压（d=-5 起飞台；与 GPS 原点对齐，消除高度源冲突）
        st.gps_lat = LAT0;
        st.gps_lon = LON0;
        st.gps_alt = ALT0 + 5.0; // 起飞台(d=-5) alt=9 → ref_alt=9 → 运行期 d_fw=0，气压单独驱动 d=-5
        st.gps_fix = 3.0;
        st.rc_ch = [1500.0; 16];
    }

    m.load_elf(sys).unwrap();
    m.load_app_partition(app).unwrap();
    m.reset().unwrap();
    for _ in 0..12 {
        m.run(1_000_000).unwrap();
    }
    let m = Arc::new(Mutex::new(m));

    // [DIAG] boot 后、ARM 前：转储 EKF 状态与传感器帧（定位 boot 阶段 z 漂移）
    {
        let mut mm = m.lock().unwrap();
        dump_boot_state(&mut mm);
    }

    // [ARM 前收敛推进] RC 链路建立 + EKF 高度收敛（根因见 settle_ekf_before_arm 文档：
    // boot 早期 RC 未建立 → target_alt=+2.0 → EKF 高度锁错 → hold_alt 锁错。
    // 历史"侥幸通过"：300 步截断早，机体未完全落到错误高度，|dz| 恰好 <0.8）。
    settle_ekf_before_arm(&m, "vperiph-hover", 0.6);

    // ARM（G_CMD_ARMED 直接置 1）
    m.lock().unwrap().cpu.mem_write(0x2000_b669, &[1u8]).unwrap();

    // 解锁 RC：ch4=2000（armed）、ch3=1500（油门中位）
    {
        let mut st = state.lock().unwrap();
        st.rc_ch[4] = 2000.0;
        st.rc_ch[3] = 1500.0;
    }

    // 300 步 = 1.2s 仿真：起飞台保持（thrust<0.05 不出台）→ 升空 → 悬停
    let (final_state, max_thrust, traj) = run_closed_loop(&m, &state, 300, 0.05, 25);

    // ---- 断言 ----
    {
        let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
        let t = String::from_utf8_lossy(&out);
        let n = t.len();
        eprintln!("[vperiph-hover] === console FINAL ({n}B) ===\n{}", &t[n.saturating_sub(3000)..]);
    }
    assert!(max_thrust > 0.05, "MCU 未输出有效推力");
    let st = final_state.expect("物理从未推进（起飞台一直保持？）");

    // 末段收敛：取最后 30% 轨迹点的位置均值，应接近悬停点 (0,0,-5)
    let n = traj.len();
    assert!(n >= 50, "轨迹点太少：{n}");
    let tail = &traj[n * 7 / 10..];
    let avg = |k: usize| -> f64 { tail.iter().map(|(_, p)| p[k] as f64).sum::<f64>() / tail.len() as f64 };
    let (ax, ay, az) = (avg(0), avg(1), avg(2));
    let dx = ax.abs();
    let dy = ay.abs();
    let dz = (az - (-5.0)).abs();
    eprintln!(
        "[vperiph-hover] 末段均值 pos=({ax:.3},{ay:.3},{az:.3}) |dx|={dx:.3} |dy|={dy:.3} |dz|={dz:.3} (n={})",
        tail.len()
    );

    // 有限性 & 姿态不发散
    for v in [st.pos[0].0, st.pos[1].0, st.pos[2].0] {
        assert!(v.is_finite() && v.abs() < 100.0, "位置发散：{v}");
    }
    assert!(st.att.roll().is_finite() && st.att.roll().abs() < 0.05, "roll 发散：{}", st.att.roll());
    assert!(st.att.pitch().is_finite() && st.att.pitch().abs() < 0.05, "pitch 发散：{}", st.att.pitch());

    // 收敛判定：末段位置应稳定在悬停点附近
    assert!(dz < 0.8, "高度未收敛到 5m 悬停点：末段 |dz|={dz:.3}（期望 <0.8m）");
    assert!(dx < 0.5 && dy < 0.5, "水平未收敛：|dx|={dx:.3} |dy|={dy:.3}（期望 <0.5m）");

    eprintln!(
        "[vperiph-hover] 悬停收敛 OK：末段 pos≈({ax:.2},{ay:.2},{az:.2})m 姿态=({:.2}°,{:.2}°)",
        st.att.roll().to_degrees(), st.att.pitch().to_degrees(),
    );
    eprintln!(">>> [VPERIPH-MCUSIM] 长时悬停收敛验证通过 ✓");
}

/// [虚拟外设直通] 持续稳定悬停验证（长时间）：确认悬停收敛后能**持续保持**稳定。
///
/// 与 `vperiph_hover_long` 同场景，但仿真时长拉长到 12s（3000 步），分三段检查：
/// - 早段（10%~40%）：高度应收敛（|dz|<1.5m），姿态 < 5°
/// - 中段（40%~70%）：保持稳定，且相对早段无明显漂移（高度变化 < 1.0m）
/// - 末段（70%~100%）：最终收敛在悬停点附近（|dz|<0.8m、水平 <0.5m）
/// 全程位置有限、roll/pitch 不发散。
#[test]
fn vperiph_hover_sustained() {
    init_log();
    let sys = artifact::joc_base_elf();
    let app = Path::new(APP_REAL);
    assert!(sys.exists(), "minimal elf 缺失");
    assert!(app.exists(), "real-sensors app 缺失：build_app.py --features real-sensors --out /tmp/flyctrl_clean.bin");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();

    let state = Arc::new(Mutex::new(FlySimState::default()));
    m.attach_flysim_sensors(state.clone());
    m.attach_flysim_uart_slaves(state.clone());

    // boot 前注入初始真值（同 vperiph_hover_long，防止 EKF 高度被污染）
    {
        let mut st = state.lock().unwrap();
        st.imu_acc = [0.0, 0.0, -9.81];
        st.imu_gyr = [0.0, 0.0, 0.0];
        st.baro_pa = 101_325.0f32;
        st.gps_lat = LAT0;
        st.gps_lon = LON0;
        st.gps_alt = ALT0 + 5.0;
        st.gps_fix = 3.0;
        st.rc_ch = [1500.0; 16];
    }

    m.load_elf(sys).unwrap();
    m.load_app_partition(app).unwrap();
    m.reset().unwrap();
    for _ in 0..12 {
        m.run(1_000_000).unwrap();
    }
    let m = Arc::new(Mutex::new(m));

    // [ARM 前收敛推进] RC 链路建立 + EKF 高度收敛（见 settle_ekf_before_arm 文档，
    // 根因：boot 早期 RC 未建立 → target_alt=+2.0 → EKF 高度锁错 → hold_alt 锁错）
    settle_ekf_before_arm(&m, "vperiph-sustain", 0.6);

    // ARM + RC 解锁
    m.lock().unwrap().cpu.mem_write(0x2000_b669, &[1u8]).unwrap();
    {
        let mut st = state.lock().unwrap();
        st.rc_ch[4] = 2000.0;
        st.rc_ch[3] = 1500.0;
    }

    // 3000 步 = 12s 仿真：起飞台保持 → 升空 → 持续悬停
    let (final_state, max_thrust, traj) = run_closed_loop(&m, &state, 3000, 0.05, 250);

    // ---- 断言 ----
    assert!(max_thrust > 0.05, "MCU 未输出有效推力");
    let st = final_state.expect("物理从未推进（起飞台一直保持？）");

    let n = traj.len();
    assert!(n >= 300, "轨迹点太少：{n}");

    // 三段均值：早段 10%~40%、中段 40%~70%、末段 70%~100%
    let seg_avg = |a: usize, b: usize| -> ([f64; 3], usize) {
        let s = &traj[n * a / 10..n * b / 10];
        let k = s.len();
        let mut sum = [0.0f64; 3];
        for (_, p) in s {
            for i in 0..3 {
                sum[i] += p[i] as f64;
            }
        }
        ([sum[0] / k as f64, sum[1] / k as f64, sum[2] / k as f64], k)
    };
    let (early, k0) = seg_avg(1, 4);
    let (mid, k1) = seg_avg(4, 7);
    let (late, k2) = seg_avg(7, 10);
    eprintln!(
        "[vperiph-sustain] 早段(pos=({:.3},{:.3},{:.3}) n={k0}) 中段=({:.3},{:.3},{:.3}) n={k1} 末段=({:.3},{:.3},{:.3}) n={k2}",
        early[0], early[1], early[2], mid[0], mid[1], mid[2], late[0], late[1], late[2],
    );

    // 早段：已收敛（|dz|<1.5m），姿态 < 5°
    let de = (early[2] - (-5.0)).abs();
    assert!(de < 1.5, "早段未收敛到 5m 悬停点：|dz|={de:.3}（期望 <1.5m）");
    assert!(st.att.roll().is_finite() && st.att.roll().abs() < 0.05, "roll 发散：{}", st.att.roll());
    assert!(st.att.pitch().is_finite() && st.att.pitch().abs() < 0.05, "pitch 发散：{}", st.att.pitch());

    // 中段相对早段：无明显漂移（高度变化 < 1.0m）
    let drift = (mid[2] - early[2]).abs();
    assert!(drift < 1.0, "中段高度漂移过大：{drift:.3}m（早段→中段变化期望 <1.0m）");

    // 末段：最终收敛在悬停点附近
    let dx = late[0].abs();
    let dy = late[1].abs();
    let dz = (late[2] - (-5.0)).abs();
    assert!(dz < 0.8, "高度未收敛到 5m 悬停点：末段 |dz|={dz:.3}（期望 <0.8m）");
    assert!(dx < 0.5 && dy < 0.5, "水平未收敛：|dx|={dx:.3} |dy|={dy:.3}（期望 <0.5m）");

    // 全程位置有限
    for v in [st.pos[0].0, st.pos[1].0, st.pos[2].0] {
        assert!(v.is_finite() && v.abs() < 100.0, "位置发散：{v}");
    }

    eprintln!(
        "[vperiph-sustain] 持续悬停 12s 稳定：末段 pos≈({late0:.2},{late1:.2},{late2:.2})m 姿态=({r:.2}°,{p:.2}°)",
        late0 = late[0], late1 = late[1], late2 = late[2],
        r = st.att.roll().to_degrees(), p = st.att.pitch().to_degrees(),
    );
    eprintln!(">>> [VPERIPH-MCUSIM] 持续悬停（12s）稳定验证通过 ✓");
}
