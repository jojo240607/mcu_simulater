//! [HIL 共享内存直连] fly-simulater（PC 物理世界）↔ mcu_simulater（飞控 MCU）闭环。
//!
//! 无 USB / 无 MAVLink：PC 每 4ms 物理步把传感器/设定点/解锁写入 SRAM3 共享区
//! （0x2002_0000，mcu_simulater 额外映射），固件 uplink 每 1ms 轮询 `pc_seq`
//! 变化后全量注入 SENSOR_FRAME 唤醒 control；telemetry 每 20ms 把执行器/诊断
//! 写回共享区，PC 读回驱动 plant（ToyWorld/SimLoop）。
//!
//! 共享区布局见 flyctrl/app/src/flyctrl/hil_shmem.rs（双方硬编码一致）。
//!
//! 构建前置：`cd joc-base && cmake -S . -B build_hil -DMCU_SIM=ON -RTOS_SELFTEST=OFF
//! && cmake --build build_hil`（minimal elf）；固件必须为 **hil feature** 构建
//! （共享内存契约仅在 hil 固件内编译）：
//! `cd flyctrl && python3 build_app.py --features hil --out /tmp/flyctrl_hil.bin`，
//! 并以 `JOC_APP_FLYCTRL=/tmp/flyctrl_hil.bin` 指向（一键联调：`./scripts/integrate.sh shmem`）。
//! 本测试启动后会校验固件确实为 hil 构建（见 `hil` 固件校验），否则给出可操作指引。

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::ToyWorld;
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample};
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;

const SHM: u64 = 0x2002_0000;
const MAGIC: u32 = 0x4849_4C31;

// 偏移（与固件 hil_shmem.rs 一致）
const O_PC_SEQ: u64 = 0x04;
const O_IMU_ACC: u64 = 0x08;
const O_IMU_GYR: u64 = 0x14;
const O_BARO: u64 = 0x20;
const O_GPS_VALID: u64 = 0x24;
const O_GPS_POS: u64 = 0x28;
const O_GPS_VEL: u64 = 0x34;
const O_SP_VALID: u64 = 0x40;
const O_SP_POS: u64 = 0x44;
const O_SP_VEL: u64 = 0x50;
const O_SP_ACC: u64 = 0x5C;
const O_SP_YAW: u64 = 0x68;
const O_ARMED: u64 = 0x6C;
const O_MCU_SEQ: u64 = 0x80;
const O_MOTOR: u64 = 0x84;
const O_DIAG: u64 = 0x94;

fn wr_u32(m: &Arc<Mutex<Machine>>, off: u64, v: u32) {
    m.lock().unwrap().cpu.mem_write(SHM + off, &v.to_le_bytes()).unwrap();
}
fn wr_f32(m: &Arc<Mutex<Machine>>, off: u64, v: f32) {
    m.lock().unwrap().cpu.mem_write(SHM + off, &v.to_le_bytes()).unwrap();
}
fn rd_u32(m: &Arc<Mutex<Machine>>, off: u64) -> u32 {
    let b = m.lock().unwrap().cpu.mem_read(SHM + off, 4).unwrap();
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}
fn rd_f32(m: &Arc<Mutex<Machine>>, off: u64) -> f32 {
    let b = m.lock().unwrap().cpu.mem_read(SHM + off, 4).unwrap();
    f32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn init_log() {
    let _ = env_logger::builder().is_test(true).filter_level(log::LevelFilter::Warn).try_init();
}

/// 初始化共享区：magic + 悬停设定点 (0,0,-5) + GPS 有效 + armed=0（测试后续 ARM）。
fn shm_init(m: &Arc<Mutex<Machine>>) {
    wr_u32(m, 0x00, MAGIC);
    wr_u32(m, O_SP_VALID, 1);
    wr_f32(m, O_SP_POS + 8, -5.0);
    wr_u32(m, O_GPS_VALID, 1);
    wr_u32(m, O_ARMED, 0);
}

fn hover_imu() -> ImuSample {
    ImuSample {
        accel: [
            flyctrl_core::units::MeterPerSecondSquared(0.0),
            flyctrl_core::units::MeterPerSecondSquared(0.0),
            flyctrl_core::units::MeterPerSecondSquared(-9.81),
        ],
        gyro: [flyctrl_core::units::RadianPerSecond(0.0); 3],
    }
}

#[test]
fn shmem_closed_loop() {
    init_log();
    let sys = artifact::joc_base_elf();
    let app = artifact::flyctrl_app_bin();
    assert!(sys.exists(), "minimal elf 缺失：先 cmake --build build_hil");
    assert!(app.exists(), "flyctrl-app 缺失：先 build_app.py --features hil");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&sys).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();
    for _ in 0..12 {
        m.run(1_000_000).unwrap();
    }

    // 【hil 固件校验】共享内存契约仅在 hil feature 固件内编译（uplink 轮询共享区）；
    // 若加载的是默认/real-sensors 固件（sensors 任务日志 `hil=0`），本测试必然
    // "物理从未推进"——此时给出可操作的产物指引，而非误导性的断言失败。
    {
        let out = m.console.lock().unwrap().output().to_vec();
        let t = String::from_utf8_lossy(&out);
        if !t.contains("hil=1") {
            panic!(
                "加载的 app 非 hil 构建（未发现传感器任务日志 'hil=1'，实际加载: {}）。\n\
                 共享内存契约仅在 hil 固件内编译。请：\n\
                 cd flyctrl && python3 build_app.py --features hil --out /tmp/flyctrl_hil.bin\n\
                 JOC_APP_FLYCTRL=/tmp/flyctrl_hil.bin cargo test --release --test x_shmem_mcusim\n\
                 （或直接 ./scripts/integrate.sh shmem 一键联调）",
                app.display()
            );
        }
    }

    let m = Arc::new(Mutex::new(m));
    shm_init(&m);

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
    let mut held = true; // 起飞台保持：MCU 未输出推力前不推进物理
    let mut max_thrust = 0.0f32;
    let mut final_state = None;
    let mut armed = false;
    let arm_step = 40u64; // 40 步（160ms）后解锁
    let mut pc_seq = 0u32;

    for step in 0..300u64 {
        // ---- 读回执行器（上一拍固件 telemetry 写入）----
        let motors = [
            rd_f32(&m, O_MOTOR + 0),
            rd_f32(&m, O_MOTOR + 4),
            rd_f32(&m, O_MOTOR + 8),
            rd_f32(&m, O_MOTOR + 12),
        ];
        let thrust = motors.iter().sum::<f32>();
        max_thrust = max_thrust.max(thrust);
        let mut diag = [0f32; 16];
        for i in 0..16 {
            diag[i] = rd_f32(&m, O_DIAG + 4 * i as u64);
        }

        // ---- 解锁（ARM）----
        if step == arm_step && !armed {
            armed = true;
            wr_u32(&m, O_ARMED, 1);
            eprintln!("[shmem] step={step} ARM 解锁（共享区 armed=1）");
        }

        // ---- 物理推进 / 起飞台保持 ----
        let (st, imu_true) = if held && thrust < 0.05 {
            (None, hover_imu())
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
        let yaw = match st { Some(s) => s.att.yaw(), None => 0.0 };

        // ---- 写共享区（传感器/设定点/armed 一次同步点）----
        pc_seq += 1;
        wr_u32(&m, O_PC_SEQ, pc_seq);
        for i in 0..3 {
            wr_f32(&m, O_IMU_ACC + 4 * i as u64, imu_true.accel[i].0);
            wr_f32(&m, O_IMU_GYR + 4 * i as u64, imu_true.gyro[i].0);
        }
        wr_f32(&m, O_BARO, -pos[2]);
        for i in 0..3 {
            wr_f32(&m, O_GPS_POS + 4 * i as u64, pos[i]);
            wr_f32(&m, O_GPS_VEL + 4 * i as u64, vel[i]);
        }
        // 悬停设定点固定 (0,0,-5)
        wr_f32(&m, O_SP_POS + 0, 0.0);
        wr_f32(&m, O_SP_POS + 4, 0.0);
        wr_f32(&m, O_SP_POS + 8, -5.0);
        for i in 0..3 {
            wr_f32(&m, O_SP_VEL + 4 * i as u64, 0.0);
            wr_f32(&m, O_SP_ACC + 4 * i as u64, 0.0);
        }
        wr_f32(&m, O_SP_YAW, yaw);

        // ---- MCU 推进（uplink 1ms poll 检测 pc_seq + control + telemetry）----
        m.lock().unwrap().run(300_000).unwrap();

        if step % 100 == 0 {
            if step == 100 {
                let v = m.lock().unwrap().vec_entries();
                let get = |n: u32| v.iter().find(|(x, _)| *x == n).map(|(_, c)| *c).unwrap_or(0);
                eprintln!("[shmem] IRQ: PendSV={} SysTick={} USB={}",
                    get(14), get(15), get(83));
            }
            eprintln!(
                "[shmem] step={step} t={:.2}s thrust={thrust:.3} m=[{:.3},{:.3},{:.3},{:.3}] thr={:.3} cnt={:.0} az={:.2} mcu_seq={}",
                step as f64 * 0.004, motors[0], motors[1], motors[2], motors[3],
                diag[4], diag[12], diag[14], rd_u32(&m, O_MCU_SEQ),
            );
        }
    }
    let wall = t0.elapsed();

    // ---- 断言（数值判据）----
    {
        let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
        let t = String::from_utf8_lossy(&out);
        let n = t.len();
        eprintln!("[shmem] === console FINAL ({n}B) ===\n{}", &t[n.saturating_sub(4000)..]);
    }
    let st = final_state.expect("物理从未推进（起飞台一直保持？未解锁或控制无输出）");
    assert!(max_thrust > 0.05, "MCU 未回传有效推力（共享区执行器未通？）");
    assert!(rd_u32(&m, O_MCU_SEQ) > 0, "MCU 从未写回执行器区（telemetry 未跑？）");
    // 不发散：位置/速度/姿态有限
    for v in [st.pos[0].0, st.pos[1].0, st.pos[2].0] {
        assert!(v.is_finite() && v.abs() < 100.0, "位置发散：{v}");
    }
    assert!(st.att.roll().is_finite() && st.att.roll().abs() < 1.0, "roll 发散：{}", st.att.roll());
    assert!(st.att.pitch().is_finite() && st.att.pitch().abs() < 1.0, "pitch 发散：{}", st.att.pitch());
    eprintln!(
        "[shmem] 闭环完成：墙钟 {:.1}s，max_thrust={max_thrust:.3}，末态 pos=({:.2},{:.2},{:.2}) roll={:.3}° pitch={:.3}°",
        wall.as_secs_f64(), st.pos[0].0, st.pos[1].0, st.pos[2].0,
        st.att.roll().to_degrees(), st.att.pitch().to_degrees(),
    );
    eprintln!(">>> [SHMEM-MCUSIM] 共享内存虚拟外设直连闭环验证通过 ✓");
}
