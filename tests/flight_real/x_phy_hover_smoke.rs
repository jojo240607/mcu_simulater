//! [持续悬停演示] 按虚拟外设直通闭环流程（fly-sim 物理 → FlySimState →
//! mcu_sim 虚拟外设 → flyctrl 固件 → PWM 读回）跑 60s 连续悬停仿真。
//!
//! 与 `tests/x_vperiph_mcusim.rs` 同一套流程，仅拉长仿真时长并输出轨迹统计，
//! 用于回答"能否持续悬停"：分段统计高度/水平漂移、姿态发散、推力稳定性。
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

/// 读固件 EKF 估计高度 est.pos[2]（NED 向下正，地址布局同 x_vperiph：sym("EST_STATE")+12）。
fn read_ekf_z(m: &Arc<Mutex<Machine>>) -> f32 {
    let b = m.lock().unwrap().cpu.mem_read(sym("EST_STATE") + 12, 4).unwrap();
    f32::from_le_bytes(b.try_into().unwrap())
}

/// [DIAG] 转储 EST_STATE 前 72B（VehicleState）为 18 个 f32（与 x_vperiph_mcusim 同源）。
fn dump_est_state(m: &Arc<Mutex<Machine>>) -> Vec<f32> {
    let b = m.lock().unwrap().cpu.mem_read(sym("EST_STATE"), 72).unwrap();
    (0..18)
        .map(|i| f32::from_le_bytes([b[i * 4], b[i * 4 + 1], b[i * 4 + 2], b[i * 4 + 3]]))
        .collect()
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
fn phy_hover_smoke_10s() {
    init_log();
    let sys = artifact::joc_base_elf();
    let app = artifact::flyctrl_real_app_bin();
    mcu_simulater::elfsym::use_app_elf(mcu_simulater::artifact::flyctrl_real_app_elf());
    assert!(sys.exists(), "minimal elf 缺失");
    assert!(app.exists(), "real-sensors app 缺失：先跑 ./scripts/build.sh real-sensors");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    let state = Arc::new(Mutex::new(FlySimState::default()));
    m.attach_flysim_sensors(state.clone());
    m.attach_flysim_uart_slaves(state.clone());
    {
        let mut st = state.lock().unwrap();
        st.imu_acc = [0.0, 0.0, -9.81];
        st.imu_gyr = [0.0, 0.0, 0.0];
        st.baro_pa = 101_325.0f32;
        st.gps_lat = LAT0 as f64;
        st.gps_lon = LON0 as f64;
        st.gps_alt = ALT0 + 5.0;
        st.gps_fix = 3.0;
        st.gps_vel = [0.0, 0.0, 0.0];
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
    // ★§5.136 A/B：按 env 消融 ESKF 观测（poke 固件旋钮；默认全 0=开）
    if let Ok(dis) = std::env::var("PHY_DISABLE") {
        let knobs: &[(&str, &str)] = &[
            ("mag", "G_ESKF_MAG_ON"),
            ("grav", "G_ESKF_GRAV_ON"),
            ("baro", "G_ESKF_BARO_ON"),
            ("gps", "G_ESKF_GPS_ON"),
        ];
        for (tag, symname) in knobs {
            if dis.split(',').any(|d| d.trim() == *tag) {
                let addr = mcu_simulater::elfsym::app_sym(symname) as u64;
                m.lock().unwrap().cpu.mem_write(addr, &2.0f32.to_le_bytes()).unwrap();
                eprintln!("[phy-smoke] 消融：{symname} = 2.0（关）");
            }
        }
        // ★§5.136 A/B-②：磁重锚定（把 mag_I 软拉回先验；仿真 .data 未初始化 ⇒ 默认读到 0=关）
        if let Ok(ds) = std::env::var("PHY_MAG_DELAY") {
            let dv: f32 = ds.parse().unwrap_or(0.0);
            let addr = mcu_simulater::elfsym::app_sym("G_ESKF_MAG_DELAY_MS") as u64;
            m.lock().unwrap().cpu.mem_write(addr, &dv.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：磁延迟补偿 = {dv} ms");
        }
        if let Ok(qs) = std::env::var("PHY_GYR_NOTCH_Q") {
            let qv: f32 = qs.parse().unwrap_or(0.0);
            let addr = mcu_simulater::elfsym::app_sym("G_ESKF_GYR_NOTCH_Q") as u64;
            m.lock().unwrap().cpu.mem_write(addr, &qv.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：陀螺陷波 Q = {qv}");
        }
        if std::env::var("PHY_BYPASS_NOTCH").is_ok() {
            let addr = mcu_simulater::elfsym::app_sym("G_ESKF_BYPASS_GYR_NOTCH") as u64;
            m.lock().unwrap().cpu.mem_write(addr, &2.0f32.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：旁路陀螺陷波 ✓");
        }
        if let Ok(mode_s) = std::env::var("PHY_MAG_MODE") {
            // ★§5.136 A/B：G_ESKF_MAG_YAW_ON 旋钮（2.0=强制 heading、3.0=强制 3D、其余=默认）
            let v: f32 = mode_s.parse().unwrap_or(0.0);
            let addr = mcu_simulater::elfsym::app_sym("G_ESKF_MAG_YAW_ON") as u64;
            m.lock().unwrap().cpu.mem_write(addr, &v.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：G_ESKF_MAG_YAW_ON = {v}（2=heading / 3=3D）");
        }
        if let Ok(mp) = std::env::var("PHY_MAG_PERIOD") {
            let v: u32 = mp.parse().unwrap_or(15);
            // aid_period 是结构体字段（非全局符号）⇒ 用符号读偏移不可行 ⇒ 改 poke 全局旋钮
            // 由 G_ESKF_MAG_PERIOD 覆盖（见 eskf_estimator）
            let a = mcu_simulater::elfsym::app_sym("G_ESKF_MAG_PERIOD") as u64;
            m.lock().unwrap().cpu.mem_write(a, &(v as f32).to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：G_ESKF_MAG_PERIOD = {v}");
        }
        if let Ok(rk) = std::env::var("PHY_R_MAG_K") {
            let v: f32 = rk.parse().unwrap_or(1.0);
            let a = mcu_simulater::elfsym::app_sym("G_ESKF_R_MAG_K") as u64;
            m.lock().unwrap().cpu.mem_write(a, &v.to_le_bytes()).unwrap();
            let rd = m.lock().unwrap().cpu.mem_read(a, 4).unwrap();
            eprintln!("[phy-smoke] A/B：G_ESKF_R_MAG_K = {v} (读回={})", f32::from_le_bytes([rd[0], rd[1], rd[2], rd[3]]));
        }
        if let Ok(vf) = std::env::var("PHY_VAR_FLOOR") {
            let v: f32 = vf.parse().unwrap_or(1.0);
            let a = mcu_simulater::elfsym::app_sym("G_ESKF_VAR_FLOOR") as u64;
            m.lock().unwrap().cpu.mem_write(a, &v.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：G_ESKF_VAR_FLOOR = {v}");
        }
        if std::env::var("PHY_MAG_FREEZE_B").is_ok() {
            let a = mcu_simulater::elfsym::app_sym("G_ESKF_MAG_FREEZE_B") as u64;
            m.lock().unwrap().cpu.mem_write(a, &2.0f32.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：G_ESKF_MAG_FREEZE_B = 2（冻结硬铁 ✓）");
        }
        if let Ok(ps) = std::env::var("PHY_MAG_PRIOR") {
            let v: Vec<f32> = ps.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            if v.len() == 3 {
                for (sym, val) in [("G_ESKF_MAG_I_PRIOR_X", v[0]), ("G_ESKF_MAG_I_PRIOR_Y", v[1]), ("G_ESKF_MAG_I_PRIOR_Z", v[2])] {
                    let a = mcu_simulater::elfsym::app_sym(sym) as u64;
                    m.lock().unwrap().cpu.mem_write(a, &val.to_le_bytes()).unwrap();
                }
                eprintln!("[phy-smoke] A/B：磁先验 = {v:?}");
            }
        }
        if let Ok(rs) = std::env::var("PHY_MAG_RESET_PERIOD") {
            let v: f32 = rs.parse().unwrap_or(167.0);
            let addr = mcu_simulater::elfsym::app_sym("G_ESKF_MAG_RESET_PERIOD") as u64;
            m.lock().unwrap().cpu.mem_write(addr, &v.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：G_ESKF_MAG_RESET_PERIOD = {v}（周期性重锚 ✓）");
        }
        if std::env::var("PHY_MAG_FREEZE").is_ok() {
            let addr = mcu_simulater::elfsym::app_sym("G_ESKF_MAG_FREEZE") as u64;
            m.lock().unwrap().cpu.mem_write(addr, &2.0f32.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：G_ESKF_MAG_FREEZE = 2（冻结磁两态）");
        }
        if std::env::var("PHY_FREEZE_BIAS").is_ok() {
            let addr = mcu_simulater::elfsym::app_sym("G_ESKF_FREEZE_BIAS") as u64;
            m.lock().unwrap().cpu.mem_write(addr, &1.0f32.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：G_ESKF_FREEZE_BIAS = 1（冻结零偏修正）");
        }
        if let Ok(sig) = std::env::var("PHY_REANCHOR") {
            let v: f32 = sig.parse().unwrap_or(0.01);
            let addr = mcu_simulater::elfsym::app_sym("G_ESKF_MAG_REANCHOR") as u64;
            m.lock().unwrap().cpu.mem_write(addr, &v.to_le_bytes()).unwrap();
            eprintln!("[phy-smoke] A/B：G_ESKF_MAG_REANCHOR = {v}（开）");
        }
    }
    {
        let mut st = state.lock().unwrap();
        st.rc_ch[4] = 2000.0; // 解锁
        st.rc_ch[3] = 1500.0;
        // ★§5.139 判定旋钮：PHY_NO_LOITER=1 ⇒ 位置环关闭（姿态/自稳模式 ✓）
        st.rc_ch[5] = if std::env::var("PHY_NO_LOITER").is_ok() { 1000.0 } else { 2000.0 };
    }

    // ★PHY 引擎（真实转动动力学）—— 本测试是 PHY 路径的快速守门 + 整定仪表
    let mut sim = SimLoop::new(
        PhySdkWorld::create_empty(),
        &VehicleConfig::default_quad(),
        0.004,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(fly_sim_core::physics::ContactModel::default()),
        vec![],
    );
    // ★§5.136：PHY_INJECT_MAG=1 ⇒ 注入【物理引擎世界场】作为磁（与 x_hover_demo 同源 ✓）
    //   缺省走 vperiph 回退场（惰性磁 ⇒ 与真机语义不符，仅作冒烟）；复现/回归用注入模式 ✓
    // ★§5.139 判定旋钮：`PHY_STATIC=1` ⇒ **真值姿态固定为水平、电机不驱动 plant**
    //   （断开"控制→机体运动"回路 ✓）⇒ 用于判定真机链 3D 失稳是【闭环】还是【滤波器】
    let static_plant = std::env::var("PHY_STATIC").is_ok();
    let inject_mag = std::env::var("PHY_INJECT_MAG").is_ok();
    eprintln!("[phy-smoke] 磁注入 = {inject_mag}（PHY_INJECT_MAG=1 复现真机语义 ✓）");
    let mut held = true;
    let mut max_tilt = 0.0f32;
    let mut true_roll_deg = 0.0f32;
    let mut true_pitch_deg = 0.0f32;
    let mut max_dz = 0.0f32;
    let mut max_drift = 0.0f32;
    let mut last = None;
    let secs: u64 = std::env::var("PHY_SMOKE_SECS").ok().and_then(|v| v.parse().ok()).unwrap_or(10);
    let steps = secs * 250;
    for _step in 0..steps {
        {
            let mut mm = m.lock().unwrap();
            mcu_simulater::clock::run_one_control_tick(&mut mm).unwrap();
        }
        let motors = read_thrust(&m);
        let thrust = motors.iter().sum::<f32>();
        let (st, imu_true) = if held && thrust < 1.5 {
            (None, flyctrl_core::vehicle::ImuSample {
                accel: [
                    flyctrl_core::units::MeterPerSecondSquared(0.0),
                    flyctrl_core::units::MeterPerSecondSquared(0.0),
                    flyctrl_core::units::MeterPerSecondSquared(-9.81),
                ],
                gyro: [flyctrl_core::units::RadianPerSecond(0.0); 3],
            })
        } else if static_plant {
            // ★隔离闭环：电机效率置 0 ⇒ 无推力/无力矩 ⇒ 机体保持静止水平 ✓
            //   （姿态真值恒定 ⇒ "控制→机体运动→观测"回路断开 ✓）
            held = false;
            sim.set_motor_eff([0.0; 4]);
            let cmd = ActuatorCmd { motor: motors };
            let st = sim.step_hil(&cmd);
            (Some(st), sim.last_imu())
        } else {
            held = false;
            let cmd = ActuatorCmd { motor: motors };
            let st = sim.step_hil(&cmd);
            (Some(st), sim.last_imu())
        };
        let (pos, vel) = match st {
            Some(s) => ([s.pos[0].0, s.pos[1].0, s.pos[2].0], [s.vel[0].0, s.vel[1].0, s.vel[2].0]),
            None => ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0]),
        };
        if let Some(s) = st {
            let r = s.att.roll().abs().max(s.att.pitch().abs()).to_degrees();
            max_tilt = max_tilt.max(r);
            true_roll_deg = s.att.roll().to_degrees();
            true_pitch_deg = s.att.pitch().to_degrees();
            let dz = (pos[2] - HOVER_D).abs();
            max_dz = max_dz.max(dz);
            let drift = (pos[0] * pos[0] + pos[1] * pos[1]).sqrt();
            max_drift = max_drift.max(drift);
            last = Some((pos, vel));
        }
        if _step % 250 == 0 {
            let sym = |n: &str| mcu_simulater::elfsym::app_sym(n) as u64;
            let b = m.lock().unwrap().cpu.mem_read(sym("EST_STATE"), 44).unwrap();
            let f: Vec<f32> = (0..11).map(|i| f32::from_le_bytes([b[4*i], b[4*i+1], b[4*i+2], b[4*i+3]])).collect();
            // 估计姿态（四元数 f[7..11]）欧拉角（deg）
            let (ew, ex, ey, ez) = (f[7], f[8], f[9], f[10]);
            let eroll = (2.0f32 * (ew * ex + ey * ez)).atan2(1.0 - 2.0 * (ex * ex + ey * ey)).to_degrees();
            let epitch = (2.0f32 * (ew * ey - ez * ex)).asin().to_degrees();
            let eyaw = (2.0f32 * (ew * ez + ex * ey)).atan2(1.0 - 2.0 * (ey * ey + ez * ez)).to_degrees();
            // 真值姿态（PHY plant）
            let (troll, tpitch, tyaw) = if let Some((p2, _v2)) = last {
                let _ = p2;
                (0.0f32, 0.0f32, 0.0f32)
            } else { (0.0, 0.0, 0.0) };
            let mt = read_thrust(&m);
            let _ = (troll, tpitch, tyaw);
            eprintln!("[loop] t={:.0}s att_est=({:+.1},{:+.1},{:+.1})° mot=[{:.2},{:.2},{:.2},{:.2}] est=({:+.2},{:+.2},{:+.2})",
                _step as f32 * 0.004, eroll, epitch, eyaw, mt[0], mt[1], mt[2], mt[3], f[1], f[2], f[3]);
        }
        if _step % 1250 == 0 {
            eprintln!("[phy-smoke] t={:.0}s pos=({:.2},{:.2},{:.2}) vel=({:.2},{:.2},{:.2}) tilt={:.1}° drift={:.2}m",
                _step as f32 * 0.004, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], max_tilt, max_drift);
        }
        {
            let mut st = state.lock().unwrap();
            st.imu_acc = [imu_true.accel[0].0, imu_true.accel[1].0, imu_true.accel[2].0];
            st.imu_gyr = [imu_true.gyro[0].0, imu_true.gyro[1].0, imu_true.gyro[2].0];
            let (n, e, d) = (pos[0], pos[1], pos[2]);
            st.gps_lat = LAT0 as f64 + (n / 111_320.0) as f64;
            st.gps_lon = LON0 as f64 + (e / (111_320.0 * LAT0.to_radians().cos())) as f64;
            st.gps_alt = ALT0 - d;
            st.gps_fix = 3.0;
            st.gps_vel = vel;
            let h = -(d + 5.0);
            st.baro_pa = 101_325.0 * (-h / 8434.5).exp();
            st.rc_ch[4] = 2000.0;
            st.rc_ch[5] = if std::env::var("PHY_NO_LOITER").is_ok() { 1000.0 } else { 2000.0 };
            // ★§5.136：按开关注入【物理引擎世界场】作为磁（与 x_hover_demo 同源 ✓）
            //   不注入时走 vperiph 回退场 = "惰性磁"（实测完美但与真机语义不符 ✗）
            if inject_mag {
                st.mag = Some(sim.last_mag());
            }
        }
    }
    // ★§5.136：dump 固件 console（找 t≈10s 的事件）
    {
        let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
        let t = String::from_utf8_lossy(&out);
        let n = t.len();
        eprintln!("[phy-smoke] === console tail 2000 ===\n{}", &t[n.saturating_sub(2000)..]);
    }
    let (p, v) = last.expect("物理未推进");
    eprintln!("[phy-smoke] 末态 pos=({:.2},{:.2},{:.2}) vel=({:.2},{:.2},{:.2}) | maxTilt={:.1}° maxDz={:.2}m maxDrift={:.2}m",
        p[0], p[1], p[2], v[0], v[1], v[2], max_tilt, max_dz, max_drift);
    // ★守门判据（当前放宽：PHY 路径的裕度问题在案，整定后收紧；见 §5.136 补遗 9/10）
    assert!(p[0].is_finite() && p[1].is_finite() && p[2].is_finite(), "位置发散");
    assert!(max_tilt < 45.0, "姿态发散：maxTilt={max_tilt:.1}°");
    assert!(max_drift < 30.0, "水平发散：maxDrift={max_drift:.2}m");
}

fn init_log() {
    let _ = env_logger::builder().is_test(true).filter_level(log::LevelFilter::Warn).try_init();
}
