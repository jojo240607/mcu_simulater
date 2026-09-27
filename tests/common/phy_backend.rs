//! ★§5.143 PHY 化后端：把 `EnvHarness` 的运动学场景替换为**真转动动力学**。
//!
//! 动机（§5.136 补遗 9 的 ②）：env 家族现用 `scn.advance(运动学)` 直接指定真值运动；
//! PHY 化后由 `SimLoop::step_hil(真实刚体)` 按固件**执行器输出**推进刚体 ⇒ 真实闭环
//! （控制↔动力学耦合、饱和、转动惯量等都在内 ✓），断言口径（位置/姿态/高度界）保持不变。
//!
//! 用法（逐目标迁移 ✓）：
//! ```ignore
//! let mut h = EnvHarness::new(scn, true);
//! h.phy = Some(Box::new(PhyBackendImpl::new()));
//! h.run_for_ms(...);          // 内部自动走真动力学 ✓
//! ```
//! 注意 ✓：PHY 化后**基线会整体位移**（真动力学更苛刻）⇒ 按 §5.38 纪律重测基线并同步
//! `integrate.sh` / `docs/h-field.md` ✓

use std::sync::{Arc, Mutex};

use fly_sim_core::physics::ContactModel;
use fly_sim_core::sim::SimLoop;
use fly_sim_core::{ControllerKind, PhySdkWorld, SensorConfig};
use flyctrl_core::config::VehicleConfig;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::vperiph::data_source::FlySimState;

use super::PhyBackend;

/// GPS 原点（与 `x_phy_hover_smoke` 一致 ✓；env 家族的运动学场景同样用经纬度承载位置）
const LAT0_REF: f32 = 31.2304;
const LON0_REF: f32 = 121.4737;
const ALT0_REF: f32 = 4.0;
/// 悬停高度参考（m）：PHY 起飞后悬停的高度 —— 用于气压基准（使 baro 与运动学口径可比 ✓）
const HOVER_ALT_REF: f32 = 5.0;

/// 真动力学后端：持有 `SimLoop<PhySdkWorld>`（PHY 引擎 ✓），每步按固件执行器输出推进。
pub struct PhyBackendImpl {
    sim: SimLoop<PhySdkWorld>,
    /// 磁是否注入物理世界场（与 `x_hover_demo` 同源 ✓ 真机语义）；false ⇒ 走 vperiph 回退场
    inject_mag: bool,
    /// 起始真值（用于 `FlySimState` 的位置/姿态回写基准 ✓）
    steps: u64,
    /// 本后端是否已初始化摇杆中位（一次性 ✓；避免每步覆盖测试设置的通道 ✗）
    rc_init: bool,
    /// ★§5.144 摇杆 override（MAVLink 语义 ✓）：`Some([ch1..ch4])` 时**每步**写入固件
    /// `G_RC_OVERRIDE`（PWM µs ✓，ch1=roll/ch2=pitch/ch3=throttle/ch4=yaw ✓）并刷新
    /// 时间戳（超时 2s ✓ `uplink::get_rc_override` 口径 ✓）
    rc_override: Option<[u16; 4]>,
}

impl PhyBackendImpl {
    /// `inject_mag`：true ⇒ 注入 PHY 世界磁场（推荐，与真机一致 ✓）
    pub fn new(inject_mag: bool) -> Self {
        let sim = SimLoop::new(
            PhySdkWorld::create_empty(),
            &VehicleConfig::default_quad(),
            0.004,
            None,
            SensorConfig::default(),
            ControllerKind::Pid,
            Some(ContactModel::default()),
            vec![],
        );
        Self { sim, inject_mag, steps: 0, rc_init: false, rc_override: None }
    }
}

impl PhyBackendImpl {
    /// ★§5.144：设置摇杆 override（PWM µs，ch1..ch4 ✓）；`None` ⇒ 关闭 ✓
    /// 每步自动刷新时间戳（固件超时判据 2s ✓）
    pub fn set_rc_override(&mut self, ch: Option<[u16; 4]>) {
        self.rc_override = ch;
    }
}

impl PhyBackend for PhyBackendImpl {
    fn disturb_torque(&mut self, tau: [f64; 3]) {
        self.sim.disturb_torque_impulse(tau);
    }

    fn step_plant(&mut self, m: &mut Machine, st: &Arc<Mutex<FlySimState>>) {
        // ★§5.144：摇杆 override（每步写入 + 刷新时间戳 ✓ 固件超时 2s ✓）
        if let Some(ch) = self.rc_override {
            write_rc_override(m, ch);
        }
        // ① 读取固件执行器输出（4 路 PWM → 归一化推力 ✓，同 x_phy_hover_smoke）
        let motors = read_thrust(m);
        let cmd = flyctrl_core::vehicle::ActuatorCmd { motor: motors };
        // ② 真动力学推进一步（由执行器驱动刚体 ✓）
        let world = self.sim.step_hil(&cmd);
        self.steps += 1;
        // ③ 回写传感器状态（姿态/位置/磁/气压 ✓）——位置用**相对原点**（与运动学口径一致 ✓）
        let imu = self.sim.last_imu();
        {
            let mut s = st.lock().unwrap();
            let q = world.att;
            s.att = [q.w, q.x, q.y, q.z];
            // 位置/速度经 GPS 通道承载（与运动学口径一致 ✓）；NED → 经纬高 ✓
            let (pn, pe, pd) = (
                world.pos[0].0 as f32,
                world.pos[1].0 as f32,
                world.pos[2].0 as f32,
            );
            s.gps_lat = LAT0_REF as f64 + (pn / 111_320.0) as f64;
            s.gps_lon = LON0_REF as f64 + (pe / (111_320.0 * LAT0_REF.to_radians().cos())) as f64;
            s.gps_alt = ALT0_REF - pd; // NED 向下正 ⇒ 高度 = −z ✓
            s.gps_vel = [
                world.vel[0].0 as f32,
                world.vel[1].0 as f32,
                world.vel[2].0 as f32,
            ];
            s.imu_acc = [
                imu.accel[0].0 as f32,
                imu.accel[1].0 as f32,
                imu.accel[2].0 as f32,
            ];
            s.imu_gyr = [
                imu.gyro[0].0 as f32,
                imu.gyro[1].0 as f32,
                imu.gyro[2].0 as f32,
            ];
            if self.inject_mag {
                s.mag = Some(self.sim.last_mag());
            }
            // 气压：由高度换算（NED 向下正 ⇒ h = −z ✓）
            let h = -pd;
            s.baro_pa = 101_325.0 * (-(h - HOVER_ALT_REF) / 8434.5).exp();
            // ★一次性初始化：全部通道置中位（1500 ✓ 同 x_hover_demo 口径），
            //   之后**不覆盖其他通道** ⇒ 测试可设摇杆（如 rc_ch[1] 前推 ✓）
            if !self.rc_init {
                for c in s.rc_ch.iter_mut() {
                    *c = 1500.0;
                }
                self.rc_init = true;
            }
            // 解锁 + LOITER（每步保持 ✓）
            s.rc_ch[4] = 2000.0;
            s.rc_ch[5] = 2000.0;
        }
    }
}

/// ★§5.144：写固件 `G_RC_OVERRIDE`（PWM µs ✓）并刷新 valid/tick（MAVLink override 语义 ✓）。
/// 固件侧：`uplink::get_rc_override()`（超时 >200 ticks × 10ms = 2s ⇒ 每步刷新即可 ✓）
fn write_rc_override(m: &mut Machine, ch: [u16; 4]) {
    let sym = |n: &str| mcu_simulater::elfsym::app_sym(n) as u64;
    let a = sym("G_RC_OVERRIDE");
    for (i, v) in ch.iter().enumerate() {
        let _ = m.cpu.mem_write(a + (i as u64) * 2, &v.to_le_bytes());
    }
    let _ = m.cpu.mem_write(sym("G_RC_OVERRIDE_VALID"), &[1u8]);
    // 时间戳：用固件自身的 `G_APP_TICKS`（单调 10ms ✓）⇒ 与超时判据同源 ✓
    let tb = m.cpu.mem_read(sym("G_APP_TICKS.0"), 4).unwrap_or_default();
    let t = u32::from_le_bytes([tb[0], tb[1], tb[2], tb[3]]);
    let _ = m.cpu.mem_write(sym("G_RC_OVERRIDE_TICK"), &t.to_le_bytes());
}

/// 读 4 路 PWM 的 CCR/ARR → 归一化推力（同 `x_phy_hover_smoke` ✓）
fn read_thrust(m: &mut Machine) -> [f32; 4] {
    // (TIM 基址, CCR 偏移)：pwm2 在 TIM5 的 CH2 → CCR2
    const TIM3: u64 = 0x4000_0400;
    const TIM2: u64 = 0x4000_0000;
    const TIM5: u64 = 0x4000_0C00;
    const TIM4: u64 = 0x4000_0800;
    const OFF_CRR_CH1: u64 = 0x34;
    const OFF_CRR_CH2: u64 = 0x38;
    const OFF_ARR: u64 = 0x2C;
    let tims = [
        (TIM3, OFF_CRR_CH1),
        (TIM2, OFF_CRR_CH1),
        (TIM5, OFF_CRR_CH2),
        (TIM4, OFF_CRR_CH1),
    ];
    let mut rd = |addr: u64| -> u32 {
        let b = m.cpu.mem_read(addr, 4).unwrap_or_default();
        u32::from_le_bytes([b[0], b[1], b[2], b[3]])
    };
    let mut out = [0f32; 4];
    for (i, &(t, ccr_off)) in tims.iter().enumerate() {
        let arr = rd(t + OFF_ARR) as f32;
        let ccr = rd(t + ccr_off) as f32;
        let duty = if arr > 0.0 { ccr / arr } else { 0.0 };
        let us = duty * 2500.0; // 400Hz 周期 2500us
        out[i] = ((us - 1000.0) / 1000.0).clamp(0.0, 1.0);
    }
    out
}
