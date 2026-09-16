//! 环境场景库：为「虚拟设备直接模拟」注入时间驱动的传感器真值 + 环境扰动 + 故障。
//!
//! 定位（与 SIL/HIL 的区别）：
//! - **SIL**：fly-sim-core 在宿主机直接调用算法（无固件、无总线）。
//! - **HIL**：fly-simulater 物理世界真值经 USB/SRAM3 注入固件（物理闭环）。
//! - **本模块（虚拟设备直接模拟）**：**不跑物理闭环**，测试自行驱动一套
//!   时间参数化的运动学场景（悬停/爬升/巡航/转弯/摆动/阶跃），把真值经
//!   [`FlySimState`]（共享状态 → 虚拟外设 I2C/UART → 固件真实驱动）喂给
//!   real-sensors 固件，专门调试**飞控软件自身**（EKF/控制器/FDIR/模式管理）
//!   在各种飞行环境与传感器异常下的稳定性。
//!
//! 用法（测试侧）：
//! ```ignore
//! let scn = EnvScenario::new(Motion::Hover, Perturb::clean(), vec![]);
//! let st = Arc::new(Mutex::new(FlySimState::default()));
//! m.attach_flysim_sensors(st.clone());
//! m.attach_flysim_uart_slaves(st.clone());
//! for step in 0..N {
//!     scn.advance(dt);                 // 推进运动学 + 应用扰动/故障
//!     scn.write_state(&mut st.lock().unwrap());   // 填充共享状态（run 之间）
//!     m.run(insns);
//!     let tr = scn.truth();            // 真值（断言估计误差）
//! }
//! ```
//!
//! 坐标系约定（与 fly-sim / 固件一致）：**NED**，z 向下为正；姿态欧拉 ZYX
//! （yaw→pitch→roll）；静止水平时加速度计比力 = [0,0,-9.81]（m/s²）。

use crate::peripheral::vperiph::data_source::FlySimState;
use crate::peripheral::vperiph::data_source::rotate_by_quat_conj;

/// 场景真值（世界系 NED + 姿态）。
#[derive(Debug, Clone, Copy, Default)]
pub struct Truth {
    /// 位置 NED（m，向下为正）
    pub pos: [f32; 3],
    /// 速度 NED（m/s）
    pub vel: [f32; 3],
    /// 世界系加速度 NED（m/s²）
    pub accel_world: [f32; 3],
    /// 欧拉角 roll/pitch/yaw（rad）
    pub att: [f32; 3],
    /// 体轴角速度 p/q/r（rad/s）
    pub omega: [f32; 3],
}

/// 运动场景原语（时间参数化闭式，无积分漂移）。
#[derive(Debug, Clone)]
pub enum Motion {
    /// 静态悬停：零运动、水平姿态。
    Hover,
    /// 恒垂直速度爬升/下降（m/s，向上为正；NED pos[2] 相应减小/增大）。
    Vertical { vel_up: f32 },
    /// 恒水平巡航速度（m/s，沿北向）。
    Cruise { vel_n: f32 },
    /// 匀速圆周（协调转弯）：radius（m）、偏航角速度 rate（rad/s，正=左转）。
    Turn { radius: f32, rate: f32 },
    /// 悬停中姿态正弦摆动：axis 0=roll 1=pitch 2=yaw，amp 幅度（rad），
    /// freq 频率（Hz）。
    Oscillate { axis: usize, amp: f32, freq: f32 },
    /// 速度阶跃：t_at 秒后从静止阶跃到 dv（NED，m/s），t_slope 秒斜坡平滑。
    Step { t_at: f32, dv: [f32; 3], t_slope: f32 },
}

/// 各传感器通道高斯噪声标准差（m/s² / rad/s / Pa / m / m/s）。
#[derive(Debug, Clone, Copy)]
pub struct Noise {
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
    pub baro: f32,
    pub gps_pos: [f32; 3],
    pub gps_vel: [f32; 3],
}

impl Default for Noise {
    fn default() -> Self {
        Self {
            accel: [0.02, 0.02, 0.02],     // ~20 mg 级（真机常见）
            gyro: [0.005, 0.005, 0.005],   // ~0.3 °/s
            baro: 0.5,                     // ~4 cm
            gps_pos: [0.5, 0.5, 0.8],      // 消费级 GPS
            gps_vel: [0.1, 0.1, 0.15],
        }
    }
}

/// 环境扰动（作用于传感器输出，不改变真值——测试据此断言估计对扰动的鲁棒性）。
#[derive(Debug, Clone, Copy, Default)]
pub struct Perturb {
    /// 噪声（None = 无噪声）。
    pub noise: Option<Noise>,
    /// 恒定加计偏置（体轴，m/s²）。
    pub accel_bias: [f32; 3],
    /// 陀螺恒定零偏（体轴，rad/s）。
    pub gyro_bias: [f32; 3],
    /// 加计偏置阶跃：(t 秒, 增量 m/s²)。
    pub accel_bias_step: Option<(f32, [f32; 3])>,
    /// 气压高度漂移率（m/s，向上为正；模拟温漂）。
    pub baro_drift: f32,
    /// GPS 位置噪声放大系数（模拟遮挡/多径）。
    pub gps_noise_scale: f32,
    /// SBUS 油门通道偏置（raw，1000..2000；模拟遥控微调）。
    pub rc_throttle_bias: f32,
}

impl Perturb {
    pub fn clean() -> Self {
        Self::default()
    }
}

/// 传感器故障事件（按虚拟时间触发一次）。
#[derive(Debug, Clone, Copy)]
pub enum FaultEvent {
    /// t 秒后 IMU 冻结（保持最后值）。
    ImuFreeze { t: f32 },
    /// t 秒后 IMU 输出钳位到 ±fs（饱和）。
    ImuSaturate { t: f32, fs: f32 },
    /// t 秒时气压计高度阶跃 dalt（m）。
    BaroStep { t: f32, dalt: f32 },
    /// t 秒后气压计冻结。
    BaroFreeze { t: f32 },
    /// t 秒起 GPS 失锁 dur 秒（fix=0，位置/速度清零）。
    GpsDrop { t: f32, dur: f32 },
    /// t 秒时 GPS 位置跳变 d（NED，m）。
    GpsJump { t: f32, d: [f32; 3] },
    /// t 秒起 RC 信号丢失 dur 秒（全部通道回中性、解锁位清零）。
    RcDrop { t: f32, dur: f32 },
    /// t 秒起 SBUS 通道 ch 强置 raw 值 dur 秒（模拟卡滞/抖动）。
    RcStuck { t: f32, ch: usize, raw: f32, dur: f32 },
    /// t 秒后磁力计冻结（保持最后值；数据仍有读数 → FDIR 不误报，靠姿态稳定验证）。
    MagFreeze { t: f32 },
    /// t 秒时注入磁干扰（硬铁偏置 bias，机体系 G；数据持续"正常" → 验证 EKF
    /// 磁观测抗偏 + 姿态不发散）。
    MagDisturb { t: f32, bias: [f32; 3] },
}

/// 传感器故障状态（由 FaultEvent 触发后置位）。
#[derive(Debug, Clone, Copy, Default)]
struct FaultState {
    imu_frozen: bool,
    imu_sat: Option<f32>,
    baro_step_done: bool,
    baro_frozen: bool,
    gps_drop_until: f32,
    gps_jump_done: bool,
    rc_drop_until: f32,
    mag_frozen: bool,
    mag_disturb: Option<[f32; 3]>,
}

/// 环境场景：运动 + 扰动 + 故障 → 传感器输出（写入 FlySimState）。
pub struct EnvScenario {
    motion: Motion,
    perturb: Perturb,
    faults: Vec<FaultEvent>,
    t: f32,
    truth: Truth,
    /// 参考点（GPS 经纬度基准）。
    lat0: f32,
    lon0: f32,
    /// GPS/气压参考高度（m，向上正）。**必须 >0**：固件 u-blox 驱动锁定 NED
    /// 原点时要求 GGA 高度 alt>0（防 RMC 帧 alt=0 误锁）；Hover 场景若参考
    /// 高度 0 则 GPS 永不定点（实测 health 恒 Degraded）。默认 4m 与
    /// x_flyctrl_real_sensors 的 baro_height=4.0 约定一致。
    alt_ref: f32,
    /// 故障注入前的最后 IMU 值（冻结用）。
    last_accel: [f32; 3],
    last_gyro: [f32; 3],
    last_baro_h: f32,
    last_mag: [f32; 3],
    rc_stuck: Vec<(usize, f32, f32)>, // (ch, raw, until)
    fstate: FaultState,
    /// 简单伪随机（xorshift）种子。
    rng: u64,
    last_fired: Vec<bool>,
    rc: [f32; 16],
}

impl EnvScenario {
    pub fn new(motion: Motion, perturb: Perturb, faults: Vec<FaultEvent>) -> Self {
        let n = faults.len();
        Self {
            motion,
            perturb,
            faults,
            t: 0.0,
            truth: Truth::default(),
            lat0: 30.0,
            lon0: 114.0,
            alt_ref: 4.0,
            last_accel: [0.0, 0.0, -9.81],
            last_gyro: [0.0; 3],
            last_baro_h: 0.0,
            last_mag: [0.0; 3],
            rc_stuck: Vec::new(),
            fstate: FaultState::default(),
            rng: 0x9E3779B97F4A7C15,
            last_fired: vec![false; n],
            rc: [1500.0; 16],
        }
    }

    /// 当前虚拟时间（s）。
    pub fn time(&self) -> f32 {
        self.t
    }

    /// 设置 GPS/气压参考高度（m，向上正；须 >0 固件才锁定 GPS 原点）。
    pub fn set_alt_ref(&mut self, v: f32) {
        self.alt_ref = v;
    }

    /// 场景真值（不含扰动/故障）。
    pub fn truth(&self) -> Truth {
        self.truth
    }

    /// 当前 SBUS 通道（含故障/扰动，测试可断言解锁语义）。
    pub fn rc_channels(&self) -> [f32; 16] {
        self.rc
    }

    /// 推进 dt 秒：运动学闭式 + 扰动 + 故障编排。
    pub fn advance(&mut self, dt: f32) {
        self.t += dt;
        let t = self.t;
        // ---- 运动学闭式（按 motion 解析推进） ----
        let (pos, vel, accel_world, att, omega) = match &self.motion {
            Motion::Hover => ([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]),
            Motion::Vertical { vel_up } => {
                let v = *vel_up;
                ([0.0, 0.0, -v * t], [0.0, 0.0, -v], [0.0; 3], [0.0; 3], [0.0; 3])
            }
            Motion::Cruise { vel_n } => {
                let v = *vel_n;
                ([v * t, 0.0, 0.0], [v, 0.0, 0.0], [0.0; 3], [0.0; 3], [0.0; 3])
            }
            Motion::Turn { radius, rate } => {
                let r = *radius;
                let w = *rate;
                // 世界系匀速圆周：初始朝北（yaw=0），x=北 前进；y=东。
                let yaw = w * t;
                let pos = [r * yaw.sin(), r * (1.0 - yaw.cos()), 0.0];
                let vel = [r * w * yaw.cos(), r * w * yaw.sin(), 0.0];
                let accel_world = [-r * w * w * yaw.sin(), r * w * w * yaw.cos(), 0.0];
                // 协调转弯：roll 补偿向心加速度。
                let v = r * w;
                let roll = (v * w / 9.81).clamp(-1.2, 1.2);
                let att = [roll, 0.0, yaw];
                let omega = [0.0, 0.0, w];
                (pos, vel, accel_world, att, omega)
            }
            Motion::Oscillate { axis, amp, freq } => {
                let a = *amp;
                let f = *freq;
                let w = 2.0 * std::f32::consts::PI * f;
                let mut att = [0.0; 3];
                att[*axis] = a * (w * t).sin();
                let mut omega = [0.0; 3];
                omega[*axis] = a * w * (w * t).cos();
                ([0.0; 3], [0.0; 3], [0.0; 3], att, omega)
            }
            Motion::Step { t_at, dv, t_slope } => {
                let ramp = ((t - t_at) / t_slope.max(1e-3)).clamp(0.0, 1.0);
                let vel = [dv[0] * ramp, dv[1] * ramp, dv[2] * ramp];
                // 斜坡段加速度 = dv / t_slope，之后为 0。
                let accel_world = if t < *t_at + *t_slope && t >= *t_at {
                    [dv[0] / t_slope, dv[1] / t_slope, dv[2] / t_slope]
                } else {
                    [0.0; 3]
                };
                // 位移闭式：斜坡段 dv·t_slope·ramp²/2，之后匀速。
                let disp = if t >= t_at + t_slope {
                    (t - t_at) - t_slope * 0.5
                } else {
                    t_slope * ramp * ramp * 0.5
                };
                let pos = [dv[0] * disp, dv[1] * disp, dv[2] * disp];
                (pos, vel, accel_world, [0.0; 3], [0.0; 3])
            }
        };
        self.truth = Truth { pos, vel, accel_world, att, omega };

        // ---- 扰动：偏置阶跃 ----
        if let Some((bt, d)) = self.perturb.accel_bias_step {
            if t >= bt {
                // 阶跃已施加（advance 前 last 保存的是施加前值，见下）
            }
        }

        // ---- 故障触发 ----
        for (i, ev) in self.faults.iter().enumerate() {
            if self.last_fired[i] {
                continue;
            }
            match ev {
                FaultEvent::ImuFreeze { t: ft } if t >= *ft => {
                    self.fstate.imu_frozen = true;
                    self.last_fired[i] = true;
                }
                FaultEvent::ImuSaturate { t: ft, fs } if t >= *ft => {
                    self.fstate.imu_sat = Some(*fs);
                    self.last_fired[i] = true;
                }
                FaultEvent::BaroStep { t: ft, .. } if t >= *ft => {
                    self.fstate.baro_step_done = true;
                    self.last_fired[i] = true;
                }
                FaultEvent::BaroFreeze { t: ft } if t >= *ft => {
                    self.fstate.baro_frozen = true;
                    self.last_fired[i] = true;
                }
                FaultEvent::GpsDrop { t: ft, dur } if t >= *ft => {
                    self.fstate.gps_drop_until = t + *dur;
                    self.last_fired[i] = true;
                }
                FaultEvent::GpsJump { t: ft, .. } if t >= *ft => {
                    self.fstate.gps_jump_done = true;
                    self.last_fired[i] = true;
                }
                FaultEvent::RcDrop { t: ft, dur } if t >= *ft => {
                    self.fstate.rc_drop_until = t + *dur;
                    self.last_fired[i] = true;
                }
                FaultEvent::RcStuck { t: ft, ch, raw, dur } if t >= *ft => {
                    self.rc_stuck.push((*ch, *raw, t + *dur));
                    self.last_fired[i] = true;
                }
                FaultEvent::MagFreeze { t: ft } if t >= *ft => {
                    self.fstate.mag_frozen = true;
                    self.last_fired[i] = true;
                }
                FaultEvent::MagDisturb { t: ft, bias } if t >= *ft => {
                    self.fstate.mag_disturb = Some(*bias);
                    self.last_fired[i] = true;
                }
                _ => {}
            }
        }
    }

    /// 计算传感器输出（含扰动/故障）并写入共享状态。
    pub fn write_state(&mut self, st: &mut FlySimState) {
        let tr = self.truth;
        // ---- 体轴比力（加速度计观测）----
        // a_body = R_bw^T * (a_world - g_world)，g_world = [0,0,9.81]（NED 向下）。
        let g_world = [0.0, 0.0, 9.81];
        let aw = [
            tr.accel_world[0] - g_world[0],
            tr.accel_world[1] - g_world[1],
            tr.accel_world[2] - g_world[2],
        ];
        let (roll, pitch, yaw) = (tr.att[0], tr.att[1], tr.att[2]);
        // R_bw^T = R_wb：ZYX 欧拉 → 世界到机体。
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        // R_wb（世界→机体）= Rz(yaw)^T * Ry(pitch)^T * Rx(roll)^T
        let r00 = cy * cp;
        let r01 = sy * cp;
        let r02 = -sp;
        let r10 = cy * sp * sr - sy * cr;
        let r11 = sy * sp * sr + cy * cr;
        let r12 = cp * sr;
        let r20 = cy * sp * cr + sy * sr;
        let r21 = sy * sp * cr - cy * sr;
        let r22 = cp * cr;
        let mut accel = [
            r00 * aw[0] + r01 * aw[1] + r02 * aw[2],
            r10 * aw[0] + r11 * aw[1] + r12 * aw[2],
            r20 * aw[0] + r21 * aw[1] + r22 * aw[2],
        ];
        // 加计偏置（恒定 + 阶跃）
        let mut bias = self.perturb.accel_bias;
        if let Some((bt, d)) = self.perturb.accel_bias_step {
            if self.t >= bt {
                for k in 0..3 {
                    bias[k] += d[k];
                }
            }
        }
        for k in 0..3 {
            accel[k] += bias[k];
        }
        let mut gyro = tr.omega;
        for k in 0..3 {
            gyro[k] += self.perturb.gyro_bias[k];
        }
        // 噪声
        if let Some(nz) = self.perturb.noise {
            for k in 0..3 {
                accel[k] += nz.accel[k] * self.gauss();
                gyro[k] += nz.gyro[k] * self.gauss();
            }
        }
        // 故障：冻结 / 饱和
        if self.fstate.imu_frozen {
            accel = self.last_accel;
            gyro = self.last_gyro;
        } else {
            self.last_accel = accel;
            self.last_gyro = gyro;
        }
        if let Some(fs) = self.fstate.imu_sat {
            for k in 0..3 {
                accel[k] = accel[k].clamp(-fs, fs);
                gyro[k] = gyro[k].clamp(-fs, fs);
            }
        }
        st.imu_acc = accel;
        st.imu_gyr = gyro;
        // 姿态四元数（w,x,y,z，ZYX 欧拉 → 四元数；供磁力计模型旋转世界地磁场）
        let (sr2, cr2) = (roll * 0.5).sin_cos();
        let (sp2, cp2) = (pitch * 0.5).sin_cos();
        let (sy2, cy2) = (yaw * 0.5).sin_cos();
        let qw = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
        let qx = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
        let qy = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
        let qz = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
        st.att = [qw, qx, qy, qz];

        // ---- 磁力计（世界系恒定地磁场随姿态旋转；故障：硬铁偏置/冻结）----
        // 正常时 st.mag=None → 虚拟外设 FlySimKind::Mag 按 att 推导（与
        // data_source.rs 同一公式）；故障时写 Some 覆盖（数据持续"正常"，FDIR
        // 不误报——按可用性判据有读数即 healthy，姿态稳定性由本场景验证）。
        if self.fstate.mag_disturb.is_some() || self.fstate.mag_frozen {
            let mut mag = rotate_by_quat_conj(&[qw, qx, qy, qz], [0.2f32, 0.0, 0.4]);
            if let Some(b) = self.fstate.mag_disturb {
                for k in 0..3 {
                    mag[k] += b[k];
                }
            }
            if self.fstate.mag_frozen {
                mag = self.last_mag;
            } else {
                self.last_mag = mag;
            }
            st.mag = Some(mag);
        } else {
            st.mag = None;
            self.last_mag = rotate_by_quat_conj(&[qw, qx, qy, qz], [0.2f32, 0.0, 0.4]);
        }

        // ---- 气压计（观测高度 = 参考高度 + 相对位移 + 漂移 + 阶跃 + 噪声；冻结保持最后值） ----
        let alt_up = self.alt_ref - tr.pos[2]; // NED pos[2] 向下 → 高度 = 参考 - pos[2]
        let mut h = alt_up + self.perturb.baro_drift * self.t;
        if self.fstate.baro_step_done {
            if let Some(FaultEvent::BaroStep { dalt, .. }) = self
                .faults
                .iter()
                .find(|e| matches!(e, FaultEvent::BaroStep { .. }))
            {
                h += *dalt;
            }
        }
        if let Some(nz) = self.perturb.noise {
            h += nz.baro * self.gauss();
        }
        if self.fstate.baro_frozen {
            h = self.last_baro_h;
        } else {
            self.last_baro_h = h;
        }
        st.baro_pa = std_pressure(h);

        // ---- GPS ----
        if self.t >= self.fstate.gps_drop_until && self.fstate.gps_drop_until > 0.0 {
            // 已恢复
        }
        if self.t < self.fstate.gps_drop_until {
            st.gps_fix = 0.0;
            st.gps_vel = [0.0; 3];
        } else {
            st.gps_fix = 3.0;
            let mut gps_pos = tr.pos;
            if self.fstate.gps_jump_done {
                if let Some(FaultEvent::GpsJump { d, .. }) = self
                    .faults
                    .iter()
                    .find(|e| matches!(e, FaultEvent::GpsJump { .. }))
                {
                    for k in 0..3 {
                        gps_pos[k] += d[k];
                    }
                }
            }
            if let Some(nz) = self.perturb.noise {
                for k in 0..3 {
                    gps_pos[k] += nz.gps_pos[k] * self.gauss() * self.perturb.gps_noise_scale.max(1.0);
                }
            }
            let dlat = gps_pos[0] / 111_320.0;
            let dlon = gps_pos[1] / (111_320.0 * self.lat0.to_radians().cos());
            st.gps_lat = self.lat0 + dlat;
            st.gps_lon = self.lon0 + dlon;
            st.gps_alt = self.alt_ref - gps_pos[2];
            let mut gvel = tr.vel;
            if let Some(nz) = self.perturb.noise {
                for k in 0..3 {
                    gvel[k] += nz.gps_vel[k] * self.gauss();
                }
            }
            st.gps_vel = gvel;
        }

        // ---- SBUS ----
        // 基础通道：ch3=油门中位（1550 微调偏置）、ch4=解锁（1500 未解锁）。
        let mut ch = [1500.0; 16];
        ch[3] = 1550.0 + self.perturb.rc_throttle_bias;
        // RC 故障：失联（RcDrop）优先于卡滞（RcStuck）——失联时接收机无输出，
        // 卡滞值不适用（真实语义；曾先置 1500 再被 RcStuck 覆盖 → 掉链后解锁位
        // 不恢复，虚拟外设实测 rc_drop_disarms 失败）。
        if self.t < self.fstate.rc_drop_until {
            for c in ch.iter_mut() {
                *c = 1500.0;
            }
        } else {
            for (c, raw, until) in self.rc_stuck.iter() {
                if self.t < *until {
                    ch[*c] = *raw;
                }
            }
        }
        self.rc = ch;
        st.rc_ch = ch;
    }

    /// 标准大气压（Pa）→ 高度向上正（m）。
    fn gauss(&mut self) -> f32 {
        // Box-Muller（xorshift 种子）。
        let mut u1 = self.next_f32().max(1e-9);
        let u2 = self.next_f32();
        let r = (-2.0 * u1.ln()).sqrt();
        (r * (2.0 * std::f32::consts::PI * u2).cos())
    }

    fn next_f32(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        (self.rng >> 40) as f32 / (1u64 << 24) as f32
    }
}

/// 标准大气压（Pa），h 为高度（m，向上为正）。
fn std_pressure(h: f32) -> f32 {
    101_325.0 * (1.0 - 2.255_77e-5 * h).max(0.0).powf(5.255_88)
}
