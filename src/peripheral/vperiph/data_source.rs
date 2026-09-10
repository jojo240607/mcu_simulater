//! 虚拟外设数据源：Constant（静态）+ Math（物理模型，随仿真时间推进）。
//!
//! `DataSource` 是动态寄存器填充回调取值的统一入口（`value(field)`），
//! `SensorModel` 是物理模型 trait：`step(dt)` 推进状态、`value(field)` 取通道值。

use std::fmt;
use std::sync::{Arc, Mutex};

/// 数据源：从设备动态寄存器的取值来源。
pub enum DataSource {
    /// 常量（忽略 field 名，恒返回该值）——静态场景/单值注入。
    Const(f32),
    /// 物理模型（随仿真时间推进）。
    Math(Box<dyn SensorModel>),
}

impl DataSource {
    /// 取某通道当前值（未知通道返回 0.0）。
    pub fn value(&self, field: &str) -> f32 {
        match self {
            DataSource::Const(v) => *v,
            DataSource::Math(m) => m.value(field),
        }
    }

    /// 推进数据源（Math 模型 step；Const 无操作）。
    pub fn step(&mut self, dt: f32) {
        if let DataSource::Math(m) = self {
            m.step(dt);
        }
    }

    /// 模型名（观测）。
    pub fn name(&self) -> &str {
        match self {
            DataSource::Const(_) => "const",
            DataSource::Math(m) => m.name(),
        }
    }
}

impl fmt::Debug for DataSource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DataSource::Const(v) => write!(f, "Const({v})"),
            DataSource::Math(m) => write!(f, "Math({})", m.name()),
        }
    }
}

/// 物理模型 trait：随仿真时间推进，按通道名取值。
///
/// 通道命名约定（首批）：
/// - IMU：`accel.x/y/z`（m/s²）、`gyro.x/y/z`（rad/s）
/// - Baro：`pressure`（Pa）、`altitude`（m，向下为正）
/// - Mag：`mag.x/y/z`（无单位原始比例）
/// - GPS：`lat/lon/alt`、`vel_n/vel_e/vel_d`、`fix`（0/1/2/3）
/// - SBUS：`ch0..ch15`（1000..2000）
pub trait SensorModel: Send + Sync {
    /// 模型名（观测/日志）
    fn name(&self) -> &str;

    /// 仿真时间推进（`dt` 秒）
    fn step(&mut self, dt: f32);

    /// 取某通道当前值（未知通道返回 0.0）
    fn value(&self, field: &str) -> f32;
}

impl SensorModel for Box<dyn SensorModel> {
    fn name(&self) -> &str {
        (**self).name()
    }
    fn step(&mut self, dt: f32) {
        (**self).step(dt)
    }
    fn value(&self, field: &str) -> f32 {
        (**self).value(field)
    }
}

/// 静态 IMU 模型：恒定 accel/gyro（悬停 = accel 抵消重力，gyro 归零）。
#[derive(Clone, Debug)]
pub struct StaticImu {
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
}

impl Default for StaticImu {
    fn default() -> Self {
        Self {
            accel: [0.0, 0.0, 9.81], // 静止于水平面：z 轴抵消重力（加速度计测比力）
            gyro: [0.0, 0.0, 0.0],
        }
    }
}

impl SensorModel for StaticImu {
    fn name(&self) -> &str {
        "static_imu"
    }
    fn step(&mut self, _dt: f32) {}
    fn value(&self, field: &str) -> f32 {
        match field {
            "accel.x" => self.accel[0],
            "accel.y" => self.accel[1],
            "accel.z" => self.accel[2],
            "gyro.x" => self.gyro[0],
            "gyro.y" => self.gyro[1],
            "gyro.z" => self.gyro[2],
            _ => 0.0,
        }
    }
}

/// 静态气压模型：恒定气压/高度（海拔 0m ≈ 101325 Pa）。
#[derive(Clone, Debug)]
pub struct StaticBaro {
    /// 气压（Pa）
    pub pressure: f32,
}

impl Default for StaticBaro {
    fn default() -> Self {
        Self { pressure: 101_325.0 }
    }
}

impl SensorModel for StaticBaro {
    fn name(&self) -> &str {
        "static_baro"
    }
    fn step(&mut self, _dt: f32) {}
    fn value(&self, field: &str) -> f32 {
        match field {
            "pressure" => self.pressure,
            // 向下为正的高度（气压→高度 ISA 近似）
            "altitude" => 44330.0 * (1.0 - (self.pressure / 101_325.0).powf(0.1903)),
            _ => 0.0,
        }
    }
}

/// 静态磁力计模型：恒定三轴磁场（无单位原始比例，量程 ±2G）。
#[derive(Clone, Debug)]
pub struct StaticMag {
    pub mag: [f32; 3],
}

impl Default for StaticMag {
    fn default() -> Self {
        // 北向地磁 ≈ 水平分量 ~0.2G（模拟器量程 2G，原始值 ~4096 对应 2G）
        Self { mag: [0.2, 0.0, 0.4] }
    }
}

impl SensorModel for StaticMag {
    fn name(&self) -> &str {
        "static_mag"
    }
    fn step(&mut self, _dt: f32) {}
    fn value(&self, field: &str) -> f32 {
        match field {
            "mag.x" => self.mag[0],
            "mag.y" => self.mag[1],
            "mag.z" => self.mag[2],
            _ => 0.0,
        }
    }
}

/// 静态 GPS 模型：恒定位置/速度（fix=3D）。
#[derive(Clone, Debug)]
pub struct StaticGps {
    /// 纬度（度）
    pub lat: f32,
    /// 经度（度）
    pub lon: f32,
    /// 海拔（米）
    pub alt: f32,
    /// 北/东/下 速度（m/s）
    pub vel: [f32; 3],
}

impl Default for StaticGps {
    fn default() -> Self {
        Self {
            lat: 31.2304,
            lon: 121.4737,
            alt: 4.0,
            vel: [0.0, 0.0, 0.0],
        }
    }
}

impl SensorModel for StaticGps {
    fn name(&self) -> &str {
        "static_gps"
    }
    fn step(&mut self, _dt: f32) {}
    fn value(&self, field: &str) -> f32 {
        match field {
            "lat" => self.lat,
            "lon" => self.lon,
            "alt" => self.alt,
            "vel_n" => self.vel[0],
            "vel_e" => self.vel[1],
            "vel_d" => self.vel[2],
            "fix" => 3.0,
            _ => 0.0,
        }
    }
}

/// 静态 SBUS 模型：恒定摇杆通道值（中位 1500；`ch2`/`ch3` 油门）。
#[derive(Clone, Debug)]
pub struct StaticSbus {
    /// 通道 0..15（1000..2000）
    pub channels: [f32; 16],
}

impl Default for StaticSbus {
    fn default() -> Self {
        let mut c = [1500.0f32; 16];
        c[2] = 1000.0; // ch2 油门最小（解锁/怠速验证）
        Self { channels: c }
    }
}

impl SensorModel for StaticSbus {
    fn name(&self) -> &str {
        "static_sbus"
    }
    fn step(&mut self, _dt: f32) {}
    fn value(&self, field: &str) -> f32 {
        if let Some(rest) = field.strip_prefix("ch") {
            if let Ok(idx) = rest.parse::<usize>() {
                if idx < self.channels.len() {
                    return self.channels[idx];
                }
            }
        }
        0.0
    }
}

/// 空数据源提供者（为 trait 对象保留入口）。
#[derive(Default)]
pub struct NopProvider;


// ─────────────────────────────────────────────────────────────
// fly_simulater 直通注入源：FlySimSource
//
// fly_sim 物理引擎每步把真值写入共享 `FlySimState`（Arc<Mutex>），虚拟外设
// 动态寄存器（IMU/气压/GPS/SBUS）经 FlySimSource 即时读到该状态，固件标准
// 驱动照常读寄存器 —— 实现"PC 物理世界 ↔ MCU 虚拟外设"同进程直通。
// ─────────────────────────────────────────────────────────────

/// fly_sim 物理引擎写入的共享传感器/RC 状态（每 4ms 物理步更新）。
#[derive(Clone, Debug, Default)]
pub struct FlySimState {
    /// 机体系加速度（比力，m/s²；静止水平时 z=+9.81 抵消重力——与 mpu6050 设备约定一致）
    pub imu_acc: [f32; 3],
    /// 机体系角速度（rad/s）
    pub imu_gyr: [f32; 3],
    /// 气压（Pa）
    pub baro_pa: f32,
    /// GPS 位置（度/米）+ 定位状态
    pub gps_lat: f32,
    pub gps_lon: f32,
    pub gps_alt: f32,
    pub gps_fix: f32,
    /// SBUS 通道 0..15（1000..2000；ch2=油门）
    pub rc_ch: [f32; 16],
}

/// FlySimSource 的数据角色（决定 value() 解析哪些通道）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FlySimKind {
    Imu,
    Baro,
    Gps,
    Sbus,
}

/// 直通注入源：value() 从共享 FlySimState 读取。
pub struct FlySimSource {
    state: Arc<Mutex<FlySimState>>,
    kind: FlySimKind,
}

impl FlySimSource {
    pub fn new(state: Arc<Mutex<FlySimState>>, kind: FlySimKind) -> Self {
        Self { state, kind }
    }
}

impl SensorModel for FlySimSource {
    fn name(&self) -> &str {
        match self.kind {
            FlySimKind::Imu => "flysim_imu",
            FlySimKind::Baro => "flysim_baro",
            FlySimKind::Gps => "flysim_gps",
            FlySimKind::Sbus => "flysim_sbus",
        }
    }
    fn step(&mut self, _dt: f32) {}
    fn value(&self, field: &str) -> f32 {
        let st = self.state.lock().unwrap();
        match self.kind {
            FlySimKind::Imu => match field {
                "accel.x" => st.imu_acc[0],
                "accel.y" => st.imu_acc[1],
                "accel.z" => st.imu_acc[2],
                "gyro.x" => st.imu_gyr[0],
                "gyro.y" => st.imu_gyr[1],
                "gyro.z" => st.imu_gyr[2],
                _ => 0.0,
            },
            FlySimKind::Baro => {
                if field == "pressure" {
                    st.baro_pa
                } else {
                    0.0
                }
            }
            FlySimKind::Gps => match field {
                "lat" => st.gps_lat,
                "lon" => st.gps_lon,
                "alt" => st.gps_alt,
                "fix" => st.gps_fix,
                _ => 0.0,
            },
            FlySimKind::Sbus => {
                let idx: usize = field.strip_prefix("ch").and_then(|n| n.parse().ok()).unwrap_or(16);
                if idx < 16 {
                    st.rc_ch[idx]
                } else {
                    0.0
                }
            }
        }
    }
}
