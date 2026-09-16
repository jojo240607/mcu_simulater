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

// ---------------------------------------------------------------------------
// 数字域缺陷（virtual_direct_mode.md §9）：模拟真实传感器数字链路的保真度损失。
// 属于 mcu_sim 外设层（模拟器侧），不污染 fly_sim 物理模型。
// ---------------------------------------------------------------------------

/// 数字域缺陷配置。
///
/// 真实传感器模拟量 → ADC → 数字总线的链路会引入三类系统性损失，固件驱动
/// 看到的寄存器值永远带这些痕迹——调试固件时（EKF 融合、健康监测阈值）应
/// 复现它们，否则固件在仿真里"太干净"而掩盖真实缺陷：
/// - **量化**：`adc_bits` 位 ADC（如 12bit）→ 输出阶梯（LSB = 2·FS / 2^bits）
/// - **量程饱和**：`full_scale` 满量程（如 ±16g）→ 超量程钳位
/// - **ODR 降采样**：`odr_hz` 输出数据率（IMU 1kHz / 气压计 25Hz）→ 零阶保持
///   （两次输出之间读到的值不变，模拟"数据寄存器只在 ODR 节拍更新"）
/// - **传输延迟**：`delay_s` 采样→寄存器可见延迟（ADC 转换 + 总线时序）→
///   读到的值是 delay 秒前的
///
/// 全部可选（None = 无该缺陷）；包装 [`DigitalModel`] 透明作用于设备寄存器
/// 填充（`DataSource::value` 链路），不改动物理模型本身。
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DigitalDefect {
    /// 量化位深（None = 全精度）。LSB = 2·FS / 2^bits。
    pub adc_bits: Option<u8>,
    /// 满量程（±FS 饱和钳位；量化基准）。
    pub full_scale: Option<f32>,
    /// 输出数据率 Hz（零阶保持）。
    pub odr_hz: Option<f32>,
    /// 传输延迟秒（读回 delay 前的值）。
    pub delay_s: Option<f32>,
}

impl DigitalDefect {
    /// 预置配置：12bit 满量程 ±16g、1kHz ODR、0.5ms 延迟（通用 IMU 数字链路）。
    pub fn imu_typical() -> Self {
        Self {
            adc_bits: Some(12),
            full_scale: Some(16.0),
            odr_hz: Some(1000.0),
            delay_s: Some(0.0005),
        }
    }

    /// 预置配置：20bit、25Hz ODR、5ms 延迟（气压计典型）。
    pub fn baro_typical() -> Self {
        Self {
            adc_bits: Some(20),
            full_scale: None,
            odr_hz: Some(25.0),
            delay_s: Some(0.005),
        }
    }
}

/// 数字域缺陷包装器：透明改写 [`SensorModel::value`] 输出（量化/饱和/ODR/延迟）。
///
/// 用法：把任意 SensorModel（Static/FlySimSource/自定义）包一层后传给设备工厂
/// （`mpu6050(…)` / `bmp280(…)` 等）——寄存器填充链路自动带上缺陷痕迹：
/// ```rust
/// let imu = DigitalModel::wrap(
///     Box::new(FlySimSource::new(st, FlySimKind::Imu)),
///     DigitalDefect::imu_typical(),
/// );
/// machine.register_i2c_slave(1, Box::new(mpu6050(imu)));
/// ```
pub struct DigitalModel {
    inner: Box<dyn SensorModel>,
    defect: DigitalDefect,
    /// 累计虚拟时间（step 推进；value 的 ODR/延迟时间基准）。
    /// 用 Mutex 而非 Cell/RefCell：SensorModel 要求 Send+Sync（虚拟外设跨线程），
    /// value() 调用频率低（寄存器读时刷新），锁开销可忽略。
    sim_t: std::sync::Mutex<f32>,
    /// ODR 零阶保持：field → (上次更新时间, 当前保持值)
    hold: std::sync::Mutex<std::collections::HashMap<String, (f32, f32)>>,
    /// 延迟历史：field → (时间, 值) 采样序列（窗口裁剪，读回 delay 前的值）
    hist: std::sync::Mutex<std::collections::HashMap<String, Vec<(f32, f32)>>>,
}

impl DigitalModel {
    pub fn wrap(inner: Box<dyn SensorModel>, defect: DigitalDefect) -> Box<dyn SensorModel> {
        Box::new(Self {
            inner,
            defect,
            sim_t: std::sync::Mutex::new(0.0),
            hold: std::sync::Mutex::new(std::collections::HashMap::new()),
            hist: std::sync::Mutex::new(std::collections::HashMap::new()),
        })
    }

    /// 内部原始值（测试/对比用）。
    pub fn raw_value(&self, field: &str) -> f32 {
        self.inner.value(field)
    }
}

impl SensorModel for DigitalModel {
    fn name(&self) -> &str {
        "digital"
    }

    fn step(&mut self, dt: f32) {
        self.inner.step(dt);
        *self.sim_t.lock().unwrap() += dt;
    }

    fn value(&self, field: &str) -> f32 {
        let mut v = self.inner.value(field);
        let t = *self.sim_t.lock().unwrap();

        // 1) 量程饱和（超量程钳位；顺带作为量化的基准 FS）
        let fs = self.defect.full_scale;
        if let Some(f) = fs {
            v = v.clamp(-f, f);
        }

        // 2) 量化：输出阶梯，LSB = 2·FS / 2^bits（FS 缺省按 ±1 计）
        if let Some(bits) = self.defect.adc_bits {
            let f = fs.unwrap_or(1.0);
            let lsb = (2.0 * f) / (1u64 << bits) as f32;
            v = (v / lsb).round() * lsb;
        }

        // 3) ODR 零阶保持：两次输出之间保持上次值（数据寄存器只在 ODR 节拍更新）
        if let Some(odr) = self.defect.odr_hz {
            let period = 1.0 / odr;
            let mut hold = self.hold.lock().unwrap();
            // 首读：entry 初始为"已过期"（t-period 且值为当前读），首读即输出新值
            let entry = hold.entry(field.to_string()).or_insert((t - period, v));
            if t - entry.0 >= period {
                *entry = (t, v);
            } else {
                v = entry.1;
            }
        }

        // 4) 传输延迟：读回 delay 秒前的值（按采样历史线性近似；窗口裁剪防增长）
        if let Some(delay) = self.defect.delay_s {
            let mut hist = self.hist.lock().unwrap();
            let h = hist.entry(field.to_string()).or_default();
            h.push((t, v));
            let cutoff = t - delay - 1.0;
            h.retain(|(ht, _)| *ht >= cutoff);
            let target = t - delay;
            let mut chosen = v;
            for (ht, hv) in h.iter().rev() {
                if *ht <= target {
                    chosen = *hv;
                    break;
                }
            }
            v = chosen;
        }

        v
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
            accel: [0.0, 0.0, -9.81], // 静止于水平面（FRD z 向下）：比力 z 轴 = -9.81
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

impl StaticBaro {
    /// 按指定高度（m，向上为正）构造静态气压（ISA 反解海平面公式）。
    /// 与虚拟 GPS 高度基准对齐：baro 与 GPS 高度不一致会令 EKF 高度被
    /// 气压观测拉偏（历史观察：海平面 baro vs GPS alt=4m → EKF 收敛 0.17m）。
    pub fn at_height(height_m: f32) -> Self {
        let p = 101_325.0 * (1.0 - height_m / 44330.0).powf(1.0 / 0.1903);
        Self { pressure: p }
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

/// 静态 SBUS 模型：恒定摇杆通道值（中位 1500；`ch3` 油门）。
#[derive(Clone, Debug)]
pub struct StaticSbus {
    /// 通道 0..15（1000..2000）
    pub channels: [f32; 16],
}

impl Default for StaticSbus {
    fn default() -> Self {
        let mut c = [1500.0f32; 16];
        c[3] = 1000.0; // ch3 油门最小（解锁/怠速验证）
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
///
/// **一致性不变量（锁步）**：本状态只在物理步间隙（两次 `machine::run()`
/// 之间）被写入，`run()` 期间必须保持冻结。虚拟外设的 `FlySimSource::value()`
/// 在固件读取瞬间从本状态取值，多字节读事务（如 14 字节 IMU burst）的字节
/// 一致性完全依赖该锁步——**禁止任何线程在 `run()` 进行中写入本状态**，
/// 否则会发生跨物理步撕裂（accel.x 来自步 k、accel.y 来自步 k+1）。
/// 参见 `docs/virtual_direct_mode.md` §8.1。
#[derive(Clone, Debug, Default)]
pub struct FlySimState {
    /// 机体系加速度（比力，m/s²；FRD z 向下，静止水平时 z=-9.81）
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
    /// GPS NED 速度（m/s；北/东/下，向下正）。Doppler 速度经 `$GNRMC` 帧下发，
    /// 固件 EKF `update_vel` 用它约束水平速度（否则长时间悬停水平速度纯积分漂移）。
    pub gps_vel: [f32; 3],
    /// SBUS 通道 0..15（1000..2000；ch3=油门）
    pub rc_ch: [f32; 16],
    /// 世界系→机体姿态四元数（w,x,y,z）。由场景 write_state 写入，供磁力计
    /// 模型把**世界系恒定地磁场**旋转到机体（真机磁场世界系恒定、机体测量 =
    /// R·磁场世界；StaticMag 固定机体磁场不随姿态转 → 固件 yaw 观测错误，
    /// 磁锚定把 yaw 拉回固定航向，实测转弯 yaw 积分慢 5.5 倍）。
    pub att: [f32; 4],
    /// 磁力计机体系输出覆盖（G）。`Some` 时优先于 att 推导（场景故障注入：
    /// 硬铁偏置/冻结），`None` 时由 `FlySimKind::Mag` 按 att 推导真机模型。
    pub mag: Option<[f32; 3]>,
}

/// FlySimSource 的数据角色（决定 value() 解析哪些通道）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FlySimKind {
    Imu,
    Baro,
    Gps,
    Sbus,
    Mag,
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
            FlySimKind::Mag => "flysim_mag",
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
                "vel_n" => st.gps_vel[0],
                "vel_e" => st.gps_vel[1],
                "vel_d" => st.gps_vel[2],
                _ => 0.0,
            }
            FlySimKind::Sbus => {
                let idx: usize = field.strip_prefix("ch").and_then(|n| n.parse().ok()).unwrap_or(16);
                if idx < 16 {
                    st.rc_ch[idx]
                } else {
                    0.0
                }
            }
            // 磁力计：世界系恒定地磁场（北 0.2G、下 0.4G）随姿态旋转到机体。
            // 真机磁场方向世界系恒定，机体测量随姿态变化 → yaw 可观测。
            // （StaticMag 固定机体磁场是错误模型：yaw 观测恒定，磁锚定拉回航向，
            // 实测转弯 yaw 积分慢 5.5 倍。）
            FlySimKind::Mag => {
                let [w, x, y, z] = st.att;
                let n = (w * w + x * x + y * y + z * z).sqrt();
                let q = if n > 1e-6 {
                    [w / n, x / n, y / n, z / n]
                } else {
                    [1.0, 0.0, 0.0, 0.0]
                };
                let m_b = match st.mag {
                    Some(m) => m, // 场景故障注入覆盖（硬铁偏置/冻结）
                    None => rotate_by_quat_conj(&q, [0.2f32, 0.0, 0.4]),
                };
                match field {
                    "mag.x" => m_b[0],
                    "mag.y" => m_b[1],
                    "mag.z" => m_b[2],
                    _ => 0.0,
                }
            }
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    /// 共享状态模型：测试经 Arc 引用改值（包装后仍可驱动，验证 ODR/延迟）。
    #[derive(Clone)]
    struct SharedCtl {
        state: Arc<Mutex<std::collections::HashMap<String, f32>>>,
    }
    impl SharedCtl {
        fn new() -> (Self, Arc<Mutex<std::collections::HashMap<String, f32>>>) {
            let state = Arc::new(Mutex::new(std::collections::HashMap::new()));
            (Self { state: state.clone() }, state)
        }
    }
    impl SensorModel for SharedCtl {
        fn name(&self) -> &str {
            "shared"
        }
        fn step(&mut self, _dt: f32) {}
        fn value(&self, field: &str) -> f32 {
            self.state.lock().unwrap().get(field).copied().unwrap_or(0.0)
        }
    }

    #[test]
    fn quantization_creates_steps() {
        let (ctl, st) = SharedCtl::new();
        st.lock().unwrap().insert("accel.z".into(), -9.81);
        let d = DigitalModel::wrap(Box::new(ctl), DigitalDefect {
            adc_bits: Some(8),
            full_scale: Some(16.0),
            ..Default::default()
        });
        // 8bit、±16g：LSB = 32/256 = 0.125 → -9.81 量化到 0.125 整数倍
        let v = d.value("accel.z");
        let nearest = (-9.81f32 / 0.125).round() * 0.125;
        assert!((v - nearest).abs() < 1e-6, "应量化到 LSB 整数倍：v={v} nearest={nearest}");
        assert!((v - (-9.81)).abs() > 1e-3, "量化后应偏离原始值（缺陷可见）");
    }

    #[test]
    fn full_scale_saturation_clamps() {
        let (ctl, st) = SharedCtl::new();
        st.lock().unwrap().insert("accel.z".into(), 25.0);
        let d = DigitalModel::wrap(Box::new(ctl), DigitalDefect {
            full_scale: Some(16.0),
            ..Default::default()
        });
        assert_eq!(d.value("accel.z"), 16.0, "超量程应钳位到 +FS");
    }

    #[test]
    fn odr_zero_order_hold_holds_between_ticks() {
        let (ctl, st) = SharedCtl::new();
        let mut d = DigitalModel::wrap(Box::new(ctl), DigitalDefect {
            odr_hz: Some(10.0), // 100ms 输出一次
            ..Default::default()
        });
        // t=0：首读 1.0（初始 hold）
        st.lock().unwrap().insert("accel.x".into(), 1.0);
        assert_eq!(d.value("accel.x"), 1.0);

        // 改值但时间未过 ODR 周期（step 50ms < 100ms）→ 保持旧值
        st.lock().unwrap().insert("accel.x".into(), 9.0);
        d.step(0.05);
        assert_eq!(d.value("accel.x"), 1.0, "ODR 未到节拍应保持上次输出");

        // 时间越过周期（再 60ms > 100ms）→ 更新为新值
        d.step(0.06);
        assert_eq!(d.value("accel.x"), 9.0, "ODR 节拍到应输出新值");
    }

    #[test]
    fn delay_returns_past_value() {
        let (ctl, st) = SharedCtl::new();
        let mut d = DigitalModel::wrap(Box::new(ctl), DigitalDefect {
            delay_s: Some(0.1),
            ..Default::default()
        });
        // t=0：采样 v=1.0
        st.lock().unwrap().insert("x".into(), 1.0);
        assert_eq!(d.value("x"), 1.0);
        // t=0.05：改值 9.0 并采样（读回的是 0.05-0.1<0 时刻 → 无历史 → 当前值）
        st.lock().unwrap().insert("x".into(), 9.0);
        d.step(0.05);
        assert_eq!(d.value("x"), 9.0);
        // t=0.12：读回 0.02 时刻的值 → 1.0（延迟可见）
        d.step(0.07);
        assert_eq!(d.value("x"), 1.0, "延迟读回应返回 delay 前的历史值");
        // t=0.2：读回 0.1 时刻 → 9.0（历史已更新）
        d.step(0.08);
        assert_eq!(d.value("x"), 9.0);
    }

    #[test]
    fn typical_profiles() {
        let d = DigitalDefect::imu_typical();
        assert_eq!(d.adc_bits, Some(12));
        assert_eq!(d.full_scale, Some(16.0));
        assert_eq!(d.odr_hz, Some(1000.0));
        assert!(d.delay_s.unwrap() > 0.0);
        let b = DigitalDefect::baro_typical();
        assert_eq!(b.odr_hz, Some(25.0));
        assert!(b.delay_s.unwrap() > 0.0);
    }
}

/// 共享静态模型：value 恒返回当前设定值，测试/脚本可经 `Arc` 引用实时改值。
///
/// 用途：集成测试与故障剧本需要"包装后改传感器值"（ODR/延迟/卡死验证），
/// 而 `DigitalModel` 的 inner 不可达——共享模型经外部 Arc 驱动即可。
#[derive(Clone, Default)]
pub struct SharedStatic {
    state: Arc<Mutex<std::collections::HashMap<String, f32>>>,
}

impl SharedStatic {
    pub fn new() -> Self {
        Self::default()
    }

    /// 设定通道值（返回 self，链式构造）。
    pub fn with(mut self, field: &str, v: f32) -> Self {
        self.set(field, v);
        self
    }

    /// 设定通道值。
    pub fn set(&mut self, field: &str, v: f32) {
        self.state.lock().unwrap().insert(field.to_string(), v);
    }

    /// 共享状态句柄（外部驱动改值）。
    pub fn state(&self) -> Arc<Mutex<std::collections::HashMap<String, f32>>> {
        self.state.clone()
    }
}

impl SensorModel for SharedStatic {
    fn name(&self) -> &str {
        "shared_static"
    }
    fn step(&mut self, _dt: f32) {}
    fn value(&self, field: &str) -> f32 {
        self.state.lock().unwrap().get(field).copied().unwrap_or(0.0)
    }
}

/// 用四元数共轭旋转向量（世界→机体；q 为世界→机体姿态 w,x,y,z）。
pub(crate) fn rotate_by_quat_conj(q: &[f32; 4], v: [f32; 3]) -> [f32; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    // v' = q^* ⊗ v ⊗ q（共轭 = 逆，单位四元数）
    // 计算 t = q^* ⊗ v
    let t = [
        -x * v[0] - y * v[1] - z * v[2],
        w * v[0] + y * v[2] - z * v[1],
        w * v[1] + z * v[0] - x * v[2],
        w * v[2] + x * v[1] - y * v[0],
    ];
    // v' = t ⊗ q（取向量部分）
    [
        w * t[1] - t[2] * z + t[3] * y - t[0] * x,
        w * t[2] + t[1] * z - t[3] * x - t[0] * y,
        w * t[3] - t[1] * y + t[2] * x - t[0] * z,
    ]
}
