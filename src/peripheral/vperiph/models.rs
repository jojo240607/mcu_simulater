//! 首批虚拟 I2C 传感器模型（对齐 flyctrl real-sensors 驱动读法）：
//! - `mpu6050` @ 0x68：WHO_AM_I + PWR_MGMT + ACCEL_XOUT_H(0x3B) 起 14B 动态（BE i16）
//! - `bmp280`  @ 0x76：ID(0xD0) + PRESS_MSB(0xF7) 起 6B 动态（20bit 压力原始值）
//! - `qmc5883` @ 0x0D：DATA_X_L(0x00) 起 6B 动态（LE i16 三轴磁力）
//!
//! 装配函数返回 [`RegFileSlave`]（默认静态物理模型数据源），由总线挂载/TOML
//! 拓扑按需实例化；动态寄存器在固件读取瞬间从数据源求值（真实传感器语义）。

use super::data_source::{DataSource, SensorModel, StaticBaro, StaticImu, StaticMag};
use super::RegFileSlave;

/// MPU6050 从设备（I2C 0x68）。
///
/// 寄存器 map（固件只依赖这些）：
/// - 0x6B PWR_MGMT_1（写 0 唤醒）
/// - 0x3B..0x48 ACCEL_XOUT_H 起 14 字节：accel×3(2B BE) + temp(2B) + gyro×3(2B BE)
/// - 0x75 WHO_AM_I = 0x68
pub fn mpu6050(imu: impl SensorModel + 'static) -> RegFileSlave {
    let mut s = RegFileSlave::new("mpu6050", 0x68, vec![0u8; 128]);
    // WHO_AM_I
    s.poke(0x75, 0x68);
    // 数据源：IMU 模型
    let src = s.add_source(DataSource::Math(Box::new(imu)));
    // ACCEL_XOUT_H 起 14B：i16 BE
    //   accel: ±2g → LSB=16384/g；gyro: ±250°/s → LSB=131/(°/s)
    s.add_dynamic(0x3B, 14, src, |ds, i| {
        let be = |v: f32| (v as i16).to_be_bytes();
        match i {
            0..=1 => be(ds.value("accel.x") * 16384.0 / 9.81)[i & 1],
            2..=3 => be(ds.value("accel.y") * 16384.0 / 9.81)[i & 1],
            4..=5 => be(ds.value("accel.z") * 16384.0 / 9.81)[i & 1],
            6..=7 => 0u8, // 温度（驱动不读，置 0）
            // gyro: rad/s → °/s → raw（驱动 raw/131*π/180 = rad/s）
            8..=9 => be(ds.value("gyro.x") * 131.0 * 180.0 / core::f32::consts::PI)[i & 1],
            10..=11 => be(ds.value("gyro.y") * 131.0 * 180.0 / core::f32::consts::PI)[i & 1],
            12..=13 => be(ds.value("gyro.z") * 131.0 * 180.0 / core::f32::consts::PI)[i & 1],
            _ => 0,
        }
    });
    s
}

/// BMP280 从设备（I2C 0x76）。
///
/// 寄存器 map（固件只依赖这些）：
/// - 0xD0 ID = 0x58
/// - 0xF7 PRESS_MSB 起 6 字节：p 20bit（驱动 `(raw<<16|raw<<8|raw)>>4` 直接当 Pa，占位未校准）
pub fn bmp280(baro: impl SensorModel + 'static) -> RegFileSlave {
    let mut s = RegFileSlave::new("bmp280", 0x76, vec![0u8; 256]);
    s.poke(0xD0, 0x58); // ID
    let src = s.add_source(DataSource::Math(Box::new(baro)));
    // 压力原始值 = Pa << 4 的 20bit（驱动 >>4 还原 Pa）
    s.add_dynamic(0xF7, 6, src, |ds, i| {
        let p20 = (ds.value("pressure") as u32) << 4;
        match i {
            0 => ((p20 >> 16) & 0xFF) as u8,
            1 => ((p20 >> 8) & 0xFF) as u8,
            2 => (p20 & 0xFF) as u8,
            // 温度原始值（驱动不读，给 25°C 折算值）
            3 => 0x00,
            4 => 0x66,
            5 => 0x26,
            _ => 0,
        }
    });
    s
}

/// QMC5883L 从设备（I2C 0x0D）。
///
/// 寄存器 map（固件只依赖这些）：
/// - 0x00 DATA_X_L 起 6 字节：三轴磁力（LE i16；量程 ±2G → ±32768）
pub fn qmc5883(mag: impl SensorModel + 'static) -> RegFileSlave {
    let mut s = RegFileSlave::new("qmc5883", 0x0D, vec![0u8; 0x40]);
    let src = s.add_source(DataSource::Math(Box::new(mag)));
    // LE i16：raw = gauss * 32768 / 2（±2G 量程）
    s.add_dynamic(0x00, 6, src, |ds, i| {
        let le = |v: f32| ((v * 32768.0 / 2.0) as i16).to_le_bytes();
        match i {
            0..=1 => le(ds.value("mag.x"))[i & 1],
            2..=3 => le(ds.value("mag.y"))[i & 1],
            4..=5 => le(ds.value("mag.z"))[i & 1],
            _ => 0,
        }
    });
    s
}

/// 默认静态模型装配（悬停：IMU 抵消重力、气压海平面、地磁北向）。
pub fn default_i2c_slaves() -> Vec<RegFileSlave> {
    vec![
        mpu6050(StaticImu::default()),
        bmp280(StaticBaro::default()),
        qmc5883(StaticMag::default()),
    ]
}
