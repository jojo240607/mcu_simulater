//! MPU6050 六轴 IMU 从设备（I2C 0x68）。
//!
//! 寄存器 map（固件只依赖这些）：
//! - 0x6B PWR_MGMT_1（写 0 唤醒）
//! - 0x3B..0x48 ACCEL_XOUT_H 起 14 字节：accel×3(2B BE) + temp(2B) + gyro×3(2B BE)
//! - 0x75 WHO_AM_I = 0x68
//!
//! 数据源：`StaticImu`（悬停：accel=[0,0,9.81] 抵消重力、gyro=0）。

use super::super::data_source::{DataSource, SensorModel, StaticImu};
use super::super::RegFileSlave;

/// MPU6050 从设备（I2C 0x68，默认静态 IMU 模型）。
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

/// 默认静态 IMU 模型（悬停：accel 抵消重力、gyro 归零）。
pub fn default_imu() -> StaticImu {
    StaticImu::default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::vperiph::i2c::write_read;
    use crate::peripheral::vperiph::{I2cDir, VirtualI2cSlave};

    #[test]
    fn static_hover_registers() {
        let mut s = mpu6050(StaticImu::default());
        // WHO_AM_I
        assert_eq!(s.peek(0x75), Some(0x68));
        // i2c_write_read(0x3B, 14)：accel BE i16 + temp + gyro BE i16
        let raw = write_read(&mut s, 0x3B, 14);
        // accel.z = 9.81 → raw = 16384 = 0x4000（BE 高字节在前）
        assert_eq!(&raw[4..6], &[0x40, 0x00], "accel.z 应为 +1g");
        // accel.x/y = 0
        assert_eq!(&raw[0..2], &[0x00, 0x00]);
        assert_eq!(&raw[2..4], &[0x00, 0x00]);
        // gyro 全 0
        assert_eq!(&raw[8..14], &[0u8; 6]);
    }

    #[test]
    fn write_pwr_mgmt() {
        let mut s = mpu6050(StaticImu::default());
        // 唤醒写：PWR_MGMT_1(0x6B) ← 0x00（写事务：寄存器地址 + 数据）
        s.on_start(I2cDir::Write);
        s.on_write(0x6B);
        s.on_write(0x00);
        assert_eq!(s.peek(0x6B), Some(0x00));
        assert_eq!(s.n_writes, 2);
    }

    #[test]
    fn nack_injection_returns_none() {
        let mut s = mpu6050(StaticImu::default());
        s.on_start(I2cDir::Write);
        s.on_write(0x3B);
        s.on_start(I2cDir::Read);
        s.nack = true; // 故障注入：断线
        assert!(s.on_read().is_none());
    }
}
