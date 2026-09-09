//! I2C 总线虚拟从设备：器件模型（一器件一文件）。
//!
//! 新增 I2C 器件落点：在 `i2c/` 下新建 `<device>.rs`（寄存器布局 + 工厂函数 +
//! 单元测试），然后在 `mod.rs` 加一行 `pub mod <device>;` 并 re-export 工厂即可——
//! 总线外设（[`crate::peripheral::i2c::I2c`]）只依赖
//! [`super::VirtualI2cSlave`] trait，与具体器件解耦。
//!
//! 首批器件（对齐 flyctrl real-sensors 驱动读法）：
//! - `mpu6050`（六轴 IMU @0x68）
//! - `bmp280`（气压 @0x76）
//! - `qmc5883`（磁力 @0x0D）

pub mod bmp280;
pub mod mpu6050;
pub mod qmc5883;

pub use bmp280::bmp280;
pub use mpu6050::mpu6050;
pub use qmc5883::qmc5883;

/// 默认静态模型装配（悬停：IMU 抵消重力、气压海平面、地磁北向）。
///
/// 对应 `attach_default_sensors()`（I2C1：mpu6050/bmp280/qmc5883）。
pub fn default_i2c_slaves() -> Vec<crate::peripheral::vperiph::RegFileSlave> {
    use super::data_source::{StaticBaro, StaticImu, StaticMag};
    vec![
        mpu6050(StaticImu::default()),
        bmp280(StaticBaro::default()),
        qmc5883(StaticMag::default()),
    ]
}

/// 模拟固件 `i2c_write_read(addr, reg, n)`：写事务设寄存器指针 → 读事务连续读。
///
/// 供各器件单元测试复用（`#[cfg(test)]` 之外无使用）。
#[cfg(test)]
pub(crate) use crate::peripheral::vperiph::{I2cDir, VirtualI2cSlave};

#[cfg(test)]
pub(crate) fn write_read(slave: &mut dyn VirtualI2cSlave, reg: u8, n: usize) -> Vec<u8> {
    slave.on_start(I2cDir::Write);
    slave.on_write(reg);
    slave.on_start(I2cDir::Read);
    (0..n).map(|_| slave.on_read().unwrap()).collect()
}
