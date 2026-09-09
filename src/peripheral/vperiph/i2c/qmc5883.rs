//! QMC5883L 磁力计从设备（I2C 0x0D）。
//!
//! 寄存器 map（固件只依赖这些）：
//! - 0x00 DATA_X_L 起 6 字节：三轴磁力（LE i16；量程 ±2G → ±32768）
//!
//! 数据源：`StaticMag`（北向地磁 [0.2,0,0.4] G）。

use super::super::data_source::{DataSource, SensorModel, StaticMag};
use super::super::RegFileSlave;

/// QMC5883L 从设备（I2C 0x0D，默认静态磁力模型）。
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

/// 默认静态磁力模型（北向地磁）。
pub fn default_mag() -> StaticMag {
    StaticMag::default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::vperiph::i2c::write_read;

    #[test]
    fn le_mag_read() {
        let mut s = qmc5883(StaticMag::default());
        // i2c_write_read(0x00, 6)：LE i16 三轴
        let raw = write_read(&mut s, 0x00, 6);
        // mag.x = 0.2G → raw = 0.2*32768/2 = 3276.8 → 截断 3276（LE：低字节在前）
        let x = i16::from_le_bytes([raw[0], raw[1]]);
        assert_eq!(x, 3276, "mag.x 0.2G → raw 3276");
        // mag.z = 0.4G → 6553.6 → 截断 6553
        let z = i16::from_le_bytes([raw[4], raw[5]]);
        assert_eq!(z, 6553, "mag.z 0.4G → raw 6553");
    }
}
