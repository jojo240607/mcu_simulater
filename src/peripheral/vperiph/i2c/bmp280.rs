//! BMP280 气压传感器从设备（I2C 0x76）。
//!
//! 寄存器 map（固件只依赖这些）：
//! - 0xD0 ID = 0x58
//! - 0xF7 PRESS_MSB 起 6 字节：p 20bit（驱动 `(raw<<16|raw<<8|raw)>>4` 直接当 Pa，占位未校准）
//!
//! 数据源：`StaticBaro`（海平面 101325 Pa）。

use super::super::data_source::{DataSource, SensorModel, StaticBaro};
use super::super::RegFileSlave;

/// BMP280 从设备（I2C 0x76，默认静态气压模型）。
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

/// 默认静态气压模型（海平面 101325 Pa）。
pub fn default_baro() -> StaticBaro {
    StaticBaro::default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::vperiph::i2c::write_read;
    use crate::peripheral::vperiph::{I2cDir, VirtualI2cSlave};

    #[test]
    fn pressure_read() {
        let mut s = bmp280(StaticBaro::default());
        assert_eq!(s.peek(0xD0), Some(0x58)); // ID
        // i2c_write_read(0xF7, 6)：20bit 压力原始值（Pa<<4）
        let raw = write_read(&mut s, 0xF7, 6);
        let p20 = ((raw[0] as u32) << 16) | ((raw[1] as u32) << 8) | (raw[2] as u32);
        let p = p20 >> 4;
        assert_eq!(p, 101_325, "压力原始值应为海平面气压");
    }

    #[test]
    fn read_preserves_pointer_across_transactions() {
        // 真实硬件：寄存器指针跨事务保持（写设指针 → 读直接从指针吐）
        let mut s = bmp280(StaticBaro::default());
        s.on_start(I2cDir::Write);
        s.on_write(0xF7);
        // 读事务直接读，无需再写寄存器地址
        s.on_start(I2cDir::Read);
        let first = s.on_read().unwrap();
        let p20 = ((first as u32) << 16)
            | ((s.on_read().unwrap() as u32) << 8)
            | (s.on_read().unwrap() as u32);
        assert_eq!(p20 >> 4, 101_325);
    }
}
