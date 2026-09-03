//! CRC 计算单元（STM32F407，M8 虚拟外设生态）。
//!
//! 32 位 CRC-32/MPEG-2 风格计算单元（与 F407 硬件语义一致）：
//! - 多项式固定 0x04C11DB7（非反射形式）、初始值 0xFFFFFFFF、
//!   无输入/输出位反转、无最终异或（等价 CRC-32/MPEG-2，标准 check=0x0376E6E7）；
//! - 数据按"写入 DR 的位串从 bit31→bit0（MSB 先）"推进——32 位写 = 4 字节
//!   （大端字节序）、16 位写 = 低 2 字节、8 位写 = 低 1 字节；
//! - 读 DR 返回当前 CRC 值（计算中可继续推进）；
//! - CR.RESET（bit0）写 1 复位计算单元（CRC 回 0xFFFFFFFF），位写后自清零；
//! - IDR（独立数据寄存器，低 8 位）用于临时存储，不参与 CRC 计算。
//!
//! 地址映射（CRC @ 0x40023000，`offset` 相对基址）：
//! DR 0x00 / IDR 0x04 / CR 0x08

use crate::peripheral::{BusError, Peripheral};

/// 寄存器偏移
const OFF_DR: u32 = 0x00;
const OFF_IDR: u32 = 0x04;
const OFF_CR: u32 = 0x08;

/// CR.RESET 位（写 1 复位计算单元，位写后自清零）
const CR_RESET: u32 = 1 << 0;

/// CRC-32 多项式（0x04C11DB7，非反射形式）
const POLY: u32 = 0x04C1_1DB7;
/// 初始值（0xFFFFFFFF）
const INIT: u32 = 0xFFFF_FFFF;

/// CRC 计算单元
pub struct Crc {
    /// 寄存器文件（DR/IDR/CR 镜像）
    regs: [u32; 3],
    /// 当前 CRC 累加值（初始 0xFFFFFFFF）
    crc: u32,
}

impl Crc {
    pub fn new() -> Self {
        Self {
            regs: [0; 3],
            crc: INIT,
        }
    }

    /// 推进一个字节的 CRC-32/MPEG-2 计算（MSB-first，非反射）。
    fn push_byte(crc: &mut u32, byte: u8) {
        *crc ^= (byte as u32) << 24;
        for _ in 0..8 {
            if *crc & 0x8000_0000 != 0 {
                *crc = (*crc << 1) ^ POLY;
            } else {
                *crc <<= 1;
            }
        }
    }

    /// 按写入宽度推进 CRC（32 位字按大端字节序 MSB 先，16 位低 2 字节，8 位低 1 字节）。
    fn push(&mut self, size: u32, value: u32) {
        match size {
            4 => {
                Self::push_byte(&mut self.crc, (value >> 24) as u8);
                Self::push_byte(&mut self.crc, (value >> 16) as u8);
                Self::push_byte(&mut self.crc, (value >> 8) as u8);
                Self::push_byte(&mut self.crc, value as u8);
            }
            2 => {
                Self::push_byte(&mut self.crc, (value >> 8) as u8);
                Self::push_byte(&mut self.crc, value as u8);
            }
            1 => Self::push_byte(&mut self.crc, value as u8),
            _ => return,
        }
        // DR 回读镜像：写值原样保存（硬件 DR 可回读上次写入值）
        self.regs[0] = value;
    }

    /// CR.RESET：写 1 复位计算单元（CRC 回初始值 0xFFFFFFFF）。
    fn reset(&mut self) {
        self.crc = INIT;
    }
}

impl Peripheral for Crc {
    fn name(&self) -> &str {
        "CRC"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_DR => Ok(self.crc),          // 读 DR = 当前 CRC 值（计算中）
            OFF_IDR => Ok(self.regs[1] & 0xFF),
            OFF_CR => Ok(0),                 // RESET 位写后自清零，读回恒 0
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        match offset {
            OFF_DR => {
                self.push(size, value);
                Ok(())
            }
            OFF_IDR => {
                self.regs[1] = value & 0xFF;
                Ok(())
            }
            OFF_CR => {
                if value & CR_RESET != 0 {
                    self.reset();
                }
                self.regs[2] = 0; // RESET 写后自清零
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.regs = [0; 3];
        self.crc = INIT;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_mpeg2_known_value() {
        // CRC-32/MPEG-2("123456789") = 0x0376E6E7（标准 check 值，独立锚点）
        let mut c = Crc::new();
        for b in b"123456789" {
            c.push(1, *b as u32);
        }
        assert_eq!(c.crc, 0x0376_E6E7);
    }

    #[test]
    fn crc_initial_is_all_ones() {
        assert_eq!(Crc::new().crc, 0xFFFF_FFFF);
    }

    #[test]
    fn crc_word_write_matches_byte_sequence() {
        // 32 位写 0x31323334 ≡ 按字节推进 [0x31,0x32,0x33,0x34]（MSB 先）
        let mut word = Crc::new();
        let mut bytes = Crc::new();
        word.push(4, 0x3132_3334);
        for b in [0x31u8, 0x32, 0x33, 0x34] {
            bytes.push(1, b as u32);
        }
        assert_eq!(word.crc, bytes.crc);
    }

    #[test]
    fn crc_halfword_matches_byte_sequence() {
        let mut half = Crc::new();
        let mut bytes = Crc::new();
        half.push(2, 0x3334);
        for b in [0x33u8, 0x34] {
            bytes.push(1, b as u32);
        }
        assert_eq!(half.crc, bytes.crc);
    }

    #[test]
    fn crc_reset_restores_initial() {
        let mut c = Crc::new();
        c.push(4, 0x1234_5678);
        assert_ne!(c.crc, INIT);
        c.reset();
        assert_eq!(c.crc, INIT);
    }
}
