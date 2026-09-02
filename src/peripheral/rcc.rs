//! RCC 复位与时钟控制（M3 T1 集：存根）。
//!
//! M3 仅实现"使能寄存器镜像"语义：固件写 AHB1/APB1/APB2 外设时钟使能位
//! （如 AHB1ENR.GPIOA、APB1ENR.TIM2、APB2ENR.USART1）时能正常写入，
//! 不做真实时钟树/分频（留待 M4）。读回镜像值，便于验证使能位。

use crate::peripheral::{BusError, Peripheral};

/// RCC 外设（寄存器文件镜像）
pub struct Rcc {
    /// 寄存器文件（CR..CSR 等 26 个 32 位寄存器）
    regs: [u32; 26],
}

impl Rcc {
    pub fn new() -> Self {
        Self { regs: [0; 26] }
    }
}

impl Default for Rcc {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Rcc {
    fn name(&self) -> &str {
        "RCC"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let idx = (offset / 4) as usize;
        self.regs.get(idx).copied().ok_or(BusError::OutOfRange)
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let idx = (offset / 4) as usize;
        let slot = self.regs.get_mut(idx).ok_or(BusError::OutOfRange)?;
        *slot = value;
        Ok(())
    }

    fn reset(&mut self) {
        self.regs = [0; 26];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enable_bit_mirror() {
        let mut r = Rcc::new();
        // AHB1ENR @ 0x40023830 → 相对基址 0x30，bit0=GPIOA 时钟
        r.write(0x30, 4, 1).unwrap();
        assert_eq!(r.read(0x30, 4).unwrap(), 1);
        // APB1ENR @ 0x40023840 → 0x40，bit0=TIM2
        r.write(0x40, 4, 1).unwrap();
        assert_eq!(r.read(0x40, 4).unwrap(), 1);
    }
}
