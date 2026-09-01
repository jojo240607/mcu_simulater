//! System Control Block（SCB）占位外设。
//!
//! M1 作为内存总线 MMIO 转发的首个演示目标：寄存器文件式外设，
//! 读写保持"镜像 RAM"语义（读返回最后写入值，初始为 0），
//! 使固件对 CPACR（0xE000ED88）等的读写能走通完整链路：
//! CPU 访问 → Unicorn mem hook → 内存总线 → 本外设。
//! M2/M3 起替换为真实语义（NVIC/SysTick/MPU 等）。

use super::{BusError, Peripheral};

/// SCB 占位外设：寄存器文件式镜像。
pub struct SystemControl {
    /// 寄存器文件（每项 4 字节，保存最后写入值）
    regs: Vec<u32>,
}

impl SystemControl {
    /// 创建 SCB 占位外设，`size` 为寄存器区间字节数（4 字节对齐）
    pub fn new(size: u32) -> Self {
        Self {
            regs: vec![0; (size / 4) as usize],
        }
    }
}

impl Peripheral for SystemControl {
    fn name(&self) -> &str {
        "SCB"
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
}
