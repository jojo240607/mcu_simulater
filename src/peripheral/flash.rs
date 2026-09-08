//! FLASH 控制器寄存器区（STM32F407，最小模型）。
//!
//! F407 片内 FLASH 控制器 @ 0x40023C00：ACR / KEYR / OPTKEYR / SR / CR。
//! 驱动（drv/flash.c）是"受管扇区管理器"——仅经 FLASH_IOCTL_GET_STATUS 透传
//! 读回 SR（BSY/编程/擦除错误标志），擦写序列由驱动内部 HAL 走 KEYR/CR。
//! 最小模型：寄存器文件镜像 + SR 恒 0（无错误/不忙）、KEYR/CR 写入按真实
//! 位掩码存储（无需真正擦写——受管扇区 11 的数据驻留于后备缓冲，仅验证
//! 寄存器访问路径不 fault）。
//!
//! 地址映射（offset 相对 0x40023C00）：
//! ACR 0x00 / KEYR 0x04 / OPTKEYR 0x08 / SR 0x0C / CR 0x10

use crate::peripheral::{BusError, Peripheral};

/// 寄存器偏移
const OFF_ACR: u32 = 0x00;
const OFF_SR: u32 = 0x0C;
const OFF_CR: u32 = 0x10;

/// SR 位（F407）：BSY=0，编程/擦除错误位只读
const SR_BSY: u32 = 1 << 16;

/// CR 位（部分；写序列经 KEYR 解锁后生效，本模型仅存储）
const CR_PG: u32 = 1 << 0; // 编程
const CR_SER: u32 = 1 << 1; // 扇区擦除
const CR_STRT: u32 = 1 << 16; // 启动擦除/编程

/// FLASH 控制器
pub struct Flash {
    /// 寄存器文件（ACR/KEYR/OPTKEYR/SR/CR）
    regs: [u32; 5],
}

impl Flash {
    pub fn new() -> Self {
        Self {
            // F407 复位值：ACR=0x00000400（LATENCY=0），SR=0（无错误/不忙），CR=0
            regs: [0x0000_0400, 0, 0, 0, 0],
        }
    }

    /// SR 读回（供测试/驱动查询；恒无错误）
    pub fn sr(&self) -> u32 {
        self.regs[(OFF_SR / 4) as usize]
    }
}

impl Peripheral for Flash {
    fn name(&self) -> &str {
        "FLASH"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_ACR | OFF_SR | OFF_CR => {
                let idx = (offset / 4) as usize;
                Ok(self.regs[idx])
            }
            // KEYR/OPTKEYR 只写，读返回 0（F407 语义）
            0x04 | 0x08 => Ok(0),
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_ACR => {
                // 等待周期/预取位可写；保持默认外只存低 8 位
                self.regs[0] = value & 0x1F;
                Ok(())
            }
            // KEYR/OPTKEYR：解锁序列（0x45670123/0xCDEF89AB 等），本模型忽略
            0x04 | 0x08 => Ok(()),
            // SR 只读（错误标志由写 0 清除）：写忽略
            OFF_SR => Ok(()),
            OFF_CR => {
                // 写 1 启动操作；简化：STRT 自清零（立即完成），无真正擦写
                self.regs[4] = value & (CR_PG | CR_SER);
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.regs = [0x0000_0400, 0, 0, 0, 0];
    }
}

/// SR.BSY 位（导出供测试断言）
#[allow(dead_code)]
pub fn sr_bsy() -> u32 {
    SR_BSY
}
