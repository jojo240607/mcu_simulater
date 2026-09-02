//! SYSCFG 系统配置控制器（STM32F407，M4 T2 增强）。
//!
//! 基址 `0x40013800`（APB2）。M4-EXTI 阶段仅实现 EXTI 端口选择
//! `EXTICR1-4 @ 0x08..0x14`（每线 2 位，选择 GPIOA..E；复位全 0 = GPIOA），
//! 其余寄存器（MEMRMP/PMC/CMPCR）作为镜像存档，不建模具体行为。
//!
//! EXTICR 与 EXTI 外设共享 [`ExtiPortSelect`]（经 `Arc<Mutex>`），写入即时生效。

use std::sync::{Arc, Mutex};

use crate::peripheral::{BusError, Peripheral};

/// 每条 EXTI 线选择的 GPIO 端口（0=GPIOA .. 4=GPIOE；复位全 0 = GPIOA）
#[derive(Default)]
pub struct ExtiPortSelect {
    pub port: [u8; 16],
}

const OFF_EXTICR1: u32 = 0x08;
const OFF_EXTICR4: u32 = 0x14;

/// SYSCFG 外设（寄存器镜像 + EXTICR 端口选择）
pub struct Syscfg {
    /// 寄存器镜像（MEMRMP..CMPCR）
    regs: [u32; 16],
    /// EXTI 端口选择（与 EXTI 外设共享）
    port_select: Arc<Mutex<ExtiPortSelect>>,
}

impl Syscfg {
    pub fn new(port_select: Arc<Mutex<ExtiPortSelect>>) -> Self {
        Self {
            regs: [0; 16],
            port_select,
        }
    }

    /// 写 EXTICR 字（EXTICR1..4，每字覆盖 4 条线）：同步共享端口选择。
    /// F407 的 EXTIx 字段为 4 位（EXTIx[3:0]）：0=GPIOA..4=GPIOE。
    fn write_exticr(&mut self, cr: u32, value: u32) {
        self.regs[(cr / 4) as usize] = value;
        let line_base = ((cr - OFF_EXTICR1) / 4) as usize * 4;
        let mut sel = self.port_select.lock().unwrap();
        for i in 0..4 {
            let line = line_base + i;
            sel.port[line] = ((value >> (4 * i)) & 0xF) as u8;
        }
    }
}

impl Default for Syscfg {
    fn default() -> Self {
        Self::new(Arc::new(Mutex::new(ExtiPortSelect::default())))
    }
}

impl Peripheral for Syscfg {
    fn name(&self) -> &str {
        "SYSCFG"
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
        if (OFF_EXTICR1..=OFF_EXTICR4).contains(&offset) && offset % 4 == 0 {
            self.write_exticr(offset, value);
            return Ok(());
        }
        let idx = (offset / 4) as usize;
        let slot = self.regs.get_mut(idx).ok_or(BusError::OutOfRange)?;
        *slot = value;
        Ok(())
    }

    fn reset(&mut self) {
        self.regs = [0; 16];
        self.port_select.lock().unwrap().port = [0; 16];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exti_port_selection() {
        let sel = Arc::new(Mutex::new(ExtiPortSelect::default()));
        let mut s = Syscfg::new(sel.clone());
        // EXTICR1 @ 0x08：line0=GPIOB(1)，line1=GPIOC(2)
        s.write(OFF_EXTICR1, 4, 0x0000_0021).unwrap();
        assert_eq!(sel.lock().unwrap().port[0], 1);
        assert_eq!(sel.lock().unwrap().port[1], 2);
        assert_eq!(sel.lock().unwrap().port[2], 0);
        // 读回镜像
        assert_eq!(s.read(OFF_EXTICR1, 4).unwrap(), 0x0000_0021);
        // EXTICR4 @ 0x14：line15=GPIOE(4)（EXTI15 字段 = bits12..15）
        s.write(OFF_EXTICR4, 4, 0x0000_4000).unwrap();
        assert_eq!(sel.lock().unwrap().port[15], 4);
    }
}
