//! TIM2 通用定时器（STM32F407，M3 T1 集）。
//!
//! M3 语义：按 [`Peripheral::tick`] 推进的虚拟周期驱动 CNT 递增
//! （块级加权周期，见 [`crate::sim::timing`]），计数溢出时置 SR.UIF，
//! 若 DIER.UIE 使能则向共享 NVIC 置挂起 IRQ28（TIM2 中断），
//! 由 Machine 的 block hook / 中断投递链路完成响应。
//!
//! 地址映射（offset 相对 TIM2 基址 0x40000000）：
//! - CR1 0x00（CEN=bit0, UDIS=bit1）/ DIER 0x0C（UIE=bit0）
//! - SR 0x10（UIF=bit0，写 0 清除）/ EGR 0x14（UG=bit0，软件更新）
//! - CNT 0x24 / PSC 0x28 / ARR 0x2C

use std::sync::{Arc, Mutex};

use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};

/// STM32F407 上 TIM2 的中断号
pub const TIM2_IRQ: u32 = 28;

/// CR1 控制位
const CR1_CEN: u32 = 1 << 0; // 计数器使能
/// DIER 中断使能位
const DIER_UIE: u32 = 1 << 0; // 更新中断使能
/// SR 状态位
const SR_UIF: u32 = 1 << 0; // 更新标志

/// 寄存器偏移
const OFF_CR1: u32 = 0x00;
const OFF_DIER: u32 = 0x0C;
const OFF_SR: u32 = 0x10;
const OFF_EGR: u32 = 0x14;
const OFF_CNT: u32 = 0x24;
const OFF_PSC: u32 = 0x28;
const OFF_ARR: u32 = 0x2C;

/// TIM2 外设
pub struct Tim2 {
    /// 寄存器文件（CR1..CCR4 共 17 个 32 位寄存器）
    regs: [u32; 17],
    /// 计数器时钟余数（PSC 分频的亚周期累积）
    prescaler_remainder: u64,
    /// 共享 NVIC（更新事件 → 置挂起 IRQ28）
    nvic: Arc<Mutex<Nvic>>,
}

impl Tim2 {
    pub fn new(nvic: Arc<Mutex<Nvic>>) -> Self {
        Self {
            regs: [0; 17],
            prescaler_remainder: 0,
            nvic,
        }
    }

    /// 生成一次更新事件：置 UIF，UIE 使能时向 NVIC 置挂起
    fn update_event(&mut self) {
        self.regs[OFF_SR as usize / 4] |= SR_UIF;
        if self.regs[OFF_DIER as usize / 4] & DIER_UIE != 0 {
            self.nvic.lock().unwrap().set_pending(TIM2_IRQ);
        }
    }

    /// 当前 CNT 值（测试读取用）
    pub fn count(&self) -> u32 {
        self.regs[OFF_CNT as usize / 4]
    }
}

impl Peripheral for Tim2 {
    fn name(&self) -> &str {
        "TIM2"
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
        match offset {
            OFF_EGR => {
                // UG：软件生成更新事件
                if value & 1 != 0 {
                    self.update_event();
                    // 向上计数更新事件：CNT 回 0
                    self.regs[OFF_CNT as usize / 4] = 0;
                    self.prescaler_remainder = 0;
                }
                Ok(())
            }
            OFF_SR => {
                // 状态位写 0 清除（rc_w0）
                self.regs[OFF_SR as usize / 4] &= value;
                Ok(())
            }
            _ => {
                let idx = (offset / 4) as usize;
                let slot = self.regs.get_mut(idx).ok_or(BusError::OutOfRange)?;
                *slot = value;
                Ok(())
            }
        }
    }

    fn tick(&mut self, cycles: u64) {
        // 未使能或禁用更新（UDIS）时不推进
        let cr1 = self.regs[OFF_CR1 as usize / 4];
        if cr1 & CR1_CEN == 0 {
            return;
        }
        let psc = (self.regs[OFF_PSC as usize / 4] & 0xFFFF) as u64 + 1;
        let arr = self.regs[OFF_ARR as usize / 4];

        // 分频：累积周期，按 (PSC+1) 折算计数器步进
        self.prescaler_remainder += cycles;
        let steps = self.prescaler_remainder / psc;
        self.prescaler_remainder %= psc;
        if steps == 0 {
            return;
        }

        let mut cnt = self.regs[OFF_CNT as usize / 4] as u64 + steps;
        // 向上计数：超过 ARR 即溢出回绕，生成更新事件
        while cnt > arr as u64 {
            cnt -= arr as u64 + 1;
            self.update_event();
        }
        self.regs[OFF_CNT as usize / 4] = cnt as u32;
    }

    fn reset(&mut self) {
        self.regs = [0; 17];
        self.prescaler_remainder = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tim2() -> (Tim2, Arc<Mutex<Nvic>>) {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        (Tim2::new(nvic.clone()), nvic)
    }

    #[test]
    fn overflow_sets_pending_and_wraps() {
        let (mut t, nvic) = tim2();
        // PSC=1（分频 2），ARR=100
        t.write(OFF_PSC, 4, 1).unwrap();
        t.write(OFF_ARR, 4, 100).unwrap();
        t.write(OFF_DIER, 4, DIER_UIE).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();

        // 未使能时不推进
        let (mut t2, _) = tim2();
        t2.write(OFF_ARR, 4, 10).unwrap();
        t2.tick(1000);
        assert_eq!(t2.count(), 0);

        // 使能后：1000 周期 / 2 = 500 步，ARR=100 → 4 次溢出，CNT=500-4*101=96
        t.tick(1000);
        assert_eq!(t.count(), 96);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF);
        assert!(nvic.lock().unwrap().is_pending(TIM2_IRQ), "UIE 使能应置挂起");

        // 写 SR 清 UIF（写 0 清除）
        t.write(OFF_SR, 4, 0).unwrap();
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, 0);
    }

    #[test]
    fn disabled_update_event_no_pending() {
        let (mut t, nvic) = tim2();
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap(); // CEN 但 UIE 未使能
        t.tick(1000);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF);
        assert!(!nvic.lock().unwrap().is_pending(TIM2_IRQ), "UIE 未使能不置挂起");
    }
}
