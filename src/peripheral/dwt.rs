//! DWT（Data Watchpoint and Trace）外设。
//!
//! jOS 依赖 DWT_CYCCNT（0xE0001004）做调度延迟测量：`rtos_cycle_init()` 使能
//! CYCCNT 并清零，`rtos_cycle_now()` 只读计数。模拟器将 CYCCNT 挂入周期外设列表，
//! 由 block hook 按块 `tick(cycles)` 推进，与虚拟时钟同步（分辨率=虚拟周期）。
//!
//! 寄存器（基址 0xE0001000）：
//! - CTRL   @ 0x00：bit0 = CYCCNTENA，写 1 使能周期计数
//! - CYCCNT @ 0x04：32 位周期计数（写置初值，读当前值）
//! 其余 DWT 寄存器未实现（读 0，写忽略）。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use super::{BusError, Peripheral};

/// DWT_CTRL 的 CYCCNTENA 位（bit0）
const CYCCNTENA: u32 = 1 << 0;

/// DWT 外设：CYCCNT 由周期外设 tick 推进。
pub struct Dwt {
    /// DWT_CTRL 寄存器值（仅 CYCCNTENA 位有意义）
    ctrl: u32,
    /// 当前周期计数（由 block hook 的 tick 推进，CYCCNTENA 置位后累加）
    cyccnt: u64,
    /// CYCCNT 使能标记（与其它周期外设一致，供 block hook 跳过未激活外设的加锁 tick）
    pub active: Arc<AtomicBool>,
}

impl Dwt {
    pub fn new() -> Self {
        Self {
            ctrl: 0,
            cyccnt: 0,
            active: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl Peripheral for Dwt {
    fn name(&self) -> &str {
        "DWT"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            0x00 => Ok(self.ctrl),
            0x04 => Ok(self.cyccnt as u32),
            _ => Ok(0),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            0x00 => {
                // 仅 CYCCNTENA 可写；其余位读回 0（简化：忽略其它 DWT 控制位）
                self.ctrl = value & CYCCNTENA;
                self.active.store(self.ctrl & CYCCNTENA != 0, Ordering::Relaxed);
            }
            0x04 => {
                self.cyccnt = value as u64;
            }
            _ => {}
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.ctrl = 0;
        self.cyccnt = 0;
        self.active.store(false, Ordering::Relaxed);
    }

    fn tick(&mut self, cycles: u64) {
        if self.ctrl & CYCCNTENA != 0 {
            self.cyccnt = self.cyccnt.wrapping_add(cycles);
        }
    }
}
