//! 全局执行状态位域：Machine / Nvic / Mpu / Wdog 共享的原子状态字。
//!
//! block hook 每基本块执行一次，需同时判读"MPU 使能 / 外设激活 / 中断挂起 /
//! 看门狗复位请求"等多个低频标志。若各自独立原子，每次块都要多次 load；
//! 合并为单字节位域后热路径仅一次 `raw()` load + 位测试（bench_probe H12：
//! 5 次独立判读 85.5 MIPS → 单状态字 115.1 MIPS）。

use std::sync::atomic::{AtomicU8, Ordering};

/// MPU 使能（CTRL.ENABLE=1，由 Mpu 同步）
pub const BIT_MPU: u8 = 1 << 0;
/// 任一 tick 外设激活（由外设区 MMIO 写置位，只置不清）
pub const BIT_ANY_ACTIVE: u8 = 1 << 1;
/// 任一中断挂起（由 Nvic 同步）
pub const BIT_NVIC_PENDING: u8 = 1 << 2;
/// 看门狗复位请求待处理（由 IWDG/WWDG/低功耗置位）
pub const BIT_WDOG: u8 = 1 << 3;

/// 无锁原子状态字。热路径 `raw()` 一次 load 判读多位；
/// 置位/清位通过 `set`/`clear` 的 RMW 完成（低频事件，非热路径）。
pub struct Status(AtomicU8);

impl Status {
    pub const fn new() -> Self {
        Self(AtomicU8::new(0))
    }

    /// 置位（`fetch_or`，低竞争，Relaxed 足够）
    pub fn set(&self, bit: u8) {
        self.0.fetch_or(bit, Ordering::Relaxed);
    }

    /// 清位
    pub fn clear(&self, bit: u8) {
        self.0.fetch_and(!bit, Ordering::Relaxed);
    }

    /// 判定单一位
    #[inline]
    pub fn has(&self, bit: u8) -> bool {
        self.0.load(Ordering::Relaxed) & bit != 0
    }

    /// 热路径单次 load，调用方按位掩码测试
    #[inline]
    pub fn raw(&self) -> u8 {
        self.0.load(Ordering::Relaxed)
    }
}

impl Default for Status {
    fn default() -> Self {
        Self::new()
    }
}
