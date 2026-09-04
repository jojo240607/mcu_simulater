//! 看门狗：IWDG 独立看门狗 + WWDG 窗口看门狗（STM32F407，M4）。
//!
//! 复位机制：IWDG/WWDG 在 [`Peripheral::tick`] 中判定超时/窗口违规后，向共享
//! [`WdogResetReq`] 发出复位请求；Machine 的 block hook 检测到请求即停机，
//! `run()` 在 CPU 间隙执行系统复位（置 RCC_CSR 复位标志 + 重载向量表 + 复位看门狗）。
//!
//! IWDG（独立看门狗 @ 0x40003000，时钟独立于主时钟，无中断）：
//! - KR 0x00 键寄存器：0x5555 解锁 PR/RLR，0xAAAA 重装载，0xCCCC 启动；
//! - PR 0x04 预分频器（bits2:0，分频 = 4 << PR）/ RLR 0x08 重装载（bits11:0）；
//! - SR 0x0C 状态（RVU=bit0, PVU=bit1，更新完成由下次 tick 清除）。
//! 递减计数器从 RLR 递减至 0 即超时 → 复位请求（不喂狗）。
//!
//! WWDG（窗口看门狗 @ 0x40002C00，时钟 = PCLK1 / 4096 / 2^WDGTB，无窗口违规即复位）：
//! - CR 0x00（WDGA=bit7 激活，T[6:0] 递减计数器，读回实时计数）；
//! - CFR 0x04（EWI=bit9 早期唤醒中断使能，WDGTB[8:7] 分频，W[6:0] 窗口值）；
//! - SR 0x08（EWIF=bit0，写 0 清除）。
//! 窗口语义：计数器 > W 时写 CR（刷新）→ 窗口违规复位；计数器跨过 0x40 置 EWIF
//! 并（EWI 使能时）挂起 IRQ0；计数器 < 0x40（T6 清零）→ 超时复位。

use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};

use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};
use crate::sim::status::{Status, BIT_WDOG};

/// WWDG 在 STM32F407 上的中断号（IRQ0）
pub const WWDG_IRQ: u32 = 0;

/// 看门狗复位原因（对应 RCC_CSR 复位标志位，F407 硬件位）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetReason {
    /// 独立看门狗超时（CSR.IWDGRSTF = bit28）
    Iwdg,
    /// 窗口看门狗超时/窗口违规（CSR.WWDGRSTF = bit27）
    Wwdg,
    /// 低功耗唤醒复位（PWR 待机唤醒；CSR.LPWRRSTF = bit31）
    LowPower,
}

/// 共享看门狗复位请求：IWDG/WWDG 置位，Machine 消费（见 [`Machine::run`]）。
///
/// 无锁原子实现：block hook 每基本块调用 `is_pending()`，Mutex 开销在性能敏感
/// 路径上不可忽略，改用 `AtomicU8`（0=None, 1=Iwdg, 2=Wwdg, 3=LowPower）。
/// 同时联动全局状态字 BIT_WDOG：`request()` 置位（block hook 据此停机）、
/// `take()` 清位（消费后放行），使 block hook 热路径免去二次原子判读。
pub struct WdogResetReq {
    req: AtomicU8,
    /// 全局状态字联动（可选：单测独立构造时为 None，不更新 BIT_WDOG）
    status: Option<Arc<Status>>,
}

impl Default for WdogResetReq {
    fn default() -> Self {
        Self::new()
    }
}

impl WdogResetReq {
    /// 独立构造（单元测试用）：不联动全局状态字
    pub fn new() -> Self {
        Self {
            req: AtomicU8::new(0),
            status: None,
        }
    }

    /// 正式构造：与全局状态字联动（request/take 同步 BIT_WDOG）
    pub fn with_status(status: Arc<Status>) -> Self {
        Self {
            req: AtomicU8::new(0),
            status: Some(status),
        }
    }

    /// 发出复位请求（多个看门狗同时超时以后写覆盖先写）
    pub fn request(&self, reason: ResetReason) {
        let code = match reason {
            ResetReason::Iwdg => 1,
            ResetReason::Wwdg => 2,
            ResetReason::LowPower => 3,
        };
        self.req.store(code, Ordering::Relaxed);
        if let Some(s) = &self.status {
            s.set(BIT_WDOG);
        }
    }

    /// 是否有待处理复位请求（探测/测试用；block hook 改用全局状态字 BIT_WDOG）
    pub fn is_pending(&self) -> bool {
        self.req.load(Ordering::Relaxed) != 0
    }

    /// 取走复位请求（消费后为 None，并清除全局状态字 BIT_WDOG）
    pub fn take(&self) -> Option<ResetReason> {
        let r = match self.req.swap(0, Ordering::Relaxed) {
            1 => Some(ResetReason::Iwdg),
            2 => Some(ResetReason::Wwdg),
            3 => Some(ResetReason::LowPower),
            _ => None,
        };
        if r.is_some() {
            if let Some(s) = &self.status {
                s.clear(BIT_WDOG);
            }
        }
        r
    }
}

// ---------------------------------------------------------------------------
// IWDG 独立看门狗
// ---------------------------------------------------------------------------

/// IWDG 寄存器偏移（基址 0x40003000）
const IW_OFF_KR: u32 = 0x00;
const IW_OFF_PR: u32 = 0x04;
const IW_OFF_RLR: u32 = 0x08;
const IW_OFF_SR: u32 = 0x0C;

/// KR 键值
const KR_RELOAD: u32 = 0xAAAA; // 重装载计数器
const KR_START: u32 = 0xCCCC; // 启动看门狗
const KR_UNLOCK: u32 = 0x5555; // 解锁 PR/RLR 写访问

/// SR 状态位
const SR_RVU: u32 = 1 << 0; // 重装载值更新进行中
const SR_PVU: u32 = 1 << 1; // 预分频值更新进行中

/// IWDG 外设
pub struct Iwdg {
    /// 寄存器文件（KR/PR/RLR/SR）
    regs: [u32; 4],
    /// 内部递减计数器（从 RLR 递减至 0 即超时）
    down: u32,
    /// 是否已启动（写 KR=0xCCCC）
    enabled: bool,
    /// PR/RLR 是否处于写解锁状态（KR=0x5555 置位，写入后上锁）
    unlocked: bool,
    /// 计数器时钟余数（分频亚周期累积）
    remainder: u64,
    /// 共享复位请求
    req: Arc<WdogResetReq>,
    /// 活动标记（enabled，KR=0xCCCC 启动）：Machine block hook 据此跳过未激活
    /// 看门狗的加锁 tick
    active: Arc<AtomicBool>,
}

impl Iwdg {
    /// 便捷构造（单元测试用）：活动标记为一次性占位，不与 Machine 联动
    pub fn new(req: Arc<WdogResetReq>) -> Self {
        Self::with_active(req, Arc::new(AtomicBool::new(false)))
    }

    /// 正式构造：`active` 由 Machine 持有（与 iwdg 字段并行），KR=0xCCCC 启动时同步
    pub fn with_active(req: Arc<WdogResetReq>, active: Arc<AtomicBool>) -> Self {
        Self {
            regs: [0; 4],
            down: 0,
            enabled: false,
            unlocked: false,
            remainder: 0,
            req,
            active,
        }
    }

    /// 重装载：down = RLR（保持使能状态不变）
    fn reload(&mut self) {
        self.down = self.regs[(IW_OFF_RLR / 4) as usize] & 0xFFF;
    }

    /// 当前递减计数器值（测试读取用）
    pub fn down_count(&self) -> u32 {
        self.down
    }
}

impl Peripheral for Iwdg {
    fn name(&self) -> &str {
        "IWDG"
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
            IW_OFF_KR => {
                match value {
                    KR_UNLOCK => self.unlocked = true,
                    KR_RELOAD => {
                        if self.enabled {
                            self.reload();
                        }
                    }
                    KR_START => {
                        self.enabled = true;
                        self.active.store(true, Ordering::Relaxed);
                        self.reload();
                    }
                    _ => self.unlocked = false, // 非法键值重新上锁
                }
                Ok(())
            }
            IW_OFF_PR => {
                // 仅解锁态可写；写入后上锁并置 PVU（更新完成由下次 tick 清除）
                if !self.unlocked {
                    return Ok(()); // 硬件写保护：忽略
                }
                self.unlocked = false;
                self.regs[(IW_OFF_PR / 4) as usize] = value & 0x7;
                self.regs[(IW_OFF_SR / 4) as usize] |= SR_PVU;
                self.remainder = 0; // 分频变化：重置余数避免瞬时大步进
                Ok(())
            }
            IW_OFF_RLR => {
                if !self.unlocked {
                    return Ok(());
                }
                self.unlocked = false;
                self.regs[(IW_OFF_RLR / 4) as usize] = value & 0xFFF;
                self.regs[(IW_OFF_SR / 4) as usize] |= SR_RVU;
                Ok(())
            }
            IW_OFF_SR => {
                // SR 只读：写忽略
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn tick(&mut self, cycles: u64) {
        if !self.enabled {
            return; // 未启动不计时
        }
        // 分频/重装载更新完成：清 RVU/PVU
        self.regs[(IW_OFF_SR / 4) as usize] &= !(SR_RVU | SR_PVU);

        let pr = (self.regs[(IW_OFF_PR / 4) as usize] & 0x7) as u64;
        let prescaler = 4u64 << pr; // 分频 = 4 << PR
        self.remainder += cycles;
        let steps = self.remainder / prescaler;
        self.remainder %= prescaler;
        if steps == 0 {
            return;
        }
        if self.down as u64 > steps {
            self.down -= steps as u32;
        } else {
            // 超时：递减到 0 → 复位请求（Machine 复位看门狗后停止）
            self.down = 0;
            self.req.request(ResetReason::Iwdg);
        }
    }

    fn reset(&mut self) {
        self.regs = [0; 4];
        self.down = 0;
        self.enabled = false;
        self.unlocked = false;
        self.remainder = 0;
        // 复位清除 enabled → 活动标记同步为未激活
        self.active.store(false, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// WWDG 窗口看门狗
// ---------------------------------------------------------------------------

/// WWDG 寄存器偏移（基址 0x40002C00）
const WW_OFF_CR: u32 = 0x00;
const WW_OFF_CFR: u32 = 0x04;
const WW_OFF_SR: u32 = 0x08;

/// CR 控制位
const CR_WDGA: u32 = 1 << 7; // 激活看门狗
/// CFR 配置位
const CFR_EWI: u32 = 1 << 9; // 早期唤醒中断使能

/// WWDG 外设
pub struct Wwdg {
    /// 寄存器文件（CR/CFR/SR）
    regs: [u32; 3],
    /// 内部 7 位递减计数器
    counter: u32,
    /// 计数器时钟余数（分频亚周期累积）
    remainder: u64,
    /// 共享 NVIC（早期唤醒 → 挂起 IRQ0）
    nvic: Arc<Mutex<Nvic>>,
    /// 共享复位请求
    req: Arc<WdogResetReq>,
    /// 活动标记（CR.WDGA 激活）：Machine block hook 据此跳过未激活看门狗的加锁 tick
    active: Arc<AtomicBool>,
}

impl Wwdg {
    /// 便捷构造（单元测试用）：活动标记为一次性占位，不与 Machine 联动
    pub fn new(nvic: Arc<Mutex<Nvic>>, req: Arc<WdogResetReq>) -> Self {
        Self::with_active(nvic, req, Arc::new(AtomicBool::new(false)))
    }

    /// 正式构造：`active` 由 Machine 持有（与 wwdg 字段并行），CR.WDGA 置位时同步
    pub fn with_active(
        nvic: Arc<Mutex<Nvic>>,
        req: Arc<WdogResetReq>,
        active: Arc<AtomicBool>,
    ) -> Self {
        Self {
            regs: [0; 3],
            counter: 0x7F,
            remainder: 0,
            nvic,
            req,
            active,
        }
    }

    /// 当前递减计数器值（测试读取用）
    pub fn counter(&self) -> u32 {
        self.counter
    }
}

impl Peripheral for Wwdg {
    fn name(&self) -> &str {
        "WWDG"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            // CR 读回实时计数器（T[6:0] 反映递减中的计数，WDGA 保持写值）
            WW_OFF_CR => Ok((self.regs[(WW_OFF_CR / 4) as usize] & CR_WDGA) | (self.counter & 0x7F)),
            _ => {
                let idx = (offset / 4) as usize;
                self.regs.get(idx).copied().ok_or(BusError::OutOfRange)
            }
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            WW_OFF_CR => {
                let wdga = value & CR_WDGA;
                let t = value & 0x7F;
                if self.regs[(WW_OFF_CR / 4) as usize] & CR_WDGA != 0 {
                    // 已激活 → 本次是刷新：窗口检查（counter > W → 窗口违规复位）
                    let w = self.regs[(WW_OFF_CFR / 4) as usize] & 0x7F;
                    if self.counter > w {
                        self.req.request(ResetReason::Wwdg);
                        return Ok(());
                    }
                }
                // 激活或刷新：WDGA 只可置位，T 写入 → 计数器重载
                self.regs[(WW_OFF_CR / 4) as usize] |= wdga;
                // 同步活动标记：WDGA 一经置位保持（写 WDGA=0 不清除）
                self.active
                    .store(self.regs[(WW_OFF_CR / 4) as usize] & CR_WDGA != 0, Ordering::Relaxed);
                self.counter = t;
                Ok(())
            }
            WW_OFF_CFR => {
                // 激活后硬件写保护（WDGA 已置位时忽略 CFR 写入）
                if self.regs[(WW_OFF_CR / 4) as usize] & CR_WDGA != 0 {
                    return Ok(());
                }
                let idx = (WW_OFF_CFR / 4) as usize;
                self.regs[idx] = value;
                Ok(())
            }
            WW_OFF_SR => {
                // 写 0 清除 EWIF（rc_w0，写 1 无效）
                self.regs[(WW_OFF_SR / 4) as usize] &= value;
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn tick(&mut self, cycles: u64) {
        if self.regs[(WW_OFF_CR / 4) as usize] & CR_WDGA == 0 {
            return; // 未激活不计时
        }
        let wdgtb = (self.regs[(WW_OFF_CFR / 4) as usize] >> 7) & 0x3;
        let prescaler = 4096u64 << wdgtb; // 分频 = 4096 × 2^WDGTB
        self.remainder += cycles;
        let steps = self.remainder / prescaler;
        self.remainder %= prescaler;
        if steps == 0 {
            return;
        }

        let old = self.counter;
        self.counter = self.counter.saturating_sub(steps as u32);

        // 早期唤醒：跨过 0x40（T6 清零的临界值）→ 置 EWIF +（EWI 使能时）挂起 IRQ0
        if old > 0x40 && self.counter <= 0x40 {
            self.regs[(WW_OFF_SR / 4) as usize] |= 1; // EWIF
            if self.regs[(WW_OFF_CFR / 4) as usize] & CFR_EWI != 0 {
                self.nvic.lock().unwrap().set_pending(WWDG_IRQ);
            }
        }
        // 超时复位：T6 清零（counter < 0x40）
        if self.counter < 0x40 {
            self.req.request(ResetReason::Wwdg);
        }
    }

    fn reset(&mut self) {
        self.regs = [0; 3];
        self.counter = 0x7F;
        self.remainder = 0;
        // 复位清除 WDGA → 活动标记同步为未激活
        self.active.store(false, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- IWDG ----

    #[test]
    fn iwdg_start_then_timeout_requests_reset() {
        let req = Arc::new(WdogResetReq::new());
        let mut w = Iwdg::new(req.clone());

        // 解锁 → 配 PR=0(÷4)、RLR=0x100 → 启动
        w.write(IW_OFF_KR, 4, KR_UNLOCK).unwrap();
        w.write(IW_OFF_PR, 4, 0).unwrap();
        w.write(IW_OFF_KR, 4, KR_UNLOCK).unwrap();
        w.write(IW_OFF_RLR, 4, 0x100).unwrap();
        w.write(IW_OFF_KR, 4, KR_START).unwrap();
        assert_eq!(w.down_count(), 0x100);

        // 未喂狗：推进 0x100*4 周期 → 递减到 0 → 复位请求
        w.tick(0x3FF);
        assert!(!req.is_pending(), "0x100-1 步不应超时");
        w.tick(4);
        assert_eq!(w.down_count(), 0);
        assert!(req.is_pending(), "递减到 0 应触发复位");
        assert_eq!(req.take(), Some(ResetReason::Iwdg));
    }

    #[test]
    fn iwdg_reload_prevents_timeout() {
        let req = Arc::new(WdogResetReq::new());
        let mut w = Iwdg::new(req.clone());
        w.write(IW_OFF_KR, 4, KR_UNLOCK).unwrap();
        w.write(IW_OFF_PR, 4, 0).unwrap();
        w.write(IW_OFF_KR, 4, KR_UNLOCK).unwrap();
        w.write(IW_OFF_RLR, 4, 0x10).unwrap();
        w.write(IW_OFF_KR, 4, KR_START).unwrap();

        // 走到 8，喂狗重装载回 0x10；重复多次不超时
        for _ in 0..10 {
            w.tick(0x20); // 0x20 周期 / 4 = 8 步，仍 > 0
            assert!(!req.is_pending());
            w.write(IW_OFF_KR, 4, KR_RELOAD).unwrap();
            assert_eq!(w.down_count(), 0x10, "喂狗应重装载回 RLR");
        }
        assert!(!req.is_pending());
    }

    #[test]
    fn iwdg_pr_rlr_write_protected_until_unlock() {
        let req = Arc::new(WdogResetReq::new());
        let mut w = Iwdg::new(req.clone());
        // 未解锁直接写 PR/RLR → 忽略
        w.write(IW_OFF_PR, 4, 3).unwrap();
        w.write(IW_OFF_RLR, 4, 0x200).unwrap();
        assert_eq!(w.read(IW_OFF_PR, 4).unwrap(), 0);
        assert_eq!(w.read(IW_OFF_RLR, 4).unwrap(), 0);

        // 解锁后可写，写入后重新上锁（PR/RLR 各自独立上锁）
        w.write(IW_OFF_KR, 4, KR_UNLOCK).unwrap();
        w.write(IW_OFF_PR, 4, 2).unwrap();
        w.write(IW_OFF_KR, 4, KR_UNLOCK).unwrap();
        w.write(IW_OFF_RLR, 4, 0x200).unwrap();
        assert_eq!(w.read(IW_OFF_PR, 4).unwrap(), 2);
        assert_eq!(w.read(IW_OFF_RLR, 4).unwrap(), 0x200);
        w.write(IW_OFF_PR, 4, 5).unwrap(); // 上锁后写无效
        assert_eq!(w.read(IW_OFF_PR, 4).unwrap(), 2);
        // PVU/RVU 已置位，tick 后清除（更新完成）
        assert_ne!(w.read(IW_OFF_SR, 4).unwrap() & (SR_PVU | SR_RVU), 0);
        w.write(IW_OFF_KR, 4, KR_START).unwrap();
        w.tick(1);
        assert_eq!(w.read(IW_OFF_SR, 4).unwrap() & (SR_PVU | SR_RVU), 0);
    }

    #[test]
    fn iwdg_reset_disables() {
        let req = Arc::new(WdogResetReq::new());
        let mut w = Iwdg::new(req.clone());
        w.write(IW_OFF_KR, 4, KR_UNLOCK).unwrap();
        w.write(IW_OFF_PR, 4, 0).unwrap();
        w.write(IW_OFF_KR, 4, KR_UNLOCK).unwrap();
        w.write(IW_OFF_RLR, 4, 0x10).unwrap();
        w.write(IW_OFF_KR, 4, KR_START).unwrap();
        w.reset();
        assert_eq!(w.down_count(), 0);
        assert!(!req.is_pending());
        w.tick(10_000);
        assert!(!req.is_pending(), "复位后应停止计时");
    }

    // ---- WWDG ----

    #[test]
    fn wwdg_timeout_when_not_refreshed() {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let req = Arc::new(WdogResetReq::new());
        let mut w = Wwdg::new(nvic, req.clone());

        // 激活：WDGA | T=0x7F（WDGTB=0 → 分频 4096）
        w.write(WW_OFF_CR, 4, CR_WDGA | 0x7F).unwrap();
        assert_eq!(w.counter(), 0x7F);

        // 推进 (0x7F-0x40)=0x3F 步：每次 4096 周期
        w.tick(4096 * 0x3F);
        assert_eq!(w.counter(), 0x40);
        assert!(!req.is_pending(), "T6 未清零不应复位");

        // 再一步 → 0x3F：T6 清零 → 复位
        w.tick(4096);
        assert_eq!(w.counter(), 0x3F);
        assert!(req.is_pending());
        assert_eq!(req.take(), Some(ResetReason::Wwdg));
    }

    #[test]
    fn wwdg_refresh_in_window_no_reset() {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let req = Arc::new(WdogResetReq::new());
        let mut w = Wwdg::new(nvic, req.clone());

        // 窗口 W=0x40，激活 T=0x7F
        w.write(WW_OFF_CFR, 4, 0x40).unwrap();
        w.write(WW_OFF_CR, 4, CR_WDGA | 0x7F).unwrap();

        // 递减 0x7F→0x41（未到窗口）→ 刷新被窗口检查拒绝 → 复位
        w.tick(4096 * (0x7F - 0x41));
        assert_eq!(w.counter(), 0x41);
        w.write(WW_OFF_CR, 4, CR_WDGA | 0x7F).unwrap();
        assert!(req.is_pending(), "窗口上方刷新应触发窗口违规复位");
        req.take(); // 消费本次复位请求，隔离重建场景

        // 重建：递减到 0x40（== W）→ 刷新合法
        let mut w2 = Wwdg::new(Arc::new(Mutex::new(Nvic::new())), req.clone());
        w2.write(WW_OFF_CFR, 4, 0x40).unwrap();
        w2.write(WW_OFF_CR, 4, CR_WDGA | 0x7F).unwrap();
        w2.tick(4096 * (0x7F - 0x40));
        assert_eq!(w2.counter(), 0x40);
        assert!(!req.is_pending()); // req 已被 take
        w2.write(WW_OFF_CR, 4, CR_WDGA | 0x7F).unwrap();
        assert!(!req.is_pending(), "窗口内刷新应放行并重载计数");
        assert_eq!(w2.counter(), 0x7F);
    }

    #[test]
    fn wwdg_early_wakeup_sets_pending_irq() {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let req = Arc::new(WdogResetReq::new());
        let mut w = Wwdg::new(nvic.clone(), req.clone());

        // EWI 使能 + 窗口 0x40 + 激活
        w.write(WW_OFF_CFR, 4, CFR_EWI | 0x40).unwrap();
        w.write(WW_OFF_CR, 4, CR_WDGA | 0x7F).unwrap();

        // 递减到 0x40：置 EWIF + 挂起 IRQ0
        w.tick(4096 * (0x7F - 0x40));
        assert_eq!(w.counter(), 0x40);
        assert_eq!(w.read(WW_OFF_SR, 4).unwrap() & 1, 1, "EWIF 应置位");
        assert!(nvic.lock().unwrap().is_pending(WWDG_IRQ), "EWI 使能应挂起 IRQ0");
        assert!(!req.is_pending(), "EWI 不应触发复位");

        // SR 写 0 清除 EWIF
        w.write(WW_OFF_SR, 4, 0).unwrap();
        assert_eq!(w.read(WW_OFF_SR, 4).unwrap() & 1, 0);
    }
}
