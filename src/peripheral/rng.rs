//! RNG 真随机数发生器（STM32F407，M9 虚拟外设生态）。
//!
//! - CR.RNGEN（bit2）写 1 使能随机数生成；CR.IE（bit3）使能错误中断；
//! - 使能后 SR.DRDY（bit0）置位（数据就绪）；读 DR 返回当前随机值并清 DRDY，
//!   随即生成下一个值并再次置位（连续生成，轮询式读取）；
//! - 错误状态：SR.CECS（bit1，时钟错误）/ SR.SECS（bit2，种子错误）经
//!   [`Rng::inject_clock_error`] / [`Rng::inject_seed_error`] 注入（模拟外部时钟/
//!   种子异常）；错误位置位且 CR.IE 使能时挂起 RNG IRQ（STM32F407 IRQ80）；
//!   注意：F407 RNG 仅错误中断，DRDY 不触发中断（固件轮询）；
//! - 随机源：确定性 xorshift32（seed 可经 [`Rng::set_seed`] 注入，测试可复现）。
//!
//! 地址映射（RNG @ 0x50060800）：CR 0x00 / SR 0x04 / DR 0x08

use std::sync::{Arc, Mutex};

use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};

/// RNG NVIC IRQ（STM32F407：RNG 全局中断 = 80，仅错误中断）
pub const RNG_IRQ: u32 = 80;

/// 寄存器偏移
const OFF_CR: u32 = 0x00;
const OFF_SR: u32 = 0x04;
const OFF_DR: u32 = 0x08;

/// CR 位
const CR_RNGEN: u32 = 1 << 2; // 随机数发生器使能
const CR_IE: u32 = 1 << 3; // 中断使能（错误中断）

/// SR 位
const SR_DRDY: u32 = 1 << 0; // 数据就绪
const SR_CECS: u32 = 1 << 1; // 时钟错误（状态）
const SR_SECS: u32 = 1 << 2; // 种子错误（状态）

/// RNG 真随机数发生器
pub struct Rng {
    /// NVIC（错误中断挂起）
    nvic: Arc<Mutex<Nvic>>,
    /// 错误中断 IRQ（RNG_IRQ）
    irq: u32,
    /// CR 镜像（RNGEN/IE）
    cr: u32,
    /// SR 镜像（DRDY/CECS/SECS）
    sr: u32,
    /// 当前随机值（读 DR 返回）
    dr: u32,
    /// PRNG 状态（xorshift32）
    state: u32,
    /// 初始 seed（外设复位时恢复）
    seed: u32,
}

impl Rng {
    pub fn new(seed: u32, nvic: Arc<Mutex<Nvic>>, irq: u32) -> Self {
        let seed = normalize(seed);
        Self {
            nvic,
            irq,
            cr: 0,
            sr: 0,
            dr: 0,
            state: seed,
            seed,
        }
    }

    /// 注入时钟错误（模拟外部时钟异常；on=true 置 CECS，false 清除）。
    pub fn inject_clock_error(&mut self, on: bool) {
        if on {
            self.sr |= SR_CECS;
        } else {
            self.sr &= !SR_CECS;
        }
        self.check_error_irq();
    }

    /// 注入种子错误（模拟随机源异常；on=true 置 SECS，false 清除）。
    pub fn inject_seed_error(&mut self, on: bool) {
        if on {
            self.sr |= SR_SECS;
        } else {
            self.sr &= !SR_SECS;
        }
        self.check_error_irq();
    }

    /// 设置随机源种子（测试可复现；重设后若已使能则立即生成新值）。
    pub fn set_seed(&mut self, seed: u32) {
        self.seed = normalize(seed);
        self.state = self.seed;
        if self.cr & CR_RNGEN != 0 {
            self.dr = next(&mut self.state);
            self.sr |= SR_DRDY;
        }
    }

    /// 使能沿（RNGEN 0→1）：无既有错误则生成首值并置 DRDY。
    fn on_enable(&mut self) {
        if self.sr & (SR_CECS | SR_SECS) == 0 {
            self.dr = next(&mut self.state);
            self.sr |= SR_DRDY;
        }
    }

    /// 错误位置位且 IE 使能 → 挂起 RNG 中断。
    fn check_error_irq(&self) {
        if self.cr & CR_IE != 0 && self.sr & (SR_CECS | SR_SECS) != 0 {
            self.nvic.lock().unwrap().set_pending(self.irq);
        }
    }
}

/// xorshift32：状态非零归一化
fn normalize(seed: u32) -> u32 {
    if seed == 0 {
        0x9E37_79B9
    } else {
        seed
    }
}

/// xorshift32 下一个随机值
fn next(state: &mut u32) -> u32 {
    let mut x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    x
}

impl Peripheral for Rng {
    fn name(&self) -> &str {
        "RNG"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_CR => Ok(self.cr),
            OFF_SR => Ok(self.sr),
            OFF_DR => {
                let v = self.dr;
                // 读 DR 清 DRDY；若已使能且无错误则立即生成下一个值并重新置位
                self.sr &= !SR_DRDY;
                if self.cr & CR_RNGEN != 0 && self.sr & (SR_CECS | SR_SECS) == 0 {
                    self.dr = next(&mut self.state);
                    self.sr |= SR_DRDY;
                }
                Ok(v)
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_CR => {
                let old = self.cr & CR_RNGEN;
                self.cr = value & (CR_RNGEN | CR_IE);
                let new = self.cr & CR_RNGEN;
                if old == 0 && new != 0 {
                    self.on_enable();
                } else if old != 0 && new == 0 {
                    self.sr &= !SR_DRDY; // 禁用：数据就绪清除
                }
                Ok(())
            }
            OFF_SR => Ok(()), // SR 只读：写忽略
            OFF_DR => Ok(()), // DR 只读：写忽略
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.cr = 0;
        self.sr = 0;
        self.dr = 0;
        self.state = self.seed; // PRNG 恢复初始 seed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::nvic::Nvic;

    fn make() -> (Rng, Arc<Mutex<Nvic>>) {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        (Rng::new(0x1234_5678, nvic.clone(), RNG_IRQ), nvic)
    }

    #[test]
    fn enable_sets_drdy() {
        let (mut r, _) = make();
        r.write(OFF_CR, 4, CR_RNGEN).unwrap();
        assert_ne!(r.read(OFF_SR, 4).unwrap() & SR_DRDY, 0, "使能后应置 DRDY");
    }

    #[test]
    fn read_dr_returns_value_and_regenerates() {
        let (mut r, _) = make();
        r.write(OFF_CR, 4, CR_RNGEN).unwrap();
        let v1 = r.read(OFF_DR, 4).unwrap();
        let v2 = r.read(OFF_DR, 4).unwrap();
        assert_ne!(v1, v2, "连续两次读 DR 值应不同（连续生成）");
        assert_ne!(v2, 0);
    }

    #[test]
    fn disable_clears_drdy() {
        let (mut r, _) = make();
        r.write(OFF_CR, 4, CR_RNGEN).unwrap();
        r.write(OFF_CR, 4, 0).unwrap(); // RNGEN=0
        assert_eq!(r.read(OFF_SR, 4).unwrap() & SR_DRDY, 0, "禁用后应清 DRDY");
    }

    #[test]
    fn inject_clock_error_sets_cecs_and_irq() {
        let (mut r, nvic) = make();
        r.write(OFF_CR, 4, CR_RNGEN | CR_IE).unwrap();
        r.inject_clock_error(true);
        let sr = r.read(OFF_SR, 4).unwrap();
        assert_ne!(sr & SR_CECS, 0, "注入时钟错误应置 CECS");
        assert!(nvic.lock().unwrap().is_pending(RNG_IRQ), "IE+错误应挂起 RNG 中断");
        r.inject_clock_error(false);
        assert_eq!(r.read(OFF_SR, 4).unwrap() & SR_CECS, 0, "清除注入应清 CECS");
    }

    #[test]
    fn inject_seed_error_sets_secs_and_irq() {
        let (mut r, nvic) = make();
        r.write(OFF_CR, 4, CR_RNGEN | CR_IE).unwrap();
        r.inject_seed_error(true);
        let sr = r.read(OFF_SR, 4).unwrap();
        assert_ne!(sr & SR_SECS, 0, "注入种子错误应置 SECS");
        assert!(nvic.lock().unwrap().is_pending(RNG_IRQ), "IE+错误应挂起 RNG 中断");
    }

    #[test]
    fn error_blocks_drdy_on_enable() {
        let (mut r, _) = make();
        r.inject_seed_error(true); // RNGEN=0 期间注入错误
        r.write(OFF_CR, 4, CR_RNGEN).unwrap(); // 使能沿：有错误 → 不产生 DRDY
        assert_eq!(r.read(OFF_SR, 4).unwrap() & SR_DRDY, 0, "存在错误时使能不应置 DRDY");
    }

    #[test]
    fn same_seed_same_sequence() {
        let (mut a, _) = make();
        let (mut b, _) = make();
        a.write(OFF_CR, 4, CR_RNGEN).unwrap();
        b.write(OFF_CR, 4, CR_RNGEN).unwrap();
        for _ in 0..8 {
            assert_eq!(
                a.read(OFF_DR, 4).unwrap(),
                b.read(OFF_DR, 4).unwrap(),
                "同 seed 应产生相同随机序列（测试可复现）"
            );
        }
    }

    #[test]
    fn diff_seed_diff_sequence() {
        let mut a = Rng::new(0x1111_1111, Arc::new(Mutex::new(Nvic::new())), RNG_IRQ);
        let mut b = Rng::new(0x2222_2222, Arc::new(Mutex::new(Nvic::new())), RNG_IRQ);
        a.write(OFF_CR, 4, CR_RNGEN).unwrap();
        b.write(OFF_CR, 4, CR_RNGEN).unwrap();
        let va = a.read(OFF_DR, 4).unwrap();
        let vb = b.read(OFF_DR, 4).unwrap();
        assert_ne!(va, vb, "不同 seed 应产生不同序列");
    }
}
