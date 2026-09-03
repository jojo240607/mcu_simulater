//! RCC 复位与时钟控制（M4：完整时钟树）。
//!
//! M3 仅实现"使能寄存器镜像"；M4 补齐 F407 时钟树：
//! - 完整寄存器文件（CR/PLLCFGR/CFGR/RSTR/ENR/LPENR/BDCR/CSR/SSCGR/PLLI2SCFGR/DCKCFGR）；
//! - 状态位联动：写 CR 的 HSEON/PLLON 立即置位 HSERDY/PLLRDY（简化立即就绪），
//!   写 CFGR 时按请求源与就绪状态推导 SWS（请求源未就绪回退 HSI）；
//! - 时钟树计算：由 SWS + HPRE + PPRE1/PPRE2 推导 SYSCLK/HCLK/PCLK1/PCLK2
//!   及 APB 定时器时钟（PPRE!=/1 时 ×2），供外设（USART 波特率、TIM 时基）与测试查询。
//!
//! 保真说明：RCC 寄存器均为"直接存值"语义（除 CR 的 RDY 位、CFGR 的 SWS 位由联动维护）。
//! 固件经 MMIO read hook 读到的是真实寄存器（含联动位）；测试用 CPU mem_read 读到的是
//! guest 镜像（固件写入的原始值），故对 RDY/SWS 这类状态位应在固件内观察后写入结果区。

use crate::peripheral::wdog::ResetReason;
use crate::peripheral::{BusError, Peripheral};

/// HSI 内部 RC 振荡器频率
pub const HSI_FREQ_HZ: u32 = 16_000_000;
/// HSE 外部晶振频率（F407 板卡常见 8MHz）
pub const HSE_FREQ_HZ: u32 = 8_000_000;

// ---- RCC 寄存器偏移（STM32F407 @ 0x40023800）----
pub const OFF_CR: u32 = 0x00;
pub const OFF_PLLCFGR: u32 = 0x04;
pub const OFF_CFGR: u32 = 0x08;
pub const OFF_CIR: u32 = 0x0C;
pub const OFF_AHB1RSTR: u32 = 0x10;
pub const OFF_AHB2RSTR: u32 = 0x14;
pub const OFF_AHB3RSTR: u32 = 0x18;
pub const OFF_APB1RSTR: u32 = 0x20;
pub const OFF_APB2RSTR: u32 = 0x24;
pub const OFF_AHB1ENR: u32 = 0x30;
pub const OFF_AHB2ENR: u32 = 0x34;
pub const OFF_AHB3ENR: u32 = 0x38;
pub const OFF_APB1ENR: u32 = 0x40;
pub const OFF_APB2ENR: u32 = 0x44;
pub const OFF_AHB1LPENR: u32 = 0x50;
pub const OFF_AHB2LPENR: u32 = 0x54;
pub const OFF_AHB3LPENR: u32 = 0x58;
pub const OFF_APB1LPENR: u32 = 0x60;
pub const OFF_APB2LPENR: u32 = 0x64;
pub const OFF_BDCR: u32 = 0x70;
pub const OFF_CSR: u32 = 0x74;
pub const OFF_SSCGR: u32 = 0x80;
pub const OFF_PLLI2SCFGR: u32 = 0x84;
pub const OFF_DCKCFGR: u32 = 0x8C;

/// 寄存器文件容量：覆盖 0x00..0x90（DCKCFGR 为最后一个，索引 35）
const REG_COUNT: usize = 36;

/// CR 状态位
const CR_HSEON: u32 = 1 << 16;
const CR_HSERDY: u32 = 1 << 17;
const CR_PLLON: u32 = 1 << 24;
const CR_PLLRDY: u32 = 1 << 25;

/// CSR 复位标志位（F407 硬件位）
/// - bit31 LPWRRSTF：低功耗唤醒复位标志
/// - bit28 IWDGRSTF：独立看门狗复位标志
/// - bit27 WWDGRSTF：窗口看门狗复位标志
/// - bit24 RMVF：清除复位标志（写 1 清除全部复位标志）
const CSR_LPWRRSTF: u32 = 1 << 31;
const CSR_IWDGRSTF: u32 = 1 << 28;
const CSR_WWDGRSTF: u32 = 1 << 27;
const CSR_RMVF: u32 = 1 << 24;
/// 全部复位标志（RMVF 写 1 时清除这些位）
const CSR_RESET_FLAGS: u32 = CSR_LPWRRSTF | CSR_IWDGRSTF | CSR_WWDGRSTF;

/// 时钟树推导结果（Hz）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockTree {
    /// 系统时钟 SYSCLK
    pub sysclk: u32,
    /// AHB 总线时钟 HCLK
    pub hclk: u32,
    /// APB1 外设时钟 PCLK1
    pub pclk1: u32,
    /// APB2 外设时钟 PCLK2
    pub pclk2: u32,
    /// APB1 定时器时钟（PPRE1 != /1 时 = PCLK1 × 2）
    pub timer_clk1: u32,
    /// APB2 定时器时钟（PPRE2 != /1 时 = PCLK2 × 2）
    pub timer_clk2: u32,
}

/// RCC 外设（寄存器文件 + 时钟树推导）
pub struct Rcc {
    regs: [u32; REG_COUNT],
}

impl Rcc {
    pub fn new() -> Self {
        let mut r = Self {
            regs: [0; REG_COUNT],
        };
        r.reset();
        r
    }

    /// 记录一次复位原因（置对应 CSR 复位标志位，供固件/测试查询）。
    pub fn record_reset(&mut self, reason: ResetReason) {
        let csr = self.regs[(OFF_CSR / 4) as usize];
        let flag = match reason {
            ResetReason::Iwdg => CSR_IWDGRSTF,
            ResetReason::Wwdg => CSR_WWDGRSTF,
            ResetReason::LowPower => CSR_LPWRRSTF,
        };
        self.regs[(OFF_CSR / 4) as usize] = csr | flag;
    }

    /// 读取 CSR（供测试/固件查询复位标志）。
    pub fn csr(&self) -> u32 {
        self.regs[(OFF_CSR / 4) as usize]
    }

    /// 推导当前时钟树。`sysclk` 按 SWS（实际生效源）计算，而非 SW（请求源）。
    pub fn clocks(&self) -> ClockTree {
        let cfgr = self.regs[(OFF_CFGR / 4) as usize];
        let sws = (cfgr >> 2) & 0x3;

        let sysclk = match sws {
            1 => HSE_FREQ_HZ,
            2 => self.pll_clk(),
            _ => HSI_FREQ_HZ,
        };

        let hclk = sysclk / ahb_div(cfgr);
        let pclk1 = hclk / apb_div((cfgr >> 10) & 0x7);
        let pclk2 = hclk / apb_div((cfgr >> 13) & 0x7);
        let timer_clk1 = if pclk1 == hclk { pclk1 } else { pclk1 * 2 };
        let timer_clk2 = if pclk2 == hclk { pclk2 } else { pclk2 * 2 };

        ClockTree {
            sysclk,
            hclk,
            pclk1,
            pclk2,
            timer_clk1,
            timer_clk2,
        }
    }

    /// PLL 输出频率：VCO_IN = 源(HSI/HSE)/PLLM，VCO_OUT = VCO_IN×PLLN，PLLCLK = VCO_OUT/PLLP
    fn pll_clk(&self) -> u32 {
        let p = self.regs[(OFF_PLLCFGR / 4) as usize];
        let m = p & 0x3F;
        let n = (p >> 6) & 0x1FF;
        let pllp = match (p >> 16) & 0x3 {
            0 => 2,
            1 => 4,
            2 => 6,
            _ => 8,
        };
        let src = if p & (1 << 22) != 0 { HSE_FREQ_HZ } else { HSI_FREQ_HZ };
        if m == 0 || n == 0 {
            return 0;
        }
        src / m * n / pllp
    }

    /// 按请求源与就绪状态推导 SWS（bit2..3）写入 CFGR；请求源未就绪回退 HSI(0)。
    fn recompute_sws(&mut self) {
        let cr = self.regs[(OFF_CR / 4) as usize];
        let cfgr = self.regs[(OFF_CFGR / 4) as usize];
        let sw = cfgr & 0x3;
        let sws = match sw {
            1 if cr & CR_HSERDY != 0 => 1, // 请求 HSE 且就绪
            2 if cr & CR_PLLRDY != 0 => 2, // 请求 PLL 且就绪
            _ => 0,                        // 否则 HSI
        };
        self.regs[(OFF_CFGR / 4) as usize] = (cfgr & !(0x3 << 2)) | (sws << 2);
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
        // CSR 只读（除 RMVF 写 1 清除复位标志）：不直接存值
        if offset == OFF_CSR {
            if value & CSR_RMVF != 0 {
                *slot &= !CSR_RESET_FLAGS;
            }
            return Ok(());
        }
        *slot = value;
        match offset {
            // CR：HSEON/PLLON 置位 → HSERDY/PLLRDY 立即置位（简化立即就绪），清位随之清除
            OFF_CR => {
                let mut cr = *slot;
                if cr & CR_HSEON != 0 {
                    cr |= CR_HSERDY;
                } else {
                    cr &= !CR_HSERDY;
                }
                if cr & CR_PLLON != 0 {
                    cr |= CR_PLLRDY;
                } else {
                    cr &= !CR_PLLRDY;
                }
                *slot = cr;
                self.recompute_sws(); // HSE/PLL 就绪状态变化可能影响 SWS
            }
            // CFGR：重算 SWS 状态位（只读位，读回由联动维护）
            OFF_CFGR => self.recompute_sws(),
            _ => {}
        }
        Ok(())
    }

    fn reset(&mut self) {
        // F407 复位值：CR = 0x00000083（HSION|HSIRDY|HSITRIM=16），其余 0
        self.regs = [0; REG_COUNT];
        self.regs[(OFF_CR / 4) as usize] = 0x0000_0083;
    }
}

/// AHB 预分频 HPRE[3:0] → 分频数
fn ahb_div(cfgr: u32) -> u32 {
    match (cfgr >> 4) & 0xF {
        0..=7 => 1,
        8 => 2,
        9 => 4,
        10 => 8,
        11 => 16,
        12 => 64,
        13 => 128,
        14 => 256,
        _ => 512,
    }
}

/// APB 预分频 PPRE[2:0] → 分频数
fn apb_div(ppre: u32) -> u32 {
    match ppre {
        0..=3 => 1,
        4 => 2,
        5 => 4,
        6 => 8,
        _ => 16,
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

    #[test]
    fn default_clocks_are_hsi_16m() {
        let r = Rcc::new();
        let c = r.clocks();
        // 复位：SW=HSI，HPRE=/1，PPRE1/PPRE2=/1 → 全部 16MHz
        assert_eq!(
            c,
            ClockTree {
                sysclk: 16_000_000,
                hclk: 16_000_000,
                pclk1: 16_000_000,
                pclk2: 16_000_000,
                timer_clk1: 16_000_000,
                timer_clk2: 16_000_000,
            }
        );
    }

    #[test]
    fn hse_pll_168mhz_clock_tree() {
        let mut r = Rcc::new();
        // 1) 使能 HSE + PLL（HSEON|PLLON），RDY 位立即置位
        r.write(OFF_CR, 4, CR_HSEON | CR_PLLON).unwrap();
        let cr = r.read(OFF_CR, 4).unwrap();
        assert_ne!(cr & CR_HSERDY, 0, "HSERDY 应立即置位");
        assert_ne!(cr & CR_PLLRDY, 0, "PLLRDY 应立即置位");

        // 2) PLLCFGR：PLLSRC=HSE, M=8, N=336, P=2, Q=7
        //    VCO_IN = 8M/8 = 1M，VCO_OUT = 336M，PLLCLK = 336/2 = 168M，PLL48 = 336/7 = 48M
        let pllcfgr = (7 << 24) | (1 << 22) | (336 << 6) | (8 << 0);
        r.write(OFF_PLLCFGR, 4, pllcfgr).unwrap();
        assert_eq!(r.pll_clk(), 168_000_000, "PLL 输出应为 168MHz");

        // 3) CFGR：SW=PLL, HPRE=/1, PPRE1=/4, PPRE2=/2
        let cfgr = (2 << 0) | (0 << 4) | (5 << 10) | (4 << 13);
        r.write(OFF_CFGR, 4, cfgr).unwrap();
        assert_eq!((r.read(OFF_CFGR, 4).unwrap() >> 2) & 0x3, 2, "SWS 应为 PLL");

        let c = r.clocks();
        assert_eq!(c.sysclk, 168_000_000);
        assert_eq!(c.hclk, 168_000_000);
        assert_eq!(c.pclk1, 42_000_000);
        assert_eq!(c.pclk2, 84_000_000);
        assert_eq!(c.timer_clk1, 84_000_000);
        assert_eq!(c.timer_clk2, 168_000_000);
    }

    #[test]
    fn sws_falls_back_to_hsi_when_source_not_ready() {
        let mut r = Rcc::new();
        // 请求 HSE 但未使能（HSEON=0）→ SWS 应为 HSI(0)
        r.write(OFF_CFGR, 4, 1 << 0).unwrap();
        assert_eq!((r.read(OFF_CFGR, 4).unwrap() >> 2) & 0x3, 0);
        assert_eq!(r.clocks().sysclk, HSI_FREQ_HZ);

        // 再使能 HSE → SWS 切到 HSE
        r.write(OFF_CR, 4, CR_HSEON).unwrap();
        assert_eq!((r.read(OFF_CFGR, 4).unwrap() >> 2) & 0x3, 1);
        assert_eq!(r.clocks().sysclk, HSE_FREQ_HZ);
    }
}
