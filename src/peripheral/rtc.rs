//! RTC 实时时钟 + 备份寄存器（STM32F407，M11 虚拟外设生态）。
//!
//! RTC @ 0x40002800（APB1）。F407 无独立 BKP 外设，备份寄存器内嵌于 RTC：
//! RTC_BKP0R..RTC_BKP19R @ +0x50..+0x9C（0x40002400 为保留地址，不映射）。
//!
//! 功能与语义（对齐 RM0090）：
//! - 日历计数：双预分频（PRER.PREDIV_A 7 位 / PREDIV_S 15 位）把 RTCCLK 分频为
//!   ck_spre（1 Hz）；TR/DR 以 BCD 编码，SSR 提供亚秒精度（自 PREDIV_S 递减到 0
//!   进位一秒）。时间推进由 [`Peripheral::tick`] 按虚拟时钟驱动（与 TIM 一致：
//!   传入周期数即视为 RTCCLK 周期，未建模 HCLK↔RTCCLK 频率比）；
//! - 初始化：写 ISR.INIT=1 → INITF=1（冻结日历）→ 写 TR/DR/PRER → 写 INIT=0 →
//!   恢复计数。简化：无时钟域同步延迟，INITF/RSF 立即就绪；INITS 在首次写日期后置位；
//! - 写保护：RTC_WPR 依次写 0xCA、0x53 解锁；写错值重新上锁；上锁期间写受保护
//!   寄存器被忽略。简化：解锁后保持，直至写错或复位；
//! - 闹钟 A/B：ALRMAR/ALRMBR（MSK1-4 屏蔽 + WDSEL 日期/星期选择），
//!   ALRAE/ALRBE 使能 + ALRAIE/ALRBIE 中断使能；匹配置 ISR.ALRAF/ALRBF 并挂起
//!   RTC_Alarm IRQ（F407 IRQ41）。亚秒闹钟（ALRMASSR/ALRMBSSR）仅当 MASKSS != 0
//!   时参与比较（简化：MASKSS=0 视为未约束，避免复位默认值误拦截时间闹钟）；
//! - 唤醒定时器：WUTR + CR.WUCKSEL（RTCCLK/2/4/8/16 或 ck_spre 及其 2^n 分频），
//!   计数到 0 置 ISR.WUTF 并挂起 RTC_WKUP IRQ（F407 IRQ3）；WUTE 上升沿清 WUTF
//!   并从 WUTR 装载计数（首次唤醒在 WUTR+1 个唤醒时钟后）；
//! - 备份寄存器：20 × 32 位，掉电保持（外设复位不丢失，仅备份域复位清除）；
//!   写访问需 PWR_CR.DBP=1（经 [`Pwr`] 共享句柄查询），读始终允许。
//!
//! 简化说明（路线 B）：无时钟域/影子寄存器同步延迟；RTC 时钟源
//! （RCC_BDCR.RTCSEL）与 LSE/LSI 就绪未建模；12 小时制（CR.FMT=1）未实现，
//! PM 位恒 0。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use crate::peripheral::nvic::Nvic;
use crate::peripheral::pwr::Pwr;
use crate::peripheral::{BusError, Peripheral};

/// RTC_Alarm 中断（F407 IRQ41）
pub const RTC_ALARM_IRQ: u32 = 41;
/// RTC_WKUP 中断（F407 IRQ3）
pub const RTC_WKUP_IRQ: u32 = 3;

/// 寄存器偏移
const OFF_TR: u32 = 0x00;
const OFF_DR: u32 = 0x04;
const OFF_CR: u32 = 0x08;
const OFF_ISR: u32 = 0x0C;
const OFF_PRER: u32 = 0x10;
const OFF_WUTR: u32 = 0x14;
const OFF_CALIBR: u32 = 0x18;
const OFF_ALRMAR: u32 = 0x1C;
const OFF_ALRMBR: u32 = 0x20;
const OFF_WPR: u32 = 0x24;
const OFF_SSR: u32 = 0x28;
const OFF_SHIFTR: u32 = 0x2C;
const OFF_TSTR: u32 = 0x30;
const OFF_TSDR: u32 = 0x34;
const OFF_TSSSR: u32 = 0x38;
const OFF_CALR: u32 = 0x3C;
const OFF_TAFCR: u32 = 0x40;
const OFF_ALRMASSR: u32 = 0x44;
const OFF_ALRMBSSR: u32 = 0x48;
/// 备份寄存器基址（BKP0R @ +0x50）
const OFF_BKP: u32 = 0x50;
/// 备份寄存器个数
const BKP_COUNT: usize = 20;

/// CR 位（F407）
const CR_WUCKSEL: u32 = 0x7 << 0; // 唤醒时钟选择
const CR_TSEDGE: u32 = 1 << 3;
const CR_REFCKON: u32 = 1 << 4;
const CR_BYPSHAD: u32 = 1 << 5; // 旁路影子寄存器（简化：恒生效）
const CR_FMT: u32 = 1 << 6; // 小时格式（0=24h，1=12h；12h 未实现）
const CR_ALRAE: u32 = 1 << 8; // 闹钟 A 使能
const CR_ALRBE: u32 = 1 << 9; // 闹钟 B 使能
const CR_WUTE: u32 = 1 << 10; // 唤醒定时器使能
const CR_TSE: u32 = 1 << 11;
const CR_ALRAIE: u32 = 1 << 12; // 闹钟 A 中断使能
const CR_ALRBIE: u32 = 1 << 13; // 闹钟 B 中断使能
const CR_WUTIE: u32 = 1 << 14; // 唤醒中断使能
const CR_TSIE: u32 = 1 << 15;
const CR_ADD1H: u32 = 1 << 16; // 软件加 1 小时（写 1 触发，自清零）
const CR_SUB1H: u32 = 1 << 17; // 软件减 1 小时
const CR_ADD1M: u32 = 1 << 18;
const CR_SUB1M: u32 = 1 << 19;
const CR_ADD1S: u32 = 1 << 20; // 软件加 1 秒
const CR_SUB1S: u32 = 1 << 21; // 软件减 1 秒
const CR_OSEL: u32 = 0x3 << 24;
/// CR 中存入镜像的可写位（ADD1*/SUB1* 为写 1 触发，不入镜像）
const CR_WRITABLE: u32 = CR_WUCKSEL | CR_TSEDGE | CR_REFCKON | CR_BYPSHAD | CR_FMT
    | CR_ALRAE | CR_ALRBE | CR_WUTE | CR_TSE | CR_ALRAIE | CR_ALRBIE | CR_WUTIE
    | CR_TSIE | CR_OSEL;
/// CR 软件加减位（触发后自清零）
const CR_SW_ADJUST: u32 = CR_ADD1H | CR_SUB1H | CR_ADD1M | CR_SUB1M | CR_ADD1S | CR_SUB1S;

/// ISR 位（F407）
const ISR_ALRAWF: u32 = 1 << 0; // 闹钟 A 可写标志（= !ALRAE，只读）
const ISR_ALRBWF: u32 = 1 << 1; // 闹钟 B 可写标志（= !ALRBE，只读）
const ISR_WUTWF: u32 = 1 << 2; // 唤醒定时器可写标志（= !WUTE，只读）
const ISR_INITS: u32 = 1 << 4; // 日历已初始化标志（只读）
const ISR_RSF: u32 = 1 << 5; // 寄存器同步标志（简化：= !init，只读）
const ISR_INITF: u32 = 1 << 6; // 初始化模式标志（= init，只读）
const ISR_INIT: u32 = 1 << 7; // 初始化模式请求（可写）
const ISR_ALRAF: u32 = 1 << 8; // 闹钟 A 标志（写 0 清除）
const ISR_ALRBF: u32 = 1 << 9; // 闹钟 B 标志（写 0 清除）
const ISR_WUTF: u32 = 1 << 10; // 唤醒标志（写 0 清除）
/// 可由软件写 0 清除的标志位
const ISR_CLEARABLE: u32 = ISR_ALRAF | ISR_ALRBF | ISR_WUTF;

/// PRER 位（RM0090：PREDIV_A@bits 22:16，PREDIV_S@bits 14:0；复位值 0x007F00FF → A=127/S=255）
const PRER_PREDIV_A: u32 = 0x7F << 16; // 异步预分频值（7 位）
const PRER_PREDIV_S: u32 = 0x7FFF; // 同步预分频值（15 位）

/// ALRMAR/ALRMBR 位
const ALR_MSK1: u32 = 1 << 7; // 秒屏蔽
const ALR_MSK2: u32 = 1 << 15; // 分屏蔽
const ALR_MSK3: u32 = 1 << 23; // 时屏蔽
const ALR_MSK4: u32 = 1 << 31; // 日期/星期屏蔽
const ALR_WDSEL: u32 = 1 << 30; // 0=日期，1=星期

/// RTC 实时时钟 + 备份寄存器
pub struct Rtc {
    /// NVIC（闹钟/唤醒中断挂起）
    nvic: Arc<Mutex<Nvic>>,
    /// PWR（查询 CR.DBP 判定备份域写保护）
    pwr: Arc<Mutex<Pwr>>,
    /// 日历秒数（自 2000-01-01 00:00:00，内部规范表示；TR/DR 为其 BCD 投影）
    time_s: u64,
    /// RTCCLK 周期累计（对整秒周期的余数）
    accum: u64,
    /// 亚秒计数（自 PREDIV_S 递减，0 后进位一秒）
    ssr: u32,
    /// CR 镜像（可写位）
    cr: u32,
    /// ISR 中可写标志位（ALRAF/ALRBF/WUTF）
    isr_flags: u32,
    /// 初始化模式（= ISR.INIT/INITF）
    init: bool,
    /// 日历是否已初始化（= ISR.INITS）
    initialized: bool,
    /// PRER 镜像
    prer: u32,
    /// WUTR 镜像（唤醒装载值）
    wutr: u32,
    /// 唤醒计数（自 WUTR 递减）
    wut_cnt: u32,
    /// 唤醒时钟周期累计
    wut_accum: u64,
    /// ALRMAR/ALRMBR 镜像
    alrmar: u32,
    alrmbr: u32,
    /// ALRMASSR/ALRMBSSR 镜像
    alrmassr: u32,
    alrmbssr: u32,
    /// 写保护状态：0=未解锁/上锁，1=已写 0xCA（待 0x53）
    wpr_step: u32,
    /// 写保护是否已解锁
    unlocked: bool,
    /// 备份寄存器（掉电保持，外设复位不丢失）
    bkp: [u32; BKP_COUNT],
    /// 活动标记（闹钟/唤醒使能或日历已初始化）：Machine block hook 据此跳过
    /// 无任何计时需求的 RTC 加锁 tick（纯计算负载下 RTC 无事可做）
    active: Arc<AtomicBool>,
}

impl Rtc {
    pub fn new(nvic: Arc<Mutex<Nvic>>, pwr: Arc<Mutex<Pwr>>) -> Self {
        Self::with_active(nvic, pwr, Arc::new(AtomicBool::new(false)))
    }

    /// 正式构造：`active` 由 Machine 持有（与 timers 列表并行），使能/初始化变化时同步
    pub fn with_active(
        nvic: Arc<Mutex<Nvic>>,
        pwr: Arc<Mutex<Pwr>>,
        active: Arc<AtomicBool>,
    ) -> Self {
        let mut r = Self {
            nvic,
            pwr,
            time_s: 0,
            accum: 0,
            ssr: 0,
            cr: 0,
            isr_flags: 0,
            init: false,
            initialized: false,
            prer: 0x007F_00FF,
            wutr: 0xFFFF,
            wut_cnt: 0xFFFF,
            wut_accum: 0,
            alrmar: 0,
            alrmbr: 0,
            alrmassr: 0,
            alrmbssr: 0,
            wpr_step: 0,
            unlocked: false,
            bkp: [0; BKP_COUNT],
            active,
        };
        // 复位值：DR=0x00002101 → 2021-01-01 00:00:00，SSR=PREDIV_S=0xFF
        r.time_s = r.time_from_ymd_hms(2021, 1, 1, 0, 0, 0);
        r.ssr = r.prer & PRER_PREDIV_S; // 同步预分频值（初始 0xFF）
        r
    }

    /// 重算活动标记：任一闹钟/唤醒使能，或日历已被固件初始化（作为时间源使用）。
    fn sync_active(&self) {
        let any = self.cr & (CR_ALRAE | CR_ALRBE | CR_WUTE) != 0 || self.initialized;
        self.active.store(any, Ordering::Relaxed);
    }

    /// 日历推进（虚拟时钟周期数即 RTCCLK 周期数）。
    fn advance(&mut self, cycles: u64) {
        if cycles == 0 {
            return;
        }
        let a = ((self.prer & PRER_PREDIV_A) >> 16) as u64 + 1;
        let s = (self.prer & PRER_PREDIV_S) as u64 + 1;
        let period = a * s;
        if period == 0 {
            return;
        }
        self.accum += cycles;
        let seconds = self.accum / period;
        self.accum %= period;
        // 整秒推进（逐个秒推进，便于闹钟在匹配秒时精确触发一次）
        for _ in 0..seconds {
            self.time_s += 1;
            self.check_alarms();
        }
        // 亚秒 = 同步预分频值 - 当前秒内已流逝的 ck_apre 数
        self.ssr = (s as u32 - 1) - (self.accum / a) as u32;
    }

    /// 唤醒定时器推进。
    fn advance_wakeup(&mut self, cycles: u64) {
        if self.cr & CR_WUTE == 0 || cycles == 0 {
            return;
        }
        let a = ((self.prer & PRER_PREDIV_A) >> 16) as u64 + 1;
        let s = (self.prer & PRER_PREDIV_S) as u64 + 1;
        let ck_spre = a * s; // RTCCLK 周期数 / 每 ck_spre 周期
        let wake_period = match self.cr & CR_WUCKSEL {
            0 => 2,
            1 => 4,
            2 => 8,
            3 => 16,
            4 => ck_spre,
            5 => ck_spre.saturating_mul(65536),
            6 => ck_spre.saturating_mul(256),
            _ => ck_spre.saturating_mul(16),
        };
        if wake_period == 0 {
            return;
        }
        self.wut_accum += cycles;
        let n = self.wut_accum / wake_period;
        self.wut_accum %= wake_period;
        for _ in 0..n {
            if self.wut_cnt == 0 {
                // 计数到 0：置 WUTF + 挂起 IRQ + 自动重装
                self.wut_cnt = self.wutr & 0xFFFF;
                self.isr_flags |= ISR_WUTF;
                if self.cr & CR_WUTIE != 0 {
                    self.nvic.lock().unwrap().set_pending(RTC_WKUP_IRQ);
                }
            } else {
                self.wut_cnt -= 1;
            }
        }
    }

    /// 检查闹钟 A/B 是否匹配（日历秒边界、ALRAE/ALRBE 上升沿、退出初始化时调用）。
    fn check_alarms(&mut self) {
        let (sec, min, hour) = self.hms();
        let (_, _, day, weekday) = self.ymd();
        if self.cr & CR_ALRAE != 0
            && self.alarm_matches(self.alrmar, sec, min, hour, day as u8, weekday)
            && self.ss_matches(self.alrmassr)
        {
            if self.isr_flags & ISR_ALRAF == 0 {
                self.isr_flags |= ISR_ALRAF;
                if self.cr & CR_ALRAIE != 0 {
                    self.nvic.lock().unwrap().set_pending(RTC_ALARM_IRQ);
                }
            }
        }
        if self.cr & CR_ALRBE != 0
            && self.alarm_matches(self.alrmbr, sec, min, hour, day as u8, weekday)
            && self.ss_matches(self.alrmbssr)
        {
            if self.isr_flags & ISR_ALRBF == 0 {
                self.isr_flags |= ISR_ALRBF;
                if self.cr & CR_ALRBIE != 0 {
                    self.nvic.lock().unwrap().set_pending(RTC_ALARM_IRQ);
                }
            }
        }
    }

    /// 闹钟匹配（掩码语义：MSK=1 忽略对应字段）。
    fn alarm_matches(&self, reg: u32, sec: u8, min: u8, hour: u8, day: u8, weekday: u8) -> bool {
        if reg & ALR_MSK1 == 0 && bcd_to_u8(reg & 0x7F) != sec {
            return false;
        }
        if reg & ALR_MSK2 == 0 && bcd_to_u8((reg >> 8) & 0x7F) != min {
            return false;
        }
        if reg & ALR_MSK3 == 0 && bcd_to_u8((reg >> 16) & 0x3F) != hour {
            return false;
        }
        if reg & ALR_MSK4 == 0 {
            if reg & ALR_WDSEL != 0 {
                if ((reg >> 13) & 0x7) as u8 != weekday {
                    return false;
                }
            } else if bcd_to_u8(reg & 0x3F) != day {
                return false;
            }
        }
        true
    }

    /// 亚秒闹钟匹配（简化：MASKSS=0 视为未约束）。
    fn ss_matches(&self, reg: u32) -> bool {
        let mask = (reg >> 28) & 0xF;
        if mask == 0 {
            return true;
        }
        (self.ssr >> mask) == ((reg & 0x7FFF) >> mask)
    }

    // ---------- 日历换算（时间 <-> BCD 寄存器） ----------

    /// 由 (年,月,日,时,分,秒) 计算自 2000-01-01 的秒数。年范围为 2000..=2099。
    fn time_from_ymd_hms(&self, year: u32, month: u32, day: u32, hour: u32, min: u32, sec: u32) -> u64 {
        let days = days_from_civil(year as i64, month as u64, day as u64) - DAYS_2000_EPOCH;
        (days * 86400 + hour as i64 * 3600 + min as i64 * 60 + sec as i64) as u64
    }

    /// 当前 时/分/秒（BCD 字段值）
    fn hms(&self) -> (u8, u8, u8) {
        let rem = self.time_s % 86400;
        let hour = (rem / 3600) as u8;
        let min = ((rem / 60) % 60) as u8;
        let sec = (rem % 60) as u8;
        (sec, min, hour)
    }

    /// 当前 年/月/日/星期（2000..=2099；星期 1=周一 .. 7=周日）
    fn ymd(&self) -> (u32, u32, u32, u8) {
        let days_since_2000 = (self.time_s / 86400) as i64;
        let (y, m, d) = civil_from_days(days_since_2000 + DAYS_2000_EPOCH);
        let weekday = ((days_since_2000 + 5).rem_euclid(7) + 1) as u8; // 2000-01-01=周六=6
        (y as u32, m as u32, d as u32, weekday)
    }

    /// 读 TR：把当前时间投影为 BCD 时/分/秒。
    fn read_tr(&self) -> u32 {
        let (sec, min, hour) = self.hms();
        bcd_pack(sec) | (bcd_pack(min) << 8) | (bcd_pack(hour) << 16)
    }

    /// 读 DR：把当前日期投影为 BCD 年/月/日 + 星期。
    fn read_dr(&self) -> u32 {
        let (y, m, d, wd) = self.ymd();
        let yy = y - 2000;
        let mut v = bcd_pack(d as u8);
        v |= (m / 10) << 12 | (m % 10) << 8; // 月
        v |= (wd as u32 & 0x7) << 13; // 星期
        v |= (yy % 10) << 16 | (yy / 10) << 20; // 年
        v
    }

    /// 写 TR（初始化模式下）：保留当前日期，更新时分秒。
    fn write_tr(&mut self, value: u32) {
        let sec = bcd_to_u8(value & 0x7F);
        let min = bcd_to_u8((value >> 8) & 0x7F);
        let hour = bcd_to_u8((value >> 16) & 0x3F);
        let (y, m, d, _) = self.ymd();
        self.time_s = self.time_from_ymd_hms(y, m, d, hour as u32, min as u32, sec as u32);
    }

    /// 写 DR（初始化模式下）：保留当前时分秒，更新年月日（星期自动计算）。
    fn write_dr(&mut self, value: u32) {
        let day = bcd_to_u8(value & 0x3F) as u32;
        let month = ((((value >> 12) & 1) as u32) * 10) + bcd_to_u8((value >> 8) & 0xF) as u32;
        let year = 2000 + (((value >> 20) & 0xF) as u32) * 10 + bcd_to_u8((value >> 16) & 0xF) as u32;
        let (_, min, hour) = self.hms();
        let month = if month == 0 { 1 } else { month };
        self.time_s = self.time_from_ymd_hms(
            year,
            month.min(12),
            day.min(31),
            hour as u32,
            min as u32,
            0,
        );
    }

    /// 进入初始化模式（冻结日历）。
    fn enter_init(&mut self) {
        self.init = true;
    }

    /// 退出初始化模式：以当前 TR/DR 装载日历并恢复计数。
    fn exit_init(&mut self) {
        self.init = false;
        self.initialized = true;
        self.accum = 0;
        self.ssr = self.prer & PRER_PREDIV_S;
        self.sync_active();
    }

    /// CR 写（软件加减秒/分/时 + 使能位边缘检测）。
    fn write_cr(&mut self, value: u32) {
        // WUCKSEL 仅在 WUTE=0 时可写
        let mut v = value & CR_WRITABLE;
        if self.cr & CR_WUTE != 0 {
            v = (self.cr & CR_WUCKSEL) | (v & !CR_WUCKSEL);
        }
        let alra_old = self.cr & CR_ALRAE != 0;
        let alrb_old = self.cr & CR_ALRBE != 0;
        let wute_old = self.cr & CR_WUTE != 0;
        self.cr = v;
        let alra_new = self.cr & CR_ALRAE != 0;
        let alrb_new = self.cr & CR_ALRBE != 0;
        let wute_new = self.cr & CR_WUTE != 0;
        // 软件加减（写 1 触发，自清零，不入镜像）
        if value & CR_SW_ADJUST != 0 {
            if value & CR_ADD1H != 0 {
                self.time_s = self.time_s.saturating_add(3600);
            }
            if value & CR_SUB1H != 0 {
                self.time_s = self.time_s.saturating_sub(3600);
            }
            if value & CR_ADD1M != 0 {
                self.time_s = self.time_s.saturating_add(60);
            }
            if value & CR_SUB1M != 0 {
                self.time_s = self.time_s.saturating_sub(60);
            }
            if value & CR_ADD1S != 0 {
                self.time_s = self.time_s.saturating_add(1);
            }
            if value & CR_SUB1S != 0 {
                self.time_s = self.time_s.saturating_sub(1);
            }
        }
        // 闹钟使能上升沿：若当前时间已匹配则立即触发
        if !alra_old && alra_new {
            self.check_alarms();
        }
        if !alrb_old && alrb_new {
            self.check_alarms();
        }
        // 唤醒使能上升沿：清 WUTF，从 WUTR 装载计数
        if !wute_old && wute_new {
            self.isr_flags &= !ISR_WUTF;
            self.wut_cnt = self.wutr & 0xFFFF;
            self.wut_accum = 0;
        }
        self.sync_active();
    }

    /// ISR 写：标志位写 0 清除；INIT 位写 1/0 进出初始化（需解锁写保护）。
    fn write_isr(&mut self, value: u32) {
        // 标志位：写 0 清除（无需写保护）
        self.isr_flags &= value & ISR_CLEARABLE;
        // INIT 位：需写保护解锁
        if !self.unlocked {
            return;
        }
        let want_init = value & ISR_INIT != 0;
        if want_init && !self.init {
            self.enter_init();
        } else if !want_init && self.init {
            self.exit_init();
        }
    }

    /// 解锁 RTC 写保护（WPR 依次写 0xCA、0x53；写错重新上锁）。
    fn write_wpr(&mut self, value: u32) {
        let v = (value & 0xFF) as u8;
        match (self.wpr_step, v) {
            (0, 0xCA) => self.wpr_step = 1,
            (1, 0x53) => {
                self.wpr_step = 0;
                self.unlocked = true;
            }
            _ => {
                self.wpr_step = 0;
                self.unlocked = false;
            }
        }
    }

    /// 备份寄存器下标（offset 落在 BKP 区间时）。
    fn bkp_index(offset: u32) -> Option<usize> {
        if offset >= OFF_BKP && offset < OFF_BKP + BKP_COUNT as u32 * 4 && offset % 4 == 0 {
            Some(((offset - OFF_BKP) / 4) as usize)
        } else {
            None
        }
    }
}

impl Peripheral for Rtc {
    fn name(&self) -> &str {
        "RTC"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        // BKP 寄存器
        if let Some(idx) = Self::bkp_index(offset) {
            if size != 4 {
                return Err(BusError::NotImplemented);
            }
            return Ok(self.bkp[idx]);
        }
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_TR => Ok(self.read_tr()),
            OFF_DR => Ok(self.read_dr()),
            OFF_CR => Ok(self.cr),
            OFF_ISR => {
                let mut v = self.isr_flags;
                v |= if self.init { ISR_INIT | ISR_INITF } else { ISR_RSF };
                if self.initialized {
                    v |= ISR_INITS;
                }
                if self.cr & CR_ALRAE == 0 {
                    v |= ISR_ALRAWF;
                }
                if self.cr & CR_ALRBE == 0 {
                    v |= ISR_ALRBWF;
                }
                if self.cr & CR_WUTE == 0 {
                    v |= ISR_WUTWF;
                }
                Ok(v)
            }
            OFF_PRER => Ok(self.prer),
            OFF_WUTR => Ok(self.wutr),
            OFF_CALIBR | OFF_SHIFTR | OFF_TSTR | OFF_TSDR | OFF_TSSSR | OFF_CALR
            | OFF_TAFCR => Ok(0),
            OFF_ALRMAR => Ok(self.alrmar),
            OFF_ALRMBR => Ok(self.alrmbr),
            OFF_WPR => Ok(0), // 只写寄存器
            OFF_SSR => Ok(self.ssr),
            OFF_ALRMASSR => Ok(self.alrmassr),
            OFF_ALRMBSSR => Ok(self.alrmbssr),
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        // WPR：支持字节/字写（固件常按 8 位访问）
        if offset == OFF_WPR {
            if size != 1 && size != 4 {
                return Err(BusError::NotImplemented);
            }
            self.write_wpr(value);
            return Ok(());
        }
        // BKP 寄存器：写访问需 PWR_CR.DBP=1（读始终允许）
        if let Some(idx) = Self::bkp_index(offset) {
            if size != 4 {
                return Err(BusError::NotImplemented);
            }
            if self.pwr.lock().unwrap().backup_domain_writable() {
                self.bkp[idx] = value;
            }
            return Ok(());
        }
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_TR | OFF_DR | OFF_PRER | OFF_WUTR | OFF_ALRMAR | OFF_ALRMBR
            | OFF_ALRMASSR | OFF_ALRMBSSR => {
                // 受保护寄存器：写保护未解锁则忽略
                if !self.unlocked {
                    return Ok(());
                }
                match offset {
                    OFF_TR => {
                        if self.init {
                            self.write_tr(value);
                        }
                    }
                    OFF_DR => {
                        if self.init {
                            self.write_dr(value);
                        }
                    }
                    OFF_PRER => {
                        if self.init {
                            self.prer = value & (PRER_PREDIV_A | PRER_PREDIV_S);
                            self.accum = 0;
                            self.ssr = self.prer & PRER_PREDIV_S;
                        }
                    }
                    OFF_WUTR => {
                        // WUTR 需 WUTWF=1（WUTE=0）才可写
                        if self.cr & CR_WUTE == 0 {
                            self.wutr = value & 0xFFFF;
                        }
                    }
                    OFF_ALRMAR => {
                        if self.cr & CR_ALRAE == 0 {
                            self.alrmar = value;
                        }
                    }
                    OFF_ALRMBR => {
                        if self.cr & CR_ALRBE == 0 {
                            self.alrmbr = value;
                        }
                    }
                    OFF_ALRMASSR => {
                        if self.cr & CR_ALRAE == 0 {
                            self.alrmassr = value;
                        }
                    }
                    OFF_ALRMBSSR => {
                        if self.cr & CR_ALRBE == 0 {
                            self.alrmbssr = value;
                        }
                    }
                    _ => unreachable!(),
                }
                Ok(())
            }
            OFF_CR => {
                if self.unlocked {
                    self.write_cr(value);
                }
                Ok(())
            }
            OFF_ISR => {
                self.write_isr(value);
                Ok(())
            }
            OFF_CALIBR | OFF_SHIFTR | OFF_TSTR | OFF_TSDR | OFF_TSSSR | OFF_CALR
            | OFF_TAFCR => Ok(()), // 未实现：写入忽略
            _ => Err(BusError::OutOfRange),
        }
    }

    /// 外设复位：RTC 核心恢复默认值；备份寄存器保留（VBAT 域掉电保持）。
    fn reset(&mut self) {
        self.time_s = self.time_from_ymd_hms(2021, 1, 1, 0, 0, 0);
        self.accum = 0;
        self.cr = 0;
        self.isr_flags = 0;
        self.init = false;
        self.initialized = false;
        self.prer = 0x007F_00FF;
        self.ssr = self.prer & PRER_PREDIV_S;
        self.wutr = 0xFFFF;
        self.wut_cnt = 0xFFFF;
        self.wut_accum = 0;
        self.alrmar = 0;
        self.alrmbr = 0;
        self.alrmassr = 0;
        self.alrmbssr = 0;
        self.wpr_step = 0;
        self.unlocked = false;
        // bkp 保持不变
        self.sync_active();
    }

    fn tick(&mut self, cycles: u64) {
        // 唤醒定时器独立于初始化模式推进
        self.advance_wakeup(cycles);
        if !self.init {
            self.advance(cycles);
        }
    }
}

// ---------- BCD 与历法换算工具 ----------

/// 两个 BCD 位组打包（个位 | 十位<<4）。
fn bcd_pack(v: u8) -> u32 {
    ((v / 10) as u32) << 4 | (v % 10) as u32
}

/// 解析 BCD 位组为十进制数（非法值截断）。
fn bcd_to_u8(v: u32) -> u8 {
    let units = v & 0xF;
    let tens = (v >> 4) & 0xF;
    let mut r = units + tens * 10;
    if r > 99 {
        r = 99;
    }
    r as u8
}

/// 2000-01-01 相对公历纪元（1970-01-01）的天数
const DAYS_2000_EPOCH: i64 = days_from_civil(2000, 1, 1);

/// 公历 (y,m,d) → 自 1970-01-01 天数（Howard Hinnant 历法算法）。
const fn days_from_civil(y: i64, m: u64, d: u64) -> i64 {
    let y = y - if m <= 2 { 1 } else { 0 };
    let era = (if y >= 0 { y } else { y - 399 }) / 400;
    let yoe = (y - era * 400) as u64;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146097 + doe as i64 - 719468
}

/// 自 1970-01-01 天数 → 公历 (y,m,d)。
fn civil_from_days(z: i64) -> (i64, u64, u64) {
    let z = z + 719468;
    let era = (if z >= 0 { z } else { z - 146096 }) / 146097;
    let doe = (z - era * 146097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (if m <= 2 { y + 1 } else { y }, m, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn civil_roundtrip() {
        for (y, m, d) in [(2000, 1, 1), (2021, 1, 1), (2023, 6, 1), (2024, 2, 29), (2099, 12, 31)] {
            let days = days_from_civil(y, m, d);
            assert_eq!(civil_from_days(days), (y, m, d), "roundtrip {y}-{m}-{d}");
        }
    }

    #[test]
    fn weekday_known() {
        // 2000-01-01 为周六（6）
        let r = Rtc::new(Arc::new(Mutex::new(Nvic::new())), Arc::new(Mutex::new(Pwr::new(
            Arc::new(crate::peripheral::wdog::WdogResetReq::new()),
        ))));
        assert_eq!(r.ymd(), (2021, 1, 1, 5)); // 2021-01-01 周五
    }

    #[test]
    fn bcd_helpers() {
        assert_eq!(bcd_pack(59), 0x59);
        assert_eq!(bcd_to_u8(0x59), 59);
        assert_eq!(bcd_to_u8(0x0F), 15);
    }
}
