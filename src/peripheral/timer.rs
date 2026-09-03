//! 通用定时器（STM32F407，TIM1-14，M6 集）。
//!
//! 一个 [`Timer`] 结构体按 [`TimerKind`] 参数化覆盖三类定时器，通道数由
//! [`TimerConfig::channels`] 指定：
//! - 基本定时器（TIM6/7）：仅时基 + 更新事件，无捕获/比较通道；
//! - 通用定时器（TIM2-5）：时基 + 4 路捕获/比较通道（TIM2/5 为 32 位）；
//!   TIM9/12 为 2 通道、TIM10/11/13/14 为 1 通道（均 16 位，无 DMA 请求能力）；
//! - 高级控制定时器（TIM1/8）：通用全部功能 + 互补输出/死区（BDTR.DTG/MOE）
//!   + 刹车（BKE/BG/BIF）+ 重复计数（RCR）。
//!
//! 中断分配（F407 共享 IRQ 行）：TIM9↔TIM1_BRK(24)、TIM10↔TIM1_UP(25)、
//! TIM11↔TIM1_TRG_COM(26)、TIM12↔TIM8_BRK(43)、TIM13↔TIM8_UP(44)、
//! TIM14↔TIM8_TRG_COM(45)——更新/捕获共用该行（NVIC 按中断号挂起）。
//!
//! 计数语义（M3 T1 集）：按 [`Peripheral::tick`] 推进的虚拟周期驱动 CNT 递增/递减
//! （块级加权周期，见 [`crate::sim::timing`]），溢出/下溢时按重复计数（高级定时器）
//! 延迟生成更新事件：置 SR.UIF、DIER.UIE 使能时向共享 NVIC 置挂起更新中断。
//!
//! 捕获/比较通道：
//! - 输出比较模式（CCxS=00，OCxM=000..011）：CNT==CCRx 时置 CCxIF，
//!   CCxIE 使能时置挂起捕获/比较中断；
//! - PWM 模式（OCxM=110 模式1：CNT<CCR 输出高；OCxM=111 模式2：CNT>=CCR 输出高）：
//!   计算 OCxREF 电平，变化时发布 [`Event::TimPwm`]（供未来 GPIO/虚拟示波器接线）；
//! - 互补输出/死区（高级）：CCER.CCxNE 使能 OCxN（= !OCxREF）；BDTR.DTG 死区
//!   延迟仅作用于"变有效"沿（OCx/OCxN 上升沿延迟 DTG、下降沿立即），死区窗口内
//!   两路同为无效电平（防直通）；BDTR.MOE 主输出门控，MOE=0 强制两路无效电平，
//!   刹车（BKE/BG）即触发该门控，AOE 更新事件恢复 MOE 并再同步输出；
//! - 输入捕获（CCxS!=00）：外部边沿经 [`Timer::feed_edge`]（或
//!   [`Event::TimEdge`] 注入）锁存 CNT→CCRx 并置 CCxIF + CCxIE 中断。
//!
//! M6 扩展（DMA 模式，对齐 USART/I2C/SPI/ADC 外设↔内存搬运语义）：
//! - 更新事件且 DIER.UDE（bit8）使能 → 发布 [`Event::TimUpdate`]，
//!   由 Machine 按 F407 固定映射路由到对应 DMA 流（TIM2_UP → DMA1_Stream5_Ch5 等）；
//! - 内存→外设：DMA 把内存表经 [`Timer::dma_write_dr`] 写入 DMAR，DCR.DBA/DBL
//!   指定突发目标寄存器（DBA=字偏移，突发长度 = DBL+1，按序号回绕）——经典用法
//!   如更新事件逐拍把 CCR 表装入 TIM 生成波形；
//! - 外设→内存：DMA 经 [`Timer::dma_read_dr`] 从 DMAR 读出（同样按 DBA/DBL 突发）。
//!
//! 地址映射（offset 相对各 TIM 基址；TIM2/5 的 CNT/ARR/PSC/CCR 为 32 位，
//! 其余为 16 位，读回按位宽掩码）：
//! - CR1 0x00（CEN=bit0, UDIS=bit1, DIR=bit4, ARPE=bit7）
//! - CR2 0x04（MMS=bit6:4）/ SMCR 0x08（外部触发，暂不实现）
//! - DIER 0x0C（UIE=bit0, CC1-4IE=bit1-4, UDE=bit8, CC1-4DE=bit9-12）
//! - SR 0x10（UIF=bit0, CC1-4IF=bit1-4, BIF=bit7 高级）/ EGR 0x14（UG=bit0,
//!   CC1-4G=bit1-4, BG=bit7 高级）
//! - CCMR1 0x18 / CCMR2 0x1C（CCxS=bit1:0, OCxM=bit6:4）
//! - CCER 0x20（CCxE=bit4x, CCxP=bit4x+1, CCxNE=bit4x+2 高级）
//! - CNT 0x24 / PSC 0x28 / ARR 0x2C / RCR 0x30（高级）/ CCR1-4 0x34-0x40
//! - BDTR 0x44（高级：DTG=bit7:0, BKE=bit12, BKP=bit13, AOE=bit14, MOE=bit15）
//! - DCR 0x48（DBL=bit4:0, DBA=bit12:8）/ DMAR 0x4C

use std::sync::{Arc, Mutex};

use crate::events::{Event, EventBus};
use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};

/// 定时器类别（决定寄存器集与功能，见 RM0090 §17-20）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerKind {
    /// 基本定时器（TIM6/7）：仅时基 + 更新事件，无通道
    Basic,
    /// 通用定时器（TIM2-5）：时基 + 4 路捕获/比较通道（TIM2/5 为 32 位）
    General,
    /// 高级控制定时器（TIM1/8）：通用全部功能 + 互补输出/死区 + 刹车 + RCR 重复计数
    Advanced,
}

/// 定时器中断表（F407 固定分配，见 stm32f407xx.h IRQn_Type）
#[derive(Debug, Clone, Copy)]
pub struct TimerIrq {
    /// 刹车中断（仅高级 TIM1/8；其它用更新号占位）
    pub brk: u32,
    /// 更新中断（TIM1=25、TIM2=28、TIM3=29、TIM4=30、TIM5=50、TIM6=54、TIM7=55、TIM8=44）
    pub up: u32,
    /// 触发/COM 中断（仅高级 TIM1/8；其它用更新号占位）
    pub trig_com: u32,
    /// 捕获/比较中断（TIM1=27、TIM8=46；通用/基本定时器与更新共用更新号）
    pub cc: u32,
}

/// 定时器配置（Machine 挂载时按型号传入）
pub struct TimerConfig {
    pub name: &'static str,
    pub kind: TimerKind,
    /// CNT/ARR/PSC/CCR 位宽：TIM2/5 = 32，其余 = 16
    pub bits: u32,
    /// 捕获/比较通道数：TIM1-5/8 = 4，TIM9/12 = 2，TIM10/11/13/14 = 1，
    /// 基本定时器（TIM6/7）= 0（仅时基 + 更新事件）
    pub channels: u8,
    pub irq: TimerIrq,
}

/// CR1 控制位
const CR1_CEN: u32 = 1 << 0; // 计数器使能
const CR1_UDIS: u32 = 1 << 1; // 更新禁止：禁止更新事件生成（计数器仍计数）
const CR1_DIR: u32 = 1 << 4; // 方向：0=向上, 1=向下

/// DIER 中断/请求使能位
const DIER_UIE: u32 = 1 << 0; // 更新中断使能
const DIER_CC1IE: u32 = 1 << 1; // 捕获/比较 1 中断使能
const DIER_UDE: u32 = 1 << 8; // 更新 DMA 请求使能

/// SR 状态位
const SR_UIF: u32 = 1 << 0; // 更新标志
const SR_CC1IF: u32 = 1 << 1; // 捕获/比较 1 标志
const SR_BIF: u32 = 1 << 7; // 刹车标志（高级）

/// EGR 事件生成位
const EGR_UG: u32 = 1 << 0; // 软件更新
const EGR_CC1G: u32 = 1 << 1; // 软件捕获/比较
const EGR_BG: u32 = 1 << 7; // 软件刹车（高级）

/// CCER 位
const CCER_CC1E: u32 = 1 << 0; // 通道1 输出使能 / 输入捕获使能
const CCER_CC1P: u32 = 1 << 1; // 通道1 极性
const CCER_CC1NE: u32 = 1 << 2; // 通道1 互补输出使能（高级）

/// BDTR 位（高级）
const BDTR_DTG_MASK: u32 = 0xFF; // 死区时间发生器
const BDTR_BKE: u32 = 1 << 12; // 刹车使能
const BDTR_AOE: u32 = 1 << 14; // 自动输出使能（下次更新事件置 MOE）
const BDTR_MOE: u32 = 1 << 15; // 主输出使能

/// 寄存器偏移
const OFF_CR1: u32 = 0x00;
const OFF_DIER: u32 = 0x0C;
const OFF_SR: u32 = 0x10;
const OFF_EGR: u32 = 0x14;
const OFF_CCMR1: u32 = 0x18;
const OFF_CCMR2: u32 = 0x1C;
const OFF_CCER: u32 = 0x20;
const OFF_CNT: u32 = 0x24;
const OFF_PSC: u32 = 0x28;
const OFF_ARR: u32 = 0x2C;
const OFF_RCR: u32 = 0x30; // 重复计数（高级）
const OFF_CCR1: u32 = 0x34;
const OFF_CCR2: u32 = 0x38;
const OFF_CCR3: u32 = 0x3C;
const OFF_CCR4: u32 = 0x40;
const OFF_BDTR: u32 = 0x44; // 刹车/死区（高级）
const OFF_DCR: u32 = 0x48;
// DMAR @ 0x4C：通用寄存器文件覆盖（idx 19），DMA 经 dma_read_dr/dma_write_dr 访问

/// 寄存器文件数（CR1..DMAR，含保留位，共 20 个 32 位寄存器）
const REG_COUNT: usize = 20;

/// DCR 位段
const DCR_DBL_MASK: u32 = 0x1F; // DBL[4:0]：突发长度（传输个数 = DBL+1）
const DCR_DBA_SHIFT: u32 = 8; // DBA[4:0]：突发基址（字偏移）

/// 死区计时器：记录一次被死区延迟的输出跳变（仅高级定时器使用）。
///
/// 死区语义（RM0090 §20.3.4）：互补输出切换时"变为有效"的一方被死区延迟，
/// "变为无效"的一方立即切换——从而在死区窗口内主/互补输出同为无效电平，
/// 避免半桥直通。等效模型：OCx 与 OCxN 的**上升沿（变有效）延迟 DTG，
/// 下降沿（变无效）立即**。
#[derive(Clone, Copy, Default)]
struct DeadTimer {
    /// 剩余 timer 内核时钟周期（0 = 空闲/已生效）
    remaining: u32,
    /// 死区结束后到达的目标电平
    target: bool,
}

/// 通用定时器外设（TIM1-8）
pub struct Timer {
    /// 端口号（TIM1..8 = 1..8，用于事件过滤 / DMA 句柄索引）
    pub port: u8,
    /// 外设名（"TIM1".."TIM8"，调试/日志）
    pub name: &'static str,
    /// 定时器类别
    pub kind: TimerKind,
    /// 计数/重装载/比较位宽（16 或 32）
    pub bits: u32,
    /// 中断表
    pub irq: TimerIrq,
    /// 寄存器文件（CR1..DMAR 共 20 个 32 位寄存器）
    regs: [u32; REG_COUNT],
    /// 计数器时钟余数（PSC 分频的亚周期累积）
    prescaler_remainder: u64,
    /// DMA 突发序号（DMAR 读/写按 DBA + 序号 % (DBL+1) 寻址目标寄存器）
    dma_burst_index: u32,
    /// 重复计数（高级 RCR 影子：向下计数，归零后生成更新事件）
    rep_counter: u32,
    /// 各通道 OCxREF 原始比较电平（PWM 比较结果，MOE 门控前的源信号；
    /// 供互补输出死区调度判定边沿）
    ocx_ref: [bool; 4],
    /// 各通道 OCx 实际驱动输出电平（受 MOE 门控；变化时发布 [`Event::TimPwm`]）
    oc_ref: [bool; 4],
    /// 各通道 OCxN 互补驱动输出电平（受 MOE 门控 + 死区延迟；
    /// 变化时以通道号 ch+4 发布 [`Event::TimPwm`]）
    ocn_ref: [bool; 4],
    /// 捕获/比较通道数（来自配置；约束 check_compare 等仅迭代有效通道，
    /// TIM9/12=2、TIM10/11/13/14=1、TIM6/7=0）
    channels: u8,
    /// 主输出 OCx 的待生效跳变（变有效沿经死区延迟）
    dead_main: [DeadTimer; 4],
    /// 互补输出 OCxN 的待生效跳变（变有效沿经死区延迟）
    dead_comp: [DeadTimer; 4],
    /// 上次已知 MOE 状态（检测 0→1 上升沿以再同步输出；高级定时器）
    moe_state: bool,
    /// 事件总线（更新事件 + UDE → 发布 TimUpdate；OCx/OCxN 电平变化 → TimPwm）
    bus: Arc<Mutex<EventBus>>,
    /// 共享 NVIC（更新事件 → 更新中断；CC 匹配/捕获 → CC 中断；刹车 → 刹车中断）
    nvic: Arc<Mutex<Nvic>>,
}

impl Timer {
    pub fn new(port: u8, cfg: TimerConfig, bus: Arc<Mutex<EventBus>>, nvic: Arc<Mutex<Nvic>>) -> Self {
        let mut t = Self {
            port,
            name: cfg.name,
            kind: cfg.kind,
            bits: cfg.bits,
            irq: cfg.irq,
            regs: [0; REG_COUNT],
            prescaler_remainder: 0,
            dma_burst_index: 0,
            rep_counter: 0,
            ocx_ref: [false; 4],
            oc_ref: [false; 4],
            ocn_ref: [false; 4],
            channels: cfg.channels,
            dead_main: [DeadTimer::default(); 4],
            dead_comp: [DeadTimer::default(); 4],
            moe_state: false,
            bus,
            nvic,
        };
        t.reset();
        t
    }

    /// CNT/ARR/PSC/CCR 位宽掩码（16 位定时器截断到 0xFFFF）
    fn cnt_mask(&self) -> u32 {
        if self.bits == 32 {
            0xFFFF_FFFF
        } else {
            0xFFFF
        }
    }

    /// 捕获/比较通道数（来自配置：TIM1-5/8 = 4、TIM9/12 = 2、TIM10/11/13/14 = 1、
    /// 基本定时器 TIM6/7 = 0 仅时基 + 更新事件）
    fn channels(&self) -> usize {
        self.channels as usize
    }

    /// 通道 ch 的 (CCxS, OCxM) 配置（取自 CCMR1/CCMR2 对应字节）
    fn cc_config(&self, ch: usize) -> (u32, u32) {
        let reg = if ch < 2 { OFF_CCMR1 } else { OFF_CCMR2 };
        let shift = (ch % 2) * 8;
        let v = self.regs[(reg / 4) as usize];
        ((v >> shift) & 0x3, (v >> (shift + 4)) & 0x7)
    }

    /// 生成一次更新事件（UEV）：置 UIF，UIE 使能时向 NVIC 置挂起更新中断；
    /// UDE 使能时发布 [`Event::TimUpdate`]（更新事件 → DMA 请求）。
    /// CR1.UDIS 置位时更新事件被禁止（计数器继续计数，见 RM0090 §17.4.4）。
    /// 高级定时器且 AOE 置位时，更新事件恢复 MOE（刹车后的自动重使能）。
    ///
    /// 注意：tick/寄存器写路径不持有事件总线锁（发布在 Machine 的事件分发之外），
    /// 因此可以直接 publish，无需像 feed_rx 那样改由订阅者二次路由。
    fn update_event(&mut self) {
        if self.regs[OFF_CR1 as usize / 4] & CR1_UDIS != 0 {
            return; // UDIS：不生成更新事件
        }
        self.regs[OFF_SR as usize / 4] |= SR_UIF;
        if self.kind == TimerKind::Advanced && self.regs[OFF_BDTR as usize / 4] & BDTR_AOE != 0 {
            self.regs[OFF_BDTR as usize / 4] |= BDTR_MOE; // 自动输出使能
        }
        let dier = self.regs[OFF_DIER as usize / 4];
        if dier & DIER_UIE != 0 {
            self.nvic.lock().unwrap().set_pending(self.irq.up);
        }
        if dier & DIER_UDE != 0 {
            self.bus
                .lock()
                .unwrap()
                .publish(&Event::TimUpdate { port: self.port });
        }
    }

    /// 计数溢出/下溢事件（每个计数周期一次）：
    /// 高级定时器按 RCR 重复计数延迟更新事件，其余定时器每次溢出即更新事件。
    fn counter_overflow(&mut self) {
        if self.kind == TimerKind::Advanced {
            if self.rep_counter > 0 {
                self.rep_counter -= 1; // 尚未到更新事件（CNT 已回绕）
                return;
            }
            self.rep_counter = self.regs[OFF_RCR as usize / 4] & 0xFF;
        }
        self.update_event();
    }

    /// 置位通道 ch 的 CCxIF；CCxIE 使能时置挂起捕获/比较中断。
    /// 重复置位不重复挂起（标志已置位时跳过）。
    fn set_cc_if(&mut self, ch: usize) {
        let bit = SR_CC1IF << ch;
        if self.regs[OFF_SR as usize / 4] & bit != 0 {
            return;
        }
        self.regs[OFF_SR as usize / 4] |= bit;
        let dier = self.regs[OFF_DIER as usize / 4];
        if dier & (DIER_CC1IE << ch) != 0 {
            self.nvic.lock().unwrap().set_pending(self.irq.cc);
        }
    }

    /// 软件捕获/比较（EGR.CC1-4G）：输出模式强制比较匹配；
    /// 输入模式强制捕获（锁存 CNT → CCRx）。
    fn generate_cc(&mut self, ch: usize) {
        let (ccs, _ocm) = self.cc_config(ch);
        if ccs == 0 {
            // 输出模式：强制产生比较事件
            self.set_cc_if(ch);
        } else {
            // 输入模式：强制捕获 CNT → CCRx
            self.regs[OFF_CCR1 as usize / 4 + ch] = self.regs[OFF_CNT as usize / 4] & self.cnt_mask();
            self.set_cc_if(ch);
        }
    }

    /// 刹车事件（仅高级）：清 MOE（主输出关闭）、置 BIF、挂起刹车中断。
    ///
    /// MOE 清零同时把主/互补输出强制为无效电平（发布低电平事件）——
    /// 半桥在刹车后两路输出同时关断（RM0090 §20.3.11，OSSI=0 简化）。
    fn break_event(&mut self) {
        if self.kind != TimerKind::Advanced {
            return;
        }
        self.regs[OFF_BDTR as usize / 4] &= !BDTR_MOE;
        self.regs[OFF_SR as usize / 4] |= SR_BIF;
        self.nvic.lock().unwrap().set_pending(self.irq.brk);
        // MOE 清零：取消进行中的死区跳变，主/互补输出强制无效电平
        for ch in 0..self.channels() {
            self.dead_main[ch].remaining = 0;
            self.dead_comp[ch].remaining = 0;
            if self.oc_ref[ch] {
                self.oc_ref[ch] = false;
                self.publish_pwm(ch as u8, false);
            }
            if self.ocn_ref[ch] {
                self.ocn_ref[ch] = false;
                self.publish_pwm(ch as u8 + 4, false);
            }
        }
    }

    /// 当前 CNT 值（测试读取用）
    pub fn count(&self) -> u32 {
        self.regs[OFF_CNT as usize / 4]
    }

    /// 通道 ch 的 OCx 输出电平（PWM/输出比较结果，受 MOE 门控，测试/外部接线查询用）
    pub fn oc_ref(&self, ch: usize) -> bool {
        self.oc_ref.get(ch).copied().unwrap_or(false)
    }

    /// 通道 ch 的 OCxN 互补输出电平（受 MOE 门控 + 死区延迟，测试/外部接线查询用）
    pub fn ocn_ref(&self, ch: usize) -> bool {
        self.ocn_ref.get(ch).copied().unwrap_or(false)
    }

    /// 发布通道电平变化事件（主通道 0-3，互补通道 4-7）。
    /// 调用方须先更新输出电平再发布（电平未变时不发布，避免事件噪声）。
    fn publish_pwm(&self, channel: u8, level: bool) {
        self.bus
            .lock()
            .unwrap()
            .publish(&Event::TimPwm { port: self.port, channel, level });
    }

    /// 设置主输出 OCx 电平并发布（MOE=0 时强制无效电平）。
    fn set_main_level(&mut self, ch: usize, level: bool, moe: bool) {
        let level = if moe { level } else { false };
        if level != self.oc_ref[ch] {
            self.oc_ref[ch] = level;
            self.publish_pwm(ch as u8, level);
        }
    }

    /// 设置互补输出 OCxN 电平并发布（MOE=0 时强制无效电平；通道号 +4）。
    fn set_comp_level(&mut self, ch: usize, level: bool, moe: bool) {
        let level = if moe { level } else { false };
        if level != self.ocn_ref[ch] {
            self.ocn_ref[ch] = level;
            self.publish_pwm(ch as u8 + 4, level);
        }
    }

    /// 推进死区计时器（timer 内核时钟周期计数）：剩余周期递减，
    /// 归零后把延迟的跳变写入输出并发布。每 tick 调用一次（含 CNT 未步进的周期）。
    fn advance_dead_time(&mut self, cycles: u64) {
        if cycles == 0 {
            return;
        }
        let moe = if self.kind == TimerKind::Advanced {
            self.regs[OFF_BDTR as usize / 4] & BDTR_MOE != 0
        } else {
            true
        };
        for ch in 0..self.channels() {
            if self.dead_main[ch].remaining > 0 {
                self.dead_main[ch].remaining =
                    self.dead_main[ch].remaining.saturating_sub(cycles as u32);
                if self.dead_main[ch].remaining == 0 {
                    self.set_main_level(ch, self.dead_main[ch].target, moe);
                }
            }
            if self.dead_comp[ch].remaining > 0 {
                self.dead_comp[ch].remaining =
                    self.dead_comp[ch].remaining.saturating_sub(cycles as u32);
                if self.dead_comp[ch].remaining == 0 {
                    self.set_comp_level(ch, self.dead_comp[ch].target, moe);
                }
            }
        }
    }

    /// BDTR.DTG 死区时长（timer 时钟周期数，RM0090 分段公式；非高级定时器为 0）
    pub fn dead_time(&self) -> u32 {
        if self.kind != TimerKind::Advanced {
            return 0;
        }
        let dtg = self.regs[OFF_BDTR as usize / 4] & BDTR_DTG_MASK;
        match dtg {
            0..=0x7F => dtg,                                          // Tdtg = tCK_INT × DTG
            0x80..=0xBF => (64 + (dtg - 0x80)) * 2,                   // × (64+DTG-0x80) × 2
            0xC0..=0xDF => (32 + (dtg - 0xC0)) * 8,                   // × (32+DTG-0xC0) × 8
            _ => (32 + (dtg - 0xE0)) * 16,                            // × (32+DTG-0xE0) × 16
        }
    }

    /// 外部输入边沿 → 输入捕获（测试/虚拟引脚驱动）。
    ///
    /// CCER.CCxE 置位且 CCxS!=00（输入模式）时，按 CCxP/CCxNP 极性判定有效边沿：
    /// 有效则锁存 CNT→CCRx、置 CCxIF，CCxIE 使能时置挂起捕获/比较中断。
    pub fn feed_edge(&mut self, channel: u8, level: bool) {
        let ch = channel as usize;
        if ch >= self.channels() {
            return;
        }
        let ccer = self.regs[OFF_CCER as usize / 4];
        if ccer & (CCER_CC1E << (ch * 4)) == 0 {
            return; // 通道未使能
        }
        let (ccs, _ocm) = self.cc_config(ch);
        if ccs == 0 {
            return; // 输出模式不接受外部边沿
        }
        // 极性（简化：CCxNP=0 时 CCxP=0→上升沿/CCxP=1→下降沿；CCxNP=1→双沿都捕获）
        let ccxp = ccer & (CCER_CC1P << (ch * 4)) != 0;
        let ccxnp = ccer & (1 << (ch * 4 + 3)) != 0;
        let active = ccxnp || if ccxp { !level } else { level };
        if !active {
            return;
        }
        self.regs[OFF_CCR1 as usize / 4 + ch] = self.regs[OFF_CNT as usize / 4] & self.cnt_mask();
        self.set_cc_if(ch);
    }

    /// 按 CCx 配置计算并更新各通道输出（PWM 电平 / 输出比较匹配标志）。
    /// 仅在 tick 推进 CNT 后调用（寄存器写路径不重入，避免事件发布重入）。
    ///
    /// 输出波形模型（高级定时器互补 + 死区，RM0090 §20.3.4）：
    /// - OCx 跟随 OCxREF；OCxN 为 OCxREF 反相；
    /// - 死区延迟仅作用于"变为有效"沿（OCx/OCxN 的上升沿延迟 DTG），
    ///   "变为无效"沿立即切换 → 死区窗口内两路同为无效电平（避免直通）；
    /// - MOE=0 时主/互补输出均强制无效电平（set_*_level 内门控）。
    fn check_compare(&mut self) {
        let ccer = self.regs[OFF_CCER as usize / 4];
        let cnt = self.regs[OFF_CNT as usize / 4] & self.cnt_mask();
        let is_adv = self.kind == TimerKind::Advanced;
        let moe = if is_adv {
            self.regs[OFF_BDTR as usize / 4] & BDTR_MOE != 0
        } else {
            true // 非高级定时器无 MOE，输出始终使能
        };
        let dt = self.dead_time(); // 高级定时器死区（timer 内核时钟周期）；否则 0

        // MOE 重新使能（0→1，如 AOE 更新事件恢复）：把主/互补输出再同步到
        // 当前 OCxREF（刹车期间被强制无效电平；无待生效死区跳变时立即恢复）
        if is_adv && moe && !self.moe_state {
            for ch in 0..self.channels() {
                let out_en = ccer & (CCER_CC1E << (ch * 4)) != 0;
                let outn_en = is_adv && ccer & (CCER_CC1NE << (ch * 4)) != 0;
                if out_en && self.dead_main[ch].remaining == 0 {
                    self.set_main_level(ch, self.ocx_ref[ch], moe);
                }
                if outn_en && self.dead_comp[ch].remaining == 0 {
                    self.set_comp_level(ch, !self.ocx_ref[ch], moe);
                }
            }
        }
        self.moe_state = moe;

        for ch in 0..self.channels() {
            let (ccs, ocm) = self.cc_config(ch);
            let out_en = ccer & (CCER_CC1E << (ch * 4)) != 0;
            let outn_en = is_adv && ccer & (CCER_CC1NE << (ch * 4)) != 0;
            if ccs != 0 || (!out_en && !outn_en) {
                continue; // 输入模式或输出未使能
            }
            let ccr = self.regs[OFF_CCR1 as usize / 4 + ch] & self.cnt_mask();
            let new_ref = match ocm {
                0b110 => cnt < ccr,  // PWM 模式1：CNT<CCR 时 OCxREF 高
                0b111 => cnt >= ccr, // PWM 模式2：CNT>=CCR 时 OCxREF 高
                _ => {
                    // 输出比较模式（OCxM 000-011）：CNT==CCR 置 CCxIF
                    if cnt == ccr && ocm < 0b100 {
                        self.set_cc_if(ch);
                    }
                    self.ocx_ref[ch] // 电平保持（强制/冻结模式）
                }
            };
            let prev_ref = self.ocx_ref[ch];
            let rise = new_ref && !prev_ref; // OCxREF 变有效（低→高）
            let fall = !new_ref && prev_ref; // OCxREF 变无效（高→低）
            if rise || fall {
                self.ocx_ref[ch] = new_ref;
                // 主输出 OCx：跟随 OCxREF，上升沿（变有效）延迟死区，下降沿立即
                if out_en {
                    if fall {
                        self.dead_main[ch].remaining = 0; // 取消未生效的上升跳变
                        self.set_main_level(ch, false, moe);
                    } else if dt == 0 {
                        self.set_main_level(ch, true, moe);
                    } else {
                        self.dead_main[ch] = DeadTimer { remaining: dt, target: true };
                    }
                }
                // 互补输出 OCxN：= !OCxREF，其上升沿（变有效）延迟死区，下降沿立即
                if outn_en {
                    if rise {
                        self.dead_comp[ch].remaining = 0; // 取消未生效的上升跳变
                        self.set_comp_level(ch, false, moe);
                    } else if dt == 0 {
                        self.set_comp_level(ch, true, moe);
                    } else {
                        self.dead_comp[ch] = DeadTimer { remaining: dt, target: true };
                    }
                }
            }
        }
    }

    /// DMA 读 DMAR（外设→内存方向）：按 DCR.DBA/DBL 突发返回目标寄存器值。
    ///
    /// 供 DMA 控制器搬运调用（每次读推进突发序号，DBL 内回绕）。
    pub fn dma_read_dr(&mut self) -> u32 {
        let idx = self.dma_target_idx();
        self.regs.get(idx).copied().unwrap_or(0)
    }

    /// DMA 写 DMAR（内存→外设方向）：按 DCR.DBA/DBL 突发写入目标寄存器。
    ///
    /// 供 DMA 控制器搬运调用；DBA=13 起始即 CCR1..CCR4 依次装入，
    /// 经典用法"更新事件 → DMA 突发装载 CCR 表生成波形"。
    pub fn dma_write_dr(&mut self, value: u32) {
        let idx = self.dma_target_idx();
        if idx < REG_COUNT {
            self.regs[idx] = value;
        }
    }

    /// 突发目标寄存器索引 = DBA + (序号 % (DBL+1))，DBA/DBL 取自 DCR。
    ///
    /// 硬件语义（RM0090）：DBL 为"传输个数 − 1"，故突发长度 = DBL+1（DBL=0 → 1 次）。
    fn dma_target_idx(&mut self) -> usize {
        let dcr = self.regs[OFF_DCR as usize / 4];
        let len = ((dcr & DCR_DBL_MASK) as usize) + 1;
        let dba = ((dcr >> DCR_DBA_SHIFT) & 0x1F) as usize;
        let idx = dba + (self.dma_burst_index as usize % len);
        self.dma_burst_index = self.dma_burst_index.wrapping_add(1);
        idx
    }
}

/// DMA 外设方向搬运接口实现（复用 inherent `dma_read_dr`/`dma_write_dr` 语义）。
impl crate::peripheral::dma::DmaByteIo for Timer {
    fn dma_read_dr(&mut self) -> u32 {
        self.dma_read_dr()
    }

    fn dma_write_dr(&mut self, value: u32) {
        self.dma_write_dr(value);
    }
}

impl Peripheral for Timer {
    fn name(&self) -> &str {
        self.name
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let idx = (offset / 4) as usize;
        let v = self.regs.get(idx).copied().ok_or(BusError::OutOfRange)?;
        // 16 位定时器的 CNT/PSC/ARR/CCR 读回截断到 16 位
        Ok(match offset {
            OFF_CNT | OFF_PSC | OFF_ARR | OFF_CCR1 | OFF_CCR2 | OFF_CCR3 | OFF_CCR4 => {
                v & self.cnt_mask()
            }
            _ => v,
        })
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_EGR => {
                // UG：软件更新事件（重复计数直接清零并生成更新事件）
                if value & EGR_UG != 0 {
                    if self.kind == TimerKind::Advanced {
                        self.rep_counter = 0;
                    }
                    self.update_event();
                    // 更新事件后 CNT 复位：向上→0，向下→ARR
                    if self.regs[OFF_CR1 as usize / 4] & CR1_DIR != 0 {
                        self.regs[OFF_CNT as usize / 4] = self.regs[OFF_ARR as usize / 4] & self.cnt_mask();
                    } else {
                        self.regs[OFF_CNT as usize / 4] = 0;
                    }
                    self.prescaler_remainder = 0;
                }
                // CC1-4G：软件捕获/比较
                for ch in 0..4 {
                    if value & (EGR_CC1G << ch) != 0 {
                        self.generate_cc(ch);
                    }
                }
                // BG：软件刹车（高级）
                if value & EGR_BG != 0 {
                    self.break_event();
                }
                Ok(())
            }
            OFF_SR => {
                // 状态位写 0 清除（rc_w0）
                self.regs[OFF_SR as usize / 4] &= value;
                Ok(())
            }
            OFF_RCR => {
                // 高级：重复计数直接装载（影子）
                self.regs[OFF_RCR as usize / 4] = value & 0xFF;
                self.rep_counter = value & 0xFF;
                Ok(())
            }
            OFF_CNT | OFF_PSC | OFF_ARR | OFF_CCR1 | OFF_CCR2 | OFF_CCR3 | OFF_CCR4 => {
                // 16 位定时器截断到 16 位
                self.regs[(offset / 4) as usize] = value & self.cnt_mask();
                Ok(())
            }
            OFF_BDTR => {
                // 高级：刹车/死区/主输出使能。软件直写 MOE 时同步 moe_state，
                // 避免首次 check_compare 把 MOE 由 0→1 误判为刹车恢复而做无意义
                // 的再同步（初始化直接置 MOE 时 OCxREF 尚为低，不应让 OCxN 假高）。
                // 刹车清 MOE 与 AOE 更新恢复 MOE 走内部路径（break_event/update_event），
                // moe_state 分别由 check_compare 与这里按实际写入值维护。
                self.regs[OFF_BDTR as usize / 4] = value;
                self.moe_state = value & BDTR_MOE != 0;
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
        // 计数器未使能（CEN=0）时不推进；UDIS 仅禁止更新事件，计数器仍计数
        let cr1 = self.regs[OFF_CR1 as usize / 4];
        if cr1 & CR1_CEN == 0 {
            return;
        }
        let psc = (self.regs[OFF_PSC as usize / 4] & self.cnt_mask()) as u64 + 1;
        let arr = self.regs[OFF_ARR as usize / 4] & self.cnt_mask();
        let period = arr as u64 + 1;

        // 死区计时按 timer 内核时钟周期推进（与 PSC 分频无关），
        // 须在 CNT 未步进（steps==0）时也推进，保证延迟按时钟到期
        self.advance_dead_time(cycles);

        // 分频：累积周期，按 (PSC+1) 折算计数器步进
        self.prescaler_remainder += cycles;
        let steps = self.prescaler_remainder / psc;
        self.prescaler_remainder %= psc;
        if steps == 0 {
            return;
        }

        if cr1 & CR1_DIR != 0 {
            // 向下计数：CNT 减到 0 后下溢回绕到 ARR
            let mut c = self.regs[OFF_CNT as usize / 4] as i64 - steps as i64;
            let mut underflows = 0u64;
            while c < 0 {
                c += period as i64;
                underflows += 1;
            }
            for _ in 0..underflows {
                self.counter_overflow();
            }
            self.regs[OFF_CNT as usize / 4] = (c as u32) & self.cnt_mask();
        } else {
            // 向上计数：CNT 超过 ARR 即溢出回绕，生成更新事件
            let mut c = self.regs[OFF_CNT as usize / 4] as u64 + steps;
            let mut overflows = 0u64;
            while c >= period {
                c -= period;
                overflows += 1;
            }
            for _ in 0..overflows {
                self.counter_overflow();
            }
            self.regs[OFF_CNT as usize / 4] = (c as u32) & self.cnt_mask();
        }

        // 捕获/比较：按更新后的 CNT 计算 OCxREF / 输出比较匹配
        self.check_compare();
    }

    fn reset(&mut self) {
        self.regs = [0; REG_COUNT];
        self.prescaler_remainder = 0;
        self.dma_burst_index = 0;
        self.rep_counter = 0;
        self.ocx_ref = [false; 4];
        self.oc_ref = [false; 4];
        self.ocn_ref = [false; 4];
        self.dead_main = [DeadTimer::default(); 4];
        self.dead_comp = [DeadTimer::default(); 4];
        self.moe_state = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::events::EventBus;
    use crate::peripheral::dma::DmaByteIo;

    /// TIM2 通用 32 位定时器（复刻现有 M6 行为）
    fn tim2() -> (Timer, Arc<Mutex<Nvic>>, Arc<Mutex<EventBus>>) {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let cfg = TimerConfig {
            name: "TIM2",
            kind: TimerKind::General,
            bits: 32,
            channels: 4,
            irq: TimerIrq { brk: 28, up: 28, trig_com: 28, cc: 28 },
        };
        (Timer::new(2, cfg, bus.clone(), nvic.clone()), nvic, bus)
    }

    /// TIM1 高级 16 位定时器
    fn tim1() -> (Timer, Arc<Mutex<Nvic>>, Arc<Mutex<EventBus>>) {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let cfg = TimerConfig {
            name: "TIM1",
            kind: TimerKind::Advanced,
            bits: 16,
            channels: 4,
            irq: TimerIrq { brk: 24, up: 25, trig_com: 26, cc: 27 },
        };
        (Timer::new(1, cfg, bus.clone(), nvic.clone()), nvic, bus)
    }

    #[test]
    fn overflow_sets_pending_and_wraps() {
        let (mut t, nvic, _) = tim2();
        // PSC=1（分频 2），ARR=100
        t.write(OFF_PSC, 4, 1).unwrap();
        t.write(OFF_ARR, 4, 100).unwrap();
        t.write(OFF_DIER, 4, DIER_UIE).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();

        // 未使能时不推进
        let (mut t2, _, _) = tim2();
        t2.write(OFF_ARR, 4, 10).unwrap();
        t2.tick(1000);
        assert_eq!(t2.count(), 0);

        // 使能后：1000 周期 / 2 = 500 步，ARR=100 → 4 次溢出，CNT=500-4*101=96
        t.tick(1000);
        assert_eq!(t.count(), 96);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF);
        assert!(nvic.lock().unwrap().is_pending(28), "UIE 使能应置挂起 TIM2_IRQ");

        // 写 SR 清 UIF（写 0 清除）
        t.write(OFF_SR, 4, 0).unwrap();
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, 0);
    }

    #[test]
    fn disabled_update_event_no_pending() {
        let (mut t, nvic, _) = tim2();
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap(); // CEN 但 UIE 未使能
        t.tick(1000);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF);
        assert!(!nvic.lock().unwrap().is_pending(28), "UIE 未使能不置挂起");
    }

    #[test]
    fn udis_suppresses_update_but_keeps_counting() {
        // UDIS=1：更新事件被禁止（不置 UIF / 不挂起中断 / 不发布 DMA 请求），
        // 但计数器继续计数（硬件语义，见 RM0090 §17.4.4）。
        let (mut t, nvic, bus) = tim2();
        let got = Arc::new(Mutex::new(0usize));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::TimUpdate { .. } = ev {
                    *g.lock().unwrap() += 1;
                }
            })));
        t.write(OFF_PSC, 4, 0).unwrap();
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_DIER, 4, DIER_UIE | DIER_UDE).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN | CR1_UDIS).unwrap();
        t.tick(21); // 21 步，ARR=10 → 1 次溢出，CNT=21-11=10
        assert_eq!(t.count(), 10, "UDIS 下计数器仍回绕计数");
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, 0, "UDIS 不置 UIF");
        assert!(!nvic.lock().unwrap().is_pending(28), "UDIS 不挂起更新中断");
        assert_eq!(*got.lock().unwrap(), 0, "UDIS 不发布 TimUpdate DMA 请求");
        // 清 UDIS 后再溢出 → 恢复更新事件
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();
        t.tick(11); // 11 步：CNT 10→10，1 次溢出
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF, "清 UDIS 后恢复更新事件");
    }

    #[test]
    fn ude_publishes_tim_update_event() {
        let (mut t, nvic, bus) = tim2();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::TimUpdate { port } = ev {
                    g.lock().unwrap().push(*port);
                }
            })));
        // UDE 使能 → 更新事件发布 TimUpdate
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_DIER, 4, DIER_UDE).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();
        t.tick(1000);
        assert!(!got.lock().unwrap().is_empty(), "UDE 使能应发布 TimUpdate");
        // UDE 未使能 → 不发布
        let (mut t2, _, bus2) = tim2();
        let got2 = Arc::new(Mutex::new(Vec::new()));
        let g2 = got2.clone();
        bus2.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::TimUpdate { .. } = ev {
                    g2.lock().unwrap().push(1);
                }
            })));
        t2.write(OFF_ARR, 4, 10).unwrap();
        t2.write(OFF_CR1, 4, CR1_CEN).unwrap();
        t2.tick(1000);
        assert!(got2.lock().unwrap().is_empty(), "UDE 未使能不发布 TimUpdate");
    }

    #[test]
    fn dma_burst_writes_ccr_table() {
        let (mut t, _, _) = tim2();
        // DCR：DBA=13（CCR1 字偏移），DBL=3（突发长度 = 3+1 = 4：CCR1..CCR4）
        t.write(OFF_DCR, 4, (13 << 8) | 3).unwrap();
        // DMA 突发写 4 字：依次装入 CCR1..CCR4
        for v in [0x1111u32, 0x2222, 0x3333, 0x4444] {
            t.dma_write_dr(v);
        }
        assert_eq!(t.read(0x34, 4).unwrap(), 0x1111); // CCR1
        assert_eq!(t.read(0x38, 4).unwrap(), 0x2222); // CCR2
        assert_eq!(t.read(0x3C, 4).unwrap(), 0x3333); // CCR3
        assert_eq!(t.read(0x40, 4).unwrap(), 0x4444); // CCR4
        // 第 5 次写回绕到 CCR1
        t.dma_write_dr(0x5555);
        assert_eq!(t.read(0x34, 4).unwrap(), 0x5555);
    }

    #[test]
    fn dma_burst_reads_back_table() {
        let (mut t, _, _) = tim2();
        t.write(OFF_DCR, 4, (13 << 8) | 3).unwrap();
        for (i, v) in [0xAAAAu32, 0xBBBB, 0xCCCC, 0xDDDD].iter().enumerate() {
            t.write(0x34 + i as u32 * 4, 4, *v).unwrap();
        }
        let mut out = Vec::new();
        for _ in 0..4 {
            out.push(t.dma_read_dr());
        }
        assert_eq!(out, vec![0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD]);
    }

    #[test]
    fn sixteen_bit_wraparound_and_32bit_contrast() {
        // TIM2 32 位：ARR=0x1_0000（>16 位）不截断
        let (mut t2, _, _) = tim2();
        t2.write(OFF_ARR, 4, 0x1_0000).unwrap();
        assert_eq!(t2.read(OFF_ARR, 4).unwrap(), 0x1_0000, "32 位定时器 ARR 不截断");

        // TIM3 16 位：ARR 写 0x1_FFFF 截断到 0xFFFF，读回 0xFFFF
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let cfg = TimerConfig {
            name: "TIM3",
            kind: TimerKind::General,
            bits: 16,
            channels: 4,
            irq: TimerIrq { brk: 29, up: 29, trig_com: 29, cc: 29 },
        };
        let mut t3 = Timer::new(3, cfg, bus, nvic);
        t3.write(OFF_ARR, 4, 0x1_FFFF).unwrap();
        assert_eq!(t3.read(OFF_ARR, 4).unwrap(), 0xFFFF, "16 位定时器 ARR 截断到 0xFFFF");

        // 16 位回绕：ARR=10，步进 20 → 1 次溢出后 CNT=20-11=9，SR.UIF 置位
        t3.write(OFF_PSC, 4, 0).unwrap();
        t3.write(OFF_ARR, 4, 10).unwrap();
        t3.write(OFF_CR1, 4, CR1_CEN).unwrap();
        t3.tick(20);
        assert_eq!(t3.count(), 9, "16 位向上计数回绕：20 步-11=9");
        assert_eq!(t3.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF);
    }

    #[test]
    fn down_counting_wraps_to_arr() {
        let (mut t, _, _) = tim2();
        t.write(OFF_PSC, 4, 0).unwrap();
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN | CR1_DIR).unwrap(); // 向下计数
        // 初始 CNT=0：向下 1 步 → 下溢回绕到 ARR=10
        t.tick(1);
        assert_eq!(t.count(), 10);
        // 再向下 3 步：10-3=7
        t.tick(3);
        assert_eq!(t.count(), 7);
        // 向下 8 步：7,6,5,4,3,2,1,0 → 第 8 步下溢回绕到 ARR=10，下溢 1 次 → UIF
        t.tick(8);
        assert_eq!(t.count(), 10);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF);
    }

    #[test]
    fn pwm_mode1_ocref_by_compare() {
        let (mut t, _, bus) = tim2();
        // 收集 TimPwm 事件
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::TimPwm { port, channel, level } = ev {
                    g.lock().unwrap().push((*port, *channel, *level));
                }
            })));
        // PWM 模式1（OC1M=110 → CCMR1 bit6:4 = 0b110），CC1E 使能，CCR1=5，ARR=10
        t.write(OFF_CCMR1, 4, 0b110 << 4).unwrap();
        t.write(OFF_CCER, 4, CCER_CC1E).unwrap();
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_CCR1, 4, 5).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();
        // 步进 1..10：CNT<5 时 OCxREF 高（PWM 模式1）
        t.tick(1); // CNT=1 → 高
        assert!(t.oc_ref(0), "CNT<CCR：OCxREF 应高");
        t.tick(3); // CNT=4 → 仍高
        assert!(t.oc_ref(0));
        t.tick(1); // CNT=5 → 低（5<5 为假）
        assert!(!t.oc_ref(0), "CNT>=CCR：OCxREF 应低");
        // 事件：变化时发布
        let evs = got.lock().unwrap();
        assert_eq!(evs.first().map(|e| e.2), Some(true), "首次 tick 应发布高电平");
        assert!(evs.iter().any(|e| !e.2), "CNT>=CCR 时应发布低电平");
        // PWM 模式不置 CCxIF
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_CC1IF, 0, "PWM 模式不置 CCxIF");
    }

    #[test]
    fn output_compare_sets_ccif_and_irq() {
        let (mut t, nvic, _) = tim2();
        // 输出比较（OC1M=000 冻结），CC1E + CC1IE 使能，CCR1=5，ARR=10
        t.write(OFF_CCMR1, 4, 0b000 << 4).unwrap();
        t.write(OFF_CCER, 4, CCER_CC1E).unwrap();
        t.write(OFF_DIER, 4, DIER_CC1IE).unwrap();
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_CCR1, 4, 5).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();
        t.tick(5); // CNT=5 匹配 CCR1
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_CC1IF, SR_CC1IF, "输出比较应置 CC1IF");
        assert!(nvic.lock().unwrap().is_pending(28), "CC1IE 使能应置挂起 CC 中断");
    }

    #[test]
    fn input_capture_feeds_cnt_to_ccr() {
        let (mut t, nvic, _) = tim2();
        // 输入捕获：CC1S=01（TI1 输入），CC1E + CC1IE 使能，上升沿捕获
        t.write(OFF_CCMR1, 4, 0b01).unwrap();
        t.write(OFF_CCER, 4, CCER_CC1E).unwrap();
        t.write(OFF_DIER, 4, DIER_CC1IE).unwrap();
        t.write(OFF_PSC, 4, 0).unwrap();
        t.write(OFF_ARR, 4, 1000).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();
        t.tick(7); // CNT=7
        assert_eq!(t.count(), 7);
        // 上升沿 → 锁存 CNT=7 → CCR1
        t.feed_edge(0, true);
        assert_eq!(t.read(OFF_CCR1, 4).unwrap(), 7, "输入捕获应锁存 CNT → CCR1");
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_CC1IF, SR_CC1IF);
        assert!(nvic.lock().unwrap().is_pending(28), "CC1IE 使能应置挂起捕获中断");
        // 下降沿（CC1P=0 上升沿有效）不触发
        t.write(OFF_SR, 4, 0).unwrap();
        t.tick(3); // CNT=10
        t.feed_edge(0, false);
        assert_eq!(t.read(OFF_CCR1, 4).unwrap(), 7, "非有效边沿不捕获");
    }

    #[test]
    fn advanced_rcr_delays_update_event() {
        let (mut t, nvic, _) = tim1();
        // RCR=1 → 更新事件每 2 次溢出一次
        t.write(OFF_RCR, 4, 1).unwrap();
        t.write(OFF_PSC, 4, 0).unwrap();
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_DIER, 4, DIER_UIE).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();
        // 11 步：1 次溢出（CNT 10→0），RCR 1→0，无更新事件
        t.tick(11);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, 0, "RCR=1 首次溢出不置 UIF");
        assert!(!nvic.lock().unwrap().is_pending(25), "首次溢出不挂起更新中断");
        // 再 11 步：第 2 次溢出 → 更新事件
        t.tick(11);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF, "第 2 次溢出置 UIF");
        assert!(nvic.lock().unwrap().is_pending(25));
    }

    #[test]
    fn advanced_break_clears_moe_sets_bif() {
        let (mut t, nvic, _) = tim1();
        // BDTR.MOE=1（主输出使能），BKE=1（刹车使能）
        t.write(OFF_BDTR, 4, BDTR_MOE | BDTR_BKE).unwrap();
        assert_eq!(t.read(OFF_BDTR, 4).unwrap() & BDTR_MOE, BDTR_MOE);
        // 软件刹车 EGR.BG → MOE 清 0 + BIF 置位 + 刹车中断挂起
        t.write(OFF_EGR, 4, EGR_BG).unwrap();
        assert_eq!(t.read(OFF_BDTR, 4).unwrap() & BDTR_MOE, 0, "刹车事件应清 MOE");
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_BIF, SR_BIF, "刹车事件应置 BIF");
        assert!(nvic.lock().unwrap().is_pending(24), "应挂起刹车中断 IRQ24");
        // AOE=1 时下一次更新事件恢复 MOE
        t.write(OFF_BDTR, 4, BDTR_AOE).unwrap();
        t.write(OFF_EGR, 4, EGR_UG).unwrap();
        assert_eq!(t.read(OFF_BDTR, 4).unwrap() & BDTR_MOE, BDTR_MOE, "AOE 时更新事件恢复 MOE");
    }

    #[test]
    fn advanced_dead_time_formula() {
        let (mut t, _, _) = tim1();
        t.write(OFF_BDTR, 4, 0x7F).unwrap();
        assert_eq!(t.dead_time(), 0x7F, "DTG<=0x7F：死区 = DTG");
        t.write(OFF_BDTR, 4, 0x80).unwrap();
        assert_eq!(t.dead_time(), (64 + 0) * 2, "DTG=0x80：64×2");
        t.write(OFF_BDTR, 4, 0xC0).unwrap();
        assert_eq!(t.dead_time(), (32 + 0) * 8, "DTG=0xC0：32×8");
        t.write(OFF_BDTR, 4, 0xE0).unwrap();
        assert_eq!(t.dead_time(), (32 + 0) * 16, "DTG=0xE0：32×16");
        // 非高级定时器无死区
        let (mut t2, _, _) = tim2();
        t2.write(OFF_BDTR, 4, 0x7F).unwrap();
        assert_eq!(t2.dead_time(), 0, "非高级定时器死区恒为 0");
    }

    #[test]
    fn advanced_complementary_dead_time_waveform() {
        let (mut t, _, bus) = tim1();
        // 收集 TimPwm 事件（ch0 主 OCx / ch4 互补 OCxN）
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::TimPwm { port: 1, channel, level } = ev {
                    g.lock().unwrap().push((*channel, *level));
                }
            })));
        // PWM 模式1（CNT<CCR1 时 OC1REF 高）+ 互补输出 CC1E|CC1NE + DTG=5 + MOE；
        // ARR=100、CCR1=8（测试窗口内无回绕）
        t.write(OFF_CCMR1, 4, 0b110 << 4).unwrap();
        t.write(OFF_CCER, 4, CCER_CC1E | CCER_CC1NE).unwrap();
        t.write(OFF_BDTR, 4, 5 | BDTR_MOE).unwrap();
        t.write(OFF_ARR, 4, 100).unwrap();
        t.write(OFF_CCR1, 4, 8).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();
        assert_eq!(t.dead_time(), 5, "DTG=5 → 死区 5 个内核时钟周期");

        // 手动推导时间线（每 tick(1) 推进 1 个 CNT，死区按内核时钟提前推进）：
        //   tick1  CNT=1  OC1REF↑ → OCx 变有效沿调度死区 5
        //   tick6  CNT=6  死区到 → OCx=1（OCx 上升沿延迟 DTG=5）
        //   tick8  CNT=8  OC1REF↓ → OCx=0 立即；OCxN 变有效沿调度死区 5
        //   tick13 CNT=13 死区到 → OCxN=1（互补上升沿延迟 DTG=5）
        //   tick14 CNT=14 OCx=0 / OCxN=1 → 反相有效区间
        // 死区窗口（CNT=8..12）：两路同为低，防半桥直通。
        for _ in 0..5 {
            t.tick(1);
        }
        assert_eq!((t.oc_ref(0), t.ocn_ref(0)), (false, false), "tick5：OCx 尚处死区延迟");
        t.tick(1); // CNT=6
        assert_eq!((t.oc_ref(0), t.ocn_ref(0)), (true, false), "tick6：OCx 死区后变高");
        t.tick(1); // CNT=7
        assert_eq!((t.oc_ref(0), t.ocn_ref(0)), (true, false), "tick7：OCx 保持高（CNT<CCR1）");
        t.tick(1); // CNT=8
        assert_eq!((t.oc_ref(0), t.ocn_ref(0)), (false, false), "tick8：OC1REF 变低，OCx 立即低、进入死区");
        for _ in 0..4 {
            t.tick(1); // CNT=9..12
        }
        assert_eq!((t.oc_ref(0), t.ocn_ref(0)), (false, false), "tick12：死区窗口内两路同为低（防直通）");
        t.tick(1); // CNT=13
        assert_eq!((t.oc_ref(0), t.ocn_ref(0)), (false, true), "tick13：OCxN 死区后变高");
        t.tick(1); // CNT=14
        assert_eq!((t.oc_ref(0), t.ocn_ref(0)), (false, true), "tick14：OCx 低 / OCxN 高（反相有效）");

        // 事件流：主通道 ch0 与互补通道 ch4 都发布高电平；且按序回放永不两路同高
        let evs = got.lock().unwrap();
        assert!(evs.iter().any(|(ch, l)| *ch == 0 && *l), "应发布主通道 OCx 高电平事件");
        assert!(evs.iter().any(|(ch, l)| *ch == 4 && *l), "应发布互补通道 OCxN 高电平事件");
        let mut oc = false;
        let mut ocn = false;
        for (ch, l) in evs.iter() {
            match ch {
                0 => oc = *l,
                4 => ocn = *l,
                _ => {}
            }
            assert!(!(oc && ocn), "事件回放中主/互补输出不得同时为高");
        }
    }

    #[test]
    fn channel_count_parameterized() {
        // TIM9 类（通用 2 通道）：channels()==2，ch0/ch1 均按 PWM 输出
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::TimPwm { port: 9, channel, .. } = ev {
                    g.lock().unwrap().push(*channel);
                }
            })));
        let cfg = TimerConfig {
            name: "TIM9",
            kind: TimerKind::General,
            bits: 16,
            channels: 2,
            irq: TimerIrq { brk: 24, up: 24, trig_com: 24, cc: 24 }, // 共享 TIM1_BRK 行
        };
        let mut t9 = Timer::new(9, cfg, bus, nvic.clone());
        assert_eq!(t9.channels(), 2, "TIM9 应为 2 通道");
        // CCMR1：ch0/ch1 均 PWM 模式1（OC1M=bit4:6、OC2M=bit12:14）；CCER：CC1E|CC2E
        t9.write(OFF_CCMR1, 4, (0b110 << 4) | (0b110 << 12)).unwrap();
        t9.write(OFF_CCER, 4, CCER_CC1E | (CCER_CC1E << 4)).unwrap();
        t9.write(OFF_ARR, 4, 100).unwrap();
        t9.write(OFF_CCR1, 4, 8).unwrap();
        t9.write(OFF_CCR2, 4, 50).unwrap();
        t9.write(OFF_CR1, 4, CR1_CEN).unwrap();
        for _ in 0..40 {
            t9.tick(1);
        }
        let chs = got.lock().unwrap();
        assert!(chs.contains(&0) && chs.contains(&1), "2 通道定时器应发布 ch0/ch1 事件（实际 {chs:?}）");
        drop(chs);

        // TIM10 类（通用 1 通道）：channels()==1，ch1 即使配置 PWM 也不输出
        let bus10 = Arc::new(Mutex::new(EventBus::new()));
        let got10 = Arc::new(Mutex::new(Vec::new()));
        let g10 = got10.clone();
        bus10
            .lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::TimPwm { port: 10, channel, .. } = ev {
                    g10.lock().unwrap().push(*channel);
                }
            })));
        let cfg = TimerConfig {
            name: "TIM10",
            kind: TimerKind::General,
            bits: 16,
            channels: 1,
            irq: TimerIrq { brk: 25, up: 25, trig_com: 25, cc: 25 }, // 共享 TIM1_UP 行
        };
        let mut t10 = Timer::new(10, cfg, bus10, nvic.clone());
        assert_eq!(t10.channels(), 1, "TIM10 应为 1 通道");
        t10.write(OFF_CCMR1, 4, (0b110 << 4) | (0b110 << 12)).unwrap(); // ch1 配置被忽略
        t10.write(OFF_CCER, 4, CCER_CC1E | (CCER_CC1E << 4)).unwrap();
        t10.write(OFF_ARR, 4, 100).unwrap();
        t10.write(OFF_CCR1, 4, 8).unwrap();
        t10.write(OFF_CCR2, 4, 50).unwrap();
        t10.write(OFF_CR1, 4, CR1_CEN).unwrap();
        for _ in 0..40 {
            t10.tick(1);
        }
        let chs10 = got10.lock().unwrap();
        assert!(chs10.contains(&0), "1 通道定时器应发布 ch0 事件（实际 {chs10:?}）");
        assert!(!chs10.contains(&1), "1 通道定时器不得发布 ch1 事件（实际 {chs10:?}）");
    }

    #[test]
    fn basic_timer_update_only() {
        // TIM6 基本定时器：无通道，仅更新事件
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let cfg = TimerConfig {
            name: "TIM6",
            kind: TimerKind::Basic,
            bits: 16,
            channels: 0,
            irq: TimerIrq { brk: 54, up: 54, trig_com: 54, cc: 54 },
        };
        let mut t = Timer::new(6, cfg, bus, nvic.clone());
        t.write(OFF_PSC, 4, 0).unwrap();
        t.write(OFF_ARR, 4, 9).unwrap();
        t.write(OFF_DIER, 4, DIER_UIE).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();
        t.tick(20); // 20 步，ARR=9 → 2 次溢出，CNT=20-2*10=0
        assert_eq!(t.count(), 0);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF);
        assert!(nvic.lock().unwrap().is_pending(54));
        // 基本定时器无通道：OCxREF 恒 false，feed_edge 无效
        assert!(!t.oc_ref(0));
    }
}
