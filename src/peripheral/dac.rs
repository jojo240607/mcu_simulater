//! DAC 外设（STM32F407，M7 虚拟外设生态）。
//!
//! M7 语义（聚焦软件/定时器触发 + DMA 内存→外设搬运，寄存器语义为最小可用集合）：
//! - 触发：SWTRIGR 软件触发（CR.TSEL=0b111）即时转换；定时器触发（TSEL=0..5）经
//!   [`Dac::timer_trigger`] 由 Machine 的 TimUpdate 订阅路由（软件与定时器两路）；
//! - 转换：触发后 DHR → DOR 锁存，DORx 寄存器回读反映转换值；
//! - 事件：转换完成发布 [`crate::events::Event::DacLevel`]（虚拟示波器/测试订阅）；
//!   触发转换且 CR.DMAEN 置位 → 发布 [`crate::events::Event::DacDma`] 请求
//!   内存→外设搬运（DMA 写 DHR12Rx → 再次转换）；
//! - DMA：内存→外设（TX）经 [`Dac::dma_write_dr`] 写 DHR 并转换（同 CPU 写 DHR
//!   后触发转换的语义）。
//!
//! 注意（事件死锁约束，与 ADC 同构）：
//! - 软件触发 / DMA 写路径在 MMIO 写或 run 间隙（未持有事件总线锁）执行，可安全发布；
//! - 定时器触发在 Machine 的 TimUpdate 订阅回调内执行（事件总线锁已被外层 publish
//!   持有），不能在此二次 publish——电平暂存 `pending_levels` 由 [`Dac::tick`] 冲刷
//!   发布，DMA 请求改由 Machine 的 TimUpdate 订阅者在 `timer_trigger` 之后直接路由
//!   （见 [`Dac::dma_requested`]）。
//!
//! 地址映射（DAC1 @ 0x40007400，`offset` 相对基址）：
//! CR 0x00 / SWTRIGR 0x04 / DHR12R1 0x08 / DHR12L1 0x0C / DHR8R1 0x10 /
//! DHR12R2 0x14 / DHR12L2 0x18 / DHR8R2 0x1C / DHR12RD 0x20 / DHR12LD 0x24 /
//! DHR8RD 0x28 / DOR1 0x2C / DOR2 0x30 / SR 0x34

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use crate::events::{Event, EventBus};
use crate::peripheral::dma::DmaDir;
use crate::peripheral::{BusError, Peripheral};

/// CR 控制位（通道 1）
const CR_EN1: u32 = 1 << 0;   // 通道 1 使能
const CR_TEN1: u32 = 1 << 2;  // 通道 1 触发使能
const CR_DMAEN1: u32 = 1 << 12; // 通道 1 DMA 请求使能
/// CR 控制位（通道 2）
const CR_EN2: u32 = 1 << 16;
const CR_TEN2: u32 = 1 << 18;
const CR_DMAEN2: u32 = 1 << 28;

/// SWTRIGR 触发位
const SWTRIG1: u32 = 1 << 0;
const SWTRIG2: u32 = 1 << 1;

/// 寄存器偏移
const OFF_CR: u32 = 0x00;
const OFF_SWTRIGR: u32 = 0x04;
const OFF_DHR12R1: u32 = 0x08;
const OFF_DHR12L1: u32 = 0x0C;
const OFF_DHR8R1: u32 = 0x10;
const OFF_DHR12R2: u32 = 0x14;
const OFF_DHR12L2: u32 = 0x18;
const OFF_DHR8R2: u32 = 0x1C;
const OFF_DHR12RD: u32 = 0x20;
const OFF_DHR12LD: u32 = 0x24;
const OFF_DHR8RD: u32 = 0x28;
const OFF_DOR1: u32 = 0x2C;
const OFF_DOR2: u32 = 0x30;
const OFF_SR: u32 = 0x34;

/// 寄存器文件数（CR..SR = 14）
const REG_COUNT: usize = 14;

/// 触发源选择 TSEL（有效选择位 = CR bit4:3 = 通道1、bit20:19 = 通道2）→ 定时器端口映射。
///
/// RM0090 中 DAC_CR.TEN1/TEN2（bit2/bit18）与 TSEL1[0]/TSEL2[0] 共享同一 bit，为规避该
/// 重叠，本模型取 TSEL 高 2 位作为有效触发源：
/// 0: TIM6 TRGO、1: TIM8 TRGO、2: TIM7 TRGO、3: TIM5 TRGO。
/// （TIM2/TIM4/EXTI9 触发源不在本模型覆盖范围；软件触发不依赖 TSEL，见 [`Dac::sw_trigger`]。）
fn tsel_to_timer(tsel: u32) -> Option<u8> {
    match tsel {
        0 => Some(6),
        1 => Some(8),
        2 => Some(7),
        3 => Some(5),
        _ => None,
    }
}

/// DAC 外设
pub struct Dac {
    /// DAC 端口号（STM32F407 仅 DAC1 = 1，用于事件过滤）
    pub port: u8,
    /// 寄存器文件（CR..SR）
    regs: [u32; REG_COUNT],
    /// 双通道数据保持寄存器（DHR 锁存值，12 位右对齐）
    dhr: [u16; 2],
    /// 双通道数据输出寄存器（DOR 锁存值，12 位右对齐）
    dor: [u16; 2],
    /// 当前 DMA 搬运通道（内存→外设写 DHR 时定位通道，触发转换时按 DMAEN 记录）
    dma_channel: u8,
    /// 定时器触发后待发布的电平（[channel, level]，由 [`Dac::tick`] 冲刷发布）
    pending_levels: Vec<(u8, u16)>,
    /// 事件总线（转换完成 → DacLevel / DMA 请求 → DacDma）
    bus: Arc<Mutex<EventBus>>,
    /// 活动标记（任一通道使能 CR.EN1/EN2）：Machine block hook 据此跳过未使能
    /// DAC 的加锁 tick（未使能时 pending_levels 必为空，无待冲刷发布）
    active: Arc<AtomicBool>,
}

impl Dac {
    /// 便捷构造（单元测试用）：活动标记为一次性占位，不与 Machine 联动
    pub fn new(port: u8, bus: Arc<Mutex<EventBus>>) -> Self {
        Self::with_active(port, bus, Arc::new(AtomicBool::new(false)))
    }

    /// 正式构造：`active` 由 Machine 持有（与 timers 列表并行），CR.ENx 置位时同步
    pub fn with_active(port: u8, bus: Arc<Mutex<EventBus>>, active: Arc<AtomicBool>) -> Self {
        Self {
            port,
            regs: [0; REG_COUNT],
            dhr: [0; 2],
            dor: [0; 2],
            dma_channel: 1,
            pending_levels: Vec::new(),
            bus,
            active,
        }
    }

    fn en_bit(ch: u8) -> u32 {
        if ch == 1 { CR_EN1 } else { CR_EN2 }
    }

    fn ten_bit(ch: u8) -> u32 {
        if ch == 1 { CR_TEN1 } else { CR_TEN2 }
    }

    fn dmaen_bit(ch: u8) -> u32 {
        if ch == 1 { CR_DMAEN1 } else { CR_DMAEN2 }
    }

    /// 通道使能（CR.ENx）
    fn enabled(&self, ch: u8) -> bool {
        self.regs[0] & Self::en_bit(ch) != 0
    }

    /// 通道触发使能（CR.TENx）
    fn ten(&self, ch: u8) -> bool {
        self.regs[0] & Self::ten_bit(ch) != 0
    }

    /// 通道触发源（CR.TSELx 高 2 位，bit4:3 / bit20:19）
    fn tsel(&self, ch: u8) -> u32 {
        let shift = if ch == 1 { 3 } else { 19 };
        (self.regs[0] >> shift) & 0x3
    }

    /// 锁存 DHR → DOR（转换核心）：更新 DOR 锁存值与 DORx 寄存器。
    fn latch(&mut self, ch: u8) -> u16 {
        let idx = ch as usize - 1;
        let level = self.dhr[idx];
        self.dor[idx] = level;
        self.regs[(OFF_DOR1 as usize / 4) + idx] = level as u32;
        level
    }

    /// 转换并发布输出电平（仅限软件触发 / DMA 写等未持有事件总线锁的路径）。
    ///
    /// `request_dma`：外部触发（软件/定时器）发起的转换且 CR.DMAENx 置位时，
    /// 发布 [`Event::DacDma`] 请求内存→外设搬运。
    fn convert(&mut self, ch: u8, request_dma: bool) {
        if !self.enabled(ch) {
            return; // 通道未使能，转换被忽略
        }
        let level = self.latch(ch);
        self.bus.lock().unwrap().publish(&Event::DacLevel {
            port: self.port,
            channel: ch,
            level,
        });
        if request_dma && self.regs[0] & Self::dmaen_bit(ch) != 0 {
            self.dma_channel = ch;
            self.bus.lock().unwrap().publish(&Event::DacDma {
                port: self.port,
                channel: ch,
                dir: DmaDir::MemToPeriph,
            });
        }
    }

    /// 软件触发（SWTRIGR 写 + TENx 使能）：即时转换并发布电平/DMA 请求。
    ///
    /// 对应 HAL 的 DAC_TRIGGER_SOFTWARE（仅置位 TEN1）写法——软件触发不依赖 TSEL，
    /// 只要通道触发使能，写 SWTRIGR 即转换。
    fn sw_trigger(&mut self, ch: u8) {
        if self.ten(ch) {
            self.convert(ch, true);
        }
    }

    /// 定时器触发（Machine 的 TimUpdate 订阅回调内调用，禁止发布事件）。
    ///
    /// TSEL 映射到 `timer_port` 的通道执行 DHR→DOR 锁存：电平暂存待 [`Dac::tick`]
    /// 冲刷发布；DMAENx 置位 → 记录当前 DMA 通道，由 Machine 按返回的锁存通道
    /// 位掩码（bit0=通道1、bit1=通道2）在返回后直接路由。
    pub fn timer_trigger(&mut self, timer_port: u8) -> u32 {
        let mut latched = 0u32;
        for ch in 1..=2u8 {
            if !self.enabled(ch) || !self.ten(ch) {
                continue;
            }
            if tsel_to_timer(self.tsel(ch)) != Some(timer_port) {
                continue;
            }
            let level = self.latch(ch);
            self.pending_levels.push((ch, level));
            if self.regs[0] & Self::dmaen_bit(ch) != 0 {
                self.dma_channel = ch;
            }
            latched |= 1 << (ch - 1);
        }
        latched
    }

    /// 通道是否使能了 DMA 请求（CR.DMAENx）。
    ///
    /// 供 Machine 的 TimUpdate 订阅者在 `timer_trigger` 之后对锁存通道直接路由
    /// （避免事件分发内二次 publish 死锁）。
    pub fn dma_requested(&self, ch: u8) -> bool {
        self.regs[0] & Self::dmaen_bit(ch) != 0
    }

    /// DMA 写 DHR（内存→外设方向）：写当前 DMA 通道的 DHR12Rx 并转换。
    ///
    /// 与 CPU 写 DHR 后触发转换同语义（发布电平；不再次请求 DMA，避免 run 间隙
    /// process 内二次发布死锁——DMA 搬运本就在进行中）。
    pub fn dma_write_dr(&mut self, value: u32) {
        let ch = self.dma_channel;
        let idx = ch as usize - 1;
        self.dhr[idx] = (value & 0xFFF) as u16;
        // DHR12R1@0x08 / DHR12R2@0x14 回读镜像
        let dhr_off = if ch == 1 { OFF_DHR12R1 } else { OFF_DHR12R2 };
        self.regs[(dhr_off / 4) as usize] = value & 0xFFF;
        self.convert(ch, false);
    }

    /// DMA 读 DR（内存→外设方向）：DAC 只输出，无读取语义。
    pub fn dma_read_dr(&mut self) -> u32 {
        0
    }

    /// 写 DHR 数据保持寄存器（不触发转换，转换由触发源驱动）。
    fn write_dhr(&mut self, offset: u32, value: u32) {
        // 各类 DHR 别名（右/左对齐、8 位、双通道）统一归一化为 12 位右对齐锁存值。
        let (ch1, ch2) = match offset {
            OFF_DHR12R1 => (Some(value & 0xFFF), None),
            OFF_DHR12L1 => (Some((value >> 4) & 0xFFF), None),
            OFF_DHR8R1 => (Some(value & 0xFF), None),
            OFF_DHR12R2 => (None, Some(value & 0xFFF)),
            OFF_DHR12L2 => (None, Some((value >> 4) & 0xFFF)),
            OFF_DHR8R2 => (None, Some(value & 0xFF)),
            OFF_DHR12RD => (Some(value & 0xFFF), Some((value >> 16) & 0xFFF)),
            OFF_DHR12LD => (Some((value >> 4) & 0xFFF), Some((value >> 20) & 0xFFF)),
            OFF_DHR8RD => (Some(value & 0xFF), Some((value >> 8) & 0xFF)),
            _ => (None, None),
        };
        if let Some(v) = ch1 {
            self.dhr[0] = v as u16;
        }
        if let Some(v) = ch2 {
            self.dhr[1] = v as u16;
        }
        // 寄存器文件回读镜像：原样存写值（硬件 DHR 可读回上次写入值）
        self.regs[(offset / 4) as usize] = value;
    }
}

/// DMA 外设方向搬运接口实现（复用 inherent `dma_write_dr`/`dma_read_dr` 语义）。
impl crate::peripheral::dma::DmaByteIo for Dac {
    fn dma_read_dr(&mut self) -> u32 {
        self.dma_read_dr()
    }

    fn dma_write_dr(&mut self, value: u32) {
        self.dma_write_dr(value);
    }
}

impl Peripheral for Dac {
    fn name(&self) -> &str {
        "DAC"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_DOR1 | OFF_DOR2 => Ok(self.regs[(offset / 4) as usize]),
            OFF_SR => Ok(0), // SR 全 0（校准/欠载状态，仿真不置位）
            0x00..=OFF_SR => {
                if ((offset / 4) as usize) < REG_COUNT {
                    Ok(self.regs[(offset / 4) as usize])
                } else {
                    Err(BusError::OutOfRange)
                }
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_SWTRIGR => {
                // 写触发位（硬件自清零，不存寄存器文件）：TSEL=7 且 TEN 时即时转换
                if value & SWTRIG1 != 0 {
                    self.sw_trigger(1);
                }
                if value & SWTRIG2 != 0 {
                    self.sw_trigger(2);
                }
                Ok(())
            }
            OFF_DHR12R1 | OFF_DHR12L1 | OFF_DHR8R1 | OFF_DHR12R2 | OFF_DHR12L2
            | OFF_DHR8R2 | OFF_DHR12RD | OFF_DHR12LD | OFF_DHR8RD => {
                self.write_dhr(offset, value);
                Ok(())
            }
            OFF_CR => {
                self.regs[0] = value;
                // 同步活动标记：任一通道使能（EN1/EN2）即需周期 tick 冲刷电平
                self.active
                    .store(value & (CR_EN1 | CR_EN2) != 0, Ordering::Relaxed);
                Ok(())
            }
            // DOR/SR 只读，写忽略
            OFF_DOR1 | OFF_DOR2 | OFF_SR => Ok(()),
            0x00..=OFF_SR => {
                if (offset / 4) as usize >= REG_COUNT {
                    return Err(BusError::OutOfRange);
                }
                self.regs[(offset / 4) as usize] = value;
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.regs = [0; REG_COUNT];
        self.dhr = [0; 2];
        self.dor = [0; 2];
        self.dma_channel = 1;
        self.pending_levels.clear();
        // 复位清除 EN → 活动标记同步为未激活
        self.active.store(false, Ordering::Relaxed);
    }

    /// 周期推进：冲刷定时器触发暂存的电平事件（定时器路径在 TimUpdate 订阅回调内
    /// 不能发布事件，见结构体文档）。DAC 本身无计数语义，仅承担发布延迟。
    fn tick(&mut self, _cycles: u64) {
        for (ch, level) in self.pending_levels.drain(..) {
            self.bus.lock().unwrap().publish(&Event::DacLevel {
                port: self.port,
                channel: ch,
                level,
            });
        }
    }
}
