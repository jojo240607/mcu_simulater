//! ADC 外设（STM32F407，M5 虚拟外设生态；DMA 模式对齐 USART/I2C/SPI 外设↔内存搬运语义）。
//!
//! M5 语义（聚焦 DMA 模式的端到端链路，寄存器语义为可用的最小集合）：
//! - 转换：测试/虚拟传感器发布 [`crate::events::Event::AdcValue`] → [`Adc::feed_value`]：
//!   锁存采样值、置 SR.EOC（EOC 已置位 → 覆盖，简化不做 OVR 锁定）；读 DR 清 EOC；
//! - 中断：CR1.EOCIE 且 SR.EOC 置位 → 挂起 ADC IRQ（STM32F407：ADC1/2/3 共享 IRQ18）；
//! - DMA：CR2.DMA 使能且 EOC 置位时，Machine 的 AdcValue 订阅者在 feed_value 之后
//!   直接路由到对应 DMA 流（HAL 默认流，全在 DMA2）；DMA 搬运经 [`Adc::dma_read_dr`]
//!   读 DR（12 位采样值按半字取宽写内存）。
//!
//! 地址映射（ADC1 @ 0x40012000、ADC2 @ 0x40012100、ADC3 @ 0x40012200，`offset` 相对基址）：
//! - SR 0x00 / CR1 0x04 / CR2 0x08 / SMPR1 0x0C / SMPR2 0x10 / JOFR1-4 0x14-0x20 /
//!   HTR 0x24 / LTR 0x28 / SQR1 0x2C / SQR2 0x30 / SQR3 0x34 / JSQR 0x38 /
//!   JDR1-4 0x3C-0x48 / DR 0x4C

use std::sync::{Arc, Mutex};

use crate::events::EventBus;
use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};

/// ADC NVIC IRQ（STM32F407：ADC1/2/3 共享一个事件中断）
pub const ADC_IRQ: u32 = 18;

/// SR 状态位
const SR_EOC: u32 = 1 << 1; // 转换结束

/// CR1 控制位
const CR1_EOCIE: u32 = 1 << 5; // EOC 中断使能

/// CR2 控制位
const CR2_ADON: u32 = 1 << 0;  // ADC 使能
const CR2_DMA: u32 = 1 << 8;   // DMA 模式使能
const CR2_SWSTART: u32 = 1 << 30; // 软件启动常规转换（写 1 启动，硬件自清零）

/// 寄存器偏移
const OFF_SR: u32 = 0x00;
const OFF_CR1: u32 = 0x04;
const OFF_CR2: u32 = 0x08;
const OFF_DR: u32 = 0x4C;

/// 寄存器文件数（SR/CR1/CR2/SMPR1-2/JOFR1-4/HTR/LTR/SQR1-3/JSQR/JDR1-4/DR = 20）
const REG_COUNT: usize = 20;

/// ADC 外设
pub struct Adc {
    /// ADC 端口号（1/2/3），用于事件过滤
    pub port: u8,
    /// 事件中断 IRQ（ADC1/2/3 共享 18）
    irq: u32,
    /// 寄存器文件（SR..DR）
    regs: [u32; REG_COUNT],
    /// 最近采样值（读 DR / DMA 读返回，12 位）
    conversion_value: u16,
    /// 事件总线（保留：与其它外设同构，便于后续发布事件）
    _bus: Arc<Mutex<EventBus>>,
    /// NVIC（EOC → 挂起 ADC IRQ）
    nvic: Arc<Mutex<Nvic>>,
}

impl Adc {
    pub fn new(port: u8, bus: Arc<Mutex<EventBus>>, nvic: Arc<Mutex<Nvic>>, irq: u32) -> Self {
        Self {
            port,
            irq,
            regs: [0; REG_COUNT],
            conversion_value: 0,
            _bus: bus,
            nvic,
        }
    }

    /// EOCIE 且 EOC 置位时挂起 ADC 中断。
    fn set_pending_if_irq(&self) {
        if (self.regs[1] & CR1_EOCIE != 0) && (self.regs[0] & SR_EOC != 0) {
            self.nvic.lock().unwrap().set_pending(self.irq);
        }
    }

    /// 注入一次采样转换（测试/虚拟传感器经 [`crate::events::Event::AdcValue`] 调用）。
    ///
    /// CR2.ADON 时锁存采样值 + 置 EOC（EOC 已置位 → 覆盖，简化不做 OVR 锁定）。
    /// EOCIE 使能 → 挂起 ADC 中断。
    /// 注意：不在 feed_value 内发布 DMA 请求——feed_value 可能在事件分发回调中被调用，
    /// 此时事件总线锁已被外层 publish 持有，二次 publish 会同线程重入死锁；
    /// DMA 请求改由 Machine 的 AdcValue 订阅者在 feed_value 之后直接路由
    /// （见 [`Adc::dma_pending`]）。
    pub fn feed_value(&mut self, value: u16) {
        if self.regs[2] & CR2_ADON == 0 {
            return; // ADC 未使能，采样值丢弃
        }
        self.conversion_value = value;
        self.regs[0] |= SR_EOC;
        self.set_pending_if_irq();
    }

    /// 是否有待 DMA 搬运的转换结果（CR2.DMA 使能且 EOC 置位）。
    ///
    /// 供 Machine 在 feed_value 之后直接路由 DMA（避免在事件分发内二次 publish）。
    pub fn dma_pending(&self) -> bool {
        (self.regs[2] & CR2_DMA != 0) && (self.regs[0] & SR_EOC != 0)
    }

    /// DMA 读 DR（外设→内存方向）：返回 12 位采样值并清 EOC。
    ///
    /// 与 CPU 读 DR 同语义（读清 EOC），供 DMA 控制器搬运调用。
    pub fn dma_read_dr(&mut self) -> u32 {
        let value = self.conversion_value as u32;
        self.regs[0] &= !SR_EOC;
        value
    }

    /// DMA 写 DR（内存→外设方向）：ADC 只读，无操作。
    pub fn dma_write_dr(&mut self, _value: u32) {}
}

/// DMA 外设方向搬运接口实现（复用 inherent `dma_read_dr`/`dma_write_dr` 语义）。
impl crate::peripheral::dma::DmaByteIo for Adc {
    fn dma_read_dr(&mut self) -> u32 {
        self.dma_read_dr()
    }

    fn dma_write_dr(&mut self, value: u32) {
        self.dma_write_dr(value);
    }
}

impl Peripheral for Adc {
    fn name(&self) -> &str {
        "ADC"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_DR => {
                // 读 DR 返回采样值并清 EOC（读清，与串口读 DR 语义对齐）
                let v = self.conversion_value as u32;
                self.regs[0] &= !SR_EOC;
                Ok(v)
            }
            0x00..=0x4C => {
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
            OFF_SR => {
                // rc_w0：写 0 清除 EOC（AWD/STRT 等状态位只读，仿真只实现 EOC）
                if value & SR_EOC == 0 {
                    self.regs[0] &= !SR_EOC;
                }
                Ok(())
            }
            0x00..=0x4C => {
                if (offset / 4) as usize >= REG_COUNT {
                    return Err(BusError::OutOfRange);
                }
                let idx = (offset / 4) as usize;
                self.regs[idx] = value;
                if offset == OFF_CR1 || offset == OFF_CR2 {
                    // CR2.SWSTART 写 1：固件软件启动一次常规转换（如 jOS HAL 的
                    // "丢首次不稳定转换" 与每次 dev_read 单次转换）。
                    // 真机转换耗时数周期后置 EOC；仿真即时完成：
                    //   - SWSTART 为硬件自清零位，写入后立即清 0；
                    //   - ADON 使能时完成一次转换（置 SR.EOC，采样值取最近一次
                    //     锁存值；无外部注入时为 0/上次值），EOCIE 时挂起 IRQ。
                    if offset == OFF_CR2 && value & CR2_SWSTART != 0 {
                        self.regs[2] &= !CR2_SWSTART;
                        if self.regs[2] & CR2_ADON != 0 {
                            self.regs[0] |= SR_EOC;
                        }
                    }
                    self.set_pending_if_irq();
                }
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.regs = [0; REG_COUNT];
        self.conversion_value = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::events::EventBus;

    fn adc() -> (Adc, Arc<Mutex<EventBus>>, Arc<Mutex<Nvic>>) {
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let a = Adc::new(1, bus.clone(), nvic.clone(), ADC_IRQ);
        (a, bus, nvic)
    }

    #[test]
    fn feed_value_sets_eoc_and_dr() {
        let (mut a, _, _) = adc();
        // 未使能 ADON：注入值被丢弃（对齐 SPI SPE 门控语义）
        a.feed_value(0x123);
        assert_eq!(a.regs[0] & SR_EOC, 0, "ADON 未使能时不应锁存");
        // ADON 使能后注入 → EOC 置位 + 读 DR 返回采样值并清 EOC
        a.write(OFF_CR2, 4, CR2_ADON).unwrap();
        a.feed_value(0x123);
        assert_ne!(a.regs[0] & SR_EOC, 0, "feed_value 后 EOC 应置位");
        assert_eq!(a.dma_read_dr(), 0x123, "DMA 读 DR 返回采样值");
        assert_eq!(a.regs[0] & SR_EOC, 0, "读 DR 后 EOC 应清除");
    }

    #[test]
    fn dma_pending_requires_cr2_dma() {
        let (mut a, _, _) = adc();
        // ADON 使能但 CR2.DMA 未使能：feed_value 后 dma_pending 应为 false
        a.write(OFF_CR2, 4, CR2_ADON).unwrap();
        a.feed_value(0x111);
        assert!(!a.dma_pending(), "CR2.DMA 未使能时不应有 DMA 请求");
        // 使能 CR2.DMA + 注入新值 → dma_pending 为 true
        a.write(OFF_CR2, 4, CR2_ADON | CR2_DMA).unwrap();
        a.feed_value(0x222);
        assert!(a.dma_pending(), "CR2.DMA 使能且 EOC 置位时应有 DMA 请求");
    }

    #[test]
    fn eocie_sets_pending_irq() {
        let (mut a, _, nvic) = adc();
        // CR1.EOCIE + CR2.ADON，注入值 → EOC → 挂起 IRQ18
        a.write(OFF_CR1, 4, CR1_EOCIE).unwrap();
        a.write(OFF_CR2, 4, CR2_ADON).unwrap();
        a.feed_value(0x400);
        assert!(nvic.lock().unwrap().is_pending(ADC_IRQ), "EOCIE+EOC 应挂起 ADC 中断");
    }

    #[test]
    fn read_dr_clears_eoc() {
        let (mut a, _, _) = adc();
        a.write(OFF_CR2, 4, CR2_ADON).unwrap();
        a.feed_value(0x345);
        let v = a.read(OFF_DR, 4).unwrap();
        assert_eq!(v, 0x345, "CPU 读 DR 返回采样值");
        assert_eq!(a.regs[0] & SR_EOC, 0, "CPU 读 DR 后 EOC 应清除");
    }

    #[test]
    fn sr_write_zero_clears_eoc() {
        let (mut a, _, _) = adc();
        a.write(OFF_CR2, 4, CR2_ADON).unwrap();
        a.feed_value(0x100);
        assert_ne!(a.regs[0] & SR_EOC, 0);
        // rc_w0：写 0 清 EOC
        a.write(OFF_SR, 4, 0).unwrap();
        assert_eq!(a.regs[0] & SR_EOC, 0, "写 SR=0 应清 EOC");
    }
}
