//! SPI 外设（STM32F407，M5 虚拟外设生态；DMA 模式对齐 USART/I2C 外设↔内存搬运语义）。
//!
//! M5 语义（聚焦 DMA 模式的端到端链路，寄存器语义为可用的最小集合）：
//! - TX：CR1.SPE 使能上升沿置 TXE（仿真简化——真实 SPI 的 TXE 需时钟使能/主模式就绪，
//!   此处对齐 USART/I2C "使能即就绪" 语义，便于 DMA 验收聚焦搬运链路）；
//!   CPU 写 DR → 发布 [`crate::events::Event::SpiByte`]（虚拟从机/测试订阅）+ 置 TXE；
//! - RX：测试/虚拟从机发布 [`crate::events::Event::SpiRx`] → [`Spi::feed_rx`]：
//!   SPE 时锁存 DR、置 RXNE（RXNE 已置位 → OVR 覆盖，简化不做锁定）；读 DR 清 RXNE；
//! - 中断：CR2.RXNEIE 且 RXNE 置位，或 CR2.TXEIE 且 TXE 置位 → 挂起 SPI IRQ
//!   （SPI1=35、SPI2=36、SPI3=51；单一 IRQ，区别于 I2C 的 ITBUFEN 双位语义）；
//! - DMA：CR2.TXDMAEN/RXDMAEN 使能后，TXE 就绪（内存→外设）或 RxNE 置位（外设→内存）
//!   时发布 [`crate::events::Event::SpiDma`]，由 Machine 路由到对应 DMA 流（HAL 默认流）；
//!   DMA 搬运经 [`Spi::dma_read_dr`]/[`Spi::dma_write_dr`] 直接读写 DR。
//!
//! 地址映射（SPI1 @ 0x40013000、SPI2 @ 0x40003800、SPI3 @ 0x40003C00，`offset` 相对基址）：
//! - CR1 0x00 / CR2 0x04 / SR 0x08 / DR 0x0C / CRCPR 0x10 / RXCRCR 0x14 /
//!   TXCRCR 0x18 / I2SCFGR 0x1C / I2SPR 0x20

use std::sync::{Arc, Mutex};

use crate::events::{Event, EventBus};
use crate::peripheral::dma::DmaDir;
use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};

/// SPI NVIC IRQ（STM32F407：SPI1/SPI2/SPI3 各一个事件中断）
pub const SPI1_IRQ: u32 = 35;
pub const SPI2_IRQ: u32 = 36;
pub const SPI3_IRQ: u32 = 51;

/// SR 状态位
const SR_RXNE: u32 = 1 << 0; // 接收数据寄存器非空
const SR_TXE: u32 = 1 << 1;  // 发送数据寄存器空
const SR_UDR: u32 = 1 << 3;  // 欠载错误（I2S）
const SR_CRCERR: u32 = 1 << 4; // CRC 错误
const SR_MODF: u32 = 1 << 5; // 模式错误
const SR_OVR: u32 = 1 << 6;  // 过载错误

/// CR1 控制位
const CR1_SPE: u32 = 1 << 6; // 外设使能

/// CR2 控制位
const CR2_RXDMAEN: u32 = 1 << 0; // RX 缓冲 DMA 使能
const CR2_TXDMAEN: u32 = 1 << 1; // TX 缓冲 DMA 使能
const CR2_RXNEIE: u32 = 1 << 6;  // RXNE 中断使能
const CR2_TXEIE: u32 = 1 << 7;   // TXE 中断使能

/// 寄存器偏移
const OFF_CR1: u32 = 0x00;
const OFF_CR2: u32 = 0x04;
const OFF_SR: u32 = 0x08;
const OFF_DR: u32 = 0x0C;

/// 寄存器文件数（CR1/CR2/SR/DR/CRCPR/RXCRCR/TXCRCR/I2SCFGR/I2SPR）
const REG_COUNT: usize = 9;

/// SPI 外设
pub struct Spi {
    /// SPI 端口号（1/2/3），用于事件过滤
    pub port: u8,
    /// 事件中断 IRQ（SPI1=35/SPI2=36/SPI3=51）
    irq: u32,
    /// 寄存器文件（CR1/CR2/SR/DR/CRCPR/RXCRCR/TXCRCR/I2SCFGR/I2SPR）
    regs: [u32; REG_COUNT],
    /// 最近接收字节（读 DR 返回）
    rx_byte: u8,
    /// 事件总线（发布 SpiByte）
    bus: Arc<Mutex<EventBus>>,
    /// NVIC（RXNE/TXE → 挂起 SPI IRQ）
    nvic: Arc<Mutex<Nvic>>,
}

impl Spi {
    pub fn new(port: u8, bus: Arc<Mutex<EventBus>>, nvic: Arc<Mutex<Nvic>>, irq: u32) -> Self {
        Self {
            port,
            irq,
            regs: [0; REG_COUNT],
            rx_byte: 0,
            bus,
            nvic,
        }
    }

    /// 发送字节（发布 SpiByte 事件；仅 SPE 生效）
    fn tx(&self, byte: u8) {
        let ev = Event::SpiByte {
            port: self.port,
            byte,
        };
        self.bus.lock().unwrap().publish(&ev);
    }

    /// RXNEIE 且 RXNE 置位，或 TXEIE 且 TXE 置位时挂起 SPI 中断。
    fn set_pending_if_irq(&self) {
        let cr2 = self.regs[1];
        let sr = self.regs[2];
        if (cr2 & CR2_RXNEIE != 0 && sr & SR_RXNE != 0)
            || (cr2 & CR2_TXEIE != 0 && sr & SR_TXE != 0)
        {
            self.nvic.lock().unwrap().set_pending(self.irq);
        }
    }

    /// 注入接收字节（测试/虚拟从机经 [`Event::SpiRx`] 调用）。
    ///
    /// SPE 时锁存 DR + 置 RXNE（RXNE 已置位 → OVR 置位并覆盖，简化不做锁定）。
    /// RXNEIE 使能 → 挂起 SPI 中断。
    pub fn feed_rx(&mut self, byte: u8) {
        if self.regs[0] & CR1_SPE == 0 {
            return; // 未使能，字节丢弃
        }
        if self.regs[2] & SR_RXNE != 0 {
            self.regs[2] |= SR_OVR; // 上次数据未读走 → 过载（仿真简化：覆盖）
        }
        self.rx_byte = byte;
        self.regs[2] |= SR_RXNE;
        self.set_pending_if_irq();
        // 注意：不在 feed_rx 内发布 SpiDma——feed_rx 可能在事件分发回调中被调用，
        // 此时事件总线锁已被外层 publish 持有，二次 publish 会同线程重入死锁；
        // RX DMA 请求改由 Machine 的 SpiRx 订阅者在 feed_rx 之后直接路由
        // （见 [`Spi::dma_rx_pending`]）。
    }

    /// 是否有待 DMA 搬运的接收请求（CR2.RXDMAEN 使能且 RXNE 置位）。
    ///
    /// 供 Machine 在 feed_rx 之后直接路由 RX DMA（避免在事件分发内二次 publish）。
    pub fn dma_rx_pending(&self) -> bool {
        (self.regs[1] & CR2_RXDMAEN != 0) && (self.regs[2] & SR_RXNE != 0)
    }

    /// 发送数据寄存器是否空（固件轮询 TXE）
    pub fn tx_ready(&self) -> bool {
        self.regs[2] & SR_TXE != 0
    }

    /// DMA 读 DR（外设→内存方向）：返回接收字节并清 RXNE。
    ///
    /// 与 CPU 读 DR 同语义（读清 RXNE），供 DMA 控制器搬运调用。
    pub fn dma_read_dr(&mut self) -> u8 {
        let byte = self.rx_byte;
        self.regs[2] &= !SR_RXNE;
        byte
    }

    /// DMA 写 DR（内存→外设方向）：发送一字节并置 TXE。
    ///
    /// 供 DMA 控制器搬运调用，等价 CPU 写 DR 的发送语义。
    pub fn dma_write_dr(&mut self, byte: u8) {
        self.tx(byte);
        self.regs[2] |= SR_TXE;
    }

    /// 检查并发布 DMA 请求（CR2.TXDMAEN/RXDMAEN 使能且对应标志置位时）。
    ///
    /// - 内存→外设（TX）：TXDMAEN 且 TXE → 发布请求，DMA 一次搬完 NDTR；
    /// - 外设→内存（RX）：RXDMAEN 且 RXNE → 发布请求，DMA 搬 1 字节。
    fn check_dma_request(&self) {
        let cr2 = self.regs[1];
        let sr = self.regs[2];
        if (cr2 & CR2_TXDMAEN != 0) && (sr & SR_TXE != 0) {
            self.bus.lock().unwrap().publish(&Event::SpiDma {
                port: self.port,
                dir: DmaDir::MemToPeriph,
            });
        }
        if (cr2 & CR2_RXDMAEN != 0) && (sr & SR_RXNE != 0) {
            self.bus.lock().unwrap().publish(&Event::SpiDma {
                port: self.port,
                dir: DmaDir::PeriphToMem,
            });
        }
    }
}

/// DMA 外设方向搬运接口实现（复用 inherent `dma_read_dr`/`dma_write_dr` 语义）。
impl crate::peripheral::dma::DmaByteIo for Spi {
    fn dma_read_dr(&mut self) -> u8 {
        self.dma_read_dr()
    }

    fn dma_write_dr(&mut self, byte: u8) {
        self.dma_write_dr(byte);
    }
}

impl Peripheral for Spi {
    fn name(&self) -> &str {
        "SPI"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_DR => {
                // 读 DR 返回接收字节并清 RXNE
                let v = self.rx_byte as u32;
                self.regs[2] &= !SR_RXNE;
                Ok(v)
            }
            0x00..=0x20 => {
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
                // rc_w0：写 0 清除错误位（UDR/CRCERR/MODF/OVR）；TXE/RXNE 只读
                let clear = !value & (SR_UDR | SR_CRCERR | SR_MODF | SR_OVR);
                self.regs[2] &= !clear;
                Ok(())
            }
            OFF_DR => {
                if self.regs[0] & CR1_SPE != 0 {
                    // 发送：发布事件 + 置 TXE（仿真快速发送）
                    self.tx((value & 0xFF) as u8);
                    self.regs[2] |= SR_TXE;
                    self.set_pending_if_irq();
                } else {
                    // 未使能发送：数据丢弃
                    self.regs[2] |= SR_TXE;
                }
                Ok(())
            }
            0x00..=0x20 => {
                if (offset / 4) as usize >= REG_COUNT {
                    return Err(BusError::OutOfRange);
                }
                let idx = (offset / 4) as usize;
                self.regs[idx] = value;
                if offset == OFF_CR1 {
                    // SPE 使能上升沿 → TXE 置位（仿真简化，见模块注释）
                    if value & CR1_SPE != 0 {
                        self.regs[2] |= SR_TXE;
                    } else {
                        self.regs[2] &= !SR_TXE;
                    }
                    self.set_pending_if_irq();
                }
                // DMA 使能位（CR2.TXDMAEN/RXDMAEN）/中断使能位变化后检查发布请求/挂起
                if offset == OFF_CR2 {
                    self.set_pending_if_irq();
                    self.check_dma_request();
                }
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.regs = [0; REG_COUNT];
        self.rx_byte = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spi() -> (Spi, Arc<Mutex<EventBus>>, Arc<Mutex<Nvic>>) {
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let s = Spi::new(1, bus.clone(), nvic.clone(), SPI1_IRQ);
        (s, bus, nvic)
    }

    #[test]
    fn tx_publishes_spi_byte_events() {
        let (mut s, bus, _) = spi();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::SpiByte { .. } = ev {
                    g.lock().unwrap().push(ev.clone());
                }
            })));

        // 未使能时不发布
        s.write(OFF_DR, 4, 0x41).unwrap();
        assert_eq!(got.lock().unwrap().len(), 0);

        // SPE 使能后置 TXE（仿真简化）
        s.write(OFF_CR1, 4, CR1_SPE).unwrap();
        assert!(s.tx_ready(), "SPE 使能后 TXE 应置位");
        s.write(OFF_DR, 4, 0x41).unwrap();
        s.write(OFF_DR, 4, 0x42).unwrap();

        let got = got.lock().unwrap();
        assert_eq!(got.len(), 2);
        assert_eq!(got[0], Event::SpiByte { port: 1, byte: 0x41 });
        assert_eq!(got[1], Event::SpiByte { port: 1, byte: 0x42 });
    }

    #[test]
    fn rx_feed_sets_rxne_and_pends_irq() {
        let (mut s, _, nvic) = spi();

        // 未使能接收：注入丢弃
        s.feed_rx(0x11);
        assert_eq!(s.read(OFF_SR, 4).unwrap() & SR_RXNE, 0);

        // SPE 使能 + RXNEIE 后注入
        s.write(OFF_CR1, 4, CR1_SPE).unwrap();
        s.write(OFF_CR2, 4, CR2_RXNEIE).unwrap();
        s.feed_rx(0xAA);
        assert_ne!(s.read(OFF_SR, 4).unwrap() & SR_RXNE, 0, "RXNE 应置位");
        assert!(nvic.lock().unwrap().is_pending(SPI1_IRQ), "RXNEIE 应挂起 SPI1 IRQ");

        // 读 DR 清 RXNE 并返回锁存字节
        assert_eq!(s.read(OFF_DR, 4).unwrap(), 0xAA);
        assert_eq!(s.read(OFF_SR, 4).unwrap() & SR_RXNE, 0, "读 DR 应清 RXNE");
    }

    #[test]
    fn tx_when_txeie_pends_irq() {
        let (mut s, _, nvic) = spi();
        s.write(OFF_CR1, 4, CR1_SPE).unwrap();
        // TXEIE 使能：SPE 使能后 TXE 置位 → 写 CR2 时应挂起 SPI IRQ
        s.write(OFF_CR2, 4, CR2_TXEIE).unwrap();
        assert!(nvic.lock().unwrap().is_pending(SPI1_IRQ), "TXEIE 且 TXE 置位应挂起 SPI1 IRQ");
    }

    #[test]
    fn sr_rc_w0_clears_ovr_but_not_txe() {
        let (mut s, _, _) = spi();
        s.write(OFF_CR1, 4, CR1_SPE).unwrap();
        s.feed_rx(0x01); // RXNE 置位
        s.feed_rx(0x02); // OVR 置位（RXNE 未清）
        assert_ne!(s.read(OFF_SR, 4).unwrap() & SR_OVR, 0);

        // 写 0 清 OVR（写 1 保留）
        s.write(OFF_SR, 4, !0u32).unwrap();
        assert_ne!(s.read(OFF_SR, 4).unwrap() & SR_OVR, 0, "写 1 不应清 OVR");
        s.write(OFF_SR, 4, 0u32).unwrap();
        assert_eq!(s.read(OFF_SR, 4).unwrap() & SR_OVR, 0, "写 0 应清 OVR");

        // TXE 不受 SR 写影响
        assert_ne!(s.read(OFF_SR, 4).unwrap() & SR_TXE, 0);
        s.write(OFF_SR, 4, 0u32).unwrap();
        assert_ne!(s.read(OFF_SR, 4).unwrap() & SR_TXE, 0, "TXE 只读");
    }

    #[test]
    fn txdmaen_write_publishes_tx_dma_request() {
        let (mut s, bus, _) = spi();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::SpiDma { .. } = ev {
                    g.lock().unwrap().push(ev.clone());
                }
            })));

        // SPE 使能后 TXE 已置位；写 CR2.TXDMAEN → 立即发布 TX DMA 请求
        s.write(OFF_CR1, 4, CR1_SPE).unwrap();
        s.write(OFF_CR2, 4, CR2_TXDMAEN).unwrap();

        let got = got.lock().unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(
            got[0],
            Event::SpiDma {
                port: 1,
                dir: DmaDir::MemToPeriph,
            },
            "TXDMAEN 写且 TXE 置位应发布 TX DMA 请求"
        );
    }

    #[test]
    fn dma_read_write_dr_roundtrip() {
        let (mut s, bus, _) = spi();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::SpiByte { .. } = ev {
                    g.lock().unwrap().push(ev.clone());
                }
            })));

        s.write(OFF_CR1, 4, CR1_SPE).unwrap();
        s.feed_rx(0x55);
        assert!(!s.dma_rx_pending(), "RXDMAEN 未使能时 dma_rx_pending 应为假");
        s.write(OFF_CR2, 4, CR2_RXDMAEN).unwrap();
        assert!(s.dma_rx_pending(), "RXDMAEN+RXNE → RX DMA 待搬运");
        assert_eq!(s.dma_read_dr(), 0x55, "DMA 读 DR 返回锁存字节");
        assert!(!s.dma_rx_pending(), "DMA 读 DR 后 RXNE 清，请求解除");

        s.dma_write_dr(0x77);
        assert_eq!(got.lock().unwrap().len(), 1, "DMA 写 DR 应发布发送事件");
    }
}
