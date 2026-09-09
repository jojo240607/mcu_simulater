//! USART 外设（STM32F407，M5 串口仿真模块，在 M3 TX 简化版之上补齐接收与中断）。
//!
//! M5 完整语义：
//! - TX：TE+UE 写 DR → 发布 [`crate::events::Event::UartByte`]（虚拟 Console 订阅），
//!   立即置 TXE/TC（仿真快速发送）；TXEIE/TCIE 可触发 NVIC 中断；
//! - RX：虚拟终端/测试发布 [`crate::events::Event::UartRx`] → [`Usart::feed_rx`]：
//!   UE+RE 时锁存 DR、置 RXNE（RXNE 已置位再来字节 → ORE）；RXNEIE 置挂起 NVIC 中断；
//!   读 DR 清 RXNE；ORE 写 0 清除（rc_w0）；
//! - 中断：RXNEIE/TCIE/TXEIE 置挂起，NVIC IRQ 映射 USART1-3 = IRQ37-39、UART4/5 = IRQ52/53、
//!   USART6 = IRQ71；CR1 中断使能位拉高而标志已置位时立即挂起（寄存器写副作用，硬件语义）。
//! - DMA：CR3.DMAT/DMAR 使能后，TXE 就绪（内存→外设）或 RXNE 置位（外设→内存）时发布
//!   [`crate::events::Event::UartDma`]，由 Machine 路由到对应 DMA 流；DMA 搬运经
//!   [`Usart::dma_read_dr`]/[`Usart::dma_write_dr`] 直接读写 DR。
//!
//! 地址映射（每个 USART 基址不同，`offset` 为相对基址偏移）：
//! - SR 0x00 / DR 0x04 / BRR 0x08 / CR1 0x0C / CR2 0x10 / CR3 0x14 / GTPR 0x18

use std::sync::{Arc, Mutex};

use std::collections::VecDeque;

use crate::events::{Event, EventBus};
use crate::peripheral::dma::DmaDir;
use crate::peripheral::nvic::Nvic;
use crate::peripheral::vperiph::uart::VirtualUartSlave;
use crate::peripheral::{BusError, Peripheral};

/// USART NVIC IRQ（STM32F407）
pub const USART1_IRQ: u32 = 37;
pub const USART2_IRQ: u32 = 38;
pub const USART3_IRQ: u32 = 39;
pub const UART4_IRQ: u32 = 52;
pub const UART5_IRQ: u32 = 53;
pub const USART6_IRQ: u32 = 71;

/// SR 状态位
const SR_TXE: u32 = 1 << 7;  // 发送数据寄存器空
const SR_TC: u32 = 1 << 6;   // 发送完成
const SR_RXNE: u32 = 1 << 5; // 接收数据寄存器非空
const SR_ORE: u32 = 1 << 3;  // 过载错误
const SR_IDLE: u32 = 1 << 4; // 空闲线路（帧结束标记；读 SR 后清）

/// CR1 控制位
const CR1_UE: u32 = 1 << 13;    // 使能
const CR1_RE: u32 = 1 << 2;     // 接收使能
const CR1_TE: u32 = 1 << 3;     // 发送使能
const CR1_RXNEIE: u32 = 1 << 5; // RXNE 中断使能
const CR1_IDLEIE: u32 = 1 << 4; // IDLE 中断使能（帧结束 → 固件 flush ring）
const CR1_TCIE: u32 = 1 << 6;   // TC 中断使能
const CR1_TXEIE: u32 = 1 << 7;  // TXE 中断使能

/// CR3 控制位
const CR3_DMAR: u32 = 1 << 6; // DMA 接收使能（RXNE → DMA 请求）
const CR3_DMAT: u32 = 1 << 7; // DMA 发送使能（TXE → DMA 请求）

/// 寄存器偏移
const OFF_SR: u32 = 0x00;
const OFF_DR: u32 = 0x04;
const OFF_CR1: u32 = 0x0C;
const OFF_CR3: u32 = 0x14;

/// USART 外设
pub struct Usart {
    /// USART 端口号（1/2/3/4/5/6），用于事件过滤
    pub port: u8,
    /// NVIC IRQ 编号（USART1-3=37-39、UART4/5=52/53、USART6=71）
    irq: u32,
    /// 寄存器文件（SR/DR/BRR/CR1/CR2/CR3/GTPR）
    regs: [u32; 7],
    /// CPU 读 DR 计数（POLL 消费观测）
    n_cpu_dr_reads: u64,
    /// 最近接收字节（读 DR 返回）
    rx_byte: u8,
    /// 事件总线（发布 UartByte）
    bus: Arc<Mutex<EventBus>>,
    /// NVIC（RXNE/TC/TXE → 置挂起）
    nvic: Arc<Mutex<Nvic>>,
    /// 虚拟 UART 从设备（推流：GPS NMEA / SBUS 遥控帧）
    slaves: Vec<Box<dyn VirtualUartSlave>>,
    /// 虚拟推流 RX FIFO：从设备字节排队，固件逐字节读走（不因 RXNE 未清而 ORE 丢弃）。
    /// 模拟硬件 USART 数据寄存器被 DMA/中断及时搬走的语义；POLL 读（uart_hal_read_dr）
    /// 与 DMA 读（PeriphToMem 读 DR）都从队首消费。
    rx_fifo: VecDeque<u8>,
}

impl Usart {
    pub fn new(
        port: u8,
        bus: Arc<Mutex<EventBus>>,
        nvic: Arc<Mutex<Nvic>>,
        irq: u32,
    ) -> Self {
        Self {
            port,
            irq,
            // 真机复位值：SR.TXE=1 / SR.TC=1（发送数据寄存器空）。Machine::reset 不
            // 遍历外设 reset，故初值即复位值——否则固件在 uart0 open 前（如
            // board_init 的设备 create 阶段 log error）调 uart_hal_putc 等 TXE
            // 会每次超时 10 万条指令，boot 显著变慢甚至超时。
            regs: [SR_TXE | SR_TC, 0, 0, 0, 0, 0, 0],
            rx_byte: 0,
            bus,
            nvic,
            slaves: Vec::new(),
            rx_fifo: VecDeque::new(),
            n_cpu_dr_reads: 0,
        }
    }

    /// 注册虚拟 UART 从设备（推流：GPS NMEA / SBUS）。
    pub fn register_slave(&mut self, slave: Box<dyn VirtualUartSlave>) {
        self.slaves.push(slave);
    }

    /// 从设备表（观测）。
    pub fn slaves(&self) -> &[Box<dyn VirtualUartSlave>] {
        &self.slaves
    }

    /// 推进所有 UART 从设备（`dt` 秒），收集推流字节（不喂入）。
    ///
    /// 返回本次推流的字节；由 Machine 直路由喂入（feed_rx + DMA 搬运），
    /// 不经事件总线（避免每字节全订阅者分发 + 锁开销）。
    pub fn collect_slave_bytes(&mut self, dt: f32) -> Vec<u8> {
        let mut out: Vec<u8> = Vec::new();
        if self.slaves.is_empty() {
            return out;
        }
        let mut buf: Vec<u8> = Vec::new();
        for sl in &mut self.slaves {
            buf.clear();
            sl.step(dt, &mut |b| buf.push(b));
            out.extend_from_slice(&buf);
        }
        out
    }

    /// 发送字节（发布 UartByte 事件；仅 TE+UE 生效）
    fn tx(&self, byte: u8) {
        let ev = Event::UartByte {
            port: self.port,
            byte,
        };
        self.bus.lock().unwrap().publish(&ev);
    }

    /// 帧结束通知（虚拟推流从设备推完一帧后由 Machine 调用）：
    /// 置 SR.IDLE + 按 IDLEIE 挂起中断——固件 IDLE ISR 读 SR/DR 清 IDLE 并
    /// flush 环形 DMA 缓冲（uart_idle_flush），read() 才有字节可读。
    pub fn notify_frame_end(&mut self) {
        self.regs[0] |= SR_IDLE;
        if self.regs[3] & CR1_IDLEIE != 0 {
            self.nvic.lock().unwrap().set_pending(self.irq);
        }
    }

    /// RXNE/ORE 置位后按 RXNEIE 挂起中断（RM：RXNE 中断事件含 ORE）
    fn set_pending_if_rx(&self) {
        if self.regs[3] & CR1_RXNEIE != 0 {
            self.nvic.lock().unwrap().set_pending(self.irq);
        }
    }

    /// 注入接收字节（虚拟终端/测试经 [`Event::UartRx`] 调用）。
    ///
    /// UE+RE 时：RXNE 已置位 → ORE（新字节丢弃）；否则锁存 DR + 置 RXNE。
    /// RXNEIE 使能 → 挂起 NVIC 中断。
    pub fn feed_rx(&mut self, byte: u8) {
        let cr1 = self.regs[3];
        if (cr1 & (CR1_UE | CR1_RE)) != (CR1_UE | CR1_RE) {
            return; // 未使能接收，字节丢弃
        }
        if self.regs[0] & SR_RXNE != 0 {
            self.regs[0] |= SR_ORE; // 过载：上次数据未读走
            self.set_pending_if_rx();
            return;
        }
        self.rx_byte = byte;
        self.regs[0] |= SR_RXNE;
        self.set_pending_if_rx();
        // 注意：不在 feed_rx 内发布 UartDma——feed_rx 可能在事件分发回调中被调用，
        // 此时事件总线锁已被外层 publish 持有，二次 publish 会同线程重入死锁；
        // RX DMA 请求改由 Machine 的 UartRx 订阅者在 feed_rx 之后直接路由
        // （见 [`Usart::dma_rx_pending`]）。
    }

    /// 虚拟推流注入接收字节：入队 + 置 RXNE（不因未读走而 ORE 丢弃）。
    ///
    /// 真实硬件上 USART DR 由 DMA/中断及时搬走，字节不会积压；虚拟从设备整帧
    /// 推流时模拟器无法模拟逐字节时序，故用 FIFO 缓冲，固件逐字节读走。
    /// 仅 UE+RE 使能时接收（与 [`Usart::feed_rx`] 相同门槛）。
    pub fn feed_rx_queued(&mut self, byte: u8) {
        let cr1 = self.regs[3];
        if (cr1 & (CR1_UE | CR1_RE)) != (CR1_UE | CR1_RE) {
            return;
        }
        self.rx_fifo.push_back(byte);
        self.regs[0] |= SR_RXNE;
        self.set_pending_if_rx();
    }

    /// 虚拟推流 FIFO 当前长度（观测/断言：固件是否消费了推流字节）。
    pub fn rx_fifo_len(&self) -> usize {
        self.rx_fifo.len()
    }

    /// 诊断：SR/CR1/CR3（故障排查用；观测接口）。
    pub fn dbg_state(&self) -> (u32, u32, u32) {
        (self.regs[0], self.regs[3], self.regs[5])
    }

    /// CPU 读 DR 次数（观测：POLL 读取是否在消费 FIFO）。
    pub fn n_cpu_dr_reads(&self) -> u64 {
        self.n_cpu_dr_reads
    }

    /// 是否有待 DMA 搬运的接收请求（CR3.DMAR 使能且 RXNE 置位）。
    ///
    /// 供 Machine 在 feed_rx 之后直接路由 RX DMA（避免在事件分发内二次 publish）。
    pub fn dma_rx_pending(&self) -> bool {
        (self.regs[5] & CR3_DMAR != 0) && (self.regs[0] & SR_RXNE != 0)
    }

    /// 发送数据寄存器是否空（固件轮询 TXE）
    pub fn tx_ready(&self) -> bool {
        self.regs[0] & SR_TXE != 0
    }

    /// DMA 读 DR（外设→内存方向）：返回接收字节并清 RXNE。
    ///
    /// 与 CPU 读 DR 同语义（读清 RXNE，虚拟推流 FIFO 优先），供 DMA 控制器搬运调用。
    pub fn dma_read_dr(&mut self) -> u32 {
        if let Some(b) = self.rx_fifo.pop_front() {
            self.rx_byte = b;
            if !self.rx_fifo.is_empty() {
                self.regs[0] |= SR_RXNE; // 还有字节：DMA 可继续请求
            } else {
                self.regs[0] &= !SR_RXNE;
            }
        } else {
            self.regs[0] &= !SR_RXNE;
        }
        self.rx_byte as u32
    }

    /// DMA 写 DR（内存→外设方向）：发送一字节并置 TXE/TC（仿真快速发送）。
    ///
    /// 供 DMA 控制器搬运调用，等价 CPU 写 DR 的发送语义。
    pub fn dma_write_dr(&mut self, value: u32) {
        self.tx(value as u8);
        self.regs[0] |= SR_TXE | SR_TC;
    }

    /// 检查并发布 DMA 请求（CR3.DMAT/DMAR 使能且对应标志置位时）。
    ///
    /// - 内存→外设（TX）：DMAT 且 TXE → 发布请求，DMA 一次搬完 NDTR；
    /// - 外设→内存（RX）：DMAR 且 RXNE → 发布请求，DMA 搬 1 字节。
    fn check_dma_request(&self) {
        let cr3 = self.regs[5]; // CR3
        let sr = self.regs[0];
        if (cr3 & CR3_DMAT != 0) && (sr & SR_TXE != 0) {
            self.bus.lock().unwrap().publish(&Event::UartDma {
                port: self.port,
                dir: DmaDir::MemToPeriph,
            });
        }
        if (cr3 & CR3_DMAR != 0) && (sr & SR_RXNE != 0) {
            self.bus.lock().unwrap().publish(&Event::UartDma {
                port: self.port,
                dir: DmaDir::PeriphToMem,
            });
        }
    }
}

/// DMA 外设方向搬运接口实现（复用 inherent `dma_read_dr`/`dma_write_dr` 语义）。
impl crate::peripheral::dma::DmaByteIo for Usart {
    fn dma_read_dr(&mut self) -> u32 {
        self.dma_read_dr()
    }

    fn dma_write_dr(&mut self, value: u32) {
        self.dma_write_dr(value);
    }
}

impl Peripheral for Usart {
    fn name(&self) -> &str {
        "USART"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_SR => {
                // 读 SR 返回状态字（含 IDLE）；IDLE 由真机序列"读 SR 再读 DR"清除
                // （OFF_DR 分支清）。不能在读 SR 时先清——否则固件
                // uart_hal_idle_pending() 永远读到 0，DMA+IDLE 的 IDLE 中断
                // 永远不触发 uart_idle_flush（flush 从未执行，GPS/SBUS 无字节）。
                Ok(self.regs[0])
            }
            OFF_DR => {
                // 读 DR 清 IDLE（读 SR 后读 DR 的标准清位序列）；返回接收字节。
                self.n_cpu_dr_reads += 1;
                self.regs[0] &= !SR_IDLE;
                if self.regs[5] & CR3_DMAR != 0 {
                    // DMA 模式：RX 数据由 DMA 搬走（dma_read_dr 弹 fifo），CPU 读 DR
                    // 仅用于清 IDLE/ORE 标志，不消费字节——否则固件 clear_idle/
                    // clear_errors 每次读 DR 都从 fifo 偷走一个 DMA 待搬字节，
                    // 每帧丢 1B → NMEA 帧永无完整。
                } else if let Some(b) = self.rx_fifo.pop_front() {
                    self.rx_byte = b;
                    if self.rx_fifo.is_empty() {
                        self.regs[0] &= !SR_RXNE;
                    }
                } else {
                    self.regs[0] &= !SR_RXNE;
                }
                Ok(self.rx_byte as u32)
            }
            0x08..=0x18 => Ok(self.regs[(offset / 4) as usize]),
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_SR => {
                // rc_w0：写 0 清除 TC/ORE；TXE/RXNE 只读（不受 SR 写影响）
                let clear = !value & (SR_TC | SR_ORE);
                self.regs[0] &= !clear;
                Ok(())
            }
            OFF_DR => {
                let cr1 = self.regs[3];
                if (cr1 & (CR1_UE | CR1_TE)) == (CR1_UE | CR1_TE) {
                    // 发送：发布事件 + 快速完成（TXE/TC 置位），TCIE/TXEIE 挂起
                    self.tx((value & 0xFF) as u8);
                    self.regs[0] |= SR_TXE | SR_TC;
                    let mut n = self.nvic.lock().unwrap();
                    if cr1 & CR1_TCIE != 0 {
                        n.set_pending(self.irq);
                    } else if cr1 & CR1_TXEIE != 0 {
                        n.set_pending(self.irq);
                    }
                } else {
                    // 未使能发送：数据丢弃，TXE 保持置位
                    self.regs[0] |= SR_TXE;
                }
                Ok(())
            }
            0x08..=0x18 => {
                self.regs[(offset / 4) as usize] = value;
                if offset == OFF_CR1 {
                    // TE+UE 使能上升沿 → TXE/TC 置位（首个字符轮询即可通过）
                    if (value & CR1_TE) != 0 && (value & CR1_UE) != 0 {
                        self.regs[0] |= SR_TXE | SR_TC;
                    }
                    // 中断使能位拉高而标志已置位 → 立即挂起
                    let sr = self.regs[0];
                    let want = (value & CR1_RXNEIE != 0 && sr & SR_RXNE != 0)
                        || (value & CR1_TCIE != 0 && sr & SR_TC != 0)
                        || (value & CR1_TXEIE != 0 && sr & SR_TXE != 0);
                    if want {
                        self.nvic.lock().unwrap().set_pending(self.irq);
                    }
                }
                // DMA 使能位（CR1 中断/CR3 DMAT/DMAR）变化后检查是否发布 DMA 请求
                if offset == OFF_CR1 || offset == OFF_CR3 {
                    self.check_dma_request();
                }
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.regs = [0; 7];
        // 真机复位值：SR.TXE=1 / SR.TC=1（发送数据寄存器空、上次发送完成）。
        // 若 TXE 复位为 0，固件在 uart0 open 前（如 board_init 的设备 create
        // 阶段 log error）调 uart_hal_putc 会每次超时 10 万条指令，boot 显著变慢。
        self.regs[0] = SR_TXE | SR_TC;
        self.rx_byte = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::nvic::Nvic;

    fn usart() -> (Usart, Arc<Mutex<EventBus>>, Arc<Mutex<Nvic>>) {
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let u = Usart::new(2, bus.clone(), nvic.clone(), USART2_IRQ);
        (u, bus, nvic)
    }

    #[test]
    fn tx_publishes_uart_byte_events() {
        let (mut u, bus, _) = usart();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::UartByte { .. } = ev {
                    g.lock().unwrap().push(ev.clone());
                }
            })));

        // 未使能时不发布
        u.write(OFF_DR, 4, b'H' as u32).unwrap();
        assert_eq!(got.lock().unwrap().len(), 0);

        // 使能 UE+TE 后发布
        u.write(0x0C, 4, CR1_UE | CR1_TE).unwrap();
        assert!(u.tx_ready(), "TE 使能后 TXE 应置位");
        u.write(OFF_DR, 4, b'H' as u32).unwrap();
        u.write(OFF_DR, 4, b'i' as u32).unwrap();
        assert!(u.tx_ready(), "发送后 TXE 应保持置位（快速发送）");

        let got = got.lock().unwrap();
        assert_eq!(got.len(), 2);
        assert_eq!(got[0], Event::UartByte { port: 2, byte: b'H' });
        assert_eq!(got[1], Event::UartByte { port: 2, byte: b'i' });
    }

    #[test]
    fn rx_feed_sets_rxne_and_pends_nvic() {
        let (mut u, _, nvic) = usart();

        // 未使能接收：注入丢弃
        u.feed_rx(b'X');
        assert_eq!(u.read(OFF_SR, 4).unwrap() & SR_RXNE, 0);

        // UE+RE 使能后注入
        u.write(OFF_CR1, 4, CR1_UE | CR1_RE | CR1_RXNEIE).unwrap();
        u.feed_rx(b'A');
        assert_ne!(u.read(OFF_SR, 4).unwrap() & SR_RXNE, 0, "RXNE 应置位");

        // RXNE 未清再注入 → ORE，新字节丢弃（DR 保留旧字节 'A'）
        u.feed_rx(b'B');
        assert_ne!(u.read(OFF_SR, 4).unwrap() & SR_ORE, 0, "过载应置位 ORE");
        assert_eq!(u.read(OFF_DR, 4).unwrap(), b'A' as u32, "过载时新字节丢弃，DR 保留旧字节");
        assert_eq!(u.read(OFF_SR, 4).unwrap() & SR_RXNE, 0, "读 DR 应清 RXNE");
        assert_ne!(u.read(OFF_SR, 4).unwrap() & SR_ORE, 0, "读 DR 不应清 ORE（写 0 才清）");

        // 写 0 清 ORE 后再注入 → 正常置 RXNE + 挂起 NVIC（IRQ38 = USART2）
        u.write(OFF_SR, 4, 0u32).unwrap();
        u.feed_rx(b'C');
        assert_eq!(u.read(OFF_DR, 4).unwrap(), b'C' as u32, "ORE 清除后新字节正常锁存");
        assert!(nvic.lock().unwrap().is_pending(USART2_IRQ), "RXNEIE 应挂起 USART2 IRQ");
    }

    #[test]
    fn sr_rc_w0_clears_ore_but_not_txe() {
        let (mut u, _, _) = usart();
        u.write(OFF_CR1, 4, CR1_UE | CR1_RE).unwrap();
        u.feed_rx(b'Z'); // RXNE 置位
        u.feed_rx(b'Y'); // ORE 置位
        let sr = u.read(OFF_SR, 4).unwrap();
        assert_ne!(sr & SR_ORE, 0);

        // 写 0 清 ORE（写 1 保留）
        u.write(OFF_SR, 4, !0u32).unwrap(); // 全 1：无清除
        assert_ne!(u.read(OFF_SR, 4).unwrap() & SR_ORE, 0, "写 1 不应清 ORE");
        u.write(OFF_SR, 4, 0u32).unwrap(); // 全 0：清 ORE（TXE 本就不置位）
        assert_eq!(u.read(OFF_SR, 4).unwrap() & SR_ORE, 0, "写 0 应清 ORE");

        // TXE 不受 SR 写影响
        u.write(OFF_CR1, 4, CR1_UE | CR1_TE).unwrap();
        assert_ne!(u.read(OFF_SR, 4).unwrap() & SR_TXE, 0);
        u.write(OFF_SR, 4, 0u32).unwrap();
        assert_ne!(u.read(OFF_SR, 4).unwrap() & SR_TXE, 0, "TXE 只读");
    }

    #[test]
    fn tx_tcie_pends_on_send() {
        let (mut u, _, nvic) = usart();
        u.write(OFF_CR1, 4, CR1_UE | CR1_TE | CR1_TCIE).unwrap();
        u.write(OFF_DR, 4, b'A' as u32).unwrap();
        assert!(nvic.lock().unwrap().is_pending(USART2_IRQ), "TCIE 使能时发送应挂起中断");
    }
}
