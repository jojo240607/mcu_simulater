//! I2C 外设（STM32F407，M5 虚拟外设生态；DMA 模式对齐 USART 外设↔内存搬运语义）。
//!
//! M5 语义（聚焦 DMA 模式的端到端链路，寄存器语义为可用的最小集合）：
//! - TX：CR1.PE 使能上升沿置 TxE（仿真简化——真实 I2C 的 TxE 需地址阶段后才置位，
//!   此处对齐 USART "使能即就绪" 语义，便于 DMA 验收聚焦搬运链路）；
//!   CPU 写 DR → 发布 [`crate::events::Event::I2cByte`]（虚拟从机/测试订阅）+ 置 TxE；
//! - RX：测试/虚拟从机发布 [`crate::events::Event::I2cRx`] → [`I2c::feed_rx`]：
//!   PE 时锁存 DR、置 RxNE（RxNE 已置位 → OVR 覆盖，简化不做 OVR 语义）；
//!   读 DR 清 RxNE；
//! - 中断：CR2.ITBUFEN+ITEVTEN 且 TxE/RxNE 置位 → 挂起 EV IRQ
//!   （I2C1_EV=31、I2C2_EV=33、I2C3_EV=72）；ITERREN → ER IRQ（暂未用）；
//! - DMA：CR2.DMAEN 使能后，TxE 就绪（内存→外设）或 RxNE 置位（外设→内存）时发布
//!   [`crate::events::Event::I2cDma`]，由 Machine 路由到对应 DMA 流（HAL 默认流）；
//!   DMA 搬运经 [`I2c::dma_read_dr`]/[`I2c::dma_write_dr`] 直接读写 DR。
//!
//! 地址映射（I2C1 @ 0x40005400、I2C2 @ 0x40005800、I2C3 @ 0x40005C00，`offset` 相对基址）：
//! - CR1 0x00 / CR2 0x04 / OAR1 0x08 / OAR2 0x0C / DR 0x10 / SR1 0x14 / SR2 0x18 /
//!   CCR 0x1C / TRISE 0x20
//!
//! 事务级扩展（虚拟外设总线协议级模拟）：CPU 写 DR 路径按真实 master 事务解析——
//! 写 CR1.START → SB 置位 + 进入地址阶段；写 DR（地址阶段）= 地址字节
//! （addr7=R/W）→ 匹配 [`VirtualI2cSlave`]：命中置 ADDR（读 SR2 清）、未命中置 AF；
//! 数据阶段写 → 直路由从设备 `on_write`；读方向（地址 bit0=1）读 SR2 清 ADDR 时
//! 预取首字节置 RXNE，读 DR 返回并预取下一字节（连续流），供固件 POLL 驱动
//! （joc-base stm32/i2c_hal.c 轮询 SB/ADDR/TXE/RXNE/BTF）直接跑通。
//! DMA 路径（`dma_write_dr`/`dma_read_dr`）保持原简化语义（向后兼容既有 DMA 测试）。

use std::sync::{Arc, Mutex};

use crate::events::{Event, EventBus};
use crate::peripheral::dma::DmaDir;
use crate::peripheral::nvic::Nvic;
use crate::peripheral::vperiph::{I2cDir, VirtualI2cSlave};
use crate::peripheral::{BusError, Peripheral};

/// I2C NVIC IRQ（STM32F407：EV = 事件中断、ER = 错误中断）
pub const I2C1_EV_IRQ: u32 = 31;
pub const I2C2_EV_IRQ: u32 = 33;
pub const I2C3_EV_IRQ: u32 = 72;

/// SR1 状态位
const SR1_SB: u32 = 1 << 0;    // 起始位（START 已发送，等待地址）
const SR1_ADDR: u32 = 1 << 1;  // 地址已发送/匹配（读 SR2 清除）
const SR1_BTF: u32 = 1 << 2;   // 字节传输完成
const SR1_TXE: u32 = 1 << 7;  // 发送数据寄存器空
const SR1_RXNE: u32 = 1 << 6; // 接收数据寄存器非空
const SR1_BERR: u32 = 1 << 8;  // 总线错误
const SR1_ARLO: u32 = 1 << 9;  // 仲裁丢失
const SR1_AF: u32 = 1 << 10;   // 应答失败
const SR1_OVR: u32 = 1 << 11;  // 过载/欠载错误

/// CR1 控制位
const CR1_PE: u32 = 1 << 0;    // 外设使能
const CR1_ACK: u32 = 1 << 2;   // 应答使能
const CR1_START: u32 = 1 << 8; // 起始位（写 1 发 START，硬件自清）
const CR1_STOP: u32 = 1 << 9;  // 停止位

/// CR2 控制位
const CR2_ITEVTEN: u32 = 1 << 9;  // 事件中断使能
const CR2_ITBUFEN: u32 = 1 << 10; // 缓冲中断使能（TxE/RxNE 需 ITEVTEN 同时置位）
const CR2_DMAEN: u32 = 1 << 11;   // DMA 请求使能

/// 寄存器偏移
const OFF_CR1: u32 = 0x00;
const OFF_CR2: u32 = 0x04;
const OFF_DR: u32 = 0x10;
const OFF_SR1: u32 = 0x14;
const OFF_SR2: u32 = 0x18;

/// 寄存器文件数（CR1/CR2/OAR1/OAR2/DR/SR1/SR2/CCR/TRISE）
const REG_COUNT: usize = 9;

/// I2C 外设
pub struct I2c {
    /// I2C 端口号（1/2/3），用于事件过滤
    pub port: u8,
    /// 事件中断 IRQ（I2C1_EV=31/I2C2_EV=33/I2C3_EV=72）
    irq_ev: u32,
    /// 寄存器文件（CR1/CR2/OAR1/OAR2/DR/SR1/SR2/CCR/TRISE）
    regs: [u32; REG_COUNT],
    /// 最近接收字节（读 DR 返回）
    rx_byte: u8,
    /// 事件总线（发布 I2cByte）
    bus: Arc<Mutex<EventBus>>,
    /// NVIC（TxE/RxNE → 挂起 EV IRQ）
    nvic: Arc<Mutex<Nvic>>,
    /// 虚拟从设备表（总线协议级：地址匹配直路由）
    slaves: Vec<Box<dyn VirtualI2cSlave>>,
    /// 地址阶段（写 CR1.START 后、地址字节期待中）
    addr_phase: bool,
    /// 当前事务方向（地址字节 bit0：1=读）
    read_dir: bool,
    /// 当前匹配从设备索引（地址阶段命中后）
    cur_slave: Option<usize>,
    /// 总线事务嗅探器（调试平台 P0-1；默认 None 零回归）
    trace: Option<Arc<Mutex<crate::trace::BusTrace>>>,
}

impl I2c {
    pub fn new(port: u8, bus: Arc<Mutex<EventBus>>, nvic: Arc<Mutex<Nvic>>, irq_ev: u32) -> Self {
        Self {
            port,
            irq_ev,
            regs: [0; REG_COUNT],
            rx_byte: 0,
            bus,
            nvic,
            slaves: Vec::new(),
            addr_phase: false,
            read_dir: false,
            cur_slave: None,
            trace: None,
        }
    }

    /// 挂载总线事务嗅探器（调试/测试；None 关闭）。
    pub fn set_trace(&mut self, trace: Option<Arc<Mutex<crate::trace::BusTrace>>>) {
        self.trace = trace;
    }

    /// 记录一条事务嗅探事件（enabled 时为 no-op）。
    fn trace_record(&self, kind: crate::trace::TraceKind) {
        if let Some(t) = &self.trace {
            t.lock().unwrap().record(crate::trace::BusKind::I2c, self.port, kind);
        }
    }

    /// 注册虚拟从设备（总线协议级；地址匹配直路由）。
    pub fn register_slave(&mut self, slave: Box<dyn VirtualI2cSlave>) {
        self.slaves.push(slave);
    }

    /// 从设备表（观测/step 推进用）。
    pub fn slaves(&self) -> &[Box<dyn VirtualI2cSlave>] {
        &self.slaves
    }

    /// 从设备可变表（故障注入：按地址改 NACK 等）。
    pub fn slaves_mut(&mut self) -> &mut [Box<dyn VirtualI2cSlave>] {
        &mut self.slaves
    }

    /// 推进所有从设备（仿真时间推进 Math 数据源）。
    pub fn step_slaves(&mut self, dt: f32) {
        for s in &mut self.slaves {
            s.step(dt);
        }
    }

    /// 从设备数量（观测/断言）。
    pub fn slave_count(&self) -> usize {
        self.slaves.len()
    }

    /// 地址阶段匹配：按 addr7 找从设备（返回索引）。

    fn match_slave(&self, addr7: u8) -> Option<usize> {
        self.slaves
            .iter()
            .position(|s| s.addr7() == addr7)
    }

    /// 处理地址字节（地址阶段写 DR）：匹配 → ADDR；未命中 → AF。
    fn handle_addr_byte(&mut self, byte: u8) {
        let addr7 = byte >> 1;
        let rw = byte & 1;
        // START 阶段结束：清 SB，进入数据阶段
        self.regs[5] &= !SR1_SB;
        self.addr_phase = false;
        self.read_dir = rw == 1;
        if let Some(idx) = self.match_slave(addr7) {
            self.cur_slave = Some(idx);
            // 故障注入的断线从设备：读方向地址阶段无 ACK → AF（真实硬件语义）。
            // 此前"匹配置 ADDR、数据阶段 on_read 才返回 None 置 AF"会让固件 POLL
            // 驱动（i2c_hal_master_read 轮询 RxNE）在半开事务上超时，且 AF 残留
            // 使后续 0x76（bmp280）数据阶段全停（见 x_fault_injection::midrun_nack）。
            // 地址阶段即 NACK → 固件读 SR1 判 AF 立即失败返回，总线状态干净。
            // 注意仅读方向：写方向（唤醒/配置写）保持原语义——real-sensors 固件
            // 启动探测用 `expect`（写失败直接 panic），写方向 NACK 会触发固件
            // panic（真实缺陷，见 docs/integration.md 已知问题；收窄以兼容）。
            if rw == 1 && self.slaves[idx].nack() {
                self.cur_slave = None; // 无事务：清路由，避免读 SR2 预取残留 on_read
                self.regs[5] &= !SR1_ADDR;
                self.regs[5] |= SR1_AF;
                self.trace_record(crate::trace::TraceKind::I2cStart {
                    addr7,
                    read: true,
                    matched: false,
                });
                return;
            }
            // 通知从设备事务开始（方向）
            self.slaves[idx].on_start(if rw == 1 { I2cDir::Read } else { I2cDir::Write });
            self.regs[5] |= SR1_ADDR; // 地址匹配 → ADDR（读 SR2 清）
            self.regs[5] &= !SR1_AF;
            self.trace_record(crate::trace::TraceKind::I2cStart {
                addr7,
                read: rw == 1,
                matched: true,
            });
        } else {
            self.cur_slave = None;
            self.regs[5] &= !SR1_ADDR;
            self.regs[5] |= SR1_AF; // 无此从设备 → AF（固件判"no such device"）
            self.trace_record(crate::trace::TraceKind::I2cStart {
                addr7,
                read: rw == 1,
                matched: false,
            });
        }
    }

    /// 数据阶段写：路由到从设备（写方向）。
    fn handle_data_write(&mut self, byte: u8) {
        if let Some(idx) = self.cur_slave {
            self.slaves[idx].on_write(byte);
        }
        self.regs[5] |= SR1_TXE;
        self.regs[5] |= SR1_BTF;
        self.trace_record(crate::trace::TraceKind::I2cWrite { byte });
    }

    /// 预取下一读字节（读方向：从设备 on_read → RXNE；None → AF）。
    fn prefetch_read(&mut self) {
        let v = match self.cur_slave {
            Some(idx) => self.slaves[idx].on_read(),
            None => None,
        };
        match v {
            Some(b) => {
                if self.regs[5] & SR1_RXNE != 0 {
                    self.regs[5] |= SR1_OVR; // 未读走 → 过载（简化覆盖）
                }
                self.rx_byte = b;
                self.regs[5] |= SR1_RXNE;
                self.regs[5] |= SR1_BTF;
                self.trace_record(crate::trace::TraceKind::I2cRead { byte: Some(b) });
            }
            None => {
                self.regs[5] |= SR1_AF; // 从设备无数据/断线 → AF（固件读失败）
                self.trace_record(crate::trace::TraceKind::I2cRead { byte: None });
            }
        }
    }

    /// 发送字节（发布 I2cByte 事件；仅 PE 生效）
    fn tx(&self, byte: u8) {
        let ev = Event::I2cByte {
            port: self.port,
            byte,
        };
        self.bus.lock().unwrap().publish(&ev);
    }

    /// ITBUFEN+ITEVTEN 且 TxE/RxNE 置位时挂起 EV 中断（RM：缓冲中断需两位置位）。
    fn set_pending_if_buf(&self) {
        let cr2 = self.regs[1];
        let sr1 = self.regs[5];
        if (cr2 & (CR2_ITBUFEN | CR2_ITEVTEN)) == (CR2_ITBUFEN | CR2_ITEVTEN)
            && (sr1 & (SR1_TXE | SR1_RXNE)) != 0
        {
            self.nvic.lock().unwrap().set_pending(self.irq_ev);
        }
    }

    /// 注入接收字节（测试/虚拟从机经 [`Event::I2cRx`] 调用）。
    ///
    /// PE 时锁存 DR + 置 RxNE（RxNE 已置位 → OVR 置位并覆盖，简化不做锁定）。
    /// ITBUFEN+ITEVTEN 使能 → 挂起 EV 中断。
    pub fn feed_rx(&mut self, byte: u8) {
        if self.regs[0] & CR1_PE == 0 {
            return; // 未使能，字节丢弃
        }
        if self.regs[5] & SR1_RXNE != 0 {
            self.regs[5] |= SR1_OVR; // 上次数据未读走 → 过载（仿真简化：覆盖）
        }
        self.rx_byte = byte;
        self.regs[5] |= SR1_RXNE;
        self.set_pending_if_buf();
        // 注意：不在 feed_rx 内发布 I2cDma——feed_rx 可能在事件分发回调中被调用，
        // 此时事件总线锁已被外层 publish 持有，二次 publish 会同线程重入死锁；
        // RX DMA 请求改由 Machine 的 I2cRx 订阅者在 feed_rx 之后直接路由
        // （见 [`I2c::dma_rx_pending`]）。
    }

    /// 是否有待 DMA 搬运的接收请求（CR2.DMAEN 使能且 RxNE 置位）。
    ///
    /// 供 Machine 在 feed_rx 之后直接路由 RX DMA（避免在事件分发内二次 publish）。
    pub fn dma_rx_pending(&self) -> bool {
        (self.regs[1] & CR2_DMAEN != 0) && (self.regs[5] & SR1_RXNE != 0)
    }

    /// 发送数据寄存器是否空（固件轮询 TxE）
    pub fn tx_ready(&self) -> bool {
        self.regs[5] & SR1_TXE != 0
    }

    /// DMA 读 DR（外设→内存方向）：返回接收字节并清 RxNE。
    ///
    /// 与 CPU 读 DR 同语义（读清 RxNE），供 DMA 控制器搬运调用。
    pub fn dma_read_dr(&mut self) -> u32 {
        let byte = self.rx_byte;
        self.regs[5] &= !SR1_RXNE;
        if self.read_dir {
            self.prefetch_read();
        }
        byte as u32
    }

    /// DMA 写 DR（内存→外设方向）：发送一字节并置 TxE。
    ///
    /// 供 DMA 控制器搬运调用，等价 CPU 写 DR 的发送语义。
    pub fn dma_write_dr(&mut self, value: u32) {
        self.tx(value as u8);
        self.regs[5] |= SR1_TXE;
    }

    /// 检查并发布 DMA 请求（CR2.DMAEN 使能且对应标志置位时）。
    ///
    /// - 内存→外设（TX）：DMAEN 且 TxE → 发布请求，DMA 一次搬完 NDTR；
    /// - 外设→内存（RX）：DMAEN 且 RxNE → 发布请求，DMA 搬 1 字节。
    fn check_dma_request(&self) {
        let cr2 = self.regs[1];
        let sr1 = self.regs[5];
        if (cr2 & CR2_DMAEN != 0) && (sr1 & SR1_TXE != 0) {
            self.bus.lock().unwrap().publish(&Event::I2cDma {
                port: self.port,
                dir: DmaDir::MemToPeriph,
            });
        }
        if (cr2 & CR2_DMAEN != 0) && (sr1 & SR1_RXNE != 0) {
            self.bus.lock().unwrap().publish(&Event::I2cDma {
                port: self.port,
                dir: DmaDir::PeriphToMem,
            });
        }
    }
}

/// DMA 外设方向搬运接口实现（复用 inherent `dma_read_dr`/`dma_write_dr` 语义）。
impl crate::peripheral::dma::DmaByteIo for I2c {
    fn dma_read_dr(&mut self) -> u32 {
        self.dma_read_dr()
    }

    fn dma_write_dr(&mut self, value: u32) {
        self.dma_write_dr(value);
    }
}

impl Peripheral for I2c {
    fn name(&self) -> &str {
        "I2C"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_DR => {
                // 读 DR 返回接收字节并清 RxNE；读方向连续流：预取下一字节（保持 RXNE）
                let v = self.rx_byte as u32;
                self.regs[5] &= !SR1_RXNE;
                if self.read_dir {
                    self.prefetch_read();
                }
                Ok(v)
            }
            0x00..=0x20 => {
                if ((offset / 4) as usize) < REG_COUNT {
                    let v = self.regs[(offset / 4) as usize];
                    if offset == OFF_SR2 {
                        // 读 SR2 清 ADDR（RM：ADDR 读 SR1+SR2 清除）；读方向预取首字节
                        self.regs[5] &= !SR1_ADDR;
                        if self.read_dir && self.cur_slave.is_some() {
                            self.prefetch_read();
                        }
                    }
                    Ok(v)
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
            OFF_SR1 => {
                // rc_w0：写 0 清除错误位（BERR/ARLO/AF/OVR）；TxE/RxNE 只读
                let clear = !value & (SR1_BERR | SR1_ARLO | SR1_AF | SR1_OVR);
                self.regs[5] &= !clear;
                Ok(())
            }
            OFF_DR => {
                if self.regs[0] & CR1_PE != 0 {
                    // 发送：发布事件 + 事务解析（地址阶段→地址匹配；数据阶段→路由从设备）
                    self.tx((value & 0xFF) as u8);
                    if self.addr_phase {
                        self.handle_addr_byte((value & 0xFF) as u8);
                    } else {
                        self.handle_data_write((value & 0xFF) as u8);
                    }
                    self.set_pending_if_buf();
                } else {
                    // 未使能发送：数据丢弃
                    self.regs[5] |= SR1_TXE;
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
                    // PE 使能上升沿 → TxE 置位（仿真简化，见模块注释）
                    if value & CR1_PE != 0 {
                        self.regs[5] |= SR1_TXE;
                    } else {
                        self.regs[5] &= !SR1_TXE;
                    }
                    // START 位写 1 → SB 置位 + 进入地址阶段（下一个 DR 写 = 地址字节）
                    if value & CR1_START != 0 {
                        self.regs[5] |= SR1_SB;
                        self.addr_phase = true;
                    }
                    // STOP 位写 1 → 事务结束（嗅探记录；STOP 硬件自清）
                    if value & CR1_STOP != 0 {
                        self.trace_record(crate::trace::TraceKind::I2cStop);
                    }
                    self.set_pending_if_buf();
                }
                // DMA 使能位（CR2.DMAEN）/缓冲中断使能位变化后检查是否发布请求/挂起
                if offset == OFF_CR2 {
                    self.set_pending_if_buf();
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

    fn i2c() -> (I2c, Arc<Mutex<EventBus>>, Arc<Mutex<Nvic>>) {
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let i = I2c::new(1, bus.clone(), nvic.clone(), I2C1_EV_IRQ);
        (i, bus, nvic)
    }

    #[test]
    fn tx_publishes_i2c_byte_events() {
        let (mut i, bus, _) = i2c();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::I2cByte { .. } = ev {
                    g.lock().unwrap().push(ev.clone());
                }
            })));

        // 未使能时不发布
        i.write(OFF_DR, 4, 0x41).unwrap();
        assert_eq!(got.lock().unwrap().len(), 0);

        // PE 使能后置 TxE（仿真简化）
        i.write(OFF_CR1, 4, CR1_PE).unwrap();
        assert!(i.tx_ready(), "PE 使能后 TxE 应置位");
        i.write(OFF_DR, 4, 0x41).unwrap();
        i.write(OFF_DR, 4, 0x42).unwrap();

        let got = got.lock().unwrap();
        assert_eq!(got.len(), 2);
        assert_eq!(got[0], Event::I2cByte { port: 1, byte: 0x41 });
        assert_eq!(got[1], Event::I2cByte { port: 1, byte: 0x42 });
    }

    #[test]
    fn rx_feed_sets_rxne_and_pends_ev_irq() {
        let (mut i, _, nvic) = i2c();

        // 未使能接收：注入丢弃
        i.feed_rx(0x11);
        assert_eq!(i.read(OFF_SR1, 4).unwrap() & SR1_RXNE, 0);

        // PE 使能 + ITBUFEN|ITEVTEN 后注入
        i.write(OFF_CR1, 4, CR1_PE).unwrap();
        i.write(OFF_CR2, 4, CR2_ITBUFEN | CR2_ITEVTEN).unwrap();
        i.feed_rx(0xAA);
        assert_ne!(i.read(OFF_SR1, 4).unwrap() & SR1_RXNE, 0, "RxNE 应置位");
        assert!(nvic.lock().unwrap().is_pending(I2C1_EV_IRQ), "ITBUFEN 应挂起 I2C1 EV IRQ");

        // 读 DR 清 RxNE 并返回锁存字节
        assert_eq!(i.read(OFF_DR, 4).unwrap(), 0xAA);
        assert_eq!(i.read(OFF_SR1, 4).unwrap() & SR1_RXNE, 0, "读 DR 应清 RxNE");
    }

    #[test]
    fn sr1_rc_w0_clears_ovr_but_not_txe() {
        let (mut i, _, _) = i2c();
        i.write(OFF_CR1, 4, CR1_PE).unwrap();
        i.feed_rx(0x01); // RxNE 置位
        i.feed_rx(0x02); // OVR 置位（RxNE 未清）
        assert_ne!(i.read(OFF_SR1, 4).unwrap() & SR1_OVR, 0);

        // 写 0 清 OVR（写 1 保留）
        i.write(OFF_SR1, 4, !0u32).unwrap();
        assert_ne!(i.read(OFF_SR1, 4).unwrap() & SR1_OVR, 0, "写 1 不应清 OVR");
        i.write(OFF_SR1, 4, 0u32).unwrap();
        assert_eq!(i.read(OFF_SR1, 4).unwrap() & SR1_OVR, 0, "写 0 应清 OVR");

        // TxE 不受 SR1 写影响
        assert_ne!(i.read(OFF_SR1, 4).unwrap() & SR1_TXE, 0);
        i.write(OFF_SR1, 4, 0u32).unwrap();
        assert_ne!(i.read(OFF_SR1, 4).unwrap() & SR1_TXE, 0, "TxE 只读");
    }

    #[test]
    fn dmaen_write_publishes_tx_dma_request() {
        let (mut i, bus, _) = i2c();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::I2cDma { .. } = ev {
                    g.lock().unwrap().push(ev.clone());
                }
            })));

        // PE 使能后 TxE 已置位；写 CR2.DMAEN → 立即发布 TX DMA 请求
        i.write(OFF_CR1, 4, CR1_PE).unwrap();
        i.write(OFF_CR2, 4, CR2_DMAEN).unwrap();

        let got = got.lock().unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(
            got[0],
            Event::I2cDma {
                port: 1,
                dir: DmaDir::MemToPeriph,
            },
            "DMAEN 写且 TxE 置位应发布 TX DMA 请求"
        );
    }

    #[test]
    fn dma_read_write_dr_roundtrip() {
        let (mut i, bus, _) = i2c();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::I2cByte { .. } = ev {
                    g.lock().unwrap().push(ev.clone());
                }
            })));

        i.write(OFF_CR1, 4, CR1_PE).unwrap();
        i.feed_rx(0x55);
        assert!(!i.dma_rx_pending(), "DMAEN 未使能时 dma_rx_pending 应为假");
        i.write(OFF_CR2, 4, CR2_DMAEN).unwrap();
        assert!(i.dma_rx_pending(), "DMAEN+RxNE → RX DMA 待搬运");
        assert_eq!(i.dma_read_dr(), 0x55, "DMA 读 DR 返回锁存字节");
        assert!(!i.dma_rx_pending(), "DMA 读 DR 后 RxNE 清，请求解除");

        i.dma_write_dr(0x77);
        assert_eq!(got.lock().unwrap().len(), 1, "DMA 写 DR 应发布发送事件");
    }
}
