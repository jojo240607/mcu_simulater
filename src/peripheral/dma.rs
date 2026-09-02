//! DMA1/DMA2 直接内存访问控制器（STM32F407，M4）。
//!
//! 同一实现参数化挂载两份：DMA1 @ 0x40026000、DMA2 @ 0x40026400（各 8 流，
//! 寄存器布局一致，仅流中断号不同）。
//!
//! M4 语义（内存到内存端到端）：
//! - 寄存器文件镜像：LISR/HISR/LIFCR/HIFCR + 8 流 × (CR/NDTR/PAR/M0AR/M1AR/FCR)；
//! - [`Peripheral::tick`]：对 EN 且 DIR=内存到内存（bit7:6=10）的流，首个 tick 判定
//!   传输完成——NDTR 清零、LISR/HISR.TCIF 置位、TCIE 使能时向共享 NVIC 置挂起
//!   （DMAx_StreamN 中断，见 [`DMA1_STREAM_IRQ`]/[`DMA2_STREAM_IRQ`]）、EN 自动清零，
//!   并把流登记到“待搬运”位图；
//! - [`Dma::process`]：Machine::run 在 CPU 空闲间隙（每次 emu_start 返回后）调用，
//!   对登记流执行真实内存搬运（PAR → M0AR，按 PSIZE 读 / 按 MSIZE 写，
//!   PINC/MINC 地址递增）。CPU 空闲时 Unicorn 内存访问不触发 MMIO hook，故仅支持
//!   纯内存目标（SRAM/Flash），外设目标（如 USART DR）留待后续；
//! - 状态位 rc_w1：写 LIFCR/HIFCR 对应位 = 1 清除 LISR/HISR。
//!
//! M5 扩展（外设↔内存，USART DMA 传输）：
//! - USART 使能 DMAR/DMAT 后在 RXNE/TXE 触发 [`crate::events::Event::UartDma`]，
//!   Machine 按固定映射表（port, 方向）路由到 DMAx_StreamN_ChannelM 并调用
//!   [`Dma::service_stream`] 登记待搬运（校验 CHSEL/DIR/EN）；
//! - 外设方向搬运经已注册的 USART 句柄直接读写 DR（[`Usart::dma_read_dr`] /
//!   [`Usart::dma_write_dr`]），避免 Unicorn CPU 内存 API 不触发 MMIO hook 的限制；
//! - 外设→内存（RX）：每收 1 字节搬 1 次；内存→外设（TX）：TXE 就绪一次搬完 NDTR，
//!   完成后 NDTR 清零、EN 自动清零、TCIF 置位、TCIE 使能时挂起对应流中断。
//!
//! 地址映射（offset 相对 DMAx 基址）：
//! - LISR 0x00 / HISR 0x04 / LIFCR 0x08 / HIFCR 0x0C
//! - S0CR 0x10, S0NDTR 0x14, S0PAR 0x18, S0M0AR 0x1C, S0M1AR 0x20, S0FCR 0x24；
//!   流间隔 0x18（S1@0x28 … S7@0xD0）。
//! 状态位每流 6 位（FEIF=0, DMEIF=2, TEIF=3, HTIF=4, TCIF=5），低 4 流在 LISR/LIFCR、
//! 高 4 流在 HISR/HIFCR，流内偏移 = (流 % 4) × 6。

use std::sync::{Arc, Mutex};

use crate::core::Cpu;
use crate::peripheral::i2c::I2c;
use crate::peripheral::nvic::Nvic;
use crate::peripheral::usart::Usart;
use crate::peripheral::{BusError, Peripheral};

/// DMA1 基址
pub const DMA1_BASE: u32 = 0x4002_6000;
/// DMA2 基址
pub const DMA2_BASE: u32 = 0x4002_6400;
/// DMA1 各流中断号（Stream0..7，F407：流0-3=11-14、流4-6=15-17、流7=47）
pub const DMA1_STREAM_IRQ: [u32; 8] = [11, 12, 13, 14, 15, 16, 17, 47];
/// DMA2 各流中断号（Stream0..7，F407：Stream0-4=56-60，Stream5-7=68-70）
pub const DMA2_STREAM_IRQ: [u32; 8] = [56, 57, 58, 59, 60, 68, 69, 70];

/// DMA 传输方向（CR.DIR bit7:6）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaDir {
    /// 外设 → 内存（DIR=00，如 USART RX）
    PeriphToMem,
    /// 内存 → 外设（DIR=01，如 USART TX）
    MemToPeriph,
}

impl DmaDir {
    fn bits(self) -> u32 {
        match self {
            DmaDir::PeriphToMem => 0,
            DmaDir::MemToPeriph => 1,
        }
    }
}

/// 外设方向 DMA 搬运目标（区分不同外设类型，供 service_stream/process 选择句柄表）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaTarget {
    /// USART 外设（port 1..6，DMA 经 [`Usart::dma_read_dr`]/[`Usart::dma_write_dr`] 读写 DR）
    Usart(u8),
    /// I2C 外设（port 1..3，DMA 经 [`I2c::dma_read_dr`]/[`I2c::dma_write_dr`] 读写 DR）
    I2c(u8),
}

/// 外设方向 DMA 搬运接口：DR 读写（供 DMA `process` 搬运外设↔内存）。
///
/// USART/I2C 等数据寄存器型外设实现，`service_stream` 登记的流在 process 中
/// 经该接口直接读写 DR，绕开 Unicorn CPU 内存 API 不触发 MMIO hook 的限制。
pub trait DmaByteIo: Send {
    /// 外设 → 内存（RX）：读数据寄存器并清接收标志
    fn dma_read_dr(&mut self) -> u8;
    /// 内存 → 外设（TX）：写数据寄存器（外设侧发送一字节）
    fn dma_write_dr(&mut self, byte: u8);
}

/// CR 控制位
const CR_EN: u32 = 1 << 0; // 使能
const CR_TCIE: u32 = 1 << 5; // 传输完成中断使能
const CR_PINC: u32 = 1 << 9; // 外设地址递增
const CR_MINC: u32 = 1 << 10; // 内存地址递增
// DIR bit7:6：00=外设→内存, 01=内存→外设, 10=内存→内存
const CR_DIR_MM: u32 = 2 << 6;
/// 通道选择（CHSEL bit28:25）
const CR_CHSEL_SHIFT: u32 = 25;

/// 状态位
const FLAG_TCIF: u32 = 5; // 传输完成

/// 寄存器偏移
const OFF_LISR: u32 = 0x00;
const OFF_HISR: u32 = 0x04;
const OFF_LIFCR: u32 = 0x08;
const OFF_HIFCR: u32 = 0x0C;
const OFF_CR: u32 = 0x10; // S0CR 起始；流间隔 0x18

/// 寄存器文件总长（4 + 8 × 6 = 52 个 32 位寄存器）
const REG_COUNT: usize = 52;

/// 流 s 的 CR 在 regs 中的索引（S0CR@0x10 → idx 4）
fn stream_cr_idx(s: usize) -> usize {
    (OFF_CR as usize / 4) + s * 6
}

/// DMA1/DMA2 外设（同一实现，name + 流中断表参数化）
pub struct Dma {
    /// 外设名（DMA1/DMA2，供日志/识别）
    name: &'static str,
    /// 各流中断号（DMA1 或 DMA2 表）
    stream_irq: [u32; 8],
    /// 寄存器文件
    regs: [u32; REG_COUNT],
    /// 共享 NVIC（传输完成 → 置挂起对应流中断）
    nvic: Arc<Mutex<Nvic>>,
    /// 待搬运流位图（tick 判定完成，run 间隙 process 执行搬运）
    pending_transfer: u32,
    /// 各流待搬运项数（tick 完成时记录，process 消费）
    pending_items: [u32; 8],
    /// 各流待搬运的外设目标（None = 内存方向/未登记）
    pending_target: [Option<DmaTarget>; 8],
    /// 注册的 USART 句柄（index 0..5 = USART1..6，外设方向搬运直接读写 DR）
    usart_handles: [Option<Arc<Mutex<Usart>>>; 6],
    /// 注册的 I2C 句柄（index 0..2 = I2C1..3，外设方向搬运直接读写 DR）
    i2c_handles: [Option<Arc<Mutex<I2c>>>; 3],
}

impl Dma {
    pub fn new(nvic: Arc<Mutex<Nvic>>, name: &'static str, stream_irq: [u32; 8]) -> Self {
        Self {
            name,
            stream_irq,
            regs: [0; REG_COUNT],
            nvic,
            pending_transfer: 0,
            pending_items: [0; 8],
            pending_target: [None; 8],
            usart_handles: Default::default(),
            i2c_handles: Default::default(),
        }
    }

    /// 注册 USART 句柄（供外设方向搬运读写 DR）。
    ///
    /// Machine 挂载 USART1-6 时对 DMA1/DMA2 各调用一次；`port` 取值 1..6。
    pub fn register_usart(&mut self, port: u8, usart: Arc<Mutex<Usart>>) {
        if (1..=6).contains(&port) {
            self.usart_handles[(port - 1) as usize] = Some(usart);
        }
    }

    /// 注册 I2C 句柄（供外设方向搬运读写 DR）。
    ///
    /// Machine 挂载 I2C1-3 时对 DMA1 调用；`port` 取值 1..3（I2C DMA 全在 DMA1）。
    pub fn register_i2c(&mut self, port: u8, i2c: Arc<Mutex<I2c>>) {
        if (1..=3).contains(&port) {
            self.i2c_handles[(port - 1) as usize] = Some(i2c);
        }
    }

    /// 处理外设发布的 DMA 请求（经 [`crate::events::Event::UartDma`]/[`Event::I2cDma`] 路由）。
    ///
    /// 校验流 CR：EN 置位、CHSEL 与请求通道一致、DIR 与请求方向一致后登记待搬运。
    /// - 外设→内存（RX）：每收 1 字节触发 1 次搬运（外设每字节发 1 次请求）；
    /// - 内存→外设（TX）：TXE 就绪一次搬运整个 NDTR（仿真快速发送一次完成）。
    pub fn service_stream(&mut self, stream: usize, channel: u32, dir: DmaDir, target: DmaTarget) {
        let cr = self.stream_reg(stream, 0);
        let ndtr = self.stream_reg(stream, 1);
        if cr & CR_EN == 0 {
            return; // 流未使能：忽略请求
        }
        if (cr >> CR_CHSEL_SHIFT) & 0x7 != channel {
            return; // CHSEL 不匹配（本流不服务该外设通道）
        }
        if (cr >> 6) & 0x3 != dir.bits() {
            return; // CR.DIR 与请求方向不一致
        }
        let items = match dir {
            DmaDir::PeriphToMem => 1,     // 每收 1 字节搬 1 项
            DmaDir::MemToPeriph => ndtr,  // TXE 就绪 → 一次搬完
        };
        if items == 0 {
            return;
        }
        self.pending_transfer |= 1 << stream;
        self.pending_items[stream] = items;
        self.pending_target[stream] = Some(target);
    }

    /// 状态位所属中断状态寄存器索引（低 4 流 → LISR，高 4 流 → HISR）
    fn isr_idx(s: usize) -> usize {
        if s < 4 {
            0
        } else {
            1
        }
    }

    /// 流 s 状态位在 ISR/IFCR 内的偏移（流内 6 位）
    fn flag_offset(s: usize) -> u32 {
        ((s % 4) as u32) * 6
    }

    /// 置位流状态位
    fn set_stream_flag(&mut self, s: usize, flag: u32) {
        self.regs[Self::isr_idx(s)] |= 1u32 << (Self::flag_offset(s) + flag);
    }

    /// 读流寄存器
    fn stream_reg(&self, s: usize, sub: usize) -> u32 {
        self.regs[stream_cr_idx(s) + sub]
    }

    /// 写流寄存器
    fn set_stream_reg(&mut self, s: usize, sub: usize, value: u32) {
        self.regs[stream_cr_idx(s) + sub] = value;
    }

    /// 执行登记流的搬运（PAR → M0AR）。
    ///
    /// 仅在 CPU 空闲间隙由 Machine 调用；一次搬运 `items` 项，
    /// 按 PSIZE（源）/MSIZE（目标）取宽，PINC/MINC 决定地址递增。
    /// - 内存到内存：PAR/M0AR 均为内存，走 CPU 内存 API；
    /// - 外设方向：内存侧用 CPU 内存 API，外设侧经已注册 USART 句柄直接读写 DR
    ///   （TX：M0AR → [`Usart::dma_write_dr`]；RX：[`Usart::dma_read_dr`] → M0AR），
    ///   完成后 NDTR 清零、EN 自动清零、TCIF 置位、TCIE 时挂起流中断。
    pub fn process(&mut self, cpu: &mut Cpu) {
        let mut mask = self.pending_transfer;
        while mask != 0 {
            let s = mask.trailing_zeros() as usize;
            mask &= mask - 1;
            let items = std::mem::take(&mut self.pending_items[s]);
            if items == 0 {
                self.pending_transfer &= !(1 << s);
                continue;
            }
            let cr = self.stream_reg(s, 0);
            let dir = (cr >> 6) & 0x3; // 00=外设→内存, 01=内存→外设, 10=内存→内存
            let psize = ((cr >> 11) & 0x3) as usize; // 0=字节,1=半字,2=字
            let msize = ((cr >> 13) & 0x3) as usize;
            let pw = 1usize << psize;
            let mw = 1usize << msize;
            let pinc = cr & CR_PINC != 0;
            let minc = cr & CR_MINC != 0;
            let mut src = self.stream_reg(s, 2); // PAR
            let mut dst = self.stream_reg(s, 3); // M0AR

            if dir == 2 {
                // 内存到内存：PAR → M0AR（原 M4 路径）
                let mut buf = [0u8; 4];
                for _ in 0..items {
                    if let Ok(data) = cpu.mem_read(src as u64, pw) {
                        buf[..pw].copy_from_slice(&data[..pw]);
                        let _ = cpu.mem_write(dst as u64, &buf[..mw]);
                    }
                    if pinc {
                        src += pw as u32;
                    }
                    if minc {
                        dst += mw as u32;
                    }
                }
                // 地址回写：PAR/M0AR 随 PINC/MINC 更新（硬件 DMA 寄存器行为）
                self.set_stream_reg(s, 2, src);
                self.set_stream_reg(s, 3, dst);
                self.pending_transfer &= !(1 << s);
                continue;
            }

            // 外设方向：经已注册外设句柄读写 DR（内存侧走 CPU 内存 API）。
            // 按 DmaTarget 选句柄表（USART/I2C 同接口 [`DmaByteIo`]）。
            let Some(target) = self.pending_target[s] else {
                self.pending_transfer &= !(1 << s); // 未登记目标（配置异常）：跳过
                continue;
            };
            let handle: Option<Arc<Mutex<dyn DmaByteIo>>> = match target {
                DmaTarget::Usart(port) => self
                    .usart_handles
                    .get((port as usize).wrapping_sub(1))
                    .and_then(|h| h.clone())
                    .map(|h| h as Arc<Mutex<dyn DmaByteIo>>),
                DmaTarget::I2c(port) => self
                    .i2c_handles
                    .get((port as usize).wrapping_sub(1))
                    .and_then(|h| h.clone())
                    .map(|h| h as Arc<Mutex<dyn DmaByteIo>>),
            };
            let Some(dev) = handle else {
                self.pending_transfer &= !(1 << s); // 未注册句柄（配置异常）：跳过
                continue;
            };
            let ndtr = self.stream_reg(s, 1);
            let new_ndtr = ndtr.saturating_sub(items);
            let mut dev = dev.lock().unwrap();
            if dir == 1 {
                // 内存 → 外设（TX）：M0AR → 外设 DR
                for _ in 0..items {
                    if let Ok(data) = cpu.mem_read(dst as u64, mw) {
                        dev.dma_write_dr(data[0]);
                    }
                    if minc {
                        dst += mw as u32;
                    }
                }
            } else {
                // 外设 → 内存（RX）：外设 DR → M0AR
                for _ in 0..items {
                    let byte = dev.dma_read_dr();
                    let _ = cpu.mem_write(dst as u64, &[byte]);
                    if minc {
                        dst += mw as u32;
                    }
                }
            }
            drop(dev);
            // MINC 地址回写：下次搬运从续接地址开始（外设方向 PAR 固定，仅回写 M0AR）
            self.set_stream_reg(s, 3, dst);
            self.set_stream_reg(s, 1, new_ndtr);
            self.pending_transfer &= !(1 << s);
            if new_ndtr == 0 {
                // 传输完成：EN 自动清零 + TCIF + 中断
                self.set_stream_reg(s, 0, cr & !CR_EN);
                self.set_stream_flag(s, FLAG_TCIF);
                if cr & CR_TCIE != 0 {
                    self.nvic.lock().unwrap().set_pending(self.stream_irq[s]);
                }
            }
        }
    }
}

impl Peripheral for Dma {
    fn name(&self) -> &str {
        self.name
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let idx = (offset / 4) as usize;
        if idx < REG_COUNT {
            // LIFCR/HIFCR 只写（rc_w1），读回 0
            if idx == (OFF_LIFCR / 4) as usize || idx == (OFF_HIFCR / 4) as usize {
                return Ok(0);
            }
            Ok(self.regs[idx])
        } else {
            Err(BusError::OutOfRange)
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let idx = (offset / 4) as usize;
        match idx {
            // LISR/HISR 只读：忽略写
            i if i == (OFF_LISR / 4) as usize || i == (OFF_HISR / 4) as usize => Ok(()),
            // LIFCR/HIFCR：写 1 清除 LISR/HISR 对应位（rc_w1）
            i if i == (OFF_LIFCR / 4) as usize => {
                self.regs[0] &= !value;
                Ok(())
            }
            i if i == (OFF_HIFCR / 4) as usize => {
                self.regs[1] &= !value;
                Ok(())
            }
            i if i < REG_COUNT => {
                self.regs[i] = value;
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn tick(&mut self, _cycles: u64) {
        for s in 0..8usize {
            let cr = self.stream_reg(s, 0);
            if cr & CR_EN == 0 {
                continue;
            }
            // M4 仅支持内存到内存（DIR=10）
            if (cr >> 6) & 0x3 != CR_DIR_MM >> 6 {
                continue;
            }
            // 一次性完成：NDTR 清零、TCIF 置位、EN 自动清零、登记待搬运
            let ndtr = self.stream_reg(s, 1);
            self.set_stream_reg(s, 1, 0);
            self.set_stream_reg(s, 0, cr & !CR_EN);
            self.set_stream_flag(s, FLAG_TCIF);
            if cr & CR_TCIE != 0 {
                self.nvic.lock().unwrap().set_pending(self.stream_irq[s]);
            }
            if ndtr != 0 {
                self.pending_transfer |= 1 << s;
                self.pending_items[s] = ndtr;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use unicorn_engine::Prot;

    fn dma() -> Dma {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        Dma::new(nvic, "DMA1", DMA1_STREAM_IRQ)
    }

    /// 便利：把 CR/NDTR/PAR/M0AR 写入流 0
    fn cfg_stream(d: &mut Dma, cr: u32, ndtr: u32, par: u32, m0ar: u32) {
        d.write(OFF_CR, 4, cr).unwrap();
        d.write(OFF_CR + 4, 4, ndtr).unwrap();
        d.write(OFF_CR + 8, 4, par).unwrap();
        d.write(OFF_CR + 0x0C, 4, m0ar).unwrap();
    }

    #[test]
    fn register_layout_and_flag_position() {
        let mut d = dma();
        // 流 3 的 TCIF 在 LISR bit 5+18=23；流 4 的 TCIF 在 HISR bit5
        for s in [3usize, 4] {
            let base = OFF_CR + s as u32 * 0x18;
            d.write(base, 4, CR_EN | CR_DIR_MM | CR_TCIE).unwrap();
            d.write(base + 4, 4, 2).unwrap();
            d.write(base + 8, 4, 0x2000_0100).unwrap();
            d.write(base + 0x0C, 4, 0x2000_0200).unwrap();
        }
        d.tick(1);
        assert_eq!(d.regs[0] & (1 << 23), 1 << 23, "Stream3 TCIF 应在 LISR bit23");
        assert_eq!(d.regs[1] & (1 << 5), 1 << 5, "Stream4 TCIF 应在 HISR bit5");
        assert_eq!(d.regs[0] & (1 << 5), 0, "Stream0 未使能不置 LISR bit5");
    }

    #[test]
    fn tick_completes_mem2mem_and_clears_en() {
        let mut d = dma();
        cfg_stream(&mut d, CR_EN | CR_DIR_MM | CR_TCIE, 8, 0x2000_0100, 0x2000_0200);
        d.tick(1);
        assert_eq!(d.stream_reg(0, 1), 0, "NDTR 完成后清零");
        assert_eq!(d.stream_reg(0, 0) & CR_EN, 0, "EN 传输完成后自动清零");
        assert_eq!(d.regs[0] & (1 << 5), 1 << 5, "Stream0 TCIF 置位");
        assert_eq!(d.pending_transfer, 1 << 0, "流 0 登记待搬运");
        assert_eq!(d.pending_items[0], 8, "待搬运 8 项");
        // 二次 tick 不重复（EN 已清）
        d.tick(1);
        assert_eq!(d.pending_transfer, 1 << 0, "不重复登记");
    }

    #[test]
    fn non_mem2mem_ignored() {
        let mut d = dma();
        // DIR=外设→内存（00）：tick 不完成
        cfg_stream(&mut d, CR_EN | CR_TCIE, 8, 0x2000_0100, 0x2000_0200);
        d.tick(1);
        assert_eq!(d.pending_transfer, 0, "非 MEM2MEM 不登记搬运");
        assert_eq!(d.regs[0] & (1 << 5), 0, "不置 TCIF");
    }

    #[test]
    fn process_copies_word_mem2mem() {
        let mut d = dma();
        // MEM2MEM：PAR=源、M0AR=目标，PINC+MINC 地址递增，字宽
        cfg_stream(
            &mut d,
            CR_EN | CR_DIR_MM | CR_PINC | CR_MINC | (2 << 13) | (2 << 11),
            4,
            0x2000_0100,
            0x2000_0200,
        );

        let mut cpu = Cpu::new_m4f().unwrap();
        cpu.mem_map(0x2000_0000, 0x4000, Prot::ALL).unwrap();
        let src: [u32; 4] = [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444];
        for (i, v) in src.iter().enumerate() {
            cpu.mem_write(0x2000_0100 + i as u64 * 4, &v.to_le_bytes()).unwrap();
        }

        d.tick(1); // 判定完成 + 登记
        d.process(&mut cpu); // 空闲间隙搬运

        for (i, v) in src.iter().enumerate() {
            let got = cpu.mem_read(0x2000_0200 + i as u64 * 4, 4).unwrap();
            assert_eq!(u32::from_le_bytes(got.try_into().unwrap()), *v, "dst[{i}] 应等于 src[{i}]");
        }
        assert_eq!(d.pending_transfer, 0, "搬运后位图清空");
    }

    #[test]
    fn process_byte_mode_without_minc() {
        let mut d = dma();
        // PSIZE=MSIZE=字节(00)，PINC=1（源递增），MINC=0（目标固定）
        cfg_stream(&mut d, CR_EN | CR_DIR_MM | CR_PINC, 3, 0x2000_0100, 0x2000_0200);

        let mut cpu = Cpu::new_m4f().unwrap();
        cpu.mem_map(0x2000_0000, 0x4000, Prot::ALL).unwrap();
        cpu.mem_write(0x2000_0100, &[0xAA, 0xBB, 0xCC]).unwrap();

        d.tick(1);
        d.process(&mut cpu);

        // 三次都写到固定目标 0x20000200，最后一次 0xCC
        let got = cpu.mem_read(0x2000_0200, 1).unwrap();
        assert_eq!(got[0], 0xCC, "MINC=0 时目标固定，最后写入 0xCC");
    }

    #[test]
    fn lifcr_clears_tcif() {
        let mut d = dma();
        cfg_stream(&mut d, CR_EN | CR_DIR_MM, 2, 0x2000_0100, 0x2000_0200);
        d.tick(1);
        assert_eq!(d.regs[0] & (1 << 5), 1 << 5);
        // 写 LIFCR bit5 = 1 清除 LISR.TCIF0
        d.write(OFF_LIFCR, 4, 1 << 5).unwrap();
        assert_eq!(d.regs[0] & (1 << 5), 0, "LIFCR 写 1 应清除 TCIF0");
        // 读 LIFCR 回 0
        assert_eq!(d.read(OFF_LIFCR, 4).unwrap(), 0);
    }

    #[test]
    fn dma2_instance_uses_own_irq_table() {
        // DMA2 实例：Stream0 完成 → 挂起 IRQ56（非 DMA1 的 IRQ11）
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let mut d = Dma::new(nvic.clone(), "DMA2", DMA2_STREAM_IRQ);
        assert_eq!(d.name(), "DMA2");

        cfg_stream(&mut d, CR_EN | CR_DIR_MM | CR_TCIE, 2, 0x2000_0100, 0x2000_0200);
        d.tick(1);
        assert!(
            nvic.lock().unwrap().is_pending(56),
            "DMA2 Stream0 完成应挂起 IRQ56"
        );
        assert!(
            !nvic.lock().unwrap().is_pending(11),
            "DMA2 不应挂起 DMA1 的 IRQ11"
        );
    }
}
