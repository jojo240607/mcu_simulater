//! DCMI 数字摄像头接口（STM32F407，M12 虚拟外设生态）。
//!
//! - CR.ENABLE（bit14）写 1 使能接口；CR.CAPTURE（bit0）使能捕获（硬件同步沿）；
//!   CR.CM（bit1）=1 为快照模式（一帧后自动清 CAPTURE），=0 为连续模式；
//! - 数据注入：测试/虚拟摄像头经 [`crate::events::Event::DcmiFrame`] 发布一帧像素
//!   数据 → [`Dcmi::feed_frame`] 把字节流拆成 32 位字压入内部 FIFO（不足 4 字节的
//!   尾字低位补零）并置 SR.FNE（FIFO 非空）+ SR.FRAME/LINE（帧/行完成）；
//!   若 CR 未使能捕获则整帧丢弃（硬件上 DCMI 关闭时像素被忽略）；
//! - 数据读出：轮询读 DR（[`Dcmi::read`]）或 DMA 模式经 [`DmaByteIo::dma_read_dr`]
//!   弹出字，FIFO 空后清 SR.FNE；DMA 映射 DMA2_Stream1_Channel1（外设→内存）；
//! - 中断：帧/行/溢出/同步等中断状态经 RIS（原始）与 MIS（RIS & IER）呈现，
//!   IER 使能位对应置位且 ICR 写 1 清除；帧完成使能时挂起 DCMI IRQ（F407 IRQ78）。
//!
//! 地址映射（DCMI @ 0x50050000）：CR 0x00 / SR 0x04 / RIS 0x08 / IER 0x0C /
//! MIS 0x10 / ICR 0x14 / ESCR 0x18 / ESUR 0x1C / CWSTRT 0x20 / CWSIZE 0x24 / DR 0x28

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use crate::peripheral::dma::DmaByteIo;
use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};

/// DCMI NVIC IRQ（STM32F407：DCMI 全局中断 = 78）
pub const DCMI_IRQ: u32 = 78;

/// 寄存器偏移
const OFF_CR: u32 = 0x00;
const OFF_SR: u32 = 0x04;
const OFF_RIS: u32 = 0x08;
const OFF_IER: u32 = 0x0C;
const OFF_MIS: u32 = 0x10;
const OFF_ICR: u32 = 0x14;
const OFF_ESCR: u32 = 0x18;
const OFF_ESUR: u32 = 0x1C;
const OFF_CWSTRT: u32 = 0x20;
const OFF_CWSIZE: u32 = 0x24;
const OFF_DR: u32 = 0x28;

/// CR 位（RM0090）
const CR_CAPTURE: u32 = 1 << 0; // 捕获使能
const CR_CM: u32 = 1 << 1; // 捕获模式：1=快照，0=连续
const CR_CRC: u32 = 1 << 2; // 捕获率控制
const CR_EDM_MASK: u32 = 0x3 << 3; // 扩展数据模式（8/10/12/14 位）
const CR_ENABLE: u32 = 1 << 14; // DCMI 使能
const CR_FCRC_MASK: u32 = 0x3 << 15; // 帧捕获率控制
const CR_VSPOL: u32 = 1 << 17; // VSYNC 极性
const CR_HSPOL: u32 = 1 << 18; // HSYNC 极性
const CR_PCPOL: u32 = 1 << 19; // 像素时钟极性
const CR_ESS: u32 = 1 << 20; // 嵌入码同步选择
const CR_JPEG: u32 = 1 << 22; // JPEG 格式
/// CR 可写位掩码（保留位忽略）
const CR_WMASK: u32 = CR_CAPTURE
    | CR_CM
    | CR_CRC
    | CR_EDM_MASK
    | CR_ENABLE
    | CR_FCRC_MASK
    | CR_VSPOL
    | CR_HSPOL
    | CR_PCPOL
    | CR_ESS
    | CR_JPEG;

/// SR 位（RM0090）
const SR_FNE: u32 = 1 << 0; // FIFO 非空
const SR_LINE: u32 = 1 << 6; // 行完成
const SR_FRAME: u32 = 1 << 7; // 帧完成

/// 中断状态位（RIS/ICR 低 6 位对齐）
const INT_FRAME: u32 = 1 << 0; // 帧
const INT_LINE: u32 = 1 << 3; // 行
const INT_MASK: u32 = 0x3F;

/// DCMI 数字摄像头接口
pub struct Dcmi {
    /// NVIC（帧/行等中断挂起）
    nvic: Arc<Mutex<Nvic>>,
    /// DCMI 全局中断 IRQ（DCMI_IRQ）
    irq: u32,
    /// CR 镜像（ENABLE/CAPTURE/CM/EDM/…）
    cr: u32,
    /// SR 镜像（FNE/OVR/LINE/FRAME/…）
    sr: u32,
    /// RIS 原始中断状态
    ris: u32,
    /// IER 中断使能
    ier: u32,
    /// ESCR/ESUR/CWSTRT/CWSIZE 配置存储（无功能仿真）
    escr: u32,
    esur: u32,
    cwstrt: u32,
    cwsize: u32,
    /// 32 位 FIFO（注入帧的字序列；读 DR/DMA 弹出）
    fifo: VecDeque<u32>,
}

impl Dcmi {
    pub fn new(nvic: Arc<Mutex<Nvic>>, irq: u32) -> Self {
        Self {
            nvic,
            irq,
            cr: 0,
            sr: 0,
            ris: 0,
            ier: 0,
            escr: 0,
            esur: 0,
            cwstrt: 0,
            cwsize: 0,
            fifo: VecDeque::new(),
        }
    }

    /// 注入一帧像素数据（测试/虚拟摄像头 → DCMI）。
    ///
    /// CR.ENABLE+CAPTURE 均置位才接受，否则整帧丢弃（返回 0）。
    /// 接受时：字节流按 4 字节拆成 32 位字压入 FIFO（尾字低位补零）、置
    /// SR.FNE + SR.FRAME/LINE 与 RIS.FRAME/LINE；快照模式（CM=1）下自动清
    /// CAPTURE；IER 使能帧/行中断时挂起 DCMI IRQ。
    ///
    /// 返回值：压入 FIFO 的 32 位字数（供 Machine 路由 DMA 搬运项数）。
    pub fn feed_frame(&mut self, data: &[u8]) -> u32 {
        // 未使能/未捕获：硬件上 DCMI 关闭时像素被丢弃
        if self.cr & (CR_ENABLE | CR_CAPTURE) != (CR_ENABLE | CR_CAPTURE) {
            return 0;
        }
        // 字节 → 32 位字（低位在前；不足 4 字节的尾字补零）
        let mut words = Vec::with_capacity(data.len().div_ceil(4));
        for chunk in data.chunks(4) {
            let mut w = 0u32;
            for (i, b) in chunk.iter().enumerate() {
                w |= (*b as u32) << (8 * i);
            }
            words.push(w);
        }
        if words.is_empty() {
            return 0;
        }
        let count = words.len() as u32;
        self.fifo.extend(words);
        self.sr |= SR_FNE;
        // 帧/行完成：单帧注入简化为一次 FRAME + LINE 事件
        self.sr |= SR_FRAME | SR_LINE;
        self.ris |= INT_FRAME | INT_LINE;
        // 快照模式：一帧后自动清 CAPTURE
        if self.cr & CR_CM != 0 {
            self.cr &= !CR_CAPTURE;
        }
        // 中断
        if self.ier & (INT_FRAME | INT_LINE) != 0 {
            self.nvic.lock().unwrap().set_pending(self.irq);
        }
        count
    }

    /// FIFO 是否非空（Machine 据此路由 DMA 请求）
    pub fn fne(&self) -> bool {
        !self.fifo.is_empty()
    }

    /// 弹出下一个 32 位字（读 DR / DMA 共用；FIFO 空返回 0 并清 FNE）
    fn pop_word(&mut self) -> u32 {
        let v = self.fifo.pop_front().unwrap_or(0);
        if self.fifo.is_empty() {
            self.sr &= !SR_FNE;
        }
        v
    }
}

impl DmaByteIo for Dcmi {
    /// 外设 → 内存：读数据寄存器（弹出 FIFO 字）
    fn dma_read_dr(&mut self) -> u32 {
        self.pop_word()
    }

    /// DCMI 仅输入：内存 → 外设写忽略
    fn dma_write_dr(&mut self, _value: u32) {}
}

impl Peripheral for Dcmi {
    fn name(&self) -> &str {
        "DCMI"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_CR => Ok(self.cr),
            OFF_SR => Ok(self.sr),
            OFF_RIS => Ok(self.ris),
            OFF_IER => Ok(self.ier),
            OFF_MIS => Ok(self.ris & self.ier),
            OFF_ICR => Ok(0), // 只写
            OFF_ESCR => Ok(self.escr),
            OFF_ESUR => Ok(self.esur),
            OFF_CWSTRT => Ok(self.cwstrt),
            OFF_CWSIZE => Ok(self.cwsize),
            OFF_DR => Ok(self.pop_word()),
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_CR => {
                self.cr = value & CR_WMASK;
                Ok(())
            }
            OFF_SR | OFF_RIS | OFF_MIS | OFF_DR => Ok(()), // 只读：写忽略
            OFF_IER => {
                self.ier = value & INT_MASK;
                Ok(())
            }
            OFF_ICR => {
                // 写 1 清除 RIS 对应位（rc_w1）
                self.ris &= !(value & INT_MASK);
                Ok(())
            }
            OFF_ESCR => {
                self.escr = value;
                Ok(())
            }
            OFF_ESUR => {
                self.esur = value;
                Ok(())
            }
            OFF_CWSTRT => {
                self.cwstrt = value;
                Ok(())
            }
            OFF_CWSIZE => {
                self.cwsize = value;
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.cr = 0;
        self.sr = 0;
        self.ris = 0;
        self.ier = 0;
        self.escr = 0;
        self.esur = 0;
        self.cwstrt = 0;
        self.cwsize = 0;
        self.fifo.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::nvic::Nvic;

    fn make() -> (Dcmi, Arc<Mutex<Nvic>>) {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        (Dcmi::new(nvic.clone(), DCMI_IRQ), nvic)
    }

    /// 便捷：使能 + 捕获（连续模式）
    fn enable_capture(d: &mut Dcmi) {
        d.write(OFF_CR, 4, CR_ENABLE | CR_CAPTURE).unwrap();
    }

    #[test]
    fn feed_requires_enable_and_capture() {
        let (mut d, _) = make();
        // 未使能捕获：帧被丢弃
        assert_eq!(d.feed_frame(&[1, 2, 3, 4]), 0, "未使能捕获应丢弃帧");
        d.write(OFF_CR, 4, CR_ENABLE).unwrap(); // 仅使能、未 CAPTURE
        assert_eq!(d.feed_frame(&[1, 2, 3, 4]), 0, "未置 CAPTURE 应丢弃帧");
        enable_capture(&mut d);
        assert_eq!(d.feed_frame(&[1, 2, 3, 4]), 1, "使能+捕获后应接受一帧");
    }

    #[test]
    fn feed_sets_fne_and_frame() {
        let (mut d, _) = make();
        enable_capture(&mut d);
        d.feed_frame(&[0x11, 0x22, 0x33, 0x44, 0xAA, 0xBB]);
        // 6 字节 → 2 个 32 位字；尾字低 2 字节 [AA, BB] + 高位补零
        assert_eq!(d.feed_frame(&[]), 0);
        let sr = d.read(OFF_SR, 4).unwrap();
        assert_ne!(sr & SR_FNE, 0, "FIFO 非空应置 FNE");
        assert_ne!(sr & SR_FRAME, 0, "注入帧应置 FRAME");
        assert_ne!(sr & SR_LINE, 0, "注入帧应置 LINE");
    }

    #[test]
    fn read_dr_pops_words_and_clears_fne() {
        let (mut d, _) = make();
        enable_capture(&mut d);
        d.feed_frame(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
        // 字 0 = 0x44332211（低位在前）
        assert_eq!(d.read(OFF_DR, 4).unwrap(), 0x4433_2211);
        assert_ne!(d.read(OFF_SR, 4).unwrap() & SR_FNE, 0, "仍剩 1 字应保持 FNE");
        assert_eq!(d.read(OFF_DR, 4).unwrap(), 0x8877_6655);
        assert_eq!(d.read(OFF_SR, 4).unwrap() & SR_FNE, 0, "FIFO 空应清 FNE");
    }

    #[test]
    fn frame_irq_when_ier_enabled() {
        let (mut d, nvic) = make();
        enable_capture(&mut d);
        // 未使能中断：不挂起
        d.feed_frame(&[1, 2, 3, 4]);
        assert!(!nvic.lock().unwrap().is_pending(DCMI_IRQ), "未使能 IER 不应挂起 IRQ");
        d.write(OFF_IER, 4, INT_FRAME).unwrap();
        d.feed_frame(&[5, 6, 7, 8]);
        assert!(nvic.lock().unwrap().is_pending(DCMI_IRQ), "IER.FRAME + 帧应挂起 IRQ78");
    }

    #[test]
    fn icr_clears_ris_and_mis_masks() {
        let (mut d, _) = make();
        enable_capture(&mut d);
        d.write(OFF_IER, 4, INT_FRAME | INT_LINE).unwrap();
        d.feed_frame(&[1, 2, 3, 4]);
        let ris = d.read(OFF_RIS, 4).unwrap();
        assert_ne!(ris & INT_FRAME, 0, "注入帧应置 RIS.FRAME");
        // MIS = RIS & IER
        let mis = d.read(OFF_MIS, 4).unwrap();
        assert_ne!(mis & INT_FRAME, 0, "MIS 应呈现 RIS & IER 的帧位");
        assert_eq!(mis & !(INT_FRAME | INT_LINE), 0, "未发生的位不应出现在 MIS");
        // ICR 写 1 清除
        d.write(OFF_ICR, 4, INT_FRAME).unwrap();
        assert_eq!(d.read(OFF_RIS, 4).unwrap() & INT_FRAME, 0, "ICR 写 1 应清 RIS.FRAME");
        assert_eq!(d.read(OFF_MIS, 4).unwrap() & INT_FRAME, 0, "清除后 MIS 帧位应消失");
    }

    #[test]
    fn snapshot_mode_clears_capture() {
        let (mut d, _) = make();
        d.write(OFF_CR, 4, CR_ENABLE | CR_CAPTURE | CR_CM).unwrap(); // 快照模式
        d.feed_frame(&[1, 2, 3, 4]);
        assert_eq!(d.read(OFF_CR, 4).unwrap() & CR_CAPTURE, 0, "快照一帧后应自动清 CAPTURE");
        assert_eq!(d.feed_frame(&[1, 2, 3, 4]), 0, "CAPTURE 已清：下一帧被丢弃");
    }

    #[test]
    fn dma_read_dr_drains_fifo() {
        let (mut d, _) = make();
        enable_capture(&mut d);
        d.feed_frame(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
        assert_eq!(d.dma_read_dr(), 0x4433_2211);
        assert_eq!(d.dma_read_dr(), 0x8877_6655);
        assert_eq!(d.fne(), false, "DMA 排空 FIFO 后 FNE 应清");
    }
}
