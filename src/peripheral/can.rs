//! CAN1/2 控制器局域网（STM32F407，M15 虚拟外设生态）。
//!
//! bxCAN 简化模型：
//! - 寄存器集：MCR/MSR/TSR/RF0R/RF1R/IER/ESR/BTR + 3 发送邮箱 + 2 接收 FIFO
//!   + 28 硬件滤波器（列表模式简化）；
//! - 发送：写 TIxR 的 TXRQ（bit0）且邮箱空（TSR.TMEx=1）→ 立即完成：发布
//!   [`Event::CanFrame`]（总线级互联）→ 置 TSR.RQCP/TXOK/TME → IER.TMEIE 使能时
//!   挂起 TX IRQ（CAN1=19 / CAN2=63）；
//! - 接收：[`Can::feed_rx`]（测试注入 / 对端 CAN 收到 CanFrame 事件后路由）→
//!   过滤 → 入 FIFO0（3 槽，满则置 FOVR 溢出）→ FMP 递增 → IER.FMPIE0 使能时
//!   挂起 RX0 IRQ（CAN1=20 / CAN2=64）；
//! - 过滤：只实现"单个 32 位标识符列表模式"（FxR1 的 IDE/STID/EXID 与帧格式
//!   完全匹配）；FA1R 激活位=1 的滤波器才参与，全部未激活则放行所有帧；
//!   RTR 位不参与匹配（简化）；
//! - 错误管理：ESR 只读标志（EWGF/EPVF/BOFF/LEC）经 [`Can::inject_error`] 注入
//!   （模拟总线异常），IER.ERRIE 使能时挂起 SCE IRQ（CAN1=22 / CAN2=66）。
//!
//! 简化点（文档注明）：发送立即完成（无总线仲裁/位时序延迟）；TSR 为只读状态
//! （ABRQ/写清 RQCP 未实现）；接收恒入 FIFO0（FFA/FMR.FINIT 分配未实现，滤波器
//! 寄存器始终可写）；错误计数（TEC/REC）以标志注入代替逐位累加。
//!
//! 地址映射（CAN1 @ 0x40006400 / CAN2 @ 0x40006800，APB1）：
//! MCR 0x00 / MSR 0x04 / TSR 0x08 / RF0R 0x0C / RF1R 0x10 / IER 0x14 / ESR 0x18 /
//! BTR 0x1C；TX 邮箱 0x180/0x190/0x1A0；RX FIFO 0x1B0/0x1C0；
//! FMR 0x400 / FM1R 0x404 / FS1R 0x40C / FFA1R 0x414 / FA1R 0x41C / F0R1..F27R2 0x420..0x4FC

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use crate::events::{Event, EventBus};
use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};

/// CAN1 NVIC IRQ（STM32F407：TX=19 / RX0=20 / RX1=21 / SCE=22）
pub const CAN1_IRQ_TX: u32 = 19;
pub const CAN1_IRQ_RX0: u32 = 20;
pub const CAN1_IRQ_RX1: u32 = 21;
pub const CAN1_IRQ_SCE: u32 = 22;
/// CAN2 NVIC IRQ（STM32F407：TX=63 / RX0=64 / RX1=65 / SCE=66）
pub const CAN2_IRQ_TX: u32 = 63;
pub const CAN2_IRQ_RX0: u32 = 64;
pub const CAN2_IRQ_RX1: u32 = 65;
pub const CAN2_IRQ_SCE: u32 = 66;

/// CAN1 基地址（APB1）
pub const CAN1_BASE: u32 = 0x4000_6400;
/// CAN2 基地址（APB1）
pub const CAN2_BASE: u32 = 0x4000_6800;

/// 寄存器偏移
const OFF_MCR: u32 = 0x00;
const OFF_MSR: u32 = 0x04;
const OFF_TSR: u32 = 0x08;
const OFF_RF0R: u32 = 0x0C;
const OFF_RF1R: u32 = 0x10;
const OFF_IER: u32 = 0x14;
const OFF_ESR: u32 = 0x18;
const OFF_BTR: u32 = 0x1C;

/// 发送邮箱寄存器基址（3 邮箱 × 16 字节：TI/TDT/TDL/TDH）
const TX_MAILBOX_BASE: u32 = 0x180;
/// 接收 FIFO 寄存器基址（2 FIFO × 16 字节：RI/RDT/RDL/RDH）
const RX_FIFO_BASE: u32 = 0x1B0;
/// 滤波器模式/尺度/分配/激活寄存器（各 2 个 32 位字）
const OFF_FMR: u32 = 0x400;
const OFF_FM1R: u32 = 0x404;
const OFF_FS1R: u32 = 0x40C;
const OFF_FFA1R: u32 = 0x414;
const OFF_FA1R: u32 = 0x41C;
/// 滤波器数据起始（F0R1；每滤波器 8 字节 = 2 个 32 位字）
const FILTER_BASE: u32 = 0x420;
const FILTER_COUNT: usize = 28;
/// 接收 FIFO 深度（bxCAN 每 FIFO 3 邮箱）
const FIFO_DEPTH: usize = 3;

/// MCR 位
const MCR_INRQ: u32 = 1 << 0; // 初始化请求
const MCR_SLEEP: u32 = 1 << 1; // 睡眠模式请求
const MCR_TXFP: u32 = 1 << 2; // 发送 FIFO 优先级
const MCR_RFLM: u32 = 1 << 3; // FIFO 锁定模式
const MCR_NART: u32 = 1 << 4; // 禁止自动重传
const MCR_AWUM: u32 = 1 << 5; // 自动唤醒
const MCR_ABOM: u32 = 1 << 6; // 自动总线关闭管理
const MCR_TTCM: u32 = 1 << 7; // 时间触发通信
const MCR_RESET: u32 = 1 << 15; // 软件复位
const MCR_DBF: u32 = 1 << 16; // 调试冻结
const MCR_WR_MASK: u32 = MCR_INRQ | MCR_SLEEP | MCR_TXFP | MCR_RFLM | MCR_NART
    | MCR_AWUM | MCR_ABOM | MCR_TTCM | MCR_RESET | MCR_DBF;

/// MSR 位（只读状态）
const MSR_INAK: u32 = 1 << 0; // 初始化确认
// （SLAK 睡眠确认 / ERRI / WKUI / TXM / RXM 等只读状态位简化未实现，恒 0）

/// TSR 位（邮箱 m 完成/状态；只读状态机）
const TSR_RQCP: [u32; 3] = [1 << 0, 1 << 8, 1 << 16]; // 请求完成
const TSR_TXOK: [u32; 3] = [1 << 1, 1 << 9, 1 << 17]; // 发送成功
const TSR_TME: [u32; 3] = [1 << 26, 1 << 27, 1 << 28]; // 邮箱空
// （ALST 仲裁丢失 / TERR 发送错误 / LOW 最低优先级位简化未实现，恒 0）

/// RFxR 位
const RF_FMP_MASK: u32 = 0x3; // [1:0] FIFO 中消息数
const RF_FULL: u32 = 1 << 3; // FIFO 满
const RF_FOVR: u32 = 1 << 4; // FIFO 溢出
const RF_RFOM: u32 = 1 << 5; // 释放输出邮箱（写 1）

/// IER 位（FFIE/FOVIE/FMPIE1 等位简化未实现，仅存储回读不触发中断）
const IER_TMEIE: u32 = 1 << 0; // 邮箱空中断使能
const IER_FMPIE0: u32 = 1 << 1; // FIFO0 非空中断使能
const IER_ERRIE: u32 = 1 << 7; // 错误中断使能
const IER_WR_MASK: u32 = 0x1FF;

/// ESR 位（只读错误状态）
const ESR_EWGF: u32 = 1 << 0; // 错误警告标志（TEC/REC ≥ 96）
const ESR_EPVF: u32 = 1 << 1; // 错误被动标志（TEC > 127）
const ESR_BOFF: u32 = 1 << 2; // 总线关闭标志（TEC > 255）
const ESR_LEC_MASK: u32 = 0x7 << 3; // [5:3] 上次错误代码

/// BTR 位（位时序；简化仅存储回读，不参与时序）
const BTR_WR_MASK: u32 = 0xFFFF_FFFF;

/// 发送邮箱标识符寄存器位
const TI_TXRQ: u32 = 1 << 0; // 发送请求
const TI_RTR: u32 = 1 << 1; // 远程帧
const TI_IDE: u32 = 1 << 2; // 扩展帧
const TI_EXID_MASK: u32 = 0xFFFF_FFF8; // [31:3] 扩展 ID（29 位）
const TI_STID_MASK: u32 = 0xFFE0_0000; // [31:21] 标准 ID

/// 数据长度寄存器位
const TDT_DLC_MASK: u32 = 0xF << 16; // [19:16] 数据长度

/// 一帧 CAN 报文（总线级互联/注入载体）
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    /// 源 CAN 端口（1/2，仅事件发布用）
    pub port: u8,
    /// 标识符（标准帧 11 位 / 扩展帧 29 位）
    pub id: u32,
    /// 扩展帧（IDE=1）
    pub ext: bool,
    /// 远程帧（RTR=1）
    pub rtr: bool,
    /// 数据长度（0..=8）
    pub dlc: u8,
    /// 数据（有效字节数为 dlc）
    pub data: [u8; 8],
}

impl CanFrame {
    pub fn new(port: u8, id: u32, ext: bool, rtr: bool, dlc: u8, data: [u8; 8]) -> Self {
        Self { port, id, ext, rtr, dlc: dlc.min(8), data }
    }
}

/// 发送邮箱（TI/TDT/TDL/TDH 四个寄存器）
#[derive(Debug, Clone, Copy, Default)]
struct Mailbox {
    ti: u32,
    tdt: u32,
    dl: u32,
    dh: u32,
}

impl Mailbox {
    /// 从邮箱寄存器构造帧
    fn frame(&self, port: u8) -> CanFrame {
        let ext = self.ti & TI_IDE != 0;
        let rtr = self.ti & TI_RTR != 0;
        let id = if ext {
            (self.ti & TI_EXID_MASK) >> 3
        } else {
            (self.ti & TI_STID_MASK) >> 21
        };
        let dlc = ((self.tdt & TDT_DLC_MASK) >> 16) as u8;
        let mut data = [0u8; 8];
        data[0] = (self.dl & 0xFF) as u8;
        data[1] = ((self.dl >> 8) & 0xFF) as u8;
        data[2] = ((self.dl >> 16) & 0xFF) as u8;
        data[3] = ((self.dl >> 24) & 0xFF) as u8;
        data[4] = (self.dh & 0xFF) as u8;
        data[5] = ((self.dh >> 8) & 0xFF) as u8;
        data[6] = ((self.dh >> 16) & 0xFF) as u8;
        data[7] = ((self.dh >> 24) & 0xFF) as u8;
        CanFrame::new(port, id, ext, rtr, dlc, data)
    }
}

/// CAN 外设（bxCAN 简化模型）
pub struct Can {
    /// 端口（1/2）
    port: u8,
    /// 共享事件总线（发布 CanFrame / 无）
    events: Option<Arc<Mutex<EventBus>>>,
    /// 共享 NVIC（中断挂起）
    nvic: Arc<Mutex<Nvic>>,
    /// TX/RX0/SCE IRQ（RX1 简化与 RX0 共用映射，见 read/write）
    irq_tx: u32,
    irq_rx0: u32,
    irq_sce: u32,
    /// 控制/状态寄存器
    mcr: u32,
    msr: u32,
    tsr: u32,
    rf0r: u32,
    rf1r: u32,
    ier: u32,
    esr: u32,
    btr: u32,
    /// 3 个发送邮箱
    tx: [Mailbox; 3],
    /// 接收 FIFO0（3 槽；FFA 分配简化恒 FIFO0）
    rx0: VecDeque<CanFrame>,
    /// 接收 FIFO1（预留；本简化版本恒空）
    rx1: VecDeque<CanFrame>,
    /// 滤波器模式/尺度/分配/激活（各 2 字，位 x 对滤波器 x）
    fm: [u32; 2],
    fs: [u32; 2],
    ffa: [u32; 2],
    fa: [u32; 2],
    /// 滤波器数据（F0R1..F27R2，每滤波器 2 字）
    filters: [[u32; 2]; FILTER_COUNT],
}

impl Can {
    pub fn new(port: u8, events: Option<Arc<Mutex<EventBus>>>, nvic: Arc<Mutex<Nvic>>) -> Self {
        let (irq_tx, irq_rx0, irq_sce) = if port == 2 {
            (CAN2_IRQ_TX, CAN2_IRQ_RX0, CAN2_IRQ_SCE)
        } else {
            (CAN1_IRQ_TX, CAN1_IRQ_RX0, CAN1_IRQ_SCE)
        };
        Self {
            port,
            events,
            nvic,
            irq_tx,
            irq_rx0,
            irq_sce,
            mcr: 0,
            msr: 0,
            tsr: TSR_TME[0] | TSR_TME[1] | TSR_TME[2], // 初始 3 邮箱空
            rf0r: 0,
            rf1r: 0,
            ier: 0,
            esr: 0,
            btr: 0,
            tx: [Mailbox::default(); 3],
            rx0: VecDeque::new(),
            rx1: VecDeque::new(),
            fm: [0; 2],
            fs: [0; 2],
            ffa: [0; 2],
            fa: [0; 2],
            filters: [[0; 2]; FILTER_COUNT],
        }
    }

    /// 注入一帧（测试直接注入 / 对端 CAN 收到 CanFrame 事件后路由）。
    /// 过滤通过 → 入 FIFO0（满置溢出）→ FMP 递增 → FMPIE0 使能时挂起 RX0 IRQ。
    pub fn feed_rx(&mut self, frame: CanFrame) {
        if !self.filter_pass(&frame) {
            return; // 未匹配任何激活滤波器 → 丢弃
        }
        if self.rx0.len() >= FIFO_DEPTH {
            // FIFO0 满：锁定模式（RFLM=1）丢弃新帧，否则覆盖最旧帧并置溢出
            if self.mcr & MCR_RFLM != 0 {
                self.rf0r |= RF_FOVR;
                return;
            }
            self.rx0.pop_front();
            self.rf0r |= RF_FOVR;
        }
        self.rx0.push_back(frame);
        self.rf0r = (self.rf0r & !RF_FMP_MASK) | (self.rx0.len() as u32 & RF_FMP_MASK);
        if self.rx0.len() == FIFO_DEPTH {
            self.rf0r |= RF_FULL;
        }
        if self.ier & IER_FMPIE0 != 0 {
            self.nvic.lock().unwrap().set_pending(self.irq_rx0);
        }
    }

    /// 注入错误状态（模拟总线异常；on=true 置位，false 清除）。
    /// EWGF/EPVF/BOFF 互斥推进；ERRIE 使能时挂起 SCE IRQ。
    pub fn inject_error(&mut self, ewgf: bool, epvf: bool, boff: bool) {
        self.esr &= !(ESR_EWGF | ESR_EPVF | ESR_BOFF | ESR_LEC_MASK);
        if boff {
            self.esr |= ESR_BOFF | (5 << 3); // LEC=5（位错误）
        } else if epvf {
            self.esr |= ESR_EPVF | (4 << 3); // LEC=4（帧格式）
        } else if ewgf {
            self.esr |= ESR_EWGF | (1 << 3); // LEC=1（位填充）
        }
        if self.ier & IER_ERRIE != 0 {
            self.nvic.lock().unwrap().set_pending(self.irq_sce);
        }
    }

    /// 过滤：帧 ID 与任一激活滤波器（列表模式 32 位）完全匹配。
    /// 所有滤波器未激活（FA1R=0）→ 放行（全收）。
    fn filter_pass(&self, frame: &CanFrame) -> bool {
        if self.fa[0] == 0 && self.fa[1] == 0 {
            return true;
        }
        for i in 0..FILTER_COUNT {
            let word = i / 32;
            let bit = i % 32;
            if self.fa[word] & (1 << bit) == 0 {
                continue;
            }
            let r1 = self.filters[i][0];
            // 列表模式：FxR1 的 IDE 位决定标准/扩展，ID 部分需与帧完全一致
            let f_ext = r1 & TI_IDE != 0;
            if f_ext != frame.ext {
                continue;
            }
            let f_id = if f_ext {
                (r1 & TI_EXID_MASK) >> 3
            } else {
                (r1 & TI_STID_MASK) >> 21
            };
            if f_id == frame.id {
                return true;
            }
        }
        false
    }

    /// 发送邮箱 m：写 TIxR 且 TXRQ=1、邮箱空 → 立即完成
    fn transmit(&mut self, m: usize) {
        if self.tsr & TSR_TME[m] == 0 {
            return; // 邮箱忙（上一帧未完成）
        }
        let frame = self.tx[m].frame(self.port);
        // 占用邮箱 → 发布帧 → 立即完成（简化：无仲裁延迟）
        self.tsr &= !TSR_TME[m];
        if let Some(ev) = &self.events {
            ev.lock()
                .unwrap()
                .publish(&Event::CanFrame { frame: Box::new(frame) });
        }
        // 清发送请求并置完成/成功/邮箱空
        self.tx[m].ti &= !TI_TXRQ;
        self.tsr |= TSR_RQCP[m] | TSR_TXOK[m] | TSR_TME[m];
        if self.ier & IER_TMEIE != 0 {
            self.nvic.lock().unwrap().set_pending(self.irq_tx);
        }
    }

    /// 释放 FIFO0 最旧帧（写 RFOM0）
    fn release_fifo0(&mut self) {
        self.rx0.pop_front();
        self.rf0r = (self.rf0r & !RF_FMP_MASK) | (self.rx0.len() as u32 & RF_FMP_MASK);
        if self.rx0.len() < FIFO_DEPTH {
            self.rf0r &= !RF_FULL;
        }
    }

    /// 读取接收 FIFO0 的第 reg 个寄存器（RI/RDT/RDL/RDH）
    fn read_rx0_reg(&self, reg: u32) -> u32 {
        let Some(f) = self.rx0.front() else {
            return 0;
        };
        match reg {
            0 => {
                // RI0R：ID + 格式位（TIME 恒 0）
                let mut v = 0;
                if f.ext {
                    v |= TI_IDE | (f.id << 3);
                } else {
                    v |= f.id << 21;
                }
                if f.rtr {
                    v |= TI_RTR;
                }
                v
            }
            1 => (f.dlc as u32) << 16, // RDT0R：DLC（FMI/TIME 简化 0）
            2 => {
                // RDL0R：DATA0-3
                let mut v = 0;
                for (i, b) in f.data[0..4].iter().enumerate() {
                    v |= (*b as u32) << (8 * i);
                }
                v
            }
            3 => {
                // RDH0R：DATA4-7
                let mut v = 0;
                for (i, b) in f.data[4..8].iter().enumerate() {
                    v |= (*b as u32) << (8 * i);
                }
                v
            }
            _ => 0,
        }
    }

    /// 滤波器数据寄存器读取（offset 0x420..0x4FC）
    fn read_filter(&self, offset: u32) -> u32 {
        let idx = ((offset - FILTER_BASE) / 4) as usize;
        if idx < FILTER_COUNT * 2 {
            self.filters[idx / 2][idx % 2]
        } else {
            0
        }
    }

    fn write_filter(&mut self, offset: u32, value: u32) {
        let idx = ((offset - FILTER_BASE) / 4) as usize;
        if idx < FILTER_COUNT * 2 {
            self.filters[idx / 2][idx % 2] = value;
        }
    }
}

impl Peripheral for Can {
    fn name(&self) -> &str {
        if self.port == 2 {
            "CAN2"
        } else {
            "CAN1"
        }
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_MCR => Ok(self.mcr),
            OFF_MSR => Ok(self.msr),
            OFF_TSR => Ok(self.tsr),
            OFF_RF0R => Ok(self.rf0r),
            OFF_RF1R => Ok(self.rf1r),
            OFF_IER => Ok(self.ier),
            OFF_ESR => Ok(self.esr),
            OFF_BTR => Ok(self.btr),
            o if o >= TX_MAILBOX_BASE && o < TX_MAILBOX_BASE + 3 * 16 => {
                let m = ((o - TX_MAILBOX_BASE) / 16) as usize;
                let r = ((o - TX_MAILBOX_BASE) % 16) / 4;
                Ok(match r {
                    0 => self.tx[m].ti,
                    1 => self.tx[m].tdt,
                    2 => self.tx[m].dl,
                    _ => self.tx[m].dh,
                })
            }
            o if o >= RX_FIFO_BASE && o < RX_FIFO_BASE + 2 * 16 => {
                let fifo = (o - RX_FIFO_BASE) / 16;
                let reg = ((o - RX_FIFO_BASE) % 16) / 4;
                if fifo == 0 {
                    Ok(self.read_rx0_reg(reg))
                } else {
                    Ok(0) // FIFO1 简化恒空
                }
            }
            OFF_FMR => Ok(0), // FMR.FINIT 简化：始终可配，回读 0
            OFF_FM1R => Ok(self.fm[0] | (self.fm[1] << 16)),
            OFF_FS1R => Ok(self.fs[0] | (self.fs[1] << 16)),
            OFF_FFA1R => Ok(self.ffa[0] | (self.ffa[1] << 16)),
            OFF_FA1R => Ok(self.fa[0] | (self.fa[1] << 16)),
            o if o >= FILTER_BASE && o < FILTER_BASE + FILTER_COUNT as u32 * 8 => {
                Ok(self.read_filter(o))
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_MCR => {
                let old_inrq = self.mcr & MCR_INRQ;
                self.mcr = value & MCR_WR_MASK;
                if self.mcr & MCR_RESET != 0 {
                    // 软件复位：回到默认状态
                    self.reset();
                    self.mcr &= !MCR_RESET;
                    return Ok(());
                }
                // 简化：INRQ 请求立即确认（INAK 同步反映）
                let new_inrq = self.mcr & MCR_INRQ;
                if new_inrq != 0 && old_inrq == 0 {
                    self.msr |= MSR_INAK;
                } else if new_inrq == 0 && old_inrq != 0 {
                    self.msr &= !MSR_INAK;
                }
                Ok(())
            }
            OFF_MSR => Ok(()), // 只读：写忽略
            OFF_TSR => Ok(()), // 只读状态机：写忽略（ABRQ 简化未实现）
            OFF_RF0R => {
                if value & RF_RFOM != 0 {
                    self.release_fifo0();
                }
                Ok(())
            }
            OFF_RF1R => {
                if value & RF_RFOM != 0 {
                    self.rx1.pop_front();
                    self.rf1r = (self.rf1r & !RF_FMP_MASK) | (self.rx1.len() as u32 & RF_FMP_MASK);
                }
                Ok(())
            }
            OFF_IER => {
                self.ier = value & IER_WR_MASK;
                Ok(())
            }
            OFF_ESR => Ok(()), // 只读错误状态：写忽略
            OFF_BTR => {
                self.btr = value & BTR_WR_MASK;
                Ok(())
            }
            o if o >= TX_MAILBOX_BASE && o < TX_MAILBOX_BASE + 3 * 16 => {
                let m = ((o - TX_MAILBOX_BASE) / 16) as usize;
                let r = ((o - TX_MAILBOX_BASE) % 16) / 4;
                match r {
                    0 => {
                        // TIxR：TXRQ=1 且邮箱空 → 触发发送；请求位写后自动清
                        self.tx[m].ti = value & !TI_TXRQ;
                        if value & TI_TXRQ != 0 {
                            self.tx[m].ti |= TI_TXRQ;
                            self.transmit(m);
                        }
                    }
                    1 => self.tx[m].tdt = value,
                    2 => self.tx[m].dl = value,
                    _ => self.tx[m].dh = value,
                }
                Ok(())
            }
            o if o >= RX_FIFO_BASE && o < RX_FIFO_BASE + 2 * 16 => Ok(()), // 接收寄存器只读
            OFF_FMR => Ok(()), // 简化：FINIT 忽略（过滤器始终可配）
            OFF_FM1R => {
                self.fm[0] = value & 0xFFFF;
                self.fm[1] = value >> 16;
                Ok(())
            }
            OFF_FS1R => {
                self.fs[0] = value & 0xFFFF;
                self.fs[1] = value >> 16;
                Ok(())
            }
            OFF_FFA1R => {
                self.ffa[0] = value & 0xFFFF;
                self.ffa[1] = value >> 16;
                Ok(())
            }
            OFF_FA1R => {
                self.fa[0] = value & 0xFFFF;
                self.fa[1] = value >> 16;
                Ok(())
            }
            o if o >= FILTER_BASE && o < FILTER_BASE + FILTER_COUNT as u32 * 8 => {
                self.write_filter(o, value);
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.mcr = 0;
        self.msr = 0;
        self.tsr = TSR_TME[0] | TSR_TME[1] | TSR_TME[2];
        self.rf0r = 0;
        self.rf1r = 0;
        self.ier = 0;
        self.esr = 0;
        self.btr = 0;
        self.tx = [Mailbox::default(); 3];
        self.rx0.clear();
        self.rx1.clear();
        self.fm = [0; 2];
        self.fs = [0; 2];
        self.ffa = [0; 2];
        self.fa = [0; 2];
        self.filters = [[0; 2]; FILTER_COUNT];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::nvic::Nvic;

    fn make() -> (Can, Arc<Mutex<Nvic>>, Arc<Mutex<EventBus>>) {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let events = Arc::new(Mutex::new(EventBus::new()));
        (Can::new(1, Some(events.clone()), nvic.clone()), nvic, events)
    }

    fn frame(id: u32, ext: bool, dlc: u8, data: [u8; 8]) -> CanFrame {
        CanFrame::new(1, id, ext, false, dlc, data)
    }

    #[test]
    fn transmit_publishes_frame_and_sets_tsr() {
        let (mut c, nvic, events) = make();
        c.write(OFF_IER, 4, IER_TMEIE).unwrap();
        let got: Arc<Mutex<Option<CanFrame>>> = Arc::new(Mutex::new(None));
        let g = got.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::CanFrame { frame } = ev {
                    *g.lock().unwrap() = Some((**frame).clone());
                }
            },
        )));
        // 配置邮箱0：标准帧 ID=0x123，DLC=2，数据 0x11 0x22
        c.write(TX_MAILBOX_BASE + 0, 4, 0x123 << 21 | TI_TXRQ).unwrap();
        c.write(TX_MAILBOX_BASE + 4, 4, 2 << 16).unwrap();
        c.write(TX_MAILBOX_BASE + 8, 4, 0x22_11).unwrap();
        c.write(TX_MAILBOX_BASE + 0, 4, 0x123 << 21 | TI_TXRQ).unwrap(); // 再次写触发发送
        assert_eq!(c.read(OFF_TSR, 4).unwrap() & TSR_TXOK[0], TSR_TXOK[0], "应置 TXOK");
        assert_eq!(c.read(OFF_TSR, 4).unwrap() & TSR_TME[0], TSR_TME[0], "完成后邮箱空");
        assert!(nvic.lock().unwrap().is_pending(CAN1_IRQ_TX), "TMEIE 应挂起 TX IRQ");
        let f = got.lock().unwrap().clone().expect("应发布 CanFrame");
        assert_eq!(f.id, 0x123);
        assert_eq!(f.dlc, 2);
        assert_eq!(&f.data[0..2], &[0x11, 0x22]);
    }

    #[test]
    fn feed_rx_matches_filter_and_sets_fmp() {
        let (mut c, nvic, _) = make();
        // 激活滤波器0（列表模式，标准帧 ID=0x456）
        c.write(OFF_FA1R, 4, 0x1).unwrap();
        c.write(FILTER_BASE, 4, 0x456 << 21).unwrap(); // F0R1
        c.write(OFF_IER, 4, IER_FMPIE0).unwrap();
        // 匹配帧 → 接收
        c.feed_rx(frame(0x456, false, 1, [0xAA; 8]));
        assert_eq!(c.read(OFF_RF0R, 4).unwrap() & RF_FMP_MASK, 1, "FMP 应为 1");
        assert!(nvic.lock().unwrap().is_pending(CAN1_IRQ_RX0), "FMPIE0 应挂起 RX0 IRQ");
        // 不匹配帧 → 过滤丢弃
        c.feed_rx(frame(0x999, false, 1, [0xBB; 8]));
        assert_eq!(c.read(OFF_RF0R, 4).unwrap() & RF_FMP_MASK, 1, "不匹配帧不应入 FIFO");
    }

    #[test]
    fn fifo_full_sets_overflow_and_release() {
        let (mut c, _, _) = make();
        // 全收模式（无激活滤波器）
        for i in 0..FIFO_DEPTH {
            c.feed_rx(frame(100 + i as u32, false, 1, [i as u8; 8]));
        }
        assert_ne!(c.read(OFF_RF0R, 4).unwrap() & RF_FULL, 0, "FIFO 满应置 FULL");
        // 第 4 帧 → 溢出（非锁定模式覆盖最旧）
        c.feed_rx(frame(999, false, 1, [0xEE; 8]));
        assert_ne!(c.read(OFF_RF0R, 4).unwrap() & RF_FOVR, 0, "满后再收应置 FOVR");
        assert_eq!(c.read(OFF_RF0R, 4).unwrap() & RF_FMP_MASK, FIFO_DEPTH as u32);
        // 读最旧帧应为 101（100 被覆盖）
        assert_eq!(c.read(RX_FIFO_BASE + 0, 4).unwrap() >> 21, 101);
        // RFOM 释放 → FMP 递减
        c.write(OFF_RF0R, 4, RF_RFOM).unwrap();
        assert_eq!(c.read(OFF_RF0R, 4).unwrap() & RF_FMP_MASK, FIFO_DEPTH as u32 - 1);
    }

    #[test]
    fn read_rx_frame_fields() {
        let (mut c, _, _) = make();
        c.feed_rx(frame(0x123, false, 4, [1, 2, 3, 4, 0, 0, 0, 0]));
        assert_eq!(c.read(RX_FIFO_BASE + 0, 4).unwrap(), 0x123 << 21, "RI0R 标准 ID");
        assert_eq!(c.read(RX_FIFO_BASE + 4, 4).unwrap(), 4 << 16, "RDT0R DLC");
        assert_eq!(c.read(RX_FIFO_BASE + 8, 4).unwrap(), 0x0403_0201, "RDL0R 数据低 4 字节");
    }

    #[test]
    fn inject_error_sets_esr_and_irq() {
        let (mut c, nvic, _) = make();
        c.write(OFF_IER, 4, IER_ERRIE).unwrap();
        c.inject_error(false, false, true); // 总线关闭
        let esr = c.read(OFF_ESR, 4).unwrap();
        assert_ne!(esr & ESR_BOFF, 0, "应置 BOFF");
        assert!(nvic.lock().unwrap().is_pending(CAN1_IRQ_SCE), "ERRIE 应挂起 SCE IRQ");
    }

    #[test]
    fn init_mode_reflects_inrq() {
        let (mut c, _, _) = make();
        c.write(OFF_MCR, 4, MCR_INRQ).unwrap();
        assert_ne!(c.read(OFF_MSR, 4).unwrap() & MSR_INAK, 0, "INRQ 应确认 INAK");
        c.write(OFF_MCR, 4, 0).unwrap();
        assert_eq!(c.read(OFF_MSR, 4).unwrap() & MSR_INAK, 0, "退出初始化应清 INAK");
    }

    #[test]
    fn ext_frame_transmit_and_receive() {
        let (mut c, _, events) = make();
        let got: Arc<Mutex<Option<CanFrame>>> = Arc::new(Mutex::new(None));
        let g = got.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::CanFrame { frame } = ev {
                    *g.lock().unwrap() = Some((**frame).clone());
                }
            },
        )));
        // 扩展帧 ID=0x1FFEDCBA，DLC=8
        c.write(TX_MAILBOX_BASE + 0, 4, TI_IDE | (0x1FFEDCBA << 3) | TI_TXRQ).unwrap();
        c.write(TX_MAILBOX_BASE + 4, 4, 8 << 16).unwrap();
        c.write(TX_MAILBOX_BASE + 8, 4, 0x8877_6655).unwrap();
        c.write(TX_MAILBOX_BASE + 12, 4, 0x4433_2211).unwrap();
        c.write(TX_MAILBOX_BASE + 0, 4, TI_IDE | (0x1FFEDCBA << 3) | TI_TXRQ).unwrap();
        let f = got.lock().unwrap().clone().expect("应发布扩展帧");
        assert!(f.ext);
        assert_eq!(f.id, 0x1FFEDCBA);
        assert_eq!(f.dlc, 8);
        assert_eq!(f.data, [0x55, 0x66, 0x77, 0x88, 0x11, 0x22, 0x33, 0x44]);
    }
}
