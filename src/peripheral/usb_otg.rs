//! USB OTG FS 全速 USB 设备控制器（STM32F407，M16 虚拟外设生态）。
//!
//! 设备模式简化模型：
//! - 寄存器集：全局（GCCFG/GUSBCFG/GAHBCFG/GRSTCTL/GINTSTS/GINTMSK/GRXSTSR/SP/
//!   GRXFSIZ/GNPTXFSIZ/DIEPTXF0..3）+ 设备（DCFG/DCTL/DSTS/DIEPMSK/DOEPMSK/
//!   DAINT/DAINTMSK）+ 端点（DIEPCTLx/DOEPCTLx/DIEPINTx/DOEPINTx/DIEPTSIZx/
//!   DOEPTSIZx，x=0..3）+ 数据 FIFO（DFIFO0..3 @ 0x1000+0x1000·x）。
//!   主寄存器统一存 `regs: [u32; 0x300]`（0x000..0xBFF，按偏移/4 索引），
//!   FIFO 区（0x1000..0x5000）单独映射为收/发数据缓冲；
//! - 虚拟主机注入（[`UsbOtg::inject_*`]，模拟主机侧枚举/传输驱动）：
//!   - [`UsbOtg::inject_usb_reset`]：总线复位 → GINTSTS.USBRST + ENUMDNE 置位；
//!   - [`UsbOtg::inject_setup`]：SETUP 包（8 字节）→ 数据入接收 FIFO +
//!     GRXSTSP 状态字（SETUP_DATA + SETUP_COMP）+ DOEPINT0.STUP + GINTSTS.RXFLVL；
//!   - [`UsbOtg::inject_out`]：OUT 数据包 → 数据入接收 FIFO + GRXSTSP 状态字
//!     （OUT_DATA）+ DOEPINTx.XFRC + GINTSTS.RXFLVL；
//! - IN 发送：固件写 DFIFOx（每字 4 字节入发送缓冲）→ 写 DIEPCTLx 置 EPENA
//!   → 满足 DIEPTSIZx.XFRSIZ 时传输完成（DIEPINTx.XFRC）。虚拟主机经
//!   [`UsbOtg::host_take_in`] 取走设备发出的数据；
//! - 中断：OTG_FS_IRQ=67。挂起由 GINTMSK（RXFLVL/USBRST/ENUMDNE）与
//!   DAINTMSK × DIEPMSK/DOEPMSK（XFRC/STUP）门控。
//!
//! 简化点（文档注明）：不模拟逐 bit 包/CRC/PHY 时序；IN 传输在写 EPENA 后
//! 立即完成（无传输延迟）；接收 FIFO 深度不限制（GRXFSIZ 仅存储回读）；端点
//! 数量取 FS 的 4 IN + 4 OUT；主机枚举细节由注入方法 + 验收固件协作完成。
//!
//! 地址映射（USB OTG FS @ 0x50000000，AHB1）：
//! GCCFG 0x038 / GUSBCFG 0x00C / GAHBCFG 0x008 / GINTSTS 0x014 / GINTMSK 0x018 /
//! GRXSTSR 0x01C / GRXSTSP 0x020 / GRXFSIZ 0x024 / GNPTXFSIZ 0x028 /
//! DIEPTXF0 0x104 / DCFG 0x800 / DCTL 0x804 / DSTS 0x808 / DIEPMSK 0x810 /
//! DOEPMSK 0x814 / DAINT 0x818 / DAINTMSK 0x81C / DIEPCTLx 0x900+0x20x /
//! DIEPINTx 0x908+0x20x / DIEPTSIZx 0x910+0x20x / DOEPCTLx 0xB00+0x20x /
//! DOEPINTx 0xB08+0x20x / DOEPTSIZx 0xB10+0x20x / DFIFOx 0x1000+0x1000x

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use crate::events::EventBus;
use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};

/// USB OTG FS 基地址（AHB1，USB_OTG_FS）
pub const USB_OTG_FS_BASE: u32 = 0x5000_0000;
/// OTG_FS NVIC IRQ（STM32F407）
pub const USB_OTG_FS_IRQ: u32 = 67;
/// OTG_FS 唤醒 IRQ（STM32F407；简化：仅注入挂起验证）
pub const USB_OTG_FS_WKUP_IRQ: u32 = 42;

/// 主寄存器数组长度（覆盖 0x000..0xBFF）
const REG_COUNT: usize = 0x300;
/// 端点数量（FS：4 IN + 4 OUT）
const EP_COUNT: usize = 4;
/// FIFO 数据区偏移（DFIFO0）
const OFF_DFIFO0: u32 = 0x1000;
/// FIFO 数据区步长
const FIFO_STRIDE: u32 = 0x1000;

// ---- 全局寄存器偏移 ----
const OFF_GAHBCFG: u32 = 0x008;
const OFF_GUSBCFG: u32 = 0x00C;
const OFF_GRSTCTL: u32 = 0x010;
const OFF_GINTSTS: u32 = 0x014;
const OFF_GINTMSK: u32 = 0x018;
const OFF_GRXSTSR: u32 = 0x01C;
const OFF_GRXSTSP: u32 = 0x020;
const OFF_GCCFG: u32 = 0x038;

// ---- 设备模式寄存器偏移 ----
const OFF_DCFG: u32 = 0x800;
const OFF_DSTS: u32 = 0x808;
const OFF_DIEPMSK: u32 = 0x810;
const OFF_DOEPMSK: u32 = 0x814;
const OFF_DAINTMSK: u32 = 0x81C;

/// IN 端点寄存器基址（DIEPCTLx / DIEPINTx / DIEPTSIZx）
const DIEP_BASE: u32 = 0x900;
/// OUT 端点寄存器基址（DOEPCTLx / DOEPINTx / DOEPTSIZx）
const DOEP_BASE: u32 = 0xB00;
/// 端点寄存器步长
const EP_STRIDE: u32 = 0x20;
/// 端点内寄存器子偏移
const EP_CTL: u32 = 0x00;
const EP_INT: u32 = 0x08;
const EP_TSIZ: u32 = 0x10;

// ---- 位定义 ----
/// GCCFG：掉电位（复位后置 1，清 0 上电 PHY）
const GCCFG_PWRDWN: u32 = 1 << 16;
/// GUSBCFG：强制设备模式 / PHY 选择（FS 内置）
const GUSBCFG_FDMOD: u32 = 1 << 30;
const GUSBCFG_PHYSEL: u32 = 1 << 6;
/// GAHBCFG：全局中断使能
const GAHBCFG_GINT: u32 = 1 << 0;
/// GRSTCTL：核心软复位（写 1 触发，仿真器立即完成）
const GRSTCTL_CSRST: u32 = 1 << 0;
const GRSTCTL_AHBIDL: u32 = 1 << 31;
/// GINTSTS/GINTMSK（设备模式相关位；SOF/挂起等简化未实现）
const GINT_RXFLVL: u32 = 1 << 4; // 接收 FIFO 非空
const GINT_USBRST: u32 = 1 << 12; // USB 复位
const GINT_ENUMDNE: u32 = 1 << 13; // 枚举完成
const GINT_WKUP: u32 = 1 << 31; // 唤醒（注入用）
const GINT_RW_MASK: u32 =
    GINT_RXFLVL | GINT_USBRST | GINT_ENUMDNE | GINT_WKUP;
/// GRXSTSP 字段
const RXS_EPNUM: u32 = 0xF; // [3:0] 端点号
const RXS_BCNT_MASK: u32 = 0x7FF << 4; // [14:4] 字节数
const RXS_PKTSTS_MASK: u32 = 0xF << 17; // [20:17] 包状态
/// 包状态（PKTSTS；GOUT_NAK/IN_COMP 简化未使用）
const PKTSTS_SETUP_COMP: u32 = 2; // SETUP 完成
const PKTSTS_SETUP_DATA: u32 = 3; // SETUP 数据
const PKTSTS_OUT_DATA: u32 = 4; // OUT 数据
/// DCFG：设备地址
const DCFG_DAD: u32 = 0x7F << 4;
/// DSTS（只读）：枚举速度复位值=FS（0b10）
const DSTS_RESET: u32 = 0x2;
/// DIEPMSK / DOEPMSK
const DIEPMSK_XFRCM: u32 = 1 << 0; // IN 传输完成
const DOEPMSK_XFRCM: u32 = 1 << 0; // OUT 传输完成
const DOEPMSK_STUPM: u32 = 1 << 3; // SETUP 完成
/// DAINT/DAINTMSK：IN 端点 [3:0]，OUT 端点 [19:16]
const fn daint_iep(n: usize) -> u32 {
    1 << n
}
const fn daint_oep(n: usize) -> u32 {
    1 << (16 + n)
}
/// DIEPCTLx / DOEPCTLx（EPTYP/STALL/TXFNUM/CNAK/SNAK/MPSIZ 等简化仅随值回读）
const EP_EPENA: u32 = 1 << 31; // 端点使能
/// DIEPTSIZx / DOEPTSIZx（PKTCNT/STUPCNT 简化仅随值回读）
const TSIZ_XFRSIZ: u32 = 0x7_FFFF; // [18:0] 传输大小（字节）
/// DIEPINTx / DOEPINTx（写 1 清除）
const EPINT_XFRC: u32 = 1 << 0; // 传输完成
const EPINT_STUP: u32 = 1 << 3; // SETUP 完成（OUT EP0）

/// USB OTG FS 外设（设备模式简化模型）
pub struct UsbOtg {
    /// 共享事件总线（发布 UsbSetup / 无）
    events: Option<Arc<Mutex<EventBus>>>,
    /// 共享 NVIC（中断挂起）
    nvic: Arc<Mutex<Nvic>>,
    /// 主寄存器（0x000..0xBFF，偏移/4 索引）
    regs: [u32; REG_COUNT],
    /// 接收 FIFO 数据（SETUP/OUT 数据，读 DFIFO0 弹出）
    rx: VecDeque<u8>,
    /// 接收 FIFO 状态队列（读 GRXSTSP 弹出）
    rx_status: VecDeque<u32>,
    /// IN 发送缓冲（写 DFIFOx 追加；host_take_in 弹出）
    tx: [Vec<u8>; EP_COUNT],
    /// 各 IN 端点是否已触发完成（写 EPENA/写 FIFO 时检查）
    in_pending: [bool; EP_COUNT],
}

impl UsbOtg {
    pub fn new(events: Option<Arc<Mutex<EventBus>>>, nvic: Arc<Mutex<Nvic>>) -> Self {
        let mut regs = [0u32; REG_COUNT];
        regs[(OFF_GCCFG >> 2) as usize] = GCCFG_PWRDWN; // 复位默认掉电
        regs[(OFF_DSTS >> 2) as usize] = DSTS_RESET; // ENUMSPD=FS
        Self {
            events,
            nvic,
            regs,
            rx: VecDeque::new(),
            rx_status: VecDeque::new(),
            tx: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
            in_pending: [false; EP_COUNT],
        }
    }

    /// 注入 USB 总线复位（虚拟主机复位设备）。
    /// GINTSTS.USBRST 置位（USBRST 中断）；随后 ENUMDNE 置位（枚举完成中断，
    /// 仿真器简化：复位后立即完成枚举）。设备地址清 0、非 EP0 端点失活。
    pub fn inject_usb_reset(&mut self) {
        // 复位：地址归 0，失活非 0 端点
        self.regs[(OFF_DCFG >> 2) as usize] &= !DCFG_DAD;
        for ep in 1..EP_COUNT {
            self.diep_ctl(ep, 0);
            self.doep_ctl(ep, 0);
        }
        self.set_gint(GINT_USBRST);
        self.set_gint(GINT_ENUMDNE);
        self.pulse();
    }

    /// 注入 SETUP 包（控制传输第一阶段；8 字节标准请求）。
    /// 数据入接收 FIFO + GRXSTSP 状态（SETUP_DATA + SETUP_COMP）；
    /// DOEPINT0.STUP 置位（STUPM 门控）+ GINTSTS.RXFLVL（RXFLVLM 门控）。
    pub fn inject_setup(&mut self, data: [u8; 8]) {
        // 数据入接收 FIFO
        for b in data {
            self.rx.push_back(b);
        }
        // SETUP 数据状态字
        self.rx_status.push_back(Self::rx_status(0, 8, PKTSTS_SETUP_DATA));
        // SETUP 完成状态字
        self.rx_status.push_back(Self::rx_status(0, 0, PKTSTS_SETUP_COMP));
        // DOEPINT0.STUP
        let int_off = Self::doep_int_off(0);
        self.regs[(int_off >> 2) as usize] |= EPINT_STUP;
        self.set_gint(GINT_RXFLVL);
        self.pulse();
    }

    /// 注入 OUT 数据包（批量/控制数据阶段；简化单包完成）。
    /// 数据入接收 FIFO + GRXSTSP 状态（OUT_DATA）；DOEPINTx.XFRC 置位
    /// （XFRCM 门控）+ GINTSTS.RXFLVL。
    pub fn inject_out(&mut self, ep: usize, data: &[u8]) {
        let ep = ep.min(EP_COUNT - 1);
        for b in data {
            self.rx.push_back(*b);
        }
        self.rx_status
            .push_back(Self::rx_status(ep as u32, data.len() as u32, PKTSTS_OUT_DATA));
        self.regs[(Self::doep_int_off(ep) >> 2) as usize] |= EPINT_XFRC;
        self.set_gint(GINT_RXFLVL);
        self.pulse();
    }

    /// 注入唤醒（GINTSTS.WKUP，WKUP 门控；用于 WKUP IRQ42 验证）。
    pub fn inject_wakeup(&mut self) {
        self.set_gint(GINT_WKUP);
        self.pulse();
    }

    /// 虚拟主机取走设备已发送的 IN 数据（并清缓冲）。
    pub fn host_take_in(&mut self, ep: usize) -> Vec<u8> {
        let ep = ep.min(EP_COUNT - 1);
        std::mem::take(&mut self.tx[ep])
    }

    /// 读 GINTSTS 的某些位（供测试断言）
    pub fn gintsts(&self) -> u32 {
        self.regs[(OFF_GINTSTS >> 2) as usize]
    }

    /// 置位 GINTSTS（内部；只写 1 由寄存器写路径清除）
    fn set_gint(&mut self, bits: u32) {
        self.regs[(OFF_GINTSTS >> 2) as usize] |= bits & GINT_RW_MASK;
    }

    /// 构造 GRXSTSP 状态字
    fn rx_status(epnum: u32, bcnt: u32, pktsts: u32) -> u32 {
        (epnum & RXS_EPNUM)
            | ((bcnt << 4) & RXS_BCNT_MASK)
            | ((pktsts << 17) & RXS_PKTSTS_MASK)
    }

    /// IN 端点控制寄存器偏移
    fn diep_ctl_off(ep: usize) -> u32 {
        DIEP_BASE + EP_STRIDE * ep as u32 + EP_CTL
    }
    /// IN 端点中断寄存器偏移
    fn diep_int_off(ep: usize) -> u32 {
        DIEP_BASE + EP_STRIDE * ep as u32 + EP_INT
    }
    /// IN 端点大小寄存器偏移
    fn diep_tsiz_off(ep: usize) -> u32 {
        DIEP_BASE + EP_STRIDE * ep as u32 + EP_TSIZ
    }
    /// OUT 端点控制寄存器偏移
    fn doep_ctl_off(ep: usize) -> u32 {
        DOEP_BASE + EP_STRIDE * ep as u32 + EP_CTL
    }
    /// OUT 端点中断寄存器偏移
    fn doep_int_off(ep: usize) -> u32 {
        DOEP_BASE + EP_STRIDE * ep as u32 + EP_INT
    }
    /// 写 DIEPCTLx
    fn diep_ctl(&mut self, ep: usize, v: u32) {
        self.regs[(Self::diep_ctl_off(ep) >> 2) as usize] = v;
    }
    /// 写 DOEPCTLx
    fn doep_ctl(&mut self, ep: usize, v: u32) {
        self.regs[(Self::doep_ctl_off(ep) >> 2) as usize] = v;
    }
    /// DIEPTSIZx 的 XFRSIZ 字段
    fn diep_xfrsiz(&self, ep: usize) -> u32 {
        self.regs[(Self::diep_tsiz_off(ep) >> 2) as usize] & TSIZ_XFRSIZ
    }

    /// 检查 IN 端点是否满足发送完成条件并触发 XFRC。
    /// 条件：EPENA 已置位 且 发送缓冲长度 ≥ XFRSIZ（0 长度包恒成立）。
    /// 由写 DIEPCTL(EPENA) 与写 DFIFO 两处驱动（覆盖固件不同写序）。
    fn try_finish_in(&mut self, ep: usize) {
        let ctl = self.regs[(Self::diep_ctl_off(ep) >> 2) as usize];
        if ctl & EP_EPENA == 0 || self.in_pending[ep] {
            return;
        }
        let need = self.diep_xfrsiz(ep);
        if (self.tx[ep].len() as u32) >= need {
            // 传输完成：置 XFRC，XFRSIZ 清零
            let int_off = Self::diep_int_off(ep);
            self.regs[(int_off >> 2) as usize] |= EPINT_XFRC;
            let tsiz = Self::diep_tsiz_off(ep);
            self.regs[(tsiz >> 2) as usize] &= !TSIZ_XFRSIZ;
            self.in_pending[ep] = true;
            let iepm = (1u32 << ep) & self.regs[(OFF_DAINTMSK >> 2) as usize];
            let xfrcm = self.regs[(OFF_DIEPMSK >> 2) as usize] & DIEPMSK_XFRCM;
            if iepm != 0 && xfrcm != 0 {
                self.nvic.lock().unwrap().set_pending(USB_OTG_FS_IRQ);
            }
        }
    }

    /// 按当前 GINTMSK/DAINTMSK×DIEPMSK/DOEPMSK 门控挂起 OTG_FS IRQ。
    /// （RXFLVL/USBRST/ENUMDNE/WKUP 经 GINTMSK；IN/OUT XFRC、OUT STUP 经 DAINTMSK。）
    fn pulse(&mut self) {
        let gint = self.regs[(OFF_GINTSTS >> 2) as usize] & GINT_RW_MASK;
        let gmsk = self.regs[(OFF_GINTMSK >> 2) as usize];
        // GINT 级：RXFLVL/USBRST/ENUMDNE/WKUP 由 GINTMSK 对应位门控
        let gint_hit = (gint & gmsk & (GINT_RXFLVL | GINT_USBRST | GINT_ENUMDNE | GINT_WKUP)) != 0;
        let gint_en = self.regs[(OFF_GAHBCFG >> 2) as usize] & GAHBCFG_GINT != 0;
        let daint_msk = self.regs[(OFF_DAINTMSK >> 2) as usize];
        let diep_msk = self.regs[(OFF_DIEPMSK >> 2) as usize];
        let doep_msk = self.regs[(OFF_DOEPMSK >> 2) as usize];
        let mut ep_hit = false;
        for ep in 0..EP_COUNT {
            let di = self.regs[(Self::diep_int_off(ep) >> 2) as usize];
            if di & EPINT_XFRC != 0 && diep_msk & DIEPMSK_XFRCM != 0
                && daint_msk & daint_iep(ep) != 0
            {
                ep_hit = true;
            }
            let dout = self.regs[(Self::doep_int_off(ep) >> 2) as usize];
            if dout & EPINT_XFRC != 0 && doep_msk & DOEPMSK_XFRCM != 0
                && daint_msk & daint_oep(ep) != 0
            {
                ep_hit = true;
            }
            if ep == 0 && dout & EPINT_STUP != 0 && doep_msk & DOEPMSK_STUPM != 0
                && daint_msk & daint_oep(0) != 0
            {
                ep_hit = true;
            }
        }
        if (gint_en && gint_hit) || ep_hit {
            self.nvic.lock().unwrap().set_pending(USB_OTG_FS_IRQ);
        }
    }
}

impl Peripheral for UsbOtg {
    fn name(&self) -> &str {
        "USB_OTG_FS"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_GRXSTSR | OFF_GRXSTSP => {
                // 读接收状态队列（GRXSTSP 弹出；GRXSTSR 是影子寄存器不弹出）
                if offset == OFF_GRXSTSP {
                    let v = self.rx_status.pop_front().unwrap_or(0);
                    if self.rx_status.is_empty() {
                        // 接收 FIFO 空 → 清 RXFLVL
                        self.regs[(OFF_GINTSTS >> 2) as usize] &= !GINT_RXFLVL;
                    }
                    Ok(v)
                } else {
                    Ok(self.rx_status.front().copied().unwrap_or(0))
                }
            }
            OFF_GINTSTS => Ok(self.gintsts()),
            o if o >= OFF_DFIFO0 && o < OFF_DFIFO0 + FIFO_STRIDE * EP_COUNT as u32 => {
                // 读 DFIFO0 = 从接收 FIFO 弹出 4 字节（SETUP/OUT 数据）
                let mut v = 0u32;
                for i in 0..4 {
                    let b = self.rx.pop_front().unwrap_or(0);
                    v |= (b as u32) << (8 * i);
                }
                Ok(v)
            }
            o if o < REG_COUNT as u32 * 4 => {
                Ok(self.regs[(o >> 2) as usize])
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_GINTSTS => {
                // 写 1 清除（W1C）
                self.regs[(OFF_GINTSTS >> 2) as usize] &= !(value & GINT_RW_MASK);
                Ok(())
            }
            OFF_GRSTCTL => {
                // 核心软复位：写 CSRST=1 → 立即完成（回读 AHBIDL）
                if value & GRSTCTL_CSRST != 0 {
                    self.regs[(OFF_GRSTCTL >> 2) as usize] = GRSTCTL_AHBIDL;
                } else {
                    self.regs[(OFF_GRSTCTL >> 2) as usize] = value;
                }
                Ok(())
            }
            o if o >= OFF_DFIFO0 && o < OFF_DFIFO0 + FIFO_STRIDE * EP_COUNT as u32 => {
                // 写 DFIFOx = IN 端点发送数据（4 字节追加到发送缓冲）
                let ep = ((o - OFF_DFIFO0) / FIFO_STRIDE) as usize;
                for i in 0..4 {
                    self.tx[ep].push(((value >> (8 * i)) & 0xFF) as u8);
                }
                self.try_finish_in(ep);
                Ok(())
            }
            o if o >= DIEP_BASE && o < DIEP_BASE + EP_STRIDE * EP_COUNT as u32 => {
                let ep = ((o - DIEP_BASE) / EP_STRIDE) as usize;
                let sub = (o - DIEP_BASE) % EP_STRIDE;
                let idx = (o >> 2) as usize;
                match sub {
                    EP_CTL => {
                        // 保存控制值（CNAK/SNAK/EPENA 为写 1 动作位，简化随值回读）
                        self.regs[idx] = value;
                        if value & EP_EPENA != 0 {
                            self.try_finish_in(ep);
                        }
                    }
                    EP_INT => {
                        // 写 1 清除端点中断
                        self.regs[idx] &= !value;
                    }
                    _ => {
                        self.regs[idx] = value;
                    }
                }
                Ok(())
            }
            o if o >= DOEP_BASE && o < DOEP_BASE + EP_STRIDE * EP_COUNT as u32 => {
                let idx = (o >> 2) as usize;
                let sub = (o - DOEP_BASE) % EP_STRIDE;
                match sub {
                    EP_INT => {
                        // 写 1 清除端点中断
                        self.regs[idx] &= !value;
                    }
                    _ => {
                        self.regs[idx] = value;
                    }
                }
                Ok(())
            }
            o if o < REG_COUNT as u32 * 4 => {
                let idx = (o >> 2) as usize;
                match offset {
                    OFF_GUSBCFG => {
                        // 保留 FDMOD/PHYSEL 等可写位
                        self.regs[idx] = value & (GUSBCFG_FDMOD | GUSBCFG_PHYSEL | 0xFFFF);
                    }
                    OFF_GCCFG => {
                        // 掉电位：写 1 掉电、写 0 上电（其余位存储回读）
                        self.regs[idx] = (value & !GCCFG_PWRDWN) | (self.regs[idx] & GCCFG_PWRDWN);
                        if value & GCCFG_PWRDWN == 0 {
                            self.regs[idx] &= !GCCFG_PWRDWN;
                        } else {
                            self.regs[idx] |= GCCFG_PWRDWN;
                        }
                    }
                    _ => {
                        self.regs[idx] = value;
                    }
                }
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        *self = Self::new(self.events.clone(), self.nvic.clone());
    }
}
