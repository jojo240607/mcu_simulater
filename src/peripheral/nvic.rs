//! Cortex-M4 嵌套向量中断控制器（NVIC）。
//!
//! M2 首版：寄存器文件（ISER/ICER/ISPR/ICPR/IABR/IPR）+ 中断管理
//! （挂起/使能/优先级/抢占选择），并承载仿真侧的中断投递状态
//! （当前异常栈 + 停机原因），供 Machine 的 block hook / intr hook 使用。
//!
//! 地址映射（相对 SCB 基址 0xE000E000 的偏移，NVIC 基址 0xE000E100）：
//! - ISER0-2 @ 0x100 / ICER0-2 @ 0x180 / ISPR0-2 @ 0x200
//! - ICPR0-2 @ 0x280 / IABR0-2 @ 0x300 / IPR0-81 @ 0x400
//!
//! 说明：总线向本外设传递的偏移为「地址 − SCB 基址」（见 [`crate::bus::Bus`]），
//! 故窗口与寄存器映射均以相对 SCB 基址 0xE000E000 的偏移表示
//! （ISER0 @ 0xE000E100 → 0x100）。
//!
//! STM32F407 优先级位数为 4（低 4 位有效，数值小 = 优先级高）。
//! 本实现仅覆盖 4 字节访问；SCB 对本窗口做委托。

use crate::peripheral::BusError;

/// STM32F407 外部中断数（IRQ 0..=81）
pub const NVIC_IRQ_COUNT: usize = 82;
/// NVIC 寄存器区起点（相对 SCB 基址 0xE000E000；ISER0）
pub const NVIC_WIN_START: u32 = 0x100;
/// NVIC 寄存器区终点（不含；0x500 起为保留/系统异常区）
pub const NVIC_WIN_END: u32 = 0x500;

/// 仿真循环停机原因（由 hook 写入，Machine::run 读取后决定下一步）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopReason {
    /// 无停机请求（emu_start 因达到指令数上限而返回）
    None,
    /// block hook 请求切换到该中断号
    Switch(u32),
    /// intr hook 检测到 EXC_RETURN，现场已恢复
    ExceptionReturn,
}

/// NVIC 寄存器组
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RegBank {
    Iser,
    Icer,
    Ispr,
    Icpr,
    Iabr,
    Ipr,
}

/// 把 SCB 相对偏移解码为（寄存器组, 下标）。不在窗口内返回 None。
fn decode(offset: u32) -> Option<(RegBank, usize)> {
    if !(NVIC_WIN_START..NVIC_WIN_END).contains(&offset) {
        return None;
    }
    let off = offset - NVIC_WIN_START;
    // IPR：0x300 + 4*irq（字对齐，每 IRQ 一字节）
    if (0x300..0x300 + 4 * NVIC_IRQ_COUNT as u32).contains(&off) && off % 4 == 0 {
        return Some((RegBank::Ipr, ((off - 0x300) / 4) as usize));
    }
    let (bank, base) = match off & 0x380 {
        0x00 => (RegBank::Iser, 0x00),
        0x80 => (RegBank::Icer, 0x80),
        0x100 => (RegBank::Ispr, 0x100),
        0x180 => (RegBank::Icpr, 0x180),
        0x200 => (RegBank::Iabr, 0x200),
        _ => return None,
    };
    let word = (off - base) / 4;
    if word < 3 {
        Some((bank, word as usize))
    } else {
        None
    }
}

/// IRQ 对应的（字下标, 位）
fn irq_bits(irq: u32) -> (usize, u32) {
    ((irq / 32) as usize, 1 << (irq % 32))
}

/// 系统异常优先级（数值小 = 优先级高；ARMv7-M 固定值）。
fn system_exception_priority(vector: u32) -> u8 {
    match vector {
        1 => 0,  // Reset
        2 => 1,  // NMI
        3 => 2,  // HardFault
        4 => 3,  // MemManage
        5 => 4,  // BusFault
        6 => 5,  // UsageFault
        11 => 6, // SVCall
        14 => 7, // PendSV
        15 => 8, // SysTick
        _ => 0xF,
    }
}

/// NVIC 外设核心：寄存器文件 + 中断管理。
pub struct Nvic {
    /// 使能位（每字 32 个 IRQ，共 3 字 = 96，够 82）
    enable: [u32; 3],
    /// 挂起位
    pending: [u32; 3],
    /// 活跃位（当前正在服务）
    active: [u32; 3],
    /// 优先级（低 4 位有效，数值小 = 优先级高）
    priority: [u8; NVIC_IRQ_COUNT],
    /// 当前异常号栈（空 = 线程模式；栈顶 = 当前异常）
    exception_stack: Vec<u32>,
    /// 仿真循环停机原因
    stop_reason: StopReason,
}

impl Default for Nvic {
    fn default() -> Self {
        Self::new()
    }
}

impl Nvic {
    pub fn new() -> Self {
        Self {
            enable: [0; 3],
            pending: [0; 3],
            active: [0; 3],
            priority: [0; NVIC_IRQ_COUNT],
            exception_stack: Vec::new(),
            stop_reason: StopReason::None,
        }
    }

    // ---- 中断状态查询/修改 ----

    pub fn is_pending(&self, irq: u32) -> bool {
        let (w, b) = irq_bits(irq);
        self.pending[w] & b != 0
    }

    pub fn is_enabled(&self, irq: u32) -> bool {
        let (w, b) = irq_bits(irq);
        self.enable[w] & b != 0
    }

    pub fn priority(&self, irq: u32) -> u8 {
        self.priority[irq as usize] & 0xF
    }

    pub fn set_pending(&mut self, irq: u32) {
        let (w, b) = irq_bits(irq);
        self.pending[w] |= b;
    }

    pub fn clear_pending(&mut self, irq: u32) {
        let (w, b) = irq_bits(irq);
        self.pending[w] &= !b;
    }

    pub fn set_active(&mut self, irq: u32) {
        let (w, b) = irq_bits(irq);
        self.active[w] |= b;
    }

    pub fn clear_active(&mut self, irq: u32) {
        let (w, b) = irq_bits(irq);
        self.active[w] &= !b;
    }

    // ---- 优先级与抢占选择 ----

    /// 当前异常优先级（数值小 = 优先级高）；线程模式返回 None（可被任意中断抢占）。
    pub fn current_priority(&self) -> Option<u8> {
        let top = *self.exception_stack.last()?;
        Some(if top >= 16 {
            self.priority[(top - 16) as usize] & 0xF
        } else {
            system_exception_priority(top)
        })
    }

    /// 是否处于 handler 模式（有活动异常）。
    pub fn in_handler(&self) -> bool {
        !self.exception_stack.is_empty()
    }

    /// 选择应抢占的挂起中断（无则 None）：
    /// - 被 PRIMASK 屏蔽 → 无；
    /// - 被 BASEPRI 屏蔽（优先级数值 >= basepri）→ 剔除；
    /// - 优先级不高于当前异常（handler 模式）→ 不抢占。
    pub fn select_pending(&self, primask: bool, basepri: u8) -> Option<u32> {
        if primask {
            return None;
        }
        let cur = self.current_priority();
        let mut best: Option<(u32, u8)> = None;
        for irq in 0..NVIC_IRQ_COUNT as u32 {
            if !self.is_pending(irq) || !self.is_enabled(irq) {
                continue;
            }
            let p = self.priority(irq);
            if basepri != 0 && p >= basepri {
                continue;
            }
            if let Some(c) = cur {
                if p >= c {
                    continue;
                }
            }
            if best.map_or(true, |(_, bp)| p < bp) {
                best = Some((irq, p));
            }
        }
        best.map(|(irq, _)| irq)
    }

    // ---- 仿真侧：异常栈与停机原因 ----

    pub fn push_exception(&mut self, vector: u32) {
        self.exception_stack.push(vector);
    }

    pub fn pop_exception(&mut self) -> Option<u32> {
        self.exception_stack.pop()
    }

    /// 当前异常号（线程模式为 0）
    pub fn current_exception(&self) -> u32 {
        *self.exception_stack.last().unwrap_or(&0)
    }

    pub fn set_stop_reason(&mut self, r: StopReason) {
        self.stop_reason = r;
    }

    pub fn take_stop_reason(&mut self) -> StopReason {
        std::mem::replace(&mut self.stop_reason, StopReason::None)
    }

    // ---- 寄存器读写（offset 为相对 SCB 基址的偏移）----

    pub fn read(&self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match decode(offset) {
            Some((RegBank::Iser, w)) => Ok(self.enable[w]),
            Some((RegBank::Icer, w)) => Ok(self.enable[w]),
            Some((RegBank::Ispr, w)) => Ok(self.pending[w]),
            Some((RegBank::Icpr, w)) => Ok(self.pending[w]),
            Some((RegBank::Iabr, w)) => Ok(self.active[w]),
            Some((RegBank::Ipr, irq)) => Ok(self.priority[irq] as u32),
            None => Err(BusError::OutOfRange),
        }
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match decode(offset) {
            Some((RegBank::Iser, w)) => {
                self.enable[w] |= value;
                Ok(())
            }
            Some((RegBank::Icer, w)) => {
                self.enable[w] &= !value;
                Ok(())
            }
            Some((RegBank::Ispr, w)) => {
                self.pending[w] |= value;
                Ok(())
            }
            Some((RegBank::Icpr, w)) => {
                self.pending[w] &= !value;
                Ok(())
            }
            Some((RegBank::Iabr, _)) => Ok(()), // 只读
            Some((RegBank::Ipr, irq)) => {
                self.priority[irq] = (value & 0xFF) as u8 & 0xF;
                Ok(())
            }
            None => Err(BusError::OutOfRange),
        }
    }
}

impl crate::peripheral::Peripheral for Nvic {
    fn name(&self) -> &str {
        "NVIC"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        Nvic::read(self, offset, size)
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        Nvic::write(self, offset, size, value)
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn irq_set_clear_pending_enable() {
        let mut n = Nvic::new();
        assert!(!n.is_enabled(30));
        n.write(NVIC_WIN_START + 0x00, 4, 1 << 30).unwrap(); // ISER0
        assert!(n.is_enabled(30));
        n.write(NVIC_WIN_START + 0x80, 4, 1 << 30).unwrap(); // ICER0
        assert!(!n.is_enabled(30));

        n.write(NVIC_WIN_START + 0x00, 4, 1).unwrap(); // ISER0 使能 IRQ0
        n.write(NVIC_WIN_START + 0x100, 4, 1).unwrap(); // ISPR0: 挂起 IRQ0
        assert!(n.is_pending(0));
        assert_eq!(n.select_pending(false, 0), Some(0));
        n.write(NVIC_WIN_START + 0x180, 4, 1).unwrap(); // ICPR0: 清挂起
        assert!(!n.is_pending(0));
        assert_eq!(n.select_pending(false, 0), None);
    }

    #[test]
    fn priority_selection() {
        let mut n = Nvic::new();
        // IRQ0 优先级 5（数值小=优先级高），IRQ1 优先级 10
        n.write(NVIC_WIN_START + 0x300, 4, 0x0000_0005).unwrap(); // IPR0
        n.write(NVIC_WIN_START + 0x304, 4, 0x0000_000A).unwrap(); // IPR1
        n.write(NVIC_WIN_START + 0x00, 4, 0x3).unwrap(); // ISER0 使能 0/1
        n.write(NVIC_WIN_START + 0x100, 4, 0x3).unwrap(); // ISPR0 挂起 0/1

        // 线程模式：应选优先级最高的 IRQ0（5 < 10）
        assert_eq!(n.select_pending(false, 0), Some(0));

        // 进入 IRQ0 后（优先级 5）：IRQ1（10）不能抢占
        n.push_exception(16);
        assert_eq!(n.select_pending(false, 0), None);
        // 当前异常为 IRQ1（优先级 10）：优先级更高的 IRQ0（5）可抢占
        n.pop_exception();
        n.push_exception(17); // 当前异常即 IRQ1（vector 17）
        assert_eq!(n.select_pending(false, 0), Some(0));

        // PRIMASK / BASEPRI 屏蔽
        n.pop_exception();
        assert_eq!(n.select_pending(true, 0), None);
        assert_eq!(n.select_pending(false, 5), None); // basepri=5 屏蔽优先级>=5
        assert_eq!(n.select_pending(false, 6), Some(0));
    }

    #[test]
    fn active_and_ipr_roundtrip() {
        let mut n = Nvic::new();
        n.write(NVIC_WIN_START + 0x100, 4, 1 << 5).unwrap();
        assert!(n.is_pending(5));
        n.set_active(5);
        n.clear_pending(5);
        // IABR 可读
        assert_eq!(n.read(NVIC_WIN_START + 0x200, 4).unwrap(), 1 << 5);
        n.clear_active(5);
        assert_eq!(n.read(NVIC_WIN_START + 0x200, 4).unwrap(), 0);
        // IPR 读写
        n.write(NVIC_WIN_START + 0x300, 4, 0xA).unwrap();
        assert_eq!(n.read(NVIC_WIN_START + 0x300, 4).unwrap(), 0xA);
    }
}
