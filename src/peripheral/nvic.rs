//! Cortex-M4 嵌套向量中断控制器（NVIC）。
//!
//! M2 首版：寄存器文件（ISER/ICER/ISPR/ICPR/IABR/IPR）+ 中断管理
//! （挂起/使能/优先级/抢占选择），并承载仿真侧的中断投递状态
//! （当前异常栈 + 停机原因），供 Machine 的 block hook / intr hook 使用。
//!
//! M4 补全：优先级分组（AIRCR.PRIGROUP 决定抢占/子优先级位数）与
//! 系统异常优先级（SHPR1-3，MemManage/BusFault/UsageFault/SVCall/
//! PendSV/SysTick 可配置），抢占判定按分组后的抢占优先级，
//! 同级仲裁按含子优先级的完整数值。
//!
//! 地址映射（相对 SCB 基址 0xE000E000 的偏移，NVIC 基址 0xE000E100）：
//! - ISER0-2 @ 0x100 / ICER0-2 @ 0x180 / ISPR0-2 @ 0x200
//! - ICPR0-2 @ 0x280 / IABR0-2 @ 0x300 / IPR0-81 @ 0x400
//! - AIRCR @ 0x00C（PRIGROUP）/ SHPR1-3 @ 0xD18/0xD1C/0xD20（SCB 委托）
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

/// AIRCR 偏移（PRIGROUP bits10:8）
pub const AIRCR_OFF: u32 = 0x00C;
/// SHPR1-3 偏移（系统异常优先级，每字节一个）
pub const SHPR1_OFF: u32 = 0xD18;
pub const SHPR2_OFF: u32 = 0xD1C;
pub const SHPR3_OFF: u32 = 0xD20;
/// SHPR 区终点（不含）
pub const SHPR_END: u32 = 0xD24;

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
    // IPR：0x300 + 4*word，每字 4 字节、每字节一个 IRQ（IRQ = word*4 + byte）
    // 与硬件一致：IPR0 @ 0xE000E400 含 IRQ0..3，IPR7 @ 0xE000E41C 含 IRQ28..31。
    let ipr_words = NVIC_IRQ_COUNT.div_ceil(4); // 82 → 21 字
    if (0x300..0x300 + 4 * ipr_words as u32).contains(&off) && off % 4 == 0 {
        let word = (off - 0x300) / 4;
        return Some((RegBank::Ipr, (word * 4) as usize)); // 返回该字起始 IRQ
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

/// SHPR 偏移对应的 4 个系统异常向量（None = 保留字节）：
/// SHPR1 = MemManage(4)/BusFault(5)/UsageFault(6)；SHPR2 = SVCall(11)；
/// SHPR3 = PendSV(14)/SysTick(15)。
fn shpr_vectors(offset: u32) -> [Option<u32>; 4] {
    match offset {
        SHPR1_OFF => [Some(4), Some(5), Some(6), None],
        SHPR2_OFF => [Some(11), None, None, None],
        SHPR3_OFF => [None, Some(14), Some(15), None],
        _ => [None; 4],
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
    /// 外部中断优先级（低 4 位有效，数值小 = 优先级高）
    priority: [u8; NVIC_IRQ_COUNT],
    /// 系统异常优先级（索引 = vector − 4，仅 4/5/6/11/14/15 有效，复位 0）
    sys_pri: [u8; 12],
    /// 优先级分组（AIRCR.PRIGROUP bits10:8，0..7，>=5 视作 4）
    group: u8,
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
            sys_pri: [0; 12],
            group: 0,
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

    /// 优先级分组：返回（抢占优先级位数, 子优先级位数）。4 位优先级实现：
    /// 抢占位数 = min(7 − PRIGROUP, 4)，子位数 = 4 − 抢占位数。即
    /// PRIGROUP 0-3 → 抢占 4/子 0（HAL 默认 NVIC_PRIORITYGROUP_4 = PRIGROUP3），
    /// 4→3/1，5→2/2，6→1/3，7→0/4。
    fn group_splits(group: u8) -> (u32, u32) {
        let pre = (7 - (group & 0x7)).min(4) as u32;
        (pre, 4 - pre)
    }

    /// 外部中断的分组后抢占优先级（数值小 = 优先级高）。
    fn preempt_priority(&self, raw: u8) -> u8 {
        let (_, sub) = Self::group_splits(self.group);
        (raw & 0xF) >> sub
    }

    /// 异常的有效优先级（i32，数值小 = 优先级高，可负表示不可配置固定优先级）：
    /// - Reset/NMI/HardFault 固定为最高（-3/-2/-1，始终高于任何可配置异常）；
    /// - 可配置系统异常（vector 4..15）取 SHPR 低 4 位；
    /// - 外部中断取分组后的抢占优先级。
    fn effective_priority(&self, vector: u32) -> i32 {
        match vector {
            1 => -3, // Reset
            2 => -2, // NMI
            3 => -1, // HardFault
            4..=15 => (self.sys_pri[(vector - 4) as usize] & 0xF) as i32,
            _ => self.preempt_priority(self.priority[(vector - 16) as usize] & 0xF) as i32,
        }
    }

    /// 当前异常优先级（数值小 = 优先级高）；线程模式返回 None（可被任意中断抢占）。
    pub fn current_priority(&self) -> Option<i32> {
        let top = *self.exception_stack.last()?;
        Some(self.effective_priority(top))
    }

    /// 是否处于 handler 模式（有活动异常）。
    pub fn in_handler(&self) -> bool {
        !self.exception_stack.is_empty()
    }

    /// 选择应抢占的挂起中断（无则 None）：
    /// - 被 PRIMASK 屏蔽 → 无；
    /// - 被 BASEPRI 屏蔽（完整优先级数值 >= basepri）→ 剔除；
    /// - 分组后抢占优先级不高于当前异常（handler 模式）→ 不抢占
    ///   （同抢占级即便子优先级更高也不抢占，仅参与同时挂起仲裁）；
    /// - 多个候选按完整优先级（含子优先级）数值小者优先。
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
            let raw = self.priority(irq);
            if basepri != 0 && raw >= basepri {
                continue;
            }
            let pre = self.preempt_priority(raw);
            if let Some(c) = cur {
                if pre as i32 >= c {
                    continue;
                }
            }
            // raw = (preempt << sub) | sub：数值排序天然先比抢占级再比子优先级
            if best.map_or(true, |(_, b)| raw < b) {
                best = Some((irq, raw));
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
        match offset {
            AIRCR_OFF => Ok((self.group as u32 & 0x7) << 8), // PRIGROUP 位
            SHPR1_OFF..SHPR_END => self.read_shpr(offset),
            _ => match decode(offset) {
                Some((RegBank::Iser, w)) => Ok(self.enable[w]),
                Some((RegBank::Icer, w)) => Ok(self.enable[w]),
                Some((RegBank::Ispr, w)) => Ok(self.pending[w]),
                Some((RegBank::Icpr, w)) => Ok(self.pending[w]),
                Some((RegBank::Iabr, w)) => Ok(self.active[w]),
                Some((RegBank::Ipr, irq_base)) => {
                    // 每字 4 字节、每字节一个 IRQ（超出 IRQ 上限的字节读 0）
                    let mut v = 0u32;
                    for i in 0..4 {
                        let irq = irq_base + i;
                        if irq < NVIC_IRQ_COUNT {
                            v |= (self.priority[irq] as u32 & 0xF) << (8 * i);
                        }
                    }
                    Ok(v)
                }
                None => Err(BusError::OutOfRange),
            },
        }
    }

    /// 读 SHPR1-3（每字节一个可配置系统异常优先级，低 4 位有效）
    fn read_shpr(&self, offset: u32) -> Result<u32, BusError> {
        let vecs = shpr_vectors(offset);
        let mut v = 0u32;
        for i in 0..4 {
            if let Some(vec) = vecs[i] {
                v |= (self.sys_pri[(vec - 4) as usize] as u32 & 0xF) << (8 * i);
            }
        }
        Ok(v)
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            AIRCR_OFF => {
                // VECTKEY = 0x05FA 才生效（bits31:16）；取 PRIGROUP bits10:8
                if (value >> 16) & 0xFFFF == 0x05FA {
                    self.group = ((value >> 8) & 0x7) as u8;
                }
                Ok(())
            }
            SHPR1_OFF..SHPR_END => {
                let vecs = shpr_vectors(offset);
                for i in 0..4 {
                    if let Some(vec) = vecs[i] {
                        self.sys_pri[(vec - 4) as usize] = ((value >> (8 * i)) & 0xFF) as u8 & 0xF;
                    }
                }
                Ok(())
            }
            _ => match decode(offset) {
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
                Some((RegBank::Ipr, irq_base)) => {
                    // 每字节一个 IRQ，低 4 位有效；超上限字节丢弃
                    for i in 0..4 {
                        let irq = irq_base + i;
                        if irq < NVIC_IRQ_COUNT {
                            self.priority[irq] = ((value >> (8 * i)) & 0xFF) as u8 & 0xF;
                        }
                    }
                    Ok(())
                }
                None => Err(BusError::OutOfRange),
            },
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
        // IPR 每字节一个 IRQ：IRQ0 优先级 5（数值小=优先级高），IRQ1 优先级 10
        n.write(NVIC_WIN_START + 0x300, 4, 0x0000_0A05).unwrap(); // IPR0: byte0=IRQ0(5), byte1=IRQ1(10)
        assert_eq!(n.priority(0), 5);
        assert_eq!(n.priority(1), 10);
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

    #[test]
    fn priority_grouping_preempts_and_subpriority_arbitration() {
        let mut n = Nvic::new();
        // PRIGROUP=5：抢占 2 位、子优先级 2 位（AIRCR bits10:8=5，VECTKEY=0x05FA）
        n.write(AIRCR_OFF, 4, (0x05FA << 16) | (5 << 8)).unwrap();
        assert_eq!(n.read(AIRCR_OFF, 4).unwrap(), 5 << 8);

        // IRQ0 raw=0x08（抢占 2, 子 0）、IRQ1 raw=0x09（抢占 2, 子 1）：同抢占级
        n.write(NVIC_WIN_START + 0x300, 4, 0x0000_0908).unwrap(); // IPR0: byte0=IRQ0(8), byte1=IRQ1(9)
        n.write(NVIC_WIN_START + 0x00, 4, 0x3).unwrap(); // ISER0 使能 0/1
        n.write(NVIC_WIN_START + 0x100, 4, 0x3).unwrap(); // 挂起 0/1

        // 线程模式同时挂起：同抢占级按子优先级小者（IRQ0, sub=0）先
        assert_eq!(n.select_pending(false, 0), Some(0));

        // 进入 IRQ0（抢占 2）：IRQ1 同抢占级不能抢占
        n.push_exception(16);
        assert_eq!(n.select_pending(false, 0), None);
        n.pop_exception();

        // IRQ2 raw=0x00（抢占 0，高于 IRQ0 的抢占 2）：可抢占
        n.write(NVIC_WIN_START + 0x300, 4, 0x0000_0808).unwrap(); // IPR0: IRQ0=8, IRQ2=0
        n.write(NVIC_WIN_START + 0x00, 4, 1 << 2).unwrap();
        n.write(NVIC_WIN_START + 0x100, 4, 1 << 2).unwrap();
        n.push_exception(16); // 当前 IRQ0（抢占 2）
        assert_eq!(n.select_pending(false, 0), Some(2)); // IRQ2 抢占 0 < 2 可抢占
    }

    #[test]
    fn aircr_vectkey_must_match() {
        let mut n = Nvic::new();
        // 非法 VECTKEY：不生效
        n.write(AIRCR_OFF, 4, (0x1234 << 16) | (2 << 8)).unwrap();
        assert_eq!(n.read(AIRCR_OFF, 4).unwrap(), 0);
        // 合法 VECTKEY：生效
        n.write(AIRCR_OFF, 4, (0x05FA << 16) | (3 << 8)).unwrap();
        assert_eq!(n.read(AIRCR_OFF, 4).unwrap(), 3 << 8);
    }

    #[test]
    fn shpr_system_exception_priority_affects_preemption() {
        let mut n = Nvic::new();
        // IRQ0 优先级 0（最高外部）
        n.write(NVIC_WIN_START + 0x300, 4, 0x0).unwrap();
        n.write(NVIC_WIN_START + 0x00, 4, 1).unwrap();
        n.write(NVIC_WIN_START + 0x100, 4, 1).unwrap();

        // 当前异常 MemManage（vector4，SHPR1 byte0 默认 0）：IRQ0（抢占 0）同级不能抢占
        n.push_exception(4);
        assert_eq!(n.select_pending(false, 0), None);

        // SHPR1 byte0 设 MemManage=1（数值更大=更低优先级）：IRQ0(0) 可抢占
        n.write(SHPR1_OFF, 4, 0x01).unwrap();
        assert_eq!(n.read(SHPR1_OFF, 4).unwrap(), 0x01);
        assert_eq!(n.select_pending(false, 0), Some(0));
        n.pop_exception();

        // SHPR3：byte1=PendSV(14)、byte2=SysTick(15) 读写
        n.write(SHPR3_OFF, 4, (0x07 << 16) | (0x03 << 8)).unwrap();
        assert_eq!(n.read(SHPR3_OFF, 4).unwrap(), (0x07 << 16) | (0x03 << 8));
        // HardFault 固定优先级 -1：即使 SHPR 配置较高也不能被外部抢占
        n.push_exception(3);
        assert_eq!(n.select_pending(false, 0), None);
    }
}
