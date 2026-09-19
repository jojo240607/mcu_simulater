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

use std::sync::Arc;

use crate::peripheral::BusError;
use crate::sim::status::{Status, BIT_NVIC_PENDING};

/// STM32F407 外部中断数（IRQ 0..=81）
pub const NVIC_IRQ_COUNT: usize = 82;

/// NVIC 实现的优先级位数（Cortex-M4 上 ST 的 `__NVIC_PRIO_BITS = 4`）。
/// IPR/SHPR 中优先级存【高 NVIC_PRIO_BITS 位】（= 数值左移 (8 - NVIC_PRIO_BITS)）。
pub const NVIC_PRIO_BITS: u32 = 4;

/// BASEPRI 寄存器值 → 优先级数值（供 [`Nvic::select_pending`] 使用）。
///
/// BASEPRI 存的是**左移 `8 - NVIC_PRIO_BITS` 位后**的值：内核临界区
/// `rtos_crit_enter` 写 `BASEPRI = 0x40`，含义是"屏蔽优先级 >= 4"。
///
/// **绝不能取低若干位**：`0x40 & 0xF == 0` 会被误判成"不屏蔽"（真实缺陷，见
/// `fix(machine): BASEPRI 优先级解析取错位段`）。后果是模拟器会在内核临界区内照样
/// 递送 SysTick/PendSV（优先级 15），任务切换切开 `sleep_add`/`sleep_remove` 的链表
/// 操作 → 睡眠链被破坏 → 任务永久睡死。真机硬件遵守 BASEPRI，故纯模拟器缺陷；
/// 又因切换落点依赖代码布局，表现为"改代码就换症状"的 heisenbug。
#[inline]
pub const fn basepri_to_prio(raw: u32) -> u8 {
    ((raw >> (8 - NVIC_PRIO_BITS)) & ((1 << NVIC_PRIO_BITS) - 1)) as u8
}
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
    /// block hook 请求切换到该向量号（系统异常 14/15 或外部中断 16+）
    Switch(u32),
    /// intr hook 检测到 EXC_RETURN，现场已恢复
    ExceptionReturn,
    /// intr hook 已补全 SVC 异常入口（现场已压栈、PC 已指向 SVC handler）——
    /// run() 继续执行即可（rtos_start → svc 0 启动首个任务依赖此延续）。
    SvcEntry,
    /// block hook 检到 MPU 已使能但数据访问 hook 未安装：run() 懒安装 hook + 刷 TB
    /// 后继续（避免启动早期 hook helper 翻译触发 Unicorn 首指令副作用丢失缺陷）。
    MpuEnable,
    /// block hook 检到 DMA 待搬运（外设忙等 DMA 完成信号量时 CPU 自旋，run() 须
    /// 返回让 DMA process 执行搬运后**继续**——不能与预算耗尽（None）混同，否则
    /// run(count) 的 count 预算在 DMA 繁忙路径下被浪费、CPU 侧虚拟时钟大幅慢于
    /// 物理步长（实测 run(300K) 仅退休 ~48K 字节 → 传感器/控制拍速降 6× 以上）。
    DmaPending,
    /// block hook 检到 GDB 指令级断点（PC 命中）：run() 停止供调试器接管
    Breakpoint,
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
///
/// 注意 SHPR3 采用【CMSIS 字节数组视图】（jOS 用 core_cm4.h 的
/// `SCB->SHP[((IRQn)&0xF)-4]`，PendSV(-2)→SHP[10]→0xE000ED22，
/// SysTick(-1)→SHP[11]→0xE000ED23），而非 ARM TRM 的寄存器视图
///（0xE000ED21=PendSV / 0xE000ED22=SysTick）。实测固件对 PendSV/SysTick
/// 优先级只做字节写（0xD22=0xF0、0xD23=0xF0），若按 ARM 寄存器视图映射，
/// PendSV 优先级会写进 SysTick 字节、PendSV 残留 0（最高）→ 能抢占 ISR，
/// context.S 在 ISR 上下文切换 → 污染任务 TCB（r10 被写坏 → 循环读 0x4C）。
fn shpr_vectors(offset: u32) -> [Option<u32>; 4] {
    match offset {
        SHPR1_OFF => [Some(4), Some(5), Some(6), None],
        SHPR2_OFF => [Some(11), None, None, None],
        SHPR3_OFF => [None, None, Some(14), Some(15)],
        _ => [None; 4],
    }
}

/// NVIC 外设核心：寄存器文件 + 中断管理。
pub struct Nvic {
    /// 使能位（每字 32 个 IRQ，共 3 字 = 96，够 82）
    enable: [u32; 3],
    /// 挂起位
    pending: [u32; 3],
    /// 系统异常挂起位（位 = 向量号，仅可配置的 PendSV=14/SysTick=15 有效）
    sys_pending: u32,
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
    /// 异常嵌套时被中断现场（线程模式或低优先级异常）的 callee-saved 寄存器
    /// r4..r11（与 exception_stack 一一对应）。模拟器在块边界停机派发中断时，
    /// CPU 寄存器 = 块首值，ISR 返回后要"重放该块"，必须还原块首的 r4..r11
    /// 才能与恢复的 PC（块首）一致——否则重放块读到的是 ISR 打断时刻的
    /// 块中间值（callee-saved 被 ISR 链改过），会读出错误基址（实测 r10 变
    /// 4 导致读 0x4C NULL+偏移 fault）。
    callee_stack: Vec<[u32; 8]>,
    /// 仿真循环停机原因
    stop_reason: StopReason,
    /// 全局状态字（BIT_NVIC_PENDING = 是否有任一挂起中断）：Machine block hook
    /// 据此在无挂起时跳过加锁的 select_pending 检查（纯计算负载下中断稀少，
    /// 省去每块锁开销；与 mpu/any_active/wdog 合并为单原子，热路径一次 load）
    pending_any: Arc<Status>,
}

impl Default for Nvic {
    fn default() -> Self {
        Self::new()
    }
}

impl Nvic {
    /// 便捷构造（单元测试/独立使用）：状态字为内部占位，不与外部共享
    pub fn new() -> Self {
        Self::with_pending_any(Arc::new(Status::new()))
    }

    /// 正式构造：`status` 由 Machine 持有，挂起位变化时同步 BIT_NVIC_PENDING
    pub fn with_pending_any(pending_any: Arc<Status>) -> Self {
        Self {
            enable: [0; 3],
            pending: [0; 3],
            sys_pending: 0,
            active: [0; 3],
            priority: [0; NVIC_IRQ_COUNT],
            sys_pri: [0; 12],
            group: 0,
            exception_stack: Vec::new(),
            callee_stack: Vec::new(),
            stop_reason: StopReason::None,
            pending_any,
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
        self.pending_any.set(BIT_NVIC_PENDING);
    }

    pub fn clear_pending(&mut self, irq: u32) {
        let (w, b) = irq_bits(irq);
        self.pending[w] &= !b;
        self.sync_pending_any();
    }

    /// 系统异常挂起（vector 14=PendSV / 15=SysTick）。挂起位为电平：
    /// 溢出/软件置位多次仅保持置位，进入异常时由 enter_exception 清除。
    pub fn set_sys_pending(&mut self, vector: u32) {
        self.sys_pending |= 1 << vector;
        self.pending_any.set(BIT_NVIC_PENDING);
    }

    pub fn clear_sys_pending(&mut self, vector: u32) {
        self.sys_pending &= !(1 << vector);
        self.sync_pending_any();
    }

    pub fn is_sys_pending(&self, vector: u32) -> bool {
        self.sys_pending & (1 << vector) != 0
    }

    /// 是否有任一挂起中断（无锁快速判定，供 block hook 跳过加锁检查）
    pub fn pending_any(&self) -> bool {
        self.pending_any.has(BIT_NVIC_PENDING)
    }

    /// 根据挂起位图重算 BIT_NVIC_PENDING（clear_pending / ICPR 清除后调用）
    fn sync_pending_any(&self) {
        let any = self.pending.iter().any(|&w| w != 0) || self.sys_pending != 0;
        if any {
            self.pending_any.set(BIT_NVIC_PENDING);
        } else {
            self.pending_any.clear(BIT_NVIC_PENDING);
        }
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

    /// 选择应抢占的挂起中断，返回**向量号**（可配置系统异常 14/15 或外部中断 16+）。
    ///
    /// 与 [`Nvic::select_pending`]（仅外部中断、返回 IRQ 号）的区别：PendSV/SysTick
    /// 属系统异常，挂起位不在 NVIC ISPR 位图，优先级取自 SHPR3；两者需与外部中断
    /// 统一参与抢占仲裁（RTOS 内核节拍/切换依赖此语义）。block hook 调用本方法。
    pub fn select_pending_vector(&self, primask: bool, basepri: u8) -> Option<u32> {
        if primask {
            return None;
        }
        let cur = self.current_priority();
        let mut best: Option<(u32, u8)> = None;
        // 可配置系统异常（PendSV=14 / SysTick=15）：优先级 = SHPR 低 4 位，无分组
        for v in [14u32, 15] {
            if !self.is_sys_pending(v) {
                continue;
            }
            let raw = self.sys_pri[(v - 4) as usize] & 0xF;
            if basepri != 0 && raw >= basepri {
                continue;
            }
            // 系统异常可被 BASEPRI 屏蔽（jOS 约定内核节拍/切换置于最低优先级）；
            // 同优先级不抢占当前异常（PendSV 等所有 ISR 退出后运行即依赖此规则）
            if let Some(c) = cur {
                if raw as i32 >= c {
                    continue;
                }
            }
            if best.map_or(true, |(_, b)| raw < b) {
                best = Some((v, raw));
            }
        }
        // 外部中断（向量号 = 16 + IRQ）
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
            if best.map_or(true, |(_, b)| raw < b) {
                best = Some((16 + irq, raw));
            }
        }
        best.map(|(v, _)| v)
    }

    // ---- 仿真侧：异常栈与停机原因 ----

    pub fn push_exception(&mut self, vector: u32) {
        self.exception_stack.push(vector);
    }

    /// 与 push_exception 配对：保存被中断现场 r4..r11（块首值），
    /// 供异常返回后"重放被打断块"时还原寄存器初值。
    pub fn push_callee(&mut self, callee: [u32; 8]) {
        self.callee_stack.push(callee);
    }

    /// 与 pop_exception 配对：弹出并返回被中断现场 r4..r11。
    pub fn pop_callee(&mut self) -> Option<[u32; 8]> {
        self.callee_stack.pop()
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
        // IPR 区字节写：CMSIS NVIC_SetPriority 用 `NVIC->IP[irq] = prio`
        //（uint8_t 数组，每个 IRQ 一字节，地址 0xE000E400+irq）做 8 位访问，
        // 且优先级左移 (8-NVIC_PRIO_BITS)=4 位存【高 4 位】（0x50 = prio 5）。
        // 32 位对齐检查（off%4==0）会拒绝字节写 → 固件设的优先级静默丢失
        //（复位 0），与 SysTick 同抢占级 → IRQ 永不抢占（ISR 饿死）。
        if (NVIC_WIN_START..NVIC_WIN_END).contains(&offset) && size == 1 {
            let off = offset - NVIC_WIN_START;
            let ipr_words = NVIC_IRQ_COUNT.div_ceil(4); // 82 → 21 字
            if (0x300..0x300 + 4 * ipr_words as u32).contains(&off) {
                let word = (off - 0x300) / 4;
                let byte = (off - 0x300) % 4;
                let irq = word * 4 + byte;
                if irq < NVIC_IRQ_COUNT as u32 {
                    self.priority[irq as usize] = ((value as u8) >> 4) & 0xF;
                }
                return Ok(());
            }
        }
        // SHPR1-3 区字节写：CMSIS NVIC_SetPriority 对系统异常（负数 IRQn）
        // 走 `SCB->SHPR[n] = prio`（uint8_t 数组，地址 0xE000ED18+n，n=0..11）
        // 的 8 位访问，同样左移 4 位存【高 4 位】。字节写被拒 → PendSV/SysTick
        // 优先级残留 0（最高）→ PendSV 能抢占 ISR，context.S 在 ISR 上下文里
        // 切换（PSP 仍指被中断任务栈）→ 污染任务 TCB（实测 control 的 r10
        // 被写成 ISR 的 4 → 重放循环读 0x4C fault）。
        if (SHPR1_OFF..SHPR_END).contains(&offset) && size == 1 {
            let word_offset = offset & !3;
            let byte = (offset & 3) as usize;
            if let Some(vec) = shpr_vectors(word_offset)[byte] {
                self.sys_pri[(vec - 4) as usize] = ((value as u8) >> 4) & 0xF;
            }
            return Ok(());
        }
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
                    self.pending_any.set(BIT_NVIC_PENDING);
                    Ok(())
                }
                Some((RegBank::Icpr, w)) => {
                    self.pending[w] &= !value;
                    self.sync_pending_any();
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

    /// BASEPRI 位段解析：必须取【高 NVIC_PRIO_BITS 位】。
    #[test]
    fn basepri_extracts_high_priority_bits() {
        // 内核临界区写 0x40 = "屏蔽优先级 >= 4"
        assert_eq!(basepri_to_prio(0x40), 4);
        assert_eq!(basepri_to_prio(0x50), 5);
        assert_eq!(basepri_to_prio(0xF0), 15);
        // 0 = 不屏蔽
        assert_eq!(basepri_to_prio(0x00), 0);
        // 回归守卫：旧实现 `raw & 0xF` 会把 0x40 解析成 0（不屏蔽）
        assert_ne!(basepri_to_prio(0x40), 0);
    }

    /// 行为守卫：内核临界区（BASEPRI=0x40 ⇒ 阈值 4）必须屏蔽最低优先级（15）的中断，
    /// 否则任务切换会在内核临界区内切入 `sleep_add`/`sleep_remove` 的链表操作。
    #[test]
    fn kernel_critical_section_masks_lowest_prio_irq() {
        let mut n = Nvic::new();
        // IRQ0：优先级 15（最低，与 SysTick/PendSV 同级），使能并挂起
        // 与固件/CMSIS 同路径：字节写，值为已左移 (8-PRIO_BITS)=4 位的优先级（15<<4）
        n.write(NVIC_WIN_START + 0x300, 1, 0xF0).unwrap();
        n.write(NVIC_WIN_START + 0x00, 4, 1).unwrap(); // ISER0: enable IRQ0
        n.set_pending(0);
        assert_eq!(n.priority(0), 15);

        // 内核临界区（阈值 4）：优先级 15 >= 4 → 必须屏蔽
        let kernel_basepri = basepri_to_prio(0x40);
        assert_eq!(kernel_basepri, 4);
        assert_eq!(
            n.select_pending(false, kernel_basepri),
            None,
            "内核临界区内 prio 15 的中断不得抢占（BASEPRI=0x40 必须解析为阈值 4）"
        );
        // 对照：BASEPRI=0（不屏蔽）时应被选中 —— 说明上面的 None 来自屏蔽而非其它条件
        assert_eq!(n.select_pending(false, 0), Some(0));
        // 对照：阈值 16（>15）不屏蔽它
        assert_eq!(n.select_pending(false, basepri_to_prio(0x00)), Some(0));
    }

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

        // SHPR3（CMSIS 字节视图）：byte2=PendSV(14)、byte3=SysTick(15) 读写
        //（固件 core_cm4.h 的 `SCB->SHP[((IRQn)&0xF)-4]` 把 PendSV 写在
        //  0xE000ED22、SysTick 写在 0xE000ED23，模拟器按此映射。）
        n.write(SHPR3_OFF, 4, (0x07 << 24) | (0x03 << 16)).unwrap();
        assert_eq!(n.read(SHPR3_OFF, 4).unwrap(), (0x07 << 24) | (0x03 << 16));
        // HardFault 固定优先级 -1：即使 SHPR 配置较高也不能被外部抢占
        n.push_exception(3);
        assert_eq!(n.select_pending(false, 0), None);
    }
}
