//! System Control Block（SCB）外设。
//!
//! M1 作为内存总线 MMIO 转发的首个演示目标：寄存器文件式外设，
//! 读写保持"镜像 RAM"语义（读返回最后写入值，初始为 0），
//! 使固件对 CPACR（0xE000ED88）等的读写能走通完整链路：
//! CPU 访问 → Unicorn mem hook → 内存总线 → 本外设。
//!
//! M2：SCB 挂接共享的 [`Mpu`]，将 MPU 寄存器窗口（0xED90-0xEDB8）与
//! MemManage 故障状态（MMFSR 0xED28 / MMFAR 0xED34）委托给 Mpu 处理，
//! 并将 NVIC 寄存器窗口（0xE100-0xE4FF）委托给共享的 [`Nvic`]，
//! 其余地址仍保持镜像语义。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use super::mpu::{MMFAR_OFF, MMFSR_OFF, MPU_WIN_END, MPU_WIN_START, Mpu};
use super::nvic::{
    AIRCR_OFF, NVIC_WIN_END, NVIC_WIN_START, SHPR1_OFF, SHPR_END, Nvic,
};
use super::{BusError, Peripheral};

/// SysTick 寄存器偏移（相对 SCB 基址 0xE000E000）
const SYST_CTRL_OFF: u32 = 0x010;
const SYST_LOAD_OFF: u32 = 0x014;
const SYST_VAL_OFF: u32 = 0x018;
const SYST_CALIB_OFF: u32 = 0x01C;
/// ICSR（中断控制与状态）偏移 0xE000ED04
const ICSR_OFF: u32 = 0xD04;

/// SysTick_CTRL 位定义
const SYST_COUNTFLAG: u32 = 1 << 16;
const SYST_TICKINT: u32 = 1 << 1;
const SYST_ENABLE: u32 = 1 << 0;

/// ICSR 位定义：软件置位/清除系统异常挂起 + VECTACTIVE 读取
const ICSR_PENDSVSET: u32 = 1 << 28;
const ICSR_PENDSVCLR: u32 = 1 << 27;
const ICSR_PENDSTSET: u32 = 1 << 26;
const ICSR_PENDSTCLR: u32 = 1 << 25;
const ICSR_VECTACTIVE_MASK: u32 = 0x1FF;

/// 系统异常向量号（PendSV=14 / SysTick=15）
const VECTOR_PENDSV: u32 = 14;
const VECTOR_SYSTICK: u32 = 15;

/// 该偏移是否属于 MPU 委托窗口（MMFSR/MMFAR + MPU 寄存器区间）
fn is_mpu_offset(offset: u32) -> bool {
    offset == MMFSR_OFF
        || offset == MMFAR_OFF
        || (offset >= MPU_WIN_START && offset <= MPU_WIN_END)
}

/// 该偏移是否属于 NVIC 委托窗口（0xE100-0xE4FF）
fn is_nvic_offset(offset: u32) -> bool {
    offset >= NVIC_WIN_START && offset < NVIC_WIN_END
}

/// 该偏移是否属于 NVIC 额外委托（AIRCR 优先级分组 + SHPR 系统异常优先级）
fn is_nvic_extra_offset(offset: u32) -> bool {
    offset == AIRCR_OFF || (offset >= SHPR1_OFF && offset < SHPR_END)
}

/// SCB 外设：寄存器文件式镜像 + MPU/NVIC 寄存器窗口委托 + SysTick 定时器。
pub struct SystemControl {
    /// 寄存器文件（每项 4 字节，保存最后写入值）
    regs: Vec<u32>,
    /// 共享的 MPU（可为空：不挂接 MPU 时保持 M1 纯镜像行为）
    mpu: Option<Arc<Mutex<Mpu>>>,
    /// 共享的 NVIC（可为空）
    nvic: Option<Arc<Mutex<Nvic>>>,
    // ---- SysTick（0xE000E010-0xE000E01C，随虚拟时钟 tick 推进）----
    /// SYST_CTRL 寄存器（ENABLE/TICKINT/CLKSOURCE/COUNTFLAG）
    syst_ctrl: u32,
    /// SYST_RVR 重载值（计数到 0 后自动重载）
    syst_load: u32,
    /// SYST_CVR 当前值（递减计数，内部以 u64 推进）
    syst_val: u64,
    /// SysTick 活动标记（CTRL.ENABLE 置位，供 block hook 跳过未激活外设的加锁 tick）
    pub active: Arc<AtomicBool>,
}

impl SystemControl {
    /// 创建 SCB 外设，`size` 为寄存器区间字节数（4 字节对齐），不挂接 MPU/NVIC。
    pub fn new(size: u32) -> Self {
        Self {
            regs: vec![0; (size / 4) as usize],
            mpu: None,
            nvic: None,
            syst_ctrl: 0,
            syst_load: 0,
            syst_val: 0,
            active: Arc::new(AtomicBool::new(false)),
        }
    }

    /// 创建 SCB 外设并挂接共享 MPU：MPU 窗口（0xED90-0xEDB8）与
    /// MMFSR/MMFAR（0xED28/0xED34）委托给 `mpu`。
    pub fn new_with_mpu(size: u32, mpu: Arc<Mutex<Mpu>>) -> Self {
        Self {
            regs: vec![0; (size / 4) as usize],
            mpu: Some(mpu),
            nvic: None,
            syst_ctrl: 0,
            syst_load: 0,
            syst_val: 0,
            active: Arc::new(AtomicBool::new(false)),
        }
    }

    /// 创建 SCB 外设并挂接共享 MPU 与 NVIC（NVIC 窗口 0xE100-0xE4FF 委托给 `nvic`）。
    pub fn new_with_mpu_nvic(size: u32, mpu: Arc<Mutex<Mpu>>, nvic: Arc<Mutex<Nvic>>) -> Self {
        Self {
            regs: vec![0; (size / 4) as usize],
            mpu: Some(mpu),
            nvic: Some(nvic),
            syst_ctrl: 0,
            syst_load: 0,
            syst_val: 0,
            active: Arc::new(AtomicBool::new(false)),
        }
    }

    /// SysTick 计数推进：ENABLE 时按虚拟周期递减 CVR，计数到 0 触发溢出
    /// （COUNTFLAG 置位 + TICKINT 使能时挂起 SysTick 异常 vector 15）。
    ///
    /// 硬件语义：CVR 每周期减 1，从 LOAD 递减到 0 共 LOAD+1 个周期后溢出并自动
    /// 重载 LOAD。挂起位为电平（多次溢出仅保持置位），故一次 tick 内多次溢出
    /// 与一次等价，仅需重算剩余计数。
    fn syst_tick(&mut self, cycles: u64) {
        if self.syst_ctrl & SYST_ENABLE == 0 {
            return;
        }
        let period = self.syst_load as u64 + 1; // 距下一次溢出所需周期数
        let to_next = self.syst_val + 1; // 当前 CVR 还需 to_next 周期到 0
        if cycles < to_next {
            self.syst_val = to_next - cycles - 1;
            return;
        }
        // 至少溢出一次：COUNTFLAG 置位；TICKINT 使能时挂起 SysTick 异常
        self.syst_ctrl |= SYST_COUNTFLAG;
        if self.syst_ctrl & SYST_TICKINT != 0 {
            if let Some(nvic) = &self.nvic {
                nvic.lock().unwrap().set_sys_pending(VECTOR_SYSTICK);
            }
        }
        // 重算 CVR：溢出后每 period 周期再次溢出（挂起位幂等，仅需推进到最终位置）
        let rem = cycles - to_next;
        self.syst_val = if period == 0 {
            0
        } else {
            let r = rem % period;
            if r == 0 { self.syst_load as u64 } else { r - 1 }
        };
    }
}

impl Peripheral for SystemControl {
    fn name(&self) -> &str {
        "SCB"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if let Some(mpu) = &self.mpu {
            if is_mpu_offset(offset) {
                return mpu.lock().unwrap().read(offset, size);
            }
        }
        if let Some(nvic) = &self.nvic {
            if is_nvic_offset(offset) || is_nvic_extra_offset(offset) {
                return nvic.lock().unwrap().read(offset, size);
            }
        }
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        // ICSR：VECTACTIVE = 当前异常号（线程模式 0）
        if offset == ICSR_OFF {
            let vectactive = self
                .nvic
                .as_ref()
                .map(|n| n.lock().unwrap().current_exception())
                .unwrap_or(0)
                & ICSR_VECTACTIVE_MASK;
            return Ok(vectactive);
        }
        // SysTick 寄存器
        match offset {
            SYST_CTRL_OFF => {
                // 真机语义：读 CTRL 会清除 COUNTFLAG（jOS 的 irq_hal_systick_clear
                // 依赖"读 CTRL 清 COUNTFLAG"重新武装节拍）。
                let v = self.syst_ctrl;
                self.syst_ctrl &= !SYST_COUNTFLAG;
                return Ok(v);
            }
            SYST_LOAD_OFF => return Ok(self.syst_load),
            SYST_VAL_OFF => return Ok(self.syst_val as u32),
            SYST_CALIB_OFF => return Ok(0),
            _ => {}
        }
        let idx = (offset / 4) as usize;
        self.regs.get(idx).copied().ok_or(BusError::OutOfRange)
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if let Some(mpu) = &self.mpu {
            if is_mpu_offset(offset) {
                return mpu.lock().unwrap().write(offset, size, value);
            }
        }
        if let Some(nvic) = &self.nvic {
            if is_nvic_offset(offset) || is_nvic_extra_offset(offset) {
                return nvic.lock().unwrap().write(offset, size, value);
            }
        }
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        // ICSR：PENDSVSET/PENDSTSET 写 1 挂起系统异常；PENDSVCLR/PENDSTCLR 写 1 清除
        if offset == ICSR_OFF {
            if let Some(nvic) = &self.nvic {
                let mut n = nvic.lock().unwrap();
                if value & ICSR_PENDSVSET != 0 {
                    n.set_sys_pending(VECTOR_PENDSV);
                }
                if value & ICSR_PENDSVCLR != 0 {
                    n.clear_sys_pending(VECTOR_PENDSV);
                }
                if value & ICSR_PENDSTSET != 0 {
                    n.set_sys_pending(VECTOR_SYSTICK);
                }
                if value & ICSR_PENDSTCLR != 0 {
                    n.clear_sys_pending(VECTOR_SYSTICK);
                }
            }
            return Ok(());
        }
        // SysTick 寄存器
        match offset {
            SYST_CTRL_OFF => {
                // 可写位：CLKSOURCE/TICKINT/ENABLE；COUNTFLAG 由硬件置位、读清除
                let was_enabled = self.syst_ctrl & SYST_ENABLE != 0;
                self.syst_ctrl = (self.syst_ctrl & SYST_COUNTFLAG) | (value & 0x7);
                // ENABLE 变化同步活动标记（block hook 据此跳过未激活外设的加锁 tick）
                self.active
                    .store(self.syst_ctrl & SYST_ENABLE != 0, Ordering::Relaxed);
                // ENABLE 0→1：真机把 CVR 重载为 RVR（此后 LOAD+1 周期才溢出）
                if !was_enabled && self.syst_ctrl & SYST_ENABLE != 0 {
                    self.syst_val = self.syst_load as u64;
                }
                Ok(())
            }
            SYST_LOAD_OFF => {
                self.syst_load = value;
                Ok(())
            }
            SYST_VAL_OFF => {
                // 写 CVR：清除 COUNTFLAG，并把计数器重装载为 LOAD（真机：VAL 写 0
                // 后下一拍重载 LOAD，此后 LOAD+1 周期才溢出）。此前置 0 会导致
                // ENABLE 后下一个 tick 立即溢出，产生一次多余的瞬间 SysTick 异常。
                self.syst_ctrl &= !SYST_COUNTFLAG;
                self.syst_val = self.syst_load as u64;
                Ok(())
            }
            _ => {
                let idx = (offset / 4) as usize;
                let slot = self.regs.get_mut(idx).ok_or(BusError::OutOfRange)?;
                *slot = value;
                Ok(())
            }
        }
    }

    fn reset(&mut self) {
        self.syst_ctrl = 0;
        self.syst_load = 0;
        self.syst_val = 0;
        self.active.store(false, Ordering::Relaxed);
    }

    fn tick(&mut self, cycles: u64) {
        self.syst_tick(cycles);
    }
}
