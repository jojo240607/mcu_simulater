//! CPU 封装：对 Unicorn Engine 的薄封装，对外隐藏 FFI。
//!
//! M0 阶段仅提供最小集：创建 M4F 实例、寄存器/内存读写、内存映射、执行控制。
//! M1 起增加 mem/block/code hook 注册（供 MMIO 转发、时序计数、中断投递使用）。

use std::fmt;

use unicorn_engine::{uc_error, Arch, ArmCpuModel, Mode, Prot, RegisterARM, Unicorn};

/// CPU 错误
#[derive(Debug)]
pub enum CoreError {
    /// Unicorn 底层错误
    Unicorn(String),
    /// I/O 错误（固件加载等）
    Io(String),
}

impl fmt::Display for CoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CoreError::Unicorn(e) => write!(f, "unicorn: {e}"),
            CoreError::Io(e) => write!(f, "io: {e}"),
        }
    }
}

impl std::error::Error for CoreError {}

impl From<uc_error> for CoreError {
    fn from(e: uc_error) -> Self {
        CoreError::Unicorn(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, CoreError>;

/// Cortex-M4F 处理器封装（Unicorn THUMB + MCLASS，CPU model = CORTEX_M4）。
///
/// 说明：Unicorn 的 hook 回调携带 `&mut Unicorn<H>`，M0 阶段 `H = ()`，
/// M1 集成总线时可改为在回调中捕获 `Arc<Mutex<Bus>>` 或使用自定义上下文类型。
pub struct Cpu {
    emu: Unicorn<'static, ()>,
}

/// Cortex-M4 模式组合（THUMB + MCLASS，小端为默认）。
/// FPU（VFPv4）不在此处开启——UC2 已弃用 `Mode::VFP4`，
/// 改为在 `new_m4f` 中通过 `ctl_set_cpu_model(CORTEX_M4)` 指定 CPU model 启用。
fn m4f_mode() -> Mode {
    Mode::THUMB | Mode::MCLASS
}

impl Cpu {
    /// 创建 Cortex-M4F 实例（CPU model = CORTEX_M4，含 FPU）
    pub fn new_m4f() -> Result<Self> {
        let mut emu = Unicorn::new(Arch::ARM, m4f_mode())?;
        emu.ctl_set_cpu_model(ArmCpuModel::CORTEX_M4 as i32)?;
        Ok(Self { emu })
    }

    // ---- 寄存器 ----

    /// 读寄存器（ARM 寄存器视图）
    pub fn reg_read(&mut self, reg: RegisterARM) -> Result<u64> {
        Ok(self.emu.reg_read(reg)?)
    }

    /// 写寄存器
    pub fn reg_write(&mut self, reg: RegisterARM, value: u64) -> Result<()> {
        self.emu.reg_write(reg, value)?;
        Ok(())
    }

    /// 读 32 位寄存器（PC/R0 等常用）
    pub fn reg_read_u32(&mut self, reg: RegisterARM) -> Result<u32> {
        Ok(self.emu.reg_read(reg)? as u32)
    }

    // ---- 内存 ----

    /// 映射内存区间（Unicorn 要求地址与大小按 4KB 对齐）
    pub fn mem_map(&mut self, addr: u64, size: u64, prot: Prot) -> Result<()> {
        self.emu.mem_map(addr, size, prot)?;
        Ok(())
    }

    /// 读取内存
    pub fn mem_read(&mut self, addr: u64, size: usize) -> Result<Vec<u8>> {
        Ok(self.emu.mem_read_as_vec(addr, size)?)
    }

    /// 写入内存
    pub fn mem_write(&mut self, addr: u64, buf: &[u8]) -> Result<()> {
        self.emu.mem_write(addr, buf)?;
        Ok(())
    }

    // ---- 执行 ----

    /// 从 `begin` 执行到 `until`（0 表示不限），超时 `timeout`（us），上限 `count` 条指令。
    ///
    /// M-class 为 Thumb 模式：`reg_read(PC)` 返回的地址不含 Thumb 位（偶数），
    /// 而 Unicorn 要求 `begin` 带 Thumb 位（奇数），此处统一置位，避免调用方踩坑。
    pub fn emu_start(&mut self, begin: u64, until: u64, timeout: u64, count: usize) -> Result<()> {
        self.emu.emu_start(begin | 1, until, timeout, count)?;
        Ok(())
    }

    /// 停止执行
    pub fn emu_stop(&mut self) -> Result<()> {
        self.emu.emu_stop()?;
        Ok(())
    }

    /// 底层访问（供 M1 注册 hook 使用）
    pub fn raw(&mut self) -> &mut Unicorn<'static, ()> {
        &mut self.emu
    }
}
