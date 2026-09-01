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

use std::sync::{Arc, Mutex};

use super::mpu::{MMFAR_OFF, MMFSR_OFF, MPU_WIN_END, MPU_WIN_START, Mpu};
use super::nvic::{NVIC_WIN_END, NVIC_WIN_START, Nvic};
use super::{BusError, Peripheral};

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

/// SCB 外设：寄存器文件式镜像 + MPU/NVIC 寄存器窗口委托。
pub struct SystemControl {
    /// 寄存器文件（每项 4 字节，保存最后写入值）
    regs: Vec<u32>,
    /// 共享的 MPU（可为空：不挂接 MPU 时保持 M1 纯镜像行为）
    mpu: Option<Arc<Mutex<Mpu>>>,
    /// 共享的 NVIC（可为空）
    nvic: Option<Arc<Mutex<Nvic>>>,
}

impl SystemControl {
    /// 创建 SCB 外设，`size` 为寄存器区间字节数（4 字节对齐），不挂接 MPU/NVIC。
    pub fn new(size: u32) -> Self {
        Self {
            regs: vec![0; (size / 4) as usize],
            mpu: None,
            nvic: None,
        }
    }

    /// 创建 SCB 外设并挂接共享 MPU：MPU 窗口（0xED90-0xEDB8）与
    /// MMFSR/MMFAR（0xED28/0xED34）委托给 `mpu`。
    pub fn new_with_mpu(size: u32, mpu: Arc<Mutex<Mpu>>) -> Self {
        Self {
            regs: vec![0; (size / 4) as usize],
            mpu: Some(mpu),
            nvic: None,
        }
    }

    /// 创建 SCB 外设并挂接共享 MPU 与 NVIC（NVIC 窗口 0xE100-0xE4FF 委托给 `nvic`）。
    pub fn new_with_mpu_nvic(size: u32, mpu: Arc<Mutex<Mpu>>, nvic: Arc<Mutex<Nvic>>) -> Self {
        Self {
            regs: vec![0; (size / 4) as usize],
            mpu: Some(mpu),
            nvic: Some(nvic),
        }
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
            if is_nvic_offset(offset) {
                return nvic.lock().unwrap().read(offset, size);
            }
        }
        if size != 4 {
            return Err(BusError::NotImplemented);
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
            if is_nvic_offset(offset) {
                return nvic.lock().unwrap().write(offset, size, value);
            }
        }
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let idx = (offset / 4) as usize;
        let slot = self.regs.get_mut(idx).ok_or(BusError::OutOfRange)?;
        *slot = value;
        Ok(())
    }
}
