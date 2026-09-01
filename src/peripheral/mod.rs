//! 外设接口与注册表。
//!
//! 任何外设实现 [`Peripheral`] trait 即可挂载到内存总线。
//! M0 仅定义接口，M1 落地总线转发链路，具体外设（GPIO/UART/TIM/NVIC/MPU…）在 M2/M3 实现。

pub mod mpu;
pub mod nvic;
pub mod scb;

/// 总线访问错误
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusError {
    /// 地址未映射到任何设备
    Unmapped(u32),
    /// 访问越界（超出外设寄存器区间）
    OutOfRange,
    /// 注册区间与已有区间重叠
    Overlap,
    /// 未实现的操作
    NotImplemented,
}

/// 外设统一接口。
///
/// `Send + Sync` 约束保证外设可通过 `Arc<Mutex<dyn Peripheral>>` 跨线程共享
/// （GDB/monitor 独立线程读取寄存器状态，以及 mem hook 闭包内转发访问）。
pub trait Peripheral: Send + Sync {
    /// 外设名称（调试/日志用）
    fn name(&self) -> &str;

    /// 读寄存器（`offset` 为相对外设基址的偏移，`size` 为访问宽度 1/2/4 字节）
    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError>;

    /// 写寄存器
    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError>;

    /// 复位
    fn reset(&mut self) {}

    /// 周期推进回调（供 TIM/SysTick 等按周期语义推进，M2 启用）
    fn tick(&mut self, _cycles: u64) {}

    /// 中断输出：外设主动拉起的 irq 编号（M2 启用）
    fn irq_line(&self) -> Option<u32> {
        None
    }
}
