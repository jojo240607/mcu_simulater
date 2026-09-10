//! SPI 虚拟从设备（vperiph 第三类总线器件，仿 `i2c/` 模式）。
//!
//! 目录组织（与 `super`（vperiph/mod.rs）文档一致）：
//! - `mod.rs`：`VirtualSpiSlave` trait（SPI 控制器直路由调用）+ 子模块声明 + 工厂 re-export
//! - `bmi088.rs`：BMI088 双片选六轴 IMU（ACCEL_CS/GYRO_CS，SPI 7bit 寄存器地址）
//!
//! 新增 SPI 器件落点：`spi/` 下建 `<device>.rs`，本文件加 `pub mod` + re-export。
//!
//! # 交互模型（全双工，控制器直拉，无事件转发）
//!
//! SPI 是全双工总线：主机（固件）每发一字节的同时从 MISO 收一字节。虚拟从机因此
//! 采用与 [`super::VirtualI2cSlave`] 相同的"控制器直路由"模式，由
//! [`crate::peripheral::spi::Spi`] 在 CPU/DMA 写 DR（发送字节）时直接调用：
//!
//! - 片选：固件用 GPIO 输出拉低 CS（无硬件 NSS）→ 模拟器 GPIO 外设发布
//!   [`crate::events::Event::GpioLevel`]，由 Machine 装配的订阅器转发到
//!   [`VirtualSpiSlave::on_cs`]（port/pin 匹配从机关注的 CS 引脚）。
//! - 字节流：CPU 写 SPI DR / DMA TX 搬运 → [`Spi`] 调 [`VirtualSpiSlave::on_byte`]，
//!   从机按帧协议（首字节 `reg<<1|rw`）解析并返回 MISO 回送字节，控制器锁存为
//!   RX（置 RXNE，DMA RX 走既有 SpiDma 路由）——与 feed_rx 同语义。
//!
//! 帧协议（BMI088 等 SPI 传感器通用）：CS 拉低开始一帧，首字节 = 7bit 寄存器地址
//! 左移 1 位 | R/W（1=读 0=写）；读帧后续字节按地址递增返回寄存器值（连续读）；
//! 写帧后续字节写入寄存器。CS 拉高结束帧（丢弃未完成部分）。

use std::sync::{Arc, Mutex};

pub mod bmi088;
pub mod flash;
pub mod pmw3901;

pub use bmi088::Bmi088;
pub use flash::SpiFlash;
pub use pmw3901::{FlowModel, Pwm3901, StaticFlow};

/// SPI 总线从设备接口。
///
/// 由 [`crate::peripheral::spi::Spi`] 在主机发送字节时直路由调用（全双工：
/// 每字节返回 MISO 回送值）；片选经 GPIO 事件由 Machine 转发。
pub trait VirtualSpiSlave: Send + Sync {
    /// 从设备名（观测/日志）
    fn name(&self) -> &str;

    /// 片选引脚电平变化（`port`/`pin` 为 GPIO 坐标；`level=false` = 拉低选中）。
    ///
    /// 从机自行过滤自己关注的引脚；拉低开始新帧（复位帧状态机），拉高结束帧。
    fn on_cs(&mut self, port: u8, pin: u8, level: bool);

    /// 主机发送一字节（全双工）：按帧协议解析并返回 MISO 回送字节。
    ///
    /// 未选中（CS 高）或帧未开始时不解析，回 `0xFF`（MISO 默认高）。
    fn on_byte(&mut self, byte: u8) -> u8;

    /// 是否被片选选中（CS 拉低 = 选中；同总线多从机按此路由字节流）。
    /// 默认 false（未选中）；实现类按自己的 CS 引脚状态返回。
    fn selected(&self) -> bool {
        false
    }

    /// 被访问次数（观测/断言：虚拟外设是否被固件访问）。
    fn access_count(&self) -> u64 {
        0
    }

    /// 仿真时间推进（Math 数据源步进；由 Machine 的 step_virtual_slaves 驱动）。
    fn step(&mut self, _dt: f32) {}

    /// 持久化（Machine 统一触发；无持久化能力的器件 no-op）。
    ///
    /// SPI NOR Flash 等"保存用途"器件借此把映像写回绑定文件，数据跨 run 存活。
    fn persist(&mut self) {}
}

/// 便捷构造（工厂命名与 `i2c/` 一致）：默认 BMI088（静态 IMU 模型）。
pub fn default_bmi088(accel_cs: (u8, u8), gyro_cs: (u8, u8)) -> Bmi088 {
    Bmi088::new(accel_cs, gyro_cs, crate::peripheral::vperiph::data_source::StaticImu::default())
}

/// 测试辅助：驱动一个 SPI 从机的读寄存器序列（首字节 `reg<<1|1` + `len` 个 dummy）。
///
/// 返回读取的 `len` 字节（dummy 位置的回送值；首字节回送被丢弃——SPI 全双工
/// 首字节的 MISO 无意义）。CS 拉低开始、拉高结束。
pub fn read_regs(s: &mut dyn VirtualSpiSlave, port: u8, pin: u8, reg: u8, len: usize) -> Vec<u8> {
    s.on_cs(port, pin, false);
    let _ = s.on_byte((reg << 1) | 1);
    let mut out = Vec::with_capacity(len);
    for _ in 0..len {
        out.push(s.on_byte(0xFF));
    }
    s.on_cs(port, pin, true);
    out
}

/// 测试辅助：向 SPI 从机写一个寄存器（首字节 `reg<<1|0` + 1 个数据字节）。
pub fn write_reg(s: &mut dyn VirtualSpiSlave, port: u8, pin: u8, reg: u8, value: u8) {
    s.on_cs(port, pin, false);
    let _ = s.on_byte(reg << 1);
    let _ = s.on_byte(value);
    s.on_cs(port, pin, true);
}

/// 断言辅助：把 SPI 从机包进 `Arc<Mutex<dyn VirtualSpiSlave>>`（控制器注册用）。
pub fn boxed(s: Bmi088) -> Arc<Mutex<dyn VirtualSpiSlave>> {
    Arc::new(Mutex::new(s))
}
