//! UART 总线虚拟从设备：主动推流（GPS NMEA / SBUS 遥控帧）。
//!
//! 由 [`crate::peripheral::usart::Usart`] 持有、Machine 仿真循环按迭代推进：
//! `step(dt)` 累积帧节拍，到期时经 `tx` 回调把帧字节逐个喂入
//! [`Usart::feed_rx`]（复用现有 RXNE/DMA/中断链路，固件驱动零改动读取）。
//!
//! 新增 UART 器件落点：在 `uart/` 下新建 `<device>.rs`（帧构造 + `VirtualUartSlave`
//! 实现 + 单元测试），然后在 `mod.rs` 加 `pub mod <device>;` 并 re-export 即可。

pub mod nmea_gps;
pub mod sbus;

pub use nmea_gps::NmeaGps;
pub use sbus::Sbus;

/// UART 总线从设备接口。
///
/// `tx` 回调把一字节喂入对应 UART 的接收缓冲（模拟器内部 feed_rx）。
pub trait VirtualUartSlave: Send + Sync {
    /// 从设备名（观测/日志）
    fn name(&self) -> &str;

    /// 推进推流节拍（`dt` 秒）；到期推帧经 `tx` 喂入。
    fn step(&mut self, dt: f32, tx: &mut dyn FnMut(u8));

    /// 已推帧数（观测/断言：推流是否在跑）。
    fn frames(&self) -> u64 {
        0
    }
}

/// NMEA-0183 校验和（`$` 与 `*` 之间所有字节异或）。
pub(crate) fn nmea_checksum(body: &[u8]) -> u8 {
    body.iter().fold(0u8, |a, &b| a ^ b)
}
