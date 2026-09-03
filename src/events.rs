//! 事件总线：虚拟外设互联（M3 启用，M0 仅占位）。
//!
//! 数据流走中央事件总线，DSL `connect a.tx -> b.rx` 语法糖映射为订阅关系，
//! 使终端、LED 面板等虚拟外设可无侵入接入。

use std::sync::{Arc, Mutex};

use crate::peripheral::can::CanFrame;
use crate::peripheral::dma::DmaDir;

/// 事件类型（M0 占位，M3 起按外设扩展）
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// UART 发送一字节（USART TX → 虚拟终端/Console 订阅）
    UartByte { port: u8, byte: u8 },
    /// UART 接收一字节（虚拟终端/测试 → USART RX，驱动 RXNE/中断）
    UartRx { port: u8, byte: u8 },
    /// UART DMA 请求（USART → DMA 控制器，触发外设↔内存搬运）
    UartDma { port: u8, dir: DmaDir },
    /// I2C 发送一字节（I2C TX → 虚拟从机/测试订阅）
    I2cByte { port: u8, byte: u8 },
    /// I2C 接收一字节（测试/虚拟从机 → I2C RX，驱动 RxNE）
    I2cRx { port: u8, byte: u8 },
    /// I2C DMA 请求（I2C → DMA 控制器，触发外设↔内存搬运）
    I2cDma { port: u8, dir: DmaDir },
    /// SPI 发送一字节（SPI TX → 虚拟从机/测试订阅）
    SpiByte { port: u8, byte: u8 },
    /// SPI 接收一字节（测试/虚拟从机 → SPI RX，驱动 RXNE）
    SpiRx { port: u8, byte: u8 },
    /// SPI DMA 请求（SPI → DMA 控制器，触发外设↔内存搬运）
    SpiDma { port: u8, dir: DmaDir },
    /// ADC 模拟采样（测试/虚拟传感器 → ADC，驱动一次转换：DR 锁存 + EOC + DMA 请求）
    AdcValue { port: u8, channel: u8, value: u16 },
    /// DAC 输出电平变化（DHR→DOR 转换完成 → 虚拟示波器/测试订阅，12 位）
    DacLevel { port: u8, channel: u8, level: u16 },
    /// DAC DMA 请求（触发转换且 CR.DMAEN 置位 → 内存→外设搬运：DMA 写 DHR 再转换）
    DacDma { port: u8, channel: u8, dir: DmaDir },
    /// TIM 更新事件（TIM 计数溢出/软件更新 → Machine 路由 DMA 请求。
    /// 仅 DIER.UDE 使能时发布，等价硬件"更新事件 → DMA 请求"）
    TimUpdate { port: u8 },
    /// TIM PWM/输出比较电平变化（OCxREF 变化 → 虚拟示波器/GPIO 接线订阅）
    TimPwm { port: u8, channel: u8, level: bool },
    /// GPIO 电平变化
    GpioLevel { port: u8, pin: u8, level: bool },
    /// DCMI 帧数据（测试/虚拟摄像头 → DCMI，注入一帧图像数据；驱动 SR.FNE + RIS.FRAME）
    DcmiFrame { port: u8, data: Vec<u8> },
    /// SDIO DMA 请求（SDIO → DMA2；读=外设→内存、写=内存→外设。
    /// items>0 表示读 FIFO 现成字数；items=0 表示写方向，由 DMA 侧取流 NDTR）
    SdioDma { port: u8, dir: DmaDir, items: u32 },
    /// CAN 帧总线级互联（CAN TX 发布 → 对端 CAN/测试订阅；驱动接收 FIFO + 中断）
    CanFrame { frame: Box<CanFrame> },
    /// 占位：其它事件后续按需扩展
    Other(String),
}

/// 订阅回调
pub type Subscriber = Arc<Mutex<dyn FnMut(&Event) + Send + Sync>>;

/// 事件总线
#[derive(Default)]
pub struct EventBus {
    subscribers: Vec<Subscriber>,
}

impl EventBus {
    pub fn new() -> Self {
        Self::default()
    }

    /// 发布事件：分发给所有订阅者
    pub fn publish(&self, ev: &Event) {
        for s in &self.subscribers {
            (s.lock().unwrap())(ev);
        }
    }

    /// 订阅事件
    pub fn subscribe(&mut self, sub: Subscriber) {
        self.subscribers.push(sub);
    }

    /// 订阅者数量（调试用）
    pub fn subscriber_count(&self) -> usize {
        self.subscribers.len()
    }
}
