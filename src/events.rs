//! 事件总线：虚拟外设互联（M3 启用，M0 仅占位）。
//!
//! 数据流走中央事件总线，DSL `connect a.tx -> b.rx` 语法糖映射为订阅关系，
//! 使终端、LED 面板等虚拟外设可无侵入接入。

use std::sync::{Arc, Mutex};

/// 事件类型（M0 占位，M3 起按外设扩展）
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// UART 收到/发送一字节
    UartByte { port: u8, byte: u8 },
    /// GPIO 电平变化
    GpioLevel { port: u8, pin: u8, level: bool },
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
