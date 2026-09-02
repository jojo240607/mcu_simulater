//! 虚拟终端（M5）：双向虚拟外设，作为 UART 的"终端"接线对象。
//!
//! - 显示方向（UART TX → 终端）：订阅 [`crate::events::Event::UartByte`]，
//!   由 Machine 的 `connect uart.tx -> terminal.rx` 建立订阅，字节累积到显示缓冲；
//! - 输入方向（终端键盘 → UART RX）：[`Terminal::type_char`] 发布
//!   [`crate::events::Event::UartRx`]，USART 已全局订阅并 `feed_rx` 注入
//!   （RXNE/中断），与显示方向共同构成完整回环。
//!
//! 与具体 USART 无耦合——只认事件总线端口号。

use std::sync::{Arc, Mutex};

use crate::events::{Event, EventBus};
use crate::peripheral::{BusError, Peripheral};

/// 虚拟终端
#[derive(Default)]
pub struct Terminal {
    /// 显示缓冲（UART TX 到达的字节流）
    display: Vec<u8>,
    /// 事件总线（type_char 发布 UartRx）
    bus: Option<Arc<Mutex<EventBus>>>,
}

impl Terminal {
    pub fn new(bus: Arc<Mutex<EventBus>>) -> Self {
        Self {
            display: Vec::new(),
            bus: Some(bus),
        }
    }

    /// 写入一个显示字节（UART TX → 终端，`connect uart.tx -> terminal.rx` 订阅回调）
    pub fn write_display(&mut self, byte: u8) {
        self.display.push(byte);
    }

    /// 终端键盘输入：向指定 UART 端口发布一个接收字节（→ 该 USART `feed_rx`）。
    pub fn type_char(&self, port: u8, byte: u8) {
        if let Some(bus) = &self.bus {
            bus.lock()
                .unwrap()
                .publish(&Event::UartRx { port, byte });
        }
    }

    /// 已显示字节数
    pub fn len(&self) -> usize {
        self.display.len()
    }

    pub fn is_empty(&self) -> bool {
        self.display.is_empty()
    }

    /// 取走全部显示字节（测试断言用）
    pub fn take_display(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.display)
    }

    /// 当前显示内容（调试用）
    pub fn display(&self) -> &[u8] {
        &self.display
    }
}

impl Peripheral for Terminal {
    fn name(&self) -> &str {
        "terminal"
    }

    fn read(&mut self, _offset: u32, _size: u32) -> Result<u32, BusError> {
        Err(BusError::NotImplemented)
    }

    fn write(&mut self, _offset: u32, _size: u32, _value: u32) -> Result<(), BusError> {
        Err(BusError::NotImplemented)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_accumulates_and_type_char_publishes_uart_rx() {
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let received = Arc::new(Mutex::new(Vec::<u8>::new()));
        {
            let rx = received.clone();
            bus.lock().unwrap().subscribe(Arc::new(Mutex::new(
                move |ev: &Event| {
                    if let Event::UartRx { byte, .. } = ev {
                        rx.lock().unwrap().push(*byte);
                    }
                },
            )));
        }
        let mut t = Terminal::new(bus);

        // 显示方向：字节累积
        t.write_display(b'T');
        t.write_display(b'x');
        assert_eq!(t.len(), 2);
        assert_eq!(t.display(), b"Tx");

        // 输入方向：type_char 发布 UartRx，订阅者收到（USART feed_rx 同一链路）
        t.type_char(4, b'a');
        t.type_char(4, b'b');
        assert_eq!(*received.lock().unwrap(), b"ab");
        assert_eq!(t.take_display(), b"Tx");
        assert!(t.is_empty());
    }
}
