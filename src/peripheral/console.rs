//! 虚拟 Console（M3 T1 集）：终端类虚拟外设。
//!
//! 订阅 USART 的 [`crate::events::Event::UartByte`] 事件，将字节累积到
//! 输出缓冲区（测试/虚拟终端读取）。与具体 USART 无耦合——通过事件总线
//! 由 Machine 的 `connect` 建立订阅关系（类 Renode `connect uart.tx -> console.rx`）。

use crate::peripheral::BusError;
use crate::peripheral::Peripheral;

/// 虚拟 Console
#[derive(Default)]
pub struct Console {
    /// 接收到的字节流
    output: Vec<u8>,
}

impl Console {
    pub fn new() -> Self {
        Self::default()
    }

    /// 写入一个字节（事件订阅回调调用）
    pub fn write_byte(&mut self, byte: u8) {
        self.output.push(byte);
    }

    /// 已接收字节数
    pub fn len(&self) -> usize {
        self.output.len()
    }

    pub fn is_empty(&self) -> bool {
        self.output.is_empty()
    }

    /// 取走全部字节（测试断言用）
    pub fn take_output(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.output)
    }

    /// 当前输出（调试用）
    pub fn output(&self) -> &[u8] {
        &self.output
    }
}

impl Peripheral for Console {
    fn name(&self) -> &str {
        "console"
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
    fn accumulate_bytes() {
        let mut c = Console::new();
        c.write_byte(b'H');
        c.write_byte(b'i');
        assert_eq!(c.len(), 2);
        assert_eq!(c.take_output(), b"Hi");
        assert!(c.is_empty());
    }
}
