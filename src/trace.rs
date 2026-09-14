//! 总线事务嗅探器（调试平台 P0-1）。
//!
//! 记录 I2C/SPI/UART 总线上的**事务级**事件，供两类用途：
//! - **固件驱动调试**：读序列、寄存器指针、总线错误一眼可见（如
//!   "t=1.2ms i2c1 START R@0x68 → R 0x3B → R 0x41 …"），替代"读计数 + 猜"；
//! - **测试断言**：验证固件确实按期望时序访问了虚拟从设备。
//!
//! 设计：
//! - 环形缓冲（容量可配，默认 [`DEFAULT_MAX`]），`record` 在 enabled 时入队，
//!   超容量丢最旧——嗅探是调试辅助，不能拖慢/污染仿真主循环；
//! - 时间戳取**退休指令计数器**（`Arc<AtomicU64>`，与 CPU 虚拟时钟同一口径，
//!   `machine::Machine::retired_insts`，`attach_bus_trace` 注入）——原子类型
//!   可跨线程共享，且是调试日志最无歧义的时间基准（显示时按 CPU 时钟
//!   折算微秒）；
//! - 埋点位置在外设内部（事务语义只有外设知道：地址阶段/寄存器指针/CS 路由），
//!   外设持 `Option<Arc<Mutex<BusTrace>>>`（默认 None，零回归）。
//!
//! 锁序注意：外设埋点在 record 语句内短暂持 trace 锁，**不与其他锁嵌套**
//! （record 结束即释放，再发事件/访问总线），避免事件回调重入死锁。

use std::collections::VecDeque;
use std::sync::Arc;



/// 环形缓冲默认容量（条）。
pub const DEFAULT_MAX: usize = 4096;

/// 总线类型。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusKind {
    I2c,
    Spi,
    Usart,
}

impl BusKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            BusKind::I2c => "i2c",
            BusKind::Spi => "spi",
            BusKind::Usart => "uart",
        }
    }
}

/// 事务级事件。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TraceKind {
    /// I2C 起始 + 地址阶段：地址字节（7bit）+ 方向 + 是否命中从设备。
    I2cStart {
        addr7: u8,
        read: bool,
        matched: bool,
    },
    /// I2C 数据写（固件 → 从机；首字节通常是寄存器地址）。
    I2cWrite {
        byte: u8,
    },
    /// I2C 数据读（从机 → 固件；`None` = 从设备 NACK/无数据）。
    I2cRead {
        byte: Option<u8>,
    },
    /// I2C STOP。
    I2cStop,
    /// SPI 片选变化（`level=false` = 拉低选中）。
    SpiCs {
        pin: u8,
        level: bool,
    },
    /// SPI 全双工字节：发送 `tx`，回送 `rx`（无选中从机时 0xFF）。
    SpiByte {
        tx: u8,
        rx: u8,
    },
    /// UART 接收字节（虚拟从设备推流 / 注入 → 固件读）。
    UartRx {
        byte: u8,
    },
    /// UART 发送字节（固件 → 外部）。
    UartTx {
        byte: u8,
    },
    /// UART 帧结束（IDLE，固件 flush 缓冲）。
    UartIdle,
}

/// 单条事务记录。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TraceEntry {
    /// 退休指令数（CPU 虚拟时钟口径；`attach_bus_trace` 注入 retired 计数器，未注入为 0）。
    pub retired: u64,
    /// 总线类型。
    pub bus: BusKind,
    /// 端口号（I2C1-3 / SPI1-3 / USART1-6，1 起）。
    pub port: u8,
    /// 事件。
    pub kind: TraceKind,
}

impl TraceEntry {
    /// 虚拟时间（微秒，按 CPU 虚拟时钟折算：~46M 退休指令/虚拟秒，`x_sys_retire_calib`
    /// 实测 SysTick 168000 周期 ≈ 4~6 万退休指令 ≈ 1ms 取均值 4.6 万）。
    pub fn time_us(&self) -> f64 {
        self.retired as f64 / 46.0e3
    }

    /// 人类可读单行格式。
    pub fn format(&self) -> String {
        let t = self.time_us();
        let head = format!("[{:>10.3}us] {}{}", t, self.bus.as_str(), self.port);
        match &self.kind {
            TraceKind::I2cStart { addr7, read, matched } => format!(
                "{} START {}@{:#04x} {}",
                head,
                if *read { "R" } else { "W" },
                addr7,
                if *matched { "match" } else { "AF" }
            ),
            TraceKind::I2cWrite { byte } => format!("{} W {:#04x}", head, byte),
            TraceKind::I2cRead { byte: Some(b) } => format!("{} R {:#04x}", head, b),
            TraceKind::I2cRead { byte: None } => format!("{} R NACK", head),
            TraceKind::I2cStop => format!("{} STOP", head),
            TraceKind::SpiCs { pin, level } => format!(
                "{} CS pin{} {}",
                head,
                pin,
                if *level { "HIGH" } else { "LOW" }
            ),
            TraceKind::SpiByte { tx, rx } => format!("{} X TX={:#04x} RX={:#04x}", head, tx, rx),
            TraceKind::UartRx { byte } => format!("{} RX {:#04x} '{}'", head, byte, printable(*byte)),
            TraceKind::UartTx { byte } => format!("{} TX {:#04x} '{}'", head, byte, printable(*byte)),
            TraceKind::UartIdle => format!("{} IDLE", head),
        }
    }
}

/// 可打印字符（否则显示 `.`）。
fn printable(b: u8) -> char {
    if (0x20..0x7F).contains(&b) {
        b as char
    } else {
        '.'
    }
}

/// 总线事务嗅探器（环形缓冲）。
///
/// 外设经 `Option<Arc<Mutex<BusTrace>>>` 持有；Machine 装配时注入
/// （`attach_bus_trace`），测试/调试直接持有同一 Arc 读取。
pub struct BusTrace {
    enabled: bool,
    max: usize,
    entries: VecDeque<TraceEntry>,
    /// 退休指令计数器（时间戳来源；可选，未注入则时间 0）。
    retired: Option<Arc<std::sync::atomic::AtomicU64>>,
}

impl BusTrace {
    pub fn new(max: usize) -> Self {
        Self {
            enabled: true,
            max,
            entries: VecDeque::with_capacity(max.min(64)),
            retired: None,
        }
    }

    /// 默认容量嗅探器。
    pub fn default_max() -> Self {
        Self::new(DEFAULT_MAX)
    }

    /// 注入退休指令计数器（时间戳来源）。
    pub fn set_retired(&mut self, retired: Option<Arc<std::sync::atomic::AtomicU64>>) {
        self.retired = retired;
    }

    /// 开/关嗅探（关时 record 为 no-op，不影响性能）。
    pub fn set_enabled(&mut self, on: bool) {
        self.enabled = on;
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// 记录一条事务（enabled 时入队，超容量丢最旧）。
    pub fn record(&mut self, bus: BusKind, port: u8, kind: TraceKind) {
        if !self.enabled {
            return;
        }
        let retired = self
            .retired
            .as_ref()
            .map(|c| c.load(std::sync::atomic::Ordering::Relaxed))
            .unwrap_or(0);
        if self.entries.len() >= self.max {
            self.entries.pop_front();
        }
        self.entries.push_back(TraceEntry {
            retired,
            bus,
            port,
            kind,
        });
    }

    /// 当前记录数。
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// 取出全部记录（清空缓冲）。
    pub fn drain(&mut self) -> Vec<TraceEntry> {
        self.entries.drain(..).collect()
    }

    /// 取出全部记录并格式化为多行字符串。
    pub fn drain_formatted(&mut self) -> String {
        self.drain()
            .iter()
            .map(|e| e.format())
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// 清空缓冲（保留开关/时钟）。
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

impl Default for BusTrace {
    fn default() -> Self {
        Self::default_max()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_and_drain_roundtrip() {
        let mut t = BusTrace::new(8);
        t.record(BusKind::I2c, 1, TraceKind::I2cStart { addr7: 0x68, read: false, matched: true });
        t.record(BusKind::I2c, 1, TraceKind::I2cWrite { byte: 0x3B });
        t.record(BusKind::I2c, 1, TraceKind::I2cRead { byte: Some(0x41) });
        let v = t.drain();
        assert_eq!(v.len(), 3);
        assert_eq!(
            v[0].kind,
            TraceKind::I2cStart { addr7: 0x68, read: false, matched: true }
        );
        assert!(v[0].format().contains("i2c1 START W@0x68 match"));
        assert!(v[1].format().contains("W 0x3b"));
        assert!(v[2].format().contains("R 0x41"));
    }

    #[test]
    fn ring_capacity_drops_oldest() {
        let mut t = BusTrace::new(4);
        for i in 0..6 {
            t.record(BusKind::Spi, 1, TraceKind::SpiByte { tx: i, rx: 0xFF });
        }
        let v = t.drain();
        assert_eq!(v.len(), 4);
        assert_eq!(v[0].kind, TraceKind::SpiByte { tx: 2, rx: 0xFF });
        assert_eq!(v[3].kind, TraceKind::SpiByte { tx: 5, rx: 0xFF });
    }

    #[test]
    fn disabled_is_noop() {
        let mut t = BusTrace::new(8);
        t.set_enabled(false);
        t.record(BusKind::Usart, 2, TraceKind::UartRx { byte: b'G' });
        assert!(t.is_empty());
        t.set_enabled(true);
        t.record(BusKind::Usart, 2, TraceKind::UartRx { byte: b'P' });
        assert_eq!(t.len(), 1);
    }

    #[test]
    fn uart_format_printable_escape() {
        let mut t = BusTrace::new(4);
        t.record(BusKind::Usart, 3, TraceKind::UartRx { byte: 0x0A });
        let s = t.drain_formatted();
        assert!(s.contains("RX 0x0a '."));
    }
}
