//! USART 外设（STM32F407，M3 T1 集：USART1-3）。
//!
//! M3 语义简化（面向 printf demo）：
//! - 寄存器文件镜像：SR/DR/BRR/CR1/CR2/CR3/GTPR；
//! - 发送：TE+UE 使能后写 DR → 发布 [`crate::events::Event::UartByte`]
//!   （虚拟 Console 订阅），并立即置 SR.TXE（仿真快速发送，固件轮询 TXE 即通过）；
//! - 接收/错误/中断（RXNE/ORE/TC/TXEIE…）留待 M4。
//!
//! 地址映射（每个 USART 基址不同，`offset` 为相对基址偏移）：
//! - SR 0x00 / DR 0x04 / BRR 0x08 / CR1 0x0C / CR2 0x10 / CR3 0x14 / GTPR 0x18

use std::sync::{Arc, Mutex};

use crate::events::{Event, EventBus};
use crate::peripheral::{BusError, Peripheral};

/// SR 状态位
const SR_TXE: u32 = 1 << 7; // 发送数据寄存器空

/// CR1 控制位
const CR1_UE: u32 = 1 << 13; // 使能
const CR1_TE: u32 = 1 << 3;  // 发送使能

/// 寄存器偏移
const OFF_SR: u32 = 0x00;
const OFF_DR: u32 = 0x04;

/// USART 外设
pub struct Usart {
    /// USART 端口号（1/2/3），用于事件过滤
    pub port: u8,
    /// 寄存器文件
    regs: [u32; 7],
    /// 事件总线（发布 UartByte）
    bus: Arc<Mutex<EventBus>>,
}

impl Usart {
    pub fn new(port: u8, bus: Arc<Mutex<EventBus>>) -> Self {
        Self {
            port,
            regs: [0; 7],
            bus,
        }
    }

    /// 发送字节（发布 UartByte 事件；仅 TE+UE 生效）
    fn tx(&self, byte: u8) {
        let ev = Event::UartByte {
            port: self.port,
            byte,
        };
        self.bus.lock().unwrap().publish(&ev);
    }

    /// 发送数据寄存器是否空（固件轮询 TXE）
    pub fn tx_ready(&self) -> bool {
        self.regs[0] & SR_TXE != 0
    }
}

impl Peripheral for Usart {
    fn name(&self) -> &str {
        "USART"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_SR => Ok(self.regs[0]),
            OFF_DR => Ok(0), // 接收未实现，读回 0
            0x08..=0x18 => Ok(self.regs[(offset / 4) as usize]),
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_SR => {
                // 状态位写 0 清除（rc_w0 语义）；TXE 可被写 1 清除（写 0 保留）
                self.regs[0] &= value;
                Ok(())
            }
            OFF_DR => {
                // 发送数据寄存器：TE+UE 使能时发布 TX 事件
                let cr1 = self.regs[3]; // CR1 = regs 索引 3
                if (cr1 & (CR1_UE | CR1_TE)) == (CR1_UE | CR1_TE) {
                    self.tx((value & 0xFF) as u8);
                }
                // 仿真快速发送：数据立即被取走 → TXE 重新置位
                self.regs[0] |= SR_TXE;
                Ok(())
            }
            0x08..=0x18 => {
                self.regs[(offset / 4) as usize] = value;
                // CR1 使能 TE 上升沿 → TXE 置位（首字符轮询即可通过）
                if offset == 0x0C && (value & CR1_TE) != 0 && (value & CR1_UE) != 0 {
                    self.regs[0] |= SR_TXE;
                }
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.regs = [0; 7];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn usart() -> (Usart, Arc<Mutex<EventBus>>) {
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let u = Usart::new(2, bus.clone());
        (u, bus)
    }

    #[test]
    fn tx_publishes_uart_byte_events() {
        let (mut u, bus) = usart();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::UartByte { .. } = ev {
                    g.lock().unwrap().push(ev.clone());
                }
            })));

        // 未使能时不发布
        u.write(OFF_DR, 4, b'H' as u32).unwrap();
        assert_eq!(got.lock().unwrap().len(), 0);

        // 使能 UE+TE 后发布
        u.write(0x0C, 4, CR1_UE | CR1_TE).unwrap();
        assert!(u.tx_ready(), "TE 使能后 TXE 应置位");
        u.write(OFF_DR, 4, b'H' as u32).unwrap();
        u.write(OFF_DR, 4, b'i' as u32).unwrap();
        assert!(u.tx_ready(), "发送后 TXE 应保持置位（快速发送）");

        let got = got.lock().unwrap();
        assert_eq!(got.len(), 2);
        assert_eq!(got[0], Event::UartByte { port: 2, byte: b'H' });
        assert_eq!(got[1], Event::UartByte { port: 2, byte: b'i' });
    }
}
