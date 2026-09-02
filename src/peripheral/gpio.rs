//! GPIO 外设（STM32F407 通用 IO）。
//!
//! M3 T1 集：寄存器文件式外设，支持模式/输出数据/置位复位，
//! 输出引脚电平变化时发布 [`crate::events::Event::GpioLevel`] 事件
//! （供 blinky demo 与 LED 面板等虚拟外设互联）。
//!
//! 地址映射（每组 GPIO 基址间隔 0x400，如 GPIOA=0x40020000）：
//! - MODER 0x00 / OTYPER 0x04 / OSPEEDR 0x08 / PUPDR 0x0C
//! - IDR 0x10（只读）/ ODR 0x14 / BSRR 0x18（只写置位复位）
//! - LCKR 0x1C / AFRL 0x20 / AFRH 0x24
//!
//! M3 语义简化：IDR 读回当前 ODR（输出自环）；输入电平的注入（外部事件）
//! 留待 M4；BSRR 写 1 置位/复位对应 ODR 位，仅对输出引脚发布事件。

use std::sync::{Arc, Mutex};

use crate::events::{Event, EventBus};
use crate::peripheral::{BusError, Peripheral};

/// MODER 每位模式值（每引脚 2 位）
const MODE_OUTPUT: u32 = 1;

/// 寄存器偏移
const OFF_MODER: u32 = 0x00;
const OFF_IDR: u32 = 0x10;
const OFF_ODR: u32 = 0x14;
const OFF_BSRR: u32 = 0x18;

/// GPIO 外设
pub struct Gpio {
    /// 端口号（0=GPIOA, 1=GPIOB, …），用于事件过滤
    pub port: u8,
    /// 寄存器文件（MODER..AFRH 共 10 个 32 位寄存器）
    regs: [u32; 10],
    /// 当前输出电平（ODR 位）
    odr: u32,
    /// 事件总线（发布 GpioLevel）
    bus: Arc<Mutex<EventBus>>,
}

impl Gpio {
    pub fn new(port: u8, bus: Arc<Mutex<EventBus>>) -> Self {
        Self {
            port,
            regs: [0; 10],
            odr: 0,
            bus,
        }
    }

    /// 当前输出电平（ODR 位，测试/虚拟外设读取用）
    pub fn output_level(&self) -> u32 {
        self.odr
    }

    /// 某引脚是否配置为输出模式
    fn is_output(&self, pin: u32) -> bool {
        (self.regs[0] >> (pin * 2)) & 0x3 == MODE_OUTPUT
    }

    /// 引脚电平变化 → 发布事件（仅输出引脚）
    fn publish_change(&self, pin: u32, level: bool) {
        if self.is_output(pin) {
            let ev = Event::GpioLevel {
                port: self.port,
                pin: pin as u8,
                level,
            };
            self.bus.lock().unwrap().publish(&ev);
        }
    }

    /// ODR 变化：逐位对比发布电平变化事件
    fn apply_odr(&mut self, new_odr: u32) {
        let changed = self.odr ^ new_odr;
        self.odr = new_odr;
        for pin in 0..16u32 {
            if changed & (1 << pin) != 0 {
                let level = new_odr & (1 << pin) != 0;
                self.publish_change(pin, level);
            }
        }
    }
}

impl Peripheral for Gpio {
    fn name(&self) -> &str {
        "GPIO"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_MODER..=0x0C => Ok(self.regs[(offset / 4) as usize]), // MODER..PUPDR
            OFF_IDR => Ok(self.odr),                                  // IDR 读回当前输出电平
            OFF_ODR => Ok(self.odr),
            0x18..=0x24 => Ok(0), // BSRR/LCKR/AFR 读回 0（只写/保留语义）
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_MODER..=0x0C => { // MODER..PUPDR
                self.regs[(offset / 4) as usize] = value;
                Ok(())
            }
            OFF_ODR => {
                self.apply_odr(value);
                Ok(())
            }
            OFF_BSRR => {
                // 低 16 位置位，高 16 位复位
                let new_odr = (self.odr | (value & 0xFFFF))
                    & !(value >> 16);
                self.apply_odr(new_odr);
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.regs = [0; 10];
        self.odr = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gpio() -> (Gpio, Arc<Mutex<EventBus>>) {
        let bus = Arc::new(Mutex::new(EventBus::new()));
        let g = Gpio::new(0, bus.clone());
        (g, bus)
    }

    #[test]
    fn odr_bsrr_toggle_publishes_events() {
        let (mut g, bus) = gpio();
        // 记录发布事件
        let events = Arc::new(Mutex::new(Vec::new()));
        let evs = events.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::GpioLevel { .. } = ev {
                    evs.lock().unwrap().push(ev.clone());
                }
            })));

        // 引脚 5 配置为输出
        g.write(OFF_MODER, 4, MODE_OUTPUT << (5 * 2)).unwrap();
        // BSRR 置位引脚 5
        g.write(OFF_BSRR, 4, 1 << 5).unwrap();
        assert_eq!(g.output_level(), 1 << 5);
        // BSRR 复位引脚 5
        g.write(OFF_BSRR, 4, 1 << (5 + 16)).unwrap();
        assert_eq!(g.output_level(), 0);

        let evs = events.lock().unwrap();
        assert_eq!(evs.len(), 2, "置位+复位应各发布一次事件");
        assert_eq!(evs[0], Event::GpioLevel { port: 0, pin: 5, level: true });
        assert_eq!(evs[1], Event::GpioLevel { port: 0, pin: 5, level: false });
    }
}
