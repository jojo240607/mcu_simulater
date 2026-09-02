//! EXTI 外部中断/事件控制器（STM32F407，M4 T2 增强）。
//!
//! 基址 `0x40013C00`（APB2）。寄存器（32 位，每线 1 位）：
//! - IMR   @ 0x00 中断屏蔽（1 = 开放 → 触发沿送 NVIC）
//! - EMR   @ 0x04 事件屏蔽（本实现仅存档，事件输出不建模）
//! - RTSR  @ 0x08 上升沿触发选择
//! - FTSR  @ 0x0C 下降沿触发选择
//! - SWIER @ 0x10 软件中断事件（写 1 → 置对应 PR 位，等价外部触发）
//! - PR    @ 0x14 挂起标志（写 1 清 0，读返回挂起）
//!
//! 外部线 0..15 由 SYSCFG_EXTICR 选择接入的 GPIO 端口；线 16..22 为
//! 固定功能（本实现暂不建模）。触发语义：线端口匹配 + 电平变化 + 对应
//! 触发沿使能 → PR 0→1；若 IMR 开放则向 NVIC 置挂起（PR 已置位时不再
//! 重复拉高，符合硬件"挂起位清除前不产生新脉冲"的行为）。
//!
//! IRQ 映射（STM32F407）：EXTI0-4 → IRQ6-10；EXTI9_5 → IRQ23；
//! EXTI15_10 → IRQ40。

use std::sync::{Arc, Mutex};

use crate::peripheral::nvic::Nvic;
use crate::peripheral::syscfg::ExtiPortSelect;
use crate::peripheral::{BusError, Peripheral};

/// EXTI 基址（APB2）
pub const EXTI_BASE: u32 = 0x4001_3C00;

/// 寄存器偏移
const OFF_IMR: u32 = 0x00;
const OFF_EMR: u32 = 0x04;
const OFF_RTSR: u32 = 0x08;
const OFF_FTSR: u32 = 0x0C;
const OFF_SWIER: u32 = 0x10;
const OFF_PR: u32 = 0x14;

/// 外部线 0..15 对应的 NVIC IRQ 编号
pub fn line_irq(line: u8) -> Option<u32> {
    match line {
        0..=4 => Some(6 + line as u32), // EXTI0-4 → IRQ6-10
        5..=9 => Some(23),              // EXTI9_5 → IRQ23
        10..=15 => Some(40),            // EXTI15_10 → IRQ40
        _ => None,
    }
}

/// EXTI 外设核心
pub struct Exti {
    imr: u32,
    emr: u32,
    rtsr: u32,
    ftsr: u32,
    swier: u32,
    pr: u32,
    /// 线 0..15 当前电平（沿检测用，复位默认低）
    last_level: [bool; 16],
    /// SYSCFG_EXTICR 端口选择（与 SYSCFG 外设共享）
    port_select: Arc<Mutex<ExtiPortSelect>>,
    /// NVIC（触发沿 → 置挂起）
    nvic: Arc<Mutex<Nvic>>,
}

impl Exti {
    pub fn new(port_select: Arc<Mutex<ExtiPortSelect>>, nvic: Arc<Mutex<Nvic>>) -> Self {
        Self {
            imr: 0,
            emr: 0,
            rtsr: 0,
            ftsr: 0,
            swier: 0,
            pr: 0,
            last_level: [false; 16],
            port_select,
            nvic,
        }
    }

    /// 由 GPIO 电平事件驱动对应 EXTI 线（模拟外部输入）。
    /// 线 0..15；端口须与 SYSCFG_EXTICR 选择一致。
    pub fn feed_gpio(&mut self, port: u8, pin: u8, level: bool) {
        let line = pin as usize;
        if line >= 16 {
            return;
        }
        // 端口过滤（SYSCFG_EXTICR 选择）
        let selected = self.port_select.lock().unwrap().port[line];
        if selected != port {
            return;
        }
        // 沿检测：仅电平变化触发
        if self.last_level[line] == level {
            return;
        }
        self.last_level[line] = level;
        // 触发沿使能检查
        let mask = 1u32 << line;
        let armed = if level {
            self.rtsr & mask != 0
        } else {
            self.ftsr & mask != 0
        };
        if !armed {
            return;
        }
        self.trigger_line(line as u8);
    }

    /// 触发一条线：PR 0→1 时置挂起；PR 已置位则保持（不重复拉高）。
    fn trigger_line(&mut self, line: u8) {
        let mask = 1u32 << line;
        if self.pr & mask != 0 {
            return;
        }
        self.pr |= mask;
        if self.imr & mask != 0 {
            if let Some(irq) = line_irq(line) {
                self.nvic.lock().unwrap().set_pending(irq);
            }
        }
    }
}

impl Default for Exti {
    fn default() -> Self {
        Self::new(Arc::new(Mutex::new(ExtiPortSelect::default())), Arc::new(Mutex::new(Nvic::new())))
    }
}

impl Peripheral for Exti {
    fn name(&self) -> &str {
        "EXTI"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let v = match offset {
            OFF_IMR => self.imr,
            OFF_EMR => self.emr,
            OFF_RTSR => self.rtsr,
            OFF_FTSR => self.ftsr,
            OFF_SWIER => 0, // 写后即清（无状态读回）
            OFF_PR => self.pr,
            _ => return Err(BusError::OutOfRange),
        };
        Ok(v)
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_IMR => self.imr = value,
            OFF_EMR => self.emr = value,
            OFF_RTSR => self.rtsr = value,
            OFF_FTSR => self.ftsr = value,
            OFF_SWIER => {
                // 写 1 置对应 PR 位（软件触发，等价外部触发沿）
                self.swier = value;
                for line in 0..16 {
                    if value & (1 << line) != 0 {
                        self.trigger_line(line);
                    }
                }
            }
            OFF_PR => {
                // 写 1 清对应挂起位
                self.pr &= !value;
            }
            _ => return Err(BusError::OutOfRange),
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.imr = 0;
        self.emr = 0;
        self.rtsr = 0;
        self.ftsr = 0;
        self.swier = 0;
        self.pr = 0;
        self.last_level = [false; 16];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup() -> (Exti, Arc<Mutex<Nvic>>, Arc<Mutex<ExtiPortSelect>>) {
        let port_select = Arc::new(Mutex::new(ExtiPortSelect::default()));
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let exti = Exti::new(port_select.clone(), nvic.clone());
        (exti, nvic, port_select)
    }

    #[test]
    fn swier_triggers_pending() {
        let (mut e, nvic, _) = setup();
        e.write(OFF_IMR, 4, 1).unwrap(); // 开放 EXTI0
        e.write(OFF_SWIER, 4, 1).unwrap(); // 软件触发 line0
        assert_eq!(e.pr & 1, 1);
        assert!(nvic.lock().unwrap().is_pending(6), "EXTI0 → IRQ6 应挂起");
        // 已挂起（PR 仍置位）时不重复拉高
        e.write(OFF_SWIER, 4, 1).unwrap();
        assert!(nvic.lock().unwrap().is_pending(6));
        // 写 1 清 PR
        e.write(OFF_PR, 4, 1).unwrap();
        assert_eq!(e.pr & 1, 0);
        // 清除后再次触发
        e.write(OFF_SWIER, 4, 1).unwrap();
        assert_eq!(e.pr & 1, 1);
    }

    #[test]
    fn imr_gates_nvic() {
        let (mut e, nvic, _) = setup();
        // IMR 未开放：SWIER 只置 PR，不送 NVIC
        e.write(OFF_SWIER, 4, 1).unwrap();
        assert_eq!(e.pr & 1, 1);
        assert!(!nvic.lock().unwrap().is_pending(6));
        // 开放后再次触发（先清 PR）
        e.write(OFF_PR, 4, 1).unwrap();
        e.write(OFF_IMR, 4, 1).unwrap();
        e.write(OFF_SWIER, 4, 1).unwrap();
        assert!(nvic.lock().unwrap().is_pending(6));
    }

    #[test]
    fn feed_gpio_edge_and_port_filter() {
        let (mut e, nvic, port_select) = setup();
        e.write(OFF_RTSR, 4, 1).unwrap(); // line0 上升沿触发
        e.write(OFF_IMR, 4, 1).unwrap();

        // 端口不匹配（EXTICR 默认 GPIOA=0，喂 GPIOB=1）→ 忽略
        e.feed_gpio(1, 0, true);
        assert_eq!(e.pr & 1, 0);
        assert!(!nvic.lock().unwrap().is_pending(6));

        // 端口匹配 GPIOA → 上升沿触发
        e.feed_gpio(0, 0, true);
        assert_eq!(e.pr & 1, 1);
        assert!(nvic.lock().unwrap().is_pending(6));

        // 同电平重复不触发（非沿）
        e.feed_gpio(0, 0, true);
        assert_eq!(e.pr & 1, 1);

        // 清 PR 后，仅 RTSR：下降沿不触发
        e.write(OFF_PR, 4, 1).unwrap();
        e.feed_gpio(0, 0, false);
        assert_eq!(e.pr & 1, 0);

        // 端口重映射：EXTICR 选择 GPIOB → 之后 GPIOB 生效、GPIOA 被忽略
        port_select.lock().unwrap().port[0] = 1;
        e.write(OFF_RTSR, 4, 1).unwrap();
        e.feed_gpio(0, 0, true); // GPIOA 已被重映射 → 忽略
        assert_eq!(e.pr & 1, 0);
        e.feed_gpio(1, 0, true); // GPIOB 生效 → 上升沿
        assert_eq!(e.pr & 1, 1);
    }

    #[test]
    fn line_irq_mapping() {
        assert_eq!(line_irq(0), Some(6));
        assert_eq!(line_irq(4), Some(10));
        assert_eq!(line_irq(5), Some(23));
        assert_eq!(line_irq(9), Some(23));
        assert_eq!(line_irq(10), Some(40));
        assert_eq!(line_irq(15), Some(40));
        assert_eq!(line_irq(16), None);
    }
}
