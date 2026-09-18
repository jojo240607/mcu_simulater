//! MCU 时间后端抽象——把「推进 dt 毫秒」与具体仿真器/硬件解耦。
//!
//! 背景：闭环测试若用裸 [`Machine::run_budget`] 表达时间，参数是"退休字节"预算，
//! 只在「固定仿真器 + 固定代码块混合比」下才近似等于时间（实测同预算的 bytes/ms
//! 在 100K~109K 间浮动）。换仿真器/固件版本即失效，于是每个测试都要手改。
//!
//! 本模块把时间收敛到**一个入口** [`McuClock::advance_ms`]：
//! - 测试只表达 `dt`（毫秒），任何地方都不出现字节数；
//! - 后端实现一次；换 Unicorn / QEMU / FPGA / 真板只改这一个实现；
//! - [`SimClock`] 按固件自身 SysTick 收敛（固件时钟与后端无关），
//!   因此对齐不依赖任何字节换算常量。

use std::sync::{Arc, Mutex};

use crate::core::Result;
use crate::machine::Machine;

/// MCU 时间后端：把「推进 dt 毫秒」与具体仿真器/硬件解耦。
pub trait McuClock {
    /// 推进 MCU 恰好 `dt_ms` 毫秒（同一时间基；**绝不是**指令/字节预算）。
    fn advance_ms(&mut self, dt_ms: f64) -> Result<()>;
    /// 后端当前时间（毫秒）。供闭环对齐断言（固件时钟/场景时钟 ≈ 1）。
    fn now_ms(&mut self) -> f64;
}

/// Unicorn 仿真后端：`run_ms` 按固件自身 SysTick 收敛推进。
pub struct SimClock {
    m: Arc<Mutex<Machine>>,
}

impl SimClock {
    pub fn new(m: Arc<Mutex<Machine>>) -> Self {
        Self { m }
    }
}

impl McuClock for SimClock {
    fn advance_ms(&mut self, dt_ms: f64) -> Result<()> {
        self.m.lock().unwrap().run_ms(dt_ms as f32)
    }
    fn now_ms(&mut self) -> f64 {
        self.m.lock().unwrap().systick_ms() as f64
    }
}

/// 直接以 `Machine` 为时间后端（拥有 `Machine` 的用例，如 `EnvHarness`）。
impl McuClock for Machine {
    fn advance_ms(&mut self, dt_ms: f64) -> Result<()> {
        self.run_ms(dt_ms as f32)
    }
    fn now_ms(&mut self) -> f64 {
        self.systick_ms() as f64
    }
}

/// 闭环步进器：一次 [`HilStepper::step`] = 场景推进 `dt_ms` + MCU 推进同一 `dt_ms`。
///
/// 测试只声明一个 `dt_ms`，两边共用，从结构上杜绝「物理推进 4ms、固件推进 3.3ms」
/// 这类时钟失配。**`dt_ms` 应为整数毫秒**：固件 SysTick 是 1ms 粒度，`run_ms`
/// 按整数拍收敛（非整数会向下取整，长期会漂）。
pub struct HilStepper<C: McuClock> {
    clock: C,
    dt_ms: f64,
}

impl<C: McuClock> HilStepper<C> {
    pub fn new(clock: C, dt_ms: f64) -> Self {
        Self { clock, dt_ms }
    }

    pub fn dt_ms(&self) -> f64 {
        self.dt_ms
    }

    /// 推进一步：先按 `dt_ms` 推进场景（闭包），再让 MCU 推进同一 `dt_ms`。
    pub fn step(&mut self, advance_scene: impl FnOnce(f64)) -> Result<()> {
        advance_scene(self.dt_ms);
        self.clock.advance_ms(self.dt_ms)
    }

    pub fn clock(&mut self) -> &mut C {
        &mut self.clock
    }
}
