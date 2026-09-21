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

// ============================ 锁相步进（控制拍为时基） ============================

/// 固件控制拍计数器的**符号名**（ELF 解析，不硬编码地址）。
const CTRL_TICKS_SYM: &str = "CTRL_TICKS";

fn read_u32(m: &mut Machine, addr: u64) -> u32 {
    let b = m.cpu.mem_read(addr, 4).unwrap_or_else(|_| vec![0; 4]);
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

/// 推进 MCU 直到**恰好完成一拍控制**（`CTRL_TICKS` +1），返回该拍耗时（固件 ms）。
///
/// # 为何需要它（这是 M 场“代码布局灵敏度”的根因）
///
/// [`HilStepper`] 让 MCU 按**固定 `dt_ms`** 推进，而固件控制拍有它**自己的、抖动的**
/// 周期。实测（亚毫秒分辨率，用退休字节当时钟，量化仅 0.148ms）：
///
/// ```text
/// 周期：均值 4.0000ms（250.00Hz）  标准差 0.8426ms  min 2.2439  max 5.2224
/// ```
///
/// ⇒ 两个时基**相位自由漂移** ⇒ **PWM 回读落在控制周期内的相位随机**
/// ⇒ 任何代码改动只要移动零点几 ms，就会改变唤醒的量化图案 → 改变相位 → 改变结果。
/// **这与算力余量无关**（实测计算仅占 26%、空转 74%）；之前“固件没时序余量”的
/// 归因是错的（那个结论建立在虚高 1.47× 的字节换算常量上）。
///
/// # 本函数的做法与为何不会漂
///
/// 把“控制拍”钉成唯一时基：**每完成一拍才返回**，调用方据此推进一次物理/场景。
/// 于是 PWM 总在“拍刚结束”被采样，**相位固定**。
/// 又因实测周期**均值恰好 4.0000ms**，调用方每拍推进名义 `dt_ms` 与之 1:1，
/// 长期不漂（漂移只会来自均值≠`dt_ms`，可用对齐断言监控）。
///
/// 注：本函数**不改变固件行为**——它只用 [crate::machine::Machine::run_budget] 分段
/// 推进同一台机器，不做任何写入。
///
/// 返回：该拍实际推进的固件毫秒数（整数，SysTick 粒度）。
pub fn run_one_control_tick(m: &mut Machine) -> Result<f64> {
    /// 轮询粒度（退休字节）：≈0.15ms 固件时间，远细于控制拍。
    const POLL_BYTES: usize = 20_000;
    /// 安全阀：按 0.15ms/次估算可覆盖 ~3s 固件时间，正常一拍只要 ~27 次。
    const MAX_POLLS: u32 = 20_000;

    let addr = crate::elfsym::app_sym(CTRL_TICKS_SYM) as u64;
    let t0 = read_u32(m, addr);
    let ms0 = m.systick_ms();
    for _ in 0..MAX_POLLS {
        m.run_budget(POLL_BYTES)?;
        if read_u32(m, addr) != t0 {
            return Ok(m.systick_ms().saturating_sub(ms0) as f64);
        }
    }
    Err(crate::core::CoreError::Io(format!(
        "等待控制拍超时：{MAX_POLLS} 次轮询未看到 {CTRL_TICKS_SYM} 增长（固件卡死或符号错）"
    )))
}
