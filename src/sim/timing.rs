//! 周期模型（路线 B 简化：块级加权 + 外设侧忠实时序语义）。
//!
//! Unicorn 非周期精确，M0 起以"块内指令数 × 平均周期"累加虚拟周期。
//! 外设侧（SysTick/TIM 的 CNT 递增、比较匹配、溢出）完全按虚拟周期推进。
//! 预留 [`CycleModel::cycles_for_insn`]，未来如需更高精度可替换为指令级周期表。
//!
//! # 时间基权威口径（2026-09 校准）
//!
//! 模拟器存在**两套独立校准的虚拟时间**，服务不同目的，不要混用：
//!
//! 1. **CPU 侧虚拟时钟**（SysTick/TIM/DMA/RTC 的 `tick`）：block hook 按
//!    「访客字节 = 虚拟周期」折算（`machine::Machine::run` 内 `cycles = size`），
//!    与 QEMU icount 口径对齐。实测（`tests/x_sys_retire_calib.rs`）：
//!    SysTick reload(168000 周期) ≈ **4~6 万退休指令 ≈ 1ms 虚拟时间**
//!    （均值 ~4.6 万；历史 ×AVG=3 口径为 ~1.98 万，已废弃）。
//!    固件的 RTOS tick / sensors / control 周期都按此时钟跑。
//!
//! 2. **虚拟从设备推流时钟**（SBUS/GPS 等 UART/I2C 推流节拍）：按
//!    「退休指令数 / [`VIRTUAL_INSNS_PER_SEC`]」折算（`machine::run` 每次调用
//!    推进一次）。刻意**与中断频率解耦**：中断风暴下段数膨胀不会自放大
//!    推流速率（旧实现每段 dt=0.001 会自放大，见 `machine::run` 注释）。
//!
//! 两套时钟的换算基准不同（前者 ~46-60M 指令/虚拟秒，后者固定 30M），
//! 比值约 0.65~0.76——这是**已知且可接受的**：推流外设只需"帧在固件
//! 读取窗口内完整到达"，无需与真机波特率/帧率精确一致。若未来需要
//! 推流节奏与 CPU 时钟严格同步，应以本模块单一常量为基准重新校准。

/// 周期模型接口
pub trait CycleModel {
    /// 一个指令块消耗的周期数（块级加权）
    fn cycles_for_block(&self, instr_count: u32) -> u64;
    /// 单条指令的周期数（精确模式预留）
    fn cycles_for_insn(&self, addr: u64) -> u64;
}

/// 路线 B 简化模型：块内指令数 × 平均周期/指令
#[derive(Debug, Clone, Copy)]
pub struct BlockWeighted {
    /// 每指令平均周期（Cortex-M4 经验值约 1~3）
    pub avg_cycles_per_insn: u64,
}

impl Default for BlockWeighted {
    fn default() -> Self {
        Self {
            avg_cycles_per_insn: 3,
        }
    }
}

impl CycleModel for BlockWeighted {
    fn cycles_for_block(&self, instr_count: u32) -> u64 {
        instr_count as u64 * self.avg_cycles_per_insn
    }

    fn cycles_for_insn(&self, _addr: u64) -> u64 {
        self.avg_cycles_per_insn
    }
}

/// 共享虚拟时钟（M3：block hook 按块推进，供 TIM/SysTick 等外设 `tick`）。
///
/// `cycles` 用 `Cell`（非原子）：模拟器单线程运行，block hook 独占写入、测试读取，
/// 无并发访问；`Cell` 的 `get`/`set` 编译为普通读写，比 `AtomicU64` 的 `lock xadd`
/// 快约 5 倍（bench_probe：H1 原子 76.8 → H1c Cell 109.9 MIPS）。
#[derive(Debug, Default)]
pub struct VirtualClock {
    /// 已推进的虚拟周期数
    pub cycles: std::cell::Cell<u64>,
}

impl VirtualClock {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn advance(&self, cycles: u64) {
        self.cycles.set(self.cycles.get() + cycles);
    }

    /// 当前周期数（测试/调试）
    pub fn count(&self) -> u64 {
        self.cycles.get()
    }
}

/// 虚拟从设备推流时钟的权威换算：退休指令数 → 虚拟秒。
///
/// `dt = Δretired / VIRTUAL_INSNS_PER_SEC`（由 `machine::Machine::run` 使用）。
///
/// **唯一权威常量**：SBUS/GPS 等 UART/I2C 虚拟从设备的推流节拍均以它为基准。
///
/// # 2026-09 校准（虚拟时钟保真度）
///
/// `retired_count()` 实为 **TB 字节数**（block hook `fetch_add(size)`，Thumb
/// 下 ≈2×指令数）。旧值 30e6 是"指令数"口径残留（size 改字节后未更新），
/// 导致**场景/推流时间比 CPU 侧虚拟时钟（SysTick）慢 5.7 倍**：
/// - 实测校准：sensor 任务 msleep(2ms) 周期 ↔ 每拍 344K 字节 →
///   172M 字节/虚拟秒（= ~86M 指令/虚拟秒，与真实 MCU ~100-150MIPS 同量级）；
/// - 校准前控制拍速 46.7Hz（场景口径）vs 名义 250Hz → EKF 时间积分
///   比场景慢 ~5.7 倍（虚拟直接模拟实测 yaw 慢 5.4 倍、爬升/巡航动态全滞后）；
/// - 校准后控制拍速 ≈178Hz（场景口径），yaw 速率与场景真值匹配（±15%）。
///
/// 剩余 ~1.4 倍为**固件固有**（EKF 每拍执行超 4ms 预算，真实 MCU 同量级），
/// 非模拟器时钟失真。调整本值会改变所有推流外设的相对节拍，须同步复核
/// `x_vperiph_mcusim` / `x_hil_mcusim` 闭环测试。
/// ★虚拟时钟（秒）＝【固件时钟】口径，由 [`RETIRED_BYTES_PER_MS`] 唯一推导。
///
/// 历史问题（§5.98 已闭合）：本常量曾是独立的 `172.0e6` ✗，与固件时钟口径
/// `RETIRED_BYTES_PER_MS = 95_600`（⇒ 95.6e6 字节/秒 ✗）**相差 1.80×**，
/// 使"用虚拟时间量周期"与"用固件 ms 量周期"给出两个不同答案（2.084ms vs 3.778ms）。
/// 现统一为**同一真值源**：`virtual秒 ≡ 固件秒`（`run_ms` 的设计意图 ✓）。
pub const VIRTUAL_INSNS_PER_SEC: f32 = RETIRED_BYTES_PER_MS as f32 * 1000.0;

/// **CPU 侧虚拟时钟**换算：固件自身时钟 1ms 对应的"退休字节"量级。
///
/// 口径来源：SysTick reload(168000 周期) = 固件 1ms。实测
/// （`tests/x_sys_retire_calib.rs` 与 `tests/x_sensor_rate.rs`）：
/// - 固件时钟 1ms ↔ **~10.3 万退休字节**（直测 retired/SysTick 在 1.01~1.11 万×10
///   区间浮动；指令口径 ≈ 4.6 万条/ms，区间 3.1~6.4 万，随代码块混合比变化）；
/// - `run(count)` 的预算因块粒度**过冲 ~1.12×**（实测 run(368000) 实际退休
///   411749 字节）。
///
/// **本常量的实际用途**：只作 `Machine::run_ms` 内层循环的**步进粒度**（≈0.5ms
/// 一次的推进预算）。对齐判据不依赖它——`run_ms` 按**固件自己的 SysTick 计数**
/// 收敛（见该方法文档），故本常量 ±5% 的不确定性不影响对齐精度。
///
/// # 为什么闭环测试要用 `run_ms`（而非裸 `run(count)`）
///
/// `run(count)` 的 `count` 是"退休字节"，本身与物理步长没有约定关系。物理闭环
/// 测试每步推进 `dt` 秒物理（如 4ms），固件侧也应当推进同样长的时间，否则固件
/// 任务周期与物理步长系统性失配。实测：`run(300_000)` ≈ **3.26ms** 固件时钟，
/// 而物理步长按 4ms 推进 → **1.23× 失配**。
///
/// 失配后果（`x_vperiph_mcusim` 垂向慢漂的真实根因）：固件 EKF 的积分步长是
/// **编译期常量 `dt=4ms`**（`flyctrl-core` 的 `HilContext`，与真机 250Hz 标称一致），
/// 而它每拍之间物理实际推进 ~4.9ms → 加速度积分**少算 23%** → 垂向速度估计
/// 系统性滞后真值（实测机体下沉段 est_vd≈0.15 vs 真值 0.08）→ 定高环
/// `kv_z*(des_vz - est_vd)` 阻尼相位偏移 → 悬停慢漂（SIL 同控制律下
/// est≡真值、垂向稳态 ±0.08m，反证问题在时钟口径而非控制律）。
///
/// 改用 `Machine::run_ms(4.0)` 后固件时钟与场景 **1:1**（`x_sensor_rate.rs` 实测
/// 比值 1.002），12s 持续悬停末段 |dz| 由 0.305m 收紧到 0.223m。
pub const RETIRED_BYTES_PER_MS: usize = 168_000;

/// ★定时器周期流的换算（§5.100 修复 #2）：定时器模型以「**84MHz 基准周期**」为输入
/// （内部再按 `clk_hz / 84e6` 缩放，见 `peripheral::timer::tick`）。因此每个退休字节
/// 应折算 `84_000 / RETIRED_BYTES_PER_MS`（= **210/239**）个基准周期，使
/// **1 固件 ms（= `RETIRED_BYTES_PER_MS` 字节）恰为 84_000 基准周期 = 84MHz** ✓。
///
/// 修复前 ✗：定时器直接收到 `Δ退休字节`（= 1 字节 1 周期）⇒ 实际速率 = 仿真器字节流
/// 速率（实测 ≈88_889 周期/固件ms ✗）⇒ 与板级声明 84_000/ms 差 +5.8% ✗，
/// 且**随代码构成浮动** ⇒ 这正是"相位漂移 / 代码布局敏感"的根 ✓。
pub const TIMER_CYC84_PER_BYTE_NUM: u64 = 84_000;
/// 分母（与 [`RETIRED_BYTES_PER_MS`] 同源 ⇒ 两者不会再次分叉 ✓）。
pub const TIMER_CYC84_PER_BYTE_DEN: u64 = RETIRED_BYTES_PER_MS as u64;
