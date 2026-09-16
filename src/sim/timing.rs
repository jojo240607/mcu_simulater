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
pub const VIRTUAL_INSNS_PER_SEC: f32 = 172.0e6;
