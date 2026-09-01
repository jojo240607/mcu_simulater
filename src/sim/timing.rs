//! 周期模型（路线 B 简化：块级加权 + 外设侧忠实时序语义）。
//!
//! Unicorn 非周期精确，M0 起以"块内指令数 × 平均周期"累加虚拟周期。
//! 外设侧（SysTick/TIM 的 CNT 递增、比较匹配、溢出）完全按虚拟周期推进。
//! 预留 [`CycleModel::cycles_for_insn`]，未来如需更高精度可替换为指令级周期表。

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
