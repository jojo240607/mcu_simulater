//! 事件调度器：维护虚拟周期与到期事件队列（M2/M3 启用，M0 仅占位）。
//!
//! 仿真循环每次 `emu_start` 跑一段指令后，用块级 hook 回报的周期数
//! 推进 [`EventScheduler::virtual_cycles`]，并触发到期事件（定时器、断点等）。

/// 到期事件（M2 起具体化）
struct PendingEvent {
    /// 触发时刻（虚拟周期）
    at_cycles: u64,
}

/// 事件调度器
#[derive(Default)]
pub struct EventScheduler {
    /// 当前虚拟周期数
    pub virtual_cycles: u64,
    pending: Vec<PendingEvent>,
}

impl EventScheduler {
    pub fn new() -> Self {
        Self::default()
    }

    /// 推进虚拟时钟
    pub fn advance(&mut self, cycles: u64) {
        self.virtual_cycles += cycles;
    }

    /// 登记一个到期事件（M2 起使用）
    pub fn schedule_at(&mut self, at_cycles: u64) {
        self.pending.push(PendingEvent { at_cycles });
    }

    /// 取出所有已到期事件（M2 起使用）
    #[allow(dead_code)]
    pub fn drain_due(&mut self) -> usize {
        let now = self.virtual_cycles;
        let mut due = 0;
        self.pending.retain(|e| {
            if e.at_cycles <= now {
                due += 1;
                false
            } else {
                true
            }
        });
        due
    }
}
