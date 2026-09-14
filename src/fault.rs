//! 时间轴脚本化故障注入 + 场景库（调试平台 P1-1）。
//!
//! 目标：飞控固件鲁棒性调试的"故障剧本"——按**虚拟时间**在指定时刻注入总线/
//! 传感器故障（NACK、掉线、丢帧），观察固件 FDIR/降级行为，无需手写测试循环。
//!
//! 用法：
//! ```rust
//! let mut script = FaultScript::new("baro_nack");
//! script.at(2.0, FaultAction::I2cNack { port: 1, addr7: 0x76, on: true });
//! script.at(5.0, FaultAction::I2cNack { port: 1, addr7: 0x76, on: false });
//! machine.attach_fault_script(script);
//! // run() 推进虚拟时间时自动触发；FaultAction::Halt 停在观察点
//! ```
//!
//! 时间基准：`retired_insts / VIRTUAL_INSNS_PER_SEC`（与虚拟外设推流时钟同口径，
//! 见 [`crate::sim::timing`]）。Machine::run 每轮调用 `step_fault` 触发到期事件。
//!
//! 场景库（[`scenarios`]）：预置飞控调试常见故障剧本
//! （传感器周期性 NACK / 整条 I2C 掉线 / GPS 丢星 / 遥控丢链）。

use std::collections::HashMap;

use crate::sim::timing::VIRTUAL_INSNS_PER_SEC;

/// 时间轴可触发的故障动作。
#[derive(Debug, Clone, PartialEq)]
pub enum FaultAction {
    /// I2C 从设备 NACK 注入/清除（数据阶段读返回失败 → SR1.AF → 固件 FDIR）。
    I2cNack {
        port: u8,
        addr7: u8,
        on: bool,
    },
    /// UART 推流丢帧：丢弃该端口后续 `frames` 帧（GPS/遥控掉线）。
    /// `frames = u32::MAX` 表示持续丢弃直到 [`FaultAction::UartResume`]。
    UartDrop {
        port: u8,
        frames: u32,
    },
    /// 恢复 UART 推流（清除该端口丢帧状态）。
    UartResume {
        port: u8,
    },
    /// 观察点：触发后 run() 提前返回（不继续推进），供测试/调试者检查状态。
    Halt,
    /// 触发时打日志标记（时间轴事件旁路，不注入故障）。
    Log {
        msg: String,
    },
}

/// 单个时间轴事件：虚拟时刻 + 动作。
#[derive(Debug, Clone, PartialEq)]
pub struct FaultEvent {
    /// 触发时刻（虚拟秒）。
    pub at_sec: f32,
    /// 动作。
    pub action: FaultAction,
}

/// 故障剧本：按虚拟时间排序的事件序列（触发一次、幂等）。
#[derive(Debug, Clone)]
pub struct FaultScript {
    pub name: String,
    events: Vec<FaultEvent>,
    /// 各事件是否已触发（幂等：同一事件只触发一次）。
    fired: Vec<bool>,
}

impl FaultScript {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            events: Vec::new(),
            fired: Vec::new(),
        }
    }

    /// 追加事件（可链式）。
    pub fn at(mut self, at_sec: f32, action: FaultAction) -> Self {
        self.events.push(FaultEvent { at_sec, action });
        self.fired.push(false);
        self
    }

    pub fn events(&self) -> &[FaultEvent] {
        &self.events
    }

    /// 已触发事件数。
    pub fn fired_count(&self) -> usize {
        self.fired.iter().filter(|f| **f).count()
    }

    pub fn all_fired(&self) -> bool {
        !self.events.is_empty() && self.fired.iter().all(|f| *f)
    }

    /// 复位后重武装（fired 清零，脚本可重放）。
    pub fn rearm(&mut self) {
        for f in &mut self.fired {
            *f = false;
        }
    }
}

/// Machine 侧的时间轴推进器（run() 每轮调用）。
///
/// 返回本次触发的动作列表（供日志/测试断言）。
pub(crate) fn step_script(
    script: &mut FaultScript,
    retired: u64,
    uart_drop: &mut HashMap<u8, u32>,
) -> Vec<FaultAction> {
    let now = retired as f32 / VIRTUAL_INSNS_PER_SEC;
    let mut fired_now = Vec::new();
    for (i, ev) in script.events.iter().enumerate() {
        if !script.fired[i] && now >= ev.at_sec {
            script.fired[i] = true;
            // 副作用动作直接在此落地（Machine 可访问的状态）
            match &ev.action {
                FaultAction::UartDrop { port, frames } => {
                    uart_drop.insert(*port, *frames);
                }
                FaultAction::UartResume { port } => {
                    uart_drop.remove(port);
                }
                _ => {}
            }
            fired_now.push(ev.action.clone());
        }
    }
    fired_now
}

/// 场景库：预置飞控调试常见故障剧本。
pub mod scenarios {
    use super::*;

    /// I2C 传感器周期性 NACK：`start` 起每 `period` 秒 NACK `dur` 秒，共 `times` 次。
    /// 用于验证固件传感器健康监测/降级-恢复循环。
    pub fn i2c_sensor_nack_periodic(
        name: &str,
        port: u8,
        addr7: u8,
        start: f32,
        period: f32,
        dur: f32,
        times: usize,
    ) -> FaultScript {
        let mut s = FaultScript::new(name);
        for i in 0..times {
            let t0 = start + i as f32 * period;
            s = s
                .at(t0, FaultAction::I2cNack { port, addr7, on: true })
                .at(t0 + dur, FaultAction::I2cNack { port, addr7, on: false });
        }
        s
    }

    /// 整条 I2C 总线传感器掉线（指定地址 NACK 直到恢复）。
    pub fn i2c_sensor_loss(
        name: &str,
        port: u8,
        addr7: u8,
        at_sec: f32,
    ) -> FaultScript {
        FaultScript::new(name).at(at_sec, FaultAction::I2cNack { port, addr7, on: true })
    }

    /// GPS 丢星（uart2 推流持续丢弃）。
    pub fn gps_loss(at_sec: f32) -> FaultScript {
        FaultScript::new("gps_loss")
            .at(at_sec, FaultAction::UartDrop { port: 2, frames: u32::MAX })
    }

    /// 遥控丢链（uart3 SBUS 推流持续丢弃）。
    pub fn rc_loss(at_sec: f32) -> FaultScript {
        FaultScript::new("rc_loss")
            .at(at_sec, FaultAction::UartDrop { port: 3, frames: u32::MAX })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn script_triggers_at_time() {
        let mut s = FaultScript::new("t");
        s = s.at(1.0, FaultAction::Log { msg: "a".into() })
            .at(2.0, FaultAction::Log { msg: "b".into() });
        // t=0.5s：未触发
        let mut drop = HashMap::new();
        assert!(step_script(&mut s, (0.5 * VIRTUAL_INSNS_PER_SEC) as u64, &mut drop).is_empty());
        // t=1.5s：触发第 1 个
        let fired = step_script(&mut s, (1.5 * VIRTUAL_INSNS_PER_SEC) as u64, &mut drop);
        assert_eq!(fired.len(), 1);
        // t=2.5s：触发第 2 个；幂等（第 1 个不重复）
        let fired = step_script(&mut s, (2.5 * VIRTUAL_INSNS_PER_SEC) as u64, &mut drop);
        assert_eq!(fired.len(), 1);
        assert_eq!(s.fired_count(), 2);
        assert!(s.all_fired());
    }

    #[test]
    fn uart_drop_side_effect_applied() {
        let mut s = FaultScript::new("drop");
        s = s.at(0.5, FaultAction::UartDrop { port: 2, frames: 3 });
        let mut drop = HashMap::new();
        step_script(&mut s, (0.5 * VIRTUAL_INSNS_PER_SEC) as u64, &mut drop);
        assert_eq!(drop.get(&2), Some(&3));
        // 恢复
        s = s.at(1.0, FaultAction::UartResume { port: 2 });
        step_script(&mut s, (1.0 * VIRTUAL_INSNS_PER_SEC) as u64, &mut drop);
        assert!(!drop.contains_key(&2));
    }

    #[test]
    fn rearm_resets_fired() {
        let mut s = FaultScript::new("r");
        s = s.at(0.1, FaultAction::Halt);
        let mut drop = HashMap::new();
        step_script(&mut s, (0.2 * VIRTUAL_INSNS_PER_SEC) as u64, &mut drop);
        assert!(s.all_fired());
        s.rearm();
        assert!(!s.all_fired());
        assert_eq!(s.fired_count(), 0);
    }

    #[test]
    fn scenario_builders() {
        let p = scenarios::i2c_sensor_nack_periodic("p", 1, 0x76, 2.0, 5.0, 1.0, 3);
        assert_eq!(p.events().len(), 6); // 3 次 × (NACK on + off)
        assert!(scenarios::gps_loss(10.0).events().len() >= 1);
        assert!(scenarios::rc_loss(10.0).events().len() >= 1);
        assert!(scenarios::i2c_sensor_loss("l", 1, 0x76, 3.0).events().len() >= 1);
    }
}
