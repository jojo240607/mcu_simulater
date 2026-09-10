//! CAN 虚拟节点实现。
//!
//! [`MotorCtrlNode`]：电机控制器节点（飞控→电调 CAN 链路）。收到目标 ID 的
//! 数据帧且首字节为命令 0x01（读状态）→ 回状态帧 `[rpm_hi, rpm_lo, temp,
//! status]`（DLC=4）。数值默认 rpm=6000（0x1770）、temp=40、status=0x03，
//! 可配置——与 ESC 电调转速口径一致（6000 RPM）。

use crate::peripheral::can::CanFrame;

/// CAN 总线虚拟节点 trait
pub trait CanNode: Send + Sync {
    /// 节点名（观测/日志）
    fn name(&self) -> &str;

    /// 收到一帧：匹配（ID/内容）则返回响应帧；`None` = 不响应。
    /// 响应帧的 `port` 应沿用接收帧端口（machine 路由时再兜底）。
    fn on_frame(&mut self, frame: &CanFrame) -> Option<CanFrame>;

    /// 被访问次数（观测/断言）
    fn access_count(&self) -> u64;

    /// 仿真时间推进
    fn step(&mut self, _dt: f32) {}
}

/// 电机控制器 CAN 节点（0x201）：读状态命令 → 回转速/温度/状态。
pub struct MotorCtrlNode {
    /// 响应仲裁 ID
    id: u32,
    /// 电机转速（RPM）
    rpm: u16,
    /// 控制器温度（°C）
    temp: u8,
    /// 状态字
    status: u8,
    /// 被访问次数
    access: u64,
}

impl MotorCtrlNode {
    pub fn new(id: u32, rpm: u16, temp: u8, status: u8) -> Self {
        Self { id, rpm, temp, status, access: 0 }
    }
}

impl Default for MotorCtrlNode {
    fn default() -> Self {
        Self::new(0x201, 6000, 40, 0x03)
    }
}

impl CanNode for MotorCtrlNode {
    fn name(&self) -> &str {
        "can_motor_ctrl"
    }

    fn on_frame(&mut self, frame: &CanFrame) -> Option<CanFrame> {
        if frame.rtr || frame.ext || frame.id != self.id || frame.dlc == 0 {
            return None;
        }
        match frame.data[0] {
            0x01 => {
                // 读状态命令：回 [rpm_hi, rpm_lo, temp, status]
                self.access += 1;
                let mut d = [0u8; 8];
                d[0] = (self.rpm >> 8) as u8;
                d[1] = (self.rpm & 0xFF) as u8;
                d[2] = self.temp;
                d[3] = self.status;
                Some(CanFrame::new(frame.port, self.id, false, false, 4, d))
            }
            _ => None,
        }
    }

    fn access_count(&self) -> u64 {
        self.access
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(port: u8, id: u32, dlc: u8, data: [u8; 8]) -> CanFrame {
        CanFrame::new(port, id, false, false, dlc, data)
    }

    #[test]
    fn read_status_command_replies() {
        let mut n = MotorCtrlNode::default(); // 0x201, rpm=6000, temp=40, status=3
        let req = frame(1, 0x201, 1, [0x01, 0, 0, 0, 0, 0, 0, 0]);
        let r = n.on_frame(&req).expect("应响应读状态");
        assert_eq!(r.id, 0x201);
        assert_eq!(r.dlc, 4);
        assert_eq!(r.data[0], 0x17, "rpm 高字节 (6000=0x1770)");
        assert_eq!(r.data[1], 0x70, "rpm 低字节");
        assert_eq!(r.data[2], 40, "温度");
        assert_eq!(r.data[3], 0x03, "状态字");
        assert_eq!(n.access_count(), 1);
    }

    #[test]
    fn non_matching_id_ignored() {
        let mut n = MotorCtrlNode::default();
        let req = frame(1, 0x301, 1, [0x01, 0, 0, 0, 0, 0, 0, 0]);
        assert!(n.on_frame(&req).is_none());
        assert_eq!(n.access_count(), 0);
    }

    #[test]
    fn unknown_command_ignored() {
        let mut n = MotorCtrlNode::default();
        let req = frame(1, 0x201, 1, [0x05, 0, 0, 0, 0, 0, 0, 0]);
        assert!(n.on_frame(&req).is_none());
        assert_eq!(n.access_count(), 0);
    }

    #[test]
    fn remote_frame_ignored() {
        let mut n = MotorCtrlNode::default();
        let req = CanFrame::new(1, 0x201, false, true, 0, [0; 8]); // RTR
        assert!(n.on_frame(&req).is_none());
    }

    #[test]
    fn custom_values() {
        let mut n = MotorCtrlNode::new(0x601, 12000, 55, 0x09);
        let req = frame(2, 0x601, 1, [0x01, 0, 0, 0, 0, 0, 0, 0]);
        let r = n.on_frame(&req).unwrap();
        assert_eq!(r.data[0], 0x2E); // 12000 = 0x2EE0
        assert_eq!(r.data[1], 0xE0);
        assert_eq!(r.data[2], 55);
        assert_eq!(r.data[3], 0x09);
    }
}
