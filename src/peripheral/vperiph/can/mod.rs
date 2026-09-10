//! CAN 总线虚拟节点（vperiph）：CAN 是广播总线（无地址），虚拟外设以
//! "节点" 形式挂载——订阅 `Event::CanFrame`，按仲裁 ID 匹配/响应，回帧经
//! machine 路由到对应 CAN 端口接收 FIFO（固件 `CAN_IOCTL_RECV_FRAME` 读走）。
//!
//! 典型用例：飞控 → 电调/传感器节点的 CAN 链路（如 0x201 电调节点）。

pub mod node;

pub use node::{CanNode, MotorCtrlNode};
