//! # MCU 仿真器
//!
//! 仿 Renode 的 MCU 仿真器：CPU 指令执行由 Unicorn Engine 承担，
//! 其余（内存映射、外设、中断、时序、调试）由 Rust 实现。
//!
//! 分层（自底向上）：
//! - [`core`]：CPU 封装（Unicorn）
//! - [`bus`]：内存总线
//! - [`peripheral`]：外设接口与注册表
//! - [`sim`]：仿真循环 / 时间模型 / 调度器
//! - [`events`]：事件总线（虚拟外设互联）
//! - [`machine`]：Machine 装配与固件加载
//! - [`config`]：配置 DSL 解析
//! - [`monitor`]：Monitor REPL
//! - [`gdbstub`]：GDB 远程调试（M1 启用）

pub mod bus;
pub mod config;
pub mod core;
pub mod events;
pub mod machine;
pub mod monitor;
pub mod peripheral;
pub mod sim;
