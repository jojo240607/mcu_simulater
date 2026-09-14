//! MCU 仿真器命令行入口。
//!
//! 用法：
//! ```text
//! # 交互式调试固件（Monitor REPL）
//! mcu_simulater --elf firmware/x.elf --monitor
//! ```
//!
//! 调试流程：加载固件 → `reset` → `break <addr>` 设断点 → `step`/`run` 执行 →
//! `regs`/`read`/`write` 观测 → `trace on` + `trace dump` 看总线事务（P0-1）。

use std::path::Path;
use std::sync::{Arc, Mutex};

use clap::Parser;
use log::info;

use mcu_simulater::machine::Machine;
use mcu_simulater::monitor::Monitor;
use mcu_simulater::trace::BusTrace;

/// 基于 Unicorn Engine 的 MCU 仿真器（仿 Renode）
#[derive(Parser, Debug)]
#[command(name = "mcu_simulater", version, about = "基于 Unicorn Engine 的 MCU 仿真器（仿 Renode）")]
struct Cli {
    /// 固件 ELF 路径（加载到模拟 MCU）
    #[arg(long)]
    elf: Option<String>,
    /// 进入交互式 Monitor REPL（调试）
    #[arg(long)]
    monitor: bool,
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let cli = Cli::parse();
    let elf_path = cli.elf.as_deref();

    let mut machine = match Machine::new_m4f() {
        Ok(m) => m,
        Err(e) => {
            eprintln!("[fatal] 创建 Machine 失败: {e}");
            std::process::exit(1);
        }
    };
    if let Err(e) = machine.map_stm32f407_layout() {
        eprintln!("[fatal] 映射内存布局失败: {e}");
        std::process::exit(1);
    }

    // 加载固件（可选：无固件也可进 REPL 观测外设/内存）
    if let Some(p) = elf_path {
        let path = Path::new(p);
        if !path.exists() {
            eprintln!("[fatal] 固件不存在: {p}");
            std::process::exit(1);
        }
        if let Err(e) = machine.load_elf(path) {
            eprintln!("[fatal] 加载固件失败: {e}");
            std::process::exit(1);
        }
        info!("固件已加载: {p}");
    }

    // 嗅探器：REPL `trace` 命令的公共底座（P0-1）
    let trace = Arc::new(Mutex::new(BusTrace::default_max()));
    machine.attach_bus_trace(trace.clone());

    if let Err(e) = machine.reset() {
        eprintln!("[fatal] 复位失败: {e}");
        std::process::exit(1);
    }

    if cli.monitor {
        info!("进入 Monitor REPL（help 查看命令）");
        let machine = Arc::new(Mutex::new(machine));
        let mut mon = Monitor::new(machine, Some(trace));
        mon.run_repl();
    } else if elf_path.is_none() {
        info!("未指定固件；提示：用 --elf 加载固件、--monitor 进入调试 REPL");
    } else {
        // 无 --monitor：一次性跑完预算（无 REPL 也给出出口）
        info!("未指定 --monitor；退出。调试请加 --monitor");
    }
}
