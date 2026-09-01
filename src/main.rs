//! MCU 仿真器命令行入口（M0 骨架）。

use clap::Parser;
use log::info;

/// 基于 Unicorn Engine 的 MCU 仿真器（仿 Renode）
#[derive(Parser, Debug)]
#[command(name = "mcu_simulater", version, about = "基于 Unicorn Engine 的 MCU 仿真器（仿 Renode）")]
struct Cli {
    /// 机器描述脚本（类 Renode DSL，M3 启用）
    #[arg(short, long)]
    script: Option<String>,
}

fn main() {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info"),
    )
    .init();

    let cli = Cli::parse();
    info!("MCU 仿真器启动（M0 骨架）");

    match cli.script {
        Some(path) => {
            // TODO(M3): config::parse 解析 DSL 并构建 Machine
            info!("配置脚本：{path}（DSL 解析器待实现）");
        }
        None => {
            info!("未指定脚本；当前为 M0 骨架，请使用 --script 指定机器描述");
        }
    }
}
