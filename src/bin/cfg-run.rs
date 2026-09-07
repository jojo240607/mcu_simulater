//! 配置文件驱动的运行入口：把加载固件等写进一个 `.cfg`，一条命令直达，流式打印串口。
//!
//! 格式（极简、无第三方依赖；`#` 注释，`key = value`，可带引号）例 `run.cfg`：
//!   elf       = D:\project\mcu\oop\joc-base\build_rel\stm32f407_minimal.elf
//!   app       = D:\project\mcu\oop\drv-bringup-app-rust\app.bin   ; 可选
//!   n         = 400000      ; 每步预算
//!   max_steps = 0           ; 0=直到 quit/EOF
//!   rx_port   = 1           ; stdin 注入用的 USART 口（尽力）
//!
//! 用法：cargo run --release --bin cfg-run -- --cfg run.cfg
//!   或　.\run.ps1 [-cfg run.cfg]

use std::collections::HashMap;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::mpsc;

use clap::Parser;
use log::LevelFilter;

use mcu_simulater::machine::Machine;

#[derive(Parser, Debug)]
#[command(name = "cfg-run", about = "Run simulator driven by a config file")]
struct Cli {
    #[arg(long, default_value = "run.cfg")]
    cfg: PathBuf,
}

fn parse_cfg(text: &str) -> HashMap<String, String> {
    let mut map = HashMap::new();
    for line in text.lines() {
        let body = line.split(['#', ';']).next().unwrap_or("").trim();
        if body.is_empty() {
            continue;
        }
        if let Some(eq) = body.find('=') {
            let k = body[..eq].trim().to_ascii_lowercase();
            let mut v = body[eq + 1..].trim().to_string();
            if v.len() >= 2 && v.starts_with('"') && v.ends_with('"') {
                v = v[1..v.len() - 1].to_string();
            }
            map.insert(k, v);
        }
    }
    map
}

fn main() {
    env_logger::Builder::new().filter_level(LevelFilter::Warn).format_timestamp(None).init();

    let cli = Cli::parse();
    let txt = std::fs::read_to_string(&cli.cfg)
        .unwrap_or_else(|e| panic!("读配置 {} 失败: {e}", cli.cfg.display()));
    let cfg = parse_cfg(&txt);
    // 为健壮：允许用相对当前目录解析 elf/app 时若不存在再尝试相对该 .cfg 所在目录。
    let cfg_dir =
        cli.cfg.parent().map(|p| p.to_path_buf()).unwrap_or_else(|| PathBuf::from("."));

    let elf_s = cfg.get("elf").cloned().expect("配置缺 elf");
    let app_s = cfg.get("app").cloned();
    let n: usize = cfg.get("n").and_then(|s| s.parse().ok()).unwrap_or(400_000);
    let max_steps: usize = cfg.get("max_steps").and_then(|s| s.parse().ok()).unwrap_or(0);
    let rx_port: u8 = cfg.get("rx_port").and_then(|s| s.parse().ok()).unwrap_or(1);

    let resolve = |p: &str| -> PathBuf {
        let b = PathBuf::from(p);
        if b.exists() || b.is_absolute() {
            b
        } else {
            cfg_dir.join(b)
        }
    };

    let mut m = Machine::new_m4f().expect("new_m4f");
    m.map_stm32f407_layout().expect("map layout");
    m.load_elf(&resolve(&elf_s)).unwrap_or_else(|e| panic!("load_elf({elf_s}): {e}"));
    if let Some(app) = &app_s {
        m.load_app_partition(&resolve(app)).unwrap_or_else(|e| panic!("load_app_partition: {e}"));
    }
    m.reset().expect("reset");

    // stdin 读线程 → channel；主循环每步后非阻塞取，不卡步进。
    let (tx, rx) = mpsc::channel::<Vec<u8>>();
    std::thread::spawn(move || {
        let mut r = std::io::stdin().lock();
        let mut buf = [0u8; 4096];
        loop {
            match r.read(&mut buf) {
                Ok(0) | Err(_) => break,
                Ok(nn) => {
                    if tx.send(buf[..nn].to_vec()).is_err() {
                        break;
                    }
                }
            }
        }
    });

    eprintln!(
        ">>> cfg={}  elf={}{}; budget/step={} max_steps={}; 'quit'/EOF to end.",
        cli.cfg.display(),
        elf_s,
        app_s.map_or(String::new(), |a| format!("  +app {a}")),
        n,
        max_steps
    );

    let mut out = std::io::BufWriter::new(std::io::stdout());
    let mut last_len = 0usize;
    for step in 0.. {
        if max_steps != 0 && step >= max_steps {
            break;
        }
        if let Err(e) = m.run(n) {
            eprintln!("\n[run Err] {e}");
            break;
        }
        // 增量打印
        {
            let c = m.console.lock().unwrap();
            let raw = c.output();
            let start = if last_len <= raw.len() { last_len } else { 0 };
            if raw.len() > start {
                let seg = String::from_utf8_lossy(&raw[start..]);
                let _ = out.write_all(seg.as_bytes());
                let _ = out.flush();
            }
            last_len = raw.len();
        }
        // 注入 stdin
        let mut stop = false;
        while let Ok(item) = rx.try_recv() {
            let t = String::from_utf8_lossy(&item).trim_end().to_string();
            if t == "quit" || t == "exit" {
                stop = true;
                break;
            }
            for b in item {
                m.terminal.lock().unwrap().type_char(rx_port, b);
            }
            eprintln!("<< rx-port={rx_port} fed: [{t}]");
        }
        if stop {
            break;
        }
    }
    eprintln!("\n>>> done");
}
