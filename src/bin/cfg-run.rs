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
    // 虚拟 USB 主机注入（C 类 usb 真实主机通信）：
    //   阶段 1：检测 App 的 READY 标记 → 注入总线复位 + 标准枚举（GET_DESCRIPTOR×2
    //     / SET_ADDRESS / SET_CONFIGURATION）；
    //   阶段 2：检测 App 的 ENUM-OK 标记 → 经 OUT EP1 注入 64B 模式数据（0x55+i）。
    let mut usb_stage1 = false;
    let mut usb_stage2 = false;
    // 分步注入状态：reset 注入后等 DAINTMSK 配置（usbreset 处理完），再逐个 SETUP
    //（每个 SETUP 等固件弹完 GRXSTSP 状态字即处理完），最后 SET_CONFIGURATION 后
    // 等 App 报 ENUM-OK 注入 OUT。
    let mut usb_setup_idx: usize = 0;
    const USB_SETUPS: [[u8; 8]; 4] = [
        [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00], // GET_DESCRIPTOR(Device)
        [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x20, 0x00], // GET_DESCRIPTOR(Config)
        [0x00, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00], // SET_ADDRESS 0x2A
        [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00], // SET_CONFIGURATION 1
    ];
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
        // 虚拟 USB 主机注入（两阶段握手，见上）
        if !usb_stage1 || !usb_stage2 {
            let text = {
                let c = m.console.lock().unwrap();
                String::from_utf8_lossy(c.output()).into_owned()
            };
            if !usb_stage1 && text.contains("DRVTEST-USB-HOST-READY") {
                m.usb_otg.lock().unwrap().inject_usb_reset();
                usb_stage1 = true;
                eprintln!("<< usb host: READY → 注入总线复位");
            }
            // reset 后分步注入 SETUP：先等 usbreset 处理完（DAINTMSK 已配置），
            // 再逐 SETUP 注入，每步等固件弹完 GRXSTSP 状态字。
            if usb_stage1 && usb_setup_idx < USB_SETUPS.len() {
                let (rx_empty, dmsk) = {
                    let u = m.usb_otg.lock().unwrap();
                    (u.rx_status_empty(), u.daintmsk())
                };
                let ready = if usb_setup_idx == 0 { dmsk != 0 } else { rx_empty };
                if ready {
                    m.usb_otg
                        .lock()
                        .unwrap()
                        .inject_setup(USB_SETUPS[usb_setup_idx]);
                    eprintln!("<< usb host: 注入 SETUP #{}", usb_setup_idx);
                    usb_setup_idx += 1;
                }
            }
            if !usb_stage2 && text.contains("DRVTEST-USB-ENUM-OK") {
                let pattern: Vec<u8> = (0..64).map(|i| 0x55u8 + i as u8).collect();
                m.usb_otg.lock().unwrap().inject_out(1, &pattern);
                usb_stage2 = true;
                eprintln!("<< usb host: ENUM-OK → OUT EP1 注入 64B 模式数据");
            }
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
