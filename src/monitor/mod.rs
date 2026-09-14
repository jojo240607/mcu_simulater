//! Monitor REPL（调试平台 P0-2）：交互式调试 MCU 仿真。
//!
//! 目标：调试飞控固件时替代"打印 + 硬编码地址读内存"——REPL 直接读寄存器、
//! 内存、外设列表、NVIC 挂起状态、总线事务嗅探器（P0-1），并单步/断点执行。
//!
//! 命令集：
//! ```text
//! help                        命令列表
//! regs                        打印 CPU 寄存器（R0-R12/SP/LR/PC/xPSR）
//! read <addr> [len]           读内存（hex dump，len 默认 16 字节）
//! write <addr> <val>          写 32 位内存
//! step [n]                    run n 个小预算步（默认 1，每步 ~200 指令）
//! run <insns>                 运行指定退休指令预算（默认 10000）
//! break <addr>                设置断点（PC==addr 时 step 停止）
//! break del <addr>            删除断点
//! break list                  列出断点
//! show devices                列出已挂载外设（bus 区域）
//! show irq                    显示 NVIC 挂起/使能状态
//! trace on|off|dump|clear     总线事务嗅探器（P0-1）开关/导出/清空
//! reset                       复位机器
//! quit / exit                 退出
//! ```
//!
//! 用法（经 main.rs `--monitor`）：
//! ```text
//! mcu_simulater --elf firmware/x.elf --monitor
//! ```

use std::sync::{Arc, Mutex};

use unicorn_engine::RegisterARM;

use crate::machine::Machine;
use crate::trace::BusTrace;

/// 单步预算（退休指令）：~200 条 ≈ 微秒级虚拟时间，断点命中粒度。
const STEP_BUDGET: usize = 200;
/// run 默认预算。
const RUN_DEFAULT: usize = 10_000;

/// Monitor REPL。
pub struct Monitor {
    machine: Arc<Mutex<Machine>>,
    /// 总线事务嗅探器（P0-1；可选，未装配时为 None）
    trace: Option<Arc<Mutex<BusTrace>>>,
    /// 断点地址集合（PC 命中即停）。
    breaks: Vec<u32>,
}

impl Monitor {
    pub fn new(machine: Arc<Mutex<Machine>>, trace: Option<Arc<Mutex<BusTrace>>>) -> Self {
        Self {
            machine,
            trace,
            breaks: Vec::new(),
        }
    }

    /// 进入 REPL 主循环（stdin 逐行）。
    pub fn run_repl(&mut self) {
        use std::io::Write;
        println!("MCU Monitor REPL（help 查看命令，quit 退出）");
        loop {
            print!("mcu> ");
            let _ = std::io::stdout().flush();
            let mut line = String::new();
            if std::io::stdin().read_line(&mut line).is_err() {
                break;
            }
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            match self.execute(line) {
                Ok(Some(false)) => break, // quit
                Ok(_) => {}
                Err(e) => eprintln!("error: {e}"),
            }
        }
    }

    /// 执行单条命令（供 REPL 与测试共用）。
    ///
    /// 返回 `Ok(Some(false))` 表示退出。
    pub fn execute(&mut self, line: &str) -> Result<Option<bool>, String> {
        let mut it = line.split_whitespace();
        let cmd = it.next().unwrap_or("");
        let arg = |n: usize| it.clone().nth(n).map(|s| s.to_string());
        match cmd {
            "help" | "?" => {
                println!(
                    "命令: regs | read <addr> [len] | write <addr> <val> | step [n] | run [insns]\n\
                     \x20     break <addr> | break del <addr> | break list\n\
                     \x20     show devices | show irq | trace on|off|dump|clear\n\
                     \x20     reset | quit"
                );
                Ok(None)
            }
            "regs" => {
                self.cmd_regs()?;
                Ok(None)
            }
            "read" => {
                let addr = parse_u32(arg(0).as_deref().ok_or("read 需要 <addr>")?)?;
                let len = arg(1).and_then(|s| s.parse::<usize>().ok()).unwrap_or(16);
                self.cmd_read(addr, len)?;
                Ok(None)
            }
            "write" => {
                let addr = parse_u32(arg(0).as_deref().ok_or("write 需要 <addr>")?)?;
                let val = parse_u32(arg(1).as_deref().ok_or("write 需要 <val>")?)?;
                self.cmd_write(addr, val)?;
                Ok(None)
            }
            "step" => {
                let n = arg(0).and_then(|s| s.parse::<usize>().ok()).unwrap_or(1);
                self.cmd_step(n)?;
                Ok(None)
            }
            "run" => {
                let n = arg(0).and_then(|s| s.parse::<usize>().ok()).unwrap_or(RUN_DEFAULT);
                self.cmd_run(n)?;
                Ok(None)
            }
            "break" => match arg(0).as_deref() {
                Some("del") => {
                    let addr = parse_u32(arg(1).as_deref().ok_or("break del 需要 <addr>")?)?;
                    self.breaks.retain(|&b| b != addr);
                    println!("断点已删除 {:#010x}", addr);
                    Ok(None)
                }
                Some("list") => {
                    if self.breaks.is_empty() {
                        println!("（无断点）");
                    } else {
                        for b in &self.breaks {
                            println!("  {:#010x}", b);
                        }
                    }
                    Ok(None)
                }
                Some(a) => {
                    let addr = parse_u32(a)?;
                    if !self.breaks.contains(&addr) {
                        self.breaks.push(addr);
                    }
                    println!("断点已设置 {:#010x}", addr);
                    Ok(None)
                }
                None => return Err("break 需要 <addr> | del <addr> | list".into()),
            },
            "show" => match arg(0).as_deref() {
                Some("devices") => {
                    self.cmd_show_devices();
                    Ok(None)
                }
                Some("irq") => {
                    self.cmd_show_irq();
                    Ok(None)
                }
                _ => Err("show 需要 devices | irq".into()),
            },
            "trace" => match arg(0).as_deref() {
                Some("on") => {
                    self.trace_op(|t| t.set_enabled(true));
                    println!("嗅探已开启");
                    Ok(None)
                }
                Some("off") => {
                    self.trace_op(|t| t.set_enabled(false));
                    println!("嗅探已关闭");
                    Ok(None)
                }
                Some("dump") => {
                    let s = self.trace_drain();
                    println!("{}", s);
                    Ok(None)
                }
                Some("clear") => {
                    self.trace_op(|t| t.clear());
                    println!("嗅探缓冲已清空");
                    Ok(None)
                }
                _ => Err("trace 需要 on | off | dump | clear".into()),
            },
            "reset" => {
                self.machine.lock().unwrap().reset().map_err(|e| format!("reset 失败: {e}"))?;
                println!("已复位");
                Ok(None)
            }
            "quit" | "exit" => Ok(Some(false)),
            "" => Ok(None),
            other => Err(format!("未知命令: {other}（help 查看）")),
        }
    }

    // ---------------- 命令实现 ----------------

    fn cmd_regs(&mut self) -> Result<(), String> {
        let mut m = self.machine.lock().unwrap();
        let regs = [
            (RegisterARM::R0, "R0"), (RegisterARM::R1, "R1"), (RegisterARM::R2, "R2"),
            (RegisterARM::R3, "R3"), (RegisterARM::R4, "R4"), (RegisterARM::R5, "R5"),
            (RegisterARM::R6, "R6"), (RegisterARM::R7, "R7"), (RegisterARM::R8, "R8"),
            (RegisterARM::R9, "R9"), (RegisterARM::R10, "R10"), (RegisterARM::R11, "R11"),
            (RegisterARM::R12, "R12"), (RegisterARM::SP, "SP"), (RegisterARM::LR, "LR"),
            (RegisterARM::PC, "PC"), (RegisterARM::XPSR, "xPSR"),
        ];
        for (r, name) in regs {
            let v = m.cpu.reg_read_u32(r).map_err(|e| format!("reg {name}: {e}"))?;
            println!("  {:<4} = {:#010x} ({})", name, v, v);
        }
        Ok(())
    }

    fn cmd_read(&mut self, addr: u32, len: usize) -> Result<(), String> {
        let mut m = self.machine.lock().unwrap();
        let len = len.clamp(1, 256);
        let buf = m
            .cpu
            .mem_read(addr as u64, len)
            .map_err(|e| format!("读内存 {addr:#010x}: {e}"))?;
        // hex dump：每行 16 字节，右侧 ASCII
        for (i, chunk) in buf.chunks(16).enumerate() {
            let base = addr as usize + i * 16;
            let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02x}")).collect();
            let ascii: String = chunk
                .iter()
                .map(|b| if (0x20..0x7F).contains(b) { *b as char } else { '.' })
                .collect();
            println!("  {base:08x}  {}  {}", hex.join(" "), ascii);
        }
        Ok(())
    }

    fn cmd_write(&mut self, addr: u32, val: u32) -> Result<(), String> {
        let mut m = self.machine.lock().unwrap();
        m.cpu
            .mem_write(addr as u64, &val.to_le_bytes())
            .map_err(|e| format!("写内存 {addr:#010x}: {e}"))?;
        println!("  {addr:#010x} <- {val:#010x}");
        Ok(())
    }

    fn cmd_step(&mut self, n: usize) -> Result<(), String> {
        for _ in 0..n {
            let pc = self.pc()?;
            // 断点命中 → 停（不执行该地址的下一块）
            if self.breaks.contains(&pc) {
                println!("  ** 断点命中 PC={:#010x} **", pc);
                return Ok(());
            }
            self.machine
                .lock()
                .unwrap()
                .run(STEP_BUDGET)
                .map_err(|e| format!("step 失败: {e}"))?;
        }
        println!("  PC = {:#010x}", self.pc()?);
        Ok(())
    }

    fn cmd_run(&mut self, budget: usize) -> Result<(), String> {
        self.machine
            .lock()
            .unwrap()
            .run(budget)
            .map_err(|e| format!("run 失败: {e}"))?;
        println!(
            "  完成（{} 条退休指令），PC = {:#010x}",
            budget,
            self.pc()?
        );
        Ok(())
    }

    fn cmd_show_devices(&mut self) {
        let m = self.machine.lock().unwrap();
        let regions = m.bus.lock().unwrap().regions().len();
        println!("已挂载外设（{regions} 个区域）:");
        for r in m.bus.lock().unwrap().regions() {
            println!("  {:#010x} +{:#06x}  {}", r.base, r.size, r.name);
        }
    }

    fn cmd_show_irq(&mut self) {
        let m = self.machine.lock().unwrap();
        let nvic = m.nvic.lock().unwrap();
        let mut any = false;
        for irq in 0..=81u32 {
            if nvic.is_pending(irq) {
                println!(
                    "  IRQ {irq}: pending（enabled={} pri={}）",
                    nvic.is_enabled(irq),
                    nvic.priority(irq)
                );
                any = true;
            }
        }
        if !any {
            println!("  （无挂起 IRQ）");
        }
    }

    fn pc(&mut self) -> Result<u32, String> {
        let mut m = self.machine.lock().unwrap();
        m.cpu.reg_read_u32(RegisterARM::PC).map_err(|e| format!("读 PC: {e}"))
    }

    // ---------------- 嗅探器辅助 ----------------

    fn trace_op<F: FnOnce(&mut BusTrace)>(&self, f: F) {
        if let Some(t) = &self.trace {
            f(&mut t.lock().unwrap());
        } else {
            println!("（嗅探器未装配——先 machine.attach_bus_trace）");
        }
    }

    /// 导出并清空嗅探缓冲（供 REPL `trace dump` 与测试共用）。
    pub fn trace_drain(&self) -> String {
        match &self.trace {
            Some(t) => t.lock().unwrap().drain_formatted(),
            None => "（嗅探器未装配）".to_string(),
        }
    }
}

/// 解析 0x 前缀十六进制或十进制。
fn parse_u32(s: &str) -> Result<u32, String> {
    let s = s.trim();
    let v = if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16)
    } else {
        s.parse::<u32>()
    };
    v.map_err(|_| format!("无法解析数字: {s}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup() -> (Arc<Mutex<Machine>>, Option<Arc<Mutex<BusTrace>>>) {
        let m = Machine::new_m4f().unwrap();
        let m = Arc::new(Mutex::new(m));
        (m, None)
    }

    #[test]
    fn parse_u32_hex_and_dec() {
        assert_eq!(parse_u32("0x1000").unwrap(), 0x1000);
        assert_eq!(parse_u32("4096").unwrap(), 4096);
        assert!(parse_u32("xyz").is_err());
    }

    #[test]
    fn execute_help_and_quit() {
        let (m, t) = setup();
        let mut mon = Monitor::new(m, t);
        assert!(mon.execute("help").is_ok());
        assert_eq!(mon.execute("quit").unwrap(), Some(false));
    }

    #[test]
    fn execute_unknown_command_errors() {
        let (m, t) = setup();
        let mut mon = Monitor::new(m, t);
        assert!(mon.execute("foobar").is_err());
    }

    #[test]
    fn read_write_roundtrip_via_repl() {
        let (m, t) = setup();
        m.lock().unwrap().map_stm32f407_layout().unwrap();
        let mut mon = Monitor::new(m, t);
        assert!(mon.execute("write 0x20000000 0xdeadbeef").is_ok());
        let v = mon
            .machine
            .lock()
            .unwrap()
            .cpu
            .mem_read(0x2000_0000, 4)
            .unwrap();
        assert_eq!(u32::from_le_bytes(v.try_into().unwrap()), 0xdeadbeef);
        assert!(mon.execute("read 0x20000000 4").is_ok());
    }

    #[test]
    fn breakpoint_set_list_del() {
        let (m, t) = setup();
        let mut mon = Monitor::new(m, t);
        assert!(mon.execute("break 0x08000000").is_ok());
        assert!(mon.execute("break 0x08000100").is_ok());
        assert!(mon.execute("break list").is_ok());
        assert!(mon.execute("break del 0x08000000").is_ok());
        assert_eq!(mon.breaks, vec![0x0800_0100]);
    }
}
