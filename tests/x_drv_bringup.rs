//! drv-bringup-app-rust 逐驱动 bring-up 在模拟器上的验收：
//! 加载系统 ELF + 新 bring-up app.bin，断言 uart0 上出现每个受测驱动的
//! `BK <drv>:` 行与收尾 `BK bk: bringup done`。
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;

const SYS_ELF: &str = r"D:\project\mcu\oop\joc-base\build_rel\stm32f407_minimal.elf";
const APP_BIN: &str = r"D:\project\mcu\oop\drv-bringup-app-rust\app.bin";

#[test]
fn drv_bringup() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&Path::new(SYS_ELF)).unwrap();
    m.load_app_partition(&Path::new(APP_BIN)).unwrap();
    m.reset().unwrap();

    // INSN_INVALID / fault 兜底
    let bad_pc = Arc::new(AtomicU32::new(0));
    let got_invalid = Arc::new(AtomicBool::new(false));
    let (b2, g2) = (bad_pc.clone(), got_invalid.clone());
    m.cpu
        .raw()
        .add_insn_invalid_hook(move |uc| {
            let pc = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
            b2.store(pc, Ordering::Relaxed);
            g2.store(true, Ordering::Relaxed);
            eprintln!("[INSN_INVALID] pc=0x{pc:08X}");
            false
        })
        .unwrap();

    let t_start = std::time::Instant::now();
    let mut done = false;
    for step in 0..200u32 {
        if t_start.elapsed().as_secs() > 150 {
            eprintln!(">>> 超时（150s）");
            break;
        }
        let r = m.run(400_000);
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let text = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        if text.contains("BK bk: bringup done") {
            eprintln!(">>> bringup done（step {step}）");
            done = true;
            break;
        }
        if got_invalid.load(Ordering::Relaxed) {
            eprintln!("!!! INSN_INVALID @0x{:08X}", bad_pc.load(Ordering::Relaxed));
            break;
        }
        if let Err(e) = r {
            eprintln!("ERR {e:?} pc=0x{pc:08X}"); 
            break;
        }
        if step % 10 == 0 {
            eprintln!("[step {step}] pc=0x{pc:08X} console={}", text.len());
        }
    }
    let out = m.console.lock().unwrap().output().to_vec();
    let text = String::from_utf8_lossy(&out);
    println!("=== console ({}B) ===\n{text}\n=== end ===", out.len());

    // 收尾必须出现（bring-up 同步写完"bringup done"后才返回），其余驱动行须各自到。
    assert!(done, "bring-up 未在时限内完成（done=true 缺席）——某驱动 ioctl/hang 阻塞了挂载");
    // 逐驱动：行前缀出现过即视为该驱动路径被执行到。
    let mut missing: Vec<&str> = Vec::new();
    for tag in ["adc0", "temp0", "timer2", "pwm0", "spi0", "i2c0", "usb0"] {
        if !text.contains(&format!("BK {tag}:")) {
            missing.push(tag);
        }
    }
    if !missing.is_empty() {
        eprintln!("!!! 未输出这些驱动行: {missing:?}");
    }
    assert!(!got_invalid.load(Ordering::Relaxed), "INSN_INVALID 出现");
    assert!(missing.is_empty(), "请核对缺失驱动行: {missing:?}");
}
