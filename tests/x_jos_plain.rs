//! jOS release 固件启动验收：分步跑到控制台 READY（含 mounting app layer）。
use std::path::Path;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;

#[test]
fn plain_boot() {
    let elf = Path::new(r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();

    let mut reached = false;
    for step in 0..40 {
        let t0 = std::time::Instant::now();
        let r = m.run(30_000_000);
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let out = m.console.lock().unwrap().output().len();
        let text = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        eprintln!(
            "[{step}] {:>5.1}s run={r:?} pc=0x{pc:08X} console={out}",
            t0.elapsed().as_secs_f64()
        );
        if text.contains("READY. Commands") && text.contains("mounting app layer") {
            eprintln!(">>> 到达 READY 横幅");
            reached = true;
            break;
        }
        if let Err(_) = r {
            break;
        }
    }
    let out = m.console.lock().unwrap().output().to_vec();
    let text = String::from_utf8_lossy(&out);
    println!("=== console ({}B) ===\n{text}", out.len());
    println!("=== end ===");
    assert!(reached, "jOS 未到达 READY 横幅");
    eprintln!("CONSOLE_LINES={}", text.lines().count());
}
