//! 临时诊断：SYS-only boot 是否卡在 usb0 open（定位 HIL 联调 boot 卡点）。
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;

fn dump(m: &Machine, label: &str) {
    let out = m.console.lock().unwrap().output().to_vec();
    let t = String::from_utf8_lossy(&out);
    let n = t.len();
    eprintln!("[diag:{label}] pc=0x{:08X} console({n}B) TAIL:\n{}",
        m.last_switch_pc(), &t[n.saturating_sub(500)..]);
}

#[test]
fn boot_diag_sys_only() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&artifact::joc_base_elf()).unwrap();
    m.reset().unwrap();
    for i in 0..30u32 {
        let r = m.run(1_000_000);
        if i % 5 == 4 {
            dump(&m, &format!("run{i}"));
        }
        if r.is_err() {
            eprintln!("[diag] run err at {i}: {:?}", r.err());
            break;
        }
    }
    dump(&m, "final");
}
