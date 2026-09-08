use std::path::Path;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

fn main() {
    let elf = Path::new(r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();

    let last_pc = Arc::new(AtomicU64::new(0));
    let last_addr = Arc::new(AtomicU64::new(0));
    let lp = last_pc.clone();
    let la = last_addr.clone();

    // 记录每条指令和每次内存读
    m.cpu.add_code_hook(1, 0, move |uc, addr, _size| {
        lp.store(addr, Ordering::Relaxed);
    }).unwrap();

    // 内存读 hook - 记录每次读的地址
    m.cpu.add_mem_hook(unicorn_engine::unicorn_const::HookType::MEM_READ, 1, 0, move |uc, _type, addr, _size, _value| {
        la.store(addr, Ordering::Relaxed);
        true
    }).unwrap();

    let r = m.run(100_000);
    let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
    let lp = last_pc.load(Ordering::Relaxed);
    let la = last_addr.load(Ordering::Relaxed);
    eprintln!("run={:?}", r);
    eprintln!("last_pc=0x{:08X} last_mem_read=0x{:08X} current_pc=0x{:08X}", lp, la, pc);
}
