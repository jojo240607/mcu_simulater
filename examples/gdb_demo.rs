//! 演示：GDB 远程调试服务器（arm-none-eabi-gdb 直连）。
//! 用法：cargo run --example gdb_demo -- <port>
use mcu_simulater::gdbstub::GdbServer;
use mcu_simulater::machine::Machine;

fn main() {
    env_logger::init();
    let elf = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/i2c_irq_demo/i2c_irq_demo.elf");
    let port: u16 = std::env::args().nth(1).unwrap().parse().unwrap();
    GdbServer::spawn(port, move || {
        let mut m = Machine::new_m4f().unwrap();
        m.map_stm32f407_layout().unwrap();
        m.load_elf(&elf).unwrap();
        m.reset().unwrap();
        m
    })
    .unwrap();
    println!("GDB server 就绪 :{port}（Ctrl-C 退出）");
    loop {
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
