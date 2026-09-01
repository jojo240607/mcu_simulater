//! 运行 mpu_violation mock 固件，观察 run() 触发的 MemManageFault 实际报错输出。
//!
//! 固件（firmware/mpu_violation）自编程 MPU：region0 将 [0x20000000, +64B)
//! 设为特权只读，随后写 0x20000000 触发数据访问违规。仿真器应使 run()
//! 返回 CoreError::MemManageFault，并在此打印 Display/Debug 两种形式的报错。

use std::path::Path;

use unicorn_engine::RegisterARM;

use mcu_simulater::machine::Machine;

fn main() {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/mpu_violation/mpu_violation.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();

    let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
    println!("复位：SP=0x{:08X}  PC=0x{:08X}", m.initial_sp, pc);

    println!("== 执行固件（firmware/mpu_violation）==");
    match m.run(200_000) {
        Ok(()) => println!("run() 正常返回（未触发 MPU 违规）"),
        Err(e) => {
            println!("run() 返回 MemManageFault：");
            println!("  Display: {e}");
            println!("  Debug  : {e:?}");
        }
    }

    // 顺带核对 MPU 故障状态寄存器（MMFSR/MMFAR）
    let bus = m.bus.lock().unwrap();
    let mmfsr = bus.read(0xE000_ED28, 4).unwrap();
    let mmfar = bus.read(0xE000_ED34, 4).unwrap();
    println!(
        "故障状态：MMFSR=0x{mmfsr:08X}  MMFAR=0x{mmfar:08X}  \
         (DACCVIOL={} MMARVALID={})",
        (mmfsr >> 1) & 1,
        (mmfsr >> 7) & 1
    );
}
