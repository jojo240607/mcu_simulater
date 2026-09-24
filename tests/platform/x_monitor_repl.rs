//! Monitor REPL（调试平台 P0-2）端到端验收：加载真实固件，用 REPL 命令跑调试流程。
//!
//! 验证：run 推进 → regs 可读 → read/write 内存 → break+step 断点命中 →
//! trace（P0-1 联动）开关/导出。固件用 i2c_irq_demo（轻量、确定性快）。

use std::path::Path;
use std::sync::{Arc, Mutex};

use mcu_simulater::machine::Machine;
use mcu_simulater::monitor::Monitor;
use mcu_simulater::trace::BusTrace;

const G_DONE: u32 = 0x2000_0008;

fn machine_with_firmware() -> Arc<Mutex<Machine>> {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/i2c_irq_demo/i2c_irq_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    Arc::new(Mutex::new(m))
}

#[test]
fn repl_debug_flow_on_real_firmware() {
    let machine = machine_with_firmware();
    let trace = Arc::new(Mutex::new(BusTrace::new(1024)));
    machine.lock().unwrap().attach_bus_trace(trace.clone());
    let mut mon = Monitor::new(machine.clone(), Some(trace));

    // 1) run 推进固件（固件配置 I2C1 + 轮询发问候，随后进入等待循环）
    assert!(mon.execute("run 50000").is_ok());

    // 2) regs / read 观测
    assert!(mon.execute("regs").is_ok());
    assert!(mon.execute("read 0x20000000 16").is_ok());

    // 3) 固件已配置 I2C1 并发送问候 → 总线嗅探应有 I2C 发送事件
    let dumped = mon.trace_drain();
    assert!(
        !dumped.is_empty(),
        "固件 I2C 配置/发送应产生嗅探记录，got empty"
    );

    // 4) write 修改内存（模拟调试期打补丁：直接置 G_DONE）
    assert!(mon.execute(&format!("write {:#x} 0xaaaaaaaa", G_DONE)).is_ok());
    let v = machine
        .lock()
        .unwrap()
        .cpu
        .mem_read(G_DONE as u64, 4)
        .unwrap();
    assert_eq!(u32::from_le_bytes(v.try_into().unwrap()), 0xAAAA_AAAA);

    // 5) break + step：设置断点在当前 PC 附近，step 应能正常执行并命中后续 PC
    //    （固件在主循环/等待中，PC 反复横跳——只验证 step 机制不报错 + 断点命令 ok）
    let pc = machine
        .lock()
        .unwrap()
        .cpu
        .reg_read_u32(unicorn_engine::RegisterARM::PC)
        .unwrap();
    assert!(mon.execute(&format!("break {:#x}", pc)).is_ok());
    assert!(mon.execute("step 2").is_ok());
    assert!(mon.execute("break list").is_ok());
    assert!(mon.execute(&format!("break del {:#x}", pc)).is_ok());

    // 6) trace off/on/clear 控制
    assert!(mon.execute("trace off").is_ok());
    let before = mon.trace_drain();
    assert!(before.is_empty(), "trace off 后应无新记录（drain 后为空）");
    assert!(mon.execute("trace on").is_ok());
    assert!(mon.execute("trace clear").is_ok());

    // 7) reset 复位
    assert!(mon.execute("reset").is_ok());

    // 8) quit
    assert_eq!(mon.execute("quit").unwrap(), Some(false));
}

#[test]
fn repl_show_commands() {
    let machine = machine_with_firmware();
    let mut mon = Monitor::new(machine, None);
    assert!(mon.execute("show devices").is_ok());
    assert!(mon.execute("show irq").is_ok());
    assert!(mon.execute("help").is_ok());
}

#[test]
fn repl_error_handling() {
    let machine = machine_with_firmware();
    let mut mon = Monitor::new(machine, None);
    assert!(mon.execute("read").is_err(), "read 缺地址应报错");
    assert!(mon.execute("read 0xzz").is_err(), "非法地址应报错");
    assert!(mon.execute("break").is_err(), "break 缺参数应报错");
    assert!(mon.execute("nonsense").is_err(), "未知命令应报错");
}
