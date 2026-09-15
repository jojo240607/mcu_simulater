//! 虚拟外设拓扑（TOML）驱动验收：flyctrl real-sensors 经拓扑文件装配的虚拟
//! 从设备读到数据——IMU/Baro（I2C）+ GPS（UART 推流）。
//!
//! 与 `x_flyctrl_real_sensors.rs`（代码侧 `attach_default_sensors/attach_default_uart_slaves`）
//! 等价，但从设备由 `config::apply_topology` 按 `examples/topology_flyctrl.toml` 装配——
//! 验证「换场景不改代码，只改 TOML」的仿真平台机制。

use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;

#[test]
fn flyctrl_real_sensors_via_toml_topology() {
    let elf = artifact::joc_base_elf();
    let app = Path::new(r"/tmp/flyctrl_real.bin");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    // 从设备由 TOML 拓扑装配（而非代码侧 attach_default_*）
    let toml = include_str!("../examples/topology_flyctrl.toml");
    let nodes = mcu_simulater::config::apply_topology(&m, toml).expect("拓扑装配失败");
    assert_eq!(nodes.len(), 5, "拓扑应有 5 个从设备（3 I2C + 2 UART）");
    // 验证装配结果：i2c1 挂 3 个、usart2/usart3 各挂 1 个
    assert_eq!(m.i2c.lock().unwrap()[0].lock().unwrap().slave_count(), 3);
    assert_eq!(m.usart.lock().unwrap()[1].lock().unwrap().slaves().len(), 1); // gps
    assert_eq!(m.usart.lock().unwrap()[2].lock().unwrap().slaves().len(), 1); // sbus

    m.load_elf(&elf).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();

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
    let mut mounted = false;
    let mut tasks = false;
    let mut hb = false;
    let mut imu_ok = false;
    let mut baro_ok = false;
    let mut gps_ok = false;
    let mut panic_seen = false;
    for step in 0..2000u32 {
        if t_start.elapsed().as_secs() > 300 {
            eprintln!(">>> 超时（300s）终止");
            break;
        }
        let r = m.run(400_000);
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let text = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        if text.contains("RUST app mounted") {
            mounted = true;
        }
        if text.contains("task started") {
            tasks = true;
        }
        if let Some(idx) = text.rfind("hb seq=") {
            let line: String = text[idx..].lines().next().unwrap_or("").to_string();
            hb = true;
            if line.contains("imu_ok=true") {
                imu_ok = true;
            }
            if line.contains("baro=true") {
                baro_ok = true;
            }
            if line.contains("gps=true") {
                gps_ok = true;
            }
        }
        // GPS 就绪：u-blox 驱动首次有效定位（独立于 hb 采样，直接证据）
        if text.contains("fix established") {
            gps_ok = true;
        }
        if text.contains("panicked") || text.contains("UDF") || text.contains("HardFault") {
            panic_seen = true;
        }
        if step % 5 == 0 || (!tasks && mounted) {
            eprintln!(
                "[step {step}] pc=0x{pc:08X} mounted={mounted} tasks={tasks} hb={hb} imu_ok={imu_ok} baro_ok={baro_ok} gps_ok={gps_ok} console={}",
                text.len()
            );
        }
        if got_invalid.load(Ordering::Relaxed) {
            eprintln!("!!! INSN_INVALID 出现 @0x{:08X}", bad_pc.load(Ordering::Relaxed));
            break;
        }
        match r {
            Err(e) => {
                eprintln!("ERR {e:?} pc=0x{pc:08X}");
                break;
            }
            Ok(()) => {}
        }
        if mounted && tasks && imu_ok && baro_ok && gps_ok {
            break;
        }
    }
    let out = m.console.lock().unwrap().output().to_vec();
    println!("=== console ({}B) ===", out.len());
    println!("{}", String::from_utf8_lossy(&out));
    println!("=== end ===");

    let cnt = {
        let i2c_vec = m.i2c.lock().unwrap();
        let i = i2c_vec[0].lock().unwrap();
        let sl = i.slaves();
        (sl[0].read_count(), sl[1].read_count(), sl[2].read_count())
    };
    eprintln!(
        "RESULT: mounted={mounted} tasks={tasks} hb={hb} imu_ok={imu_ok} baro_ok={baro_ok} gps_ok={gps_ok} panic={panic_seen} slave_reads={cnt:?}"
    );
    assert!(!panic_seen, "应用 panic");
    assert!(mounted, "App 分区未挂载");
    assert!(tasks, "业务任务未启动");
    assert!(hb, "未出现周期心跳");
    assert!(imu_ok, "IMU 未经 TOML 拓扑虚拟从设备读到数据");
    assert!(baro_ok, "Baro 未经 TOML 拓扑虚拟从设备读到数据");
    assert!(gps_ok, "GPS 未经 TOML 拓扑 UART 推流定位（无 fix established）");
    assert!(cnt.0 > 0 && cnt.1 > 0, "I2C 从设备无读取");
}
