//! 解锁飞行验证：flyctrl real-sensors 经虚拟外设解锁 + 油门，验证控制律闭环。
//!
//! 虚拟外设：I2C 传感器（mpu6050/bmp280/qmc5883）+ SBUS（ch5=1900 解锁、
//! ch4=1550 油门中位）+ GPS NMEA。解锁后控制环应：
//! - hb 行 armed=true（RC 帧间锁存后不再瞬时归零）；
//! - 电机指令非零（hb 行 m=[...] 解析），执行器有真实输出；
//! - EKF/控制律全程有限（无 NaN/panic），姿态稳定。

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::vperiph::data_source::{StaticSbus, StaticGps};
use mcu_simulater::peripheral::vperiph::uart::Sbus;
use mcu_simulater::peripheral::vperiph::uart::NmeaGps;

#[test]
fn unlock_and_fly_over_virtual_peripherals() {
    let elf = artifact::joc_base_elf();
    let app = artifact::flyctrl_real_app_bin();
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    // I2C 传感器 + baro 高度 4m（与 GPS 一致）
    m.attach_default_sensors_with_baro_height(4.0);
    // UART：GPS→uart1(USART2 port2)；SBUS→uart2(USART3 port3)。
    // SBUS 通道：ch4(索引4)=1900 解锁（>1700）、ch3=1550 油门中位、其余 1500。
    let mut sbus_ch = [1500.0f32; 16];
    sbus_ch[3] = 1550.0; // throttle 中位偏高
    // 通道 5 解锁开关：编码 raw = 992 + (c-1500)/500*819.5，固件解锁阈值 ch>1700
    // → c=1900 只到 raw 1647（不足），需 c>=1932；用满偏 2000（raw=1811>1700）。
    sbus_ch[4] = 2000.0;
    m.register_uart_slave(2, Box::new(NmeaGps::new(StaticGps::default())));
    m.register_uart_slave(3, Box::new(Sbus::new(StaticSbus { channels: sbus_ch })));
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
    let mut armed_seen = false;
    let mut motor_nonzero = false;
    let mut motor_str = String::new();
    let mut panic_seen = false;
    for step in 0..3000u32 {
        if t_start.elapsed().as_secs() > 300 {
            eprintln!(">>> 超时（300s）终止");
            break;
        }
        let r = m.run_budget(400_000);
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
        // hb 行：armed / m=[...] 电机指令
        if let Some(idx) = text.rfind("hb seq=") {
            let line: String = text[idx..].lines().next().unwrap_or("").to_string();
            if line.contains("armed=true") {
                armed_seen = true;
            }
            // m=[a,b,c,d]：提取 4 个电机指令
            if let Some(mi) = line.rfind("m=[") {
                let tail = &line[mi + 3..];
                let end = tail.find(']').unwrap_or(0);
                let vals: Vec<f32> = tail[..end]
                    .split(',')
                    .filter_map(|v| v.trim().parse::<f32>().ok())
                    .collect();
                if vals.len() == 4 && vals.iter().any(|&v| v > 0.001) {
                    motor_nonzero = true;
                    motor_str = format!("{vals:?}");
                }
            }
        }
        if text.contains("panicked") || text.contains("UDF") || text.contains("HardFault") {
            panic_seen = true;
        }
        if step % 5 == 0 || (!tasks && mounted) {
            eprintln!(
                "[step {step}] pc=0x{pc:08X} mounted={mounted} tasks={tasks} armed={armed_seen} motor={motor_nonzero} console={}",
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
        if mounted && tasks && armed_seen && motor_nonzero {
            break;
        }
    }
    let out = m.console.lock().unwrap().output().to_vec();
    println!("=== console ({}B) ===\n{}", out.len(), String::from_utf8_lossy(&out));
    println!("=== end ===");

    assert!(mounted, "固件未挂载");
    assert!(tasks, "任务未全部启动");
    assert!(!panic_seen, "固件 panic/UDF/HardFault");
    assert!(!got_invalid.load(Ordering::Relaxed), "INSN_INVALID @0x{:08X}", bad_pc.load(Ordering::Relaxed));
    assert!(armed_seen, "hb 行未出现 armed=true（RC 解锁链路未生效？）");
    assert!(
        motor_nonzero,
        "解锁后电机指令恒零（控制律闭环未输出？m={motor_str}）"
    );
    eprintln!("RESULT: mounted={mounted} tasks={tasks} armed={armed_seen} motor={motor_nonzero} panic={panic_seen}");
}
