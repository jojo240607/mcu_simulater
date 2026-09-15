//! 虚拟外设总线协议级验证：flyctrl real-sensors（真实传感器驱动）经模拟器
//! I2C 虚拟从设备（mpu6050/bmp280/qmc5883 挂在 i2c1）读到数据。
//!
//! 里程碑（控制台日志）：
//!   mounted  = "RUST app mounted"
//!   tasks    = 任一 "task started"
//!   real     = sensor 任务日志 "real=1"（real-sensors feature 生效）
//!   hb       = "hb seq=" 且 imu_ok=true baro=true（真实驱动经 I2C 虚拟从设备读到数据）
//!   gps      = "fix established"（u-blox 驱动首次有效定位；UART 推流 NMEA → 固件解析定位）
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;

#[test]
fn flyctrl_real_sensors_over_virtual_i2c() {
    let elf = Path::new(r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf");
    let app = Path::new(r"/tmp/flyctrl_real.bin");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    // 虚拟外设：3 个 I2C 传感器从设备挂到 i2c1（flyctrl real-sensors 的 i2c0 = I2C1）
    m.attach_default_sensors();
    assert_eq!(m.i2c.lock().unwrap()[0].lock().unwrap().slave_count(), 3);
    // UART 推流从设备：gps→uart1(USART2 port2)、sbus→uart2(USART3 port3)
    m.attach_default_uart_slaves();
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
    let mut mag_ok = false;
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
        // 心跳行：ctrl: hb seq=... imu_ok=... gps=... baro=...
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
            if line.contains("mag=true") {
                mag_ok = true;
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
        if mounted && tasks && imu_ok && baro_ok && gps_ok && mag_ok {
            break;
        }
    }
    let out = m.console.lock().unwrap().output().to_vec();
    println!("=== console ({}B) ===\n{}", out.len(), String::from_utf8_lossy(&out));
    println!("=== end ===");
    let cnt = {
        let i2c_vec = m.i2c.lock().unwrap();
        let i = i2c_vec[0].lock().unwrap();
        let sl = i.slaves();
        (sl[0].read_count(), sl[1].read_count(), sl[2].read_count())
    };
    let ucnt = {
        let uv = m.usart.lock().unwrap();
        let gps = uv[1].lock().unwrap();
        let sbus = uv[2].lock().unwrap();
        (
            gps.slaves()[0].frames(),
            sbus.slaves()[0].frames(),
            gps.rx_fifo_len(),
            sbus.rx_fifo_len(),
        )
    };
    eprintln!(
        "RESULT: mounted={mounted} tasks={tasks} hb={hb} imu_ok={imu_ok} baro_ok={baro_ok} gps_ok={gps_ok} mag_ok={mag_ok} panic={panic_seen} slave_reads={cnt:?} uart_frames={ucnt:?}"
    );
    assert!(!panic_seen, "应用 panic（IMU/baro 构造失败？）");
    assert!(mounted, "App 分区未挂载");
    assert!(tasks, "业务任务未启动");
    assert!(hb, "未出现周期心跳");
    assert!(imu_ok, "IMU(MPU6050) 未经 I2C 虚拟从设备读到数据（imu_ok=false）");
    assert!(baro_ok, "Baro(BMP280) 未经 I2C 虚拟从设备读到数据（baro=false）");
    assert!(gps_ok, "GPS(u-blox) 未经 UART 推流从设备读到 NMEA（gps=false）");
    assert!(mag_ok, "Mag(QMC5883) 未经 I2C 虚拟从设备读到数据（mag=false，hb 行 mag 字段）");
    assert!(cnt.0 > 0 && cnt.1 > 0, "I2C 从设备无读取（虚拟外设未工作）");
}
