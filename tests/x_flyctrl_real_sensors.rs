//! 虚拟外设总线协议级验证：flyctrl real-sensors（真实传感器驱动）经模拟器
//! 虚拟从设备读到数据——IMU=BMI088 挂 SPI3（板级 "spi2" 设备=SPI3）、
//! baro/mag 挂 I2C3（固件 i2c2，DMA 引擎搬运）。
//!
//! 里程碑（控制台日志）：
//!   mounted  = "RUST app mounted"
//!   tasks    = 任一 "task started"
//!   real     = sensor 任务日志 "real=1"（real-sensors feature 生效）
//!   hb       = "hb seq=" 且 imu_ok=true baro=true mag=true（真实驱动经虚拟从设备读到数据）
//!   gps      = "fix established"（u-blox 驱动首次有效定位；UART 推流 NMEA → 固件解析定位）
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;

#[test]
fn flyctrl_real_sensors_over_virtual_i2c() {
    let elf = artifact::joc_base_elf();
    let app = artifact::flyctrl_real_app_bin();
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    // 虚拟外设：IMU=BMI088 挂 SPI3（port 3），baro/mag 挂 I2C3（port 3，
    // 固件 i2c2 走 DMA）；baro 高度基准与虚拟 GPS（alt=4.0）对齐，
    // EKF 高度收敛到 4m 而非海平面 0m。
    m.attach_default_sensors_with_baro_height(4.0);
    assert_eq!(m.spi.lock().unwrap()[2].lock().unwrap().slaves().len(), 1); // bmi088
    assert_eq!(m.i2c.lock().unwrap()[2].lock().unwrap().slave_count(), 3); // I2C3
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
    // 步数预算：bmi088 走 SPI DMA 后，固件在 dma_wait_done 忙等（模拟器每
    // 256 块量子让出搬运），传感器/控制推进较 POLL 模式慢 ~2×；心跳 seq=250
    // 需 ~2s 仿真时间，3200 步仅覆盖 ~1.5s → 提到 6400 步（~170s 墙钟）。
    for step in 0..6400u32 {
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
        let i = i2c_vec[2].lock().unwrap(); // I2C3：bmp280(1)/qmc5883(2) 被固件读取（mpu6050(0) 保留不读）
        let sl = i.slaves();
        (sl[0].read_count(), sl[1].read_count(), sl[2].read_count())
    };
    let spi_cnt = {
        let spi_vec = m.spi.lock().unwrap();
        let s = spi_vec[2].lock().unwrap(); // SPI3：bmi088
        s.slaves().iter().map(|sl| sl.access_count()).sum::<u64>()
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
        "RESULT: mounted={mounted} tasks={tasks} hb={hb} imu_ok={imu_ok} baro_ok={baro_ok} gps_ok={gps_ok} mag_ok={mag_ok} panic={panic_seen} slave_reads={cnt:?} spi_access={spi_cnt} uart_frames={ucnt:?}"
    );
    assert!(!panic_seen, "应用 panic（IMU/baro 构造失败？）");
    assert!(mounted, "App 分区未挂载");
    assert!(tasks, "业务任务未启动");
    assert!(hb, "未出现周期心跳");
    assert!(imu_ok, "IMU(BMI088) 未经 SPI3 虚拟从设备读到数据（imu_ok=false）");
    assert!(baro_ok, "Baro(BMP280) 未经 I2C3 虚拟从设备读到数据（baro=false）");
    assert!(gps_ok, "GPS(u-blox) 未经 UART 推流从设备读到 NMEA（gps=false）");
    assert!(mag_ok, "Mag(QMC5883) 未经 I2C3 虚拟从设备读到数据（mag=false，hb 行 mag 字段）");
    assert!(cnt.1 > 0 && cnt.2 > 0, "I2C3 从设备无读取（bmp280(1)/qmc5883(2)，DMA 引擎未工作？）");
    assert!(spi_cnt > 0, "SPI3 从设备无读取（bmi088 未工作）");
}
