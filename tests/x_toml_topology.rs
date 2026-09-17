//! 虚拟外设拓扑（TOML）驱动验收：flyctrl real-sensors 经拓扑文件装配的虚拟
//! 从设备读到数据——IMU（SPI3 bmi088）+ Baro/Mag（I2C3）+ GPS（UART 推流）。
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
    assert_eq!(nodes.len(), 6, "拓扑应有 6 个从设备（1 SPI + 3 I2C + 2 UART）");
    // 验证装配结果：spi3 挂 1（bmi088）、i2c3 挂 3、usart2/usart3 各挂 1
    assert_eq!(m.spi.lock().unwrap()[2].lock().unwrap().slaves().len(), 1); // bmi088
    assert_eq!(m.i2c.lock().unwrap()[2].lock().unwrap().slave_count(), 3);
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
    // bmi088 走 SPI DMA 后 boot 略慢（dma_wait_done 忙等占指令预算），
    // 心跳 seq=250 需 ~2s 仿真时间 → 3200 步不够，提到 6400。
    for step in 0..6400u32 {
        if t_start.elapsed().as_secs() > 300 {
            eprintln!(">>> 超时（300s）终止");
            break;
        }
        let r = m.run(400_000);
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        // 日志可见性 = console（已 drain 部分 + raw 直写）⊕ SDK log ring（未
        // drain 部分）。log_task（prio 28）被 telem/uplink 等业务任务饿死时（实测
        // ring head 冻结、tail 继续增长），应用 info! 行（hb/dbg）滞留 ring 永不
        // 到 console——因此里程碑检测必须直接扫描 ring，不能只看 console。
        let ring_text = scan_log_ring(&mut m);
        let text = {
            let outv = m.console.lock().unwrap().output().to_vec();
            let mut t = String::from_utf8_lossy(&outv).into_owned();
            t.push('\n');
            t.push_str(&ring_text);
            t
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
        let i = i2c_vec[2].lock().unwrap(); // I2C3（i2c2）：bmp280/qmc5883 所在
        let sl = i.slaves();
        (sl[0].read_count(), sl[1].read_count(), sl[2].read_count())
    };
    let spi_cnt = {
        let spi_vec = m.spi.lock().unwrap();
        let s = spi_vec[2].lock().unwrap(); // SPI3（"spi2" 设备）：bmi088
        s.slaves().iter().map(|sl| sl.access_count()).sum::<u64>()
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

    assert!(spi_cnt > 0, "SPI 从设备无读取（bmi088）");
}

/// 直接读取 App SDK 日志 ring（LOG_RING=0x2000_add0，2048B，head/tail 见下），
/// 解出其中尚未被 log_task drain 的行。条目格式：[len][level][payload]，
/// payload = "R/{level} {tick} {tag}: {msg}\n"。
/// 地址来自 `arm-none-eabi-nm app.elf | grep -E 'LOG_(RING|HEAD|TAIL)'`：
///   LOG_RING=0x2000_add0 LOG_HEAD=0x2000_b5e8 LOG_TAIL=0x2000_b5ec
/// （与 EST_STATE/SENSOR_FRAME 等约定一致：App 重建后需同步更新。）
fn scan_log_ring(m: &mut mcu_simulater::machine::Machine) -> String {
    const RING_ADDR: u64 = 0x2000_add0;
    const HEAD_ADDR: u64 = 0x2000_b5e8;
    const TAIL_ADDR: u64 = 0x2000_b5ec;
    const RING_SIZE: usize = 2048;
    let mut hb = [0u8; 4];
    let mut tb = [0u8; 4];
    let _ = m.cpu.raw().mem_read(HEAD_ADDR, &mut hb);
    let _ = m.cpu.raw().mem_read(TAIL_ADDR, &mut tb);
    let head = u32::from_le_bytes(hb) as usize % RING_SIZE;
    let tail = u32::from_le_bytes(tb) as usize % RING_SIZE;
    let mut ring = [0u8; RING_SIZE];
    let _ = m.cpu.raw().mem_read(RING_ADDR, &mut ring);
    let n = if tail >= head { tail - head } else { RING_SIZE - head + tail };
    if n == 0 || n > RING_SIZE {
        return String::new();
    }
    // 展平为 head→tail 的线性视图，再按 [len][level][payload] 解条目。
    let mut buf = Vec::with_capacity(n);
    for k in 0..n {
        buf.push(ring[(head + k) % RING_SIZE]);
    }
    let mut out = String::new();
    let mut i = 0usize;
    while i < buf.len() {
        let len = buf[i] as usize;
        // len 不含 len 字节、含 level 字节；防御越界/损坏（半写条目）。
        if len < 1 || len > 250 || i + 1 + len > buf.len() {
            break;
        }
        if let Ok(s) = std::str::from_utf8(&buf[i + 2..i + 1 + len]) {
            out.push_str(s);
            out.push('\n');
        }
        i += 1 + len;
    }
    out
}
