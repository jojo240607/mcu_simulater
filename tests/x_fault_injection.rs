//! 故障注入验证（总线协议级 NACK / SPI 故障态）：
//!
//! 1) `spi_fault_from_boot_fdir_critical`：开机即对 bmi088（IMU 主源，SPI3）
//!    注入故障态（MISO 恒高 → WHO_AM_I 校验失败）→ 固件 IMU healthy=false →
//!    `read()` 返回零加速度 → 输入滤波（零初始状态 + 零输入 = 恒定 0）→
//!    FDIR 冻结判据（范数<6 且恒定）连续 20 拍 → `Health::Critical` →
//!    hb 行 `crit=true`（安全模式、执行器归零）。同时验证其余外设不受影响：
//!    baro/mag/gps 仍读到数据（healthy 从设备照常服务）。
//!
//! 2) `midrun_nack_isolates_slave`：运行中注入 NACK → 该从设备读计数冻结
//!    （不再成功读），同总线 healthy 从设备（bmp280）读计数继续增长——故障
//!    注入在总线协议层按地址精准隔离，不波及其他从设备。
//!
//! 机制：I2C `inject_i2c_nack(port, addr7, nack)` → `RegFileSlave.nack` →
//! `on_read → None` → 模拟器 I2C 置 SR1.AF（固件读失败）；SPI
//! `inject_spi_fault(port, name, on)` → `Bmi088.faulted` → `on_byte` 恒回 0xFF。

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;

/// 读取从设备成功读计数（i2c3: 0=mpu6050, 1=bmp280, 2=qmc5883）。
/// 固件 baro/mag 走 i2c2(I2C3)——I2C1 的 DMA1_Stream6 与 uart1 TX 冲突。
fn i2c1_read_counts(m: &Machine) -> (u64, u64, u64) {
    let i2c_vec = m.i2c.lock().unwrap();
    let i = i2c_vec[2].lock().unwrap();
    let sl = i.slaves();
    (sl[0].read_count(), sl[1].read_count(), sl[2].read_count())
}

/// 装配 flyctrl real-sensors 全链路；`fault_spi_imu=true` 时开机即对
/// bmi088（SPI3）注入故障态（模拟 IMU 出厂即断线）。
fn setup(fault_spi_imu: bool) -> (Machine, Arc<AtomicBool>, Arc<AtomicU32>) {
    let elf = artifact::joc_base_elf();
    let app = artifact::flyctrl_real_app_bin();
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.attach_default_sensors();
    m.attach_default_uart_slaves();
    if fault_spi_imu {
        assert!(m.inject_spi_fault(3, "bmi088", true), "未命中 bmi088 从设备");
    }
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
            false
        })
        .unwrap();
    (m, got_invalid, bad_pc)
}

/// 运行至满足条件或超步数；返回是否满足。
fn run_until<F: Fn(&str) -> bool>(
    m: &mut Machine,
    got_invalid: &AtomicBool,
    cond: F,
    max_steps: u32,
) -> bool {
    for _ in 0..max_steps {
        let r = m.run_budget(400_000);
        let text = merged_log(m);
        if cond(&text) {
            return true;
        }
        if got_invalid.load(Ordering::Relaxed) {
            return false;
        }
        if let Err(e) = r {
            eprintln!("ERR {e:?}");
            return false;
        }
    }
    false
}

/// 场景 1：开机即对 bmi088（IMU）注入 SPI 故障态 → FDIR Critical + 其余外设不受影响。
#[test]
fn spi_fault_from_boot_fdir_critical() {
    let (mut m, got_invalid, _bad) = setup(true);
    let t_start = std::time::Instant::now();

    let mut mounted = false;
    let mut tasks = false;
    let mut hb = false;
    let mut baro_ok = false;
    let mut crit_seen = false;
    let mut panic_seen = false;
    for step in 0..3200u32 {
        if t_start.elapsed().as_secs() > 300 {
            break;
        }
        let r = m.run_budget(400_000);
        let text = merged_log(&mut m);
        if text.contains("RUST app mounted") {
            mounted = true;
        }
        if text.contains("task started") {
            tasks = true;
        }
        if let Some(idx) = text.rfind("hb seq=") {
            let line: String = text[idx..].lines().next().unwrap_or("").to_string();
            hb = true;
            if line.contains("crit=true") {
                crit_seen = true;
            }
            if line.contains("baro=true") {
                baro_ok = true;
            }
        }
        if text.contains("panicked") || text.contains("UDF") || text.contains("HardFault") {
            panic_seen = true;
        }
        if step % 10 == 0 {
            eprintln!("[step {step}] mounted={mounted} tasks={tasks} hb={hb} crit={crit_seen} baro_ok={baro_ok} console={}", text.len());
        }
        if got_invalid.load(Ordering::Relaxed) {
            break;
        }
        if let Err(e) = r {
            eprintln!("ERR {e:?}");
            break;
        }
        if mounted && tasks && hb && crit_seen {
            break;
        }
    }
    let out = m.console.lock().unwrap().output().to_vec();
    println!("=== console ({}B) ===\n{}", out.len(), String::from_utf8_lossy(&out));
    println!("=== end ===");
    let rc = i2c1_read_counts(&m);
    let spi_access = {
        let sv = m.spi.lock().unwrap();
        let s = sv[2].lock().unwrap();
        s.slaves().iter().map(|sl| sl.access_count()).sum::<u64>()
    };
    let gps_uart = {
        let uv = m.usart.lock().unwrap();
        let g = uv[1].lock().unwrap();
        (g.slaves()[0].frames(), g.rx_fifo_len(), g.n_cpu_dr_reads())
    };
    eprintln!(
        "RESULT: mounted={mounted} tasks={tasks} hb={hb} crit={crit_seen} baro_ok={baro_ok} panic={panic_seen} read_counts={rc:?} spi_access={spi_access} gps_frames={} gps_fifo={} dr_reads={}",
        gps_uart.0, gps_uart.1, gps_uart.2
    );
    assert!(!panic_seen, "应用 panic");
    assert!(mounted, "App 分区未挂载");
    assert!(tasks, "业务任务未启动");
    assert!(hb, "未出现周期心跳");
    assert!(crit_seen, "开机即注入 SPI 故障后未出现 FDIR Critical（crit=true）——故障注入未触发安全模式");
    assert!(baro_ok, "healthy 从设备 bmp280 受故障波及其他外设（baro=false）");
    assert!(rc.1 > 0, "bmp280 应继续成功读（故障未隔离）");
    assert!(rc.2 > 0, "qmc5883 应继续成功读（故障未隔离）");
    // UART 侧不受 SPI/I2C 故障影响：GPS 推流从设备仍在生成帧、固件仍在消费
    assert!(gps_uart.0 > 0, "UART GPS 推流从设备受故障波及其他总线（无帧生成）");
    assert!(gps_uart.2 > 0, "固件应仍在消费 UART RX（POLL 读无响应）");
}

/// 场景 2：运行中注入 NACK → 仅该从设备读计数冻结，healthy 从设备继续。
#[test]
fn midrun_nack_isolates_slave() {
    let (mut m, got_invalid, bad_pc) = setup(false);
    // 阶段 1：正常运行，确认 mpu6050 有成功读
    let ok = run_until(&mut m, &got_invalid, |t| {
        if let Some(idx) = t.rfind("hb seq=") {
            t[idx..].lines().next().unwrap_or("").contains("crit=false")
        } else {
            false
        }
    }, 6400);
    if !ok {
        let out = m.console.lock().unwrap().output().to_vec();
        eprintln!("=== console ({:?}B) ===\n{}\n=== end ===", out.len(), String::from_utf8_lossy(&out));
        eprintln!("invalid_insn={} bad_pc=0x{:08X}", got_invalid.load(Ordering::Relaxed), bad_pc.load(Ordering::Relaxed));
    }
    assert!(ok, "基线 hb 未出现（SPI DMA 下 sensors 读略慢，6400 步应足够）");
    let before = i2c1_read_counts(&m);
    // 固件 imu 已切换 BMI088(SPI)：I2C1 上固件实际读取的是 baro(0x76)/mag(0x0D)，
    // mpu6050(0x68) 从设备保留但不再被读（读计数恒 0）。隔离验证改用 mag。
    assert!(before.2 > 0, "基线 qmc5883 应已有成功读，得 {before:?}");

    // 阶段 2：注入 NACK 到 qmc5883(mag)
    assert!(m.inject_i2c_nack(3, 0x0D, true), "未命中 qmc5883");
    // 阶段 3：继续运行，确认 qmc5883 读计数冻结、bmp280 继续增长
    let mut after = before;
    let mut stable_rounds = 0u32;
    for _ in 0..200u32 {
        let r = m.run_budget(400_000);
        if let Err(e) = r {
            eprintln!("ERR {e:?}");
            break;
        }
        after = i2c1_read_counts(&m);
        if after.2 == before.2 {
            stable_rounds += 1;
            if stable_rounds > 10 {
                break; // 连续 10 轮无 qmc5883 成功读 → 已冻结
            }
        } else {
            stable_rounds = 0;
        }
    }
    eprintln!(
        "RESULT: qmc5883 before={} after={} (冻结={}) bmp280 before={} after={} (增长={})",
        before.2, after.2, before.2 == after.2, before.1, after.1, after.1 > before.1
    );
    assert_eq!(before.2, after.2, "注入后 qmc5883 不应再成功读（NACK 未隔离）");
    assert!(after.1 > before.1, "bmp280 读计数应继续增长（同总线被波及其他从设备）");
}

/// console（已 drain + raw 直写）⊕ SDK log ring（未 drain 部分）。
/// log_task（prio 28）被业务任务饿死时应用 info! 行滞留 ring 永不到 console，
/// 因此里程碑检测必须合并 ring 内容（见 x_toml_topology.rs 同款说明）。
fn merged_log(m: &mut Machine) -> String {
    let outv = m.console.lock().unwrap().output().to_vec();
    let mut t = String::from_utf8_lossy(&outv).into_owned();
    t.push('\n');
    t.push_str(&scan_log_ring(m));
    t
}

/// 直接读取 App SDK 日志 ring（LOG_RING=0x2000_add0，2048B）。条目格式：
/// [len][level][payload]，payload = "R/{level} {tick} {tag}: {msg}\n"。
/// 地址来自 `arm-none-eabi-nm app.elf | grep -E 'LOG_(RING|HEAD|TAIL)'`：
///   LOG_RING=0x2000_add0 LOG_HEAD=0x2000_b5e8 LOG_TAIL=0x2000_b5ec
/// （App 重建后需按 nm 同步更新。）
fn scan_log_ring(m: &mut Machine) -> String {
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
    let mut buf = Vec::with_capacity(n);
    for k in 0..n {
        buf.push(ring[(head + k) % RING_SIZE]);
    }
    let mut out = String::new();
    let mut i = 0usize;
    while i < buf.len() {
        let len = buf[i] as usize;
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
