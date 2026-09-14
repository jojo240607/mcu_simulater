//! 遥测时间线导出（调试平台 P2-1）端到端验收。
//!
//! 加载真实固件（i2c_irq_demo：轮询发送问候 + 事件中断接收），用遥测记录器
//! 按退休间隔采样固件诊断共享内存（G_TX/G_RX/G_DONE），验证：
//! - run() 自动采样 → 行数随执行增长、时间列单调递增；
//! - 观测到 G_TX/G_RX 计数变化（遥测能"看见"固件行为时间线）；
//! - CSV 导出形状正确（头 + 行，可被外部工具消费）。

use std::path::Path;

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;
use mcu_simulater::telemetry::{Telemetry, WatchType};

const G_TX: u32 = 0x2000_0000;
const G_RX: u32 = 0x2000_0004;
const G_DONE: u32 = 0x2000_0008;

fn machine_with_firmware() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/i2c_irq_demo/i2c_irq_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

#[test]
fn telemetry_captures_firmware_timeline() {
    let mut m = machine_with_firmware();
    // 采样周期 2000 退休字节（~0.2ms 粒度，运行 10 万预算 ≈ 50 行）
    let mut tel = Telemetry::new(2_000);
    tel.add_watch("g_tx", G_TX, WatchType::U32)
        .add_watch("g_rx", G_RX, WatchType::U32)
        .add_watch("g_done", G_DONE, WatchType::U32);
    m.attach_telemetry(tel);

    // 推进固件：配置 I2C1 + 轮询发送问候（G_TX=1），随后进入等待循环
    m.run(50_000).unwrap();
    let rows_after_cfg = m.telemetry_rows();
    assert!(rows_after_cfg >= 5, "配置阶段应有多次采样，got {rows_after_cfg}");

    // 注入 RX 字节触发事件中断接收（G_RX 增长）
    for b in [b'a', b'b', b'c', b'd'] {
        m.events
            .lock()
            .unwrap()
            .publish(&Event::I2cRx { port: 1, byte: b });
        m.run(20_000).unwrap();
    }
    let rows_total = m.telemetry_rows();
    assert!(rows_total > rows_after_cfg, "后续执行应继续采样");

    // CSV 导出：头 + 行，时间列单调
    let csv = m.telemetry_csv();
    let lines: Vec<&str> = csv.lines().collect();
    assert_eq!(lines[0], "time_us,g_tx,g_rx,g_done", "CSV 头：{}", lines[0]);
    assert_eq!(lines.len(), rows_total + 1, "CSV 行数 = 采样行数 + 头");

    // 最后一行的 G_TX 应已变成 1（遥测能看到固件行为时间线）
    let last = lines.last().unwrap();
    let cols: Vec<&str> = last.split(',').collect();
    assert_eq!(cols.len(), 4);
    let g_tx_last: f64 = cols[1].parse().unwrap();
    assert!(g_tx_last >= 1.0, "遥测应观测到 G_TX 增长：last={last}");
    let g_done_last: f64 = cols[3].parse().unwrap();
    assert_eq!(
        g_done_last, 0xAAAA_AAAAu32 as f64,
        "注入 4 字节后固件应写 G_DONE 完成标志：last={last}"
    );

    // 时间列单调递增
    let times: Vec<f64> = lines[1..]
        .iter()
        .map(|l| l.split(',').next().unwrap().parse().unwrap())
        .collect();
    assert!(times.windows(2).all(|w| w[1] >= w[0]), "时间列应单调不减");
}

#[test]
fn telemetry_csv_writable_and_parseable() {
    let mut m = machine_with_firmware();
    let mut tel = Telemetry::new(5_000);
    tel.add_watch("g_tx", G_TX, WatchType::U32);
    m.attach_telemetry(tel);
    m.run(30_000).unwrap();

    let csv = m.telemetry_csv();
    // 简单校验：每行固定列数、无空行
    for (i, line) in csv.lines().enumerate() {
        assert_eq!(line.split(',').count(), 2, "第 {i} 行应有 2 列：{line}");
    }
    assert!(csv.lines().count() >= 2, "至少头 + 1 行");
}
