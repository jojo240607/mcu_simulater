//! 总线事务嗅探器（调试平台 P0-1）端到端验收。
//!
//! 不跑固件：直接经 Machine 挂载的 I2C/SPI/USART 外设句柄模拟"固件寄存器写"
//! （CPU 写 MMIO → 外设 write/read），验证嗅探器记录完整事务序列：
//! - I2C：START(地址+方向+命中) → W(寄存器地址) → R(数据) → STOP，含 NACK；
//! - SPI：CS 变化 + 全双工字节（TX→RX）；
//! - UART：推流 RX 字节 + IDLE 帧结束。
//!
//! 时间戳（retired 指令计数）未注入时恒 0——本测试聚焦序列正确性，
//! 时间戳链路由 Machine::attach_bus_trace 注入 retired 计数器保证（类型级）。

use std::sync::{Arc, Mutex};

use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::vperiph::data_source::{StaticBaro, StaticImu, StaticMag};
use mcu_simulater::peripheral::vperiph::i2c::{bmp280, mpu6050, qmc5883};
use mcu_simulater::peripheral::Peripheral;
use mcu_simulater::trace::{BusKind, BusTrace, TraceKind};

const CR1_PE: u32 = 1 << 0;
const CR1_START: u32 = 1 << 8;
const CR1_STOP: u32 = 1 << 9;
const OFF_CR1: u32 = 0x00;
const OFF_DR: u32 = 0x10;
const OFF_SR2: u32 = 0x18;

fn machine_with_sensors() -> Machine {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    // 直接挂 3 个 I2C 从设备（不加载固件，手动驱动外设）
    m.attach_default_sensors();
    m
}

/// 模拟固件对 I2C1 的"写寄存器地址 + 重复起始读数据"事务（MPU6050 标准读法）。
fn i2c_read_reg(m: &mut Machine, addr7: u8, reg: u8, n: usize) {
    let i2c1 = m.i2c.lock().unwrap()[0].clone();
    let mut i = i2c1.lock().unwrap();
    // 使能 + START
    i.write(OFF_CR1, 4, CR1_PE | CR1_START).unwrap();
    // 地址阶段：写 addr7<<1|0（W 方向）
    i.write(OFF_DR, 4, ((addr7 as u32) << 1)).unwrap();
    // 数据阶段：寄存器地址
    i.write(OFF_DR, 4, reg as u32).unwrap();
    // 重复 START：读方向
    i.write(OFF_CR1, 4, CR1_PE | CR1_START).unwrap();
    i.write(OFF_DR, 4, ((addr7 as u32) << 1) | 1).unwrap();
    // 读 SR2 清 ADDR → 预取首字节
    let _ = i.read(OFF_SR2, 4).unwrap();
    // 读 n 字节（每读预取下一字节）
    for _ in 0..n {
        let _ = i.read(OFF_DR, 4).unwrap();
    }
    // STOP
    i.write(OFF_CR1, 4, CR1_PE | CR1_STOP).unwrap();
}

#[test]
fn i2c_transaction_traced_end_to_end() {
    let mut m = machine_with_sensors();
    let trace = Arc::new(Mutex::new(BusTrace::new(256)));
    m.attach_bus_trace(trace.clone());

    // 模拟固件读 mpu6050 寄存器 0x3B 两字节
    i2c_read_reg(&mut m, 0x68, 0x3B, 2);

    let entries = trace.lock().unwrap().drain();
    // 序列：START(W) + W(reg) + START(R) + R×3(预取+2次读预取) + STOP = 7 条
    assert_eq!(entries.len(), 7, "应记录 7 条事务，got {}", entries.len());

    // 关键序列断言（顺序敏感）
    let kinds: Vec<&TraceKind> = entries.iter().map(|e| &e.kind).collect();
    assert_eq!(
        kinds[0],
        &TraceKind::I2cStart { addr7: 0x68, read: false, matched: true },
        "首条应为 W 方向 START@0x68 命中"
    );
    assert_eq!(kinds[1], &TraceKind::I2cWrite { byte: 0x3B }, "第二条应为寄存器地址写");
    assert_eq!(
        kinds[2],
        &TraceKind::I2cStart { addr7: 0x68, read: true, matched: true },
        "第三条应为重复起始 R 方向"
    );
    // 读方向：读 2 字节产生 3 条 R（SR2 清 ADDR 预取首字节 + 2 次读 DR 各预取下一字节）
    assert!(matches!(kinds[3], TraceKind::I2cRead { byte: Some(_) }));
    assert!(matches!(kinds[4], TraceKind::I2cRead { byte: Some(_) }));
    assert!(matches!(kinds[5], TraceKind::I2cRead { byte: Some(_) }));
    assert_eq!(kinds[6], &TraceKind::I2cStop, "最后应为 STOP");

    // 每条记录都带总线/端口标识
    assert!(entries.iter().all(|e| e.bus == BusKind::I2c && e.port == 1));

    // 格式化输出应人类可读（调试场景验证）
    let s = entries.iter().map(|e| e.format()).collect::<Vec<_>>().join("\n");
    assert!(s.contains("i2c1 START W@0x68 match"), "格式化输出应含 START 行：\n{s}");
    assert!(s.contains("W 0x3b"), "格式化输出应含寄存器地址写：\n{s}");
}

#[test]
fn i2c_nack_traced() {
    let mut m = machine_with_sensors();
    let trace = Arc::new(Mutex::new(BusTrace::new(256)));
    m.attach_bus_trace(trace.clone());
    // 故障注入：mpu6050 NACK → 读方向 on_read → None → AF
    assert!(m.inject_i2c_nack(1, 0x68, true), "NACK 注入应命中 mpu6050");

    let i2c1 = m.i2c.lock().unwrap()[0].clone();
    {
        let mut i = i2c1.lock().unwrap();
        i.write(OFF_CR1, 4, CR1_PE | CR1_START).unwrap();
        i.write(OFF_DR, 4, (0x68u32 << 1) | 1).unwrap();
        let _ = i.read(OFF_SR2, 4).unwrap();
        i.write(OFF_CR1, 4, CR1_PE | CR1_STOP).unwrap();
    }
    let entries = trace.lock().unwrap().drain();
    assert!(entries.iter().any(|e| matches!(e.kind, TraceKind::I2cRead { byte: None })),
        "NACK 应记录 I2cRead{{None}}：\n{}",
        entries.iter().map(|e| e.format()).collect::<Vec<_>>().join("\n"));
    assert!(entries.iter().any(|e| matches!(e.kind, TraceKind::I2cStart { matched: true, .. })),
        "地址仍应命中（NACK 是数据阶段）");
}

#[test]
fn usart_rx_and_idle_traced() {
    let mut m = machine_with_sensors();
    let trace = Arc::new(Mutex::new(BusTrace::new(256)));
    m.attach_bus_trace(trace.clone());

    let u2 = m.usart.lock().unwrap()[1].clone(); // USART2（port=2，index 1）
    {
        let mut u = u2.lock().unwrap();
        u.write(0x0C, 4, (1 << 13) | (1 << 2)).unwrap(); // CR1.UE|RE 使能接收
        u.feed_rx_queued(b'$');
        u.feed_rx_queued(b'G');
        u.notify_frame_end();
    }
    let entries = trace.lock().unwrap().drain();
    assert_eq!(entries.len(), 3);
    assert_eq!(entries[0].kind, TraceKind::UartRx { byte: b'$' });
    assert_eq!(entries[1].kind, TraceKind::UartRx { byte: b'G' });
    assert_eq!(entries[2].kind, TraceKind::UartIdle);
    assert!(entries.iter().all(|e| e.bus == BusKind::Usart && e.port == 2));
}

#[test]
fn spi_byte_and_cs_traced() {
    let mut m = machine_with_sensors();
    let trace = Arc::new(Mutex::new(BusTrace::new(256)));
    m.attach_bus_trace(trace.clone());
    // 挂 BMI088 从设备（SPI1），并模拟 CS 路由 + 全双工字节
    use mcu_simulater::peripheral::vperiph::data_source::StaticImu;
    use mcu_simulater::peripheral::vperiph::spi::bmi088;
    let s = bmi088::Bmi088::new((4, 10), (4, 11), StaticImu::default());
    m.spi.lock().unwrap()[0].lock().unwrap().register_slave(Box::new(s));
    let spi1 = m.spi.lock().unwrap()[0].clone();
    {
        let mut sp = spi1.lock().unwrap();
        sp.write(0x00, 4, 1 << 6).unwrap(); // CR1.SPE 使能（全双工交换的前提）
        sp.route_cs(4, 10, false); // CS 拉低（GPIOE pin10，BMI088 ACCEL_CS）
        sp.write(0x0C, 4, 0x00).unwrap(); // 写 SPI DR：首字节（寄存器地址 0，读 WHO）
        sp.route_cs(4, 10, true); // CS 拉高
    }
    let entries = trace.lock().unwrap().drain();
    assert!(entries.len() >= 3);
    assert!(matches!(entries[0].kind, TraceKind::SpiCs { level: false, .. }), "CS 拉低应记录");
    assert!(matches!(entries[1].kind, TraceKind::SpiByte { tx: 0x00, .. }), "全双工字节应记录");
    assert!(matches!(entries[2].kind, TraceKind::SpiCs { level: true, .. }), "CS 拉高应记录");
    assert!(entries.iter().all(|e| e.bus == BusKind::Spi && e.port == 1));
}

#[test]
fn trace_disable_stops_recording() {
    let mut m = machine_with_sensors();
    let trace = Arc::new(Mutex::new(BusTrace::new(256)));
    trace.lock().unwrap().set_enabled(false);
    m.attach_bus_trace(trace.clone());

    i2c_read_reg(&mut m, 0x68, 0x3B, 1);
    assert!(trace.lock().unwrap().is_empty(), "嗅探关闭后不应记录");

    trace.lock().unwrap().set_enabled(true);
    i2c_read_reg(&mut m, 0x68, 0x3B, 1);
    assert!(trace.lock().unwrap().len() > 0, "嗅探重新开启后应记录");
}
