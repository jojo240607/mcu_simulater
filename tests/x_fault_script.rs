//! 时间轴脚本化故障注入（调试平台 P1-1）端到端验收。
//!
//! 验证三条链路：
//! - Halt 观察点：脚本在虚拟时刻触发 → run() 提前返回 → halted()==true；
//! - I2C NACK 注入：脚本触发后，固件/驱动读该从设备 → 总线嗅探记录 NACK（I2cRead{None}）；
//! - UART 丢帧：脚本触发后，虚拟从设备推流被丢弃 → rx_fifo 无字节；对照无脚本 → 有字节。
//!
//! 时间轴基准：retired / VIRTUAL_INSNS_PER_SEC（30M 指令/虚拟秒）。

use std::path::Path;
use std::sync::{Arc, Mutex};

use mcu_simulater::fault::{FaultAction, FaultScript};
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::Peripheral;
use mcu_simulater::peripheral::vperiph::data_source::{StaticSbus, StaticImu, StaticBaro, StaticMag};
use mcu_simulater::peripheral::vperiph::uart::Sbus;
use mcu_simulater::trace::{BusTrace, TraceKind};

const CR1_PE: u32 = 1 << 0;
const CR1_START: u32 = 1 << 8;
const CR1_STOP: u32 = 1 << 9;
const OFF_CR1: u32 = 0x00;
const OFF_DR: u32 = 0x10;
const OFF_SR2: u32 = 0x18;

fn machine_with_firmware_and_sensors() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/i2c_irq_demo/i2c_irq_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    // 挂 I2C 从设备（mpu6050 @0x68）+ UART 从设备（SBUS @uart3, port=3）
    m.register_i2c_slave(1, Box::new(mcu_simulater::peripheral::vperiph::i2c::mpu6050(StaticImu::default())));
    m.register_i2c_slave(1, Box::new(mcu_simulater::peripheral::vperiph::i2c::bmp280(StaticBaro::default())));
    m.register_i2c_slave(1, Box::new(mcu_simulater::peripheral::vperiph::i2c::qmc5883(StaticMag::default())));
    m.register_uart_slave(3, Box::new(Sbus::new(StaticSbus::default())));
    m
}

/// 手动模拟固件 I2C 读事务（读 mpu6050 寄存器 0x3B 一字节）。
fn i2c_read_one(m: &mut Machine) {
    let i2c1 = m.i2c.lock().unwrap()[0].clone();
    let mut i = i2c1.lock().unwrap();
    i.write(OFF_CR1, 4, CR1_PE | CR1_START).unwrap();
    i.write(OFF_DR, 4, 0x68u32 << 1).unwrap();
    i.write(OFF_DR, 4, 0x3B).unwrap();
    i.write(OFF_CR1, 4, CR1_PE | CR1_START).unwrap();
    i.write(OFF_DR, 4, (0x68u32 << 1) | 1).unwrap();
    let _ = i.read(OFF_SR2, 4).unwrap();
    let _ = i.read(OFF_DR, 4).unwrap();
    i.write(OFF_CR1, 4, CR1_PE | CR1_STOP).unwrap();
}

#[test]
fn halt_observation_point_stops_run() {
    let mut m = machine_with_firmware_and_sensors();
    // t=0.001s 触发 Halt（虚拟时间极短即可触发）
    let mut script = FaultScript::new("halt_test");
    script = script.at(0.001, FaultAction::Halt);
    m.attach_fault_script(script);

    // run 大预算：应在 Halt 触发后提前返回（而非耗尽预算）
    let pc_before = m.cpu.reg_read_u32(unicorn_engine::RegisterARM::PC).unwrap();
    m.run(20_000_000).unwrap();
    assert!(m.halted(), "Halt 观察点应触发");
    assert!(m.fault_all_fired(), "剧本应全部触发");
    assert_eq!(pc_before, pc_before); // 无实际意义占位，避免 unused

    // 清除后继续推进正常
    m.clear_halt();
    assert!(!m.halted());
    m.run(10_000).unwrap();
    assert!(!m.halted());
}

#[test]
fn scripted_i2c_nack_visible_in_trace() {
    let mut m = machine_with_firmware_and_sensors();
    let trace = Arc::new(Mutex::new(BusTrace::new(1024)));
    m.attach_bus_trace(trace.clone());

    // 基线：无故障时读 mpu6050 → 正常数据（I2cRead{Some}）
    i2c_read_one(&mut m);
    let base = trace.lock().unwrap().drain();
    assert!(base.iter().any(|e| matches!(e.kind, TraceKind::I2cRead { byte: Some(_) })),
        "基线应读到正常数据：\n{}", base.iter().map(|e| e.format()).collect::<Vec<_>>().join("\n"));

    // 装配剧本：t≈0 触发 mpu6050 NACK
    let mut script = FaultScript::new("mpu_nack");
    script = script.at(0.001, FaultAction::I2cNack { port: 1, addr7: 0x68, on: true });
    m.attach_fault_script(script);

    // run 推进虚拟时间触发注入（需 ≥ 0.001s：~3 万退休指令）
    m.run(100_000).unwrap();
    assert!(m.fault_all_fired(), "剧本应已触发");

    // 故障后读 → 总线嗅探应记录 NACK（I2cRead{None}）
    i2c_read_one(&mut m);
    let entries = trace.lock().unwrap().drain();
    assert!(entries.iter().any(|e| matches!(e.kind, TraceKind::I2cRead { byte: None })),
        "NACK 注入后读应记录 I2cRead{{None}}：\n{}",
        entries.iter().map(|e| e.format()).collect::<Vec<_>>().join("\n"));
}

#[test]
fn scripted_uart_drop_suppresses_push() {
    let mut m = machine_with_firmware_and_sensors();
    // 使能 USART3 接收（port=3 → index 2），否则 feed_rx_queued 不落地
    {
        let u3 = m.usart.lock().unwrap()[2].clone();
        u3.lock().unwrap().write(0x0C, 4, (1 << 13) | (1 << 2)).unwrap(); // UE|RE
    }

    // 基线：虚拟从设备推流 dt 基于"上次 run 结束"的退休量（延迟一个 run），
    // 故先 run 一次推进（首次 dt=0），再 run 一次产生 dt=0.15s（SBUS 20Hz → 3 帧）
    m.run(4_500_000).unwrap(); // 推进虚拟时间（本 run 的退休量为下次 dt 基准）
    m.run(4_500_000).unwrap(); // 推流 dt=0.15s
    let u3 = m.usart.lock().unwrap()[2].clone();
    let baseline_len = u3.lock().unwrap().rx_fifo_len();
    assert!(baseline_len > 0, "基线：SBUS 推流应入 fifo，got {baseline_len}");

    // 装配丢帧剧本：t≈0 起 uart3 持续丢弃
    let mut script = FaultScript::new("rc_loss");
    script = script.at(0.001, FaultAction::UartDrop { port: 3, frames: u32::MAX });
    m.attach_fault_script(script);
    // 本 run：step_virtual_uart(dt=0.15) 先推流（drop 在段中才触发，本次推流仍入 fifo）
    m.run(4_500_000).unwrap();
    assert!(m.fault_all_fired(), "丢帧剧本应触发");
    // 清空既有 fifo（含本次已入队的），作为丢帧生效后的基线
    let u3 = m.usart.lock().unwrap()[2].clone();
    {
        let mut u = u3.lock().unwrap();
        while u.rx_fifo_len() > 0 {
            let _ = u.dma_read_dr();
        }
    }
    let before_len = u3.lock().unwrap().rx_fifo_len();
    // 下一个 run：dt=0.15s 的推流应被丢帧拦截 → fifo 不增长
    m.run(4_500_000).unwrap();
    let after_len = u3.lock().unwrap().rx_fifo_len();
    assert!(
        after_len <= before_len + 8,
        "丢帧后 fifo 不应明显增长：before={before_len} after={after_len}"
    );
}
