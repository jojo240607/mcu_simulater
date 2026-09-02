//! M5 验收测试：I2C1 事件中断接收端到端（EV IRQ + 轮询 TxE 发送）。
//!
//! 复用 firmware/i2c_irq_demo 固件（I2C1 @ 0x40005400 + IRQ31）：
//! 1. 固件使能 I2C1 PE + NVIC IRQ31 优先级 15 + 使能，CR2.ITEVTEN|ITBUFEN；
//! 2. 轮询 TxE 发送问候 'I' → Event::I2cByte 捕获（G_TX=1）；
//! 3. 测试逐字节发布 Event::I2cRx{port=1} → I2C feed_rx：置 RxNE + 挂起 IRQ31；
//! 4. I2C1_EV_IRQHandler：读 DR（清 RxNE）→ RX_BUF[G_RX] → G_RX++；
//! 5. 主线等待 G_RX==4 → 写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_TX   = 1（轮询发送的问候字节数）
//!   0x20000004 G_RX   = 4（事件中断接收次数）
//!   0x20000008 G_DONE = 0xAAAAAAAA（主线完成）
//!   0x20000200 RX_BUF = "abcd"（handler 逐字节接收）
//!   订阅 I2cByte 收集 = "I"

use std::path::Path;
use std::sync::{Arc, Mutex};

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const G_TX: u32 = 0x2000_0000;
const G_RX: u32 = 0x2000_0004;
const G_DONE: u32 = 0x2000_0008;
const RX_BUF: u32 = 0x2000_0200;

/// 测试注入的 RX 字节（固件 EV 中断逐字节接收写入 RX_BUF）
const RX_BYTES: [u8; 4] = [b'a', b'b', b'c', b'd'];

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/i2c_irq_demo/i2c_irq_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

fn read_u32(m: &mut Machine, addr: u32) -> u32 {
    let out = m.cpu.mem_read(addr as u64, 4).unwrap();
    u32::from_le_bytes(out.try_into().unwrap())
}

#[test]
fn m5_i2c_ev_irq_receive_end_to_end() {
    let mut m = load_machine();

    // 订阅 I2C1 TX 事件：收集固件轮询发送的问候字节
    let got: Arc<Mutex<Vec<u8>>> = Arc::new(Mutex::new(Vec::new()));
    let got2 = got.clone();
    m.events.lock().unwrap().subscribe(Arc::new(Mutex::new(
        move |ev: &Event| {
            if let Event::I2cByte { port: 1, byte } = ev {
                got2.lock().unwrap().push(*byte);
            }
        },
    )));

    // 固件配置 I2C1 + NVIC + 轮询发问候，随后进入等待循环（run 按指令预算返回）
    m.run(10_000).unwrap();

    // 逐字节注入 RX：每注入 1 字节 → 运行让 EV IRQ handler 读 DR 接收
    for &b in &RX_BYTES {
        m.events
            .lock()
            .unwrap()
            .publish(&Event::I2cRx { port: 1, byte: b });
        m.run(50_000).unwrap();
    }
    // 主线退出循环，写 G_DONE
    m.run(50_000).unwrap();

    // 1) 主线完成 + 发送/接收计数
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_TX), 1, "应轮询发送 1 个问候字节");
    assert_eq!(read_u32(&mut m, G_RX), 4, "EV 中断应接收 4 次");

    // 2) 事件中断接收结果：RX_BUF == "abcd"
    for (i, b) in RX_BYTES.iter().enumerate() {
        let out = m.cpu.mem_read((RX_BUF + i as u32) as u64, 1).unwrap();
        assert_eq!(out[0], *b, "RX_BUF[{i}] 应为 {b:?}（EV 中断接收结果）");
    }

    // 3) 轮询发送结果：I2cByte 事件收到问候 'I'
    assert_eq!(
        got.lock().unwrap().as_slice(),
        [b'I'],
        "I2cByte 事件应收到轮询发送的问候字节"
    );
}
