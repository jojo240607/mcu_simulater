//! M3 验收测试：T1 外设集端到端（GPIO 事件 / USART TX→Console / TIM2 溢出中断）。
//!
//! 复用 firmware/m3_demo 固件（场景见其 main.c 注释）：
//! 1. RCC 使能寄存器镜像（AHB1ENR/APB1ENR/APB2ENR）；
//! 2. PA5 输出，USART1 轮询 TXE 发送 "M3" → 虚拟 Console 捕获；
//! 3. TIM2（PSC=0/ARR=31/UIE）溢出触发 IRQ28，handler 清 UIF、翻转 PA5（GpioLevel 事件）、
//!    G_TMR 达 4 后停表，主线写 G_DONE。
//!
//! 期望结果区（SRAM 固定地址）：
//!   0x20000000 G_LED   = 4（BSRR 翻转次数 = 中断次数）
//!   0x20000004 G_UART  = 2（"M3"）
//!   0x20000008 G_TMR   = 4（TIM2 IRQ28 执行次数）
//!   0x2000000C G_DONE  = 0xAAAAAAAA（主线完成）

use std::path::Path;
use std::sync::{Arc, Mutex};

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::timer::TIM2_IRQ;

const G_LED: u32 = 0x2000_0000;
const G_UART: u32 = 0x2000_0004;
const G_TMR: u32 = 0x2000_0008;
const G_DONE: u32 = 0x2000_000C;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/m3_demo/m3_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

fn read_u32(m: &mut Machine, addr: u32) -> u32 {
    // SRAM 结果区直接经 CPU 读取；MMIO 外设（RCC 镜像）由 mem hook 注入，同样可读
    let out = m.cpu.mem_read(addr as u64, 4).unwrap();
    u32::from_le_bytes(out.try_into().unwrap())
}

#[test]
fn m3_t1_peripherals_end_to_end() {
    let mut m = load_machine();

    // 订阅 GpioLevel（GPIOA=port0, PA5=pin5）事件，捕获 LED 翻转
    let gpio_events: Arc<Mutex<Vec<Event>>> = Arc::new(Mutex::new(Vec::new()));
    let g = gpio_events.clone();
    m.events.lock().unwrap().subscribe(Arc::new(Mutex::new(
        move |ev: &Event| {
            if let Event::GpioLevel { port: 0, pin: 5, .. } = ev {
                g.lock().unwrap().push(ev.clone());
            }
        },
    )));

    m.run(200_000).unwrap();

    // 1) 主线完成 + 结果区
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_UART), 2, "USART 应发送 2 字节");
    assert_eq!(read_u32(&mut m, G_TMR), 4, "TIM2 IRQ28 应执行 4 次");
    assert_eq!(read_u32(&mut m, G_LED), 4, "BSRR 应翻转 4 次");

    // 2) USART TX → Console 字节流
    {
        let c = m.console.lock().unwrap();
        assert_eq!(c.output(), b"M3", "Console 应收到固件发送的 \"M3\"");
    }

    // 3) GPIO PA5 翻转事件（4 次，电平交替：高→低→高→低）
    {
        let evs = gpio_events.lock().unwrap();
        assert_eq!(evs.len(), 4, "PA5 应发布 4 次 GpioLevel 事件");
        let levels: Vec<bool> = evs
            .iter()
            .map(|e| match e {
                Event::GpioLevel { level, .. } => *level,
                _ => false,
            })
            .collect();
        assert_eq!(levels, vec![true, false, true, false], "PA5 电平应交替翻转");
    }

    // 4) TIM2 中断状态：全部返回线程模式，无残留挂起，优先级生效
    {
        let n = m.nvic.lock().unwrap();
        assert_eq!(n.priority(TIM2_IRQ), 0xF, "IRQ28 优先级应为 15");
        assert!(!n.in_handler(), "全部中断返回后应回到线程模式");
        assert!(!n.is_pending(TIM2_IRQ), "无残留挂起");
    }

    // 5) RCC 使能寄存器镜像
    assert_eq!(read_u32(&mut m, 0x4002_3830), 1, "AHB1ENR.GPIOA 应置位");
    assert_eq!(read_u32(&mut m, 0x4002_3840), 1, "APB1ENR.TIM2 应置位");
    assert_eq!(read_u32(&mut m, 0x4002_3844), 0x4000, "APB2ENR.USART1 应置位");

    // 6) 虚拟时钟已推进
    assert!(
        m.clock.lock().unwrap().cycles > 0,
        "block hook 应推进虚拟时钟"
    );
}
