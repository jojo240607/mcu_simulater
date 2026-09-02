//! M4 验收测试：EXTI 外部中断端到端（GPIO 事件 → EXTI 沿检测 → NVIC IRQ6 → handler）。
//!
//! 复用 firmware/exti_demo 固件（场景见其 main.c 注释）：
//! 1. RCC 使能 GPIOA/SYSCFG 时钟镜像；PA5 输出、PA0 输入（EXTI0 线）；
//! 2. SYSCFG_EXTICR1=0 → EXTI0 接 GPIOA（复位默认，显式写出）；
//! 3. EXTI0：RTSR=1（上升沿触发）、IMR=1（开放中断）；
//! 4. NVIC IRQ6（EXTI0）优先级 15 + 使能；
//! 5. 测试向事件总线发布 PA0 低→高脉冲（模拟外部驱动上升沿）4 次；
//! 6. EXTI0_IRQHandler：清 PR → G_EXTI++ → BSRR 翻转 PA5（发布 GpioLevel 事件）。
//!
//! 期望结果区（SRAM 固定地址）：
//!   0x20000000 G_EXTI = 4（EXTI0 handler 执行次数）
//!   0x20000004 G_LED  = 4（BSRR 翻转次数）
//!   0x20000008 G_DONE = 0xAAAAAAAA（主线完成）

use std::path::Path;
use std::sync::{Arc, Mutex};

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const G_EXTI: u32 = 0x2000_0000;
const G_LED: u32 = 0x2000_0004;
const G_DONE: u32 = 0x2000_0008;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/exti_demo/exti_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

fn read_u32(m: &mut Machine, addr: u32) -> u32 {
    // SRAM 结果区直接经 CPU 读取；MMIO 外设（RCC/SYSCFG/EXTI）由 mem hook 注入，同样可读
    let out = m.cpu.mem_read(addr as u64, 4).unwrap();
    u32::from_le_bytes(out.try_into().unwrap())
}

/// 向事件总线发布 PA0（GPIOA=port0, pin0）电平，模拟外部驱动。
fn drive_pa0(m: &mut Machine, level: bool) {
    m.events
        .lock()
        .unwrap()
        .publish(&Event::GpioLevel { port: 0, pin: 0, level });
}

#[test]
fn m4_exti_external_interrupt_end_to_end() {
    let mut m = load_machine();

    // 订阅 PA5（GPIOA=port0, pin5）GpioLevel 事件，捕获 LED 翻转
    let gpio_events: Arc<Mutex<Vec<Event>>> = Arc::new(Mutex::new(Vec::new()));
    let g = gpio_events.clone();
    m.events.lock().unwrap().subscribe(Arc::new(Mutex::new(
        move |ev: &Event| {
            if let Event::GpioLevel { port: 0, pin: 5, .. } = ev {
                g.lock().unwrap().push(ev.clone());
            }
        },
    )));

    // 1) 先跑一段让固件完成 RCC/SYSCFG/EXTI/NVIC 配置并进入主循环
    m.run(10_000).unwrap();

    // 2) 注入 4 次 PA0 低→高脉冲（上升沿触发 EXTI0 → IRQ6），每次后运行让 handler 执行
    for _ in 0..4 {
        drive_pa0(&mut m, false); // 下降沿（FTSR 未使能，不触发），复位沿检测状态
        drive_pa0(&mut m, true); // 上升沿 → EXTI0 PR=1 → NVIC IRQ6 pending
        m.run(50_000).unwrap(); // handler：清 PR、G_EXTI++、BSRR 翻转 PA5
    }
    m.run(50_000).unwrap(); // 主线退出循环，写 G_DONE

    // 3) 结果区
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_EXTI), 4, "EXTI0 handler 应执行 4 次");
    assert_eq!(read_u32(&mut m, G_LED), 4, "BSRR 应翻转 4 次");

    // 4) GPIO PA5 翻转事件（4 次，电平交替：高→低→高→低）
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

    // 5) EXTI 状态：PR 清除由 G_EXTI=4 间接验证——
    //    若某次 PR 未清，后续注入（trigger_line 的"PR 已置位不重复拉高"）将不触发，G_EXTI 到不了 4。
    //    （注：CPU mem_read 不触发 MMIO read hook，直接读 guest 镜像，故不在此断言寄存器值）

    // 6) NVIC 状态：IRQ6 优先级 15，返回线程模式，无残留挂起
    {
        let n = m.nvic.lock().unwrap();
        assert_eq!(n.priority(6), 0xF, "IRQ6 优先级应为 15");
        assert!(!n.in_handler(), "全部中断返回后应回到线程模式");
        assert!(!n.is_pending(6), "无残留挂起");
    }

    // 7) RCC 使能寄存器镜像 + SYSCFG EXTICR1 显式写 0（EXTI0 接 GPIOA）
    assert_eq!(read_u32(&mut m, 0x4002_3830), 1, "AHB1ENR.GPIOA 应置位");
    assert_eq!(read_u32(&mut m, 0x4002_3844), 0x4000, "APB2ENR.SYSCFG 应置位");
    assert_eq!(read_u32(&mut m, 0x4001_3808), 0, "SYSCFG_EXTICR1 应为 0（EXTI0→GPIOA）");
}
