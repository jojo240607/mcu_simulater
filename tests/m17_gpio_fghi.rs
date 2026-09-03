//! M17 验收测试：GPIO 端口补齐 A-I（9 个端口齐全）。
//!
//! F407 共 GPIOA..GPIOI 9 个端口；此前仅挂载 A-E（port 0..4）。本测试验证：
//! 1. GPIOF/G/H/I（port 5..8）经总线 attach 后可读写（MODER/ODR/IDR/BSRR）；
//! 2. port 5..8 的 GpioLevel 事件经 SYSCFG_EXTICR 端口选择 → EXTI → NVIC IRQ6 挂起
//!    （EXTICR/EXTI 无端口上限，补齐端口后全链路生效）；
//! 3. RCC AHB1ENR bit5..8（GPIOF..I 时钟）镜像写读。
//!
//! 说明：CPU `mem_read/mem_write` API 不触发 MMIO hook（仅直写底层内存），
//! 动态值（IDR/PR 等）与写转发须经 `Machine::bus` 直接访问外设寄存器验证；
//! CPU 指令经 hook 转发的路径已由 m3/m4/m16 等固件测试覆盖。

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const GPIO_BASE: u32 = 0x4002_0000;
const OFF_MODER: u32 = 0x00;
const OFF_IDR: u32 = 0x10;
const OFF_ODR: u32 = 0x14;
const OFF_BSRR: u32 = 0x18;
const SYSCFG_EXTICR1: u32 = 0x4001_3808;
const EXTI_BASE: u32 = 0x4001_3C00;
const OFF_IMR: u32 = 0x00;
const OFF_RTSR: u32 = 0x08;
const OFF_PR: u32 = 0x14;
const RCC_AHB1ENR: u32 = 0x4002_3830;

fn load_machine() -> Machine {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.reset().unwrap();
    m
}

#[test]
fn m17_gpio_fghi_bus_rw() {
    let m = load_machine();
    let bus = m.bus.clone();
    // 全部 9 个端口 A..I：MODER/ODR/IDR/BSRR 经总线可写读
    for port in 0..9u32 {
        let base = GPIO_BASE + port * 0x400;
        let label = (b'A' + port as u8) as char;
        // MODER 全输出
        bus.lock().unwrap().write(base + OFF_MODER, 4, 0xFFFF_FFFF).unwrap();
        assert_eq!(
            bus.lock().unwrap().read(base + OFF_MODER, 4).unwrap(),
            0xFFFF_FFFF,
            "GPIO{label} MODER 应可写读"
        );
        // ODR 写读 + IDR 自环
        bus.lock().unwrap().write(base + OFF_ODR, 4, 0x8000).unwrap();
        assert_eq!(bus.lock().unwrap().read(base + OFF_ODR, 4).unwrap(), 0x8000, "GPIO{label} ODR 应可写读");
        assert_eq!(bus.lock().unwrap().read(base + OFF_IDR, 4).unwrap(), 0x8000, "GPIO{label} IDR 应读回 ODR");
        // BSRR 置位 pin0/2 → ODR 变 0x8005
        bus.lock().unwrap().write(base + OFF_BSRR, 4, 0x5).unwrap();
        assert_eq!(bus.lock().unwrap().read(base + OFF_ODR, 4).unwrap(), 0x8005, "GPIO{label} BSRR 置位应生效");
    }
}

#[test]
fn m17_gpio_fghi_exti_route() {
    let m = load_machine();
    let bus = m.bus.clone();
    // EXTI 线0：开放中断 + 上升沿触发
    bus.lock().unwrap().write(EXTI_BASE + OFF_IMR, 4, 0x1).unwrap();
    bus.lock().unwrap().write(EXTI_BASE + OFF_RTSR, 4, 0x1).unwrap();

    // 对 port 5..8（GPIOF..I）逐一验证：EXTICR 选择 → GpioLevel 脉冲 → EXTI PR + NVIC IRQ6
    for port in 5..9u8 {
        let label = (b'A' + port) as char;
        bus.lock().unwrap().write(SYSCFG_EXTICR1, 4, port as u32).unwrap(); // 线0 选择 GPIO{label}
        m.events.lock().unwrap().publish(&Event::GpioLevel {
            port,
            pin: 0,
            level: false, // 复位沿检测状态
        });
        m.events.lock().unwrap().publish(&Event::GpioLevel {
            port,
            pin: 0,
            level: true, // 上升沿 → EXTI0 PR + IRQ6 挂起
        });
        assert_eq!(
            bus.lock().unwrap().read(EXTI_BASE + OFF_PR, 4).unwrap(),
            1,
            "GPIO{label} 事件应置 EXTI0 PR"
        );
        assert!(
            m.nvic.lock().unwrap().is_pending(6),
            "GPIO{label} 事件应挂起 IRQ6"
        );
        // 清 PR（写 1 清 0），进入下一轮
        bus.lock().unwrap().write(EXTI_BASE + OFF_PR, 4, 0x1).unwrap();
        assert_eq!(bus.lock().unwrap().read(EXTI_BASE + OFF_PR, 4).unwrap(), 0, "EXTI0 PR 应写 1 清除");
    }

    // 端口不匹配（EXTICR 选 GPIOI=8，喂 GPIOA=0）→ 不应触发线0
    m.events.lock().unwrap().publish(&Event::GpioLevel {
        port: 0,
        pin: 0,
        level: true,
    });
    assert_eq!(
        bus.lock().unwrap().read(EXTI_BASE + OFF_PR, 4).unwrap(),
        0,
        "端口不匹配不应触发 EXTI0"
    );
}

#[test]
fn m17_gpio_fghi_rcc_clocks() {
    let m = load_machine();
    let bus = m.bus.clone();
    // AHB1ENR bit5..8 = GPIOF..I 时钟使能（GPIOA..E 为 bit0..4）
    bus.lock().unwrap().write(RCC_AHB1ENR, 4, 0x1E0).unwrap();
    assert_eq!(
        bus.lock().unwrap().read(RCC_AHB1ENR, 4).unwrap(),
        0x1E0,
        "AHB1ENR GPIOF..I 时钟位应镜像"
    );
}
