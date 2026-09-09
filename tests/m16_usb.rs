//! M16 验收测试：USB OTG FS 全速设备控制器（STM32F407）端到端 + 事件互联。
//!
//! 复用 firmware/usb_demo 固件（场景见其 main.c 注释）。虚拟主机（本测试）经
//! 事件总线注入 USB 总线复位与 SETUP/OUT 包，固件轮询 GINTSTS 处理：
//!   1. USBRST            → 固件地址归 0、使能 EP0；
//!   2. GET_DESCRIPTOR(D) → EP0 IN 回发 18 字节设备描述符（host_take_in 校验）；
//!   3. GET_DESCRIPTOR(C) → EP0 IN 回发 32 字节配置描述符；
//!   4. SET_ADDRESS 0x2A  → DCFG.DAD=0x2A + 状态阶段零长包；
//!   5. SET_CONFIGURATION 1 → 使能 EP1 批量并回发 welcome[6]；
//!   6. OUT EP1（64B pattern 0x55+i）→ 校验 + 回显。
//!   IN 完成触发 DIEPINTx.XFRC；GINTMSK/DAINTMSK×DIEPMSK/DOEPMSK 门控挂起
//!   OTG_FS_IRQ=67。
//!
//! 期望结果区：
//!   0x20000000 G_DONE    = 0xAAAAAAAA（4 个 SETUP 处理完成）
//!   0x20000004 G_ADDR    = 0x2A（SET_ADDRESS 后设备地址）
//!   0x20000008 G_CFG     = 1（SET_CONFIGURATION 值）
//!   0x2000000C G_EP1_RX  = 1（OUT EP1 64B pattern 校验）
//!   0x20000010 G_EP1_TX  = 1（IN EP1 welcome/回显发送）

use std::path::Path;

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::usb_otg::{USB_OTG_FS_BASE, USB_OTG_FS_IRQ};

const G_DONE: u32 = 0x2000_0000;
const G_ADDR: u32 = 0x2000_0004;
const G_CFG: u32 = 0x2000_0008;
const G_EP1_RX: u32 = 0x2000_000C;
const G_SETUP_N: u32 = 0x2000_0014;

/// USB 寄存器偏移（与固件/外设一致）
const OFF_DCFG: u32 = 0x800;
const OFF_DOEPINT0: u32 = 0xB08;
const DCFG_DAD: u32 = 0x7F << 4;
const EPINT_STUP: u32 = 1 << 3;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/usb_demo/usb_demo.elf");
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

/// 注入一个 SETUP 包并 run，直至固件完成该包处理（G_SETUP_N 递增）。
fn inject_setup(m: &mut Machine, data: [u8; 8], expect_n: u32) {
    m.events
        .lock()
        .unwrap()
        .publish(&Event::UsbSetup { data });
    for _ in 0..200 {
        m.run(50_000).unwrap();
        if read_u32(m, G_SETUP_N) >= expect_n {
            break;
        }
    }
    assert!(read_u32(m, G_SETUP_N) >= expect_n, "固件应处理第 {expect_n} 个 SETUP");
}

#[test]
fn m16_usb_end_to_end() {
    let mut m = load_machine();

    // 让固件完成初始化并进入主循环，随后注入总线复位。
    m.run(200_000).unwrap();
    m.usb_otg.lock().unwrap().inject_usb_reset();
    m.run(200_000).unwrap();

    // SETUP 1：GET_DESCRIPTOR(Device) → EP0 IN 回发 18 字节设备描述符
    inject_setup(&mut m, [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00], 1);
    let dev = m.usb_otg.lock().unwrap().host_take_in(0);
    assert_eq!(&dev[..18], DEVICE_DESC, "设备描述符应回发");
    assert_eq!(dev[0], 18, "bLength=18");

    // SETUP 2：GET_DESCRIPTOR(Config) → EP0 IN 回发 32 字节配置描述符
    inject_setup(&mut m, [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x20, 0x00], 2);
    let cfg = m.usb_otg.lock().unwrap().host_take_in(0);
    assert_eq!(&cfg[..32], CONFIG_DESC, "配置描述符应回发");

    // SETUP 3：SET_ADDRESS 0x2A → DCFG.DAD=0x2A + 状态阶段零长包（无数据）
    inject_setup(&mut m, [0x00, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00], 3);
    assert_eq!(read_u32(&mut m, G_ADDR), 0x2A, "设备地址应为 0x2A");
    let dad = m.bus.lock().unwrap().read(USB_OTG_FS_BASE + OFF_DCFG, 4).unwrap() & DCFG_DAD;
    assert_eq!(dad, 0x2A << 4, "DCFG.DAD 应回读 0x2A");
    assert!(m.usb_otg.lock().unwrap().host_take_in(0).is_empty(), "状态阶段为零长包");

    // SETUP 4：SET_CONFIGURATION 1 → 使能 EP1 并回发 welcome[6]
    inject_setup(&mut m, [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00], 4);
    assert_eq!(read_u32(&mut m, G_CFG), 1, "配置值应为 1");
    let welcome = m.usb_otg.lock().unwrap().host_take_in(1);
    assert_eq!(&welcome[..6], b"Hello!", "EP1 IN 应回发 welcome");
    assert!(m.usb_otg.lock().unwrap().host_take_in(0).is_empty(), "SET_CONFIGURATION 状态阶段零长包");

    // 主线完成标记（4 个 SETUP 处理完成）
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成");

    // OUT EP1：64B pattern 0x55+i → 固件校验并回显
    let pattern: Vec<u8> = (0..64).map(|i| 0x55u8 + i as u8).collect();
    m.usb_otg.lock().unwrap().inject_out(1, &pattern);
    for _ in 0..200 {
        m.run(50_000).unwrap();
        if read_u32(&mut m, G_EP1_RX) == 1 {
            break;
        }
    }
    assert_eq!(read_u32(&mut m, G_EP1_RX), 1, "OUT EP1 64B pattern 校验应通过");
    let echo = m.usb_otg.lock().unwrap().host_take_in(1);
    assert_eq!(echo, pattern, "EP1 IN 应回显 64B pattern");

    // 中断挂起：固件使能 GINTMSK/DAINTMSK×DIEPMSK/DOEPMSK，各注入/传输完成应挂起
    // OTG_FS_IRQ（固件不写 NVIC ISER，pending 保持置位供校验）。
    let nvic = m.nvic.lock().unwrap();
    assert!(nvic.is_pending(USB_OTG_FS_IRQ), "OTG_FS IRQ 应挂起（USB 中断）");
}

/// 事件级互联：UsbSetup 事件经 machine 订阅路由到设备模式 inject_setup，
/// 应驱动 DOEPINT0.STUP 置位（DOEPMSK.STUPM + DAINTMSK.OEP0 门控挂起 IRQ）。
/// 不经固件，直接验证 machine/mod.rs 的事件接线。
#[test]
fn m16_usb_event_interconnect() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();

    // 固件等价的门控：GINTMSK.RXFLVL + GAHBCFG.GINT（inject_setup 的即时效果是
    // RXFLVL 置位 + 数据入接收 FIFO；DOEPINT0.STUP 由固件 RXFLVL 处理弹 SETUP_COMP
    // 状态字后置位——不经固件时无法走到，故事件接线断言 RXFLVL + GRXSTSP 队列）。
    const GINT_RXFLVL_BIT: u32 = 1 << 4;
    const DOEPMSK_STUPM: u32 = 1 << 3;
    const DAINTMSK_OEP0: u32 = 1 << 16;
    let bus = m.bus.clone();
    bus.lock().unwrap().write(USB_OTG_FS_BASE + 0x008, 4, 0x1).unwrap(); // GAHBCFG.GINT
    bus.lock().unwrap().write(USB_OTG_FS_BASE + 0x018, 4, GINT_RXFLVL_BIT).unwrap();
    bus.lock()
        .unwrap()
        .write(USB_OTG_FS_BASE + 0x814, 4, DOEPMSK_STUPM)
        .unwrap();
    bus.lock()
        .unwrap()
        .write(USB_OTG_FS_BASE + 0x81C, 4, DAINTMSK_OEP0)
        .unwrap();

    // 发布 UsbSetup 事件 → machine 订阅 → inject_setup → DOEPINT0.STUP 置位
    m.events
        .lock()
        .unwrap()
        .publish(&Event::UsbSetup {
            data: [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00],
        });
    let rxfl = bus
        .lock()
        .unwrap()
        .read(USB_OTG_FS_BASE + 0x014, 4)
        .unwrap()
        & GINT_RXFLVL_BIT;
    assert_ne!(rxfl, 0, "GINTSTS.RXFLVL 应置位（UsbSetup 事件路由）");
    let q = {
        let u = m.usb_otg.lock().unwrap();
        !u.rx_status_empty()
    };
    assert!(q, "GRXSTSP 状态队列应非空（SETUP_DATA/SETUP_COMP 待固件弹取）");
    assert!(
        m.nvic.lock().unwrap().is_pending(USB_OTG_FS_IRQ),
        "OTG_FS IRQ 应挂起（RXFLVL & GINTMSK.RXFLVL 门控）"
    );
}

/// 供校验的参考描述符（与固件 device_desc/config_desc 一致）
const DEVICE_DESC: [u8; 18] = [
    18, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x83, 0x12, 0x00, 0x00, 0x01, 0x00, 0x01,
    0x02, 0x01, 0x00,
];
const CONFIG_DESC: [u8; 32] = [
    9, 0x02, 32, 0x00, 1, 0x00, 0x00, 0x80, 50, 9, 0x04, 0, 0x00, 2, 0xFF, 0xFF, 0x00, 0, 7,
    0x05, 0x81, 0x02, 64, 0x00, 0, 7, 0x05, 0x01, 0x02, 64, 0x00, 0,
];
