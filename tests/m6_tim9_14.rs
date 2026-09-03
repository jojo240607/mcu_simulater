//! M6 验收测试：TIM9-14 挂载 + 随虚拟时钟推进（F407 补充定时器）。
//!
//! STM32F407 共 14 个定时器：既有 TIM1-8，还有 TIM9/12（通用 16 位 2 通道）
//! 与 TIM10/11/13/14（通用 16 位 1 通道）。本测试验证：
//! 1. TIM9（APB2 0x40014000）、TIM12（APB1 0x40001800）、TIM14（APB1 0x40002000）
//!    已挂载到 MMIO，寄存器可写读（CNT 直写回读）；
//! 2. 使能 CEN 后随块级加权虚拟时钟推进（CNT 递增，分频后不溢出）；
//! 3. TIM9-14 无 DMA 请求能力（machine 不为其注册 DMA 句柄/路由）——
//!    该项由代码结构保证（`register_tim` 仅对 port≤8），本测试不重复断言。
//!
//! 外设寄存器经 Machine 公共总线 `bus.write/read` 直写（等价 CPU 访问 MMIO 触发
//! 外设写路径，与固件驱动方式同链）。复用的固件 tim_advanced_demo 自身驱动
//! TIM1/TIM3，与本测试探测的 TIM9/12/14 互不冲突。

use std::path::Path;

use mcu_simulater::machine::Machine;

const TIM9_BASE: u32 = 0x4001_4000; // APB2
const TIM12_BASE: u32 = 0x4000_1800; // APB1
const TIM14_BASE: u32 = 0x4000_2000; // APB1
const OFF_CR1: u32 = 0x00;
const OFF_PSC: u32 = 0x28;
const OFF_ARR: u32 = 0x2C;
const OFF_CNT: u32 = 0x24;
const CR1_CEN: u32 = 1 << 0;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/tim_advanced_demo/tim_advanced_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

fn write_u32(m: &mut Machine, addr: u32, value: u32) {
    m.bus.lock().unwrap().write(addr, 4, value).unwrap();
}

fn read_u32(m: &mut Machine, addr: u32) -> u32 {
    m.bus.lock().unwrap().read(addr, 4).unwrap()
}

#[test]
fn m6_tim9_14_mounted_and_tick() {
    let mut m = load_machine();

    // 1) 挂载 + 寄存器可写读（CNT 直写回读，命中 TIM 寄存器文件）
    write_u32(&mut m, TIM9_BASE + OFF_CNT, 0x1234);
    assert_eq!(read_u32(&mut m, TIM9_BASE + OFF_CNT), 0x1234, "TIM9 CNT 可写读");
    write_u32(&mut m, TIM12_BASE + OFF_CNT, 0xABCD);
    assert_eq!(read_u32(&mut m, TIM12_BASE + OFF_CNT), 0xABCD, "TIM12 CNT 可写读");
    write_u32(&mut m, TIM14_BASE + OFF_CNT, 0x5678);
    assert_eq!(read_u32(&mut m, TIM14_BASE + OFF_CNT), 0x5678, "TIM14 CNT 可写读");

    // 2) 使能计数并随虚拟时钟推进：PSC=0xFFFF 分频（每 65536 周期 CNT+1），
    //    ARR=0xFFFF 上限足够大，run(50_000)≈150k 周期内 CNT 小且不回绕。
    for base in [TIM9_BASE, TIM12_BASE, TIM14_BASE] {
        write_u32(&mut m, base + OFF_PSC, 0xFFFF);
        write_u32(&mut m, base + OFF_ARR, 0xFFFF);
        write_u32(&mut m, base + OFF_CNT, 0);
        write_u32(&mut m, base + OFF_CR1, CR1_CEN);
    }
    m.run(50_000).unwrap();

    for (name, base) in [("TIM9", TIM9_BASE), ("TIM12", TIM12_BASE), ("TIM14", TIM14_BASE)] {
        let cnt = read_u32(&mut m, base + OFF_CNT);
        assert!(cnt > 0 && cnt < 0xFFFF, "{name} 应随虚拟时钟推进（CNT={cnt}）");
    }
}
