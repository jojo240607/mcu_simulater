//! M4 验收测试：RCC 时钟树端到端（HSE → PLL → 168MHz 系统时钟）。
//!
//! 复用 firmware/rcc_demo 固件（场景见其 main.c 注释）：
//! 1. 使能 HSE，轮询 HSERDY（状态位联动：写 HSEON 立即置 HSERDY）；
//! 2. 配置 PLLCFGR（HSE/8MHz：M=8,N=336,P=2 → 168MHz），使能 PLL，轮询 PLLRDY；
//! 3. 切换 SW=PLL，轮询 SWS==PLL（写 CFGR 时按请求源+就绪推导 SWS）；
//! 4. 配置 HPRE=/1, PPRE1=/4, PPRE2=/2，写 G_DONE。
//!
//! 期望结果区（SRAM 固定地址）：
//!   0x20000000 G_OBS  = 7（HSERDY/PLLRDY/SWS 三阶段观察全部成功）
//!   0x20000004 G_DONE = 0xAAAAAAAA（主线完成，证明无死循环、状态位联动正确）
//!
//! RCC 寄存器均为"直接存值"语义（RDY/SWS 为联动位，只出现在 MMIO 读路径），
//! 故 mem_read 读到的 guest 镜像 = 固件写入的原始值，可直接断言。

use std::path::Path;

use mcu_simulater::machine::Machine;

const G_OBS: u32 = 0x2000_0000;
const G_DONE: u32 = 0x2000_0004;

const RCC_CR: u32 = 0x4002_3800;
const RCC_PLLCFGR: u32 = 0x4002_3804;
const RCC_CFGR: u32 = 0x4002_3808;
const RCC_AHB1ENR: u32 = 0x4002_3830;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/rcc_demo/rcc_demo.elf");
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
fn m4_rcc_clock_tree_end_to_end() {
    let mut m = load_machine();
    m.run(200_000).unwrap();

    // 1) 主线完成 + 状态位联动观察位
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_OBS), 7, "HSERDY/PLLRDY/SWS 三阶段观察应全部成功");

    // 2) RCC 寄存器镜像
    //    CR：固件最后写 HSEON|PLLON=0x01010000，但轮询读 CR 时 MMIO read hook 把联动后的
    //    读值注入 guest 内存 → 0x03030000（HSEON|HSERDY|PLLON|PLLRDY），证明 RDY 联动生效
    assert_eq!(
        read_u32(&mut m, RCC_CR),
        0x0303_0000,
        "CR 读回应含 HSERDY|PLLRDY（状态位联动）"
    );
    //    PLLCFGR：PLLSRC=HSE(bit22), M=8, N=336, P=0(2), Q=7（固件未再读，镜像=写入值）
    assert_eq!(
        read_u32(&mut m, RCC_PLLCFGR),
        0x0740_5408,
        "PLLCFGR 应为 HSE/M8/N336/P2/Q7 配置"
    );
    //    CFGR：固件写 SW=PLL,HPRE=/1,PPRE1=/4,PPRE2=/2=0x9402，轮询读 CFGR 时注入
    //    联动后的读值 → 0x940A（SWS=PLL），证明 SWS 切换生效
    assert_eq!(
        read_u32(&mut m, RCC_CFGR),
        0x940A,
        "CFGR 读回应含 SWS=PLL（系统时钟已切到 PLL）"
    );
    //    AHB1ENR：GPIOA 时钟使能（固件未再读，镜像=写入值）
    assert_eq!(read_u32(&mut m, RCC_AHB1ENR), 1, "AHB1ENR.GPIOA 应置位");
}
