//! M6 验收测试：TIM2 更新事件 → DMA1 内存→外设突发装载 CCR 表端到端。
//!
//! 复用 firmware/tim_dma_demo 固件（场景见其 main.c 注释）：
//! 1. 固件配置 TIM2（ARR=1000、DCR.DBA=13/DBL=3、DIER.UDE、CR1.CEN）+ 接
//!    DMA1_Stream5_Ch5（TX：内存→外设，字宽，MINC，TCIE），NVIC 使能 IRQ16；
//! 2. TIM2 计数溢出（tick 推进）→ 更新事件 + UDE → 发布 TimUpdate → Machine 路由
//!    service_stream(5,5,MemToPeriph,Tim(2)) 登记待搬运（NDTR=4）；
//! 3. run 间隙 Dma::process：从 CCR_TABLE 依次读 4 字 → Tim2::dma_write_dr 按
//!    DCR.DBA/DBL（突发长度 = DBL+1 = 4）写入 CCR1..CCR4；NDTR 归零 → EN 自清 +
//!    TCIF → IRQ16 → DMA1_Stream5_IRQHandler 校验 CCR1..CCR4 ==
//!    {0x1111,0x2222,0x3333,0x4444} → G_DMA_TC++；
//! 4. 主线轮询 G_DMA_TC 达 1 → 写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_DMA_TC = 1（DMA1_Stream5 完成中断执行次数）
//!   0x20000004 G_DONE  = 0xAAAAAAAA（主线完成）
//!   0x40000034..0x40   TIM2 CCR1..CCR4 = {0x1111,0x2222,0x3333,0x4444}
//!                      （DMA 突发装载结果，仿真器直接校验）

use std::path::Path;

use mcu_simulater::machine::Machine;

const G_DMA_TC: u32 = 0x2000_0000;
const G_DONE: u32 = 0x2000_0004;
const TIM2_CCR1: u32 = 0x4000_0034;
const CCR_EXPECT: [u32; 4] = [0x1111, 0x2222, 0x3333, 0x4444];

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/tim_dma_demo/tim_dma_demo.elf");
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
fn m5_tim_dma_end_to_end() {
    let mut m = load_machine();

    // 固件初始化：配置 TIM2 + DMA1_Stream5 + NVIC，随后进入主循环等待 DMA 完成。
    // 时序说明：TIM2 计数溢出（tick 推进）→ 更新事件 DMA 请求在 CPU 空闲间隙
    // （每次 emu_start 返回后）执行搬运并置起 IRQ15；故分多次 run 让 process 有
    // 机会执行、中断得以投递，直到主线写 G_DONE。
    for _ in 0..10 {
        m.run(200_000).unwrap();
        if read_u32(&mut m, G_DONE) == 0xAAAA_AAAA {
            break;
        }
    }

    // 1) DMA 完成中断执行 1 次 + 主线完成
    assert_eq!(read_u32(&mut m, G_DMA_TC), 1, "DMA1_Stream5 完成中断应执行 1 次");
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");

    // 2) TIM2 CCR1..CCR4 经 DMAR 突发装载为表内容
    for (i, v) in CCR_EXPECT.iter().enumerate() {
        let got = read_u32(&mut m, TIM2_CCR1 + (i as u32) * 4);
        assert_eq!(got, *v, "TIM2_CCR{} 应为 0x{:04X}（DMA 突发装载结果）", i + 1, v);
    }
}
