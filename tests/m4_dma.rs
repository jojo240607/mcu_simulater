//! M4 验收测试：DMA1 内存到内存传输端到端（MEM2MEM → TC 中断 → handler 校验数据）。
//!
//! 复用 firmware/dma_demo 固件（场景见其 main.c 注释）：
//! 1. 测试预置源数组 SRC@0x20000100（16 字）；
//! 2. 固件配置 DMA1 Stream0：PAR=SRC、M0AR=DST@0x20000200、NDTR=16、字宽、
//!    PINC+MINC、DIR=内存到内存、TCIE，写 EN 启动；
//! 3. 仿真器首个块 tick 判定完成：置 TCIF、挂起 IRQ11、run 间隙搬运 SRC→DST；
//! 4. DMA1_Stream0_IRQHandler：校验 DST==SRC → G_DMA_OK；清 LIFCR → G_DMA_TC++；
//! 5. 主线写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_DMA_TC = 1（TC 中断执行次数）
//!   0x20000004 G_DMA_OK = 1（handler 内 DST==SRC 校验）
//!   0x20000008 G_DONE   = 0xAAAAAAAA（主线完成）
//!   0x20000200 DST      = SRC（仿真器直接内存校验搬运结果）

use std::path::Path;

use mcu_simulater::machine::Machine;

const G_DMA_TC: u32 = 0x2000_0000;
const G_DMA_OK: u32 = 0x2000_0004;
const G_DONE: u32 = 0x2000_0008;
const SRC: u32 = 0x2000_0100;
const DST: u32 = 0x2000_0200;
const NUM_WORDS: usize = 16;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/dma_demo/dma_demo.elf");
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
fn m4_dma_mem2mem_transfer_end_to_end() {
    let mut m = load_machine();

    // 预置源数组（16 个字，测试可辨识值）
    let src: [u32; NUM_WORDS] = [
        0xDEAD_BEEF, 0x0123_4567, 0x89AB_CDEF, 0xCAFE_F00D,
        0x0000_0001, 0x0000_0002, 0x0000_0004, 0x0000_0008,
        0x1000_0000, 0x2000_0000, 0x4000_0000, 0x8000_0000,
        0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444,
    ];
    for (i, v) in src.iter().enumerate() {
        m.cpu
            .mem_write(SRC as u64 + i as u64 * 4, &v.to_le_bytes())
            .unwrap();
    }

    m.run(200_000).unwrap();

    // 1) 主线完成 + 中断执行 + handler 校验
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_DMA_TC), 1, "DMA 完成中断应执行 1 次");
    assert_eq!(read_u32(&mut m, G_DMA_OK), 1, "handler 内 DST==SRC 校验应通过");

    // 2) 直接内存校验：DMA 搬运结果 DST == SRC
    for (i, v) in src.iter().enumerate() {
        let got = read_u32(&mut m, DST + i as u32 * 4);
        assert_eq!(got, *v, "DST[{i}] 应等于 SRC[{i}]");
    }
}
