//! M4 验收测试：DMA2 四流并发内存到内存传输端到端。
//!
//! 复用 firmware/dma2_demo 固件（DMA2 @ 0x40026400 + IRQ56-59）：
//! 1. 测试预置 4 段源数组 SRC0-3（0x20000100..0x200001C0，各 16 字）；
//! 2. 固件配置 DMA2 Stream0-3：各 PAR=SRCn、M0AR=DSTn@0x20000200..、NDTR=16、字宽、
//!    PINC+MINC、DIR=内存到内存、TCIE，写 EN 同时启动；
//! 3. 仿真器首个块 tick 遍历 8 流判定完成：各置 TCIF、按流挂起 IRQ56-59，
//!    run 间隙按 pending 位图逐流搬运 SRCn→DSTn；
//! 4. 四个 handler 各自校验 DSTn==SRCn → G_DMA_OK 置位；清 LIFCR → G_DMA_TC++；
//! 5. 主线写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_DMA_TC = 4（四条流 TC 中断各执行一次）
//!   0x20000004 G_DMA_OK = 0xF（各流 handler 内 DSTn==SRCn 校验位图）
//!   0x20000008 G_DONE   = 0xAAAAAAAA（主线完成）
//!   0x20000200/40/80/C0 DST0-3 = SRC0-3（仿真器直接内存校验搬运结果）

use std::path::Path;

use mcu_simulater::machine::Machine;

const G_DMA_TC: u32 = 0x2000_0000;
const G_DMA_OK: u32 = 0x2000_0004;
const G_DONE: u32 = 0x2000_0008;
const SRC0: u32 = 0x2000_0100;
const DST0: u32 = 0x2000_0200;
const NUM_WORDS: usize = 16;

/// 流 s 的 SRC 基址（固件固定布局：流间距 0x40 = 16 字）
fn src_base(s: u32) -> u32 {
    SRC0 + s * 0x40
}

fn dst_base(s: u32) -> u32 {
    DST0 + s * 0x40
}

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/dma2_demo/dma2_demo.elf");
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

/// 每条流独立的源数据，值可辨识、流间不重复
fn source_for(stream: usize) -> [u32; NUM_WORDS] {
    match stream {
        0 => [
            0xAAAA_5555, 0x0123_4567, 0x89AB_CDEF, 0xCAFE_F00D,
            0x0000_0001, 0x0000_0002, 0x0000_0004, 0x0000_0008,
            0x1000_0000, 0x2000_0000, 0x4000_0000, 0x8000_0000,
            0x5555_5555, 0x6666_6666, 0x7777_7777, 0x8888_8888,
        ],
        1 => {
            let mut a = [0u32; NUM_WORDS];
            for i in 0..NUM_WORDS {
                a[i] = 0xDEAD_0000 | i as u32;
            }
            a
        }
        2 => {
            let mut a = [0u32; NUM_WORDS];
            for i in 0..NUM_WORDS {
                a[i] = 1u32 << i;
            }
            a
        }
        _ => {
            let mut a = [0u32; NUM_WORDS];
            for i in 0..NUM_WORDS {
                a[i] = 0xCCCC_3333 ^ (i as u32).wrapping_mul(0x10203040);
            }
            a
        }
    }
}

#[test]
fn m4_dma2_multi_stream_concurrent_transfer_end_to_end() {
    let mut m = load_machine();

    // 预置 4 段源数组
    let srcs: Vec<[u32; NUM_WORDS]> = (0..4).map(source_for).collect();
    for (s, src) in srcs.iter().enumerate() {
        for (i, v) in src.iter().enumerate() {
            m.cpu
                .mem_write(src_base(s as u32) as u64 + i as u64 * 4, &v.to_le_bytes())
                .unwrap();
        }
    }

    m.run(200_000).unwrap();

    // 1) 主线完成 + 四条流中断各执行一次 + 各流 handler 校验位图
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_DMA_TC), 4, "四条流完成中断应各执行 1 次");
    assert_eq!(read_u32(&mut m, G_DMA_OK), 0xF, "四条流 handler 内 DST==SRC 校验应全部通过");

    // 2) 直接内存校验：各流搬运结果 DSTn == SRCn（互不干扰）
    for (s, src) in srcs.iter().enumerate() {
        for (i, v) in src.iter().enumerate() {
            let got = read_u32(&mut m, dst_base(s as u32) + i as u32 * 4);
            assert_eq!(got, *v, "DST{s}[{i}] 应等于 SRC{s}[{i}]");
        }
    }
}
