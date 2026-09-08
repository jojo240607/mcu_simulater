//! M14 验收测试：SDIO 安全数字 IO（STM32F407）端到端 + 总线命令/写路径。
//!
//! 复用 firmware/sdio_demo 固件（场景见其 main.c 注释）：
//!   Phase A（卡初始化命令路径）：CMD0/8/55/ACMD41/3/7 依次响应正确
//!         （R7 回显 0x1AA、R1+APP_CMD、R3 OCR、R6 RCA、R1 TRAN）→ G_INIT_OK=1；
//!   Phase B（单块写 CMD24 + DMA2_Stream6_Ch4 TX）：内存缓冲 128 字经 DMA
//!         内存→外设搬运到卡块 0 → DATAEND + TCIF6 → G_WRITE_OK=1；
//!   Phase C（单块读 CMD17 + DMA2_Stream3_Ch4 RX）：卡块 0 经 DMA 外设→内存
//!         搬回并与写缓冲一致 → G_READ_OK=1；主线写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_INIT_OK  = 1（Phase A 命令路径校验）
//!   0x20000004 G_WRITE_OK = 1（Phase B 写 DMA 搬运校验）
//!   0x20000008 G_READ_OK  = 1（Phase C 读回校验）
//!   0x2000000C G_DONE     = 0xAAAAAAAA（主线完成）
//!   0x20000040 SD_BUF     = 写源缓冲（128 字，字 i = 0x11110000 + i）
//!   0x20000240 RD_BUF     = 读回缓冲（与 SD_BUF 一致）

use std::path::Path;

use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::sdio::{BLOCK_SIZE, CARD_SIZE};

const G_INIT_OK: u32 = 0x2000_0000;
const G_WRITE_OK: u32 = 0x2000_0004;
const G_READ_OK: u32 = 0x2000_0008;
const G_DONE: u32 = 0x2000_000C;
const SD_BUF: u32 = 0x2000_0040;
const RD_BUF: u32 = 0x2000_0240;

/// SDIO 寄存器（与固件/外设一致）
const SDIO_BASE: u32 = 0x4001_2C00;
const OFF_ARG: u32 = 0x08;
const OFF_CMD: u32 = 0x0C;
const OFF_RESPCMD: u32 = 0x10;
const OFF_RESP1: u32 = 0x14;
const OFF_DLEN: u32 = 0x28;
const OFF_DCTRL: u32 = 0x2C;
const OFF_STATUS: u32 = 0x34; // F4: STA@0x34
const OFF_FIFO: u32 = 0x80;

/// 位定义（与 src/peripheral/sdio.rs 一致）
const CMD_CPSMEN: u32 = 1 << 10;
const CMD_WAITRESP_1: u32 = 1 << 6;
const DCTRL_DTEN: u32 = 1 << 0;
const ST_CMDREND: u32 = 1 << 6;
const ST_CMDSENT: u32 = 1 << 7;
const ST_DATAEND: u32 = 1 << 8;

/// 写模式：字 i = 0x11110000 + i（小端落盘）
fn write_pattern(i: u32) -> u32 {
    0x1111_0000 + i
}

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/sdio_demo/sdio_demo.elf");
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
fn m14_sdio_end_to_end() {
    let mut m = load_machine();

    // 全流程自包含（命令 + DMA 搬运），无外部注入。DMA 搬运在 run() 间隙执行
    // （emu_start 返回后），故分多次 run() 让小预算自旋让出 CPU → 搬运完成。
    for _ in 0..200 {
        m.run(500_000).unwrap();
        if read_u32(&mut m, G_DONE) == 0xAAAA_AAAA {
            break;
        }
    }
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");
    assert_eq!(read_u32(&mut m, G_INIT_OK), 1, "Phase A 卡初始化命令路径校验应通过");
    assert_eq!(read_u32(&mut m, G_WRITE_OK), 1, "Phase B 单块写 DMA 搬运应通过");
    assert_eq!(read_u32(&mut m, G_READ_OK), 1, "Phase C 单块读回校验应通过");

    // 额外校验：写缓冲确实落盘到虚拟卡块 0（小端逐字节）
    {
        let sdio = m.sdio.lock().unwrap();
        let card = sdio.card_bytes(0, BLOCK_SIZE);
        assert_eq!(card.len(), BLOCK_SIZE, "卡块 0 应完整");
        for i in 0..128u32 {
            let w = write_pattern(i);
            for b in 0..4 {
                let expect = (w >> (8 * b)) as u8;
                assert_eq!(
                    card[(i * 4 + b) as usize],
                    expect,
                    "卡块 0 字节 {} 应等于写模式（字 {i}）",
                    i * 4 + b
                );
            }
        }
        // 块 1 之后仍为初始值 0xA5（未写入）
        assert_eq!(sdio.card_bytes(BLOCK_SIZE, 4), &[0xA5; 4], "块 1 应保持初始值");
    }

    // 额外校验：读回缓冲与写源缓冲逐字一致
    for i in 0..128u32 {
        assert_eq!(
            read_u32(&mut m, RD_BUF + i * 4),
            read_u32(&mut m, SD_BUF + i * 4),
            "RD_BUF 字 {i} 应与 SD_BUF 一致"
        );
    }

    // 卡容量边界（1MB）未被越界访问
    assert_eq!(
        m.sdio.lock().unwrap().card_bytes(0, CARD_SIZE).len(),
        CARD_SIZE
    );
}

/// 总线直写：不经固件，验证 SDIO 挂载的命令/响应链路（MMIO + 状态机）。
#[test]
fn m14_sdio_bus_cmd() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    let bus = m.bus.clone();

    // CMD8（R7）：ARG=0x1AA → 写 CMD（CPSMEN）→ RESP1 回显 + CMDREND
    bus.lock().unwrap().write(SDIO_BASE + OFF_ARG, 4, 0x1AA).unwrap();
    bus.lock()
        .unwrap()
        .write(SDIO_BASE + OFF_CMD, 4, 8 | CMD_WAITRESP_1 | CMD_CPSMEN)
        .unwrap();
    assert_eq!(
        bus.lock().unwrap().read(SDIO_BASE + OFF_RESP1, 4).unwrap(),
        0x1AA,
        "R7 应回显校验模式"
    );
    assert_eq!(
        bus.lock().unwrap().read(SDIO_BASE + OFF_RESPCMD, 4).unwrap(),
        8,
        "RESPCMD 应为 8"
    );
    assert_ne!(
        bus.lock().unwrap().read(SDIO_BASE + OFF_STATUS, 4).unwrap() & ST_CMDREND,
        0,
        "应置 CMDREND"
    );

    // CMD0（无响应）：清状态后写 → 仅 CMDSENT，无 CMDREND
    bus.lock().unwrap().write(SDIO_BASE + OFF_STATUS, 4, 0x0).unwrap_err(); // 只读
    bus.lock().unwrap().write(SDIO_BASE + 0x38, 4, 0x3FF).unwrap(); // ICR@0x38 清 bit0-9
    bus.lock()
        .unwrap()
        .write(SDIO_BASE + OFF_CMD, 4, 0 | CMD_CPSMEN)
        .unwrap();
    let st = bus.lock().unwrap().read(SDIO_BASE + OFF_STATUS, 4).unwrap();
    assert_ne!(st & ST_CMDSENT, 0, "无响应命令应置 CMDSENT");
    assert_eq!(st & ST_CMDREND, 0, "无响应命令不应置 CMDREND");
}

/// 总线直写：验证写路径流式落盘（CMD24 + DCTRL 写方向 + FIFO 推字 → 卡块 1）。
#[test]
fn m14_sdio_bus_write_block() {
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    let bus = m.bus.clone();

    // DLEN=512、CMD24（块 1）→ DCTRL 写方向启动传输
    bus.lock().unwrap().write(SDIO_BASE + OFF_DLEN, 4, 512).unwrap();
    bus.lock().unwrap().write(SDIO_BASE + OFF_ARG, 4, 512).unwrap();
    bus.lock()
        .unwrap()
        .write(SDIO_BASE + OFF_CMD, 4, 24 | CMD_WAITRESP_1 | CMD_CPSMEN)
        .unwrap();
    bus.lock().unwrap().write(SDIO_BASE + OFF_DCTRL, 4, DCTRL_DTEN).unwrap();

    // 推 128 字到 FIFO（流式落盘）→ DATAEND
    for i in 0..128u32 {
        bus.lock().unwrap().write(SDIO_BASE + OFF_FIFO, 4, write_pattern(i)).unwrap();
    }
    let st = bus.lock().unwrap().read(SDIO_BASE + OFF_STATUS, 4).unwrap();
    assert_ne!(st & ST_DATAEND, 0, "推满 128 字应置 DATAEND");

    // 校验卡块 1 已按小端写入
    let sdio = m.sdio.lock().unwrap();
    let card = sdio.card_bytes(512, 4);
    assert_eq!(card, &[0x00, 0x00, 0x11, 0x11], "卡块 1 首字应等于 0x11110000 小端");
}
