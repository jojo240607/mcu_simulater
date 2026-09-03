//! M12 验收测试：DCMI 数字摄像头接口（STM32F407）端到端 + 事件注入 + DMA 搬运。
//!
//! 复用 firmware/dcmi_demo 固件（场景见其 main.c 注释）：
//!   Phase A（轮询读 DR）：固件 CR.ENABLE+CAPTURE 使能 → 写 G_READY_A=1 →
//!         测试发布 DcmiFrame{帧 A：字节 01..10} → feed_frame 拆 4 字入 FIFO
//!         （SR.FRAME + RIS）→ 固件轮询 SR.FRAME → 读 DR×4 校验一致 → G_POLL_OK=1；
//!   Phase B（DMA 搬运）：固件配置 DMA2_Stream1_Channel1（外设→内存，字，MINC，
//!         NDTR=4，PAR=DCMI_DR）→ 写 G_READY_B=1 → 测试发布 DcmiFrame{帧 B：
//!         字节 11..20} → feed_frame 后路由 service_stream_n → run 间隙
//!         Dma::process 从 DCMI DR 搬 4 字到 0x20000040 → TCIF → 固件校验
//!         缓冲区 → G_DMA_OK=1 → 主线写 G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_POLL_OK = 1（Phase A 轮询读 DR 校验）
//!   0x20000004 G_READY_A = 1（Phase A 就绪）
//!   0x20000008 G_DMA_OK  = 1（Phase B DMA 搬运校验）
//!   0x2000000C G_READY_B = 1（Phase B 就绪）
//!   0x20000010 G_DONE    = 0xAAAAAAAA（主线完成）
//!   0x20000040 DCMI_BUF  = 帧 B 四字 {0x14131211,0x18171615,0x1C1B1A19,0x201F1E1D}

use std::path::Path;

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const G_POLL_OK: u32 = 0x2000_0000;
const G_READY_A: u32 = 0x2000_0004;
const G_DMA_OK: u32 = 0x2000_0008;
const G_READY_B: u32 = 0x2000_000C;
const G_DONE: u32 = 0x2000_0010;
const DCMI_BUF: u32 = 0x2000_0040;

/// 帧 B（DMA 搬运）期望缓冲字：字节 0x11..0x20 → 低位在前 4 字
const DMA_BUF_EXPECT: [u32; 4] = [0x1413_1211, 0x1817_1615, 0x1C1B_1A19, 0x201F_1E1D];

/// DCMI 寄存器地址（与固件一致）
const DCMI_CR: u32 = 0x5005_0000;
const DCMI_SR: u32 = 0x5005_0004;
const DCMI_IER: u32 = 0x5005_000C;
const DCMI_DR: u32 = 0x5005_0028;

/// 位定义（与 src/peripheral/dcmi.rs 一致）
const CR_CAPTURE: u32 = 1 << 0;
const CR_ENABLE: u32 = 1 << 14;
const SR_FNE: u32 = 1 << 0;
const SR_FRAME: u32 = 1 << 7;
const INT_FRAME: u32 = 1 << 0;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/dcmi_demo/dcmi_demo.elf");
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

/// 注入一帧并推进 CPU，直到条件成立或达到最大轮次。
fn inject_and_run(m: &mut Machine, data: Vec<u8>, target: u32, value: u32) {
    m.events
        .lock()
        .unwrap()
        .publish(&Event::DcmiFrame { port: 1, data });
    for _ in 0..20 {
        m.run(200_000).unwrap();
        if read_u32(m, target) == value {
            break;
        }
    }
}

#[test]
fn m12_dcmi_end_to_end() {
    let mut m = load_machine();

    // 阶段一：固件使能 DCMI 并写 G_READY_A（等待注入帧 A）
    m.run(10_000).unwrap();
    assert_eq!(read_u32(&mut m, G_READY_A), 1, "Phase A 应就绪等待帧注入");

    // 注入帧 A（16 字节 0x01..0x10 → 4 字）→ 固件轮询读 DR 校验 → G_POLL_OK
    inject_and_run(&mut m, (0x01u8..=0x10).collect(), G_POLL_OK, 1);
    assert_eq!(
        read_u32(&mut m, G_POLL_OK),
        1,
        "Phase A 轮询读 DR 校验应通过"
    );
    // 固件随后配置 DMA 并写 G_READY_B
    assert_eq!(read_u32(&mut m, G_READY_B), 1, "Phase B 应就绪等待帧注入");

    // 注入帧 B（16 字节 0x11..0x20 → 4 字）→ DMA 搬运 + 固件校验 → G_DONE
    inject_and_run(&mut m, (0x11u8..=0x20).collect(), G_DONE, 0xAAAA_AAAA);
    assert_eq!(read_u32(&mut m, G_DMA_OK), 1, "Phase B DMA 搬运校验应通过");
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");

    // 额外校验 DMA 缓冲区内容（帧 B 四字，低位在前）
    for (i, w) in DMA_BUF_EXPECT.iter().enumerate() {
        assert_eq!(
            read_u32(&mut m, DCMI_BUF + (i as u32) * 4),
            *w,
            "DMA 缓冲区字 {i} 应等于期望（帧 B）"
        );
    }
}

/// 事件注入 + 总线直读：不经固件，验证 DCMI 挂载（MMIO/事件/DMA 句柄）链路。
#[test]
fn m12_dcmi_bus_and_event() {
    let m = load_machine();
    let nvic = m.nvic.clone();

    // 使能 + 捕获（连续模式）→ 发布 DcmiFrame 注入一帧
    m.bus
        .lock()
        .unwrap()
        .write(DCMI_CR, 4, CR_ENABLE | CR_CAPTURE)
        .unwrap();
    m.events
        .lock()
        .unwrap()
        .publish(&Event::DcmiFrame {
            port: 1,
            data: vec![0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        });

    // SR.FRAME + FNE 置位；读 DR 按字弹出
    let sr = m.bus.lock().unwrap().read(DCMI_SR, 4).unwrap();
    assert_ne!(sr & SR_FRAME, 0, "注入帧应置 SR.FRAME");
    assert_ne!(sr & SR_FNE, 0, "注入帧应置 SR.FNE");
    assert_eq!(
        m.bus.lock().unwrap().read(DCMI_DR, 4).unwrap(),
        0x0403_0201,
        "DR 应按低位在前弹出第 1 字"
    );
    assert_eq!(
        m.bus.lock().unwrap().read(DCMI_DR, 4).unwrap(),
        0x0807_0605,
        "DR 应弹出第 2 字"
    );

    // 使能 IER 帧中断后注入 → 挂起 DCMI 全局中断（IRQ78）
    m.bus
        .lock()
        .unwrap()
        .write(DCMI_IER, 4, INT_FRAME)
        .unwrap();
    m.events
        .lock()
        .unwrap()
        .publish(&Event::DcmiFrame {
            port: 1,
            data: vec![0xAA; 4],
        });
    assert!(
        nvic.lock().unwrap().is_pending(78),
        "IER 使能帧中断后注入应挂起 DCMI IRQ"
    );
}
