//! Checkpoint/快照恢复（调试平台 P2-2）端到端验收。
//!
//! 加载真实固件（i2c_irq_demo），验证时间旅行语义：
//! - **确定性回滚**：A 点快照 → 继续 run → restore(A) → 再 run 相同量 →
//!   PC 与"未回滚直接 run"相同（固件行为确定性重放）；
//! - **内存污染恢复**：快照后改 RAM → restore → 值恢复原样；
//! - **快照后续跑**：恢复后固件继续执行不崩（G_TX/G_DONE 正常推进）。

use std::path::Path;

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;
use unicorn_engine::RegisterARM;

const G_TX: u32 = 0x2000_0000;

fn machine_with_firmware() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware/i2c_irq_demo/i2c_irq_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

fn pc(m: &mut Machine) -> u32 {
    m.cpu.reg_read_u32(RegisterARM::PC).unwrap()
}

#[test]
fn deterministic_rollback_replays_identically() {
    let mut m = machine_with_firmware();
    m.run_budget(20_000).unwrap(); // 配置阶段
    let snap = m.snapshot().unwrap();
    let pc_a = pc(&mut m);

    // 路径 1：从 A 直接 run 固定量
    m.run_budget(30_000).unwrap();
    let pc_fwd = pc(&mut m);
    let gtx_fwd = u32::from_le_bytes(m.cpu.mem_read(G_TX as u64, 4).unwrap().try_into().unwrap());

    // 路径 2：回滚到 A，再 run 相同量 → 应完全一致（确定性）
    m.restore(&snap).unwrap();
    assert_eq!(pc(&mut m), pc_a, "restore 后 PC 应回到保存点");
    m.run_budget(30_000).unwrap();
    let pc_replay = pc(&mut m);
    let gtx_replay = u32::from_le_bytes(m.cpu.mem_read(G_TX as u64, 4).unwrap().try_into().unwrap());

    assert_eq!(pc_fwd, pc_replay, "回滚重放后 PC 应一致（确定性）");
    assert_eq!(gtx_fwd, gtx_replay, "回滚重放后 G_TX 应一致");
    assert!(gtx_replay >= 1, "固件应已完成问候发送：G_TX={gtx_replay}");
}

#[test]
fn restore_clears_memory_pollution() {
    let mut m = machine_with_firmware();
    m.run_budget(10_000).unwrap();
    let snap = m.snapshot().unwrap();
    let orig = u32::from_le_bytes(m.cpu.mem_read(G_TX as u64, 4).unwrap().try_into().unwrap());

    // 污染 G_TX（模拟调试期误写）
    m.cpu.mem_write(G_TX as u64, &0xDEADBEEFu32.to_le_bytes()).unwrap();
    assert_ne!(
        u32::from_le_bytes(m.cpu.mem_read(G_TX as u64, 4).unwrap().try_into().unwrap()),
        orig
    );

    // restore → 值恢复
    m.restore(&snap).unwrap();
    let after = u32::from_le_bytes(m.cpu.mem_read(G_TX as u64, 4).unwrap().try_into().unwrap());
    assert_eq!(after, orig, "restore 后内存污染应清除");
}

#[test]
fn continues_running_after_restore() {
    let mut m = machine_with_firmware();
    m.run_budget(15_000).unwrap();
    let snap = m.snapshot().unwrap();
    m.run_budget(5_000).unwrap(); // 推进一点
    m.restore(&snap).unwrap();

    // 恢复后继续跑：注入 RX 字节 → 固件事件中断接收链路仍工作
    for b in [b'a', b'b', b'c', b'd'] {
        m.events
            .lock()
            .unwrap()
            .publish(&Event::I2cRx { port: 1, byte: b });
        m.run_budget(20_000).unwrap();
    }
    let done = u32::from_le_bytes(m.cpu.mem_read(0x2000_0008, 4).unwrap().try_into().unwrap());
    assert_eq!(done, 0xAAAA_AAAA, "恢复后固件应能完成主循环（G_DONE）");
}
