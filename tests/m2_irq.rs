//! M2 验收测试：NVIC 中断投递链路（挂起 → 抢占 → 嵌套 → EXC_RETURN 出栈恢复）。
//!
//! 复用 firmware/irq_acceptance 固件（场景见其 main.c 注释）：
//! 1. IRQ0 优先级 5（高），IRQ1 优先级 10（低），线程模式同时挂起两者 → 先进 IRQ0；
//! 2. IRQ0 返回后进 IRQ1；IRQ1 内自挂起 IRQ0 → 高优先级嵌套抢占；
//! 3. IRQ0 嵌套返回后 IRQ1 继续，全部返回主线写 G_DONE。
//!
//! 期望结果区（SRAM 固定地址）：
//!   0x20000000 G_COUNT0 = 2（首次 + 嵌套）
//!   0x20000004 G_COUNT1 = 1
//!   0x20000008 G_INNER  = 0x22222222（IRQ1 在嵌套返回后继续）
//!   0x2000000C G_DONE   = 0xAAAAAAAA（主线完成）

use std::path::Path;

use mcu_simulater::machine::Machine;

const G_COUNT0: u32 = 0x2000_0000;
const G_COUNT1: u32 = 0x2000_0004;
const G_INNER: u32 = 0x2000_0008;
const G_DONE: u32 = 0x2000_000C;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/irq_acceptance/irq_acceptance.elf");
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
fn m2_irq_preemption_nesting_return() {
    let mut m = load_machine();

    // 执行至固件完成（G_DONE 写完后进入死循环，达到指令数上限自然停止）
    m.run(200_000).unwrap();

    // 优先级配置已生效（数值小 = 优先级高）
    {
        let n = m.nvic.lock().unwrap();
        assert_eq!(n.priority(0), 0x5, "IRQ0 优先级应为 5");
        assert_eq!(n.priority(1), 0xA, "IRQ1 优先级应为 10");
        assert!(!n.in_handler(), "全部中断返回后应回到线程模式");
        assert!(!n.is_pending(0) && !n.is_pending(1), "无残留挂起");
    }

    // 结果区校验
    assert_eq!(read_u32(&mut m, G_COUNT0), 2, "IRQ0 应执行 2 次（首次 + 嵌套抢占）");
    assert_eq!(read_u32(&mut m, G_COUNT1), 1, "IRQ1 应执行 1 次");
    assert_eq!(read_u32(&mut m, G_INNER), 0x2222_2222, "IRQ1 嵌套返回后应继续");
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "全部中断返回后主线应完成");
}
