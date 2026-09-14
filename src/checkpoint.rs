//! Checkpoint/快照恢复（调试平台 P2-2）。
//!
//! 目标：调试飞控固件的"时间旅行"——跑到某点保存快照，继续跑/注入故障/
//! 污染内存后**回滚**到保存点重放，或对比两条路径的固件行为差异。
//!
//! 快照内容（确定性回滚所需的最小状态）：
//! - **CPU 寄存器**：R0-R12/SP/LR/PC/xPSR + 特殊寄存器（MSP/PSP/PRIMASK 等）；
//! - **RAM**：SRAM1/2/3 + CCM（固件变量/栈/堆/诊断共享内存全在其中）；
//! - **虚拟时间**：retired 指令计数（恢复后时间线连续，推流/故障剧本续跑）。
//!
//! 明确**不**保存（fidelity 边界）：FLASH（固件代码不变）、外设寄存器/MMIO
//! （写回会触发外设副作用，且外设状态由外设层各自推进）、总线事务历史。
//! 恢复后外设（虚拟从设备/NVIC）继续按各自时钟推进——对"固件逻辑状态回滚"
//! 场景（断点→观测→回滚重放）语义正确。
//!
//! 用法：
//! ```rust
//! let snap = machine.snapshot()?;           // 保存点
//! machine.run(…)?;                          // 继续跑 / 注入故障
//! machine.restore(&snap)?;                  // 回滚
//! ```

use unicorn_engine::RegisterARM;

use crate::core::Cpu;

/// RAM 区域判定（快照保存范围）：SRAM1+2（0x2000_0000..0x2002_0000）、
/// SRAM3（0x2002_0000..0x2003_0000）、CCM（0x1000_0000..0x1001_0000）。
fn is_ram(addr: u64) -> bool {
    (0x2000_0000..0x2003_0000).contains(&addr) || (0x1000_0000..0x1001_0000).contains(&addr)
}

/// 单块内存映像。
#[derive(Debug, Clone)]
pub struct MemImage {
    pub begin: u64,
    pub data: Vec<u8>,
}

/// 机器快照。
#[derive(Debug, Clone)]
pub struct Snapshot {
    /// 退休指令计数（虚拟时间基准）。
    pub retired: u64,
    /// CPU 寄存器。
    pub regs: Vec<(RegisterARM, u64)>,
    /// RAM 内存映像。
    pub mem: Vec<MemImage>,
}

/// 快照/恢复所需的寄存器集合。
const SNAPSHOT_REGS: &[RegisterARM] = &[
    RegisterARM::R0, RegisterARM::R1, RegisterARM::R2, RegisterARM::R3,
    RegisterARM::R4, RegisterARM::R5, RegisterARM::R6, RegisterARM::R7,
    RegisterARM::R8, RegisterARM::R9, RegisterARM::R10, RegisterARM::R11,
    RegisterARM::R12, RegisterARM::SP, RegisterARM::LR, RegisterARM::PC,
    RegisterARM::XPSR, RegisterARM::MSP, RegisterARM::PSP, RegisterARM::PRIMASK,
    RegisterARM::BASEPRI, RegisterARM::FAULTMASK, RegisterARM::CONTROL,
];

/// 保存快照（CPU 寄存器 + RAM）。
pub fn snapshot(cpu: &mut Cpu, retired: u64) -> Result<Snapshot, String> {
    let mut regs = Vec::with_capacity(SNAPSHOT_REGS.len());
    for r in SNAPSHOT_REGS {
        let v = cpu
            .reg_read(*r)
            .map_err(|e| format!("快照读寄存器 {r:?}: {e}"))?;
        regs.push((*r, v));
    }

    let mut mem = Vec::new();
    let regions = cpu
        .raw()
        .mem_regions()
        .map_err(|e| format!("快照枚举内存区域: {e:?}"))?;
    for r in regions {
        let begin = r.begin;
        if !is_ram(begin) {
            continue;
        }
        let size = r.end - r.begin;
        let data = cpu
            .mem_read(begin, size as usize)
            .map_err(|e| format!("快照读内存 {begin:#x}: {e}"))?;
        mem.push(MemImage { begin, data });
    }

    Ok(Snapshot { retired, regs, mem })
}

/// 恢复快照（写回寄存器 + RAM + retired）。
pub fn restore(cpu: &mut Cpu, snap: &Snapshot, retired_sink: &std::sync::atomic::AtomicU64, last_virt_retired: &std::cell::Cell<u64>) -> Result<(), String> {
    // 1) 寄存器（先恢复，再写内存——避免写内存触发寄存器相关钩子读旧值）
    for (r, v) in &snap.regs {
        cpu.reg_write(*r, *v)
            .map_err(|e| format!("恢复寄存器 {r:?}: {e}"))?;
    }
    // 2) RAM
    for img in &snap.mem {
        cpu.mem_write(img.begin, &img.data)
            .map_err(|e| format!("恢复内存 {:#x}: {e}", img.begin))?;
    }
    // 3) 虚拟时间
    retired_sink.store(snap.retired, std::sync::atomic::Ordering::Relaxed);
    last_virt_retired.set(snap.retired);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::machine::Machine;

    fn machine() -> Machine {
        let mut m = Machine::new_m4f().unwrap();
        m.map_stm32f407_layout().unwrap();
        m
    }

    #[test]
    fn snapshot_restore_ram_roundtrip() {
        let mut m = machine();
        m.cpu.mem_write(0x2000_1000, &0x11223344u32.to_le_bytes()).unwrap();
        let snap = snapshot(&mut m.cpu, 0).unwrap();
        // 污染
        m.cpu.mem_write(0x2000_1000, &0xDEADBEEFu32.to_le_bytes()).unwrap();
        restore(&mut m.cpu, &snap, &std::sync::atomic::AtomicU64::new(0), &std::cell::Cell::new(0)).unwrap();
        let v = m.cpu.mem_read(0x2000_1000, 4).unwrap();
        assert_eq!(u32::from_le_bytes(v.try_into().unwrap()), 0x11223344);
    }

    #[test]
    fn snapshot_captures_registers() {
        let mut m = machine();
        m.cpu.reg_write(RegisterARM::R3, 0xCAFE).unwrap();
        let snap = snapshot(&mut m.cpu, 0).unwrap();
        let r3 = snap.regs.iter().find(|(r, _)| *r == RegisterARM::R3).unwrap().1;
        assert_eq!(r3, 0xCAFE);
    }

    #[test]
    fn snapshot_skips_flash_and_mmio() {
        let mut m = machine();
        let snap = snapshot(&mut m.cpu, 0).unwrap();
        // 只应包含 RAM 区域（SRAM1/2/3 + CCM）
        assert!(!snap.mem.is_empty());
        assert!(snap.mem.iter().all(|img| is_ram(img.begin)), "快照只应含 RAM");
    }
}
