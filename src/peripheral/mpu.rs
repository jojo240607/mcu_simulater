//! Cortex-M4 内存保护单元（MPU）。
//!
//! M2 首版：寄存器文件 + 8 个 region 管理 + 数据/取指访问控制 + MemManage fault 记录。
//! 遵循设计文档 §5.9「默认全强制（保真优先）」：
//! MPU 使能后对 RAM/Flash/CCM 挂数据访问 hook 做检查；未使能时 hook 内部快速放行。
//!
//! 地址映射（相对 SCB 基址 0xE000E000 的偏移）：
//! - MMFSR 0xD28 / MMFAR 0xD34（SCB 内 MemManage 故障状态，由本模块维护，
//!   SCB 对该偏移做委托）
//! - MPU_TYPE 0xD90 / CTRL 0xD94 / RNR 0xD98 / RBAR 0xD9C / RASR 0xDA0
//! - 别名 0xDA4-0xDB8（region 1-3 的 RBAR/RASR 快速访问）

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use crate::peripheral::BusError;

/// 8 个 region
pub const MPU_REGION_COUNT: usize = 8;

// ---- 寄存器偏移（相对 SCB 基址 0xE000E000）----

/// MMFSR：MemManage 故障状态寄存器
pub const MMFSR_OFF: u32 = 0xD28;
/// MMFAR：MemManage 故障地址寄存器
pub const MMFAR_OFF: u32 = 0xD34;
/// MPU_TYPE（只读：DREGION=8）
pub const MPU_TYPE_OFF: u32 = 0xD90;
/// MPU_CTRL
pub const MPU_CTRL_OFF: u32 = 0xD94;
/// MPU_RNR
pub const MPU_RNR_OFF: u32 = 0xD98;
/// MPU_RBAR
pub const MPU_RBAR_OFF: u32 = 0xD9C;
/// MPU_RASR
pub const MPU_RASR_OFF: u32 = 0xDA0;
/// MPU 寄存器窗口起点/终点（含别名 0xDA4-0xDB8，窗口 0xD90..=0xDB8）
pub const MPU_WIN_START: u32 = MPU_TYPE_OFF;
pub const MPU_WIN_END: u32 = 0xDB8;

// ---- 访问类型与故障描述 ----

/// 一次内存访问的类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    /// 数据读
    Read,
    /// 数据写
    Write,
    /// 取指（执行）
    Fetch,
}

/// MemManage fault 种类
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemManageKind {
    /// 取指违规（IACCVIOL）
    InstructionAccess,
    /// 数据访问违规（DACCVIOL）
    DataAccess,
}

/// 一次 MPU 违规的描述（地址 + 种类）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MemManageFault {
    pub addr: u32,
    pub kind: MemManageKind,
}

/// 单个 region 表项：保存最后写入的 RBAR/RASR 原始值，命中判定时解码。
#[derive(Debug, Clone, Copy, Default)]
struct Region {
    rbar: u32,
    rasr: u32,
}

/// MPU 外设核心（寄存器文件 + region 表 + 访问控制）。
///
/// 挂载方式：SCB 委托本模块的寄存器窗口（MPU 区间 + MMFSR/MMFAR），
/// 访问控制由 Machine 注册的 mem/code hook 调用 [`Mpu::check`]。
pub struct Mpu {
    ctrl: u32,
    rnr: u32,
    regions: [Region; MPU_REGION_COUNT],
    mmfsr: u32,
    mmfar: u32,
    /// MPU 使能原子快速判定（CTRL.ENABLE 变化时同步）：Machine mem/code hook 据此
    /// 在未使能时跳过加锁的 check（纯计算负载下 MPU 未使能，省去每指令/每内存访问加锁）
    enabled: Arc<AtomicBool>,
}

impl Default for Mpu {
    fn default() -> Self {
        Self::new()
    }
}

impl Mpu {
    pub fn new() -> Self {
        Self::with_enabled(Arc::new(AtomicBool::new(false)))
    }

    /// 正式构造：`enabled` 由 Machine 持有（与 mpu 字段并行），CTRL.ENABLE 变化时同步
    pub fn with_enabled(enabled: Arc<AtomicBool>) -> Self {
        Self {
            ctrl: 0,
            rnr: 0,
            regions: [Region::default(); MPU_REGION_COUNT],
            mmfsr: 0,
            mmfar: 0,
            enabled,
        }
    }

    /// MPU 使能原子标记（无锁快速判定，供 mem/code hook 跳过加锁检查）
    pub fn enabled(&self) -> &Arc<AtomicBool> {
        &self.enabled
    }

    /// MPU 是否使能（CTRL.ENABLE=1）
    pub fn is_enabled(&self) -> bool {
        self.ctrl & 1 != 0
    }

    // ---- 访问控制 ----

    /// 对一次访问做权限检查。
    ///
    /// - MPU 未使能 → 放行（不强制）。
    /// - 命中 region：取指仅看 XN；数据访问看 AP（按特权级）。
    ///   ARMv7-M B3.5.4：地址命中多个 region 时，**最高编号 region 优先**，
    ///   故从高到低遍历，第一个命中即为优先级最高者（含 sub-region 排除后回退到低编号 region）。
    /// - 未命中（后台 region）：`PRIVDEFENA=1` 且特权 → 放行，否则违规。
    pub fn check(
        &self,
        addr: u32,
        access: Access,
        privileged: bool,
    ) -> Result<(), MemManageFault> {
        if !self.is_enabled() {
            return Ok(());
        }

        for r in self.regions.iter().rev() {
            if region_contains(r, addr) {
                // 取指：仅 XN 位决定（AP 不约束取指）
                if access == Access::Fetch {
                    if r.rasr & (1 << 28) != 0 {
                        return Err(MemManageFault {
                            addr,
                            kind: MemManageKind::InstructionAccess,
                        });
                    }
                    return Ok(());
                }
                // 数据访问：AP 权限表
                let ap = ((r.rasr >> 24) & 0x7) as u8;
                if !ap_allows(ap, access, privileged) {
                    return Err(MemManageFault {
                        addr,
                        kind: MemManageKind::DataAccess,
                    });
                }
                return Ok(());
            }
        }

        // 后台 region：PRIVDEFENA=1 且特权 → 全内存特权访问放行
        if self.ctrl & (1 << 2) != 0 && privileged {
            Ok(())
        } else {
            Err(MemManageFault {
                addr,
                kind: if access == Access::Fetch {
                    MemManageKind::InstructionAccess
                } else {
                    MemManageKind::DataAccess
                },
            })
        }
    }

    // ---- 故障记录 ----

    /// 记录一次违规：置位 MMFSR 对应位（IACCVIOL/DACCVIOL + MMARVALID），写 MMFAR。
    pub fn record_violation(&mut self, f: MemManageFault) {
        self.mmfsr |= 0x80; // MMARVALID
        self.mmfsr |= match f.kind {
            MemManageKind::InstructionAccess => 0x1, // IACCVIOL
            MemManageKind::DataAccess => 0x2,        // DACCVIOL
        };
        self.mmfar = f.addr;
    }

    /// 当前是否存在未清除的 MemManage fault（供仿真循环/测试检测）
    pub fn pending_fault(&self) -> Option<MemManageFault> {
        if self.mmfsr & 0x3 != 0 {
            Some(MemManageFault {
                addr: self.mmfar,
                kind: if self.mmfsr & 0x1 != 0 {
                    MemManageKind::InstructionAccess
                } else {
                    MemManageKind::DataAccess
                },
            })
        } else {
            None
        }
    }

    /// MMFSR 当前值（调试/测试）
    pub fn mmfsr(&self) -> u32 {
        self.mmfsr
    }

    /// MMFAR 当前值（调试/测试）
    pub fn mmfar(&self) -> u32 {
        self.mmfar
    }

    // ---- 寄存器读写（offset 为相对 SCB 基址的偏移）----

    pub fn read(&self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            MMFSR_OFF => Ok(self.mmfsr),
            MMFAR_OFF => Ok(self.mmfar),
            MPU_TYPE_OFF => Ok(0x0800), // DREGION=8，SEPARATE=0
            MPU_CTRL_OFF => Ok(self.ctrl),
            MPU_RNR_OFF => Ok(self.rnr),
            MPU_RBAR_OFF => Ok(self.rbar_for(self.rnr)),
            MPU_RASR_OFF => Ok(self.rasr_for(self.rnr)),
            o if o > MPU_RASR_OFF && o <= MPU_WIN_END => {
                // 别名区：0xDA4/0xDA8=region1，0xDAC/0xDB0=region2，0xDB4/0xDB8=region3
                let (idx, is_rbar) = alias_decode(o);
                if is_rbar {
                    Ok(self.rbar_for(idx as u32))
                } else {
                    Ok(self.rasr_for(idx as u32))
                }
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            MMFSR_OFF => {
                // 写 1 清零（write-1-to-clear）
                self.mmfsr &= !(value & 0xFF);
                Ok(())
            }
            MMFAR_OFF => Ok(()), // 只读
            MPU_TYPE_OFF => Err(BusError::NotImplemented),
            MPU_CTRL_OFF => {
                // ENABLE(0) / HFNMIENA(1) / PRIVDEFENA(2)；HFNMIENA 首版仅存储
                self.ctrl = value & 0x7;
                self.enabled.store(self.ctrl & 1 != 0, Ordering::Relaxed);
                Ok(())
            }
            MPU_RNR_OFF => {
                self.rnr = value & 0x7;
                Ok(())
            }
            MPU_RBAR_OFF => {
                self.write_rbar((self.rnr & 0x7) as usize, value, true);
                Ok(())
            }
            MPU_RASR_OFF => {
                self.regions[(self.rnr & 0x7) as usize].rasr = value;
                Ok(())
            }
            o if o > MPU_RASR_OFF && o <= MPU_WIN_END => {
                let (idx, is_rbar) = alias_decode(o);
                if is_rbar {
                    self.write_rbar(idx, value, false);
                } else {
                    self.regions[idx].rasr = value;
                }
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn rbar_for(&self, rnr: u32) -> u32 {
        self.regions[(rnr & 0x7) as usize].rbar
    }

    fn rasr_for(&self, rnr: u32) -> u32 {
        self.regions[(rnr & 0x7) as usize].rasr
    }

    /// 写 RBAR。`honor_valid=true`（主 RBAR 寄存器）时按 VALID/REGION 字段语义：
    /// VALID=1 → REGION 指定目标 region 且 RNR 同步更新；VALID=0 → 写入当前 RNR。
    /// 别名寄存器写（`honor_valid=false`）固定写别名 region。
    fn write_rbar(&mut self, idx: usize, value: u32, honor_valid: bool) {
        if honor_valid && value & (1 << 4) != 0 {
            let n = (value & 0xF) as usize;
            self.regions[n].rbar = value;
            self.rnr = value & 0xF;
        } else {
            self.regions[idx].rbar = value;
        }
    }
}

/// 解析别名寄存器偏移 → (region 索引, 是否 RBAR)
fn alias_decode(offset: u32) -> (usize, bool) {
    let rel = offset - (MPU_RASR_OFF + 4); // 0xDA4 起
    let idx = (rel / 8) as usize + 1;
    (idx, rel % 8 == 0)
}

/// region 是否覆盖 `addr`（含 sub-region 使能判断）
fn region_contains(r: &Region, addr: u32) -> bool {
    if r.rasr & 1 == 0 {
        return false; // ENABLE=0
    }
    let size = region_size_bytes(r.rasr);
    if size == 0 {
        return false; // SIZE 字段过小（<32B），语义不确定，视为不覆盖
    }
    // 精确语义：region 基址为 RBAR.ADDR[31:5] 再按 region 大小向下对齐
    // （ARMv7-M RBAR.ADDR 语义；u64 计算避免 SIZE=31 的 4GB 溢出）。
    let base = (u64::from(r.rbar) & !0x1F) & !(size - 1);
    let a = u64::from(addr);
    if a < base || a >= base + size {
        return false;
    }
    // sub-region：每区 1/8，SRD 位=1 表示禁用
    let sub = ((a - base) / (size / 8)) as u32;
    if r.rasr & (1 << (8 + sub)) != 0 {
        return false;
    }
    true
}

/// 解码 RASR.SIZE → region 字节数（2^(SIZE+1)），最小 32B；
/// 用 u64 表示，SIZE=31 时 4GB 不溢出。
fn region_size_bytes(rasr: u32) -> u64 {
    let size = (rasr >> 1) & 0x1F;
    if size < 4 {
        0
    } else {
        1u64 << (size + 1)
    }
}

/// ARMv7-M AP[2:0] 权限表（PL1=特权，PL0=用户）
fn ap_allows(ap: u8, access: Access, privileged: bool) -> bool {
    match ap {
        0b000 | 0b100 => false,                  // 无访问 / 保留
        0b001 => privileged,                     // 特权 rw，用户无
        0b010 => privileged || access == Access::Read, // 特权 rw，用户 ro
        0b011 => true,                           // 全部 rw
        0b101 => privileged && access == Access::Read, // 特权 ro，用户无
        0b110 | 0b111 => access == Access::Read, // 全部 ro
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mpu() -> Mpu {
        Mpu::new()
    }

    /// 程序化 region0：base + size_bytes + ap + xn
    fn prog_region0(m: &mut Mpu, base: u32, size_bytes: u32, ap: u8, xn: bool) {
        let rbar = base | (1 << 4) | 0; // VALID, REGION=0
        m.write(MPU_RBAR_OFF, 4, rbar).unwrap();
        let size_field = size_bytes.trailing_zeros() as u32 - 1; // size = 2^(SIZE+1)
        let mut rasr = (size_field << 1) | 1; // ENABLE
        rasr |= (ap as u32) << 24;
        if xn {
            rasr |= 1 << 28; // RASR.XN（bit28）
        }
        m.write(MPU_RASR_OFF, 4, rasr).unwrap();
    }

    /// 程序化任意 region：base + size_bytes + ap + xn（经 RNR + RBAR/VALID + RASR）
    fn prog_region(m: &mut Mpu, idx: u8, base: u32, size_bytes: u32, ap: u8, xn: bool) {
        let rbar = base | (1 << 4) | u32::from(idx); // VALID, REGION=idx
        m.write(MPU_RBAR_OFF, 4, rbar).unwrap();
        let size_field = size_bytes.trailing_zeros() as u32 - 1; // size = 2^(SIZE+1)
        let mut rasr = (size_field << 1) | 1; // ENABLE
        rasr |= (ap as u32) << 24;
        if xn {
            rasr |= 1 << 28; // RASR.XN（bit28）
        }
        m.write(MPU_RASR_OFF, 4, rasr).unwrap();
    }

    #[test]
    fn disabled_mpu_allows_all() {
        let m = mpu();
        // 未使能：任何访问均放行
        assert!(m.check(0x2000_0000, Access::Read, true).is_ok());
        assert!(m.check(0x2000_0000, Access::Write, false).is_ok());
        assert!(m.check(0x2000_0000, Access::Fetch, true).is_ok());
    }

    #[test]
    fn background_region_privdef() {
        // 使能 MPU，无 region：PRIVDEFENA=0 → 任何访问违规
        let mut m = mpu();
        m.write(MPU_CTRL_OFF, 4, 0x1).unwrap(); // ENABLE only
        assert!(matches!(
            m.check(0x2000_0000, Access::Read, true),
            Err(MemManageFault { kind: MemManageKind::DataAccess, .. })
        ));

        // PRIVDEFENA=1 → 特权放行，用户违规
        let mut m = mpu();
        m.write(MPU_CTRL_OFF, 4, 0x5).unwrap(); // ENABLE | PRIVDEFENA
        assert!(m.check(0x2000_0000, Access::Write, true).is_ok());
        assert!(m.check(0x2000_0000, Access::Fetch, true).is_ok());
        assert!(matches!(
            m.check(0x2000_0000, Access::Write, false),
            Err(MemManageFault { kind: MemManageKind::DataAccess, .. })
        ));
    }

    #[test]
    fn region_ap_permission() {
        // region0 = [0x20000000, +0x40) AP=010（特权 rw / 用户 ro）
        let mut m = mpu();
        prog_region0(&mut m, 0x2000_0000, 0x40, 0b010, false);
        m.write(MPU_CTRL_OFF, 4, 0x5).unwrap();

        assert!(m.check(0x2000_0000, Access::Write, true).is_ok());
        assert!(m.check(0x2000_0000, Access::Read, true).is_ok());
        assert!(m.check(0x2000_0010, Access::Read, false).is_ok());
        assert!(m.check(0x2000_0010, Access::Write, false).is_err()); // 用户写被拒
        // 区域外（后台特权）放行
        assert!(m.check(0x2000_1000, Access::Write, true).is_ok());
    }

    #[test]
    fn region_xn_blocks_fetch() {
        let mut m = mpu();
        prog_region0(&mut m, 0x0800_0000, 0x10000, 0b011, true); // XN=1
        m.write(MPU_CTRL_OFF, 4, 0x5).unwrap();

        // 取指被拒（IACCVIOL），数据访问仍放行
        assert!(matches!(
            m.check(0x0800_0100, Access::Fetch, true),
            Err(MemManageFault { kind: MemManageKind::InstructionAccess, .. })
        ));
        assert!(m.check(0x0800_0100, Access::Read, true).is_ok());
        assert!(m.check(0x0800_0100, Access::Write, true).is_ok());
    }

    #[test]
    fn subregion_disable() {
        // region0 = [0x20000000, +0x100) = 8 个 32B sub-region；禁用 sub0（SRD bit0=1）
        let mut m = mpu();
        let base = 0x2000_0000u32;
        let rbar = base | (1 << 4);
        m.write(MPU_RBAR_OFF, 4, rbar).unwrap();
        // SIZE=7 → 2^8=256B；SRD[0]=1 禁用第一个 sub-region；AP=011
        let rasr = (7 << 1) | (0b011 << 24) | (0x01 << 8) | 1;
        m.write(MPU_RASR_OFF, 4, rasr).unwrap();
        m.write(MPU_CTRL_OFF, 4, 0x1).unwrap(); // ENABLE，无 PRIVDEFENA

        // 被禁用 sub0 内（0x20000000-0x2000001F）→ 后台 region（无 PRIVDEFENA）→ 违规
        assert!(m.check(0x2000_0000, Access::Read, true).is_err());
        // sub1（0x20000020）→ 命中 region，AP=011 放行
        assert!(m.check(0x2000_0020, Access::Write, true).is_ok());
    }

    #[test]
    fn overlap_higher_region_number_wins() {
        // ARMv7-M B3.5.4：重叠 region 取最高编号者。
        // 场景 A：region0 放行（AP=011 全 rw）、region7 只读（AP=101 特权 ro）→ 写应被拒（region7 胜）。
        let mut m = mpu();
        prog_region(&mut m, 0, 0x2000_0000, 0x40, 0b011, false);
        prog_region(&mut m, 7, 0x2000_0000, 0x40, 0b101, false);
        m.write(MPU_CTRL_OFF, 4, 0x5).unwrap(); // ENABLE + PRIVDEFENA

        assert!(m.check(0x2000_0000, Access::Read, true).is_ok());
        assert!(matches!(
            m.check(0x2000_0000, Access::Write, true),
            Err(MemManageFault { kind: MemManageKind::DataAccess, .. })
        ), "高编号 region7（只读）应覆盖低编号 region0（rw）");

        // 场景 B：region0 只读、region7 全 rw → 写应放行（region7 胜）。
        let mut m = mpu();
        prog_region(&mut m, 0, 0x2000_0000, 0x40, 0b101, false);
        prog_region(&mut m, 7, 0x2000_0000, 0x40, 0b011, false);
        m.write(MPU_CTRL_OFF, 4, 0x5).unwrap();

        assert!(m.check(0x2000_0000, Access::Write, true).is_ok());
    }

    #[test]
    fn base_aligned_to_region_size() {
        // 精确语义：RBAR.ADDR 按 region 大小向下对齐。
        // SIZE=6 → 128B；写入基址 0x20000040（32 对齐但非 128 对齐）→ 有效基址 0x20000000，
        // region 覆盖 [0x20000000, 0x20000080)。
        let mut m = mpu();
        m.write(MPU_RNR_OFF, 4, 0).unwrap();
        m.write(MPU_RBAR_OFF, 4, 0x2000_0040 | (1 << 4)).unwrap(); // VALID, REGION=0
        let rasr = (6 << 1) | (0b011 << 24) | 1; // SIZE=6(128B), AP=011, ENABLE
        m.write(MPU_RASR_OFF, 4, rasr).unwrap();
        m.write(MPU_CTRL_OFF, 4, 0x1).unwrap(); // ENABLE（无 PRIVDEFENA）

        // 有效基址内的地址命中 region（放行）
        assert!(m.check(0x2000_0010, Access::Write, true).is_ok());
        assert!(m.check(0x2000_0070, Access::Write, true).is_ok());
        // 超出有效 region 范围 → 后台 region（无 PRIVDEFENA）→ 违规
        assert!(m.check(0x2000_0080, Access::Read, true).is_err());
    }

    #[test]
    fn register_programming_and_readback() {
        let mut m = mpu();
        // RNR=2，写 RBAR（VALID 清位：REGION 字段被忽略）
        m.write(MPU_RNR_OFF, 4, 0x2).unwrap();
        m.write(MPU_RBAR_OFF, 4, 0x1234_0000 & !0x1F).unwrap();
        m.write(MPU_RASR_OFF, 4, 0x1000_001B).unwrap(); // XN(bit28), SIZE=13(16KB), ENABLE
        assert_eq!(m.read(MPU_RBAR_OFF, 4).unwrap(), 0x1234_0000 & !0x1F);
        assert_eq!(m.read(MPU_RASR_OFF, 4).unwrap(), 0x1000_001B);

        // VALID=1 写 RBAR 选中 region1 并更新 RNR
        m.write(MPU_RBAR_OFF, 4, 0x2000_0000 | (1 << 4) | 1).unwrap();
        assert_eq!(m.rnr, 1);
        assert_eq!(m.read(MPU_RBAR_OFF, 4).unwrap(), 0x2000_0000 | (1 << 4) | 1);

        // 别名区：region1 RBAR/RASR
        m.write(MPU_RASR_OFF, 4, 0x0101_0011).unwrap();
        assert_eq!(m.read(0xDA4, 4).unwrap(), 0x2000_0000 | (1 << 4) | 1);
        assert_eq!(m.read(0xDA8, 4).unwrap(), 0x0101_0011);

        // MPU_TYPE 只读，DREGION=8
        assert_eq!(m.read(MPU_TYPE_OFF, 4).unwrap(), 0x0800);
    }

    #[test]
    fn fault_recording() {
        let mut m = mpu();
        m.record_violation(MemManageFault {
            addr: 0x2000_0004,
            kind: MemManageKind::DataAccess,
        });
        assert_eq!(m.mmfsr() & 0x82, 0x82); // DACCVIOL + MMARVALID
        assert_eq!(m.mmfar(), 0x2000_0004);
        assert_eq!(
            m.pending_fault(),
            Some(MemManageFault {
                addr: 0x2000_0004,
                kind: MemManageKind::DataAccess,
            })
        );

        // MMFSR 写 1 清零
        m.write(MMFSR_OFF, 4, 0x82).unwrap();
        assert!(m.pending_fault().is_none());
    }
}
