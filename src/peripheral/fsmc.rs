//! FSMC 灵活静态存储器控制器（STM32F407，M13 虚拟外设生态）。
//!
//! 简化模型：NOR/PSRAM/SRAM 片选窗口（Bank1-4）+ 寄存器文件（BCR/BTR/BWTR）。
//! - 寄存器基址 0xA0000000；BCR1-4/BTR1-4（0x00-0x1C）、BWTR1-4（0x104-0x110）
//!   为"配置存储"：写值按可写位掩码保存、可读回，时序/类型/宽度等位无功能仿真；
//! - BCRn.MBKEN（bit0）= 1 使能对应片选窗口（Bankn 基址起 64KB 后备缓冲）。
//!   MBKEN=0 时窗口读返回 0、写被丢弃（外设未选通的简化行为）；
//! - 片选窗口映射（"简化为窗口映射"）：
//!   Bank1 @0x60000000、Bank2 @0x64000000、Bank3 @0x68000000、Bank4 @0x6C000000，
//!   各 64KB（真实片选区间 64MB 的简化；按字节后备，支持 8/16/32 位访问宽度）。

use crate::peripheral::{BusError, Peripheral};

/// FSMC 寄存器基址
pub const FSMC_BASE: u32 = 0xA000_0000;
/// Bank1-4 片选窗口基址
pub const FSMC_BANK1_BASE: u32 = 0x6000_0000;
pub const FSMC_BANK2_BASE: u32 = 0x6400_0000;
pub const FSMC_BANK3_BASE: u32 = 0x6800_0000;
pub const FSMC_BANK4_BASE: u32 = 0x6C00_0000;
/// 每个片选窗口大小（64KB，简化窗口映射）
pub const FSMC_BANK_SIZE: usize = 0x1_0000;

/// BCRn.MBKEN：片选使能（窗口映射生效）
pub const BCR_MBKEN: u32 = 1 << 0;
/// BCR 可写位掩码（类型/宽度/时序等位仅存储，无功能仿真）
const BCR_WMASK: u32 = BCR_MBKEN
    | (0x3 << 2)   // MTYP
    | (0x3 << 4)   // MWID
    | (1 << 6)     // FACCEN
    | (1 << 8)     // BURSTEN
    | (1 << 9)     // WAITPOL
    | (0x3 << 10)  // WAITCFG
    | (1 << 12)    // WREN
    | (1 << 13)    // WAITEN
    | (1 << 14)    // EXTMOD
    | (1 << 15)    // ASYNCWAIT
    | (1 << 19)    // CBURSTRW
    | (1 << 20);   // CDIS

// 寄存器偏移
const OFF_BCR1: u32 = 0x00;
const OFF_BTR1: u32 = 0x04;
const OFF_BCR2: u32 = 0x08;
const OFF_BTR2: u32 = 0x0C;
const OFF_BCR3: u32 = 0x10;
const OFF_BTR3: u32 = 0x14;
const OFF_BCR4: u32 = 0x18;
const OFF_BTR4: u32 = 0x1C;
const OFF_BWTR1: u32 = 0x104;
const OFF_BWTR2: u32 = 0x108;
const OFF_BWTR3: u32 = 0x10C;
const OFF_BWTR4: u32 = 0x110;

/// 寄存器文件索引（BCR1-4 = 0..4，BTR1-4 = 4..8，BWTR1-4 = 8..12）
fn bcr_idx(offset: u32) -> usize {
    match offset {
        OFF_BCR1 => 0,
        OFF_BCR2 => 1,
        OFF_BCR3 => 2,
        OFF_BCR4 => 3,
        _ => unreachable!(),
    }
}

fn btr_idx(offset: u32) -> usize {
    match offset {
        OFF_BTR1 => 4,
        OFF_BTR2 => 5,
        OFF_BTR3 => 6,
        OFF_BTR4 => 7,
        _ => unreachable!(),
    }
}

fn bwtr_idx(offset: u32) -> usize {
    match offset {
        OFF_BWTR1 => 8,
        OFF_BWTR2 => 9,
        OFF_BWTR3 => 10,
        OFF_BWTR4 => 11,
        _ => unreachable!(),
    }
}

/// 窗口地址 → (bank 索引 0..4, 窗口内偏移)；落在任一 64KB 窗口外返回 None
fn window_bank(addr: u32) -> Option<(usize, u32)> {
    const BASES: [u32; 4] = [
        FSMC_BANK1_BASE,
        FSMC_BANK2_BASE,
        FSMC_BANK3_BASE,
        FSMC_BANK4_BASE,
    ];
    for (i, base) in BASES.iter().enumerate() {
        let start = *base as usize;
        let end = start + FSMC_BANK_SIZE;
        let a = addr as usize;
        if (start..end).contains(&a) {
            return Some((i, addr.wrapping_sub(*base)));
        }
    }
    None
}

/// FSMC 灵活静态存储器控制器
pub struct Fsmc {
    /// 寄存器文件（BCR1-4/BTR1-4/BWTR1-4，索引 0..12）
    regs: [u32; 12],
    /// Bank1-4 片选窗口后备缓冲（各 64KB；MBKEN=1 时 R/W 命中）
    banks: [Box<[u8]>; 4],
}

impl Fsmc {
    pub fn new() -> Self {
        let banks = std::array::from_fn(|_| vec![0u8; FSMC_BANK_SIZE].into_boxed_slice());
        Self {
            regs: [0; 12],
            banks,
        }
    }

    /// Bank（0..4）片选是否使能（BCRn.MBKEN）
    pub fn bank_enabled(&self, bank: usize) -> bool {
        self.regs[bank] & BCR_MBKEN != 0
    }

    /// 窗口读：MBKEN=1 且窗口内 → 后备缓冲数据（低字节在前）；否则返回 0。
    pub fn window_read(&self, addr: u32, size: u32) -> u32 {
        let Some((bank, off)) = window_bank(addr) else {
            return 0;
        };
        if !self.bank_enabled(bank) {
            return 0;
        }
        let base = off as usize;
        let end = base + size as usize;
        if end > FSMC_BANK_SIZE {
            return 0;
        }
        let b = &self.banks[bank];
        let mut v = 0u32;
        for (i, byte) in b[base..end].iter().enumerate() {
            v |= (*byte as u32) << (8 * i);
        }
        v
    }

    /// 窗口写：MBKEN=1 且窗口内 → 写入后备缓冲；否则丢弃。
    pub fn window_write(&mut self, addr: u32, size: u32, value: u32) {
        let Some((bank, off)) = window_bank(addr) else {
            return;
        };
        if !self.bank_enabled(bank) {
            return;
        }
        let base = off as usize;
        let end = base + size as usize;
        if end > FSMC_BANK_SIZE {
            return;
        }
        let b = &mut self.banks[bank];
        for i in 0..size as usize {
            b[base + i] = (value >> (8 * i)) as u8;
        }
    }
}

impl Peripheral for Fsmc {
    fn name(&self) -> &str {
        "FSMC"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let idx = match offset {
            OFF_BCR1 | OFF_BCR2 | OFF_BCR3 | OFF_BCR4 => bcr_idx(offset),
            OFF_BTR1 | OFF_BTR2 | OFF_BTR3 | OFF_BTR4 => btr_idx(offset),
            OFF_BWTR1 | OFF_BWTR2 | OFF_BWTR3 | OFF_BWTR4 => bwtr_idx(offset),
            _ => return Err(BusError::OutOfRange),
        };
        Ok(self.regs[idx])
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let idx = match offset {
            OFF_BCR1 | OFF_BCR2 | OFF_BCR3 | OFF_BCR4 => bcr_idx(offset),
            OFF_BTR1 | OFF_BTR2 | OFF_BTR3 | OFF_BTR4 => btr_idx(offset),
            OFF_BWTR1 | OFF_BWTR2 | OFF_BWTR3 | OFF_BWTR4 => bwtr_idx(offset),
            _ => return Err(BusError::OutOfRange),
        };
        // BCR 仅保留可写位；BTR/BWTR 时序位按写值存储
        let masked = if idx < 4 { value & BCR_WMASK } else { value };
        self.regs[idx] = masked;
        Ok(())
    }

    fn reset(&mut self) {
        self.regs = [0; 12];
        for b in &mut self.banks {
            b.fill(0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make() -> Fsmc {
        Fsmc::new()
    }

    #[test]
    fn bank_requires_mbken_for_window_access() {
        let mut f = make();
        // 未使能：写被丢弃、读恒 0
        f.window_write(FSMC_BANK1_BASE, 4, 0xCAFE_BABE);
        assert_eq!(f.window_read(FSMC_BANK1_BASE, 4), 0, "未使能 Bank1 应读 0");
        // 使能后：读回写值（Bank 映射命中）
        f.write(OFF_BCR1, 4, BCR_MBKEN).unwrap();
        f.window_write(FSMC_BANK1_BASE, 4, 0xCAFE_BABE);
        assert_eq!(f.window_read(FSMC_BANK1_BASE, 4), 0xCAFE_BABE);
    }

    #[test]
    fn each_bank_independent() {
        let mut f = make();
        // 仅使能 Bank2
        f.write(OFF_BCR2, 4, BCR_MBKEN).unwrap();
        f.window_write(FSMC_BANK2_BASE + 8, 4, 0x1122_3344);
        assert_eq!(f.window_read(FSMC_BANK2_BASE + 8, 4), 0x1122_3344);
        // Bank1/3/4 未使能：读写无效
        f.window_write(FSMC_BANK1_BASE, 4, 0xFFFF_FFFF);
        assert_eq!(f.window_read(FSMC_BANK1_BASE, 4), 0);
        f.window_write(FSMC_BANK3_BASE, 4, 0xFFFF_FFFF);
        assert_eq!(f.window_read(FSMC_BANK3_BASE, 4), 0);
        f.window_write(FSMC_BANK4_BASE, 4, 0xFFFF_FFFF);
        assert_eq!(f.window_read(FSMC_BANK4_BASE, 4), 0);
    }

    #[test]
    fn window_supports_widths_and_byte_layout() {
        let mut f = make();
        f.write(OFF_BCR1, 4, BCR_MBKEN).unwrap();
        // 8/16/32 位访问同窗口不同偏移
        f.window_write(FSMC_BANK1_BASE + 0, 1, 0xAB);
        assert_eq!(f.window_read(FSMC_BANK1_BASE + 0, 1), 0xAB);
        f.window_write(FSMC_BANK1_BASE + 4, 2, 0xCDEF);
        assert_eq!(f.window_read(FSMC_BANK1_BASE + 4, 2), 0xCDEF);
        f.window_write(FSMC_BANK1_BASE + 8, 4, 0x0123_4567);
        assert_eq!(f.window_read(FSMC_BANK1_BASE + 8, 4), 0x0123_4567);
        // 字节为低字节在前
        assert_eq!(f.window_read(FSMC_BANK1_BASE + 8, 1), 0x67);
    }

    #[test]
    fn window_out_of_range_ignored() {
        let mut f = make();
        f.write(OFF_BCR1, 4, BCR_MBKEN).unwrap();
        // 超出窗口末尾（64KB）访问被忽略
        f.window_write(FSMC_BANK1_BASE + FSMC_BANK_SIZE as u32, 4, 0xDEAD_BEEF);
        assert_eq!(
            f.window_read(FSMC_BANK1_BASE + FSMC_BANK_SIZE as u32, 4),
            0,
            "窗口外地址应忽略"
        );
    }

    #[test]
    fn registers_roundtrip_and_mask() {
        let mut f = make();
        // BCR1：MBKEN 写 1 读回 1；保留位被掩码清 0
        f.write(OFF_BCR1, 4, 0xFFFF_FFFF).unwrap();
        assert_eq!(f.read(OFF_BCR1, 4).unwrap(), BCR_WMASK);
        // BTR1/BWTR1 全位可回读
        f.write(OFF_BTR1, 4, 0x1234_5678).unwrap();
        assert_eq!(f.read(OFF_BTR1, 4).unwrap(), 0x1234_5678);
        f.write(OFF_BWTR1, 4, 0x8765_4321).unwrap();
        assert_eq!(f.read(OFF_BWTR1, 4).unwrap(), 0x8765_4321);
        // 各 bank BCR/BTR 独立
        f.write(OFF_BCR4, 4, BCR_MBKEN).unwrap();
        f.write(OFF_BTR4, 4, 0x0F0F_0F0F).unwrap();
        assert_eq!(f.read(OFF_BCR4, 4).unwrap(), BCR_MBKEN);
        assert_eq!(f.read(OFF_BTR4, 4).unwrap(), 0x0F0F_0F0F);
    }

    #[test]
    fn reset_clears_regs_and_windows() {
        let mut f = make();
        f.write(OFF_BCR1, 4, BCR_MBKEN).unwrap();
        f.window_write(FSMC_BANK1_BASE, 4, 0xCAFE_BABE);
        f.write(OFF_BTR1, 4, 0x1111_1111).unwrap();
        f.reset();
        assert_eq!(f.read(OFF_BCR1, 4).unwrap(), 0);
        assert_eq!(f.read(OFF_BTR1, 4).unwrap(), 0);
        assert_eq!(f.window_read(FSMC_BANK1_BASE, 4), 0, "复位应清空窗口缓冲");
    }
}
