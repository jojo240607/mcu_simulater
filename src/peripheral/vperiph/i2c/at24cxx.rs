//! AT24Cxx EEPROM I2C 从设备（保存用途；AT24C256：32KB，16 位地址指针）。
//!
//! # I2C 语义（16 位地址指针，高字节在前）
//!
//! 写：START(W) → addr_hi → addr_lo → data...（指针递增）；字节写/页写皆可。
//! 随机读：START(W) 写 16 位地址设指针 → START(R) 从指针连续读（递增）。
//! 当前地址读：直接 START(R) 从当前指针读。
//!
//! # 持久化
//!
//! [`At24cxx::with_file`] 加载文件映像；[`At24cxx::persist`] 写回（与 SPI NOR
//! Flash 同一保存用途语义：掉电不丢失 → 模拟器文件持久化）。
//!
//! # 数据源 / 联动预留
//!
//! EEPROM 存储用户参数/校准数据（飞控 PID、磁偏角等）；预留独立存储模型
//! 接口，将来可驱动"固件写入 → 重启后读回"的掉电保持闭环。

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};

use super::super::{I2cDir, VirtualI2cSlave};

/// I2C 7 位地址（A2A1A0=000）
pub const AT24CXX_ADDR7: u8 = 0x50;
/// 容量（AT24C256 = 32KB）
pub const EEPROM_SIZE: usize = 32 * 1024;
/// 页大小（AT24C256 64B；仿真简化：不强制回卷，仅作文档）
pub const PAGE_SIZE: usize = 64;

/// I2C 状态机
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// 写事务：等地址高字节
    ExpectAddrHi,
    /// 写事务：等地址低字节（设指针）
    ExpectAddrLo,
    /// 写事务：数据写入指针并递增
    WriteData,
    /// 读事务：从指针连续读并递增
    ReadStream,
}

/// AT24Cxx EEPROM 从设备
pub struct At24cxx {
    /// 存储映像
    mem: Vec<u8>,
    /// 当前 16 位地址指针
    ptr: u16,
    /// I2C 状态机
    state: State,
    /// 持久化文件（None = 仅内存）
    file: Option<std::path::PathBuf>,
    /// 脏标记（有写未持久化）
    dirty: AtomicBool,
    /// 寄存器写次数（观测）
    pub n_writes: u64,
    /// 寄存器读次数（观测）
    pub n_reads: u64,
}

impl At24cxx {
    pub fn new() -> Self {
        Self {
            mem: vec![0xFF; EEPROM_SIZE],
            ptr: 0,
            state: State::ExpectAddrHi,
            file: None,
            dirty: AtomicBool::new(false),
            n_writes: 0,
            n_reads: 0,
        }
    }

    /// 以文件为后备：加载映像（不存在则全 0xFF），[`persist`] 写回。
    pub fn with_file<P: AsRef<Path>>(path: P) -> Self {
        let mut s = Self::new();
        if let Ok(data) = std::fs::read(path.as_ref()) {
            let n = data.len().min(EEPROM_SIZE);
            s.mem[..n].copy_from_slice(&data[..n]);
            s.file = Some(path.as_ref().to_path_buf());
        } else {
            s.file = Some(path.as_ref().to_path_buf());
        }
        s
    }

    /// 存储映像字节数（观测）
    pub fn len(&self) -> usize {
        self.mem.len()
    }

    /// 读存储字节（观测辅助）
    pub fn peek(&self, addr: u16) -> u8 {
        self.mem.get(addr as usize).copied().unwrap_or(0xFF)
    }

    /// 直接写存储字节（模型初始化/测试）
    pub fn poke(&mut self, addr: u16, value: u8) {
        if let Some(b) = self.mem.get_mut(addr as usize) {
            *b = value;
            self.dirty.store(true, Ordering::Relaxed);
        }
    }

    /// 写回持久化文件（保存用途：掉电保持）。
    pub fn persist(&mut self) {
        if let Some(p) = &self.file {
            let _ = std::fs::write(p, &self.mem);
        }
        self.dirty.store(false, Ordering::Relaxed);
    }

    /// 是否脏（有未持久化的写）
    pub fn is_dirty(&self) -> bool {
        self.dirty.load(Ordering::Relaxed)
    }
}

impl Default for At24cxx {
    fn default() -> Self {
        Self::new()
    }
}

impl VirtualI2cSlave for At24cxx {
    fn name(&self) -> &str {
        "at24cxx"
    }

    fn addr7(&self) -> u8 {
        AT24CXX_ADDR7
    }

    fn on_start(&mut self, dir: I2cDir) {
        self.state = match dir {
            I2cDir::Read => State::ReadStream,
            I2cDir::Write => State::ExpectAddrHi,
        };
    }

    fn on_write(&mut self, byte: u8) {
        match self.state {
            State::ExpectAddrHi => {
                self.ptr = (byte as u16) << 8;
                self.state = State::ExpectAddrLo;
            }
            State::ExpectAddrLo => {
                self.ptr |= byte as u16;
                self.state = State::WriteData;
            }
            State::WriteData => {
                self.n_writes += 1;
                if let Some(b) = self.mem.get_mut(self.ptr as usize) {
                    *b = byte;
                    self.dirty.store(true, Ordering::Relaxed);
                }
                self.ptr = self.ptr.wrapping_add(1);
            }
            State::ReadStream => {
                self.n_writes += 1; // 读事务中异常写：忽略数据
            }
        }
    }

    fn on_read(&mut self) -> Option<u8> {
        match self.state {
            State::ReadStream => {
                self.n_reads += 1;
                let v = self.mem.get(self.ptr as usize).copied().unwrap_or(0xFF);
                self.ptr = self.ptr.wrapping_add(1);
                Some(v)
            }
            _ => {
                self.n_reads += 1;
                Some(0xFF) // 未设指针的读（真实器件当前地址读；简化回 0xFF）
            }
        }
    }

    fn read_count(&self) -> u64 {
        self.n_reads
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set_ptr(sl: &mut At24cxx, addr: u16) {
        sl.on_start(I2cDir::Write);
        sl.on_write((addr >> 8) as u8);
        sl.on_write((addr & 0xFF) as u8);
    }

    fn read_n(sl: &mut At24cxx, n: usize) -> Vec<u8> {
        sl.on_start(I2cDir::Read);
        (0..n).filter_map(|_| sl.on_read()).collect()
    }

    #[test]
    fn addr7_and_default_ff() {
        let mut e = At24cxx::new();
        assert_eq!(e.addr7(), AT24CXX_ADDR7);
        assert_eq!(e.len(), EEPROM_SIZE);
        assert_eq!(e.peek(0), 0xFF, "出厂全 0xFF");
    }

    #[test]
    fn byte_write_and_random_read() {
        let mut e = At24cxx::new();
        // 字节写：0x1000 处写 0x5A
        e.on_start(I2cDir::Write);
        e.on_write(0x10);
        e.on_write(0x00);
        e.on_write(0x5A);
        assert_eq!(e.peek(0x1000), 0x5A);
        assert_eq!(e.n_writes, 1);
        // 随机读
        set_ptr(&mut e, 0x1000);
        let r = read_n(&mut e, 2);
        assert_eq!(r, vec![0x5A, 0xFF], "指针递增连续读");
        assert_eq!(e.n_reads, 2);
    }

    #[test]
    fn page_write_sequential() {
        let mut e = At24cxx::new();
        e.on_start(I2cDir::Write);
        e.on_write(0x00);
        e.on_write(0x00);
        for i in 0u8..4 {
            e.on_write(0x40 + i); // 0x40..0x43
        }
        assert_eq!(e.peek(0), 0x40);
        assert_eq!(e.peek(1), 0x41);
        assert_eq!(e.peek(2), 0x42);
        assert_eq!(e.peek(3), 0x43);
    }

    #[test]
    fn file_persist_roundtrip() {
        let dir = std::env::temp_dir();
        let path = dir.join("dsh_at24cxx_test.bin");
        let _ = std::fs::remove_file(&path);
        {
            let mut e = At24cxx::with_file(&path);
            assert_eq!(e.peek(0), 0xFF, "新文件全 0xFF");
            e.poke(0x1000, 0x42);
            e.persist();
            assert!(!e.is_dirty());
        }
        {
            let e = At24cxx::with_file(&path);
            assert_eq!(e.peek(0x1000), 0x42, "重新加载映像");
        }
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn dirty_tracking() {
        let mut e = At24cxx::new();
        assert!(!e.is_dirty());
        set_ptr(&mut e, 0x0000);
        e.on_start(I2cDir::Write);
        e.on_write(0x00);
        e.on_write(0x00);
        e.on_write(0x01);
        assert!(e.is_dirty(), "写后脏");
    }
}
