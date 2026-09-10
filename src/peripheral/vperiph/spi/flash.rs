//! SPI NOR Flash 虚拟从设备（W25Q 类，用于保存/存储场景）。
//!
//! # 器件语义（固件驱动只依赖这些）
//!
//! 帧协议为**命令流**（与 BMI088 的寄存器流不同）：CS 拉低 → 首字节为命令码
//! → 按命令消费后续字节（多为 3 字节地址 + 数据）。命令集（W25Q128 常用子集）：
//!
//! | 命令 | 码   | 格式                              | 语义                                  |
//! |------|------|-----------------------------------|---------------------------------------|
//! | WREN | 0x06 | 无                                | 置写使能锁存 WEL=1（写/擦除前置）      |
//! | WRDI | 0x04 | 无                                | 清 WEL                                |
//! | RDSR | 0x05 | 回 SR1                            | 状态寄存器 1（bit1=WEL）              |
//! | RDSR2| 0x35 | 回 SR2                            | 状态寄存器 2（恒 0）                   |
//! | RDSR3| 0x15 | 回 SR3                            | 状态寄存器 3（恒 0）                   |
//! | WRSR | 0x01 | 1~2 数据字节                      | 写状态寄存器（需要 WEL）               |
//! | READ | 0x03 | 3B 地址 + 连续数据                | 读数据（地址自增，整片回绕）           |
//! | FREAD| 0x0B | 3B 地址 + 1 dummy + 连续数据      | 快速读                                |
//! | PP   | 0x02 | 3B 地址 + 数据（需 WEL）          | 页编程（256B 页内回绕）               |
//! | SE   | 0x20 | 3B 地址（需 WEL）                 | 扇区擦除 4KB→0xFF                     |
//! | BE32 | 0x52 | 3B 地址（需 WEL）                 | 32KB 块擦除                           |
//! | BE64 | 0xD8 | 3B 地址（需 WEL）                 | 64KB 块擦除                           |
//! | CE   | 0xC7/0x60 | 无（需 WEL）                 | 全片擦除                              |
//! | JEDEC| 0x9F | 回 3B JEDEC ID                    | 厂商 0xEF + 容量码 + 型号码           |
//! | DEVID| 0x90 | 3 dummy + 回 2B（0xEF 型号码）    | 器件 ID                               |
//!
//! 语义简化（真机为电擦写、以 CS 上升沿提交）：
//! - 写/擦除即时生效（BUSY 恒 0，固件轮询 RDSR 立即通过）；
//! - 页编程逐字节落盘但 **WEL 保持到 CS 拉高才清**（真机：PP 在 CS 上升沿
//!   提交并清 WEL）；擦除在地址收齐后立即执行并清 WEL；
//! - 无 WEL 的写/擦除命令静默忽略（数据不落盘）——真机同样忽略。
//!
//! # 持久化（保存用途）
//!
//! 可选绑定一个宿主机文件：构造时若文件存在则载入（不足补 0xFF、超出截断），
//! `save()` 把整片映像写回文件（可经 [`VirtualSpiSlave::persist`] 由 Machine
//! 统一触发）。固件写入的数据因此可跨模拟器 run 存活，模拟真实掉电保存。
//!
//! 默认容量 16MB（W25Q128，JEDEC ID `EF 40 18`）；`new` 可按字节数定制，
//! 容量不必是 2 的幂（地址自增按 `addr % size` 回绕）。

use std::path::{Path, PathBuf};

use super::VirtualSpiSlave;

/// W25Q128 默认容量（16MB）
pub const DEFAULT_SIZE: usize = 16 * 1024 * 1024;
/// 页大小（页编程回绕边界）
pub const PAGE_SIZE: usize = 256;
/// 扇区大小（扇区擦除粒度）
pub const SECTOR_SIZE: usize = 4096;
/// W25Q128 JEDEC ID：厂商 0xEF（Winbond）、容量码 0x40、型号码 0x18
pub const W25Q128_JEDEC: [u8; 3] = [0xEF, 0x40, 0x18];

/// 命令码
const CMD_WREN: u8 = 0x06;
const CMD_WRDI: u8 = 0x04;
const CMD_RDSR: u8 = 0x05;
const CMD_WRSR: u8 = 0x01;
const CMD_RDSR2: u8 = 0x35;
const CMD_RDSR3: u8 = 0x15;
const CMD_READ: u8 = 0x03;
const CMD_FAST_READ: u8 = 0x0B;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_SECTOR_ERASE: u8 = 0x20;
const CMD_BLOCK_ERASE_32K: u8 = 0x52;
const CMD_BLOCK_ERASE_64K: u8 = 0xD8;
const CMD_CHIP_ERASE: u8 = 0xC7;
const CMD_CHIP_ERASE_ALT: u8 = 0x60;
const CMD_JEDEC_ID: u8 = 0x9F;
const CMD_DEVICE_ID: u8 = 0x90;

/// SR1 位：bit1 = 写使能锁存 WEL
const SR1_WEL: u8 = 1 << 1;
/// SR1 位：bit0 = BUSY（模拟即时完成，恒 0）
const SR1_BUSY: u8 = 1 << 0;

/// 带 3 字节地址的命令类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AddrCmd {
    Read,
    FastRead,
    PageProgram,
    SectorErase,
    BlockErase32K,
    BlockErase64K,
}

/// 帧协议状态机（CS 拉低开始，拉高结束）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Frame {
    /// 等待命令字节
    ExpectCmd,
    /// RDSR/RDSR2/RDSR3：持续回状态寄存器 `idx`（0/1/2）
    ReadStatus { idx: usize },
    /// 0x01 WRSR：收集 1~2 个数据字节（n 已收字节数）
    WriteStatus { n: u8 },
    /// 收集 3 字节地址（n 已收字节数）
    ReadAddr { cmd: AddrCmd, addr: [u8; 3], n: u8 },
    /// FAST_READ 的 1 个 dummy 字节
    ReadAddrDummy { addr: u32 },
    /// 连续读（addr 自增，整片回绕）
    StreamRead { addr: u32 },
    /// 页编程（addr 自增，页内回绕）
    StreamProgram { addr: u32 },
    /// JEDEC ID 输出流（n 已输出字节数；超出回 0xFF）
    JedecId { n: u8 },
    /// 器件 ID 的 3 个 dummy 字节（n 已收）
    DevIdDummy { n: u8 },
    /// 器件 ID 输出流（n 已输出字节数）
    DevId { n: u8 },
}

/// W25Q 类 SPI NOR Flash 虚拟从设备。
pub struct SpiFlash {
    /// 片选 GPIO 坐标（port, pin）
    cs: (u8, u8),
    /// 是否被选中（CS 拉低）
    selected: bool,
    /// 帧状态机
    frame: Frame,
    /// 状态寄存器（SR1/SR2/SR3；SR1.bit1=WEL）
    status: [u8; 3],
    /// 本帧是否发生过页编程字节写入（CS 拉高时据此清 WEL）
    programmed: bool,
    /// 映像（初始化全 0xFF；容量即长度）
    mem: Vec<u8>,
    /// JEDEC ID（3 字节）
    jedec: [u8; 3],
    /// 器件 ID（0x90 回：厂商 + 型号码）
    device_id: [u8; 2],
    /// 持久化文件（None = 仅内存）
    path: Option<PathBuf>,
    /// 总访问字节数（观测）
    access: u64,
    /// 页编程字节数（观测）
    pub n_program: u64,
    /// 擦除次数（观测）
    pub n_erase: u64,
}

impl SpiFlash {
    /// 新建纯内存 Flash（容量 `size` 字节，JEDEC = W25Q128）。
    pub fn new(size: usize, cs: (u8, u8)) -> Self {
        Self::with_parts(size, cs, W25Q128_JEDEC, [0xEF, 0x18], None)
    }

    /// 新建并绑定持久化文件（文件存在则载入，不足补 0xFF、超出截断）。
    pub fn with_file(size: usize, cs: (u8, u8), path: impl Into<PathBuf>) -> Self {
        let path = path.into();
        let mut s = Self::with_parts(size, cs, W25Q128_JEDEC, [0xEF, 0x18], Some(path.clone()));
        if let Ok(img) = std::fs::read(&path) {
            s.mem.fill(0xFF);
            let n = img.len().min(size);
            s.mem[..n].copy_from_slice(&img[..n]);
        }
        s
    }

    /// 完整构造（容量/ID/持久化路径全定制）。
    pub fn with_parts(size: usize, cs: (u8, u8), jedec: [u8; 3], device_id: [u8; 2], path: Option<PathBuf>) -> Self {
        Self {
            cs,
            selected: false,
            frame: Frame::ExpectCmd,
            status: [0, 0, 0],
            programmed: false,
            mem: vec![0xFF; size.max(1)],
            jedec,
            device_id,
            path,
            access: 0,
            n_program: 0,
            n_erase: 0,
        }
    }

    /// 写使能锁存（SR1.bit1）
    fn wel(&self) -> bool {
        self.status[0] & SR1_WEL != 0
    }

    fn set_wel(&mut self, v: bool) {
        if v {
            self.status[0] |= SR1_WEL;
        } else {
            self.status[0] &= !SR1_WEL;
        }
    }

    /// 3 字节地址 → u32（大端）
    fn u24(a: &[u8; 3]) -> u32 {
        ((a[0] as u32) << 16) | ((a[1] as u32) << 8) | a[2] as u32
    }

    /// 读字节（地址按容量回绕）
    fn read_byte_at(&self, addr: u32) -> u8 {
        if self.mem.is_empty() {
            return 0xFF;
        }
        self.mem[(addr as usize) % self.mem.len()]
    }

    /// 页编程写入一字节（调用方已带数据）
    fn program_byte(&mut self, addr: u32, byte: u8) {
        if !self.wel() || self.mem.is_empty() {
            return;
        }
        let base = (addr as usize / PAGE_SIZE) * PAGE_SIZE;
        let off = addr as usize % PAGE_SIZE;
        let idx = (base + off) % self.mem.len();
        self.mem[idx] = byte;
        self.n_program += 1;
        self.programmed = true;
    }

    /// 擦除（需 WEL；`aligned` 为真时地址按对齐掩码取整）
    fn erase(&mut self, addr: u32, align_mask: usize) {
        if !self.wel() || self.mem.is_empty() {
            return;
        }
        let base = (addr as usize) & !align_mask;
        let end = (base + align_mask + 1).min(self.mem.len());
        self.mem[base..end].fill(0xFF);
        self.n_erase += 1;
        self.set_wel(false); // 真机：擦除后 WEL 清
    }

    /// 把映像写回持久化文件（无 path 时 no-op）。
    pub fn save(&mut self) -> std::io::Result<()> {
        match &self.path {
            Some(p) => std::fs::write(p, &self.mem),
            None => Ok(()),
        }
    }

    /// 从文件重载映像（无 path 时 no-op）。
    pub fn reload(&mut self) -> std::io::Result<()> {
        match &self.path {
            Some(p) => {
                let img = std::fs::read(p)?;
                self.mem.fill(0xFF);
                let n = img.len().min(self.mem.len());
                self.mem[..n].copy_from_slice(&img[..n]);
                Ok(())
            }
            None => Ok(()),
        }
    }

    /// 映像只读访问（观测/断言）
    pub fn image(&self) -> &[u8] {
        &self.mem
    }

    /// 映像可变访问（模型预置/测试）
    pub fn image_mut(&mut self) -> &mut [u8] {
        &mut self.mem
    }

    /// 状态寄存器回读（观测）
    pub fn status_register(&self) -> [u8; 3] {
        self.status
    }

    /// 命令分发：按命令码设置帧状态机（命令字节的 MISO 无意义，回 0xFF）。
    fn dispatch(&mut self, cmd: u8) -> u8 {
        match cmd {
            CMD_WREN => {
                self.set_wel(true);
                self.frame = Frame::ExpectCmd;
            }
            CMD_WRDI => {
                self.set_wel(false);
                self.frame = Frame::ExpectCmd;
            }
            CMD_RDSR => self.frame = Frame::ReadStatus { idx: 0 },
            CMD_RDSR2 => self.frame = Frame::ReadStatus { idx: 1 },
            CMD_RDSR3 => self.frame = Frame::ReadStatus { idx: 2 },
            CMD_WRSR => self.frame = Frame::WriteStatus { n: 0 },
            CMD_READ => self.frame = Frame::ReadAddr { cmd: AddrCmd::Read, addr: [0; 3], n: 0 },
            CMD_FAST_READ => self.frame = Frame::ReadAddr { cmd: AddrCmd::FastRead, addr: [0; 3], n: 0 },
            CMD_PAGE_PROGRAM => self.frame = Frame::ReadAddr { cmd: AddrCmd::PageProgram, addr: [0; 3], n: 0 },
            CMD_SECTOR_ERASE => self.frame = Frame::ReadAddr { cmd: AddrCmd::SectorErase, addr: [0; 3], n: 0 },
            CMD_BLOCK_ERASE_32K => self.frame = Frame::ReadAddr { cmd: AddrCmd::BlockErase32K, addr: [0; 3], n: 0 },
            CMD_BLOCK_ERASE_64K => self.frame = Frame::ReadAddr { cmd: AddrCmd::BlockErase64K, addr: [0; 3], n: 0 },
            CMD_CHIP_ERASE | CMD_CHIP_ERASE_ALT => {
                if self.wel() {
                    self.mem.fill(0xFF);
                    self.n_erase += 1;
                    self.set_wel(false);
                }
                self.frame = Frame::ExpectCmd;
            }
            CMD_JEDEC_ID => self.frame = Frame::JedecId { n: 0 },
            CMD_DEVICE_ID => self.frame = Frame::DevIdDummy { n: 0 },
            _ => {
                // 未实现命令：忽略整帧（后续字节按 ExpectCmd 消费，等效丢弃）
                self.frame = Frame::ExpectCmd;
            }
        }
        0xFF // 命令字节的 MISO 无意义
    }
}

impl VirtualSpiSlave for SpiFlash {
    fn name(&self) -> &str {
        "spi_flash"
    }

    fn selected(&self) -> bool {
        self.selected
    }

    fn on_cs(&mut self, port: u8, pin: u8, level: bool) {
        if level {
            // 拉高：帧结束。页编程在 CS 上升沿提交并清 WEL（真机语义）。
            if self.selected {
                if self.programmed {
                    self.set_wel(false);
                    self.programmed = false;
                }
                self.selected = false;
                self.frame = Frame::ExpectCmd;
            }
            return;
        }
        if (port, pin) == self.cs {
            self.selected = true;
            self.frame = Frame::ExpectCmd;
        }
    }

    fn on_byte(&mut self, byte: u8) -> u8 {
        if !self.selected {
            return 0xFF; // 未选中：MISO 默认高
        }
        self.access += 1;
        match self.frame {
            Frame::ExpectCmd => self.dispatch(byte),
            Frame::ReadStatus { idx } => {
                // 持续回同一状态寄存器字节（真机 CS 低期间持续输出）
                self.status[idx]
            }
            Frame::WriteStatus { n } => {
                // 首字节写 SR1（保留 WEL 位不变，模拟忽略大部分控制位），
                // 次字节写 SR2；收满 2 字节后回到 ExpectCmd 并清 WEL。
                if n == 0 {
                    self.status[0] = (self.status[0] & SR1_WEL) | (byte & !SR1_WEL);
                } else {
                    self.status[1] = byte;
                }
                let n = n + 1;
                self.frame = if n >= 2 {
                    self.set_wel(false);
                    Frame::ExpectCmd
                } else {
                    Frame::WriteStatus { n }
                };
                0xFF
            }
            Frame::ReadAddr { cmd, mut addr, n } => {
                addr[n as usize] = byte;
                let n = n + 1;
                if n < 3 {
                    self.frame = Frame::ReadAddr { cmd, addr, n };
                    return 0xFF;
                }
                let a = Self::u24(&addr);
                match cmd {
                    AddrCmd::Read => {
                        self.frame = Frame::StreamRead { addr: a };
                    }
                    AddrCmd::FastRead => {
                        self.frame = Frame::ReadAddrDummy { addr: a };
                    }
                    AddrCmd::PageProgram => {
                        self.frame = Frame::StreamProgram { addr: a };
                    }
                    AddrCmd::SectorErase => {
                        self.erase(a, SECTOR_SIZE - 1);
                        self.frame = Frame::ExpectCmd;
                    }
                    AddrCmd::BlockErase32K => {
                        self.erase(a, 32 * 1024 - 1);
                        self.frame = Frame::ExpectCmd;
                    }
                    AddrCmd::BlockErase64K => {
                        self.erase(a, 64 * 1024 - 1);
                        self.frame = Frame::ExpectCmd;
                    }
                }
                0xFF // 第 3 个地址字节的 MISO 无意义
            }
            Frame::ReadAddrDummy { addr } => {
                self.frame = Frame::StreamRead { addr };
                0xFF // dummy 字节
            }
            Frame::StreamRead { addr } => {
                let v = self.read_byte_at(addr);
                let next = if self.mem.is_empty() {
                    addr
                } else {
                    (addr + 1) % self.mem.len() as u32
                };
                self.frame = Frame::StreamRead { addr: next };
                v
            }
            Frame::StreamProgram { addr } => {
                self.program_byte(addr, byte);
                let next = if self.mem.is_empty() {
                    addr
                } else {
                    // 页内回绕：addr 落在页内偏移，越界回页首（不跨页）
                    let base = (addr as usize / PAGE_SIZE) * PAGE_SIZE;
                    let off = (addr as usize % PAGE_SIZE) + 1;
                    ((base + off) % self.mem.len()) as u32
                };
                self.frame = Frame::StreamProgram { addr: next };
                0xFF
            }
            Frame::JedecId { n } => {
                let v = if (n as usize) < self.jedec.len() {
                    self.jedec[n as usize]
                } else {
                    0xFF // 超出 JEDEC 流：MISO 拉高
                };
                self.frame = Frame::JedecId { n: n.wrapping_add(1) };
                v
            }
            Frame::DevIdDummy { n } => {
                let n = n + 1;
                self.frame = if n >= 3 {
                    Frame::DevId { n: 0 }
                } else {
                    Frame::DevIdDummy { n }
                };
                0xFF
            }
            Frame::DevId { n } => {
                let v = if (n as usize) < self.device_id.len() {
                    self.device_id[n as usize]
                } else {
                    0xFF
                };
                self.frame = Frame::DevId { n: n.wrapping_add(1) };
                v
            }
        }
    }

    fn access_count(&self) -> u64 {
        self.access
    }

    /// 持久化：把当前映像写回绑定文件（Machine 统一触发；无文件 no-op）。
    fn persist(&mut self) {
        let _ = self.save();
    }
}

impl Default for SpiFlash {
    fn default() -> Self {
        Self::new(DEFAULT_SIZE, (4, 9))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::vperiph::spi::VirtualSpiSlave;

    /// 默认 CS：(4,9) = GPIOE9
    const CS: (u8, u8) = (4, 9);

    /// 发送完整帧：CS 低 → 字节序列 → CS 高；收集每字节的 MISO 回送。
    fn frame(s: &mut dyn VirtualSpiSlave, bytes: &[u8]) -> Vec<u8> {
        s.on_cs(CS.0, CS.1, false);
        let out: Vec<u8> = bytes.iter().map(|&b| s.on_byte(b)).collect();
        s.on_cs(CS.0, CS.1, true);
        out
    }

    /// 读命令：READ(0x03) + 3B 地址 + len 个数据字节
    fn flash_read(s: &mut dyn VirtualSpiSlave, addr: u32, len: usize) -> Vec<u8> {
        let a = addr.to_be_bytes();
        let mut frame_bytes = vec![CMD_READ, a[1], a[2], a[3]];
        frame_bytes.extend(std::iter::repeat(0xFF).take(len));
        let out = frame(s, &frame_bytes);
        out[4..].to_vec()
    }

    /// 页编程：WREN 帧 + PP(0x02) + 3B 地址 + 数据
    fn flash_program(s: &mut dyn VirtualSpiSlave, addr: u32, data: &[u8]) {
        frame(s, &[CMD_WREN]);
        let a = addr.to_be_bytes();
        let mut f = vec![CMD_PAGE_PROGRAM, a[1], a[2], a[3]];
        f.extend_from_slice(data);
        frame(s, &f);
    }

    #[test]
    fn jedec_id() {
        let mut s = SpiFlash::default();
        let out = frame(&mut s, &[CMD_JEDEC_ID, 0xFF, 0xFF, 0xFF, 0xFF]);
        assert_eq!(&out[1..4], &W25Q128_JEDEC, "JEDEC=EF 40 18（out[0] 为命令字节 MISO）");
        assert_eq!(out[4], 0xFF, "超出 JEDEC 流回 0xFF");
    }

    #[test]
    fn fresh_memory_all_0xff() {
        let mut s = SpiFlash::new(4096, CS);
        let r = flash_read(&mut s, 0x0000, 8);
        assert_eq!(&r, &[0xFF; 8], "新片全 0xFF");
    }

    #[test]
    fn program_requires_wren() {
        let mut s = SpiFlash::new(4096, CS);
        // 无 WREN：PP 被忽略
        let a = 0x0010u32.to_be_bytes();
        frame(&mut s, &[CMD_PAGE_PROGRAM, a[1], a[2], a[3], 0xAA]);
        let r = flash_read(&mut s, 0x0010, 1);
        assert_eq!(r[0], 0xFF, "无 WEL 的页编程被忽略");
        // WREN 后成功
        flash_program(&mut s, 0x0010, &[0xAA]);
        let r = flash_read(&mut s, 0x0010, 1);
        assert_eq!(r[0], 0xAA, "WREN 后页编程生效");
    }

    #[test]
    fn program_read_back_and_wel_cleared() {
        let mut s = SpiFlash::new(4096, CS);
        flash_program(&mut s, 0x0020, &[0xDE, 0xAD, 0xBE, 0xEF]);
        let r = flash_read(&mut s, 0x0020, 4);
        assert_eq!(&r, &[0xDE, 0xAD, 0xBE, 0xEF], "写读回一致");
        // 帧结束后 WEL 已清（PP 提交清 WEL）
        let st = frame(&mut s, &[CMD_RDSR, 0xFF]);
        assert_eq!(st[1] & SR1_WEL, 0, "CS 上升沿后 WEL 清");
    }

    #[test]
    fn page_program_wraps_within_page() {
        let mut s = SpiFlash::new(4096, CS);
        // 从页末 0x00FF 写 2 字节：回绕到 0x0000（同页首）
        flash_program(&mut s, 0x00FF, &[0x11, 0x22]);
        let r = flash_read(&mut s, 0x00FE, 4);
        assert_eq!(r[0], 0xFF, "0x00FE 未写");
        assert_eq!(r[1], 0x11, "0x00FF 写入");
        assert_eq!(r[2], 0x22, "回绕到页首 0x0000");
        assert_eq!(r[3], 0xFF, "0x0001 未写");
    }

    #[test]
    fn read_wraps_at_chip_size() {
        let mut s = SpiFlash::new(256, CS); // 1 页容量
        flash_program(&mut s, 0x00FE, &[0x11, 0x22]);
        // 读 0x00FE 起 4 字节：FE, FF, 00(回绕), 01
        let r = flash_read(&mut s, 0x00FE, 4);
        assert_eq!(&r, &[0x11, 0x22, 0xFF, 0xFF], "读地址整片回绕");
    }

    #[test]
    fn fast_read_skips_dummy() {
        let mut s = SpiFlash::new(4096, CS);
        flash_program(&mut s, 0x0030, &[0x5A]);
        let a = 0x0030u32.to_be_bytes();
        let out = frame(&mut s, &[CMD_FAST_READ, a[1], a[2], a[3], 0xFF, 0xFF, 0xFF]);
        assert_eq!(out[5], 0x5A, "FAST_READ 跳 1 dummy（out[4]）后读数据");
    }

    #[test]
    fn sector_erase_clears_4k() {
        let mut s = SpiFlash::new(16 * 1024, CS);
        flash_program(&mut s, 0x0100, &[0xAA, 0xBB]);
        flash_program(&mut s, 0x1000, &[0xCC]); // 扇区 1（0x1000-0x1FFF）
        // 擦除扇区 0（0x0000-0x0FFF）
        let a = 0x0100u32.to_be_bytes();
        frame(&mut s, &[CMD_WREN]);
        frame(&mut s, &[CMD_SECTOR_ERASE, a[1], a[2], a[3]]);
        let r = flash_read(&mut s, 0x0100, 2);
        assert_eq!(&r, &[0xFF, 0xFF], "扇区 0 擦除后全 FF");
        let r = flash_read(&mut s, 0x1000, 1);
        assert_eq!(r[0], 0xCC, "邻扇区（扇区 1）不受影响");
    }

    #[test]
    fn erase_requires_wren_and_clears_wel() {
        let mut s = SpiFlash::new(16 * 1024, CS);
        flash_program(&mut s, 0x0000, &[0x99]);
        // 无 WREN 直接擦除：忽略
        let a = 0x0000u32.to_be_bytes();
        frame(&mut s, &[CMD_SECTOR_ERASE, a[1], a[2], a[3]]);
        let r = flash_read(&mut s, 0x0000, 1);
        assert_eq!(r[0], 0x99, "无 WEL 擦除被忽略");
        // WREN + 擦除
        frame(&mut s, &[CMD_WREN]);
        frame(&mut s, &[CMD_SECTOR_ERASE, a[1], a[2], a[3]]);
        let st = frame(&mut s, &[CMD_RDSR, 0xFF]);
        assert_eq!(st[1] & SR1_WEL, 0, "擦除后 WEL 清");
        let r = flash_read(&mut s, 0x0000, 1);
        assert_eq!(r[0], 0xFF, "擦除生效");
    }

    #[test]
    fn chip_erase_clears_all() {
        let mut s = SpiFlash::new(16 * 1024, CS);
        flash_program(&mut s, 0x0000, &[0x11]);
        flash_program(&mut s, 0x3FFF, &[0x22]);
        frame(&mut s, &[CMD_WREN]);
        frame(&mut s, &[CMD_CHIP_ERASE]);
        let r = flash_read(&mut s, 0x0000, 1);
        assert_eq!(r[0], 0xFF);
        let r = flash_read(&mut s, 0x3FFF, 1);
        assert_eq!(r[0], 0xFF, "全片擦除");
    }

    #[test]
    fn device_id_90() {
        let mut s = SpiFlash::default();
        let out = frame(&mut s, &[CMD_DEVICE_ID, 0, 0, 0, 0xFF, 0xFF]);
        assert_eq!(out[4], 0xEF, "厂商码（3 dummy 在 out[1..4]）");
        assert_eq!(out[5], 0x18, "型号码");
    }

    #[test]
    fn unselected_byte_returns_0xff() {
        let mut s = SpiFlash::default();
        assert_eq!(s.on_byte(CMD_READ), 0xFF, "未选中回 0xFF");
    }

    #[test]
    fn persistence_save_reload_roundtrip() {
        let dir = std::env::temp_dir().join(format!("spi_flash_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("flash.bin");
        let _ = std::fs::remove_file(&path);
        // 写数据 + save
        let mut s = SpiFlash::with_file(4096, CS, path.clone());
        flash_program(&mut s, 0x0100, &[0xCA, 0xFE]);
        s.save().unwrap();
        drop(s);
        // 新实例从文件载入：数据存活（跨 run 持久化）
        let mut s2 = SpiFlash::with_file(4096, CS, path.clone());
        let r = flash_read(&mut s2, 0x0100, 2);
        assert_eq!(&r, &[0xCA, 0xFE], "持久化数据跨实例存活");
        // persist() trait 方法路径
        let mut s3 = SpiFlash::with_file(4096, CS, path.clone());
        flash_program(&mut s3, 0x0200, &[0x77]);
        VirtualSpiSlave::persist(&mut s3);
        let mut s4 = SpiFlash::with_file(4096, CS, path.clone());
        let r = flash_read(&mut s4, 0x0200, 1);
        assert_eq!(r[0], 0x77, "persist() 写回文件");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn access_count_tracks_bytes() {
        let mut s = SpiFlash::default();
        assert_eq!(s.access_count(), 0);
        frame(&mut s, &[CMD_JEDEC_ID, 0xFF, 0xFF]);
        assert_eq!(s.access_count(), 3);
    }
}
