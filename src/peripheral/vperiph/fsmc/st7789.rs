//! ST7789 LCD 控制器虚拟器件（FSMC 8080 并行接口，Bank1 NE1）。
//!
//! # 接口
//!
//! ST7789 挂在 FSMC Bank1 NE1（0x6000_0000，64KB 窗口）：A16 接 RS 线，
//! 窗口内偏移 bit15 区分命令（A16=0，0x6000_0000）与数据（A16=1，
//! 0x6000_8000）。命令区写低 8 位 = 命令字节；数据区写低 16 位 = 参数/
//! 16bpp 像素。
//!
//! # 命令集（固件 lcd_st7789 驱动用到）
//!
//! | 命令 | 名称 | 语义 |
//! |---|---|---|
//! | 0x01 | SWRESET | 软复位（清窗口/状态，显存保留） |
//! | 0x04 | RDDID | 读 ID：后续读回 0x85（ST7789） |
//! | 0x11 | SLPOUT | 退出睡眠 |
//! | 0x20/0x21 | INVOFF/INVON | 反色开关 |
//! | 0x29 | DISPON | 开显示 |
//! | 0x36 | MADCTL | 1 参数：扫描方向 |
//! | 0x3A | COLMOD | 1 参数：像素格式（0x55=16bpp） |
//! | 0x2A | CASET | 4 参数：列窗口 x0h x0l x1h x1l |
//! | 0x2B | RASET | 4 参数：行窗口 |
//! | 0x2C | RAMWR | 进入显存写模式（数据区后续写 = 像素，窗口内游标递增） |
//!
//! # 显存
//!
//! 240×320×16bpp（0x25800 字节）。像素色序 RGB565。观测：`pixel(x,y)` /
//! 填充统计，供验收断言（固件填充图案 → 模拟器校验显存）。

use crate::peripheral::fsmc::LcdWindowDevice;

/// ST7789 分辨率
pub const LCD_W: u16 = 240;
pub const LCD_H: u16 = 320;
/// RDDID 回读 ID（ST7789）
pub const ST7789_ID: u8 = 0x85;

/// 命令区边界（窗口内偏移 < CMD_END 为命令区）
const CMD_END: u32 = 0x8000;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    /// 等命令
    Idle,
    /// 收参数（命令期望 total 个，已收 got 个）
    Params { total: usize, got: usize },
    /// 显存写（RAMWR 后）
    RamWrite,
}

/// ST7789 LCD 虚拟器件
pub struct St7789 {
    /// 显存 16bpp（行优先）
    fb: Vec<u16>,
    /// 当前命令
    cmd: u8,
    /// 参数缓冲（CASET/RASET 用）
    params: [u8; 8],
    /// 状态机
    mode: Mode,
    /// 写窗口（含边界）
    wx0: u16,
    wx1: u16,
    wy0: u16,
    wy1: u16,
    /// RAMWR 游标
    cx: u16,
    cy: u16,
    /// 读 ID 模式（RDDID 后 read 回 ID）
    read_id: bool,
    /// 观测
    pub n_cmd: u64,
    pub n_data: u64,
}

impl St7789 {
    pub fn new() -> Self {
        Self {
            fb: vec![0; LCD_W as usize * LCD_H as usize],
            cmd: 0,
            params: [0; 8],
            mode: Mode::Idle,
            wx0: 0,
            wx1: LCD_W - 1,
            wy0: 0,
            wy1: LCD_H - 1,
            cx: 0,
            cy: 0,
            read_id: false,
            n_cmd: 0,
            n_data: 0,
        }
    }

    /// 像素值（观测/断言）
    pub fn pixel(&self, x: u16, y: u16) -> Option<u16> {
        if x >= LCD_W || y >= LCD_H {
            return None;
        }
        self.fb.get(y as usize * LCD_W as usize + x as usize).copied()
    }

    /// 非背景（非 0）像素数（观测）
    pub fn filled_pixels(&self) -> usize {
        self.fb.iter().filter(|&&p| p != 0).count()
    }

    /// 当前命令（观测）
    pub fn current_cmd(&self) -> u8 {
        self.cmd
    }

    fn apply_params(&mut self) {
        match self.cmd {
            0x2A => {
                // CASET：x0h x0l x1h x1l
                self.wx0 = ((self.params[0] as u16) << 8) | self.params[1] as u16;
                self.wx1 = ((self.params[2] as u16) << 8) | self.params[3] as u16;
            }
            0x2B => {
                self.wy0 = ((self.params[0] as u16) << 8) | self.params[1] as u16;
                self.wy1 = ((self.params[2] as u16) << 8) | self.params[3] as u16;
            }
            _ => {}
        }
    }

    fn on_cmd(&mut self, byte: u8) {
        self.cmd = byte;
        self.mode = match byte {
            0x2A | 0x2B => Mode::Params { total: 4, got: 0 },
            0x36 | 0x3A => Mode::Params { total: 1, got: 0 },
            0x2C => {
                // RAMWR：游标回窗口起点，进入显存写
                self.cx = self.wx0;
                self.cy = self.wy0;
                Mode::RamWrite
            }
            0x04 => {
                self.read_id = true;
                Mode::Idle
            }
            0x01 => {
                // SWRESET：清窗口/状态
                self.wx0 = 0;
                self.wx1 = LCD_W - 1;
                self.wy0 = 0;
                self.wy1 = LCD_H - 1;
                self.read_id = false;
                Mode::Idle
            }
            _ => Mode::Idle, // SLPOUT/INVON/DISPON 等无参数
        };
    }

    fn on_data(&mut self, value: u16) {
        match self.mode {
            Mode::RamWrite => {
                let idx = self.cy as usize * LCD_W as usize + self.cx as usize;
                if idx < self.fb.len() {
                    self.fb[idx] = value;
                }
                self.cx += 1;
                if self.cx > self.wx1 {
                    self.cx = self.wx0;
                    self.cy += 1;
                    if self.cy > self.wy1 {
                        self.cy = self.wy0; // 回卷
                    }
                }
            }
            Mode::Params { total, got } => {
                self.params[got] = (value & 0xFF) as u8;
                if got + 1 == total {
                    self.apply_params();
                    self.mode = Mode::Idle;
                } else {
                    self.mode = Mode::Params { total, got: got + 1 };
                }
            }
            Mode::Idle => {
                // 无参数命令后的多余数据：忽略
            }
        }
    }
}

impl Default for St7789 {
    fn default() -> Self {
        Self::new()
    }
}

impl LcdWindowDevice for St7789 {
    fn name(&self) -> &str {
        "st7789"
    }

    fn window_read(&mut self, _off: u32, _size: u32) -> u32 {
        if self.read_id {
            self.read_id = false;
            return ST7789_ID as u32;
        }
        0xFF // 其他读：数据总线默认高
    }

    fn window_write(&mut self, off: u32, _size: u32, value: u32) {
        if off < CMD_END {
            // 命令区：低 8 位命令字节
            self.n_cmd += 1;
            self.on_cmd((value & 0xFF) as u8);
        } else {
            // 数据区：低 16 位（参数/像素）
            self.n_data += 1;
            self.on_data((value & 0xFFFF) as u16);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(l: &mut St7789, c: u8) {
        l.window_write(0x0000, 2, c as u32);
    }

    fn data(l: &mut St7789, v: u16) {
        l.window_write(0x8000, 2, v as u32);
    }

    fn params(l: &mut St7789, c: u8, p: &[u8]) {
        cmd(l, c);
        for &b in p {
            data(l, b as u16);
        }
    }

    #[test]
    fn cmd_data_region_split() {
        let mut l = St7789::new();
        cmd(&mut l, 0x11); // SLPOUT
        assert_eq!(l.n_cmd, 1);
        assert_eq!(l.n_data, 0);
        data(&mut l, 0x1234); // 数据区（Idle 下忽略）
        assert_eq!(l.n_data, 1);
        assert_eq!(l.current_cmd(), 0x11);
    }

    #[test]
    fn read_id() {
        let mut l = St7789::new();
        cmd(&mut l, 0x04); // RDDID
        assert_eq!(l.window_read(0, 2), ST7789_ID as u32, "RDDID 读回 0x85");
        assert_eq!(l.window_read(0, 2), 0xFF, "仅一次");
    }

    #[test]
    fn colmod_param() {
        let mut l = St7789::new();
        params(&mut l, 0x3A, &[0x55]); // COLMOD 16bpp
        assert_eq!(l.current_cmd(), 0x3A);
        // 参数后回到 Idle：后续数据不再当参数
        data(&mut l, 0xFFFF);
    }

    #[test]
    fn fullscreen_fill() {
        let mut l = St7789::new();
        params(&mut l, 0x2A, &[0, 0, 0, (LCD_W - 1) as u8]);
        params(&mut l, 0x2B, &[0, 0, 1, (LCD_H - 1) as u8]); // 319 = 0x013F
        cmd(&mut l, 0x2C); // RAMWR
        let color: u16 = 0xF800; // 红
        for _ in 0..(LCD_W as usize * LCD_H as usize) {
            data(&mut l, color);
        }
        assert_eq!(l.pixel(0, 0), Some(color));
        assert_eq!(l.pixel(LCD_W - 1, LCD_H - 1), Some(color));
        assert_eq!(l.filled_pixels(), LCD_W as usize * LCD_H as usize);
    }

    #[test]
    fn window_fill_wraps() {
        let mut l = St7789::new();
        // 2x2 窗口 (1,1)-(2,2)
        params(&mut l, 0x2A, &[0, 1, 0, 2]);
        params(&mut l, 0x2B, &[0, 1, 0, 2]);
        cmd(&mut l, 0x2C);
        for i in 0..4u16 {
            data(&mut l, 0x0000 + i);
        }
        assert_eq!(l.pixel(1, 1), Some(0x0000));
        assert_eq!(l.pixel(2, 1), Some(0x0001));
        assert_eq!(l.pixel(1, 2), Some(0x0002));
        assert_eq!(l.pixel(2, 2), Some(0x0003));
        // 窗口外不受影响
        assert_eq!(l.pixel(0, 0), Some(0));
    }

    #[test]
    fn soft_reset_clears_window() {
        let mut l = St7789::new();
        params(&mut l, 0x2A, &[0, 0, 0, 10]);
        assert_eq!(l.wx1, 10);
        cmd(&mut l, 0x01); // SWRESET
        assert_eq!(l.wx1, LCD_W - 1, "SWRESET 重置窗口");
        assert!(!l.read_id);
    }
}
