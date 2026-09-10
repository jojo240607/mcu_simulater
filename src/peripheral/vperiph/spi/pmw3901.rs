//! PMW3901 光流传感器 SPI 从设备（PixArt 光学流，SPI 7bit 寄存器地址协议）。
//!
//! # 寄存器（datasheet 常用区）
//!
//! | 地址 | 名称 | 值 |
//! |---|---|---|
//! | 0x00 | Product_ID | 0x49（PMW3901） |
//! | 0x01 | Revision_ID | 0x01 |
//! | 0x02 | Motion | bit7=数据就绪（简化：恒 0x80） |
//! | 0x03/0x04 | Delta_X_L/H | 16 位有符号 8.8 定点（1.0px = 0x0100） |
//! | 0x05/0x06 | Delta_Y_L/H | 同上 |
//! | 0x07 | SQUAL | 表面质量 0..169 |
//!
//! SPI 帧（与 BMI088 同）：CS 拉低开始，首字节 = `reg<<1|rw`；读帧后续字节按
//! 地址递增连续读，写帧 1 字节数据（只读区忽略）。CS 拉高结束帧。
//!
//! # 数据源 / 联动预留
//!
//! [`FlowModel`] 提供 delta_x/delta_y/squal：默认静态模型（固定像素位移）；
//! 将来可升级为"动力学驱动"——由机体水平速度/姿态积分出帧间像素位移，与
//! ESC 电机转速联动（rpm → 速度 → 光流）构成飞控闭环的观测环。

use super::VirtualSpiSlave;

/// PMW3901 寄存器文件大小（0x00..0x1F 常用区）
const REGS: usize = 32;
/// Product_ID（PMW3901MB-TXQT）
pub const PMW3901_PRODUCT_ID: u8 = 0x49;
/// Revision_ID
pub const PMW3901_REVISION: u8 = 0x01;

/// 光流数据模型（像素位移/表面质量；联动预留：由机体动力学驱动）
pub trait FlowModel: Send + Sync {
    /// 帧间 X 位移（像素，8.8 定点口径：1.0 = 0x0100）
    fn delta_x(&self) -> f32;
    /// 帧间 Y 位移（像素）
    fn delta_y(&self) -> f32;
    /// 表面质量（0..169）
    fn squal(&self) -> u8;
    /// 仿真时间推进
    fn step(&mut self, _dt: f32) {}
}

/// 静态光流模型（固定像素位移，验收数值稳定）
pub struct StaticFlow {
    dx: f32,
    dy: f32,
    squal: u8,
}

impl StaticFlow {
    pub fn new(dx: f32, dy: f32, squal: u8) -> Self {
        Self { dx, dy, squal }
    }
}

impl Default for StaticFlow {
    fn default() -> Self {
        Self::new(1.0, 0.5, 120)
    }
}

impl FlowModel for StaticFlow {
    fn delta_x(&self) -> f32 {
        self.dx
    }
    fn delta_y(&self) -> f32 {
        self.dy
    }
    fn squal(&self) -> u8 {
        self.squal
    }
}

/// SPI 帧状态机
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Frame {
    /// 等待首字节（reg<<1|rw）
    ExpectAddr,
    /// 写帧：等待 1 字节数据（只读区忽略）
    WriteData { reg: u8 },
    /// 读帧：返回当前地址并递增（连续读）
    ReadStream { next: u8 },
}

/// PMW3901 SPI 光流从设备
pub struct Pwm3901 {
    /// 寄存器文件
    regs: [u8; REGS],
    /// 光流数据模型
    flow: Option<Box<dyn FlowModel>>,
    /// CS GPIO 坐标（port, pin）
    cs: (u8, u8),
    /// CS 是否拉低（选中）
    selected: bool,
    /// 帧状态机
    frame: Frame,
    /// 总访问字节数（观测）
    access: u64,
    /// 寄存器写次数（观测；只读区忽略不计）
    pub n_writes: u64,
    /// 寄存器读次数（观测）
    pub n_reads: u64,
}

impl Pwm3901 {
    /// 新建 PMW3901 从设备。`cs` = CS 的 GPIO (port, pin)。
    pub fn new(cs: (u8, u8), flow: impl FlowModel + 'static) -> Self {
        let mut s = Self {
            regs: [0; REGS],
            flow: Some(Box::new(flow)),
            cs,
            selected: false,
            frame: Frame::ExpectAddr,
            access: 0,
            n_writes: 0,
            n_reads: 0,
        };
        s.poke(0x00, PMW3901_PRODUCT_ID);
        s.poke(0x01, PMW3901_REVISION);
        s.poke(0x02, 0x80); // Motion：数据就绪
        s.refresh_delta();
        s
    }

    /// 写寄存器文件（模型初始化/观测用；固件写只读区不影响）。
    pub fn poke(&mut self, reg: u8, value: u8) {
        if (reg as usize) < REGS {
            self.regs[reg as usize] = value;
        }
    }

    /// 读寄存器值（观测辅助，不推进帧状态）。
    pub fn peek(&self, reg: u8) -> Option<u8> {
        self.regs.get(reg as usize).copied()
    }

    /// 从光流模型刷新 Delta_X/Y（8.8 定点）与 SQUAL。
    fn refresh_delta(&mut self) {
        if let Some(f) = &self.flow {
            let dx = (f.delta_x() * 256.0).round() as i16;
            let dy = (f.delta_y() * 256.0).round() as i16;
            self.regs[0x03] = dx as u8;
            self.regs[0x04] = (dx >> 8) as u8;
            self.regs[0x05] = dy as u8;
            self.regs[0x06] = (dy >> 8) as u8;
            self.regs[0x07] = f.squal();
        }
    }
}

impl VirtualSpiSlave for Pwm3901 {
    fn name(&self) -> &str {
        "pmw3901"
    }

    fn selected(&self) -> bool {
        self.selected
    }

    fn on_cs(&mut self, port: u8, pin: u8, level: bool) {
        if (port, pin) != self.cs {
            return;
        }
        if level {
            // 拉高：帧结束
            self.selected = false;
            self.frame = Frame::ExpectAddr;
        } else {
            // 拉低：选中并开始新帧
            self.selected = true;
            self.frame = Frame::ExpectAddr;
        }
    }

    fn on_byte(&mut self, byte: u8) -> u8 {
        if !self.selected {
            return 0xFF; // 未选中：MISO 默认高
        }
        self.access += 1;
        match self.frame {
            Frame::ExpectAddr => {
                let reg = byte >> 1;
                let rw = byte & 1;
                self.frame = if rw != 0 {
                    Frame::ReadStream { next: reg }
                } else {
                    Frame::WriteData { reg }
                };
                0xFF // 首字节 MISO 无意义
            }
            Frame::WriteData { reg } => {
                // PMW3901 数据寄存器只读：写忽略（固件可能写配置区，仿真不建模）
                self.n_writes += 1;
                self.frame = Frame::ExpectAddr;
                0xFF
            }
            Frame::ReadStream { next } => {
                self.n_reads += 1;
                let v = if (next as usize) < REGS { self.regs[next as usize] } else { 0xFF };
                self.frame = Frame::ReadStream {
                    next: next.wrapping_add(1),
                };
                v
            }
        }
    }

    fn access_count(&self) -> u64 {
        self.access
    }

    fn step(&mut self, dt: f32) {
        if let Some(f) = &mut self.flow {
            f.step(dt);
        }
        self.refresh_delta();
    }
}

impl Default for Pwm3901 {
    fn default() -> Self {
        Self::new((4, 11), StaticFlow::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 模拟一帧：CS 拉低 → 字节序列（首字节 reg<<1|rw）→ CS 拉高
    fn frame(sl: &mut Pwm3901, cs: (u8, u8), bytes: &[u8]) -> Vec<u8> {
        sl.on_cs(cs.0, cs.1, false);
        let out: Vec<u8> = bytes.iter().map(|&b| sl.on_byte(b)).collect();
        sl.on_cs(cs.0, cs.1, true);
        out
    }

    /// 读一寄存器：发 (reg<<1)|1 后发 1 字节（0xFF 任意），返回第 2 字节的 MISO
    fn read_reg(sl: &mut Pwm3901, cs: (u8, u8), reg: u8) -> u8 {
        let out = frame(sl, cs, &[(reg << 1) | 1, 0xFF]);
        out[1]
    }

    #[test]
    fn product_and_revision() {
        let mut p = Pwm3901::default(); // cs=(4,11)
        assert_eq!(read_reg(&mut p, (4, 11), 0x00), PMW3901_PRODUCT_ID);
        assert_eq!(read_reg(&mut p, (4, 11), 0x01), PMW3901_REVISION);
    }

    #[test]
    fn delta_xy_squal_8_8_fixed() {
        // 默认模型 dx=1.0（0x0100）、dy=0.5（0x0080）、squal=120
        let mut p = Pwm3901::default();
        let out = frame(&mut p, (4, 11), &[(0x03 << 1) | 1, 0xFF, 0xFF, 0xFF, 0xFF]);
        // 连续读 0x03..0x06：DX_L=0x00 DX_H=0x01 DY_L=0x80 DY_H=0x00
        assert_eq!(out[1], 0x00);
        assert_eq!(out[2], 0x01);
        assert_eq!(out[3], 0x80);
        assert_eq!(out[4], 0x00);
        assert_eq!(read_reg(&mut p, (4, 11), 0x07), 120, "SQUAL");
        // 16 位小端合成
        let dx = (p.peek(0x04).unwrap() as i16) << 8 | p.peek(0x03).unwrap() as i16;
        assert_eq!(dx, 256);
        let dy = (p.peek(0x06).unwrap() as i16) << 8 | p.peek(0x05).unwrap() as i16;
        assert_eq!(dy, 128);
    }

    #[test]
    fn motion_ready_bit() {
        let mut p = Pwm3901::default();
        assert_eq!(read_reg(&mut p, (4, 11), 0x02) & 0x80, 0x80, "Motion 数据就绪位");
    }

    #[test]
    fn unselected_returns_0xff() {
        let mut p = Pwm3901::default();
        // 未拉低 CS 直接发字节 → MISO 0xFF
        assert_eq!(p.on_byte((0x03 << 1) | 1), 0xFF);
        assert_eq!(p.access, 0, "未选中不计数访问");
        // 其他 CS 引脚（如 bmi088 的 PE7）不影响本从机
        p.on_cs(4, 7, false);
        assert_eq!(p.on_byte((0x00 << 1) | 1), 0xFF);
        assert!(!p.selected());
    }

    #[test]
    fn write_ignored_readonly() {
        let mut p = Pwm3901::default();
        // 写帧：首字节 0x00<<1|0 = 0x00，数据 0xAA → Product_ID 不应变
        frame(&mut p, (4, 11), &[0x00, 0xAA]);
        assert_eq!(p.peek(0x00).unwrap(), PMW3901_PRODUCT_ID, "只读区写忽略");
        assert_eq!(p.n_writes, 1, "写帧仍计为访问");
    }
}
