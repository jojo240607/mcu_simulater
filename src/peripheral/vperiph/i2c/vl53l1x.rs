//! VL53L1X ToF 激光测距传感器 I2C 从设备（ST，I2C 7 位地址 0x29，16 位寄存器地址）。
//!
//! # I2C 语义（16 位寄存器地址，高字节在前）
//!
//! 写寄存器：START(W) → reg_hi → reg_lo → data...（指针递增）；
//! 读寄存器：先 START(W) 写 16 位地址设指针，再 START(R) 从指针连续读。
//!
//! # 寄存器（固件 vl53l1x 驱动用到的）
//!
//! | 地址 | 名称 | 语义 |
//! |---|---|---|
//! | 0x010F | I_AM_VL53L1X | WHO_AM_I = 0xEA |
//! | 0x000F | FIRMWARE_SYSTEM_STATUS | bit3 = 固件已加载（boot 后置 1） |
//! | 0x0000 | SOFT_RESET | 写任意值 → 软复位（置固件就绪） |
//! | 0x0040 | RANGE_START | 写任意值 → 测距：置 0x0013 bit3 + 刷新 0x0096 距离 |
//! | 0x0013 | RESULT_INTERRUPT_STATUS | bit3 = 测距完成；写任意值清中断 |
//! | 0x0096 | RESULT_RANGE_MM | 距离 mm（16 位小端） |
//!
//! # 数据源 / 联动预留
//!
//! [`ToFModel`] 提供距离（默认静态模型固定值）；将来由"机体高度/地面模型"
//! 驱动——与 ESC 转速、PMW3901 光流共同构成飞控闭环的观测环。

use super::super::{I2cDir, VirtualI2cSlave};

/// I2C 7 位地址
pub const VL53L1X_ADDR7: u8 = 0x29;
/// WHO_AM_I
pub const VL53L1X_WHO_AM_I: u8 = 0xEA;
/// 寄存器文件大小（0x0000..0x01FF）
const REGS: usize = 0x200;

/// 常用寄存器地址
pub const REG_SOFT_RESET: u16 = 0x0000;
pub const REG_FW_SYSTEM_STATUS: u16 = 0x000F;
pub const REG_INTERRUPT_STATUS: u16 = 0x0013;
pub const REG_RANGE_START: u16 = 0x0040;
pub const REG_RANGE_MM: u16 = 0x0096;
pub const REG_WHO_AM_I: u16 = 0x010F;

/// ToF 距离模型（联动预留：由机体高度/地面模型驱动）
pub trait ToFModel: Send + Sync {
    /// 当前距离（mm）
    fn distance_mm(&self) -> u16;
    /// 仿真时间推进
    fn step(&mut self, _dt: f32) {}
}

/// 静态 ToF 模型（固定距离，验收数值稳定）
pub struct StaticToF {
    mm: u16,
}

impl StaticToF {
    pub fn new(mm: u16) -> Self {
        Self { mm }
    }
}

impl Default for StaticToF {
    fn default() -> Self {
        Self::new(500)
    }
}

impl ToFModel for StaticToF {
    fn distance_mm(&self) -> u16 {
        self.mm
    }
}

/// I2C 状态机
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// 写事务：等寄存器地址高字节
    ExpectAddrHi,
    /// 写事务：等寄存器地址低字节（设 16 位指针）
    ExpectAddrLo,
    /// 写事务：数据写入指针并递增
    WriteData,
    /// 读事务：从指针连续读并递增
    ReadStream,
}

/// VL53L1X ToF 激光测距从设备
pub struct Vl53l1x {
    /// 寄存器文件（16 位地址索引）
    regs: [u8; REGS],
    /// 距离模型
    tof: Option<Box<dyn ToFModel>>,
    /// 当前 16 位寄存器指针
    ptr: u16,
    /// I2C 状态机
    state: State,
    /// 寄存器写次数（观测）
    pub n_writes: u64,
    /// 寄存器读次数（观测）
    pub n_reads: u64,
}

impl Vl53l1x {
    pub fn new(tof: impl ToFModel + 'static) -> Self {
        let mut s = Self {
            regs: [0; REGS],
            tof: Some(Box::new(tof)),
            ptr: 0,
            state: State::ExpectAddrHi,
            n_writes: 0,
            n_reads: 0,
        };
        s.write_reg(REG_WHO_AM_I, VL53L1X_WHO_AM_I as u16);
        s
    }

    /// 直接读写寄存器文件（观测辅助；写只读区不影响语义）。
    pub fn poke(&mut self, reg: u16, value: u8) {
        if (reg as usize) < REGS {
            self.regs[reg as usize] = value;
        }
    }

    pub fn peek(&self, reg: u16) -> Option<u8> {
        self.regs.get(reg as usize).copied()
    }

    fn write_reg(&mut self, reg: u16, value: u16) {
        if (reg as usize) + 1 < REGS {
            self.regs[reg as usize] = (value & 0xFF) as u8;
            self.regs[reg as usize + 1] = (value >> 8) as u8;
        }
    }

    /// 软复位：置固件就绪（FIRMWARE_SYSTEM_STATUS bit3）
    fn soft_reset(&mut self) {
        self.poke(REG_FW_SYSTEM_STATUS, 0x08);
    }

    /// 启动测距：刷新距离到 RESULT_RANGE_MM，置测距完成中断
    fn start_range(&mut self) {
        let mm = self.tof.as_ref().map(|t| t.distance_mm()).unwrap_or(0);
        self.write_reg(REG_RANGE_MM, mm);
        let st = self.peek(REG_INTERRUPT_STATUS).unwrap_or(0);
        self.poke(REG_INTERRUPT_STATUS, st | 0x08); // bit3 = range complete
    }

    /// 写寄存器操作（按 I2C 状态机；对 RANGE_START/SOFT_RESET 触发模型行为）
    fn apply_write(&mut self, reg: u16, value: u8) {
        match reg {
            REG_SOFT_RESET => self.soft_reset(),
            REG_RANGE_START => self.start_range(),
            REG_INTERRUPT_STATUS => {
                // 写任意值清中断标志（bit3 清零）
                let st = self.peek(REG_INTERRUPT_STATUS).unwrap_or(0);
                self.poke(REG_INTERRUPT_STATUS, st & !0x08);
            }
            _ => {
                if (reg as usize) < REGS {
                    self.regs[reg as usize] = value;
                }
            }
        }
    }
}

impl VirtualI2cSlave for Vl53l1x {
    fn name(&self) -> &str {
        "vl53l1x"
    }

    fn addr7(&self) -> u8 {
        VL53L1X_ADDR7
    }

    fn on_start(&mut self, dir: I2cDir) {
        // 读事务从当前指针连续读；写事务先收 16 位寄存器地址
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
                let reg = self.ptr;
                self.apply_write(reg, byte);
                self.ptr = self.ptr.wrapping_add(1); // 连续写指针递增
            }
            State::ReadStream => {
                // 读事务中出现写（通常不会；忽略）
                self.n_writes += 1;
            }
        }
    }

    fn on_read(&mut self) -> Option<u8> {
        match self.state {
            State::ReadStream => {
                self.n_reads += 1;
                let v = if (self.ptr as usize) < REGS {
                    self.regs[self.ptr as usize]
                } else {
                    0xFF
                };
                self.ptr = self.ptr.wrapping_add(1);
                Some(v)
            }
            _ => {
                // 未设指针的读：返回 0xFF（真实器件 NACK；简化回 0xFF 便于轮询不卡死）
                self.n_reads += 1;
                Some(0xFF)
            }
        }
    }

    fn read_count(&self) -> u64 {
        self.n_reads
    }

    fn step(&mut self, dt: f32) {
        if let Some(t) = &mut self.tof {
            t.step(dt);
        }
    }
}

impl Default for Vl53l1x {
    fn default() -> Self {
        Self::new(StaticToF::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 写 16 位寄存器：START(W) → hi → lo → data
    fn write16(sl: &mut Vl53l1x, reg: u16, val: u16) {
        sl.on_start(I2cDir::Write);
        sl.on_write((reg >> 8) as u8);
        sl.on_write((reg & 0xFF) as u8);
        sl.on_write((val & 0xFF) as u8);
        sl.on_write((val >> 8) as u8);
    }

    /// 设 16 位指针（写地址事务，不写数据）
    fn set_ptr(sl: &mut Vl53l1x, reg: u16) {
        sl.on_start(I2cDir::Write);
        sl.on_write((reg >> 8) as u8);
        sl.on_write((reg & 0xFF) as u8);
    }

    /// 从当前指针读 n 字节（START(R) 事务）
    fn read_n(sl: &mut Vl53l1x, n: usize) -> Vec<u8> {
        sl.on_start(I2cDir::Read);
        (0..n).filter_map(|_| sl.on_read()).collect()
    }

    #[test]
    fn who_am_i() {
        let mut v = Vl53l1x::default();
        set_ptr(&mut v, REG_WHO_AM_I);
        let r = read_n(&mut v, 1);
        assert_eq!(r[0], VL53L1X_WHO_AM_I, "WHO_AM_I=0xEA");
        assert_eq!(v.addr7(), VL53L1X_ADDR7);
    }

    #[test]
    fn soft_reset_then_fw_ready() {
        let mut v = Vl53l1x::default();
        assert_eq!(v.peek(REG_FW_SYSTEM_STATUS), Some(0), "复位前固件未就绪");
        write16(&mut v, REG_SOFT_RESET, 0x00);
        let st = v.peek(REG_FW_SYSTEM_STATUS).unwrap();
        assert_ne!(st & 0x08, 0, "软复位后 FIRMWARE_SYSTEM_STATUS bit3=1");
    }

    #[test]
    fn range_start_distance_and_interrupt() {
        let mut v = Vl53l1x::new(StaticToF::new(500));
        // 启动测距 → 距离 500mm (0x01F4) 写入 0x0096，中断 bit3 置位
        write16(&mut v, REG_RANGE_START, 0x40);
        let st = v.peek(REG_INTERRUPT_STATUS).unwrap();
        assert_ne!(st & 0x08, 0, "RANGE_START 后测距完成中断置位");
        set_ptr(&mut v, REG_RANGE_MM);
        let r = read_n(&mut v, 2);
        assert_eq!(r[0], 0xF4, "距离低字节");
        assert_eq!(r[1], 0x01, "距离高字节");
        // 清中断
        write16(&mut v, REG_INTERRUPT_STATUS, 0x01);
        let st = v.peek(REG_INTERRUPT_STATUS).unwrap();
        assert_eq!(st & 0x08, 0, "清中断后 bit3 清零");
        // 再次测距刷新新距离
        let mut v2 = Vl53l1x::new(StaticToF::new(42));
        write16(&mut v2, REG_RANGE_START, 0x40);
        set_ptr(&mut v2, REG_RANGE_MM);
        let r = read_n(&mut v2, 2);
        assert_eq!((r[1] as u16) << 8 | r[0] as u16, 42);
    }

    #[test]
    fn pointer_advances_on_stream_read() {
        let mut v = Vl53l1x::default();
        // 设指针到 0x0096（距离区），连读 2 字节 = 距离小端
        write16(&mut v, REG_RANGE_START, 0x40); // 刷新距离 500mm
        set_ptr(&mut v, REG_RANGE_MM);
        let r = read_n(&mut v, 2);
        assert_eq!((r[1] as u16) << 8 | r[0] as u16, 500);
        assert_eq!(v.n_reads, 2);
    }
}
