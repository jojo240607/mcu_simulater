//! SHT30 数字温湿度传感器 I2C 从设备（0x44，Sensirion）。
//!
//! # I2C 语义
//!
//! 命令（单次事务写 2 字节）：0xE000 = 高重复度测量触发；0xE001/0xE002 = 中/低。
//! 数据（后续读事务，6 字节）：T_msb, T_lsb, T_crc, RH_msb, RH_lsb, RH_crc。
//! 温度编码：T(°C) = -45 + 175 × raw/65535；湿度：RH(%) = 100 × raw/65535。
//!
//! # 数据源 / 联动预留
//!
//! [`TempHumiModel`] 提供温度/湿度（默认静态 25.0°C / 50.0%）；将来由机体
//! 热模型/环境模型驱动（与气压高度、IMU 共同构成环境感知环）。

use super::super::{I2cDir, VirtualI2cSlave};

/// I2C 7 位地址
pub const SHT30_ADDR7: u8 = 0x44;
/// 测量触发命令（高重复度）
pub const CMD_MEAS_HIGH: u16 = 0xE000;

/// 温湿度模型（联动预留：由环境/热模型驱动）
pub trait TempHumiModel: Send + Sync {
    /// 温度（°C）
    fn temp_c(&self) -> f32;
    /// 相对湿度（%）
    fn rh_pct(&self) -> f32;
    /// 仿真时间推进
    fn step(&mut self, _dt: f32) {}
}

/// 静态温湿度模型（固定值，验收数值稳定）
pub struct StaticTempHumi {
    temp_c: f32,
    rh_pct: f32,
}

impl StaticTempHumi {
    pub fn new(temp_c: f32, rh_pct: f32) -> Self {
        Self { temp_c, rh_pct }
    }
}

impl Default for StaticTempHumi {
    fn default() -> Self {
        Self::new(25.0, 50.0)
    }
}

impl TempHumiModel for StaticTempHumi {
    fn temp_c(&self) -> f32 {
        self.temp_c
    }
    fn rh_pct(&self) -> f32 {
        self.rh_pct
    }
}

/// I2C 状态机
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// 写事务：等命令高字节
    ExpectCmdHi,
    /// 写事务：等命令低字节
    ExpectCmdLo,
    /// 读事务：输出 6 字节测量数据
    ReadStream { n: usize },
}

/// SHT30 温湿度从设备
pub struct Sht30 {
    /// 温湿度模型
    model: Option<Box<dyn TempHumiModel>>,
    /// 最近命令
    cmd: u16,
    /// I2C 状态机
    state: State,
    /// 读次数（观测）
    pub n_reads: u64,
}

impl Sht30 {
    pub fn new(model: impl TempHumiModel + 'static) -> Self {
        Self {
            model: Some(Box::new(model)),
            cmd: 0,
            state: State::ExpectCmdHi,
            n_reads: 0,
        }
    }

    /// 温度编码原始值（16 位，SHT30 线性映射）
    fn temp_raw(&self) -> u16 {
        let t = self.model.as_ref().map(|m| m.temp_c()).unwrap_or(25.0);
        ((t + 45.0) / 175.0 * 65535.0).round().clamp(0.0, 65535.0) as u16
    }

    /// 湿度编码原始值
    fn rh_raw(&self) -> u16 {
        let rh = self.model.as_ref().map(|m| m.rh_pct()).unwrap_or(50.0);
        (rh / 100.0 * 65535.0).round().clamp(0.0, 65535.0) as u16
    }
}

impl Default for Sht30 {
    fn default() -> Self {
        Self::new(StaticTempHumi::default())
    }
}

impl VirtualI2cSlave for Sht30 {
    fn name(&self) -> &str {
        "sht30"
    }

    fn addr7(&self) -> u8 {
        SHT30_ADDR7
    }

    fn on_start(&mut self, dir: I2cDir) {
        match dir {
            I2cDir::Read => {
                // 测量命令后读：输出 6 字节（T_hi T_lo CRC RH_hi RH_lo CRC）
                if self.cmd == CMD_MEAS_HIGH {
                    self.state = State::ReadStream { n: 0 };
                } else {
                    self.state = State::ReadStream { n: 6 }; // 未触发：空读
                }
            }
            I2cDir::Write => self.state = State::ExpectCmdHi,
        }
    }

    fn on_write(&mut self, byte: u8) {
        match self.state {
            State::ExpectCmdHi => {
                self.cmd = (byte as u16) << 8;
                self.state = State::ExpectCmdLo;
            }
            State::ExpectCmdLo => {
                self.cmd |= byte as u16;
                self.state = State::ExpectCmdHi; // 命令完成，等下一事务
            }
            State::ReadStream { .. } => {
                // 读事务中异常写：忽略
            }
        }
    }

    fn on_read(&mut self) -> Option<u8> {
        if let State::ReadStream { n } = self.state {
            self.n_reads += 1;
            let t = self.temp_raw();
            let rh = self.rh_raw();
            // 索引：0 T_hi 1 T_lo 2 CRC 3 RH_hi 4 RH_lo 5 CRC（CRC 简化 0xFF）
            let v = match n {
                0 => (t >> 8) as u8,
                1 => (t & 0xFF) as u8,
                2 => 0xFF,
                3 => (rh >> 8) as u8,
                4 => (rh & 0xFF) as u8,
                _ => 0xFF,
            };
            if n >= 5 {
                self.state = State::ReadStream { n: 6 };
            } else {
                self.state = State::ReadStream { n: n + 1 };
            }
            Some(v)
        } else {
            Some(0xFF)
        }
    }

    fn read_count(&self) -> u64 {
        self.n_reads
    }

    fn step(&mut self, dt: f32) {
        if let Some(m) = &mut self.model {
            m.step(dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn addr7_and_default() {
        let s = Sht30::default();
        assert_eq!(s.addr7(), SHT30_ADDR7);
    }

    #[test]
    fn meas_high_reads_6bytes() {
        // 默认 25.0°C / 50.0%RH
        let mut s = Sht30::default();
        // 写命令 0xE000
        s.on_start(I2cDir::Write);
        s.on_write(0xE0);
        s.on_write(0x00);
        // 读 6 字节
        s.on_start(I2cDir::Read);
        let mut buf = [0u8; 6];
        for b in buf.iter_mut() {
            *b = s.on_read().unwrap();
        }
        let t = ((buf[0] as u16) << 8) | buf[1] as u16;
        let rh = ((buf[3] as u16) << 8) | buf[4] as u16;
        // 25°C → (25+45)/175*65535 = 26214 = 0x6666
        assert_eq!(t, 26214, "温度原始值 0x6666");
        // 50% → 0x8000 = 32768
        assert_eq!(rh, 32768, "湿度原始值 0x8000");
        assert_eq!(s.n_reads, 6);
    }

    #[test]
    fn custom_model_values() {
        let mut s = Sht30::new(StaticTempHumi::new(28.5, 65.0));
        s.on_start(I2cDir::Write);
        s.on_write(0xE0);
        s.on_write(0x00);
        s.on_start(I2cDir::Read);
        let mut buf = [0u8; 6];
        for b in buf.iter_mut() {
            *b = s.on_read().unwrap();
        }
        let t = ((buf[0] as u16) << 8) | buf[1] as u16;
        let rh = ((buf[3] as u16) << 8) | buf[4] as u16;
        // 28.5°C → (28.5+45)/175*65535 = 27525.3 ≈ 27525
        assert_eq!(t, 27525);
        // 65% → 42597
        assert_eq!(rh, 42598);
    }
}
