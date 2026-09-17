//! BMI088 六轴 IMU 虚拟从设备（SPI，双片选 ACCEL_CS/GYRO_CS）。
//!
//! # 器件语义（固件驱动只依赖这些）
//!
//! SPI 寄存器（7bit 地址；帧首字节 = `reg<<1|rw`，见 [`super::VirtualSpiSlave`]）：
//! - ACCEL：WHO_AM_I 0x00 = 0x1E；数据 ACC_X_L 0x12 .. ACC_Z_H 0x17（6B，16bit LE）
//! - GYRO ：WHO_AM_I 0x00 = 0x0F；数据 GYR_X_L 0x02 .. GYR_Z_H 0x07（6B，16bit LE）
//!
//! 双片选：拉低 ACCEL_CS 访问加速度计寄存器文件，拉低 GYRO_CS 访问陀螺仪文件
//! （0x00 在两者都是 WHO_AM_I 但值不同——必须靠片选区分，单靠地址无法分辨）。
//! 片选经 [`crate::events::Event::GpioLevel`] 由 Machine 转发到 [`on_cs`]。
//!
//! 数据换算（与固件驱动一致，datasheet 灵敏度）：
//! - ACCEL ±3g 量程：10920 LSB/g（raw = g × 10920，16bit 有符号）
//! - GYRO  ±2000dps：16.4 LSB/dps（raw = dps × 16.4，16bit 有符号）
//!
//! # 坐标系约定（重要）
//!
//! 模型输入是**芯片坐标系比力**（SensorModel 的 accel.x/y/z，单位 m/s²）：
//! **芯片平放（z 轴朝上）静止时 accel.z = +9.81**（datasheet 灵敏度表 1g=10920
//! LSB 取正号；drvtest 固件即按此验收）。安装朝向由**传入的模型**表达：
//! - 平放 z 朝上：`StaticImu { accel: [0.0, 0.0, 9.81], .. }` → raw z ≈ +10920
//! - 倒装（FRD z 向下，`StaticImu::default()`）：raw z ≈ -10920（比力 -1g）
//!
//! 读取瞬间刷新（仿 `RegFileSlave` 动态寄存器）。

use super::VirtualSpiSlave;
use crate::peripheral::vperiph::data_source::{DataSource, SensorModel, StaticImu};

/// ACCEL WHO_AM_I 期望值
const ACCEL_WHO_AM_I: u8 = 0x1E;
/// GYRO WHO_AM_I 期望值
const GYRO_WHO_AM_I: u8 = 0x0F;

/// 加速度计寄存器文件大小（0x00..0x7F：WHO_AM_I + 数据 + 配置区，如 ACCEL_CONFIG 0x41）
const REGS: usize = 128;

/// ACCEL 数据区起点（ACC_X_L）与长度（6B：X/Y/Z × L/H，16bit LE）
const ACC_DATA_BASE: u8 = 0x12;
const ACC_DATA_LEN: u8 = 6;
/// GYRO 数据区起点（GYR_X_L）与长度
const GYR_DATA_BASE: u8 = 0x02;
const GYR_DATA_LEN: u8 = 6;

/// BMI088 加速度计 ±3g 量程灵敏度（LSB/g）
pub const ACCEL_LSB_PER_G: f32 = 10920.0;
/// BMI088 陀螺仪 ±2000dps 量程灵敏度（LSB/dps）
pub const GYRO_LSB_PER_DPS: f32 = 16.4;

/// 片选选中的芯片
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Chip {
    Accel,
    Gyro,
}

/// 帧协议状态机（CS 拉低开始一帧，拉高结束）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Frame {
    /// 等待首字节（寄存器地址 | R/W）
    ExpectAddr,
    /// 写帧：等待 1 个数据字节写入 `reg`
    WriteData { reg: u8 },
    /// 读帧：后续字节返回 `next` 寄存器值并递增（连续读）
    ReadStream { next: u8 },
}

/// BMI088 SPI 虚拟从设备。
pub struct Bmi088 {
    /// 加速度计寄存器文件
    accel: [u8; REGS],
    /// 陀螺仪寄存器文件
    gyro: [u8; REGS],
    /// 数据源（动态刷新 ACC/GYR 数据区）
    source: Option<DataSource>,
    /// ACCEL_CS GPIO 坐标（port, pin）
    accel_cs: (u8, u8),
    /// GYRO_CS GPIO 坐标（port, pin）
    gyro_cs: (u8, u8),
    /// 当前片选选中的芯片（None = 无选中）
    selected: Option<Chip>,
    /// 帧状态机
    frame: Frame,
    /// 总访问字节数（观测）
    access: u64,
    /// 寄存器写次数（观测）
    pub n_writes: u64,
    /// 寄存器读次数（观测）
    pub n_reads: u64,
    /// 故障注入：置位后 MISO 恒高（0xFF），帧状态机不推进——模拟芯片断线/
    /// 无响应（固件侧 WHO_AM_I 校验失败 → healthy=false → 读零值 → FDIR 冻结）。
    faulted: bool,
}

impl Bmi088 {
    /// 故障注入开关：`on=true` 使该从设备对一切访问回 0xFF（总线像悬空一样）。
    pub fn set_fault(&mut self, on: bool) {
        self.faulted = on;
    }

    /// 是否处于故障态（观测/断言）。
    pub fn faulted(&self) -> bool {
        self.faulted
    }
}

impl Bmi088 {
    /// 新建 BMI088 从设备。`accel_cs`/`gyro_cs` = ACCEL_CS/GYRO_CS 的 GPIO (port, pin)。
    pub fn new(accel_cs: (u8, u8), gyro_cs: (u8, u8), imu: impl SensorModel + 'static) -> Self {
        let mut s = Self {
            accel: [0; REGS],
            gyro: [0; REGS],
            source: Some(DataSource::Math(Box::new(imu))),
            accel_cs,
            gyro_cs,
            selected: None,
            frame: Frame::ExpectAddr,
            access: 0,
            n_writes: 0,
            n_reads: 0,
            faulted: false,
        };
        s.poke(Chip::Accel, 0x00, ACCEL_WHO_AM_I);
        s.poke(Chip::Gyro, 0x00, GYRO_WHO_AM_I);
        s
    }

    /// 替换数据源（装配期：默认 StaticImu → FlySimSource 等实时模型）。
    pub fn with_source(mut self, imu: impl SensorModel + 'static) -> Self {
        self.source = Some(DataSource::Math(Box::new(imu)));
        self
    }

    /// 写寄存器文件（模型初始化用；WHO_AM_I 等只读区由驱动写保护忽略）。
    pub fn poke(&mut self, chip: Chip, reg: u8, value: u8) {
        let regs = self.regs_mut(chip);
        if (reg as usize) < regs.len() {
            regs[reg as usize] = value;
        }
    }

    /// 读寄存器值（观测辅助，不推进帧状态）。
    pub fn peek(&self, chip: Chip, reg: u8) -> Option<u8> {
        self.regs(chip).get(reg as usize).copied()
    }

    /// 当前片选芯片（观测/断言）
    pub fn selected(&self) -> Option<Chip> {
        self.selected
    }

    fn regs(&self, chip: Chip) -> &[u8; REGS] {
        match chip {
            Chip::Accel => &self.accel,
            Chip::Gyro => &self.gyro,
        }
    }

    fn regs_mut(&mut self, chip: Chip) -> &mut [u8; REGS] {
        match chip {
            Chip::Accel => &mut self.accel,
            Chip::Gyro => &mut self.gyro,
        }
    }

    /// 读取瞬间刷新动态数据区（仿 RegFileSlave 动态寄存器：传感器数据读时求值）。
    fn refresh_dynamic(&mut self, chip: Chip) {
        let Some(src) = &self.source else { return };
        match chip {
            Chip::Accel => {
                // 6B：X/Y/Z × (L,H)，16bit LE，raw = g × LSB_PER_G
                let g = |f: &str| src.value(f) / 9.81;
                for (i, off) in (0..ACC_DATA_LEN).enumerate() {
                    let raw = match i / 2 {
                        0 => (g("accel.x") * ACCEL_LSB_PER_G) as i16,
                        1 => (g("accel.y") * ACCEL_LSB_PER_G) as i16,
                        _ => (g("accel.z") * ACCEL_LSB_PER_G) as i16,
                    };
                    self.accel[ACC_DATA_BASE as usize + i] = raw.to_le_bytes()[i & 1];
                    let _ = off;
                }
            }
            Chip::Gyro => {
                // 6B：X/Y/Z × (L,H)，16bit LE，raw = dps × LSB_PER_DPS
                let dps = |f: &str| src.value(f) * 180.0 / core::f32::consts::PI;
                for i in 0..GYR_DATA_LEN as usize {
                    let raw = match i / 2 {
                        0 => (dps("gyro.x") * GYRO_LSB_PER_DPS) as i16,
                        1 => (dps("gyro.y") * GYRO_LSB_PER_DPS) as i16,
                        _ => (dps("gyro.z") * GYRO_LSB_PER_DPS) as i16,
                    };
                    self.gyro[GYR_DATA_BASE as usize + i] = raw.to_le_bytes()[i & 1];
                }
            }
        }
    }

    /// 从机读一字节（帧内：地址递增连续读；出界回 0xFF）。
    fn read_byte(&mut self, chip: Chip, reg: u8) -> u8 {
        self.refresh_dynamic(chip);
        self.regs(chip).get(reg as usize).copied().unwrap_or(0xFF)
    }

    /// 从机写一字节（WHO_AM_I 只读区忽略）。
    fn write_byte(&mut self, chip: Chip, reg: u8, value: u8) {
        if reg == 0x00 {
            return; // WHO_AM_I 只读
        }
        self.poke(chip, reg, value);
    }
}

impl VirtualSpiSlave for Bmi088 {
    fn name(&self) -> &str {
        "bmi088"
    }

    fn selected(&self) -> bool {
        self.selected.is_some()
    }

    fn on_cs(&mut self, port: u8, pin: u8, level: bool) {
        if level {
            // 拉高：帧结束（丢弃未完成部分）
            if self.selected.is_some() {
                self.selected = None;
                self.frame = Frame::ExpectAddr;
            }
            return;
        }
        // 拉低：匹配 CS 引脚 → 选中对应芯片并开始新帧
        let chip = if (port, pin) == self.accel_cs {
            Some(Chip::Accel)
        } else if (port, pin) == self.gyro_cs {
            Some(Chip::Gyro)
        } else {
            None
        };
        if let Some(c) = chip {
            self.selected = Some(c);
            self.frame = Frame::ExpectAddr;
        }
    }

    fn on_byte(&mut self, byte: u8) -> u8 {
        if self.faulted {
            return 0xFF; // 故障态：MISO 恒高，帧状态机不推进
        }
        let Some(chip) = self.selected else {
            return 0xFF; // 未选中：MISO 默认高
        };
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
                0xFF // 首字节 MISO 无意义（全双工）
            }
            Frame::WriteData { reg } => {
                self.n_writes += 1;
                self.write_byte(chip, reg, byte);
                self.frame = Frame::ExpectAddr;
                0xFF
            }
            Frame::ReadStream { next } => {
                self.n_reads += 1;
                let v = self.read_byte(chip, next);
                self.frame = Frame::ReadStream {
                    next: next.wrapping_add(1), // 连续读地址递增
                };
                v
            }
        }
    }

    fn access_count(&self) -> u64 {
        self.access
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn step(&mut self, dt: f32) {
        if let Some(src) = &mut self.source {
            src.step(dt);
        }
    }
}

impl Default for Bmi088 {
    fn default() -> Self {
        Self::new((4, 7), (4, 8), StaticImu::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::vperiph::spi::{read_regs, write_reg};

    /// 默认 CS：ACCEL_CS=(4,7)（GPIOE7）、GYRO_CS=(4,8)（GPIOE8）
    const ACC: (u8, u8) = (4, 7);
    const GYR: (u8, u8) = (4, 8);

    #[test]
    fn who_am_i_differs_by_chip_select() {
        let mut s = Bmi088::default();
        // 片选区分：同一地址 0x00 在不同片选下值不同
        let acc = read_regs(&mut s, ACC.0, ACC.1, 0x00, 1);
        assert_eq!(acc[0], ACCEL_WHO_AM_I, "ACCEL WHO_AM_I=0x1E");
        let gyr = read_regs(&mut s, GYR.0, GYR.1, 0x00, 1);
        assert_eq!(gyr[0], GYRO_WHO_AM_I, "GYRO WHO_AM_I=0x0F");
    }

    #[test]
    fn static_hover_accel_and_gyro() {
        // 芯片平放（z 轴朝上）静止：accel.z = +9.81 → raw = +10920 = 0x2A98 LE
        let imu = StaticImu {
            accel: [0.0, 0.0, 9.81],
            gyro: [0.0; 3],
        };
        let mut s = Bmi088::new(ACC, GYR, imu);
        // ACCEL 数据：0x12 起 6B 顺序 X/Y/Z × (L,H)
        let acc = read_regs(&mut s, ACC.0, ACC.1, 0x12, 6);
        assert_eq!(&acc[0..4], &[0x00, 0x00, 0x00, 0x00], "accel.x/y=0");
        assert_eq!(&acc[4..6], &[0xA8, 0x2A], "accel.z raw≈+10920 LE（1g=10920 LSB）");
        let gyr = read_regs(&mut s, GYR.0, GYR.1, 0x02, 6);
        assert_eq!(&gyr, &[0x00, 0x00, 0x00, 0x00, 0x00, 0x00], "gyro=0");
    }

    #[test]
    fn frd_downward_mount_reads_negative_g() {
        // 倒装（FRD z 向下，StaticImu::default() 比力 -1g）→ raw = -10920 = 0xD558 LE：
        // 安装朝向由模型表达，换算只做 g → LSB 线性映射
        let mut s = Bmi088::default();
        let acc = read_regs(&mut s, ACC.0, ACC.1, 0x12, 6);
        assert_eq!(&acc[0..4], &[0x00, 0x00, 0x00, 0x00], "accel.x/y=0");
        assert_eq!(&acc[4..6], &[0x58, 0xD5], "accel.z raw≈-10920 LE（比力 -1g）");
    }

    #[test]
    fn write_then_read_back() {
        let mut s = Bmi088::default();
        // 写 ACCEL_CONFIG(0x41) 0x51 → 读回 0x51
        write_reg(&mut s, ACC.0, ACC.1, 0x41, 0x51);
        let r = read_regs(&mut s, ACC.0, ACC.1, 0x41, 1);
        assert_eq!(r[0], 0x51, "写后读回");
        // WHO_AM_I 只读：写 0x00 被忽略
        write_reg(&mut s, ACC.0, ACC.1, 0x00, 0x00);
        let r = read_regs(&mut s, ACC.0, ACC.1, 0x00, 1);
        assert_eq!(r[0], ACCEL_WHO_AM_I, "WHO_AM_I 只读");
    }

    #[test]
    fn continuous_read_increments_addr() {
        // 芯片平放 z 朝上：z=+1g → 0x2A98 LE
        let imu = StaticImu {
            accel: [0.0, 0.0, 9.81],
            gyro: [0.0; 3],
        };
        let mut s = Bmi088::new(ACC, GYR, imu);
        // 读 0x12 起 8 字节：前 6 = 数据区，第 7/8 字节 = 0x18/0x19 寄存器值（0）
        let r = read_regs(&mut s, ACC.0, ACC.1, 0x12, 8);
        assert_eq!(r.len(), 8);
        assert_eq!(&r[4..6], &[0xA8, 0x2A], "ACC_Z 正确（x/y=0，z=+1g 平放）");
        assert_eq!(r[6], 0x00, "0x18 寄存器值");
        assert_eq!(r[7], 0x00, "0x19 寄存器值");
        // 超出文件（0x7F 后）→ 0xFF
        let r = read_regs(&mut s, ACC.0, ACC.1, 0x7E, 3);
        assert_eq!(r[0], 0x00, "0x7E 在文件内");
        assert_eq!(r[1], 0x00, "0x7F 在文件内（末位）");
        assert_eq!(r[2], 0xFF, "0x80 出界回 0xFF");
    }

    #[test]
    fn cs_high_discards_partial_frame() {
        let mut s = Bmi088::default();
        // 拉低后只发首字节（读 0x00）不读完 → 拉高 → 下一帧重新解析首字节
        s.on_cs(ACC.0, ACC.1, false);
        let _ = s.on_byte((0x00 << 1) | 1); // 读 WHO_AM_I 首字节
        s.on_cs(ACC.0, ACC.1, true);        // 中途拉高：丢弃
        // 新帧：首字节应为地址，而非延续读流
        let r = read_regs(&mut s, ACC.0, ACC.1, 0x00, 1);
        assert_eq!(r[0], ACCEL_WHO_AM_I, "CS 高后新帧重新解析");
    }

    #[test]
    fn access_count_tracks_bytes() {
        let mut s = Bmi088::default();
        assert_eq!(s.access_count(), 0);
        read_regs(&mut s, ACC.0, ACC.1, 0x12, 6); // 首字节 + 6 数据 = 7
        assert_eq!(s.access_count(), 7);
    }

    #[test]
    fn unselected_byte_returns_0xff() {
        let mut s = Bmi088::default();
        // 未选中时 on_byte 回 0xFF
        assert_eq!(s.on_byte(0x01), 0xFF);
    }
}
