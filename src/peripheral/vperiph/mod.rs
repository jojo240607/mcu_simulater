//! 虚拟外设层：总线协议级模拟真实挂在 I2C/SPI/UART 总线上的传感器/模块从设备。
//!
//! 设计（与用户确认）：
//! - **总线协议级**：从设备模拟真实 I2C/SPI/UART 从机行为（地址匹配、寄存器指针、
//!   ACK/NACK），固件真实驱动（flyctrl real-sensors）零改动直接跑通；
//! - **挂载在总线外设内部直路由**：i2c/spi/usart 外设持有从设备表，事务发生时
//!   直接路由到匹配从设备（不走 EventBus 字节流解析——避免无 addr/无方向歧义），
//!   EventBus 仍保留供外部注入/观测；
//! - **数据源**：Constant（固定寄存器值/WHO_AM_I）+ Math（物理模型，随仿真时间
//!   推进 step）；后续可扩展 Replay（真机数据回放）/Script（脚本注入）。
//!
//! 首批覆盖（对齐 flyctrl real-sensors 全链路）：
//! - I2C：mpu6050(0x68) / bmp280(0x76) / qmc5883(0x0D)
//! - UART：ublox gps(usart1) / sbus(usart2) —— 见 `uart.rs`（推流）

pub mod data_source;
pub mod models;
pub mod uart;

use data_source::DataSource;

/// I2C 事务方向（地址字节 bit0）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cDir {
    /// master 写（地址字节 R/W=0）
    Write,
    /// master 读（地址字节 R/W=1）
    Read,
}

/// I2C 总线从设备接口。
///
/// 由 [`crate::peripheral::i2c::I2c`] 在 master 事务（地址阶段+数据阶段）中直路由调用。
/// 寄存器指针语义（真实硬件）：写事务首个数据字节 = 寄存器地址（后续写为数据）；
/// 读事务从**当前寄存器指针**连续吐数据（指针跨事务保持——标准传感器
/// `i2c_write_read` = 先写寄存器地址、再读数据）。
pub trait VirtualI2cSlave: Send + Sync {
    /// 从设备名（观测/日志）
    fn name(&self) -> &str;

    /// 7 位 I2C 地址（匹配 addr7）
    fn addr7(&self) -> u8;

    /// 新事务开始（地址阶段匹配后、数据阶段前）。
    fn on_start(&mut self, dir: I2cDir);

    /// master 写一字节（数据阶段）。
    fn on_write(&mut self, byte: u8);

    /// master 读一字节：返回当前寄存器值；`None` = NACK（读失败）。
    fn on_read(&mut self) -> Option<u8>;

    /// 仿真时间推进（Math 数据源步进；UART 从设备也用它驱动推流节拍）。
    fn step(&mut self, _dt: f32) {}

    /// 被读次数（观测/断言：虚拟外设是否被固件访问）。
    fn read_count(&self) -> u64 {
        0
    }
}

/// 通用寄存器文件 I2C 从设备：寄存器 map + 动态数据源刷新 + 寄存器指针语义。
///
/// 覆盖 MPU6050/BMP280/QMC5883 等标准"写寄存器地址→读数据"传感器。
pub struct RegFileSlave {
    name: String,
    addr7: u8,
    /// 寄存器文件（0..N-1）
    regs: Vec<u8>,
    /// 数据源列表（动态寄存器填充回调经索引引用）
    sources: Vec<DataSource>,
    /// 动态寄存器：读前从数据源刷新（传感器数据寄存器）
    dynamic: Vec<DynamicReg>,
    /// 当前寄存器指针
    ptr: u8,
    /// 期待首字节为寄存器地址（每次 START 后置位）
    expect_reg: bool,
    /// 诊断计数（观测用）
    pub n_writes: u64,
    pub n_reads: u64,
    /// 手动 NACK（故障注入：模拟断线/无响应 → 固件 healthy=false → FDIR 降级）
    pub nack: bool,
}

/// 动态寄存器：读该地址区间前从数据源刷新。
struct DynamicReg {
    /// 寄存器偏移（低字节）
    offset: u8,
    /// 长度（字节）
    len: u8,
    /// 刷新回调：按字节索引返回该寄存器字节值（数据源经 `src.value(field)` 取值）
    fill: Box<dyn Fn(&DataSource, usize) -> u8 + Send + Sync>,
    /// 关联的数据源索引（sources）
    source: usize,
}

impl RegFileSlave {
    /// 新建寄存器文件从设备。
    ///
    /// `regs` 初始寄存器文件；`name`/`addr7` 用于地址匹配与观测。
    pub fn new(name: &str, addr7: u8, regs: Vec<u8>) -> Self {
        Self {
            name: name.to_string(),
            addr7,
            regs,
            sources: Vec::new(),
            dynamic: Vec::new(),
            ptr: 0,
            expect_reg: true,
            n_writes: 0,
            n_reads: 0,
            nack: false,
        }
    }

    /// 添加数据源（动态寄存器填充经索引引用）。
    pub fn add_source(&mut self, source: DataSource) -> usize {
        self.sources.push(source);
        self.sources.len() - 1
    }

    /// 注册动态寄存器区间（读前从 `source` 索引经 `fill` 刷新）。
    pub fn add_dynamic<F>(&mut self, offset: u8, len: u8, source: usize, fill: F)
    where
        F: Fn(&DataSource, usize) -> u8 + Send + Sync + 'static,
    {
        self.dynamic.push(DynamicReg {
            offset,
            len,
            fill: Box::new(fill),
            source,
        });
    }

    /// 写寄存器文件（供模型初始化默认值）。
    pub fn poke(&mut self, offset: u8, value: u8) {
        if (offset as usize) < self.regs.len() {
            self.regs[offset as usize] = value;
        }
    }

    fn refresh_dynamic(&mut self) {
        if self.sources.is_empty() {
            return;
        }
        for d in &self.dynamic {
            let lo = d.offset as usize;
            let hi = (d.offset as usize + d.len as usize).min(self.regs.len());
            for i in lo..hi {
                self.regs[i] = (d.fill)(&self.sources[d.source], i - lo);
            }
        }
    }

    /// 读当前寄存器值（不推进指针）——观测辅助。
    pub fn peek(&self, offset: u8) -> Option<u8> {
        self.regs.get(offset as usize).copied()
    }

    /// 数据源引用（观测/模型装配用）。
    pub fn source(&self, idx: usize) -> Option<&DataSource> {
        self.sources.get(idx)
    }
}

impl VirtualI2cSlave for RegFileSlave {
    fn name(&self) -> &str {
        &self.name
    }

    fn addr7(&self) -> u8 {
        self.addr7
    }

    fn on_start(&mut self, dir: I2cDir) {
        // 写事务：首写字节 = 寄存器地址（指针复位）；读事务：保留当前指针直接吐数据
        self.expect_reg = dir == I2cDir::Write;
    }

    fn on_write(&mut self, byte: u8) {
        self.n_writes += 1;
        if self.expect_reg {
            // 首字节 = 寄存器地址（写事务：寄存器指针复位）
            self.ptr = byte;
            self.expect_reg = false;
            return;
        }
        // 数据字节：写入寄存器文件并推进指针
        if (self.ptr as usize) < self.regs.len() {
            self.regs[self.ptr as usize] = byte;
        }
        self.ptr = self.ptr.wrapping_add(1);
    }

    fn read_count(&self) -> u64 {
        self.n_reads
    }

    fn on_read(&mut self) -> Option<u8> {
        if self.nack {
            return None;
        }
        self.n_reads += 1;
        // 读前刷新动态寄存器（传感器数据在读取瞬间求值）
        self.refresh_dynamic();
        let idx = self.ptr as usize;
        if idx >= self.regs.len() {
            return None;
        }
        let v = self.regs[idx];
        self.ptr = self.ptr.wrapping_add(1);
        Some(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::vperiph::data_source::{StaticBaro, StaticImu, StaticMag};
    use crate::peripheral::vperiph::models::{bmp280, mpu6050, qmc5883};

    /// 模拟固件 `i2c_write_read(addr, reg, n)`：写事务设寄存器指针 → 读事务连续读。
    fn write_read(slave: &mut dyn VirtualI2cSlave, reg: u8, n: usize) -> Vec<u8> {
        slave.on_start(I2cDir::Write);
        slave.on_write(reg);
        slave.on_start(I2cDir::Read);
        (0..n).map(|_| slave.on_read().unwrap()).collect()
    }

    #[test]
    fn mpu6050_static_hover_registers() {
        let mut s = mpu6050(StaticImu::default());
        // WHO_AM_I
        assert_eq!(s.peek(0x75), Some(0x68));
        // i2c_write_read(0x3B, 14)：accel BE i16 + temp + gyro BE i16
        let raw = write_read(&mut s, 0x3B, 14);
        // accel.z = 9.81 → raw = 16384 = 0x4000（BE 高字节在前）
        assert_eq!(&raw[4..6], &[0x40, 0x00], "accel.z 应为 +1g");
        // accel.x/y = 0
        assert_eq!(&raw[0..2], &[0x00, 0x00]);
        assert_eq!(&raw[2..4], &[0x00, 0x00]);
        // gyro 全 0
        assert_eq!(&raw[8..14], &[0u8; 6]);
    }

    #[test]
    fn mpu6050_write_pwr_mgmt() {
        let mut s = mpu6050(StaticImu::default());
        // 唤醒写：PWR_MGMT_1(0x6B) ← 0x00（写事务：寄存器地址 + 数据）
        s.on_start(I2cDir::Write);
        s.on_write(0x6B);
        s.on_write(0x00);
        assert_eq!(s.peek(0x6B), Some(0x00));
        assert_eq!(s.n_writes, 2);
    }

    #[test]
    fn bmp280_pressure_read() {
        let mut s = bmp280(StaticBaro::default());
        assert_eq!(s.peek(0xD0), Some(0x58)); // ID
        // i2c_write_read(0xF7, 6)：20bit 压力原始值（Pa<<4）
        let raw = write_read(&mut s, 0xF7, 6);
        let p20 = ((raw[0] as u32) << 16) | ((raw[1] as u32) << 8) | (raw[2] as u32);
        let p = p20 >> 4;
        assert_eq!(p, 101_325, "压力原始值应为海平面气压");
    }

    #[test]
    fn qmc5883_le_mag_read() {
        let mut s = qmc5883(StaticMag::default());
        // i2c_write_read(0x00, 6)：LE i16 三轴
        let raw = write_read(&mut s, 0x00, 6);
        // mag.x = 0.2G → raw = 0.2*32768/2 = 3276.8 → 截断 3276（LE：低字节在前）
        let x = i16::from_le_bytes([raw[0], raw[1]]);
        assert_eq!(x, 3276, "mag.x 0.2G → raw 3276");
        // mag.z = 0.4G → 6553.6 → 截断 6553
        let z = i16::from_le_bytes([raw[4], raw[5]]);
        assert_eq!(z, 6553, "mag.z 0.4G → raw 6553");
    }

    #[test]
    fn nack_injection_returns_none() {
        let mut s = mpu6050(StaticImu::default());
        s.on_start(I2cDir::Write);
        s.on_write(0x3B);
        s.on_start(I2cDir::Read);
        s.nack = true; // 故障注入：断线
        assert!(s.on_read().is_none());
    }

    #[test]
    fn read_preserves_pointer_across_transactions() {
        // 真实硬件：寄存器指针跨事务保持（写设指针 → 读直接从指针吐）
        let mut s = bmp280(StaticBaro::default());
        s.on_start(I2cDir::Write);
        s.on_write(0xF7);
        // 读事务直接读，无需再写寄存器地址
        s.on_start(I2cDir::Read);
        let first = s.on_read().unwrap();
        let p20 = ((first as u32) << 16) | ((s.on_read().unwrap() as u32) << 8) | (s.on_read().unwrap() as u32);
        assert_eq!(p20 >> 4, 101_325);
    }
}
