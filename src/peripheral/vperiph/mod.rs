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
//! # 目录组织（一器件一文件）
//!
//! ```text
//! vperiph/
//! ├── mod.rs          总线从设备 trait + RegFileSlave（通用寄存器文件从设备）
//! ├── data_source.rs  DataSource（Const/Math）+ SensorModel + 静态物理模型
//! ├── i2c/             I2C 总线从设备器件（一器件一文件）
//! │   ├── mod.rs       子模块声明 + 工厂 re-export + default_i2c_slaves()
//! │   ├── mpu6050.rs   六轴 IMU @0x68
//! │   ├── bmp280.rs    气压 @0x76
//! │   └── qmc5883.rs   磁力 @0x0D
//! ├── spi/             SPI 总线从设备器件（一器件一文件，全双工直路由）
//! │   ├── mod.rs       VirtualSpiSlave trait + 帧工具 + re-export
//! │   └── bmi088.rs    双片选六轴 IMU（ACCEL_CS/GYRO_CS）
//! └── uart/             UART 推流从设备器件（一器件一文件）
//!     ├── mod.rs       VirtualUartSlave trait + NMEA 工具 + re-export
//!     ├── nmea_gps.rs  $GNGGA 推流 GPS
//!     └── sbus.rs      SBUS 遥控帧
//! ```
//!
//! **新增器件**：在 `i2c/`（或 `spi/`、`uart/`）下新建 `<device>.rs`（寄存器布局/
//! 帧构造 + 工厂函数 + 单元测试），在对应 `mod.rs` 加 `pub mod <device>;` + re-export；
//! 总线外设只依赖 trait，与具体器件解耦。
//!
//! 首批覆盖（对齐 flyctrl real-sensors 全链路）：
//! - I2C：mpu6050(0x68) / bmp280(0x76) / qmc5883(0x0D)
//! - SPI：bmi088（双片选六轴 IMU，可扩展任意 SPI 传感器）
//! - UART：ublox gps(usart1) / sbus(usart2)

pub mod data_source;
pub mod i2c;
pub mod can;
pub mod esc;
pub mod fsmc;
pub mod spi;
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

    /// 故障注入：手动 NACK（模拟断线/无响应 → 固件 healthy=false → FDIR 降级）。
    /// 默认无操作；`RegFileSlave` 实现按 `nack` 字段生效。
    fn set_nack(&mut self, _nack: bool) {}
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
    fn set_nack(&mut self, nack: bool) {
        self.nack = nack;
    }

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
