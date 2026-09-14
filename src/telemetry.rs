//! 波形/遥测时间线导出（调试平台 P2-1）。
//!
//! 目标：调试飞控固件时把关键变量**随时间的变化**导出为 CSV，供外部工具
//! （pyplot/Excel/时序分析）可视化——替代"打印单点值"看不到趋势的问题。
//!
//! 观测点 = 内存地址 + 解码类型（u8/u16/u32/f32），典型是固件诊断共享内存
//! （如 `0x2000_9074+28` = EKF 高度 f32）。按固定**虚拟时间间隔**采样，
//! 时间基准 = retired 指令数（与 fault/trace 同口径，46K 退休/虚拟 ms）。
//!
//! 用法：
//! ```rust
//! let mut tel = Telemetry::new(1_000); // 每 1K 退休字节采样一次
//! tel.add_watch("ekf_z", 0x2000_9074 + 28, WatchType::F32);
//! tel.add_watch("healthy", 0x2000_9010, WatchType::U8);
//! machine.attach_telemetry(tel);
//! // ... run() 自动采样 ...
//! let csv = machine.telemetry_csv();
//! ```

use crate::core::Cpu;

/// 观测点解码类型。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchType {
    U8,
    U16,
    U32,
    F32,
}

/// 观测点：名称 + 内存地址 + 解码类型。
#[derive(Debug, Clone)]
pub struct Watch {
    pub name: String,
    pub addr: u32,
    pub ty: WatchType,
}

/// 单行采样。
#[derive(Debug, Clone)]
pub struct TelemetryRow {
    /// 采样点退休指令数（时间基准）。
    pub retired: u64,
    /// 各观测点值（顺序与 watches 一致）。
    pub values: Vec<f64>,
}

/// 遥测记录器：按退休间隔采样观测点。
pub struct Telemetry {
    watches: Vec<Watch>,
    /// 采样间隔（退休字节；与 run() 预算同单位）。
    period_retired: u64,
    last_sample: u64,
    rows: Vec<TelemetryRow>,
}

impl Telemetry {
    pub fn new(period_retired: u64) -> Self {
        Self {
            watches: Vec::new(),
            period_retired: period_retired.max(1),
            last_sample: 0,
            rows: Vec::new(),
        }
    }

    pub fn add_watch(&mut self, name: &str, addr: u32, ty: WatchType) -> &mut Self {
        self.watches.push(Watch {
            name: name.to_string(),
            addr,
            ty,
        });
        self
    }

    pub fn watches(&self) -> &[Watch] {
        &self.watches
    }

    /// 采样周期（退休字节）。
    pub fn period_retired(&self) -> u64 {
        self.period_retired
    }

    pub fn rows(&self) -> &[TelemetryRow] {
        &self.rows
    }

    pub fn row_count(&self) -> usize {
        self.rows.len()
    }

    /// 采样（run() 段后调用）：到间隔则读所有观测点。
    ///
    /// 单段可能跨越多个采样周期（无中断固件一个大预算 run 只停一次），
    /// 此时**补采**多行：时间戳按周期推进，值取当前读到的内存值（近似——
    /// 中间时刻的值不可恢复；采样周期足够小时误差可忽略，趋势正确）。
    pub fn sample(&mut self, cpu: &mut Cpu, retired: u64) {
        if retired.saturating_sub(self.last_sample) < self.period_retired {
            return;
        }
        let n = retired.saturating_sub(self.last_sample) / self.period_retired;
        for k in 0..n {
            let t = self.last_sample + (k + 1) * self.period_retired;
            let mut values = Vec::with_capacity(self.watches.len());
            for w in &self.watches {
                values.push(read_watch(cpu, w));
            }
            self.rows.push(TelemetryRow { retired: t, values });
        }
        self.last_sample = retired;
    }

    /// 导出 CSV（时间列 = 虚拟微秒，46K 退休/ms 口径，与 trace 一致）。
    pub fn to_csv(&self) -> String {
        let mut s = String::new();
        // 头
        s.push_str("time_us");
        for w in &self.watches {
            s.push(',');
            s.push_str(&w.name);
        }
        s.push('\n');
        // 行
        for r in &self.rows {
            s.push_str(&format!("{:.3}", r.retired as f64 / 46.0e3));
            for v in &r.values {
                s.push(',');
                s.push_str(&format!("{v:.6}"));
            }
            s.push('\n');
        }
        s
    }

    /// 写 CSV 到文件。
    pub fn write_csv(&self, path: &std::path::Path) -> std::io::Result<()> {
        std::fs::write(path, self.to_csv())
    }
}

/// 按类型读内存并解码。
fn read_watch(cpu: &mut Cpu, w: &Watch) -> f64 {
    let buf = match cpu.mem_read(w.addr as u64, size_of(w.ty)) {
        Ok(b) => b,
        Err(_) => return f64::NAN, // 未映射/不可读：NaN 标记（CSV 中可见断线）
    };
    match w.ty {
        WatchType::U8 => buf[0] as f64,
        WatchType::U16 => u16::from_le_bytes([buf[0], buf[1]]) as f64,
        WatchType::U32 => u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as f64,
        WatchType::F32 => f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as f64,
    }
}

fn size_of(ty: WatchType) -> usize {
    match ty {
        WatchType::U8 => 1,
        WatchType::U16 => 2,
        WatchType::U32 | WatchType::F32 => 4,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::machine::Machine;

    fn machine() -> Machine {
        let mut m = Machine::new_m4f().unwrap();
        m.map_stm32f407_layout().unwrap();
        m
    }

    #[test]
    fn samples_at_period_and_decodes_types() {
        let mut m = machine();
        // 写各类观测值到 RAM
        m.cpu.mem_write(0x2000_0000, &[0x2A]).unwrap(); // u8 = 42
        m.cpu.mem_write(0x2000_0004, &0x1234u16.to_le_bytes()).unwrap();
        m.cpu.mem_write(0x2000_0008, &0xDEADBEEFu32.to_le_bytes()).unwrap();
        m.cpu.mem_write(0x2000_000C, &1.5f32.to_le_bytes()).unwrap();

        let mut tel = Telemetry::new(1000);
        tel.add_watch("u8", 0x2000_0000, WatchType::U8)
            .add_watch("u16", 0x2000_0004, WatchType::U16)
            .add_watch("u32", 0x2000_0008, WatchType::U32)
            .add_watch("f32", 0x2000_000C, WatchType::F32);

        // 未到间隔不采样
        tel.sample(&mut m.cpu, 500);
        assert_eq!(tel.row_count(), 0);
        // 到间隔采样
        tel.sample(&mut m.cpu, 1000);
        assert_eq!(tel.row_count(), 1);
        let r = &tel.rows[0];
        assert_eq!(r.values, vec![42.0, 0x1234u16 as f64, 0xDEADBEEFu32 as f64, 1.5]);
    }

    #[test]
    fn csv_export_shape() {
        let mut m = machine();
        m.cpu.mem_write(0x2000_0000, &7.0f32.to_le_bytes()).unwrap();
        let mut tel = Telemetry::new(100);
        tel.add_watch("v", 0x2000_0000, WatchType::F32);
        tel.sample(&mut m.cpu, 100);
        tel.sample(&mut m.cpu, 200);
        let csv = tel.to_csv();
        let lines: Vec<&str> = csv.lines().collect();
        assert_eq!(lines[0], "time_us,v");
        assert_eq!(lines.len(), 3, "1 头 + 2 行：\n{csv}");
        assert!(lines[1].contains("7.000000"), "值应解码为 7.0：{csv}");
    }

    #[test]
    fn unmapped_watch_yields_nan() {
        let mut m = machine();
        let mut tel = Telemetry::new(10);
        tel.add_watch("hole", 0xE000_0000, WatchType::F32); // 真未映射（SCB 在 0xE000E000）
        tel.sample(&mut m.cpu, 10);
        assert!(tel.rows[0].values[0].is_nan(), "未映射读应得 NaN");
    }
}
