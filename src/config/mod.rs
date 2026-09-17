//! 虚拟外设拓扑：TOML 文件加载 + 模型工厂 + 装配到 Machine。
//!
//! 设计（与用户确认）：用 TOML 拓扑文件描述「哪个虚拟从设备挂哪条总线/端口、
//! 用哪个物理模型」，仿真平台按文件装配，无需改代码即可换场景。
//!
//! 语法示例（`examples/topology_flyctrl.toml`）：
//! ```toml
//! # 虚拟外设拓扑：总线协议级从设备装配
//! [[i2c_slave]]
//! port = 1
//! name = "mpu6050"      # 观测名（日志/断言）
//! addr = 0x68           # I2C 7bit 地址
//! model = "mpu6050"     # 模型名（models::mpu6050 等）
//! source = "imu"        # 物理模型名（StaticImu 等）
//!
//! [[uart_slave]]
//! port = 2
//! name = "gps"
//! model = "nmea_gga"    # $GNGGA 推流
//! source = "gps"
//! ```
//!
//! 说明：offline 环境无 `toml` crate，故内置一个覆盖上述子集的极简 TOML 解析器
//! （数组表 + 字符串/整数/浮点/十六进制；不支持嵌套内联表/多行字符串等超集）。

use crate::machine::Machine;

/// 极简 TOML 标量值（拓扑子集）。
#[derive(Debug, Clone, PartialEq)]
pub enum TomlValue {
    Str(String),
    Int(i64),
    Float(f64),
    Bool(bool),
}

impl TomlValue {
    /// 取字符串；非字符串/缺失返回 None。
    pub fn as_str(&self) -> Option<&str> {
        match self {
            TomlValue::Str(s) => Some(s),
            _ => None,
        }
    }
    /// 取整数（含 0x 十六进制）；非整数返回 None。
    pub fn as_int(&self) -> Option<i64> {
        match self {
            TomlValue::Int(v) => Some(*v),
            _ => None,
        }
    }
    /// 取浮点（整数可作浮点用）。
    pub fn as_float(&self) -> Option<f64> {
        match self {
            TomlValue::Float(v) => Some(*v),
            TomlValue::Int(v) => Some(*v as f64),
            _ => None,
        }
    }
    /// 取布尔。
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            TomlValue::Bool(v) => Some(*v),
            _ => None,
        }
    }
}

/// 一个 TOML 段（数组表或普通表）：`[name]` / `[[name]]` + 键值。
#[derive(Debug, Clone, Default)]
pub struct TomlSection {
    pub name: String,
    /// 是否为数组表（`[[...]]`）
    pub is_array: bool,
    pub fields: Vec<(String, TomlValue)>,
}

impl TomlSection {
    /// 取字段（缺省返回 None）。
    pub fn get(&self, key: &str) -> Option<&TomlValue> {
        self.fields.iter().find(|(k, _)| k == key).map(|(_, v)| v)
    }
    /// 取字符串字段；缺失/类型不符返回 `def`。
    pub fn str_or(&self, key: &str, def: &str) -> String {
        self.get(key)
            .and_then(TomlValue::as_str)
            .map(|s| s.to_string())
            .unwrap_or_else(|| def.to_string())
    }
    /// 取整数端口（1 基）；缺失默认 `def`。
    pub fn port_or(&self, def: u8) -> u8 {
        self.get("port")
            .and_then(TomlValue::as_int)
            .map(|v| v as u8)
            .unwrap_or(def)
    }
}

/// 极简 TOML 解析：按行解析出段列表（覆盖拓扑子集）。
///
/// 不支持：嵌套内联表（`{...}`）、多行字符串、转义（`\n` 等）、裸字符串外的数组。
pub fn parse_toml(src: &str) -> Result<Vec<TomlSection>, String> {
    let mut sections: Vec<TomlSection> = Vec::new();
    let mut cur: Option<TomlSection> = None;
    for (ln, raw) in src.lines().enumerate() {
        let line = strip_comment(raw).trim();
        if line.is_empty() {
            continue;
        }
        // 段头：[[name]] 或 [name]
        if line.starts_with("[[") {
            push_cur(&mut sections, &mut cur);
            let inner = line.trim_start_matches("[[").trim_end_matches("]]").trim();
            if inner.is_empty() {
                return Err(format!("line {}: 空数组表名", ln + 1));
            }
            cur = Some(TomlSection { name: inner.to_string(), is_array: true, fields: Vec::new() });
            continue;
        }
        if line.starts_with('[') {
            push_cur(&mut sections, &mut cur);
            let inner = line.trim_start_matches('[').trim_end_matches(']').trim();
            cur = Some(TomlSection { name: inner.to_string(), is_array: false, fields: Vec::new() });
            continue;
        }
        // 键值
        let Some(eq) = line.find('=') else {
            return Err(format!("line {}: 期望 `key = value`，得 `{line}`", ln + 1));
        };
        let key = line[..eq].trim().to_string();
        let val = parse_value(line[eq + 1..].trim())
            .map_err(|e| format!("line {}: {e}", ln + 1))?;
        let sec = cur
            .as_mut()
            .ok_or_else(|| format!("line {}: 键 `{key}` 未在任何段内", ln + 1))?;
        sec.fields.push((key, val));
    }
    push_cur(&mut sections, &mut cur);
    Ok(sections)
}

fn push_cur(sections: &mut Vec<TomlSection>, cur: &mut Option<TomlSection>) {
    if let Some(sec) = cur.take() {
        sections.push(sec);
    }
}

fn strip_comment(line: &str) -> &str {
    match line.find('#') {
        Some(i) => &line[..i],
        None => line,
    }
}

/// 解析 TOML 标量（拓扑子集）：字符串 / 整数(含 0x) / 浮点 / 布尔。
fn parse_value(s: &str) -> Result<TomlValue, String> {
    let s = s.trim();
    if s.is_empty() {
        return Err("空值".into());
    }
    // 字符串
    if s.starts_with('"') {
        if !s.ends_with('"') || s.len() < 2 {
            return Err(format!("未闭合字符串: {s}"));
        }
        return Ok(TomlValue::Str(s[1..s.len() - 1].to_string()));
    }
    // 布尔
    if s == "true" {
        return Ok(TomlValue::Bool(true));
    }
    if s == "false" {
        return Ok(TomlValue::Bool(false));
    }
    // 整数（十进制/十六进制）
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        return i64::from_str_radix(hex, 16)
            .map(TomlValue::Int)
            .map_err(|_| format!("非法十六进制: {s}"));
    }
    if let Ok(v) = s.parse::<i64>() {
        return Ok(TomlValue::Int(v));
    }
    // 浮点
    if let Ok(v) = s.parse::<f64>() {
        return Ok(TomlValue::Float(v));
    }
    // 数组 [..] / 内联表 {..} —— 拓扑子集暂不支持，报清晰错误
    Err(format!("不支持的标量: {s}（拓扑子集仅支持字符串/整数/浮点/布尔）"))
}

/// 拓扑节点：一条虚拟从设备装配指令。
#[derive(Debug, Clone)]
pub struct TopologyNode {
    /// 总线类型：`i2c` / `uart` / `spi`
    pub bus: String,
    /// 端口（1 基；I2C 1..3、SPI 1..3、USART 1..6）
    pub port: u8,
    /// 从设备名（观测/断言）
    pub name: String,
    /// 模型名（工厂分派）
    pub model: String,
    /// 数据源名（工厂分派）
    pub source: String,
    /// I2C 从设备地址（7bit；uart/spi 忽略）
    pub addr: Option<u8>,
}

/// 解析拓扑文件并装配到 Machine。
///
/// 失败时返回含行号/段的错误（便于排查拓扑写错）。
pub fn apply_topology(m: &Machine, toml_src: &str) -> Result<Vec<TopologyNode>, String> {
    let sections = parse_toml(toml_src)?;
    let mut nodes = Vec::new();
    for sec in &sections {
        match sec.name.as_str() {
            "i2c_slave" => {
                let node = TopologyNode {
                    bus: "i2c".into(),
                    port: sec.port_or(1),
                    name: sec.str_or("name", "i2c_slave"),
                    model: sec.str_or("model", ""),
                    source: sec.str_or("source", ""),
                    addr: sec.get("addr").and_then(TomlValue::as_int).map(|v| v as u8),
                };
                if node.model.is_empty() || node.source.is_empty() {
                    return Err(format!("[i2c_slave] `{}` 缺 model/source", node.name));
                }
                let slave = build_i2c_slave(&node)?;
                m.register_i2c_slave(node.port, slave);
                nodes.push(node);
            }
            "uart_slave" => {
                let node = TopologyNode {
                    bus: "uart".into(),
                    port: sec.port_or(1),
                    name: sec.str_or("name", "uart_slave"),
                    model: sec.str_or("model", ""),
                    source: sec.str_or("source", ""),
                    addr: None,
                };
                if node.model.is_empty() || node.source.is_empty() {
                    return Err(format!("[uart_slave] `{}` 缺 model/source", node.name));
                }
                let slave = build_uart_slave(&node)?;
                m.register_uart_slave(node.port, slave);
                nodes.push(node);
            }
            "spi_slave" => {
                let node = TopologyNode {
                    bus: "spi".into(),
                    port: sec.port_or(1),
                    name: sec.str_or("name", "spi_slave"),
                    model: sec.str_or("model", ""),
                    source: sec.str_or("source", ""),
                    addr: None,
                };
                if node.model.is_empty() || node.source.is_empty() {
                    return Err(format!("[spi_slave] `{}` 缺 model/source", node.name));
                }
                let slave = build_spi_slave(&node)?;
                m.register_spi_slave(node.port, slave);
                nodes.push(node);
            }
            other => {
                return Err(format!(
                    "未知段 `[{other}]`（支持 i2c_slave / uart_slave / spi_slave）"
                ));
            }
        }
    }
    if nodes.is_empty() {
        return Err("拓扑文件为空：未装配任何虚拟从设备".into());
    }
    Ok(nodes)
}

/// I2C 从设备模型工厂（model + source → RegFileSlave）。
fn build_i2c_slave(
    node: &TopologyNode,
) -> Result<Box<dyn crate::peripheral::vperiph::VirtualI2cSlave>, String> {
    use crate::peripheral::vperiph::data_source::{StaticBaro, StaticImu, StaticMag};
    use crate::peripheral::vperiph::i2c::{bmp280, mpu6050, qmc5883};
    let boxed: Box<dyn crate::peripheral::vperiph::VirtualI2cSlave> = match (
        node.model.as_str(),
        node.source.as_str(),
    ) {
        ("mpu6050", "imu") => Box::new(mpu6050(StaticImu::default())),
        ("bmp280", "baro") => Box::new(bmp280(StaticBaro::default())),
        ("qmc5883", "mag") => Box::new(qmc5883(StaticMag::default())),
        (m, s) => {
            return Err(format!(
                "[i2c_slave] `{}` 不支持的 model/source 组合: ({m}, {s})（支持 mpu6050/imu、bmp280/baro、qmc5883/mag）",
                node.name
            ))
        }
    };
    Ok(boxed)
}

/// SPI 从设备模型工厂（model + source → VirtualSpiSlave）。
///
/// bmi088 的片选固定为板级 GPIOE_7(ACCEL_CS)/GPIOE_8(GYRO_CS)（4,7）/(4,8)，
/// 与固件 bmi088 设备（"spi2"=SPI3 外设 + 双片选）一致。
fn build_spi_slave(
    node: &TopologyNode,
) -> Result<Box<dyn crate::peripheral::vperiph::spi::VirtualSpiSlave>, String> {
    use crate::peripheral::vperiph::data_source::StaticImu;
    use crate::peripheral::vperiph::spi::default_bmi088;
    let boxed: Box<dyn crate::peripheral::vperiph::spi::VirtualSpiSlave> = match (
        node.model.as_str(),
        node.source.as_str(),
    ) {
        ("bmi088", "imu") => Box::new(default_bmi088((4, 7), (4, 8)).with_source(StaticImu::default())),
        (m, s) => {
            return Err(format!(
                "[spi_slave] `{}` 不支持的 model/source 组合: ({m}, {s})（支持 bmi088/imu）",
                node.name
            ))
        }
    };
    Ok(boxed)
}

/// UART 推流从设备模型工厂（model + source → VirtualUartSlave）。
fn build_uart_slave(
    node: &TopologyNode,
) -> Result<Box<dyn crate::peripheral::vperiph::uart::VirtualUartSlave>, String> {
    use crate::peripheral::vperiph::data_source::{StaticGps, StaticSbus};
    use crate::peripheral::vperiph::uart::{NmeaGps, Sbus};
    let boxed: Box<dyn crate::peripheral::vperiph::uart::VirtualUartSlave> = match (
        node.model.as_str(),
        node.source.as_str(),
    ) {
        ("nmea_gga", "gps") => Box::new(NmeaGps::new(StaticGps::default())),
        ("sbus", "sbus") => Box::new(Sbus::new(StaticSbus::default())),
        (m, s) => {
            return Err(format!(
                "[uart_slave] `{}` 不支持的 model/source 组合: ({m}, {s})（支持 nmea_gga/gps、sbus/sbus）",
                node.name
            ))
        }
    };
    Ok(boxed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_simple_topology() {
        let src = r#"
# 悬停场景拓扑
[[i2c_slave]]
port = 1
name = "mpu6050"
addr = 0x68
model = "mpu6050"
source = "imu"

[[uart_slave]]
port = 2
name = "gps"
model = "nmea_gga"
source = "gps"
"#;
        let secs = parse_toml(src).unwrap();
        assert_eq!(secs.len(), 2);
        assert_eq!(secs[0].name, "i2c_slave");
        assert!(secs[0].is_array);
        assert_eq!(secs[0].str_or("name", ""), "mpu6050");
        assert_eq!(secs[0].get("addr").unwrap().as_int(), Some(0x68));
        assert_eq!(secs[0].port_or(0), 1);
        assert_eq!(secs[1].name, "uart_slave");
        assert_eq!(secs[1].str_or("source", ""), "gps");
    }

    #[test]
    fn parse_hex_and_float() {
        let secs = parse_toml("[[a]]\nx = 0x1F\ny = 9.81\nz = -3\nb = true\ns = \"hi\"\n")
            .unwrap();
        assert_eq!(secs[0].get("x").unwrap().as_int(), Some(31));
        assert_eq!(secs[0].get("y").unwrap().as_float(), Some(9.81));
        assert_eq!(secs[0].get("z").unwrap().as_int(), Some(-3));
        assert_eq!(secs[0].get("b").unwrap().as_bool(), Some(true));
        assert_eq!(secs[0].get("s").unwrap().as_str(), Some("hi"));
    }

    #[test]
    fn parse_unknown_section_ok() {
        // 解析层不校验段名；apply_topology 才报错
        let err = parse_toml("[[unknown]]\nport = 1\n").unwrap();
        assert_eq!(err[0].name, "unknown");
    }

    #[test]
    fn factory_i2c_slave() {
        let node = TopologyNode {
            bus: "i2c".into(),
            port: 1,
            name: "mpu6050".into(),
            model: "mpu6050".into(),
            source: "imu".into(),
            addr: Some(0x68),
        };
        let s = build_i2c_slave(&node).unwrap();
        assert_eq!(s.name(), "mpu6050");
        assert_eq!(s.addr7(), 0x68);
    }

    #[test]
    fn factory_bad_combo() {
        let node = TopologyNode {
            bus: "i2c".into(),
            port: 1,
            name: "x".into(),
            model: "mpu6050".into(),
            source: "gps".into(),
            addr: None,
        };
        assert!(build_i2c_slave(&node).is_err());
    }
}
