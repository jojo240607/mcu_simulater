//! UART 总线虚拟从设备：主动推流（GPS NMEA / SBUS 遥控帧）。
//!
//! 由 [`crate::peripheral::usart::Usart`] 持有、Machine 仿真循环按迭代推进：
//! `step(dt)` 累积帧节拍，到期时经 `tx` 回调把帧字节逐个喂入
//! [`Usart::feed_rx`]（复用现有 RXNE/DMA/中断链路，固件驱动零改动读取）。

use super::data_source::{DataSource, SensorModel};

/// UART 总线从设备接口。
///
/// `tx` 回调把一字节喂入对应 UART 的接收缓冲（模拟器内部 feed_rx）。
pub trait VirtualUartSlave: Send + Sync {
    /// 从设备名（观测/日志）
    fn name(&self) -> &str;

    /// 推进推流节拍（`dt` 秒）；到期推帧经 `tx` 喂入。
    fn step(&mut self, dt: f32, tx: &mut dyn FnMut(u8));

    /// 已推帧数（观测/断言：推流是否在跑）。
    fn frames(&self) -> u64 {
        0
    }
}

/// NMEA GPS 从设备：按帧周期推 `$GNGGA` 帧（含位置/速度/定位质量）。
pub struct NmeaGps {
    source: DataSource,
    /// 帧周期（秒）
    period: f32,
    /// 已累积时间
    acc: f32,
    /// 诊断：已推帧数
    pub frames: u64,
    /// 手动静默（故障注入：GPS 断流 → 固件无新定位）
    pub silent: bool,
}

impl NmeaGps {
    pub fn new(source: impl SensorModel + 'static) -> Self {
        Self {
            source: DataSource::Math(Box::new(source)),
            period: 0.05,
            acc: 0.0,
            frames: 0,
            silent: false,
        }
    }

    /// 帧周期默认 0.2s（5Hz）：u-blox 驱动 probe 每 10ms 读 1 字节（300ms 窗口
    /// 读不完 70 字节帧），提高推流频率让 drain（2ms/字节）在测试窗口内读到
    /// 完整 GGA 建立定位。
    pub fn set_period(&mut self, period: f32) {
        self.period = period;
    }

    /// 生成一条 `$GNGGA` 帧字节（NMEA-0183）。
    ///
    /// 格式：`$GNGGA,hhmmss,ddmm.mmmm,N,dddmm.mmmm,E,quality,numSat,hdop,alt,M,sep,M,,*cs\r\n`
    fn build_gga(&self, v: &dyn SensorModel) -> Vec<u8> {
        let lat = v.value("lat"); // 度（北正）
        let lon = v.value("lon"); // 度（东正）
        let alt = v.value("alt"); // 米
        let fix = v.value("fix") as u8; // 0/1/2/3
        let lat_abs = lat.abs();
        let lat_deg = lat_abs as u32;
        let lat_min = (lat_abs - lat_deg as f32) * 60.0;
        let lon_abs = lon.abs();
        let lon_deg = lon_abs as u32;
        let lon_min = (lon_abs - lon_deg as f32) * 60.0;
        let ns = if lat >= 0.0 { 'N' } else { 'S' };
        let ew = if lon >= 0.0 { 'E' } else { 'W' };
        let body = format!(
            "GNGGA,{:06},{:02}{:07.4},{},{:03}{:07.4},{},{},06,1.0,{:.1},M,0.0,M,,",
            120000u32,
            lat_deg,
            lat_min,
            ns,
            lon_deg,
            lon_min,
            ew,
            if fix > 0 { fix } else { 0 },
            alt,
        );
        let mut line = format!("${body}");
        let cs = nmea_checksum(body.as_bytes());
        line.push_str(&format!("*{:02X}\r\n", cs));
        line.into_bytes()
    }
}

impl VirtualUartSlave for NmeaGps {
    fn name(&self) -> &str {
        "ublox_gps"
    }

    fn frames(&self) -> u64 {
        self.frames
    }

    fn step(&mut self, dt: f32, tx: &mut dyn FnMut(u8)) {
        if self.silent {
            return;
        }
        self.source.step(dt);
        self.acc += dt;
        if self.acc >= self.period {
            self.acc -= self.period;
            self.frames += 1;
            if let DataSource::Math(m) = &self.source {
                let frame = self.build_gga(m.as_ref());
                for b in frame {
                    tx(b);
                }
            }
        }
    }
}

/// SBUS 遥控从设备：按帧周期推 25 字节 SBUS 帧（16×11bit 通道，LSB 位流）。
pub struct Sbus {
    source: DataSource,
    /// 帧周期（秒；SBUS ~100Hz）
    period: f32,
    acc: f32,
    pub frames: u64,
    pub silent: bool,
}

impl Sbus {
    pub fn new(source: impl SensorModel + 'static) -> Self {
        Self {
            source: DataSource::Math(Box::new(source)),
            period: 0.05,
            acc: 0.0,
            frames: 0,
            silent: false,
        }
    }

    /// 生成一帧 SBUS：`0x0F` + 22B payload + 1B flags + `0x00`。
    ///
    /// 通道编码：1000..2000 的通道值 → 11bit 原始值
    /// （`SBUS_MID≈992 / RANGE=819.5`；norm=(raw-992)/819.5）。
    /// 编码 = 解码逆过程（LSB-first 位流）。
    fn build_frame(&self, v: &dyn SensorModel) -> Vec<u8> {
        let mut frame = vec![0x0F];
        // 通道值（1000..2000）→ 11bit 原始（992 + (ch-1500)/500*819.5）
        let ch: Vec<u32> = (0..16)
            .map(|i| {
                let c = v.value(&format!("ch{i}"));
                let raw = (992.0 + (c - 1500.0) / 500.0 * 819.5) as i32;
                raw.clamp(0, 2047) as u32
            })
            .collect();
        // 打包 22B payload（LSB-first 位流）
        let mut payload = [0u8; 22];
        let mut bits: u64 = 0;
        let mut bitcount: u32 = 0;
        let mut idx = 0usize;
        for &c in &ch {
            bits |= (c as u64) << bitcount;
            bitcount += 11;
            while bitcount >= 8 && idx < 22 {
                payload[idx] = (bits & 0xFF) as u8;
                bits >>= 8;
                bitcount -= 8;
                idx += 1;
            }
        }
        frame.extend_from_slice(&payload);
        frame.push(0x00); // flags：无失败保护/帧丢失
        frame.push(0x00); // footer
        frame
    }
}

impl VirtualUartSlave for Sbus {
    fn name(&self) -> &str {
        "sbus_rc"
    }

    fn frames(&self) -> u64 {
        self.frames
    }

    fn step(&mut self, dt: f32, tx: &mut dyn FnMut(u8)) {
        if self.silent {
            return;
        }
        self.source.step(dt);
        self.acc += dt;
        if self.acc >= self.period {
            self.acc -= self.period;
            self.frames += 1;
            if let DataSource::Math(m) = &self.source {
                let frame = self.build_frame(m.as_ref());
                for b in frame {
                    tx(b);
                }
            }
        }
    }
}

// ---- NMEA 工具 ----

/// NMEA 校验和（`$` 与 `*` 之间所有字节异或）。
fn nmea_checksum(body: &[u8]) -> u8 {
    body.iter().fold(0u8, |a, &b| a ^ b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::vperiph::data_source::StaticGps;

    fn nmea_cs(body: &[u8]) -> u8 {
        body.iter().fold(0u8, |a, &b| a ^ b)
    }

    #[test]
    fn gga_frame_format_and_checksum() {
        let mut gps = NmeaGps::new(StaticGps::default());
        if let DataSource::Math(m) = &gps.source {
            let frame = gps.build_gga(m.as_ref());
            let s = String::from_utf8_lossy(&frame);
            eprintln!("GGA frame: {s:?}");
            assert!(s.starts_with("$GNGGA,"), "应以 $GNGGA 开头: {s:?}");
            assert!(s.ends_with("\r\n"), "应以 CRLF 结尾: {s:?}");
            // 校验和：$ 与 * 之间
            let star = s.find('*').expect("应有 *");
            let body = &s[1..star];
            let cs = nmea_cs(body.as_bytes());
            let exp = u8::from_str_radix(&s[star + 1..star + 3], 16).unwrap();
            assert_eq!(cs, exp, "校验和不符: body={body}");
            // 字段：lat/lon/alt/fix（StaticGps: 31.2304, 121.4737, 4m, fix=3）
            let f: Vec<&str> = s.split(',').collect();
            assert_eq!(f[1], "120000", "UTC 时间");
            assert_eq!(f[2], "3113.8240", "纬度 ddmm.mmmm");
            assert_eq!(f[3], "N");
            assert_eq!(f[6], "3", "fix=3");
            assert_eq!(f[9], "4.0", "alt=4.0m");
        } else {
            panic!("source 应为 Math");
        }
    }

    #[test]
    fn sbus_frame_roundtrip() {
        // 用 StaticSbus 生成帧 → 按固件 sbus 驱动解码逻辑还原通道值
        let mut sbus = Sbus::new(crate::peripheral::vperiph::data_source::StaticSbus::default());
        if let DataSource::Math(m) = &sbus.source {
            let frame = sbus.build_frame(m.as_ref());
            assert_eq!(frame.len(), 25, "SBUS 帧 = 25 字节");
            assert_eq!(frame[0], 0x0F, "头");
            assert_eq!(frame[24], 0x00, "尾");
            // 解码（与 flyctrl sbus.rs decode 相同：LSB-first 位流）
            let p = &frame[1..23];
            let mut bits: u32 = 0;
            let mut bitcount = 0u32;
            let mut ch = [0u16; 16];
            let mut idx = 0usize;
            for &byte in p {
                bits |= (byte as u32) << bitcount;
                bitcount += 8;
                while bitcount >= 11 && idx < 16 {
                    ch[idx] = (bits & 0x7FF) as u16;
                    bits >>= 11;
                    bitcount -= 11;
                    idx += 1;
                }
            }
            // StaticSbus：ch0..15 默认 1500，ch2=1000
            // ch=1500 → raw=992+0=992 → norm 0；ch=1000 → raw=172 → norm -1.0
            let norm = |v: u16| (v as f32 - 992.0) / 819.5;
            eprintln!("sbus ch={ch:?}");
            assert!((norm(ch[0]) - 0.0).abs() < 0.01, "ch0≈1500(norm 0)");
            assert!((norm(ch[1]) - 0.0).abs() < 0.01, "ch1≈1500(norm 0)");
            assert!((norm(ch[2]) - (-1.0)).abs() < 0.01, "ch2=1000(norm -1.0)");
        } else {
            panic!("source 应为 Math");
        }
    }
}
