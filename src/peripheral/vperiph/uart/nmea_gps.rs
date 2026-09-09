//! NMEA GPS 从设备：按帧周期推 `$GNGGA` 帧（含位置/速度/定位质量）。
//!
//! 数据源：`StaticGps`（31.2304N / 121.4737E / alt 4m / fix=3）。

use super::super::data_source::{DataSource, SensorModel};
use super::{nmea_checksum, VirtualUartSlave};

/// NMEA GPS 从设备：按帧周期推 `$GNGGA` 帧。
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
}
