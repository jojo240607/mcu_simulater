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

    /// 设置帧周期（秒）。构造函数默认 0.05s（20Hz）：u-blox 驱动 probe 每 10ms
    /// 读 1 字节（300ms 窗口读不完 70 字节帧），20Hz 推流让 drain（2ms/字节）
    /// 在测试窗口内读到完整 GGA 建立定位。
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

    /// 生成一条 `$GNRMC` 帧字节（NMEA-0183 推荐最小导航信息，含地速/航向）。
    ///
    /// 格式：`$GNRMC,hhmmss,A,ddmm.mmmm,N,dddmm.mmmm,E,speed,course,ddmmyy,,,D*cs`
    /// - speed：地速（节，knots = m/s × 1.94384）
    /// - course：对地航向（真北顺时针，度；0=北 90=东）
    ///
    /// 速度来源：数据源的 `vel_n`/`vel_e`（NED 北/东向 m/s）。固件 `GpsUblox`
    /// 解析 RMC 得 Doppler 速度 → `PosSample::with_vel` → EKF `update_vel`，
    /// 约束水平速度估计（无此约束时长时间悬停水平速度纯积分漂移失稳）。
    /// status 随 fix：fix=0（失锁）→ 'V'（Void），与真实 NMEA 一致。曾硬编码 'A'
    /// → GpsDrop 时 GGA quality=0 置无效、但 RMC 仍有效，固件 drain 取最后一行
    /// 为有效 → gps 恒 Some → FDIR 永不降级（虚拟外设实测 gps_drop 测试不触发）。
    fn build_rmc(&self, v: &dyn SensorModel) -> Vec<u8> {
        let lat = v.value("lat"); // 度（北正）
        let lon = v.value("lon"); // 度（东正）
        let vn = v.value("vel_n"); // m/s（北）
        let ve = v.value("vel_e"); // m/s（东）
        let lat_abs = lat.abs();
        let lat_deg = lat_abs as u32;
        let lat_min = (lat_abs - lat_deg as f32) * 60.0;
        let lon_abs = lon.abs();
        let lon_deg = lon_abs as u32;
        let lon_min = (lon_abs - lon_deg as f32) * 60.0;
        let ns = if lat >= 0.0 { 'N' } else { 'S' };
        let ew = if lon >= 0.0 { 'E' } else { 'W' };
        // NED 北/东速度 → 地速（节）+ 航向（真北顺时针）
        let speed_knots = (vn * vn + ve * ve).sqrt() * 1.943_84;
        let course_deg = ve.atan2(vn).to_degrees().rem_euclid(360.0);
        let fix = v.value("fix");
        let status = if fix > 0.0 { 'A' } else { 'V' };
        let body = format!(
            "GNRMC,{:06},{},{:02}{:07.4},{},{:03}{:07.4},{},{:.1},{:.1},010100,,,D",
            120000u32, status,
            lat_deg,
            lat_min,
            ns,
            lon_deg,
            lon_min,
            ew,
            speed_knots,
            course_deg,
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
                // 每周期推 GGA（位置/高度）+ RMC（位置/速度）两条：
                // 固件 u-blox 驱动 drain 逐行解析，GGA 建定位锁 NED 原点，
                // RMC 提供 Doppler 速度供 EKF update_vel（水平速度约束）。
                // 【注】曾实验 GGA-only（禁 RMC）——正确 baro 下 t≈32s 仍失稳，
                // 证实水平失稳根因不在 RMC 注入（见 x_hover_demo 注释）。
                for b in self.build_gga(m.as_ref()) {
                    tx(b);
                }
                for b in self.build_rmc(m.as_ref()) {
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

    #[test]
    fn rmc_frame_format_and_checksum() {
        let mut gps = NmeaGps::new(StaticGps::default());
        if let DataSource::Math(m) = &gps.source {
            let frame = gps.build_rmc(m.as_ref());
            let s = String::from_utf8_lossy(&frame);
            eprintln!("RMC frame: {s:?}");
            assert!(s.starts_with("$GNRMC,"), "应以 $GNRMC 开头: {s:?}");
            assert!(s.ends_with("\r\n"), "应以 CRLF 结尾: {s:?}");
            let star = s.find('*').expect("应有 *");
            let body = &s[1..star];
            let cs = nmea_cs(body.as_bytes());
            let exp = u8::from_str_radix(&s[star + 1..star + 3], 16).unwrap();
            assert_eq!(cs, exp, "校验和不符: body={body}");
            // 字段：status=A、lat/lon、speed/course（StaticGps vel=0 → speed=0 course=0）
            let f: Vec<&str> = s.split(',').collect();
            assert_eq!(f[1], "120000", "UTC 时间");
            assert_eq!(f[2], "A", "status");
            assert_eq!(f[3], "3113.8240", "纬度 ddmm.mmmm");
            assert_eq!(f[4], "N");
            assert_eq!(f[7], "0.0", "地速=0（静止）");
            assert_eq!(f[8], "0.0", "航向=0（北）");
        } else {
            panic!("source 应为 Math");
        }
    }

    #[test]
    fn rmc_velocity_from_vel_fields() {
        // vel_n=5 m/s、vel_e=0 → speed≈9.72 节、course=0（北）
        let src = StaticGps { lat: 31.2304, lon: 121.4737, alt: 4.0, vel: [5.0, 0.0, 0.0] };
        let mut gps = NmeaGps::new(src);
        if let DataSource::Math(m) = &gps.source {
            let frame = gps.build_rmc(m.as_ref());
            let s = String::from_utf8_lossy(&frame).into_owned();
            let f: Vec<&str> = s.split(',').collect();
            let speed: f32 = f[7].parse().unwrap();
            let course: f32 = f[8].parse().unwrap();
            assert!((speed - 9.72).abs() < 0.1, "speed={speed} 应≈9.72节");
            assert!((course - 0.0).abs() < 0.1, "course={course} 应=0（北向）");
        } else {
            panic!("source 应为 Math");
        }
    }
}
