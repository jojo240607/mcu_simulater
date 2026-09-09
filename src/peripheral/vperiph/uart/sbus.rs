//! SBUS 遥控从设备：按帧周期推 25 字节 SBUS 帧（16×11bit 通道，LSB 位流）。
//!
//! 数据源：`StaticSbus`（通道中位 1500，ch2=1000 油门最小）。

use super::super::data_source::{DataSource, SensorModel};
use super::VirtualUartSlave;

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::peripheral::vperiph::data_source::StaticSbus;

    #[test]
    fn sbus_frame_roundtrip() {
        // 用 StaticSbus 生成帧 → 按固件 sbus 驱动解码逻辑还原通道值
        let mut sbus = Sbus::new(StaticSbus::default());
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
