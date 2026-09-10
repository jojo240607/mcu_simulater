//! ESC 电调 + BLDC 无刷电机虚拟外设（开环 + 电机转速 → IMU 数据源联动接口预留）。
//!
//! # 输入信号（二选一或并存，最近一次输入类型生效）
//!
//! 1. **PWM**：MCU 用 TIM PWM 输出控制电调（传统 1~2ms 脉宽约定）。
//!    订阅 [`Event::TimPwm`]（含虚拟 tick = 退役字节计数），测相邻上升沿间
//!    的周期 tick 与高电平 tick → 占空比 → 按配置的 `pwm_period_us` 换算脉宽
//!    （1000~2000μs 映射电调量 0~2000）。速率无关：只需占空比与周期配置，
//!    不依赖绝对虚拟时钟率。
//! 2. **DShot**：MCU 用 GPIO bit-bang 发送 DShot300 数字帧（行空闲 HIGH，
//!    1 位 = 短 LOW 脉冲、0 位 = 长 LOW 脉冲，位周期恒定）。
//!    订阅 [`Event::GpioLevel`]，按**相邻事件 tick 差**测每个位的 LOW 脉宽，
//!    与该位所在位周期（相邻 LOW 起始沿间距）比较：LOW < 周期/2 → 1 位，
//!    否则 0 位。16 位帧 = (油门 11 位 + 遥测 1 位)<<4 | CRC4，CRC 校验通过
//!    才更新电调量。帧内比例自校准（1 位/0 位脉宽 1:2），与绝对速率无关。
//!
//! # 电机模型（开环）
//!
//! 电调量 0~2000 → 转速 RPM = 死区（低于 `dead_throttle` 停转）+ 线性到
//! `max_rpm`（满油门）。`step(dt)` 预留机械一阶响应（时间常数 = 0 即瞬态）。
//!
//! # 联动接口预留（后续飞控闭环）
//!
//! [`EscMotor::rpm`] 是电机转速观测出口：将来把 IMU 数据源（data_source）升级为
//! "动力学驱动"时，按 4 电机转速计算推力/力矩 → 刚体姿态动力学 → 加速度/角速度
//! → IMU 读数，即可闭环验证固件姿态控制律。本模块只保证 rpm 数值口径稳定。

use std::sync::{Arc, Mutex};

/// 传统 PWM 电调脉宽约定（μs）：1000 = 0 油门、2000 = 满油门
const PWM_PULSE_MIN_US: f32 = 1000.0;
const PWM_PULSE_MAX_US: f32 = 2000.0;
/// DShot 帧长（位）
const DSHOT_BITS: usize = 16;
/// DShot 有效载荷位长（油门 11 + 遥测 1）
const DSHOT_PAYLOAD_BITS: usize = 12;

/// DShot 输入状态机
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DshotState {
    /// 等待帧起始沿（行空闲 HIGH，拉低 = 帧开始）
    Idle,
    /// 收集 16 位的 LOW 起始/结束沿
    Frame { n: u8, falls: [u64; DSHOT_BITS], rises: [u64; DSHOT_BITS] },
}

/// 电调输入来源（最近一次生效）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Src {
    None,
    Pwm,
    Dshot,
}

/// 电调配置
#[derive(Debug, Clone)]
pub struct EscConfig {
    pub name: &'static str,
    /// PWM 输入：TIM 端口 + 通道（0-3）
    pub pwm_port: Option<u8>,
    pub pwm_channel: Option<u8>,
    /// PWM 周期（μs），用于占空比 → 脉宽换算（如 400Hz → 2500）
    pub pwm_period_us: u32,
    /// DShot 输入：GPIO 端口 + 引脚
    pub dshot_port: Option<u8>,
    pub dshot_pin: Option<u8>,
    /// 电机模型：满油门转速（RPM）
    pub max_rpm: f32,
    /// 电机模型：死区电调量（低于此停转）
    pub dead_throttle: u16,
}

impl Default for EscConfig {
    fn default() -> Self {
        Self {
            name: "esc0",
            pwm_port: None,
            pwm_channel: None,
            pwm_period_us: 2500,
            dshot_port: None,
            dshot_pin: None,
            max_rpm: 12000.0,
            dead_throttle: 48,
        }
    }
}

/// ESC 电调 + BLDC 无刷电机虚拟外设
pub struct Esc {
    cfg: EscConfig,
    /// 输入来源（最近一次生效）
    src: Src,
    // ---- PWM 解码 ----
    /// 当前脉冲上升沿 tick（R_n）
    pwm_rise: Option<u64>,
    /// 当前脉冲高电平 tick 数（F_n - R_n）
    pwm_high: Option<u64>,
    /// 已解码的 PWM 脉冲数
    pub n_pwm_pulses: u64,
    // ---- DShot 解码 ----
    dshot: DshotState,
    /// 已解码的 DShot 帧数（CRC 通过）
    pub n_dshot_frames: u64,
    /// CRC 校验失败次数
    pub n_crc_errors: u64,
    /// 最近一帧原始 16 位（观测）
    pub last_frame: u16,
    // ---- 电调量 / 电机模型 ----
    throttle: u16,
    rpm: f32,
    /// 输入事件总数（观测）
    access: u64,
}

impl Esc {
    pub fn new(cfg: EscConfig) -> Self {
        Self {
            cfg,
            src: Src::None,
            pwm_rise: None,
            pwm_high: None,
            n_pwm_pulses: 0,
            dshot: DshotState::Idle,
            n_dshot_frames: 0,
            n_crc_errors: 0,
            last_frame: 0,
            throttle: 0,
            rpm: 0.0,
            access: 0,
        }
    }

    /// DShot CRC：对 12 位有效载荷按 DShot 算法算 4 位校验（XNOR 取反）。
    /// 与固件 drv/dshot.c 的 dshot_crc 完全一致。
    pub fn dshot_crc(payload: u16) -> u8 {
        let mut crc: u8 = 0;
        for i in 0..DSHOT_PAYLOAD_BITS {
            crc <<= 1;
            if payload & (0x800 >> i) != 0 {
                crc |= 1;
            }
            if crc & 0x10 != 0 {
                crc ^= 0x05;
            }
        }
        (!crc) & 0x0F
    }

    /// 用当前输入更新电机模型（死区 + 线性）
    fn update_motor(&mut self) {
        let t = self.throttle;
        self.rpm = if t < self.cfg.dead_throttle {
            0.0
        } else {
            (t as f32 / 2000.0) * self.cfg.max_rpm
        };
    }

    /// PWM 输入：按占空比 + 配置周期换算脉宽 → 电调量 0~2000
    fn apply_pwm_pulse(&mut self, high: u64, period: u64) {
        if period == 0 {
            return;
        }
        let duty = high as f32 / period as f32;
        let pulse_us = duty * self.cfg.pwm_period_us as f32;
        let t = ((pulse_us - PWM_PULSE_MIN_US) / (PWM_PULSE_MAX_US - PWM_PULSE_MIN_US) * 2000.0)
            .clamp(0.0, 2000.0) as u16;
        self.throttle = t;
        self.src = Src::Pwm;
        self.update_motor();
    }

    /// DShot 帧解码（16 位，MSB 先发）：CRC 通过才更新电调量
    fn decode_dshot_frame(&mut self, falls: &[u64; DSHOT_BITS], rises: &[u64; DSHOT_BITS]) {
        // 位周期估计：帧内 15 个单元的平均（bit15 无后继单元，用平均）
        let avg_cell = if falls[DSHOT_BITS - 1] > falls[0] {
            (falls[DSHOT_BITS - 1] - falls[0]) / (DSHOT_BITS - 1) as u64
        } else {
            0
        };
        let mut frame: u16 = 0;
        for i in 0..DSHOT_BITS {
            let low = rises[i].saturating_sub(falls[i]);
            let cell = if i + 1 < DSHOT_BITS {
                falls[i + 1].saturating_sub(falls[i])
            } else {
                avg_cell
            };
            // LOW 脉宽 < 位周期一半 → 1 位（短 LOW）；否则 0 位（长 LOW）
            let bit = if cell > 0 && low < cell / 2 { 1u16 } else { 0u16 };
            frame |= bit << (DSHOT_BITS - 1 - i);
        }
        self.last_frame = frame;
        let crc = (frame & 0xF) as u8;
        let payload = frame >> 4;
        if Self::dshot_crc(payload) == crc {
            self.throttle = (payload >> 1) & 0x7FF;
            self.n_dshot_frames += 1;
            self.src = Src::Dshot;
            self.update_motor();
        } else {
            self.n_crc_errors += 1;
        }
    }
}

/// ESC 电调 + 电机观测接口（Machine 持有；联动预留：IMU 数据源据此取转速）
pub trait EscMotor: Send {
    fn name(&self) -> &str;
    /// TIM PWM 电平变化事件（含虚拟 tick）
    fn on_pwm(&mut self, port: u8, channel: u8, level: bool, tick: u64);
    /// GPIO 电平变化事件（含虚拟 tick）
    fn on_gpio(&mut self, port: u8, pin: u8, level: bool, tick: u64);
    /// 虚拟时钟推进（电机模型/机械响应）
    fn step(&mut self, _dt: f32) {}
    /// 当前转速（RPM）——飞控闭环联动的观测出口
    fn rpm(&self) -> f32;
    /// 当前电调量（0-2000）
    fn throttle(&self) -> u16;
    /// CRC 通过并更新的 DShot 帧数（观测）
    fn dshot_frames(&self) -> u64 {
        0
    }
    /// DShot CRC 校验失败次数（观测）
    fn crc_errors(&self) -> u64 {
        0
    }
}

impl EscMotor for Esc {
    fn name(&self) -> &str {
        self.cfg.name
    }

    fn on_pwm(&mut self, port: u8, channel: u8, level: bool, tick: u64) {
        // 仅处理本机配置的 PWM 通道
        if self.cfg.pwm_port != Some(port) || self.cfg.pwm_channel != Some(channel) {
            return;
        }
        self.access += 1;
        if level {
            // 上升沿：上一脉冲周期结束（R_{n+1} - R_n），更新电调量
            if let (Some(r), Some(h)) = (self.pwm_rise, self.pwm_high) {
                if tick > r {
                    self.apply_pwm_pulse(h, tick - r);
                    self.n_pwm_pulses += 1;
                }
            }
            self.pwm_rise = Some(tick);
            self.pwm_high = None;
        } else if let Some(r) = self.pwm_rise {
            // 下降沿：记录当前脉冲高电平（F_n - R_n）
            self.pwm_high = Some(tick.saturating_sub(r));
        }
    }

    fn on_gpio(&mut self, port: u8, pin: u8, level: bool, tick: u64) {
        if self.cfg.dshot_port != Some(port) || self.cfg.dshot_pin != Some(pin) {
            return;
        }
        self.access += 1;
        match self.dshot {
            DshotState::Idle => {
                // 行空闲 HIGH；拉低 = 帧起始沿
                if !level {
                    let mut falls = [0u64; DSHOT_BITS];
                    falls[0] = tick;
                    self.dshot = DshotState::Frame { n: 0, falls, rises: [0u64; DSHOT_BITS] };
                }
            }
            DshotState::Frame { n, mut falls, mut rises } => {
                if n >= DSHOT_BITS as u8 {
                    self.dshot = DshotState::Idle;
                    return;
                }
                let i = n as usize;
                if !level {
                    falls[i] = tick; // 位 i 的 LOW 起始沿（须写回状态）
                    self.dshot = DshotState::Frame { n, falls, rises };
                } else {
                    rises[i] = tick; // 位 i 的 LOW 结束沿 → 该位完成
                    let n = n + 1;
                    if n >= DSHOT_BITS as u8 {
                        self.decode_dshot_frame(&falls, &rises);
                        self.dshot = DshotState::Idle;
                    } else {
                        self.dshot = DshotState::Frame { n, falls, rises };
                    }
                }
            }
        }
    }

    fn rpm(&self) -> f32 {
        self.rpm
    }

    fn throttle(&self) -> u16 {
        self.throttle
    }

    fn dshot_frames(&self) -> u64 {
        self.n_dshot_frames
    }

    fn crc_errors(&self) -> u64 {
        self.n_crc_errors
    }
}

impl Default for Esc {
    fn default() -> Self {
        Self::new(EscConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 直接喂 PWM 事件序列（合成一帧脉冲：上升沿 + 下降沿 + 下一上升沿）
    fn feed_pwm_pulse(e: &mut Esc, rise0: u64, fall: u64, rise1: u64) {
        e.on_pwm(3, 0, true, rise0);
        e.on_pwm(3, 0, false, fall);
        e.on_pwm(3, 0, true, rise1);
    }

    #[test]
    fn pwm_duty_to_throttle() {
        let mut e = Esc::new(EscConfig {
            name: "esc0",
            pwm_port: Some(3),
            pwm_channel: Some(0),
            pwm_period_us: 2500, // 400Hz
            ..Default::default()
        });
        // 50% 占空比：脉宽 1250μs → 电调量 (1250-1000)/1000*2000 = 500
        feed_pwm_pulse(&mut e, 0, 1000, 2000);
        assert_eq!(e.throttle(), 500);
        assert_eq!(e.rpm(), 500.0 / 2000.0 * 12000.0, "50% 油门 → 3000 RPM");
        // 80% 占空比：脉宽 2000μs → 满油门
        feed_pwm_pulse(&mut e, 3000, 3800, 4000);
        assert_eq!(e.throttle(), 2000);
        // 20% 占空比：脉宽 500μs → 0 油门（低于 1000μs 夹到 0）
        feed_pwm_pulse(&mut e, 5000, 5100, 6000);
        assert_eq!(e.throttle(), 0);
    }

    #[test]
    fn pwm_ignores_other_channels() {
        let mut e = Esc::new(EscConfig {
            name: "esc0",
            pwm_port: Some(3),
            pwm_channel: Some(0),
            ..Default::default()
        });
        // 非本机通道的事件被忽略
        e.on_pwm(1, 0, true, 0);
        e.on_pwm(1, 0, false, 100);
        e.on_pwm(1, 0, true, 200);
        assert_eq!(e.throttle(), 0);
        assert_eq!(e.access, 0);
    }

    #[test]
    fn dead_zone_stops_motor() {
        let mut e = Esc::new(EscConfig {
            name: "esc0",
            pwm_port: Some(3),
            pwm_channel: Some(0),
            max_rpm: 12000.0,
            dead_throttle: 48,
            ..Default::default()
        });
        // 48% 占空比：脉宽 1200μs → 电调量 400（> 死区 48）
        feed_pwm_pulse(&mut e, 0, 960, 2000);
        assert_eq!(e.throttle(), 400);
        assert!(e.rpm() > 0.0);
        // 死区内：占空比 40%？不——改 40% 对应 1200μs=400。用极小脉宽 → 0 油门 → 停转
        feed_pwm_pulse(&mut e, 5000, 5001, 6000); // 20% → 0 油门
        assert_eq!(e.throttle(), 0);
        assert_eq!(e.rpm(), 0.0, "死区内停转");
    }

    /// 合成一帧 DShot：payload → 16 位帧 → 生成 GPIO 电平事件序列（tick 递增）
    fn feed_dshot_frame(e: &mut Esc, payload: u16) {
        let frame: u16 = (payload << 4) | Esc::dshot_crc(payload) as u16;
        let mut tick = 1000u64;
        // 行空闲 HIGH → 帧起始（拉低）
        e.on_gpio(4, 10, false, tick); // bit15 (MSB) LOW 起始
        for i in 0..DSHOT_BITS {
            let bit = (frame >> (DSHOT_BITS - 1 - i)) & 1;
            // LOW 脉宽：1 位 = 1/3 单元（短）、0 位 = 2/3 单元（长）
            let cell = 100u64;
            let low = if bit == 1 { cell / 3 } else { (cell * 2) / 3 };
            // 下降沿已在上一次迭代/帧首发出；这里发上升沿 + 下一位的下降沿
            let rise = tick + low;
            e.on_gpio(4, 10, true, rise);
            tick = rise + (cell - low);
            if i + 1 < DSHOT_BITS {
                e.on_gpio(4, 10, false, tick); // 下一位 LOW 起始
            }
        }
    }

    #[test]
    fn dshot_throttle_decode_and_crc() {
        let mut e = Esc::new(EscConfig {
            name: "esc1",
            dshot_port: Some(4),
            dshot_pin: Some(10),
            ..Default::default()
        });
        feed_dshot_frame(&mut e, 1000 << 1); // 油门 1000
        assert_eq!(e.n_crc_errors, 0, "CRC 应通过");
        assert_eq!(e.n_dshot_frames, 1);
        assert_eq!(e.throttle(), 1000);
        assert_eq!(e.rpm(), 1000.0 / 2000.0 * 12000.0, "50% 油门 → 6000 RPM");
    }

    #[test]
    fn dshot_crc_mismatch_rejected() {
        let mut e = Esc::new(EscConfig {
            name: "esc1",
            dshot_port: Some(4),
            dshot_pin: Some(10),
            ..Default::default()
        });
        // 篡改一帧：发一个 CRC 错的帧
        let frame: u16 = ((500u16 << 1) << 4) | 0x0; // 错误 CRC（应为 Esc::dshot_crc(1000)）
        let mut tick = 1000u64;
        e.on_gpio(4, 10, false, tick);
        for i in 0..DSHOT_BITS {
            let bit = (frame >> (DSHOT_BITS - 1 - i)) & 1;
            let cell = 100u64;
            let low = if bit == 1 { cell / 3 } else { (cell * 2) / 3 };
            let rise = tick + low;
            e.on_gpio(4, 10, true, rise);
            tick = rise + (cell - low);
            if i + 1 < DSHOT_BITS {
                e.on_gpio(4, 10, false, tick);
            }
        }
        assert_eq!(e.n_crc_errors, 1, "CRC 错帧应被拒绝");
        assert_eq!(e.n_dshot_frames, 0);
        assert_eq!(e.throttle(), 0, "CRC 错不更新电调量");
    }
}
