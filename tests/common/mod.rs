//! 「虚拟设备直接模拟」测试公共辅助：场景驱动 harness + 固件内存/日志断言工具。
//!
//! 链路：EnvScenario（真值/扰动/故障）→ FlySimState（共享状态，run 之间写）
//! → mcu_simulater 虚拟外设（I2C IMU/baro/mag + UART GPS/SBUS）→ real-sensors
//! 固件真实驱动。不经 fly-simulater 物理闭环（非 SIL/HIL）。

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use unicorn_engine::RegisterARM;

use mcu_simulater::artifact;
use mcu_simulater::env::scenario::EnvScenario;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::vperiph::data_source::FlySimState;

/// 单步场景时间（毫秒）——**整数毫秒**（固件 SysTick 是 1ms 粒度）。
///
/// 闭环步进 = 场景推进 `STEP_DT_MS` ms + 固件经 [`mcu_simulater::clock::McuClock`]
/// 推进**同一** `STEP_DT_MS`（`run_ms` 按固件自身 SysTick 收敛）。
///
/// **不再用 `run(字节预算)` 表达时间**：字节预算是后端实现细节（实测同预算的
/// bytes/ms 随代码块混合比在 100K~109K 之间浮动）。旧口径把 `2_300_000` 当成
/// 「≈13.3ms 固件时间」，实测固件实走 **~21.5ms** → 固件比场景快 ~1.6×，
/// 与 c62ec21 修的悬停路径是同类时钟失配（场景/固件时间错配 → EKF 积分漂）。
pub const STEP_DT_MS: f32 = 13.0;
/// 单步场景时间（秒）。
pub const STEP_DT: f32 = STEP_DT_MS / 1000.0;

/// 固件 EST_STATE 地址（app.elf 符号，布局见 [`EstReadout`]）。
pub const EST_STATE: u32 = 0x2000_F184;
/// 虚拟 USB 主机模型：建模"PC 连着 usb0（CDC 虚拟串口）收 MAVLink 遥测"。
///
/// **只作用于虚拟设备侧，不额外推进固件时间。** 与其它虚拟外设同构：由测试自身的
/// 步进驱动，每步最多发一个枚举动作，固件按自己的节奏处理。绝不在此调用
/// `advance_ms`/`run_budget`——那会改变固件的任务执行轨迹（实测会让固件在 step0 之前
/// 多跑约 136ms，控制任务行为随之偏移）。
///
/// 为什么必须建模：固件 `telemetry_entry` 每 20ms 无条件写 usb0、`uplink_task` 以
/// 1kHz 轮询 `usb0.read`。若没有主机消费 IN 传输，`UsbOtg` 的传输永不完成，固件
/// `usb_tx_pump` 会反复打 `TX_PUMP busy` 诊断日志（实测 560 条/秒），冲爆 2KB 日志环，
/// 把心跳等真正有用的日志挤掉。
pub struct UsbHostModel {
    /// 已执行的枚举动作数：0=未开始，1=已复位，2..=5=已发第 1..4 个 SETUP
    stage: u8,
    /// 主机累计取走的下行字节数（确认遥测真的在流）
    pub rx_bytes: u64,
    /// 是否已连接（`detach()` 后恒为 false，用于 A/B 对照）
    attached: bool,
}

impl UsbHostModel {
    /// 固件启动到 USB 就绪所需的步数（x_hil_mcusim 的预启动约 130ms，此处 13ms/步）。
    const START_STEP: u64 = 10;
    /// 标准枚举：设备/配置描述符、SET_ADDRESS、SET_CONFIGURATION
    const SETUPS: [[u8; 8]; 4] = [
        [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00],
        [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x20, 0x00],
        [0x00, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00],
        [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
    ];

    pub fn new() -> Self {
        UsbHostModel { stage: 0, rx_bytes: 0, attached: true }
    }

    /// 断开主机：此后不再做任何 USB 操作（复位/SETUP/取走 IN）。
    /// 仅用于 A/B 测量（对照"无主机"的真实使用形态）；不影响时间轴——
    /// 调用方仍按同样步数推进 `advance_ms`。
    pub fn detach(&mut self) {
        self.attached = false;
    }

    /// 每步调用一次（须在固件时间推进**之后**）。返回本步取走的下行字节数。
    pub fn tick(&mut self, m: &mut Machine, step: u64) -> usize {
        use mcu_simulater::events::Event;
        if !self.attached || step < Self::START_STEP {
            return 0;
        }
        if self.stage == 0 {
            m.usb_otg.lock().unwrap().inject_usb_reset();
            self.stage = 1;
            return 0;
        }
        if (self.stage as usize) <= Self::SETUPS.len() {
            let data = Self::SETUPS[self.stage as usize - 1];
            m.events.lock().unwrap().publish(&Event::UsbSetup { data });
            self.stage += 1;
            return 0;
        }
        let n = m.usb_otg.lock().unwrap().host_take_in(1).len();
        self.rx_bytes += n as u64;
        n
    }
}

impl Default for UsbHostModel {
    fn default() -> Self {
        Self::new()
    }
}

/// 固件 SENSOR_SEQ（sensors 任务推进计数，冻结即任务停滞）。
pub const SENSOR_SEQ: u32 = 0x2001_16DC;

/// 从 app.elf 读出的估计状态（VehicleState + health + armed 的内存视图）。
#[derive(Debug, Clone, Copy, Default)]
pub struct EstReadout {
    pub time_boot_ms: i32,
    pub pos: [f32; 3],   // NED (m)
    pub vel: [f32; 3],   // NED (m/s)
    pub att_wxyz: [f32; 4], // 四元数 (w,x,y,z)
    pub omega: [f32; 3], // 体轴 (rad/s)
    pub airspeed: f32,
    pub accel_bias: [f32; 3],
    pub health: u32,     // 0=Nominal 1=Degraded 2=Critical
    pub armed: u8,
}

impl EstReadout {
    /// 欧拉 roll/pitch/yaw（rad），从四元数（w,x,y,z）。
    pub fn euler(&self) -> [f32; 3] {
        let [w, x, y, z] = self.att_wxyz;
        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let sp = 2.0 * (w * y - z * x);
        let pitch = sp.clamp(-1.0, 1.0).asin();
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        [roll, pitch, yaw]
    }
}

/// 场景驱动 harness。
pub struct EnvHarness {
    pub m: Machine,
    pub st: Arc<Mutex<FlySimState>>,
    pub scn: EnvScenario,
    pub got_invalid: Arc<AtomicBool>,
    pub bad_pc: Arc<AtomicU32>,
    pub steps: u64,
    /// 固件时钟起点（SysTick ms）——用于步进对齐断言（固件时间 ≈ 步数×dt）。
    pub step0_ms: u64,
    /// 已收集的全部 console 输出。
    pub log: String,
    /// 上次日志长度（增量解析用）。
    log_pos: usize,
    pub last_hb: Option<HbLine>,
    /// 虚拟 USB 主机（被动模型，由本步进驱动）
    pub usb_host: UsbHostModel,
}

/// 解析后的 hb 心跳行。
#[derive(Debug, Clone, Copy, Default)]
pub struct HbLine {
    pub seq: u32,
    pub armed: bool,
    pub crit: bool,
    pub alt: f32,
    pub imu_ok: bool,
    pub mag_ok: bool,
    pub gps_ok: bool,
    pub baro_ok: bool,
    pub gv: [f32; 3],
    pub gpsd: f32,
    pub gz: f32,
}

impl EnvHarness {
    /// 构建 harness：jOS + real-sensors app + 虚拟外设（flysim 直通）。
    /// `prefill`：attach 后、固件首跑前立即写入的场景初始状态（默认 Hover 静止）。
    pub fn new(mut scn: EnvScenario, prefill: bool) -> Self {
        let elf = artifact::joc_base_elf();
        let app = artifact::flyctrl_real_app_bin();
        let mut m = Machine::new_m4f().unwrap();
        m.map_stm32f407_layout().unwrap();
        let st = Arc::new(Mutex::new(FlySimState::default()));
        m.attach_flysim_sensors(st.clone());
        m.attach_flysim_uart_slaves(st.clone());
        m.load_elf(&elf).unwrap();
        m.load_app_partition(&app).unwrap();
        m.reset().unwrap();

        let bad_pc = Arc::new(AtomicU32::new(0));
        let got_invalid = Arc::new(AtomicBool::new(false));
        let (b2, g2) = (bad_pc.clone(), got_invalid.clone());
        m.cpu.raw().add_insn_invalid_hook(move |uc| {
            let pc = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
            b2.store(pc, Ordering::Relaxed);
            g2.store(true, Ordering::Relaxed);
            eprintln!("[insn_invalid] pc=0x{pc:08x}");
            false
        }).unwrap();

        if prefill {
            scn.write_state(&mut st.lock().unwrap());
        }
        let step0_ms = m.systick_ms();
        Self {
            m,
            st,
            scn,
            got_invalid,
            bad_pc,
            steps: 0,
            step0_ms,
            log: String::new(),
            log_pos: 0,
            last_hb: None,
            usb_host: UsbHostModel::new(),
        }
    }

    /// 跑一步：场景推进 → 写共享状态 → 固件经 McuClock 推进同一 dt
    /// （run 期间状态冻结，符合一致性不变量）。
    pub fn step(&mut self) {
        use mcu_simulater::clock::McuClock;
        self.scn.advance(STEP_DT);
        {
            let mut st = self.st.lock().unwrap();
            self.scn.write_state(&mut st);
        }
        let r = self.m.advance_ms(STEP_DT_MS as f64);
        assert!(r.is_ok(), "[step {}] advance_ms 错误: {r:?}", self.steps);
        // 虚拟 USB 主机：只操作虚拟设备侧，不推进固件时间（在 advance_ms **之后**）
        self.usb_host.tick(&mut self.m, self.steps);
        self.steps += 1;
        // 对齐不变量：固件时间必须 ≈ 步数 × dt（run_ms 的 SysTick 收敛允许 ≤1ms 量化）。
        // 若有人改回 run(字节预算) 表达时间，这里立刻红。
        let elapsed = self.m.systick_ms() - self.step0_ms;
        let expect = self.steps * STEP_DT_MS as u64;
        assert!(
            elapsed.abs_diff(expect) <= 1,
            "[clock] 固件时间 {elapsed}ms != 步数×dt {expect}ms —— 场景/固件时钟失配             （时间推进一律走 McuClock/run_ms，不得用 run(字节预算)）"
        );
        self.pump_log();
    }

    /// 跑 n 步。
    pub fn run_steps(&mut self, n: u64) {
        for _ in 0..n {
            self.step();
        }
    }

    /// 拉取 console 增量到 self.log，解析 hb 行。
    fn pump_log(&mut self) {
        let out = {
            let outv = self.m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        if out.len() > self.log_pos {
            let chunk = &out[self.log_pos..];
            self.log.push_str(chunk);
            self.log_pos = out.len();
            // 解析最后一条 hb 行
            if let Some(idx) = chunk.rfind("hb seq=") {
                if let Some(line) = chunk[idx..].lines().next() {
                    if let Some(hb) = parse_hb(line) {
                        self.last_hb = Some(hb);
                    }
                }
            }
        }
    }

    /// 读取固件估计状态（含布局 sanity 校验）。
    pub fn read_est(&mut self) -> EstReadout {
        let mut e = EstReadout::default();
        let rd = |m: &mut Machine, addr: u32| -> u32 {
            m.cpu.mem_read(addr as u64, 4).ok().map(|b| u32::from_le_bytes(b.try_into().unwrap())).unwrap_or(0)
        };
        let rf = |m: &mut Machine, addr: u32| -> f32 { f32::from_bits(rd(m, addr)) };
        e.time_boot_ms = rd(&mut self.m, EST_STATE) as i32;
        for k in 0..3 {
            e.pos[k] = rf(&mut self.m, EST_STATE + 4 + 4 * k as u32);
        }
        for k in 0..3 {
            e.vel[k] = rf(&mut self.m, EST_STATE + 16 + 4 * k as u32);
        }
        for k in 0..4 {
            e.att_wxyz[k] = rf(&mut self.m, EST_STATE + 28 + 4 * k as u32);
        }
        for k in 0..3 {
            e.omega[k] = rf(&mut self.m, EST_STATE + 44 + 4 * k as u32);
        }
        e.airspeed = rf(&mut self.m, EST_STATE + 56);
        for k in 0..3 {
            e.accel_bias[k] = rf(&mut self.m, EST_STATE + 60 + 4 * k as u32);
        }
        // 【实测布局】EstState(repr(C)) = VehicleState(72B) + Health + bool(armed)，
        // 但编译后 Health 实际占 1B（mem 实证：hb 行 armed=true 时 EST+73=1、EST+76=0；
        // repr(C) enum 未标判别值在 ARM 上编译为 1B，而非 C int 4B）。
        // → health@72(1B)、armed@73(1B)。
        e.health = self.m.cpu.mem_read((EST_STATE + 72) as u64, 1).ok().map(|b| b[0] as u32).unwrap_or(0);
        e.armed = self.m.cpu.mem_read((EST_STATE + 73) as u64, 1).ok().map(|b| b[0]).unwrap_or(0);
        e
    }

    /// 读取 SENSOR_SEQ（sensors 任务推进计数）。
    pub fn read_sensor_seq(&mut self) -> u32 {
        self.m
            .cpu
            .mem_read(SENSOR_SEQ as u64, 4)
            .ok()
            .map(|b| u32::from_le_bytes(b.try_into().unwrap()))
            .unwrap_or(0)
    }

    /// console 全量（诊断用）。
    pub fn console_all(&mut self) -> String {
        self.pump_log();
        self.log.clone()
    }
}

/// 解析 hb 行：`hb seq=.. armed=.. crit=.. alt=.. imu_ok=.. mag=.. gps=.. gv=(..) baro=.. gpsd=.. gz=.. m=[..]`
pub fn parse_hb(line: &str) -> Option<HbLine> {
    let mut hb = HbLine::default();
    let mut ok = true;
    let mut gv = [0.0f32; 3];
    for part in line.split_whitespace() {
        if let Some(v) = part.strip_prefix("seq=") {
            hb.seq = v.parse().ok()?;
        } else if let Some(v) = part.strip_prefix("armed=") {
            hb.armed = v == "true";
        } else if let Some(v) = part.strip_prefix("crit=") {
            hb.crit = v == "true";
        } else if let Some(v) = part.strip_prefix("alt=") {
            hb.alt = v.parse().ok()?;
        } else if let Some(v) = part.strip_prefix("imu_ok=") {
            hb.imu_ok = v == "true";
        } else if let Some(v) = part.strip_prefix("mag=") {
            hb.mag_ok = v == "true";
        } else if let Some(v) = part.strip_prefix("gps=") {
            hb.gps_ok = v == "true";
        } else if let Some(v) = part.strip_prefix("baro=") {
            hb.baro_ok = v == "true";
        } else if let Some(v) = part.strip_prefix("gpsd=") {
            hb.gpsd = v.parse().ok()?;
        } else if let Some(v) = part.strip_prefix("gz=") {
            hb.gz = v.parse().ok()?;
        } else if let Some(v) = part.strip_prefix("gv=(") {
            let inner = v.trim_end_matches(')');
            let mut it = inner.split(',');
            for k in 0..3 {
                gv[k] = it.next()?.trim().parse().ok()?;
            }
            hb.gv = gv;
        }
    }
    if !ok {
        return None;
    }
    Some(hb)
}
