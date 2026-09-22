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
/// ★**2026-09-21 回退说明**：曾改为 4.0 ✗ 已回退为 13.0 ✓。
/// 原因：**13.0 是本 M 场测试套件【已验证的基线标定】** ✓（该套件曾用于与 H 场做验收 ✓）。
/// 我此前把“固件自身能跑 249.7Hz（zz_ctlprof 实测 ✓）”与“本 harness 的场景步长”
/// 混为一谈 ✗ ⇒ 改了步长 ⇒ 大量按步数标定的测例失效 ✗✓。
/// 二者的区别 ✓：
///   · **固件自身控制拍** = 249.7Hz ≈ 4ms ✓（固件 dt 取【实际 tick 差】自动适应 ✓）
///   · **本 harness 场景步长** = 13ms ✓（测试套件的既定标定 ✓，改它会破坏全部按步数的断言 ✗）
/// **不再用 `run(字节预算)` 表达时间**：字节预算是后端实现细节（实测同预算的
/// bytes/ms 随代码块混合比在 100K~109K 之间浮动）。旧口径把 `2_300_000` 当成
/// 「≈13.3ms 固件时间」，实测固件实走 **~21.5ms** → 固件比场景快 ~1.6×，
/// 与 c62ec21 修的悬停路径是同类时钟失配（场景/固件时间错配 → EKF 积分漂）。
pub const STEP_DT_MS: f32 = 13.0;
/// 单步场景时间（秒）。
pub const STEP_DT: f32 = STEP_DT_MS / 1000.0;

/// [临时标定] 经环境变量向固件写入 EKF 参数覆盖（当前支持 `G_Q_ACCEL`）。
///
/// 用途：协方差更新改用 Joseph 形式后位置/高度通道需重标定；把过程噪声做成可运行时
/// 写入的旋钮，即可不重编固件扫描"加计偏置容忍度 vs 噪声鲁棒性"。未设环境变量 = 不注入。
pub fn apply_env_calib(m: &mut mcu_simulater::machine::Machine) {
    for (env, symname) in [
        ("ZZ_Q_ACCEL", "G_Q_ACCEL"),
        ("ZZ_Q_VEL", "G_Q_VEL"),
        ("ZZ_R_VEL", "G_R_VEL"),
        ("ZZ_R_POS", "G_R_POS"),
        ("ZZ_TAU_XY", "G_TAU_XY"),
        // 磁航向锚定强度。注意哨兵：**-1 = 用编译期值**；**0 = 显式关闭锚定**
        // （隔离磁路影响用；其它旋钮的哨兵是 0，本旋钮不是——因为 0 是有效值）。
        ("ZZ_MAG_ALPHA", "G_MAG_ALPHA"),
        // 重力锚定强度。哨兵同规：**-1 = 用编译期值**（0 = 显式关闭锚定）。
        // 用途：**在 M 场自己的闭环口径下**实测 att_alpha（不抄 SIL 的值 ——
        // 锚定是场景依赖的：估计类测试需要、闭环类有害，见 flyctrl 提交 mrev）。
        ("ZZ_ATT_ALPHA", "G_ATT_ALPHA"),
    ] {
        if let Ok(v) = std::env::var(env) {
            if let Ok(t) = v.parse::<f32>() {
                let a = mcu_simulater::elfsym::app_sym(symname) as u64;
                m.cpu.mem_write(a, &t.to_le_bytes()).unwrap();
                eprintln!("[calib] {symname} = {t}");
            }
        }
    }
}

/// 固件 `EST_STATE` 地址（布局见 [`EstReadout`]）。
///
/// **从 app.elf 符号表解析，不硬编码**：linker 只钉住 `.app_globals` 段起始地址，
/// 段内符号顺序随固件代码变化（实测一次 USB 相关改动就把 EST_STATE 从 0x2000F184
/// 挪到 0x2000F06C），硬编码会静默读到垃圾并产生"假失败"。
pub fn est_state() -> u32 {
    mcu_simulater::elfsym::app_sym("EST_STATE")
}
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

    /// 一期固件不带机载电脑 → **默认不接主机**（模型存在但不动作）。
    /// 后续接入机载电脑时调用 [`UsbHostModel::attach`] 打开。
    pub fn new() -> Self {
        UsbHostModel { stage: 0, rx_bytes: 0, attached: false }
    }

    /// 接入虚拟主机（启用枚举与下行取走）。对应"机载电脑已连接"的形态。
    pub fn attach(&mut self) {
        self.attached = true;
    }

    /// 断开主机（默认即断开）：此后不再做任何 USB 操作（复位/SETUP/取走 IN）。
    /// 不影响时间轴——调用方仍按同样步数推进 `advance_ms`。
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

/// 固件 `SENSOR_SEQ`（sensors 任务推进计数，冻结即任务停滞）。同样从符号表解析。
pub fn sensor_seq() -> u32 {
    mcu_simulater::elfsym::app_sym("SENSOR_SEQ")
}

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

        apply_env_calib(&mut m);

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

    /// 跑一步：**以固件控制拍为时基**（照 `982e0cf` 的锁相范式 ✓）
    ///
    /// 流程 ✓：① 读固件当前时间 → ② 推进固件【直到恰好完成一拍控制】→
    /// ③ 按**实测流逝时长**推进场景并写共享状态 ✓（不假设任何固定 dt ✗）。
    ///
    /// 为何改（用户指正 + `982e0cf` 诊断 ✓）：
    /// ```text
    /// 原实现按【固定 13ms】推进 ⇒ 固件控制拍（均值 4.0000ms、std 0.84ms）与场景
    /// 时基【相位自由漂移】⇒ 代码布局灵敏度 ✗；且 13ms 使传感器等效更新率仅 74.9Hz ✗
    /// （设计：控制 250Hz / **传感器 500Hz** ✓，见 flyctrl/app 的 mod.rs / sensors_task.rs ✓）
    /// 锁相后：时基由【固件自身】决定 ✓ ⇒ 250Hz/500Hz 自动正确 ✓，harness 不再假定 ✗
    /// ```
    /// ★**按时间跑**（迁移后的首选 ✓）：持续 `step()` 直到【固件时间】走过 `ms` 毫秒 ✓。
    ///
    /// 为何用固件时间而非步数 ✓：锁相后每步对应固件的一拍控制（名义 4ms ✓），
    /// 用时间表达 ⇒ **步数成为实现细节** ✓ ⇒ 测例不再依赖"每步多少 ms"的隐式假设 ✗
    /// （这正是旧前提遗留问题 ✗：原来 `run_steps(N)` 隐含"13ms/步"✗）。
    ///
    /// 迁移映射 ✓（保持测例的**时间长度**不变 ✓，但时基改为固件 ✓）：
    ///   `run_steps(N)`  →  `run_for_ms(N as f64 * 13.0)`   // 旧口径 N 步 ≈ N×13ms ✓
    pub fn run_for_ms(&mut self, ms: f64) {
        let target = ms;
        let t0 = self.m.systick_ms();
        let mut guard = 0u32;
        while (self.m.systick_ms() - t0) as f64 + 4.0 < target {
            self.step();
            guard += 1;
            assert!(
                guard < 2_000_000,
                "run_for_ms({ms}) 未在合理步数内达成（guard={guard}）—— 固件时钟异常？"
            );
        }
    }

    /// 按【秒】跑（同 `run_for_ms` ✓）
    pub fn run_for_secs(&mut self, secs: f64) {
        self.run_for_ms(secs * 1000.0);
    }

    pub fn step(&mut self) {
        use mcu_simulater::clock::run_one_control_tick;
        // ② 固件推进恰好一拍控制（内部轮询固件符号 CTRL_TICKS ✓，不改固件行为 ✓）
        let t_before = self.m.systick_ms();
        run_one_control_tick(&mut self.m).expect("run_one_control_tick 失败");
        let t_after = self.m.systick_ms();
        // ③ 按【实测流逝】推进场景（自洽 ✓：不假设 dt ✗）
        let elapsed_ms = (t_after.saturating_sub(t_before)) as f32;
        let dt = if elapsed_ms > 0.0 { elapsed_ms } else { 1.0 };
        self.scn.advance(dt / 1000.0);
        {
            let mut st = self.st.lock().unwrap();
            self.scn.write_state(&mut st);
        }
        self.steps += 1;
        // 锁相漂移守卫（照 x_hover_noise 模板 ✓）：长期均值必须贴合名义控制周期 ✓
        let elapsed = self.m.systick_ms() - self.step0_ms;
        let expect = (self.steps as f64 * 4.0) as u64; // 名义控制周期 4ms（250Hz）
        if elapsed.abs_diff(expect) > 200 {
            panic!("[clock] 锁相漂移过大：固件 {elapsed}ms vs 名义 {expect}ms（步 {}）", self.steps);
        }
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
        e.time_boot_ms = rd(&mut self.m, est_state()) as i32;
        for k in 0..3 {
            e.pos[k] = rf(&mut self.m, est_state() + 4 + 4 * k as u32);
        }
        for k in 0..3 {
            e.vel[k] = rf(&mut self.m, est_state() + 16 + 4 * k as u32);
        }
        for k in 0..4 {
            e.att_wxyz[k] = rf(&mut self.m, est_state() + 28 + 4 * k as u32);
        }
        for k in 0..3 {
            e.omega[k] = rf(&mut self.m, est_state() + 44 + 4 * k as u32);
        }
        e.airspeed = rf(&mut self.m, est_state() + 56);
        for k in 0..3 {
            e.accel_bias[k] = rf(&mut self.m, est_state() + 60 + 4 * k as u32);
        }
        // 【实测布局】EstState(repr(C)) = VehicleState(72B) + Health + bool(armed)，
        // 但编译后 Health 实际占 1B（mem 实证：hb 行 armed=true 时 EST+73=1、EST+76=0；
        // repr(C) enum 未标判别值在 ARM 上编译为 1B，而非 C int 4B）。
        // → health@72(1B)、armed@73(1B)。
        e.health = self.m.cpu.mem_read((est_state() + 72) as u64, 1).ok().map(|b| b[0] as u32).unwrap_or(0);
        e.armed = self.m.cpu.mem_read((est_state() + 73) as u64, 1).ok().map(|b| b[0]).unwrap_or(0);
        e
    }

    /// 读取 sensor_seq()（sensors 任务推进计数）。
    pub fn read_sensor_seq(&mut self) -> u32 {
        self.m
            .cpu
            .mem_read(sensor_seq() as u64, 4)
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
