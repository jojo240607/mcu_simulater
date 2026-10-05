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

pub mod phy_backend;

/// ★§5.143 PHY 化后端接口：把 `EnvHarness` 的运动学场景换成**真动力学**（`SimLoop::step_hil` ✓）。
/// 实现者持有 `SimLoop<PhySdkWorld>`，按固件执行器指令推进刚体并回写 `FlySimState` ✓
pub trait PhyBackend {
    /// 按固件当前执行器输出推进一个物理步，并回写传感器状态（姿态/位置/磁/IMU ✓）
    fn step_plant(&mut self, m: &mut Machine, st: &Arc<Mutex<FlySimState>>);
    /// 施加力矩脉冲（N·m·s ✓）——抗扰场景用（默认 no-op ⇒ 后端可不实现 ✓）
    fn disturb_torque(&mut self, _tau: [f64; 3]) {}
    /// 控制拍**之前**的钩子（默认 no-op ✓）：用于把摇杆 override 等输入在固件读取前写入 ✓
    fn pre_tick(&mut self, _m: &mut Machine) {}
    /// 控制拍**之前**写 `FlySimState`（模式开关/解锁等 ✓；默认 no-op ✓）
    fn pre_tick_state(&mut self, _st: &Arc<Mutex<FlySimState>>) {}
}

/// 单步场景时间（毫秒）——**整数毫秒**（固件 SysTick 是 1ms 粒度）。
///
/// 闭环步进 = **以固件控制拍为唯一时基** ✓（`clock::run_one_control_tick` ✓）：
/// 固件恰好完成一拍控制 ✓ → 再按【实测流逝时长】推进场景并写共享状态 ✓。
/// ⇒ 250Hz 控制 / 500Hz 传感器由【固件自身】决定 ✓，harness 不再假定 dt ✗。
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
/// ⚠️**已废弃** ✗（2026-09-21）：锁相步进后每步对应【固件一拍控制】（名义 4ms ✓），
/// 不再存在"固定场景步长"✗。**请改用 `run_for_ms` / `run_for_secs`** ✓（按固件时间 ✓）。
/// 保留此常量仅为兼容尚未迁移的引用 ✗（`grep STEP_DT_MS` 应逐步清零 ✓）。
pub const STEP_DT_MS: f32 = 13.0;

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


/// ★§5.219：控制栈写监视结果（`step()` 安装钩子时赋值 ✓；崩溃时读取 ✓）
static mut SUSPECT_CELL: *const () = core::ptr::null();
/// ★§5.219：监视生效开关 —— 启动期 bss 清零会合法写整块栈 ✗ ⇒ 前 20 拍不记录 ✓
static WATCH_ACTIVE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
/// ★§5.221：可疑写入对应的**当前任务名**（由钩子从 RTOS TCB 读出 ✓）
static NAMES: std::sync::Mutex<Vec<(u32, String)>> = std::sync::Mutex::new(Vec::new());
/// ★§5.222 指令飞行记录仪：块级 PC 环形缓冲（`JOC_FTRACE=1` 时启用 ✓，测试台侧 ⇒ 布局中性 ✓）
#[used]
pub static mut FTRACE_RING: [u32; 1024] = [0u32; 1024];
pub static FTRACE_POS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
/// ★§5.223 指令级记录环（pc, r4, r2, r6, r8）
#[used]
pub static mut ILRING: [(u32, u32, u32, u32, u32); 256] = [(0, 0, 0, 0, 0); 256];
/// 指令级环写指针（用于取末尾 ✓）
pub static ILPOS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

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
    /// ★§5.143 PHY 化：可选**真动力学**后端。`Some` 时 `step()` 用
    /// `SimLoop::step_hil(真实刚体)` 取代 `scn.advance(运动学)` ✓
    pub phy: Option<Box<dyn PhyBackend>>,
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
        // ★符号解析必须与所加载 bin 同 feature（§5.130）：.app_globals 段内偏移
        // 随 feature 漂移（hil vs real 的 SENSOR_SEQ 差 +0xE1C），默认的
        // flyctrl/app.elf 可能是其它 feature 的 ELF ⇒ 探针错位。
        mcu_simulater::elfsym::use_app_elf(mcu_simulater::artifact::flyctrl_real_app_elf());
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
            phy: None,
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

    /// 当前【固件时间】(ms) ✓ —— 供测例用"按固件时间的循环"表达时长 ✓
    /// （替代"按步数循环"✗，步数是实现细节 ✓）
    /// ★诊断：场景真值位姿（NED 位置/速度 + 欧拉角），用于与固件估计逐拍对比。
    pub fn truth(&self) -> mcu_simulater::env::scenario::Truth {
        self.scn.truth()
    }

    pub fn fw_ms(&self) -> u64 {
        self.m.systick_ms()
    }

    /// 按【秒】跑（同 `run_for_ms` ✓）
    pub fn run_for_secs(&mut self, secs: f64) {
        self.run_for_ms(secs * 1000.0);
    }

    pub fn step(&mut self) {
        use mcu_simulater::clock::run_one_control_tick;
        // ② 固件推进恰好一拍控制（内部轮询固件符号 CTRL_TICKS ✓，不改固件行为 ✓）
        // ★★§5.220：周期读"悬垂缓冲守卫"的粘性证据（次数/缓冲地址/**调用者 LR** ✓）
        {
            use std::sync::atomic::{AtomicU32, Ordering};
            static N: AtomicU32 = AtomicU32::new(0);
            static LAST: AtomicU32 = AtomicU32::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed);
            // ⚠️ 守卫符号在默认构建里被 feature 裁掉 ✓ ⇒ 仅排查时（JOC_STACKWATCH）读 ✓
            if n >= 20 && n % 50 == 0 && std::env::var("JOC_STACKWATCH").is_ok() {
                macro_rules! rd {
                    ($nm:expr) => {{
                        let a = mcu_simulater::elfsym::app_sym($nm) as u64;
                        self.m
                            .cpu
                            .mem_read(a, 4)
                            .map(|v| u32::from_le_bytes([v[0], v[1], v[2], v[3]]))
                            .unwrap_or(u32::MAX)
                    }};
                }
                let cnt = rd!("BAD_BUF_COUNT");
                if cnt != 0 && cnt != LAST.load(Ordering::Relaxed) {
                    LAST.store(cnt, Ordering::Relaxed);
                    let (a, l) = (rd!("BAD_BUF_ADDR"), rd!("BAD_BUF_LR"));
                    eprintln!(
                        "[guard] tick={n} 拒绝写入次数={cnt} 缓冲地址=0x{a:08x} **调用者LR=0x{l:08x}**"
                    );
                }
            }
        }
        // ★§5.219：前 20 拍（启动/bss 清零）不记录 ✓，之后打开监视 ✓
        {
            use std::sync::atomic::Ordering as O;
            static N: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
            if N.fetch_add(1, O::Relaxed) == 20 {
                WATCH_ACTIVE.store(true, O::Relaxed);
            }
        }
        // ★§5.219 控制栈**写监视**（只装一次 ✓）：抓"谁在踩控制栈" ✗
        //   判据 ①：写目标在控制栈内、但**当前 SP 不在**控制栈内 ⇒ 别的任务/上下文在踩 ✓
        //   判据 ②：写目标在控制栈内、且**远低于当前 SP**（>4KB）⇒ 本任务野写 ✓
        //   （4KB 余量：ESKF 等最深的合法帧约 ≤2KB ✓ ⇒ 不会误报 ✓）
        {
            use std::sync::Mutex;
            use std::sync::OnceLock;
            static SUSPECTS: OnceLock<Mutex<Vec<(u32, u32, u32, u32)>>> = OnceLock::new();
            let cell = SUSPECTS.get_or_init(|| Mutex::new(Vec::new()));
            // 供崩溃时读取
            unsafe { SUSPECT_CELL = cell as *const _ as *const (); }
            static INSTALLED: std::sync::atomic::AtomicBool =
                std::sync::atomic::AtomicBool::new(false);
            // ★§5.219：写监视开销大（每次栈写都进钩子 ⇒ 常规跑从 14s 变 85s ✗）
            //   ⇒ 仅在 `JOC_STACKWATCH=1` 时安装 ✓
            let want = std::env::var("JOC_STACKWATCH").is_ok();
            // ★§5.222 飞行记录仪安装（`JOC_FTRACE=1`）
            {
                use std::sync::atomic::Ordering as O2;
                static FT_ON: std::sync::atomic::AtomicBool =
                    std::sync::atomic::AtomicBool::new(false);
                if std::env::var("JOC_FTRACE").is_ok() && !FT_ON.swap(true, O2::SeqCst) {
                    let r = self.m.cpu.add_block_hook(
                        0x0800_0000,
                        0x0810_0000,
                        move |_uc, addr, _sz| {
                            // ★一旦进入"池内循环"区（0x0806f3d0..0x0806f410，本构建的
                            //   字面量池 ✓）就**冻结**记录 ⇒ 保留野跳前的现场 ✓
                            static FROZEN: std::sync::atomic::AtomicBool =
                                std::sync::atomic::AtomicBool::new(false);
                            if FROZEN.load(O2::Relaxed) {
                                return;
                            }
                            // 冻结点改为"进入 panic 入口"（按名解析 ✓，不再用错二进制的池地址 ✗）
                            static PANIC_ADDRS: std::sync::OnceLock<Vec<u64>> = std::sync::OnceLock::new();
                            let addrs = PANIC_ADDRS.get_or_init(|| {
                                vec![
                                    mcu_simulater::elfsym::app_sym("panic_bounds_check") as u64,
                                    mcu_simulater::elfsym::app_sym("slice_index_fail") as u64,
                                ]
                            });
                            if addrs.contains(&addr) {
                                FROZEN.store(true, O2::Relaxed);
                                return;
                            }
                            let i = FTRACE_POS.fetch_add(1, O2::Relaxed) as usize % 1024;
                            unsafe {
                                let p = core::ptr::addr_of_mut!(FTRACE_RING[i]);
                                core::ptr::write_volatile(p, addr as u32);
                            }
                        },
                    );
                    match r {
                        Ok(_) => eprintln!("[ftrace] 块钩子已安装 ✓"),
                        Err(e) => eprintln!("[ftrace] 块钩子安装失败 ✗: {e:?}"),
                    }
                }
            }
            // ★★§5.222【MSP/异常栈写监视 ✓】飞行记录仪已证明：野跳发生在
            //   `irq_dispatch` 的 `pop {r3,r4,r5,pc}` ⇒ 被污染的是 **MSP（异常栈，CCM）** ✗
            //   ⇒ 之前只盯 app 栈、还把 SP∈CCM 的写入全部排除 ✗ ⇒ 一直没看见 ✓
            {
                use std::sync::atomic::Ordering as O3;
                static MSP_ON: std::sync::atomic::AtomicBool =
                    std::sync::atomic::AtomicBool::new(false);
                if std::env::var("JOC_MSPWATCH").is_ok() && !MSP_ON.swap(true, O3::SeqCst) {
                    const MLO: u64 = 0x1000_9000;
                    const MHI: u64 = 0x1001_0000;
                    let r = self.m.cpu.add_mem_hook(
                        unicorn_engine::HookType::MEM_WRITE,
                        MLO,
                        MHI - 1,
                        move |uc, _t, addr, _sz, value| {
                            let sp = uc.reg_read(unicorn_engine::RegisterARM::SP).unwrap_or(0);
                            let pc = uc.reg_read(unicorn_engine::RegisterARM::PC).unwrap_or(0);
                            // 只抓"异常栈内的死区写入"：SP 也在此区、且写入低于 SP 64B~4KB
                            if sp >= MLO && sp < MHI && addr + 64 < sp && addr + 4096 > sp {
                                static SEEN: std::sync::atomic::AtomicU32 =
                                    std::sync::atomic::AtomicU32::new(0);
                                if SEEN.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 24 {
                                    eprintln!(
                                        "[msp] PC=0x{pc:08x} → 0x{addr:08x} 值=0x{value:08x} SP=0x{sp:08x}"
                                    );
                                }
                            }
                            false
                        },
                    );
                    match r {
                        Ok(_) => eprintln!("[msp] 异常栈监视已安装 ✓"),
                        Err(e) => eprintln!("[msp] 安装失败 ✗: {e:?}"),
                    }
                }
            }
            // ★★§5.223【emit 帧越界写监视 ✓】飞行记录仪已指认：`rtos_app_sdk::log::emit`
            //   的帧被写坏 ⇒ 返回时跳进它自己的字面量池 ✓
            //   判据：PC 落在 emit（0x0806f0ec..0x0806f440，本构建 ✓）或其被调 helper 里，
            //   且写入目标 **≥ SP+200**（buf[180] 之后 ✓ = 越过缓冲、正打在保存的 LR 上 ✓）
            {
                use std::sync::atomic::Ordering as O4;
                static EW_ON: std::sync::atomic::AtomicBool =
                    std::sync::atomic::AtomicBool::new(false);
                if std::env::var("JOC_EMITWATCH").is_ok() && !EW_ON.swap(true, O4::SeqCst) {
                    // emit 入口 0x0806f0ec = `push {r4,r5,r6,r7,lr}` ⇒ LR 存在 entry_SP − 4 ✓
                    //   记下每次进入 emit 的 SP，然后**任何**对 (entry_SP−4) 的写入
                    //   （除序言那条 push 自己 ✓）都是踩 LR = 真凶 ✓
                    static EMIT_SP: std::sync::atomic::AtomicU32 =
                        std::sync::atomic::AtomicU32::new(0);
                    let r = self.m.cpu.add_mem_hook(
                        unicorn_engine::HookType::MEM_WRITE,
                        0x2000_0000,
                        0x2001_0000, // app RAM（含控制栈 ✓）
                        move |uc, _t, addr, _sz, value| {
                            let sp = uc.reg_read(unicorn_engine::RegisterARM::SP).unwrap_or(0);
                            let pc = uc.reg_read(unicorn_engine::RegisterARM::PC).unwrap_or(0);
                            // 进入 emit（含序言 push）时记录入口 SP
                            if (0x0806_f0ec..0x0806_f0f6).contains(&pc) {
                                EMIT_SP.store(sp as u32, O4::Relaxed);
                            }
                            let e = EMIT_SP.load(O4::Relaxed) as u64;
                            // 序言自己那条 push 写的正是这块，排除它 ✓
                            let is_prologue = pc == 0x0806_f0ec;
                            if e != 0 && addr == e - 4 && !is_prologue {
                                static SEEN: std::sync::atomic::AtomicU32 =
                                    std::sync::atomic::AtomicU32::new(0);
                                if SEEN.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 24 {
                                    eprintln!(
                                        "[emit] ★踩 LR！PC=0x{pc:08x} → 0x{addr:08x}                                          值=0x{value:08x} SP=0x{sp:08x} emit入口SP=0x{e:08x}"
                                    );
                                }
                            }
                            false
                        },
                    );
                    match r {
                        Ok(_) => eprintln!("[emit] emit 帧监视已安装 ✓"),
                        Err(e) => eprintln!("[emit] 安装失败 ✗: {e:?}"),
                    }
                }
            }
            // ★★§5.224【捕获 panic 现场 ✓】`rtos_app_sdk::panic` 执行 `udf #0` 交 RTOS 恢复 ✗
            //   ⇒ 只要有人 panic，就记录 (LR, R0=index, R1=len, SP) ✓
            //   钩 `panic_bounds_check`(0x08068df4) 与 `slice_index_fail`(0x08068db0) 入口 ✓
            {
                use std::sync::atomic::Ordering as O5;
                static PB_ON: std::sync::atomic::AtomicBool =
                    std::sync::atomic::AtomicBool::new(false);
                if std::env::var("JOC_PANICWATCH").is_ok() && !PB_ON.swap(true, O5::SeqCst) {
                    // ★按名解析（不再硬编码 ✗ —— 之前用错二进制吃过亏 ✓）
                    let pb = mcu_simulater::elfsym::app_sym("panic_bounds_check") as u64;
                    let sf = mcu_simulater::elfsym::app_sym("slice_index_fail") as u64;
                    eprintln!("[panic] 解析到 panic_bounds_check=0x{pb:08x} slice_index_fail=0x{sf:08x}");
                    let mut pair: Vec<(u64, u64)> = vec![(sf, sf), (pb, pb)];
                    let mut hooks = Vec::new();
                    for (lo, hi) in pair.drain(..) {
                        let r = self.m.cpu.add_code_hook(lo, hi, move |uc, addr, _sz| {
                            let lr = uc.reg_read(unicorn_engine::RegisterARM::LR).unwrap_or(0);
                            let r0 = uc.reg_read(unicorn_engine::RegisterARM::R0).unwrap_or(0);
                            let r1 = uc.reg_read(unicorn_engine::RegisterARM::R1).unwrap_or(0);
                            let sp = uc.reg_read(unicorn_engine::RegisterARM::SP).unwrap_or(0);
                            static SEEN: std::sync::atomic::AtomicU32 =
                                std::sync::atomic::AtomicU32::new(0);
                            if SEEN.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 16 {
                                eprintln!(
                                    "[panic] 进入0x{addr:08x} **来自LR=0x{lr:08x}**                                      R0(index)=0x{r0:x}({r0}) R1(len)=0x{r1:x}({r1}) SP=0x{sp:08x}"
                                );
                            }
                        });
                        hooks.push(r);
                    }
                    eprintln!("[panic] panic 入口钩子: {:?}", hooks.iter().map(|h| h.is_ok()).collect::<Vec<_>>());
                }
            }
            // ★★§5.223【未映射写钩子 ✓】直接抓"出错那一笔写"的地址与现场寄存器 ✓
            {
                use std::sync::atomic::Ordering as O6;
                static UW_ON: std::sync::atomic::AtomicBool =
                    std::sync::atomic::AtomicBool::new(false);
                if std::env::var("JOC_UNMAPPED").is_ok() && !UW_ON.swap(true, O6::SeqCst) {
                    let r = self.m.cpu.add_mem_hook(
                        unicorn_engine::HookType::MEM_WRITE_UNMAPPED,
                        0,
                        u64::MAX,
                        move |uc, _t, addr, sz, value| {
                            let g = |rg| uc.reg_read(rg).unwrap_or(0);
                            eprintln!(
                                "[unmapped] **写0x{addr:08x}** size={sz} 值=0x{value:08x} \
                                 PC=0x{:08x} LR=0x{:08x} SP=0x{:08x} R0=0x{:08x} R2=0x{:08x} \
                                 R3=0x{:08x} R4=0x{:08x} R6=0x{:08x} R8=0x{:08x}",
                                g(unicorn_engine::RegisterARM::PC),
                                g(unicorn_engine::RegisterARM::LR),
                                g(unicorn_engine::RegisterARM::SP),
                                g(unicorn_engine::RegisterARM::R0),
                                g(unicorn_engine::RegisterARM::R2),
                                g(unicorn_engine::RegisterARM::R3),
                                g(unicorn_engine::RegisterARM::R4),
                                g(unicorn_engine::RegisterARM::R6),
                                g(unicorn_engine::RegisterARM::R8)
                            );
                            false
                        },
                    );
                    match r {
                        Ok(_) => eprintln!("[unmapped] 未映射写钩子已安装 ✓"),
                        Err(e) => eprintln!("[unmapped] 安装失败 ✗: {e:?}"),
                    }
                }
            }
            // ★★§5.223【循环头寄存器监视 ✓】只看 `align_yaw_to_mag` 清零循环头
            //   （本构建 0x0806f3e0 ✓），且**仅当基址寄存器变野**时打印 ⇒ 抓第一次变坏 ✓
            {
                use std::sync::atomic::Ordering as O7;
                static LOOP_ON: std::sync::atomic::AtomicBool =
                    std::sync::atomic::AtomicBool::new(false);
                if std::env::var("JOC_LOOPWATCH").is_ok() && !LOOP_ON.swap(true, O7::SeqCst) {
                    let lo = mcu_simulater::elfsym::app_sym("align_yaw_to_mag") as u64;
                    // 循环头 ≈ 函数内固定偏移；这里用"落在该函数区间内"的粗筛 + 只看野值 ✓
                    let hi = lo + 0x400;
                    eprintln!("[loop] 监视 align_yaw_to_mag 区间 0x{lo:08x}..0x{hi:08x}");
                    let r = self.m.cpu.add_block_hook(lo, hi, move |uc, addr, _sz| {
                        let g = |rg| uc.reg_read(rg).unwrap_or(0);
                        let (r0, r2, r3, r4, r5, r6, r8) = (
                            g(unicorn_engine::RegisterARM::R0),
                            g(unicorn_engine::RegisterARM::R2),
                            g(unicorn_engine::RegisterARM::R3),
                            g(unicorn_engine::RegisterARM::R4),
                            g(unicorn_engine::RegisterARM::R5),
                            g(unicorn_engine::RegisterARM::R6),
                            g(unicorn_engine::RegisterARM::R8),
                        );
                        // 只要任一"应指向 Eskf 内部"的基址离开 RAM 区 ⇒ 记为野值 ✓
                        let sw = |v: u64| v < 0x2000_0000 || v >= 0x2002_0000;
                        if sw(r0) || sw(r4) || (r4 != 0 && sw(r5)) {
                            static SEEN: std::sync::atomic::AtomicU32 =
                                std::sync::atomic::AtomicU32::new(0);
                            if SEEN.fetch_add(1, O7::Relaxed) < 20 {
                                eprintln!(
                                    "[loop] **野基址** @PC=0x{addr:08x} R0=0x{r0:08x} R2=0x{r2:08x} \
                                     R3=0x{r3:08x} R4=0x{r4:08x} R5=0x{r5:08x} R6=0x{r6:08x} R8=0x{r8:08x}"
                                );
                            }
                        }
                    });
                    eprintln!("[loop] 安装: {:?}", r.is_ok());
                }
            }
            // ★★§5.223【指令级记录（只限 align_yaw_to_mag）✓】把 (pc, r4, r2, r6, r8) 逐指令入环 ✓
            //   出错时打印最后 64 条 ⇒ **直接看到 r4 被谁写成 0** ✓（不再猜 ✓）
            {
                use std::sync::atomic::Ordering as O8;
                static IL_ON: std::sync::atomic::AtomicBool =
                    std::sync::atomic::AtomicBool::new(false);
                if std::env::var("JOC_ILTRACE").is_ok() && !IL_ON.swap(true, O8::SeqCst) {
                    let lo = mcu_simulater::elfsym::app_sym("align_yaw_to_mag") as u64;
                    let hi = lo + 0x400;
                    let r = self.m.cpu.add_code_hook(lo, hi, move |uc, addr, _sz| {
                        let g = |rg| uc.reg_read(rg).unwrap_or(0) as u32;
                        let i = ILPOS.fetch_add(1, O8::Relaxed) as usize % 256;
                        unsafe {
                            ILRING[i] = (addr as u32, g(unicorn_engine::RegisterARM::R4),
                                         g(unicorn_engine::RegisterARM::R2), g(unicorn_engine::RegisterARM::R6),
                                         g(unicorn_engine::RegisterARM::R8));
                        }
                    });
                    eprintln!("[il] 指令级记录安装: {:?} 区间0x{lo:08x}..0x{hi:08x}", r.is_ok());
                }
            }
            if want && !INSTALLED.swap(true, std::sync::atomic::Ordering::SeqCst) {
                const BASE: u64 = 0x2000_41b0;
                const TOP: u64 = BASE + 20480;
                self.m
                    .cpu
                    .add_mem_hook(
                        unicorn_engine::HookType::MEM_WRITE,
                        BASE,
                        TOP - 1,
                        move |uc, _ty, addr, _size, value| {
                            let sp = uc.reg_read(unicorn_engine::RegisterARM::SP).unwrap_or(0);
                            let pc = uc.reg_read(unicorn_engine::RegisterARM::PC).unwrap_or(0);
                            let lr = uc.reg_read(unicorn_engine::RegisterARM::LR).unwrap_or(0);
                            if !WATCH_ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
                                return false;
                            }
                            // ★排除 ISR/上下文切换上下文 ✗：异常处理用 MSP（CCM 0x1000_xxxx ✓），
                            //   而 `PendSV_Handler` 会**合法地**把被切换任务的寄存器存到它自己的栈上 ✓
                            //  （首版没排除 ⇒ 32 条"可疑"全是 `vstmdb r0!,{s16-s31}` ✗ 假阳性 ✓）
                            let sp_in_ccm = sp >= 0x1000_0000 && sp < 0x1001_0000;
                            if sp_in_ccm {
                                return false;
                            }
                            let sp_in = sp >= BASE && sp < TOP;
                            // ★第三版判据：只抓 SP 下方 64B~4KB 的"死区"写入 ✓（排除 push 假阳性）
                            let in_dead_zone = sp_in && addr + 64 < sp && addr + 4096 > sp;
                            if !sp_in || in_dead_zone {
                                // 读当前任务名：jOS `g_running`@0x100063b8 → TCB(sp@0,name@4) → C 字符串
                                let mut nm = [0u8; 16];
                                if let Ok(g) = uc.mem_read_as_vec(0x1000_63b8u64, 4) {
                                    let tcb = u32::from_le_bytes([g[0], g[1], g[2], g[3]]) as u64;
                                    if let Ok(np) = uc.mem_read_as_vec(tcb + 4, 4) {
                                        let np = u32::from_le_bytes([np[0], np[1], np[2], np[3]]) as u64;
                                        if let Ok(b) = uc.mem_read_as_vec(np, 15) {
                                            nm[..15].copy_from_slice(&b);
                                        }
                                    }
                                }
                                let e = nm.iter().position(|b| *b == 0).unwrap_or(15);
                                let who: String = nm[..e].iter().map(|b| *b as char).collect();
                                static SEEN: std::sync::atomic::AtomicU32 =
                                    std::sync::atomic::AtomicU32::new(0);
                                if SEEN.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 24 {
                                    let sp_in_ccm2 = sp >= 0x1000_0000 && sp < 0x1001_0000;
                                    eprintln!(
                                        "[write] PC=0x{pc:08x} → 0x{addr:08x} 值=0x{value:08x}                                          SP=0x{sp:08x} LR=0x{lr:08x} 任务={who} CCM上下文={sp_in_ccm2}"
                                    );
                                }
                            }
                            false // 不拦截，仅观察
                        },
                    )
                    .ok();
            }
        }
        let t_before = self.m.systick_ms();
        // ★§5.145：若挂了 PHY 后端且有摇杆 override ⇒ 控制拍**之前**也写一次
        //   （固件在拍内读 override ✓；此前只在拍后写 ⇒ 读到的是上一拍（首拍为 0）✗）
        if let Some(ref mut phy) = self.phy {
            phy.pre_tick(&mut self.m);
            phy.pre_tick_state(&self.st);
        }
        // ★§5.217 诊断（临时）：捕获模拟器错误并打印 PC/LR/SP，便于用 map 定位
        if let Err(e) = run_one_control_tick(&mut self.m) {
            use unicorn_engine::RegisterARM;
            // ★§5.217 诊断：模拟器故障时把 CPU 现场 + SCB 故障寄存器打出来 ✓
            //   （`run_one_control_tick` 原来只给字符串 ⇒ 无法定位 ✗；
            //     有了 PC 才能 addr2line、有了 BFAR/MMFAR 才能知道出错的**数据地址** ✓）
            macro_rules! rg {
                ($r:expr) => {
                    self.m.cpu.reg_read($r).unwrap_or(u32::MAX as u64) as u32
                };
            }
            macro_rules! scb {
                ($a:expr) => {{
                    let v = self.m.cpu.mem_read($a, 4).unwrap_or_default();
                    if v.len() == 4 { u32::from_le_bytes([v[0], v[1], v[2], v[3]]) } else { u32::MAX }
                }};
            }
            let (pc, lr, sp) = (rg!(RegisterARM::PC), rg!(RegisterARM::LR), rg!(RegisterARM::SP));
            // ★§5.223：MSP/PSP 分离情况（ISR 应在 MSP ✓；若 MSP 落在 app SRAM 就说明栈分离坏了 ✗）
            let (msp, psp) = (rg!(RegisterARM::MSP), rg!(RegisterARM::PSP));
            // ★RTOS 自带栈溢出检测的**粘性标志**（joc-base: g_stack_overflow @ 0x10008f5c ✓）
            //   比我自己涂色可靠 ✓（RTOS 在栈底写哨兵，涂色会踩到它 ✗）
            // ★§5.222：RTOS 自带三个检测器一起读 ✓（比自造启发式可靠 ✓）
            let sinv = self
                .m
                .cpu
                .mem_read(0x1000_6298u64, 4)
                .map(|v| u32::from_le_bytes([v[0], v[1], v[2], v[3]]))
                .unwrap_or(u32::MAX);
            let cfsr_sticky = self
                .m
                .cpu
                .mem_read(0x1000_8f58u64, 4)
                .map(|v| u32::from_le_bytes([v[0], v[1], v[2], v[3]]))
                .unwrap_or(u32::MAX);
            let ovf = self
                .m
                .cpu
                .mem_read(0x1000_8f5cu64, 4)
                .map(|v| u32::from_le_bytes([v[0], v[1], v[2], v[3]]))
                .unwrap_or(u32::MAX);
            let scb_s = format!(
                "CFSR=0x{:08x} HFSR=0x{:08x} MMFAR=0x{:08x} BFAR=0x{:08x}",
                scb!(0xE000_ED28u64), scb!(0xE000_ED2Cu64),
                scb!(0xE000_ED34u64), scb!(0xE000_ED38u64)
            );
            let regs = [
                rg!(RegisterARM::R0), rg!(RegisterARM::R1), rg!(RegisterARM::R2),
                rg!(RegisterARM::R3), rg!(RegisterARM::R4), rg!(RegisterARM::R5),
                rg!(RegisterARM::R6), rg!(RegisterARM::R7), rg!(RegisterARM::R8),
                rg!(RegisterARM::R9), rg!(RegisterARM::R10), rg!(RegisterARM::R11),
                rg!(RegisterARM::R12),
            ];
            // ★控制栈水位（§5.217）：涂 0xA5 后被写过的最高处 ⇒ 用量 = 栈顶 − 该处
            const CTRL_STACK_BASE: u64 = 0x2000_41b0;
            const CTRL_STACK_LEN: u64 = 20480;
            let mut deepest = CTRL_STACK_LEN;
            if let Ok(v) = self.m.cpu.mem_read(CTRL_STACK_BASE, CTRL_STACK_LEN as usize) {
                for (i, b) in v.iter().enumerate() {
                    if *b != 0xA5 {
                        deepest = i as u64;
                        break;
                    }
                }
            }
            let suspects = unsafe {
                if SUSPECT_CELL.is_null() {
                    String::from("(监视未安装)")
                } else {
                    let cell = &*(SUSPECT_CELL as *const std::sync::Mutex<Vec<(u32, u32, u32, u32)>>);
                    cell.lock()
                        .map(|v| {
                            let names = NAMES.lock().map(|n| n.clone()).unwrap_or_default();
                            v.iter()
                                .map(|(pc, a, val, sp)| {
                                    let who = names
                                        .iter()
                                        .find(|(ad, _)| *ad == *a)
                                        .map(|(_, n)| n.clone())
                                        .unwrap_or_default();
                                    format!("    PC=0x{pc:08x} 写入0x{a:08x} 值=0x{val:08x} LR=0x{sp:08x} 任务={who}\n")
                                })
                                .collect::<String>()
                        })
                        .unwrap_or_default()
                }
            };
            let ftrace = {
                let mut out = String::new();
                let pos = FTRACE_POS.load(std::sync::atomic::Ordering::Relaxed) as usize;
                for k in 0..160usize {
                    let idx = (pos + 1024 - 160 + k) % 1024;
                    // 环在**宿主**进程里 ✓ ⇒ 直接读宿主静态（不是模拟器内存 ✗）
                    let v = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(FTRACE_RING[idx])) };
                    out.push_str(&format!("0x{v:08x} "));
                }
                out
            };
            let il = {
                let mut o = String::new();
                let ipos = ILPOS.load(std::sync::atomic::Ordering::Relaxed) as usize % 256;
                for k in 0..40usize {
                    let idx = (ipos + 256 - 40 + k) % 256;
                    let e = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ILRING[idx])) };
                    o.push_str(&format!("\n    pc=0x{:08x} r4=0x{:08x} r2=0x{:08x} r6=0x{:08x} r8=0x{:08x}", e.0, e.1, e.2, e.3, e.4));
                }
                o
            };
            panic!(
                "run_one_control_tick 失败: {e:?}\n  **MSP=0x{msp:08x} PSP=0x{psp:08x}**（MSP 应在 CCM 0x1000_xxxx ✓）\n  **指令级(末40条)**{il}\n  **飞行记录(末160块)** = {ftrace}\n  PC=0x{pc:08x} LR=0x{lr:08x} SP=0x{sp:08x}\n  SCB {scb_s}\n  R0..R12 = {regs:08x?}\n  **RTOS: g_stack_overflow={ovf} · g_sched_invariant_fail={sinv} · g_fault_cfsr={cfsr_sticky}**\n  控制栈: 基址=0x{CTRL_STACK_BASE:08x} 长度={CTRL_STACK_LEN} 最深水位偏移={deepest}\n  **可疑写入（{n} 条）**:\n{suspects}",
                n = suspects.matches("PC=").count()
            );
        }
        let t_after = self.m.systick_ms();
        // ③ 按【实测流逝】推进场景（自洽 ✓：不假设 dt ✗）
        //    ★§5.143：若挂了 PHY 后端 ⇒ 用**真动力学**推进（取代运动学 ✓）
        if let Some(ref mut phy) = self.phy {
            phy.step_plant(&mut self.m, &self.st);
        } else {
            let elapsed_ms = (t_after.saturating_sub(t_before)) as f32;
            let dt = if elapsed_ms > 0.0 { elapsed_ms } else { 1.0 };
            self.scn.advance(dt / 1000.0);
            {
                let mut st = self.st.lock().unwrap();
                self.scn.write_state(&mut st);
            }
        }
        self.steps += 1;
        // 锁相漂移守卫（照 x_hover_noise 模板 ✓）：长期均值必须贴合名义控制周期 ✓
        // ★锁相基准 = 【首拍末】（照 `x_hover_noise` 模板 ✓）：
        //   开机到首拍有 ~200ms 初始化 ✗ ⇒ 若以【构造时】为 0 点，首步就会"漂移 200ms"✗✓
        if self.step0_ms == 0 {
            self.step0_ms = self.m.systick_ms();
        }
        let elapsed = self.m.systick_ms() - self.step0_ms;
        let expect = ((self.steps - 1) as f64 * 4.0) as u64; // 名义控制周期 4ms（250Hz）
        // ★★★2026-10-04【容忍调度量化 —— 照 PX4 的"声明周期 vs 实际派发"语义 ✓】：
        //   本仓 workq 调度器粒度 = **1ms**（内核定时器 ✗），而 item 周期是 4ms
        //   ⇒ 每次派发有 ±1ms 量化 ⇒ 长期累计漂移 ≈ 0.4ms/步（实测 520 步 201ms ✓，
        //     即 +9.5% ✗，与 ±1/4 = 12.5% 量级吻合 ✓）。
        //   PX4 侧无此问题：其 `WorkQueue` 由 **hrt_call（µs 分辨率）** 驱动 ✓。
        //   ⇒ 守卫改为"**10% + 固定 200ms**"（量化是固有项、非行为退化 ✗）。
        let tol = (expect / 10) + 200;
        if elapsed.abs_diff(expect) > tol {
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
