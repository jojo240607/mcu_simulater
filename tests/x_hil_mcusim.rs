//! HIL 联调：fly-simulater（PC 物理世界）↔ mcu_simulater（飞控 MCU）闭环。
//!
//! 组件与数据流（与 fly-sim-server 的 HIL 场景同构，链路换成虚拟口）：
//!   - PC 端物理世界：`fly_sim_core::sim::SimLoop`（PhySdkWorld 物理引擎，`step_hil`/`last_imu`）；
//!   - 链路：`fly_sim_hil::hil_link::HilLink`（`open_virtual` 接 [`McuSimPort`]，
//!     MAVLink 编解码/注入节奏/心跳/ARM 与真实板 HIL 完全同一份代码）；
//!   - MCU：mcu_simulater 跑 joc-base minimal + flyctrl-app（--features hil）：
//!     uplink 从 usb0 读 HIL_SENSOR/SET_POSITION 注入 SENSOR_FRAME，control 算执行器
//!     指令，telemetry 经 usb0 回 HEARTBEAT(HIL flag)/LOCAL_POSITION_NED/HIL_ACTUATOR_CONTROLS；
//!   - 虚拟 USB-CDC：usb_otg 外设（inject_out=PC→MCU 上行，host_take_in=MCU→PC 下行）。
//!
//! 构建前置：`cd joc-base && cmake --build build_rel`（minimal elf）、
//! `cd flyctrl && python3 build_app.py --features hil`（app.bin）。

use std::io;
use std::path::Path;

fn init_log() {
    // Info 级会打印 machine 的「中断进入/异常返回」每中断一行（PendSV 风暴时
    // 上万行刷屏拖慢模拟）——HIL 联调期间用 Warn 过滤，保留 run 风暴护栏告警。
    let _ = env_logger::builder().is_test(true).filter_level(log::LevelFilter::Warn).try_init();
}
use std::sync::{Arc, Mutex};
use std::time::Instant;

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use fly_sim_hil::hil_link::{HilLink, HilPort};
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample};
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;

/// 虚拟 USB-CDC 口（HilPort 后端）：EP1 OUT = PC→MCU 上行，EP1 IN = MCU→PC 下行。
/// 与固件 usb0（CDC 数据端点 0x81/0x01）一致。
struct McuSimPort {
    m: Arc<Mutex<Machine>>,
}

impl HilPort for McuSimPort {
    fn bytes_to_read(&self) -> io::Result<u32> {
        let mm = self.m.lock().unwrap();
        Ok(mm.usb_otg.lock().unwrap().in_tx_len(1) as u32)
    }
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mm = self.m.lock().unwrap();
        let mut u = mm.usb_otg.lock().unwrap();
        let data = u.host_take_in(1);
        let n = data.len().min(buf.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(n)
    }
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        let mm = self.m.lock().unwrap();
        let mut u = mm.usb_otg.lock().unwrap();
        u.inject_out(1, buf);
        Ok(())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// 注入一个 USB SETUP 包（设备模式主机侧），并 run 一小段让固件处理。
fn inject_setup(m: &Arc<Mutex<Machine>>, data: [u8; 8]) {
    let mut mm = m.lock().unwrap();
    mm.events
        .lock()
        .unwrap()
        .publish(&mcu_simulater::events::Event::UsbSetup { data });
    mm.run_budget(60_000).unwrap();
}

/// USB 总线枚举：复位 + 标准 4 个 SETUP（设备/配置描述符、地址、配置）。
fn usb_enumerate(m: &Arc<Mutex<Machine>>) {
    {
        let mut mm = m.lock().unwrap();
        mm.usb_otg.lock().unwrap().inject_usb_reset();
        mm.run_budget(60_000).unwrap();
    }
    // GET_DESCRIPTOR(Device, 18B)
    inject_setup(m, [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]);
    // GET_DESCRIPTOR(Config, 32B)
    inject_setup(m, [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x20, 0x00]);
    // SET_ADDRESS 0x2A
    inject_setup(m, [0x00, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00]);
    // SET_CONFIGURATION 1（使能 EP1 IN/OUT）
    inject_setup(m, [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
    m.lock().unwrap().run_budget(200_000).unwrap();
}

#[test]
fn hil_mcusim_closed_loop() {
    init_log();
    let sys = artifact::joc_base_elf();
    let app = artifact::flyctrl_app_bin();
    assert!(sys.exists(), "minimal elf 缺失：先 cd joc-base && cmake -S . -B build_hil -DMCU_SIM=ON -DRTOS_SELFTEST=OFF && cmake --build build_hil");
    assert!(app.exists(), "flyctrl-app 缺失：先 cd flyctrl && python3 build_app.py --features hil");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&sys).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();

    // 固件启动 + App 挂载 + 任务创建（telemetry/uplink/control/sensors）
    for _ in 0..12 {
        m.run_budget(1_000_000).unwrap();
    }
    let m = Arc::new(Mutex::new(m));

    // USB 枚举（虚拟主机侧）
    usb_enumerate(&m);

    // HIL 链路：虚拟口 → HilLink（同一份 MAVLink HIL 协议）
    let mut link = HilLink::open_virtual(Box::new(McuSimPort { m: m.clone() }));

    // ---- 等 HIL 心跳（telemetry 20ms 周期，hil feature 下 base_mode 带 HIL flag）----
    let mut waited = 0u32;
    while !link.is_hil_ready() {
        m.lock().unwrap().run_budget(800_000).unwrap();
        link.poll();
        waited += 1;
        if waited == 10 {
            let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
            let t = String::from_utf8_lossy(&out);
            let n = t.len();
            eprintln!("[hil-mcusim] === console FULL ({n}B) ===\n{}", &t[..n.min(1200)]);
            eprintln!("[hil-mcusim] === console TAIL ===\n{}", &t[n.saturating_sub(800)..]);
        }
        assert!(waited < 40, "40 轮未等到 HIL 心跳（usb0 下行未通？rx_buf={}", link.rx_buf_len());
    }
    eprintln!("[hil-mcusim] HIL 心跳确认（链路建立）");
    {
        let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
        let t = String::from_utf8_lossy(&out);
        let n = t.len();
        eprintln!("[hil-mcusim] === console AFTER HEARTBEAT ({n}B) ===\n{}", &t[n.saturating_sub(1200)..]);
    }

    // ---- ARM（解锁）----
    link.send_arm(true).unwrap();
    m.lock().unwrap().run_budget(600_000).unwrap();

    // ---- 闭环：PC 物理步 ↔ MCU 控制拍 ----
    // PC 每步：读回 actuator → 推进物理 → 注入 IMU/GPS/SET_POSITION 真值
    //（nav 节流 HIL_NAV_EVERY=8 步，与 fly-sim-server 一致）。MCU 由 run 预算推进。
    let mut sim = SimLoop::new(
        PhySdkWorld::create_empty(),
        &VehicleConfig::default_quad(),
        0.004,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(fly_sim_core::physics::ContactModel::default()),
        vec![],
    );

    let t0 = Instant::now();
    let mut held = true; // 起飞台保持：MCU 未输出推力前不推进物理
    let mut max_thrust = 0.0f32;
    let mut final_state = None;
    let mut rx_frames = 0u32;
    for step in 0..300u64 {
        // MCU 侧推进（处理上行注入 + 跑控制 + 发下行遥测）
        m.lock().unwrap().run_budget(200_000).unwrap();
        if step % 100 == 0 {
            let (d, tin, ip1, ctl1, tsiz1, tx1, tx0, daintmsk, diepmsk, int1) = {
                let mm = m.lock().unwrap();
                let uo = mm.usb_otg.lock().unwrap();
                (uo.dbg, uo.in_tx_len(1), uo.in_pending_obs(1),
                 uo.reg_obs(0x928), uo.reg_obs(0x930), uo.reg_obs(0x924),
                 uo.in_tx_len(1), uo.in_tx_len(0),
                 uo.daintmsk(), uo.diepmsk_obs())
            };
            eprintln!("[hil-mcusim] regs ctl1=0x{ctl1:08X} tsiz1=0x{tsiz1:08X} tx1={tx1} tx0={tx0} daintmsk=0x{daintmsk:08X} diepmsk=0x{diepmsk:08X}");
            eprintln!("[hil-mcusim] usb_dbg txfe={} fifo={} xfrc={} ep1xfrc={} w1c={} take={} epena={} epena1={} ip1={} tf_fail_epena={} tf_fail_pend={} tf_short={}",
                d[0], d[1], d[2], d[6], d[3], d[4], d[5], d[10], ip1, d[7], d[8], d[9]);
            eprintln!("[hil-mcusim] regs ctl1=0x{:08X} tsiz1=0x{:08X} int1=0x{:08X} daintmsk=0x{:08X} diepmsk=0x{:08X} tx1={}",
                ctl1, tsiz1, int1, daintmsk, diepmsk, tx1);
            let rxl = m.lock().unwrap().usb_otg.lock().unwrap().rx_len_obs();
            eprintln!("[hil-mcusim] step={step} PRE-poll in_tx_len={tin} rx_len={rxl}");
        }
        link.poll();
        rx_frames += 1;
        // 心跳保持
        if step == 60 {
            let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
            let t = String::from_utf8_lossy(&out);
            let n = t.len();
            eprintln!("[hil-mcusim] === console@60 ({n}B) ===\n{}", &t[n.saturating_sub(1800)..]);
        }
        if !link.is_hil_ready() {
            eprintln!("[hil-mcusim] 链路中断 at step {step}");
            break;
        }
        let cmd = ActuatorCmd { motor: link.actuator() };
        let thrust = cmd.motor.iter().sum::<f32>();
        max_thrust = max_thrust.max(thrust);
        let nav = step % 8 == 0;
        let (st, imu_true) = if held && thrust < 0.05 {
            // 起飞台保持：注入悬停真值（FRD 静止比力 + 零角速度）
            (None, ImuSample {
                accel: [
                    flyctrl_core::units::MeterPerSecondSquared(0.0),
                    flyctrl_core::units::MeterPerSecondSquared(0.0),
                    flyctrl_core::units::MeterPerSecondSquared(-9.81),
                ],
                gyro: [flyctrl_core::units::RadianPerSecond(0.0); 3],
            })
        } else {
            held = false;
            let st = sim.step_hil(&cmd);
            (Some(st), sim.last_imu())
        };
        final_state = st;
        // 注入真值：位置/速度用物理真值（或保持时初始悬停点 NED (0,0,-5)）
        let (pos, vel) = match st {
            Some(s) => ([s.pos[0].0, s.pos[1].0, s.pos[2].0], [s.vel[0].0, s.vel[1].0, s.vel[2].0]),
            None => ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0]),
        };
        let yaw = match st { Some(s) => s.att.yaw(), None => 0.0 };
        let pressure_alt = -pos[2];
        link.inject(
            step * 4000,
            &imu_true,
            yaw,
            pressure_alt,
            pos,
            vel,
            nav,
        );
        // 注入后 run 一段让固件消费上行（uplink 1ms 轮询）
        m.lock().unwrap().run_budget(200_000).unwrap();
        if step % 100 == 0 {
            if step == 100 {
                let v = m.lock().unwrap().vec_entries();
                let get = |n: u32| v.iter().find(|(x, _)| *x == n).map(|(_, c)| *c).unwrap_or(0);
                eprintln!("[hil-mcusim] IRQ: PendSV={} SysTick={} USB={}",
                    get(14), get(15), get(83));
                let rd_f = |addr: u64| -> f32 {
                    let b = m.lock().unwrap().cpu.mem_read(addr, 4).unwrap_or_default();
                    f32::from_le_bytes([b[0], b[1], b[2], b[3]])
                };
                eprintln!("[hil-mcusim] mem: CTRL_ATT_I={:.0} CTRL_POS_I={:.0} HIL_AIN={:.0}",
                    rd_f(0x2000b6c4), rd_f(0x2000b6c8), rd_f(0x2000b6e8));
            }
            let att = link.mcu_att().map(|a| (a.roll, a.pitch, a.yaw));
            let af = link.actuator_full;
            eprintln!(
                "[hil-mcusim] step={step} t={:.2}s thrust={thrust:.3} mcu_att={att:?} m=[{:.3},{:.3},{:.3},{:.3}] thr={:.3} pqr=({:.2},{:.2},{:.2}) calls={:.0} accd={:.3} att_i={:.0} pos_i={:.0} cnt={:.0} az={:.2}",
                step as f64 * 0.004, af[0], af[1], af[2], af[3], af[4], af[5], af[6], af[7],
                af[8], af[9], af[10], af[11], af[12], af[14],
            );
        }
    }
    let wall = t0.elapsed();

    // ---- 断言（数值判据）----
    {
        let out = m.lock().unwrap().console.lock().unwrap().output().to_vec();
        let t = String::from_utf8_lossy(&out);
        let n = t.len();
        eprintln!("[hil-mcusim] === console FINAL ({n}B) ===\n{}", &t[n.saturating_sub(4500)..]);
    }
    let st = final_state.expect("物理从未推进（起飞台一直保持？）");
    assert!(max_thrust > 0.05, "MCU 未回传有效推力（HIL_ACTUATOR_CONTROLS 未通？）");
    assert!(link.is_hil_ready(), "闭环中链路中断");
    // 不发散：位置/速度/姿态有限
    for v in [st.pos[0].0, st.pos[1].0, st.pos[2].0] {
        assert!(v.is_finite() && v.abs() < 100.0, "位置发散：{v}");
    }
    assert!(st.att.roll().is_finite() && st.att.roll().abs() < 1.0, "roll 发散：{}", st.att.roll());
    assert!(st.att.pitch().is_finite() && st.att.pitch().abs() < 1.0, "pitch 发散：{}", st.att.pitch());
    eprintln!(
        "[hil-mcusim] 闭环完成：{rx_frames} 轮 poll，墙钟 {:.1}s，max_thrust={max_thrust:.3}，末态 pos=({:.2},{:.2},{:.2}) roll={:.3}° pitch={:.3}°",
        wall.as_secs_f64(), st.pos[0].0, st.pos[1].0, st.pos[2].0,
        st.att.roll().to_degrees(), st.att.pitch().to_degrees(),
    );
    eprintln!(">>> [HIL-MCUSIM] PC 物理世界 ↔ mcu_simulater 飞控 MCU 闭环验证通过 ✓");
}
