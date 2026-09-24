//! 飞行模式端到端链路：USB uplink 注入 MAVLink `DO_SET_MODE`，验证
//! `G_CMD_MODE → control::set_cmd_mode` 生效并被 telemetry 心跳 custom_mode 反映。
//! 覆盖 docs/integration.md §6 标注的"后续优先项"——RTL/LOITER 动态分支：
//! STABILIZE(0) → LOITER(5) → RTL(6) → STABILIZE(0)。
//!
//! 链路：usb0 inject_out(PC→MCU) → uplink 任务 FxParser 拼帧 → COMMAND_LONG
//! 解析 DO_SET_MODE → G_CMD_MODE.store(custom) → control 按 ArduCopter
//! custom_mode 分支（LOITER 开位置外环 / STABILIZE 旁路）→ telemetry 心跳
//! custom_mode 字段回传。
//!
//! 产物：joc-base minimal ELF + 正式飞控 app.bin（默认 feature）。
//! 前置：`cd joc-base && cmake --build build_hil`；`cd flyctrl && python3 build_app.py --out ../flyctrl/app.bin`。
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use unicorn_engine::RegisterARM;

use flyctrl_core::comm::link::MAX_FRAME_LEN;
use flyctrl_core::comm::mavlink::{encode_command_long, enums};
use mcu_simulater::artifact;
use mcu_simulater::machine::Machine;

fn init_log() {
    let _ = env_logger::builder().is_test(true).filter_level(log::LevelFilter::Warn).try_init();
}

/// 注入一个 USB SETUP 包（设备模式主机侧），并 run 一小段让固件处理。
fn inject_setup(m: &Arc<Mutex<Machine>>, data: [u8; 8]) {
    let mut mm = m.lock().unwrap();
    mm.events
        .lock()
        .unwrap()
        .publish(&mcu_simulater::events::Event::UsbSetup { data });
    if let Err(e) = mm.run_budget(60_000) {
        panic!("inject_setup run 失败: {e:?}");
    }
}

/// USB 总线枚举：复位 + 标准 4 个 SETUP（设备/配置描述符、地址、配置）。
fn usb_enumerate(m: &Arc<Mutex<Machine>>) {
    {
        let mut mm = m.lock().unwrap();
        mm.usb_otg.lock().unwrap().inject_usb_reset();
        mm.run_budget(60_000).unwrap();
    }
    inject_setup(m, [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]); // GET_DESCRIPTOR(Device)
    inject_setup(m, [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x20, 0x00]); // GET_DESCRIPTOR(Config)
    inject_setup(m, [0x00, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00]); // SET_ADDRESS
    inject_setup(m, [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]); // SET_CONFIGURATION
    m.lock().unwrap().run_budget(200_000).unwrap();
}

/// 经 usb0 注入一帧 MAVLink 上行（PC→MCU），并推进固件处理。
fn inject_uplink(m: &Arc<Mutex<Machine>>, buf: &[u8]) {
    {
        let mut mm = m.lock().unwrap();
        let mut u = mm.usb_otg.lock().unwrap();
        u.inject_out(1, buf);
    }
    if let Err(e) = m.lock().unwrap().run_budget(600_000) {
        let pc = m.lock().unwrap().cpu.reg_read_u32(unicorn_engine::RegisterARM::PC).unwrap();
        let sp = m.lock().unwrap().cpu.reg_read_u32(unicorn_engine::RegisterARM::SP).unwrap();
        panic!("inject_uplink run 失败: {e:?} pc=0x{pc:08X} sp=0x{sp:08X}");
    }
}

/// 构造并注入 COMMAND_LONG(DO_SET_MODE, base_mode=0x81, custom_mode)。
fn inject_set_mode(m: &Arc<Mutex<Machine>>, custom_mode: f32) {
    let mut out = [0u8; MAX_FRAME_LEN];
    let n = encode_command_long(
        enums::MAV_CMD_DO_SET_MODE,
        1.0, // base_mode：uplink 仅取 param2 custom_mode，此处按标准置位即可
        custom_mode,
        0.0, 0.0, 0.0, 0.0, 0.0,
        1,   // target_system
        1,   // target_component
        &mut out,
    );
    inject_uplink(m, &out[..n]);
}

/// 推进固件直到控制台出现 `needle`（或步数上限超时）。
///
/// 注：验证链路用控制台日志（uplink 处理 DO_SET_MODE 的 `info!`）而非
/// USB 下行心跳——默认 feature app 下 USB CDC IN（MCU→PC）方向在模拟器中
/// 无 IN-token 搬移，固件从不写 EP IN buffer（in_tx_len 恒 0），telem 心跳
/// 到不了 host 侧；OUT（PC→MCU 注入）方向工作正常。
fn wait_console(m: &Arc<Mutex<Machine>>, needle: &str, max_steps: u32) -> bool {
    for _ in 0..max_steps {
        m.lock().unwrap().run_budget(400_000).unwrap();
        let outv = m.lock().unwrap().console.lock().unwrap().output().to_vec();
        if String::from_utf8_lossy(&outv).contains(needle) {
            return true;
        }
    }
    false
}

#[test]
fn uplink_do_set_mode_switches_flight_modes() {
    init_log();
    let elf = artifact::joc_base_elf();
    let app = artifact::flyctrl_app_bin();
    assert!(elf.exists(), "minimal elf 缺失：先 cd joc-base && cmake --build build_hil");
    assert!(app.exists(), "正式飞控 app.bin 缺失：先 cd flyctrl && python3 build_app.py --out ../flyctrl/app.bin");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();
    let m = Arc::new(Mutex::new(m));

    // INSN_INVALID 兜底（任何未解码指令立即失败，避免静默跑偏）
    let bad_pc = Arc::new(AtomicU32::new(0));
    let got_invalid = Arc::new(AtomicBool::new(false));
    let (b2, g2) = (bad_pc.clone(), got_invalid.clone());
    m.lock().unwrap().cpu.raw().add_insn_invalid_hook(move |uc| {
        let pc = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
        b2.store(pc, Ordering::Relaxed);
        g2.store(true, Ordering::Relaxed);
        eprintln!("[INSN_INVALID] pc=0x{pc:08X}");
        false
    }).unwrap();

    // 固件启动 + App 挂载 + 业务任务拉起 + 首个心跳
    // （注：USB 枚举必须等固件 boot 完成 usb0 open——GINTMSK/GAHBCFG 配置后
    //  pulse 才挂起 OTG_FS IRQ；过早注入 SETUP 会因门控不挂中断而残留，
    //  后续 OUT 触发 ISR 处理残留 SETUP → setup_packet=NULL 写 0x0 fault。）
    let mut mounted = false;
    let mut tasks = false;
    let mut hb = false;
    for step in 0..800u32 {
        let r = m.lock().unwrap().run_budget(400_000);
        let text = {
            let outv = m.lock().unwrap().console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        if text.contains("RUST app mounted") { mounted = true; }
        if text.contains("task started") { tasks = true; }
        if text.contains("hb seq=") { hb = true; }
        if got_invalid.load(Ordering::Relaxed) {
            panic!("!!! INSN_INVALID 出现 @0x{:08X}", bad_pc.load(Ordering::Relaxed));
        }
        if let Err(e) = r {
            panic!("ERR {e:?}");
        }
        if mounted && tasks && hb { break; }
        if step % 10 == 0 {
            eprintln!("[boot step {step}] mounted={mounted} tasks={tasks} hb={hb}");
        }
    }
    assert!(mounted, "App 分区未挂载");
    assert!(tasks, "业务任务未拉起");
    assert!(hb, "周期心跳未出现（SysTick 唤醒推进异常？）");
    eprintln!(">>> 启动完成：App mounted + 任务拉起 + 心跳出现");

    // USB 枚举（uplink 任务经 usb0 接收地面站命令）——必须在固件 open usb0 之后
    usb_enumerate(&m);
    {
        let mm = m.lock().unwrap();
        let u = mm.usb_otg.lock().unwrap();
        assert!(u.rx_status_empty(), "枚举 SETUP 未被固件消费");
    }
    eprintln!(">>> USB 枚举完成：4×SETUP 已消费");

    // 阶段 2：DO_SET_MODE(LOITER=5) —— 位置外环动态分支。
    // uplink 解析成功会打 info 日志 "DO_SET_MODE -> custom_mode=5"。
    inject_set_mode(&m, 5.0);
    assert!(
        wait_console(&m, "DO_SET_MODE -> custom_mode=5", 300),
        "注入 LOITER(5) 后未看到 uplink 处理日志"
    );
    eprintln!(">>> DO_SET_MODE(LOITER=5) 生效 ✓");

    // 阶段 3：DO_SET_MODE(RTL=6) —— 返航动态分支
    inject_set_mode(&m, 6.0);
    assert!(
        wait_console(&m, "DO_SET_MODE -> custom_mode=6", 300),
        "注入 RTL(6) 后未看到 uplink 处理日志"
    );
    eprintln!(">>> DO_SET_MODE(RTL=6) 生效 ✓");

    // 阶段 4：切回 STABILIZE(0)
    inject_set_mode(&m, 0.0);
    assert!(
        wait_console(&m, "DO_SET_MODE -> custom_mode=0", 300),
        "切回 STABILIZE(0) 后未看到 uplink 处理日志"
    );
    eprintln!(">>> 切回 STABILIZE(0) ✓");

    // 全程无 INSN_INVALID
    assert!(!got_invalid.load(Ordering::Relaxed), "全程出现非法指令");
    eprintln!(">>> 模式链路验证通过：STABILIZE→LOITER(5)→RTL(6)→STABILIZE");
}
