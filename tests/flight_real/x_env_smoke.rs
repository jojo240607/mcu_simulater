//! 冒烟：静态悬停 —— 验证「虚拟设备直接模拟」全链路。
//!
//! 链路：EnvScenario(Hover) → FlySimState → 虚拟外设 I2C/UART → real-sensors
//! 固件真实驱动 → EKF。断言：
//! - hb 心跳持续（imu/baro/gps 健康全 true，任务不冻结）；
//! - EKF 收敛：悬停静止时速度估计 ≈ 0、姿态 ≈ 水平、高度 ≈ 真值；
//! - SENSOR_SEQ 持续推进（sensors 任务不冻结）；
//! - 无非法指令 / 无 panic。

#[path = "../common/mod.rs"]
mod common;

use common::{EnvHarness, EstReadout};
use mcu_simulater::env::scenario::{EnvScenario, Motion};

/// 布局探针 + 全链路冒烟：任务推进、EKF 输出可读、health/armed 值域正确。
#[test]
fn est_layout_probe() {
    let scn = EnvScenario::new(Motion::Hover, mcu_simulater::env::scenario::Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);
    // 跑 400 步（~2.7s 虚拟时间），EKF 应已稳定。
    h.run_for_ms(400 as f64 * 13.0);
        {
            let t = h.truth();
            eprintln!("[probe] TRUTH att(rpy rad) = {:?} | pos = {:?}", t.att, t.pos);
        }
    for l in h.console_all().lines() {
        if l.contains("diag") { eprintln!("[console] {}", l); }
    }
    let e = h.read_est();
    // 判别用打印（2026-10-05）：只用**本文件已用过**的 EstReadout 字段（零新 API 风险）
    // 问题意识：探针断言"机体在悬停"，但机体是否真的水平**从未被验证**。
    // health/armed 若异常（降级/未解锁）⇒ 机体不在正常状态 ⇒ 姿态大角可能是【真实】的。
    eprintln!(
        "[probe] est wxyz = {:?} | health = {} | armed = {}",
        e.att_wxyz, e.health, e.armed
    );
    assert!(
        e.att_wxyz[0].abs() > 0.9,
        "姿态四元数 w 应 ≈1（单位四元数，布局/读取可能错位），得 {}",
        e.att_wxyz[0]
    );
    assert!(
        e.health <= 2,
        "health 应在 0..=2（Nominal/Degraded/Critical），得 {}（EST 布局偏移可能变了）",
        e.health
    );
    assert!(
        e.armed == 0 || e.armed == 1,
        "armed 应为 0/1，得 {}（EST 布局偏移可能变了）",
        e.armed
    );
    // EKF 高度收敛（Hover 真值 0m）：初始 ~2m，应收敛到 <1m
    assert!(
        e.pos[2].abs() < 1.0,
        "EKF 高度应收敛到 ~0m，得 {}",
        e.pos[2]
    );
    // SENSOR_SEQ 持续推进
    let s1 = h.read_sensor_seq();
    h.run_for_ms(50 as f64 * 13.0);
    let s2 = h.read_sensor_seq();
    assert!(s2 > s1, "SENSOR_SEQ 应持续推进（sensors 任务冻结？）：{s1} → {s2}");
    eprintln!("RESULT: layout ok, pos[2]={:.2} health={} sensor_seq {s1}→{s2}", e.pos[2], e.health);
}

/// 静态悬停收敛：速度≈0、姿态≈水平、高度≈真值、无漂移发散。
#[test]
fn hover_converges_stable() {
    let scn = EnvScenario::new(Motion::Hover, mcu_simulater::env::scenario::Perturb::clean(), vec![]);
    let mut h = EnvHarness::new(scn, true);

    // ★★★相位剖分（固件侧自计 STAGE_CYC/STAGE_N ✓，ELFSYM 直读 ✓）
    //   ⚠**必须先跑起来再读**：`STAGE_CYC`/`STAGE_N`/`PROBE` 都是 .bss 里的累加量，
    //   开机即读必为全 0 ✗ —— 此前 "raw 非零项:" 为空、`PROBE[0]=0`、
    //   `DWT_CTRL=0x0/CYCCNT=0` 全部由此而来（读点在首拍之前，DWT 还没被内核使能）。
    for _ in 0..2 {
        h.run_for_ms(1000.0);
    }
    {
        // 预热（解锁不必要，估计器独立于 armed；跑足让 EKF 收敛）
        let names = ["0 step_hil入口","1 IMU预处理","2 姿态/位置门控","3 EKF预测+更新完",
                     "4 外部观测注入完","5 FDIR完","6 健康闸+EKF完","7 限幅完",
                     "8 ESKF step进入","9 ESKF predict完",
                     "10 重力完","11 GPS位前","12 GPS位后","13 GPS速后","14 空速后",
                     "15 state组装前","16 气压前","17 气压后","18 磁前",
                     "19 gpsP:构H完","20 gpsP:gain_apply完","21 gpsP:update_vec3完",
                     "22 pred:F构建完","23 pred:Q构建完","24 uv3:PHᵗ完","25 uv3:S+inv3完",
                     "26 uv3:增益K完","27 uv3:HP完","28 uv3:Step1完","29 uv3:PH2完",
                     "30 uv3:Step2完","31 uv3:钳位+拷回完","32 pred:协方差传播完",
                     "33","34","35","36","37","38","39",
                     "40 att:循环顶","41 att:参数同步后","42 att:sync_gains完","43 att:读帧+设定点完",
                     "44 att:姿态层完","45 att:发布前","46 att:发布完","47","48"];
        eprintln!("[ph] elf={:?}", mcu_simulater::elfsym::flyctrl_app_elf());
        eprintln!("[ph] sym STAGE_CYC={:?} STAGE_N={:?} ESKF_PRED={:?}",
                  mcu_simulater::elfsym::try_app_sym("STAGE_CYC"),
                  mcu_simulater::elfsym::try_app_sym("STAGE_N"),
                  mcu_simulater::elfsym::try_app_sym("ESKF_PRED"));
        if let (Some(a), Some(b)) = (mcu_simulater::elfsym::try_app_sym("STAGE_CYC"),
                                     mcu_simulater::elfsym::try_app_sym("STAGE_N")) {
            if let (Ok(c), Ok(n)) = (h.m.cpu.mem_read(a as u64, 48*4), h.m.cpu.mem_read(b as u64, 48*4)) {
                {
            // PROBE 在固定 VMA 0x2001F100：[0]=段号 [1]=调用计数
            if let Ok(b) = h.m.cpu.mem_read(0x2001_F100u64, 8) {
                let st = u32::from_le_bytes([b[0],b[1],b[2],b[3]]);
                let cn = u32::from_le_bytes([b[4],b[5],b[6],b[7]]);
                eprintln!("[ph] PROBE[0]={st} PROBE[1](stage0调用数)={cn}");
            }
            // 同时看 SYSTICK/时基是否推进（对照）
            if let Ok(b) = h.m.cpu.mem_read(0xE000_1000u64, 8) {
                eprintln!("[ph] DWT_CTRL=0x{:08X} DWT_CYCCNT={}",
                          u32::from_le_bytes([b[0],b[1],b[2],b[3]]),
                          u32::from_le_bytes([b[4],b[5],b[6],b[7]]));
            }
        }
        eprintln!("[ph] ---- 相位剖分（累计 DWT 周期 @168MHz ⇒ ms）----");
                {
                    let mut raw = String::new();
                    for i in 0..48usize {
                        let nn = u32::from_le_bytes([n[i*4],n[i*4+1],n[i*4+2],n[i*4+3]]);
                        let cy = u32::from_le_bytes([c[i*4],c[i*4+1],c[i*4+2],c[i*4+3]]);
                        if nn != 0 || cy != 0 { raw.push_str(&format!(" [{i}:n={nn},c={cy}]")); }
                    }
                    eprintln!("[ph] raw 非零项:{raw}");
                }
                for i in 0..48usize {
                    let cy = u32::from_le_bytes([c[i*4],c[i*4+1],c[i*4+2],c[i*4+3]]);
                    let nn = u32::from_le_bytes([n[i*4],n[i*4+1],n[i*4+2],n[i*4+3]]);
                    if nn > 0 {
                        eprintln!("[ph] {:<18} cyc={:>12} (n={:>5}) ⇒ {:>8.3} ms/次",
                                  names[i], cy, nn, cy as f32 / 168_000.0 / nn as f32);
                    }
                }
            }
        }
    }
    // ★★★时间基准实测：固件自己的 tick 计数 vs harness 步进/虚拟时间 ✓
    {
        // ★`g_tick` 已不在固件符号表 ⇒ 改用 **DWT_CYCCNT**（内核 `rtos_cycle_init()`
        //   使能、模拟器 `peripheral/dwt.rs` 实现）直接标定 **cyc / 固件ms**：
        //   这个系数决定 `IT_EXEC_*` 与 `STAGE_CYC` 的 cyc→ms 换算是否真的 168k/ms ✓✗。
        let dwt = |h: &mut EnvHarness| -> u32 {
            h.m.cpu
                .mem_read(0xE000_1004u64, 4)
                .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                .unwrap_or(0)
        };
        // ★★钟源三方对照（**增量**口径 ✓ —— 避免"上电初期 RVR=0 窗口"污染全量比值 ✗）
        //   要回答：DWT 实测 ~191.6k/固件ms vs 规范 168k —— **哪一侧在说谎？**
        //   · 若 `实收周期 == DWT 增量` ⇒ 两边同源（均为 block_cycles ✓），“168k/ms”的前提不成立；
        //   · 若 `溢出次数 > 异常进入次数` ⇒ SysTick **挂起位被合并**（一次 tick 内多次溢出
        //     只投递一次 ✗）⇒ `fw_ms()`（= 向量 15 进入次数，见 `machine.rs:2438`）**少记固件毫秒** ✗。
        let dwt = |h: &mut EnvHarness| -> u32 {
            h.m.cpu
                .mem_read(0xE000_1004u64, 4)
                .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                .unwrap_or(0)
        };
        // (DWT, SCB 实收周期, SysTick 溢出, 挂起设定, 退休字节, 固件ms)
        let snap = |h: &mut EnvHarness| -> (u32, u64, u64, u32, u64, u64) {
            let d = dwt(h);
            let (pend, ovf) = h.m.syst_diag_counts();
            (d, h.m.scb_cycles_in(), ovf, pend, h.m.retired_count(), h.fw_ms())
        };
        let sq0 = h.read_sensor_seq();
        let st0 = h.steps;
        let a = snap(&mut h);
        h.run_for_ms(1000.0);
        let b = snap(&mut h);
        let d_steps = h.steps - st0;
        let sq1 = h.read_sensor_seq();
        let d_dwt = b.0.wrapping_sub(a.0) as u64;
        let (d_scb, d_ovf, d_pend, d_ret) = (b.1 - a.1, b.2 - a.2, b.3 - a.3, b.4 - a.4);
        let d_ms = (b.5 - a.5).max(1);
        eprintln!(
            "[tb] steps {st0} → {} (Δ{d_steps}), seq {sq0} → {sq1}",
            st0 + d_steps
        );
        eprintln!(
            "[tb] 增量窗口：DWT={d_dwt}  SCB实收={d_scb}  SysTick溢出={d_ovf}  挂起设定={d_pend}  退休字节={d_ret}  固件ms={d_ms}"
        );
        eprintln!(
            "[tb] ⇒ DWT/固件ms = **{:.0}**（名义 168000）· DWT/溢出 = {:.0}（应 168000 = RVR+1）· 实收/溢出 = {:.0}",
            d_dwt as f64 / d_ms as f64,
            d_dwt as f64 / d_ovf.max(1) as f64,
            d_scb as f64 / d_ovf.max(1) as f64
        );
        eprintln!(
            "[tb] ⇒ 溢出 {d_ovf} vs 异常进入 {d_ms}：差 {}（{:.2}%）⇒ {}",
            d_ovf as i64 - d_ms as i64,
            100.0 * (d_ovf as f64 - d_ms as f64) / (d_ovf.max(1) as f64),
            if d_ovf > d_ms + 2 {
                "SysTick 挂起被合并 ⇒ fw_ms 少记固件毫秒 ✗"
            } else {
                "溢出≈进入 ⇒ fw_ms 可信 ✓（则 168k 假设本身有问题 ✗）"
            }
        );
    }
    // ★仪器标定：**DWT 计数 ↔ 退休字节 ↔ 每次估计器调用** ✓
    //   为何必要：`block_cycles(size) = size × 633/1000` 是**指令量代理**（访客字节×系数），
    //   **不是真实周期** ✗ ⇒ 相位剖分里那些"cyc/MAC"的结论必须先靠本标定落地 ✓
    //   （此前把 PH2 段算成 57 cyc/MAC 并据此去改循环 ⇒ 实测**反而 +8.7%** ✗ ⇒ 模型错了）。
    {
        let dwt = |h: &mut EnvHarness| -> u32 {
            h.m.cpu
                .mem_read(0xE000_1004u64, 4)
                .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                .unwrap_or(0)
        };
        // `STAGE_N[0]` = `probe(0)` 的采样数 = **`ekf_hil` 调用次数** ✓（段号 0 时自增计数）
        let n0addr = mcu_simulater::elfsym::try_app_sym("STAGE_N").map(|a| a as u64 + 4);
        let rd = |h: &mut EnvHarness, a: Option<u64>| -> u32 {
            match a {
                Some(a) => h
                    .m
                    .cpu
                    .mem_read(a, 4)
                    .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                    .unwrap_or(0),
                None => 0,
            }
        };
        let a0 = dwt(&mut h);
        let a1 = h.m.retired_count();
        let a2 = rd(&mut h, n0addr);
        h.run_for_ms(1000.0);
        let b0 = dwt(&mut h);
        let b1 = h.m.retired_count();
        let b2 = rd(&mut h, n0addr);
        let d_dwt = b0.wrapping_sub(a0) as u64;
        let d_ret = b1 - a1;
        let d_n = b2.wrapping_sub(a2) as u64;
        if d_n > 0 {
            eprintln!(
                "[cal] 每退休字节 DWT 计数 = **{:.4}**（`block_cycles` 名义 0.633）· 每次 `ekf_hil`：\
                 DWT **{:.0}** 计数 ↔ {:.0} 字节（≈{:.0} 条指令 @2B/条）",
                d_dwt as f64 / d_ret.max(1) as f64,
                d_dwt as f64 / d_n as f64,
                d_ret as f64 / d_n as f64,
                d_ret as f64 / d_n as f64 / 2.0,
            );
        } else {
            eprintln!("[cal] STAGE_N 不可读 ⇒ 跳过（n={d_n}）");
        }
    }
    // ★先把固件 console 里的 RT 统计打出来（exec_us/jit_us/超时 ✓）
    {
        let c = h.console_all();
        let mut n = 0;
        for ln in c.lines() {
            if ln.contains("exec_us") || ln.contains("overrun") || ln.contains("deadline")
               || ln.contains("wq") || ln.contains("rt") {
                eprintln!("[rt] {ln}");
                n += 1;
                if n > 40 { break; }
            }
        }
        eprintln!("[rt] (console 共 {} 字节, 命中 {n} 行)", c.len());
    }
    // ★预热期逐段仪表（发散发生在预热期内 ⇒ 必须看这里 ✓）
    for j in 0..13 {
        h.run_for_ms(450.0);
        let e: EstReadout = h.read_est();
        let eu = e.euler();
        {
            // ★L2 estimator item 内部实际耗时（cycles @168MHz ⇒ ms）
            for nm in ["IT_EXEC_EKF", "IT_EXEC_ATT", "IT_EXEC_SENS", "WQ_EXEC_L2"] {
                if let Some(a) = mcu_simulater::elfsym::try_app_sym(nm) {
                    if let Ok(b) = h.m.cpu.mem_read(a as u64, 4) {
                        let v = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                        eprintln!("[exec] {nm} = {v} cyc = {:.3} ms", v as f32 / 168_000.0);
                    }
                }
            }
        }
        {
            let a = mcu_simulater::elfsym::app_sym("ESKF_PRED") as u64;
            if let Ok(b) = h.m.cpu.mem_read(a, 8 * 4) {
                let g = |i: usize| f32::from_le_bytes([b[i*4], b[i*4+1], b[i*4+2], b[i*4+3]]);
                eprintln!("[pred] N={:.0} dtt_a={:.6} dtt_v={:.6} dv=({:.2},{:.2},{:.2})",
                          g(0), g(1), g(2), g(3), g(4), g(5));
            }
        }
        {
            let a = mcu_simulater::elfsym::app_sym("ESKF_VZ") as u64;
            if let Ok(b) = h.m.cpu.mem_read(a, 12 * 4) {
                let g = |i: usize| f32::from_le_bytes([b[i*4], b[i*4+1], b[i*4+2], b[i*4+3]]);
                eprintln!("[vz] obs={:8.3} pz={:8.3} vz={:8.3} ok={:.0}", g(0), g(1), g(2), g(3));
            }
        }
        eprintln!(
            "[warm] j={j:2} pos=({:10.3},{:10.3},{:10.3}) vel=({:8.3},{:8.3},{:8.3}) rpy=({:7.3},{:7.3},{:7.3})",
            e.pos[0], e.pos[1], e.pos[2], e.vel[0], e.vel[1], e.vel[2], eu[0], eu[1], eu[2]
        );
    }
    // ★初始化瞬间的姿态 + 比力（固件静态直读 ✓，绕开日志 ring ✓）
    {
        // ⚠`app_sym` 对缺失符号会 panic ⇒ 用 `try_app_sym`：该诊断量已随固件改名/裁剪
        //   消失时**只跳过本段**，不要连带把测例的收敛断言一起打死 ✗。
        match mcu_simulater::elfsym::try_app_sym("ESKF_INIT_Q") {
            Some(a) => {
                if let Ok(b) = h.m.cpu.mem_read(a as u64, 8 * 4) {
                    let g = |i: usize| f32::from_le_bytes([b[i*4], b[i*4+1], b[i*4+2], b[i*4+3]]);
                    eprintln!("[initq] q0=({:.4},{:.4},{:.4},{:.4}) acc0=({:.3},{:.3},{:.3})",
                              g(0), g(1), g(2), g(3), g(4), g(5), g(6));
                } else { eprintln!("[initq] READ FAIL"); }
            }
            None => eprintln!("[initq] ESKF_INIT_Q 不在现固件符号表（改名/裁剪）⇒ 跳过"),
        }
    }

    // 收敛后连续采样 500 步（~3.4s）：断言误差全程有界、健康保持 Nominal、任务不冻结
    let mut worst_vel = 0.0f32;
    let mut worst_tilt = 0.0f32;
    let mut worst_alt_err = 0.0f32;
    let mut health_ok = true;
    let mut est_ok = true;
    let mut last_seq = h.read_sensor_seq();
    let mut last_adv = 0;
    for k in 0..500 {
        h.step();
        let e: EstReadout = h.read_est();
        let eu = e.euler();
        // ★逐段仪表（Hover 真值恒 0 ⇒ 任何偏离都是纯误差 ✓，一眼看出哪一量先偏 ✓）
        if k % 25 == 0 {
            eprintln!(
                "[hov] k={k:4} pos=({:9.3},{:9.3},{:9.3}) vel=({:8.3},{:8.3},{:8.3})                  rpy=({:7.3},{:7.3},{:7.3}) ab=({:7.3},{:7.3},{:7.3})",
                e.pos[0], e.pos[1], e.pos[2],
                e.vel[0], e.vel[1], e.vel[2],
                eu[0], eu[1], eu[2],
                e.accel_bias[0], e.accel_bias[1], e.accel_bias[2]
            );
        }
        // 悬停真值：vel=0、att=0、pos[2]=0（Hover 参考 0m）
        worst_vel = worst_vel.max(e.vel.iter().map(|v| v.abs()).fold(0.0, f32::max));
        worst_tilt = worst_tilt.max(eu[0].abs()).max(eu[1].abs());
        worst_alt_err = worst_alt_err.max(e.pos[2].abs());
        if e.health != 0 {
            health_ok = false; // FDIR 应保持 Nominal（全传感器正常）
        }
        let sq = h.read_sensor_seq();
        if sq > last_seq {
            last_seq = sq;
            last_adv = 0; // 重置"未推进计数"
        } else {
            last_adv += 1;
            if last_adv > 20 {
                // 连续 >20 步（~270ms 虚拟）SENSOR_SEQ 未推进 → sensors 任务冻结
                est_ok = false;
            }
        }
        if h.got_invalid.load(std::sync::atomic::Ordering::Relaxed) {
            est_ok = false;
            break;
        }
    }
    assert!(est_ok, "任务冻结/非法指令：console tail:\n{}", h.console_all());
    assert!(health_ok, "FDIR 应保持 Nominal（全传感器正常）");
    assert!(
        worst_vel < 0.5,
        "悬停速度估计应收敛到 ~0，worst|vel|={worst_vel:.3} m/s"
    );
    assert!(
        worst_tilt < 0.12,
        "悬停姿态应接近水平，worst tilt={worst_tilt:.3} rad"
    );
    assert!(
        worst_alt_err < 1.0,
        "悬停高度估计应接近真值 0m，worst|alt_err|={worst_alt_err:.3} m"
    );
    eprintln!(
        "RESULT: hover stable worst vel={worst_vel:.3} tilt={worst_tilt:.3} alt_err={worst_alt_err:.3} health_ok={health_ok} (steps={})",
        h.steps
    );
}
