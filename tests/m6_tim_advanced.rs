//! M6 验收测试：多定时器（TIM1 高级 + TIM3 通用）并跑 + TIM1 高级特性端到端。
//!
//! 复用 firmware/tim_advanced_demo 固件（场景见其 main.c 注释）：
//! 1. 固件配置 TIM1 高级（ARR=999、CCR1=250 → PWM 模式1 25% 占空比、CC1E|CC1NE
//!    互补输出、BDTR.DTG=20 死区 + MOE、UIE 更新中断）与 TIM3 通用（UIE 更新中断），
//!    NVIC 使能 IRQ24/25/29 后启动计数；
//! 2. PWM 运行中 TIM1/TIM3 各自溢出 → 更新中断执行（G_T1_UEV/G_T3_UEV++），
//!    TIM1 OCx/OCxN 变化经 Event::TimPwm 发布（测试订阅校验：主通道 25% 占空比
//!    波形、互补通道 ch4 反相、死区防直通——两路任何时刻不同时高、刹车后 MOE
//!    门控两路强制无效电平）；两定时器各累计 ≥4 次更新后固件关闭 UIE
//!    （PWM 波形经 TimPwm 继续输出）——中断风暴停止后 run() 才能自然到达指令
//!    上限返回（与 M2/M5 固件语义一致）；
//! 3. 测试写 G_TRIGGER_BREAK=1 → 固件软件刹车 EGR.BG → BDTR.MOE 清零、SR.BIF 置位、
//!    IRQ24（TIM1_BRK）执行 → G_T1_BRK++；随后记录 G_MOE=0/G_BIF=1/G_DONE。
//!
//! 期望结果区：
//!   0x20000000 G_T3_UEV  ≥ 4（TIM3 更新中断，多定时器并跑）
//!   0x20000004 G_T1_UEV  ≥ 4（TIM1 更新中断）
//!   0x20000008 G_T1_BRK  = 1（TIM1 刹车中断执行）
//!   0x2000000C G_MOE     = 0（刹车后 BDTR.MOE 清零）
//!   0x20000010 G_BIF     = 1（刹车后 SR.BIF 置位）
//!   0x20000014 G_DONE    = 0xAAAAAAAA（主线完成）
//!   TIM1 CCER = CC1E|CC1NE（互补输出使能）、BDTR = DTG20|MOE（死区 + 主输出）

use std::path::Path;
use std::sync::{Arc, Mutex};

use mcu_simulater::events::Event;
use mcu_simulater::machine::Machine;

const G_T3_UEV: u32 = 0x2000_0000;
const G_T1_UEV: u32 = 0x2000_0004;
const G_T1_BRK: u32 = 0x2000_0008;
const G_MOE: u32 = 0x2000_000C;
const G_BIF: u32 = 0x2000_0010;
const G_DONE: u32 = 0x2000_0014;
const G_TRIGGER_BREAK: u32 = 0x2000_0018;

const TIM1_CCER: u32 = 0x4001_0020;
const TIM1_BDTR: u32 = 0x4001_0044;

const CCER_CC1E: u32 = 1 << 0;
const CCER_CC1NE: u32 = 1 << 2;
const BDTR_DTG20: u32 = 20;
const BDTR_MOE: u32 = 1 << 15;

fn load_machine() -> Machine {
    let elf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("firmware/tim_advanced_demo/tim_advanced_demo.elf");
    assert!(elf.exists(), "固件未编译：{elf:?}，请先用 arm-none-eabi-gcc 构建");

    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();
    m
}

fn read_u32(m: &mut Machine, addr: u32) -> u32 {
    let out = m.cpu.mem_read(addr as u64, 4).unwrap();
    u32::from_le_bytes(out.try_into().unwrap())
}

#[test]
fn m6_tim_advanced_end_to_end() {
    let mut m = load_machine();

    // 订阅 TIM1 主通道 OCx（ch0）与互补通道 OCxN（ch4）PWM 电平变化事件（按序记录）
    let trace = Arc::new(Mutex::new(Vec::new()));
    let tr = trace.clone();
    m.events.lock().unwrap().subscribe(Arc::new(Mutex::new(move |ev: &Event| {
        if let Event::TimPwm { port: 1, channel, level } = ev {
            if *channel == 0 || *channel == 4 {
                tr.lock().unwrap().push((*channel, *level));
            }
        }
    })));

    // 阶段一：执行至固件关闭更新中断（各累计 ≥4 次更新后停 UIE），run() 自然返回。
    m.run(200_000).unwrap();

    // 1) 多定时器并跑：TIM1（IRQ25）与 TIM3（IRQ29）更新中断都应执行 ≥4 次
    let t1_uev = read_u32(&mut m, G_T1_UEV);
    let t3_uev = read_u32(&mut m, G_T3_UEV);
    assert!(t1_uev >= 4, "TIM1 更新中断应累计 ≥4（G_T1_UEV={t1_uev}）");
    assert!(t3_uev >= 4, "TIM3 更新中断应累计 ≥4（G_T3_UEV={t3_uev}）");

    // 2) TIM1 高级配置读回：互补输出 CC1E|CC1NE + 死区 DTG=20 + MOE
    let ccer = read_u32(&mut m, TIM1_CCER);
    assert_eq!(
        ccer & (CCER_CC1E | CCER_CC1NE),
        CCER_CC1E | CCER_CC1NE,
        "TIM1 CCER 应使能 OC1 与 OC1N 互补输出（读回 0x{ccer:X}）"
    );
    let bdtr = read_u32(&mut m, TIM1_BDTR);
    assert_eq!(bdtr & 0xFF, BDTR_DTG20, "TIM1 BDTR.DTG 应为 20（死区 20×tCK）");
    assert_ne!(bdtr & BDTR_MOE, 0, "TIM1 BDTR.MOE 应置位（主输出使能）");

    // 3) PWM 波形：ARR=999/CCR1=250 → 主通道应有上升沿与下降沿（25% 高电平）
    let evs = trace.lock().unwrap();
    let ch0: Vec<bool> = evs.iter().filter(|(c, _)| *c == 0).map(|(_, l)| *l).collect();
    let ch4: Vec<bool> = evs.iter().filter(|(c, _)| *c == 4).map(|(_, l)| *l).collect();
    assert!(!ch0.is_empty(), "应发布主通道 TimPwm 电平事件");
    assert!(!ch4.is_empty(), "应发布互补通道 TimPwm 电平事件（CC1NE 使能）");
    let has_high = ch0.iter().any(|l| *l);
    let has_low = ch0.iter().any(|l| !*l);
    assert!(has_high && has_low, "PWM 波形应含高/低电平（25% 占空比）");
    let transitions = ch0.windows(2).filter(|w| w[0] != w[1]).count();
    assert!(transitions >= 2, "PWM 波形应有电平翻转（实际 {transitions} 次）");
    // 互补通道（ch4）也应含高/低电平（与主通道反相的有效区间）
    assert!(ch4.iter().any(|l| *l) && ch4.iter().any(|l| !*l), "互补通道应含高/低电平");
    // 死区防直通：按事件序回放主/互补电平，任何时刻两路不得同时为高
    let mut oc = false;
    let mut ocn = false;
    let mut both_low = false;
    for (c, l) in evs.iter() {
        match c {
            0 => oc = *l,
            4 => ocn = *l,
            _ => {}
        }
        assert!(!(oc && ocn), "死区防直通：主/互补输出不得同时为高");
        if !oc && !ocn {
            both_low = true;
        }
    }
    assert!(both_low, "死区窗口内应存在两路同为低的时刻");
    drop(evs);

    // 阶段二：软件刹车。写 G_TRIGGER_BREAK=1 → 固件 EGR.BG → MOE 清零/BIF 置位/IRQ24。
    m.cpu.mem_write(G_TRIGGER_BREAK as u64, &1u32.to_le_bytes()).unwrap();
    m.run(200_000).unwrap();

    // 4) 刹车结果：IRQ24 执行 1 次、MOE 清零、BIF 置位、主线完成
    assert_eq!(read_u32(&mut m, G_T1_BRK), 1, "TIM1 刹车中断应执行 1 次");
    assert_eq!(read_u32(&mut m, G_MOE), 0, "刹车后 BDTR.MOE 应清零");
    assert_eq!(read_u32(&mut m, G_BIF), 1, "刹车后 SR.BIF 应置位");
    assert_eq!(read_u32(&mut m, G_DONE), 0xAAAA_AAAA, "主线应完成（写 G_DONE）");

    // 5) MOE 门控：刹车后 MOE=0 → 主/互补输出均强制无效电平（事件流末尾两路同为低）
    let evs = trace.lock().unwrap();
    let mut oc = false;
    let mut ocn = false;
    for (c, l) in evs.iter() {
        match c {
            0 => oc = *l,
            4 => ocn = *l,
            _ => {}
        }
    }
    assert!(!oc && !ocn, "刹车后（MOE=0）主/互补输出应强制为无效电平");
}
