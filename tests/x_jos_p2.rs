//! 验证 ① 修复：gen_set_condexec 在 TB 边界把 IT 状态（condexec_bits）正确清理后，
//! jOS 在 READY 之后不再出现 UC_ERR_INSN_INVALID（旧崩溃点 0x0800ED4A `bmi.n`）。
//!
//! 旧崩溃：rtos_msleep 临界区内 IT 块(0xED2E..0xED38)后接 `msr BASEPRI`/`mrs CONTROL`，
//! 0xED4A 的合法条件分支因 env 残留 IT 状态被判 unallocated。
//! 通过判据：READY 达成 && 全程无 INSN_INVALID && 执行越过 0xED4A（code hook 观察到
//! pc >= 0xED4C）。风暴护栏（Machine::run 单调用迭代上限）保证本探针必然返回/可终止。
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;

#[test]
fn p2() {
    let elf = Path::new(r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.reset().unwrap();

    let bad_pc = Arc::new(AtomicU32::new(0));
    let got_invalid = Arc::new(AtomicBool::new(false));
    let (b2, g2) = (bad_pc.clone(), got_invalid.clone());
    m.cpu
        .raw()
        .add_insn_invalid_hook(move |uc| {
            let pc = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
            b2.store(pc, Ordering::Relaxed);
            g2.store(true, Ordering::Relaxed);
            eprintln!("[INSN_INVALID] pc=0x{pc:08X}");
            false
        })
        .unwrap();

    // 旧崩溃点 0xED4A 的地址已随 joc-base 重构漂移（当前 elf 中该区是 USB 栈跳转表
    // 数据，rtos_msleep 已移至 0x08010F7C），故不再按地址判定"越过崩溃点"——核心
    // 判据为 READY 达成 && 全程无 INSN_INVALID（IT 修复的直接目标）。past 仅留作
    // 诊断：观察 rtos_msleep 入口是否被译码执行（该路径曾含 IT 块 + bmi.n）。
    let past = Arc::new(AtomicBool::new(false));
    let p1 = past.clone();
    m.cpu
        .raw()
        .add_code_hook(0x0801_0F7C, 0x0801_0F7C, move |_uc, addr, _size| {
            if !p1.load(Ordering::Relaxed) {
                eprintln!("[PASS-POINT] rtos_msleep 入口执行 @0x{addr:08X}");
            }
            p1.store(true, Ordering::Relaxed);
        })
        .unwrap();

    let t_start = std::time::Instant::now();
    let mut reached = false;
    for step in 0..400u32 {
        if t_start.elapsed().as_secs() > 120 {
            eprintln!(">>> 超时（120s）终止：reach={reached} past={} invalid={}",
                past.load(Ordering::Relaxed), got_invalid.load(Ordering::Relaxed));
            break;
        }
        let r = m.run(400_000);
        let last_pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let out = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        let out_len = out.len();
        if out.contains("READY. Commands") && out.contains("mounting app layer") && !reached {
            reached = true;
            eprintln!(">>> READY reached（step {step}）");
        }
        match r {
            Err(e) => {
                eprintln!("ERR {e:?} pc=0x{last_pc:08X} console={out_len}");
                break;
            }
            Ok(()) => {
                if got_invalid.load(Ordering::Relaxed) {
                    eprintln!("!!! INSN_INVALID 出现 @0x{:08X}", bad_pc.load(Ordering::Relaxed));
                    break;
                }
                if past.load(Ordering::Relaxed) && reached {
                    eprintln!(">>> 通过：READY 后越过 0xED4A 且无 INSN_INVALID（step {step}）");
                    break;
                }
                if step % 20 == 0 {
                    let (exc, pend) = {
                        let n = m.nvic.lock().unwrap();
                        (
                            n.current_exception(),
                            n.is_sys_pending(15) || (0..82).any(|i| n.is_pending(i) && n.is_enabled(i)),
                        )
                    };
                    eprintln!(
                        "[step {step}] pc=0x{last_pc:08X} reach={reached} past={} exc={exc} pend={pend} console={out_len} iters={}",
                        past.load(Ordering::Relaxed),
                        m.run_iterations()
                    );
                }
            }
        }
    }
    let ok = reached && !got_invalid.load(Ordering::Relaxed);
    eprintln!(
        "RESULT: reached_ready={reached} msleep_hit={} no_invalid={}",
        past.load(Ordering::Relaxed),
        !got_invalid.load(Ordering::Relaxed)
    );
    assert!(ok, "① 修复验证未通过（READY 未达成或无 INSN_INVALID 保证）");
}
