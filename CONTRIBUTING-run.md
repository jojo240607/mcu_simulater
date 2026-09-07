# mcu_simulater 运行指南 + 当前进度手账

> 目标是给"怎么跑 / 已修到哪 / 下一步"一个可续的底稿，尤其面向 RTOS(joc-base) + App 分区
> (drv-bringup-app-rust / joc-app-rust) 的验收。仓库当前**无交互 CLI**（`src/main.rs` 是 M0 骨架，
> `config.parse`/Monitor/GDB 占位）——固件上线靠 `#[test]` 集成测试驱动。

## 一、怎么跑（最快可达）

前置：`arm-none-eabi` 编译固件、`rustup target add thumbv7em`（App 侧），本工程 `vendor/` 已离线自包含。
Windows PowerShell（必须先设 libclang；跑测试务必 `--release`，debug 约 2–3 MIPS 太慢）：

```powershell
cd D:\project\mcu\oop\mcu_simulater
$env:LIBCLANG_PATH = "D:\soft\llvm\bin"

# 全量 / 单个
cargo test --offline
cargo test --release --offline --test x_jos_plain    # 系统固件跑到 READY 横幅
cargo test --release --offline --test x_drv_bringup  # 系统 + drv-bringup app 分区 → BK 各驱动行
cargo test --release --offline --test x_jos_app      # 系统 + joc-app(flyctrl) → 任务拉起（当前到首轮）
```

把任意测试的 `load_elf(&Path)`（+`load_app_partition(&app.bin)`）路径换成你自己的固件即可；
控制台输出在 `m.console.lock().output()`。

要“交互式看串口/发命令”：`src/bin/repl.rs`（见下 REPL 段）。

## 二、当前已知、已验证的进度（截至本次手账）

### 已落地（net 源码改动，全在 `src/`，未含临时探针）
- `src/machine/mod.rs`：
  - `run()` 风暴护栏（异常风暴下不致无限自旋，单调用上界后返回）。
  - 只读诊断访问器：`vec_entries()/last_switch_pc()/run_iterations()`。
  - **icount 节拍对齐**：block hook 虚拟周期由 `size字节×AVG(3)` 改为 `size字节`（删掉幻数；
    使 SysTick.LOAD=168000 差不多对应真机 ~6 万条退休指令/ms 口径）。
  - 数据访问 MEM hook 改为 MPU 使能后懒安装（规避本仓库早前 Unicorn“带 hook 新译 TB 首条
    32 位副作用丢失”缺陷对启动链的影响）。早先多处外围/SCB/ADC/USB 修复见 git diff。
- 这些构建 `--offline`，`x_drv_bringup` 绿，`m_unicorn_bn_bug` 9/9。

### 本会话核心判定（带可复核数据）
1. **① INSN_INVALID 已修**（Unicorn `translate.c` `gen_set_condexec`：IT 结束后把
   `condexec_bits` 写 0。校验哈希在 `.cargo-checksum.json`）。验收 `tests/x_jos_p2.rs`。
2. **节拍**：校准探针 `tests/x_sys_retire_calib.rs`（code-hook 逐指令真数）测每 SysTick
   退休指令 **19786 → 52639**（对齐 icount 后，逼近真机 ~60k）。
3. **SCHED_ASSERT `sched.c:280`（ready_add 双挂）在节拍对齐后消失**（drv 不再出现该行；
   起因即“1# SysTick 过密 → 任务秒睡秒醒 → sleep_head 335 万次重入 → TCB sched 字段自坏”）。
4. **未收敛（下一目标）**：节拍对了、调度也不再双挂，但 joc-app flyctrl 的周期心跳
   `hb seq=` **300s 不出现**。空闲时观测 **PendSV ≈ 109× / 每 SysTick**（x_jos_hb: step0
   pendSV≈493k / sysTick≈4.5k），高上下文切换密度把墙钟吃满，任务即便被 SysTick 唤醒也难跑满
   250 次控制循环打到 hb。→ 判定：这是第二量纲——**PendSV 在“让出/空闲/从 SysTick 退平”点的
   高密度自激/非严格一次性**，与我早先“PendSV 未切走”判据被确认为假（pend_stats 曾显示 48.5 万
   次被正常切走、无掩码）不矛盾：现在反而是“切太多/太密”。

### 推荐下一步（对新引擎/干净会话可续）
1. 先量 idle 时那 ~109 次/每 SysTick PendSV 在**切谁**：采样 PendSV 抢占点 PC/被切任务
   （diagnostic `last_switch_pc()` + `g_running`0x10006040）判断是「同一 idle 任务自我空切」
   还是「系统每 Tick 不停唤醒再让出」。
2. 若 idle 自我空切 → 给 PendSV 上 QEMU 语义：“退到线程的最低优先级、至多一次、且只在
   SysTick/更高 ISR 全部退出后才被取”的门控（machine/nvic，勿绕 SCB SYST 语义）。
3. 判据：`x_jos_hb` / `x_jos_app` 出 `hb seq=`；`x_drv_bringup` 保持绿且无 `[SCHED_ASSERT]`；
   校准探针保持每 SysTick≈60k。

## 三、配置化一键运行（推荐）

把“加载哪个固件”写进配置，一条脚本直达（`cfg-run` 读取 `run.cfg`，流式打印串口，stdin 喂命令）：

```powershell
# 1) 编辑 run.cfg（仓库根示例；键见文件内注释）
#    elf = D:\...\joc-base\build_rel\stm32f407_minimal.elf
#    app = D:\...\drv-bringup-app-rust\app.bin    # 可选(轨道B应用分区)
#    n=400000  max_steps=0  rx_port=1

# 2) 一键跑（cmd，无 ExecutionPolicy 限制）
.\run.cmd                  # 等于 cargo run release+offline+libclang 并传 run.cfg；也可 .\run.cmd my\b.cfg
# 或直接用
cargo run --release --offline --bin cfg-run -- --cfg run.cfg
```

输出会把虚拟 Console 收到的 UART 文本实时打到 stdout（例：READY、App 分区 found、BK 各驱动）；
命令行输入会按行尽力经 USART `rx_port` 注入（能否被固件读到取决于其 RX 接线）。`quit`/EOF/`Ctrl-C` 均退出。
配置文件无第三方解析依赖（`key=value`/`#;` 注释）；源码在 `src/bin/cfg-run.rs`，配置可按别的 `--cfg` 提供。
