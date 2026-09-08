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
4. **✅ 已收敛（本次修复）**：周期心跳 `hb seq=` / `alive seq=` 已打通（x_jos_app /
   x_jos_hb 均绿，demo-app 心跳每虚拟秒一条）。根因与修复（两部分）：
   - **RTOS 侧（joc-base sched.c rtos_yield）**：idle 任务是 `for(;;) rtos_yield();`
     忙等；当所有任务睡眠时它是唯一就绪任务，每轮 yield 空切一次 PendSV
     （rtos_pendsv_switch 的 `if (!nxt) nxt = cur` 切回自身），实测 **~124 次 PendSV/
     每 SysTick**。修复：yield 时若就绪队列除自己外无更高优先级/同伴候选，跳过置
     PENDSVSET（空切防护）。真机同样受益（不再白烧 CPU）。
   - **模拟器侧（machine run()）**：旧实现 `let remaining = count` 恒不递减，每次
     emu_start 都拿全额预算，只能靠风暴护栏（1M 段）兜底——PendSV 风暴消失后每段
     很长（到 SysTick 才被抢，~25-52K 指令/段），护栏 1M 段 × 每段 ~40ms = 永不返回。
     修复：block hook 累计退休字节，run() 每轮按实际退休量递减 count，预算耗尽即返回。
   - 效果：x_jos_plain 47.8s → 7.6s；x_jos_app 48s → 14.5s（且 hb 断言通过）；
     PendSV/SysTick ≈ 124 → ≈ 1/2。

### 推荐下一步（对新引擎/干净会话可续）
1. ~~先量 idle 时那 ~109 次/每 SysTick PendSV 在**切谁**~~（已采样确认：PendSV 抢占点
   PC=0x080038F6=console_run 的 msleep(1) 循环尾；空闲期是 idle 任务 `for(;;) rtos_yield()`
   自我空切，见 §二.4 修复记录）。
2. ~~若 idle 自我空切 → 给 PendSV 上 QEMU 语义门控~~（未走该路线：直接在 RTOS 侧
   rtos_yield 做空切防护更干净，见 §二.4）。
3. 判据：`x_jos_hb` / `x_jos_app` 出 `hb seq=` / `alive seq=`（已通过）；`x_drv_bringup`
   保持绿且无 `[SCHED_ASSERT]`；校准探针保持每 SysTick≈60k。

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
