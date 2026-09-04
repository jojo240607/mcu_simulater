# MCU 仿真器（mcu_simulater）

> 仿 Renode 的 MCU 仿真器：CPU 指令执行由 **Unicorn Engine**（C 库，基于 QEMU）承担，
> 其余一切——内存映射、外设、中断、时序、调试——由 **Rust** 实现，通过事件总线可
> 挂载/互联各种虚拟外设。目标板卡：**STM32F407VET6**（Cortex-M4F @168MHz）。
> 设计方案见 [docs/design.md](docs/design.md)。

## 当前状态（M0→M17 已全部落地）

- **Cortex-M4F** 指令执行（THUMB + MCLASS + VFP4，Unicorn 2 内核）
- **STM32F407 内存布局** + ELF 加载 + 向量表复位
- **内存总线**：RAM/Flash 直接 map 给 Unicorn，MMIO 经 mem hook 转发 Rust 外设
- **MPU**：8 region 访问控制（取指 XN / 数据读写三入口）+ MemManage fault
- **NVIC**：挂起抢占 + 异常入栈/出栈 + `BX LR` 异常返回拦截
- **时序**：块级加权虚拟时钟 + 外设 tick（TIM/DMA/DAC/RTC/IWDG/WWDG）
- **事件总线**：虚拟外设互联（UART/I2C/SPI/ADC/DAC/TIM/GPIO/CAN/USB/DCMI/SDIO）
- **性能**：纯计算负载 **105 MIPS**（min-of-3，见 [性能基准](#性能基准)）

## 目录结构

```
mcu_simulater/
├── src/
│   ├── core/         # CPU 封装（Unicorn 薄封装：寄存器/内存/执行/hook）
│   ├── bus/          # 内存总线（MMIO 区间注册与读写分发）
│   ├── machine/      # Machine 装配：内存布局、外设挂载、ELF 加载、复位、block hook
│   ├── peripheral/   # 外设框架（Peripheral trait）+ 25 个具体外设
│   ├── sim/          # 仿真内核：虚拟时钟 / 事件调度器 / 全局状态位域(status.rs)
│   ├── events.rs     # 事件总线（虚拟外设互联）
│   ├── config/       # 类 Renode DSL 配置解析（占位）
│   ├── monitor/      # Monitor REPL（占位）
│   ├── gdbstub/      # GDB 远程调试（占位）
│   ├── lib.rs        # 模块声明
│   └── main.rs       # 命令行入口
├── firmware/         # 30 个验收固件（C 裸机，覆盖各外设 demo）
├── tests/            # 集成测试（m0-m17 里程碑 + bench_mips/bench_probe/bench_tb）
├── examples/         # 示例
├── docs/design.md    # 设计方案
├── vendor/           # cargo vendor：全部依赖源码（离线自包含构建）
│   └── unicorn-engine-sys/  # 含完整 Unicorn C 核心（QEMU 源码）
└── .cargo/config.toml      # 源码替换：crates-io -> vendor
```

## 已实现外设（STM32F407VET6）

| 类别 | 外设 | 基址 | 备注 |
|---|---|---|---|
| 核心 | NVIC / SCB / MPU / SysTick | `0xE000E000` | 中断投递、MemManage、复位 |
| 系统 | RCC | `0x40023800` | 复位标志、AHB1ENR 时钟位镜像 |
| GPIO | GPIOA-I（9 端口） | `0x40020000` + 0x400×port | MODER/ODR/IDR/BSRR + 电平事件 |
| 串口 | USART1-6 / UART4-5 | `0x40011000` 等 | TX/RX + DMA + 中断，IRQ37-39/52/53/71 |
| I2C | I2C1-3 | `0x40005400` 等 | EV 中断 + DMA，IRQ31/33/72 |
| SPI | SPI1-3 | `0x40013000` 等 | 中断 + DMA，IRQ35/36/51 |
| 定时器 | TIM1-14 | `0x40000000` 起 | 高级/通用/基本，更新中断 + 更新事件 DMA |
| ADC | ADC1-3 | `0x40012000` 起 | DMA2 采样，IRQ18 |
| DAC | DAC1 | `0x40007400` | 2 通道 12 位，软件/定时器触发 + DMA |
| DMA | DMA1 / DMA2 | `0x40026000` / `0x40026400` | 内存搬运 + 外设请求路由 + TC 中断 |
| 看门狗 | IWDG / WWDG | `0x40003000` / `0x40002C00` | 超时/违规 → 系统复位 |
| 电源 | PWR | `0x40007000` | 低功耗位 + 待机唤醒复位链路 |
| RTC | RTC + BKP | `0x40002800` | 日历 + 闹钟/唤醒中断（IRQ41/3）+ 备份域写保护 |
| 摄像头 | DCMI | `0x50050000` | 帧注入 + DMA2 + IRQ78（AHB2 单独映射） |
| 存储器 | FSMC | `0xA0000000` | Bank1-4 片选窗口 + 寄存器块 |
| SD | SDIO | `0x40012C00` | 命令/响应 + FIFO + DMA2 + IRQ49 |
| CAN | CAN1 / CAN2 | `0x40006400` / `0x40006800` | 3 邮箱 + 2 RX FIFO + 滤波 + 总线互联（IRQ19/20/22、63/64/66） |
| USB | USB OTG FS | `0x50000000` | 设备模式 + 虚拟主机注入 + IRQ67（AHB1 单独映射） |
| 其它 | CRC / RNG / EXTI+SYSCFG | `0x40023000` / `0x50060800` / `0x40013C00` | CRC-32、真随机数（IRQ80）、外部中断路由 |
| 虚拟 | Console / Terminal | — | 订阅 UART TX 显示 / 双向接线终端 |

## 架构分层

```
┌─────────────────────────────────────────────────────────┐
│ 前端层：CLI / Monitor(REPL) / GDB Server / 日志          │
├─────────────────────────────────────────────────────────┤
│ 配置层：Machine 描述（Board 定义 / 外设挂载 / connect）  │
├─────────────────────────────────────────────────────────┤
│ 仿真内核层：仿真循环 / 时间模型(周期) / 事件调度器        │
├─────────────────────────────────────────────────────────┤
│ 设备层：外设框架(Peripheral trait + 注册表) + 具体外设    │
├─────────────────────────────────────────────────────────┤
│ 核心层：Unicorn Engine（CPU 执行）+ Rust Memory Bus       │
└─────────────────────────────────────────────────────────┘
```

职责边界：**Unicorn 只管**执行指令、访问已 map 内存、寄存器读写、触发 hook 回调；
**Rust 管** MMIO 外设、NVIC、内核私密外设（SysTick/SCB/MPU）、虚拟时钟、事件互联、
调试、固件加载。

## 里程碑进度

| 里程碑 | 内容 | 状态 |
|---|---|---|
| M0 | 工程骨架 + Cortex-M4F + ELF 加载 + VFP 验收 | ✅ |
| M1 | 内存总线 + 外设框架 + mem hook 转发 | ✅ |
| M2 | MPU 访问控制 + 中断投递 + 异常栈帧 | ✅ |
| M3 | T1 外设集（GPIO/USART/TIM/RCC）+ 事件总线 | ✅ |
| M4 | DMA1/2 + EXTI + RCC + IWDG/WWDG + MPU 深化 | ✅ |
| M5 | 串口/UART、I2C、SPI（含 DMA/IRQ 模式）+ 虚拟终端 | ✅ |
| M6 | TIM9-14 + 高级定时器 | ✅ |
| M7 | DAC（含定时器触发 + DMA） | ✅ |
| M8 | CRC 计算单元 | ✅ |
| M9 | RNG 真随机数发生器 | ✅ |
| M10 | PWR 电源控制 + 待机唤醒复位 | ✅ |
| M11 | RTC + 备份寄存器 | ✅ |
| M12 | DCMI 数字摄像头接口 | ✅ |
| M13 | FSMC 外部存储器控制器 | ✅ |
| M14 | SDIO | ✅ |
| M15 | CAN1/2 控制器局域网（总线互联） | ✅ |
| M16 | USB OTG FS 设备控制器（虚拟主机注入） | ✅ |
| M17 | GPIO 寄存器级 pinmux（AFRL/AFRH + MODER 复用模式） | ✅ |

## 性能基准

纯计算负载（`fp_acceptance` 固件热循环）下完整机器 MIPS 演进：

| 阶段 | MIPS（min-of-3） |
|---|---|
| block hook 完整逻辑（早期） | ~37 |
| 外设激活检测移至 MMIO 写回调 | 85.5 |
| 单状态字合并 + Cell 时钟 + 闭包瘦身 | **105.2** |

关键优化（详见 [tests/bench_probe.rs](tests/bench_probe.rs) 探针归因）：

- **单状态字**：MPU/外设激活/中断挂起/看门狗 4 个低频标志合并为一个 `AtomicU8`
  位域（[src/sim/status.rs](src/sim/status.rs)），block hook 热路径 1 次原子 load
  替代 5 次独立判读；
- **Cell 时钟**：虚拟时钟 `AtomicU64 → Cell<u64>`（模拟器单线程独占）；
- **外设激活检测**从 block hook 移至外设区 MMIO 写回调，免每块全扫 ~20 个 active 标记；
- **闭包瘦身**：block hook 冷路径（MPU/NVIC/外设列表）捆为单 `Arc<BlockHookCold>`。

> 注：测量方差较大（全新进程单次 ~85.7 MIPS vs 同进程 min-of-3 105.2），由 CPU 睿频/
> 热状态主导，min-of-3 口径全程一致。

## 验收固件与测试

- `firmware/` 下 30 个裸机 demo（`*_demo/`），覆盖各外设端到端路径，经
  `arm-none-eabi-gcc` 交叉编译为 ELF；
- `tests/` 下按里程碑编号的集成测试（`m0_acceptance.rs` ~ `m17_gpio_fghi.rs`），
  加载固件 → 运行 → 断言寄存器/事件/中断结果；
- 性能探针：`bench_mips.rs`（纯计算 MIPS）、`bench_probe.rs`（分级开销归因）、
  `bench_tb.rs`（TB 尺寸 bisect）。

## 构建与测试

```bash
# 构建（vendor 已配置，无需网络；unicorn-engine-sys 需要 libclang 用于 bindgen）
$env:LIBCLANG_PATH = "D:\soft\llvm\bin"
cargo build --offline

# 编译验收固件（需 arm-none-eabi-gcc），以 fp_acceptance 为例：
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
  -O0 -ffreestanding -nostdlib -nostartfiles "-Wl,-T,linker.ld" \
  -o fp_acceptance.elf main.c   # 在 firmware/fp_acceptance/ 下执行

# 运行全部测试 / 单个里程碑测试
cargo test --offline
cargo test --offline --test m0_acceptance

# 性能基准
cargo test --release --test bench_mips -- --nocapture
```

## 依赖与源码关系

工程通过 Cargo 引入 Unicorn Engine，分为三层。构建时 `vendor/` 内的本地副本
（通过 `.cargo/config.toml` 的 `[source]` 替换）替代 crates.io，实现离线自包含构建。

| 层 | 工程内路径（vendor） | 作用 |
|---|---|---|
| Rust 安全 API 层 | `vendor/unicorn-engine/` | `Unicorn::new` / `emu_start` / hook 等安全封装，即 `src/core` 所调用的接口 |
| FFI 绑定层 | `vendor/unicorn-engine-sys/bindings/rust/sys/src/` | bindgen 生成的 C 接口声明（`bindings.rs`） |
| **Unicorn C 核心（QEMU 内核）** | `vendor/unicorn-engine-sys/qemu/` | 真正执行指令翻译的引擎，构建时经 CMake 编译成静态库 |

### 关键 C 源码路径（Cortex-M / VFP）

- `vendor/unicorn-engine-sys/qemu/target/arm/translate.c` — ARM 指令翻译主逻辑
- `vendor/unicorn-engine-sys/qemu/target/arm/translate-vfp.inc.c` — VFP 指令翻译
- `vendor/unicorn-engine-sys/qemu/target/arm/decode-vfp.inc.c` — VFP 指令解码
- `vendor/unicorn-engine-sys/qemu/target/arm/vfp_helper.c` — VFP 运算辅助
- `vendor/unicorn-engine-sys/qemu/target/arm/m_helper.c` — M-profile 特殊行为
- `vendor/unicorn-engine-sys/qemu/target/arm/cpu.c` — CPU 模型定义（含 Cortex-M4）
- `vendor/unicorn-engine-sys/include/unicorn/arm.h` — ARM CPU model 枚举（如 `CORTEX_M4`）

> 我们的封装层 `src/core/mod.rs` 只做薄封装：`Cpu` 持有 `Unicorn<'static, ()>`，
> 指令执行全部委托给 Unicorn，Rust 侧负责外设/总线/时序等上层逻辑。
> 注意：Unicorn 为 GPL-2.0，需在合规层面留意。
