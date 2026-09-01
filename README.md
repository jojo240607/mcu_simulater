# MCU 仿真器（mcu_simulater）

基于 **Unicorn Engine** 内核 + **Rust** 外设的 MCU 仿真器（思路类似 Renode）。M0 阶段已跑通：
Cortex-M4F CPU 指令执行 + ELF 加载 + 含浮点运算的裸机固件验收。

## 目录结构

```
mcu_simulater/
├── src/
│   ├── core/         # CPU 封装（Unicorn 薄封装：寄存器/内存/执行/hook）
│   ├── machine/      # 机器装配：内存布局、ELF 加载、复位
│   ├── bus/          # 内存总线（分发 CPU 访问到 RAM 或外设）
│   ├── peripheral/   # 外设接口 Peripheral trait
│   ├── sim/          # 仿真内核（调度器、时间模型）
│   ├── events.rs     # 事件总线（虚拟外设互联）
│   ├── config/       # 类 Renode DSL 配置解析（占位）
│   ├── monitor/      # Monitor REPL（占位）
│   ├── gdbstub/      # GDB 远程调试（占位）
│   ├── lib.rs        # 模块声明
│   └── main.rs       # 命令行入口
├── firmware/
│   └── fp_acceptance/   # M0 验收固件（含 VFP 运算的裸机代码）
├── tests/
│   └── m0_acceptance.rs # M0 验收测试
├── vendor/           # cargo vendor：全部依赖源码（离线自包含构建）
│   └── unicorn-engine-sys/  # 含完整 Unicorn C 核心（QEMU 源码）
├── .cargo/config.toml      # 源码替换：crates-io -> vendor
└── docs/design.md          # 设计方案
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

## 构建与测试

```bash
# 构建（vendor 已配置，无需网络；unicorn-engine-sys 需要 libclang 用于 bindgen）
$env:LIBCLANG_PATH = "D:\soft\llvm\bin"
cargo build --offline

# 编译验收固件（需 arm-none-eabi-gcc）
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
  -O0 -ffreestanding -nostdlib -nostartfiles "-Wl,-T,linker.ld" \
  -o fp_acceptance.elf main.c   # 在 firmware/fp_acceptance/ 下执行

# 运行验收测试
cargo test --offline --test m0_acceptance
```

## M0 里程碑状态

- [x] 工程骨架 + 核心模块结构
- [x] Cortex-M4F CPU（THUMB + MCLASS，CPU model = CORTEX_M4，含 FPU）
- [x] STM32F407 内存布局（FLASH/SRAM/CCM/SCB）
- [x] ELF 加载 + 向量表复位
- [x] M0 验收：含 VFP（vadd/vmul/vcvt）的裸机固件执行通过
- [ ] M1：内存总线接入、外设框架落地、mem hook 注册
- [ ] M2：MPU（默认全强制，保真优先）
