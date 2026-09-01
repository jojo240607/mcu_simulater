# MCU 仿真器设计文档

> 目标：仿 Renode 的 MCU 仿真器。CPU 指令执行由 Unicorn Engine（C 库，基于 QEMU）承担，其余一切（内存映射、外设、中断、时序、调试）由 Rust 实现，可通过配置挂载/连接各种虚拟外设。
>
> 首期目标板卡：**STM32F407VET6**（Cortex-M4F @168MHz）。

## 1. 已收敛的决策

| 项目 | 决策 |
|---|---|
| 板卡 | STM32F407VET6（Cortex-M4F，带 FPU） |
| CPU 引擎 | Unicorn 2，Rust 绑定 `unicorn-engine = "2.1"` |
| CPU 模式 | `Arch::ARM` + `Mode::MCLASS` + `Mode::VFP4`（启用 M4F 浮点） |
| 内存模型 | RAM/Flash 由 Unicorn 直接 map；MMIO 经 mem hook 转发 Rust 外设 |
| 保真度 | 目标 T3 时序语义，但实现走"路线 B 简化" |
| 时间模型 | 块级加权（block hook 计指令数 × 平均周期）+ 外设侧忠实时序语义 |
| 中断 | Rust 实现 NVIC + 异常栈帧压栈/出栈 + `BX LR` 拦截 |
| MPU | Rust 实现 Cortex-M4 MPU（8 region）+ 访问控制 + MemManage fault |
| 外设互联 | 事件总线（EventBus）为核心，DSL `connect` 语法糖映射为订阅 |
| 配置格式 | 类 Renode 的 DSL 脚本 |
| 调试 | 首版贯穿 GDB Server（gdbstub）+ Monitor REPL |
| 许可证注意 | Unicorn 为 GPL-2.0，需在合规层面留意 |

## 2. 总体架构（分层）

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

职责边界：
- **Unicorn 只管**：执行指令、访问已 map 的内存、寄存器读写、触发 hook 回调。
- **Rust 管**：MMIO 外设、中断控制器（NVIC）、内核私密外设（SysTick/SCB）、虚拟时钟、事件互联、调试、固件加载。

## 3. 技术选型

| 组件 | 选型 | 用途 |
|---|---|---|
| CPU 内核 | `unicorn-engine = "2.1"` | Unicorn 2 官方 Rust 绑定 |
| ELF 加载 | `object` | 解析 .elf（含符号表，供 GDB/monitor 用），同时支持 .bin/.hex |
| 配置解析 | 自研 DSL 解析器 | 类 Renode 脚本 |
| 命令行 | `clap` | CLI 入口 |
| 日志 | `tracing`/`log` | 关键步骤日志（仿真过程可观测） |
| GDB | `gdbstub` | GDB 远程调试协议 |
| 并发 | 标准线程 | 仿真为单线程密集计算；GDB/REPL 用独立线程 |
| 测试 | `cargo test` + 集成测试固件 | 行为验证 |

## 4. 目标板卡资源与地址映射

| 区域 | 地址 | 大小 | 归属 |
|---|---|---|---|
| Flash（含向量表） | `0x08000000` | 512KB | Unicorn map（可执行） |
| CCM SRAM | `0x10000000` | 64KB | Unicorn map |
| SRAM1 | `0x20000000` | 112KB | Unicorn map |
| SRAM2 | `0x2001C000` | 16KB | Unicorn map |
| APB1 外设 | `0x40000000` | 64KB | Rust（UART2-5、TIM2-7、I2C、DAC…） |
| APB2 外设 | `0x40010000` | 64KB | Rust（USART1/6、TIM1/8-11、ADC、EXTI…） |
| AHB1 外设 | `0x40020000` | 64KB | Rust（GPIOA-I、RCC、FLASH 接口） |
| AHB2 / AHB3 | `0x50000000` / `0xA0000000` | — | 后续扩展 |
| SCB / NVIC / SysTick / MPU | `0xE000E000` | 4KB | Rust（Core 私密外设） |

关键外设基址（首期）：
- USART1 `0x40011000` / USART2 `0x40004400` / USART3 `0x40004800`
- GPIOA `0x40020000`（每组 GPIO 间隔 0x400）
- TIM2 `0x40000000` / TIM3 `0x40000400` / TIM4 `0x40000800`
- RCC `0x40023800` / FLASH 接口 `0x40023C00`
- SysTick `0xE000E010` / SCB `0xE000ED00` / NVIC `0xE000E100`
- MPU 寄存器 `0xE000ED90-0xE000EDB8`（TYPE/CTRL/RNR/RBAR/RASR + 别名）

## 5. 核心模块设计

### 5.1 CPU 封装（core/）

封装 Unicorn，对外隐藏 FFI：

```rust
pub struct Cpu { emu: Unicorn<()> /* 或携带事件总线回调 */ }

impl Cpu {
    pub fn new_m4f() -> Result<Self>;          // ARM + MCLASS + VFP4
    pub fn reg_read(&mut self, reg) -> Result<u64>;
    pub fn reg_write(&mut self, reg, val) -> Result<()>;
    pub fn mem_read(&mut self, addr, buf) / mem_write(...);
    pub fn mem_map(&mut self, addr, size, prot);
    pub fn add_mem_hook(&mut self, range, cb);   // MMIO 转发
    pub fn add_block_hook(&mut self, cb);        // 时序计数 + 中断检查
    pub fn add_code_hook(&mut self, cb);         // 单步/断点/精确模式
    pub fn emu_start(&mut self, begin, until, timeout, count);
    pub fn emu_stop(&mut self);
}
```

### 5.2 内存总线（bus/）

- `Bus` 持有区间表：`Vec<(Range, Arc<dyn Peripheral>)>`。
- RAM/Flash 区域直接 `mem_map` 给 Unicorn，无 hook（快）。
- 每个 MMIO 外设区域单独 `mem_map` 并挂 mem hook；CPU 访问该区时 hook 回调按地址偏移分发到外设的 `read/write`。

```rust
pub struct Bus { regions: Vec<Region> }

pub trait BusOps {
    fn read(&mut self, addr: u32, size: u32) -> Result<u32, BusError>;
    fn write(&mut self, addr: u32, size: u32, val: u32) -> Result<(), BusError>;
}
```

### 5.3 外设接口（peripheral/）

统一"插座"接口，任何外设实现即可挂载：

```rust
pub trait Peripheral: Send + Sync {
    fn name(&self) -> &str;
    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError>;
    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError>;
    fn reset(&mut self) {}
    // 中断输出：外设主动拉起的 irq 编号
    fn irq_line(&self) -> Option<u32> { None }
    // 周期推进回调：供 TIM/SysTick 等按周期语义推进
    fn tick(&mut self, cycles: u64) {}
}
```

内核配套外设（NVIC/SysTick/SCB）实现同一 trait，额外标记 `CorePeripheral`（中断投递有特殊逻辑）。

### 5.4 中断模型（重点）

Unicorn 不认 STM32 向量表，Cortex-M 的异常入栈/出栈由 Rust 完整实现：

1. 外设 `irq_line()` 置位 → Rust 版 NVIC 维护 `pending/enable/priority` 寄存器。
2. 仿真循环在 **block hook** 中检查"是否有更高优先级挂起中断" → `emu_stop` 跳出。
3. Rust 手工压栈（xPSR/PC/LR/R12/R3-R0 共 8 字到当前 SP）→ 读向量表 `*(0x08000000 + 16*4 + irq*4)` → 设置 LR=`0xFFFFFFF9`（返回 Thread 模式）→ 写 PC 跳转。
4. 在 **code hook** 中拦截 `BX LR`（LR 为 0xFFFFFFxx 特殊值）→ Rust 自动出栈恢复现场。

中断响应延迟可由时间模型配置（块级投递默认几条指令延迟）。

### 5.5 时间模型（路线 B 简化）

Unicorn 非周期精确，方案：

- 以 **block 为单位**计数：block hook 记录块内指令数，乘以配置的"平均周期/指令"累加到虚拟周期。
- **外设侧忠实时序语义**：SysTick/TIM 的 CNT 递增、比较匹配、溢出置位、中断产生，完全按"虚拟周期"语义推进（`tick(cycles)` 回调）。
- 调度器维护事件队列（定时器到期、断点、虚拟外设轮询），`emu_start` 跑一段指令后检查时钟。
- 预留 `CycleModel` trait 接口，未来如需更精确可替换为指令级周期表，而不改外设代码。

```rust
pub trait CycleModel {
    fn cycles_for_block(&self, instr_count: u32) -> u64; // 默认 instr_count * AVG_CYCLES
    fn cycles_for_insn(&self, addr: u64) -> u64;         // 精确模式预留
}
```

### 5.6 事件总线（虚拟外设互联）

核心诉求"可配置连接各种虚拟外设"。采用事件总线承载数据流：

```rust
pub enum Event {
    UartByte { port: u8, byte: u8 },
    GpioLevel { port: u8, pin: u8, level: bool },
    GpioEdge  { port: u8, pin: u8, rising: bool },
    TimerMatch { tim: u8 },
    // 可扩展：I2cFrame / SpiFrame / DmaDone ...
}

pub struct EventBus {
    subscribers: Vec<(EventFilter, Arc<dyn Fn(&Event) + Send + Sync>)>,
}

impl EventBus {
    pub fn publish(&self, ev: &Event);
    pub fn subscribe(&mut self, filter: EventFilter, cb: ...);
}
```

DSL 语法糖 `connect uart1.tx -> console.rx` 编译为"UART 发布 UartByte → console 订阅"。虚拟外设（终端、LED 面板、逻辑分析仪）以订阅者身份无侵入接入。

### 5.7 配置 DSL（config/）

类 Renode 脚本，示例：

```
machine create stm32f407vet6
cpu add cortex-m4f freq 168MHz
memory add FLASH 0x08000000 0x80000
memory add SRAM1 0x20000000 0x1C000
peripheral add rcc     @ 0x40023800 size 0x400
peripheral add gpioa   @ 0x40020000 size 0x400
peripheral add usart1  @ 0x40011000 size 0x400 irq=37
peripheral add timer2  @ 0x40000000 size 0x400 irq=28
peripheral add sysTick core 1ms
connect usart1.tx -> console.rx
load elf @ firmware/fw.elf
start
```

`Machine` 负责装配：创建 CPU → 映射内存 → 实例化外设注册到 Bus → 建立事件订阅 → 加载固件。

### 5.8 调试（monitor/ + gdbstub/）

- **GDB Server**（首版贯穿）：`gdbstub` 实现寄存器/内存读写、运行/停止、单步、断点、`vCont` 等，支持 IDE（VS Code/CLion）连接。
- **Monitor REPL**：`step`、`regs`、`read/write addr`、`break`、`show devices`、`show irq`。

### 5.9 MPU（内存保护单元）

STM32F407 的 Cortex-M4 自带 MPU（8 个 region），是核内私密外设，与 NVIC/SysTick/SCB 同属 `CorePeripheral`（`plugins/mpu.rs`）。实现分两部分：

**寄存器文件**
- 寄存器：`MPU_TYPE 0xE000ED90`、`MPU_CTRL 0xE000ED94`、`MPU_RNR 0xE000ED98`、`MPU_RBAR 0xE000ED9C`、`MPU_RASR 0xE000EDA0`、别名寄存器 `0xE000EDA4-0xE000EDB8`
- 维护 8 个 region 表项：基址、大小、TEX/C/B/S、AP 权限、XN（禁止执行）、sub-region 使能位
- 地址命中匹配：按"地址 → region（含 sub-region）"查表，未命中区域落入默认后台 region 语义

**访问控制（强制点）**
MPU 使能（`CTRL.ENABLE=1`）后，在三个入口做权限检查：

1. **MMIO 数据访问**：Bus 转发外设前查 MPU（AP 权限 + 特权级）
2. **RAM/Flash 数据访问**：**默认全强制（保真优先）**——MPU 使能期间即对该区挂 mem hook（read/write）检查；MPU 未使能时不挂 hook，保持 RAM 无 hook 的快路径
3. **取指（执行）**：XN 检查——可结合 Unicorn 的 `Prot::EXEC` 或 mem hook 实现

**特权级来源**：Handler 模式恒为特权；Thread 模式由 `CONTROL.nPRIV` 决定（Unicorn 暴露 CONTROL 寄存器）。

**违规处理**：命中违规 → 记录 `MMFAR` 故障地址 → 触发 **MemManage fault**（向量 4）→ 走 5.4 已有的异常栈帧/投递机制（MemManage 可被异常处理程序"返回重试"，需模拟该语义）。

## 6. 目录结构

```
mcu_simulater/
├── Cargo.toml
├── docs/design.md            # 本文档
├── src/
│   ├── main.rs               # CLI 入口
│   ├── core/                 # Unicorn 封装 + CPU（含 M4F/VFP4）
│   ├── bus/                  # 内存总线 + hook 转发
│   ├── peripheral/           # Peripheral trait + 注册表
│   ├── machine/              # Machine/Board 装配 + ELF 加载
│   ├── sim/                  # 仿真循环 / 调度器
│   │   ├── timing.rs         # CycleModel（路线 B）
│   │   └── scheduler.rs
│   ├── events.rs             # 事件总线
│   ├── config/               # DSL 解析
│   ├── monitor/              # REPL
│   └── gdbstub/              # GDB server
├── plugins/                  # 具体外设
│   ├── nvic.rs  sys_tick.rs  mpu.rs  gpio.rs  usart.rs  timer.rs  rcc.rs
│   └── virtual/              # 虚拟外设：console.rs  led_panel.rs ...
├── boards/                   # 板级定义（stm32f407vet6.dsl）
└── firmware/                 # 测试固件（.elf/.bin）
```

## 7. 里程碑

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| M0 内核骨架 | 工程 + unicorn-engine + M4F/VFP4 验证 + ELF 加载 | 跑通含浮点运算的裸机代码 |
| M1 总线+时序+最小 GDB | Memory Bus + hook 转发；路线 B 周期计数 + 调度器；GDB 最小集 | GDB 可读写寄存器/内存并控制运行 |
| M2 中断+MPU（重难点） | Rust NVIC + 异常栈帧 + BX LR 拦截 + SysTick + MPU 基础（寄存器/访问控制/MemManage fault）；GDB 单步/断点 | Timer 溢出进中断服务函数；越权访问触发 MemManage |
| M3 外设 T1 集 + 事件总线 | GPIO + USART1-3 + 虚拟 Console + TIM2 + RCC 存根；connect 语法 | blinky / printf / 中断 demo 跑通 |
| M4 T2 增强 | RCC 时钟树、EXTI、DMA、看门狗、优先级完整、MPU 完善（sub-region/别名/精确语义） | 可运行 FreeRTOS（依赖 MPU 隔离） |
| M5 虚拟外设生态 | LED 面板/终端/逻辑分析仪、多架构（RISC-V）、性能调优 | 任意外设互联 |

## 8. 风险与注意点

- **中断投递延迟**：块级 hook 检查，中断最迟延迟一个 block；必要时可在精确位置插入 `emu_stop`。
- **`BX LR` 特殊返回**：Unicorn 不自动处理 0xFFFFFFxx 返回地址，必须由 code hook 拦截，否则中断返回直接跑飞。
- **FPU 使能**：固件写 CPACR 使能 FPU 的指令在 Unicorn VFP 模式下通常可直接执行，需在 M0 验证。
- **MMIO hook 性能**：hook 只挂在外设区间，RAM/Flash 默认不挂 hook（MPU 使能时除外，见下条）。
- **MPU 与性能互斥（已定：默认全强制）**：MPU 使能后需对 RAM/Flash 挂数据访问 hook，会拖慢仿真——该性能代价为保真优先决策的已知接受项；"仅检查/日志"降级开关仅作未来可选扩展，不作为默认行为。
- **MemManage 重试语义**：MPU 违规触发异常后，异常处理程序可能"修复后返回重试"原访问，需正确模拟（否则易死循环）。
- **周期模型误差**：路线 B 为近似模型，Timer 级精确、单指令周期不等于真硅片；如需更高精度走 `CycleModel` 替换。
- **Windows 构建**：`unicorn-engine-sys` 需 C 工具链（MSVC）编译 Unicorn。
- **许可证**：Unicorn GPL-2.0，注意项目分发合规。
