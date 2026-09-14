# 虚拟外设直通仿真（HIL 无 USB 方案）——调试模式总纲

> 本文件是**调试模式的权威定义**，供所有后续开发与对话参考。
> 结论先行：**本方案既不是 USB-HIL，也不是 SIL**，而是**物理模型直控虚拟外设**。

## 0. 一句话总结

**物理模型（fly-simulater）直接控制 mcu_simulater 的虚拟外设数据源；飞控代码（flyctrl，
运行在 joc-base RTOS 之上、被 mcu_simulater 仿真）直接读虚拟设备的标准驱动接口运行算法。
全程不使用 USB 传输数据。**

---

## 1. 各组件角色（务必分清）

| 组件 | 仓库 | 角色 |
|---|---|---|
| `mcu_simulater` | `/home/ubuntu/work/mcu_simulater` | **仿真 MCU + 片外虚拟外设**。虚拟外设含各种传感器（IMU/气压计/磁力计/GPS/SBUS 等）与电机（PWM 驱动虚拟 TIM） |
| `fly-simulater` | `/home/ubuntu/work/fly-simulater` | **物理建模/仿真真实物理环境**（动力学、空气、电机推力模型）。核心 crate：`fly-sim-core` |
| `joc-base` | `/home/ubuntu/work/joc-base` | **自研 RTOS**（非商用 RTOS） |
| `flyctrl` | `/home/ubuntu/work/flyctrl` | **飞控代码**（应用层，运行在 joc-base 之上，real-sensors 特性走标准驱动） |
| `groundctrl` | `/home/ubuntu/work/groundctrl` | 地面站（外部链路，不在本调试链路内） |

依赖关系：

```
flyctrl（飞控算法）
   │  运行于
   ▼
joc-base（自研 RTOS）
   │  被仿真
   ▼
mcu_simulater（MCU + 片外虚拟外设：传感器 / 电机）
   ▲
   │  物理真值直控虚拟外设数据源
fly-simulater（物理建模：真实物理环境）
```

---

## 2. 仿真方案的定义（核心）

### 2.1 不是 USB-HIL

USB-HIL 的典型做法：真实/仿真飞控与物理模型通过 USB（如 MAVLink over USB）双向传输
传感器数据和控制量。**本方案不用 USB、不用 MAVLink 帧连接飞控**。

### 2.2 不是 SIL

SIL 的典型做法：飞控算法编译成 PC 程序直接跑，不经过 MCU 仿真、不经过真实驱动。
**本方案 MCU 被完整仿真（指令级），飞控代码跑在仿真 MCU + 自研 RTOS 上，走真实驱动。**

### 2.3 本方案：虚拟外设直通（物理模型直控虚拟外设）

数据流（全程无 USB）：

```
┌──────────────────────────────────────────────────────────────┐
│ fly-simulater（物理建模，同进程测试线程）                       │
│   SimLoop::step_hil(&cmd) → 更新动力学（FRD）                  │
│   每 4ms 物理步写 FlySimState（Arc<Mutex> 共享）               │
│   读回虚拟 TIM CCR1/ARR → 算推力 → 下一次动力学                 │
├──────────────────────────────────────────────────────────────┤
│ mcu_simulater：虚拟外设直通（动态寄存器）                       │
│   FlySimSource: SensorModel ──value(field)──► FlySimState      │
│   RegFileSlave.add_dynamic(offset,len,src,fill)                 │
│     ├ 每读字节前 refresh_dynamic() → fill(value(field))        │
│     ├ i2c1: mpu6050@0x68 / bmp280@0x76 / qmc5883@0x0D          │
│     └ USART2: NmeaGps（GPS）/ USART3: Sbus（SBUS 推流）        │
├──────────────────────────────────────────────────────────────┤
│ 固件 flyctrl（app real-sensors，跑在仿真 joc-base 之上）        │
│   sensors_task 2ms 采样 → SensorStack                          │
│     ImuMpu6050("i2c0") / BaroBmp280("i2c0") / Qmc5883("i2c0") │
│     GpsUblox("uart1") / RcSbus("uart2")                       │
│   control 4ms → PidController → x4_mix → cmd.motor[0..3]      │
│   PWM 驱动写 TIM3/2/1/4 CCR1（400Hz）                          │
└──────────────────────────────────────────────────────────────┘
```

关键点：
1. **物理真值注入**：fly-simulater 每 4ms 物理步把真值（IMU/气压/GPS/RC）写进共享
   `FlySimState`（Arc\<Mutex\>）。
2. **虚拟外设读真值**：mcu_simulater 的虚拟从设备（I2C/UART）经 `FlySimSource` 即时
   读取 `FlySimState`，动态寄存器随之更新。固件读到的是虚拟从设备按物理模型产生的数据。
3. **固件走标准驱动**：飞控代码不 mock 驱动、不注入内存值，走真实 I2C/UART/PWM 寄存器
   与驱动代码（sensors_task → SensorStack → i2c0/uart 驱动）。
4. **控制量回读**：fly-simulater 读回虚拟 TIM CCR/ARR 得占空比 → 电机推力 → 更新动力学。
5. **无 USB / 无 MAVLink**：全部数据经共享内存 + 虚拟总线外设，不经过任何串行链路。

---

## 3. 物理世界 / 固件视图 的设备映射

| 固件设备名 | mcu_sim 外设 | port（挂载索引） | 虚拟从设备 | 地址 |
|---|---|---|---|---|
| `i2c0` | I2C1 | 1 | mpu6050 | 0x68 |
| `i2c0` | I2C1 | 1 | bmp280 | 0x76 |
| `i2c0` | I2C1 | 1 | qmc5883（静态磁） | 0x0D |
| `uart1` | USART2 | 2 | NmeaGps（$GNGGA 推流） | — |
| `uart2` | USART3 | 3 | Sbus（SBUS 推流） | — |
| PWM0-3 | TIM3/2/1/4 CH1 | 0x40000400 / 0x40000000 / 0x40010000 / 0x40000800 | CCR1=+0x34，ARR=+0x2C | 400Hz |

---

## 4. 闭环验证协议（`mcu_simulater/tests/x_vperiph_mcusim.rs`）

两个子测试共享 `run_closed_loop` 单步逻辑：
读回 PWM 推力 → 物理推进（起飞台保持/飞行）→ 注入真值到 FlySimState → `run(N)` 推进固件。

| 测试 | 步数 | 场景 | 判定 |
|---|---|---|---|
| `vperiph_closed_loop` | 60（0.24s） | 起飞台保持 → 升空 | max_thrust>0.05；pos 有限；roll/pitch<1° |
| `vperiph_hover_long` | 300（1.2s） | 升到 5m 悬停点保持 | 末段（后 30%）\|dz\|<0.8m、\|dx\|,\|dy\|<0.5m；roll/pitch<0.05rad |

运行前置（构建固件）：

```bash
cd /home/ubuntu/work/joc-base && cmake --build build_rel          # minimal elf
cd /home/ubuntu/work/flyctrl && python3 build_app.py --features real-sensors --out /tmp/flyctrl_clean.bin
cd /home/ubuntu/work/mcu_simulater && cargo test --release --offline --test x_vperiph_mcusim
```

预期输出末尾：
`>>> [VPERIPH-MCUSIM] 虚拟外设直通闭环验证通过 ✓` 与
`>>> [VPERIPH-MCUSIM] 长时悬停收敛验证通过 ✓`。

---

## 5. 关键注入细节

### 5.1 boot 前注入初始真值（必要前置）

固件 boot 阶段（12×1M 周期）sensors/control 任务已开始采样，若 FlySimState 保持默认
（`baro_pa=0` → 气压高 44330m、GPS 无效），EKF 高度会被污染到 -6454m，解锁瞬间
hold_alt 锁定垃圾值 → 持续下沉（历史根因）。因此 boot 前必须注入：

```rust
st.imu_acc = [0.0, 0.0, -9.81];      // 静止水平 FRD 悬停
st.imu_gyr = [0.0, 0.0, 0.0];
st.baro_pa = 101_325.0f32;           // h=0 家庭点气压（起飞台 d=-5）
st.gps_lat = LAT0;                    // 31.2304
st.gps_lon = LON0;                    // 121.4737
st.gps_alt = ALT0 + 5.0;             // 起飞台(d=-5) alt=9 → ref_alt=9
st.gps_fix = 3.0;
st.rc_ch = [1500.0; 16];             // RC 中性，boot 阶段未解锁
```

### 5.2 每物理步注入

```rust
st.imu_acc = [ax, ay, az];   // 机体系比力 m/s²（FRD 悬停 = (0,0,-9.81)）
st.imu_gyr = [gx, gy, gz];   // rad/s
st.baro_pa = 101_325.0 * (-h / 8434.5).exp();  // 标准大气，h 向上正
st.gps_lat = LAT0 + n / 111_320.0;             // NED → 度
st.gps_lon = LON0 + e / (111_320.0 * LAT0.to_radians().cos());
st.gps_alt = ALT0 - d;                         // d 向下正
st.gps_fix = 3.0;
st.rc_ch[4] = 2000.0;        // 解锁（SBUS raw 1811 > 1700 armed）
st.rc_ch[3] = 1500.0;        // 油门中位（SBUS raw 992 → 0.5）
```

### 5.3 解锁方式

- **RC 解锁（闭环测试实际路径）**：每物理步注入 `rc_ch[4]=2000`（armed）、
  `rc_ch[3]=1500`（油门中位）到 FlySimState → 虚拟 SBUS 从设备按 20Hz 推流
  （SBUS raw 1811 > 1700 armed）→ 固件 RcSbus 经 uart2 标准驱动解帧解锁。
  这条路径**完整走真实 UART/SBUS 驱动链路**，是本方案"固件走标准驱动"的
  直接体现（对应测试 `run_closed_loop` 的注入）。
- 地面站 ARM（旁路，调试用）：直接写 `G_CMD_ARMED`（`0x2000_b669`，
  AtomicBool）= 1，不经 SBUS 链路。

### 5.4 PWM 推力读回

```rust
duty = ccr / arr;            // 0..1 占空比
us = duty * 2500.0;          // 400Hz 周期 2500us
m = ((us - 1000.0) / 1000.0).clamp(0.0, 1.0);
```

---

## 6. 已解决阻塞及根因（2025-06 阻塞 → 2026-09 全部解决）

> 注：时间跨度经 git 历史核实（最新提交 2026-09-13 "2025-06 阻塞全部解决并
> 记录闭环验证协议"），非笔误——阻塞自 2025-06 持续到 2026-09 才闭环。

| # | 阻塞 | 根因 | 解决方案 | 验证 |
|---|---|---|---|---|
| 1 | run 偶发 `UC_ERR_INSN_INVALID` / emu_start 卡死 | Unicorn `translate.c` 在 IT 指令结束后未清零 `condexec_bits` → 后续指令被错误解释为条件执行 | 打补丁：`gen_set_condexec` 在 IT 块结束时写 0（校验哈希在 `.cargo-checksum.json`） | `x_jos_p2`、`x_vperiph_mcusim` 长跑稳定 |
| 2 | 固件 I2C 读间歇返回 0 → EKF 垂向发散 vz=-130 → PWM 中位 | 悬停比力符号约定不一致：I2C/SPI 注入 +9.81、hil.rs tilt alignment 期望 -9.81 → EKF 失配 | 统一 FRD 约定：悬停比力 `(0,0,-9.81)`（data_source/mpu6050/bmi088 同改）；clamp 恢复标准 `clamp(v,0,1)` | 闭环 300 步稳定收敛 |
| 3 | SBUS UART 推流：CR3 无 DMAR → IDLE ring 空 → RcSbus 恒 n=0 | 固件 uart2 期望 DMA+IDLE 收帧；模拟器 CR3 无 DMAR，字节只进 rx_fifo | 模拟器按真机语义走 **CPU 轮询读 DR 路径**：`read(SR)` 返回含 IDLE 状态、`read(DR)` 弹 rx_fifo 并清 IDLE（`src/peripheral/usart.rs:292-326`）；SBUS 驱动改为标准 8E2 100kbps 反向电平 + `UART_IOCTL_SET_INVERTED` | `RcSbus::read()` 解出 ch、armed=ch4>1700、throttle=ch3 |
| 4 | GPS 链路（gps_w=false） | 同 #3 的 UART 推流问题 | 同 #3 解决；`NmeaGps` 经同一 rx_fifo 路径推流 | 闭环内 gps_fix=3 生效 |

另有两项**结构性根因修复**（非阻塞，属必要修正）：
- **Non-HIL setpoint_valid 恒 false** → 改 true，使 `hil_pos_inited` gate 可置位、执行器不再恒 0。
- **HIL IMU 单次消费**：`f.imu = None` 读后即清，防止同一 32ms 帧被 8 次积分。

---

## 7. 诊断共享区地址（随固件构建变化，重建后以 nm 复核）

| 地址 | 符号 | 含义 |
|---|---|---|
| `0x2000_b61c` | `DBG_MOTOR[4]` | 4 路 clamp 后电机指令（control.rs:240 每拍刷新） |
| `0x2000_b62c` | `HIL_DIAG_GATES` | 7-bit 位域：armed(0)/rc_fresh(1)/health(2)/est_finite(3)/sp_finite(4)/att_inited(5)/pos_inited(6) |
| `0x2000_b630` | `DBG_PID[12]` | 垂向 PID 环诊断（每组 4 f32） |
| `0x2000_b660` | `DBG_PRE` | 垂向环误差 |
| `0x2000_b664` | `DBG_THR` | 垂向环输出油门 |
| `0x2000_b669` | `G_CMD_ARMED` | 地面站 ARM 标志（测试直接置 1 模拟解锁） |

复核命令：
`arm-none-eabi-nm app.elf | grep -E 'HIL_DIAG_GATES|DBG_MOTOR|DBG_PID|DBG_PRE|DBG_THR|G_CMD_ARMED'`

---

## 8. 同步机制：两套并存，勿混淆

### 8.1 vperiph 模式：锁步（闭环测试实际机制）

`tests/x_vperiph_mcusim.rs` 的 `run_closed_loop` 是**锁步**驱动，无信号量：

```
读回 PWM 推力 → SimLoop::step_hil(&cmd)（物理 4ms）→ 写 FlySimState
→ machine.run(300_000)（固件 sensors 2ms 采样 + control 4ms + PWM 写）→ 下一拍
```

- `run(N)` 的 `N` 是退休指令预算；control 任务在预算内跑几拍由退休量决定，
  PWM 读回取**最后一拍**输出。因此"4ms 物理步与 control 拍对齐"是**近似**成立：
  实际是 control 节奏略快于物理步（对稳定性有利），依赖 `run(N)` 与
  `VIRTUAL_INSNS_PER_SEC` 的比例关系，是隐式调参而非架构保证。
- **一致性不变量（重要）**：FlySimState 只在物理步间隙（两次 `run()` 之间）被
  写入，`run()` 期间必须冻结。多字节读事务（如 14 字节 IMU burst）的字节一致性
  由该锁步保证——**禁止并发线程在 run() 期间写 FlySimState**，否则会发生
  跨物理步撕裂（accel.x 来自步 k、accel.y 来自步 k+1）。

### 8.2 HIL 模式：HIL_EVT 事件同步（另一条路径）

`flyctrl/core/src/hil.rs`（cfg feature="hil"）的 `Semaphore<1>` 信号量同步是
**HIL 模式**机制：PC 每 ~32ms 注入一帧 `HIL_SENSOR`，`HilContext::step_hil()`
为 SIL/HIL 共用步进（注入 IMU 真值 → 推进 EKF → 取期望轨迹点 → 更新执行器）。
**vperiph 模式不使用 HIL_EVT**。

### 8.3 虚拟外设推流时钟（两模式共用）

虚拟外设/推流时钟以**退休指令数**为基准
（`dt = Δretired / VIRTUAL_INSNS_PER_SEC`，权威常量在
`mcu_simulater/src/sim/timing.rs`），与中断频率解耦——中断风暴下推流速率恒定
（SBUS/GPS 均 20Hz）。注意该时钟与 CPU 侧虚拟时钟（SysTick/TIM，按「访客字节
= 虚拟周期」）**独立校准**，比值 ~0.65-0.76，属已知可接受偏差（推流只需帧在
固件读取窗口内完整到达）。

---

## 9. 保真度分层（传感器缺陷归属）

当前 vperiph 链路是**理想同步零延迟传感器**：FlySimState 存物理**真值**
（无噪声/无偏置/无量化/无延迟），虚拟外设原样填寄存器。这对验证控制律
正确性足够（确定性、可复现），但不足以验证 EKF/控制律对传感器缺陷的鲁棒性。

职责划分（后续扩展方向）：

| 层 | 缺陷类型 | 归属 | 现状 |
|---|---|---|---|
| 模拟域 | 白噪声、零偏、随机游走、振动耦合、GPS 延迟/丢星 | **fly-sim 侧** `SensorConfig`（已有 `realistic()` 预设），经 `last_imu()` 注入 | 默认 `SensorConfig::default()` = 全零噪声 |
| 数字域 | 位深/量程量化（如 MPU6050 ±8g 16bit）、ODR 降频、传输延迟 | **mcu_sim 虚拟外设层**（FlySimSource 装饰器/设备配置） | 未实现，f32 真值直填寄存器 |

注意：fly-sim 的 `SensorConfig::realistic()` 已存在但 vperiph 测试未启用
（`run_closed_loop` 用 `SensorConfig::default()`）。开启后 EKF 将沿**真实
驱动链路**消费带噪数据，这正是 SIL 无法覆盖的部分。数字域量化/ODR 属
器件属性，应留在 mcu_sim 外设层，避免污染物理模型。

## 10. FRD 坐标约定（易错点速查）

| 量 | 约定 | 悬停值 |
|---|---|---|
| 机体系比力（accel） | FRD（前右下） | `(0,0,-9.81)` m/s² |
| 世界位置 | NED（n,e,d 向下正） | 起飞台 d=-5 |
| baro 高度 | 固件内部**向上正**，step_hil 内部取反为 D 向下 | 家庭点 h=0 |
| GPS alt | 海拔（m） | 起飞台 alt=9 → ref_alt=9 |

关键对齐：GPS 原点（首次定位锁定）与气压家庭点（起飞台 d=-5）对齐，避免 EKF 高度源冲突
（历史根因：GPS d=0 vs baro d=-5 折中 → hold_alt 锁错 → 机体持续下沉）。

---

## 11. 关键术语对照（防误解）

| 术语 | 本方案中的含义 |
|---|---|
| USB-HIL | ❌ 不用。典型为 MAVLink over USB 双向传数据 |
| SIL | ❌ 不是。飞控不编译成 PC 程序直跑 |
| 虚拟外设直通 | ✅ 物理模型直控虚拟外设数据源，固件走标准驱动读虚拟设备 |
| FlySimState | 共享状态（Arc\<Mutex\>），物理真值注入点 |
| FlySimSource | mcu_sim 侧传感器模型，从 FlySimState 即时取值填动态寄存器 |
| joc-base | 自研 RTOS，被 mcu_simulater 仿真 |
| flyctrl | 飞控代码，跑在 joc-base 之上，real-sensors 走标准驱动 |
