# 环境仿真调试飞控模拟——闭环验证说明

> ## ⚠️ 口径更正（2026-09-21）
>
> 1. **IMU 已改为 BMI088（SPI3 双片选）**，不再是 mpu6050(I2C)（见 `e541b47`）。
> 2. **仓库根路径为 `~/work/fc-umbrella/<repo>`**（本文写 `~/work/<repo>` 是旧布局）。
> 3. **构建用 `./scripts/build.sh real-sensors`**，不再是 `python3 build_app.py ...`。
> 4. **场景时间 = 固件时间（1:1）**，控制拍 249.7Hz（旧“慢 N 倍”说法已废）。

> 目标：记录 fly_simulater（PC 物理）↔ mcu_simulater（MCU + 固件）HIL 闭环验证的
> 完整架构、验证协议、已解决问题与诊断手段，供后续复现与修改参考。

## 1. 架构分层

```
┌──────────────────────────────────────────────────────────────┐
│ fly_simulater（同进程测试线程）                                 │
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
│ 固件 flyctrl（app real-sensors）                               │
│   sensors_task 2ms 采样 → SensorStack                          │
│     ImuMpu6050("i2c0") / BaroBmp280("i2c0") / Qmc5883("i2c0") │
│     GpsUblox("uart1") / RcSbus("uart2")                       │
│   control 4ms → PidController → x4_mix → cmd.motor[0..3]      │
│   PWM 驱动写 TIM3/2/1/4 CCR1（400Hz）                          │
└──────────────────────────────────────────────────────────────┘
```

特点：全程无 USB / 无 MAVLink；固件走标准驱动（不 mock 驱动、不注入内存值），
读到的是虚拟从设备按物理模型产生的数据。

## 2. 总线 / 设备映射

| 固件设备名 | mcu_sim 外设 | port（挂载索引） | 虚拟从设备 | 地址 |
|---|---|---|---|---|
| `i2c0` | I2C1 | 1 | mpu6050 | 0x68 |
| `i2c0` | I2C1 | 1 | bmp280 | 0x76 |
| `i2c0` | I2C1 | 1 | qmc5883（静态磁） | 0x0D |
| `uart1` | USART2 | 2 | NmeaGps（$GNGGA 推流） | — |
| `uart2` | USART3 | 3 | Sbus（SBUS 推流） | — |
| PWM0-3 | TIM3/2/1/4 CH1 | 0x40000400 / 0x40000000 / 0x40010000 / 0x40000800 | CCR1=+0x34，ARR=+0x2C | 400Hz |

## 3. 闭环验证协议（`tests/x_vperiph_mcusim.rs`）

两个子测试共享 `run_closed_loop` 单步逻辑：
读回 PWM 推力 → 物理推进（起飞台保持/飞行）→ 注入真值到 FlySimState → `run(N)` 推进固件。

| 测试 | 步数 | 场景 | 判定 |
|---|---|---|---|
| `vperiph_closed_loop` | 60（0.24s） | 起飞台保持 → 升空 | max_thrust>0.05；pos 有限；roll/pitch<1° |
| `vperiph_hover_long` | 300（1.2s） | 升到 5m 悬停点保持 | 末段（后 30%）\|dz\|<0.8m、\|dx\|,\|dy\|<0.5m；roll/pitch<0.05rad |

运行前置（构建固件）：

```bash
cd ~/work/fc-umbrella && ./scripts/build.sh real-sensors   # minimal ELF + 两种固件均由此驱动
cd ~/work/fc-umbrella/mcu_simulater && cargo test --release --test x_vperiph_mcusim
```

预期输出末尾：
`>>> [VPERIPH-MCUSIM] 虚拟外设直通闭环验证通过 ✓` 与
`>>> [VPERIPH-MCUSIM] 长时悬停收敛验证通过 ✓`。

## 4. 关键注入细节

### 4.1 boot 前注入初始真值（必要前置）

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

### 4.2 每物理步注入

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

### 4.3 解锁方式

- **RC 解锁（闭环测试实际路径）**：每物理步注入 `rc_ch[4]=2000`（armed）、
  `rc_ch[3]=1500`（油门中位）→ 虚拟 SBUS 从设备按 20Hz 推流 → 固件 RcSbus
  经 uart2 标准驱动解帧解锁（完整走真实 UART/SBUS 驱动链路）。
- 地面站 ARM（旁路，调试用）：直接写 `G_CMD_ARMED`（`0x2000_b669`，
  AtomicBool）= 1，不经 SBUS 链路。

### 4.4 PWM 推力读回

```rust
duty = ccr / arr;            // 0..1 占空比
us = duty * 2500.0;          // 400Hz 周期 2500us
m = ((us - 1000.0) / 1000.0).clamp(0.0, 1.0);
```

## 5. 已解决阻塞及根因（2025-06 → 2026-09）

| # | 阻塞 | 根因 | 解决方案 | 验证 |
|---|---|---|---|---|
| 1 | run 偶发 `UC_ERR_INSN_INVALID` / emu_start 卡死 | Unicorn `translate.c` 在 IT 指令结束后未清零 `condexec_bits` → 后续指令被错误解释为条件执行 | 打补丁：`gen_set_condexec` 在 IT 块结束时写 0（校验哈希在 `.cargo-checksum.json`） | `x_jos_p2`、`x_vperiph_mcusim` 长跑稳定 |
| 2 | 固件 I2C 读间歇返回 0 → EKF 垂向发散 vz=-130 → PWM 中位 | 悬停比力符号约定不一致：I2C/SPI 注入 +9.81、hil.rs tilt alignment 期望 -9.81 → EKF 失配 | 统一 FRD 约定：悬停比力 `(0,0,-9.81)`（data_source/mpu6050/bmi088 同改）；clamp 恢复标准 `clamp(v,0,1)` | 闭环 300 步稳定收敛 |
| 3 | SBUS UART 推流：CR3 无 DMAR → IDLE ring 空 → RcSbus 恒 n=0 | 固件 uart2 期望 DMA+IDLE 收帧；模拟器 CR3 无 DMAR，字节只进 rx_fifo | 模拟器按真机语义走 **CPU 轮询读 DR 路径**：`read(SR)` 返回含 IDLE 状态、`read(DR)` 弹 rx_fifo 并清 IDLE（`src/peripheral/usart.rs:292-326`）；SBUS 驱动改为标准 8E2 100kbps 反向电平 + `UART_IOCTL_SET_INVERTED` | `RcSbus::read()` 解出 ch、armed=ch4>1700、throttle=ch3 |
| 4 | GPS 链路（gps_w=false） | 同 #3 的 UART 推流问题 | 同 #3 解决；`NmeaGps` 经同一 rx_fifo 路径推流 | 闭环内 gps_fix=3 生效 |

另有两项**结构性根因修复**（非阻塞，属必要修正）：
- **Non-HIL setpoint_valid 恒 false** → 改 true，使 `hil_pos_inited` gate 可置位、执行器不再恒 0。
- **HIL IMU 单次消费**：`f.imu = None` 读后即清，防止同一 32ms 帧被 8 次积分。

## 6. 诊断共享区地址（随固件构建变化，重建后以 nm 复核）

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

## 7. 同步机制（vperiph 锁步 + HIL 事件同步并存）

- **vperiph 模式（闭环测试实际机制）**：`run_closed_loop` 是**锁步**驱动——
  读回 PWM 推力 → 物理推进（4ms）→ 写 FlySimState → `machine.run(N)` 推进固件，
  无信号量。FlySimState 只在两次 `run()` 之间写入（`run()` 期间冻结），
  多字节读事务的一致性由该锁步保证。
- **HIL 模式（另一条路径）**：`HIL_EVT` 信号量同步
  （`flyctrl/core/src/hil.rs`，`Semaphore<1>`，cfg feature="hil" 引入），
  PC 每 ~32ms 注入一帧 `HIL_SENSOR`。`HilContext::step_hil()` 为 SIL/HIL 共用
  步进：注入 IMU 真值 → 推进 EKF → 取期望轨迹点 → 更新执行器。
- 虚拟外设/推流时钟以**退休指令数**为基准（`dt = Δretired / VIRTUAL_INSNS_PER_SEC`，
  权威常量见 `src/sim/timing.rs`），与中断频率解耦（SBUS/GPS 均 20Hz）。

## 8. FRD 坐标约定（易错点速查）

| 量 | 约定 | 悬停值 |
|---|---|---|
| 机体系比力（accel） | FRD（前右下） | `(0,0,-9.81)` m/s² |
| 世界位置 | NED（n,e,d 向下正） | 起飞台 d=-5 |
| baro 高度 | 固件内部**向上正**，step_hil 内部取反为 D 向下 | 家庭点 h=0 |
| GPS alt | 海拔（m） | 起飞台 alt=9 → ref_alt=9 |

关键对齐：GPS 原点（首次定位锁定）与气压家庭点（起飞台 d=-5）对齐，避免 EKF 高度源冲突
（历史根因：GPS d=0 vs baro d=-5 折中 → hold_alt 锁错 → 机体持续下沉）。
