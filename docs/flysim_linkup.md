# fly_simulater × mcu_simulater 虚拟外设直通接口协议

> 目标：fly_simulater（物理真值）与 mcu_simulater（MCU + 固件）联调。
> **不走 USB、不走共享内存式驱动旁路**：fly_sim 每步把物理真值写进共享
> `FlySimState`（Arc\<Mutex\>），mcu_sim 的虚拟外设（I2C/UART 从设备）经
> `FlySimSource` 即时读取填动态寄存器；固件 real-sensors **走标准驱动**
> （sensors_task → SensorStack → i2c0/uart 驱动）读取；控制量走标准 PWM 驱动
> 写虚拟 TIM，fly_sim 读回 CCR/ARR 更新动力学。全程无 USB。

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

## 2. 总线 / 设备映射（固件视图 ↔ mcu_sim 挂载点）

| 固件设备名 | mcu_sim 外设 | port（挂载索引） | 虚拟从设备 | 地址 |
|---|---|---|---|---|
| `i2c0` | I2C1 | 1（`register_i2c_slave(1, …)`） | mpu6050 | 0x68 |
| `i2c0` | I2C1 | 1 | bmp280 | 0x76 |
| `i2c0` | I2C1 | 1 | qmc5883（静态磁，无真值源） | 0x0D |
| `uart1` | USART2 | 2（`register_uart_slave(2, …)`） | NmeaGps（\$GNGGA 推流） | — |
| `uart2` | USART3 | 3（`register_uart_slave(3, …)`） | Sbus（SBUS 推流） | — |
| PWM0-3 | TIM3/2/1/4 CH1 | 0x40000400/0x40000000/0x40010000/0x40000800 | CCR1=+0x34，ARR=+0x2C | 400Hz |

## 3. 数据通道协议（FlySimState 字段 → 设备寄存器换算）

`FlySimState`（`src/peripheral/vperiph/data_source.rs`）：

| 字段 | 类型 | 物理量 | 说明 |
|---|---|---|---|
| `imu_acc[3]` | f32 | m/s² | 机体系**比力**（FRD）。静止水平悬停 = `(0,0,-9.81)`（抵消重力，与 mpu6050 设备约定、hil.rs tilt alignment 一致） |
| `imu_gyr[3]` | f32 | rad/s | 机体系角速度 |
| `baro_pa` | f32 | Pa | 气压（101325 = 海平面） |
| `gps_lat/lon` | f32 | 度 | 位置 |
| `gps_alt` | f32 | 米 | 海拔（向下为正，NED） |
| `gps_fix` | f32 | — | 定位状态（0/1/2/3） |
| `rc_ch[16]` | f32 | 1000..2000 | SBUS 通道（ch3=油门，ch4=armed；注入 1000..2000，SBUS raw 换算见 §7.1 #3） |

设备寄存器换算（`vperiph/i2c/mpu6050.rs`、`bmp280.rs`，动态寄存器读时求值）：

| 通道 | 寄存器 | 换算（值 → raw） | 固件还原 |
|---|---|---|---|
| accel.x/y/z | 0x3B 起 6B（i16 BE） | `raw = v * 16384 / 9.81`（±2g 量程，LSB=16384/g） | `raw * 9.81 / 16384` |
| gyro.x/y/z | 0x3B+8 起 6B（i16 BE） | `raw = v * 131 * 180 / π`（±250°/s，LSB=131/(°/s)） | `raw / 131 * π / 180` |
| 温度 | 0x3B+6（2B） | 置 0（驱动不读） | — |
| pressure | 0xF7 起 6B（20bit，`p20 = Pa << 4`） | `p20 = v << 4` | 固件 `(raw >> 4)` 当 Pa |
| altitude | （气压→高度） | 由固件计算 `h = 44330*(1-(p/101325)^0.1903)`，alt=-h 向下正 | — |

## 4. 时间基准（lock-step 4ms）

- 测试主循环每步 = 一个物理步（4ms）：读回 PWM 推力 → `SimLoop::step_hil(&cmd)` 更新动力学 → 写 `FlySimState`（imu/pos/baro/gps/rc）→ `machine.run(N)` 推进固件（sensors 2ms 采样 + control 4ms + PWM 写）→ 下一物理步。
- 固件 control 拍（4ms）与物理步（4ms）对齐；sensors 采样 2ms 为固件内部频率。
- `run(N)` 的虚拟时间按退休指令数推进（`dt = Δretired / VIRTUAL_INSNS_PER_SEC`，
  权威常量见 `src/sim/timing.rs`），虚拟外设/推流时钟以指令为基准，与中断频率解耦。

## 5. 注入 API

```rust
// 1) 共享状态
let state = Arc::new(Mutex::new(FlySimState::default()));
// 2) 装配虚拟外设（挂到 I2C1 / UART2,3）
m.attach_flysim_sensors(state.clone());      // mpu6050/bmp280/qmc5883（qmc 静态）
m.attach_flysim_uart_slaves(state.clone());  // NmeaGps(USART2) + Sbus(USART3) 推流
// 3) 每物理步注入（写 FlySimState 字段）
let mut st = state.lock().unwrap();
st.imu_acc = [ax, ay, az];   // 机体系比力 m/s²（FRD 悬停 = (0,0,-9.81)）
st.imu_gyr = [gx, gy, gz];   // rad/s
st.baro_pa = 101_325.0 * (-h / 8434.5).exp();  // 标准大气
st.gps_lat = LAT0 + n / 111_320.0;             // NED → 度
st.gps_lon = LON0 + e / (111_320.0 * LAT0.to_radians().cos());
st.gps_alt = ALT0 - d;                         // 向下正
st.gps_fix = 3.0;
st.rc_ch[..] = /* 1000..2000 */;
// 4) 读回 PWM 推力（测试读虚拟 TIM CCR1/ARR）
//    us = 1000 + 1000*m；ticks = us*period/2500；period = ARR+1
```

## 6. 固件侧观测约定（联调诊断共享区）

control.rs / pid.rs 用 `#[used] static` 直写诊断变量（测试按符号地址 rd() 读 LE），
地址随固件构建变化，重建后以 `arm-none-eabi-nm app.elf` 复核（见 §7.3 地址表）：

- `DBG_MOTOR[4]`（`app/src/flyctrl/control.rs:240`）：4 路 clamp 后电机指令，每 control 拍刷新。
- `DBG_PID[12]` / `DBG_PRE` / `DBG_THR`（`core/src/controller/pid.rs`）：垂向 PID 环诊断。
- `HIL_DIAG_GATES`（`core/src/hil.rs`）：7-bit 联调门（armed/rc_fresh/health/est_finite/sp_finite/att_inited/pos_inited）。

RcSbus 观测此前约定在 0x2002_0100 起共享 RAM，已随旧共享数组废弃；当前 SBUS 状态
以 `RcSbus::read()` 返回的 `RcInput`（fresh/armed/throttle）为准。

## 7. 验证状态（2026-09 更新：2025-06 阻塞全部解决）

### 7.1 已解决阻塞及根因

| # | 2025-06 阻塞 | 根因 | 解决方案 | 验证 |
|---|---|---|---|---|
| 1 | run 偶发 `UC_ERR_INSN_INVALID` / emu_start 卡死 | Unicorn `translate.c` 在 IT 指令结束后未清零 `condexec_bits` → 后续指令被错误解释为条件执行 | 打补丁：`gen_set_condexec` 在 IT 块结束时写 0（校验哈希在 `.cargo-checksum.json`） | `x_jos_p2`、`x_vperiph_mcusim` 长跑稳定 |
| 2 | 固件 I2C 读间歇返回 0 → EKF 垂向发散 vz=-130 → PWM 中位 | 悬停比力符号约定不一致：I2C/SPI 注入 +9.81、hil.rs tilt alignment 期望 -9.81 → EKF 失配 | 统一 FRD 约定：悬停比力 `(0,0,-9.81)`（data_source/mpu6050/bmi088 同改）；clamp 恢复标准 `clamp(v,0,1)` | 闭环 300 步稳定收敛 |
| 3 | SBUS UART 推流：CR3 无 DMAR → IDLE ring 空 → RcSbus 恒 n=0 | 固件 uart2 期望 DMA+IDLE 收帧；模拟器 CR3 无 DMAR，字节只进 rx_fifo | 模拟器按真机语义走 **CPU 轮询读 DR 路径**：`read(SR)` 返回含 IDLE 状态、`read(DR)` 弹 rx_fifo 并清 IDLE（`src/peripheral/usart.rs:292-326`）；SBUS 驱动改为标准 8E2 100kbps 反向电平 + `UART_IOCTL_SET_INVERTED` | `RcSbus::read()` 解出 ch、armed=ch4>1700、throttle=ch3 |
| 4 | GPS 链路（gps_w=false） | 同 #3 的 UART 推流问题 | 同 #3 解决；`NmeaGps` 经同一 rx_fifo 路径推流 | 闭环内 gps_fix=3 生效 |

另有两项**结构性根因修复**（非阻塞，属必要修正）：
- **Non-HIL setpoint_valid 恒 false** → 改 true，使 `hil_pos_inited` gate 可置位、执行器不再恒 0。
- **HIL IMU 单次消费**：`f.imu = None` 读后即清，防止同一 32ms 帧被 8 次积分。

### 7.2 闭环验证协议（`tests/x_vperiph_mcusim.rs`）

两个子测试共享 `run_closed_loop` 单步逻辑（读 PWM → 物理推进 → 注入真值 → run(N) 推进固件）：

| 测试 | 步数 | 场景 | 判定 |
|---|---|---|---|
| `vperiph_closed_loop` | 60（0.24s） | 起飞台保持 → 升空 | max_thrust>0.05；pos 有限；roll/pitch<1° |
| `vperiph_hover_long` | 300（1.2s） | 升到 5m 悬停点保持 | 末段（后 30%）|dz|<0.8m、|dx|,|dy|<0.5m；roll/pitch<0.05rad |

运行前置（构建固件）：
```bash
cd /home/ubuntu/work/joc-base && cmake --build build_rel          # minimal elf
cd /home/ubuntu/work/flyctrl && python3 build_app.py --features real-sensors --out /tmp/flyctrl_clean.bin
cd /home/ubuntu/work/mcu_simulater && cargo test --release --offline --test x_vperiph_mcusim
```
预期输出末尾：`>>> [VPERIPH-MCUSIM] 虚拟外设直通闭环验证通过 ✓` 与 `>>> [VPERIPH-MCUSIM] 长时悬停收敛验证通过 ✓`。

### 7.3 诊断共享区地址（随固件构建变化，重建后以 nm 复核）

| 地址 | 符号 | 含义 |
|---|---|---|
| `0x2000_b61c` | `DBG_MOTOR[4]` | 4 路 clamp 后电机指令 |
| `0x2000_b62c` | `HIL_DIAG_GATES` | 7-bit 位域：armed(0)/rc_fresh(1)/health(2)/est_finite(3)/sp_finite(4)/att_inited(5)/pos_inited(6) |
| `0x2000_b630` | `DBG_PID[12]` | 垂向 PID 环诊断（每组 4 f32） |
| `0x2000_b660` | `DBG_PRE` | 垂向环误差 |
| `0x2000_b664` | `DBG_THR` | 垂向环输出油门 |
| `0x2000_b669` | `G_CMD_ARMED` | 地面站 ARM 标志（测试直接置 1 模拟解锁） |

复核命令：`arm-none-eabi-nm app.elf | grep -E 'HIL_DIAG_GATES|DBG_MOTOR|DBG_PID|DBG_PRE|DBG_THR|G_CMD_ARMED'`。
这些 `#[used] static` 诊断变量的语义另见 §6。

### 7.4 同步机制（vperiph 锁步 + HIL 事件同步并存）

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

### 7.5 FRD 坐标约定（易错点速查）

| 量 | 约定 | 悬停值 |
|---|---|---|
| 机体系比力（accel） | FRD（前右下） | `(0,0,-9.81)` m/s² |
| 世界位置 | NED（n,e,d 向下正） | 起飞台 d=-5 |
| baro 高度 | 固件内部**向上正**，step_hil 内部取反为 D 向下 | 家庭点 h=0 |
| GPS alt | 海拔（m） | 起飞台 alt=9 → ref_alt=9 |

关键对齐：GPS 原点（首次定位锁定）与气压家庭点（起飞台 d=-5）对齐，避免 EKF 高度源冲突（历史根因：GPS d=0 vs baro d=-5 折中 → hold_alt 锁错 → 机体持续下沉）。
