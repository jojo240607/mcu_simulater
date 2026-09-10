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
│     └ uart2: NmeaGps（GPS）/ uart3: Sbus（SBUS 推流）          │
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
| `imu_acc[3]` | f32 | m/s² | 机体系**比力**（FRD）。静止水平悬停 = `(0,0,9.81)`（抵消重力，与 mpu6050 设备约定一致）；hil.rs tilt alignment 期望悬停比力 `(0,0,-9.81)`（FRD） |
| `imu_gyr[3]` | f32 | rad/s | 机体系角速度 |
| `baro_pa` | f32 | Pa | 气压（101325 = 海平面） |
| `gps_lat/lon` | f32 | 度 | 位置 |
| `gps_alt` | f32 | 米 | 海拔（向下为正，NED） |
| `gps_fix` | f32 | — | 定位状态（0/1/2/3） |
| `rc_ch[16]` | f32 | 1000..2000 | SBUS 通道（ch2=油门，ch4=armed 常见约定） |

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
- `run(N)` 的虚拟时间按退休指令数推进（`dt = Δretired / VIRT_INSN_PER_SEC`），虚拟外设/推流时钟以指令为基准，与中断频率解耦。

## 5. 注入 API

```rust
// 1) 共享状态
let state = Arc::new(Mutex::new(FlySimState::default()));
// 2) 装配虚拟外设（挂到 I2C1 / UART2,3）
m.attach_flysim_sensors(state.clone());      // mpu6050/bmp280/qmc5883（qmc 静态）
m.attach_flysim_uart_slaves(state.clone());  // NmeaGps(Sbus 推流)
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

control.rs 每 control 拍直写共享 RAM（0x2002_0000 起 f32 数组，测试 rd(off) 读 4B LE）：

| off | 含义 | off | 含义 |
|---|---|---|---|
| 0 | hil gates（HIL_DIAG_GATES 位域） | 11 | est.vel[2] |
| 1 | est.pos[2] | 12 | yaw |
| 2 | armed_eff | 13 | imu.accel[2] |
| 3 | roll | 14 | imu.gyro[2] |
| 4 | pitch | 15 | armed 强制标志（1.0） |
| 5 | setpoint_valid | 16 | cmd.motor[0]（clamp 后） |
| 6 | rc.fresh | 17 | PWM ticks |
| 7 | rc.armed | 18 | pwm ioctl 返回 |
| 8 | rc.throttle | 19 | pwm_dev[0] is_some |
| 9 | est.pos[0] | 20-22 | imu.accel[0..2]（全量） |
| 10 | est.pos[1] | 23 | baro_alt |

0x2002_0100 起：RcSbus 观测（n / fill / fresh / ch[4] / buf[0]）。

## 7. 验证状态与已知阻塞（2025-06 记录）

**已验证（架构链路端到端）**：
- FlySimState 注入 → FlySimSource → RegFile 动态寄存器编码正确（测试模拟 on_read：
  accel.z=9.81 → raw=0x4000；-9.81 → 0xC000；baro 101325 → p20=0x18BCD0）。
- hil gates 全过（HIL_DIAG_GATES=127：armed/rc_fresh/health/est_finite/sp_finite/att/pos）。
- control 输出 cmd 非 0；PWM 设备 open 成功（pwmok=true），TIM CCR1 写路径通
  （中位 us=1000 → ticks≈206，ARR≈515 对应 400Hz）。

**已知阻塞（mcu_sim/Unicorn 模拟层，待深挖）**：
1. run 不稳定：偶发 UC_ERR_INSN_INVALID（~step 172），或 emu_start 内部卡死
   （block 内不返回，风暴护栏 1M 段无法兜底；同一代码多次运行结果不同）。
2. run 内固件 I2C 读间歇返回 0（注入 -9.81 但固件偶发读到 0 → EKF 垂向速度发散
   vz=-130 → des_thrust 被 clamp 到 0 → PWM 中位）。测试直接模拟 on_read 读对，
   断点在固件经 mcu_sim I2C1 状态机的事务路径。
3. SBUS UART 推流链路：固件 uart2（DMA+IDLE）的 CR3 无 DMAR → 字节只进 rx_fifo，
   IDLE ring 空 → RcSbus read 恒 n=0。当前用 control 临时放宽
   （`let rc_fresh = true` / `armed_eff = true`，均带 TODO）绕过，最终需修 DMA
   使能时序或改 board engine。
4. GPS 链路（gps_w=false）同 UART 推流问题，可后补。
