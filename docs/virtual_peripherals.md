# 虚拟外设仿真平台设计

> 目标：在 mcu_simulater 之上提供**总线协议级**的虚拟外设仿真，使 MCU 应用（如
> flyctrl real-sensors）在硬件未就绪时可直接在仿真平台上调试验证——不 mock 驱动、
> 不注入内存值，固件走真实 I2C/UART 寄存器与驱动代码，读到的是虚拟从设备按
> 物理模型产生的数据。

## 1. 设计决策（与需求方确认）

| # | 决策 | 说明 |
|---|---|---|
| 1 | 总线协议级模拟 | 从设备挂在 I2C/UART 总线外设内部，固件通过寄存器级事务（START/地址/R/W/数据/AF）与之交互，驱动代码零改动 |
| 2 | 从设备直路由 | 从设备由总线外设持有（`i2c.slaves` / `usart.slaves`），地址匹配后**直路由**，不经事件总线（保持事务原子性） |
| 3 | 数据源两段式 | `DataSource` = **Constant**（静态寄存器值）+ **Math**（物理模型随仿真时间 `step(dt)` 推进）；首批均为静态模型（悬停场景） |
| 4 | 首批外设 | MPU6050（IMU）、BMP280（气压）、QMC5883（磁力）、u-blox GPS（$GNGGA 推流）、SBUS（RC 推流）——对齐 flyctrl real-sensors 全链路 |
| 5 | TOML 拓扑文件 | 换场景不改代码：`[[i2c_slave]]` / `[[uart_slave]]` 描述「挂哪条总线/端口、用哪个模型」，`config::apply_topology` 装配 |

## 2. 架构分层

```
┌──────────────────────────────────────────────────────────────┐
│ 场景层：examples/topology_flyctrl.toml（TOML 拓扑描述）        │
│          config::apply_topology(machine, toml)                │
├──────────────────────────────────────────────────────────────┤
│ 模型层：vperiph/models.rs（寄存器布局）                        │
│          vperiph/data_source.rs（Const/Math + SensorModel）    │
│          vperiph/uart.rs（NmeaGps/Sbus 推流）                  │
├──────────────────────────────────────────────────────────────┤
│ 从设备层：vperiph/mod.rs（VirtualI2cSlave / RegFileSlave）     │
│            vperiph/uart.rs（VirtualUartSlave）                 │
├──────────────────────────────────────────────────────────────┤
│ 总线外设层：peripheral/i2c.rs（事务级状态机 + 从设备表）        │
│              peripheral/usart.rs（RX FIFO + 帧末 IDLE）         │
├──────────────────────────────────────────────────────────────┤
│ Machine 装配：register_i2c_slave / register_uart_slave          │
│               inject_i2c_nack（故障注入）                       │
│               run() 每迭代 step_virtual_slaves + step_virtual_uart │
└──────────────────────────────────────────────────────────────┘
```

## 3. I2C 事务级升级（固件 master 语义）

`peripheral/i2c.rs` 在保持既有 DMA 简化路径（向后兼容）的同时，把 **CPU 写 DR 路径**
升级为真实 STM32F1 I2C master 事务状态机：

| 固件动作 | 模拟器响应 |
|---|---|
| 写 `CR1.START` | 置 `SR1.SB`（起始位），进入地址阶段 |
| 写 `DR`（地址阶段，addr7=R/W） | 匹配从设备表：命中置 `SR1.ADDR`（读 `SR2` 清）、未命中置 `SR1.AF` |
| 读 `SR2`（清 ADDR，读方向） | 预取首字节置 `RXNE`；此后读 `DR` 返回并预取下一字节（连续流） |
| 数据阶段写 `DR` | 直路由当前从设备 `on_write`（寄存器指针语义） |
| 从设备无数据/断线（`on_read → None`） | 置 `SR1.AF`（固件判读失败） |
| 写 `CR1.STOP` | 结束事务 |

这使 joc-base `stm32/i2c_hal.c` 的轮询驱动（SB/ADDR/TXE/RXNE/BTF）**无需任何改动**
即可在模拟器上跑通（flyctrl real-sensors 的 mpu6050/bmp280 驱动即如此验证）。

## 4. 从设备模型

### 4.1 `RegFileSlave`（通用寄存器文件 I2C 从设备）

- 寄存器文件 + 寄存器指针（`ptr`）+ 期待首字节为地址（`expect_reg`）
- **动态寄存器**：`add_dynamic(offset, len, source, fill)`——固件读取瞬间从 `DataSource`
  求值（真实传感器语义：数据在读取时产生）
- 故障注入：`nack=true` 时 `on_read → None` → 总线置 AF（断线/无响应模拟）
- 观测：`read_count()/n_writes/n_reads`（断言固件是否真的读了虚拟外设）

### 4.2 首批模型（对齐 flyctrl real-sensors 驱动读法）

| 模型 | 地址 | 寄存器布局 | 物理模型（默认） |
|---|---|---|---|
| `mpu6050` | 0x68 | WHO_AM_I=0x68@0x75；ACCEL_XOUT_H 0x3B 起 14B（accel×3 + temp + gyro×3，BE i16） | StaticImu：accel=[0,0,9.81]（悬停抵消重力）、gyro=0 |
| `bmp280` | 0x76 | ID=0x58@0xD0；0xF7 起 6B（压力 20bit，驱动 `>>4` 还原 Pa） | StaticBaro：101325 Pa（海平面） |
| `qmc5883` | 0x0D | 0x00 起 6B（三轴磁力，LE i16，±2G） | StaticMag：[0.2,0,0.4] G（北向地磁） |

### 4.3 UART 推流从设备

- **`NmeaGps`**：按 period（默认 0.05s/20Hz；模拟器用高帧率补偿固件 POLL 慢读）生成
  `$GNGGA` 帧（校验和正确、quality=3 有效定位、NED 原点 31.2304N/121.4737E/alt4m）
- **`Sbus`**：25B 帧 16×11bit LSB-first 打包（通道值 1000..2000；悬停中位 + ch2 油门最小）
- 推流经 `Usart::feed_rx_queued` 进 **RX FIFO**（字节排队，固件逐字节读走不 ORE 丢弃，
  CPU/DMA 读 DR 同语义队首消费、队列非空保持 RXNE）；帧末置 `SR.IDLE` + `CR1.IDLEIE`
  挂起中断（固件 IDLE 帧边界检测）

## 5. TOML 拓扑配置

```toml
# 悬停场景（examples/topology_flyctrl.toml）
[[i2c_slave]]          # I2C 从设备
port = 1               # I2C1（flyctrl real-sensors 的 i2c0）
name = "mpu6050"       # 观测名
addr = 0x68            # 7bit 地址
model = "mpu6050"      # 模型名
source = "imu"         # 物理模型名

[[uart_slave]]         # UART 推流从设备
port = 2               # USART2（flyctrl 的 uart1）
name = "gps"
model = "nmea_gga"
source = "gps"
```

装配（代码侧一行）：

```rust
let toml = include_str!("../examples/topology_flyctrl.toml");
mcu_simulater::config::apply_topology(&m, toml)?;  // 返回 Vec<TopologyNode>（可断言）
```

支持 `i2c_slave` / `uart_slave`（`spi_slave` 预留）。offline 环境无 `toml` crate，内置
极简 TOML 子集解析器（数组表 + 字符串/整数/浮点/十六进制，带行号报错）；模型工厂
`build_i2c_slave`/`build_uart_slave` 按 (model, source) 分派到具体静态模型。

## 6. 故障注入

- **NACK**：`Machine::inject_i2c_nack(port, addr7, nack)` → `RegFileSlave.nack` →
  `on_read → None` → 总线 `SR1.AF` → 固件驱动读失败 → `healthy()=false` → 应用层
  FDIR 降级/安全（flyctrl：IMU 冻结判 `Critical` → 执行器归零）。
- 按地址精准隔离：同总线 healthy 从设备不受影响（验收断言 read_count 冻结/继续）。

## 7. 验收结果

| 测试 | 内容 | 结果 |
|---|---|---|
| `x_flyctrl_real_sensors` | flyctrl real-sensors 经虚拟从设备全链路：挂载+四任务+心跳+IMU/Baro 数据+GPS 定位 | ✅ imu_ok/baro_ok/gps_ok，`fix established: lat=31.230400 lon=121.473701 alt=4.0` |
| `x_toml_topology` | 同一验收改由 TOML 拓扑装配（换场景不改代码） | ✅ 与代码侧装配等价 |
| `x_fault_injection` | ①开机即 NACK mpu6050 → FDIR Critical + baro/GPS 照常 ②运行中 NACK → 该从设备读冻结、bmp280 继续 | ✅ 两场景 |
| 全量回归 | lib 140 单测 + 53 集成测试二进制 | ✅ 通过 |

## 8. 已知限制与后续

- **GPS fix 在故障场景下变慢**：固件 uart1 为 POLL 模式（1 字节/传感器循环），死 IMU
  的 I2C 超时把传感器循环拖慢 ~4x → 从设备推流（20Hz）远快于固件消费（FIFO 积压
  数十 KB）；fix 出现时间随故障放大。非故障场景验收正常。
- **固件 uart DMA 在模拟器上不可用**（joc-base 引擎的 DMA malloc 在 8KB heap 下失败
  回退 POLL）——既有集成限制，非虚拟外设设计缺陷；GPS 端到端已协议级验证。
- **FDIR 冻结判据依赖精确浮点相等**（`a == last_acc`）：IIR 输入滤波衰减到 f32 抖动
  时可能长期不判冻结。从开机即故障（零初始滤波状态）可确定性触发；运行中注入需
  滤波稳定（~200 拍）或改进判据（容差比较）——flyctrl-core 侧改进项。
- **SPI 从设备**：`VirtualSpiSlave` trait 预留，本轮未实现（flyctrl 首批无 SPI 传感器）。
- **动态 Math 模型**：`DataSource::Math` 已支持 `step(dt)` 推进，后续可加噪声/漂移/
  运动学模型（如起飞爬升、GPS 漂移）丰富场景。
