//! Machine：装配 CPU、内存与外设；加载固件。
//!
//! M0 阶段：CPU + 固定板级内存布局 + ELF 加载 + 复位（向量表）。
//! M1 接入内存总线（MMIO 经 mem hook 转发到 Rust 外设），M3 起由配置 DSL 驱动装配。
//! M2 接入 MPU（MMIO/RAM-Flash-CCM 数据访问 + 取指 XN 三入口访问控制）
//!     与中断投递（NVIC 挂起抢占 + 异常入栈/出栈）。

use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use object::read::elf::{ElfFile, FileHeader, ProgramHeader};
use object::read::ReadRef;
use object::{elf, Object, ObjectSection, ObjectSegment, SectionFlags, SectionKind};
use unicorn_engine::{HookType, MemType, Prot, RegisterARM, Unicorn};

use crate::bus::Bus;
use crate::core::{CoreError, Cpu, Result};
use crate::events::{Event, EventBus};
use crate::peripheral::adc::{Adc, ADC_IRQ};
use crate::peripheral::can::{Can, CAN1_BASE, CAN2_BASE};
use crate::peripheral::dac::Dac;
use crate::peripheral::console::Console;
use crate::peripheral::crc::Crc;
use crate::peripheral::dcmi::{Dcmi, DCMI_IRQ};
use crate::peripheral::dwt::Dwt;
use crate::peripheral::terminal::Terminal;
use crate::peripheral::dma::{Dma, DmaDir, DMA1_BASE, DMA1_STREAM_IRQ, DMA2_BASE, DMA2_STREAM_IRQ};
use crate::peripheral::usb_otg::{UsbOtg, USB_OTG_FS_BASE};
use crate::peripheral::exti::{Exti, EXTI_BASE};
use crate::peripheral::flash::Flash;
use crate::peripheral::fsmc::{
    Fsmc, FSMC_BANK1_BASE, FSMC_BANK2_BASE, FSMC_BANK3_BASE, FSMC_BANK4_BASE, FSMC_BANK_SIZE,
    FSMC_BASE,
};
use crate::peripheral::gpio::Gpio;
use crate::peripheral::i2c::{I2c, I2C1_EV_IRQ, I2C2_EV_IRQ, I2C3_EV_IRQ};
use crate::peripheral::mpu::{Access, MemManageFault, Mpu};
use crate::peripheral::nvic::{Nvic, StopReason};
use crate::peripheral::rcc::Rcc;
use crate::peripheral::rng::{Rng, RNG_IRQ};
use crate::peripheral::pwr::Pwr;
use crate::peripheral::rtc::Rtc;
use crate::peripheral::scb::SystemControl;
use crate::peripheral::sdio::{Sdio, SDIO_BASE};
use crate::peripheral::spi::{Spi, SPI1_IRQ, SPI2_IRQ, SPI3_IRQ};
use crate::peripheral::syscfg::{ExtiPortSelect, Syscfg};
use crate::peripheral::timer::{Timer, TimerConfig, TimerIrq, TimerKind};
use crate::peripheral::usart::{Usart, USART1_IRQ, USART2_IRQ, USART3_IRQ, UART4_IRQ, UART5_IRQ, USART6_IRQ};
use crate::peripheral::wdog::{Iwdg, ResetReason, WdogResetReq, Wwdg};
use crate::peripheral::{Peripheral};
use crate::sim::status::{Status, BIT_ANY_ACTIVE, BIT_MPU, BIT_NVIC_PENDING, BIT_WDOG};
use crate::sim::timing::VirtualClock;


/// block hook 冷路径状态（MPU/NVIC/外设 tick 列表），捆进单个 Arc 以缩小闭包捕获体
///（bench_probe：H13d 6 字段捕获 97.7 → H13e 3 字段 121+ MIPS，闭包捕获字段数即热路径成本）。
struct BlockHookCold {
    mpu: Arc<Mutex<Mpu>>,
    nvic: Arc<Mutex<Nvic>>,
    timers: Vec<Arc<Mutex<dyn Peripheral>>>,
    /// ★§5.102 ④：与该 `timers` 平行的标记 —— `true` = 该外设吃【原始字节流】
    /// （**SCB/SysTick** ✓：其 RVR=168_000 就是"1 固件ms = 168_000 周期"的定义 ✓）；
    /// `false` = TIM ✓，改吃【声明时钟折算流】（84MHz 基准 ✓）。
    /// ★用 **Arc 指针同一性**判定（不用 `name()` ✗，避免逐设备加锁 ✗ —— 上一版
    /// 用名称映射时该用例从 17s 变成 >640s ✗，见 §5.103）。
    tick_raw: Vec<bool>,
    tick_actives: Vec<Arc<AtomicBool>>,
    /// 退休指令计数（block hook 每块累加 TB 字节≈Thumb 指令数×2；run() 按此递减预算）
    retired: Arc<AtomicU64>,
    /// DMA 控制器（块 hook 每 N 块只读 pending 位图；有请求则停机让 run() 搬）
    dma1: Arc<Mutex<Dma>>,
    dma2: Arc<Mutex<Dma>>,
    /// 块计数器（每 256 块检查一次 DMA 待搬运请求）
    dma_ticks: AtomicU32,
}

/// 一台仿真的 MCU
pub struct Machine {
    /// 处理器（Unicorn）
    pub cpu: Cpu,
    /// 内存总线（MMIO 外设注册与分发；hook 闭包持有其克隆）
    pub bus: Arc<Mutex<Bus>>,
    /// MPU（内存保护单元，挂载于 SCB 窗口，访问控制 hook 共享）
    pub mpu: Arc<Mutex<Mpu>>,
    /// NVIC（嵌套向量中断控制器，挂载于 SCB 窗口，中断投递 hook 共享）
    pub nvic: Arc<Mutex<Nvic>>,
    /// 事件总线（虚拟外设互联，M3）
    pub events: Arc<Mutex<EventBus>>,
    /// 虚拟 Console（订阅 UART TX，M3）
    pub console: Arc<Mutex<Console>>,
    /// 虚拟终端（M5：双向接线对象，显示缓冲 + 键盘 → UartRx）
    pub terminal: Arc<Mutex<Terminal>>,
    /// 共享虚拟时钟（block hook 推进，供 TIM 等外设 tick；原子计数无锁）
    pub clock: Arc<VirtualClock>,
    /// 时钟外设列表（block hook 按块 tick 推进）
    timers: Arc<Mutex<Vec<Arc<Mutex<dyn Peripheral>>>>>,
    /// 与 `timers` 并行的活动标记列表（外设使能状态原子同步，block hook 据此跳过
    /// 未激活外设的加锁 tick）
    tick_actives: Arc<Mutex<Vec<Arc<AtomicBool>>>>,
    /// 全局执行状态位域（MPU 使能 / 任一外设激活 / 中断挂起 / 看门狗复位请求）。
    /// block hook 热路径单次 `raw()` load + 位测试，替代原先 4~5 次独立原子判读
    ///（bench_probe：H2 独立原子 37.8 → H12 单状态字 115.1 MIPS）。
    status: Arc<Status>,
    /// DMA1/DMA2（tick 判传输完成；run 间隙 process 执行内存搬运）
    dma: Arc<Mutex<Dma>>,
    dma2: Arc<Mutex<Dma>>,
    /// DMA1/DMA2 活动标记（任一流 CR.EN 置位，Machine 持有并推入 tick_actives）
    dma_active: Arc<AtomicBool>,
    dma2_active: Arc<AtomicBool>,
    /// RCC（看门狗复位时置 CSR 复位标志）
    rcc: Arc<Mutex<Rcc>>,
    /// RNG 真随机数发生器（@0x50060800，AHB2；seed 可控供测试复现）
    pub rng: Arc<Mutex<Rng>>,
    /// PWR 电源控制（@0x40007000；低功耗位 + WUF/SBF 标志 + 待机唤醒复位路径）
    pub pwr: Arc<Mutex<Pwr>>,
    /// RTC 实时时钟 + 备份寄存器（@0x40002800，APB1；日历 + 闹钟/唤醒中断 + 掉电保持）
    pub rtc: Arc<Mutex<Rtc>>,
    /// DCMI 数字摄像头接口（@0x50050000，AHB2；帧注入 + DMA2 搬运 + IRQ78）
    pub dcmi: Arc<Mutex<Dcmi>>,
    /// FSMC 外部存储器控制器（@0xA0000000；Bank1-4 片选窗口 64KB 简化映射）
    pub fsmc: Arc<Mutex<Fsmc>>,
    /// SDIO 安全数字 IO（@0x40012C00；命令/响应 + FIFO + DMA2 + IRQ49 + 虚拟 SD 卡）
    pub sdio: Arc<Mutex<Sdio>>,
    /// I2C1-3 外设句柄（index = port-1；虚拟从设备挂载点，总线协议级直路由）
    pub i2c: Arc<Mutex<Vec<Arc<Mutex<I2c>>>>>,
    /// USART1-6 外设句柄（index = port-1；UART 虚拟从设备挂载点，推流）
    pub usart: Arc<Mutex<Vec<Arc<Mutex<Usart>>>>>,
    /// SPI1-3 外设句柄（index = port-1；SPI 虚拟从设备挂载点，全双工直路由）
    pub spi: Arc<Mutex<Vec<Arc<Mutex<Spi>>>>>,
    /// ESC 电调 + 无刷电机虚拟外设（信号观测：订阅 TimPwm/GpioLevel）
    pub escs: Arc<Mutex<Vec<Arc<Mutex<Box<dyn crate::peripheral::vperiph::esc::EscMotor>>>>>>,
    /// CAN 总线虚拟节点（订阅 CanFrame 按 ID 响应，回帧 feed_rx 到本端口 CAN）
    pub can_nodes: Arc<Mutex<Vec<Arc<Mutex<Box<dyn crate::peripheral::vperiph::can::CanNode>>>>>>,
    /// FSMC Bank1 挂载的 ST7789 LCD 虚拟器件（观测/断言：显存、命令计数）
    pub st7789: Arc<Mutex<crate::peripheral::vperiph::fsmc::St7789>>,
    /// CAN 虚拟节点待回帧（事件回调只入队，run 主循环 flush——避免事件分发
    /// 时本端口 CAN 锁重入死锁）
    can_pending: Arc<Mutex<Vec<crate::peripheral::can::CanFrame>>>,
    /// CAN1 控制器局域网（@0x40006400，APB1；邮箱/接收 FIFO/过滤 + CanFrame 总线互联）
    pub can1: Arc<Mutex<Can>>,
    /// CAN2 控制器局域网（@0x40006800，APB1；同上，与 CAN1 互联）
    pub can2: Arc<Mutex<Can>>,
    /// FLASH 控制器寄存器区（@0x40023C00，AHB1；ACR/KEYR/SR/CR 最小模型）
    pub flash: Arc<Mutex<Flash>>,
    /// USB OTG FS 全速设备控制器（@0x50000000，AHB1；设备模式核心寄存器/端点/
    /// FIFO/枚举 + 虚拟主机 UsbSetup 事件注入 + OTG_FS_IRQ=67）
    pub usb_otg: Arc<Mutex<UsbOtg>>,
    /// IWDG 独立看门狗（系统复位时复位外设，避免复位后立即再次超时）
    iwdg: Arc<Mutex<Iwdg>>,
    /// IWDG 活动标记（KR_START 启动，Machine 持有并推入 tick_actives）
    iwdg_active: Arc<AtomicBool>,
    /// WWDG 窗口看门狗（同上）
    wwdg: Arc<Mutex<Wwdg>>,
    /// WWDG 活动标记（CR.WDGA 置位，Machine 持有并推入 tick_actives）
    wwdg_active: Arc<AtomicBool>,
    /// RTC 活动标记（闹钟/唤醒使能或日历初始化，Machine 持有并推入 tick_actives）
    rtc_active: Arc<AtomicBool>,
    /// 看门狗复位请求（IWDG/WWDG/PWR 待机唤醒置位，block hook 经 BIT_WDOG 停机，run() 执行系统复位）
    pub wdog_req: Arc<WdogResetReq>,
    /// 数据访问 MEM hook 是否已安装（MPU 使能后才懒安装；见 [`Machine::install_data_access_hook`]）
    data_hook_installed: Arc<AtomicBool>,
    /// 初始 SP（向量表首字）
    pub initial_sp: u32,
    /// 复位向量（向量表第二字，含 Thumb 位处理见 [`Machine::reset`]）
    pub entry: u32,
    /// run() 外层循环迭代次数（探测 emu_start 是否频繁提前返回，纯调试用）
    run_iterations: std::cell::Cell<u64>,
    /// 退休指令计数（block hook 累加，run() 每轮据此递减 count 预算，保证正常终止）
    retired_insts: Arc<AtomicU64>,
    /// 上一次推进虚拟从设备时钟时的退休指令数（run() 起点按 Δretired 推进，
    /// 与 emu 段数解耦——修复：段数随中断风暴膨胀会把虚拟时间/推流速率自放大）
    last_virt_retired: std::cell::Cell<u64>,
    /// 异常入场计数（按向量号，诊中断暴风/唤醒停滞用；Switch 停机每进一次加 1）
    vec_entries: std::cell::RefCell<Vec<u64>>,
    /// ★SCB 句柄（§5.102）：`systick_ticks()` 改取 **SysTick 溢出次数** ✓
    /// （旧实现按「异常进入次数」✗ ⇒ 大块推进时溢出被合并 ⇒ 固件毫秒少记 ✗）。
    scb: Option<Arc<Mutex<SystemControl>>>,
    /// `run_ms` 的累计目标（固件 SysTick 拍数）：过冲跨调用携带，长期无漂移
    run_ms_target: u64,
    /// 最近一次异常抢占前的 PC（= 被中断块的 PC，诊在何处不停被抢）
    last_switch_pc: std::cell::Cell<u32>,
    /// 时间轴故障剧本（P1-1）：run() 按虚拟时间触发到期动作（NACK/丢帧/观察点）
    fault: Option<crate::fault::FaultScript>,
    /// UART 端口丢帧计数（port → 剩余丢弃帧数；u32::MAX = 持续丢弃）
    uart_drop: std::cell::RefCell<std::collections::HashMap<u8, u32>>,
    /// Halt 观察点：FaultAction::Halt 触发后 run() 提前返回
    halt_requested: std::cell::Cell<bool>,
    /// 遥测记录器（P2-1）：run() 段后按退休间隔采样观测点
    telemetry: Option<crate::telemetry::Telemetry>,
    /// GDB 指令级断点集（P2-3）：block hook 检查块起始地址命中
    gdb_breaks: Arc<std::sync::Mutex<std::collections::HashSet<u32>>>,
    /// GDB 断点命中标志（run() 段循环置位，GDB 继续命令消费）
    gdb_hit: std::cell::Cell<bool>,
    /// 软件断点原指令保存（地址 → 原字节；清除时恢复）
    sw_breaks: std::cell::RefCell<std::collections::HashMap<u32, Vec<u8>>>,
    /// GDB 精确断点 code hook（有断点时安装：逐指令检查 PC 命中；无断点时移除，
    /// 避免每指令回调开销）
    gdb_code_hook: std::cell::RefCell<Option<crate::core::UcHookId>>,
}

impl Machine {
    /// 创建 Cortex-M4F 机器
    pub fn new_m4f() -> Result<Self> {
        let cpu = Cpu::new_m4f()?;
        // 全局状态位域：Nvic/Mpu/WdogResetReq 共享，block hook 热路径一次 load
        let status = Arc::new(Status::new());
        let nvic = Arc::new(Mutex::new(Nvic::with_pending_any(status.clone())));
        let dma_active = Arc::new(AtomicBool::new(false));
        let dma = Arc::new(Mutex::new(Dma::with_active(
            nvic.clone(),
            "DMA1",
            DMA1_STREAM_IRQ,
            dma_active.clone(),
        )));
        let dma2_active = Arc::new(AtomicBool::new(false));
        let dma2 = Arc::new(Mutex::new(Dma::with_active(
            nvic.clone(),
            "DMA2",
            DMA2_STREAM_IRQ,
            dma2_active.clone(),
        )));
        let wdog_req = Arc::new(WdogResetReq::with_status(status.clone()));
        let events = Arc::new(Mutex::new(EventBus::new()));
        let pwr = Arc::new(Mutex::new(Pwr::new(wdog_req.clone())));
        let rtc_active = Arc::new(AtomicBool::new(false));
        let rtc = Arc::new(Mutex::new(Rtc::with_active(
            nvic.clone(),
            pwr.clone(),
            rtc_active.clone(),
        )));
        let iwdg_active = Arc::new(AtomicBool::new(false));
        let wwdg_active = Arc::new(AtomicBool::new(false));
        Ok(Self {
            cpu,
            bus: Arc::new(Mutex::new(Bus::new())),
            mpu: Arc::new(Mutex::new(Mpu::with_enabled(status.clone()))),
            nvic: nvic.clone(),
            events: events.clone(),
            console: Arc::new(Mutex::new(Console::new())),
            terminal: Arc::new(Mutex::new(Terminal::new(events.clone()))),
            clock: Arc::new(VirtualClock::new()),
            timers: Arc::new(Mutex::new(Vec::new())),
            tick_actives: Arc::new(Mutex::new(Vec::new())),
            status,
            dma,
            dma2,
            dma_active,
            dma2_active,
            rcc: Arc::new(Mutex::new(Rcc::new())),
            rng: Arc::new(Mutex::new(Rng::new(
                0x5EED_2026,
                nvic.clone(),
                RNG_IRQ,
            ))),
            pwr,
            rtc,
            dcmi: Arc::new(Mutex::new(Dcmi::new(nvic.clone(), DCMI_IRQ))),
            fsmc: Arc::new(Mutex::new(Fsmc::new())),
            sdio: Arc::new(Mutex::new(Sdio::new(events.clone(), nvic.clone()))),
            i2c: Arc::new(Mutex::new(Vec::new())),
            usart: Arc::new(Mutex::new(Vec::new())),
            spi: Arc::new(Mutex::new(Vec::new())),
            escs: Arc::new(Mutex::new(Vec::new())),
            can1: Arc::new(Mutex::new(Can::new(1, Some(events.clone()), nvic.clone()))),
            can2: Arc::new(Mutex::new(Can::new(2, Some(events.clone()), nvic.clone()))),
            can_nodes: Arc::new(Mutex::new(Vec::new())),
            st7789: Arc::new(Mutex::new(crate::peripheral::vperiph::fsmc::St7789::new())),
            can_pending: Arc::new(Mutex::new(Vec::new())),
            flash: Arc::new(Mutex::new(Flash::new())),
            usb_otg: Arc::new(Mutex::new(UsbOtg::new(
                Some(events.clone()),
                nvic.clone(),
            ))),
            iwdg: Arc::new(Mutex::new(Iwdg::with_active(
                wdog_req.clone(),
                iwdg_active.clone(),
            ))),
            iwdg_active,
            wwdg: Arc::new(Mutex::new(Wwdg::with_active(
                nvic.clone(),
                wdog_req.clone(),
                wwdg_active.clone(),
            ))),
            wwdg_active,
            rtc_active,
            wdog_req,
            data_hook_installed: Arc::new(AtomicBool::new(false)),
            initial_sp: 0,
            entry: 0,
            run_iterations: std::cell::Cell::new(0),
            retired_insts: Arc::new(AtomicU64::new(0)),
            last_virt_retired: std::cell::Cell::new(0),
            vec_entries: std::cell::RefCell::new(vec![0u64; 97]),
            scb: None,
            run_ms_target: 0,
            last_switch_pc: std::cell::Cell::new(0),
            fault: None,
            uart_drop: std::cell::RefCell::new(std::collections::HashMap::new()),
            halt_requested: std::cell::Cell::new(false),
            telemetry: None,
            gdb_breaks: Arc::new(std::sync::Mutex::new(std::collections::HashSet::new())),
            gdb_hit: std::cell::Cell::new(false),
            sw_breaks: std::cell::RefCell::new(std::collections::HashMap::new()),
            gdb_code_hook: std::cell::RefCell::new(None),
        })
    }

    /// 映射 STM32F407VET6 基础内存布局（FLASH + SRAM1/SRAM2 + CCM + SCB + T1 外设区）。
    /// M3 起由 DSL 配置驱动，此处为 M0/M1/M2 固化布局 + M3 T1 外设集。
    /// 注册 I2C 虚拟从设备（总线协议级直路由挂载）。
    ///
    /// `port` = I2C 端口（1/2/3）。多个从设备可挂同一总线（地址区分）。
    pub fn register_i2c_slave(&self, port: u8, slave: Box<dyn crate::peripheral::vperiph::VirtualI2cSlave>) {
        let idx = (port as usize).saturating_sub(1);
        if let Some(i) = self.i2c.lock().unwrap().get(idx) {
            i.lock().unwrap().register_slave(slave);
        }
    }

    /// 故障注入：按 7bit 地址对 I2C 从设备设置 NACK（模拟断线/无响应）。
    ///
    /// 返回是否命中从设备。固件侧表现为事务 AF → 传感器 healthy=false → FDIR 降级/安全。
    pub fn inject_i2c_nack(&self, port: u8, addr7: u8, nack: bool) -> bool {
        let idx = (port as usize).saturating_sub(1);
        if let Some(i) = self.i2c.lock().unwrap().get(idx) {
            let mut i = i.lock().unwrap();
            if let Some(sl) = i.slaves_mut().iter_mut().find(|s| s.addr7() == addr7) {
                sl.set_nack(nack);
                return true;
            }
        }
        false
    }

    /// 故障注入：按从设备名对 SPI 从设备设置故障态（模拟芯片断线/无响应——
    /// 故障态下对一切访问回 0xFF，WHO_AM_I 校验失败 → 固件 healthy=false）。
    ///
    /// 返回是否命中从设备。SPI 从设备需实现 `set_fault(bool)`（如 [`Bmi088`]）。
    pub fn inject_spi_fault(&self, port: u8, name: &str, on: bool) -> bool {
        let idx = (port as usize).saturating_sub(1);
        if let Some(s) = self.spi.lock().unwrap().get(idx) {
            let mut s = s.lock().unwrap();
            let mut hit = false;
            for sl in s.slaves_mut().iter_mut() {
                if sl.name() == name {
                    if let Some(f) = sl
                        .as_any_mut()
                        .and_then(|a| a.downcast_mut::<crate::peripheral::vperiph::spi::Bmi088>())
                    {
                        f.set_fault(on);
                        hit = true;
                    }
                }
            }
            return hit;
        }
        false
    }

    /// SD 卡映像写回指定文件（保存用途检查点：固件写块后落盘可跨 run 存活）。
    pub fn persist_sd_card(&self, path: &std::path::Path) {
        self.sdio.lock().unwrap().persist_to(path);
    }

    /// 触发全部 SPI 虚拟从设备持久化（保存用途器件把映像写回绑定文件）。
    ///
    /// 供测试/上层在关键检查点（如固件写入 flash 后）调用，证明数据可跨 run 存活。
    pub fn persist_spi_slaves(&self) {
        for s in self.spi.lock().unwrap().iter() {
            s.lock().unwrap().persist_slave();
        }
    }

    /// 注册 ESC 电调 + 无刷电机虚拟外设。
    ///
    /// ESC 是信号观测者（非总线从设备）：订阅 `TimPwm`（PWM 电调输入）与
    /// `GpioLevel`（DShot bit-bang 输入），内部按配置过滤端口/通道/引脚。
    /// 回调内只更新 ESC 状态、不发布事件（避免事件分发回调内二次 publish 死锁）。
    pub fn register_esc(&self, esc: Arc<Mutex<Box<dyn crate::peripheral::vperiph::esc::EscMotor>>>) {
        let esc2 = esc.clone();
        let esc3 = esc.clone();
        self.events.lock().unwrap().subscribe(Arc::new(Mutex::new(move |ev: &Event| {
            match ev {
                Event::TimPwm { port, channel, level, tick } => {
                    esc2.lock().unwrap().on_pwm(*port, *channel, *level, *tick);
                }
                Event::GpioLevel { port, pin, level, tick } => {
                    esc2.lock().unwrap().on_gpio(*port, *pin, *level, *tick);
                }
                _ => {}
            }
        })));
        let _ = esc3;
        self.escs.lock().unwrap().push(esc);
    }

    /// 注册 SPI 虚拟从设备（全双工直路由挂载；片选经 GPIO 事件转发）。
    ///
    /// `port` = SPI 端口（1/2/3）。固件用 GPIO 输出拉低 CS 选中从机（无硬件 NSS），
    /// 从机构造时需提供其 CS 引脚的 GPIO 坐标（如 BMI088 的 ACCEL_CS/GYRO_CS）。
    pub fn register_spi_slave(&self, port: u8, slave: Box<dyn crate::peripheral::vperiph::spi::VirtualSpiSlave>) {
        let idx = (port as usize).saturating_sub(1);
        if let Some(s) = self.spi.lock().unwrap().get(idx) {
            s.lock().unwrap().register_slave(slave);
        }
    }

    /// 推进所有虚拟从设备（仿真时间 `dt` 秒；Math 数据源步进 / UART 推流节拍）。
    pub fn step_virtual_slaves(&self, dt: f32) {
        for i in self.i2c.lock().unwrap().iter() {
            i.lock().unwrap().step_slaves(dt);
        }
        for s in self.spi.lock().unwrap().iter() {
            s.lock().unwrap().step_slave(dt);
        }
        for e in self.escs.lock().unwrap().iter() {
            e.lock().unwrap().step(dt);
        }
        for n in self.can_nodes.lock().unwrap().iter() {
            n.lock().unwrap().step(dt);
        }
        // flush CAN 虚拟节点响应帧（此处无 can 锁；feed_rx 只挂 IRQ 不发布）
        let frames: Vec<crate::peripheral::can::CanFrame> =
            std::mem::take(&mut *self.can_pending.lock().unwrap());
        for f in frames {
            match f.port {
                1 => self.can1.lock().unwrap().feed_rx(f),
                2 => self.can2.lock().unwrap().feed_rx(f),
                _ => {}
            }
        }
    }

    /// 显式挂载 ST7789 LCD 到 FSMC Bank1（窗口读写转发给 LCD 器件）。
    /// 默认不挂载：m13 FSMC 固件测试用 Bank1 后备缓冲；验收测试需 LCD 时调用。
    pub fn enable_st7789(&self) {
        self.fsmc
            .lock()
            .unwrap()
            .set_lcd(Some(self.st7789.clone()));
    }

    /// 注册 CAN 总线虚拟节点（按 ID 响应，回帧路由到本端口 CAN 接收 FIFO）。
    ///
    /// CAN 是广播总线：节点订阅 `CanFrame`（port 匹配），`on_frame` 匹配则
    /// 返回响应帧；回调内只 feed_rx（挂 IRQ 不发布事件，无重入死锁）。
    pub fn register_can_node(&self, port: u8, node: Box<dyn crate::peripheral::vperiph::can::CanNode>) {
        let node = Arc::new(Mutex::new(node));
        let nodes = self.can_nodes.clone();
        let pending = self.can_pending.clone();
        self.events.lock().unwrap().subscribe(Arc::new(Mutex::new(move |ev: &Event| {
            if let Event::CanFrame { frame } = ev {
                if frame.port != port {
                    return;
                }
                let reply = {
                    let guard = nodes.lock().unwrap();
                    let mut out: Option<crate::peripheral::can::CanFrame> = None;
                    for n in guard.iter() {
                        if let Some(mut r) = n.lock().unwrap().on_frame(frame) {
                            r.port = port; // 兜底：响应发到当前端口
                            out = Some(r);
                            break;
                        }
                    }
                    out
                };
                if let Some(r) = reply {
                    // 只入队：事件分发时本端口 CAN 已被 publish 方持锁，直接
                    // feed_rx 会锁重入死锁；run 主循环 flush 时再喂（无 can 锁）
                    pending.lock().unwrap().push(r);
                }
            }
        })));
        self.can_nodes.lock().unwrap().push(node);
    }

    /// 注册 UART 虚拟从设备（推流：GPS NMEA / SBUS）。
    pub fn register_uart_slave(&self, port: u8, slave: Box<dyn crate::peripheral::vperiph::uart::VirtualUartSlave>) {
        let idx = (port as usize).saturating_sub(1);
        if let Some(u) = self.usart.lock().unwrap().get(idx) {
            u.lock().unwrap().register_slave(slave);
        }
    }

    /// 注入 UART RX 字节（虚拟推流/测试直路由）：feed_rx + RX DMA 搬运路由。
    ///
    /// 与 Machine 的 `Event::UartRx` 订阅者同语义（DMA 模式 uart1/2 需在 feed_rx
    /// 后路由 DMA 搬运，固件才能读到字节），但直调避免事件分发开销。
    pub fn inject_uart_rx(&self, port: u8, byte: u8) {
        let idx = (port as usize).saturating_sub(1);
        let Some(u) = self.usart.lock().unwrap().get(idx).cloned() else {
            return;
        };
        let mut uu = u.lock().unwrap();
        uu.feed_rx_queued(byte);
        if uu.dma_rx_pending() {
            let (ctrl, stream, channel) = match port {
                1 => (self.dma2.clone(), 2, 4),
                2 => (self.dma.clone(), 5, 4),
                3 => (self.dma.clone(), 1, 4),
                4 => (self.dma.clone(), 2, 4),
                5 => (self.dma.clone(), 0, 4),
                6 => (self.dma2.clone(), 1, 5),
                _ => return,
            };
            ctrl.lock().unwrap().service_stream(
                stream,
                channel,
                DmaDir::PeriphToMem,
                crate::peripheral::dma::DmaTarget::Usart(port),
            );
        }
    }

    /// 推进所有 UART 虚拟从设备（`dt` 秒：推流节拍），字节直路由喂入。
    pub fn step_virtual_uart(&self, dt: f32) {
        // 先收集各端口推流字节（避免 usart 表/句柄锁与 inject 重入冲突）
        let mut feeds: Vec<(u8, Vec<u8>)> = Vec::new();
        {
            let uv = self.usart.lock().unwrap();
            let mut drop = self.uart_drop.borrow_mut();
            for u in uv.iter() {
                let mut uu = u.lock().unwrap();
                let port = uu.port;
                // 丢帧（P1-1 故障剧本）：该端口剩余丢弃帧数 > 0 → 跳过本轮推流
                let skipping = drop.get(&port).copied().unwrap_or(0) > 0;
                if skipping {
                    if let Some(f) = drop.get_mut(&port) {
                        if *f != u32::MAX {
                            *f -= 1; // 有限丢帧：逐帧递减
                        }
                    }
                    continue;
                }
                let bytes = uu.collect_slave_bytes(dt);
                if !bytes.is_empty() {
                    feeds.push((port, bytes));
                }
            }
        }
        for (port, bytes) in feeds {
            for b in &bytes {
                self.inject_uart_rx(port, *b);
            }
            // 帧结束（虚拟从设备一次推一帧）→ IDLE 中断 → 固件 flush ring 可读
            if let Some(u) = self
                .usart
                .lock()
                .unwrap()
                .get((port as usize).saturating_sub(1))
            {
                u.lock().unwrap().notify_frame_end();
            }
        }
    }

    /// 便捷装配：默认 UART 推流从设备。
    ///
    /// flyctrl real-sensors：gps 挂 uart1（USART2, port=2）、sbus 挂 uart2（USART3, port=3）。
    pub fn attach_default_uart_slaves(&self) {
        use crate::peripheral::vperiph::data_source::{StaticGps, StaticSbus};
        use crate::peripheral::vperiph::uart::{NmeaGps, Sbus};
        self.register_uart_slave(2, Box::new(NmeaGps::new(StaticGps::default())));
        self.register_uart_slave(3, Box::new(Sbus::new(StaticSbus::default())));
    }

    /// [HIL 虚拟外设直通] 用 fly_sim 共享状态装配 I2C 传感器（mpu6050/bmp280/qmc5883）。
    ///
    /// fly_sim 每步写 `FlySimState`（Arc<Mutex>），设备动态寄存器经 FlySimSource
    /// 即时读到；固件 real-sensors 驱动照常经 i2c0 读寄存器。磁力计无真值源，
    /// 用默认静态模型（不影响 EKF 姿态，mag 未融合）。
    ///
    /// **一致性不变量**：`FlySimState` 只能在两次 `run()` 之间写入（run 期间冻结），
    /// 见 `FlySimState` 文档与 `docs/virtual_direct_mode.md` §8.1。
    pub fn attach_flysim_sensors(&self, st: std::sync::Arc<std::sync::Mutex<crate::peripheral::vperiph::data_source::FlySimState>>) {
        use crate::peripheral::vperiph::data_source::{FlySimKind, FlySimSource};
        use crate::peripheral::vperiph::i2c::{bmp280, mpu6050, qmc5883};
        use crate::peripheral::vperiph::spi::default_bmi088;
        // IMU 主源为 BMI088（SPI3，ACCEL_CS=GPIOE_7、GYRO_CS=GPIOE_8）：与固件
        // real-sensors 的 ImuBmi088("bmi088") 对应——注意板级设备名 "spi2" 实际是
        // SPI3 外设（g_spi2），故从设备必须挂在 SPI 端口 3（x_drvtest 同此约定）。
        self.register_spi_slave(3, Box::new(default_bmi088((4, 7), (4, 8)).with_source(FlySimSource::new(st.clone(), FlySimKind::Imu))));
        // I2C 总线：固件 baro/mag 走 i2c2(I2C3)——I2C1 的 DMA1_Stream6 与 uart1(USART2)
        // TX 冲突（固件 dma_acquire 流级互斥），I2C3 用 Stream4/2 空闲。
        self.register_i2c_slave(3, Box::new(mpu6050(FlySimSource::new(st.clone(), FlySimKind::Imu))));
        self.register_i2c_slave(3, Box::new(bmp280(FlySimSource::new(st.clone(), FlySimKind::Baro))));
        // 磁力计用 FlySimSource(Mag)：世界系恒定地磁场随姿态旋转到机体（真机模型），
        // 取代 StaticMag 固定机体磁场（yaw 观测恒定 → 磁锚定拉回航向）。
        self.register_i2c_slave(3, Box::new(qmc5883(FlySimSource::new(st, FlySimKind::Mag))));
    }

    /// [HIL 虚拟外设直通] 用 fly_sim 共享状态装配 UART 推流（gps/sbus）。
    ///
    /// gps 挂 uart1（USART2, port=2）、sbus 挂 uart2（USART3, port=3），
    /// 与固件 real-sensors 的 GpsUblox("uart1")/RcSbus("uart2") 对应。
    pub fn attach_flysim_uart_slaves(&self, st: std::sync::Arc<std::sync::Mutex<crate::peripheral::vperiph::data_source::FlySimState>>) {
        use crate::peripheral::vperiph::data_source::{FlySimKind, FlySimSource};
        use crate::peripheral::vperiph::uart::{NmeaGps, Sbus};
        self.register_uart_slave(2, Box::new(NmeaGps::new(FlySimSource::new(st.clone(), FlySimKind::Gps))));
        self.register_uart_slave(3, Box::new(Sbus::new(FlySimSource::new(st.clone(), FlySimKind::Sbus))));
    }

    /// 便捷装配：默认传感器挂载——IMU=BMI088 挂 SPI3（port 3），
    /// baro/mag（mpu6050 保留挂载）挂 I2C3（port 3）。baro 高度基准默认
    /// 海平面（0m），与虚拟 GPS 高度（默认 alt=4.0）不一致时可用
    /// [`attach_default_sensors_with_baro_height`] 对齐基准。
    pub fn attach_default_sensors(&self) {
        self.attach_default_sensors_with_baro_height(0.0);
    }

    /// 同 [`attach_default_sensors`]，但指定气压计模拟高度（m，向上为正）。
    /// 虚拟 GPS 与 baro 高度基准一致时，EKF 高度收敛到该基准（而非被海平面
    /// 气压拉偏——历史观察：海平面 baro vs GPS alt=4m → EKF 收敛 0.17m）。
    pub fn attach_default_sensors_with_baro_height(&self, baro_height: f32) {
        use crate::peripheral::vperiph::data_source::{StaticBaro, StaticImu, StaticMag};
        use crate::peripheral::vperiph::i2c::{bmp280, mpu6050, qmc5883};
        use crate::peripheral::vperiph::spi::default_bmi088;
        self.register_spi_slave(3, Box::new(default_bmi088((4, 7), (4, 8)).with_source(StaticImu::default())));
        // I2C 从设备挂 I2C3（port 3）：见 attach_flysim_sensors 的 DMA 冲突说明
        self.register_i2c_slave(3, Box::new(mpu6050(StaticImu::default())));
        self.register_i2c_slave(3, Box::new(bmp280(StaticBaro::at_height(baro_height))));
        self.register_i2c_slave(3, Box::new(qmc5883(StaticMag::default())));
    }

    /// 装配总线事务嗅探器（调试平台 P0-1）：把同一嗅探器注入到全部已挂载
    /// I2C/SPI/USART 外设，并注入虚拟时钟作为时间戳来源。
    ///
    /// 调用时机：外设挂载完成后（`attach_peripherals` 之后）。之后调用者/测试持
    /// 同一 `Arc<Mutex<BusTrace>>` 读取事务日志（`drain` / `drain_formatted`）。
    /// 装配时间轴故障剧本（调试平台 P1-1）。
    ///
    /// run() 按虚拟时间（retired / VIRTUAL_INSNS_PER_SEC）触发到期事件：
    /// NACK 注入 / UART 丢帧 / Halt 观察点。`reset()` 后脚本保持（可重放）。
    pub fn attach_fault_script(&mut self, script: crate::fault::FaultScript) {
        log::info!(
            "[fault] 装配剧本 '{}'：{} 个事件",
            script.name,
            script.events().len()
        );
        self.fault = Some(script);
    }

    /// 当前故障剧本是否已全部触发（测试/调试断言）。
    pub fn fault_all_fired(&self) -> bool {
        self.fault.as_ref().map(|s| s.all_fired()).unwrap_or(true)
    }

    /// 当前是否处于 Halt 观察点（FaultAction::Halt 触发后为 true）。
    pub fn halted(&self) -> bool {
        self.halt_requested.get()
    }

    /// 清除 Halt 观察点（继续推进）。
    pub fn clear_halt(&self) {
        self.halt_requested.set(false);
    }

    /// 推进时间轴故障剧本（run() 每轮调用）：触发到期事件并落地动作。
    fn step_fault(&mut self) {
        let actions = match &mut self.fault {
            Some(script) => {
                let retired = self.retired_insts.load(Ordering::Relaxed);
                crate::fault::step_script(script, retired, &mut self.uart_drop.borrow_mut())
            }
            None => Vec::new(),
        };
        for a in actions {
            match a {
                crate::fault::FaultAction::I2cNack { port, addr7, on } => {
                    let ok = self.inject_i2c_nack(port, addr7, on);
                    log::info!("[fault] t={:.3}s I2C{port} NACK@{addr7:#04x} on={on} (applied={ok})",
                        self.virtual_sec());
                }
                crate::fault::FaultAction::UartDrop { port, frames } => {
                    log::info!("[fault] t={:.3}s UART{port} 丢帧开始（frames={frames}）",
                        self.virtual_sec());
                }
                crate::fault::FaultAction::UartResume { port } => {
                    log::info!("[fault] t={:.3}s UART{port} 推流恢复", self.virtual_sec());
                }
                crate::fault::FaultAction::Log { msg } => {
                    log::info!("[fault] t={:.3}s {msg}", self.virtual_sec());
                }
                crate::fault::FaultAction::Halt => {
                    self.halt_requested.set(true);
                    log::info!("[fault] t={:.3}s Halt 观察点（run 提前返回）", self.virtual_sec());
                }
            }
        }
    }

    /// 当前虚拟时间（秒，retired / VIRTUAL_INSNS_PER_SEC 口径，与推流时钟一致）。
    pub fn virtual_sec(&self) -> f32 {
        self.retired_insts.load(Ordering::Relaxed) as f32 / crate::sim::timing::VIRTUAL_INSNS_PER_SEC
    }

    /// 保存快照（调试平台 P2-2）：CPU 寄存器 + RAM + retired 虚拟时间。
    pub fn snapshot(&mut self) -> std::result::Result<crate::checkpoint::Snapshot, String> {
        let retired = self.retired_insts.load(Ordering::Relaxed);
        crate::checkpoint::snapshot(&mut self.cpu, retired)
    }

    /// 恢复快照（回滚到保存点）：寄存器 + RAM + retired（时间线连续续跑）。
    pub fn restore(&mut self, snap: &crate::checkpoint::Snapshot) -> std::result::Result<(), String> {
        // 恢复后清 Halt 观察点（快照点不处于 halt 状态）
        self.halt_requested.set(false);
        crate::checkpoint::restore(&mut self.cpu, snap, &self.retired_insts, &self.last_virt_retired)
    }

    /// GDB 设置软件断点（P2-3）：block hook 区间检查（主停机制）+ 把断点地址的
    /// Thumb 指令改写为 BKPT(0xBEBE)（GDB 插入后读内存验证软件断点生效；
    /// 若区间检查未截停，BKPT 落入 HardFault 由固件异常路径兜底）。
    pub fn gdb_set_break(&mut self, addr: u32) -> bool {
        let addr = addr & !1;
        self.gdb_breaks.lock().unwrap().insert(addr);
        // 保存原指令并写入 BKPT（仅 FLASH/RAM 可写区域；外设区忽略）
        let orig = match self.cpu.mem_read(addr as u64, 2) {
            Ok(b) if b.len() == 2 => b,
            _ => return false,
        };
        self.sw_break_save(addr, orig);
        let ok = self.cpu.mem_write(addr as u64, &[0xBE, 0xBE]).is_ok();
        self.ensure_gdb_code_hook();
        ok
    }
    /// 清除软件断点：恢复原指令。
    pub fn gdb_clear_break(&mut self, addr: u32) {
        let addr = addr & !1;
        self.gdb_breaks.lock().unwrap().remove(&addr);
        if let Some(orig) = self.sw_break_take(addr) {
            let _ = self.cpu.mem_write(addr as u64, &orig);
        }
        self.maybe_drop_gdb_code_hook();
    }
    /// 安装精确断点 code hook（幂等；逐指令检查 PC 命中，命中即停）。
    /// 断点调试期间性能下降（每指令一次回调），无断点时移除恢复全速。
    fn ensure_gdb_code_hook(&mut self) {
        if self.gdb_code_hook.borrow().is_some() {
            return;
        }
        let breaks = self.gdb_breaks.clone();
        let nvic = self.nvic.clone();
        let id = self
            .cpu
            .add_code_hook(0, u64::MAX, move |uc, addr, _size| {
                if breaks
                    .lock()
                    .unwrap()
                    .contains(&((addr as u32) & !1))
                {
                    nvic.lock()
                        .unwrap()
                        .set_stop_reason(StopReason::Breakpoint);
                    let _ = uc.emu_stop();
                }
            })
            .ok();
        *self.gdb_code_hook.borrow_mut() = id;
    }
    /// 断点集清空时移除 code hook（恢复全速）。
    fn maybe_drop_gdb_code_hook(&mut self) {
        if !self.gdb_breaks.lock().unwrap().is_empty() {
            return;
        }
        if let Some(id) = self.gdb_code_hook.borrow_mut().take() {
            let _ = self.cpu.remove_hook(id);
        }
    }
    /// 软件断点原指令保存表（地址 → 原 2 字节）。
    fn sw_break_save(&self, addr: u32, orig: Vec<u8>) {
        self.sw_breaks.borrow_mut().insert(addr, orig);
    }
    fn sw_break_take(&self, addr: u32) -> Option<Vec<u8>> {
        self.sw_breaks.borrow_mut().remove(&addr)
    }
    /// 断点命中标志（继续命令消费后复位）。
    pub fn gdb_break_hit(&self) -> bool {
        self.gdb_hit.get()
    }
    pub fn gdb_break_hit_take(&self) -> bool {
        self.gdb_hit.replace(false)
    }
    pub fn gdb_breaks_len(&self) -> usize {
        self.gdb_breaks.lock().unwrap().len()
    }

    /// 装配遥测记录器（调试平台 P2-1）：run() 段后按退休间隔采样观测点。
    pub fn attach_telemetry(&mut self, telemetry: crate::telemetry::Telemetry) {
        log::info!(
            "[telemetry] 装配 {} 个观测点，周期 {} 退休",
            telemetry.watches().len(),
            telemetry.period_retired()
        );
        self.telemetry = Some(telemetry);
    }

    /// 导出遥测 CSV（time_us + 各观测点列）。
    pub fn telemetry_csv(&self) -> String {
        match &self.telemetry {
            Some(t) => t.to_csv(),
            None => String::new(),
        }
    }

    /// 遥测采样行数（测试/调试断言）。
    pub fn telemetry_rows(&self) -> usize {
        self.telemetry.as_ref().map(|t| t.row_count()).unwrap_or(0)
    }

    /// 段后遥测采样（run() 内调用）。
    fn step_telemetry(&mut self) {
        if let Some(t) = &mut self.telemetry {
            let retired = self.retired_insts.load(Ordering::Relaxed);
            t.sample(&mut self.cpu, retired);
        }
    }

    pub fn attach_bus_trace(&self, trace: std::sync::Arc<std::sync::Mutex<crate::trace::BusTrace>>) {
        trace.lock().unwrap().set_retired(Some(self.retired_insts.clone()));
        for p in self.i2c.lock().unwrap().iter() {
            p.lock().unwrap().set_trace(Some(trace.clone()));
        }
        for p in self.spi.lock().unwrap().iter() {
            p.lock().unwrap().set_trace(Some(trace.clone()));
        }
        for p in self.usart.lock().unwrap().iter() {
            p.lock().unwrap().set_trace(Some(trace.clone()));
        }
    }

    pub fn map_stm32f407_layout(&mut self) -> Result<()> {        self.cpu.mem_map(0x0800_0000, 0x0008_0000, Prot::ALL)?; // FLASH 512KB
        self.cpu.mem_map(0x2000_0000, 0x0002_0000, Prot::ALL)?; // SRAM1+SRAM2 128KB
        // SRAM3 64KB（0x2002_0000..0x2002_FFFF）：固件链接脚本未使用，
        // 专用于 HIL 共享内存虚拟外设（fly_simulater <-> flyctrl 直连，无 USB）。
        self.cpu.mem_map(0x2002_0000, 0x0001_0000, Prot::ALL)?; // SRAM3
        self.cpu.mem_map(0x1000_0000, 0x0001_0000, Prot::ALL)?; // CCM SRAM 64KB
        self.cpu.mem_map(0x1FFF_0000, 0x0001_0000, Prot::ALL)?; // 系统存储区（电子签名/flash大小等）
        // F407 出厂校准字（系统存储区只读，固件 create 时直读）：
        //   VREFINT_CAL @0x1FFF7A10 = 0x03F8（1.21V 参考电压 12 位码，25°C）
        //   TS_CAL1 @0x1FFF7A2C = 0x0482（内部温度传感器 30°C 码）
        //   TS_CAL2 @0x1FFF7A2E = 0x05F6（内部温度传感器 110°C 码）
        // 代表性量产值：temp_sensor 驱动据此算温度，缺了会 cal1=cal2=0 → 0/0=NaN。
        self.cpu.mem_write(0x1FFF_7A10, &0x03F8u32.to_le_bytes())?;
        self.cpu.mem_write(0x1FFF_7A2C, &0x0482u16.to_le_bytes())?;
        self.cpu.mem_write(0x1FFF_7A2E, &0x05F6u16.to_le_bytes())?;
        // 系统控制空间（SCB/NVIC/SysTick/MPU，含 CPACR@0xE000ED88）。
        // 仍映射为普通内存避免读写异常，同时由 mem hook 转发到总线上的 SCB 外设。
        self.cpu.mem_map(0xE000_E000, 0x0000_1000, Prot::ALL)?;
        self.attach_system_control()?;
        // DWT 调试部件（0xE0001000，CYCCNT 周期计数）：jOS 用其测调度延迟。
        self.cpu.mem_map(0xE000_1000, 0x0000_1000, Prot::ALL)?;
        self.attach_dwt()?;
        // M3 T1 外设集：GPIOA-I + USART1-3 + TIM2 + RCC 存根
        self.attach_t1_peripherals()?;
        // 中断投递 hook 最后注册：此时 timers/tick_actives 已挂载完毕，
        // 可冻结活动标记列表（Arc<Vec>）→ block hook 快路径无需每块加锁。
        self.attach_interrupt_delivery()?;
        Ok(())
    }

    /// 挂载系统控制空间（SCB）与 MPU 访问控制。
    ///
    /// M1 演示 MMIO 完整链路：CPU 访问 0xE000E000..0xE000F000 →
    /// Unicorn mem hook → 内存总线 → SystemControl 外设。
    /// 读：hook 在 CPU 读取前把外设读值注入 RAM（Unicorn 的 MEM_READ 在读取前触发）；
    /// 写：hook 转发到总线，Unicorn 随后照常写 RAM，RAM 视图与总线保持一致。
    ///
    /// M2 在此基础上接入 MPU（默认全强制，保真优先）与中断投递：
    /// 1. MMIO 数据访问：SCB 转发前先过 MPU 检查；
    /// 2. RAM/Flash/CCM 数据访问：挂 MEM_READ|MEM_WRITE hook，MPU 使能后全强制；
    /// 3. 取指（执行）：挂全范围 code hook 做 XN 检查；
    /// 4. 中断投递：block hook 检查挂起中断抢占，intr hook 处理 EXC_RETURN 返回。
    pub fn attach_system_control(&mut self) -> Result<()> {
        const SCB_BASE: u64 = 0xE000_E000;
        const SCB_SIZE: u32 = 0x1000;

        // SCB 挂接共享 MPU 与 NVIC：MPU/NVIC 寄存器窗口由 SCB 委托
        let scb = Arc::new(Mutex::new(SystemControl::new_with_mpu_nvic(
            SCB_SIZE,
            self.mpu.clone(),
            self.nvic.clone(),
        )));
        let bus = self.bus.clone();
        bus.lock()
            .unwrap()
            .attach(SCB_BASE as u32, SCB_SIZE, "SCB", scb.clone())?;
        self.scb = Some(scb.clone());   // ★§5.102：留句柄以取溢出计数 ✓
        // SysTick 定时器随虚拟时钟推进：加入周期外设列表（block hook 按 active 标记
        // 跳过未激活外设；SysTick 活动标记由 CTRL.ENABLE 置位）
        self.timers.lock().unwrap().push(scb.clone());
        self.tick_actives
            .lock()
            .unwrap()
            .push(scb.lock().unwrap().active.clone());

        // 1) MMIO 入口：先 MPU 检查，再转发总线
        let bus2 = bus.clone();
        let mpu_mmio = self.mpu.clone();
        let status_scb = self.status.clone();
        self.cpu.add_mmio_hook(
            SCB_BASE,
            SCB_BASE + SCB_SIZE as u64,
            move |uc, ty, addr, size, value| {
                if let Some(access) = mem_type_to_access(ty) {
                    let fault = {
                        let m = mpu_mmio.lock().unwrap();
                        m.check(addr as u32, access, cpu_privileged(uc)).err()
                    };
                    if let Some(f) = fault {
                        fault_and_stop(uc, &mpu_mmio, f);
                        return true;
                    }
                }
                match ty {
                    MemType::READ => {
                        if let Ok(v) = bus2.lock().unwrap().read(addr as u32, size as u32) {
                            let _ = uc.mem_write(addr, &v.to_le_bytes()[..size]);
                        }
                    }
                    MemType::WRITE => {
                        let _ =
                            bus2.lock().unwrap().write(addr as u32, size as u32, value as u32);
                        // SysTick ENABLE(0xE000E010 bit0) 置位 → 置 BIT_ANY_ACTIVE，
                        // 使 block hook 的 tick 循环推进 SysTick（此前仅外设区写会置位，
                        // SysTick 从不计数）。
                        if addr == 0xE000_E010 && value as u32 & 1 != 0 {
                            status_scb.set(BIT_ANY_ACTIVE);
                        }
                    }
                    _ => {}
                }
                false // 放行：RAM 视图保持与总线一致
            },
        )?;

        // 2) RAM/Flash/CCM 数据访问入口（MPU 全强制）——懒安装：MPU 使能后才挂载
        //    （启动早期挂载会让每条内存访问走 hook helper 翻译，触发 Unicorn 2.1.5
        //    "新译 TB 首条 32 位指令副作用丢失"缺陷，见 [`Machine::install_data_access_hook`]）。
        //    由 block hook 检到 BIT_MPU 置位且未安装时经 run() 安装。

        // 3) 取指 XN 检查——并入 block hook（attach_interrupt_delivery，块级判定），
        //    不再注册每指令 code hook：每指令 FFI 会强制单指令翻译块，
        //    是纯计算负载的最大性能黑洞（见 bench_probe）。

        // 4) 中断投递入口（block 检查 + EXC_RETURN 拦截）——在全部外设挂载后注册
        //（见 map_stm32f407_layout），此处不再调用。

        log::info!("SCB+MPU+NVIC 已挂载：0x{SCB_BASE:08X} +0x{SCB_SIZE:X}");
        Ok(())
    }

    /// 挂载 DWT 调试部件（0xE0001000..0xE0002000）。
    ///
    /// CYCCNT 周期计数由虚拟时钟驱动（block hook 推进），供固件做调度延迟测量
    /// （jOS 的 `rtos_cycle_now`）。与 SCB 相同的 MMIO 转发链路：读注入 / 写转发，
    /// 无 MPU 检查（调试部件不在 MPU 管理的程序/数据区语义内）。
    pub fn attach_dwt(&mut self) -> Result<()> {
        const DWT_BASE: u64 = 0xE000_1000;
        const DWT_SIZE: u32 = 0x1000;

        let dwt = Arc::new(Mutex::new(Dwt::new()));
        let bus = self.bus.clone();
        bus.lock()
            .unwrap()
            .attach(DWT_BASE as u32, DWT_SIZE, "DWT", dwt.clone())?;
        // CYCCNT 随虚拟时钟推进：加入周期外设列表（tick 时按 active 标记跳过）
        self.timers.lock().unwrap().push(dwt.clone());
        self.tick_actives
            .lock()
            .unwrap()
            .push(dwt.lock().unwrap().active.clone());

        let bus2 = bus.clone();
        let status_dwt = self.status.clone();
        self.cpu.add_mmio_hook(
            DWT_BASE,
            DWT_BASE + DWT_SIZE as u64,
            move |uc, ty, addr, size, value| {
                match ty {
                    MemType::READ => {
                        if let Ok(v) = bus2.lock().unwrap().read(addr as u32, size as u32) {
                            let _ = uc.mem_write(addr, &v.to_le_bytes()[..size]);
                        }
                    }
                    MemType::WRITE => {
                        let _ =
                            bus2.lock().unwrap().write(addr as u32, size as u32, value as u32);
                        // DWT.CTRL(0xE0001000) bit0=CYCCNTENA → 置 BIT_ANY_ACTIVE，
                        // 使 CYCCNT 随虚拟时钟推进（rtos_cycle_now 调度延迟测量）。
                        if addr == 0xE000_1000 && value as u32 & 1 != 0 {
                            status_dwt.set(BIT_ANY_ACTIVE);
                        }
                    }
                    _ => {}
                }
                false // 放行：RAM 视图保持与总线一致
            },
        )?;

        log::info!("DWT 已挂载：0x{DWT_BASE:08X} +0x{DWT_SIZE:X}");
        Ok(())
    }

    /// 挂载外设集：GPIOA-I + USART1-6 + TIM2 + RCC 存根 + SYSCFG/EXTI + 虚拟 Console/Terminal。
    ///
    /// 外设区 0x40000000..0x40024000 通过 mem hook 转发到总线（MPU 检查 + 读注入），
    /// 与 SCB 窗口相同的 MMIO 链路。TIM2 加入时钟外设列表由 block hook 推进；
    /// EXTI 订阅 GPIO 电平事件作为外部输入（M4）。
    fn attach_t1_peripherals(&mut self) -> Result<()> {
        // 外设区整体映射（含 AHB1 GPIO/RCC/DMA、APB1 TIM2/USART2-3、APB2 USART1）
        let periph_base: u64 = 0x4000_0000;
        let periph_size: u64 = 0x40000; // 覆盖至 DMA2 区（0x40026400+0x400）
        self.cpu.mem_map(periph_base, periph_size, Prot::ALL)?;

        // 事件互联：USART TX → Console（默认连接，等价 connect uart.tx -> console.rx）
        let events = self.events.clone();
        let console = self.console.clone();
        {
            let c = console.clone();
            events.lock().unwrap().subscribe(Arc::new(Mutex::new(
                move |ev: &Event| {
                    if let Event::UartByte { byte, .. } = ev {
                        c.lock().unwrap().write_byte(*byte);
                    }
                },
            )));
        }

        // RCC（共享字段：看门狗复位时置 CSR 复位标志）
        let rcc = self.rcc.clone();
        self.bus.lock().unwrap().attach(0x4002_3800, 0x400, "RCC", rcc)?;

        // GPIOA-I（port 0..8；F407 共 9 个端口，基址 0x40020000 起每 0x400 一个）
        for port in 0..9u8 {
            let gpio = Arc::new(Mutex::new(Gpio::with_retired(
                port,
                events.clone(),
                self.retired_insts.clone(),
            )));
            let base = 0x4002_0000 + (port as u32) * 0x400;
            self.bus.lock().unwrap().attach(base, 0x400, format!("GPIO{}", (b'A' + port) as char), gpio)?;
        }

        // USART1-6（port 1..6；M5 串口仿真：接 NVIC IRQ37-39/52/53/71 + 订阅 UartRx 喂 RX）
        for (port, base, irq) in [
            (1u8, 0x4001_1000u32, USART1_IRQ),
            (2, 0x4000_4400, USART2_IRQ),
            (3, 0x4000_4800, USART3_IRQ),
            (4, 0x4000_4C00, UART4_IRQ),
            (5, 0x4000_5000, UART5_IRQ),
            (6, 0x4001_1400, USART6_IRQ),
        ] {
            let uart = Arc::new(Mutex::new(Usart::new(port, events.clone(), self.nvic.clone(), irq)));
            self.bus
                .lock()
                .unwrap()
                .attach(base, 0x400, format!("USART{port}"), uart.clone())?;
            // 注册 USART 句柄到 DMA1/DMA2（外设方向搬运经句柄直接读写 DR）
            self.dma.lock().unwrap().register_usart(port, uart.clone());
            self.dma2.lock().unwrap().register_usart(port, uart.clone());
            self.usart.lock().unwrap().push(uart.clone());
            // 虚拟终端/测试发布 UartRx → 对应端口 feed_rx；
            // RX DMA 请求在 feed_rx 之后直接路由（不能在 feed_rx 内二次 publish，
            // 否则事件分发回调中同线程重入 events.lock() 死锁，见 [`Usart::dma_rx_pending`]）
            let u = uart.clone();
            let dma1 = self.dma.clone();
            let dma2 = self.dma2.clone();
            events.lock().unwrap().subscribe(Arc::new(Mutex::new(
                move |ev: &Event| {
                    if let Event::UartRx { port: p, byte } = ev {
                        if *p == u.lock().unwrap().port {
                            u.lock().unwrap().feed_rx(*byte);
                            if u.lock().unwrap().dma_rx_pending() {
                                // 外设→内存：DMAR 使能且 RXNE 置位 → 登记 DMA 搬运
                                //（映射与 UartDma 订阅一致，F407 HAL 默认流）
                                let (ctrl, stream, channel) = match *p {
                                    1 => (dma2.clone(), 2, 4),
                                    2 => (dma1.clone(), 5, 4),
                                    3 => (dma1.clone(), 1, 4),
                                    4 => (dma1.clone(), 2, 4),
                                    5 => (dma1.clone(), 0, 4),
                                    6 => (dma2.clone(), 1, 5),
                                    _ => return,
                                };
                                ctrl.lock().unwrap().service_stream(stream, channel, DmaDir::PeriphToMem, crate::peripheral::dma::DmaTarget::Usart(*p));
                            }
                        }
                    }
                },
            )));
        }

        // I2C1-3（port 1..3；DMA 模式：接 NVIC EV IRQ + 订阅 I2cRx 喂 RX）
        for (port, base, irq_ev) in [
            (1u8, 0x4000_5400u32, I2C1_EV_IRQ),
            (2, 0x4000_5800, I2C2_EV_IRQ),
            (3, 0x4000_5C00, I2C3_EV_IRQ),
        ] {
            let i2c = Arc::new(Mutex::new(I2c::new(port, events.clone(), self.nvic.clone(), irq_ev)));
            self.bus
                .lock()
                .unwrap()
                .attach(base, 0x400, format!("I2C{port}"), i2c.clone())?;
            // 注册 I2C 句柄到 DMA1（I2C DMA 全在 DMA1，外设方向搬运经句柄直接读写 DR）
            self.dma.lock().unwrap().register_i2c(port, i2c.clone());
            self.i2c.lock().unwrap().push(i2c.clone());
            // 测试/虚拟从机发布 I2cRx → 对应端口 feed_rx；
            // RX DMA 请求在 feed_rx 之后直接路由（不能在 feed_rx 内二次 publish，
            // 否则事件分发回调中同线程重入 events.lock() 死锁，见 [`I2c::dma_rx_pending`]）
            let i = i2c.clone();
            let dma1 = self.dma.clone();
            events.lock().unwrap().subscribe(Arc::new(Mutex::new(
                move |ev: &Event| {
                    if let Event::I2cRx { port: p, byte } = ev {
                        if *p == i.lock().unwrap().port {
                            i.lock().unwrap().feed_rx(*byte);
                            if i.lock().unwrap().dma_rx_pending() {
                                // 外设→内存：DMAEN 使能且 RxNE 置位 → 登记 DMA 搬运
                                //（映射与 I2cDma 订阅一致，F407 HAL 默认流）
                                let (stream, channel) = match *p {
                                    1 => (0, 1), // I2C1_RX: DMA1_Stream0_Channel1
                                    2 => (3, 7), // I2C2_RX: DMA1_Stream3_Channel7
                                    3 => (2, 3), // I2C3_RX: DMA1_Stream2_Channel3
                                    _ => return,
                                };
                                dma1.lock()
                                    .unwrap()
                                    .service_stream(stream, channel, DmaDir::PeriphToMem, crate::peripheral::dma::DmaTarget::I2c(*p));
                            }
                        }
                    }
                },
            )));
        }

        // SPI1-3（port 1..3；DMA 模式：接 NVIC IRQ + 订阅 SpiRx 喂 RX。
        // SPI1 DMA 在 DMA2、SPI2/3 在 DMA1，见 F407 请求映射）
        for (port, base, irq, dma_ctrl) in [
            (1u8, 0x4001_3000u32, SPI1_IRQ, true),  // SPI1 → DMA2
            (2, 0x4000_3800, SPI2_IRQ, false),      // SPI2 → DMA1
            (3, 0x4000_3C00, SPI3_IRQ, false),      // SPI3 → DMA1
        ] {
            let spi = Arc::new(Mutex::new(Spi::new(port, events.clone(), self.nvic.clone(), irq)));
            self.spi.lock().unwrap().push(spi.clone());
            self.bus
                .lock()
                .unwrap()
                .attach(base, 0x400, format!("SPI{port}"), spi.clone())?;
            // 注册 SPI 句柄到对应 DMA 控制器（外设方向搬运经句柄直接读写 DR）
            let reg_ctrl = if dma_ctrl { self.dma2.clone() } else { self.dma.clone() };
            reg_ctrl.lock().unwrap().register_spi(port, spi.clone());
            // 测试/虚拟从机发布 SpiRx → 对应端口 feed_rx；
            // RX DMA 请求在 feed_rx 之后直接路由（不能在 feed_rx 内二次 publish，
            // 否则事件分发回调中同线程重入 events.lock() 死锁，见 [`Spi::dma_rx_pending`]）
            let i = spi.clone();
            let ctrl = if dma_ctrl { self.dma2.clone() } else { self.dma.clone() };
            events.lock().unwrap().subscribe(Arc::new(Mutex::new(
                move |ev: &Event| {
                    if let Event::SpiRx { port: p, byte } = ev {
                        if *p == i.lock().unwrap().port {
                            i.lock().unwrap().feed_rx(*byte);
                            if i.lock().unwrap().dma_rx_pending() {
                                // 外设→内存：RXDMAEN 使能且 RXNE 置位 → 登记 DMA 搬运
                                //（映射与 SpiDma 订阅一致，F407 HAL 默认流）
                                let (stream, channel) = match *p {
                                    1 => (0, 3), // SPI1_RX: DMA2_Stream0_Channel3
                                    2 => (3, 0), // SPI2_RX: DMA1_Stream3_Channel0
                                    3 => (0, 0), // SPI3_RX: DMA1_Stream0_Channel0
                                    _ => return,
                                };
                                ctrl.lock()
                                    .unwrap()
                                    .service_stream(stream, channel, DmaDir::PeriphToMem, crate::peripheral::dma::DmaTarget::Spi(*p));
                            }
                        }
                    }
                },
            )));
        }

        // SPI 虚拟从机片选转发：固件用 GPIO 输出拉低 CS（无硬件 NSS）→ GpioLevel
        // 事件 → 各 SPI 从机 on_cs（从机自行过滤关注的引脚；拉低开始新帧）。
        // 与 SpiRx 订阅同上下文（事件分发回调内只调控制器方法，不二次 publish）。
        {
            let spi_vec = self.spi.clone();
            events.lock().unwrap().subscribe(Arc::new(Mutex::new(
                move |ev: &Event| {
                    if let Event::GpioLevel { port, pin, level, .. } = ev {
                        for s in spi_vec.lock().unwrap().iter() {
                            s.lock().unwrap().route_cs(*port, *pin, *level);
                        }
                    }
                },
            )));
        }

        // ADC1-3（port 1..3；DMA 模式：接 NVIC IRQ18（共享）+ 订阅 AdcValue 喂采样值。
        // ADC DMA 全在 DMA2，见 F407 请求映射：ADC1→Stream0_Ch0、ADC2→Stream2_Ch1、
        // ADC3→Stream1_Ch2）
        for (port, base) in [
            (1u8, 0x4001_2000u32), // ADC1
            (2, 0x4001_2100),      // ADC2
            (3, 0x4001_2200),      // ADC3
        ] {
            let adc = Arc::new(Mutex::new(Adc::new(port, events.clone(), self.nvic.clone(), ADC_IRQ)));
            // 注意 size=0x100：ADC1/2/3 基址相邻仅差 0x100（0x40012000/0x40012100/0x40012200），
            // 用 0x400 会与相邻 ADC 区间重叠（寄存器仅到 DR@0x4C，0x100 足够）
            self.bus
                .lock()
                .unwrap()
                .attach(base, 0x100, format!("ADC{port}"), adc.clone())?;
            // 注册 ADC 句柄到 DMA2（外设→内存搬运经句柄直接读 DR）
            self.dma2.lock().unwrap().register_adc(port, adc.clone());
            // 测试/虚拟传感器发布 AdcValue → 对应端口 feed_value；
            // DMA 请求在 feed_value 之后直接路由（不能在 feed_value 内二次 publish，
            // 否则事件分发回调中同线程重入 events.lock() 死锁，见 [`Adc::dma_pending`]）
            let i = adc.clone();
            let dma2 = self.dma2.clone();
            events.lock().unwrap().subscribe(Arc::new(Mutex::new(
                move |ev: &Event| {
                    if let Event::AdcValue { port: p, value, .. } = ev {
                        if *p == i.lock().unwrap().port {
                            i.lock().unwrap().feed_value(*value);
                            if i.lock().unwrap().dma_pending() {
                                // 外设→内存：CR2.DMA 使能且 EOC 置位 → 登记 DMA 搬运
                                //（HAL 默认流，全在 DMA2）
                                let (stream, channel) = match *p {
                                    1 => (0, 0), // ADC1: DMA2_Stream0_Channel0
                                    2 => (2, 1), // ADC2: DMA2_Stream2_Channel1
                                    3 => (1, 2), // ADC3: DMA2_Stream1_Channel2
                                    _ => return,
                                };
                                dma2.lock()
                                    .unwrap()
                                    .service_stream(stream, channel, DmaDir::PeriphToMem, crate::peripheral::dma::DmaTarget::Adc(*p));
                            }
                        }
                    }
                },
            )));
        }

        // DAC1（STM32F407 唯一 DAC，2 通道 12 位，@0x40007400）。
        // 触发源：软件（SWTRIGR，TSEL=7）+ 定时器（TSEL=0..5，订阅 TimUpdate 路由）；
        // 触发转换 → DacLevel 事件发布（虚拟示波器/测试订阅），CR.DMAENx 置位时发布
        // DacDma 请求内存→外设搬运（DMA 写 DHR12Rx 再转换）。DMA 映射（RM0090）：
        // DAC1_CH1 → DMA1_Stream5_Channel7、DAC1_CH2 → DMA1_Stream6_Channel7。
        let dac_active = Arc::new(AtomicBool::new(false));
        let dac = Arc::new(Mutex::new(Dac::with_active(1, events.clone(), dac_active.clone())));
        self.bus
            .lock()
            .unwrap()
            .attach(0x4000_7400, 0x400, "DAC", dac.clone())?;
        // 注册 DAC 句柄到 DMA1（内存→外设搬运经句柄直接写 DHR）
        self.dma.lock().unwrap().register_dac(1, dac.clone());
        // 推入时钟外设列表：DAC 无计数语义，tick 仅冲刷定时器触发暂存的电平事件
        self.timers.lock().unwrap().push(dac.clone());
        self.tick_actives.lock().unwrap().push(dac_active);

        // DAC 触发 → DMA 请求（内存→外设：DMA 写 DHR 再转换）
        let dma1_for_dac = self.dma.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::DacDma { port: p, channel, .. } = ev {
                    if *p != 1 {
                        return;
                    }
                    let (stream, channel) = match *channel {
                        1 => (5, 7), // DAC1_CH1: DMA1_Stream5_Channel7
                        2 => (6, 7), // DAC1_CH2: DMA1_Stream6_Channel7
                        _ => return,
                    };
                    dma1_for_dac.lock().unwrap().service_stream(
                        stream,
                        channel,
                        DmaDir::MemToPeriph,
                        crate::peripheral::dma::DmaTarget::Dac(*p),
                    );
                }
            },
        )));

        // DAC 定时器触发：订阅 TimUpdate，TSEL 匹配通道锁存 DHR→DOR 并发起 DMA 请求
        //（事件分发回调内不发布事件——二次 publish 死锁，电平由 DAC::tick 冲刷发布；
        //  DMA 请求在 timer_trigger 之后对锁存通道直接路由，见 [`Dac::dma_requested`]）
        let dac_for_tim = dac.clone();
        let dma1_for_tim = self.dma.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::TimUpdate { port } = ev {
                    let latched = dac_for_tim.lock().unwrap().timer_trigger(*port);
                    for ch in 1..=2u8 {
                        if latched & (1 << (ch - 1)) != 0 && dac_for_tim.lock().unwrap().dma_requested(ch) {
                            let (stream, channel) = if ch == 1 { (5, 7) } else { (6, 7) };
                            dma1_for_tim.lock().unwrap().service_stream(
                                stream,
                                channel,
                                DmaDir::MemToPeriph,
                                crate::peripheral::dma::DmaTarget::Dac(1),
                            );
                        }
                    }
                }
            },
        )));

        // M8-CRC 计算单元（@0x40023000，AHB1）。
        // 32 位 CRC-32/MPEG-2 风格：DR 写数据（8/16/32 位）按 MSB 先推进计算、读 DR 返回
        // 当前值；CR.RESET 写 1 复位计算单元（回 0xFFFFFFFF）；IDR 为不影响计算的独立
        // 数据寄存器。无时钟门控（F407 RCC 无 CRCEN 位，始终使能）、无中断/DMA/tick。
        let crc = Arc::new(Mutex::new(Crc::new()));
        self.bus.lock().unwrap().attach(0x4002_3000, 0x100, "CRC", crc.clone())?;

        // M9-RNG 真随机数发生器（@0x50060800，AHB2）。
        // CR.RNGEN 使能 → SR.DRDY 置位、读 DR 返回随机值并连续生成；CECS/SECS 错误
        // 经 [`Rng::inject_*`] 注入（IRQ80 错误中断）。外设区 MMIO hook 只覆盖
        // 0x40000000..0x40040000，RNG 位于 AHB2 需单独映射 + 转发 hook（同一 MPU 链路）。
        let rng = self.rng.clone();
        self.bus.lock().unwrap().attach(0x5006_0800, 0x100, "RNG", rng)?;
        {
            let rng_base: u64 = 0x5006_0800;
            // Unicorn mem_map 需页对齐：映射 RNG 所在 4KB 页，hook 精确到寄存器窗口
            self.cpu.mem_map(0x5006_0000, 0x1000, Prot::ALL)?;
            let bus = self.bus.clone();
            let mpu = self.mpu.clone();
            self.cpu.add_mmio_hook(rng_base, rng_base + 0x100, move |uc, ty, addr, size, value| {
                let fault = {
                    let m = mpu.lock().unwrap();
                    match mem_type_to_access(ty) {
                        Some(access) => m.check(addr as u32, access, cpu_privileged(uc)).err(),
                        None => None,
                    }
                };
                if let Some(f) = fault {
                    fault_and_stop(uc, &mpu, f);
                    return true;
                }
                match ty {
                    MemType::READ => {
                        if let Ok(v) = bus.lock().unwrap().read(addr as u32, size as u32) {
                            let _ = uc.mem_write(addr, &v.to_le_bytes()[..size]);
                        }
                    }
                    MemType::WRITE => {
                        let _ = bus.lock().unwrap().write(addr as u32, size as u32, value as u32);
                    }
                    _ => {}
                }
                false
            })?;
        }

        // M12-DCMI 数字摄像头接口（@0x50050000，AHB2）。
        // CR.ENABLE+CAPTURE 使能捕获；测试/虚拟摄像头发布 DcmiFrame 注入一帧像素 →
        // feed_frame 拆 32 位字入 FIFO（SR.FNE/FRAME + RIS + IRQ78 帧完成中断）；
        // DMA 模式：DMA2_Stream1_Channel1 外设→内存搬运（RM0090 请求映射）。
        // 与 RNG 相同：AHB2 不在外设区 hook 覆盖内，需单独页映射 + 转发 hook（同 MPU 链路）。
        let dcmi = self.dcmi.clone();
        self.bus
            .lock()
            .unwrap()
            .attach(0x5005_0000, 0x40, "DCMI", dcmi.clone())?;
        // 注册 DCMI 句柄到 DMA2（外设→内存搬运经句柄直接读 DR）
        self.dma2.lock().unwrap().register_dcmi(1, dcmi.clone());
        {
            let dcmi_base: u64 = 0x5005_0000;
            // Unicorn mem_map 需页对齐：映射 DCMI 所在 4KB 页，hook 精确到寄存器窗口
            self.cpu.mem_map(0x5005_0000, 0x1000, Prot::ALL)?;
            let bus = self.bus.clone();
            let mpu = self.mpu.clone();
            self.cpu.add_mmio_hook(dcmi_base, dcmi_base + 0x40, move |uc, ty, addr, size, value| {
                let fault = {
                    let m = mpu.lock().unwrap();
                    match mem_type_to_access(ty) {
                        Some(access) => m.check(addr as u32, access, cpu_privileged(uc)).err(),
                        None => None,
                    }
                };
                if let Some(f) = fault {
                    fault_and_stop(uc, &mpu, f);
                    return true;
                }
                match ty {
                    MemType::READ => {
                        if let Ok(v) = bus.lock().unwrap().read(addr as u32, size as u32) {
                            let _ = uc.mem_write(addr, &v.to_le_bytes()[..size]);
                        }
                    }
                    MemType::WRITE => {
                        let _ = bus.lock().unwrap().write(addr as u32, size as u32, value as u32);
                    }
                    _ => {}
                }
                false
            })?;
        }
        // DCMI 帧事件 → 注入 + DMA 请求路由。发布者（测试/虚拟摄像头）在事件分发
        // 回调内不能再 publish（二次 publish 死锁）；DMA 请求在 feed_frame 之后
        // 直接路由（DMA2_Stream1_Channel1，外设→内存，一次搬完整帧字）
        let dcmi_ev = dcmi.clone();
        let dma2_dcmi = self.dma2.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::DcmiFrame { port: p, data } = ev {
                    if *p != 1 {
                        return;
                    }
                    let words = dcmi_ev.lock().unwrap().feed_frame(data);
                    if words > 0 {
                        dma2_dcmi.lock().unwrap().service_stream_n(
                            1,
                            1,
                            DmaDir::PeriphToMem,
                            crate::peripheral::dma::DmaTarget::Dcmi(*p),
                            words,
                        );
                    }
                }
            },
        )));

        // M13-FSMC 外部存储器控制器（@0xA0000000，AHB3/AHB1 域）。
        // 寄存器文件（BCR1-4/BTR1-4/BWTR1-4）挂在总线上；Bank1-4 片选窗口
        // （@0x60000000/0x64000000/0x68000000/0x6C000000，各 64KB 简化映射）经
        // 页映射 + hook 转发到 Fsmc::window_read/write，BCRn.MBKEN 使能才命中。
        // 寄存器块与窗口均位于外设区 hook（0x40000000）与数据区 hook 之外，
        // 与 RNG/DCMI 相同：单独页映射 + 转发 hook。
        let fsmc = self.fsmc.clone();
        self.bus
            .lock()
            .unwrap()
            .attach(FSMC_BASE, 0x200, "FSMC", fsmc.clone())?;
        // Bank1 默认不挂载 LCD（保持后备缓冲语义，m13 FSMC 固件测试依赖）；
        // 需 LCD 时由上层显式调用 [`Machine::enable_st7789`]（如 x_drvtest）。
        let _ = fsmc;
        {
            // 寄存器块：映射 1 页，hook 精确到寄存器窗口
            self.cpu.mem_map(FSMC_BASE as u64, 0x1000, Prot::ALL)?;
            let bus = self.bus.clone();
            let mpu = self.mpu.clone();
            self.cpu.add_mmio_hook(
                FSMC_BASE as u64,
                FSMC_BASE as u64 + 0x200,
                move |uc, ty, addr, size, value| {
                    let fault = {
                        let m = mpu.lock().unwrap();
                        match mem_type_to_access(ty) {
                            Some(access) => m.check(addr as u32, access, cpu_privileged(uc)).err(),
                            None => None,
                        }
                    };
                    if let Some(f) = fault {
                        fault_and_stop(uc, &mpu, f);
                        return true;
                    }
                    match ty {
                        MemType::READ => {
                            if let Ok(v) = bus.lock().unwrap().read(addr as u32, size as u32) {
                                let _ = uc.mem_write(addr, &v.to_le_bytes()[..size]);
                            }
                        }
                        MemType::WRITE => {
                            let _ = bus.lock().unwrap().write(addr as u32, size as u32, value as u32);
                        }
                        _ => {}
                    }
                    false
                },
            )?;
            // 4 个片选窗口：映射 + hook 转发（MBKEN 门控由 Fsmc 内部处理）
            for (base, size) in [
                (FSMC_BANK1_BASE, FSMC_BANK_SIZE),
                (FSMC_BANK2_BASE, FSMC_BANK_SIZE),
                (FSMC_BANK3_BASE, FSMC_BANK_SIZE),
                (FSMC_BANK4_BASE, FSMC_BANK_SIZE),
            ] {
                self.cpu.mem_map(base as u64, size as u64, Prot::ALL)?;
                let fsmc_win = fsmc.clone();
                let mpu = self.mpu.clone();
                let (b, e) = (base as u64, base as u64 + size as u64);
                self.cpu.add_mmio_hook(b, e, move |uc, ty, addr, size, value| {
                    let fault = {
                        let m = mpu.lock().unwrap();
                        match mem_type_to_access(ty) {
                            Some(access) => m.check(addr as u32, access, cpu_privileged(uc)).err(),
                            None => None,
                        }
                    };
                    if let Some(f) = fault {
                        fault_and_stop(uc, &mpu, f);
                        return true;
                    }
                    match ty {
                        MemType::READ => {
                            let v =
                                fsmc_win.lock().unwrap().window_read(addr as u32, size as u32);
                            let _ = uc.mem_write(addr, &v.to_le_bytes()[..size]);
                        }
                        MemType::WRITE => {
                            fsmc_win
                                .lock()
                                .unwrap()
                                .window_write(addr as u32, size as u32, value as u32);
                        }
                        _ => {}
                    }
                    false
                })?;
            }
        }

        // M14-SDIO 安全数字 IO（@0x40012C00，APB2 区；命令/响应 + FIFO + DMA2 + IRQ49）。
        // 命令路径：写 CMD（CPSMEN）→ 虚拟 SD 卡按 index 返回 RESP1-4/STATUS 标志；
        // 数据路径：DCTRL.DTEN 使能后读方向填充 FIFO、写方向由 DMA 推入 FIFO，
        // DMAEN 时发布 SdioDma → DMA2 按方向路由（RX=DMA2_Stream3_Channel4、
        // TX=DMA2_Stream6_Channel4，RM0090 请求映射）搬运外设↔内存。
        let sdio = self.sdio.clone();
        self.bus
            .lock()
            .unwrap()
            .attach(SDIO_BASE, 0x400, "SDIO", sdio.clone())?;
        // 注册 SDIO 句柄到 DMA2（外设↔内存搬运经句柄直接读写 FIFO）
        self.dma2.lock().unwrap().register_sdio(1, sdio.clone());
        // SDIO DMA 事件 → 按方向路由到 DMA2 对应流
        let dma2_sdio = self.dma2.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::SdioDma { port: p, dir, items } = ev {
                    if *p != 1 {
                        return;
                    }
                    let mut d = dma2_sdio.lock().unwrap();
                    // SDIO 单请求线：写恒走 DMA2_Stream6_Ch4（固件 joc-base 与
                    // m14 sdio_demo 一致）；读方向先按 RM0090 RX 流（DMA2_Stream3
                    // _Ch4，m14 固件语义），未配置（joc-base 驱动恒 acquire S6）
                    // 时 fallback 到 S6——两种固件行为均服务。
                    match dir {
                        // 读：FIFO 现成字数一次搬完
                        DmaDir::PeriphToMem => {
                            let served = d.service_stream_n(
                                3,
                                4,
                                *dir,
                                crate::peripheral::dma::DmaTarget::Sdio(*p),
                                *items,
                            );
                            if !served {
                                d.service_stream_n(
                                    6,
                                    4,
                                    *dir,
                                    crate::peripheral::dma::DmaTarget::Sdio(*p),
                                    *items,
                                );
                            }
                        }
                        // 写：items=0，由 service_stream 取 NDTR 一次搬完
                        DmaDir::MemToPeriph => d.service_stream(
                            6,
                            4,
                            *dir,
                            crate::peripheral::dma::DmaTarget::Sdio(*p),
                        ),
                    }
                }
            },
        )));

        // M15-CAN1/2 控制器局域网（@0x40006400/@0x40006800，APB1 区）。
        // bxCAN 简化：3 发送邮箱 + 2 接收 FIFO（3 槽）+ 28 滤波器（列表模式）+ 错误管理。
        // 总线级互联：发送方写 TIxR（TXRQ）→ 发布 CanFrame 事件 → 路由到对端 feed_rx
        //（过滤通过入 FIFO0、FMP 递增、FMPIE0 使能时挂起 RX IRQ）。CAN1 挂起
        // IRQ19/20/22（TX/RX0/SCE），CAN2 挂起 IRQ63/64/66。位于外设区 hook 覆盖内，
        // 直接 attach 即可（无需单独页映射）。
        let can1 = self.can1.clone();
        self.bus.lock().unwrap().attach(CAN1_BASE, 0x400, "CAN1", can1)?;
        let can2 = self.can2.clone();
        self.bus.lock().unwrap().attach(CAN2_BASE, 0x400, "CAN2", can2)?;
        // FLASH 控制器寄存器区（@0x40023C00，AHB1，外设区 hook 覆盖内直接 attach）。
        let flash = self.flash.clone();
        self.bus
            .lock()
            .unwrap()
            .attach(0x4002_3C00, 0x100, "FLASH", flash)?;
        // CanFrame 事件 → 路由到对端 CAN（CAN1↔CAN2 互联；feed_rx 只挂 IRQ 不发布，
        // 无事件重入死锁风险）
        let c1 = self.can1.clone();
        let c2 = self.can2.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::CanFrame { frame } = ev {
                    match frame.port {
                        // 总线级互联：对端接收（CAN1↔CAN2）。发送端回环（LBKM）
                        // 在 Can::transmit 内自行 feed_rx，不经此处（避免锁重入）。
                        1 => c2.lock().unwrap().feed_rx((**frame).clone()),
                        2 => c1.lock().unwrap().feed_rx((**frame).clone()),
                        _ => {}
                    }
                }
            },
        )));

        // M16-USB OTG FS 全速设备控制器（@0x50000000，AHB1 区）。
        // 设备模式简化：全局寄存器 + 4 IN/4 OUT 端点 + 数据 FIFO + 枚举。
        // 虚拟主机侧经 inject_usb_reset/inject_setup/inject_out 注入（总线复位/
        // SETUP 包/OUT 数据 → 接收 FIFO + GRXSTSP + DOEPINT.STUP/XFRC + RXFLVL），
        // 固件写 DFIFOx+DIEPCTL.EPENA 完成 IN（DIEPINT.XFRC）。挂起 OTG_FS_IRQ=67
        //（GINTMSK + DAINTMSK×DIEPMSK/DOEPMSK 门控）。USB 区位于外设区
        // 0x40000000..0x40040000 之外（AHB1 0x50000000），需单独映射 + 转发 hook。
        let usb = self.usb_otg.clone();
        self.bus
            .lock()
            .unwrap()
            .attach(USB_OTG_FS_BASE, 0x5000, "USB_OTG_FS", usb)?;
        // USB OTG FS 位于 AHB1 0x50000000（外设区 0x40000000..0x40040000 之外），
        // 需单独映射 + 转发 hook（同一 MPU 链路），否则 CPU 访问 READ_UNMAPPED。
        {
            let usb_base: u64 = USB_OTG_FS_BASE as u64;
            self.cpu.mem_map(usb_base, 0x5000, Prot::ALL)?;
            let bus = self.bus.clone();
            let mpu = self.mpu.clone();
            self.cpu
                .add_mmio_hook(usb_base, usb_base + 0x5000, move |uc, ty, addr, size, value| {
                    let fault = {
                        let m = mpu.lock().unwrap();
                        match mem_type_to_access(ty) {
                            Some(access) => m.check(addr as u32, access, cpu_privileged(uc)).err(),
                            None => None,
                        }
                    };
                    if let Some(f) = fault {
                        fault_and_stop(uc, &mpu, f);
                        return true;
                    }
                    match ty {
                        MemType::READ => {
                            if let Ok(v) = bus.lock().unwrap().read(addr as u32, size as u32) {
                                let _ = uc.mem_write(addr, &v.to_le_bytes()[..size]);
                            }
                        }
                        MemType::WRITE => {
                            let _ = bus.lock().unwrap().write(addr as u32, size as u32, value as u32);
                        }
                        _ => {}
                    }
                    false
                })?;
        }
        // UsbSetup 事件（虚拟主机/测试）→ 注入设备模式 SETUP 包（驱动 DOEPINT0.STUP）
        let u0 = self.usb_otg.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::UsbSetup { data } = ev {
                    u0.lock().unwrap().inject_setup(*data);
                }
            },
        )));

        // TIM1-14（tick 推进 + 溢出 → NVIC 更新中断；TIM1-8 的 DIER.UDE → 更新事件
        // DMA 请求，TIM9-14 无 DMA 请求能力）。类别/位宽/通道数/中断号按 F407 硬件：
        // TIM1/8 高级 16 位 4 通道，TIM2/5 通用 32 位 4 通道，TIM3/4 通用 16 位
        // 4 通道，TIM6/7 基本 16 位（无通道），TIM9/12 通用 16 位 2 通道，
        // TIM10/11/13/14 通用 16 位 1 通道。TIM9-14 中断行与高级定时器共享
        // （TIM9↔TIM1_BRK24、TIM10↔TIM1_UP25、TIM11↔TIM1_TRG_COM26、
        // TIM12↔TIM8_BRK43、TIM13↔TIM8_UP44、TIM14↔TIM8_TRG_COM45）。
        // 更新事件 DMA 映射采用 HAL 默认流（TIM1/8 → DMA2，TIM2-7 → DMA1）。
        // (port, base, name, kind, bits, channels, irq, clk_hz)
        let tim_cfgs: &[(u8, u32, &str, TimerKind, u32, u8, TimerIrq, u32)] = &[
            // APB2 定时器（TIM1/8/9/10/11）168MHz，其余 APB1 84MHz。
            (1, 0x4001_0000, "TIM1", TimerKind::Advanced, 16, 4,
             TimerIrq { brk: 24, up: 25, trig_com: 26, cc: 27 }, 168_000_000),
            (2, 0x4000_0000, "TIM2", TimerKind::General, 32, 4,
             TimerIrq { brk: 28, up: 28, trig_com: 28, cc: 28 }, 84_000_000),
            (3, 0x4000_0400, "TIM3", TimerKind::General, 16, 4,
             TimerIrq { brk: 29, up: 29, trig_com: 29, cc: 29 }, 84_000_000),
            (4, 0x4000_0800, "TIM4", TimerKind::General, 16, 4,
             TimerIrq { brk: 30, up: 30, trig_com: 30, cc: 30 }, 84_000_000),
            (5, 0x4000_0C00, "TIM5", TimerKind::General, 32, 4,
             TimerIrq { brk: 50, up: 50, trig_com: 50, cc: 50 }, 84_000_000),
            (6, 0x4000_1000, "TIM6", TimerKind::Basic, 16, 0,
             TimerIrq { brk: 54, up: 54, trig_com: 54, cc: 54 }, 84_000_000),
            (7, 0x4000_1400, "TIM7", TimerKind::Basic, 16, 0,
             TimerIrq { brk: 55, up: 55, trig_com: 55, cc: 55 }, 84_000_000),
            (8, 0x4001_0400, "TIM8", TimerKind::Advanced, 16, 4,
             TimerIrq { brk: 43, up: 44, trig_com: 45, cc: 46 }, 168_000_000),
            (9, 0x4001_4000, "TIM9", TimerKind::General, 16, 2,
             TimerIrq { brk: 24, up: 24, trig_com: 24, cc: 24 }, 168_000_000),
            (10, 0x4001_4400, "TIM10", TimerKind::General, 16, 1,
             TimerIrq { brk: 25, up: 25, trig_com: 25, cc: 25 }, 168_000_000),
            (11, 0x4001_4800, "TIM11", TimerKind::General, 16, 1,
             TimerIrq { brk: 26, up: 26, trig_com: 26, cc: 26 }, 168_000_000),
            (12, 0x4000_1800, "TIM12", TimerKind::General, 16, 2,
             TimerIrq { brk: 43, up: 43, trig_com: 43, cc: 43 }, 84_000_000),
            (13, 0x4000_1C00, "TIM13", TimerKind::General, 16, 1,
             TimerIrq { brk: 44, up: 44, trig_com: 44, cc: 44 }, 84_000_000),
            (14, 0x4000_2000, "TIM14", TimerKind::General, 16, 1,
             TimerIrq { brk: 45, up: 45, trig_com: 45, cc: 45 }, 84_000_000),
        ];
        for (port, base, name, kind, bits, channels, irq, clk_hz) in tim_cfgs {
            let cfg = TimerConfig { name, kind: *kind, bits: *bits, channels: *channels, irq: *irq, clk_hz: *clk_hz };
            let tim_active = Arc::new(AtomicBool::new(false));
            let tim = Arc::new(Mutex::new(Timer::with_active(
                *port,
                cfg,
                events.clone(),
                self.nvic.clone(),
                tim_active.clone(),
                self.retired_insts.clone(),
            )));
            self.bus
                .lock()
                .unwrap()
                .attach(*base, 0x400, format!("{name}"), tim.clone())?;
            // 注册 TIM 句柄到对应 DMA 控制器（TIM1/8 → DMA2，TIM2-7 → DMA1，
            // 内存→外设搬运经句柄按 DCR 突发写 DMAR）。TIM9-14 在 F407 上
            // 无 DMA 请求映射（RM0090 DMA 请求表不含 TIM9-14），不注册。
            if *port <= 8 {
                let reg_ctrl = if *port == 1 || *port == 8 { self.dma2.clone() } else { self.dma.clone() };
                reg_ctrl.lock().unwrap().register_tim(*port, tim.clone());
            }
            self.timers.lock().unwrap().push(tim);
            self.tick_actives.lock().unwrap().push(tim_active);
        }

        // TIM 更新事件 → DMA 请求（F407 固定映射 + HAL 默认流，见
        // STM32F4xx_hal_tim.c TIM_DMA_GetConfig：TIM1_UP→DMA2_S5_Ch6、
        // TIM2_UP→DMA1_S5_Ch5、TIM3_UP→DMA1_S3_Ch5、TIM4_UP→DMA1_S3_Ch2、
        // TIM5_UP→DMA1_S6_Ch6、TIM6_UP→DMA1_S0_Ch7、TIM7_UP→DMA1_S5_Ch4、
        // TIM8_UP→DMA2_S3_Ch7；方向按流 CR.DIR 取，支持 PWM 装载（内存→外设）
        // 与捕获（外设→内存））
        let dma1 = self.dma.clone();
        let dma2 = self.dma2.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::TimUpdate { port } = ev {
                    let (ctrl, stream, channel) = match *port {
                        1 => (dma2.clone(), 5, 6), // TIM1_UP: DMA2_Stream5_Channel6
                        2 => (dma1.clone(), 5, 5), // TIM2_UP: DMA1_Stream5_Channel5
                        3 => (dma1.clone(), 3, 5), // TIM3_UP: DMA1_Stream3_Channel5
                        4 => (dma1.clone(), 3, 2), // TIM4_UP: DMA1_Stream3_Channel2
                        5 => (dma1.clone(), 6, 6), // TIM5_UP: DMA1_Stream6_Channel6
                        6 => (dma1.clone(), 0, 7), // TIM6_UP: DMA1_Stream0_Channel7
                        7 => (dma1.clone(), 5, 4), // TIM7_UP: DMA1_Stream5_Channel4
                        8 => (dma2.clone(), 3, 7), // TIM8_UP: DMA2_Stream3_Channel7
                        _ => return,
                    };
                    let dir = ctrl.lock().unwrap().stream_dir(stream);
                    ctrl.lock()
                        .unwrap()
                        .service_stream(stream, channel, dir, crate::peripheral::dma::DmaTarget::Tim(*port));
                }
            },
        )));

        // M4-DMA1/DMA2（MEM2MEM 传输 + TC 中断；tick 判完成，run 间隙 process 搬运）
        let dma = self.dma.clone();
        self.bus.lock().unwrap().attach(DMA1_BASE, 0x400, "DMA1", dma.clone())?;
        self.timers.lock().unwrap().push(dma);
        self.tick_actives.lock().unwrap().push(self.dma_active.clone());

        let dma2 = self.dma2.clone();
        self.bus.lock().unwrap().attach(DMA2_BASE, 0x400, "DMA2", dma2.clone())?;
        self.timers.lock().unwrap().push(dma2);
        self.tick_actives.lock().unwrap().push(self.dma2_active.clone());

        // USART DMA 请求路由（F407 固定映射：port + 方向 → DMAx_StreamN_ChannelM，
        // 采用 HAL 默认流，见 STM32F4xx_hal_uart.c UART_DMA_GetConfig）。
        let dma1 = self.dma.clone();
        let dma2 = self.dma2.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::UartDma { port, dir } = ev {
                    let (ctrl, stream, channel) = match (*port, *dir) {
                        // USART1：DMA2_Stream7_Channel4(TX) / DMA2_Stream2_Channel4(RX)
                        (1, DmaDir::MemToPeriph) => (dma2.clone(), 7, 4),
                        (1, DmaDir::PeriphToMem) => (dma2.clone(), 2, 4),
                        // USART2：DMA1_Stream6_Channel4(TX) / DMA1_Stream5_Channel4(RX)
                        (2, DmaDir::MemToPeriph) => (dma1.clone(), 6, 4),
                        (2, DmaDir::PeriphToMem) => (dma1.clone(), 5, 4),
                        // USART3：DMA1_Stream3_Channel4(TX) / DMA1_Stream1_Channel4(RX)
                        (3, DmaDir::MemToPeriph) => (dma1.clone(), 3, 4),
                        (3, DmaDir::PeriphToMem) => (dma1.clone(), 1, 4),
                        // UART4：DMA1_Stream4_Channel4(TX) / DMA1_Stream2_Channel4(RX)
                        (4, DmaDir::MemToPeriph) => (dma1.clone(), 4, 4),
                        (4, DmaDir::PeriphToMem) => (dma1.clone(), 2, 4),
                        // UART5：DMA1_Stream7_Channel4(TX) / DMA1_Stream0_Channel4(RX)
                        (5, DmaDir::MemToPeriph) => (dma1.clone(), 7, 4),
                        (5, DmaDir::PeriphToMem) => (dma1.clone(), 0, 4),
                        // USART6：DMA2_Stream6_Channel5(TX) / DMA2_Stream1_Channel5(RX)
                        (6, DmaDir::MemToPeriph) => (dma2.clone(), 6, 5),
                        (6, DmaDir::PeriphToMem) => (dma2.clone(), 1, 5),
                        _ => return,
                    };
                    ctrl.lock()
                        .unwrap()
                        .service_stream(stream, channel, *dir, crate::peripheral::dma::DmaTarget::Usart(*port));
                }
            },
        )));

        // I2C DMA 请求路由（F407 固定映射：port + 方向 → DMA1_StreamN_ChannelM，
        // 采用 HAL 默认流，见 stm32f4xx_hal_i2c.c I2C_DMA_GetConfig）。
        let dma1 = self.dma.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::I2cDma { port, dir } = ev {
                    let (stream, channel) = match (*port, *dir) {
                        // I2C1：DMA1_Stream6_Channel1(TX) / DMA1_Stream0_Channel1(RX)
                        (1, DmaDir::MemToPeriph) => (6, 1),
                        (1, DmaDir::PeriphToMem) => (0, 1),
                        // I2C2：DMA1_Stream7_Channel7(TX) / DMA1_Stream3_Channel7(RX)
                        (2, DmaDir::MemToPeriph) => (7, 7),
                        (2, DmaDir::PeriphToMem) => (3, 7),
                        // I2C3：DMA1_Stream4_Channel3(TX) / DMA1_Stream2_Channel3(RX)
                        (3, DmaDir::MemToPeriph) => (4, 3),
                        (3, DmaDir::PeriphToMem) => (2, 3),
                        _ => return,
                    };
                    dma1.lock()
                        .unwrap()
                        .service_stream(stream, channel, *dir, crate::peripheral::dma::DmaTarget::I2c(*port));
                }
            },
        )));

        // SPI DMA 请求路由（F407 固定映射：port + 方向 → DMAx_StreamN_ChannelM，
        // 采用 HAL 默认流，见 stm32f4xx_hal_spi.c SPI_DMA_GetConfig）。
        let dma1 = self.dma.clone();
        let dma2 = self.dma2.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::SpiDma { port, dir } = ev {
                    let (ctrl, stream, channel) = match (*port, *dir) {
                        // SPI1：DMA2_Stream3_Channel3(TX) / DMA2_Stream0_Channel3(RX)
                        (1, DmaDir::MemToPeriph) => (dma2.clone(), 3, 3),
                        (1, DmaDir::PeriphToMem) => (dma2.clone(), 0, 3),
                        // SPI2：DMA1_Stream4_Channel0(TX) / DMA1_Stream3_Channel0(RX)
                        (2, DmaDir::MemToPeriph) => (dma1.clone(), 4, 0),
                        (2, DmaDir::PeriphToMem) => (dma1.clone(), 3, 0),
                        // SPI3：DMA1_Stream7_Channel0(TX) / DMA1_Stream0_Channel0(RX)
                        (3, DmaDir::MemToPeriph) => (dma1.clone(), 7, 0),
                        (3, DmaDir::PeriphToMem) => (dma1.clone(), 0, 0),
                        _ => return,
                    };
                    ctrl.lock()
                        .unwrap()
                        .service_stream(stream, channel, *dir, crate::peripheral::dma::DmaTarget::Spi(*port));
                    // SPI 主全双工：固件 spi_dma_xfer 同时 arm TX+RX 双流。TX 请求
                    // 到达时把配对 RX 流也登记上（DMA RX 回复由 TX 交换入 FIFO 后
                    // 搬运；流号 RX< TX 时 process 先遇空 FIFO → 节流保留，见
                    // [`Dma::process`] 的 paired_tx_pending 判定）。
                    if *dir == DmaDir::MemToPeriph {
                        let (rctrl, rstream, rchannel) = match *port {
                            1 => (dma2.clone(), 0, 3), // SPI1_RX: DMA2_Stream0_Channel3
                            2 => (dma1.clone(), 3, 0), // SPI2_RX: DMA1_Stream3_Channel0
                            3 => (dma1.clone(), 0, 0), // SPI3_RX: DMA1_Stream0_Channel0
                            _ => return,
                        };
                        rctrl.lock()
                            .unwrap()
                            .service_stream(rstream, rchannel, DmaDir::PeriphToMem, crate::peripheral::dma::DmaTarget::Spi(*port));
                    }
                }
            },
        )));

        // M4-看门狗：IWDG（独立，@0x40003000）+ WWDG（窗口，@0x40002C00）
        // 共享复位请求：超时/违规 → block hook 停机 → run() 执行系统复位。
        let iwdg = self.iwdg.clone();
        self.bus.lock().unwrap().attach(0x4000_3000, 0x400, "IWDG", iwdg.clone())?;
        self.timers.lock().unwrap().push(iwdg);
        self.tick_actives.lock().unwrap().push(self.iwdg_active.clone());

        let wwdg = self.wwdg.clone();
        self.bus.lock().unwrap().attach(0x4000_2C00, 0x400, "WWDG", wwdg.clone())?;
        self.timers.lock().unwrap().push(wwdg);
        self.tick_actives.lock().unwrap().push(self.wwdg_active.clone());

        // M10-PWR 电源控制（@0x40007000，APB1）。
        // CR 低功耗位写读 + CWUF/CSBF 写 1 清 WUF/SBF；CSR.WUF/SBF/PVDO 只读标志
        // 经 [`Pwr::inject_*`] 注入（模拟 WKUP/PVD 事件）。待机唤醒复位：固件/测试
        // 先 enter_standby（模拟 WFI/WFE）再 inject_wakeup → 经共享复位请求
        // （同一 wdog_req 链路）发出 ResetReason::LowPower，run() 执行系统复位。
        let pwr = self.pwr.clone();
        self.bus.lock().unwrap().attach(0x4000_7000, 0x400, "PWR", pwr)?;

        // M11-RTC + 备份寄存器（@0x40002800，APB1）。
        // 日历：双预分频（PRER）把 RTCCLK 分频为 1 Hz ck_spre，tick 按周期推进秒计数；
        // 闹钟 A/B + 唤醒定时器：匹配置 ISR 标志并经 NVIC 挂起 RTC_Alarm(41)/RTC_WKUP(3)；
        // 备份寄存器 BKP0R-19R（+0x50..+0x9C）写访问需 PWR_CR.DBP=1（备份域写保护）。
        // 写入 tick 列表：随虚拟时钟推进（与 TIM/IWDG/WWDG 共享同一周期源）。
        let rtc = self.rtc.clone();
        self.bus.lock().unwrap().attach(0x4000_2800, 0x400, "RTC", rtc.clone())?;
        self.timers.lock().unwrap().push(rtc);
        self.tick_actives.lock().unwrap().push(self.rtc_active.clone());

        // M4-EXTI：SYSCFG（EXTICR 端口选择） + EXTI（外部中断，GPIO 事件 → NVIC）
        let port_select = Arc::new(Mutex::new(ExtiPortSelect::default()));
        let syscfg = Arc::new(Mutex::new(Syscfg::new(port_select.clone())));
        self.bus.lock().unwrap().attach(0x4001_3800, 0x400, "SYSCFG", syscfg)?;

        let exti = Arc::new(Mutex::new(Exti::new(port_select.clone(), self.nvic.clone())));
        self.bus.lock().unwrap().attach(EXTI_BASE, 0x400, "EXTI", exti.clone())?;

        // GPIO 电平事件 → EXTI 输入（模拟外部驱动；EXTI 侧做端口/沿/屏蔽过滤）
        {
            let exti2 = exti.clone();
            events.lock().unwrap().subscribe(Arc::new(Mutex::new(
                move |ev: &Event| {
                    if let Event::GpioLevel { port, pin, level, .. } = ev {
                        exti2.lock().unwrap().feed_gpio(*port, *pin, *level);
                    }
                },
            )));
        }

        // 外设区 MMIO 转发 hook（MPU 检查 + 读注入 / 写转发）
        let bus = self.bus.clone();
        let mpu = self.mpu.clone();
        // 参与 tick 的外设地址区间（TIM1-14 + DAC + DMA1/2 + IWDG + WWDG + RTC）。
        // 写这些区间即置位 BIT_ANY_ACTIVE，block hook 据此走 tick 循环；免去 block hook
        // 每块全扫 ~20 个 active 标记（bench_probe：actives 7→20，MIPS 57.9→34.7）。
        // 注：BIT_ANY_ACTIVE 只置不清（外设激活后需持续 tick；未激活外设在 tick 循环内
        // 按各自标记跳过，开销仅为逐标记判读）。
        let status = self.status.clone();
        let tick_regions: Arc<Vec<(u64, u64)>> = Arc::new(vec![
            // TIM1-14（每块 0x400）
            (0x4001_0000, 0x400), (0x4000_0000, 0x400), (0x4000_0400, 0x400),
            (0x4000_0800, 0x400), (0x4000_0C00, 0x400), (0x4000_1000, 0x400),
            (0x4000_1400, 0x400), (0x4001_0400, 0x400), (0x4001_4000, 0x400),
            (0x4001_4400, 0x400), (0x4001_4800, 0x400), (0x4000_1800, 0x400),
            (0x4000_1C00, 0x400), (0x4000_2000, 0x400),
            // DAC / DMA1 / DMA2
            (0x4000_7400, 0x400), (DMA1_BASE as u64, 0x400), (DMA2_BASE as u64, 0x400),
            // IWDG / WWDG / RTC
            (0x4000_3000, 0x400), (0x4000_2C00, 0x400), (0x4000_2800, 0x400),
        ]);
        self.cpu.add_mmio_hook(periph_base, periph_base + periph_size, move |uc, ty, addr, size, value| {
            let fault = {
                let m = mpu.lock().unwrap();
                match mem_type_to_access(ty) {
                    Some(access) => m.check(addr as u32, access, cpu_privileged(uc)).err(),
                    None => None,
                }
            };
            if let Some(f) = fault {
                fault_and_stop(uc, &mpu, f);
                return true;
            }
            match ty {
                MemType::READ => {
                    if let Ok(v) = bus.lock().unwrap().read(addr as u32, size as u32) {
                        let _ = uc.mem_write(addr, &v.to_le_bytes()[..size]);
                    }
                }
                MemType::WRITE => {
                    if !status.has(BIT_ANY_ACTIVE)
                        && tick_regions.iter().any(|(b, s)| addr >= *b && addr < *b + *s)
                    {
                        status.set(BIT_ANY_ACTIVE);
                    }
                    let _ = bus.lock().unwrap().write(addr as u32, size as u32, value as u32);
                }
                _ => {}
            }
            false
        })?;

        log::info!("T1 外设已挂载：GPIOA-I + USART1-6 + I2C1-3 + SPI1-3 + ADC1-3 + TIM1-14 + RCC + SYSCFG/EXTI + DMA1/DMA2 + Console/Terminal @ 0x{periph_base:08X} +0x{periph_size:X}");
        Ok(())
    }

    /// 外设互联（类 Renode `connect` 语法，M3 DSL 入口）。
    ///
    /// 当前支持：
    /// - `uart.tx -> console.rx`：把指定 USART 端口的 TX 字节事件订阅到虚拟 Console；
    /// - `uart.tx -> terminal.rx`：把指定 USART 端口的 TX 字节事件订阅到虚拟终端显示。
    ///
    /// 反向（终端键盘 → UART RX）由 [`crate::peripheral::terminal::Terminal::type_char`]
    /// 发布 `UartRx` 事件、USART 全局订阅完成，无需显式 connect。
    pub fn connect(&mut self, src: ConnectSource, dst: ConnectTarget) -> Result<()> {
        match (src, dst) {
            (ConnectSource::UartTx(port), ConnectTarget::ConsoleRx) => {
                let events = self.events.clone();
                let console = self.console.clone();
                let c = console.clone();
                events.lock().unwrap().subscribe(Arc::new(Mutex::new(
                    move |ev: &Event| {
                        if let Event::UartByte { port: p, byte } = ev {
                            if *p == port {
                                c.lock().unwrap().write_byte(*byte);
                            }
                        }
                    },
                )));
                Ok(())
            }
            (ConnectSource::UartTx(port), ConnectTarget::TerminalRx) => {
                let events = self.events.clone();
                let terminal = self.terminal.clone();
                let t = terminal.clone();
                events.lock().unwrap().subscribe(Arc::new(Mutex::new(
                    move |ev: &Event| {
                        if let Event::UartByte { port: p, byte } = ev {
                            if *p == port {
                                t.lock().unwrap().write_display(*byte);
                            }
                        }
                    },
                )));
                Ok(())
            }
        }
    }

    /// 中断投递 hook：
    /// 1. block hook：每个基本块开头检查是否有更高优先级挂起中断，若有则
    ///    记录停机原因并停止执行（由 [`Machine::run`] 做异常入栈）；
    /// 2. intr hook：捕获 EXC_RETURN（intno=8）异常返回事件，出栈恢复现场。
    ///
    /// 看门狗复位：block hook 同时检查共享 [`WdogResetReq`]，有请求即停机
    /// （不设置中断停机原因，由 [`Machine::run`] 识别复位请求并执行系统复位）。
    fn attach_interrupt_delivery(&mut self) -> Result<()> {
        // 1) block hook：XN 检查 + 挂起中断抢占检查 + 块级时钟推进（begin=1,end=0 全范围）
        let status = self.status.clone();
        let clock = self.clock.clone();
        // 冷路径状态捆成单个 Arc<BlockHookCold>：缩小闭包捕获体（3 字段 → 24B），
        // 热路径仅 status/clock 两个指针进闭包（bench_probe：H13d 6 字段 97.7 → H13e 3 字段 121+ MIPS）
        let cold = Arc::new(BlockHookCold {
            mpu: self.mpu.clone(),
            nvic: self.nvic.clone(),
            // 冻结时钟外设列表：所有外设已挂载，转成 Vec，block hook 免每块加锁
            //（bench_probe：H4 每块 timers.lock() 28.8 → H5 冻结无锁 47.3 MIPS）
            // ★§5.102 ④：按 **Arc 指针同一性**标出"吃原始流"的外设（只 SCB ✓）。
            // `Arc<Mutex<SystemControl>>` 被强制转换为 `Arc<Mutex<dyn Peripheral>>` 时
            // **数据指针不变** ⇒ 取 `Arc::as_ptr(..) as *const () as usize` 可比 ✓。
            tick_raw: {
                let scb_p = self
                    .scb
                    .as_ref()
                    .map(|s| Arc::as_ptr(s) as *const () as usize);
                let ts = self.timers.lock().unwrap();
                ts.iter()
                    .map(|t| Some(Arc::as_ptr(t) as *const () as usize) == scb_p)
                    .collect()
            },
            timers: self.timers.lock().unwrap().clone(),
            // 冻结活动标记列表：所有外设已挂载，转成 Vec，block hook 快路径免加锁
            tick_actives: self.tick_actives.lock().unwrap().clone(),
            retired: self.retired_insts.clone(),
            dma1: self.dma.clone(),
            dma2: self.dma2.clone(),
            dma_ticks: AtomicU32::new(0),
        });
        // 数据 hook 安装标记：MPU 首次使能且未安装时停机一次，交 run() 懒安装
        let dh_installed = self.data_hook_installed.clone();
        // ★定时器周期流的余数累加器（§5.100 #2）：`Δ字节 × 210/239` 的截断余数
        // 逐块累积 ⇒ 长期精确、无截断漂移 ✓。置于闭包外并随 move 捕获
        //（block hook 是 `move` 闭包，拿不到 `self` ✗）。
        let timer_frac = std::cell::Cell::new(0u64);
        self.cpu.add_block_hook(1, 0, move |uc, _addr, size| {
            // 热路径单次原子 load，按位测试 MPU/外设激活/看门狗/中断挂起四个低频标志
            //（bench_probe：H2 独立原子 37.8 → H12 单状态字 115.1 MIPS）
            let s = status.raw();
            // 退休指令计数：每块累加 TB 字节（run() 按此递减预算，保证正常终止）。
            // Thumb 下字节≈2×指令；预算只作工作量的粗粒度上界，误差可接受。
            cold.retired.fetch_add(size as u64, Ordering::Relaxed);
            // MPU 使能但数据 hook 未装：停机由 run() 懒安装（含刷 TB）
            if s & BIT_MPU != 0 && !dh_installed.load(Ordering::Relaxed) {
                cold
                    .nvic
                    .lock()
                    .unwrap()
                    .set_stop_reason(StopReason::MpuEnable);
                let _ = uc.emu_stop();
                return;
            }
            // 取指 XN 检查（块级）：MPU 未使能时无锁跳过。
            // 块执行前触发，块内指令同属一个区域，等效原每指令检查。
            if s & BIT_MPU != 0 {
                let addr = (_addr & !1) as u32;
                let fault = {
                    let m = cold.mpu.lock().unwrap();
                    m.check(addr, Access::Fetch, cpu_privileged(uc)).err()
                };
                if let Some(f) = fault {
                    fault_and_stop(uc, &cold.mpu, f);
                    return;
                }
            }
            // 块级推进虚拟时钟，并 tick 活动外设（TIM/DMA/DAC/RTC/IWDG/WWDG）。
            // 对齐 QEMU icount 口径：每个 TB 按「访客字节 = 虚拟周期」折算，使 SysTick
            // reload(168000 周期) 大致对应 ~4-6 万条退休指令（真机 1ms 同量级；
            // 实测均值 ~4.6 万，见 tests/x_sys_retire_calib.rs）。注意历史注释中的
            // "≈1.98e4 退休"是旧 ×AVG=3 口径（timing.rs BlockWeighted 已废弃），
            // 与新口径不矛盾——当前生效的是「访客字节 = 虚拟周期」。
            let cycles = size as u64;
            clock.advance(cycles);
            // ★定时器周期流（§5.100 #2 修复）：按【声明时钟 84MHz 基准】折算，
            //   而不是直接用退休字节数 ✗（那会让定时器速率=仿真器吞吐 ⇒ 随代码浮动 ✗）。
            //   余数累加 ⇒ 长期精确 ✓。
            let cyc_acc = timer_frac.get()
                + size as u64 * crate::sim::timing::TIMER_CYC84_PER_BYTE_NUM;
            let timer_cycles = cyc_acc / crate::sim::timing::TIMER_CYC84_PER_BYTE_DEN;
            timer_frac.set(cyc_acc % crate::sim::timing::TIMER_CYC84_PER_BYTE_DEN);
            // 快路径：BIT_ANY_ACTIVE 由外设区 MMIO 写置位（外设激活只可能发生在 MMIO 写，
            // 见 attach_peripherals 的 TICK_REGIONS 判定），block hook 免去每块全扫
            // ~20 个 active 标记（bench_probe：H10 actives 7→20，MIPS 57.9→34.7）。
            if s & BIT_ANY_ACTIVE != 0 {
                for ((t, a), raw) in cold
                    .timers
                    .iter()
                    .zip(cold.tick_actives.iter())
                    .zip(cold.tick_raw.iter())
                {
                    if a.load(Ordering::Relaxed) {
                        // ★SCB/SysTick 吃【原始流】✓（其 RVR 定义固件毫秒 ✓）；
                        //   TIM 吃【声明时钟折算流】✓。
                        let c = if *raw { cycles } else { timer_cycles };
                        t.lock().unwrap().tick(c);
                    }
                }
            }
            // 看门狗复位请求（BIT_WDOG 由 WdogResetReq::request 置位）：停机
            //（run() 经 take() 消费请求、清除 BIT_WDOG 并执行系统复位）
            if s & BIT_WDOG != 0 {
                let _ = uc.emu_stop();
                return;
            }
            // DMA 待搬运检查（每 256 块，仅读位图无内存访问）：CPU 忙等外设标志
            //（SDIO DATAEND / DMA 完成信号量）时 emu_start 不返回 → run() 循环的
            // process 永不执行 → DMA 请求饿死。此处停机让 run() 返回后搬运（真机
            // DMA 与 CPU 并行，模拟器以此对齐）。无 pending 时快速返回，热路径可忽略。
            //
            // 间隔 256 为实测选定值：曾怀疑"本间隔决定 dma_wait_done 自旋量 →
            // 限制 sensors 采样率"，故试过 16 与 4。用 `tests/x_sensor_rate.rs`
            // 以 880ms 场景窗口实测（固件时钟与场景 1:1 对齐后）：
            //   间隔 256 → 采样 254.5Hz、retired/step 405145
            //   间隔 16  → 采样 254.5Hz、retired/step 409072
            // 采样率与 retired 均无差异（<1%，噪声级）——采样率上限由 CPU 总负载
            // （5 路驱动状态机 + control 任务 EKF 在同核抢占）决定，而非本间隔；
            // 故保持 256（检查频次低、热路径开销小）。
            if cold.dma_ticks.fetch_add(1, Ordering::Relaxed) & 0xFF == 0 {
                let p1 = cold.dma1.lock().unwrap().has_pending();
                let p2 = cold.dma2.lock().unwrap().has_pending();
                if p1 || p2 {
                    // 必须显式写停机原因：裸 emu_stop() 会让 run() 的 take_stop_reason
                    // 拿到 None → 误判预算耗尽 break，DMA 繁忙时 run(count) 只退休
                    // 一小段预算（实测 ~48K/300K），固件虚拟时钟大幅慢于物理步长。
                    cold.nvic
                        .lock()
                        .unwrap()
                        .set_stop_reason(StopReason::DmaPending);
                    let _ = uc.emu_stop();
                }
            }
            // 挂起中断抢占检查（快路径：无挂起中断时跳过加锁的 select_pending_vector）
            if s & BIT_NVIC_PENDING != 0 {
                let primask = uc.reg_read(RegisterARM::PRIMASK).unwrap_or(0) != 0;
                // BASEPRI → 优先级数值。转换收在 `nvic::basepri_to_prio` 一处，
                // 并带单测守卫（曾因取错位段导致内核临界区被当成"不屏蔽"，使
                // SysTick/PendSV 切入 sleep_add/sleep_remove 破坏睡眠链）。
                let basepri = crate::peripheral::nvic::basepri_to_prio(
                    uc.reg_read(RegisterARM::BASEPRI).unwrap_or(0) as u32,
                );
                let mut n = cold.nvic.lock().unwrap();
                if let Some(vector) = n.select_pending_vector(primask, basepri) {
                    n.set_stop_reason(StopReason::Switch(vector));
                    let _ = uc.emu_stop();
                }
            }
        })?;

        // 2) intr hook：EXC_RETURN 异常返回（intno=8）与 SVC 异常入口（intno=2）。
        //    - Unicorn 的 do_v7m_exception_exit / v7m_exception_taken 均被置空：
        //      * 异常返回：现场恢复完全由本回调完成（弹出异常栈、按 EXC_RETURN 选栈出栈）；
        //      * SVC 入口：arm_v7m_cpu_do_interrupt 已调 v7m_push_stack 把 8 字异常帧压栈，
        //        但"进入 handler"（LR/IPSR/PC）被空置，故此处补全入口设置。
        let nvic2 = self.nvic.clone();
        self.cpu.add_intr_hook(move |uc, intno| {
            if intno == 8 {
                let exc_return = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
                if let Err(e) = exception_return(uc, &nvic2, exc_return) {
                    log::error!("异常返回失败：{e:?}");
                }
                nvic2
                    .lock()
                    .unwrap()
                    .set_stop_reason(StopReason::ExceptionReturn);
                let _ = uc.emu_stop();
            } else if intno == 2 {
                // EXCP_SWI（SVC 指令）：Unicorn 通过 intr hook 完全接管异常处理，
                // QEMU 原生 v7m_push_stack / v7m_exception_taken 均被绕过，
                // 故此处需完整补全 Cortex-M 异常入口：压 8 字异常帧 + 设置 LR/IPSR/PC。
                let in_handler = nvic2.lock().unwrap().in_handler();
                let control = uc.reg_read(RegisterARM::CONTROL).unwrap_or(0) as u32;
                let mut exc_return: u32 = if in_handler {
                    0xFFFF_FFF1 // 从 handler 进入，恒 MSP
                } else if control & 2 != 0 {
                    0xFFFF_FFFD // 线程模式 + PSP
                } else {
                    0xFFFF_FFF9 // 线程模式 + MSP
                };
                // 与 enter_exception 相同：FPCA=1 → 扩展帧（bit4=0），供 RTOS
                // context.S 的 PendSV 判定 `tst r14,#0x10` 保存/恢复 FPU 寄存器。
                if control & (1 << 2) != 0 {
                    exc_return &= !0x10;
                }
                // 选栈：与 enter_exception/exception_return 一致，显式读写 MSP/PSP
                let sp = if in_handler || exc_return & 0x4 == 0 {
                    uc.reg_read(RegisterARM::MSP).unwrap_or(0) as u32
                } else {
                    uc.reg_read(RegisterARM::PSP).unwrap_or(0) as u32
                };
                // 采集被中断现场（SVC 后 PC 已指向下一条指令，Unicorn 保证）
                let r0 = uc.reg_read(RegisterARM::R0).unwrap_or(0) as u32;
                let r1 = uc.reg_read(RegisterARM::R1).unwrap_or(0) as u32;
                let r2 = uc.reg_read(RegisterARM::R2).unwrap_or(0) as u32;
                let r3 = uc.reg_read(RegisterARM::R3).unwrap_or(0) as u32;
                let r12 = uc.reg_read(RegisterARM::R12).unwrap_or(0) as u32;
                let lr = uc.reg_read(RegisterARM::LR).unwrap_or(0) as u32;
                let pc_saved = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
                let xpsr = uc.reg_read(RegisterARM::XPSR).unwrap_or(0) as u32 & 0xFF00_0000;
                // 压 8 字帧（低地址→高地址：r0 r1 r2 r3 r12 LR PC xPSR），SP -= 32
                let sp = sp - 32;
                let mut frame = [0u8; 32];
                for (i, v) in [r0, r1, r2, r3, r12, lr, pc_saved, xpsr]
                    .iter()
                    .enumerate()
                {
                    frame[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
                }
                let _ = uc.mem_write(sp as u64, &frame);
                if in_handler || exc_return & 0x4 == 0 {
                    let _ = uc.reg_write(RegisterARM::MSP, sp as u64);
                } else {
                    let _ = uc.reg_write(RegisterARM::PSP, sp as u64);
                }
                // 进入 handler：LR=EXC_RETURN，IPSR=11，PC=向量表[11]|1
                let handler = uc
                    .mem_read_as_vec(0x0800_0000 + 11 * 4, 4)
                    .ok()
                    .and_then(|b| b.try_into().ok())
                    .map(u32::from_le_bytes)
                    .unwrap_or(0);
                let _ = uc.reg_write(RegisterARM::LR, exc_return as u64);
                let _ = uc.reg_write(RegisterARM::IPSR, 11);
                let _ = uc.reg_write(RegisterARM::PC, (handler | 1) as u64);
                // 与 enter_exception 对称：保存被中断现场 r4..r11（SVC 调用者现场），
                // EXC_RETURN 返回时 pop_callee 还原——exception_return 统一要求
                // callee 栈与异常栈同步（缺则返回报"无 callee 现场"）。
                let callee = [
                    uc.reg_read(RegisterARM::R4).unwrap_or(0) as u32,
                    uc.reg_read(RegisterARM::R5).unwrap_or(0) as u32,
                    uc.reg_read(RegisterARM::R6).unwrap_or(0) as u32,
                    uc.reg_read(RegisterARM::R7).unwrap_or(0) as u32,
                    uc.reg_read(RegisterARM::R8).unwrap_or(0) as u32,
                    uc.reg_read(RegisterARM::R9).unwrap_or(0) as u32,
                    uc.reg_read(RegisterARM::R10).unwrap_or(0) as u32,
                    uc.reg_read(RegisterARM::R11).unwrap_or(0) as u32,
                ];
                nvic2.lock().unwrap().push_exception(11);
                nvic2.lock().unwrap().push_callee(callee);
                nvic2.lock().unwrap().set_stop_reason(StopReason::SvcEntry);
                log::info!(
                    "SVC 进入：vector=11 handler=0x{handler:08X} EXC_RETURN=0x{exc_return:08X} sp=0x{sp:08X}"
                );
                let _ = uc.emu_stop();
            }
        })?;

        log::info!("中断投递 hook 已挂载（block 抢占检查 + SVC 入口 + EXC_RETURN 拦截）");

        Ok(())
    }

    /// RAM/Flash/CCM 数据访问 hook：MPU 使能后全强制（保真优先）。
    ///
    /// 区间覆盖 FLASH(0x08000000)/CCM(0x10000000)/SRAM(0x20000000)，
    /// 不含 SCB（0xE0000000+，由 MMIO hook 单独处理）。
    /// 快速路径：MPU 未使能时直接放行，保持 RAM 无 hook 的原有行为（仅一次判读）。
    ///
    /// **懒安装**：MPU 关闭期（固件启动早期）不挂载本 hook——该阶段挂载会使每条
    /// 内存访问经 hook helper 调用翻译（Unicorn 2.1.5 存在"带 hook 的新译 TB 首条
    /// 32 位指令副作用丢失"缺陷，board_tick_init 的 ldmia.w 曾因此空转）。MPU 使能
    /// （block hook 见 BIT_MPU 且未安装）时经 [`Machine::run`] 的 StopReason::MpuEnable
    /// 安装并刷 TB，使此后（含已缓存 TB 重译）的内存访问都被 MPU 检查覆盖。
    fn install_data_access_hook(&mut self) -> Result<()> {
        if self.data_hook_installed.load(Ordering::Relaxed) {
            return Ok(());
        }
        const DATA_BEGIN: u64 = 0x0800_0000;
        const DATA_END: u64 = 0x2002_0000; // 覆盖 FLASH/CCM/SRAM，止于 SRAM 末端

        let mpu = self.mpu.clone();
        let status = self.status.clone();
        self.cpu.add_mem_hook(
            HookType::MEM_READ | HookType::MEM_WRITE,
            DATA_BEGIN,
            DATA_END,
            move |uc, ty, addr, _size, _value| {
                // 快路径：MPU 未使能时无锁放行（纯计算负载下省去每内存访问的加锁）
                if status.raw() & BIT_MPU == 0 {
                    return false;
                }
                let fault = {
                    let m = mpu.lock().unwrap();
                    match mem_type_to_access(ty) {
                        Some(access) => {
                            m.check(addr as u32, access, cpu_privileged(uc)).err()
                        }
                        None => None,
                    }
                };
                if let Some(f) = fault {
                    fault_and_stop(uc, &mpu, f);
                    return true; // 阻断该次访问
                }
                false
            },
        )?;
        self.data_hook_installed.store(true, Ordering::Relaxed);
        // 刷 TB：已按"无 hook"翻译的缓存 TB 需重译，使内存访问纳入 MPU 检查
        let _ = self.cpu.raw().ctl_flush_tb();
        log::info!("RAM/Flash/CCM 数据访问 hook 已挂载（MPU 使能懒安装，TB 已刷新）");
        Ok(())
    }

    /// 取指 XN 检查——已并入 block hook（块级判定），不再注册每指令 code hook。
    ///
    /// 为什么块级可行：XN 是内存区域属性，一条直行翻译块内的所有指令同属一个
    /// 区域；若该区域 XN，则块入口处的取指即违规，block hook 在块执行前触发，
    /// 与原先每指令 code hook 在语义上等价，但避免每指令一次 FFI（强制单指令
    /// 翻译块，纯计算负载最大性能黑洞，见 bench_probe）。

    /// 运行 `count` 条指令（从当前 PC 继续）。
    ///
    /// 执行期间：
    /// - 触发 MPU MemManage fault → 返回 [`CoreError::MemManageFault`]；
    /// - 挂起中断抢占（block hook 停机）→ 异常入栈并进入 handler；
    /// - 异常返回（EXC_RETURN，intr hook 停机）→ 现场已恢复，继续；
    /// - 看门狗复位请求（block hook 停机）→ 记录 RCC_CSR 复位标志 + 系统复位，继续。
    /// `count` 以线程模式指令计：每次 emu_start 命中 `remaining` 即结束。
    ///
    /// 风暴护栏：若某外设/中断使 emu_start 频繁提前返回（单次调用迭代段数超上限、
    /// 仍未消耗完 count），记录诊断并提前返回 Ok，避免挂起中断风暴下无限自旋
    ///（由调用方继续分步推进）。
    /// 是否有外设处于活动状态（探测/调试用）。
    pub fn peripheral_any_active(&self) -> bool {
        self.status.has(BIT_ANY_ACTIVE)
    }

    /// 活动外设标记数量（探测/调试用）。
    pub fn peripheral_active_count(&self) -> usize {
        self.tick_actives
            .lock()
            .unwrap()
            .iter()
            .filter(|a| a.load(Ordering::Relaxed))
            .count()
    }

    /// 是否有挂起中断（探测/调试用）。
    /// 退役指令数（Thumb 字节计数；≈2 字节/指令）。性能观测：Δretired/墙钟 = 吞吐。
    pub fn retired_count(&self) -> u64 {
        self.retired_insts.load(Ordering::Relaxed)
    }

    /// 退休计数句柄（`Arc<AtomicU64>`，单位=TB 字节）——供测试挂内存钩子做按段剖析
    /// （在 Unicorn 回调里读它，不经过 `Machine` 借用）。
    pub fn retired_handle(&self) -> Arc<AtomicU64> {
        self.retired_insts.clone()
    }

    /// 按**固件自身虚拟时钟**推进 `ms` 毫秒。
    ///
    /// 物理闭环测试（每步推进 `dt` 秒物理）应当用本方法而非裸 `run(count)`：
    /// 让固件侧推进与物理步长一致，避免固件任务周期/固件内固定 `dt` 相对物理
    /// 步长系统性失配（根因与实测见 [`crate::sim::timing::RETIRED_BYTES_PER_MS`]）。
    ///
    /// 判据用**固件自己的 SysTick 计数**（vector 15，1ms/拍）而不是"预算×换算
    /// 常量"：实测"退休字节/SysTick"随代码块混合比浮动 ±4%，用常量换算无法精确
    /// 对齐；按 SysTick 计数推进则天然精确，且把量化过冲（≤1 拍）通过**累计目标**
    /// 带到下一次调用，长期无累积漂移（均值恰为 1ms 固件时间 / 1ms 物理时间）。
    pub fn run_ms(&mut self, ms: f32) -> Result<()> {
        let add = ms.max(0.0).round() as u64;
        if add == 0 {
            return Ok(());
        }
        let now = self.systick_ticks();
        if self.run_ms_target == 0 {
            self.run_ms_target = now; // 首次调用：以当前拍为基准
        }
        self.run_ms_target += add;
        // 小步推进（约 0.5ms 预算）：单次过冲更小；累计目标自动补偿残余。
        let step = (crate::sim::timing::RETIRED_BYTES_PER_MS / 2).max(1);
        // 迭代护栏：SysTick 未推进时（异常停机/时钟被关）不得自旋。
        for _ in 0..64 {
            if self.systick_ticks() >= self.run_ms_target {
                break;
            }
            self.run_budget(step)?;
        }
        Ok(())
    }

    /// 固件 SysTick 已发生的次数（= 固件自身时钟推进的毫秒数）。
    fn systick_ticks(&self) -> u64 {
        // ★§5.107 裁定：**回退到「SysTick 异常进入次数」** ✓ —— 它才是
        //   "**固件自己的毫秒**"（固件正是按 ISR 次数累加 ms ✓），也是 harness 的原口径 ✓。
        //
        // 曾改为「溢出次数」✗（§5.102 ⑤）但**已否证** ✗：
        //   · 模型内溢出数被【上电初期 RVR=0 窗口】灌水 ✗（period=1 ⇒ 每周期都"溢出"）
        //     实测 实收/溢出 = **5236.6** ✗（应 168_000 ✓）⇒ 多记 32× ✗
        //   · 用溢出口径当 fw-ms ⇒ `run_ms` 目标**提前满足** ⇒ 少推进（3000ms 名义只走 859ms ✗）
        // 溢出计数**保留为诊断** ✓（`syst_overflows`/`syst_cycles_in` 探针 ✓），但**不作时基** ✓。
        self.vec_entries
            .borrow()
            .get(15)
            .copied()
            .unwrap_or(0)
    }

    /// ★诊断（§5.112）：(挂起设定次数, 溢出次数) ✓ —— 与 `systick_ms()`（进入次数 ✓）三方对比 ✓。
    pub fn syst_diag_counts(&self) -> (u32, u64) {
        match &self.scb {
            Some(s) => {
                let g = s.lock().unwrap();
                (g.syst_pending_sets(), g.syst_overflows())
            }
            None => (0, 0),
        }
    }

    /// ★诊断（§5.111）：SysTick RVR 写入次数与溢出时刻 RVR 极值 ✓。
    pub fn syst_load_stats(&self) -> (u32, u32, u32) {
        self.scb
            .as_ref()
            .map(|s| s.lock().unwrap().syst_load_stats())
            .unwrap_or((0, 0, 0))
    }

    /// ★诊断（§5.105）：SCB 实收周期累计（应 == `retired_count()` ✓）。
    pub fn scb_cycles_in(&self) -> u64 {
        self.scb
            .as_ref()
            .map(|s| s.lock().unwrap().syst_cycles_in())
            .unwrap_or(0)
    }

    /// 固件自身时钟（SysTick 毫秒）。供 [`crate::clock::McuClock`] 做闭环对齐断言。
    pub fn systick_ms(&self) -> u64 {
        self.systick_ticks()
    }

    pub fn nvic_pending(&self) -> bool {
        self.status.has(BIT_NVIC_PENDING)
    }

    /// MPU 是否已使能（探测/调试用）。
    pub fn mpu_enabled(&self) -> bool {
        self.status.has(BIT_MPU)
    }

    /// 数据访问 hook 是否已安装（MPU 使能懒安装；探测/调试用）。
    pub fn data_hook_active(&self) -> bool {
        self.data_hook_installed.load(Ordering::Relaxed)
    }

    /// run() 外层循环迭代次数（探测 emu_start 是否频繁提前返回）。
    pub fn run_iterations(&self) -> u64 {
        self.run_iterations.get()
    }

    /// 异常入场计数快照（按向量号 0..=96；诊中断暴风归因用）。
    pub fn vec_entries(&self) -> Vec<(u32, u64)> {
        self.vec_entries
            .borrow()
            .iter()
            .enumerate()
            .filter(|(_, n)| **n > 0)
            .map(|(v, n)| (v as u32, *n))
            .collect()
    }

    /// 最近一次异常抢占前的 PC（被中断的块地址）。
    pub fn last_switch_pc(&self) -> u32 {
        self.last_switch_pc.get()
    }

    /// 按**退休字节预算**推进（性能/指令级用）。
    ///
    /// **不是时间语义**：`count` 是“退休字节”预算，与毫秒无固定换算关系（实测同预算的
    /// bytes/ms 随代码块混合比浮动）。时间推进一律用 [`Machine::run_ms`] 或
    /// [`crate::clock::McuClock`]；闭环测试不得直接调本方法。
    pub fn run_budget(&mut self, count: usize) -> Result<()> {
        // 风暴护栏：正常时 count 预算在单次 emu_start 内耗尽即返回；但若某外设/中断
        // 持续可抢占，emu_start 每段都提前返回而 remaining 不递减，本循环会无限自旋
        //（jOS 调度器启动后即可能触发）。达上限时记诊断并提前返回（调用方继续分步）。
        const MAX_ITERS_PER_CALL: u64 = 1_000_000;
        let base_iters = self.run_iterations.get();
        // 预算递减基准：每次 run() 从该点起按 block hook 累计的退休量扣减 count。
        // 【修复】旧实现 `let remaining = count` 恒不递减，每次 emu_start 都拿全额预算：
        // 只有风暴护栏(1M 段)兜底。空闲固件（无 PendSV 风暴）时每段很长（到 SysTick 才被抢，
        // ~25-52K 指令/段），1M 段 × 每段 ~40ms = 永不返回。现在每轮按实际退休量递减，
        // count 耗尽即正常返回；风暴护栏仅作为"段内 0 退休"的异常风暴安全网。
        let retired_base = self.retired_insts.load(Ordering::Relaxed);

        // 虚拟从设备时钟按退役指令数推进（每个 run() 一次）：
        //   dt = Δretired / VIRTUAL_INSNS_PER_SEC（权威常量见 sim::timing）。
        // 口径 = 当前非风暴稳态（~13 段 × 1ms 每 run(400K) → ~30M 指令/虚拟秒）。
        // 段内逐段推进（旧实现每段 dt=0.001）会在中断风暴下自放大：段数膨胀 →
        // 虚拟时间膨胀 → 推流字节膨胀 → 更多中断。改为指令基准后推流速率恒定
        //（GPS 20Hz / SBUS 20Hz，period=0.05s），与中断频率无关。
        // 虚拟从设备/外设推流时钟：必须与**固件自身时钟**同一钟。
        // 旧实现用 `VIRTUAL_INSNS_PER_SEC`(172e6，推流标定口径)，而固件 SysTick 实测
        // ~95.6K~104.5K 字节/ms（`retired/SysTick`）——两者不是同一个钟，且本 `dt`
        // 是**每次 run() 调用**算一次：改用 run_ms（内部分 47800 字节小块）后会改变
        // 从设备落帧粒度 → GPS GGA/RMC 投递被打碎 → 固件解析不到 RMC 速度（gv=0）。
        // 改以固件时钟（退休字节 / RETIRED_BYTES_PER_MS）折算秒，推流时间 ≡ 固件时间，
        // 与 run() 的调用分块无关。
        let retired_now = self.retired_insts.load(Ordering::Relaxed);
        let dt = (retired_now - self.last_virt_retired.get()) as f32
            / (crate::sim::timing::RETIRED_BYTES_PER_MS as f32 * 1000.0);
        self.last_virt_retired.set(retired_now);
        self.step_virtual_slaves(dt);
        self.step_virtual_uart(dt);

        let mut remaining = count;
        while remaining > 0 {
            // Halt 观察点：故障剧本触发后提前返回（调用方检查 halted()）
            if self.halt_requested.get() {
                break;
            }
            // 时间轴故障剧本（P1-1）：段边界按虚拟时间（retired 实时折算）检查到期
            // 事件。段前 + 段后各查一次：段前覆盖"上次段已过观察点"；段后覆盖
            // "本段新退休量越过观察点"（无中断固件单段即跑满预算，仅段前查会漏触发）。
            self.step_fault();
            if self.halt_requested.get() {
                break; // 本段触发 Halt → 不执行本段，立即返回
            }
            let iters = self.run_iterations.get();
            if iters - base_iters >= MAX_ITERS_PER_CALL {
                let pc = self.cpu.reg_read_u32(RegisterARM::PC)?;
                let exc = self
                    .nvic
                    .lock()
                    .unwrap()
                    .current_exception();
                log::warn!(
                    "run() 风暴护栏触发：单调用 {MAX_ITERS_PER_CALL} 段未耗尽 count={count}，\
                     提前返回 pc=0x{pc:08X} exc={exc}"
                );
                break;
            }
            self.run_iterations.set(iters + 1);
            let pc = self.cpu.reg_read_u32(RegisterARM::PC)?;
            self.cpu.emu_start(pc as u64, 0, 0, remaining)?;

            // 看门狗复位请求优先处理：置 CSR 复位标志 + 系统复位（回到复位向量）
            if let Some(reason) = self.wdog_req.take() {
                self.system_reset(reason)?;
                continue;
            }

            // MPU 违规优先返回
            if let Some(f) = self.mpu.lock().unwrap().pending_fault() {
                return Err(CoreError::MemManageFault {
                    addr: f.addr,
                    kind: f.kind,
                });
            }

            // DMA 内存搬运：CPU 空闲间隙执行（tick 已把完成流登记到待搬运位图）
            self.dma.lock().unwrap().process(&mut self.cpu);
            self.dma2.lock().unwrap().process(&mut self.cpu);

            let reason = self.nvic.lock().unwrap().take_stop_reason();
            // 预算耗尽（StopReason::None）时延迟到段后检查完成再退出：
            // 否则 match 内 break 会跳过段后故障检查（无中断固件单段耗尽预算，
            // Halt 观察点/故障注入会漏触发）。
            let mut budget_exhausted = false;
            match reason {
                StopReason::Switch(vector) => {
                    // 诊断：记录被抢占点与按向量入场计数（中断暴风/唤醒停滞归因）
                    if let Ok(pc) = self.cpu.reg_read_u32(RegisterARM::PC) {
                        self.last_switch_pc.set(pc);
                    }
                    self.vec_entries
                        .borrow_mut()
                        .get_mut(vector as usize)
                        .map(|c| *c += 1);
                    self.enter_exception(vector)?;
                }
                StopReason::ExceptionReturn => {}
                StopReason::SvcEntry => {} // SVC 入口已由 intr hook 补全，继续执行 handler
                StopReason::MpuEnable => {
                    // MPU 已使能：懒安装数据访问 hook（含刷 TB），此后内存访问受 MPU 检查
                    self.install_data_access_hook()?;
                }
                // DMA 待搬运：DMA process 已在段后执行，继续消耗剩余预算（不得当作
                // 预算耗尽 break——否则 run(count) 在 DMA 繁忙路径只退休一小段预算，
                // 固件任务周期相对物理步长被拉长 6× 以上，见 StopReason::DmaPending）。
                StopReason::DmaPending => {}
                StopReason::None => budget_exhausted = true, // 达到指令数上限
                StopReason::Breakpoint => {
                    // GDB 断点命中：置标志供调试器消费（继续命令检查）
                    self.gdb_hit.set(true);
                    break;
                }
            }

            // 段后故障检查：本段新退休量可能已越过观察点（Halt 停在本段后）
            self.step_fault();
            if self.halt_requested.get() {
                break;
            }

            // 段后遥测采样（P2-1）：按退休间隔记录观测点时间线
            self.step_telemetry();

            // 按本轮实际退休量递减预算（block hook 已累计；看门狗 continue 分支因未退休指令，
            // 用累计式扣减不受影响——下一轮仍按"本次 run() 起点以来的总退休量"计算）。
            let retired = self.retired_insts.load(Ordering::Relaxed) - retired_base;
            remaining = count.saturating_sub(retired as usize);
            if budget_exhausted {
                break;
            }
        }
        Ok(())
    }

    /// 系统复位（看门狗超时/违规触发）：置 RCC_CSR 复位标志，CPU 回到复位向量。
    ///
    /// 与 [`Machine::reset`] 不同，这里保留已加载的固件（仅重设 SP/PC），
    /// 不重载向量表——复位后固件重新从 Reset_Handler 执行。
    fn system_reset(&mut self, reason: ResetReason) -> Result<()> {
        // 1) 置 RCC_CSR 复位标志（IWDGRSTF/WWDGRSTF），供固件/测试查询复位原因
        self.rcc.lock().unwrap().record_reset(reason);

        // 2) 复位看门狗外设（硬件系统复位会停止/复位看门狗；
        //    否则 IWDG 保持使能且 down=0，复位后每个块立即再次超时 → 死循环）
        self.iwdg.lock().unwrap().reset();
        self.wwdg.lock().unwrap().reset();

        // 3) CPU 回到复位向量（SP/PC 重设，等效硬件复位入口）
        self.reset()?;

        log::info!("看门狗复位：{reason:?} → 系统复位（CSR 复位标志已置位）");
        Ok(())
    }

    /// 异常入栈：保存现场到当前栈，跳转到中断向量，进入 handler 模式。
    ///
    /// 按 ARMv7-M 入栈顺序压 8 字（低地址→高地址）：
    /// r0 r1 r2 r3 r12 LR(被中断现场) PC xPSR；SP -= 32。
    /// 栈选择与 EXC_RETURN：handler 模式恒 MSP(0xFFFFFFF1)；
    /// 线程模式按 CONTROL.SPSEL：MSP(0xFFFFFFF9) 或 PSP(0xFFFFFFFD)。
    ///
    /// EXC_RETURN.FTYPE（bit4）按被中断上下文的 CONTROL.FPCA（bit2）设置：
    /// FPCA=1（任务用过 VFP）→ bit4=0（扩展帧，0xFFFFFFE1/E9/ED），使 RTOS
    /// context.S 的 PendSV 保存/恢复 S0-S31+FPSCR（其判定即 `tst r14,#0x10`）。
    /// 模拟器自身不压 FPU 扩展区——context.S 手动完成 FPU 上下文保存/恢复。
    ///
    /// `vector` 为向量号：可配置系统异常（PendSV=14 / SysTick=15）或外部中断
    /// （IRQ0..81 → vector 16..97，由 block hook 的 select_pending_vector 给出）。
    fn enter_exception(&mut self, vector: u32) -> Result<()> {
        // 外部中断的 IRQ 号（向量号 >= 16 时；系统异常无对应 IRQ）
        let irq = vector.checked_sub(16);

        let in_handler = self.nvic.lock().unwrap().in_handler();
        let control = self.cpu.reg_read_u32(RegisterARM::CONTROL)?;
        let (sp, mut exc_return) = if in_handler {
            (self.cpu.reg_read_u32(RegisterARM::MSP)?, 0xFFFF_FFF1u32)
        } else if control & 2 != 0 {
            (self.cpu.reg_read_u32(RegisterARM::PSP)?, 0xFFFF_FFFD)
        } else {
            (self.cpu.reg_read_u32(RegisterARM::MSP)?, 0xFFFF_FFF9u32)
        };
        // CONTROL.FPCA=1 → 被中断上下文 FPU 活动：EXC_RETURN 置扩展帧（bit4=0），
        // 让 RTOS context.S 在 PendSV 里手动保存/恢复 S0-S31+FPSCR（s16-s31 硬件
        // 从不保存，且模拟器不做懒栈，故必须走 context.S 的 FPU 保存路径）。
        if control & (1 << 2) != 0 {
            exc_return &= !0x10;
        }

        // 采集被中断现场（block hook 停机时 PC 停在块首，返回后该块重放）
        let r0 = self.cpu.reg_read_u32(RegisterARM::R0)?;
        let r1 = self.cpu.reg_read_u32(RegisterARM::R1)?;
        let r2 = self.cpu.reg_read_u32(RegisterARM::R2)?;
        let r3 = self.cpu.reg_read_u32(RegisterARM::R3)?;
        let r12 = self.cpu.reg_read_u32(RegisterARM::R12)?;
        let lr = self.cpu.reg_read_u32(RegisterARM::LR)?;
        let pc_saved = self.cpu.reg_read_u32(RegisterARM::PC)?;
        // xPSR 仅保留 APSR 标志位（bit31..24）；IPSR/EPSR 由本机接管
        let xpsr = self.cpu.reg_read_u32(RegisterARM::XPSR)? & 0xFF00_0000;
        // callee-saved r4..r11：停机在块首，此刻即块首初值。异常返回要重放
        // 被打断的块（PC 恢复为块首），必须同步还原 r4..r11——ISR 链（C 函数
        // prologue/epilogue）虽然自身保存/恢复它们，但恢复的是"打断时刻"的
        // 值，而打断时刻已处于块中间（块内指令改过 r4..r11），与块首初值
        // 不一致 → 重放读到错误基址（实测 r10 变 4 → 读 0x4C fault）。
        let callee = [
            self.cpu.reg_read_u32(RegisterARM::R4)?,
            self.cpu.reg_read_u32(RegisterARM::R5)?,
            self.cpu.reg_read_u32(RegisterARM::R6)?,
            self.cpu.reg_read_u32(RegisterARM::R7)?,
            self.cpu.reg_read_u32(RegisterARM::R8)?,
            self.cpu.reg_read_u32(RegisterARM::R9)?,
            self.cpu.reg_read_u32(RegisterARM::R10)?,
            self.cpu.reg_read_u32(RegisterARM::R11)?,
        ];

        let sp = sp - 32;
        let mut frame = [0u8; 32];
        for (i, v) in [r0, r1, r2, r3, r12, lr, pc_saved, xpsr]
            .iter()
            .enumerate()
        {
            frame[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        self.cpu.mem_write(sp as u64, &frame)?;
        // 选 SP 寄存器：handler 模式恒 MSP；线程模式按 EXC_RETURN bit2（0=MSP 1=PSP），
        // 不依赖 FPCA 修改前的 SPSEL 分支——扩展帧（bit4=0）下 bit2 判定不受影响。
        let sp_reg = if in_handler || exc_return & 0x4 == 0 {
            RegisterARM::MSP
        } else {
            RegisterARM::PSP
        };
        self.cpu.reg_write(sp_reg, sp as u64)?;

        // 进入 handler：LR=EXC_RETURN，IPSR=向量号，PC=向量表[vector]|1
        self.cpu.reg_write(RegisterARM::LR, exc_return as u64)?;
        self.cpu.reg_write(RegisterARM::IPSR, vector as u64)?;
        let handler = u32::from_le_bytes(
            self.cpu
                .mem_read(0x0800_0000 + (vector as u64) * 4, 4)?
                .try_into()
                .unwrap(),
        );
        self.cpu.reg_write(RegisterARM::PC, (handler | 1) as u64)?;

        // NVIC 状态：清挂起（外部中断清 ISPR+置 IABR 活跃；系统异常清 sys_pending）、
        // 压异常栈（表示该异常活跃，供 EXC_RETURN 返回时弹栈）
        let mut n = self.nvic.lock().unwrap();
        if let Some(irq) = irq {
            n.clear_pending(irq);
            n.set_active(irq);
        } else {
            n.clear_sys_pending(vector);
        }
        n.push_exception(vector);
        // PendSV（vector 14）= RTOS 上下文切换：r4..r11 由 context.S 手动
        // 保存旧任务到 TCB / 加载新任务，异常返回后必须保留切换结果，不能
        // 用进入时的 r4..r11 覆盖（否则新任务现场损坏 → 早期启动即 fault）。
        if vector != 14 {
            n.push_callee(callee);
        }

        log::info!(
            "中断进入：vector={vector} handler=0x{handler:08X} EXC_RETURN=0x{exc_return:08X}"
        );
        Ok(())
    }

    /// 加载 ELF 固件：将分配节（.text/.rodata/.data/.bss）写入对应地址。
    /// 约定固件链接地址落在 FLASH/RAM 布局内（调用 [`Machine::map_stm32f407_layout`] 后）。
    pub fn load_elf(&mut self, path: &Path) -> Result<()> {
        let data = std::fs::read(path).map_err(|e| CoreError::Io(e.to_string()))?;
        let file = object::File::parse(&*data).map_err(|e| CoreError::Io(e.to_string()))?;

        for section in file.sections() {
            // 以 ELF SHF_ALLOC 判断节是否占用目标内存（覆盖 .init_array/.fini_array/
            // .ARM.exidx 等 kind 分类为 Unknown/Other 但确实需要加载的节）。
            let is_alloc = match section.flags() {
                SectionFlags::Elf { sh_flags, .. } => sh_flags.contains(elf::SHF_ALLOC),
                _ => {
                    let kind = section.kind();
                    matches!(
                        kind,
                        SectionKind::Text
                            | SectionKind::Data
                            | SectionKind::ReadOnlyData
                            | SectionKind::ReadOnlyDataWithRel
                            | SectionKind::ReadOnlyString
                            | SectionKind::UninitializedData
                    )
                }
            };
            if !is_alloc {
                continue;
            }

            let addr = section.address();
            let size = section.size();
            if size == 0 {
                continue;
            }

            let data = section.data().map_err(|e| CoreError::Io(e.to_string()))?;
            self.cpu.mem_write(addr, data)?;

            // bss 等无文件内容的部分清零
            let rest = size as usize - data.len();
            if rest > 0 {
                self.cpu.mem_write(addr + data.len() as u64, &vec![0u8; rest])?;
            }

            log::info!(
                "ELF 节 {:<16} @ 0x{:08X}  size={:>8}",
                section.name().unwrap_or("?"),
                addr,
                size
            );
        }

        // 对 p_paddr != p_vaddr 的 PT_LOAD 段，把文件内容额外写入 LMA。
        // 固件启动代码（Reset_Handler 的 LoopCopyDataInit）以 FLASH 中 LMA
        // 为源把 .data/.rodata 拷贝到 RAM；若只写 VMA，拷贝源为 0，会覆盖掉
        // 模拟器预置的正确 .data。
        match &file {
            object::File::Elf32(elf) => write_lma_image(&mut self.cpu, elf)?,
            object::File::Elf64(elf) => write_lma_image(&mut self.cpu, elf)?,
            _ => {}
        }

        // 从向量表读取初始 SP 与复位向量（Cortex-M 启动约定）
        let sp = u32::from_le_bytes(
            self.cpu.mem_read(0x0800_0000, 4)?.try_into().unwrap(),
        );
        let entry = u32::from_le_bytes(
            self.cpu.mem_read(0x0800_0004, 4)?.try_into().unwrap(),
        );
        self.initial_sp = sp;
        self.entry = entry;
        log::info!("复位向量：SP=0x{sp:08X}  entry=0x{entry:08X}");
        Ok(())
    }

    /// 加载 App 分区镜像（轨 B 双分区）：把 app.bin 原样写入 APP_FLASH 起点
    /// 0x08060000（XIP 运行，无需重定位）。
    ///
    /// 与 joc-base `app_slot_load_app()` 的契约一致：镜像须以 32B `app_header_t`
    /// 开头（magic=0x41504800 "APH\0" / abi_version / entry 绝对地址 / app_size /
    /// reserved[4]）；系统启动后自举读头、校验 magic/abi_version、清零 App RAM、
    /// 创建 app_host 任务异步调入口。本方法仅负责把镜像放进 Flash 分区。
    pub fn load_app_partition(&mut self, path: &Path) -> Result<()> {
        const APP_FLASH_BASE: u64 = 0x0806_0000;
        const APP_HEADER_MAGIC: u32 = 0x4150_4800;

        let data = std::fs::read(path).map_err(|e| CoreError::Io(e.to_string()))?;
        if data.len() < 32 {
            return Err(CoreError::Io(format!(
                "App 分区镜像过小：{}B（不足 32B 头部）",
                data.len()
            )));
        }
        let magic = u32::from_le_bytes(data[0..4].try_into().unwrap());
        if magic != APP_HEADER_MAGIC {
            return Err(CoreError::Io(format!(
                "App 分区头部 magic 错误：0x{magic:08X}（期望 0x{APP_HEADER_MAGIC:08X}）"
            )));
        }
        let entry = u32::from_le_bytes(data[8..12].try_into().unwrap());
        self.cpu.mem_write(APP_FLASH_BASE, &data)?;
        log::info!(
            "App 分区镜像已写入 0x{APP_FLASH_BASE:08X}: size={} entry=0x{entry:08X}",
            data.len()
        );
        Ok(())
    }

    /// 复位：设置 SP 与 PC（PC 置 Thumb 位）
    pub fn reset(&mut self) -> Result<()> {
        let sp = self.initial_sp;
        let pc = self.entry | 1; // Thumb 位
        self.cpu.reg_write(RegisterARM::SP, sp as u64)?;
        self.cpu.reg_write(RegisterARM::PC, pc as u64)?;
        log::info!("复位：SP=0x{sp:08X}  PC=0x{pc:08X}");
        Ok(())
    }
}

/// 把 PT_LOAD 段内容额外写入 LMA（p_paddr != p_vaddr 时），供固件启动代码
/// 从 FLASH 拷贝 .data/.rodata 到 RAM 使用。见 [`Machine::load_elf`]。
fn write_lma_image<'data, Elf: FileHeader, R: ReadRef<'data>>(
    cpu: &mut Cpu,
    elf: &ElfFile<'data, Elf, R>,
) -> Result<()> {
    for segment in elf.segments() {
        let vaddr = segment.address();
        let data = segment.data().map_err(|e| CoreError::Io(e.to_string()))?;
        let paddr: u64 = segment
            .elf_program_header()
            .p_paddr(segment.elf_file().endian())
            .into();
        if paddr != vaddr && !data.is_empty() {
            log::info!(
                "ELF LMA 段 0x{paddr:08X} <- vaddr 0x{vaddr:08X}  size={}",
                data.len()
            );
            cpu.mem_write(paddr, data)?;
        }
    }
    Ok(())
}

/// 将 Unicorn 内存事件映射为 MPU 数据访问类型（取指由 code hook 单独处理）
fn mem_type_to_access(ty: MemType) -> Option<Access> {
    match ty {
        MemType::READ => Some(Access::Read),
        MemType::WRITE => Some(Access::Write),
        _ => None,
    }
}

/// 当前是否为特权执行：
/// Handler 模式恒特权（IPSR≠0）；Thread 模式由 CONTROL.nPRIV 决定（0=特权）。
fn cpu_privileged(uc: &mut Unicorn<()>) -> bool {
    let ipsr = uc.reg_read(RegisterARM::IPSR).unwrap_or(0);
    if ipsr != 0 {
        return true;
    }
    let control = uc.reg_read(RegisterARM::CONTROL).unwrap_or(0);
    control & 1 == 0
}

/// 记录 MPU 违规并停止执行（违规后该次访问由调用方决定是否阻断）
fn fault_and_stop(uc: &mut Unicorn<()>, mpu: &Arc<Mutex<Mpu>>, f: MemManageFault) {
    mpu.lock().unwrap().record_violation(f);
    let _ = uc.emu_stop();
}

/// 异常返回（intr hook 回调内调用）：EXC_RETURN 时出栈恢复被中断现场。
///
/// 被中断的 8 字帧布局（低地址→高地址）：
/// r0 r1 r2 r3 r12 LR(现场) PC(返回地址) xPSR。
/// 按 EXC_RETURN 解码返回模式与栈：bit3=0 回 handler，bit3=1 回线程
/// （bit2=0 → MSP，bit2=1 → PSP）。
fn exception_return<'b>(
    uc: &mut Unicorn<'b, ()>,
    nvic: &Arc<Mutex<Nvic>>,
    exc_return: u32,
) -> Result<()> {
    let return_to_thread = (exc_return & 0x8) != 0;
    let use_psp = (exc_return & 0x4) != 0;

    // 出栈帧
    let sp_reg = if use_psp {
        RegisterARM::PSP
    } else {
        RegisterARM::MSP
    };
    let sp = uc.reg_read(sp_reg)? as u32;
    let bytes = uc.mem_read_as_vec(sp as u64, 32)?;
    let mut frame = [0u32; 8];
    for (i, slot) in frame.iter_mut().enumerate() {
        *slot = u32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap());
    }
    // frame: [r0 r1 r2 r3 r12 lr pc xpsr]

    // NVIC 状态：弹出异常栈顶、清 active（仅外部中断）
    let mut n = nvic.lock().unwrap();
    let vector = n
        .pop_exception()
        .ok_or_else(|| CoreError::Unicorn("EXC_RETURN 但无活动异常".into()))?;
    if vector >= 16 {
        n.clear_active(vector - 16);
    }
    // 还原被中断现场的 r4..r11（进入时保存的块首初值），保证重放块一致。
    // PendSV（vector 14）除外：上下文切换后的 r4..r11 是 context.S 加载的
    // 新任务现场，pop_callee 只会覆盖它——PendSV 进入时也未 push。
    let callee = if vector != 14 {
        n.pop_callee()
            .ok_or_else(|| CoreError::Unicorn("EXC_RETURN 但无 callee 现场".into()))?
    } else {
        [0; 8]
    };

    // 恢复寄存器
    uc.reg_write(RegisterARM::R0, frame[0] as u64)?;
    uc.reg_write(RegisterARM::R1, frame[1] as u64)?;
    uc.reg_write(RegisterARM::R2, frame[2] as u64)?;
    uc.reg_write(RegisterARM::R3, frame[3] as u64)?;
    uc.reg_write(RegisterARM::R12, frame[4] as u64)?;
    uc.reg_write(RegisterARM::LR, frame[5] as u64)?; // 恢复被中断现场的调用者 LR
    uc.reg_write(RegisterARM::PC, frame[6] as u64)?; // 恢复返回地址（含 Thumb 位）
    let _ = uc.reg_write(RegisterARM::XPSR, frame[7] as u64); // 尽力恢复标志
    if vector != 14 {
        uc.reg_write(RegisterARM::R4, callee[0] as u64)?;
        uc.reg_write(RegisterARM::R5, callee[1] as u64)?;
        uc.reg_write(RegisterARM::R6, callee[2] as u64)?;
        uc.reg_write(RegisterARM::R7, callee[3] as u64)?;
        uc.reg_write(RegisterARM::R8, callee[4] as u64)?;
        uc.reg_write(RegisterARM::R9, callee[5] as u64)?;
        uc.reg_write(RegisterARM::R10, callee[6] as u64)?;
        uc.reg_write(RegisterARM::R11, callee[7] as u64)?;
    }
    uc.reg_write(sp_reg, (sp + 32) as u64)?;
    let control = uc.reg_read(RegisterARM::CONTROL)? as u32;
    let control = if return_to_thread {
        (control & !0x2) | if use_psp { 0x2 } else { 0x0 }
    } else {
        control
    };
    uc.reg_write(RegisterARM::CONTROL, control as u64)?;

    // IPSR：回线程 → 0；回 handler → 上一层异常号（已在异常栈顶）
    let ipsr = if return_to_thread {
        0
    } else {
        n.current_exception()
    };
    let _ = uc.reg_write(RegisterARM::IPSR, ipsr as u64);
    drop(n);

    log::info!(
        "异常返回：EXC_RETURN=0x{exc_return:08X} → PC=0x{:08X}（{}）",
        frame[6],
        if return_to_thread { "回线程" } else { "回 handler" }
    );
    Ok(())
}

/// 外设互联源端（类 Renode `connect` 左侧）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectSource {
    /// USART 端口的 TX 事件
    UartTx(u8),
}

/// 外设互联目标端（类 Renode `connect` 右侧）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectTarget {
    /// 虚拟 Console 接收
    ConsoleRx,
    /// 虚拟终端接收（显示）
    TerminalRx,
}
