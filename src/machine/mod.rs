//! Machine：装配 CPU、内存与外设；加载固件。
//!
//! M0 阶段：CPU + 固定板级内存布局 + ELF 加载 + 复位（向量表）。
//! M1 接入内存总线（MMIO 经 mem hook 转发到 Rust 外设），M3 起由配置 DSL 驱动装配。
//! M2 接入 MPU（MMIO/RAM-Flash-CCM 数据访问 + 取指 XN 三入口访问控制）
//!     与中断投递（NVIC 挂起抢占 + 异常入栈/出栈）。

use std::path::Path;
use std::sync::{Arc, Mutex};

use object::{Object, ObjectSection, SectionKind};
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
use crate::peripheral::terminal::Terminal;
use crate::peripheral::dma::{Dma, DmaDir, DMA1_BASE, DMA1_STREAM_IRQ, DMA2_BASE, DMA2_STREAM_IRQ};
use crate::peripheral::exti::{Exti, EXTI_BASE};
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
use crate::sim::timing::VirtualClock;

/// 块级加权周期的平均周期/指令（见 [`crate::sim::timing::BlockWeighted`]）
const AVG_CYCLES_PER_INS: u64 = 3;

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
    /// 共享虚拟时钟（block hook 推进，供 TIM 等外设 tick）
    pub clock: Arc<Mutex<VirtualClock>>,
    /// 时钟外设列表（block hook 按块 tick 推进）
    timers: Arc<Mutex<Vec<Arc<Mutex<dyn Peripheral>>>>>,
    /// DMA1/DMA2（tick 判传输完成；run 间隙 process 执行内存搬运）
    dma: Arc<Mutex<Dma>>,
    dma2: Arc<Mutex<Dma>>,
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
    /// CAN1 控制器局域网（@0x40006400，APB1；邮箱/接收 FIFO/过滤 + CanFrame 总线互联）
    pub can1: Arc<Mutex<Can>>,
    /// CAN2 控制器局域网（@0x40006800，APB1；同上，与 CAN1 互联）
    pub can2: Arc<Mutex<Can>>,
    /// IWDG 独立看门狗（系统复位时复位外设，避免复位后立即再次超时）
    iwdg: Arc<Mutex<Iwdg>>,
    /// WWDG 窗口看门狗（同上）
    wwdg: Arc<Mutex<Wwdg>>,
    /// 看门狗复位请求（IWDG/WWDG/PWR 待机唤醒置位，block hook 停机，run() 执行系统复位）
    pub wdog_req: Arc<WdogResetReq>,
    /// 初始 SP（向量表首字）
    pub initial_sp: u32,
    /// 复位向量（向量表第二字，含 Thumb 位处理见 [`Machine::reset`]）
    pub entry: u32,
}

impl Machine {
    /// 创建 Cortex-M4F 机器
    pub fn new_m4f() -> Result<Self> {
        let cpu = Cpu::new_m4f()?;
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let dma = Arc::new(Mutex::new(Dma::new(nvic.clone(), "DMA1", DMA1_STREAM_IRQ)));
        let dma2 = Arc::new(Mutex::new(Dma::new(nvic.clone(), "DMA2", DMA2_STREAM_IRQ)));
        let wdog_req = Arc::new(WdogResetReq::new());
        let events = Arc::new(Mutex::new(EventBus::new()));
        let pwr = Arc::new(Mutex::new(Pwr::new(wdog_req.clone())));
        let rtc = Arc::new(Mutex::new(Rtc::new(nvic.clone(), pwr.clone())));
        Ok(Self {
            cpu,
            bus: Arc::new(Mutex::new(Bus::new())),
            mpu: Arc::new(Mutex::new(Mpu::new())),
            nvic: nvic.clone(),
            events: events.clone(),
            console: Arc::new(Mutex::new(Console::new())),
            terminal: Arc::new(Mutex::new(Terminal::new(events.clone()))),
            clock: Arc::new(Mutex::new(VirtualClock::new())),
            timers: Arc::new(Mutex::new(Vec::new())),
            dma,
            dma2,
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
            can1: Arc::new(Mutex::new(Can::new(1, Some(events.clone()), nvic.clone()))),
            can2: Arc::new(Mutex::new(Can::new(2, Some(events.clone()), nvic.clone()))),
            iwdg: Arc::new(Mutex::new(Iwdg::new(wdog_req.clone()))),
            wwdg: Arc::new(Mutex::new(Wwdg::new(
                nvic.clone(),
                wdog_req.clone(),
            ))),
            wdog_req,
            initial_sp: 0,
            entry: 0,
        })
    }

    /// 映射 STM32F407VET6 基础内存布局（FLASH + SRAM1/SRAM2 + CCM + SCB + T1 外设区）。
    /// M3 起由 DSL 配置驱动，此处为 M0/M1/M2 固化布局 + M3 T1 外设集。
    pub fn map_stm32f407_layout(&mut self) -> Result<()> {
        self.cpu.mem_map(0x0800_0000, 0x0008_0000, Prot::ALL)?; // FLASH 512KB
        self.cpu.mem_map(0x2000_0000, 0x0002_0000, Prot::ALL)?; // SRAM1+SRAM2 128KB
        self.cpu.mem_map(0x1000_0000, 0x0001_0000, Prot::ALL)?; // CCM SRAM 64KB
        // 系统控制空间（SCB/NVIC/SysTick/MPU，含 CPACR@0xE000ED88）。
        // 仍映射为普通内存避免读写异常，同时由 mem hook 转发到总线上的 SCB 外设。
        self.cpu.mem_map(0xE000_E000, 0x0000_1000, Prot::ALL)?;
        self.attach_system_control()?;
        // M3 T1 外设集：GPIOA-E + USART1-3 + TIM2 + RCC 存根
        self.attach_t1_peripherals()?;
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
            .attach(SCB_BASE as u32, SCB_SIZE, "SCB", scb)?;

        // 1) MMIO 入口：先 MPU 检查，再转发总线
        let bus2 = bus.clone();
        let mpu_mmio = self.mpu.clone();
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
                    }
                    _ => {}
                }
                false // 放行：RAM 视图保持与总线一致
            },
        )?;

        // 2) RAM/Flash/CCM 数据访问入口（MPU 全强制）
        self.attach_data_access_hook()?;

        // 3) 取指 XN 检查入口（code hook）
        self.attach_fetch_xn_hook()?;

        // 4) 中断投递入口（block 检查 + EXC_RETURN 拦截）
        self.attach_interrupt_delivery()?;

        log::info!("SCB+MPU+NVIC 已挂载：0x{SCB_BASE:08X} +0x{SCB_SIZE:X}");
        Ok(())
    }

    /// 挂载外设集：GPIOA-E + USART1-6 + TIM2 + RCC 存根 + SYSCFG/EXTI + 虚拟 Console/Terminal。
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

        // GPIOA-E（port 0..4）
        for port in 0..5u8 {
            let gpio = Arc::new(Mutex::new(Gpio::new(port, events.clone())));
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
        let dac = Arc::new(Mutex::new(Dac::new(1, events.clone())));
        self.bus
            .lock()
            .unwrap()
            .attach(0x4000_7400, 0x400, "DAC", dac.clone())?;
        // 注册 DAC 句柄到 DMA1（内存→外设搬运经句柄直接写 DHR）
        self.dma.lock().unwrap().register_dac(1, dac.clone());
        // 推入时钟外设列表：DAC 无计数语义，tick 仅冲刷定时器触发暂存的电平事件
        self.timers.lock().unwrap().push(dac.clone());

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
                    let stream = match dir {
                        DmaDir::PeriphToMem => 3, // DMA2_Stream3（RX）
                        DmaDir::MemToPeriph => 6, // DMA2_Stream6（TX）
                    };
                    let mut d = dma2_sdio.lock().unwrap();
                    match dir {
                        // 读：FIFO 现成字数一次搬完
                        DmaDir::PeriphToMem => d.service_stream_n(
                            stream,
                            4,
                            *dir,
                            crate::peripheral::dma::DmaTarget::Sdio(*p),
                            *items,
                        ),
                        // 写：items=0，由 service_stream 取 NDTR 一次搬完
                        DmaDir::MemToPeriph => d.service_stream(
                            stream,
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
        // CanFrame 事件 → 路由到对端 CAN（CAN1↔CAN2 互联；feed_rx 只挂 IRQ 不发布，
        // 无事件重入死锁风险）
        let c1 = self.can1.clone();
        let c2 = self.can2.clone();
        events.lock().unwrap().subscribe(Arc::new(Mutex::new(
            move |ev: &Event| {
                if let Event::CanFrame { frame } = ev {
                    match frame.port {
                        1 => c2.lock().unwrap().feed_rx((**frame).clone()),
                        2 => c1.lock().unwrap().feed_rx((**frame).clone()),
                        _ => {}
                    }
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
        let tim_cfgs: &[(u8, u32, &str, TimerKind, u32, u8, TimerIrq)] = &[
            // (port, base, name, kind, bits, channels, irq)
            (1, 0x4001_0000, "TIM1", TimerKind::Advanced, 16, 4,
             TimerIrq { brk: 24, up: 25, trig_com: 26, cc: 27 }),
            (2, 0x4000_0000, "TIM2", TimerKind::General, 32, 4,
             TimerIrq { brk: 28, up: 28, trig_com: 28, cc: 28 }),
            (3, 0x4000_0400, "TIM3", TimerKind::General, 16, 4,
             TimerIrq { brk: 29, up: 29, trig_com: 29, cc: 29 }),
            (4, 0x4000_0800, "TIM4", TimerKind::General, 16, 4,
             TimerIrq { brk: 30, up: 30, trig_com: 30, cc: 30 }),
            (5, 0x4000_0C00, "TIM5", TimerKind::General, 32, 4,
             TimerIrq { brk: 50, up: 50, trig_com: 50, cc: 50 }),
            (6, 0x4000_1000, "TIM6", TimerKind::Basic, 16, 0,
             TimerIrq { brk: 54, up: 54, trig_com: 54, cc: 54 }),
            (7, 0x4000_1400, "TIM7", TimerKind::Basic, 16, 0,
             TimerIrq { brk: 55, up: 55, trig_com: 55, cc: 55 }),
            (8, 0x4001_0400, "TIM8", TimerKind::Advanced, 16, 4,
             TimerIrq { brk: 43, up: 44, trig_com: 45, cc: 46 }),
            (9, 0x4001_4000, "TIM9", TimerKind::General, 16, 2,
             TimerIrq { brk: 24, up: 24, trig_com: 24, cc: 24 }),
            (10, 0x4001_4400, "TIM10", TimerKind::General, 16, 1,
             TimerIrq { brk: 25, up: 25, trig_com: 25, cc: 25 }),
            (11, 0x4001_4800, "TIM11", TimerKind::General, 16, 1,
             TimerIrq { brk: 26, up: 26, trig_com: 26, cc: 26 }),
            (12, 0x4000_1800, "TIM12", TimerKind::General, 16, 2,
             TimerIrq { brk: 43, up: 43, trig_com: 43, cc: 43 }),
            (13, 0x4000_1C00, "TIM13", TimerKind::General, 16, 1,
             TimerIrq { brk: 44, up: 44, trig_com: 44, cc: 44 }),
            (14, 0x4000_2000, "TIM14", TimerKind::General, 16, 1,
             TimerIrq { brk: 45, up: 45, trig_com: 45, cc: 45 }),
        ];
        for (port, base, name, kind, bits, channels, irq) in tim_cfgs {
            let cfg = TimerConfig { name, kind: *kind, bits: *bits, channels: *channels, irq: *irq };
            let tim = Arc::new(Mutex::new(Timer::new(*port, cfg, events.clone(), self.nvic.clone())));
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

        let dma2 = self.dma2.clone();
        self.bus.lock().unwrap().attach(DMA2_BASE, 0x400, "DMA2", dma2.clone())?;
        self.timers.lock().unwrap().push(dma2);

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
                }
            },
        )));

        // M4-看门狗：IWDG（独立，@0x40003000）+ WWDG（窗口，@0x40002C00）
        // 共享复位请求：超时/违规 → block hook 停机 → run() 执行系统复位。
        let iwdg = self.iwdg.clone();
        self.bus.lock().unwrap().attach(0x4000_3000, 0x400, "IWDG", iwdg.clone())?;
        self.timers.lock().unwrap().push(iwdg);

        let wwdg = self.wwdg.clone();
        self.bus.lock().unwrap().attach(0x4000_2C00, 0x400, "WWDG", wwdg.clone())?;
        self.timers.lock().unwrap().push(wwdg);

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
                    if let Event::GpioLevel { port, pin, level } = ev {
                        exti2.lock().unwrap().feed_gpio(*port, *pin, *level);
                    }
                },
            )));
        }

        // 外设区 MMIO 转发 hook（MPU 检查 + 读注入 / 写转发）
        let bus = self.bus.clone();
        let mpu = self.mpu.clone();
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
                    let _ = bus.lock().unwrap().write(addr as u32, size as u32, value as u32);
                }
                _ => {}
            }
            false
        })?;

        log::info!("T1 外设已挂载：GPIOA-E + USART1-6 + I2C1-3 + SPI1-3 + ADC1-3 + TIM1-14 + RCC + SYSCFG/EXTI + DMA1/DMA2 + Console/Terminal @ 0x{periph_base:08X} +0x{periph_size:X}");
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
        // 1) block hook：挂起中断抢占检查 + 块级时钟推进（begin=1,end=0 全范围）
        let nvic = self.nvic.clone();
        let clock = self.clock.clone();
        let timers = self.timers.clone();
        let wdog_req = self.wdog_req.clone();
        self.cpu.add_block_hook(1, 0, move |uc, _addr, size| {
            // 块级加权周期推进虚拟时钟，并 tick 时钟外设（TIM2…，含 IWDG/WWDG）
            let cycles = size as u64 * AVG_CYCLES_PER_INS;
            {
                let mut c = clock.lock().unwrap();
                c.advance(cycles);
                let timers = timers.lock().unwrap();
                for t in timers.iter() {
                    t.lock().unwrap().tick(cycles);
                }
            }
            // 看门狗复位请求：停机（run() 消费请求并执行系统复位）
            if wdog_req.is_pending() {
                let _ = uc.emu_stop();
                return;
            }
            // 挂起中断抢占检查
            let primask = uc.reg_read(RegisterARM::PRIMASK).unwrap_or(0) != 0;
            let basepri = (uc.reg_read(RegisterARM::BASEPRI).unwrap_or(0) & 0xF) as u8;
            let mut n = nvic.lock().unwrap();
            if let Some(irq) = n.select_pending(primask, basepri) {
                n.set_stop_reason(StopReason::Switch(irq));
                let _ = uc.emu_stop();
            }
        })?;

        // 2) intr hook：EXC_RETURN 异常返回（Unicorn 的 do_v7m_exception_exit 被置空，
        //    现场恢复完全由本回调完成：弹出异常栈、按 EXC_RETURN 选栈出栈、恢复寄存器）
        let nvic2 = self.nvic.clone();
        self.cpu.add_intr_hook(move |uc, intno| {
            if intno != 8 {
                return; // 仅处理 EXCP_EXCEPTION_EXIT
            }
            let exc_return = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
            if let Err(e) = exception_return(uc, &nvic2, exc_return) {
                log::error!("异常返回失败：{e:?}");
            }
            nvic2
                .lock()
                .unwrap()
                .set_stop_reason(StopReason::ExceptionReturn);
            let _ = uc.emu_stop();
        })?;

        log::info!("中断投递 hook 已挂载（block 抢占检查 + EXC_RETURN 拦截）");
        Ok(())
    }

    /// RAM/Flash/CCM 数据访问 hook：MPU 使能后全强制（保真优先）。
    ///
    /// 区间覆盖 FLASH(0x08000000)/CCM(0x10000000)/SRAM(0x20000000)，
    /// 不含 SCB（0xE0000000+，由 MMIO hook 单独处理）。
    /// 快速路径：MPU 未使能时直接放行，保持 RAM 无 hook 的原有行为（仅一次判读）。
    fn attach_data_access_hook(&mut self) -> Result<()> {
        const DATA_BEGIN: u64 = 0x0800_0000;
        const DATA_END: u64 = 0x2002_0000; // 覆盖 FLASH/CCM/SRAM，止于 SRAM 末端

        let mpu = self.mpu.clone();
        self.cpu.add_mem_hook(
            HookType::MEM_READ | HookType::MEM_WRITE,
            DATA_BEGIN,
            DATA_END,
            move |uc, ty, addr, _size, _value| {
                let fault = {
                    let m = mpu.lock().unwrap();
                    if !m.is_enabled() {
                        return false; // 快速路径
                    }
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
        log::info!("RAM/Flash/CCM 数据访问 hook 已挂载（MPU 全强制）");
        Ok(())
    }

    /// 取指 XN 检查 code hook：全范围监听每条指令执行地址。
    ///
    /// 仅 MPU 使能后做检查；`begin=1, end=0` 为 Unicorn 全范围约定。
    /// Thumb 位（bit0）在匹配前剥离。
    fn attach_fetch_xn_hook(&mut self) -> Result<()> {
        let mpu = self.mpu.clone();
        self.cpu.add_code_hook(1, 0, move |uc, address, _size| {
            let addr = (address & !1) as u32;
            let fault = {
                let m = mpu.lock().unwrap();
                if !m.is_enabled() {
                    return;
                }
                m.check(addr, Access::Fetch, cpu_privileged(uc)).err()
            };
            if let Some(f) = fault {
                fault_and_stop(uc, &mpu, f);
            }
        })?;
        log::info!("取指 XN 检查 code hook 已挂载");
        Ok(())
    }

    /// 运行 `count` 条指令（从当前 PC 继续）。
    ///
    /// 执行期间：
    /// - 触发 MPU MemManage fault → 返回 [`CoreError::MemManageFault`]；
    /// - 挂起中断抢占（block hook 停机）→ 异常入栈并进入 handler；
    /// - 异常返回（EXC_RETURN，intr hook 停机）→ 现场已恢复，继续；
    /// - 看门狗复位请求（block hook 停机）→ 记录 RCC_CSR 复位标志 + 系统复位，继续。
    /// `count` 以线程模式指令计：每次 emu_start 命中 `remaining` 即结束。
    pub fn run(&mut self, count: usize) -> Result<()> {
        let remaining = count;
        while remaining > 0 {
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
            match reason {
                StopReason::Switch(irq) => self.enter_exception(irq)?,
                StopReason::ExceptionReturn => {}
                StopReason::None => break, // 达到指令数上限
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
    fn enter_exception(&mut self, irq: u32) -> Result<()> {
        let vector = 16 + irq; // 向量号（IRQ0 = vector 16）

        let in_handler = self.nvic.lock().unwrap().in_handler();
        let control = self.cpu.reg_read_u32(RegisterARM::CONTROL)?;
        let (sp, exc_return) = if in_handler {
            (self.cpu.reg_read_u32(RegisterARM::MSP)?, 0xFFFF_FFF1u32)
        } else if control & 2 != 0 {
            (self.cpu.reg_read_u32(RegisterARM::PSP)?, 0xFFFF_FFFD)
        } else {
            (self.cpu.reg_read_u32(RegisterARM::MSP)?, 0xFFFF_FFF9u32)
        };

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

        let sp = sp - 32;
        let mut frame = [0u8; 32];
        for (i, v) in [r0, r1, r2, r3, r12, lr, pc_saved, xpsr]
            .iter()
            .enumerate()
        {
            frame[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        self.cpu.mem_write(sp as u64, &frame)?;
        let sp_reg = if in_handler || exc_return == 0xFFFF_FFF9 {
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

        // NVIC 状态：清挂起、置活跃、压异常栈
        let mut n = self.nvic.lock().unwrap();
        n.clear_pending(irq);
        n.set_active(irq);
        n.push_exception(vector);
        drop(n);

        log::info!(
            "中断进入：IRQ{irq} → vector={vector} handler=0x{handler:08X} EXC_RETURN=0x{exc_return:08X}"
        );
        Ok(())
    }

    /// 加载 ELF 固件：将分配节（.text/.rodata/.data/.bss）写入对应地址。
    /// 约定固件链接地址落在 FLASH/RAM 布局内（调用 [`Machine::map_stm32f407_layout`] 后）。
    pub fn load_elf(&mut self, path: &Path) -> Result<()> {
        let data = std::fs::read(path).map_err(|e| CoreError::Io(e.to_string()))?;
        let file = object::File::parse(&*data).map_err(|e| CoreError::Io(e.to_string()))?;

        for section in file.sections() {
            let kind = section.kind();
            let is_alloc = matches!(
                kind,
                SectionKind::Text
                    | SectionKind::Data
                    | SectionKind::ReadOnlyData
                    | SectionKind::ReadOnlyDataWithRel
                    | SectionKind::ReadOnlyString
                    | SectionKind::UninitializedData
            );
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
/// 按 EXC_RETURN 解码返回模式与栈：bit1=0 回 handler（MSP），bit1=1 回线程
/// （bit2=0 → MSP，bit2=1 → PSP）。
fn exception_return<'b>(
    uc: &mut Unicorn<'b, ()>,
    nvic: &Arc<Mutex<Nvic>>,
    exc_return: u32,
) -> Result<()> {
    let return_to_thread = exc_return & 0x2 != 0;
    let use_psp = exc_return & 0x4 != 0;

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

    // 恢复寄存器
    uc.reg_write(RegisterARM::R0, frame[0] as u64)?;
    uc.reg_write(RegisterARM::R1, frame[1] as u64)?;
    uc.reg_write(RegisterARM::R2, frame[2] as u64)?;
    uc.reg_write(RegisterARM::R3, frame[3] as u64)?;
    uc.reg_write(RegisterARM::R12, frame[4] as u64)?;
    uc.reg_write(RegisterARM::LR, frame[5] as u64)?; // 恢复被中断现场的调用者 LR
    uc.reg_write(RegisterARM::PC, frame[6] as u64)?; // 恢复返回地址（含 Thumb 位）
    let _ = uc.reg_write(RegisterARM::XPSR, frame[7] as u64); // 尽力恢复标志
    uc.reg_write(sp_reg, (sp + 32) as u64)?;

    // CONTROL.SPSEL：返回线程时按 EXC_RETURN bit2 更新；返回 handler 时不改
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
