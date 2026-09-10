//! joc-drvtest-app 整机验收：系统分区 + 驱动验证 App（双分区）。
//!
//! 验证目标（App 内部 21 个用例，覆盖 15 组驱动）：
//!   k_sdk（tick/msleep/信号量/互斥量/设备表）、d_uart、d_gpio、d_adc、d_temp、
//!   d_rng、d_crc（与软件模型逐值比对）、d_rtc、d_timer（溢出 + App ISR）、
//!   d_exti（共享线）、d_pwm、d_dac、d_spi_i2c、d_dma。
//!
//! 里程碑（控制台日志）：
//!   mounted = "RUST app mounted"   App 分区挂载成功
//!   hb      = "hb n="              心跳任务推进（SysTick 周期唤醒链路）
//!   report  = "DRVTEST REPORT total=.. pass=.. fail=.. skip=.."  用例跑完
//! 断言：fail == 0（任何驱动用例失败即验收失败）；INSN_INVALID 兜底判失败。
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use unicorn_engine::RegisterARM;
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::vperiph::data_source::StaticImu;
use mcu_simulater::peripheral::vperiph::esc::{Esc, EscConfig, EscMotor};
use mcu_simulater::peripheral::vperiph::i2c::{StaticToF, Vl53l1x};
use mcu_simulater::peripheral::vperiph::spi::{Bmi088, Pwm3901, SpiFlash, StaticFlow};

#[test]
fn drvtest_all_drivers_pass() {
    let elf = Path::new(r"/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf");
    let app = Path::new(r"/home/ubuntu/work/joc-drvtest-app/app.bin");
    let mut m = Machine::new_m4f().unwrap();
    m.map_stm32f407_layout().unwrap();
    m.load_elf(&elf).unwrap();
    m.load_app_partition(&app).unwrap();
    m.reset().unwrap();

    // SPI 虚拟从机：BMI088 挂 SPI2（板级 g_bmi088 依赖 "spi2"），ACCEL_CS=GPIOE7、
    // GYRO_CS=GPIOE8（port4/pin7,8）。固件 bmi088 驱动经 GPIO 拉低 CS + SPI XFER
    // 全链路读传感器数据；须在 run() 前挂（系统分区启动时 bmi088_create 会 open
    // spi2 并读 WHO_AM_I 校验——从机此时必须已挂载）。
    m.register_spi_slave(3, Box::new(Bmi088::new((4, 7), (4, 8), StaticImu::default())));

    // SPI NOR Flash 虚拟从机：挂 SPI1（板级 spi_flash0 依赖 "spi1" = SPI2 硬件），
    // CS=GPIOE9（port4/pin9）。绑定唯一临时文件——固件写数据后宿主机侧
    // persist_spi_slaves() 把映像写回，据此证明"保存"数据跨 run 存活。
    let flash_path = std::env::temp_dir().join(format!("drvtest_spiflash_{}.bin", std::process::id()));
    let _ = std::fs::remove_file(&flash_path);
    m.register_spi_slave(2, Box::new(SpiFlash::with_file(64 * 1024, (4, 9), flash_path.clone())));

    // ESC 电调 + 无刷电机虚拟外设（信号观测，非总线从设备）：
    //   - PWM 电调输入：pwm0 = TIM3_CH1（port 3/ch 0，400Hz/2500μs 周期）
    //     60% 占空比 → 脉宽 1500μs → 电调量 1000 → 转速 6000 RPM（max 12000）
    //   - DShot 电调输入：dshot0 = GPIOE_10（port 4/pin 10）bit-bang 油门 1000
    let esc_pwm = Arc::new(Mutex::new(Box::new(Esc::new(EscConfig {
        name: "esc_pwm",
        pwm_port: Some(3),
        pwm_channel: Some(0),
        pwm_period_us: 2500,
        ..Default::default()
    })) as Box<dyn EscMotor>));
    let esc_dshot = Arc::new(Mutex::new(Box::new(Esc::new(EscConfig {
        name: "esc_dshot",
        dshot_port: Some(4),
        dshot_pin: Some(10),
        ..Default::default()
    })) as Box<dyn EscMotor>));
    m.register_esc(esc_pwm.clone());
    m.register_esc(esc_dshot.clone());

    // PMW3901 光流：与 BMI088 共享 SPI3 总线（port 3），CS=GPIOE_11（port4/pin11）。
    // 验证同总线多从机按 CS 路由（bmi088 用 PE7/8，pmw3901 用 PE11）。
    // 默认模型：dx=1.0px（256）、dy=0.5px（128）、squal=120。
    m.register_spi_slave(3, Box::new(Pwm3901::new((4, 11), StaticFlow::default())));
    // VL53L1X ToF：I2C 0x29 挂 i2c0（port 1，与 mpu6050/bmp280/qmc5883 同总线）。
    // 默认模型：距离 500mm。
    m.register_i2c_slave(1, Box::new(Vl53l1x::new(StaticToF::new(500))));

    // INSN_INVALID 兜底：任何非法指令直接判失败（回归守卫）。
    let bad_pc = Arc::new(AtomicU32::new(0));
    let got_invalid = Arc::new(AtomicBool::new(false));
    let (b2, g2) = (bad_pc.clone(), got_invalid.clone());
    m.cpu
        .raw()
        .add_insn_invalid_hook(move |uc| {
            let pc = uc.reg_read(RegisterARM::PC).unwrap_or(0) as u32;
            b2.store(pc, Ordering::Relaxed);
            g2.store(true, Ordering::Relaxed);
            eprintln!("[INSN_INVALID] pc=0x{pc:08X}");
            false
        })
        .unwrap();

    let t_start = std::time::Instant::now();
    let mut mounted = false;
    let mut hb = false;
    let mut report: Option<(u32, u32, u32, u32)> = None; // (total, pass, fail, skip)
    // 虚拟 USB 主机注入（C 类 usb 真实主机通信，逻辑同 cfg-run 两阶段握手）：
    // READY → 总线复位 + 分步标准枚举；ENUM-OK → OUT EP1 注入 64B 模式数据。
    let mut usb_stage1 = false;
    let mut usb_stage2 = false;
    let mut usb_setup_idx: usize = 0;
    const USB_SETUPS: [[u8; 8]; 4] = [
        [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00], // GET_DESCRIPTOR(Device)
        [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x20, 0x00], // GET_DESCRIPTOR(Config)
        [0x00, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00], // SET_ADDRESS 0x2A
        [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00], // SET_CONFIGURATION 1
    ];
    // 21 个用例含若干 msleep（timer 溢出 120ms、exti 死线轮询等）→ 虚拟时间预算
    // 需 ~2.5s；每步 400K 字节 ≈ 2.4ms 虚拟，留 5000 步（~12s 虚拟）充足余量。
    for step in 0..5600u32 {
        if t_start.elapsed().as_secs() > 300 {
            eprintln!(">>> 超时（300s）终止");
            break;
        }
        let r = m.run(400_000);
        let pc = m.cpu.reg_read_u32(RegisterARM::PC).unwrap();
        let text = {
            let outv = m.console.lock().unwrap().output().to_vec();
            String::from_utf8_lossy(&outv).into_owned()
        };
        if text.contains("RUST app mounted") {
            mounted = true;
        }
        if text.contains("hb n=") {
            hb = true;
        }
        // 虚拟 USB 主机注入（分步，逻辑同 cfg-run；见函数头注释）
        if !usb_stage1 || !usb_stage2 {
            if !usb_stage1 && text.contains("DRVTEST-USB-HOST-READY") {
                m.usb_otg.lock().unwrap().inject_usb_reset();
                usb_stage1 = true;
                eprintln!("[usb-host] READY → 注入总线复位");
            }
            if usb_stage1 && usb_setup_idx < USB_SETUPS.len() {
                let (rx_empty, dmsk) = {
                    let u = m.usb_otg.lock().unwrap();
                    (u.rx_status_empty(), u.daintmsk())
                };
                let ready = if usb_setup_idx == 0 { dmsk != 0 } else { rx_empty };
                if ready {
                    m.usb_otg
                        .lock()
                        .unwrap()
                        .inject_setup(USB_SETUPS[usb_setup_idx]);
                    eprintln!("[usb-host] 注入 SETUP #{}", usb_setup_idx);
                    usb_setup_idx += 1;
                }
            }
            if !usb_stage2 && text.contains("DRVTEST-USB-ENUM-OK") {
                let pattern: Vec<u8> = (0..64).map(|i| 0x55u8 + i as u8).collect();
                m.usb_otg.lock().unwrap().inject_out(1, &pattern);
                usb_stage2 = true;
                eprintln!("[usb-host] ENUM-OK → OUT EP1 注入 64B 模式数据");
            }
        }
        // 解析 "DRVTEST REPORT total=28 pass=28 fail=0 skip=0"
        if report.is_none() {
            for line in text.lines() {
                if let Some(rest) = line.strip_prefix("R/I/drvtest: DRVTEST REPORT") {
                    let mut t = 0u32;
                    let mut p = 0u32;
                    let mut f = 0u32;
                    let mut s = 0u32;
                    for kv in rest.split_whitespace() {
                        let mut it = kv.split('=');
                        if let (Some(k), Some(v)) = (it.next(), it.next()) {
                            let n = v.parse::<u32>().unwrap_or(0);
                            match k {
                                "total" => t = n,
                                "pass" => p = n,
                                "fail" => f = n,
                                "skip" => s = n,
                                _ => {}
                            }
                        }
                    }
                    if t > 0 {
                        report = Some((t, p, f, s));
                    }
                }
            }
        }
        if step % 5 == 0 || (report.is_none() && mounted) {
            eprintln!(
                "[step {step}] pc=0x{pc:08X} mounted={mounted} hb={hb} report={report:?} console={} iters={}",
                text.len(),
                m.run_iterations()
            );
        }
        if got_invalid.load(Ordering::Relaxed) {
            eprintln!("!!! INSN_INVALID 出现 @0x{:08X}", bad_pc.load(Ordering::Relaxed));
            break;
        }
        match r {
            Err(e) => {
                eprintln!("ERR {e:?} pc=0x{pc:08X}");
                break;
            }
            Ok(()) => {}
        }
        if mounted && hb && report.is_some() {
            break;
        }
    }
    let out = m.console.lock().unwrap().output().to_vec();
    println!("=== console ({}B) ===\n{}", out.len(), String::from_utf8_lossy(&out));
    println!("=== end ===");
    eprintln!("RESULT: mounted={mounted} hb={hb} report={report:?}");
    assert!(mounted, "App 分区未挂载（无 RUST app mounted）");
    assert!(hb, "心跳未出现（hb n=）：SysTick 周期唤醒链路异常？");
    let (total, pass, fail, skip) = report.expect("未出现 DRVTEST REPORT（用例卡死？）");
    assert!(total >= 20, "用例总数异常（total={total}）");
    assert_eq!(fail, 0, "存在驱动用例失败（fail={fail}，pass={pass}，skip={skip}）");
    assert_eq!(pass, total - skip, "pass 数不吻合（total={total} pass={pass} skip={skip}）");

    // 宿主字节级判据（C 类 uart DMA TX 真机判据）：uart0 DMA TX 的模式串必须
    // 以完整 48 字节序列出现在虚拟主机（console）的原始接收缓冲里——证明数据
    // **真实从 TX 发出**（到达终端侧），而非仅写入 USART 数据寄存器。模式串与
    // d_uart::uart0_dma_tx_real 完全一致：18 字节标记头 + 30 字节递增字节。
    {
        let mut pattern: Vec<u8> = b"DRVTEST-DMA-TX-REAL:".to_vec();
        // 与 d_uart::uart0_dma_tx_real 完全一致：20 字节标记头 + 28 字节递增
        //（App 侧 skip(hdr.len())=skip(20)，首递增字节 i=20 → 0x9C）。
        for i in 20u8..48 {
            pattern.push(i.wrapping_mul(7).wrapping_add(0x10));
        }
        let found = out.windows(pattern.len()).any(|w| w == pattern.as_slice());
        assert!(found, "宿主未捕获到 uart0 DMA TX 模式串（{pattern:?}）——数据未真实发出");
        eprintln!(">>> 宿主捕获 uart0 DMA TX 模式串（{}B，字节级一致）", pattern.len());
    }
    // d_bmi088 全链路数值验收：WHO_AM_I 片选区分（1E/0F）+ 原始计数值（accel.z≈10920）
    // + 模拟器从机真实被访问（固件确实与虚拟从机交换字节，而非仅寄存器回读）。
    {
        let bmi_ok_who = String::from_utf8_lossy(&out).contains("WHO accel=0x1E gyro=0x0F");
        let bmi_ok_raw = String::from_utf8_lossy(&out).contains("az=10920");
        let access = m.spi.lock().unwrap()[2].lock().unwrap().slave_access();
        eprintln!(">>> [BMI088-DIAG] access={access} who_str={bmi_ok_who} raw_str={bmi_ok_raw}");
        assert!(bmi_ok_who, "BMI088 WHO_AM_I 断言未命中（片选区分失败？）");
        assert!(bmi_ok_raw, "BMI088 accel.z≈10920 断言未命中（数据回填异常？）");
        assert!(access > 0, "SPI2 虚拟从机未被固件访问（access={access}）——链路未打通");
        eprintln!(">>> BMI088 全链路：WHO=1E/0F ✓ accel.z=10920 ✓ 从机访问 {access} 字节 ✓");
    }
    // d_spi_flash 全链路验收：JEDEC=EF4018（固件 open 校验已隐含）+ 写读回 + 擦除
    // + 持久化标记；宿主机侧调用 persist_spi_slaves() 后校验文件映像含标记与数据
    //（证明固件写入经 SPI 落到可保存的文件，数据跨 run 存活）。
    {
        let acc = m.spi.lock().unwrap()[1].lock().unwrap().slave_access();
        let has_jedec = String::from_utf8_lossy(&out).contains("JEDEC=0xEF4018 正确");
        let has_wr = String::from_utf8_lossy(&out).contains("写读回 8B 一致");
        let has_erase = String::from_utf8_lossy(&out).contains("扇区擦除后读回全 0xFF");
        let has_marker = String::from_utf8_lossy(&out).contains("持久化标记 @0x300 已写入");
        eprintln!(">>> [FLASH-DIAG] access={acc} jedec={has_jedec} wr={has_wr} erase={has_erase} marker={has_marker}");
        assert!(has_jedec && has_wr && has_erase && has_marker,
            "SPI NOR Flash 用例断言未命中（jedec={has_jedec} wr={has_wr} erase={has_erase} marker={has_marker}）");
        assert!(acc > 0, "SPI1 虚拟从机未被固件访问（access={acc}）——flash 链路未打通");

        // 持久化：触发映像写回文件，校验文件包含固件写入的特征串（跨 run 保存）
        m.persist_spi_slaves();
        let img = std::fs::read(&flash_path).expect("flash 持久化文件应存在");
        let text = String::from_utf8_lossy(&img);
        assert!(text.contains("SPIFLASH-SAVE"), "持久化文件缺少固件写入的特征串（保存失效）");
        // 写读回数据段（0x2000 起 8B：DE AD BE EF 01 23 45 67；扇区 2，未被擦除）
        assert_eq!(&img[0x2000..0x2008], &[0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67],
            "持久化文件 0x2000 写读回数据不一致");
        // 擦除扇区 0（0x0000-0x0FFF）后：0x0200 应回 0xFF（擦除生效）；
        // 0x0300 标记在同扇区但为擦除后写入 → 存活；0x2000 数据跨扇区存活。
        assert_eq!(&img[0x0200..0x0204], &[0xFF; 4], "0x0200 所在扇区擦除后应为 0xFF");
        assert!(text.contains("SPIFLASH-SAVE"), "持久化文件标记仍在（擦除后写入存活）");
        eprintln!(">>> SPI NOR Flash 全链路：JEDEC=EF4018 ✓ 写读回 ✓ 擦除 ✓ 持久化文件含标记 ✓（{}B 映像）", img.len());
    }
    // d_esc 全链路验收（模拟器 ESC 虚拟外设按事件解码）：
    //   PWM 电调输入：pwm0 60% 占空比（脉宽 1500μs）→ 电调量 1000 → 转速 6000 RPM；
    //   DShot 电调输入：dshot0 发送油门 1000 → 电调量 1000，CRC 0 错、帧 ≥1。
    {
        let pwm = esc_pwm.lock().unwrap();
        let dshot = esc_dshot.lock().unwrap();
        let out_str = String::from_utf8_lossy(&out);
        let has_pwm_str = out_str.contains("pwm0 60% duty 已输出");
        let has_dshot_str = out_str.contains("dshot0 油门 1000 已发送");
        eprintln!(
            ">>> [ESC-DIAG] pwm_throttle={} rpm={:.0} dshot_throttle={} frames={} crc_err={} pwms={}",
            pwm.throttle(), pwm.rpm(), dshot.throttle(), dshot.dshot_frames(), dshot.crc_errors(), has_pwm_str
        );
        assert!(has_pwm_str, "d_esc pwm_throttle 用例未命中（PWM 链路未跑通）");
        assert!(has_dshot_str, "d_esc dshot_throttle 用例未命中（DShot 链路未跑通）");
        // 模拟器虚拟时钟按基本块量化（事件 tick = 块起点退役字节），PWM 脉宽测量
        // 存在 ≤1% 的块对齐误差（如 995/1000）；DShot 为比例解码不受影响（精确 1000）。
        let t_err = (pwm.throttle() as i32 - 1000).abs();
        assert!(t_err <= 20, "PWM 电调量应≈1000（实测 {}，块量化误差 ≤1%）", pwm.throttle());
        assert!((pwm.rpm() - 6000.0).abs() < 120.0, "PWM 电机转速应≈6000 RPM（实测 {:.0}）", pwm.rpm());
        assert_eq!(dshot.throttle(), 1000, "DShot 电调量应为 1000（固件发送油门 1000）");
        assert!(dshot.dshot_frames() >= 1, "DShot 应至少解码 1 帧（frames={}）", dshot.dshot_frames());
        assert_eq!(dshot.crc_errors(), 0, "DShot CRC 不应出错（errors={}）", dshot.crc_errors());
        eprintln!(">>> ESC 全链路：PWM 60%→电调量1000/转速6000 ✓ DShot 油门1000/CRC0 错 ✓");
    }
    // d_pmw3901 / d_vl53l1x 全链路验收（模拟器虚拟从机回送真实数据）：
    //   PMW3901：Product_ID=0x49、Delta_X=256（1.0px 8.8 定点）、Delta_Y=128、
    //   SQUAL=120、Motion 就绪位；与 bmi088 共享 SPI3 且按 CS 区分（多从机路由）
    //   VL53L1X：WHO_AM_I=0xEA、测距 500mm、清中断；I2C 0x29 挂 i2c0
    {
        let out_str = String::from_utf8_lossy(&out);
        let has_pid = out_str.contains("Product_ID=0x49 正确");
        let has_delta = out_str.contains("dx=256 dy=128");
        let has_who = out_str.contains("WHO_AM_I=0xEA 正确");
        let has_mm = out_str.contains("distance=500mm 正确");
        eprintln!(">>> [SENSOR-DIAG] pid={has_pid} delta={has_delta} who={has_who} mm={has_mm}");
        assert!(has_pid && has_delta, "PMW3901 用例断言未命中（pid={has_pid} delta={has_delta}）");
        assert!(has_who && has_mm, "VL53L1X 用例断言未命中（who={has_who} mm={has_mm}）");
        // 模拟器从机真实被访问：SPI3（port 3）总访问 = bmi088 + pmw3901；I2C1 读计数 > 0
        let spi3_access = m.spi.lock().unwrap()[2].lock().unwrap().slave_access();
        assert!(spi3_access > 0, "SPI3 总线未被固件访问（access={spi3_access}）");
        let i2c1_reads: u64 = m.i2c.lock().unwrap()[0].lock().unwrap().slaves().iter()
            .map(|s| s.read_count()).sum();
        assert!(i2c1_reads > 0, "I2C1 总线读计数为 0（VL53L1X 链路未打通）");
        eprintln!(">>> PMW3901（SPI3 共享总线多从机）+ VL53L1X（I2C 0x29）全链路 ✓（SPI3 访问 {spi3_access}B，I2C1 读 {i2c1_reads} 次）");
    }
    eprintln!(">>> 验收通过：{pass}/{total} 通过，{skip} 跳过，0 失败");
}
