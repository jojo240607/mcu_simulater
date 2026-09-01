//! Machine：装配 CPU、内存与外设；加载固件。
//!
//! M0 阶段：CPU + 固定板级内存布局 + ELF 加载 + 复位（向量表）。
//! M1 接入内存总线（MMIO 经 mem hook 转发到 Rust 外设），M3 起由配置 DSL 驱动装配。
//! M2 接入 MPU：MMIO/RAM-Flash-CCM 数据访问 + 取指 XN 三入口访问控制。

use std::path::Path;
use std::sync::{Arc, Mutex};

use object::{Object, ObjectSection, SectionKind};
use unicorn_engine::{HookType, MemType, Prot, RegisterARM, Unicorn};

use crate::bus::Bus;
use crate::core::{CoreError, Cpu, Result};
use crate::peripheral::mpu::{Access, MemManageFault, Mpu};
use crate::peripheral::scb::SystemControl;

/// 一台仿真的 MCU
pub struct Machine {
    /// 处理器（Unicorn）
    pub cpu: Cpu,
    /// 内存总线（MMIO 外设注册与分发；hook 闭包持有其克隆）
    pub bus: Arc<Mutex<Bus>>,
    /// MPU（内存保护单元，挂载于 SCB 窗口，访问控制 hook 共享）
    pub mpu: Arc<Mutex<Mpu>>,
    /// 初始 SP（向量表首字）
    pub initial_sp: u32,
    /// 复位向量（向量表第二字，含 Thumb 位处理见 [`Machine::reset`]）
    pub entry: u32,
}

impl Machine {
    /// 创建 Cortex-M4F 机器
    pub fn new_m4f() -> Result<Self> {
        let cpu = Cpu::new_m4f()?;
        Ok(Self {
            cpu,
            bus: Arc::new(Mutex::new(Bus::new())),
            mpu: Arc::new(Mutex::new(Mpu::new())),
            initial_sp: 0,
            entry: 0,
        })
    }

    /// 映射 STM32F407VET6 基础内存布局（FLASH + SRAM1/SRAM2 + CCM + SCB）。
    /// M3 起由 DSL 配置驱动，此处为 M0/M1 固化布局。
    pub fn map_stm32f407_layout(&mut self) -> Result<()> {
        self.cpu.mem_map(0x0800_0000, 0x0008_0000, Prot::ALL)?; // FLASH 512KB
        self.cpu.mem_map(0x2000_0000, 0x0002_0000, Prot::ALL)?; // SRAM1+SRAM2 128KB
        self.cpu.mem_map(0x1000_0000, 0x0001_0000, Prot::ALL)?; // CCM SRAM 64KB
        // 系统控制空间（SCB/NVIC/SysTick/MPU，含 CPACR@0xE000ED88）。
        // 仍映射为普通内存避免读写异常，同时由 mem hook 转发到总线上的 SCB 外设。
        self.cpu.mem_map(0xE000_E000, 0x0000_1000, Prot::ALL)?;
        self.attach_system_control()?;
        Ok(())
    }

    /// 挂载系统控制空间（SCB）与 MPU 访问控制。
    ///
    /// M1 演示 MMIO 完整链路：CPU 访问 0xE000E000..0xE000F000 →
    /// Unicorn mem hook → 内存总线 → SystemControl 外设。
    /// 读：hook 在 CPU 读取前把外设读值注入 RAM（Unicorn 的 MEM_READ 在读取前触发）；
    /// 写：hook 转发到总线，Unicorn 随后照常写 RAM，RAM 视图与总线保持一致。
    ///
    /// M2 在此基础上接入 MPU（默认全强制，保真优先）：
    /// 1. MMIO 数据访问：SCB 转发前先过 MPU 检查；
    /// 2. RAM/Flash/CCM 数据访问：挂 MEM_READ|MEM_WRITE hook，MPU 使能后全强制；
    /// 3. 取指（执行）：挂全范围 code hook 做 XN 检查。
    pub fn attach_system_control(&mut self) -> Result<()> {
        const SCB_BASE: u64 = 0xE000_E000;
        const SCB_SIZE: u32 = 0x1000;

        // SCB 挂接共享 MPU：MPU 寄存器窗口与 MMFSR/MMFAR 由 SCB 委托
        let scb = Arc::new(Mutex::new(SystemControl::new_with_mpu(
            SCB_SIZE,
            self.mpu.clone(),
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

        log::info!("SCB+MPU 已挂载：0x{SCB_BASE:08X} +0x{SCB_SIZE:X}");
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
    /// 若执行期间触发 MPU MemManage fault，返回 [`CoreError::MemManageFault`]。
    pub fn run(&mut self, count: usize) -> Result<()> {
        let pc = self.cpu.reg_read_u32(RegisterARM::PC)?;
        self.cpu.emu_start(pc as u64, 0, 0, count)?;
        if let Some(f) = self.mpu.lock().unwrap().pending_fault() {
            return Err(CoreError::MemManageFault {
                addr: f.addr,
                kind: f.kind,
            });
        }
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
