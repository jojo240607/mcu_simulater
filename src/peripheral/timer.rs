//! TIM2 通用定时器（STM32F407，M3 T1 集）。
//!
//! M3 语义：按 [`Peripheral::tick`] 推进的虚拟周期驱动 CNT 递增
//! （块级加权周期，见 [`crate::sim::timing`]），计数溢出时置 SR.UIF，
//! 若 DIER.UIE 使能则向共享 NVIC 置挂起 IRQ28（TIM2 中断），
//! 由 Machine 的 block hook / 中断投递链路完成响应。
//!
//! M6 扩展（DMA 模式，对齐 USART/I2C/SPI/ADC 外设↔内存搬运语义）：
//! - 更新事件且 DIER.UDE（bit8）使能 → 发布 [`crate::events::Event::TimUpdate`]，
//!   由 Machine 路由到 DMA1_Stream5_Channel5（TIM2_UP，HAL 默认流）；
//! - 内存→外设：DMA 把内存表经 [`Tim2::dma_write_dr`] 写入 DMAR，DCR.DBA/DBL
//!   指定突发目标寄存器（DBA=字偏移，DBL=突发长度，按序号回绕）——经典用法
//!   如更新事件逐拍把 CCR 表装入 TIM2 生成波形；
//! - 外设→内存：DMA 经 [`Tim2::dma_read_dr`] 从 DMAR 读出（同样按 DBA/DBL 突发）。
//!
//! 地址映射（offset 相对 TIM2 基址 0x40000000）：
//! - CR1 0x00（CEN=bit0, UDIS=bit1）/ DIER 0x0C（UIE=bit0, UDE=bit8）
//! - SR 0x10（UIF=bit0，写 0 清除）/ EGR 0x14（UG=bit0，软件更新）
//! - CNT 0x24 / PSC 0x28 / ARR 0x2C / CCR1-4 0x34-0x40
//! - DCR 0x48（DBL=bit4:0, DBA=bit12:8）/ DMAR 0x4C

use std::sync::{Arc, Mutex};

use crate::events::{Event, EventBus};
use crate::peripheral::nvic::Nvic;
use crate::peripheral::{BusError, Peripheral};

/// STM32F407 上 TIM2 的中断号
pub const TIM2_IRQ: u32 = 28;

/// CR1 控制位
const CR1_CEN: u32 = 1 << 0; // 计数器使能
/// DIER 中断/请求使能位
const DIER_UIE: u32 = 1 << 0; // 更新中断使能
const DIER_UDE: u32 = 1 << 8; // 更新 DMA 请求使能
/// SR 状态位
const SR_UIF: u32 = 1 << 0; // 更新标志

/// 寄存器偏移
const OFF_CR1: u32 = 0x00;
const OFF_DIER: u32 = 0x0C;
const OFF_SR: u32 = 0x10;
const OFF_EGR: u32 = 0x14;
const OFF_CNT: u32 = 0x24;
const OFF_PSC: u32 = 0x28;
const OFF_ARR: u32 = 0x2C;
const OFF_DCR: u32 = 0x48;
// DMAR @ 0x4C：通用寄存器文件覆盖（idx 19），DMA 经 dma_read_dr/dma_write_dr 访问

/// 寄存器文件数（CR1..DMAR，含保留位，共 20 个 32 位寄存器）
const REG_COUNT: usize = 20;

/// DCR 位段
const DCR_DBL_MASK: u32 = 0x1F; // DBL[4:0]：突发长度
const DCR_DBA_SHIFT: u32 = 8;   // DBA[4:0]：突发基址（字偏移）

/// TIM2 外设
pub struct Tim2 {
    /// 端口号（TIM2 = 2，用于事件过滤）
    pub port: u8,
    /// 寄存器文件（CR1..DMAR 共 20 个 32 位寄存器）
    regs: [u32; REG_COUNT],
    /// 计数器时钟余数（PSC 分频的亚周期累积）
    prescaler_remainder: u64,
    /// DMA 突发序号（DMAR 读/写按 DBA + 序号 % DBL 寻址目标寄存器）
    dma_burst_index: u32,
    /// 事件总线（更新事件 + UDE → 发布 TimUpdate，供 DMA 请求路由）
    bus: Arc<Mutex<EventBus>>,
    /// 共享 NVIC（更新事件 → 置挂起 IRQ28）
    nvic: Arc<Mutex<Nvic>>,
}

impl Tim2 {
    pub fn new(port: u8, bus: Arc<Mutex<EventBus>>, nvic: Arc<Mutex<Nvic>>) -> Self {
        Self {
            port,
            regs: [0; REG_COUNT],
            prescaler_remainder: 0,
            dma_burst_index: 0,
            bus,
            nvic,
        }
    }

    /// 生成一次更新事件：置 UIF，UIE 使能时向 NVIC 置挂起；
    /// UDE 使能时发布 [`Event::TimUpdate`]（更新事件 → DMA 请求）。
    ///
    /// 注意：tick/寄存器写路径不持有事件总线锁（发布在 Machine 的事件分发之外），
    /// 因此可以直接 publish，无需像 feed_rx 那样改由订阅者二次路由。
    fn update_event(&mut self) {
        self.regs[OFF_SR as usize / 4] |= SR_UIF;
        let dier = self.regs[OFF_DIER as usize / 4];
        if dier & DIER_UIE != 0 {
            self.nvic.lock().unwrap().set_pending(TIM2_IRQ);
        }
        if dier & DIER_UDE != 0 {
            self.bus.lock().unwrap().publish(&Event::TimUpdate { port: self.port });
        }
    }

    /// 当前 CNT 值（测试读取用）
    pub fn count(&self) -> u32 {
        self.regs[OFF_CNT as usize / 4]
    }

    /// DMA 读 DMAR（外设→内存方向）：按 DCR.DBA/DBL 突发返回目标寄存器值。
    ///
    /// 供 DMA 控制器搬运调用（每次读推进突发序号，DBL 内回绕）。
    pub fn dma_read_dr(&mut self) -> u32 {
        let idx = self.dma_target_idx();
        self.regs.get(idx).copied().unwrap_or(0)
    }

    /// DMA 写 DMAR（内存→外设方向）：按 DCR.DBA/DBL 突发写入目标寄存器。
    ///
    /// 供 DMA 控制器搬运调用；DBA=13 起始即 CCR1..CCR4 依次装入，
    /// 经典用法"更新事件 → DMA 突发装载 CCR 表生成波形"。
    pub fn dma_write_dr(&mut self, value: u32) {
        let idx = self.dma_target_idx();
        if idx < REG_COUNT {
            self.regs[idx] = value;
        }
    }

    /// 突发目标寄存器索引 = DBA + (序号 % DBL)，DBA/DBL 取自 DCR。
    fn dma_target_idx(&mut self) -> usize {
        let dcr = self.regs[OFF_DCR as usize / 4];
        let dbl = ((dcr & DCR_DBL_MASK) as usize).max(1); // DBL=0 → 1 次
        let dba = ((dcr >> DCR_DBA_SHIFT) & 0x1F) as usize;
        let idx = dba + (self.dma_burst_index as usize % dbl);
        self.dma_burst_index = self.dma_burst_index.wrapping_add(1);
        idx
    }
}

/// DMA 外设方向搬运接口实现（复用 inherent `dma_read_dr`/`dma_write_dr` 语义）。
impl crate::peripheral::dma::DmaByteIo for Tim2 {
    fn dma_read_dr(&mut self) -> u32 {
        self.dma_read_dr()
    }

    fn dma_write_dr(&mut self, value: u32) {
        self.dma_write_dr(value);
    }
}

impl Peripheral for Tim2 {
    fn name(&self) -> &str {
        "TIM2"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        let idx = (offset / 4) as usize;
        self.regs.get(idx).copied().ok_or(BusError::OutOfRange)
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_EGR => {
                // UG：软件生成更新事件
                if value & 1 != 0 {
                    self.update_event();
                    // 向上计数更新事件：CNT 回 0
                    self.regs[OFF_CNT as usize / 4] = 0;
                    self.prescaler_remainder = 0;
                }
                Ok(())
            }
            OFF_SR => {
                // 状态位写 0 清除（rc_w0）
                self.regs[OFF_SR as usize / 4] &= value;
                Ok(())
            }
            _ => {
                let idx = (offset / 4) as usize;
                let slot = self.regs.get_mut(idx).ok_or(BusError::OutOfRange)?;
                *slot = value;
                Ok(())
            }
        }
    }

    fn tick(&mut self, cycles: u64) {
        // 未使能或禁用更新（UDIS）时不推进
        let cr1 = self.regs[OFF_CR1 as usize / 4];
        if cr1 & CR1_CEN == 0 {
            return;
        }
        let psc = (self.regs[OFF_PSC as usize / 4] & 0xFFFF) as u64 + 1;
        let arr = self.regs[OFF_ARR as usize / 4];

        // 分频：累积周期，按 (PSC+1) 折算计数器步进
        self.prescaler_remainder += cycles;
        let steps = self.prescaler_remainder / psc;
        self.prescaler_remainder %= psc;
        if steps == 0 {
            return;
        }

        let mut cnt = self.regs[OFF_CNT as usize / 4] as u64 + steps;
        // 向上计数：超过 ARR 即溢出回绕，生成更新事件
        while cnt > arr as u64 {
            cnt -= arr as u64 + 1;
            self.update_event();
        }
        self.regs[OFF_CNT as usize / 4] = cnt as u32;
    }

    fn reset(&mut self) {
        self.regs = [0; REG_COUNT];
        self.prescaler_remainder = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::events::EventBus;
    use crate::peripheral::dma::DmaByteIo;

    fn tim2() -> (Tim2, Arc<Mutex<Nvic>>, Arc<Mutex<EventBus>>) {
        let nvic = Arc::new(Mutex::new(Nvic::new()));
        let bus = Arc::new(Mutex::new(EventBus::new()));
        (Tim2::new(2, bus.clone(), nvic.clone()), nvic, bus)
    }

    #[test]
    fn overflow_sets_pending_and_wraps() {
        let (mut t, nvic, _) = tim2();
        // PSC=1（分频 2），ARR=100
        t.write(OFF_PSC, 4, 1).unwrap();
        t.write(OFF_ARR, 4, 100).unwrap();
        t.write(OFF_DIER, 4, DIER_UIE).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();

        // 未使能时不推进
        let (mut t2, _, _) = tim2();
        t2.write(OFF_ARR, 4, 10).unwrap();
        t2.tick(1000);
        assert_eq!(t2.count(), 0);

        // 使能后：1000 周期 / 2 = 500 步，ARR=100 → 4 次溢出，CNT=500-4*101=96
        t.tick(1000);
        assert_eq!(t.count(), 96);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF);
        assert!(nvic.lock().unwrap().is_pending(TIM2_IRQ), "UIE 使能应置挂起");

        // 写 SR 清 UIF（写 0 清除）
        t.write(OFF_SR, 4, 0).unwrap();
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, 0);
    }

    #[test]
    fn disabled_update_event_no_pending() {
        let (mut t, nvic, _) = tim2();
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap(); // CEN 但 UIE 未使能
        t.tick(1000);
        assert_eq!(t.read(OFF_SR, 4).unwrap() & SR_UIF, SR_UIF);
        assert!(!nvic.lock().unwrap().is_pending(TIM2_IRQ), "UIE 未使能不置挂起");
    }

    #[test]
    fn ude_publishes_tim_update_event() {
        let (mut t, nvic, bus) = tim2();
        let got = Arc::new(Mutex::new(Vec::new()));
        let g = got.clone();
        bus.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::TimUpdate { port } = ev {
                    g.lock().unwrap().push(*port);
                }
            })));
        // UDE 使能 → 更新事件发布 TimUpdate
        t.write(OFF_ARR, 4, 10).unwrap();
        t.write(OFF_DIER, 4, DIER_UDE).unwrap();
        t.write(OFF_CR1, 4, CR1_CEN).unwrap();
        t.tick(1000);
        assert!(!got.lock().unwrap().is_empty(), "UDE 使能应发布 TimUpdate");
        // UDE 未使能 → 不发布
        let (mut t2, _, bus2) = tim2();
        let got2 = Arc::new(Mutex::new(Vec::new()));
        let g2 = got2.clone();
        bus2.lock()
            .unwrap()
            .subscribe(Arc::new(Mutex::new(move |ev: &Event| {
                if let Event::TimUpdate { .. } = ev {
                    g2.lock().unwrap().push(1);
                }
            })));
        t2.write(OFF_ARR, 4, 10).unwrap();
        t2.write(OFF_CR1, 4, CR1_CEN).unwrap();
        t2.tick(1000);
        assert!(got2.lock().unwrap().is_empty(), "UDE 未使能不发布 TimUpdate");
    }

    #[test]
    fn dma_burst_writes_ccr_table() {
        let (mut t, _, _) = tim2();
        // DCR：DBA=13（CCR1 字偏移），DBL=4（CCR1..CCR4）
        t.write(OFF_DCR, 4, (13 << 8) | 4).unwrap();
        // DMA 突发写 4 字：依次装入 CCR1..CCR4
        for v in [0x1111u32, 0x2222, 0x3333, 0x4444] {
            t.dma_write_dr(v);
        }
        assert_eq!(t.read(0x34, 4).unwrap(), 0x1111); // CCR1
        assert_eq!(t.read(0x38, 4).unwrap(), 0x2222); // CCR2
        assert_eq!(t.read(0x3C, 4).unwrap(), 0x3333); // CCR3
        assert_eq!(t.read(0x40, 4).unwrap(), 0x4444); // CCR4
        // 第 5 次写回绕到 CCR1
        t.dma_write_dr(0x5555);
        assert_eq!(t.read(0x34, 4).unwrap(), 0x5555);
    }

    #[test]
    fn dma_burst_reads_back_table() {
        let (mut t, _, _) = tim2();
        t.write(OFF_DCR, 4, (13 << 8) | 4).unwrap();
        for (i, v) in [0xAAAAu32, 0xBBBB, 0xCCCC, 0xDDDD].iter().enumerate() {
            t.write(0x34 + i as u32 * 4, 4, *v).unwrap();
        }
        let mut out = Vec::new();
        for _ in 0..4 {
            out.push(t.dma_read_dr());
        }
        assert_eq!(out, vec![0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD]);
    }
}
