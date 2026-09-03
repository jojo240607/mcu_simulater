//! PWR 电源控制（STM32F407，M10 虚拟外设生态）。
//!
//! 低功耗模式控制 + 状态标志（简化但语义自洽，聚焦验收点）：
//! - CR（偏移 0x00）：低功耗模式选择位（LPDS 低功耗深度睡眠 / PDDS 深度睡眠模式
//!   选择）/ PVD 使能 PVDE + 阈值 PLS / 备份域写保护 DBP / 过驱动 ODSW 等可写；
//!   CWUF / CSBF 为写 1 清除（rc_w1）对应 CSR.WUF / CSR.SBF；
//! - CSR（偏移 0x04）：WUF（唤醒标志）/ SBF（待机标志）/ PVDO（PVD 输出）只读，
//!   经 [`Pwr::inject_*`] 注入（模拟外部唤醒/PVD 事件）；EWUP / BRE / BRR 可读写；
//! - 待机唤醒复位路径：固件/测试先 [`Pwr::enter_standby`]（模拟 WFI/WFE 进入待机，
//!   置 CSR.SBF），再 [`Pwr::inject_wakeup`]（WKUP 引脚/外部事件唤醒）→ 置 CSR.WUF
//!   并向共享复位请求（与看门狗同一 [`WdogResetReq`] 链路）发出
//!   [`ResetReason::LowPower`]，Machine 消费后执行系统复位并置 RCC_CSR.LPWRRSTF。
//!
//! 地址映射（PWR @ 0x40007000）：CR 0x00 / CSR 0x04

use std::sync::Arc;

use crate::peripheral::wdog::{ResetReason, WdogResetReq};
use crate::peripheral::{BusError, Peripheral};

/// 寄存器偏移
const OFF_CR: u32 = 0x00;
const OFF_CSR: u32 = 0x04;

/// CR 位（F407）
const CR_LPDS: u32 = 1 << 0; // 低功耗深度睡眠
const CR_PDDS: u32 = 1 << 1; // 深度睡眠模式选择（停止/待机）
const CR_CWUF: u32 = 1 << 2; // 清除唤醒标志（写 1，rc_w1）
const CR_CSBF: u32 = 1 << 3; // 清除待机标志（写 1，rc_w1）
const CR_PVDE: u32 = 1 << 4; // PVD 使能
const CR_PLS: u32 = 0x7 << 5; // PVD 电平选择
const CR_DBP: u32 = 1 << 8; // 禁用备份域写保护
const CR_FPDS: u32 = 1 << 9; // 闪存深度睡眠
const CR_LPLVDS: u32 = 1 << 10; // 低功耗低电压检测
const CR_MRUDS: u32 = 1 << 11; // 主调节器欠压检测
const CR_LPUDS: u32 = 1 << 12; // 低功耗调节器欠压检测
const CR_VOS: u32 = 0x3 << 14; // 调压器输出电压缩放
const CR_ODSWEN: u32 = 1 << 16; // 过驱动使能
const CR_ODSW: u32 = 1 << 17; // 过驱动开关使能
const CR_UDEN: u32 = 1 << 18; // 欠压检测使能
const CR_UDIS: u32 = 1 << 19; // 欠压检测禁用
/// CR 全部可写位（CWUF/CSBF 为 rc_w1，不入镜像）
const CR_WRITABLE: u32 = CR_LPDS | CR_PDDS | CR_PVDE | CR_PLS | CR_DBP | CR_FPDS
    | CR_LPLVDS | CR_MRUDS | CR_LPUDS | CR_VOS | CR_ODSWEN | CR_ODSW | CR_UDEN
    | CR_UDIS;

/// CSR 位（F407）
const CSR_WUF: u32 = 1 << 0; // 唤醒标志（只读）
const CSR_SBF: u32 = 1 << 1; // 待机标志（只读）
const CSR_PVDO: u32 = 1 << 2; // PVD 输出（只读）
const CSR_BRR: u32 = 1 << 3; // 备份域复位（可读写）
const CSR_EWUP: u32 = 1 << 8; // 使能 WKUP 引脚（可读写）
const CSR_BRE: u32 = 1 << 9; // 备份域使能（可读写）
/// CSR 全部可写位（WUF/SBF/PVDO/VOSRDY 只读，不入镜像）
const CSR_WRITABLE: u32 = CSR_BRR | CSR_EWUP | CSR_BRE;

/// PWR 电源控制外设
pub struct Pwr {
    /// CR 镜像（可写位）
    cr: u32,
    /// CSR 镜像（只读标志 + 可写位）
    csr: u32,
    /// 是否已进入待机模式（模拟固件 WFI/WFE 后的状态）
    standby: bool,
    /// 共享复位请求（待机唤醒 → [`ResetReason::LowPower`] 系统复位）
    req: Arc<WdogResetReq>,
}

impl Pwr {
    pub fn new(req: Arc<WdogResetReq>) -> Self {
        Self {
            cr: 0,
            csr: 0,
            standby: false,
            req,
        }
    }

    /// 注入唤醒事件（模拟 WKUP 引脚/外部事件）：
    /// 置 CSR.WUF；若已进入待机则触发低功耗唤醒复位（复用看门狗复位链路）。
    pub fn inject_wakeup(&mut self) {
        self.csr |= CSR_WUF;
        if self.standby {
            self.req.request(ResetReason::LowPower);
        }
    }

    /// 进入待机模式（模拟固件置 PDDS 后执行 WFI/WFE）：
    /// 置 CSR.SBF，此后 [`Pwr::inject_wakeup`] 触发低功耗唤醒复位。
    pub fn enter_standby(&mut self) {
        self.standby = true;
        self.csr |= CSR_SBF;
    }

    /// 注入 PVD 事件（模拟电源电压越过阈值）：on=true 置 CSR.PVDO，false 清除。
    pub fn inject_pvd(&mut self, on: bool) {
        if on {
            self.csr |= CSR_PVDO;
        } else {
            self.csr &= !CSR_PVDO;
        }
    }
}

impl Peripheral for Pwr {
    fn name(&self) -> &str {
        "PWR"
    }

    fn read(&mut self, offset: u32, size: u32) -> Result<u32, BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_CR => Ok(self.cr),
            OFF_CSR => Ok(self.csr),
            _ => Err(BusError::OutOfRange),
        }
    }

    fn write(&mut self, offset: u32, size: u32, value: u32) -> Result<(), BusError> {
        if size != 4 {
            return Err(BusError::NotImplemented);
        }
        match offset {
            OFF_CR => {
                // CWUF/CSBF 写 1 清除对应 CSR 标志（rc_w1，不入镜像）
                if value & CR_CWUF != 0 {
                    self.csr &= !CSR_WUF;
                }
                if value & CR_CSBF != 0 {
                    self.csr &= !CSR_SBF;
                }
                self.cr = value & CR_WRITABLE;
                Ok(())
            }
            OFF_CSR => {
                // 只读标志位保持；仅可写位更新
                self.csr = (self.csr & !CSR_WRITABLE) | (value & CSR_WRITABLE);
                Ok(())
            }
            _ => Err(BusError::OutOfRange),
        }
    }

    fn reset(&mut self) {
        self.cr = 0;
        self.csr = 0;
        self.standby = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make() -> (Pwr, Arc<WdogResetReq>) {
        let req = Arc::new(WdogResetReq::new());
        (Pwr::new(req.clone()), req)
    }

    #[test]
    fn write_low_power_bits_reads_back() {
        let (mut p, _) = make();
        let v = CR_LPDS | CR_PDDS | CR_DBP;
        p.write(OFF_CR, 4, v).unwrap();
        assert_eq!(p.read(OFF_CR, 4).unwrap() & v, v, "低功耗位应写读一致");
    }

    #[test]
    fn cwuf_clears_wuf() {
        let (mut p, _) = make();
        p.inject_wakeup(); // 置 WUF
        assert_ne!(p.read(OFF_CSR, 4).unwrap() & CSR_WUF, 0, "注入唤醒应置 WUF");
        p.write(OFF_CR, 4, CR_CWUF).unwrap(); // 写 1 清 WUF
        assert_eq!(p.read(OFF_CSR, 4).unwrap() & CSR_WUF, 0, "CWUF 应清 WUF");
    }

    #[test]
    fn wakeup_without_standby_no_reset() {
        let (mut p, req) = make();
        p.inject_wakeup();
        assert!(!req.is_pending(), "未进入待机，唤醒不应触发复位");
    }

    #[test]
    fn standby_wakeup_requests_low_power_reset() {
        let (mut p, req) = make();
        p.enter_standby();
        assert_ne!(p.read(OFF_CSR, 4).unwrap() & CSR_SBF, 0, "进入待机应置 SBF");
        p.inject_wakeup();
        assert_eq!(req.take(), Some(ResetReason::LowPower), "待机唤醒应请求 LowPower 复位");
    }

    #[test]
    fn csbf_clears_sbf() {
        let (mut p, _) = make();
        p.enter_standby(); // 置 SBF
        p.write(OFF_CR, 4, CR_CSBF).unwrap(); // 写 1 清 SBF
        assert_eq!(p.read(OFF_CSR, 4).unwrap() & CSR_SBF, 0, "CSBF 应清 SBF");
    }

    #[test]
    fn pvd_injection_toggles_pvdo() {
        let (mut p, _) = make();
        p.inject_pvd(true);
        assert_ne!(p.read(OFF_CSR, 4).unwrap() & CSR_PVDO, 0, "PVD 事件应置 PVDO");
        p.inject_pvd(false);
        assert_eq!(p.read(OFF_CSR, 4).unwrap() & CSR_PVDO, 0, "清除 PVD 应清 PVDO");
    }

    #[test]
    fn csr_writable_bits_and_readonly_flags() {
        let (mut p, _) = make();
        // 写 EWUP/BRE + 尝试覆盖只读标志 WUF
        p.write(OFF_CSR, 4, CSR_EWUP | CSR_BRE | CSR_WUF).unwrap();
        let csr = p.read(OFF_CSR, 4).unwrap();
        assert_ne!(csr & CSR_EWUP, 0, "EWUP 应可写");
        assert_eq!(csr & CSR_WUF, 0, "只读 WUF 不应被写覆盖");
    }

    #[test]
    fn reset_clears_all() {
        let (mut p, _) = make();
        p.enter_standby();
        p.inject_wakeup();
        p.reset();
        assert_eq!(p.read(OFF_CR, 4).unwrap(), 0, "复位后 CR 应清零");
        assert_eq!(p.read(OFF_CSR, 4).unwrap(), 0, "复位后 CSR 应清零");
        assert!(!p.standby);
    }
}
