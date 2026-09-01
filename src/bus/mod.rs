//! 内存总线：按地址区间将 CPU 访问分发给 RAM 或外设。
//!
//! - RAM/Flash 区域直接 map 给 Unicorn，不经总线（快路径）
//! - MMIO 外设区域：M1 起由 Unicorn mem hook 转发到本总线，再分发到外设
//!
//! M0 仅实现总线的区间注册与读写分发，MMIO hook 转发在 M1 接入。

use std::sync::{Arc, Mutex};

use crate::peripheral::{BusError, Peripheral};

/// 一个地址区间及其归属设备
pub struct Region {
    /// 区间基址
    pub base: u32,
    /// 区间大小
    pub size: u32,
    /// 区间名（调试用）
    pub name: String,
    /// 归属外设
    pub device: Arc<Mutex<dyn Peripheral>>,
}

/// 内存总线
#[derive(Default)]
pub struct Bus {
    regions: Vec<Region>,
}

impl Bus {
    pub fn new() -> Self {
        Self::default()
    }

    /// 注册一个 MMIO 外设区间
    pub fn attach(
        &mut self,
        base: u32,
        size: u32,
        name: impl Into<String>,
        device: Arc<Mutex<dyn Peripheral>>,
    ) -> Result<(), BusError> {
        // 简单重叠检查：新区间不得与已注册区间重叠
        if self.regions.iter().any(|r| {
            base < r.base + r.size && r.base < base + size
        }) {
            return Err(BusError::NotImplemented); // 占位：后续用明确错误类型
        }
        self.regions.push(Region {
            base,
            size,
            name: name.into(),
            device,
        });
        Ok(())
    }

    /// 按地址查找区间，返回 `(区间, 区间内偏移)`
    fn lookup(&self, addr: u32) -> Result<(&Region, u32), BusError> {
        for r in &self.regions {
            if addr >= r.base && addr < r.base + r.size {
                return Ok((r, addr - r.base));
            }
        }
        Err(BusError::Unmapped(addr))
    }

    /// 读：将访问分发给对应外设
    pub fn read(&self, addr: u32, size: u32) -> Result<u32, BusError> {
        let (r, offset) = self.lookup(addr)?;
        r.device.lock().unwrap().read(offset, size)
    }

    /// 写：将访问分发给对应外设
    pub fn write(&self, addr: u32, size: u32, value: u32) -> Result<(), BusError> {
        let (r, offset) = self.lookup(addr)?;
        r.device.lock().unwrap().write(offset, size, value)
    }

    /// 复位所有外设
    pub fn reset(&self) {
        for r in &self.regions {
            r.device.lock().unwrap().reset();
        }
    }

    /// 已注册区间列表（调试/monitor 用）
    pub fn regions(&self) -> &[Region] {
        &self.regions
    }
}
