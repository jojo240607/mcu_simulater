//! 配置 DSL 解析（类 Renode 脚本，M3 启用，M0 仅占位）。
//!
//! 目标语法示例：
//! ```text
//! machine create stm32f407vet6
//! cpu add cortex-m4f freq 168MHz
//! memory add FLASH 0x08000000 0x80000
//! peripheral add usart1 @ 0x40011000 size 0x400 irq=37
//! connect usart1.tx -> console.rx
//! load elf @ firmware/fw.elf
//! start
//! ```
