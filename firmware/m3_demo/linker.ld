/* STM32F407VET6 基础内存布局（与仿真器 map_stm32f407_layout 对齐） */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text       : { *(.text*) *(.rodata*) } > FLASH
    .data       : { *(.data*) } > SRAM AT> FLASH
    .bss        : { *(.bss*) *(COMMON) } > SRAM
}
