/* 链接脚本：STM32F407 内存布局（与其余 demo 固件一致） */
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
