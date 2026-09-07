/* malloc_demo 链接脚本：与仿真器 map_stm32f407_layout 对齐。
 * SRAM 起始 0x20000000 预留 0x20 字节作为结果区（.data/.bss 从 0x20000020 起）。 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    SRAM  (rwx) : ORIGIN = 0x20000020, LENGTH = 128K - 0x20
}

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text       : { *(.text*) *(.rodata*) } > FLASH
    .data       : { *(.data*) } > SRAM AT> FLASH
    .bss        : { *(.bss*) *(COMMON) } > SRAM
}
