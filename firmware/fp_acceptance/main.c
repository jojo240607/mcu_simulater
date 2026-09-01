// M0 验收固件：Cortex-M4F 裸机（无 RTOS/无 libc）
// 目标：验证仿真器能跑通含浮点运算（VFP）的代码。
//
// 流程：使能 FPU -> 执行 vadd/vmul -> vcvt 转整数后写入固定 SRAM 地址 -> 死循环。
// 仿真器校验 0x20000000 处数值应为 12（0x0000000C，即 12.0f 经 vcvt.u32.f32）。

#include <stdint.h>

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* Cortex-M 向量表：前两字为初始 SP 与复位向量（仿真器从这里取复位信息） */
__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    0x20001000u,               /* 初始 SP（SRAM 内） */
    (uint32_t)Reset_Handler,   /* 复位向量 */
    (uint32_t)Default_Handler, /* NMI */
    (uint32_t)Default_Handler, /* HardFault */
};

static void enable_fpu(void) {
    /* CPACR (0xE000ED88)：CP10/CP11 置为全权限，启用 VFP */
    volatile uint32_t *cpacr = (volatile uint32_t *)0xE000ED88u;
    *cpacr = (*cpacr & ~(0xFu << 20)) | (0xFu << 20);
    __asm volatile("dsb" ::: "memory");
    __asm volatile("isb" ::: "memory");
}

void Reset_Handler(void) {
    enable_fpu();

    volatile float a = 1.5f;
    volatile float b = 2.5f;
    volatile float c = a + b;    /* vadd.f32 -> 4.0f */
    volatile float d = c * 3.0f; /* vmul.f32 -> 12.0f */

    /* 结果写入固定地址供仿真器校验 */
    *(volatile uint32_t *)0x20000000u = (uint32_t)d;

    for (;;) {}
}
