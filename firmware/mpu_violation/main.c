/* M2 演示固件：自编程 MPU 后触发一次数据访问违规（写只读区域）。
 *
 * 流程：配置 region0 = [0x20000000, +64B) 特权只读（AP=101）→ 使能 MPU
 *       （ENABLE|PRIVDEFENA）→ 向 0x20000000 写入 → 触发 MemManage 违规。
 * 仿真器通过 run() 返回 CoreError::MemManageFault，用于观察实际报错输出。
 */

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

/* MPU 寄存器（STM32F407 SCB 内，绝对地址） */
#define MPU_CTRL (*(volatile uint32_t *)0xE000ED94u)
#define MPU_RBAR (*(volatile uint32_t *)0xE000ED9Cu)
#define MPU_RASR (*(volatile uint32_t *)0xE000EDA0u)

void Reset_Handler(void) {
    /* region0 = [0x20000000, +64B)，AP=101（特权只读），ENABLE */
    MPU_RBAR = 0x20000000u | (1u << 4); /* VALID, REGION=0 */
    MPU_RASR = (5u << 1) | (0b101u << 24) | 1u;
    /* ENABLE + PRIVDEFENA：后台 region 特权放行，隔离本违规的干扰源 */
    MPU_CTRL = 0x1u | 0x4u;

    /* 写只读区域 → 触发 MPU 数据访问违规（DACCVIOL @ 0x20000000） */
    *(volatile uint32_t *)0x20000000u = 0xDEADBEEFu;

    for (;;) {}
}
