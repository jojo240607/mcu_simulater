// M9 demo 固件：RNG 真随机数发生器（STM32F407）验收。
//
// 场景（RNG @ 0x50060800，AHB2）：
//   Phase A：写 CR.RNGEN=1 使能 → 轮询 SR.DRDY 置位 → 读 DR 得 v1 →
//            轮询 SR.DRDY 再次置位（连续生成语义）→ 读 DR 得 v2 →
//            v1 != v2 且 v2 != 0 → G_RNG_OK=1；
//   主线写 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_RNG_OK 使能后两次读取随机值不同且非零（期望 1）
//   0x20000004 G_DONE   主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* RNG @ 0x50060800 */
#define RNG_CR (*(volatile uint32_t *)0x50060800u)
#define RNG_SR (*(volatile uint32_t *)0x50060804u)
#define RNG_DR (*(volatile uint32_t *)0x50060808u)

/* RNG 位 */
#define RNG_CR_RNGEN (1u << 2) /* 随机数发生器使能 */
#define RNG_SR_DRDY  (1u << 0) /* 数据就绪 */

/* 结果区 */
#define G_RNG_OK (*(volatile uint32_t *)0x20000000u)
#define G_DONE   (*(volatile uint32_t *)0x20000004u)

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 向量表：系统异常 + IRQ0..IRQ1 占位（RNG 错误中断不在此固件使用） */
__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    0x20001000u,               /* 0: 初始 SP（SRAM 内） */
    (uint32_t)Reset_Handler,   /* 1: Reset */
    (uint32_t)Default_Handler, /* 2: NMI */
    (uint32_t)Default_Handler, /* 3: HardFault */
    (uint32_t)Default_Handler, /* 4: MemManage */
    (uint32_t)Default_Handler, /* 5: BusFault */
    (uint32_t)Default_Handler, /* 6: UsageFault */
    (uint32_t)Default_Handler, /* 7 */
    (uint32_t)Default_Handler, /* 8 */
    (uint32_t)Default_Handler, /* 9 */
    (uint32_t)Default_Handler, /* 10 */
    (uint32_t)Default_Handler, /* 11 */
    (uint32_t)Default_Handler, /* 12 */
    (uint32_t)Default_Handler, /* 13 */
    (uint32_t)Default_Handler, /* 14 */
    (uint32_t)Default_Handler, /* 15 */
    (uint32_t)Default_Handler, /* 16: IRQ0 */
    (uint32_t)Default_Handler, /* 17: IRQ1 */
};

void Reset_Handler(void) {
    uint32_t v1, v2;

    /* Phase A：使能 → 轮询 DRDY → 连续读两次随机值（应不同且非零） */
    RNG_CR = RNG_CR_RNGEN;
    while ((RNG_SR & RNG_SR_DRDY) == 0u) {} /* 等待数据就绪 */
    v1 = RNG_DR;                             /* 读第一个随机值（清 DRDY 后连续生成） */
    while ((RNG_SR & RNG_SR_DRDY) == 0u) {} /* 连续生成：DRDY 应再次置位 */
    v2 = RNG_DR;                             /* 读第二个随机值 */

    if (v1 != v2 && v2 != 0u) {
        G_RNG_OK = 1u;
    }

    G_DONE = 0xAAAAAAAAu;
    for (;;) {}
}
