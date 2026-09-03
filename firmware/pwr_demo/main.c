// M10 demo 固件：PWR 电源控制（STM32F407）验收。
//
// 场景（PWR @ 0x40007000）：
//   Phase A：写 CR 低功耗位（LPDS|PDDS）→ 读回校验一致 → G_PWR_CR_OK=1；
//   Phase B：写 CSR 可写位（EWUP|BRE）→ 读回校验一致 → G_PWR_CSR_OK=1；
//   写 G_PWR_READY=1（低功耗位已设置，等待测试注入 WKUP 唤醒→待机唤醒复位）；
//   复位后再次进入 Reset_Handler：读 RCC_CSR 确认 LPWRRSTF（bit31）→ G_PWR_WOKE=1、
//   G_RESET_FLAG=CSR；读 PWR_CSR 确认 SBF（bit1）保持 → G_PWR_SBF=1；
//   主线写 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_PWR_CR_OK  低功耗位写读一致（期望 1）
//   0x20000004 G_PWR_CSR_OK CSR 可写位写读一致（期望 1）
//   0x20000008 G_PWR_READY  低功耗位已设置、等待唤醒（期望 1）
//   0x2000000C G_PWR_WOKE   待机唤醒复位后检测到 LPWRRSTF（期望 1）
//   0x20000010 G_PWR_SBF    待机唤醒复位后 PWR_CSR.SBF 保持（期望 1）
//   0x20000014 G_RESET_FLAG 复位后读到的 RCC_CSR（期望含 LPWRRSTF=bit31）
//   0x20000018 G_DONE       主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* PWR @ 0x40007000 */
#define PWR_CR  (*(volatile uint32_t *)0x40007000u)
#define PWR_CSR (*(volatile uint32_t *)0x40007004u)

/* PWR 位 */
#define PWR_CR_LPDS (1u << 0) /* 低功耗深度睡眠 */
#define PWR_CR_PDDS (1u << 1) /* 深度睡眠模式选择（停止/待机） */
#define PWR_CSR_EWUP (1u << 8) /* 使能 WKUP 引脚 */
#define PWR_CSR_BRE  (1u << 9) /* 备份域使能 */
#define PWR_CSR_SBF  (1u << 1) /* 待机标志（只读） */

/* RCC @ 0x40023800：CSR @ 0x74，LPWRRSTF = bit31 */
#define RCC_CSR (*(volatile uint32_t *)0x40023874u)
#define CSR_LPWRRSTF (1u << 31)

/* 结果区 */
#define G_PWR_CR_OK  (*(volatile uint32_t *)0x20000000u)
#define G_PWR_CSR_OK (*(volatile uint32_t *)0x20000004u)
#define G_PWR_READY  (*(volatile uint32_t *)0x20000008u)
#define G_PWR_WOKE   (*(volatile uint32_t *)0x2000000Cu)
#define G_PWR_SBF    (*(volatile uint32_t *)0x20000010u)
#define G_RESET_FLAG (*(volatile uint32_t *)0x20000014u)
#define G_DONE       (*(volatile uint32_t *)0x20000018u)

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 向量表：系统异常 + IRQ0..IRQ1 占位（PWR 无中断） */
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
    uint32_t cr, csr;

    /* 复位后再进：检测 LPWRRSTF（待机唤醒复位） */
    if (RCC_CSR & CSR_LPWRRSTF) {
        G_PWR_WOKE = 1u;
        G_RESET_FLAG = RCC_CSR;
        /* 待机唤醒复位后 PWR_CSR.SBF 应保持置位（指示上次来自待机） */
        if (PWR_CSR & PWR_CSR_SBF) {
            G_PWR_SBF = 1u;
        }
        G_DONE = 0xAAAAAAAAu;
        for (;;) {}
    }

    /* Phase A：写低功耗位 → 读回校验一致 */
    PWR_CR = PWR_CR_LPDS | PWR_CR_PDDS;
    cr = PWR_CR;
    if ((cr & (PWR_CR_LPDS | PWR_CR_PDDS)) == (PWR_CR_LPDS | PWR_CR_PDDS)) {
        G_PWR_CR_OK = 1u;
    }

    /* Phase B：写 CSR 可写位（EWUP|BRE）→ 读回校验一致 */
    PWR_CSR = PWR_CSR_EWUP | PWR_CSR_BRE;
    csr = PWR_CSR;
    if ((csr & (PWR_CSR_EWUP | PWR_CSR_BRE)) == (PWR_CSR_EWUP | PWR_CSR_BRE)) {
        G_PWR_CSR_OK = 1u;
    }

    /* 低功耗位已设置；等待测试注入 WKUP 唤醒 → 待机唤醒复位 */
    G_PWR_READY = 1u;

    G_DONE = 0xAAAAAAAAu;
    for (;;) {}
}
