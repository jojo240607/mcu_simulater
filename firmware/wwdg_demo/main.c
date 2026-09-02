// M4 demo 固件：验证 WWDG 窗口看门狗 早期唤醒中断(EWI) + 超时 → 系统复位 → RCC_CSR.WWDGRSTF。
//
// 场景：
// 1. 首次进入 Reset_Handler：使能 IRQ0，配置 WWDG CFR=EWI|W=0x40（WDGTB=0 → ÷4096），
//    启动 CR=WDGA|T=0x41（计数器从 0x41 递减，跨越 0x40 触发 EWI）；
// 2. 自旋不刷新：计数器 0x41→0x40 置 EWIF + 挂起 IRQ0 → IRQ0 handler 置 G_EWI、清 EWIF；
//    再一步 0x40→0x3F（T6 清零）→ 超时 → 复位请求 → 系统复位（置 CSR.WWDGRSTF）；
// 3. 复位后再次进入 Reset_Handler（G_BOOT==2）：读 RCC_CSR 确认 WWDGRSTF → G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_BOOT       进入 Reset_Handler 的次数（期望 2）
//   0x20000004 G_RESET_FLAG 复位后读到的 RCC_CSR（期望含 WWDGRSTF=bit27）
//   0x20000008 G_DONE       完成标记（期望 0xAAAAAAAA）
//   0x2000000C G_EWI        IRQ0(EWI) 中断执行次数（期望 1）

#include <stdint.h>

/* WWDG @ 0x40002C00 */
#define WWDG_CR  (*(volatile uint32_t *)0x40002C00u)
#define WWDG_CFR (*(volatile uint32_t *)0x40002C04u)
#define WWDG_SR  (*(volatile uint32_t *)0x40002C08u)
#define WWDG_CR_WDGA (1u << 7)
#define WWDG_CFR_EWI (1u << 9)

/* NVIC（SCB 基址 0xE000E000）：IRQ0 = WWDG */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_IPR0  (*(volatile uint32_t *)0xE000E400u) /* IRQ0 = 字节0 */

/* RCC @ 0x40023800：CSR @ 0x74，WWDGRSTF = bit27 */
#define RCC_CSR (*(volatile uint32_t *)0x40023874u)
#define CSR_WWDGRSTF (1u << 27)

/* 结果区 */
#define G_BOOT       (*(volatile uint32_t *)0x20000000u)
#define G_RESET_FLAG (*(volatile uint32_t *)0x20000004u)
#define G_DONE       (*(volatile uint32_t *)0x20000008u)
#define G_EWI        (*(volatile uint32_t *)0x2000000Cu)
#define G_SPIN       (*(volatile uint32_t *)0x20000010u)

extern void Reset_Handler(void);
void WWDG_IRQHandler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void wwdg_started_hook(void) __attribute__((noinline));
static void wwdg_started_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + IRQ0..IRQ28（WWDG = IRQ0 → index 16） */
__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    0x20001000u,               /* 0: 初始 SP（SRAM 内） */
    (uint32_t)Reset_Handler,   /* 1: Reset */
    (uint32_t)Default_Handler, /* 2: NMI */
    (uint32_t)Default_Handler, /* 3: HardFault */
    (uint32_t)Default_Handler, /* 4: MemManage */
    (uint32_t)Default_Handler, /* 5: BusFault */
    (uint32_t)Default_Handler, /* 6: UsageFault */
    (uint32_t)Default_Handler, /* 7..15: 保留/系统异常 */
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)WWDG_IRQHandler, /* 16: IRQ0 = WWDG */
    (uint32_t)Default_Handler, /* 17: IRQ1 */
    (uint32_t)Default_Handler, /* 18 */
    (uint32_t)Default_Handler, /* 19 */
    (uint32_t)Default_Handler, /* 20 */
    (uint32_t)Default_Handler, /* 21 */
    (uint32_t)Default_Handler, /* 22 */
    (uint32_t)Default_Handler, /* 23 */
    (uint32_t)Default_Handler, /* 24 */
    (uint32_t)Default_Handler, /* 25 */
    (uint32_t)Default_Handler, /* 26 */
    (uint32_t)Default_Handler, /* 27 */
    (uint32_t)Default_Handler, /* 28 */
    (uint32_t)Default_Handler, /* 29 */
    (uint32_t)Default_Handler, /* 30 */
    (uint32_t)Default_Handler, /* 31 */
    (uint32_t)Default_Handler, /* 32 */
    (uint32_t)Default_Handler, /* 33 */
    (uint32_t)Default_Handler, /* 34 */
    (uint32_t)Default_Handler, /* 35 */
    (uint32_t)Default_Handler, /* 36 */
    (uint32_t)Default_Handler, /* 37 */
    (uint32_t)Default_Handler, /* 38 */
    (uint32_t)Default_Handler, /* 39 */
    (uint32_t)Default_Handler, /* 40 */
    (uint32_t)Default_Handler, /* 41 */
    (uint32_t)Default_Handler, /* 42 */
    (uint32_t)Default_Handler, /* 43 */
    (uint32_t)Default_Handler, /* 44: IRQ28 = TIM2 */
};

void WWDG_IRQHandler(void) {
    G_EWI++;
    WWDG_SR = 0u; /* 写 0 清除 EWIF，避免复位请求前重复进入 */
}

void Reset_Handler(void) {
    G_BOOT++;

    if (G_BOOT >= 2u) {
        /* 看门狗复位后再次进入：读 RCC_CSR 确认 WWDGRSTF → 完成 */
        G_RESET_FLAG = RCC_CSR;
        G_DONE = 0xAAAAAAAAu;
        for (;;) {}
    }

    /* 使能 IRQ0（WWDG），优先级 0 */
    NVIC_IPR0 = 0u;
    NVIC_ISER0 = (1u << 0);
    __asm volatile("cpsie i" ::: "memory");

    /* 配置 WWDG：EWI + 窗口 0x40（WDGTB=0 → ÷4096） */
    WWDG_CFR = WWDG_CFR_EWI | 0x40u;
    /* 启动：WDGA | T=0x41（计数器从 0x41 起，跨越 0x40 触发 EWI，再降超时） */
    WWDG_CR = WWDG_CR_WDGA | 0x41u;

    wwdg_started_hook();

    /* 自旋不刷新：volatile 写保证多基本块，块 tick 反复推进递减计数器 */
    while (1) {
        G_SPIN++;
    }
}
