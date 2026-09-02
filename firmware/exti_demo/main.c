// M4 demo 固件：验证 EXTI 外部中断端到端链路
// （GPIO 事件 → EXTI 沿检测 → NVIC IRQ6 → handler）。
//
// 外部输入源由仿真器测试注入：测试在 block hook 里向事件总线发布
// PA0（port 0 / pin 0）的 GpioLevel 事件模拟外部驱动（上升沿）。
//
// 流程：
// 1. RCC 使能 GPIOA / SYSCFG 时钟（寄存器镜像）；
// 2. PA5 配置为输出（LED，handler 中翻转）；PA0 保持输入（EXTI 输入线）；
// 3. SYSCFG_EXTICR1 = 0 → EXTI0 接 GPIOA（复位默认，显式写出）；
// 4. EXTI0：RTSR=1（上升沿触发）、IMR=1（开放中断）；
// 5. NVIC IRQ6（EXTI0）优先级 + 使能；
// 6. 主线轮询 G_EXTI，达 4 后写完成标记 G_DONE。
//
// EXTI0_IRQHandler：清 PR → G_EXTI++ → BSRR 翻转 PA5。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_EXTI EXTI0 handler 执行次数（期望 4）
//   0x20000004 G_LED  PA5 翻转次数（期望 4）
//   0x20000008 G_DONE 主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830u) /* bit0 = GPIOA 时钟 */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844u) /* bit14 = SYSCFG 时钟 */

/* GPIOA @ 0x40020000 */
#define GPIOA_MODER (*(volatile uint32_t *)0x40020000u)
#define GPIOA_BSRR  (*(volatile uint32_t *)0x40020018u)

/* SYSCFG @ 0x40013800 */
#define SYSCFG_EXTICR1 (*(volatile uint32_t *)0x40013808u) /* EXTI0-3 端口选择 */

/* EXTI @ 0x40013C00 */
#define EXTI_IMR   (*(volatile uint32_t *)0x40013C00u)
#define EXTI_RTSR  (*(volatile uint32_t *)0x40013C08u)
#define EXTI_PR    (*(volatile uint32_t *)0x40013C14u)

/* NVIC（SCB 基址 0xE000E000） */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_IPR1  (*(volatile uint32_t *)0xE000E404u) /* IRQ4-7：IRQ6 = 字节2 */

/* 结果区（固定 SRAM 地址） */
#define G_EXTI (*(volatile uint32_t *)0x20000000u)
#define G_LED  (*(volatile uint32_t *)0x20000004u)
#define G_DONE (*(volatile uint32_t *)0x20000008u)

extern void Reset_Handler(void);
void EXTI0_IRQHandler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void exti_ready_hook(void) __attribute__((noinline));
static void exti_ready_hook(void) {
    __asm volatile("nop" ::: "memory");
}

static void exti_done_hook(void) __attribute__((noinline));
static void exti_done_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + IRQ0..IRQ28（index 16..44；EXTI0 = IRQ6 → index 22） */
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
    (uint32_t)Default_Handler, /* 16: IRQ0 */
    (uint32_t)Default_Handler, /* 17: IRQ1 */
    (uint32_t)Default_Handler, /* 18 */
    (uint32_t)Default_Handler, /* 19 */
    (uint32_t)Default_Handler, /* 20 */
    (uint32_t)Default_Handler, /* 21 */
    (uint32_t)EXTI0_IRQHandler, /* 22: IRQ6 = EXTI0 */
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

void EXTI0_IRQHandler(void) {
    if (EXTI_PR & 1u) {
        EXTI_PR = 1u; /* 写 1 清除 PR 位 */
        G_EXTI++;
        /* BSRR 翻转 PA5（发布 GpioLevel 事件） */
        GPIOA_BSRR = (G_EXTI & 1u) ? (1u << 5) : (1u << 21);
        G_LED++;
    }
}

void Reset_Handler(void) {
    /* 1. 使能时钟（寄存器镜像） */
    RCC_AHB1ENR = 1u;      /* GPIOA */
    RCC_APB2ENR = (1u << 14); /* SYSCFG */

    /* 2. PA5 输出（PA0 默认输入，作为 EXTI0 输入线） */
    GPIOA_MODER = 1u << (5 * 2);

    /* 3. EXTI0 接 GPIOA（复位默认 0，显式写出） */
    SYSCFG_EXTICR1 = 0u;

    /* 4. EXTI0：上升沿触发 + 开放中断 */
    EXTI_RTSR = 1u;
    EXTI_IMR = 1u;

    /* 5. IRQ6（EXTI0）优先级 + 使能；IPR1 字节2（bits16..23）= 15 */
    NVIC_IPR1 = 0xFu << 16;
    NVIC_ISER0 = (1u << 6);
    __asm volatile("cpsie i" ::: "memory");

    exti_ready_hook();

    /* 6. 主循环：等待 4 次 EXTI0 中断（handler 递增 G_EXTI） */
    while (G_EXTI < 4u) {}
    exti_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
