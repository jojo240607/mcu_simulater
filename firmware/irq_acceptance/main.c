// M2 IRQ 验收固件：验证 NVIC 中断投递链路
// （挂起 → block hook 捕获 → 异常入栈 → handler → EXC_RETURN 出栈恢复现场）。
//
// 场景（优先级：数值小 = 优先级高）：
// 1. IRQ0 优先级 5，IRQ1 优先级 10；
// 2. 主线同时挂起 IRQ0 + IRQ1 → 应先进入高优先级的 IRQ0（count0=1）；
// 3. IRQ0 返回后，低优先级 IRQ1 再进入（count1=1）；
// 4. IRQ1 内自挂起 IRQ0 → 高优先级抢占（嵌套）→ count0=2，IRQ1 随后继续；
// 5. 全部返回主线 → 写完成标记 G_DONE。
//
// 说明：block hook 只在基本块边界触发，因此用 noinline 函数调用
// （pending_dispatch_point / irq1_pending_hook）刻意制造块边界，
// 使中断投递/抢占发生在确定位置，避免编译器把整段代码合并为单个块。
//
// 固定地址结果区（供仿真器校验，避开栈顶 0x20001000 及异常帧压栈区）：
//   0x20000000 G_COUNT0  IRQ0 handler 执行次数（期望 2）
//   0x20000004 G_COUNT1  IRQ1 handler 执行次数（期望 1）
//   0x20000008 G_INNER   IRQ1 在 IRQ0 嵌套返回后是否继续（期望 0x22222222）
//   0x2000000C G_DONE    主线中断全部返回后完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* NVIC 寄存器（STM32F407，绝对地址；SCB 基址 0xE000E000） */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_ISPR0 (*(volatile uint32_t *)0xE000E200u)
#define NVIC_IPR0  (*(volatile uint32_t *)0xE000E400u)
#define NVIC_IPR1  (*(volatile uint32_t *)0xE000E404u)

/* 结果区（固定 SRAM 地址） */
#define G_COUNT0 (*(volatile uint32_t *)0x20000000u)
#define G_COUNT1 (*(volatile uint32_t *)0x20000004u)
#define G_INNER  (*(volatile uint32_t *)0x20000008u)
#define G_DONE   (*(volatile uint32_t *)0x2000000Cu)

extern void Reset_Handler(void);
void IRQ0_Handler(void);
void IRQ1_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void pending_dispatch_point(void) __attribute__((noinline));
static void pending_dispatch_point(void) {
    __asm volatile("nop" ::: "memory");
}

static void irq1_pending_hook(void) __attribute__((noinline));
static void irq1_pending_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + IRQ0(vector 16) + IRQ1(vector 17) */
__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    0x20001000u,               /* 0: 初始 SP（SRAM 内） */
    (uint32_t)Reset_Handler,   /* 1: Reset */
    (uint32_t)Default_Handler, /* 2: NMI */
    (uint32_t)Default_Handler, /* 3: HardFault */
    (uint32_t)Default_Handler, /* 4: MemManage */
    (uint32_t)Default_Handler, /* 5: BusFault */
    (uint32_t)Default_Handler, /* 6: UsageFault */
    (uint32_t)Default_Handler, /* 7: Reserved */
    (uint32_t)Default_Handler, /* 8: Reserved */
    (uint32_t)Default_Handler, /* 9: Reserved */
    (uint32_t)Default_Handler, /* 10: Reserved */
    (uint32_t)Default_Handler, /* 11: SVCall */
    (uint32_t)Default_Handler, /* 12: Reserved */
    (uint32_t)Default_Handler, /* 13: Reserved */
    (uint32_t)Default_Handler, /* 14: PendSV */
    (uint32_t)Default_Handler, /* 15: SysTick */
    (uint32_t)IRQ0_Handler,    /* 16: IRQ0 */
    (uint32_t)IRQ1_Handler,    /* 17: IRQ1 */
};

void IRQ0_Handler(void) {
    G_COUNT0++;
}

void IRQ1_Handler(void) {
    G_COUNT1++;
    /* 在 handler 内自挂起更高优先级 IRQ0 → 验证嵌套抢占 */
    NVIC_ISPR0 = (1u << 0);
    irq1_pending_hook(); /* 块边界：IRQ0 抢占应在此投递 */
    /* IRQ0 嵌套返回后，IRQ1 应在此继续 */
    G_INNER = 0x22222222u;
}

void Reset_Handler(void) {
    /* IRQ0 优先级 5（高），IRQ1 优先级 10（低） */
    NVIC_IPR0 = 0x5u;
    NVIC_IPR1 = 0xAu;
    NVIC_ISER0 = (1u << 0) | (1u << 1); /* 使能 IRQ0/IRQ1 */
    __asm volatile("cpsie i" ::: "memory");

    /* 同时挂起 IRQ0 + IRQ1：应先进入优先级高的 IRQ0 */
    NVIC_ISPR0 = (1u << 0) | (1u << 1);
    pending_dispatch_point(); /* 块边界：首个中断投递应在此发生 */

    /* 中断全部返回后，主线在此继续 */
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
