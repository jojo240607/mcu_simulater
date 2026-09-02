// M3 demo 固件：验证 T1 外设集端到端链路
// （GPIO 事件 / USART TX→Console / TIM2 溢出中断）。
//
// 流程：
// 1. RCC 使能 GPIOA/TIM2/USART1 时钟（寄存器镜像）；
// 2. PA5 配置为输出（blinky LED）；
// 3. USART1 使能（UE+TE），轮询 TXE 发送 "M3" → 虚拟 Console 捕获；
// 4. TIM2 使能（PSC=0/ARR=31/UIE）→ 溢出触发 IRQ28；
// 5. TIM2_IRQHandler：清 UIF → G_TMR++ → BSRR 翻转 PA5（发 GpioLevel 事件）；
//    G_TMR 达 4 时停表（CEN=0），保证主线确定性退出循环；
// 6. 主线轮询 G_TMR，达 4 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_LED   BSRR 翻转次数（= TIM2 中断次数，期望 4）
//   0x20000004 G_UART  USART 发送字节数（期望 2："M3"）
//   0x20000008 G_TMR   TIM2 IRQ28 handler 执行次数（期望 4）
//   0x2000000C G_DONE  主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830u) /* bit0 = GPIOA 时钟 */
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840u) /* bit0 = TIM2 时钟 */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844u) /* bit14 = USART1 时钟 */

/* GPIOA @ 0x40020000 */
#define GPIOA_MODER (*(volatile uint32_t *)0x40020000u)
#define GPIOA_BSRR  (*(volatile uint32_t *)0x40020018u)

/* USART1 @ 0x40011000 */
#define USART1_SR  (*(volatile uint32_t *)0x40011000u)
#define USART1_DR  (*(volatile uint32_t *)0x40011004u)
#define USART1_CR1 (*(volatile uint32_t *)0x4001100Cu)

/* TIM2 @ 0x40000000 */
#define TIM2_CR1  (*(volatile uint32_t *)0x40000000u)
#define TIM2_DIER (*(volatile uint32_t *)0x4000000Cu)
#define TIM2_SR   (*(volatile uint32_t *)0x40000010u)
#define TIM2_PSC  (*(volatile uint32_t *)0x40000028u)
#define TIM2_ARR  (*(volatile uint32_t *)0x4000002Cu)

/* NVIC（SCB 基址 0xE000E000） */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_IPR7  (*(volatile uint32_t *)0xE000E41Cu) /* IRQ28 → 字节 28 → IPR7 */

/* 位定义 */
#define SR_TXE  (1u << 7)
#define CR1_UE  (1u << 13)
#define CR1_TE  (1u << 3)
#define TIM_UIE (1u << 0)
#define TIM_CEN (1u << 0)
#define TIM_UIF (1u << 0)

/* 结果区（固定 SRAM 地址） */
#define G_LED  (*(volatile uint32_t *)0x20000000u)
#define G_UART (*(volatile uint32_t *)0x20000004u)
#define G_TMR  (*(volatile uint32_t *)0x20000008u)
#define G_DONE (*(volatile uint32_t *)0x2000000Cu)

extern void Reset_Handler(void);
void TIM2_IRQHandler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void timer_start_hook(void) __attribute__((noinline));
static void timer_start_hook(void) {
    __asm volatile("nop" ::: "memory");
}

static void timer_done_hook(void) __attribute__((noinline));
static void timer_done_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 轮询 TXE 发送单字节（noinline 制造块边界，避免长发送合并为单块） */
static void uart_putc(char c) __attribute__((noinline));
static void uart_putc(char c) {
    while ((USART1_SR & SR_TXE) == 0) {}
    USART1_DR = (uint32_t)(unsigned char)c;
    G_UART++;
}

static void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

/* 向量表：系统异常 + IRQ0..IRQ28（index 16..44） */
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
    (uint32_t)TIM2_IRQHandler, /* 44: IRQ28 = TIM2 */
};

void TIM2_IRQHandler(void) {
    if (TIM2_SR & TIM_UIF) {
        TIM2_SR = 0; /* 写 0 清除 UIF */
        G_TMR++;
        if (G_TMR >= 4u) {
            TIM2_CR1 = 0; /* 停表，保证主线确定性退出 */
        }
        /* BSRR 翻转 PA5（发布 GpioLevel 事件） */
        GPIOA_BSRR = (G_TMR & 1u) ? (1u << 5) : (1u << 21);
        G_LED++;
    }
}

void Reset_Handler(void) {
    /* 1. 使能时钟（寄存器镜像） */
    RCC_AHB1ENR = 1u;
    RCC_APB1ENR = 1u;
    RCC_APB2ENR = (1u << 14);

    /* 2. PA5 输出 */
    GPIOA_MODER = 1u << (5 * 2);

    /* 3. USART1 使能，发送 "M3" */
    USART1_CR1 = CR1_UE | CR1_TE;
    uart_puts("M3");

    /* 4. TIM2：PSC=0（÷1），ARR=255 → 每 256 个虚拟周期溢出一次。
     *    ARR 取大值保证 handler 入口的块级 tick 不会在 CEN 清零前二次溢出
     *    （handler 单块约 48 周期，远小于 256），从而确定性触发恰好 4 次。 */
    TIM2_PSC = 0;
    TIM2_ARR = 255;
    TIM2_DIER = TIM_UIE;

    /* 5. IRQ28 优先级 + 使能 */
    NVIC_IPR7 = 0xFu;
    NVIC_ISER0 = (1u << 28);
    __asm volatile("cpsie i" ::: "memory");

    /* 6. 启动 TIM2，制造块边界后进入主循环 */
    TIM2_CR1 = TIM_CEN;
    timer_start_hook();

    /* 7. 主循环：等待 4 次 TIM2 溢出中断（handler 停表后退出） */
    while (G_TMR < 4u) {}
    timer_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
