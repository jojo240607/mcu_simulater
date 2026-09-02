// M5 demo 固件：USART 完整串口仿真（RX 中断接收 + 回显 + TX→Console）。
//
// 流程：
// 1. RCC 使能 USART1 时钟（APB2ENR bit14，镜像）；
// 2. USART1 使能 UE+TE+RE + RXNEIE；NVIC IRQ37（USART1）优先级 15 + 使能；
// 3. 轮询 TXE 发送问候 'U' → 虚拟 Console 捕获（G_TX=1）；
// 4. 外部（测试）逐字节发布 Event::UartRx → USART feed_rx：锁存 DR + 置 RXNE + 挂起 IRQ37；
// 5. USART1_IRQHandler：RXNE → 读 DR（清 RXNE）→ 回显写 DR（TE+UE → Console）→ G_RX++；
//    ORE → G_ORE++ 并写 0 清除；
// 6. 主线轮询 G_RX 达 4 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_TX   发送字节数（问候 1 + 回显 4，期望 5）
//   0x20000004 G_RX   接收/回显次数（期望 4）
//   0x20000008 G_ORE  过载次数（测试逐字节注入，期望 0）
//   0x2000000C G_DONE 主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844u) /* bit14 = USART1 时钟 */

/* USART1 @ 0x40011000 */
#define USART1_SR  (*(volatile uint32_t *)0x40011000u)
#define USART1_DR  (*(volatile uint32_t *)0x40011004u)
#define USART1_CR1 (*(volatile uint32_t *)0x4001100Cu)

/* NVIC（SCB 基址 0xE000E000） */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u) /* IRQ37-39 → bit5-7 */
#define NVIC_IPR9  (*(volatile uint32_t *)0xE000E424u) /* IRQ37 → 字节1（bits8-15） */

/* 位定义 */
#define SR_TXE  (1u << 7)
#define SR_TC   (1u << 6)
#define SR_RXNE (1u << 5)
#define SR_ORE  (1u << 3)
#define CR1_UE  (1u << 13)
#define CR1_RE  (1u << 2)
#define CR1_TE  (1u << 3)
#define CR1_RXNEIE (1u << 5)

/* 结果区（固定 SRAM 地址） */
#define G_TX   (*(volatile uint32_t *)0x20000000u)
#define G_RX   (*(volatile uint32_t *)0x20000004u)
#define G_ORE  (*(volatile uint32_t *)0x20000008u)
#define G_DONE (*(volatile uint32_t *)0x2000000Cu)

extern void Reset_Handler(void);
void USART1_IRQHandler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void uart_wait_hook(void) __attribute__((noinline));
static void uart_wait_hook(void) {
    __asm volatile("nop" ::: "memory");
}

static void uart_done_hook(void) __attribute__((noinline));
static void uart_done_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + IRQ0..IRQ37（USART1 = IRQ37 → index 53） */
__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    0x20001000u,               /* 0: 初始 SP（SRAM 内） */
    (uint32_t)Reset_Handler,   /* 1: Reset */
    (uint32_t)Default_Handler, /* 2: NMI */
    (uint32_t)Default_Handler, /* 3: HardFault */
    (uint32_t)Default_Handler, /* 4: MemManage */
    (uint32_t)Default_Handler, /* 5: BusFault */
    (uint32_t)Default_Handler, /* 6: UsageFault */
    (uint32_t)Default_Handler, /* 7..15: 系统异常 */
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler, /* 16: IRQ0 */
    (uint32_t)Default_Handler, /* 17 */
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
    (uint32_t)Default_Handler, /* 44 */
    (uint32_t)Default_Handler, /* 45 */
    (uint32_t)Default_Handler, /* 46 */
    (uint32_t)Default_Handler, /* 47 */
    (uint32_t)Default_Handler, /* 48 */
    (uint32_t)Default_Handler, /* 49 */
    (uint32_t)Default_Handler, /* 50 */
    (uint32_t)Default_Handler, /* 51 */
    (uint32_t)Default_Handler, /* 52: IRQ36 */
    (uint32_t)USART1_IRQHandler, /* 53: IRQ37 = USART1 */
};

void USART1_IRQHandler(void) {
    uint32_t sr = USART1_SR;
    if (sr & SR_RXNE) {
        uint8_t c = (uint8_t)(USART1_DR & 0xFFu); /* 读 DR 清 RXNE */
        USART1_DR = c;                            /* 回显（TE+UE → Console） */
        G_RX++;
        G_TX++;
    }
    if (sr & SR_ORE) {
        G_ORE++;
        USART1_SR = ~(uint32_t)SR_ORE; /* 写 0 清除 ORE */
    }
}

void Reset_Handler(void) {
    /* 1. 使能 USART1 时钟（寄存器镜像） */
    RCC_APB2ENR = (1u << 14);

    /* 2. NVIC：IRQ37 优先级 15 + 使能 */
    NVIC_IPR9 = 0xFu << 8;
    NVIC_ISER1 = (1u << 5);

    /* 3. USART1：UE+TE+RE + RXNEIE */
    USART1_CR1 = CR1_UE | CR1_TE | CR1_RE | CR1_RXNEIE;

    /* 4. 轮询 TXE 发送问候 'U' → Console 捕获 */
    while ((USART1_SR & SR_TXE) == 0) {}
    USART1_DR = 'U';
    G_TX = 1;

    __asm volatile("cpsie i" ::: "memory");
    uart_wait_hook();

    /* 5. 等待 4 次回显（RX 中断 handler 逐字节回显） */
    while (G_RX != 4u) {}
    uart_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
