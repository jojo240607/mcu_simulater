// M5 demo 固件：UART4-6 + 虚拟终端接线（终端回环）。
//
// 流程：
// 1. RCC 使能 UART4/UART5（APB1ENR bit19/20）+ USART6（APB2ENR bit5）；
// 2. UART4 使能 UE+TE+RE+RXNEIE；NVIC IRQ52（UART4）优先级 15 + 使能；
//    UART5/USART6 仅使能 UE+TE（问候输出，验证端口挂载与 TX）；
// 3. 轮询 TXE 发问候：UART4→'T'、UART5→'5'、USART6→'6'：
//    - 默认接线：全部 UART TX → Console；
//    - connect(uart.tx, terminal.rx)（port 4）→ 'T' 与回显也进虚拟终端显示；
// 4. 外部（测试）经虚拟终端键盘 type_char(4, c) 发布 Event::UartRx → UART4 feed_rx
//    → 锁存 DR + 置 RXNE + 挂起 IRQ52；
// 5. UART4_IRQHandler：RXNE → 读 DR（清 RXNE）→ 回显写 DR（TE+UE）→ G_RX4++/G_TX4++；
//    ORE → G_ORE++ 并写 0 清除；
// 6. 主线轮询 G_RX4 达 4 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_TX4   UART4 发送字节数（问候 1 + 回显 4，期望 5）
//   0x20000004 G_RX4   UART4 回显次数（期望 4）
//   0x20000008 G_TX56  UART5+USART6 问候数（期望 2）
//   0x2000000C G_DONE  主线完成标记（期望 0xAAAAAAAA）
//   0x20000010 G_ORE   UART4 过载次数（逐字符注入，期望 0）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840u) /* bit19=UART4EN bit20=UART5EN */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844u) /* bit5 =USART6EN */

/* UART4 @ 0x40004C00（APB1） */
#define UART4_SR  (*(volatile uint32_t *)0x40004C00u)
#define UART4_DR  (*(volatile uint32_t *)0x40004C04u)
#define UART4_CR1 (*(volatile uint32_t *)0x40004C0Cu)
/* UART5 @ 0x40005000（APB1） */
#define UART5_SR  (*(volatile uint32_t *)0x40005000u)
#define UART5_DR  (*(volatile uint32_t *)0x40005004u)
#define UART5_CR1 (*(volatile uint32_t *)0x4000500Cu)
/* USART6 @ 0x40011400（APB2） */
#define USART6_SR  (*(volatile uint32_t *)0x40011400u)
#define USART6_DR  (*(volatile uint32_t *)0x40011404u)
#define USART6_CR1 (*(volatile uint32_t *)0x4001140Cu)

/* NVIC（SCB 基址 0xE000E000） */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u) /* IRQ52 → bit20 */
#define NVIC_IPR13 (*(volatile uint32_t *)0xE000E434u) /* IRQ52 → 字节0（bits0-7） */

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
#define G_TX4   (*(volatile uint32_t *)0x20000000u)
#define G_RX4   (*(volatile uint32_t *)0x20000004u)
#define G_TX56  (*(volatile uint32_t *)0x20000008u)
#define G_DONE  (*(volatile uint32_t *)0x2000000Cu)
#define G_ORE   (*(volatile uint32_t *)0x20000010u)

extern void Reset_Handler(void);
void UART4_IRQHandler(void);

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

/* 向量表：系统异常 + IRQ0..IRQ52（UART4 = IRQ52 → index 68） */
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
    (uint32_t)Default_Handler, /* 53: IRQ37 */
    (uint32_t)Default_Handler, /* 54 */
    (uint32_t)Default_Handler, /* 55 */
    (uint32_t)Default_Handler, /* 56 */
    (uint32_t)Default_Handler, /* 57 */
    (uint32_t)Default_Handler, /* 58 */
    (uint32_t)Default_Handler, /* 59 */
    (uint32_t)Default_Handler, /* 60 */
    (uint32_t)Default_Handler, /* 61 */
    (uint32_t)Default_Handler, /* 62 */
    (uint32_t)Default_Handler, /* 63 */
    (uint32_t)Default_Handler, /* 64 */
    (uint32_t)Default_Handler, /* 65 */
    (uint32_t)Default_Handler, /* 66 */
    (uint32_t)Default_Handler, /* 67: IRQ51 */
    (uint32_t)UART4_IRQHandler, /* 68: IRQ52 = UART4 */
};

void UART4_IRQHandler(void) {
    uint32_t sr = UART4_SR;
    if (sr & SR_RXNE) {
        uint8_t c = (uint8_t)(UART4_DR & 0xFFu); /* 读 DR 清 RXNE */
        UART4_DR = c;                            /* 回显（TE+UE → Terminal/Console） */
        G_RX4++;
        G_TX4++;
    }
    if (sr & SR_ORE) {
        G_ORE++;
        UART4_SR = ~(uint32_t)SR_ORE; /* 写 0 清除 ORE */
    }
}

void Reset_Handler(void) {
    /* 1. 使能 UART4/5（APB1）+ USART6（APB2）时钟（寄存器镜像） */
    RCC_APB1ENR = (1u << 19) | (1u << 20);
    RCC_APB2ENR = (1u << 5);

    /* 2. NVIC：IRQ52（UART4）优先级 15 + 使能 */
    NVIC_IPR13 = 0xFu;
    NVIC_ISER1 = (1u << 20);

    /* 3. UART4：UE+TE+RE + RXNEIE；UART5/USART6：UE+TE */
    UART4_CR1 = CR1_UE | CR1_TE | CR1_RE | CR1_RXNEIE;
    UART5_CR1 = CR1_UE | CR1_TE;
    USART6_CR1 = CR1_UE | CR1_TE;

    /* 4. 轮询 TXE 发问候：UART4→'T'（终端显示），UART5→'5'、USART6→'6'（Console） */
    while ((UART4_SR & SR_TXE) == 0) {}
    UART4_DR = 'T';
    G_TX4 = 1;

    while ((UART5_SR & SR_TXE) == 0) {}
    UART5_DR = '5';
    G_TX56 = 1;

    while ((USART6_SR & SR_TXE) == 0) {}
    USART6_DR = '6';
    G_TX56 = 2;

    __asm volatile("cpsie i" ::: "memory");
    uart_wait_hook();

    /* 5. 等待 4 次回显（UART4 RX 中断 handler 逐字节回显） */
    while (G_RX4 != 4u) {}
    uart_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
