// M5 demo 固件：SPI 完整外设仿真（RX 事件中断接收 + 轮询 TXE 发送）。
//
// 流程：
// 1. RCC 使能 SPI1 时钟（APB2ENR bit12，镜像）；
// 2. SPI1 CR1.SPE 使能（SPE 上升沿置 TXE，仿真语义）；
// 3. 轮询 TXE 发送问候 'H' → 虚拟从机（Event::SpiByte）捕获（G_TX=1）；
// 4. NVIC IRQ35（SPI1）优先级 15 + 使能；CR2.RXNEIE 使能 RX 中断；
// 5. 外部（测试）逐字节发布 Event::SpiRx → feed_rx：置 RXNE + 挂起 IRQ35；
// 6. SPI1_IRQHandler：RXNE → 读 DR（清 RXNE）→ RX_BUF[G_RX] → G_RX++；
// 7. 主线轮询 G_RX 达 4 后写完成标记 G_DONE。
//
// 注：只使能 RXNEIE（不使能 TXEIE），写 CR2 时 RXNE 未置位不会挂起，
//     仅 RX 注入才触发中断；handler 只读 DR、主循环不写 SPI 寄存器，
//     故不会因 TXE 恒置位造成中断风暴。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_TX   发送字节数（问候 1，期望 1）
//   0x20000004 G_RX   中断接收次数（期望 4）
//   0x20000008 G_DONE 主线完成标记（期望 0xAAAAAAAA）
//   0x20000200 RX_BUF 中断接收结果（期望 "abcd"）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844u) /* bit12 = SPI1 时钟 */

/* SPI1 @ 0x40013000 */
#define SPI1_CR1 (*(volatile uint32_t *)0x40013000u)
#define SPI1_CR2 (*(volatile uint32_t *)0x40013004u)
#define SPI1_SR  (*(volatile uint32_t *)0x40013008u)
#define SPI1_DR  (*(volatile uint32_t *)0x4001300Cu)

/* NVIC（SCB 基址 0xE000E000）：IRQ35 → ISER1 bit3 + IPR8 字节3 */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u)
#define NVIC_IPR8  (*(volatile uint32_t *)0xE000E420u)

/* 位定义 */
#define CR1_SPE      (1u << 6)
#define CR2_RXNEIE   (1u << 6)
#define SR_TXE       (1u << 1)
#define SR_RXNE      (1u << 0)

/* 结果区（固定 SRAM 地址） */
#define G_TX   (*(volatile uint32_t *)0x20000000u)
#define G_RX   (*(volatile uint32_t *)0x20000004u)
#define G_DONE (*(volatile uint32_t *)0x20000008u)
#define RX_BUF ((volatile uint8_t *)0x20000200u)

extern void Reset_Handler(void);
void SPI1_IRQHandler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void spi_wait_hook(void) __attribute__((noinline));
static void spi_wait_hook(void) {
    __asm volatile("nop" ::: "memory");
}

static void spi_done_hook(void) __attribute__((noinline));
static void spi_done_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + IRQ0..IRQ35（SPI1 = IRQ35 → index 51） */
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
    (uint32_t)SPI1_IRQHandler, /* 51: IRQ35 = SPI1 */
};

void SPI1_IRQHandler(void) {
    uint32_t sr = SPI1_SR;
    if (sr & SR_RXNE) {
        RX_BUF[G_RX] = (uint8_t)(SPI1_DR & 0xFFu); /* 读 DR 清 RXNE */
        G_RX++;
    }
}

void Reset_Handler(void) {
    /* 1. 使能 SPI1 时钟（寄存器镜像） */
    RCC_APB2ENR = (1u << 12);

    /* 2. SPI1：SPE 使能（上升沿置 TXE，仿真语义） */
    SPI1_CR1 = CR1_SPE;

    /* 3. 轮询 TXE 发送问候 'H' → 虚拟从机（Event::SpiByte）捕获 */
    while ((SPI1_SR & SR_TXE) == 0) {}
    SPI1_DR = 'H';
    G_TX = 1;

    /* 4. NVIC：IRQ35 优先级 15 + 使能 */
    NVIC_IPR8 = 0xFu << 24;
    NVIC_ISER1 = (1u << 3);

    /* 5. SPI1：RX 中断使能（RXNE 未置位 → 不挂起） */
    SPI1_CR2 = CR2_RXNEIE;

    __asm volatile("cpsie i" ::: "memory");
    spi_wait_hook();

    /* 6. 等待 4 次中断接收（handler 逐字节写入 RX_BUF） */
    while (G_RX != 4u) {}
    spi_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
