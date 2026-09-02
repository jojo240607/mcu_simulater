// M5 demo 固件：SPI1 ↔ DMA2 外设方向传输（HAL 默认流）端到端验证。
//
// 场景（SPI1 使能 CR2.TXDMAEN|RXDMAEN，接 DMA2 默认流）：
//   TX：DMA2_Stream3_Channel3（内存→外设）把 TX_BUF 搬运到 SPI1_DR → 虚拟从机/测试；
//   RX：DMA2_Stream0_Channel3（外设→内存）把 SPI1_DR 搬到 RX_BUF（测试注入 4 字节）。
// 1. RCC 使能 SPI1（APB2ENR bit12）+ DMA2（AHB1ENR bit22，镜像）；
// 2. SPI1 CR1.SPE 使能（上升沿置 TXE，仿真语义）；
// 3. 配置 DMA2_Stream3（TX）：DIR=内存→外设(01)、CHSEL=3、PAR=SPI1_DR、
//    M0AR=TX_BUF、NDTR=TX_LEN、MINC、TCIE，写 EN 启动；
// 4. 配置 DMA2_Stream0（RX）：DIR=外设→内存(00)、CHSEL=3、PAR=SPI1_DR、
//    M0AR=RX_BUF、NDTR=RX_LEN、MINC、TCIE，写 EN 启动；
// 5. NVIC 使能 IRQ59（DMA2_Stream3）+ IRQ56（DMA2_Stream0）；
// 6. SPI1 CR2.TXDMAEN|RXDMAEN：写 CR2 时 TXE 已置位 → 发布 TX DMA 请求 →
//    路由到 Stream3 → 一次搬完 TX_BUF（每字节发 SpiByte 事件）；RX 每收 1 字节发 1 次请求；
// 7. 两条流完成后各自 TC 中断：DMA2_Stream3_IRQHandler 校验发送完成 → G_TX_TC++；
//    DMA2_Stream0_IRQHandler 校验 RX_BUF 内容 → G_RX_TC++；清 LIFCR；
// 8. 主线轮询 G_TX_TC && G_RX_TC 达 1 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_TX_TC  DMA2_Stream3 TX 完成中断执行次数（期望 1）
//   0x20000004 G_RX_TC  DMA2_Stream0 RX 完成中断执行次数（期望 1）
//   0x20000008 G_DONE   主线完成标记（期望 0xAAAAAAAA）
//   0x20000100 TX_BUF   发送源（固件预置 "SPIDMA" 6 字节，经 DMA TX 发送）
//   0x20000200 RX_BUF   接收目标（测试注入 4 字节，经 DMA RX 搬运，仿真器直接校验）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844u) /* bit12 = SPI1 时钟 */
#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830u) /* bit22 = DMA2 时钟 */

/* SPI1 @ 0x40013000 */
#define SPI1_CR1 (*(volatile uint32_t *)0x40013000u)
#define SPI1_CR2 (*(volatile uint32_t *)0x40013004u)
#define SPI1_SR  (*(volatile uint32_t *)0x40013008u)
#define SPI1_DR  (*(volatile uint32_t *)0x4001300Cu)

/* DMA2 @ 0x40026400：LIFCR@0x08（流0-3）、HIFCR@0x0C（流4-7）；
   流 s 寄存器基址 0x40026410 + s*0x18 */
#define DMA2_LIFCR (*(volatile uint32_t *)0x40026408u)
#define DMA2_STREAM(s) ((volatile uint32_t *)(0x40026410u + (s) * 0x18u))
#define DMA2_SsCR(s)  DMA2_STREAM(s)[0]
#define DMA2_SsNDTR(s) DMA2_STREAM(s)[1]
#define DMA2_SsPAR(s) DMA2_STREAM(s)[2]
#define DMA2_SsM0AR(s) DMA2_STREAM(s)[3]

/* NVIC（SCB 基址 0xE000E000）：
   IRQ56 = DMA2_Stream0 → ISER1 bit24、IPR14 字节0（bits0-7）
   IRQ59 = DMA2_Stream3 → ISER1 bit27、IPR14 字节3（bits24-31） */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u)
#define NVIC_IPR14 (*(volatile uint32_t *)0xE000E438u)

/* SPI CR 位 */
#define CR1_SPE      (1u << 6)
#define CR2_RXDMAEN  (1u << 0)
#define CR2_TXDMAEN  (1u << 1)

/* DMA CR 位（F407） */
#define DMA_CR_EN      (1u << 0)
#define DMA_CR_TCIE    (1u << 5)
#define DMA_CR_DIR_MTM (1u << 6)   /* 01 = 内存→外设（TX） */
#define DMA_CR_DIR_PTM (0u << 6)   /* 00 = 外设→内存（RX） */
#define DMA_CR_MINC    (1u << 10)
#define DMA_CR_CHSEL3  (3u << 25)  /* CHSEL = 3（SPI1 通道） */

/* 数据区（固定 SRAM 地址） */
#define TX_BUF ((volatile uint8_t *)0x20000100u)
#define RX_BUF ((volatile uint8_t *)0x20000200u)
#define TX_LEN 6u
#define RX_LEN 4u

/* 结果区 */
#define G_TX_TC (*(volatile uint32_t *)0x20000000u)
#define G_RX_TC (*(volatile uint32_t *)0x20000004u)
#define G_DONE  (*(volatile uint32_t *)0x20000008u)

extern void Reset_Handler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void dma_ready_hook(void) __attribute__((noinline));
static void dma_ready_hook(void) {
    __asm volatile("nop" ::: "memory");
}

static void dma_done_hook(void) __attribute__((noinline));
static void dma_done_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + IRQ0..IRQ59（IRQ56 = DMA2_Stream0 → index 72，
   IRQ59 = DMA2_Stream3 → index 75） */
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
    (uint32_t)Default_Handler, /* 52 */
    (uint32_t)Default_Handler, /* 53 */
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
    (uint32_t)Default_Handler, /* 67 */
    (uint32_t)Default_Handler, /* 68 */
    (uint32_t)Default_Handler, /* 69 */
    (uint32_t)Default_Handler, /* 70 */
    (uint32_t)Default_Handler, /* 71 */
    (uint32_t)DMA2_Stream0_IRQHandler, /* 72: IRQ56 = DMA2_Stream0 (RX) */
    (uint32_t)Default_Handler, /* 73 */
    (uint32_t)Default_Handler, /* 74 */
    (uint32_t)DMA2_Stream3_IRQHandler, /* 75: IRQ59 = DMA2_Stream3 (TX) */
};

void DMA2_Stream3_IRQHandler(void) {
    /* TX 完成：发送已结束（NDTR 应已清零）→ G_TX_TC++ */
    DMA2_LIFCR = (1u << 23); /* 写 1 清除 TCIF3（LISR 偏移 3*6 + 5 = 23） */
    G_TX_TC++;
}

void DMA2_Stream0_IRQHandler(void) {
    /* RX 完成：校验 RX_BUF 收到 4 字节 */
    uint32_t ok = 1u;
    for (uint32_t i = 0u; i < RX_LEN; i++) {
        if (RX_BUF[i] != (uint8_t)('a' + i)) {
            ok = 0u;
            break;
        }
    }
    DMA2_LIFCR = (1u << 5); /* 写 1 清除 TCIF0（LISR 偏移 0 + 5 = 5） */
    if (ok) {
        G_RX_TC++;
    }
}

void Reset_Handler(void) {
    /* 0. 预置 TX_BUF 发送源 "SPIDMA" */
    const uint8_t msg[TX_LEN] = { 'S', 'P', 'I', 'D', 'M', 'A' };
    for (uint32_t i = 0u; i < TX_LEN; i++) {
        TX_BUF[i] = msg[i];
    }

    /* 1. RCC 时钟（镜像） */
    RCC_APB2ENR = (1u << 12); /* SPI1 */
    RCC_AHB1ENR = (1u << 22); /* DMA2 */

    /* 2. SPI1 SPE 使能（上升沿置 TXE，仿真语义） */
    SPI1_CR1 = CR1_SPE;

    /* 3. 配置 DMA2_Stream3（TX：内存→外设 CHSEL3） */
    DMA2_SsPAR(3) = 0x4001300Cu; /* SPI1_DR */
    DMA2_SsM0AR(3) = (uint32_t)TX_BUF;
    DMA2_SsNDTR(3) = TX_LEN;
    DMA2_SsCR(3) = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_MTM | DMA_CR_MINC | DMA_CR_CHSEL3;

    /* 4. 配置 DMA2_Stream0（RX：外设→内存 CHSEL3） */
    DMA2_SsPAR(0) = 0x4001300Cu; /* SPI1_DR */
    DMA2_SsM0AR(0) = (uint32_t)RX_BUF;
    DMA2_SsNDTR(0) = RX_LEN;
    DMA2_SsCR(0) = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_PTM | DMA_CR_MINC | DMA_CR_CHSEL3;

    /* 5. NVIC：IRQ56/IRQ59 优先级 + 使能 */
    NVIC_IPR14 = (0xFu << 0) | (0xFu << 24); /* IRQ56 → 字节0、IRQ59 → 字节3 */
    NVIC_ISER1 = (1u << 24);  /* IRQ56 */
    NVIC_ISER1 = (1u << 27);  /* IRQ59 */
    __asm volatile("cpsie i" ::: "memory");

    /* 6. 使能 SPI DMA：写 CR2 时 TXE 已置位 → 立即发布 TX DMA 请求 */
    SPI1_CR2 = CR2_TXDMAEN | CR2_RXDMAEN;

    dma_ready_hook();

    /* 7. 主循环：等待 TX/RX 两条流完成中断（handler 各递增计数） */
    while ((G_TX_TC == 0u) || (G_RX_TC == 0u)) {}
    dma_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
