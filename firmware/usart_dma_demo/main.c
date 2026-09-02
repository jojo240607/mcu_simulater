// M5 demo 固件：USART1 ↔ DMA2 外设方向传输（HAL 默认流）端到端验证。
//
// 场景（USART1 使能 DMAR/DMAT，接 DMA2 默认流）：
//   TX：DMA2_Stream7_Channel4（内存→外设）把 TX_BUF 搬运到 USART1_DR → Console；
//   RX：DMA2_Stream2_Channel4（外设→内存）把 USART1_DR 搬到 RX_BUF（测试注入 4 字节）。
// 1. RCC 使能 USART1（APB2ENR bit14）+ DMA2（AHB1ENR bit22，镜像）；
// 2. 配置 DMA2_Stream7（TX）：DIR=内存→外设(01)、CHSEL=4、PAR=USART1_DR、
//    M0AR=TX_BUF、NDTR=TX_LEN、MINC、TCIE，写 EN 启动；
// 3. 配置 DMA2_Stream2（RX）：DIR=外设→内存(00)、CHSEL=4、PAR=USART1_DR、
//    M0AR=RX_BUF、NDTR=RX_LEN、MINC、TCIE，写 EN 启动；
// 4. NVIC 使能 IRQ70（DMA2_Stream7）+ IRQ58（DMA2_Stream2）；
// 5. USART1 UE+TE+RE + CR3.DMAT|DMAR：写 CR3 时 TXE 已置位 → 发布 TX DMA 请求 →
//    路由到 Stream7 → 一次搬完 TX_BUF（每字节发 Console）；RX 每收 1 字节发 1 次请求；
// 6. 两条流完成后各自 TC 中断：DMA2_Stream7_IRQHandler 校验发送完成 → G_TX_TC++；
//    DMA2_Stream2_IRQHandler 校验 RX_BUF 内容 → G_RX_TC++；清 HIFCR/LIFCR；
// 7. 主线轮询 G_TX_TC && G_RX_TC 达 1 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_TX_TC  DMA2_Stream7 TX 完成中断执行次数（期望 1）
//   0x20000004 G_RX_TC  DMA2_Stream2 RX 完成中断执行次数（期望 1）
//   0x20000008 G_DONE   主线完成标记（期望 0xAAAAAAAA）
//   0x20000100 TX_BUF   发送源（固件预置 "M5DMA" 6 字节，经 DMA TX → Console）
//   0x20000200 RX_BUF   接收目标（测试注入 4 字节，经 DMA RX 搬运，仿真器直接校验）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844u) /* bit14 = USART1 时钟 */
#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830u) /* bit22 = DMA2 时钟 */

/* USART1 @ 0x40011000 */
#define USART1_DR  (*(volatile uint32_t *)0x40011004u)
#define USART1_CR1 (*(volatile uint32_t *)0x4001100Cu)
#define USART1_CR3 (*(volatile uint32_t *)0x40011014u)

/* DMA2 @ 0x40026400：LIFCR@0x08（流0-3）、HIFCR@0x0C（流4-7）；
   流 s 寄存器基址 0x40026410 + s*0x18 */
#define DMA2_LIFCR (*(volatile uint32_t *)0x40026408u)
#define DMA2_HIFCR (*(volatile uint32_t *)0x4002640Cu)
#define DMA2_STREAM(s) ((volatile uint32_t *)(0x40026410u + (s) * 0x18u))
#define DMA2_SsCR(s)  DMA2_STREAM(s)[0]
#define DMA2_SsNDTR(s) DMA2_STREAM(s)[1]
#define DMA2_SsPAR(s) DMA2_STREAM(s)[2]
#define DMA2_SsM0AR(s) DMA2_STREAM(s)[3]

/* NVIC（SCB 基址 0xE000E000）：
   IRQ58 = DMA2_Stream2 → ISER1 bit26、IPR14 字节2（bits16-23）
   IRQ70 = DMA2_Stream7 → ISER2 bit6、IPR17 字节2（bits16-23） */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u)
#define NVIC_ISER2 (*(volatile uint32_t *)0xE000E108u)
#define NVIC_IPR14 (*(volatile uint32_t *)0xE000E438u)
#define NVIC_IPR17 (*(volatile uint32_t *)0xE000E444u)

/* USART CR 位 */
#define CR1_UE  (1u << 13)
#define CR1_RE  (1u << 2)
#define CR1_TE  (1u << 3)
#define CR3_DMAT (1u << 7)
#define CR3_DMAR (1u << 6)

/* DMA CR 位（F407） */
#define DMA_CR_EN     (1u << 0)
#define DMA_CR_TCIE   (1u << 5)
#define DMA_CR_DIR_MTM (1u << 6)   /* 01 = 内存→外设（TX） */
#define DMA_CR_DIR_PTM (0u << 6)   /* 00 = 外设→内存（RX） */
#define DMA_CR_MINC   (1u << 10)
#define DMA_CR_CHSEL4 (4u << 25)   /* CHSEL = 4（USART1 通道） */

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
void DMA2_Stream7_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);

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

/* 向量表：系统异常 + IRQ0..IRQ70（IRQ58 = DMA2_Stream2 → index 74，
   IRQ70 = DMA2_Stream7 → index 86） */
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
    (uint32_t)Default_Handler, /* 72 */
    (uint32_t)Default_Handler, /* 73 */
    (uint32_t)DMA2_Stream2_IRQHandler, /* 74: IRQ58 = DMA2_Stream2 (RX) */
    (uint32_t)Default_Handler, /* 75 */
    (uint32_t)Default_Handler, /* 76 */
    (uint32_t)Default_Handler, /* 77 */
    (uint32_t)Default_Handler, /* 78 */
    (uint32_t)Default_Handler, /* 79 */
    (uint32_t)Default_Handler, /* 80 */
    (uint32_t)Default_Handler, /* 81 */
    (uint32_t)Default_Handler, /* 82 */
    (uint32_t)Default_Handler, /* 83 */
    (uint32_t)Default_Handler, /* 84: IRQ68 = DMA2_Stream5 */
    (uint32_t)Default_Handler, /* 85: IRQ69 = DMA2_Stream6 */
    (uint32_t)DMA2_Stream7_IRQHandler, /* 86: IRQ70 = DMA2_Stream7 (TX) */
};

void DMA2_Stream7_IRQHandler(void) {
    /* TX 完成：发送已结束（NDTR 应已清零）→ G_TX_TC++ */
    DMA2_HIFCR = (1u << 23); /* 写 1 清除 TCIF7 */
    G_TX_TC++;
}

void DMA2_Stream2_IRQHandler(void) {
    /* RX 完成：校验 RX_BUF 收到 4 字节 */
    uint32_t ok = 1u;
    for (uint32_t i = 0u; i < RX_LEN; i++) {
        if (RX_BUF[i] != (uint8_t)('a' + i)) {
            ok = 0u;
            break;
        }
    }
    DMA2_LIFCR = (1u << 17); /* 写 1 清除 TCIF2 */
    if (ok) {
        G_RX_TC++;
    }
}

void Reset_Handler(void) {
    /* 0. 预置 TX_BUF 发送源 "M5DMA" */
    const uint8_t msg[TX_LEN] = { 'M', '5', 'D', 'M', 'A', '!' };
    for (uint32_t i = 0u; i < TX_LEN; i++) {
        TX_BUF[i] = msg[i];
    }

    /* 1. RCC 时钟（镜像） */
    RCC_APB2ENR = (1u << 14);
    RCC_AHB1ENR = (1u << 22);

    /* 2. USART1 UE+TE+RE（写 CR1 使 TXE/TC 置位） */
    USART1_CR1 = CR1_UE | CR1_TE | CR1_RE;

    /* 3. 配置 DMA2_Stream7（TX：内存→外设 CHSEL4） */
    DMA2_SsPAR(7) = 0x40011004u; /* USART1_DR */
    DMA2_SsM0AR(7) = (uint32_t)TX_BUF;
    DMA2_SsNDTR(7) = TX_LEN;
    DMA2_SsCR(7) = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_MTM | DMA_CR_MINC | DMA_CR_CHSEL4;

    /* 4. 配置 DMA2_Stream2（RX：外设→内存 CHSEL4） */
    DMA2_SsPAR(2) = 0x40011004u; /* USART1_DR */
    DMA2_SsM0AR(2) = (uint32_t)RX_BUF;
    DMA2_SsNDTR(2) = RX_LEN;
    DMA2_SsCR(2) = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_PTM | DMA_CR_MINC | DMA_CR_CHSEL4;

    /* 5. NVIC：IRQ58/IRQ70 优先级 + 使能 */
    NVIC_IPR14 = (0xFu << 16); /* IRQ58 → IPR14 字节2 */
    NVIC_IPR17 = (0xFu << 16); /* IRQ70 → IPR17 字节2 */
    NVIC_ISER1 = (1u << 26);   /* IRQ58 */
    NVIC_ISER2 = (1u << 6);    /* IRQ70 */
    __asm volatile("cpsie i" ::: "memory");

    /* 6. 使能 USART DMA：写 CR3 时 TXE 已置位 → 立即发布 TX DMA 请求 */
    USART1_CR3 = CR3_DMAT | CR3_DMAR;

    dma_ready_hook();

    /* 7. 主循环：等待 TX/RX 两条流完成中断（handler 各递增计数） */
    while ((G_TX_TC == 0u) || (G_RX_TC == 0u)) {}
    dma_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
