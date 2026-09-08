// M5 demo 固件：I2C1 ↔ DMA1 外设方向传输（HAL 默认流）端到端验证。
//
// 场景（I2C1 使能 CR2.DMAEN，接 DMA1 默认流）：
//   TX：DMA1_Stream6_Channel1（内存→外设）把 TX_BUF 搬运到 I2C1_DR → 虚拟从机/测试；
//   RX：DMA1_Stream0_Channel1（外设→内存）把 I2C1_DR 搬到 RX_BUF（测试注入 4 字节）。
// 1. RCC 使能 I2C1（APB1ENR bit21）+ DMA1（AHB1ENR bit21，镜像）；
// 2. I2C1 CR1.PE 使能（上升沿置 TxE，仿真语义）；
// 3. 配置 DMA1_Stream6（TX）：DIR=内存→外设(01)、CHSEL=1、PAR=I2C1_DR、
//    M0AR=TX_BUF、NDTR=TX_LEN、MINC、TCIE，写 EN 启动；
// 4. 配置 DMA1_Stream0（RX）：DIR=外设→内存(00)、CHSEL=1、PAR=I2C1_DR、
//    M0AR=RX_BUF、NDTR=RX_LEN、MINC、TCIE，写 EN 启动；
// 5. NVIC 使能 IRQ17（DMA1_Stream6）+ IRQ11（DMA1_Stream0）；
// 6. I2C1 CR2.DMAEN：写 CR2 时 TxE 已置位 → 发布 TX DMA 请求 →
//    路由到 Stream6 → 一次搬完 TX_BUF（每字节发 I2cByte 事件）；RX 每收 1 字节发 1 次请求；
// 7. 两条流完成后各自 TC 中断：DMA1_Stream6_IRQHandler 校验发送完成 → G_TX_TC++；
//    DMA1_Stream0_IRQHandler 校验 RX_BUF 内容 → G_RX_TC++；清 HIFCR/LIFCR；
// 8. 主线轮询 G_TX_TC && G_RX_TC 达 1 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_TX_TC  DMA1_Stream6 TX 完成中断执行次数（期望 1）
//   0x20000004 G_RX_TC  DMA1_Stream0 RX 完成中断执行次数（期望 1）
//   0x20000008 G_DONE   主线完成标记（期望 0xAAAAAAAA）
//   0x20000100 TX_BUF   发送源（固件预置 "I2CDMA" 6 字节，经 DMA TX 发送）
//   0x20000200 RX_BUF   接收目标（测试注入 4 字节，经 DMA RX 搬运，仿真器直接校验）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840u) /* bit21 = I2C1 时钟 */
#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830u) /* bit21 = DMA1 时钟 */

/* I2C1 @ 0x40005400 */
#define I2C1_CR1  (*(volatile uint32_t *)0x40005400u)
#define I2C1_CR2  (*(volatile uint32_t *)0x40005404u)
#define I2C1_DR   (*(volatile uint32_t *)0x40005410u)
#define I2C1_SR1  (*(volatile uint32_t *)0x40005414u)

/* DMA1 @ 0x40026000：LIFCR@0x08（流0-3）、HIFCR@0x0C（流4-7）；
   流 s 寄存器基址 0x40026010 + s*0x18 */
#define DMA1_LIFCR (*(volatile uint32_t *)0x40026008u)
#define DMA1_HIFCR (*(volatile uint32_t *)0x4002600Cu)
#define DMA1_STREAM(s) ((volatile uint32_t *)(0x40026010u + (s) * 0x18u))
#define DMA1_SsCR(s)  DMA1_STREAM(s)[0]
#define DMA1_SsNDTR(s) DMA1_STREAM(s)[1]
#define DMA1_SsPAR(s) DMA1_STREAM(s)[2]
#define DMA1_SsM0AR(s) DMA1_STREAM(s)[3]

/* NVIC（SCB 基址 0xE000E000）：
   IRQ11 = DMA1_Stream0 → ISER0 bit11、IPR2 字节3（bits24-31）
   IRQ17 = DMA1_Stream6 → ISER0 bit17、IPR4 字节1（bits8-15） */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_IPR2  (*(volatile uint32_t *)0xE000E408u)
#define NVIC_IPR4  (*(volatile uint32_t *)0xE000E410u)

/* I2C CR 位 */
#define CR1_PE      (1u << 0)
#define CR2_DMAEN   (1u << 11)
#define SR1_TXE     (1u << 7)

/* DMA CR 位（F407） */
#define DMA_CR_EN      (1u << 0)
#define DMA_CR_TCIE    (1u << 4)
#define DMA_CR_DIR_MTM (1u << 6)   /* 01 = 内存→外设（TX） */
#define DMA_CR_DIR_PTM (0u << 6)   /* 00 = 外设→内存（RX） */
#define DMA_CR_MINC    (1u << 10)
#define DMA_CR_CHSEL1  (1u << 25)  /* CHSEL = 1（I2C1 通道） */

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
void DMA1_Stream6_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);

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

/* 向量表：系统异常 + IRQ0..IRQ31（IRQ11 = DMA1_Stream0 → index 27，
   IRQ17 = DMA1_Stream6 → index 33） */
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
    (uint32_t)DMA1_Stream0_IRQHandler, /* 27: IRQ11 = DMA1_Stream0 (RX) */
    (uint32_t)Default_Handler, /* 28 */
    (uint32_t)Default_Handler, /* 29 */
    (uint32_t)Default_Handler, /* 30 */
    (uint32_t)Default_Handler, /* 31 */
    (uint32_t)Default_Handler, /* 32 */
    (uint32_t)DMA1_Stream6_IRQHandler, /* 33: IRQ17 = DMA1_Stream6 (TX) */
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
    (uint32_t)Default_Handler, /* 74 */
    (uint32_t)Default_Handler, /* 75 */
    (uint32_t)Default_Handler, /* 76 */
    (uint32_t)Default_Handler, /* 77 */
    (uint32_t)Default_Handler, /* 78 */
    (uint32_t)Default_Handler, /* 79 */
    (uint32_t)Default_Handler, /* 80 */
    (uint32_t)Default_Handler, /* 81 */
    (uint32_t)Default_Handler, /* 82 */
    (uint32_t)Default_Handler, /* 83 */
    (uint32_t)Default_Handler, /* 84 */
    (uint32_t)Default_Handler, /* 85 */
    (uint32_t)Default_Handler, /* 86 */
};

void DMA1_Stream6_IRQHandler(void) {
    /* TX 完成：发送已结束（NDTR 应已清零）→ G_TX_TC++ */
    DMA1_HIFCR = (1u << 17); /* 写 1 清除 TCIF6（HISR 偏移 (6-4)*6 + 5 = 17） */
    G_TX_TC++;
}

void DMA1_Stream0_IRQHandler(void) {
    /* RX 完成：校验 RX_BUF 收到 4 字节 */
    uint32_t ok = 1u;
    for (uint32_t i = 0u; i < RX_LEN; i++) {
        if (RX_BUF[i] != (uint8_t)('a' + i)) {
            ok = 0u;
            break;
        }
    }
    DMA1_LIFCR = (1u << 5); /* 写 1 清除 TCIF0（LISR 偏移 0 + 5 = 5） */
    if (ok) {
        G_RX_TC++;
    }
}

void Reset_Handler(void) {
    /* 0. 预置 TX_BUF 发送源 "I2CDMA" */
    const uint8_t msg[TX_LEN] = { 'I', '2', 'C', 'D', 'M', 'A' };
    for (uint32_t i = 0u; i < TX_LEN; i++) {
        TX_BUF[i] = msg[i];
    }

    /* 1. RCC 时钟（镜像） */
    RCC_APB1ENR = (1u << 21); /* I2C1 */
    RCC_AHB1ENR = (1u << 21); /* DMA1 */

    /* 2. I2C1 PE 使能（上升沿置 TxE，仿真语义） */
    I2C1_CR1 = CR1_PE;

    /* 3. 配置 DMA1_Stream6（TX：内存→外设 CHSEL1） */
    DMA1_SsPAR(6) = 0x40005410u; /* I2C1_DR */
    DMA1_SsM0AR(6) = (uint32_t)TX_BUF;
    DMA1_SsNDTR(6) = TX_LEN;
    DMA1_SsCR(6) = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_MTM | DMA_CR_MINC | DMA_CR_CHSEL1;

    /* 4. 配置 DMA1_Stream0（RX：外设→内存 CHSEL1） */
    DMA1_SsPAR(0) = 0x40005410u; /* I2C1_DR */
    DMA1_SsM0AR(0) = (uint32_t)RX_BUF;
    DMA1_SsNDTR(0) = RX_LEN;
    DMA1_SsCR(0) = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_PTM | DMA_CR_MINC | DMA_CR_CHSEL1;

    /* 5. NVIC：IRQ11/IRQ17 优先级 + 使能 */
    NVIC_IPR2 = (0xFu << 24); /* IRQ11 → IPR2 字节3 */
    NVIC_IPR4 = (0xFu << 8);  /* IRQ17 → IPR4 字节1 */
    NVIC_ISER0 = (1u << 11);  /* IRQ11 */
    NVIC_ISER0 = (1u << 17);  /* IRQ17 */
    __asm volatile("cpsie i" ::: "memory");

    /* 6. 使能 I2C DMA：写 CR2 时 TxE 已置位 → 立即发布 TX DMA 请求 */
    I2C1_CR2 = CR2_DMAEN;

    dma_ready_hook();

    /* 7. 主循环：等待 TX/RX 两条流完成中断（handler 各递增计数） */
    while ((G_TX_TC == 0u) || (G_RX_TC == 0u)) {}
    dma_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
