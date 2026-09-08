// M7 demo 固件：DAC1 软件触发 + 定时器触发 DMA（内存→外设）端到端验证。
//
// 场景（DAC1 双触发源 + DMA1_Stream5_Channel7 内存→外设）：
//   Phase A（软件触发）：
//     1. DAC1 CR = EN1|TEN1（软件触发 = TEN1 置位，对应 HAL DAC_TRIGGER_SOFTWARE）；
//     2. 写 DHR12R1 = 0x0321 → 写 SWTRIGR.SWTRIG1 → DHR→DOR 锁存 → DOR1 = 0x0321
//        （转换完成发布 DacLevel{level=0x0321}）；
//     3. 固件回读 DOR1 校验 → G_SW_OK = 1。
//   Phase B（定时器触发 + DMA 内存→外设，DAC1_CH1 → DMA1_Stream5_Channel7）：
//     1. DAC1 CR = EN1|TEN1|DMAEN1（TSEL 高 2 位 = 00 → TIM6 TRGO）；
//     2. 配置 DMA1_Stream5（TX：DIR=内存→外设、CHSEL=7、PAR=DAC_DHR12R1、
//        M0AR=DAC_BUF、NDTR=DAC_LEN、MSIZE=半字、MINC、TCIE），写 EN 启动；
//     3. NVIC 使能 IRQ16（DMA1_Stream5，向量 index 32）；
//     4. TIM6：ARR=1 + DIER.UDE（更新事件 → DAC 触发 DMA 请求）+ CR1.CEN 启动；
//     5. TIM6 更新 → TimUpdate{port=6} → DAC 锁存 DHR→DOR + 路由 Stream5 →
//        run 间隙一次搬完 4 个半字（M0AR → dma_write_dr → DHR 锁存 + DacLevel 发布）
//        → NDTR 归零 → EN 自清 + TCIF + IRQ16；
//     6. DMA1_Stream5_IRQHandler 回读 DOR1（应为最后一个搬运值 0x0DDD）→ G_DMA_TC++；
//     7. 主线轮询 G_DMA_TC 达 1 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_SW_OK   Phase A 软件触发回读校验通过（期望 1）
//   0x20000004 G_DMA_TC  DMA1_Stream5 完成中断执行次数（期望 1）
//   0x20000008 G_DONE    主线完成标记（期望 0xAAAAAAAA）
//   0x20000200 DAC_BUF   发送源（DMA 半字搬运到 DHR12R1，仿真器经 DacLevel 校验）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830u) /* bit21 = DMA1 时钟 */
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840u) /* bit29 = DAC、bit4 = TIM6 时钟 */

/* DAC1 @ 0x40007400 */
#define DAC1_CR      (*(volatile uint32_t *)0x40007400u)
#define DAC1_SWTRIGR (*(volatile uint32_t *)0x40007404u)
#define DAC1_DHR12R1 (*(volatile uint32_t *)0x40007408u)
#define DAC1_DOR1    (*(volatile uint32_t *)0x4000742Cu)

/* TIM6 @ 0x40001000 */
#define TIM6_CR1 (*(volatile uint32_t *)0x40001000u)
#define TIM6_DIER (*(volatile uint32_t *)0x4000100Cu)
#define TIM6_ARR  (*(volatile uint32_t *)0x4000102Cu)

/* DMA1 @ 0x40026000：HIFCR@0x0C（流4-7）；流 5 基址 0x40026010 + 5*0x18 = 0x40026088 */
#define DMA1_HIFCR (*(volatile uint32_t *)0x4002600Cu)
#define DMA1_S5CR   (*(volatile uint32_t *)0x40026088u)
#define DMA1_S5NDTR (*(volatile uint32_t *)0x4002608Cu)
#define DMA1_S5PAR  (*(volatile uint32_t *)0x40026090u)
#define DMA1_S5M0AR (*(volatile uint32_t *)0x40026094u)

/* NVIC（SCB 基址 0xE000E000）：
   IRQ16 = DMA1_Stream5 → ISER0 bit16、IPR4 字节0（bits0-7） */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_IPR4  (*(volatile uint32_t *)0xE000E410u)

/* DAC CR 位（TSEL 高 2 位为有效触发源：00 = TIM6；软件触发 = TENx 置位 + SWTRIGR） */
#define DAC_CR_EN1    (1u << 0)
#define DAC_CR_TEN1   (1u << 2)
#define DAC_CR_DMAEN1 (1u << 12)

/* DMA CR 位（F407） */
#define DMA_CR_EN      (1u << 0)
#define DMA_CR_TCIE    (1u << 4)
#define DMA_CR_DIR_M2P (1u << 6)      /* 01 = 内存→外设（TX） */
#define DMA_CR_MINC    (1u << 10)
#define DMA_CR_MSIZE_H (1u << 13)     /* MSIZE = 01 = 半字（16 位） */
#define DMA_CR_CHSEL7  (7u << 25)     /* CHSEL = 7（DAC1_CH1 通道） */

/* TIM DIER/CR1 位 */
#define TIM_DIER_UDE (1u << 8) /* 更新 DMA 请求使能 */
#define TIM_CR1_CEN  (1u << 0)

/* 数据区（固定 SRAM 地址） */
#define DAC_BUF ((volatile uint16_t *)0x20000200u)
#define DAC_LEN 4u

/* 结果区 */
#define G_SW_OK  (*(volatile uint32_t *)0x20000000u)
#define G_DMA_TC (*(volatile uint32_t *)0x20000004u)
#define G_DONE   (*(volatile uint32_t *)0x20000008u)

extern void Reset_Handler(void);
void DMA1_Stream5_IRQHandler(void);

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

/* 向量表：系统异常 + IRQ0..IRQ16（IRQ16 = DMA1_Stream5 → index 32） */
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
    (uint32_t)DMA1_Stream5_IRQHandler, /* 32: IRQ16 = DMA1_Stream5 (TX) */
};

void DMA1_Stream5_IRQHandler(void) {
    /* TX 完成：DOR1 应锁存最后一个搬运值（DMA 半字搬运结果回读） */
    uint32_t ok = (DAC1_DOR1 == DAC_BUF[DAC_LEN - 1u]) ? 1u : 0u;
    DMA1_HIFCR = (1u << 11); /* 写 1 清除 TCIF5（HISR 偏移 6 + 5 = 11） */
    if (ok) {
        G_DMA_TC++;
    }
}

void Reset_Handler(void) {
    /* 1. RCC 时钟（镜像） */
    RCC_AHB1ENR = (1u << 21); /* DMA1 */
    RCC_APB1ENR = (1u << 29) | (1u << 4); /* DAC + TIM6 */

    /* Phase A：软件触发转换（TEN1 置位，DHR→DOR 锁存 + DacLevel 发布） */
    DAC1_CR = DAC_CR_EN1 | DAC_CR_TEN1;
    DAC1_DHR12R1 = 0x0321u;
    DAC1_SWTRIGR = (1u << 0); /* SWTRIG1 */
    if (DAC1_DOR1 == 0x0321u) {
        G_SW_OK = 1u;
    }

    /* Phase B：定时器触发（TSEL 高 2 位 = 00 → TIM6 TRGO）+ DMA 内存→外设 */
    DAC1_CR = DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_DMAEN1;

    /* 配置 DMA1_Stream5（TX：内存→外设 CHSEL7，半字） */
    DMA1_S5PAR = 0x40007408u; /* DAC1_DHR12R1 */
    DMA1_S5M0AR = (uint32_t)DAC_BUF;
    DMA1_S5NDTR = DAC_LEN;
    DMA1_S5CR = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_M2P | DMA_CR_MINC
              | DMA_CR_MSIZE_H | DMA_CR_CHSEL7;

    /* NVIC：IRQ16 优先级 + 使能 */
    NVIC_IPR4 = (0xFu << 0); /* IRQ16 → 字节0 */
    NVIC_ISER0 = (1u << 16); /* IRQ16 */
    __asm volatile("cpsie i" ::: "memory");

    dma_ready_hook();

    /* 配置 TIM6：ARR=1 + UDE（更新事件 → DAC 触发 DMA 请求），启动计数 */
    TIM6_ARR = 1u;
    TIM6_DIER = TIM_DIER_UDE;
    TIM6_CR1 = TIM_CR1_CEN;

    /* 主循环：等待 DMA 完成中断（handler 回读 DOR1 校验并递增计数） */
    while (G_DMA_TC == 0u) {}
    TIM6_CR1 &= ~TIM_CR1_CEN; /* 完成：停止 TIM6 触发，避免持续锁存陈旧 DHR */
    dma_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
