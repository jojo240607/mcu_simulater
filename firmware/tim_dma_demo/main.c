// M6 demo 固件：TIM2 更新事件 → DMA1 内存→外设突发装载 CCR 表端到端验证。
//
// 场景（TIM2 DIER.UDE + DCR 突发，接 DMA1 默认流）：
//   TIM2 更新事件（DIER.UDE 使能）→ 发布 TimUpdate → Machine 路由
//   DMA1_Stream5_Channel5（TIM2_UP，HAL 默认流）→ 内存→外设：把内存 CCR 表
//   经 DMAR 突发（DCR.DBA=13/DBL=3，突发长度 = 3+1 = 4）依次写入 CCR1..CCR4。
// 1. RCC 使能 TIM2（APB1ENR bit0）+ DMA1（AHB1ENR bit21，镜像）；
// 2. 准备 CCR 表（SRAM 固定地址 0x20000300：{0x1111,0x2222,0x3333,0x4444}）；
// 3. 配置 TIM2：ARR=1000、DCR=(13<<8)|3（DBA=13=CCR1，DBL=3）、DIER.UDE、CR1.CEN；
// 4. 配置 DMA1_Stream5（TX）：DIR=内存→外设(01)、CHSEL=5、PAR=TIM2_DMAR、
//    M0AR=CCR_TABLE、NDTR=4、MSIZE=字(10)、MINC、TCIE，写 EN 启动；
// 5. NVIC 使能 IRQ16（DMA1_Stream5）；cpsie i；
// 6. 主线轮询：TIM2 溢出 → 更新事件 → DMA 突发搬 4 字 → TCIF → IRQ16 →
//    DMA1_Stream5_IRQHandler 校验 CCR1..CCR4 == 表 → G_DMA_TC++；清 HIFCR；
// 7. 主线轮询 G_DMA_TC 达 1 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_DMA_TC DMA1_Stream5 完成中断执行次数（期望 1）
//   0x20000004 G_DONE   主线完成标记（期望 0xAAAAAAAA）
//   0x40000034..0x40 TIM2 CCR1..CCR4（DMA 突发装载结果，仿真器直接校验）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840u) /* bit0 = TIM2 时钟 */
#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830u) /* bit21 = DMA1 时钟 */

/* TIM2 @ 0x40000000 */
#define TIM2_CR1  (*(volatile uint32_t *)0x40000000u)
#define TIM2_DIER (*(volatile uint32_t *)0x4000000Cu)
#define TIM2_EGR  (*(volatile uint32_t *)0x40000014u)
#define TIM2_PSC  (*(volatile uint32_t *)0x40000028u)
#define TIM2_ARR  (*(volatile uint32_t *)0x4000002Cu)
#define TIM2_CCR1 (*(volatile uint32_t *)0x40000034u)
#define TIM2_CCR2 (*(volatile uint32_t *)0x40000038u)
#define TIM2_CCR3 (*(volatile uint32_t *)0x4000003Cu)
#define TIM2_CCR4 (*(volatile uint32_t *)0x40000040u)
#define TIM2_DCR  (*(volatile uint32_t *)0x40000048u)
#define TIM2_DMAR (*(volatile uint32_t *)0x4000004Cu)

/* DMA1 @ 0x40026000：HIFCR@0x0C（流4-7）；流5 基址 0x40026010 + 5*0x18 = 0x40026088 */
#define DMA1_HIFCR (*(volatile uint32_t *)0x4002600Cu)
#define DMA1_S5CR   (*(volatile uint32_t *)0x40026088u)
#define DMA1_S5NDTR (*(volatile uint32_t *)0x4002608Cu)
#define DMA1_S5PAR  (*(volatile uint32_t *)0x40026090u)
#define DMA1_S5M0AR (*(volatile uint32_t *)0x40026094u)

/* NVIC（SCB 基址 0xE000E000）：
   IRQ16 = DMA1_Stream5 → ISER0 bit16、IPR4 字节0（bits0-7） */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_IPR4  (*(volatile uint32_t *)0xE000E410u)

/* TIM 位 */
#define TIM_DIER_UDE (1u << 8) /* 更新 DMA 请求使能 */
#define TIM_CR1_CEN  (1u << 0)
#define TIM_DCR_DBA_CCR1 (13u << 8) /* DBA=13：CCR1 字偏移 */
#define TIM_DCR_DBL_3    (3u)       /* DBL=3：突发长度 = 3+1 = 4（CCR1..CCR4） */

/* DMA CR 位（F407） */
#define DMA_CR_EN       (1u << 0)
#define DMA_CR_TCIE     (1u << 4)
#define DMA_CR_DIR_MTP  (1u << 6)      /* 01 = 内存→外设（TX） */
#define DMA_CR_MINC     (1u << 10)
#define DMA_CR_MSIZE_W  (2u << 13)     /* MSIZE = 10 = 字（32 位） */
#define DMA_CR_CHSEL5   (5u << 25)     /* CHSEL = 5（TIM2_UP 通道） */

/* CCR 表（固定 SRAM 地址） */
#define CCR_TABLE ((volatile uint32_t *)0x20000300u)
#define CCR_LEN   4u

/* 结果区 */
#define G_DMA_TC (*(volatile uint32_t *)0x20000000u)
#define G_DONE   (*(volatile uint32_t *)0x20000004u)

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
    (uint32_t)Default_Handler, /* 31: IRQ15 = DMA1_Stream4 */
    (uint32_t)DMA1_Stream5_IRQHandler, /* 32: IRQ16 = DMA1_Stream5 */
};

void DMA1_Stream5_IRQHandler(void) {
    /* TX 完成：校验 CCR1..CCR4 经 DMAR 突发装载为表内容 */
    const uint32_t expect[CCR_LEN] = { 0x1111u, 0x2222u, 0x3333u, 0x4444u };
    const uint32_t ccr[CCR_LEN] = {
        TIM2_CCR1, TIM2_CCR2, TIM2_CCR3, TIM2_CCR4,
    };
    uint32_t ok = 1u;
    for (uint32_t i = 0u; i < CCR_LEN; i++) {
        if (ccr[i] != expect[i]) {
            ok = 0u;
            break;
        }
    }
    DMA1_HIFCR = (1u << 11); /* 写 1 清除 TCIF5（HISR 偏移 6 + 5 = 11） */
    if (ok) {
        G_DMA_TC++;
    }
}

void Reset_Handler(void) {
    /* 1. RCC 时钟（镜像） */
    RCC_APB1ENR = (1u << 0);  /* TIM2 */
    RCC_AHB1ENR = (1u << 21); /* DMA1 */

    /* 2. 准备 CCR 表（SRAM 固定地址） */
    {
        static const uint32_t aCC[CCR_LEN] = { 0x1111u, 0x2222u, 0x3333u, 0x4444u };
        for (uint32_t i = 0u; i < CCR_LEN; i++) {
            CCR_TABLE[i] = aCC[i];
        }
    }

    /* 3. 配置 TIM2：ARR=1000、DCR 突发基址/长度、UDE、CEN */
    TIM2_PSC = 0u;
    TIM2_ARR = 1000u;
    TIM2_DCR = TIM_DCR_DBA_CCR1 | TIM_DCR_DBL_3;
    TIM2_DIER = TIM_DIER_UDE;
    TIM2_EGR = 1u; /* UG：软件更新（准备） */
    TIM2_CR1 = TIM_CR1_CEN;

    /* 4. 配置 DMA1_Stream5（TX：内存→外设 CHSEL5，字宽，MINC，TCIE） */
    DMA1_S5PAR = (uint32_t)&TIM2_DMAR;
    DMA1_S5M0AR = (uint32_t)CCR_TABLE;
    DMA1_S5NDTR = CCR_LEN;
    DMA1_S5CR = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_MTP | DMA_CR_MINC
              | DMA_CR_MSIZE_W | DMA_CR_CHSEL5;

    /* 5. NVIC：IRQ16 优先级 + 使能（IRQ16 = DMA1_Stream5） */
    NVIC_IPR4 = (0xFu << 0); /* IRQ16 → IPR4 字节0 */
    NVIC_ISER0 = (1u << 16); /* IRQ16 */
    __asm volatile("cpsie i" ::: "memory");

    dma_ready_hook();

    /* 6. 主循环：TIM2 溢出 → 更新事件 DMA 请求 → 突发装载 CCR → 中断校验 */
    while (G_DMA_TC == 0u) {}
    dma_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
