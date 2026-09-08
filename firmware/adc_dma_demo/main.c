// M5 demo 固件：ADC1 ↔ DMA2 外设→内存传输（HAL 默认流）端到端验证。
//
// 场景（ADC1 使能 CR2.ADON|DMA + CR1.EOCIE，接 DMA2 默认流）：
//   RX：DMA2_Stream0_Channel0（外设→内存）把 ADC1_DR 搬到 ADC_BUF
//       （测试逐次注入 AdcValue 采样值，每次搬 1 个半字）。
// 1. RCC 使能 ADC1（APB2ENR bit8）+ DMA2（AHB1ENR bit22，镜像）；
// 2. ADC1 CR2.ADON 使能；
// 3. 配置 DMA2_Stream0（RX）：DIR=外设→内存(00)、CHSEL=0、PAR=ADC1_DR、
//    M0AR=ADC_BUF、NDTR=ADC_LEN、MSIZE=半字(01)、MINC、TCIE，写 EN 启动；
// 4. NVIC 使能 IRQ56（DMA2_Stream0）；
// 5. ADC1 CR1.EOCIE + CR2.DMA：测试注入 AdcValue{port=1} → feed_value 置 EOC →
//    路由 Stream0 → 每采样值搬 1 个半字到 ADC_BUF；
// 6. DMA2_Stream0_IRQHandler 校验 ADC_BUF 内容 → G_DMA_TC++；清 LIFCR；
// 7. 主线轮询 G_DMA_TC 达 1 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_DMA_TC DMA2_Stream0 完成中断执行次数（期望 1）
//   0x20000004 G_DONE   主线完成标记（期望 0xAAAAAAAA）
//   0x20000200 ADC_BUF  接收目标（测试注入 4 个 12 位采样值，经 DMA 半字搬运，
//                        仿真器直接校验）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844u) /* bit8 = ADC1 时钟 */
#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830u) /* bit22 = DMA2 时钟 */

/* ADC1 @ 0x40012000 */
#define ADC1_SR  (*(volatile uint32_t *)0x40012000u)
#define ADC1_CR1 (*(volatile uint32_t *)0x40012004u)
#define ADC1_CR2 (*(volatile uint32_t *)0x40012008u)
#define ADC1_DR  (*(volatile uint32_t *)0x4001204Cu)

/* DMA2 @ 0x40026400：LIFCR@0x08（流0-3）；
   流 s 寄存器基址 0x40026410 + s*0x18 */
#define DMA2_LIFCR (*(volatile uint32_t *)0x40026408u)
#define DMA2_STREAM(s) ((volatile uint32_t *)(0x40026410u + (s) * 0x18u))
#define DMA2_SsCR(s)   DMA2_STREAM(s)[0]
#define DMA2_SsNDTR(s) DMA2_STREAM(s)[1]
#define DMA2_SsPAR(s)  DMA2_STREAM(s)[2]
#define DMA2_SsM0AR(s) DMA2_STREAM(s)[3]
#define DMA2_S0CR   DMA2_SsCR(0)
#define DMA2_S0NDTR DMA2_SsNDTR(0)
#define DMA2_S0PAR  DMA2_SsPAR(0)
#define DMA2_S0M0AR DMA2_SsM0AR(0)

/* NVIC（SCB 基址 0xE000E000）：
   IRQ56 = DMA2_Stream0 → ISER1 bit24、IPR14 字节0（bits0-7） */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u)
#define NVIC_IPR14 (*(volatile uint32_t *)0xE000E438u)

/* ADC CR 位 */
#define CR1_EOCIE (1u << 5)
#define CR2_ADON  (1u << 0)
#define CR2_DMA   (1u << 8)

/* DMA CR 位（F407） */
#define DMA_CR_EN      (1u << 0)
#define DMA_CR_TCIE    (1u << 4)
#define DMA_CR_DIR_PTM (0u << 6)      /* 00 = 外设→内存（RX） */
#define DMA_CR_MINC    (1u << 10)
#define DMA_CR_MSIZE_H (1u << 13)     /* MSIZE = 01 = 半字（16 位） */
#define DMA_CR_CHSEL0  (0u << 25)     /* CHSEL = 0（ADC1 通道） */

/* 数据区（固定 SRAM 地址） */
#define ADC_BUF ((volatile uint16_t *)0x20000200u)
#define ADC_LEN 4u

/* 结果区 */
#define G_DMA_TC (*(volatile uint32_t *)0x20000000u)
#define G_DONE   (*(volatile uint32_t *)0x20000004u)

extern void Reset_Handler(void);
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

/* 向量表：系统异常 + IRQ0..IRQ56（IRQ56 = DMA2_Stream0 → index 72） */
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
};

void DMA2_Stream0_IRQHandler(void) {
    /* RX 完成：校验 ADC_BUF 收到 4 个采样值（测试注入序列） */
    const uint16_t expect[ADC_LEN] = { 0x0123u, 0x0456u, 0x0789u, 0x0ABCu };
    uint32_t ok = 1u;
    for (uint32_t i = 0u; i < ADC_LEN; i++) {
        if (ADC_BUF[i] != expect[i]) {
            ok = 0u;
            break;
        }
    }
    DMA2_LIFCR = (1u << 5); /* 写 1 清除 TCIF0（LISR 偏移 0 + 5 = 5） */
    if (ok) {
        G_DMA_TC++;
    }
}

void Reset_Handler(void) {
    /* 1. RCC 时钟（镜像） */
    RCC_APB2ENR = (1u << 8);  /* ADC1 */
    RCC_AHB1ENR = (1u << 22); /* DMA2 */

    /* 2. ADC1 CR2.ADON 使能 */
    ADC1_CR2 = CR2_ADON;

    /* 3. 配置 DMA2_Stream0（RX：外设→内存 CHSEL0，半字） */
    DMA2_S0PAR = 0x4001204Cu; /* ADC1_DR */
    DMA2_S0M0AR = (uint32_t)ADC_BUF;
    DMA2_S0NDTR = ADC_LEN;
    DMA2_S0CR = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_PTM | DMA_CR_MINC
              | DMA_CR_MSIZE_H | DMA_CR_CHSEL0;

    /* 4. NVIC：IRQ56 优先级 + 使能 */
    NVIC_IPR14 = (0xFu << 0); /* IRQ56 → 字节0 */
    NVIC_ISER1 = (1u << 24);  /* IRQ56 */
    __asm volatile("cpsie i" ::: "memory");

    /* 5. ADC1：EOCIE 中断 + DMA 模式使能（转换完成 → DMA 请求） */
    ADC1_CR1 = CR1_EOCIE;
    ADC1_CR2 |= CR2_DMA;

    dma_ready_hook();

    /* 6. 主循环：等待 DMA 完成中断（handler 校验并递增计数） */
    while (G_DMA_TC == 0u) {}
    dma_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
