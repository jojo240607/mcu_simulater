// M4 demo 固件：验证 DMA1 内存到内存（MEM2MEM）传输 + 传输完成中断。
//
// 场景：
// 1. 测试在 SRAM 固定地址预置源数组 SRC（0x20000100，16 个字）；
// 2. 固件配置 DMA1 Stream0：PAR=SRC（源）、M0AR=DST（目标，0x20000200）、
//    NDTR=16、字宽（PSIZE=MSIZE=字）、PINC+MINC 地址递增、DIR=内存到内存、
//    TCIE=1，最后写 CR.EN=1 启动；
// 3. 仿真器首个块 tick 判定传输完成：置 TCIF、挂起 IRQ11（DMA1_Stream0）、
//    run 间隙执行真实内存搬运（SRC → DST）；
// 4. DMA1_Stream0_IRQHandler：校验 DST==SRC → G_DMA_OK；清 LIFCR.TCIF0 → G_DMA_TC++；
// 5. 主线轮询 G_DMA_TC 达 1 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_DMA_TC  DMA 完成中断执行次数（期望 1）
//   0x20000004 G_DMA_OK  DST==SRC 校验结果（期望 1）
//   0x20000008 G_DONE    主线完成标记（期望 0xAAAAAAAA）
//   0x20000100 SRC       源数组（测试预置 16 字）
//   0x20000200 DST       目标数组（DMA 搬运，仿真器直接校验）

#include <stdint.h>

/* DMA1 @ 0x40026000 */
#define DMA1_LIFCR (*(volatile uint32_t *)0x40026008u) /* TCIF0 = bit5 */
#define DMA1_S0CR  (*(volatile uint32_t *)0x40026010u)
#define DMA1_S0NDTR (*(volatile uint32_t *)0x40026014u)
#define DMA1_S0PAR (*(volatile uint32_t *)0x40026018u)
#define DMA1_S0M0AR (*(volatile uint32_t *)0x4002601Cu)

/* NVIC（SCB 基址 0xE000E000）：IRQ11 = DMA1_Stream0 */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_IPR2  (*(volatile uint32_t *)0xE000E408u) /* IRQ8-11：IRQ11 = 字节3 */

/* DMA1 Stream0 CR 位（F407） */
#define DMA_CR_EN    (1u << 0)
#define DMA_CR_TCIE  (1u << 5)
#define DMA_CR_DIR_MM (2u << 6)   /* 内存到内存 */
#define DMA_CR_PINC  (1u << 9)
#define DMA_CR_MINC  (1u << 10)
#define DMA_CR_PSIZE_W (2u << 11) /* 字 */
#define DMA_CR_MSIZE_W (2u << 13) /* 字 */

/* 数据区（固定 SRAM 地址） */
#define SRC ((volatile const uint32_t *)0x20000100u)
#define DST ((volatile uint32_t *)0x20000200u)
#define NUM_WORDS 16

/* 结果区 */
#define G_DMA_TC (*(volatile uint32_t *)0x20000000u)
#define G_DMA_OK (*(volatile uint32_t *)0x20000004u)
#define G_DONE   (*(volatile uint32_t *)0x20000008u)

extern void Reset_Handler(void);
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

/* 向量表：系统异常 + IRQ0..IRQ28（DMA1_Stream0 = IRQ11 → index 27） */
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
    (uint32_t)DMA1_Stream0_IRQHandler, /* 27: IRQ11 = DMA1_Stream0 */
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
    (uint32_t)Default_Handler, /* 44: IRQ28 = TIM2 */
};

void DMA1_Stream0_IRQHandler(void) {
    /* 校验 DST == SRC */
    uint32_t ok = 1u;
    for (uint32_t i = 0u; i < NUM_WORDS; i++) {
        if (DST[i] != SRC[i]) {
            ok = 0u;
            break;
        }
    }
    G_DMA_OK = ok;

    DMA1_LIFCR = (1u << 5); /* 写 1 清除 TCIF0 */
    G_DMA_TC++;
}

void Reset_Handler(void) {
    /* 1. IRQ11（DMA1_Stream0）优先级 + 使能；IPR2 字节3（bits24..31）= 15 */
    NVIC_IPR2 = 0xFu << 24;
    NVIC_ISER0 = (1u << 11);
    __asm volatile("cpsie i" ::: "memory");

    /* 2. 配置 DMA1 Stream0：MEM2MEM 16 字，PAR=SRC → M0AR=DST，地址递增 */
    DMA1_S0PAR = (uint32_t)SRC;
    DMA1_S0M0AR = (uint32_t)DST;
    DMA1_S0NDTR = NUM_WORDS;
    DMA1_S0CR = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_MM |
                DMA_CR_PINC | DMA_CR_MINC | DMA_CR_PSIZE_W | DMA_CR_MSIZE_W;

    dma_ready_hook();

    /* 3. 主循环：等待 DMA 完成中断（handler 递增 G_DMA_TC） */
    while (G_DMA_TC == 0u) {}
    dma_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
