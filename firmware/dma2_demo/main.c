// M4 demo 固件：验证 DMA2 内存到内存（MEM2MEM）传输 + 传输完成中断。
//
// 场景（与 dma_demo 完全同构，仅换 DMA2 + IRQ56）：
// 1. 测试在 SRAM 固定地址预置源数组 SRC（0x20000100，16 个字）；
// 2. 固件配置 DMA2 Stream0：PAR=SRC（源）、M0AR=DST（目标，0x20000200）、
//    NDTR=16、字宽（PSIZE=MSIZE=字）、PINC+MINC 地址递增、DIR=内存到内存、
//    TCIE=1，最后写 CR.EN=1 启动；
// 3. 仿真器首个块 tick 判定传输完成：置 TCIF、挂起 IRQ56（DMA2_Stream0）、
//    run 间隙执行真实内存搬运（SRC → DST）；
// 4. DMA2_Stream0_IRQHandler：校验 DST==SRC → G_DMA_OK；清 LIFCR.TCIF0 → G_DMA_TC++；
// 5. 主线轮询 G_DMA_TC 达 1 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_DMA_TC  DMA 完成中断执行次数（期望 1）
//   0x20000004 G_DMA_OK  DST==SRC 校验结果（期望 1）
//   0x20000008 G_DONE    主线完成标记（期望 0xAAAAAAAA）
//   0x20000100 SRC       源数组（测试预置 16 字）
//   0x20000200 DST       目标数组（DMA 搬运，仿真器直接校验）

#include <stdint.h>

/* DMA2 @ 0x40026400 */
#define DMA2_LIFCR (*(volatile uint32_t *)0x40026408u) /* TCIF0 = bit5 */
#define DMA2_S0CR  (*(volatile uint32_t *)0x40026410u)
#define DMA2_S0NDTR (*(volatile uint32_t *)0x40026414u)
#define DMA2_S0PAR (*(volatile uint32_t *)0x40026418u)
#define DMA2_S0M0AR (*(volatile uint32_t *)0x4002641Cu)

/* NVIC（SCB 基址 0xE000E000）：IRQ56 = DMA2_Stream0
   ISER1 @ 0xE000E104 bit24（56-32）；IPR14 @ 0xE000E438 字节0（56%4=0 → bits0..7） */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u)
#define NVIC_IPR14 (*(volatile uint32_t *)0xE000E438u)

/* DMA2 Stream0 CR 位（F407，与 DMA1 相同） */
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

/* 向量表：系统异常 + IRQ0..IRQ56（DMA2_Stream0 = IRQ56 → index 72） */
__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    0x20001000u,               /* 0: 初始 SP（SRAM 内） */
    (uint32_t)Reset_Handler,   /* 1: Reset */
    (uint32_t)Default_Handler, /* 2: NMI */
    (uint32_t)Default_Handler, /* 3: HardFault */
    (uint32_t)Default_Handler, /* 4: MemManage */
    (uint32_t)Default_Handler, /* 5: BusFault */
    (uint32_t)Default_Handler, /* 6: UsageFault */
    (uint32_t)Default_Handler, /* 7 */
    (uint32_t)Default_Handler, /* 8 */
    (uint32_t)Default_Handler, /* 9 */
    (uint32_t)Default_Handler, /* 10 */
    (uint32_t)Default_Handler, /* 11 */
    (uint32_t)Default_Handler, /* 12 */
    (uint32_t)Default_Handler, /* 13 */
    (uint32_t)Default_Handler, /* 14 */
    (uint32_t)Default_Handler, /* 15: 系统异常区 */
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
    (uint32_t)DMA2_Stream0_IRQHandler, /* 72: IRQ56 = DMA2_Stream0 */
};

void DMA2_Stream0_IRQHandler(void) {
    /* 校验 DST == SRC */
    uint32_t ok = 1u;
    for (uint32_t i = 0u; i < NUM_WORDS; i++) {
        if (DST[i] != SRC[i]) {
            ok = 0u;
            break;
        }
    }
    G_DMA_OK = ok;

    DMA2_LIFCR = (1u << 5); /* 写 1 清除 TCIF0 */
    G_DMA_TC++;
}

void Reset_Handler(void) {
    /* 1. IRQ56（DMA2_Stream0）优先级 + 使能；IPR14 字节0（bits0..7）= 15 */
    NVIC_IPR14 = 0xFu << 0;
    NVIC_ISER1 = (1u << 24);
    __asm volatile("cpsie i" ::: "memory");

    /* 2. 配置 DMA2 Stream0：MEM2MEM 16 字，PAR=SRC → M0AR=DST，地址递增 */
    DMA2_S0PAR = (uint32_t)SRC;
    DMA2_S0M0AR = (uint32_t)DST;
    DMA2_S0NDTR = NUM_WORDS;
    DMA2_S0CR = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_MM |
                DMA_CR_PINC | DMA_CR_MINC | DMA_CR_PSIZE_W | DMA_CR_MSIZE_W;

    dma_ready_hook();

    /* 3. 主循环：等待 DMA 完成中断（handler 递增 G_DMA_TC） */
    while (G_DMA_TC == 0u) {}
    dma_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
