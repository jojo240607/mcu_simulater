// M4 demo 固件：验证 DMA2 四流并发内存到内存（MEM2MEM）传输 + 各自完成中断。
//
// 场景（四流 Stream0-3 同一 tick 并发完成，互不干扰）：
// 1. 测试在 SRAM 固定地址预置 4 段源数组 SRC0-3（0x20000100..0x200001C0，各 16 字）；
// 2. 固件配置 DMA2 Stream0-3（每条流：PAR=SRCn、M0AR=DSTn、NDTR=16、字宽、
//    PINC+MINC、DIR=内存到内存、TCIE=1，最后写 CR.EN=1 同时启动）；
// 3. 仿真器首个块 tick 遍历 8 流判定完成：各自置 TCIF、按流挂起 IRQ56-59，
//    run 间隙按 pending 位图逐流执行真实内存搬运（SRCn → DSTn）；
// 4. 四个 handler 各自校验 DSTn==SRCn → G_DMA_OK 置对应位；清 LIFCR.TCIFn → G_DMA_TC++；
// 5. 主线轮询 G_DMA_TC 达 4 后写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_DMA_TC  DMA 完成中断执行次数（期望 4）
//   0x20000004 G_DMA_OK  各流 DST==SRC 校验位图（bit0-3，期望 0xF）
//   0x20000008 G_DONE    主线完成标记（期望 0xAAAAAAAA）
//   0x20000100 SRC0      流0 源数组（测试预置 16 字）
//   0x20000140 SRC1      流1 源数组
//   0x20000180 SRC2      流2 源数组
//   0x200001C0 SRC3      流3 源数组
//   0x20000200 DST0      流0 目标数组（DMA 搬运，仿真器直接校验）
//   0x20000240 DST1      流1 目标数组
//   0x20000280 DST2      流2 目标数组
//   0x200002C0 DST3      流3 目标数组

#include <stdint.h>

/* DMA2 @ 0x40026400：LIFCR@0x08（TCIFs 偏移 s%4*6+5，写 1 清除）；
   流 s 寄存器基址 0x40026410 + s*0x18 */
#define DMA2_LIFCR (*(volatile uint32_t *)0x40026408u)
#define DMA2_STREAM(s) ((volatile uint32_t *)(0x40026410u + (s) * 0x18u))
#define DMA2_SsCR(s)  DMA2_STREAM(s)[0]
#define DMA2_SsNDTR(s) DMA2_STREAM(s)[1]
#define DMA2_SsPAR(s) DMA2_STREAM(s)[2]
#define DMA2_SsM0AR(s) DMA2_STREAM(s)[3]

/* NVIC（SCB 基址 0xE000E000）：IRQ56-59 = DMA2_Stream0-3
   ISER1 @ 0xE000E104 bit24-27（56-59-32）；IPR14 @ 0xE000E438 字节0-3（56%4=0） */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u)
#define NVIC_IPR14 (*(volatile uint32_t *)0xE000E438u)

/* DMA2 Stream CR 位（F407，与 DMA1 相同） */
#define DMA_CR_EN    (1u << 0)
#define DMA_CR_TCIE  (1u << 5)
#define DMA_CR_DIR_MM (2u << 6)   /* 内存到内存 */
#define DMA_CR_PINC  (1u << 9)
#define DMA_CR_MINC  (1u << 10)
#define DMA_CR_PSIZE_W (2u << 11) /* 字 */
#define DMA_CR_MSIZE_W (2u << 13) /* 字 */

/* 数据区（固定 SRAM 地址）：流 s 的 SRC/DST */
#define SRC(s) ((volatile const uint32_t *)(0x20000100u + (s) * 0x40u))
#define DST(s) ((volatile uint32_t *)(0x20000200u + (s) * 0x40u))
#define NUM_WORDS 16

/* 结果区 */
#define G_DMA_TC (*(volatile uint32_t *)0x20000000u)
#define G_DMA_OK (*(volatile uint32_t *)0x20000004u)
#define G_DONE   (*(volatile uint32_t *)0x20000008u)

extern void Reset_Handler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);

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

/* 逐字校验 DSTn == SRCn */
static uint32_t verify_stream(uint32_t s) {
    for (uint32_t i = 0u; i < NUM_WORDS; i++) {
        if (DST(s)[i] != SRC(s)[i]) {
            return 0u;
        }
    }
    return 1u;
}

/* 向量表：系统异常 + IRQ0..IRQ70（DMA2 Stream0-3 = IRQ56-59 → index 72-75，Stream5-7 → index 84-86） */
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
    (uint32_t)DMA2_Stream1_IRQHandler, /* 73: IRQ57 = DMA2_Stream1 */
    (uint32_t)DMA2_Stream2_IRQHandler, /* 74: IRQ58 = DMA2_Stream2 */
    (uint32_t)DMA2_Stream3_IRQHandler, /* 75: IRQ59 = DMA2_Stream3 */
    (uint32_t)Default_Handler, /* 76: IRQ60 = DMA2_Stream4 */
    (uint32_t)Default_Handler, /* 77 */
    (uint32_t)Default_Handler, /* 78 */
    (uint32_t)Default_Handler, /* 79 */
    (uint32_t)Default_Handler, /* 80 */
    (uint32_t)Default_Handler, /* 81 */
    (uint32_t)Default_Handler, /* 82 */
    (uint32_t)Default_Handler, /* 83 */
    (uint32_t)Default_Handler, /* 84: IRQ68 = DMA2_Stream5 */
    (uint32_t)Default_Handler, /* 85: IRQ69 = DMA2_Stream6 */
    (uint32_t)Default_Handler, /* 86: IRQ70 = DMA2_Stream7 */
};

void DMA2_Stream0_IRQHandler(void) {
    if (verify_stream(0u)) {
        G_DMA_OK |= (1u << 0);
    }
    DMA2_LIFCR = (1u << 5); /* 写 1 清除 TCIF0 */
    G_DMA_TC++;
}

void DMA2_Stream1_IRQHandler(void) {
    if (verify_stream(1u)) {
        G_DMA_OK |= (1u << 1);
    }
    DMA2_LIFCR = (1u << 11); /* 写 1 清除 TCIF1 */
    G_DMA_TC++;
}

void DMA2_Stream2_IRQHandler(void) {
    if (verify_stream(2u)) {
        G_DMA_OK |= (1u << 2);
    }
    DMA2_LIFCR = (1u << 17); /* 写 1 清除 TCIF2 */
    G_DMA_TC++;
}

void DMA2_Stream3_IRQHandler(void) {
    if (verify_stream(3u)) {
        G_DMA_OK |= (1u << 3);
    }
    DMA2_LIFCR = (1u << 23); /* 写 1 清除 TCIF3 */
    G_DMA_TC++;
}

void Reset_Handler(void) {
    /* 1. IRQ56-59（DMA2_Stream0-3）优先级 + 使能；IPR14 字节0-3 全 15 */
    NVIC_IPR14 = 0xFFFFFFFFu;
    NVIC_ISER1 = (1u << 24) | (1u << 25) | (1u << 26) | (1u << 27);
    __asm volatile("cpsie i" ::: "memory");

    /* 2. 配置 DMA2 Stream0-3：MEM2MEM 各 16 字，PAR=SRCn → M0AR=DSTn，地址递增 */
    for (uint32_t s = 0u; s < 4u; s++) {
        DMA2_SsPAR(s) = (uint32_t)SRC(s);
        DMA2_SsM0AR(s) = (uint32_t)DST(s);
        DMA2_SsNDTR(s) = NUM_WORDS;
        DMA2_SsCR(s) = DMA_CR_EN | DMA_CR_TCIE | DMA_CR_DIR_MM |
                       DMA_CR_PINC | DMA_CR_MINC | DMA_CR_PSIZE_W | DMA_CR_MSIZE_W;
    }

    dma_ready_hook();

    /* 3. 主循环：等待四条流完成中断（handler 各递增 G_DMA_TC） */
    while (G_DMA_TC != 4u) {}
    dma_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
