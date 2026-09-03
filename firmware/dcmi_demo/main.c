// M12 demo 固件：DCMI 数字摄像头接口（STM32F407）验收。
//
// 场景（DCMI @ 0x50050000，AHB2；帧数据由仿真器经 DcmiFrame 事件注入）：
//   Phase A（轮询读 DR）：CR.ENABLE+CAPTURE 使能捕获 → 写 G_READY_A=1 →
//        等待 SR.FRAME 置位（测试注入帧 A：字节 01..10，4 字）→ 读 DR 4 次 →
//        与期望字比较一致 → G_POLL_OK=1；
//   Phase B（DMA 搬运）：配置 DMA2_Stream1_Channel1（外设→内存，32 位字，
//        MINC，NDTR=4，PAR=DCMI_DR，M0AR=0x20000040）→ 写 G_READY_B=1 →
//        等待 DMA2 LISR.S1TC（bit11）（测试注入帧 B：字节 11..20，4 字）→
//        校验缓冲区 4 字与期望一致 → G_DMA_OK=1；
//   主线写 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_POLL_OK  轮询读 DR 校验通过（期望 1）
//   0x20000004 G_READY_A  Phase A 就绪（等待注入帧 A）
//   0x20000008 G_DMA_OK   DMA 搬运校验通过（期望 1）
//   0x2000000C G_READY_B  Phase B 就绪（等待注入帧 B）
//   0x20000010 G_DONE     主线完成标记（期望 0xAAAAAAAA）
//   0x20000040 DCMI_BUF   DMA 图像缓冲区（4 字）

#include <stdint.h>

/* DCMI @ 0x50050000 */
#define DCMI_CR   (*(volatile uint32_t *)0x50050000u)
#define DCMI_SR   (*(volatile uint32_t *)0x50050004u)
#define DCMI_RIS  (*(volatile uint32_t *)0x50050008u)
#define DCMI_IER  (*(volatile uint32_t *)0x5005000Cu)
#define DCMI_DR   (*(volatile uint32_t *)0x50050028u)

/* DCMI 位 */
#define DCMI_CR_CAPTURE (1u << 0)  /* 捕获使能 */
#define DCMI_CR_ENABLE  (1u << 14) /* DCMI 使能 */
#define DCMI_SR_FRAME   (1u << 7)  /* 帧完成 */

/* DMA2（@0x40026400）Stream1 */
#define DMA2_LISR  (*(volatile uint32_t *)0x40026400u)
#define DMA2_S1CR  (*(volatile uint32_t *)0x40026428u)
#define DMA2_S1NDTR (*(volatile uint32_t *)0x4002642Cu)
#define DMA2_S1PAR (*(volatile uint32_t *)0x40026430u)
#define DMA2_S1M0AR (*(volatile uint32_t *)0x40026434u)

/* DMA2 Stream1 CR：EN + MINC + PSIZE=字 + MSIZE=字 + DIR=外设→内存 + CHSEL=1 */
#define DMA2_S1CR_CFG (1u << 0 | (1u << 10) | (2u << 11) | (2u << 13) | (1u << 25))
#define DMA2_LISR_TCIF1 (1u << 11) /* Stream1 传输完成标志 */

/* 结果区 */
#define G_POLL_OK (*(volatile uint32_t *)0x20000000u)
#define G_READY_A (*(volatile uint32_t *)0x20000004u)
#define G_DMA_OK  (*(volatile uint32_t *)0x20000008u)
#define G_READY_B (*(volatile uint32_t *)0x2000000Cu)
#define G_DONE    (*(volatile uint32_t *)0x20000010u)
#define DCMI_BUF  ((volatile uint32_t *)0x20000040u)

/* 期望图像字（低位在前；测试按帧字节 01..10 / 11..20 注入） */
#define W_A0 0x04030201u
#define W_A1 0x08070605u
#define W_A2 0x0C0B0A09u
#define W_A3 0x100F0E0Du
#define W_B0 0x14131211u
#define W_B1 0x18171615u
#define W_B2 0x1C1B1A19u
#define W_B3 0x201F1E1Du

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 向量表：系统异常 + IRQ0..IRQ1 占位（本固件轮询，不用 DCMI IRQ78） */
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
    (uint32_t)Default_Handler, /* 15 */
    (uint32_t)Default_Handler, /* 16: IRQ0 */
    (uint32_t)Default_Handler, /* 17: IRQ1 */
};

static uint32_t check_poll(void) {
    return (DCMI_DR == W_A0) && (DCMI_DR == W_A1) &&
           (DCMI_DR == W_A2) && (DCMI_DR == W_A3);
}

static uint32_t check_dma(void) {
    return (DCMI_BUF[0] == W_B0) && (DCMI_BUF[1] == W_B1) &&
           (DCMI_BUF[2] == W_B2) && (DCMI_BUF[3] == W_B3);
}

void Reset_Handler(void) {
    /* Phase A：使能 + 捕获（连续模式）→ 就绪 → 等待帧注入 → 轮询读 DR 校验 */
    DCMI_CR = DCMI_CR_ENABLE | DCMI_CR_CAPTURE;
    G_READY_A = 1u;
    while ((DCMI_SR & DCMI_SR_FRAME) == 0u) {} /* 等待测试注入帧 A */
    if (check_poll()) {
        G_POLL_OK = 1u;
    }

    /* Phase B：DMA2_Stream1_Channel1 外设→内存搬运整帧 */
    DMA2_S1CR = 0u;                    /* 先复位配置 */
    DMA2_S1PAR = (uint32_t)&DCMI_DR;   /* 源：DCMI_DR（0x50050028） */
    DMA2_S1M0AR = (uint32_t)DCMI_BUF;  /* 目标：0x20000040 */
    DMA2_S1NDTR = 4u;                  /* 4 个 32 位字 */
    DMA2_S1CR = DMA2_S1CR_CFG;         /* 使能传输 */
    G_READY_B = 1u;
    while ((DMA2_LISR & DMA2_LISR_TCIF1) == 0u) {} /* 等待测试注入帧 B + DMA 完成 */
    if (check_dma()) {
        G_DMA_OK = 1u;
    }

    G_DONE = 0xAAAAAAAAu;
    for (;;) {}
}
