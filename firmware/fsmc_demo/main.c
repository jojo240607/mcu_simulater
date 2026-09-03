// M13 demo 固件：FSMC 外部存储器控制器（STM32F407）验收。
//
// 场景（FSMC 寄存器 @0xA0000000；Bank1-4 片选窗口各 64KB 简化映射）：
//   Phase A（Bank1 窗口读写）：BCR1 = MBKEN|WREN|MTYP=SRAM|MWID=16 → 写 16 字
//        模式到 0x60000000..0x6000003C 并读回校验 → 写/读窗口末尾
//        0x6000FFFC 校验窗口深度 → 读回 BCR1 校验寄存器文件 → G_READ_OK=1；
//   Phase B（片选门控）：未使能 BCR2 时写 0x64000000 读回应为 0（写被丢弃）→
//        G_GATE_OK=1；
//   Phase C（Bank2 使能后）：BCR2 = MBKEN|WREN → 写/读 0x64000000 校验 →
//        G_BANK2_OK=1；
//   主线写 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_READ_OK   Bank1 窗口读写校验通过（期望 1）
//   0x20000004 G_GATE_OK   未使能窗口访问被忽略（期望 1）
//   0x20000008 G_BANK2_OK  Bank2 使能后读写校验通过（期望 1）
//   0x2000000C G_DONE      主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* FSMC 寄存器 @ 0xA0000000 */
#define FSMC_BCR1 (*(volatile uint32_t *)0xA0000000u)
#define FSMC_BTR1 (*(volatile uint32_t *)0xA0000004u)
#define FSMC_BCR2 (*(volatile uint32_t *)0xA0000008u)

/* BCR 位 */
#define BCR_MBKEN    (1u << 0)  /* 片选使能 */
#define BCR_MTYP_SRAM (0u << 2)
#define BCR_MWID_16   (1u << 4)
#define BCR_WREN      (1u << 12) /* 写使能 */

/* 片选窗口（32 位视图） */
#define EXTWIN1 ((volatile uint32_t *)0x60000000u)
#define EXTWIN2 ((volatile uint32_t *)0x64000000u)

/* 结果区 */
#define G_READ_OK  (*(volatile uint32_t *)0x20000000u)
#define G_GATE_OK  (*(volatile uint32_t *)0x20000004u)
#define G_BANK2_OK (*(volatile uint32_t *)0x20000008u)
#define G_DONE     (*(volatile uint32_t *)0x2000000Cu)

#define N_WORDS 16u

/* 写 N_WORDS 模式并读回校验；全部一致返回 1 */
static uint32_t fill_and_check(volatile uint32_t *win) {
    uint32_t i;
    for (i = 0u; i < N_WORDS; i++) {
        win[i] = 0xCAFE0000u + i;
    }
    for (i = 0u; i < N_WORDS; i++) {
        if (win[i] != (0xCAFE0000u + i)) return 0u;
    }
    return 1u;
}

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 向量表：系统异常 + IRQ0..IRQ1 占位（本固件轮询，不用中断） */
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

void Reset_Handler(void) {
    /* Phase A：使能 Bank1 窗口 + 写使能 + SRAM 16 位，配置时序寄存器 */
    FSMC_BCR1 = BCR_MBKEN | BCR_WREN | BCR_MTYP_SRAM | BCR_MWID_16;
    FSMC_BTR1 = 0x0000000Fu; /* 时序配置（无功能仿真，仅可回读） */
    if (fill_and_check(EXTWIN1)) {
        /* 校验窗口末尾（64KB 深度：0x60000000 + 0xFFFC）先写后读 */
        EXTWIN1[0x3FFFu] = 0xCAFE0000u + 0x3FFFu;
        if (EXTWIN1[0x3FFFu] == (0xCAFE0000u + 0x3FFFu)) {
            /* 校验寄存器文件回读（BCR1 含 MBKEN/WREN） */
            if ((FSMC_BCR1 & (BCR_MBKEN | BCR_WREN)) == (BCR_MBKEN | BCR_WREN)) {
                G_READ_OK = 1u;
            }
        }
    }

    /* Phase B：Bank2 未使能 → 写被丢弃、读恒 0 */
    EXTWIN2[0] = 0x12345678u;
    if (EXTWIN2[0] == 0u) {
        G_GATE_OK = 1u;
    }

    /* Phase C：使能 Bank2 → 写/读校验 */
    FSMC_BCR2 = BCR_MBKEN | BCR_WREN | BCR_MTYP_SRAM | BCR_MWID_16;
    if (fill_and_check(EXTWIN2)) {
        G_BANK2_OK = 1u;
    }

    G_DONE = 0xAAAAAAAAu;
    for (;;) {}
}
