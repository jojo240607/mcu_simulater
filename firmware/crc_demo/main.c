// M8 demo 固件：CRC 计算单元（CRC-32/MPEG-2 风格）验收。
//
// 场景（F407 CRC @ 0x40023000，AHB1，始终使能）：
//   Phase A：CR.RESET 写 1 复位计算单元 → 读 DR 应 = 0xFFFFFFFF（初始值）；
//   Phase B：按字节写 DR（8 位访问）喂 "123456789"（9 字节）→ 读 DR 应 = 0x0376E6E7
//            （CRC-32/MPEG-2 标准 check 值，独立锚点）；
//   Phase C：CR.RESET 写 1 → 读 DR 应回 0xFFFFFFFF（计算单元复位，RESET 位自清零）；
//   Phase D：IDR 写 0xAB → 读回 0xAB 且不影响 DR（IDR 为独立数据寄存器，不参与计算）。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_INIT_OK  Phase A 复位后 DR 读回初始值 0xFFFFFFFF（期望 1）
//   0x20000004 G_CRC_OK   Phase B "123456789" CRC == 0x0376E6E7（期望 1）
//   0x20000008 G_RESET_OK Phase C CR.RESET 后 DR 回 0xFFFFFFFF（期望 1）
//   0x2000000C G_IDR_OK   Phase D IDR 写读回保持且 CRC 不变（期望 1）
//   0x20000010 G_DONE     主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* CRC @ 0x40023000（DR 支持 8/16/32 位访问；8 位访问仅低字节参与计算） */
#define CRC_DR  (*(volatile uint32_t *)0x40023000u)
#define CRC_DR8 (*(volatile uint8_t *)0x40023000u)
#define CRC_IDR (*(volatile uint32_t *)0x40023004u)
#define CRC_CR  (*(volatile uint32_t *)0x40023008u)

/* CRC CR 位 */
#define CRC_CR_RESET (1u << 0) /* 写 1 复位计算单元（位自清零） */

/* 结果区 */
#define G_INIT_OK  (*(volatile uint32_t *)0x20000000u)
#define G_CRC_OK   (*(volatile uint32_t *)0x20000004u)
#define G_RESET_OK (*(volatile uint32_t *)0x20000008u)
#define G_IDR_OK   (*(volatile uint32_t *)0x2000000Cu)
#define G_DONE     (*(volatile uint32_t *)0x20000010u)

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 向量表：系统异常 + IRQ0..IRQ1 占位（CRC 无中断） */
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
    /* Phase A：复位计算单元 → DR 读回初始值 0xFFFFFFFF */
    CRC_CR = CRC_CR_RESET;
    if (CRC_DR == 0xFFFFFFFFu) {
        G_INIT_OK = 1u;
    }

    /* Phase B：按字节写 DR 喂 "123456789" → CRC-32/MPEG-2 check 0x0376E6E7 */
    static const uint8_t msg[] = "123456789";
    for (unsigned i = 0; i < sizeof(msg) - 1u; i++) {
        CRC_DR8 = msg[i]; /* 8 位访问：仅低字节参与计算 */
    }
    if (CRC_DR == 0x0376E6E7u) {
        G_CRC_OK = 1u;
    }

    /* Phase C：CR.RESET 写 1 → DR 回 0xFFFFFFFF（计算单元复位） */
    CRC_CR = CRC_CR_RESET;
    if (CRC_DR == 0xFFFFFFFFu) {
        G_RESET_OK = 1u;
    }

    /* Phase D：IDR 写读回保持（独立数据寄存器，不影响 CRC 计算） */
    CRC_IDR = 0xABu;
    if (CRC_IDR == 0xABu && CRC_DR == 0xFFFFFFFFu) {
        G_IDR_OK = 1u;
    }

    G_DONE = 0xAAAAAAAAu;
    for (;;) {}
}
