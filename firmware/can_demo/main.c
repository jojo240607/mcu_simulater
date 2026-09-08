// M15 demo 固件：CAN1/2 控制器局域网（STM32F407）验收。
//
// 场景（CAN1 @ 0x40006400，CAN2 @ 0x40006800，APB1；CanFrame 总线互联）：
//   Phase A（CAN1 TX → CAN2 RX，标准帧）：CAN1 过滤器 F0=0x123 标准 + F1=
//         0x1FFEDCBA 扩展，CAN2 全收；CAN1 邮箱0 发送标准帧 ID=0x123 DLC=2
//         [0x11,0x22] → 总线发布 → CAN2 FIFO0 命中（FMP=1）→ 轮询读回校验
//         （RI/RDT/RDL/RDH）→ G_TX2RX_OK=1；
//   Phase B（CAN2 TX → CAN1 RX，扩展帧）：CAN2 发送扩展帧 ID=0x1FFEDCBA
//         DLC=8 [55 66 77 88 11 22 33 44] → CAN1 过滤器 F1 命中 FIFO0 → 读回
//         校验 → G_RX2TX_OK=1；
//   Phase C（FIFO 排队 + RFOM 释放）：CAN2 连发两帧 ID=0x123（数据不同）→
//         CAN1 FIFO0 FMP=2 排队 → 读第 1 帧 → RFOM 释放 → FMP=1 → 读第 2 帧
//         → 释放 → FMP=0 → G_FIFO_OK=1；
//   主线写 G_DONE。发送在写 TIRx 时同步完成并发布/路由，故单次 run 即可完成。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_TX2RX_OK Phase A CAN1→CAN2 标准帧互联校验（期望 1）
//   0x20000004 G_RX2TX_OK Phase B CAN2→CAN1 扩展帧互联校验（期望 1）
//   0x20000008 G_FIFO_OK  Phase C FIFO 排队/RFOM 释放校验（期望 1）
//   0x2000000C G_DONE     主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* CAN1/2 基地址（APB1） */
#define CAN1_BASE 0x40006400u
#define CAN2_BASE 0x40006800u

/* 寄存器偏移（bxCAN 寄存器集） */
#define REG_MCR   0x00u
#define REG_MSR   0x04u
#define REG_TSR   0x08u
#define REG_RF0R  0x0Cu
#define REG_RF1R  0x10u
#define REG_IER   0x14u
#define REG_ESR   0x18u
#define REG_BTR   0x1Cu
#define REG_TIR   0x180u /* 发送邮箱0 标识符 */
#define REG_TDTR  0x184u /* 发送邮箱0 数据长度 */
#define REG_TDLR  0x188u /* 发送邮箱0 数据低 32 位 */
#define REG_TDHR  0x18Cu /* 发送邮箱0 数据高 32 位 */
#define REG_RI0R  0x1B0u /* 接收 FIFO0 标识符 */
#define REG_RDT0R 0x1B4u /* 接收 FIFO0 数据长度 */
#define REG_RDL0R 0x1B8u /* 接收 FIFO0 数据低 32 位 */
#define REG_RDH0R 0x1BCu /* 接收 FIFO0 数据高 32 位 */
#define REG_F0R1  0x420u /* 滤波器 0 数据字 1 */
#define REG_F1R1  0x428u /* 滤波器 1 数据字 1 */
#define REG_FA1R  0x41Cu /* 滤波器激活 */

/* 位定义（与 src/peripheral/can.rs 一致） */
#define MCR_INRQ  (1u << 0)
#define MSR_INAK  (1u << 0)
#define TSR_TXOK0 (1u << 1)  /* 邮箱0 发送成功 */
#define TSR_TME0  (1u << 26) /* 邮箱0 空 */
#define RF_FMP    (0x3u)     /* FIFO 中消息数 */
#define RF_RFOM   (1u << 5)  /* 释放输出邮箱 */
#define IER_TMEIE (1u << 0)  /* 邮箱空中断使能 */
#define IER_FMPIE0 (1u << 1) /* FIFO0 非空中断使能 */
#define TI_TXRQ   (1u << 0)
#define TI_RTR    (1u << 1)
#define TI_IDE    (1u << 2)
#define STID(id)  ((id) << 21)   /* 标准帧 ID 在 [31:21] */
#define EXID(id)  ((id) << 3)    /* 扩展帧 ID 在 [31:3] */
#define DLC(n)    ((n) & 0xFu)          /* 数据长度 DLC[3:0]（真机 bxCAN TDTR/RDT0R 均在 bit0-3；原宏 <<16 错位使模拟器解析 dlc=0） */

/* 结果区 */
#define G_TX2RX_OK (*(volatile uint32_t *)0x20000000u)
#define G_RX2TX_OK (*(volatile uint32_t *)0x20000004u)
#define G_FIFO_OK  (*(volatile uint32_t *)0x20000008u)
#define G_DONE     (*(volatile uint32_t *)0x2000000Cu)

static inline uint32_t rd(uint32_t base, uint32_t off) {
    return *(volatile uint32_t *)(base + off);
}
static inline void wr(uint32_t base, uint32_t off, uint32_t v) {
    *(volatile uint32_t *)(base + off) = v;
}

/* 进入初始化模式：置 INRQ 并等待 INAK 确认 */
static void can_enter_init(uint32_t base) {
    wr(base, REG_MCR, rd(base, REG_MCR) | MCR_INRQ);
    while ((rd(base, REG_MSR) & MSR_INAK) == 0u) {}
}
/* 退出初始化模式：清 INRQ 并等待 INAK 清除 */
static void can_leave_init(uint32_t base) {
    wr(base, REG_MCR, rd(base, REG_MCR) & ~MCR_INRQ);
    while ((rd(base, REG_MSR) & MSR_INAK) != 0u) {}
}
/* 触发邮箱0 发送并等待完成 */
static void can_send(uint32_t base, uint32_t tir) {
    wr(base, REG_TIR, tir | TI_TXRQ);
    while ((rd(base, REG_TSR) & TSR_TXOK0) == 0u) {}
}
/* 等待 FIFO0 有 n 条消息 */
static void can_wait_fmp(uint32_t base, uint32_t n) {
    while ((rd(base, REG_RF0R) & RF_FMP) < n) {}
}

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 向量表：系统异常 + IRQ0..IRQ1 占位（本固件轮询，不用 CAN IRQ） */
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
    uint32_t ok;

    /* ---- Phase A：CAN1 TX → CAN2 RX（标准帧） ---- */
    /* CAN1：初始化 + 过滤器 F0=0x123 标准 / F1=0x1FFEDCBA 扩展 + 中断使能 */
    can_enter_init(CAN1_BASE);
    wr(CAN1_BASE, REG_BTR, 0x001C0000u); /* 位时序（简化存储回读） */
    wr(CAN1_BASE, REG_F0R1, STID(0x123u));
    wr(CAN1_BASE, REG_F1R1, TI_IDE | EXID(0x1FFEDCBAu));
    wr(CAN1_BASE, REG_FA1R, 0x3u); /* 激活 F0/F1 */
    can_leave_init(CAN1_BASE);
    wr(CAN1_BASE, REG_IER, IER_TMEIE | IER_FMPIE0);
    /* CAN2：初始化 + 全收（FA1R=0） */
    can_enter_init(CAN2_BASE);
    wr(CAN2_BASE, REG_BTR, 0x001C0000u);
    can_leave_init(CAN2_BASE);
    wr(CAN2_BASE, REG_IER, IER_TMEIE | IER_FMPIE0);

    /* CAN1 发送标准帧 ID=0x123 DLC=2 [0x11,0x22] */
    wr(CAN1_BASE, REG_TDTR, DLC(2u));
    wr(CAN1_BASE, REG_TDLR, 0x2211u);
    can_send(CAN1_BASE, STID(0x123u));

    /* CAN2 读回校验 */
    can_wait_fmp(CAN2_BASE, 1u);
    ok = (rd(CAN2_BASE, REG_RI0R) == STID(0x123u))
      && (rd(CAN2_BASE, REG_RDT0R) == DLC(2u))
      && (rd(CAN2_BASE, REG_RDL0R) == 0x2211u)
      && (rd(CAN2_BASE, REG_RDH0R) == 0u);
    if (ok) {
        G_TX2RX_OK = 1u;
    }
    wr(CAN2_BASE, REG_RF0R, RF_RFOM); /* 释放 */

    /* ---- Phase B：CAN2 TX → CAN1 RX（扩展帧） ---- */
    /* CAN2 发送扩展帧 ID=0x1FFEDCBA DLC=8 [55 66 77 88 11 22 33 44] */
    wr(CAN2_BASE, REG_TDTR, DLC(8u));
    wr(CAN2_BASE, REG_TDLR, 0x88776655u);
    wr(CAN2_BASE, REG_TDHR, 0x44332211u);
    can_send(CAN2_BASE, TI_IDE | EXID(0x1FFEDCBAu));

    /* CAN1 读回校验（过滤器 F1 命中） */
    can_wait_fmp(CAN1_BASE, 1u);
    ok = ((rd(CAN1_BASE, REG_RI0R) & TI_IDE) != 0u)
      && ((rd(CAN1_BASE, REG_RI0R) & 0xFFFFFFF8u) == EXID(0x1FFEDCBAu))
      && (rd(CAN1_BASE, REG_RDT0R) == DLC(8u))
      && (rd(CAN1_BASE, REG_RDL0R) == 0x88776655u)
      && (rd(CAN1_BASE, REG_RDH0R) == 0x44332211u);
    if (ok) {
        G_RX2TX_OK = 1u;
    }
    wr(CAN1_BASE, REG_RF0R, RF_RFOM); /* 释放 */

    /* ---- Phase C：FIFO 排队 + RFOM 释放（多帧） ---- */
    /* CAN2 连发两帧 ID=0x123（数据不同） → CAN1 FIFO0 排队 FMP=2 */
    wr(CAN2_BASE, REG_TDTR, DLC(2u));
    wr(CAN2_BASE, REG_TDLR, 0x4433u);
    can_send(CAN2_BASE, STID(0x123u));
    wr(CAN2_BASE, REG_TDLR, 0x6655u);
    can_send(CAN2_BASE, STID(0x123u));

    can_wait_fmp(CAN1_BASE, 2u); /* 两帧均入 FIFO0 */
    ok = (rd(CAN1_BASE, REG_RDL0R) == 0x4433u); /* 第 1 帧 */
    wr(CAN1_BASE, REG_RF0R, RF_RFOM);           /* 释放 → FMP=1 */
    ok = ok && ((rd(CAN1_BASE, REG_RF0R) & RF_FMP) == 1u);
    ok = ok && (rd(CAN1_BASE, REG_RDL0R) == 0x6655u); /* 第 2 帧 */
    wr(CAN1_BASE, REG_RF0R, RF_RFOM);           /* 释放 → FMP=0 */
    ok = ok && ((rd(CAN1_BASE, REG_RF0R) & RF_FMP) == 0u);
    if (ok) {
        G_FIFO_OK = 1u;
    }

    G_DONE = 0xAAAAAAAAu;
    for (;;) {}
}
