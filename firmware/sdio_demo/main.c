// M14 demo 固件：SDIO 安全数字 IO（STM32F407）验收。
//
// 场景（SDIO @ 0x40012C00，APB2；虚拟 SD 卡 1MB/512B 块）：
//   Phase A（卡初始化 + 命令路径）：CMD0（无响应→CMDSENT）→ CMD8（R7 回显
//         0x1AA）→ CMD55（R1+APP_CMD 位）→ ACMD41（R3 OCR 0x40FF8000）→
//         CMD3（R6 分配 RCA）→ CMD7（R1 进入 TRAN 态）→ G_INIT_OK=1；
//   Phase B（单块写 CMD24 + DMA2_Stream6 TX）：内存缓冲 128 字 → DMA
//         内存→外设搬运到卡块 0 → DATAEND + TCIF6 → G_WRITE_OK=1；
//   Phase C（单块读 CMD17 + DMA2_Stream3 RX）：卡块 0 → DMA 外设→内存搬回
//         校验与写缓冲一致 → G_READ_OK=1；
//   主线写 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_INIT_OK  卡初始化命令路径校验（期望 1）
//   0x20000004 G_WRITE_OK 单块写 DMA 搬运校验（期望 1）
//   0x20000008 G_READ_OK  单块读回校验（期望 1）
//   0x2000000C G_DONE     主线完成标记（期望 0xAAAAAAAA）
//   0x20000040 SD_BUF     写源缓冲（128 字）
//   0x20000240 RD_BUF     读回缓冲（128 字）

#include <stdint.h>

/* SDIO @ 0x40012C00 */
#define SDIO_ARG     (*(volatile uint32_t *)0x40012C08u)
#define SDIO_CMD     (*(volatile uint32_t *)0x40012C0Cu)
#define SDIO_RESPCMD (*(volatile uint32_t *)0x40012C10u)
#define SDIO_RESP1   (*(volatile uint32_t *)0x40012C14u)
#define SDIO_DLEN    (*(volatile uint32_t *)0x40012C28u)
#define SDIO_DCTRL   (*(volatile uint32_t *)0x40012C2Cu)
#define SDIO_STATUS  (*(volatile uint32_t *)0x40012C38u)
#define SDIO_ICR     (*(volatile uint32_t *)0x40012C3Cu)
#define SDIO_FIFO    (*(volatile uint32_t *)0x40012C80u)

/* SDIO 位 */
#define SDIO_CMD_CPSMEN (1u << 10)     /* 命令路径状态机使能 */
#define SDIO_CMD_WRESP_0 (0u << 6)     /* 无响应 */
#define SDIO_CMD_WRESP_1 (1u << 6)     /* 短响应 R1/R3/R6/R7 */
#define SDIO_DCTRL_DTEN  (1u << 0)     /* 数据路径使能 */
#define SDIO_DCTRL_DTDIR (1u << 1)     /* 1=读（卡→内存），0=写（内存→卡） */
#define SDIO_DCTRL_DMAEN (1u << 3)     /* DMA 使能 */
#define SDIO_STATUS_CMDREND (1u << 6)
#define SDIO_STATUS_CMDSENT (1u << 7)
#define SDIO_STATUS_DATAEND (1u << 8)
#define SDIO_ICR_ALL (0x3FFu)          /* bit0-9 写 1 清 */

/* DMA2（@0x40026400）Stream3/6 */
#define DMA2_LISR   (*(volatile uint32_t *)0x40026400u)
#define DMA2_HISR   (*(volatile uint32_t *)0x40026404u)
#define DMA2_S3CR   (*(volatile uint32_t *)0x40026458u)
#define DMA2_S3NDTR (*(volatile uint32_t *)0x4002645Cu)
#define DMA2_S3PAR  (*(volatile uint32_t *)0x40026460u)
#define DMA2_S3M0AR (*(volatile uint32_t *)0x40026464u)
#define DMA2_S6CR   (*(volatile uint32_t *)0x400264A0u)
#define DMA2_S6NDTR (*(volatile uint32_t *)0x400264A4u)
#define DMA2_S6PAR  (*(volatile uint32_t *)0x400264A8u)
#define DMA2_S6M0AR (*(volatile uint32_t *)0x400264ACu)

/* DMA2 流配置：CHSEL=4（SDIO）、PSIZE=MSIZE=字、MINC；DIR 区分读/写 */
#define DMA_CFG_RX (1u << 0 | (1u << 10) | (2u << 11) | (2u << 13) | (4u << 25)) /* DIR=00 外设→内存 */
#define DMA_CFG_TX (1u << 0 | (1u << 6) | (1u << 10) | (2u << 11) | (2u << 13) | (4u << 25)) /* DIR=01 内存→外设 */
#define DMA2_LISR_TCIF3 (1u << 23)     /* Stream3 传输完成（LISR） */
#define DMA2_HISR_TCIF6 (1u << 17)     /* Stream6 传输完成（HISR） */

/* 结果区 */
#define G_INIT_OK  (*(volatile uint32_t *)0x20000000u)
#define G_WRITE_OK (*(volatile uint32_t *)0x20000004u)
#define G_READ_OK  (*(volatile uint32_t *)0x20000008u)
#define G_DONE     (*(volatile uint32_t *)0x2000000Cu)
#define SD_BUF     ((volatile uint32_t *)0x20000040u)
#define RD_BUF     ((volatile uint32_t *)0x20000240u)

/* 期望 OCR（ACMD41 返回：bit30=CCS + 2.7-3.6V 电压窗） */
#define OCR_EXPECT 0x40FF8000u

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 向量表：系统异常 + IRQ0..IRQ1 占位（本固件轮询，不用 SDIO IRQ49） */
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

/* 发命令：写 ARG + 清状态 + 写 CMD（CPSMEN 启动状态机） */
static void send_cmd(uint32_t index, uint32_t arg, uint32_t waitresp) {
    SDIO_ARG = arg;
    SDIO_ICR = SDIO_ICR_ALL;
    SDIO_CMD = index | waitresp | SDIO_CMD_CPSMEN;
}

static void wait_cmdsent(void) {
    while ((SDIO_STATUS & SDIO_STATUS_CMDSENT) == 0u) {}
}

static void wait_cmdrend(void) {
    while ((SDIO_STATUS & SDIO_STATUS_CMDREND) == 0u) {}
}

void Reset_Handler(void) {
    uint32_t i;
    uint32_t ok;
    uint32_t rca;

    /* ---- Phase A：SD 卡初始化（命令路径） ---- */
    ok = 1u;

    send_cmd(0, 0u, SDIO_CMD_WRESP_0);   /* CMD0：复位 */
    wait_cmdsent();

    send_cmd(8, 0x1AAu, SDIO_CMD_WRESP_1); /* CMD8：电压/校验回显 */
    wait_cmdrend();
    ok &= (SDIO_RESP1 == 0x1AAu);

    send_cmd(55, 0u, SDIO_CMD_WRESP_1);  /* CMD55：进入 APP_CMD */
    wait_cmdrend();
    ok &= ((SDIO_RESP1 & 0x80000000u) != 0u); /* APP_CMD 位 */

    send_cmd(41, 0u, SDIO_CMD_WRESP_1);  /* ACMD41：OCR */
    wait_cmdrend();
    ok &= (SDIO_RESP1 == OCR_EXPECT);

    send_cmd(3, 0u, SDIO_CMD_WRESP_1);   /* CMD3：分配 RCA */
    wait_cmdrend();
    rca = SDIO_RESP1 >> 16;
    ok &= (rca != 0u);

    send_cmd(7, rca << 16, SDIO_CMD_WRESP_1); /* CMD7：select → TRAN */
    wait_cmdrend();
    ok &= (((SDIO_RESP1 >> 9) & 0xFu) == 4u); /* CURRENT_STATE=TRAN */

    if (ok) {
        G_INIT_OK = 1u;
    }

    /* ---- Phase B：单块写（CMD24 + DMA2_Stream6 TX） ---- */
    for (i = 0u; i < 128u; i++) {
        SD_BUF[i] = 0x11110000u + i; /* 128 字写模式 */
    }

    /* 配置 DMA2_Stream6：内存→外设，字，MINC，CHSEL=4 */
    DMA2_S6CR = 0u;
    DMA2_S6PAR = (uint32_t)&SDIO_FIFO;  /* 源：SDIO FIFO（0x40012C80） */
    DMA2_S6M0AR = (uint32_t)SD_BUF;     /* 目标：0x20000040 */
    DMA2_S6NDTR = 128u;                 /* 128 个 32 位字 */
    DMA2_S6CR = DMA_CFG_TX;             /* 使能传输 */

    send_cmd(24, 0u, SDIO_CMD_WRESP_1); /* CMD24：写块 0 */
    wait_cmdrend();

    SDIO_DLEN = 512u;
    SDIO_DCTRL = SDIO_DCTRL_DTEN | SDIO_DCTRL_DMAEN; /* 写方向 + DMA */

    while ((DMA2_HISR & DMA2_HISR_TCIF6) == 0u) {}   /* 等待 DMA 完成 */
    while ((SDIO_STATUS & SDIO_STATUS_DATAEND) == 0u) {} /* 等待数据路径结束 */
    G_WRITE_OK = 1u;

    /* ---- Phase C：单块读回（CMD17 + DMA2_Stream3 RX） ---- */
    for (i = 0u; i < 128u; i++) {
        RD_BUF[i] = 0u;
    }

    /* 配置 DMA2_Stream3：外设→内存，字，MINC，CHSEL=4 */
    DMA2_S3CR = 0u;
    DMA2_S3PAR = (uint32_t)&SDIO_FIFO;  /* 源：SDIO FIFO */
    DMA2_S3M0AR = (uint32_t)RD_BUF;     /* 目标：0x20000240 */
    DMA2_S3NDTR = 128u;
    DMA2_S3CR = DMA_CFG_RX;             /* 使能传输 */

    SDIO_DLEN = 512u;
    SDIO_DCTRL = SDIO_DCTRL_DTEN | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DMAEN; /* 读方向 + DMA */

    send_cmd(17, 0u, SDIO_CMD_WRESP_1); /* CMD17：读块 0 */
    wait_cmdrend();

    while ((DMA2_LISR & DMA2_LISR_TCIF3) == 0u) {}   /* 等待 DMA 完成 */
    while ((SDIO_STATUS & SDIO_STATUS_DATAEND) == 0u) {} /* 等待数据路径结束 */

    /* 校验读回 == 写缓冲 */
    ok = 1u;
    for (i = 0u; i < 128u; i++) {
        if (RD_BUF[i] != SD_BUF[i]) {
            ok = 0u;
        }
    }
    if (ok) {
        G_READ_OK = 1u;
    }

    G_DONE = 0xAAAAAAAAu;
    for (;;) {}
}
