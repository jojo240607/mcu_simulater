// M5 demo 固件：I2C 完整外设仿真（RX 事件中断接收 + 轮询 TxE 发送）。
//
// 流程：
// 1. RCC 使能 I2C1 时钟（APB1ENR bit21，镜像）；
// 2. I2C1 CR1.PE 使能（PE 上升沿置 TxE，仿真语义）；
// 3. 轮询 TxE 发送问候 'I' → 虚拟从机（Event::I2cByte）捕获（G_TX=1）；
// 4. NVIC IRQ31（I2C1_EV）优先级 15 + 使能；CR2.ITEVTEN|ITBUFEN 使能事件中断；
// 5. 外部（测试）逐字节发布 Event::I2cRx → feed_rx：置 RxNE + 挂起 IRQ31；
// 6. I2C1_EV_IRQHandler：RxNE → 读 DR（清 RxNE）→ RX_BUF[G_RX] → G_RX++；
// 7. 主线轮询 G_RX 达 4 后写完成标记 G_DONE。
//
// 注：ITBUFEN 对 TxE/RxNE 同时生效（RM 缓冲中断语义），PE 使能后 TxE 恒置位，
//     故 CR2 写中断使能时必挂起一次 EV（PRIMASK=1 期间仅置 pending）；
//     handler 只处理 RxNE、不写 DR/CR，故不会二次挂起造成中断风暴。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_TX   发送字节数（问候 1，期望 1）
//   0x20000004 G_RX   事件中断接收次数（期望 4）
//   0x20000008 G_DONE 主线完成标记（期望 0xAAAAAAAA）
//   0x20000200 RX_BUF 事件中断接收结果（期望 "abcd"）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840u) /* bit21 = I2C1 时钟 */

/* I2C1 @ 0x40005400 */
#define I2C1_CR1  (*(volatile uint32_t *)0x40005400u)
#define I2C1_CR2  (*(volatile uint32_t *)0x40005404u)
#define I2C1_DR   (*(volatile uint32_t *)0x40005410u)
#define I2C1_SR1  (*(volatile uint32_t *)0x40005414u)

/* NVIC（SCB 基址 0xE000E000）：IRQ31 → ISER0 bit31 + IPR7 字节3 */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_IPR7  (*(volatile uint32_t *)0xE000E41Cu)

/* 位定义 */
#define CR1_PE       (1u << 0)
#define CR2_ITEVTEN  (1u << 9)
#define CR2_ITBUFEN  (1u << 10)
#define SR1_TXE      (1u << 7)
#define SR1_RXNE     (1u << 6)

/* 结果区（固定 SRAM 地址） */
#define G_TX   (*(volatile uint32_t *)0x20000000u)
#define G_RX   (*(volatile uint32_t *)0x20000004u)
#define G_DONE (*(volatile uint32_t *)0x20000008u)
#define RX_BUF ((volatile uint8_t *)0x20000200u)

extern void Reset_Handler(void);
void I2C1_EV_IRQHandler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void i2c_wait_hook(void) __attribute__((noinline));
static void i2c_wait_hook(void) {
    __asm volatile("nop" ::: "memory");
}

static void i2c_done_hook(void) __attribute__((noinline));
static void i2c_done_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + IRQ0..IRQ31（I2C1_EV = IRQ31 → index 47） */
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
    (uint32_t)Default_Handler, /* 31: IRQ15 */
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
    (uint32_t)I2C1_EV_IRQHandler, /* 47: IRQ31 = I2C1_EV */
};

void I2C1_EV_IRQHandler(void) {
    uint32_t sr1 = I2C1_SR1;
    if (sr1 & SR1_RXNE) {
        RX_BUF[G_RX] = (uint8_t)(I2C1_DR & 0xFFu); /* 读 DR 清 RxNE */
        G_RX++;
    }
    /* TxE 侧（ITBUFEN 同时生效）不需要处理：发送用主循环轮询 */
}

void Reset_Handler(void) {
    /* 1. 使能 I2C1 时钟（寄存器镜像） */
    RCC_APB1ENR = (1u << 21);

    /* 2. I2C1：PE 使能（PE 上升沿置 TxE，仿真语义） */
    I2C1_CR1 = CR1_PE;

    /* 3. 轮询 TxE 发送问候 'I' → 虚拟从机（Event::I2cByte）捕获 */
    while ((I2C1_SR1 & SR1_TXE) == 0) {}
    I2C1_DR = 'I';
    G_TX = 1;

    /* 4. NVIC：IRQ31 优先级 15 + 使能 */
    NVIC_IPR7 = 0xFu << 24;
    NVIC_ISER0 = (1u << 31);

    /* 5. I2C1：事件中断使能（写 CR2 时 TxE 置位 → 挂起一次 EV，PRIMASK=1 期间仅 pending） */
    I2C1_CR2 = CR2_ITEVTEN | CR2_ITBUFEN;

    __asm volatile("cpsie i" ::: "memory");
    i2c_wait_hook();

    /* 6. 等待 4 次事件中断接收（handler 逐字节写入 RX_BUF） */
    while (G_RX != 4u) {}
    i2c_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
