// M4 demo 固件：验证 IWDG 独立看门狗超时 → 系统复位 → RCC_CSR.IWDGRSTF。
//
// 场景：
// 1. 首次进入 Reset_Handler：解锁 IWDG，PR=0（÷4）、RLR=0x10（重装载 16），
//    写 KR=0xCCCC 启动，之后主循环不再喂狗；
// 2. 仿真器块 tick 递减 IWDG 计数器（16 步 × ÷4），递减到 0 → 复位请求 →
//    block hook 停机 → run() 执行系统复位（置 CSR.IWDGRSTF + 复位看门狗 + 回复位向量）；
// 3. 复位后再次进入 Reset_Handler（G_BOOT==2）：读 RCC_CSR 确认 IWDGRSTF → G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_BOOT       进入 Reset_Handler 的次数（期望 2）
//   0x20000004 G_RESET_FLAG 复位后读到的 RCC_CSR（期望含 IWDGRSTF=bit28）
//   0x20000008 G_DONE       完成标记（期望 0xAAAAAAAA）
//   0x2000000C G_SPIN       首次启动自旋计数（防死循环折叠，保证块 tick 推进）

#include <stdint.h>

/* IWDG @ 0x40003000 */
#define IWDG_KR  (*(volatile uint32_t *)0x40003000u)
#define IWDG_PR  (*(volatile uint32_t *)0x40003004u)
#define IWDG_RLR (*(volatile uint32_t *)0x40003008u)

/* RCC @ 0x40023800：CSR @ 0x74，IWDGRSTF = bit28 */
#define RCC_CSR (*(volatile uint32_t *)0x40023874u)
#define CSR_IWDGRSTF (1u << 28)

/* 键值 */
#define KR_UNLOCK 0x5555u
#define KR_START  0xCCCCu

/* 结果区 */
#define G_BOOT       (*(volatile uint32_t *)0x20000000u)
#define G_RESET_FLAG (*(volatile uint32_t *)0x20000004u)
#define G_DONE       (*(volatile uint32_t *)0x20000008u)
#define G_SPIN       (*(volatile uint32_t *)0x2000000Cu)

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void wdog_started_hook(void) __attribute__((noinline));
static void wdog_started_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + IRQ0..IRQ28（本固件不用中断，全部 Default_Handler） */
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
    (uint32_t)Default_Handler, /* 44: IRQ28 = TIM2 */
};

void Reset_Handler(void) {
    G_BOOT++;

    if (G_BOOT >= 2u) {
        /* 看门狗复位后再次进入：读 RCC_CSR 确认 IWDGRSTF → 完成 */
        G_RESET_FLAG = RCC_CSR;
        G_DONE = 0xAAAAAAAAu;
        for (;;) {}
    }

    /* 首次进入：解锁 → 配 PR=0(÷4)、RLR=0x10 → 启动，之后不喂狗 */
    IWDG_KR = KR_UNLOCK;
    IWDG_PR = 0u;
    IWDG_KR = KR_UNLOCK;
    IWDG_RLR = 0x10u;
    IWDG_KR = KR_START;

    wdog_started_hook();

    /* 自旋不喂狗：volatile 写保证多基本块，块 tick 反复推进递减计数器 */
    while (1) {
        G_SPIN++;
    }
}
