// M6 demo 固件：多定时器 + TIM1 高级定时器（PWM 互补输出/死区/刹车）端到端验证。
//
// 场景（TIM1 高级 + TIM3 通用，同跑展示多定时器；验证通用 Timer 覆盖 TIM1-8）：
//   A. 多定时器并跑：TIM1（高级 16 位）与 TIM3（通用 16 位）同时计数，
//      各自更新中断（IRQ25 / IRQ29）递增 G_T1_UEV / G_T3_UEV。
//   B. TIM1 高级特性：
//      1) PWM 模式1（OC1M=110）：ARR=999、CCR1=250 → OC1REF 25% 占空比
//         （CNT<CCR1 输出高），经 Event::TimPwm 发布供测试订阅校验波形；
//      2) 互补输出 + 死区：CCER.CC1E|CC1NE、BDTR.DTG=20（死区时长 20×tCK）
//         + BDTR.MOE=1（主输出使能）；
//      3) 刹车：测试写 G_TRIGGER_BREAK=1 → 软件刹车 EGR.BG → BDTR.MOE 清零、
//         SR.BIF 置位、TIM1_BRK 中断（IRQ24）执行 → G_T1_BRK++。
// 1. RCC 使能 TIM1（APB2ENR bit0）+ TIM3（APB1ENR bit1）；
// 2. 配置 TIM1：PWM 模式1 + 互补输出 + 死区（DTG=20）+ MOE + 更新中断 + CEN；
// 3. 配置 TIM3：更新中断 + CEN；
// 4. NVIC 使能 IRQ24/25/29（TIM1_BRK / TIM1_UP / TIM3）；cpsie i；
// 5. 主线：PWM 运行中等待 TIM1/TIM3 各累计 ≥4 次更新中断（供测试采集波形/计数），
//    随后关闭更新中断（PWM 波形经 TimPwm 事件继续输出）——中断风暴停止后
//    仿真 run() 才能自然到达指令上限返回（与 M2/M5 固件语义一致）；
// 6. 等待测试写 G_TRIGGER_BREAK=1 → 软件刹车 EGR.BG → 记录 BDTR.MOE（=0）与
//    SR.BIF（=1）→ 写完成标记 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_T3_UEV  TIM3 更新中断次数（期望 >0，多定时器）
//   0x20000004 G_T1_UEV  TIM1 更新中断次数（期望 >0）
//   0x20000008 G_T1_BRK  TIM1 刹车中断次数（期望 1）
//   0x2000000C G_MOE     刹车后 BDTR.MOE（期望 0）
//   0x20000010 G_BIF     刹车后 SR.BIF（期望 1）
//   0x20000014 G_DONE    主线完成标记（期望 0xAAAAAAAA）
//   0x20000018 G_TRIGGER_BREAK  测试写 1 触发软件刹车

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840u) /* bit1 = TIM3 时钟 */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844u) /* bit0 = TIM1 时钟 */

/* TIM1（高级）@ 0x40010000 */
#define TIM1_CR1   (*(volatile uint32_t *)0x40010000u)
#define TIM1_DIER  (*(volatile uint32_t *)0x4001000Cu)
#define TIM1_SR    (*(volatile uint32_t *)0x40010010u)
#define TIM1_EGR   (*(volatile uint32_t *)0x40010014u)
#define TIM1_CCMR1 (*(volatile uint32_t *)0x40010018u)
#define TIM1_CCER  (*(volatile uint32_t *)0x40010020u)
#define TIM1_PSC   (*(volatile uint32_t *)0x40010028u)
#define TIM1_ARR   (*(volatile uint32_t *)0x4001002Cu)
#define TIM1_CCR1  (*(volatile uint32_t *)0x40010034u)
#define TIM1_BDTR  (*(volatile uint32_t *)0x40010044u)

/* TIM3（通用）@ 0x40000400 */
#define TIM3_CR1   (*(volatile uint32_t *)0x40000400u)
#define TIM3_DIER  (*(volatile uint32_t *)0x4000040Cu)
#define TIM3_SR    (*(volatile uint32_t *)0x40000410u)
#define TIM3_EGR   (*(volatile uint32_t *)0x40000414u)
#define TIM3_PSC   (*(volatile uint32_t *)0x40000428u)
#define TIM3_ARR   (*(volatile uint32_t *)0x4000042Cu)

/* NVIC（SCB 基址 0xE000E000）：
   IRQ24/25/29 → ISER0 bit24/25/29；IPR6（IRQ24-27）@0xE000E418、IPR7（IRQ28-31）@0xE000E41C */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_IPR6  (*(volatile uint32_t *)0xE000E418u)
#define NVIC_IPR7  (*(volatile uint32_t *)0xE000E41Cu)

/* TIM 位定义 */
#define TIM_CR1_CEN        (1u << 0)
#define TIM_DIER_UIE       (1u << 0)
#define TIM_SR_UIF         (1u << 0)
#define TIM_SR_BIF         (1u << 7)
#define TIM_EGR_UG         (1u << 0)
#define TIM_EGR_BG         (1u << 7)
#define TIM_CCMR_OC1M_PWM1 (6u << 4)   /* OC1M=110：PWM 模式1（CNT<CCR1 输出高） */
#define TIM_CCER_CC1E      (1u << 0)   /* OC1 主输出使能 */
#define TIM_CCER_CC1NE     (1u << 2)   /* OC1N 互补输出使能（高级） */
#define TIM_BDTR_DTG20     (20u)       /* DTG=20：死区时长 20×tCK */
#define TIM_BDTR_MOE       (1u << 15)  /* 主输出使能 */

/* 结果区（固定 SRAM 地址） */
#define G_T3_UEV         (*(volatile uint32_t *)0x20000000u)
#define G_T1_UEV         (*(volatile uint32_t *)0x20000004u)
#define G_T1_BRK         (*(volatile uint32_t *)0x20000008u)
#define G_MOE            (*(volatile uint32_t *)0x2000000Cu)
#define G_BIF            (*(volatile uint32_t *)0x20000010u)
#define G_DONE           (*(volatile uint32_t *)0x20000014u)
#define G_TRIGGER_BREAK  (*(volatile uint32_t *)0x20000018u)

extern void Reset_Handler(void);
void TIM1_BRK_IRQHandler(void);
void TIM1_UP_IRQHandler(void);
void TIM3_IRQHandler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void pwm_ready_hook(void) __attribute__((noinline));
static void pwm_ready_hook(void) {
    __asm volatile("nop" ::: "memory");
}

static void break_done_hook(void) __attribute__((noinline));
static void break_done_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + IRQ0..29
   （IRQ24=TIM1_BRK → 40、IRQ25=TIM1_UP → 41、IRQ29=TIM3 → 45） */
__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    /* 0..15：系统异常区 */
    0x20001000u,               /* 0: 初始 SP（SRAM 内） */
    (uint32_t)Reset_Handler,   /* 1: Reset */
    (uint32_t)Default_Handler, /* 2: NMI */
    (uint32_t)Default_Handler, /* 3: HardFault */
    (uint32_t)Default_Handler, /* 4: MemManage */
    (uint32_t)Default_Handler, /* 5: BusFault */
    (uint32_t)Default_Handler, /* 6: UsageFault */
    (uint32_t)Default_Handler, /* 7: 保留 */
    (uint32_t)Default_Handler, /* 8 */
    (uint32_t)Default_Handler, /* 9 */
    (uint32_t)Default_Handler, /* 10 */
    (uint32_t)Default_Handler, /* 11 */
    (uint32_t)Default_Handler, /* 12 */
    (uint32_t)Default_Handler, /* 13 */
    (uint32_t)Default_Handler, /* 14 */
    (uint32_t)Default_Handler, /* 15 */
    /* 16..39：IRQ0..23 */
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
    (uint32_t)Default_Handler, /* 39: IRQ23 */
    (uint32_t)TIM1_BRK_IRQHandler, /* 40: IRQ24 = TIM1_BRK */
    (uint32_t)TIM1_UP_IRQHandler,  /* 41: IRQ25 = TIM1_UP */
    (uint32_t)Default_Handler,     /* 42: IRQ26 */
    (uint32_t)Default_Handler,     /* 43: IRQ27 */
    (uint32_t)Default_Handler,     /* 44: IRQ28 */
    (uint32_t)TIM3_IRQHandler,     /* 45: IRQ29 = TIM3 */
};

void TIM1_BRK_IRQHandler(void) {
    /* 刹车中断：计数（BIF 留待主线读取验证，不在此清） */
    G_T1_BRK++;
}

void TIM1_UP_IRQHandler(void) {
    G_T1_UEV++;
    TIM1_SR &= ~TIM_SR_UIF; /* 写 0 清除更新标志 */
}

void TIM3_IRQHandler(void) {
    G_T3_UEV++;
    TIM3_SR &= ~TIM_SR_UIF;
}

void Reset_Handler(void) {
    /* 1. RCC 时钟（镜像） */
    RCC_APB2ENR = (1u << 0);  /* TIM1 */
    RCC_APB1ENR = (1u << 1);  /* TIM3 */

    /* 2. TIM1 高级定时器：PWM 模式1 + 互补输出 + 死区 + 更新中断 */
    TIM1_PSC = 0u;
    TIM1_ARR = 999u;
    TIM1_CCR1 = 250u;                        /* 25% 占空比 */
    TIM1_CCMR1 = TIM_CCMR_OC1M_PWM1;         /* OC1M=110：PWM 模式1 */
    TIM1_CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE; /* OC1 + OC1N 互补输出 */
    TIM1_BDTR = TIM_BDTR_DTG20 | TIM_BDTR_MOE;  /* DTG=20 死区 + MOE=1 主输出 */
    TIM1_DIER = TIM_DIER_UIE;                /* 更新中断使能 */
    TIM1_EGR = TIM_EGR_UG;                   /* UG：软件更新（预装载） */
    TIM1_CR1 = TIM_CR1_CEN;                  /* 启动计数 */

    /* 3. TIM3 通用定时器：更新中断 */
    TIM3_PSC = 0u;
    TIM3_ARR = 999u;
    TIM3_DIER = TIM_DIER_UIE;
    TIM3_EGR = TIM_EGR_UG;
    TIM3_CR1 = TIM_CR1_CEN;

    /* 4. NVIC：IRQ24/25/29 优先级 + 使能 */
    NVIC_IPR6 = 0xF0F0F0F0u; /* IRQ24-27 优先级 15 */
    NVIC_IPR7 = 0xF0F0F0F0u; /* IRQ28-31 优先级 15 */
    NVIC_ISER0 = (1u << 24) | (1u << 25) | (1u << 29);
    __asm volatile("cpsie i" ::: "memory");

    pwm_ready_hook();

    /* 5. PWM 运行：等待 TIM1/TIM3 各累计 ≥4 次更新中断（测试采集波形/计数），
          随后关闭更新中断（PWM 波形经 TimPwm 事件继续输出）——中断风暴停止
          后仿真 run() 才能自然到达指令上限返回。 */
    while (G_T1_UEV < 4u || G_T3_UEV < 4u) {}
    TIM1_DIER &= ~TIM_DIER_UIE;
    TIM3_DIER &= ~TIM_DIER_UIE;

    /* 6. 等待测试写 G_TRIGGER_BREAK=1 → 软件刹车 EGR.BG → MOE 清零 + BIF 置位 + IRQ24 */
    while (G_TRIGGER_BREAK == 0u) {}

    /* 7. 软件刹车：EGR.BG → MOE 清零 + BIF 置位 + IRQ24 刹车中断 */
    TIM1_EGR = TIM_EGR_BG;

    break_done_hook();

    /* 8. 记录刹车结果 */
    G_MOE = (TIM1_BDTR >> 15) & 1u;
    G_BIF = (TIM1_SR >> 7) & 1u;
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
