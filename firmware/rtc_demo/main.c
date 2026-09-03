// M11 demo 固件：RTC 实时时钟 + 备份寄存器（STM32F407）验收。
//
// 场景（RTC @ 0x40002800）：
//   Phase A：RTC 初始化序列——WPR 解锁（0xCA→0x53）→ ISR.INIT=1 等 INITF →
//     写 PRER（LSE 32768→1Hz）/TR/DR → INIT=0 等 RSF → 校验 INITS → G_RTC_INIT_OK=1；
//   Phase B：备份寄存器——未设 PWR_CR.DBP 时写 BKP0R 被忽略（保持 0）→
//     置 DBP 后写 BKP0R/BKP1R 读回一致 → G_BKP_NODBP=1、G_BKP_OK=1；
//   Phase C：闹钟 A（23:59:57）+ 唤醒定时器（ck_spre 1Hz）——使能 ALRAIE/WUTIE，
//     并开 NVIC IRQ3（RTC_WKUP）/IRQ41（RTC_Alarm）；时间推进到匹配点后
//     RTC_Alarm_IRQHandler / RTC_WKUP_IRQHandler 各置计数并清 ISR 标志；
//   主线轮询两 IRQ 计数均 ≥1 → G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_RTC_INIT_OK 初始化序列完成、INITS 置位（期望 1）
//   0x20000004 G_BKP_NODBP   未解锁备份域时 BKP 写被忽略（期望 1）
//   0x20000008 G_BKP_OK      解锁后 BKP 写读一致（期望 1）
//   0x2000000C G_ALARM_IRQ   RTC_Alarm(IRQ41) handler 计数（期望 ≥1）
//   0x20000010 G_WAKEUP_IRQ  RTC_WKUP(IRQ3) handler 计数（期望 ≥1）
//   0x20000014 G_TR          轮询开始时读到的 TR（期望 0x00235955+ 或后续值）
//   0x20000018 G_DONE        主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* RTC @ 0x40002800 */
#define RTC_TR       (*(volatile uint32_t *)0x40002800u)
#define RTC_DR       (*(volatile uint32_t *)0x40002804u)
#define RTC_CR       (*(volatile uint32_t *)0x40002808u)
#define RTC_ISR      (*(volatile uint32_t *)0x4000280Cu)
#define RTC_PRER     (*(volatile uint32_t *)0x40002810u)
#define RTC_WUTR     (*(volatile uint32_t *)0x40002814u)
#define RTC_ALRMAR   (*(volatile uint32_t *)0x4000281Cu)
#define RTC_ALRMBR   (*(volatile uint32_t *)0x40002820u)
#define RTC_WPR      (*(volatile uint32_t *)0x40002824u)
#define RTC_SSR      (*(volatile uint32_t *)0x40002828u)
#define RTC_ALRMASSR (*(volatile uint32_t *)0x40002844u)
#define RTC_ALRMBSSR (*(volatile uint32_t *)0x40002848u)
#define RTC_BKP0R    (*(volatile uint32_t *)0x40002850u)
#define RTC_BKP1R    (*(volatile uint32_t *)0x40002854u)

/* RTC_ISR 位 */
#define ISR_ALRAWF (1u << 0)
#define ISR_INITS  (1u << 4)
#define ISR_RSF    (1u << 5)
#define ISR_INITF  (1u << 6)
#define ISR_INIT   (1u << 7)
#define ISR_ALRAF  (1u << 8)
#define ISR_ALRBF  (1u << 9)
#define ISR_WUTF   (1u << 10)

/* RTC_CR 位 */
#define CR_WUCKSEL_CKSPRE (4u << 0) /* 唤醒时钟 = ck_spre（1Hz） */
#define CR_ALRAE   (1u << 8)
#define CR_ALRBE   (1u << 9)
#define CR_WUTE    (1u << 10)
#define CR_ALRAIE  (1u << 12)
#define CR_ALRBIE  (1u << 13)
#define CR_WUTIE   (1u << 14)

/* ALRMAR：MSK4=1 屏蔽日期，仅按 时:分:秒 匹配 */
#define ALR_MSK4   (1u << 31)

/* PWR @ 0x40007000：CR.DBP = bit8 禁用备份域写保护 */
#define PWR_CR     (*(volatile uint32_t *)0x40007000u)
#define CR_DBP     (1u << 8)

/* NVIC（SCB 基址 0xE000E000） */
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u) /* IRQ0-31 */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u) /* IRQ32-63 */

/* 结果区 */
#define G_RTC_INIT_OK (*(volatile uint32_t *)0x20000000u)
#define G_BKP_NODBP   (*(volatile uint32_t *)0x20000004u)
#define G_BKP_OK      (*(volatile uint32_t *)0x20000008u)
#define G_ALARM_IRQ   (*(volatile uint32_t *)0x2000000Cu)
#define G_WAKEUP_IRQ  (*(volatile uint32_t *)0x20000010u)
#define G_TR          (*(volatile uint32_t *)0x20000014u)
#define G_DONE        (*(volatile uint32_t *)0x20000018u)

extern void Reset_Handler(void);
void RTC_WKUP_IRQHandler(void);
void RTC_Alarm_IRQHandler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 向量表：系统异常 + IRQ0..IRQ41（index 16..57；RTC_WKUP=IRQ3→19，RTC_Alarm=IRQ41→57） */
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
    (uint32_t)Default_Handler, /* 17 */
    (uint32_t)Default_Handler, /* 18 */
    (uint32_t)RTC_WKUP_IRQHandler, /* 19: IRQ3 = RTC_WKUP */
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
    (uint32_t)RTC_Alarm_IRQHandler, /* 57: IRQ41 = RTC_Alarm */
};

/* 清 ISR 标志（写 0 清除；其他标志位写回保持） */
static void rtc_clear_flag(uint32_t bit) {
    RTC_ISR = (RTC_ISR & ~bit);
}

void RTC_Alarm_IRQHandler(void) {
    G_ALARM_IRQ++;
    rtc_clear_flag(ISR_ALRAF);
}

void RTC_WKUP_IRQHandler(void) {
    G_WAKEUP_IRQ++;
    rtc_clear_flag(ISR_WUTF);
}

void Reset_Handler(void) {
    /* Phase A：RTC 初始化（WPR 解锁 → INIT → 写 PRER/TR/DR → 退出初始化） */
    RTC_WPR = 0xCAu;
    RTC_WPR = 0x53u;

    RTC_ISR |= ISR_INIT;                 /* 请求初始化模式 */
    while (!(RTC_ISR & ISR_INITF)) {}    /* 等 INITF（冻结日历） */

    RTC_PRER = 0x007F00FFu;              /* PREDIV_A=127, PREDIV_S=255 → 1Hz */
    RTC_DR = 0x0024C601u;                /* 2024-06-01（星期自动计算） */
    RTC_TR = 0x00235955u;                /* 23:59:55（后写 TR 以保留秒） */

    RTC_ISR &= ~ISR_INIT;                /* 退出初始化模式 */
    while (!(RTC_ISR & ISR_RSF)) {}      /* 等 RSF（寄存器同步） */

    if (RTC_ISR & ISR_INITS) {
        G_RTC_INIT_OK = 1u;
    }

    /* Phase B：备份寄存器（写访问需 PWR_CR.DBP=1） */
    RTC_BKP0R = 0xDEADBEEFu;             /* 未解锁备份域 → 应被忽略 */
    if (RTC_BKP0R == 0u) {
        G_BKP_NODBP = 1u;
    }

    PWR_CR |= CR_DBP;                    /* 禁用备份域写保护 */
    RTC_BKP0R = 0xCAFEBABEu;
    RTC_BKP1R = 0x12345678u;
    if (RTC_BKP0R == 0xCAFEBABEu && RTC_BKP1R == 0x12345678u) {
        G_BKP_OK = 1u;
    }

    /* Phase C：闹钟 A（23:59:57，屏蔽日期）+ 唤醒定时器（1Hz 周期） */
    RTC_ALRMAR = ALR_MSK4 | (0x23u << 16) | (0x59u << 8) | 0x57u;
    RTC_ALRMASSR = 0u;                   /* 无亚秒约束 */

    RTC_CR = CR_ALRAE | CR_ALRAIE;       /* 使能闹钟 A + 中断 */

    RTC_CR |= CR_WUCKSEL_CKSPRE;         /* 唤醒时钟 = ck_spre（1Hz）；WUTE=0 时可写，保持已使能的闹钟位 */
    RTC_WUTR = 0u;                       /* 1 秒周期（WUTR+1） */
    RTC_CR |= CR_WUTE | CR_WUTIE;        /* 使能唤醒定时器 + 中断 */

    NVIC_ISER0 |= (1u << 3);             /* RTC_WKUP = IRQ3 */
    NVIC_ISER1 |= (1u << 9);             /* RTC_Alarm = IRQ41（41-32=9） */

    G_TR = RTC_TR;                       /* 记录当前 TR */

    /* 主线轮询：两 IRQ handler 均触发过 → 完成 */
    while (G_ALARM_IRQ < 1u || G_WAKEUP_IRQ < 1u) {
    }

    /* 停掉 RTC 中断源（唤醒定时器周期触发，不停则仿真 run() 无法自然返回） */
    RTC_CR &= ~(CR_ALRAE | CR_ALRAIE | CR_WUTE | CR_WUTIE);

    G_DONE = 0xAAAAAAAAu;
    for (;;) {}
}
