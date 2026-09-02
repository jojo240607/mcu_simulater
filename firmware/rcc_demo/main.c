// M4 demo 固件：验证 RCC 时钟树（HSE → PLL → 168MHz 系统时钟）。
//
// 流程（与 STM32F407 启动时序一致）：
// 1. 使能 GPIOA 时钟（AHB1ENR）；
// 2. 使能 HSE（CR.HSEON），轮询 CR.HSERDY 就绪；
// 3. 配置 PLLCFGR：PLLSRC=HSE, M=8, N=336, P=2, Q=7
//    → VCO_IN=8M/8=1M, VCO_OUT=336M, PLLCLK=336/2=168MHz, PLL48=336/7=48MHz；
// 4. 使能 PLL（CR.PLLON），轮询 CR.PLLRDY 就绪；
// 5. 配置 CFGR：SW=PLL(2), HPRE=/1, PPRE1=/4(5<<10), PPRE2=/2(4<<13)
//    → SYSCLK=168M, HCLK=168M, PCLK1=42M, PCLK2=84M；
//    轮询 CFGR.SWS==2 确认系统时钟已切到 PLL；
// 6. 写完成标记 G_DONE。
//
// 每次轮询成功后置对应观察位到 G_OBS，供仿真器校验状态位联动：
//   bit0 = HSERDY 观察成功；bit1 = PLLRDY 观察成功；bit2 = SWS==PLL 观察成功。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_OBS  状态位联动观察位（期望 7）
//   0x20000004 G_DONE 主线完成标记（期望 0xAAAAAAAA）

#include <stdint.h>

/* RCC @ 0x40023800 */
#define RCC_CR       (*(volatile uint32_t *)0x40023800u)
#define RCC_PLLCFGR  (*(volatile uint32_t *)0x40023804u)
#define RCC_CFGR     (*(volatile uint32_t *)0x40023808u)
#define RCC_AHB1ENR  (*(volatile uint32_t *)0x40023830u) /* bit0 = GPIOA 时钟 */

#define CR_HSEON   (1u << 16)
#define CR_HSERDY  (1u << 17)
#define CR_PLLON   (1u << 24)
#define CR_PLLRDY  (1u << 25)

#define CFGR_SWS_MASK (0x3u << 2)

/* 结果区（固定 SRAM 地址） */
#define G_OBS  (*(volatile uint32_t *)0x20000000u)
#define G_DONE (*(volatile uint32_t *)0x20000004u)

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 块边界辅助（noinline 保证成块边界）：调用返回位置即 block hook 检查点 */
static void rcc_ready_hook(void) __attribute__((noinline));
static void rcc_ready_hook(void) {
    __asm volatile("nop" ::: "memory");
}

static void rcc_done_hook(void) __attribute__((noinline));
static void rcc_done_hook(void) {
    __asm volatile("nop" ::: "memory");
}

/* 向量表：系统异常 + 若干 IRQ（index 16..；本固件无中断，全 Default） */
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
    (uint32_t)Default_Handler, /* 16..18: IRQ0-2 */
    (uint32_t)Default_Handler,
    (uint32_t)Default_Handler,
};

void Reset_Handler(void) {
    /* 1. 使能 GPIOA 时钟（寄存器镜像） */
    RCC_AHB1ENR = 1u;

    /* 2. 使能 HSE，轮询 HSERDY 就绪 */
    RCC_CR = CR_HSEON;
    while (!(RCC_CR & CR_HSERDY)) {}
    G_OBS |= 1u;

    /* 3. 配置 PLLCFGR（HSE 8MHz：M=8 → VCO_IN 1MHz，N=336 → VCO 336MHz，P=2 → 168MHz） */
    RCC_PLLCFGR = (7u << 24) | (1u << 22) | (336u << 6) | (8u << 0);

    /* 4. 使能 PLL，轮询 PLLRDY 就绪 */
    RCC_CR = CR_HSEON | CR_PLLON;
    while (!(RCC_CR & CR_PLLRDY)) {}
    G_OBS |= 2u;

    /* 5. 切换系统时钟到 PLL，并配置 AHB/APB 预分频 */
    RCC_CFGR = (2u << 0) | (0u << 4) | (5u << 10) | (4u << 13); /* SW=PLL, HPRE=/1, PPRE1=/4, PPRE2=/2 */
    while ((RCC_CFGR & CFGR_SWS_MASK) != (2u << 2)) {} /* 等 SWS==PLL */
    G_OBS |= 4u;

    rcc_ready_hook();

    rcc_done_hook();
    G_DONE = 0xAAAAAAAAu;

    for (;;) {}
}
