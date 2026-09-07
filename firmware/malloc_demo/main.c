// malloc_demo 固件：验证模拟器对 newlib-nano malloc/calloc/free 的支持是否正常。
//
// 目的：jOS 启动诊断发现 board_init 阶段堆被快速耗尽（sbrk 145 次 / 累计 0x34E8B），
// 且第 3 个设备创建触发 88 次 malloc(80)。为隔离"是模拟器问题还是 jOS 问题"，
// 本固件用与 jOS 完全相同的 newlib-nano(malloc/calloc/free) + 静态堆(_sbrk) 技术栈，
// 跑一段确定性测试：
//   1. malloc(32) 写读回验证可写性；
//   2. calloc(10,4) 验证清零；
//   3. free 后地址复用（验证空闲链表）；
//   4. malloc(80) 填充循环直到返回 NULL，统计累计分配字节 + 验证耗尽正确返回 NULL；
//   5. 耗尽后 free 首块再 malloc，验证空闲块能被复用且可写。
//
// 结果区（固定 SRAM 地址，SRAM 起始 0x20000000 预留 0x20 字节）：
//   0x20000000 R_BASIC            1=基本 malloc 可写性 OK
//   0x20000004 R_CALLOC           1=calloc 清零 OK
//   0x20000008 R_FREE_REUSE       1=free 后地址复用 OK
//   0x2000000C R_HEAP_TOTAL       填充循环成功分配的总字节数
//   0x20000010 R_HEAP_EXHAUST     1=耗尽时 malloc 正确返回 NULL
//   0x20000014 R_REUSE_EXHAUST    1=耗尽后 free 首块可被复用且可写
//   0x20000018 R_DONE             完成标记 0xA5A5A5A5
//
// 编译（arm-none-eabi-gcc，与 jOS 相同 CPU_FLAGS + nano.specs）：
//   arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
//     -O1 -ffreestanding -nostartfiles -specs=nano.specs \
//     -Wl,-e,Reset_Handler -Wl,-T,linker.ld -o malloc_demo.elf main.c

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>

/* ---------------- 结果区（固定 SRAM 地址） ---------------- */
#define R_BASIC          (*(volatile uint32_t *)0x20000000u)
#define R_CALLOC         (*(volatile uint32_t *)0x20000004u)
#define R_FREE_REUSE     (*(volatile uint32_t *)0x20000008u)
#define R_HEAP_TOTAL     (*(volatile uint32_t *)0x2000000Cu)
#define R_HEAP_EXHAUST   (*(volatile uint32_t *)0x20000010u)
#define R_REUSE_EXHAUST  (*(volatile uint32_t *)0x20000014u)
#define R_DONE           (*(volatile uint32_t *)0x20000018u)

/* ---------------- 静态系统堆（8KB，与 jOS 发布版 SYS_HEAP_SIZE 相同） ---------------- */
static uint8_t g_sys_heap[0x2000] __attribute__((aligned(8)));

/* ---------------- newlib syscalls（最小集，风格同 jOS syscalls.c） ---------------- */
void *_sbrk(ptrdiff_t incr)
{
    static char *heap = (char *)g_sys_heap;
    char *prev = heap;
    char *limit = (char *)g_sys_heap + sizeof(g_sys_heap);
    /* 越界保护：堆不得超过 g_sys_heap 上界，否则返回 (void *)-1 让 malloc 失败 */
    if (incr > 0 && (heap + incr) > limit) {
        errno = ENOMEM;
        return (void *)-1;
    }
    heap += incr;
    return (void *)prev;
}

int _write(int file, char *ptr, int len) { (void)file; (void)ptr; return len; }
int _read(int file, char *ptr, int len)  { (void)file; (void)ptr; return 0; }
int _close(int file)                    { (void)file; return -1; }
int _lseek(int file, int ptr, int dir)  { (void)file; (void)ptr; (void)dir; return 0; }
int _fstat(int file, struct stat *st)   { (void)file; st->st_mode = S_IFCHR; return 0; }
int _isatty(int file)                   { (void)file; return (file < 3) ? 1 : 0; }
void _exit(int status)                  { (void)status; for (;;) {} }
int _kill(int pid, int sig)             { (void)pid; (void)sig; return -1; }
int _getpid(void)                       { return 1; }

/* ---------------- 向量表 ---------------- */
extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    0x20010000u,               /* 0: 初始 SP（SRAM 内，堆上方） */
    (uint32_t)Reset_Handler,   /* 1: Reset */
    (uint32_t)Default_Handler, /* 2: NMI */
    (uint32_t)Default_Handler, /* 3: HardFault */
    (uint32_t)Default_Handler, /* 4: MemManage */
    (uint32_t)Default_Handler, /* 5: BusFault */
    (uint32_t)Default_Handler, /* 6: UsageFault */
    (uint32_t)Default_Handler, /* 7: 保留 */
    (uint32_t)Default_Handler, /* 8: 保留 */
    (uint32_t)Default_Handler, /* 9: 保留 */
    (uint32_t)Default_Handler, /* 10: 保留 */
    (uint32_t)Default_Handler, /* 11: SVCall */
    (uint32_t)Default_Handler, /* 12: DebugMon */
    (uint32_t)Default_Handler, /* 13: 保留 */
    (uint32_t)Default_Handler, /* 14: PendSV */
    (uint32_t)Default_Handler, /* 15: SysTick */
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
};

/* ---------------- Reset_Handler：执行 malloc 测试序列 ---------------- */
void Reset_Handler(void)
{
    uint32_t ok;

    /* ---- 1. 基本 malloc + 可写性 ---- */
    ok = 0;
    {
        volatile uint8_t *b = (volatile uint8_t *)malloc(32);
        if (b) {
            int i, good = 1;
            for (i = 0; i < 32; i++) b[i] = (uint8_t)(0x11 + i);
            for (i = 0; i < 32; i++) if (b[i] != (uint8_t)(0x11 + i)) good = 0;
            if (good) ok = 1;
            free((void *)b);
        }
    }
    R_BASIC = ok;

    /* ---- 2. calloc 清零 ---- */
    ok = 0;
    {
        volatile uint8_t *b = (volatile uint8_t *)calloc(10, 4);
        if (b) {
            int i, good = 1;
            for (i = 0; i < 40; i++) if (b[i] != 0) good = 0;
            if (good) ok = 1;
            free((void *)b);
        }
    }
    R_CALLOC = ok;

    /* ---- 3. free 后地址复用（验证空闲链表） ---- */
    ok = 0;
    {
        void *a = malloc(80);
        void *b = malloc(80);
        if (a && b) {
            free(a);
            void *c = malloc(80);
            ok = (c == a) ? 1 : 0;
            if (c) free(c);
        }
        if (a) free(a);
        if (b) free(b);
    }
    R_FREE_REUSE = ok;

    /* ---- 4. malloc(80) 填充循环直到返回 NULL ---- */
    {
        static void *ptrs[256];
        int n = 0;
        uint32_t total = 0;
        for (n = 0; n < 256; n++) {
            uint8_t *q = (uint8_t *)malloc(80);
            if (!q) break;
            ptrs[n] = q;
            {
                int i;
                for (i = 0; i < 80; i++) q[i] = (uint8_t)(n + 1);
            }
            total += 80;
        }
        R_HEAP_TOTAL = total;
        R_HEAP_EXHAUST = (n < 256) ? 1 : 0;

        /* ---- 5. 耗尽后 free 首块，应能被复用且可写 ---- */
        ok = 0;
        if (n > 0) {
            free(ptrs[0]);
            {
                volatile uint8_t *r = (volatile uint8_t *)malloc(80);
                if (r) {
                    int i, good = 1;
                    for (i = 0; i < 80; i++) r[i] = 0xA5;
                    for (i = 0; i < 80; i++) if (r[i] != 0xA5) good = 0;
                    if (good) ok = 1;
                    free((void *)r);
                }
            }
        }
        R_REUSE_EXHAUST = ok;

        /* 释放全部剩余块，验证 free 不崩溃 */
        {
            int k;
            for (k = 0; k < n; k++) free(ptrs[k]);
        }
    }

    R_DONE = 0xA5A5A5A5u;

    for (;;) {}
}
