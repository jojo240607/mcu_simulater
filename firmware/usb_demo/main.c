// M16 demo 固件：USB OTG FS 全速设备控制器（STM32F407）验收。
//
// 场景（USB_OTG_FS @ 0x50000000，AHB1；设备模式简化模型）：
//   虚拟主机（仿真器测试）经事件总线注入 USBRST/SETUP/OUT：
//   1. USBRST   → 固件地址归 0、使能 EP0（DOEPTSIZ0.STUPCNT/PKTCNT + DOEPCTL0.EPENA）；
//   2. SETUP 请求（8 字节标准请求，读 GRXSTSP+DFIFO0）：
//      - GET_DESCRIPTOR(Device)    → EP0 IN 回发 18 字节设备描述符；
//      - GET_DESCRIPTOR(Config)    → EP0 IN 回发 32 字节配置描述符；
//      - SET_ADDRESS 0x2A          → DCFG.DAD=0x2A + 状态阶段零长包；
//      - SET_CONFIGURATION 1       → 使能 EP1（批量）并回发 welcome[6]；
//   3. OUT EP1（64 字节 pattern 0x55+i）→ 校验通过置 G_EP1_RX=1 并回显。
//   IN 完成：写 DIEPTSIZx+DIEPCTLx.EPENA+DFIFOx 触发 DIEPINTx.XFRC，
//   GINTMSK(USBRST/ENUMDNE/RXFLVL)+DAINTMSK×DIEPMSK/DOEPMSK 门控挂起 OTG_FS IRQ。
//   处理完 4 个 SETUP 后置 G_DONE。
//
// 固定地址结果区（供仿真器校验）：
//   0x20000000 G_DONE     处理完 4 个 SETUP（期望 0xAAAAAAAA）
//   0x20000004 G_ADDR     SET_ADDRESS 后设备地址（期望 0x2A）
//   0x20000008 G_CFG      SET_CONFIGURATION 值（期望 1）
//   0x2000000C G_EP1_RX   OUT EP1 64B pattern 校验（期望 1）
//   0x20000010 G_EP1_TX   IN EP1 welcome/回显发送校验（期望 1）
//   0x20000014 G_SETUP_N  处理的 SETUP 数（期望 >= 4）

#include <stdint.h>

/* USB OTG FS 基地址（AHB1） */
#define USB_BASE 0x50000000u
/* RCC：AHB1 外设时钟使能（USB OTG FS = bit30） */
#define RCC_BASE    0x40023800u
#define REG_AHB1ENR 0x30u
#define AHB1ENR_OTGFSEN (1u << 30)

/* 全局寄存器偏移（与外设一致） */
#define REG_GAHBCFG  0x008u
#define REG_GUSBCFG  0x00Cu
#define REG_GRSTCTL  0x010u
#define REG_GINTSTS  0x014u
#define REG_GINTMSK  0x018u
#define REG_GRXSTSP  0x020u
#define REG_GCCFG    0x038u

/* 设备模式寄存器偏移 */
#define REG_DCFG     0x800u
#define REG_DSTS     0x808u
#define REG_DIEPMSK  0x810u
#define REG_DOEPMSK  0x814u
#define REG_DAINTMSK 0x81Cu

/* 端点寄存器偏移（每端点 0x20） */
#define REG_DIEPCTL0  0x900u
#define REG_DIEPTSIZ0 0x910u
#define REG_DIEPCTL1  0x920u
#define REG_DIEPTSIZ1 0x930u
#define REG_DOEPCTL0  0xB00u
#define REG_DOEPTSIZ0 0xB10u
#define REG_DOEPCTL1  0xB20u
#define REG_DOEPTSIZ1 0xB30u
#define REG_DFIFO0    0x1000u
#define REG_DFIFO1    0x2000u

/* 位定义（与 src/peripheral/usb_otg.rs 一致） */
#define GCCFG_PWRDWN   (1u << 16)
#define GUSBCFG_FDMOD  (1u << 30) /* 强制设备模式 */
#define GUSBCFG_PHYSEL (1u << 6)
#define GAHBCFG_GINT   (1u << 0)
#define GINT_RXFLVL    (1u << 4)
#define GINT_USBRST    (1u << 12)
#define GINT_ENUMDNE   (1u << 13)
#define DIEPMSK_XFRCM  (1u << 0)
#define DOEPMSK_XFRCM  (1u << 0)
#define DOEPMSK_STUPM  (1u << 3)
#define DAINT_IEP0     (1u << 0)
#define DAINT_OEP0     (1u << 16)
#define DAINT_IEP1     (1u << 1)
#define DAINT_OEP1     (1u << 17)
#define DCFG_DAD       (0x7Fu << 4)
#define EP_EPENA       (1u << 31)
#define EP_CNAK        (1u << 26)
#define EP_USBAEP      (1u << 15)
#define EP_EPTYP_CTRL  (0u << 18)
#define EP_EPTYP_BULK  (2u << 18)
#define EP_TXFNUM1     (1u << 22)
#define EP_MPS64       (64u)
#define TSIZ_PKTCNT1   (1u << 19)
#define TSIZ_STUPCNT1  (1u << 29)
#define GRX_PKTSTS_MASK (0xFu << 17)
/* pktsts 编码对齐 joc-base ST 设备库（st_usb/usb_defines.h）：
 * STS_DATA_UPDT=2 / STS_SETUP_COMP=4 / STS_SETUP_UPDT=6 */
#define GRX_PKTSTS_SETUP_DATA 6u
#define GRX_PKTSTS_OUT_DATA   2u
#define GRX_EPNUM_MASK (0xFu)
#define GRX_BCNT_MASK  (0x7FFu << 4)

/* 结果区 */
#define G_DONE     (*(volatile uint32_t *)0x20000000u)
#define G_ADDR     (*(volatile uint32_t *)0x20000004u)
#define G_CFG      (*(volatile uint32_t *)0x20000008u)
#define G_EP1_RX   (*(volatile uint32_t *)0x2000000Cu)
#define G_EP1_TX   (*(volatile uint32_t *)0x20000010u)
#define G_SETUP_N  (*(volatile uint32_t *)0x20000014u)

static inline uint32_t rd(uint32_t base, uint32_t off) {
    return *(volatile uint32_t *)(base + off);
}
static inline void wr(uint32_t base, uint32_t off, uint32_t v) {
    *(volatile uint32_t *)(base + off) = v;
}

/* ---- 标准 USB 描述符 ---- */
static const uint8_t device_desc[18] = {
    18, 0x01, 0x00, 0x02,          /* bLength, bDescriptorType=DEVICE, bcdUSB=0x0200 */
    0x00, 0x00, 0x00, 0x40,        /* class/subclass/proto, bMaxPacketSize0=64 */
    0x83, 0x12, 0x00, 0x00,        /* idVendor=0x0483, idProduct */
    0x01, 0x00, 0x01, 0x02,        /* bcdDevice, iMfr, iProduct, iSerial */
    0x01,                          /* bNumConfigurations */
};
static const uint8_t config_desc[32] = {
    9, 0x02, 32, 0x00, 1, 0x00, 0x00, 0x80, 50, /* 配置：总长 32、1 接口 */
    9, 0x04, 0, 0x00, 2, 0xFF, 0xFF, 0x00, 0,   /* 接口：2 端点、厂商类 */
    7, 0x05, 0x81, 0x02, 64, 0x00, 0,           /* EP1 IN 批量 64B */
    7, 0x05, 0x01, 0x02, 64, 0x00, 0,           /* EP1 OUT 批量 64B */
};
static const uint8_t welcome_msg[6] = { 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x21 }; /* "Hello!" */

/* ---- EP0 发送（IN；len 可为 0 = 状态阶段零长包） ---- */
static void ep0_send(const uint8_t *data, uint32_t len) {
    wr(USB_BASE, REG_DIEPTSIZ0, TSIZ_PKTCNT1 | len);
    wr(USB_BASE, REG_DIEPCTL0,
       EP_EPENA | EP_CNAK | EP_USBAEP | EP_EPTYP_CTRL | EP_MPS64);
    if (data) {
        for (uint32_t i = 0; i < len; i += 4) {
            uint32_t w = 0;
            for (uint32_t j = 0; j < 4 && i + j < len; j++) {
                w |= (uint32_t)data[i + j] << (8 * j);
            }
            wr(USB_BASE, REG_DFIFO0, w);
        }
    }
}
/* ---- EP1（批量）发送（IN） ---- */
static void ep1_send(const uint8_t *data, uint32_t len) {
    wr(USB_BASE, REG_DIEPTSIZ1, TSIZ_PKTCNT1 | len);
    wr(USB_BASE, REG_DIEPCTL1,
       EP_EPENA | EP_CNAK | EP_USBAEP | EP_EPTYP_BULK | EP_TXFNUM1 | EP_MPS64);
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t w = 0;
        for (uint32_t j = 0; j < 4 && i + j < len; j++) {
            w |= (uint32_t)data[i + j] << (8 * j);
        }
        wr(USB_BASE, REG_DFIFO1, w);
    }
}
/* 从接收 FIFO（DFIFO0）读 bcnt 字节 */
static void read_fifo(uint8_t *buf, uint32_t bcnt) {
    for (uint32_t i = 0; i < bcnt; i += 4) {
        uint32_t w = rd(USB_BASE, REG_DFIFO0);
        for (uint32_t j = 0; j < 4 && i + j < bcnt; j++) {
            buf[i + j] = (uint8_t)(w >> (8 * j));
        }
    }
}
/* 使能 EP0 OUT 接收 SETUP + EP0 IN */
static void setup_ep0(void) {
    wr(USB_BASE, REG_DOEPTSIZ0, TSIZ_STUPCNT1 | TSIZ_PKTCNT1 | 64u);
    wr(USB_BASE, REG_DOEPCTL0, EP_EPENA | EP_CNAK | EP_USBAEP | EP_EPTYP_CTRL | EP_MPS64);
    wr(USB_BASE, REG_DIEPCTL0, EP_USBAEP | EP_EPTYP_CTRL | EP_MPS64);
}
/* 使能 EP1（批量）收发 */
static void setup_ep1(void) {
    wr(USB_BASE, REG_DOEPTSIZ1, TSIZ_PKTCNT1 | 64u);
    wr(USB_BASE, REG_DOEPCTL1, EP_EPENA | EP_CNAK | EP_USBAEP | EP_EPTYP_BULK | EP_MPS64);
    wr(USB_BASE, REG_DIEPCTL1, EP_USBAEP | EP_EPTYP_BULK | EP_TXFNUM1 | EP_MPS64);
}

/* 处理 8 字节 SETUP 请求 */
static void handle_setup(const uint8_t *b) {
    uint32_t bmReq = b[0];
    uint32_t bReq = b[1];
    uint32_t wValue = (uint32_t)b[2] | ((uint32_t)b[3] << 8);
    uint32_t wLength = (uint32_t)b[6] | ((uint32_t)b[7] << 8);
    G_SETUP_N++;

    if (bmReq & 0x80u) { /* 设备 → 主机 */
        if (bReq == 0x06u) { /* GET_DESCRIPTOR */
            uint32_t descType = wValue >> 8;
            if (descType == 1u) {
                ep0_send(device_desc, sizeof(device_desc));
            } else if (descType == 2u) {
                ep0_send(config_desc, sizeof(config_desc));
            }
        }
        /* wLength 忽略（简化：整包回发） */
    } else { /* 主机 → 设备 */
        if (bReq == 0x05u) { /* SET_ADDRESS */
            uint32_t addr = wValue & 0x7Fu;
            wr(USB_BASE, REG_DCFG, (rd(USB_BASE, REG_DCFG) & ~DCFG_DAD) | (addr << 4));
            G_ADDR = addr;
            ep0_send(0, 0); /* 状态阶段零长包 */
        } else if (bReq == 0x09u) { /* SET_CONFIGURATION */
            G_CFG = wValue;
            if (wValue == 1u) {
                setup_ep1();
                ep1_send(welcome_msg, sizeof(welcome_msg));
                G_EP1_TX = 1;
            }
            ep0_send(0, 0); /* 状态阶段零长包 */
        }
    }
}

/* 处理 OUT 数据包（批量 EP1 回显校验） */
static void handle_out(uint32_t ep, const uint8_t *buf, uint32_t bcnt) {
    if (ep == 1u && bcnt == 64u) {
        uint32_t ok = 1;
        for (uint32_t i = 0; i < 64u; i++) {
            if (buf[i] != (uint8_t)(0x55u + i)) ok = 0;
        }
        if (ok) {
            G_EP1_RX = 1;
            ep1_send(buf, 64); /* 回显 */
            G_EP1_TX = 1;
        }
    }
}

extern void Reset_Handler(void);

static void Default_Handler(void) {
    for (;;) {}
}

/* 向量表：系统异常 + IRQ 占位（本固件轮询 GINTSTS，不使用 USB IRQ 中断） */
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
    /* 上电 USB PHY（清 PWRDWN）+ RCC AHB1 时钟使能（OTGFSEN） */
    wr(RCC_BASE, REG_AHB1ENR, rd(RCC_BASE, REG_AHB1ENR) | AHB1ENR_OTGFSEN);
    wr(USB_BASE, REG_GCCFG, rd(USB_BASE, REG_GCCFG) & ~GCCFG_PWRDWN);

    /* 设备模式 + 全局中断使能 + 中断门控 */
    wr(USB_BASE, REG_GUSBCFG, GUSBCFG_FDMOD | GUSBCFG_PHYSEL);
    wr(USB_BASE, REG_GAHBCFG, GAHBCFG_GINT);
    wr(USB_BASE, REG_GINTMSK, GINT_USBRST | GINT_ENUMDNE | GINT_RXFLVL);
    wr(USB_BASE, REG_DIEPMSK, DIEPMSK_XFRCM);
    wr(USB_BASE, REG_DOEPMSK, DOEPMSK_STUPM | DOEPMSK_XFRCM);
    wr(USB_BASE, REG_DAINTMSK, DAINT_IEP0 | DAINT_OEP0 | DAINT_IEP1 | DAINT_OEP1);

    for (;;) {
        uint32_t g = rd(USB_BASE, REG_GINTSTS);
        if (g & GINT_USBRST) {
            wr(USB_BASE, REG_GINTSTS, GINT_USBRST);
            wr(USB_BASE, REG_DCFG, rd(USB_BASE, REG_DCFG) & ~DCFG_DAD); /* 地址归 0 */
            setup_ep0();
        }
        if (g & GINT_ENUMDNE) {
            wr(USB_BASE, REG_GINTSTS, GINT_ENUMDNE);
            /* ENUMSPD=FS（DSTS 复位默认）；EP0 MPS 已在 setup_ep0 设为 64 */
        }
        if (g & GINT_RXFLVL) {
            for (;;) {
                uint32_t st = rd(USB_BASE, REG_GRXSTSP);
                if (st == 0u) break; /* 接收状态队列空 */
                uint32_t pktsts = (st & GRX_PKTSTS_MASK) >> 17;
                uint32_t epnum = st & GRX_EPNUM_MASK;
                uint32_t bcnt = (st & GRX_BCNT_MASK) >> 4;
                if (pktsts == GRX_PKTSTS_SETUP_DATA) {
                    uint8_t buf[8];
                    read_fifo(buf, 8);
                    handle_setup(buf);
                } else if (pktsts == GRX_PKTSTS_OUT_DATA) {
                    uint8_t buf[64];
                    read_fifo(buf, bcnt);
                    handle_out(epnum, buf, bcnt);
                }
                /* SETUP_COMP 等其余状态忽略 */
            }
        }
        if (G_SETUP_N >= 4u) {
            G_DONE = 0xAAAAAAAAu;
        }
    }
}
