/* SPDX-License-Identifier: MIT */
/* CPU2 side of the CPU1<->CPU2 mailbox: reference table, system channel and
 * the Thread/Spinel channels, as laid out by ST's public transport layer. */
#include "tl.h"

#define RD8(a) (*(volatile uint8_t *)(uintptr_t)(a))
#define WR8(a, v) (*(volatile uint8_t *)(uintptr_t)(a) = (uint8_t)(v))

static uint32_t ref_table;
static volatile uint32_t pending; /* channels masked by the ISR, awaiting service */

static uint32_t ref_entry(unsigned i) { return RD32(ref_table + 4 * i); }

static void list_insert_tail(uint32_t head, uint32_t node) {
    uint32_t tail = RD32(head + 4);
    if (tail == 0) tail = head;
    WR32(node, head);
    WR32(node + 4, tail);
    WR32(tail, node);
    WR32(head + 4, node);
}

static void set_c2_flag(unsigned ch) { WR32(IPCC_C2SCR, 1u << (16 + ch - 1)); }
static void release_c1_flag(unsigned ch) {
    WR32(IPCC_C2SCR, CH_BIT(ch));
    WR32(IPCC_C2MR, RD32(IPCC_C2MR) & ~CH_BIT(ch));
}

void tl_isr(void) {
    uint32_t mr = RD32(IPCC_C2MR);
    uint32_t rx = RD32(IPCC_C1TOC2SR) & 0x3fu & ~mr;
    if (rx) {
        WR32(IPCC_C2MR, mr | rx);
        pending |= rx;
    }
    /* "channel free" interrupt: one shot, re-armed by tl_notify() */
    uint32_t txf = ~RD32(IPCC_C2TOC1SR) & ~mr & 0x3f0000u;
    if (txf) WR32(IPCC_C2MR, RD32(IPCC_C2MR) | txf);
}

/* Wireless firmware info as read by SHCI_GetWirelessFwInfo(): Thread MTD stack. */
static void fill_device_info(void) {
    uint32_t info = ref_entry(0);
    if (!info) return;
    WR32(info + 16, 0x00010000u);                              /* v0.1.0 */
    WR32(info + 20, (20u << 24) | (10u << 16) | (192u << 8) | 220u);
    WR32(info + 24, 0x11u);                                    /* Thread MTD */
    WR32(info + 28, 0);
}

static void send_ready_event(void) {
    uint32_t sys = ref_entry(3), mm = ref_entry(4);
    if (!sys || !mm) return;
    uint32_t pkt = RD32(mm + 4); /* spare system event buffer */
    if (!pkt) return;
    list_insert_tail(RD32(sys + 4), pkt);
    WR8(pkt + 8, 0x12);  /* TL_SYSEVT_PKT_TYPE */
    WR8(pkt + 9, 0xff);  /* vendor event */
    WR8(pkt + 10, 3);
    WR8(pkt + 11, 0x00); /* SHCI_SUB_EVT_CODE_READY = 0x9200 */
    WR8(pkt + 12, 0x92);
    WR8(pkt + 13, 0x00); /* wireless firmware running */
    set_c2_flag(CH_SYS_CMD);
}

void tl_init(void) {
    ref_table = REF_TABLE_BASE + 4 * (RD32(FLASH_IPCCBR) & 0x3fffu);
    fill_device_info();
    WR32(IPCC_C2MR, 0xffffffffu & ~(CH_BIT(CH_SYS_CMD) | CH_BIT(CH_CLI_CMD)));
    WR32(IPCC_C2CR, IPCC_CR_RXOIE | IPCC_CR_TXFIE);
    WR32(NVIC_ISER, (1u << IRQ_IPCC));
    send_ready_event();
}

static void sys_command(void) {
    uint32_t buf = RD32(ref_entry(3)); /* system command buffer */
    uint16_t opcode = (uint16_t)(RD8(buf + 9) | (RD8(buf + 10) << 8));
    /* Every SHCI command completes with status 0. The response replaces the
     * buffer from byte 0 (no packet header), as the SHCI layer expects. */
    WR8(buf + 0, 0x04); WR8(buf + 1, 0x0e); WR8(buf + 2, 4); WR8(buf + 3, 1);
    WR8(buf + 4, opcode & 0xff); WR8(buf + 5, opcode >> 8); WR8(buf + 6, 0);
    release_c1_flag(CH_SYS_CMD);
}

static void cli_command(void) {
    uint32_t thread = ref_entry(2);
    uint32_t buf = RD32(thread + 4); /* CLI command buffer */
    uint8_t local[255];
    uint32_t len = RD8(buf + 11);
    for (uint32_t i = 0; i < len; i++) local[i] = RD8(buf + 12 + i);
    release_c1_flag(CH_CLI_CMD);
    spinel_rx_hdlc(local, len);
}

void tl_poll(void) {
    uint32_t p;
    __asm volatile("cpsid i");
    p = pending;
    pending = 0;
    __asm volatile("cpsie i");
    if (p & CH_BIT(CH_SYS_CMD)) sys_command();
    if (p & CH_BIT(CH_CLI_CMD)) cli_command();
}

bool tl_notify(const uint8_t *data, uint32_t len) {
    if (RD32(IPCC_C2TOC1SR) & CH_BIT(CH_THREAD_NOT)) {
        /* wake us when CPU1 releases the channel */
        WR32(IPCC_C2MR, RD32(IPCC_C2MR) & ~(1u << (16 + CH_THREAD_NOT - 1)));
        return false;
    }
    uint32_t buf = RD32(ref_entry(2)); /* notification + ack buffer */
    WR8(buf + 8, 0x0c);  /* TL_OTNOT_PKT_TYPE */
    WR8(buf + 9, 0xff);
    WR8(buf + 10, 8 + len);
    /* The packet header is byte-packed: these fields are not word-aligned.
     * Cortex-M0+ faults on unaligned STR, so serialize them bytewise. */
    WR8(buf + 11, 0);   /* packet id */
    WR8(buf + 12, 0);
    WR8(buf + 13, 0);
    WR8(buf + 14, 0);
    WR8(buf + 15, len);
    WR8(buf + 16, len >> 8);
    WR8(buf + 17, len >> 16);
    WR8(buf + 18, len >> 24);
    for (uint32_t i = 0; i < len; i++) WR8(buf + 19 + i, data[i]);
    set_c2_flag(CH_THREAD_NOT);
    return true;
}
