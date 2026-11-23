/* SPDX-License-Identifier: MIT */
#include "radio.h"

#define R(off)    (0x60001000u + (off))   /* 16-bit MAC registers */
#define R_SET(off) (0x60001400u + (off))  /* write 1: set bits    */
#define R_CLR(off) (0x60001800u + (off))  /* write 1: clear bits  */

#define RFBUS       0x6000000cu           /* RF register bus command word */
#define RADIO_CTRL  0x60000404u
#define RADIO_IRQ   0x60000418u
#define RCC_BASE    0x58000000u
#define RCC_C2APB3ENR (RCC_BASE + 0x164)
#define PWR_SCR     0x58000418u

#define OP_TX_FIFO  0x16u
#define OP_TX_NOW   0x04u
#define OP_TX_DELAY 0x06u
#define OP_ABORT    0x14u
#define OP_RX_FLUSH 0x15u

#define SPIN_CAP 200000u

static uint16_t rd(uint32_t a) { return (uint16_t)RD32(a); }

/* ---- time base: 24-bit counter, 16 us per tick ---- */
uint32_t radio_ticks(void) {
    uint32_t lo, hi;
    __asm volatile("cpsid i");
    lo = rd(R(0x090)) & 0xffffu;
    hi = rd(R(0x094)) & 0xffu;
    __asm volatile("cpsie i");
    return (hi << 16) | lo;
}
uint32_t radio_ticks_add(uint32_t t, uint32_t d) {
    t += d;
    if (t >= RADIO_COUNTER_MOD) t -= RADIO_COUNTER_MOD;
    return t;
}
uint32_t radio_ticks_until(uint32_t at) {
    uint32_t now = radio_ticks();
    uint32_t d = at >= now ? at - now : at + RADIO_COUNTER_MOD - now;
    return d > RADIO_COUNTER_MOD / 2 ? 0 : d; /* "behind" counts as reached */
}
void radio_delay_ticks(uint32_t n) {
    uint32_t end = radio_ticks_add(radio_ticks(), n + 1);
    for (uint32_t i = 0; i < SPIN_CAP && radio_ticks_until(end); i++) {}
}

/* ---- RF register bus: descriptors live in SRAM2 (offset from 0x20030000) ---- */
static volatile uint8_t desc[24] __attribute__((aligned(4)));

static bool rf_run(void) {
    WR32(RFBUS, (((uint32_t)(uintptr_t)desc - 0x20030000u) & 0xffffu) | 0x10000u);
    for (uint32_t i = 0; i < SPIN_CAP; i++)
        if (!(RD32(RFBUS) & 0x10000u)) return true;
    return false;
}
static bool rf_write_n(uint8_t idx, const uint8_t *d, uint8_t n) {
    desc[0] = n + 1; desc[1] = idx;
    for (uint8_t i = 0; i < n; i++) desc[2 + i] = d[i];
    desc[2 + n] = 0;
    return rf_run();
}
static bool rf_write(uint8_t idx, uint8_t v) { return rf_write_n(idx, &v, 1); }
static uint8_t rf_read(uint8_t idx) {
    desc[0] = 0x82; desc[1] = idx; desc[2] = 0xa5; desc[3] = 0;
    return rf_run() ? desc[2] : 0;
}
static void rf_update(uint8_t idx, uint8_t clear, uint8_t set) {
    rf_write(idx, (uint8_t)((rf_read(idx) & ~clear) | set));
}
static bool rf_wait_state(uint8_t want) {
    for (uint32_t i = 0; i < SPIN_CAP; i++)
        if ((rf_read(0x5c) & 0x1f) == want) return true;
    return false;
}
static void mac_wait_idle(void) {
    for (uint32_t i = 0; i < SPIN_CAP && (rd(R(0x000)) & 0x1f); i++) {}
}

/* ---- bring-up ---- */
bool radio_init(void) {
    static const uint8_t rf_table[21] = { 0x4c, 0x85, 0x06, 0x1a, 0x72, 0x0f, 0x00, 0x00,
        0x00, 0x90, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0d, 0x1d, 0x2d, 0x3d, 0x80 };

    /* radio clocks: reset pulse and 802.15.4 enable */
    WR32(RCC_BASE + 0x44, RD32(RCC_BASE + 0x44) | 1u);
    WR32(RCC_C2APB3ENR, RD32(RCC_C2APB3ENR) | 1u);
    WR32(RCC_C2APB3ENR, RD32(RCC_C2APB3ENR) | 2u);
    WR32(RCC_BASE + 0x44, RD32(RCC_BASE + 0x44) & ~1u);
    for (uint32_t i = 0; i < SPIN_CAP && (RD32(RCC_BASE + 0x94) & (1u << 16)); i++) {}

    /* power the RF domain; the bus word only holds a value while powered */
    WR32(RADIO_CTRL, 3);
    bool up = false;
    for (int i = 0; i < 1000 && !up; i++) {
        WR32(RFBUS, 0xa5a5);
        up = (RD32(RFBUS) & 0xffffu) == 0xa5a5u;
    }
    if (!up) return false;
    WR32(RADIO_IRQ, RD32(RADIO_IRQ) | 2u);
    WR32(RADIO_IRQ, RD32(RADIO_IRQ) & 0xffffc0f7u);

    rf_write(0x2d, 0x60);
    rf_update(0x05, 0, 0x06);
    rf_update(0x06, 0, 0x01);
    rf_update(0x41, 0x40, 0x80);
    rf_write_n(0x65, rf_table, sizeof rf_table);
    rf_write(0x39, 0);
    rf_update(0x0e, 0x1e, 0x02);
    rf_wait_state(4);

    /* MAC block: mask everything, then the fixed configuration */
    WR32(R(0x0b4), 0); WR32(R(0x0b8), 0);
    WR32(R(0x0f8), 0xc0);
    WR32(R(0x1e8), 0x87d);
    WR32(R_CLR(0x0ec), 0xff);
    WR32(PWR_SCR, 0x400);
    for (uint32_t i = 0; i < SPIN_CAP && !(RD32(R(0x3ec)) & 0x80); i++) {}
    WR32(R(0x188), 0);
    WR32(R(0x194), 0x4000u | (0x2fu << 8) | 0x32u); /* CCA: mode 1, thresholds */
    WR32(R(0x184), 0x1ff);
    WR32(R(0x1c8), 0x084d); WR32(R(0x1cc), 0x8600);  /* IRQ8 masks  */
    WR32(R(0x1d0), 0x1700); WR32(R(0x1d4), 0x7801);  /* IRQ22 masks */
    WR32(R(0x1d8), 0);
    WR32(R(0x1f0), 0x5f80);
    WR32(R_CLR(0x100), 0x400);

    /* counter modulus, compares off */
    WR32(R(0x198), 0x322e); WR32(R(0x19c), 0xab);
    WR32(R(0x1a8), 0); WR32(R(0x1b0), 0); WR32(R(0x1b8), 0); WR32(R(0x1c0), 0);
    WR32(R_CLR(0x1cc), 0x40);
    WR32(R_CLR(0x0ec), 1);
    WR32(R_SET(0x0ec), 2);
    radio_delay_ticks(7);
    WR32(R(0x0b4), 0); WR32(R(0x0b8), 0);
    WR32(PWR_SCR, 0x400);
    WR32(R_CLR(0x100), 1);
    WR32(R(0x100), rd(R(0x100)) & 0xff3fu);
    WR32(R_CLR(0x100), 4);
    WR32(R(0x100), (rd(R(0x100)) & 0x9fffu) | 0x2000u);
    WR32(R(0x100), rd(R(0x100)) & 0xefffu);
    WR32(R_SET(0x100), 2);
    WR32(R_SET(0x100), 0x8000); /* hardware appends/checks the FCS */
    WR32(R(0x138), rd(R(0x138)) & 0xf000u); /* address filters off: software filters */
    WR32(R(0x130), rd(R(0x130)) & 0xfffcu);
    WR32(R(0x0b4), 0); WR32(R(0x0b8), 0);
    WR32(PWR_SCR, 0x400);

    WR32(NVIC_ISER, (1u << IRQ_RADIO_RX) | (1u << IRQ_RADIO_TX));
    return true;
}

/* ---- state, channel, power ---- */
bool radio_rx_is_on(void) { return (rd(R(0x100)) & 1u) != 0; }

void radio_rx(bool on) {
    mac_wait_idle();
    if (on && !radio_rx_is_on()) {
        WR32(R_SET(0x100), 1);
        rf_wait_state(0x1a);
    } else if (!on && radio_rx_is_on()) {
        WR32(R_CLR(0x100), 1);
        rf_wait_state(4);
    }
}

bool radio_set_channel(uint8_t ch) {
    if (ch < 11 || ch > 26) return false;
    bool was_on = radio_rx_is_on();
    radio_rx(false);
    rf_write(4, (uint8_t)((ch - 11) * 2));
    radio_delay_ticks(7);
    radio_rx(was_on);
    return true;
}

bool radio_set_power(int8_t dbm) {
    static const uint8_t code[29] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x14, 0x16, 0x19, 0x1a,
        0x1b, 0x1c, 0x1d, 0x1e, 0x1f };
    if (dbm < -21 || dbm > 6) return false;
    rf_update(0x05, 0xf8, (uint8_t)(code[dbm + 22] << 3));
    return true;
}

int8_t radio_rssi_now(void) {
    uint16_t v = 0;
    for (int i = 0; i < 200; i++) {
        v = rd(R(0x080));
        if (v & 0x8000u) break;
    }
    return (int8_t)(((v >> 4) & 0xffu) - RADIO_RSSI_OFFSET);
}

void radio_eui64(uint8_t out[8]) {
    /* UID64 in system memory; Spinel wants the most significant byte first */
    for (int i = 0; i < 8; i++) out[i] = *(volatile uint8_t *)(uintptr_t)(0x1fff7580u + 7 - i);
}

/* ---- transmit ---- */
bool radio_tx_busy(void) { return (rd(R(0x000)) & 7u) != 0 || (rd(R(0x048)) & 0x8000u) != 0; }

bool radio_tx_load(const uint8_t *mpdu, uint8_t len) {
    if (len > 125 || radio_tx_busy()) return false;
    WR32(R(0x3fc), ((uint32_t)(len + 2) << 8) | OP_TX_FIFO); /* PHR includes the FCS */
    for (uint8_t i = 0; i < len; i++) WR32(R(0x3fc), ((uint32_t)mpdu[i] << 8) | OP_TX_FIFO);
    return true;
}

void radio_tx_start(uint32_t delay) {
    if (delay == 0) {
        WR32(R(0x3fc), OP_TX_NOW);
    } else {
        WR32(R(0x3f8), delay > 0xffffu ? 0xffffu : delay);
        WR32(R(0x3fc), OP_TX_DELAY);
    }
}

/* ---- compares ---- */
void radio_alarm_set(uint32_t at) {
    WR32(R(0x1c0), at & 0xffffu); WR32(R(0x1c4), at >> 16);
    WR32(R_CLR(0x0ec), 8); WR32(R_SET(0x0ec), 4);
    WR32(R(0x8b8), 0x80);                 /* drop a stale E1.7 */
    WR32(R_SET(0x1d4), 0x80);
}
void radio_alarm_clear(void) { WR32(R_CLR(0x1d4), 0x80); WR32(R_CLR(0x0ec), 0xc); }
void radio_ack_timer_set(uint32_t at) {
    WR32(R(0x1a8), at & 0xffffu); WR32(R(0x1ac), at >> 16);
    WR32(R(0x8b8), 0x10);                 /* drop a stale E1.4 */
    WR32(R_SET(0x1d4), 0x10);
}
void radio_ack_timer_clear(void) { WR32(R_CLR(0x1d4), 0x10); }

/* ---- interrupts ---- */
void radio_isr_rx(void) {
    for (;;) {
        uint32_t e0 = rd(R(0x0b4)) & rd(R(0x1c8));
        uint32_t e1 = rd(R(0x0b8)) & rd(R(0x1cc));
        if (!e0 && !e1) break;
        WR32(R(0x8b4), e0);
        WR32(R(0x8b8), e1);
        if (e0 & 0x1u) { /* frame complete: PHR length includes the FCS */
            uint8_t len = (uint8_t)(rd(R(0x008)) & 0x7fu);
            uint16_t rs = rd(R(0x00c));
            uint8_t buf[127];
            uint32_t now = radio_ticks();
            for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)RD32(R(0x3e8));
            if (len >= 5)
                mac_rx_isr(buf, (uint8_t)(len - 2),
                           (int8_t)(((rs >> 4) & 0xffu) - RADIO_RSSI_OFFSET), now);
        }
        if (e0 & 0x2u) WR32(R(0x3fc), OP_RX_FLUSH); /* rejected frame */
    }
}

void radio_isr_tx(void) {
    for (;;) {
        uint32_t e0 = rd(R(0x0b4)) & rd(R(0x1d0));
        uint32_t e1 = rd(R(0x0b8)) & rd(R(0x1d4));
        if (!e0 && !e1) break;
        WR32(R(0x8b4), e0);
        WR32(R(0x8b8), e1);
        if (e0 & 0x1000u) mac_txdone_isr();
        if (e0 & 0x0700u) { WR32(R(0x3fc), OP_ABORT); mac_txerror_isr((e0 & 0x400u) != 0); }
        if (e1 & 0x1800u) { WR32(R(0x3fc), OP_ABORT); mac_txerror_isr(false); }
        if (e1 & 0x10u) { radio_ack_timer_clear(); mac_ack_timeout_isr(); }
        if (e1 & 0x80u) { radio_alarm_clear(); mac_alarm_isr(); }
    }
}
