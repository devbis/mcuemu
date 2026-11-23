/* SPDX-License-Identifier: MIT */
/* Minimal OpenThread-style RCP for IEEE 802.15.4: the Spinel subset that the
 * Zephyr STM32WB driver uses, plus the MAC duties a real radio co-processor
 * has (address filter, automatic ACK, CSMA-CA, ACK wait). */
#include "mac.h"
#include "tl.h"
#include "radio.h"

enum { CMD_GET = 2, CMD_SET = 3, CMD_IS = 6 };
enum { P_LAST_STATUS = 0x00, P_HWADDR = 0x08, P_PHY_ENABLED = 0x20, P_PHY_CHAN = 0x21,
       P_CCA_THRESHOLD = 0x24, P_TX_POWER = 0x25, P_RSSI = 0x26, P_LADDR = 0x34,
       P_SADDR = 0x35, P_PANID = 0x36, P_RAW_STREAM = 0x37, P_PROMISC = 0x38,
       P_STREAM_RAW = 0x71 };
enum { S_OK = 0, S_FAILURE = 1, S_INVALID_ARG = 3, S_INVALID_STATE = 4, S_BUSY = 12, S_PROP_NOT_FOUND = 13,
       S_NO_ACK = 17, S_CCA_FAILURE = 18 };

static struct {
    bool enabled, raw_stream;
    uint8_t channel, promisc;
    int8_t cca_threshold, tx_power;
    uint8_t eui[8];   /* Spinel order: most significant byte first */
    uint8_t laddr[8]; /* same */
    uint16_t saddr, panid;
} cfg;

/* ---- CRCs ---- */
static uint16_t crc16_kermit(const uint8_t *d, uint32_t n, uint16_t crc) {
    while (n--) {
        crc ^= *d++;
        for (int i = 0; i < 8; i++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    return crc;
}

/* ---- outgoing Spinel/HDLC queue ---- */
#define QN 6
#define QSIZE 400
#define CHUNK 240
static struct { uint16_t len, sent; uint8_t d[QSIZE]; } q[QN];
static uint8_t q_head, q_count;

static uint32_t hdlc_put(uint8_t *o, uint32_t n, uint8_t b) {
    if (b == 0x7e || b == 0x7d || b == 0x11 || b == 0x13 || b == 0xf8) {
        o[n++] = 0x7d; o[n++] = b ^ 0x20;
    } else o[n++] = b;
    return n;
}

static void send_frame(const uint8_t *f, uint32_t len) {
    if (q_count == QN || len > 180) return; /* host retries; never overruns */
    uint8_t *o = q[(q_head + q_count) % QN].d;
    uint32_t n = 0;
    uint16_t fcs = (uint16_t)~crc16_kermit(f, len, 0xffff);
    o[n++] = 0x7e;
    for (uint32_t i = 0; i < len; i++) n = hdlc_put(o, n, f[i]);
    n = hdlc_put(o, n, fcs & 0xff);
    n = hdlc_put(o, n, fcs >> 8);
    o[n++] = 0x7e;
    q[(q_head + q_count) % QN].len = (uint16_t)n;
    q[(q_head + q_count) % QN].sent = 0;
    q_count++;
}

void mac_flush(void) {
    while (q_count) {
        uint16_t left = q[q_head].len - q[q_head].sent;
        uint16_t n = left > CHUNK ? CHUNK : left;
        if (!tl_notify(q[q_head].d + q[q_head].sent, n)) return;
        q[q_head].sent += n;
        if (q[q_head].sent == q[q_head].len) { q_head = (q_head + 1) % QN; q_count--; }
        return; /* one chunk per ownership of the channel */
    }
}

/* ---- Spinel helpers ---- */
static uint32_t put_uint(uint8_t *o, uint32_t v) {
    uint32_t n = 0;
    do { uint8_t b = v & 0x7f; v >>= 7; o[n++] = b | (v ? 0x80 : 0); } while (v);
    return n;
}
static uint32_t get_uint(const uint8_t *i, uint32_t avail, uint32_t *v) {
    uint32_t r = 0, n = 0, sh = 0;
    while (n < avail && sh < 32) {
        uint8_t b = i[n++];
        r |= (uint32_t)(b & 0x7f) << sh; sh += 7;
        if (!(b & 0x80)) { *v = r; return n; }
    }
    return 0;
}

static void reply_status(uint8_t tid, uint32_t status) {
    uint8_t f[8]; uint32_t n = 0;
    f[n++] = 0x80 | tid; f[n++] = CMD_IS; n += put_uint(f + n, P_LAST_STATUS);
    n += put_uint(f + n, status);
    send_frame(f, n);
}

static void reply_value(uint8_t tid, uint32_t prop, const uint8_t *v, uint32_t vlen) {
    uint8_t f[24]; uint32_t n = 0;
    f[n++] = 0x80 | tid; f[n++] = CMD_IS; n += put_uint(f + n, prop);
    for (uint32_t i = 0; i < vlen; i++) f[n++] = v[i];
    send_frame(f, n);
}

/* ---- TX state machine (states change in IRQ context only toward TX_DONE) ---- */
enum { TX_IDLE, TX_BACKOFF, TX_ONAIR, TX_ACKWAIT, TX_DONE };
#define BACKOFF_TICKS 20u /* aUnitBackoffPeriod: 20 symbols of one tick */
#define ACK_WAIT_TICKS 54u /* macAckWaitDuration */
static struct {
    volatile uint8_t state;
    uint8_t tid, len, channel, max_nb, nb, be, seq;
    bool csma, wants_ack;
    uint32_t deadline;
    uint8_t mpdu[127];
    volatile uint8_t status;
    volatile bool got_ack, ack_pending;
    uint8_t ack[3]; int8_t ack_rssi;
} tx;
static volatile bool ack_in_flight;
static uint32_t rng = 0x2545f491u;

static uint32_t rnd(void) {
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
}

static void tx_finish(uint32_t status) {
    uint8_t f[64]; uint32_t n = 0;
    f[n++] = 0x80 | tx.tid; f[n++] = CMD_IS; n += put_uint(f + n, P_LAST_STATUS);
    n += put_uint(f + n, status);
    f[n++] = tx.got_ack ? tx.ack_pending : 0; /* frame pending */
    f[n++] = 0;                               /* header updated */
    uint32_t alen = tx.got_ack ? 5 : 0;       /* ACK + FCS */
    f[n++] = alen; f[n++] = 0;
    if (alen) {
        uint16_t fcs = crc16_kermit(tx.ack, 3, 0);
        for (int i = 0; i < 3; i++) f[n++] = tx.ack[i];
        f[n++] = fcs & 0xff; f[n++] = fcs >> 8;
    }
    f[n++] = (uint8_t)tx.ack_rssi;
    send_frame(f, n);
    tx.state = TX_IDLE;
}

static void tx_schedule_backoff(void) {
    uint32_t slots = rnd() & ((1u << tx.be) - 1u);
    tx.deadline = radio_ticks_add(radio_ticks(), slots * BACKOFF_TICKS);
    tx.state = TX_BACKOFF;
    if (radio_ticks_until(tx.deadline) >= 2) radio_alarm_set(tx.deadline);
}

/* Returns false when the radio is busy (our own ACK on air); caller retries. */
static bool tx_start_air(void) {
    bool ok;
    __asm volatile("cpsid i");
    ok = !ack_in_flight && radio_tx_load(tx.mpdu, tx.len);
    if (ok) {
        tx.state = TX_ONAIR;
        radio_tx_start(0);
    }
    __asm volatile("cpsie i");
    return ok;
}

static void handle_stream_raw(uint8_t tid, const uint8_t *v, uint32_t n) {
    if (!cfg.enabled) { reply_status(tid, S_INVALID_STATE); return; }
    if (tx.state != TX_IDLE) { reply_status(tid, S_BUSY); return; }
    if (n < 2) { reply_status(tid, S_INVALID_ARG); return; }
    uint32_t psdu = v[0] | (v[1] << 8);
    if (psdu < 5 || psdu > 129 || n < 2 + psdu + 6) { reply_status(tid, S_INVALID_ARG); return; }
    const uint8_t *p = v + 2 + psdu;
    if (p[0] < 11 || p[0] > 26) { reply_status(tid, S_INVALID_ARG); return; }
    tx.len = (uint8_t)(psdu - 2); /* the radio appends the FCS */
    for (uint32_t i = 0; i < tx.len; i++) tx.mpdu[i] = v[2 + i];
    tx.channel = p[0];
    tx.max_nb = p[1];
    tx.csma = p[3] != 0;
    tx.tid = tid;
    tx.seq = tx.mpdu[2];
    tx.wants_ack = (tx.mpdu[0] & 0x20) != 0;
    tx.got_ack = false;
    tx.ack_rssi = 127;
    if (tx.channel != cfg.channel) { cfg.channel = tx.channel; radio_set_channel(cfg.channel); }
    if (tx.csma) { tx.nb = 0; tx.be = 3; tx_schedule_backoff(); }
    else if (!tx_start_air()) { tx.nb = 0; tx.be = 3; tx_schedule_backoff(); }
}

/* ---- RX in IRQ context: filter, ACK, queue for Spinel ---- */
#define RXQ 4
static struct { uint8_t len; int8_t rssi; uint8_t d[127]; } rxq[RXQ];
static volatile uint8_t rxq_w, rxq_r;

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void send_ack_isr(uint8_t seq, bool pending, uint32_t end_ticks) {
    if (ack_in_flight || tx.state == TX_ONAIR || radio_tx_busy()) return;
    uint8_t a[3] = { (uint8_t)(0x02 | (pending ? 0x10 : 0)), 0, seq };
    /* aTurnaroundTime is 12 symbols (ticks); the delay counts from the command */
    uint32_t used = radio_ticks_add(radio_ticks(), RADIO_COUNTER_MOD - end_ticks);
    uint32_t delay = used < 11 ? 11 - used : 1;
    if (!radio_tx_load(a, 3)) return;
    ack_in_flight = true;
    radio_tx_start(delay);
}

void mac_rx_isr(const uint8_t *m, uint8_t len, int8_t rssi, uint32_t end_ticks) {
    if (len < 3 || !cfg.enabled) return;
    uint16_t fcf = rd16(m);
    unsigned type = fcf & 7, dmode = (fcf >> 10) & 3;
    if (type == 2) { /* ACK for our transmission */
        if (tx.state == TX_ACKWAIT && m[2] == tx.seq) {
            radio_ack_timer_clear();
            tx.got_ack = true;
            tx.ack_pending = (fcf & 0x10) != 0;
            tx.ack[0] = m[0]; tx.ack[1] = m[1]; tx.ack[2] = m[2];
            tx.ack_rssi = rssi;
            tx.status = S_OK;
            tx.state = TX_DONE;
        }
        return;
    }
    bool unicast_to_us = false, pass = cfg.promisc != 0;
    uint32_t o = 3;
    if (dmode == 2 || dmode == 3) {
        uint32_t alen = dmode == 2 ? 2 : 8;
        if (o + 2 + alen > len) return;
        uint16_t pan = rd16(m + o);
        bool pan_ok = pan == 0xffff || pan == cfg.panid;
        const uint8_t *a = m + o + 2;
        bool addr_ok;
        if (dmode == 2) {
            uint16_t s = rd16(a);
            bool ours = s == cfg.saddr && cfg.saddr != 0xfffe && cfg.saddr != 0xffff;
            addr_ok = s == 0xffff || ours;
            unicast_to_us = pan_ok && ours;
        } else {
            addr_ok = true;
            for (int i = 0; i < 8; i++) addr_ok &= a[i] == cfg.laddr[7 - i];
            unicast_to_us = pan_ok && addr_ok;
        }
        if (pan_ok && addr_ok) pass = true;
    } else if (type == 0) {
        pass = true; /* beacons carry no destination */
    }
    if (!pass) return;
    if (unicast_to_us && (fcf & 0x20)) {
        /* MAC command 0x04 is a Data Request: tell the child to keep listening,
         * as ST's RCP does while its source-match table is off. */
        bool data_req = false;
        if (type == 3) {
            uint32_t p = o + 2 + (dmode == 2 ? 2 : 8);
            unsigned smode = (fcf >> 14) & 3;
            if (smode && !(fcf & 0x40)) p += 2;
            p += smode == 2 ? 2 : smode == 3 ? 8 : 0;
            data_req = p < len && m[p] == 0x04;
        }
        send_ack_isr(m[2], data_req, end_ticks);
    }
    if (cfg.raw_stream && (uint8_t)(rxq_w - rxq_r) < RXQ) {
        uint8_t i = rxq_w % RXQ;
        rxq[i].len = len; rxq[i].rssi = rssi;
        for (uint8_t k = 0; k < len; k++) rxq[i].d[k] = m[k];
        rxq_w++;
    }
}

void mac_txdone_isr(void) {
    if (ack_in_flight) { ack_in_flight = false; return; }
    if (tx.state != TX_ONAIR) return;
    if (tx.wants_ack) {
        tx.state = TX_ACKWAIT;
        radio_ack_timer_set(radio_ticks_add(radio_ticks(), ACK_WAIT_TICKS));
    } else { tx.status = S_OK; tx.state = TX_DONE; }
}
void mac_txerror_isr(bool cca_failure) {
    ack_in_flight = false;
    if (tx.state == TX_ONAIR || tx.state == TX_ACKWAIT) {
        tx.status = cca_failure ? S_CCA_FAILURE : S_FAILURE;
        tx.state = TX_DONE;
    }
}
void mac_ack_timeout_isr(void) {
    if (tx.state == TX_ACKWAIT) { tx.status = S_NO_ACK; tx.state = TX_DONE; }
}
void mac_alarm_isr(void) { /* only wakes the main loop */ }

static void rx_forward(const uint8_t *m, uint8_t len, int8_t rssi) {
    uint8_t f[160]; uint32_t n = 0;
    uint16_t fcs = crc16_kermit(m, len, 0);
    int lq = rssi + 100;
    lq = lq < 0 ? 0 : lq > 97 ? 97 : lq;
    f[n++] = 0x80; f[n++] = CMD_IS; n += put_uint(f + n, P_STREAM_RAW);
    f[n++] = (len + 2) & 0xff; f[n++] = (len + 2) >> 8;
    for (uint32_t i = 0; i < len; i++) f[n++] = m[i];
    f[n++] = fcs & 0xff; f[n++] = fcs >> 8;
    f[n++] = (uint8_t)rssi; f[n++] = (uint8_t)-100; /* rssi, noise floor */
    f[n++] = 0; f[n++] = 0;                         /* flags: FCS good */
    f[n++] = 2; f[n++] = 0;                         /* PHY data length */
    f[n++] = cfg.channel; f[n++] = (uint8_t)(lq * 255 / 97);
    send_frame(f, n);
}

static void apply_phy(void) {
    radio_set_channel(cfg.channel);
    radio_rx(cfg.enabled);
}

/* ---- Spinel requests ---- */
static void spinel_handle(const uint8_t *f, uint32_t n) {
    if (n < 3 || (f[0] & 0xc0) != 0x80) return;
    uint8_t tid = f[0] & 0x0f, cmd = f[1];
    uint32_t prop, used = get_uint(f + 2, n - 2, &prop);
    if (!used || (cmd != CMD_GET && cmd != CMD_SET)) { if (used) reply_status(tid, 5); return; }
    const uint8_t *v = f + 2 + used; uint32_t vn = n - 2 - used;
    uint8_t b[8];

    if (cmd == CMD_SET && prop == P_STREAM_RAW) { handle_stream_raw(tid, v, vn); return; }
    if (cmd == CMD_SET) {
        switch (prop) {
        case P_PHY_ENABLED: if (!vn) goto bad; cfg.enabled = v[0] != 0; apply_phy(); break;
        case P_PHY_CHAN: if (!vn || v[0] < 11 || v[0] > 26) goto bad; cfg.channel = v[0]; apply_phy(); break;
        case P_CCA_THRESHOLD: if (!vn) goto bad; cfg.cca_threshold = (int8_t)v[0]; break;
        case P_TX_POWER: if (!vn) goto bad; cfg.tx_power = (int8_t)v[0]; radio_set_power(cfg.tx_power); break;
        case P_LADDR: if (vn < 8) goto bad; for (int i = 0; i < 8; i++) cfg.laddr[i] = v[i]; break;
        case P_SADDR: if (vn < 2) goto bad; cfg.saddr = rd16(v); break;
        case P_PANID: if (vn < 2) goto bad; cfg.panid = rd16(v); break;
        case P_RAW_STREAM: if (!vn) goto bad; cfg.raw_stream = v[0] != 0; break;
        case P_PROMISC: if (!vn) goto bad; cfg.promisc = v[0]; break;
        default: reply_status(tid, S_PROP_NOT_FOUND); return;
        }
    }
    switch (prop) { /* the answer to SET is the stored value, as for GET */
    case P_HWADDR: reply_value(tid, prop, cfg.eui, 8); break;
    case P_PHY_ENABLED: b[0] = cfg.enabled; reply_value(tid, prop, b, 1); break;
    case P_PHY_CHAN: b[0] = cfg.channel; reply_value(tid, prop, b, 1); break;
    case P_CCA_THRESHOLD: b[0] = (uint8_t)cfg.cca_threshold; reply_value(tid, prop, b, 1); break;
    case P_TX_POWER: b[0] = (uint8_t)cfg.tx_power; reply_value(tid, prop, b, 1); break;
    case P_RSSI: b[0] = (uint8_t)radio_rssi_now(); reply_value(tid, prop, b, 1); break;
    case P_LADDR: reply_value(tid, prop, cfg.laddr, 8); break;
    case P_SADDR: b[0] = cfg.saddr; b[1] = cfg.saddr >> 8; reply_value(tid, prop, b, 2); break;
    case P_PANID: b[0] = cfg.panid; b[1] = cfg.panid >> 8; reply_value(tid, prop, b, 2); break;
    case P_RAW_STREAM: b[0] = cfg.raw_stream; reply_value(tid, prop, b, 1); break;
    case P_PROMISC: b[0] = cfg.promisc; reply_value(tid, prop, b, 1); break;
    default: reply_status(tid, S_PROP_NOT_FOUND); break;
    }
    return;
bad:
    reply_status(tid, S_INVALID_ARG);
}

void spinel_rx_hdlc(const uint8_t *d, uint32_t len) {
    uint8_t raw[260]; uint32_t n = 0;
    bool in = false, esc = false;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t b = d[i];
        if (b == 0x7e) {
            if (in && n >= 5 && n <= sizeof raw) {
                uint16_t want = rd16(raw + n - 2);
                if ((uint16_t)~crc16_kermit(raw, n - 2, 0xffff) == want) {
                    spinel_handle(raw, n - 2);
                    return;
                }
            }
            in = true; esc = false; n = 0;
        } else if (in) {
            if (b == 0x7d) { esc = true; continue; }
            if (esc) { b ^= 0x20; esc = false; }
            if (n < sizeof raw) raw[n++] = b;
        }
    }
}

/* ---- scheduler ---- */
void mac_init(void) {
    radio_init();
    radio_eui64(cfg.eui);
    for (int i = 0; i < 8; i++) { cfg.laddr[i] = 0; rng ^= (uint32_t)cfg.eui[i] << ((i & 3) * 8); }
    cfg.saddr = 0xffff; cfg.panid = 0xffff; cfg.channel = 11; cfg.cca_threshold = -75;
    if (!rng) rng = 1;
}

void mac_poll(void) {
    while (rxq_r != rxq_w) {
        uint8_t i = rxq_r % RXQ;
        rx_forward(rxq[i].d, rxq[i].len, rxq[i].rssi);
        rxq_r++;
    }
    if (tx.state == TX_DONE) { radio_alarm_clear(); tx_finish(tx.status); }

    if (tx.state == TX_BACKOFF && radio_ticks_until(tx.deadline) == 0) {
        if (radio_tx_busy() || ack_in_flight) { /* our own ACK is on air */
            tx.deadline = radio_ticks_add(radio_ticks(), 2);
        } else if (radio_rssi_now() < cfg.cca_threshold) {
            if (!tx_start_air()) tx.deadline = radio_ticks_add(radio_ticks(), 2);
        } else if (++tx.nb > tx.max_nb) {
            tx_finish(S_CCA_FAILURE);
        } else {
            if (tx.be < 5) tx.be++;
            tx_schedule_backoff();
        }
    }
}

bool mac_busy(void) {
    return rxq_r != rxq_w || tx.state == TX_DONE ||
           (tx.state == TX_BACKOFF && radio_ticks_until(tx.deadline) == 0);
}
