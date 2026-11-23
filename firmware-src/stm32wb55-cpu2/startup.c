/* SPDX-License-Identifier: MIT */
#include "regs.h"
#include "radio.h"
#include "tl.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);

volatile uint32_t wake;       /* set by ISRs, consumed by the main loop */

void *memcpy(void *d, const void *s, unsigned n) {
    uint8_t *dd = d; const uint8_t *ss = s;
    while (n--) *dd++ = *ss++;
    return d;
}
void *memset(void *d, int c, unsigned n) {
    uint8_t *dd = d;
    while (n--) *dd++ = (uint8_t)c;
    return d;
}

void reset_handler(void) {
    uint32_t *s = &_sidata, *d = &_sdata;
    while (d < &_edata) *d++ = *s++;
    for (d = &_sbss; d < &_ebss;) *d++ = 0;
    main();
    for (;;) __asm volatile("wfi");
}

static void default_handler(void) { for (;;) {} }

static void ipcc_isr(void) { tl_isr(); wake = 1; }
static void radio_rx_irq(void) { radio_isr_rx(); wake = 1; }
static void radio_tx_irq(void) { radio_isr_tx(); wake = 1; }

__attribute__((section(".vectors"), used))
void (*const vectors[16 + 32])(void) = {
    (void (*)(void))&_estack, reset_handler, default_handler, default_handler,
    [11] = default_handler, [14] = default_handler, [15] = default_handler,
    [16 + IRQ_RADIO_RX] = radio_rx_irq,
    [16 + IRQ_RADIO_TX] = radio_tx_irq,
    [16 + IRQ_IPCC] = ipcc_isr,
};
