/* SPDX-License-Identifier: MIT */
#pragma once
#include "regs.h"

extern volatile uint32_t wake;

void tl_init(void);
void tl_isr(void);
void tl_poll(void);
/* Sends one Spinel/HDLC chunk (<= 240 bytes) to CPU1; false while the
 * notification channel is still owned by CPU1 (asks for a wake-up then). */
bool tl_notify(const uint8_t *data, uint32_t len);
/* Spinel/HDLC bytes received from CPU1 (implemented in mac.c). */
void spinel_rx_hdlc(const uint8_t *data, uint32_t len);
