/* SPDX-License-Identifier: MIT */
#pragma once
#include "regs.h"
void mac_init(void);
void mac_poll(void);   /* RX, address filter, auto-ACK, TX state machine */
void mac_flush(void);  /* pushes queued Spinel output to CPU1 */
bool mac_busy(void);   /* work left that does not wait for an interrupt */
