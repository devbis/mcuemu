/* SPDX-License-Identifier: MIT */
/* Driver for the STM32WB55 CPU2 802.15.4 radio, written from the register facts
 * recorded in REGISTERS.md (derived from observed register accesses, not from
 * ST documentation: parts are inferred, see the tags there). */
#pragma once
#include "regs.h"

#define RADIO_TICK_US     16u         /* one symbol */
#define RADIO_COUNTER_MOD 0xab322eu   /* counter period, in ticks */
#define RADIO_RSSI_OFFSET 100         /* ASSUMPTION: dBm = raw - 100 (ST's value is unknown) */

bool radio_init(void);
bool radio_set_channel(uint8_t channel);   /* 11..26 */
bool radio_set_power(int8_t dbm);          /* -21..6 */
void radio_rx(bool on);
bool radio_rx_is_on(void);
int8_t radio_rssi_now(void);               /* dBm, instantaneous */
void radio_eui64(uint8_t out[8]);          /* most significant byte first */

uint32_t radio_ticks(void);
/* Ticks left until `at` (0 when reached or just passed). */
uint32_t radio_ticks_until(uint32_t at);
uint32_t radio_ticks_add(uint32_t t, uint32_t d);
void radio_delay_ticks(uint32_t n);

bool radio_tx_busy(void);                  /* sequencer busy or TX FIFO full */
/* Loads PHR + MPDU (without FCS: the hardware appends it) into the TX FIFO. */
bool radio_tx_load(const uint8_t *mpdu, uint8_t len);
void radio_tx_start(uint32_t delay_ticks); /* 0 = now */

void radio_alarm_set(uint32_t at);         /* compare C3 -> mac_alarm_isr() */
void radio_alarm_clear(void);
void radio_ack_timer_set(uint32_t at);     /* compare C0 -> mac_ack_timeout_isr() */
void radio_ack_timer_clear(void);

void radio_isr_rx(void);                   /* NVIC IRQ 8 */
void radio_isr_tx(void);                   /* NVIC IRQ 22 */

/* Implemented by the MAC layer; called from interrupt context. */
void mac_rx_isr(const uint8_t *mpdu, uint8_t len, int8_t rssi, uint32_t end_ticks);
void mac_txdone_isr(void);
void mac_txerror_isr(bool cca_failure);
void mac_ack_timeout_isr(void);
void mac_alarm_isr(void);
