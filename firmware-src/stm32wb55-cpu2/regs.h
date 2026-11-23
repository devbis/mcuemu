/* SPDX-License-Identifier: MIT */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define RD32(a)    (*(volatile uint32_t *)(uintptr_t)(a))
#define WR32(a, v) (*(volatile uint32_t *)(uintptr_t)(a) = (v))

/* IPCC (RM0434): C2 side. Mask bits: 1 = masked. C2SCR[5:0] clears
 * C1TOC2SR flags, C2SCR[21:16] sets C2TOC1SR flags. */
#define IPCC_BASE   0x58000C00u
#define IPCC_C2CR   (IPCC_BASE + 0x10)
#define IPCC_C2MR   (IPCC_BASE + 0x14)
#define IPCC_C2SCR  (IPCC_BASE + 0x18)
#define IPCC_C1TOC2SR (IPCC_BASE + 0x0c)
#define IPCC_C2TOC1SR (IPCC_BASE + 0x1c)
#define IPCC_CR_RXOIE (1u << 0)
#define IPCC_CR_TXFIE (1u << 16)

#define FLASH_IPCCBR (0x58004000u + 0x3c)

/* Mailbox channels (1-based), as used by ST's CubeWB transport layer. */
#define CH_SYS_CMD      2 /* C1 -> C2 system command, response in place */
#define CH_THREAD_NOT   3 /* C2 -> C1 OT notification (Spinel chunks)   */
#define CH_CLI_CMD      5 /* C1 -> C2 Spinel/HDLC frame                 */
#define CH_BIT(ch)      (1u << ((ch) - 1))

#define NVIC_ISER   0xE000E100u
#define IRQ_RADIO_RX 8u
#define IRQ_RADIO_TX 22u
#define IRQ_IPCC    19u

#define REF_TABLE_BASE 0x20030000u
