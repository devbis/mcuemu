# Minimal STM32WB55 CPU2 firmware (MIT)

A small, readable replacement for ST's encrypted Thread RCP image, for use in
the mcuemu `stm32wb55` model. It speaks the CPU1 side's contract used by
Zephyr's `drivers/ieee802154/ieee802154_stm32wb.c`; it contains no ST code.

**Status: builds (about 6.3 KB), not yet run in the emulator.** It drives the
real STM32WB55 802.15.4 registers (not an invented block), so it targets the
same emulator radio model as ST's own image. The register facts come from
`REGISTERS.md`, derived from the register accesses of ST's decompiled Thread RCP
(facts only, no ST code is used); many of them are marked INFERRED there.

## What it implements

- Mailbox (`tl.c`): reads the reference table at `0x20030000 + 4*IPCCDBA`, fills
  the wireless-firmware info (Thread MTD), sends the SHCI "ready" event, answers
  every system command with Command Complete / status 0, and exchanges
  HDLC-framed Spinel with CPU1: CLI channel 5 in (frame in the CLI command
  buffer, `plen` at +11, data at +12), channel 3 out (`[id][u32 len][data]`,
  chunks up to 240 bytes).
- Spinel RCP subset (`mac.c`): HWADDR, PHY_ENABLED, PHY_CHAN, CCA_THRESHOLD,
  TX_POWER, RSSI, LADDR/SADDR/PANID, RAW_STREAM, PROMISCUOUS, STREAM_RAW (TX
  and RX). Unknown properties answer `LAST_STATUS = PROP_NOT_FOUND`.
- MAC duties: address filter, automatic ACK 192 us after the frame (frame
  pending set for Data Request), CSMA-CA, ACK wait (864 us), `NO_ACK` /
  `CCA_FAILURE` statuses.

## Build

    make            # needs arm-none-eabi-gcc; produces build/stm32wb55-cpu2.bin

Link address `0x080E0000` (install with `offset: 0xE0000`), private RAM in
SRAM2b `0x20038000..0x2003FFFF`. Example config:

    cpu2_firmware:
      - { file: ../firmware-src/stm32wb55-cpu2/build/stm32wb55-cpu2.bin, offset: 0xE0000 }

## Radio (`radio.c`)

Follows `REGISTERS.md`: RF register bus at `0x6000000c` (SRAM2 descriptors),
power at `0x60000404`, MAC registers at `0x60001000 + 4*index` with set/clear
aliases at `+0x400` / `+0x800`, TX FIFO and command port at `0x600013fc`, RX FIFO
at `0x600013e8`, events E0/E1 (`0x600010b4/b8`) on NVIC IRQ 8 (RX) and IRQ 22
(TX, compares), 24-bit counter at 16 us per tick. Hardware FCS is enabled,
address filtering and ACKs are done in software (`mac.c`, in IRQ context).

Assumptions the emulator model has to agree with (all uncertain):

- RSSI registers hold `dBm + 100` in bits 11:4 (`RADIO_RSSI_OFFSET`; ST's value is unknown).
- The EUI-64 is the byte-reversed UID64 at `0x1FFF7580..0x1FFF7587` (non-zero).
- A delayed TX (`R3f8` delay, opcode 6) counts 16 us ticks from the command and
  the radio returns to RX after a TX; the ACK is queued with delay 11 minus the
  ticks already spent after the frame end.
- Clearing/setting aliases work as in REGISTERS.md section 1; opcode `0x16` pushes
  the PHR (length including FCS) first.
- `RCC+0x94` bit16 reads 0, `RCC_C2APB3ENR` bits 0/1 are writable, the RF bus
  retains `0xa5a5` once `0x60000404` bit0 is set.

The init sequence uses bounded spin loops, so a missing radio model slows the
boot but does not hang it.
