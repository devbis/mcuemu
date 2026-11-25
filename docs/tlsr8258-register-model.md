# TLSR8258 register model and evidence

This document records what the emulator currently knows about the TLSR8258
registers used by its firmware scenarios. It separates documented behavior
from SDK behavior and emulator approximations so that an experiment does not
silently become a hardware fact.

## Evidence labels

| Label | Meaning |
| --- | --- |
| `DS` | Specified in the Telink TLSR8258 datasheet. |
| `SDK` | Named or exercised by Telink SDK code/documentation. This may describe a supported sequence without specifying the silicon implementation. |
| `TRACE` | Observed in a firmware or board trace. Record the image and scenario; do not generalize to all firmware without another check. |
| `INFERRED` | Emulator behavior inferred from firmware behavior or adjacent documentation. |
| `SIMPLIFIED` | A deliberate abstraction that is not cycle-accurate or register-complete. |

## Sleep, wake, and clocks

| Register | Current interpretation | Evidence and model status |
| --- | --- | --- |
| AREG `0x05` | Bit 0 powers down the 32 kHz RC oscillator; bit 1 powers down the 32 kHz crystal. Reset values are 0 and 1 respectively. | `DS`; model uses these bits to decide whether its selected 32 kHz clock advances. |
| AREG `0x26` | Bit 6 enables wake from the 32 kHz timer. | `DS`; represented by `AnalogWakeSource` / `AnalogWakeTimerEnable`. |
| AREG `0x44` | Bits 0–3 report the wake cause; a set status bit is cleared by writing 1. Bit 1 is the 32 kHz timer status. | `DS`; timer status is modeled as write-one-to-clear. This is a status register, not the wake-enable register. |
| AREG `0x40..0x43` | Firmware-visible 32-bit low-frequency tick counter. | `SDK`/`TRACE`; modeled from virtual time using the selected 32 kHz source. RC frequency is configurable; crystal uses 32768 Hz. |
| Digital `0x800740..0x800743` | 16 MHz system timer. Its low three bits are invalid, giving 0.5 us resolution. | `DS`; modeled. |
| Digital `0x80074c` | System timer control; bit 7 enables 32 kHz calibration, bit 1 the compare IRQ, bit 4 (reset value 1) selects the 16 MHz clock of the counter. | `DS` for bits 7 and 1; bit 4 `TRACE` (board): the counter at `0x800740` advances only while started and with bit 4 set. Modeled; the 32 kHz counter is unaffected. |
| Digital `0x80074f` | System timer command/control strobes, including 32 kHz calibration and timer setup. | `DS`/`SDK`; the model clears command strobes synchronously. Real BUSY timing is not modeled yet. Bit 0 START and bit 1 STOP start and stop the counter (it runs after reset); a read returns bit 1 RUNNING while it advances, which the Zephyr driver polls after STOP (`TRACE`; a counter stopped by START with the clock deselected read `0xf4` on a TB03F). |
| Digital `0x800750..0x800751` | Calibration value produced by the 32 kHz measurement. | `SDK`/`TRACE`; the model emits a configured value after 16 low-frequency ticks. It is not an analog circuit simulation. |
| Digital `0x800754..0x800757` | 32 kHz wake compare used by the tested firmware. | `SDK`/`TRACE`; compare and wake are modeled. Verify against the applicable SDK/chip revision before treating the address as universal. |
| Digital `0x80006f` | Firmware writes `0x81` to request sleep. | `TRACE`; the exact sleep-entry delay is not specified by the register write alone. In the tested `count_power_c.bin`, the path reaches the `start_suspend + 0x58` boundary after 40 16-bit delay instructions. The emulator currently approximates this with 120 core cycles, using its estimated three cycles per instruction. |
| AREG `0x7e`, `0x7f` | Sleep-mode selection and PM wake/cold-boot status used by the SDK path. | `SDK`/`INFERRED`; these meanings and reset/retention details should be checked per wake mode. |
| AREG `0x2c` (with `0x2b`) | Power-down and wake gate written by the vendor sleep routine: base `0x96` (suspend), `0xd6` (deep), `0x56` (deep retention); bits 7/6 follow the mode. A timer wake is enabled by bit 0 in suspend (`0x97`) and by bit 3 in deep sleep (`0xde`, retention `0x5e`). | `SDK`/`INFERRED`/`TRACE`; from the reconstructed vendor routine and a board result: deep sleep with `0xd7` never woke. `wake_gate_open()` models only that; other bit combinations still wake. |

The datasheet identifies AREG `0x26[6]` as the 32 kHz timer wake enable and
AREG `0x44[1]` as the timer wake status. An older local register cheatsheet
labels `0x44` as the wake source; use the datasheet distinction above.

## Analog-register access path

The MCU accesses AREGs through digital registers `0x8000b8` (address),
`0x8000b9` (data), and `0x8000ba` (control). The model stores the 256-byte
analog map and gives special behavior to the clock, wake, tick, and calibration
registers. `mcuemu run` and `network` can decode each completed transfer with
`--trace-analog`; each event includes virtual time, direction, AREG address,
and transferred value. The event buffer is bounded by `--trace-limit`, and in
`network` it follows the `--trace-node` filter. AREG transfers still complete
synchronously and BUSY always reads clear (`SIMPLIFIED`), so this trace records
the model's transaction boundary and does not establish real BUSY timing.

## Radio and DMA

The current RF bridge is a packet-level Zigbee model. It does not simulate the
analog RF chain or decode every RF/baseband register.

| Register group | Current model behavior | Evidence and limitation |
| --- | --- | --- |
| `0x800f02` | Bits 4 and 5 gate the modeled TX and RX paths. | Firmware-derived; keep as `INFERRED` until validated against more RF modes. |
| `0x800c08`, `0x800c42` | RX DMA buffer address, including the high-byte encoding. | Exercised by the Zigbee firmware and board-model tests. |
| `0x800c0c`, `0x800c43`, `0x800c24` | TX DMA buffer and transmit-ready trigger. | Firmware-derived; starts a packet transmission on the virtual medium. |
| `0x800c20[2]`, `0x800428[0]` | RX DMA and receiver enable gates. | Firmware-derived; packet reception is delivered only when both are enabled. |
| `0x800f1c..0x800f21` | RF interrupt mask/status; modeled RX/TX completion bits and IRQ propagation. | Tested at the board-model level; not a complete RF interrupt model. |
| `0x801244..0x801245` | Synthesizer frequency writes are decoded to a Zigbee channel. | Firmware-observed mapping. Other modulation modes are not implemented through these registers. |

The SDK guide documents supported RF modes and API sequences, but that does
not establish the undocumented register-level semantics of every mode. The
emulator currently maps the tested Zigbee TX/RX DMA sequence to packets on
`VirtualAir`; BLE is not connected to these TLSR8258 RF registers.

## GPIO pads, I2C master and SAR ADC

These were added for the system simulation (see
[system-config.md](system-config.md)); evidence is the datasheet (sections
7.3.4 and 11) and the Telink SDK drivers (`drivers/B85/i2c.c`,
`platform/chip_8258/adc.c`, `adc.h`, `dfifo.h`, `gpio.c`).

| Registers | Model | Evidence |
|---|---|---|
| GPIO `base+4` (polarity), `base+7` (pin interrupt enable), `0x5b5[3]` (core interrupt enable) | A pin whose source is enabled and whose level is its active one (polarity 0: high, 1: low) latches IRQ 18 (source byte `0x64a`, bit 2) when it reaches that level, each pin on its own, so a press is not lost while another pin is active. Firmware clears the source by writing one to `0x648..0x64a`. The pin source does not latch by itself while the level only stays. Wake from sleep by GPIO is not modelled. | SDK GPIO IRQ and the Zephyr `gpio_tlsr8258` driver; inferred, not measured. |
| GPIO `base+2` (OEN), `base+3` (OUT), `base+6` (ACT_AS_GPIO) | Pad observers report changes. OEN resets to `0xff` and ACT_AS_GPIO to `0xff` (all pads start as GPIO inputs). A pad whose ACT_AS_GPIO bit is 0 is not driven by OUT/OEN. | Datasheet reset values; SDK `gpio_set_func`. |
| AREG `0x0e..0x17` | Internal pull resistors, two bits per pad: 1 pull-up 1M, 2 pull-down 100K, 3 pull-up 10K. Modeled as weak pull-up/down on the pad's net. | SDK `gpio_setup_up_down_resistor`. |
| `0x00..0x07` | I2C master: speed `0x00`, id `0x01`, status `0x02` (CMD_BUSY, BUS_BUSY, NAK), mode `0x03[1]` master enable, ADR/DO/DI `0x04..0x06`, command `0x07` (ID, ADDR, DO, DI, START, STOP, READ_ID, ACK). Phases run START, ID, ADR, DO, DI, STOP; each bit takes `4 * speed` cycles. Pads follow the pair whose ACT_AS_GPIO bits are cleared: A3/A4, B6/D7, C0/C1 or C2/C3. | Datasheet 7.3.4 and SDK I2C driver. |
| `0x96[4]`, `0x96[5]`, `0xc00..0xc03`, `0xc40`, `0xc04..0xc06`, `0xc41`, `0xc20`, `0xc21`, `0xc24[1]`, `0xc26` | UART DMA (the SDK's `drv_uart`): with `0x96[4]` and DMA channel 0 enabled, received bytes go to the buffer at `0xc00` (high byte `0xc40`, size `0xc02 * 16`) as `u32 length + payload`; the packet is closed 12 bit times (times `0x9b[1:0]+1`) after the last byte, or when the buffer is full, and sets `0xc26[0]`. A write of `1` to `0xc24[1]` sends the `u32 length + payload` buffer at `0xc04` (high byte `0xc41`) through the UART transmitter; completion sets `0xc26[1]`. `0xc26` is write-one-to-clear; a set bit in `0xc26 & 0xc21` raises IRQ 4 (`FLD_IRQ_DMA_EN`). | Disassembly of the SDK's `uart_recbuff_init`, `uart_dma_send`, `uart_dma_enable` and `irq_handler` (a Tuya thermostat firmware). The idle-timeout formula is inferred; a longer or shorter timeout only shifts when the packet is delivered. |
| AREG `0xe7..0xfc`, `0xb08..0xb0b`, `0xb10`, `0xb1e` | SAR ADC misc channel into DFIFO2: sample period `(R_max_mc + R_max_s) / 24 MHz`, P/N input selectors, reference, resolution and 1/2^n input prescaler; samples are written to the SRAM ring buffer while DFIFO2 input is enabled. Positive selector `0xf` is VBAT (the supply, `supply_mv` of the MCU, default 3300 mV; negative `0xf` is GND). A pad the chip drives itself reads as the supply (high) or 0 mV (low), which is how the SDK's VBAT mode measures the supply. | Datasheet section 11 and SDK `adc.c`. The sample period reproduces the SDK's 23/96/192 kHz rates. |

Inferred, not documented: the I2C `ACK` command bit makes the master release
SDA (NACK) after a received byte, as the SDK sets it on the last byte; `NAK`
reports a missing slave ACK on any written byte of the command and the
remaining phases still run; clock stretching is not modeled. ADC samples are
stored as 32-bit entries, the code in the low 16 bits and the DFIFO2 write
pointer moving by 4 (the ATC firmware of the LYWSD03MMC waits for the pointer
to move and then reads the buffer as 32-bit words; with 2-byte entries it
read words that were not written yet and took a full battery for an empty one;
the upper half of an entry is written as 0, its content is not known); the 1.2 V reference is `1175 mV` like the SDK's default calibration;
left/right channels, PGA and the temperature sensor are not modeled. The I2C
slave mode and SPI master/slave are not modeled.

## BLE mode of the RF

With `protocols: [ble]` on the MCU's radio the link layer commands send and receive BLE packets. The
register protocol is that of the vendor drivers (`rf_drv.h` of the SDK; `rf_start_stx2rx()`,
`rf_start_brx()`, `rf_start_stx()` and `rf_set_ble_channel()` of the ATC firmware's binary library were
read from its ELF, `irq_blc_ll_rx` and `cpu_sleep_wakeup_32k_rc` by disassembling it):

| Register | Meaning in the model |
|----------|----------------------|
| `0xf00` | Command: `0x85` STX, `0x87` STX2RX, `0x88` SRX2TX, `0x86` SRX, `0x82` BRX; `0x80` stops a start that is pending or a receive window |
| `0xf16` bit 2, `0xf18` (32 bit) | With the bit set the command starts when the 16 MHz system timer reaches the tick; the start is at least 5 us after the command |
| `0xc0c` (16 bit) | Address of the packet to transmit, read when the transmit starts (the firmware writes it after the command). The high byte is `0xc43`, 0 meaning the SRAM at `0x840000` |
| `0xc08` (16 bit) | Address of the receive buffer; the high byte is `0xc42` |
| `0x40d`, `0x408` | Channel number 0..39 (`rf_set_ble_channel()` also writes the frequency index to `0x1228`) and the access address, byte-swapped |
| `0x424` | CRC init, 24 bits: the bytes of CRCInit of a CONNECT_IND for a connection, `0x555555` for advertising |
| `0xf03`, `0xf0a`, `0xf28` | Timeout enables, the SRX/STX2RX timeout (us) and the first timeout of BRX (us): the window ends without a packet only when no packet has begun |
| `0xf20` | Interrupt flags: bit 0 RX done, bit 1 TX done, bit 2 RX timeout, bit 10 (`0xf21` bit 2) first timeout |
| `0x448` bit 5 | A packet is being received; set 56 us after the start of the packet, when the header is decoded |

A packet that starts in an open window, on the channel and with the access address the radio is set to,
is written to the receive buffer in the vendor format: the 4-byte DMA length (payload + 13), the PDU header,
the payload length, the payload, the CRC (3 bytes), the time stamp of the start of the packet (4 bytes,
16 MHz system timer), the frequency offset (2), the RSSI + 110 and a status byte (bit 0 set: CRC error).
The stack checks the CRC of every packet in software, so the radio writes the real one: the reflected
CRC-24 of the PDU with `0x424` bit-reversed as the seed (`ble_radio_crc24()`), the bytes of the state,
low first. A CRC that does not match makes the stack drop every packet of the connection and time it out
after the supervision timeout. The length and the header are in the buffer when the header is decoded,
the rest and the RX flag come with the last bit.

BRX (the slave side of a connection event) and STX2RX answer by themselves, 150 us (T_IFS) after the end
of the received packet, with the packet that DMA channel 3 points to at that moment; for BRX the radio
keeps SN and NESN of the slave side and puts them into the header of the answer (the firmware only
sets the LLID). A packet that arrives after the window ended, or on another channel or access address, is
not received.

The 32 kHz counter is read through the same address as the wake compare value: `0x74c = 0x28` and the
command `0x08` in `0x74f` latch the counter into `0x754` (read back by `cpu_get_32k_tick()`), while
`0x74c = 0x2c` makes the writes to `0x754` the compare value and the command arms the wakeup. Before
the read mode existed the model returned the compare value of the last sleep as the current tick, the
vendor sleep code (`cpu_sleep_wakeup_32k_rc`) computed the wakeup of the next connection event from it
and asked for a wakeup in the past, so the firmware never woke up.

The vendor stack hands data to the radio through a queue of 16 packet addresses (read from
`blt_push_fifo_hw`, `blt_brx_start` and `blt_brx_post` of the ATC firmware). A write to `0xc2c` (16 bit,
high byte `0xc43`) pushes an address and moves the write pointer, read back at `0xc2b`. The read pointer
is `0xc2a`; the stack restores it at the start of each event by writing the value it read at the end
of the last one with bit 6 set (bit 4 resets both pointers). The answer of a BRX event is the entry at
the read pointer when the queue is not empty (else the packet of `0xc0c`), and the pointer moves on
when the next packet of the master acknowledges it. The stack frees its own software queue when it
pushes, so a data packet is pushed once, behind an empty PDU, and goes out in the event after the one
that acknowledged the empty one; if the master does not acknowledge it, it is repeated.

SN and NESN of the slave side persist between events through the stack: at BRX start the radio loads
them from bits 4 (SN) and 5 (NESN) of `0xf03`, and after a received packet it shows them in bit 0 of
`0xf22` (SN) and bit 4 of `0xf23` (NESN), which the stack copies to `0xf03` for the next event. The
assignment of the bits follows from the stack's copy and from the result (a response that is neither
lost nor repeated); there is no hardware trace.

Not modeled: encryption, whitening (the air adds the framing), the radio state machine of the BRX events,
window widening, and the transmit power. The Zigbee path is unchanged and is used when no `ble`
protocol is configured.

## Next accuracy steps

1. Compare the decoded AREG sequence and virtual timestamps against a captured
   hardware trace; use that evidence to decide whether BUSY duration needs a
   time-based model.
2. Add reset/retention tests for each sleep mode and compare register state
   before sleep, at wake, and after startup.
3. For RF, create small deterministic firmware probes for one TX, one RX, and
   one ACK exchange. Compare register/DMA/IRQ traces, then add only the
   observed side effects needed by those sequences.
4. Keep radio internals abstract at the packet level unless a firmware test
   depends on a lower-level behavior such as CCA, retry timing, ACK timing, or
   receive filtering.

## References

- [Telink TLSR8258 datasheet](https://wiki.telink-semi.cn/doc/ds/DS_TLSR8258-E_Datasheet%20for%20Telink%20BLE%2BIEEE802.15.4%20Multi-Standard%20Wireless%20SoC%20TLSR8258.pdf), especially sections 2.6–2.7 and 5.3.
- [Telink TLSR8258F512 SDK core features](https://wiki.telink-semi.cn/tools_and_sdk/Driver/doc/kite/html/md__project_1__table_of__content_03__t_s_i__core__features.html).
- [Telink TLSR8258F512 SDK RF features](https://wiki.telink-semi.cn/tools_and_sdk/Driver/doc/kite/html/md__project_1__table_of__content_14__t_s_i__r_f__features.html).
- Emulator implementation: [`tlsr8258.hpp`](../emulator/include/mcuemu/tlsr8258.hpp), [`tlsr8258.cpp`](../emulator/src/tlsr8258.cpp).
- Relevant tests: [`test_tlsr8258.cpp`](../tests/machine/test_tlsr8258.cpp), [`test_cpu.cpp`](../tests/tc32/test_cpu.cpp).
