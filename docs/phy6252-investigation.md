# PHY6252 investigation

The first target is the Ai-Thinker PB-03F and the experimental Zephyr port in
the Telink TC32/ARM tree. The model is experimental and its register behavior
has not been checked on silicon.

## Image and memory map

The package format starts at physical flash offset `0x2000`. A full flash dump
starts at offset zero and has the table at `0x2000`; both forms are accepted.
Each table entry describes either an XIP segment or an SRAM copy, and the run
descriptor supplies the initial stack pointer and Thumb reset address. The
packer writes `0xffffffff` in CRC fields, which is not evidence that the ROM
checks them. `scripts/inspect-phy6252-image.py IMAGE` validates the table,
segment bounds, XIP destinations and run descriptor; pass `--full-flash` to
inspect a full dump.

The PHY6252 model has 512 KiB of XIP flash at `0x11000000` and 64 KiB of SRAM
at `0x1fff0000`. The Zephyr board reserves the region below `0x1fff1840` and
the final 4 KiB; its usable SRAM is `0x1fff1840` for `0xe7c0` bytes. The model
starts at the package's RAM run descriptor. The Zephyr port relocates its
vector table to address zero; the model maps a 96 KiB ROM window at zero and
`0x10000000`, and permits writes to the first `0x100` bytes of the low window
as the vector alias. This alias is inferred from `relocate_vector_table()` and
has not been hardware-verified.

The supplied TS0001 full flash dump is 512 KiB. Its table describes XIP
segments at physical offsets `0x10000` (3288 bytes) and `0xa000` (24576 bytes),
plus SRAM copies from `0x5000`, `0x5414`, `0x98e0` and `0x996c`. Its run
descriptor is at `0x1fff1838`, with initial SP `0x1fffff00` and reset vector
`0x1fff193d`. The sample UART log reports NVM at `0x11079000`, size `0x6000`.
These are observations from the supplied image and log, not hardware-validated
memory-map guarantees.

## Peripheral model

UART0 supports polling byte input/output, without interrupt or bit timing.
GPIO0..GPIO22 support data, direction, input, AON pulls and basic edge/level
interrupts; alternate pad functions are not modeled. Timer5 and Timer6 use the
observed DesignWare register layout, use four core cycles per tick (4 MHz on
the Zephyr path at 16 MHz), and provide down-counting, one-shot alarms and IRQs
24/25. The AON RTC counter advances at 32.768 kHz when `RTCCTL.RUN` is set;
direct image boot starts it in the state inferred for the skipped ROM board-init
path. The AON RTC clock-select, control, compare and flag registers are stored,
but compare events and IRQ generation are not. The watchdog uses the SDK's
2–256 second timeout selections, feed key and IRQ 10;
its clock source and reset details are inferred. PCR software reset and clock
registers, including SW_CLK1 and APB_CLK, retain writes; clock gating and reset
effects are not enforced. AP_CACHE and the observed AP_SPIF registers are
write-backed. DMA channel 3 supports the observed
software-triggered memory copy used with SPIF remapping; other channels and
request sources are not modeled. The ADC and a complete clock tree are not
implemented.

The experimental Zephyr radio driver uses RF registers at `0x40030000`, link
layer registers at `0x40031000`, and TX/RX FIFOs at `0x40031400` and
`0x40031c00`. The model handles its observed RF/LL register set, adds or strips
the two-byte FCS boundary used by the driver, schedules TX completion from
802.15.4 airtime, raises IRQ 4, and exchanges MAC payloads with `sim`'s shared
radio medium. RX is polled every 16 us while enabled. CCA energy is inferred
from frames on the selected channel over the driver's 128 us measurement
window. The executed Zephyr driver performs MAC address filtering in its RX
handler and sends ACKs by writing an ACK frame to the TX FIFO after its
192-us turnaround wait; the emulator supplies the FIFO, IRQ and timed TX path
used by that code. In the 10-second network run, the pcap contained 47 MAC ACK
frames; 46 of 47 ACK-request frames had a same-sequence ACK within 3 ms. The
measured ACK starts were 186–220 us after the preceding data PPDU ended. The
medium carries frames without FCS, and the model does not compute or validate
the FIFO's FCS bytes. Register values and timing come from the experimental
Zephyr driver, adapted from an unverified PHY6222 implementation; they remain
unverified on silicon.

## ROM calls and known gaps

Implemented ROM entries include clock setup/query, IRQ critical-section
helpers, eFuse read, AES-128 block encryption, OSAL heap and memory functions,
`log_printf`, the ARM runtime memory/string functions,
`__ARM_common_switch8`, `spif_config`, `spif_cmd`, `spif_rddata`, and the
flash write/erase entries used by the Zephyr port. SPIF command completion is
synchronous; status reads report WIP clear. Flash timing, write protection,
page limits and most opcodes are not modeled. Invalid arguments return
`-EINVAL`.

The eFuse shim covers only block 1 and supplies RF cap bytes `0x2b,0x23` from
the user's sample log, with zeros for the remaining six bytes. The current
TS0001 UART still prints `tpCap[00 00]`, so the actual read path or data layout
is not yet matched; the remaining calibration bytes have not been recovered.

Two calls are deliberately visible limitations. The image reaches unexported
ROM entry `0xa2e0`, inside the SDK symbol range for `ble_crc24_gen`; its body is
not present in the SDK package, so the current return-only approximation logs
a warning. `LL_InitConnectContext` returns success for the unused BLE context
and logs that BLE connection state is not modeled. Neither behavior establishes
BLE or Zigbee stack compatibility.

The TS0001 boot path calls the exported ROM symbol `phy_sec_encrypt`, entry
`0x15338`, with `r0=0x1fff9440`, `r1=0x11002830`, source `r2=0x11020000`,
length `r3=0x4000`, and stack arguments `0` and `0x1ffff6d4`. The expected
16-bit MIC at the caller's record is `0xcf3f`. CRC-16/ARC over those 16 KiB
matches it exactly. The same candidate matches the record values for each
subsequent boot block reached by the image. The model therefore has a narrowly
gated inferred path for this argument shape: it writes the CRC-16/ARC result to
the supplied output pointer, where the caller still compares it with the
recorded MIC. Other argument shapes remain visible unimplemented ROM calls.
This evidence identifies the boot-check operation used by this image; it does
not establish the semantics of `phy_sec_encrypt` as a general SDK API.

The SDK package provides the symbol name for `phy_sec_decrypt` at entry
`0x1531c`, but not its ROM implementation or a prototype. For the calls observed
in this image, the emulator computes CRC-16/ARC over the source bytes, compares
it with the record's 16-bit MIC, and copies the bytes unchanged to the supplied
SRAM destination only when they match. Three observed blocks pass this check.
This is an image-specific inferred compatibility path; it does not establish
that the general SDK function uses CRC-16/ARC or that other call shapes are
supported. The destination can contain executable code, so the model invalidates
the Cortex-M translated-code cache after each successful copy.

The image also reaches the SDK's `__aeabi_uidiv` and `__aeabi_idiv` ROM helpers,
which are modeled with quotient in `r0` and remainder in `r1`; division by zero
faults. The observed `rtc_get_counter` call returns the model's AON RTC count.
Several observed `ll_hw_*` BLE setup/FIFO calls are logged no-ops, and
`ll_hw_get_tr_mode` returns idle. They do not model BLE state or radio effects.
The SDK's `gpio_sel[pin / 4]` accesses are served from the inferred register
window `0x40003818..0x4000382c`; AON wake-source reads at `0x4000f0b0` and
`0x4000f0b4` return zero without generating wake events. These are code-driven
compatibility observations, not hardware-verified peripheral contracts.

The TS0001 image fills an LL extension/table area at `0x40031e00..0x40031ffc`,
inside the modeled 2 KiB RX FIFO range. Its RF effect is unknown. After the
300 ms RF startup wait, the current run reaches PCR setup and stops when
exception entry attempts a stack write at `0x400313e0`, with `SP=0x40031400`,
`PC=0x1fffdd00`, and `LR=0xfffffff1`. The write is below the 2 KiB TX FIFO
starting at `0x40031400` and is not mapped as memory. The model reads VTOR as
`0x1fff8b00`, while the SDK's PHY6222 device header declares no VTOR; the exact
exception source and stack setup remain unresolved. The run has not printed
`Stack Init`, has not started Zigbee radio traffic, and has not joined a
coordinator. The user-supplied device log reaches `Stack Init`, restores network
state 12 and emits Tuya reports, but that is evidence about the hardware image,
not this emulator run.

## Firmware evidence

The PB-03F Zephyr `zigbee_shell` end-device image was run for 10 s of virtual
time with the TLSR8258 sample gateway. The UART reported
`zigbee_shell joined network (bdb status: 0x00)` at 9.858 s. The pcap checker
saw association at 7.80 s, 15 uplink NWK frames (11 unicast), and 8 downlink
NWK frames. This is an emulator-to-emulator result, not a hardware comparison.
The runnable system is `examples/phy6252-zephyr.yaml`; its image remains a
local input under `firmware/`.

The user-supplied TS0001 log shows startup through `Stack Init`,
`Zigbe stack initialized`, successful restoration of network state 12 and Tuya
reports. The emulator passes three observed CRC-checked copy blocks and prints
configuration through `rfClk`; it reports `tpCap[00 00]` instead of the
reference log's `tpCap[2b 23]`, omits `OS NVM Start Addr` and `Stack Init`, then
stops on the exception stack write described above. The image's runtime
Zigbee-stack location and its ability to join or exchange frames therefore
remain undetermined. Do not claim TS0001 relay support until it starts, joins,
and exchanges Zigbee traffic in a captured network scenario.

Run an image with `mcuemu run --model PHY6252 --firmware IMAGE --for 1s` or use
the YAML model name `phy6252` in `sim`. `--trace-pc-range START:END` enables
instruction tracing for a PC interval and prints registers on a fault. UART0 is
exposed as `mcu.uart0`. Firmware inputs can be the package beginning at physical
flash offset `0x2000` or a 512 KiB full flash dump; keep local images outside
the repository.
