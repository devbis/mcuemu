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
interrupts; alternate pad functions are not modeled. Timer1..Timer6 share the
SDK's AP_TIM register layout, use four core cycles per tick, and provide
down-counting, one-shot alarms and IRQs 20..25. The 4 MHz rate is observed in
the SDK Timer5/6 driver and applied to Timer1..Timer4 by inference. Their
current-count registers are writable, as required by the SDK timer stop path.
The AON RTC counter advances at 32.768 kHz when `RTCCTL.RUN` is set;
direct image boot starts it in the state inferred for the skipped ROM board-init
path. The AON RTC clock-select, control, compare and flag registers are stored,
but compare events and IRQ generation are not. The four retained AON sleep
words at `0x4000f0c0..0x4000f0cc` are read/write storage, as indicated by the
SDK's sleep/tracking fields. The watchdog uses the SDK's
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
used by that code. On RX, the model computes and appends the little-endian
CRC-16/KERMIT FCS to the FIFO bytes, matching the reported length; the medium
continues to carry MAC payloads without an FCS. Outgoing FIFO FCS bytes are
stripped at that boundary and are not validated, and the medium does not model
corrupted frames. In the 10-second network run, the pcap contained 47 MAC ACK
frames; 46 of 47 ACK-request frames had a same-sequence ACK within 3 ms. The
measured ACK starts were 186–220 us after the preceding data PPDU ended.
Register values and timing come from the experimental Zephyr driver, adapted
from an unverified PHY6222 implementation; they remain unverified on silicon.

## ROM calls and known gaps

Implemented ROM entries include clock setup/query, IRQ critical-section
helpers, eFuse read, AES-128 block encryption, OSAL heap and memory functions,
`log_printf`, the ARM runtime memory/string functions,
`__ARM_common_switch8`, `spif_config`, `spif_cmd`, `spif_rddata`,
`spif_write`, `spif_write_dma`, sector erase, and `set_timer`. Both flash write
calls share the SDK's `(address, RAM source, byte count)` contract and program flash with NOR
1-to-0 semantics. SPIF command completion is synchronous; status reads report
WIP clear. Flash timing, write protection, page limits and most opcodes are not
modeled. Invalid arguments return `-EINVAL`.

The SDK declares ROM `set_timer(AP_TIM_TypeDef *, int)` at `0x16c2c`, but its
body is not included. The model follows the SDK `hal_timer_set_loadtimer()`
sequence: stop, select user mode, load `4 * time`, then enable the one-shot
timer. TS0001 calls it for Timer1 with `time=1512` while entering network
setup. This inferred ROM behavior is covered by a Timer1 expiry test.

The SDK exports `efuse_read` but has no public prototype. Its simple
`(block, buffer)` path only remains as a test fixture; firmware-observed calls
are logged and leave memory unchanged. For block 1, TS0001 passes a stack
address in `r1`, `0x60` in `r2`, and a Thumb code address in `r3`; a block 3
call passes a stack address, `0x1fff0100`, and `0x10`. The actual call ABI and
data layout remain unresolved, and the UART still prints `tpCap[00 00]`
instead of the user's `tpCap[2b 23]`. The remaining calibration bytes have not
been recovered, so eFuse compatibility is not established.

Two calls are deliberately visible limitations. The image reaches unexported
ROM entry `0xa2e0`, inside the SDK symbol range for `ble_crc24_gen`; its body is
not present in the SDK package, so the current return-only approximation logs
a warning. `LL_InitConnectContext` returns success for the unused BLE context
and logs that BLE connection state is not modeled. Neither behavior establishes
BLE or Zigbee stack compatibility.

The SDK declares `phy_sec_encrypt` and `phy_sec_decrypt` at ROM entries
`0x15338` and `0x1531c`. It also contains a disabled (`#if 0`) implementation
of the PHY+ AES-CCM-shaped decrypt operation: a 13-byte IV, a 4-byte MIC, a
counter increment of 13 per block, and a CBC-MAC over the plaintext. The model
implements that transform using the existing AES-128 primitive, plus the
matching encrypt direction. This is source evidence for the transform, but the
ROM implementation and provisioning rules are not included in the SDK.

The supplied TS0001 image has sixteen erased bytes at full-flash offset
`0x2920`, where its secure-area MIC would otherwise be. Its partition table
stores CRC-16/ARC values, and the CRCs over the source blocks match all eleven
entries. For this erased-MIC image, the model infers an unencrypted CRC path:
`phy_sec_encrypt` writes a two-byte CRC (zero padded to four bytes) to its MIC
output, while `phy_sec_decrypt` verifies the two-byte MIC and copies the bytes
unchanged. Three observed copy blocks pass, including executable SRAM blocks;
the Cortex-M translated-code cache is invalidated after a successful copy.
When the secure-area MIC is provisioned, the model instead uses the AES-CCM
transform and checks the four-byte MIC. That branch is covered by round-trip
and tamper-rejection tests, but has not been matched against a provisioned
device image. The erased-area selector and CRC fallback are image-driven
inferences, not a general PHY6252 ROM contract.

The image also reaches the SDK's `__aeabi_uidiv` and `__aeabi_idiv` ROM helpers,
which are modeled with quotient in `r0` and remainder in `r1`; division by zero
faults. `__aeabi_uread4` performs an unaligned little-endian 32-bit read, and
the observed `LL_Rand` call returns success and fills the requested 1..255
bytes with deterministic xorshift output. This is not cryptographic
randomness. The observed `rtc_get_counter` call returns the model's AON RTC count.
The PHY62 SDK places `ll_hw_*` declarations under its BLE controller, but the
PHY+ example also uses those helpers for Zigbee. The model implements the
observed mode selections (`STX`, `SRX`, `TRX`, `RTX`), IRQ status/mask/clear,
go/trigger, FIFO reset, `ll_hw_get_tr_mode`, `ll_hw_get_irq_status`,
`ll_hw_read_rfifo_zb`, and `read_current_fine_time` against the modeled
baseband registers and FIFOs. The Zigbee FIFO helper copies the length byte,
PSDU and FCS to firmware memory and returns the packet length and inferred
RSSI footer. The status register exposes the SDK ROM layout (`MD` bit 0,
`RTO` bit 2, `COK` bit 10) and the PB-03F Zephyr driver's compact layout
(`COK` bit 1, `RTO` bit 3) for the same completion; clearing either alias
clears the shared event. This reconciles the two checked firmware contracts,
whose observed bit definitions differ. Fine time uses the 16 MHz core cycle
count with the SDK's `BASE_TIME_UNITS` wrap. The PHY+ example names the ROM
timeout parameter `rxTimeOutUs`, so the model treats it as microseconds and completes
a receive with `RTO` when no frame arrives. Settle timing remains unmodeled
because the available sources do not identify its backing registers or clock
units. These operations do not establish Zigbee stack compatibility.
The Zephyr PHY driver samples `RF+0xe4` after its 128 us CCA settling delay;
the model returns the strongest shared-air signal at that time. Zephyr performs
frame filtering in software, and its ISR constructs requested ACKs in the TX
FIFO; its successful coordinator capture contains ACK frames from this path.
The SDK's `gpio_sel[pin / 4]` accesses are served from the inferred register
window `0x40003818..0x4000382c`; AON wake-source reads at `0x4000f0b0` and
`0x4000f0b4` return zero without generating wake events. These are code-driven
compatibility observations, not hardware-verified peripheral contracts.

The TS0001 image fills an LL extension/table area at `0x40031e00..0x40031ffc`,
inside the modeled 2 KiB RX FIFO range. Its RF effect is unknown. The PHY6222
SDK header declares `__VTOR_PRESENT` as zero, and its vector source provides only
the initial SP and reset entry. The model therefore ignores VTOR reads/writes
and initializes NMI, HardFault and PendSV entries in the low vector alias from
the SDK's `bb_rom_sym_m0` addresses. This fixed-vector mapping is inferred:
the ROM image and its full vector table are not included in the SDK.

After the 300 ms RF startup wait, TS0001 reaches PCR setup and pends PendSV from
`0x1fff66e6` by writing `0x10000000` to `ICSR`. The ROM symbol body is not in
the SDK, but the firmware contains a FreeRTOS PendSV handler at `0x1fff1972`
and a SysTick handler at `0x1fff6f10`; it installs their Thumb pointers in the
observed slots at `0x1fff0378` and `0x1fff037c`. The model dispatches the ROM
PendSV vector to the registered image handler and exposes the registered
SysTick handler through the fixed vector alias. This callback-table contract
is inferred from the image, not documented by the SDK.

The SDK maps external interrupt vectors 4 and 16 to ROM `LL_IRQHandler` and
`GPIO_IRQHandler`. Both ROM bodies are absent from the package. The model
dispatches these vectors through the image's `JUMP_FUNCTION` table: baseband
slot 228 (`V4_IRQ_HANDLER`) and GPIO slot 240 (`V16_IRQ_HANDLER`). The SDK's
GPIO setup first changes pin interrupt mode and later registers/enables its
callback. Clearing stale level status when a pin changes to edge mode avoids a
false interrupt in that interval. The callback slots are inferred from the
SDK header aliases and observed image accesses; the dispatch behavior is not
hardware-verified.

The SDK assigns Timer1..Timer6 to IRQs 20..25 and `jump_function.h` maps their
callbacks to `JUMP_FUNCTION` slots 244..249. The SDK `timer.c` specifically
installs the Timer5/6 callbacks before enabling those IRQs. Since ROM vector
bodies are unavailable, the model uses a private host trap for the six vectors
and dispatches the registered callback. Timer1 and Timer6 dispatch are tested
with synthetic handlers; real TS0001 reaches the Timer1 callback without a
CPU fault.

The supplied run prints `Stack Init`, initializes the Zigbee stack, creates the
application tasks and prints `zb_app_task_init`. The GPIO callback now runs and
the key/timer initialization proceeds. Execution reaches ROM symbol
`spif_write_dma` at `0x0001744c` (the Thumb symbol is `0x0001744d`), writing into
the printed NVM area `0x11079000..0x1107efff`. With DMA flash programming and
Timer6 IRQ dispatch modeled, the image restores network state 12, prints
`set read local time`, and emits Tuya reports during a 100M instruction run
without a CPU fault. Earlier HardFaults were caused by the GPIO level-to-edge
transition and the missing Timer6 vector; the first was fixed by clearing
stale level status, and the second had `CFSR.INVSTATE` with stacked IPSR 41.
The Cortex-M0 PSP PendSV regression test still passes. The earlier
`VTOR=0x1fff8b00` emulation selected an invalid RAM vector table and caused a
misleading HardFault; that is not the current failure.

## Firmware evidence

The PB-03F Zephyr `zigbee_shell` end-device image was run for 10 s of virtual
time with the TLSR8258 sample gateway. The UART reported
`zigbee_shell joined network (bdb status: 0x00)` at 9.858 s. The pcap checker
saw association at 7.80 s, 15 uplink NWK frames (11 unicast), and 8 downlink
NWK frames. This is an emulator-to-emulator result, not a hardware comparison.
The runnable system is `examples/phy6252-zephyr.yaml`; its image remains a
local input under `firmware/`.

The TS0001 dump contains Zigbee and FreeRTOS code, including retained source
path strings for the Zigbee LMAC, PHY62 radio driver and OS task sources. The
public PHY62XX SDK package has no `components/zigbee` source tree, so the image
is the available source of evidence for that stack's runtime behavior.

The user-supplied TS0001 log shows startup through `Stack Init`,
`Zigbee stack initialized`, restoration of network state 12 and Tuya reports.
The supplied binary contains the Zigbee and FreeRTOS executable image, and the
emulator reaches its Zigbee stack initialization path. It also passes three
observed CRC-checked copy blocks and prints the NVM range
`0x11079000..0x1107efff`. It still reports `tpCap[00 00]` instead of the
reference log's `tpCap[2b 23]`. The emulator runs past the `spif_write_dma`
NVM update, restores network state 12 and continues through timer callbacks. In
a 15 s `sim` run with a locally built Telink sampleGW, the pcap contained 19
frames: 16 beacon requests and 3 broadcast data frames. A 6 s coordinator-only
control run with the same `samplegw_tb03f.bin` produced the same frame types at
the same times, so these frames are attributable to the gateway image; no
TS0001 association request, NWK data or ACK was captured. Repeating the
first-boot scenario after adding generated RX FCS bytes did not change that
result. The emulated UART otherwise reaches the same startup milestones as the
supplied `log.txt`, but differs in reported values: `power report time = 4744`
(reference 3288), `set read local time: 39` (reference 44), and
`fix Report time = 326790` (reference 330785). Join and relay commands remain
unverified. Do not claim TS0001 relay support until it starts, joins, and
exchanges Zigbee traffic in a captured network scenario.

The reusable `examples/phy6252-ts0001.yaml` probe also reproduces the reference
short key1 press on GPIO8 and key-level transitions on GPIO3. It does not print
the reference relay `On`/`Off` reports, and the paired pcap remains identical
to the gateway-only capture.

For a separate first-boot probe, a temporary copy of the TS0001 dump had its
`0x6000`-byte NVM range at `0x11079000` erased. The image reported
`Device started for the first time` with network state 0. A long press on
GPIO8 reached ROM `set_timer` for Timer1, dispatched its registered callback,
and read RF register `0x400300e8`; the model returns the virtual-air RSSI
sample there because this SDK package does not document the register. The
image then ran through a 15 s simulation without a CPU fault, but still sent
no Zigbee frames. A temporary MMIO trace found no later TS0001 TX FIFO writes
or link-layer TX/RX commands after its BDB request; only coordinator frames
were present in the pcap. The trace narrows the next investigation to the
commissioning-to-PHY path, but does not establish whether the missing step is
commissioning state or a radio contract. The cleared-NVM copy and all other
firmware images remain external test inputs.

Run an image with `mcuemu run --model PHY6252 --firmware IMAGE --for 1s` or use
the YAML model name `phy6252` in `sim`. `--trace-pc-range START:END` enables
instruction tracing for a PC interval and prints registers on a fault.
`--trace-stack` reports when MSP or PSP leaves or returns to the PHY6252 SRAM range,
with the preceding instruction address. UART0 is exposed as `mcu.uart0`.
Firmware inputs can be the package beginning at physical flash offset `0x2000`
or a 512 KiB full flash dump; keep local images outside the repository.
For the TS0001 capture setup, place the full dump at
`firmware/phy6252-ts0001.bin` and run `examples/phy6252-ts0001.yaml` from a
scratch working directory. The scenario writes `air.pcap` there and probes
GPIO8 and GPIO3; inspect both the UART output and the captured frames.
