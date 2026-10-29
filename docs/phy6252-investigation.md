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
at `0x1fff0000`. ROM loader reads through the uncached `0x22000000` alias;
the model maps that window onto the same 512 KiB flash contents. The Zephyr
board reserves the region below `0x1fff1840` and
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
interrupts; `SWPORTA_CTL` at `0x40008008` is retained, but its pad-control
effects and alternate pad functions are not modeled. Timer1..Timer6 share the
SDK's AP_TIM register layout and a 4 MHz timer source, providing down-counting,
one-shot alarms and IRQs 20..25. The SDK Timer5/6 driver documents the 4 MHz
source and applies a factor of four when programming microsecond intervals;
the model applies the same rate to Timer1..Timer4 by inference. Timer ticks
therefore take four HCLK cycles at 16 MHz and eight at 32 MHz. Their
current-count registers are writable, as required by the SDK timer stop path.
The AON RTC counter advances at 32.768 kHz when `RTCCTL.RUN` is set;
direct image boot starts it in the state inferred for the skipped ROM board-init
path. The AON RTC clock-select, control, compare and flag registers are stored,
but compare events and IRQ generation are not. The four retained AON sleep
words at `0x4000f0c0..0x4000f0cc` are read/write storage, as indicated by the
SDK's sleep/tracking fields. AON `REG_S11` at `0x4000f0a8` stores the SDK sleep
flag. With an external ROM, the `0xa5a55a5a` write to `PWRSLP` at `0x4000f004`
waits for enabled RTC compare 0 and resets through the external ROM vector;
this wake path is inferred from the related SDK and has not been checked on
hardware. The watchdog uses the SDK's
2–256 second timeout selections, feed key and IRQ 10;
its clock source and reset details are inferred. PCR software reset and clock
registers, including SW_CLK1 and APB_CLK, retain writes; clock gating and reset
effects are not enforced. AP_CACHE and the observed AP_SPIF registers are
write-backed. AP_SPIF write-protection registers at `0x4000c850..0x4000c858`
start at the related SDK's `0, 0x10, 2` cache-init values. CPU stores through
XIP program flash only while `WR_PROTECTION` is zero; the ROM AHB programming
loop applies NOR 1-to-0 semantics. This is inferred from the PHY6222 SDK ROM
and register snapshot, not verified on PB-03F silicon. DMA channel 3 supports the observed
software-triggered memory copy used with SPIF remapping; other channels and
request sources are not modeled. The ADC and a complete clock tree are not
implemented.

AP_TIM3 starts as a masked free-running down-counter at `0xffffffff`. The SDK's
`WaitUs()` reads `AP_TIM3->CurrentCount` and derives elapsed microseconds from
the changing count; this is another ROM board-init state that direct image boot
must supply. The startup value and mask are inferred from that SDK code and the
TS0001 callback path, not verified on hardware.

Direct image boot skips the ROM's earlier RF initialization. The SDK symbol
table places `g_rfPhyTpCal0` and `g_rfPhyTpCal1` at `0x1fff0a44` and
`0x1fff0a45`; reset seeds them to `0x2b` and `0x23`, matching the supplied
TS0001 UART log. This restores the observed board startup state, but does not
model the analog calibration routine or establish values for the 2 Mbps
calibration globals.

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

The supplied PHY6222 ROM's `ll_hw_read_rfifo_zb` reads the 11-bit FIFO read
pointer from `LL+0x54[10:0]` and write pointer from `[26:16]`, then reads each
word repeatedly from the fixed RX FIFO port at `0x40031c00`; it reads two more
words for packet footer data after the word-aligned packet. The model exposes
that streaming port and pointer pair while preserving offset reads for the
Zephyr driver. Byte-based pointer advancement and the footer defaults are
inferred from this ROM path and the SDK contract, not measured on silicon.

The related SDK defines `LL_HW_FIFO_TX_1K_RX_3K` as `0x0100`; the external
ROM writes it to `LL+0x58` during TS0001 startup. The model retains and reads
this setting but still exposes its fixed 2 KiB TX / 2 KiB RX buffers. Zigbee
frames fit that modeled capacity, while the selected hardware partition and
its address boundaries remain unmodeled.

Ghidra decompiles the external ROM helper at `0xa92c` as a DMA channel 0
memory copy: it programs SAR, DAR, `CTL=1`, a byte count in CTL_H, `CFG=0`,
`CFG_H=4`, enables the controller, starts channel 0, polls transfer status,
then clears status and disables the channel. The model supports this observed
byte-wide transfer between mapped SRAM and XIP flash. Flash destinations obey
the modeled SPIF write-protection bit and NOR 1-to-0 programming; other DMA
modes and peripheral endpoints remain unimplemented.

## ROM calls and known gaps

Implemented ROM entries include clock setup/query, IRQ critical-section
helpers, eFuse read, AES-128 block encryption, OSAL heap and memory functions,
`log_printf`, the ARM runtime memory/string functions,
`__ARM_common_switch8`, `spif_config`, `spif_cmd`, `spif_rddata`,
`spif_write`, `spif_write_dma`, sector erase, `set_timer`, and `clear_timer`.
`clear_timer` is modeled with the SDK's `hal_timer_stop_counter()` sequence.
Both flash write calls share the SDK's `(address, RAM source, byte count)`
contract and program flash with NOR 1-to-0 semantics. SPIF command completion
is synchronous; status reads report WIP clear. The host-trap APIs do not model
flash timing, protection ranges, page limits or most opcodes. Invalid flash
and crypto arguments return `-EINVAL`; `efuse_read` returns its separate SDK
status codes described below.

The SDK declares ROM `set_timer(AP_TIM_TypeDef *, int)` at `0x16c2c`, but its
body is not included. The model follows the SDK `hal_timer_set_loadtimer()`
sequence: stop, select user mode, load `4 * time`, then enable the one-shot
timer. TS0001 calls it for Timer1 with `time=1512` while entering network
setup. This inferred ROM behavior is covered by a Timer1 expiry test.

The SDK exports `efuse_read` but has no public prototype, and the 512 KiB flash
dump does not contain the separate eFuse contents. Captured TS0001 calls pass
block 1 with `r1` pointing to a stack buffer, `r2=0x60`, and `r3=0x1fff2b29`;
another block 1 call uses `r2=0x60` and `r3=0x1fff2eb9`. A block 3 call uses
`r1=0x1fffce38`, `r2=0x1fff0100`, and `r3=0x10`. These `r2`/`r3` values are
caller state, not additional arguments: the related PHY6222 SDK declares the
function as `efuse_read(block, uint32_t *buf)`.

Two calls are deliberately visible limitations. The image reaches unexported
ROM entry `0xa2e0`, inside the SDK symbol range for `ble_crc24_gen`; the SDK
package has no body or contract for this entry, so the current return-only
approximation logs a warning. The external ROM dump discussed below contains
code at this address, but its state changes and version match have not been
confirmed against the TS0001 image. `LL_InitConnectContext` returns success for
the unused BLE context and logs that BLE connection state is not modeled.
Neither behavior establishes BLE or Zigbee stack compatibility.

### External ROM dump analysis

The supplied `rom_10000000_20000.bin` file is 128 KiB. Its last `0x8000` bytes
are erased (`0xff`); the first `0x18000` bytes are the 96 KiB ROM window used by
the model. The dump's SHA-256 is
`430b26391488ad95451310526aaff9ec9df4321897282fc7cd9d1865611517f4`. Ghidra
12.2 with the Homebrew OpenJDK 25 runtime analyzed the 96 KiB as ARM Cortex
Thumb code at address zero, matching the ROM's low-address vector pointers. The
dump starts with SP `0x1fff1830` and reset vector `0x00000d91`. The accompanying
`SoC_info.txt` identifies the source module as a PHY6222 (6222M005), so execution
on the PHY6252 model is a compatibility experiment; the dump has not been
established as the matching PB-03F ROM revision.

The ROM contains radio and link-layer code, not just boot support. Ghidra finds
several routines that access the RF block at `0x40030000` and LL block at
`0x40031000`; one packet-processing routine has a 6.6 KiB body and inspects
packet fields while using the LL registers. A setup routine at `0x178bc` also
writes the RF and LL register blocks. These are disassembly observations; they
do not identify a Zigbee NWK, APS, or BDB implementation. The ASCII strings
include boot-monitor commands such as `uartbin`, `uartrun`, `wrreg`, `clock`,
and `reset`, but no explicit Zigbee or IEEE 802.15.4 stack names. The lack of
such strings does not prove that higher layers are absent.

At `0x3fc4`, the dump has a small dispatch wrapper: it saves `r4` and `lr`,
loads a callback pointer from `0x1fff0118`, and calls it when nonzero. With a
zero slot it calls `0x3fdc`, the AES-128 encrypt helper already modeled by the
host ROM. The host trap now reproduces both branches and the wrapper's stack
epilogues without embedding the dumped instructions. Unit coverage checks the
AES fallback and a RAM callback. In the saved-state TS0001 run the slot contains
`0x1fff2671`; its callback now executes and returns instead of faulting at
`0x3fc4`.

The dump confirms the model's NMI (`0x8481`), HardFault (`0x28f1`), and PendSV
(`0x84cd`) vector targets. It also has a full interrupt vector table: IRQ4
points to `0x8871`, whose code dispatches through a callback pointer and falls
back to `0x4e24`. The current direct-boot model instead places a host trap at
`0x4e25` in the IRQ4 vector slot and dispatches `JUMP_FUNCTION(228)` there.
This keeps the observed Zephyr and TS0001 paths running, but the relationship
between the dump's full vector wrappers and the direct-boot callback table has
not been resolved. The dump remains an external analysis input; no ROM bytes
or Ghidra project files are stored in the repository.

The supplied `SoC_info.txt` reports the attached flash ID as `0x1364c8` and a
512 KiB capacity. The external-ROM path accepts the SDK's three-byte `RDID`
controller command and returns `c8 64 13` in wire order. This is tied to the
captured module; flash IDs are not yet configurable for other PHY6252 boards.

At `0xa2e0`, Ghidra sees a real function prologue, global-pointer setup, a
switch-helper call, and a later return in the ROM dump. The current shim skips
those effects. The SDK places this address within its `ble_crc24_gen` symbol
range, while the TS0001 image calls it as a zero-argument entry. Until the ROM
revision and the resulting globals are matched to the TS0001 call path, keep
the warning and do not treat this address as a Zigbee radio API. To compare
runtime behavior, PHY6252 can optionally execute the supplied dump as external
ROM code using its vector table; the default remains the explicit host-trap
ROM. This is an experiment path and does not itself establish ROM revision
compatibility or Zigbee stack support.

For a hybrid TS0001 run, a scratch copy of the ROM was patched at offset
`0xa2e0` to BKPT so this explicit return-only shim could be exercised while the
other dumped ROM routines ran. The relay image reached application code and
logged `Restored nwk success on reboot` and `network state=12`, but a 30-second
simulator capture contained no successful association and no TS0001 radio
frames. Erasing the image's `0x6000`-byte NVM range at `0x11079000` let the
fresh-start path continue after adding SPIF Write Enable and sector erase.
That run logged Zigbee stack initialization and `network state=0`, but the
30-second capture still contained only coordinator traffic. The TS0001 starts
one SRX operation; its modeled 20 ms timeout completes, but the firmware emits
no later radio trigger or TX frame in the capture. Neither run establishes
Zigbee relay support.

A follow-up 30-second run used the unmodified external PHY6222 ROM dump with
the same cleared-NVM TS0001 image, eFuse and RF calibration inputs, and the
sample gateway. On a fresh emulator build, the pcap contained 20 frames and no
TS0001 transmission; the checker found no successful association response.
Executing the real ROM code instead of the `0xa2e0` return shim therefore does
not by itself resolve the relay's first-boot network stall. The same ROM dump
does work on the PB-03F Zephyr direct-image path, which joins and exchanges
NWK traffic; these results establish a useful PHY6222-to-PHY6252 compatibility
case, not identity of every ROM revision or TS0001 support.

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

The PHY6252 SDK exports ROM `efuse_read` at `0xace1` but omits its prototype
and body. The related PHY6222 SDK declares `efuse_read(EFUSE_block_t,
uint32_t *)`; each of its four blocks is two words (8 bytes), and its public
error enum defines success `0`, invalid parameter `6`, and uninitialized `24`.
The emulator uses that ABI at the PHY6252 ROM trap: a valid supplied block is
copied to the output buffer, a missing block returns `24` without changing the
buffer, and an invalid block or output address returns `6`. This contract is
inferred from the related SDK, not hardware-verified. A raw 32-byte dump can be
provided with `mcuemu run --efuse DUMP` or `efuse: DUMP` on a `phy6252` in YAML;
it is an external input and is not stored in the repository. No eFuse data is
embedded in the firmware image, so the default model treats all blocks as
uninitialized rather than inventing device-specific values.

The external ROM dump contains its own `efuse_read` implementation. The SDK
header identifies `0x4000f160..0x4000f17c` as the four two-word eFuse data
registers and `0x4000f180` as `SECURTY_STATE`; the supplied register snapshot
reads `0x0f` from that status register. The emulator exposes each data block
only after its `efuse_cfg` read strobe and only when a raw `--efuse`/`efuse:`
input supplied the bytes. Without one, external-ROM execution reports the
unavailable data-register access instead of filling in unknown device data.
The supplied `SoC_info.txt` reports four zero-valued blocks, but no separate
raw 32-byte eFuse file was included.

The related SDK's `rf_calibrate1()` runs `rf_tpCal_gen_cap_arrary()`, then
`rf_rxDcoc_cfg(88, 1, &g_rfPhyRxDcIQ)`, then `rc32k_calibration()`. The RAM
snapshot contains `g_rfPhyRxDcIQ = 0x20200000`, which is the fallback the SDK
sets before polling `RF+0xec`. The model retains the documented DCOC setup
writes and reports the calibration-not-ready state at `RF+0xec`, allowing the
SDK fallback path to proceed; it does not model the analog conversion. It
retains AON `PMCTL1` and seeds it from the register snapshot (`0x01064040`),
which the SDK uses for RC32K cap-bank control. The captured `cal_ro2 = 0x100`
has no RC32K-ready bit set, so the related SDK's bounded calibration loop
leaves its result at zero. These AON values are module-specific observations.

The captured RAM bytes at `g_rfPhyTpCal0..g_rfPhyTpCal1_2Mbps` are
`2d 23 47 45`. The related SDK stores each `rf_tp_cal()` result plus two, so
the inferred `kCal` values are `2b 21 45 43`. A raw four-byte file with those
values can be passed using `mcuemu run --rf-calibration DUMP` or
`rf_calibration: DUMP` in `sim`; the emulator exposes them only after the
matching RF calibration trigger, channel, and `fDev` are observed. This
derivation depends on the related PHY6222 SDK and the supplied unit snapshot.
It is not an embedded default and does not establish that this ROM or RF
calibration contract matches PB-03F silicon.

With that ROM and the TS0001 image, emulation passes the early eFuse path and
reaches PHY initialization. Disassembly of the copied image shows a consecutive
setup table writing `0x40030054..0x40030074`; the model retains those values and
warns that their PHY6252 effects are unknown. The image then writes `0x0e` to
`0x400300a0`; the related PHY6222 SDK calls this the PLL auto override. That
write is also retained with a warning, without applying an analog effect.

The external-ROM run initially stopped on `RF+0xf4` at 19.06 ms. The related
SDK's `rf_tp_cal` triggers RF calibration and extracts `kCal` from bits 16..23
of this register. Supplying the four inferred bytes lets the image pass that
read and enter RX DCOC setup. The model then reaches the three-byte `RDID`
command (`0x9fa00001`) at `0x4000c890` and returns the captured module's JEDEC
ID. It also passes the later ROM initialization and reaches `PWRSLP` at
454.7235 ms. On RTC compare wake, the captured ROM enters its image-copy path
and faults at `PC=0x00000e8a` reading `0x22010000`.

A separate two-second saved-flash run used the four zero eFuse blocks recorded
in `SoC_info.txt` and the inferred RF calibration bytes `2b 21 45 43` from the
RAM snapshot. It produced no UART output. The CPU entered the ROM routine at
`0xafb8` with `r0=1` near 454.7 ms and again near 1.58 s, and was still in its
fallback loop at `PC=0xafc6` when the run ended. This pairing did not reach the
TS0001 key callback during the run; the reason the ROM callback path takes this
fallback remains unknown.

The image table stores XIP source addresses such as `0x11010000` and
`0x1100a000`. ROM loader code adds `0x11000000` before reading them, so the
first post-wake read is at `0x22010000`. This is the uncached XIP alias, not an
invalid source or evidence of a ROM/image format mismatch. Mapping that alias
to the same flash lets the captured ROM continue through the copy path. With
the matching full-flash dump, execution proceeds without a CPU fault; the
TS0001 image still does not transmit in the tested network scenarios, so relay
network support remains unverified. A 20-second same-channel first-boot run
with this external ROM produced 76 captured frames from the coordinator and
sample light, but no successful association response for TS0001 and no TS0001
transmission. Its MCU remained running throughout.

The supplied ROM is usable as executable PHY62xx ROM for direct-image runs:
its bytes are mapped at zero and `0x10000000`, and the Zephyr driver exercises
its flash and wake paths. A separate experiment started the CPU at the ROM's
reset vector for cold boot. That run remained alive for 10 seconds but produced
only 596 bytes of coordinator-side pcap traffic and did not associate. The
model therefore keeps direct image entry as the default; ROM reset startup
still needs its missing board and retained-state initialization modeled.

Using the same external ROM with the PB-03F Zephyr image reaches the Zephyr
boot banner and Zigbee thread. Its first flash update exposed a CPU store at
`PC=0x17558` to `0x1103dff0`. The Thumb routine at `0x17504` is the ROM's
memory-mapped flash program path: it clears `WR_PROTECTION`, copies bytes
through the XIP window, then restores protection. The model applies that store
path with NOR semantics. With the supplied zero-valued eFuse blocks and the
four RF calibration bytes derived from the register/RAM snapshot, the 10-second
air-check now passes: association at 7.80 s, 15 uplink NWK frames (11 unicast)
and 8 downlink frames. The UART still reports an invalid factory IEEE address
and uses its chip-ID fallback. This verifies the supplied ROM for this PB-03F
Zephyr scenario; it does not establish that ROM/image combinations or Zigbee
network stacks are interchangeable in general.

With the default host-trap ROM, the corrected ABI and no eFuse dump, an
8-second saved-state TS0001 run reads
blocks 1 and 3 as uninitialized, reaches `network state=12`, and remains
running. Its pcap has 19 frames (16 command and 3 data frames), all from the
coordinator; TS0001 sends none. UART timing values remain `4744`, `39` and
`326790`, so the eFuse fix does not explain the previously observed network or
reference-log differences. No genuine device eFuse dump was available for this
probe.

The image also reaches the SDK's `__aeabi_uidiv` and `__aeabi_idiv` ROM helpers,
which are modeled with quotient in `r0` and remainder in `r1`; division by zero
faults. `__aeabi_uread4` performs an unaligned little-endian 32-bit read, and
the observed `LL_Rand` call returns success and fills the requested 1..255
bytes with deterministic xorshift output. This is not cryptographic
randomness. The observed `rtc_get_counter` call returns the model's AON RTC count.
The PHY62 SDK places `ll_hw_*` declarations under its BLE controller, but the
PHY+ example also uses those helpers for Zigbee. The four mode helpers write
`0x000100` (`STX`), `0x010001` (`SRX`), `0x010102` (`TRX`), or `0x010103`
(`RTX`) to LL+`0x04`, and write `7` to LL+`0x38`; `ll_hw_get_tr_mode()` returns
the low nibble of LL+`0x04`. `ll_hw_go()` clears the `0x3fff` W1C status mask,
selects `MD`-only interrupts, triggers LL+`0x00`, and updates RF+`0x94` from
the two RAM-resident TP calibration endpoints and the clamped RF channel index.
`ll_hw_trigger()` only strobes LL+`0x00`. The model implements these contracts,
IRQ status/mask/clear, FIFO reset, `ll_hw_get_irq_status`,
`ll_hw_read_rfifo_zb`, and `read_current_fine_time` against the modeled
baseband registers and FIFOs. Only standalone TX and RX modes currently drive
the shared air; `TRX` and `RTX` remain register-only. The Zigbee FIFO helper
copies FIFO words containing the length byte, PSDU and FCS to firmware memory;
it returns the copied word count (`1 + (blen >> 2)`) and writes `blen + 1` to
`pktLen`, matching the SDK helper contract. SDK callers treat the high byte of
`pktFoot1` and RF+`0xe8` as a positive RSSI magnitude; the model keeps RF+`0xe4`
in the signed form used by the observed Zephyr CCA path. Carrier sensitivity
and frequency offset are not modeled beyond the inferred footer defaults. The
status register exposes the SDK ROM
layout (`MD` bit 0, `RTO` bit 2, `COK` bit 10) and the PB-03F Zephyr driver's compact layout
(`COK` bit 1, `RTO` bit 3) for the same completion; clearing either alias
clears the shared event. This reconciles the two checked firmware contracts,
whose observed bit definitions differ. Fine time uses the 16 MHz core cycle
count with the SDK's `BASE_TIME_UNITS` wrap. In the supplied ROM,
`ll_hw_set_rx_timeout()` at `0x11278` writes its microsecond argument to
LL+`0x28`; the related first-timeout helper at `0x11284` writes LL+`0x24`.
The model uses LL+`0x28` to schedule RX completion; LL+`0x24` is retained but
its first-timeout behavior is not modeled. In the 30-second fresh-NVM TS0001
run, the ROM writes `20000` us before its first SRX trigger. The empty receive
then completes with both `MD` (operation complete) and `RTO` (timeout cause)
when no frame arrives. This pairing is inferred from the PHY62xx SDK link-layer
driver: its IRQ handler only processes `MD`, while its receive-timeout path checks
`RTO`. The TS0001 image also enables only `MD` before its first receive. The ROM
helper `ll_hw_set_trx_settle(bb, afe, pll)` packs its byte arguments as
`bb << 16 | afe << 8 | pll` at `0x400300bc`, matching the related PHY62xx SDK
driver. The model retains this register value but does not delay radio operation
for settling; PHY6252 timing behavior is unverified. These operations do not
establish Zigbee stack compatibility.
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
the SDK's `bb_rom_sym_m0` addresses. The external dump independently confirms
these three addresses, but the model's IRQ dispatch traps have not been
reconciled with the dump's full vector table.

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

The same 10-second air-check passes when `rom:` selects the supplied PHY6222
dump, with external eFuse and RF-calibration inputs reconstructed from its
module snapshot. It joins at 7.80 s and exchanges the same 15 uplink (11
unicast) and 8 downlink NWK frames. This demonstrates compatibility for the
ROM code and register contracts exercised by this PHY6252 Zephyr image. It does
not establish that every PHY6222 and PHY6252 ROM revision or cold-boot path is
identical.

The TS0001 dump contains Zigbee and FreeRTOS code, including retained source
path strings for the Zigbee LMAC, PHY62 radio driver and OS task sources. The
public PHY62XX SDK package has no `components/zigbee` source tree, so the image
is the available source of evidence for that stack's runtime behavior.

The user-supplied TS0001 log shows startup through `Stack Init`,
`Zigbee stack initialized`, restoration of network state 12 and Tuya reports.
The supplied binary contains the Zigbee and FreeRTOS executable image, and the
emulator reaches its Zigbee stack initialization path. It passes three
observed CRC-checked copy blocks and prints the NVM range
`0x11079000..0x1107efff`. With the observed RF calibration globals seeded, the
UART now reports `tpCap[2b 23]`, matching the supplied log. The image runs past
the `spif_write_dma` NVM update, restores network state 12 and continues through
timer callbacks.

In a 65 s `sim` run with a locally built Telink sampleGW, the pcap contains 23
frames: 16 beacon requests and 7 broadcast data frames. Attaching
`air0=stdout` shows the gateway scans channels 11–26; a coordinator-only run
produces the same traffic. The capture contains no TS0001 association request,
NWK data, ACK or any other TS0001 transmission. The UART reaches the same
startup milestones as `log.txt`, but the reported values still differ:
`power report time = 4744` (reference 3288),
`set read local time: 39` (reference 44), and `fix Report time = 326790`
(reference 330785). Join and relay commands remain unverified. Do not claim
TS0001 relay support until it starts, joins and exchanges Zigbee traffic in a
captured network scenario.

The downloaded `TS0001_TZ3000_fdxihpp7_ff.bin` is byte-identical to the local
`firmware/phy6252-ts0001.bin` (SHA-256
`48808835c2c24db916617d8c780fad1e97b6ba92e9c8d73b27650cfa6b249517`). This is
the saved-state image: its NVM at `0x11079000..0x1107efff` is populated, unlike
the separate cleared-NVM first-boot probe. A 65 s run of this exact image with
host ROM traps, the supplied eFuse/RF-calibration inputs, and a short GPIO8
press prints the expected startup sequence through `network state=12`, the
Tuya report setup, and `key 1 Down/Up`. It repeats the local-time and stack
statistics at 30.305 s and 60.305 s. The `power report time`, local-time
interval, and `fix Report time` values remain `4744`, `39`, and `326790`; it
does not print the reference `On/Off` counters.

That run's pcap contains 82 frames. The successful association at 9.045 s is
sampleLight receiving short address `0x3acc`; all captured transmitters are the
coordinator and sampleLight. A 20 s probe with the same inputs accepts two
channel-25 beacon requests while TS0001 is listening, but TS0001 transmits no
frame. A 20 s run with the external PHY6222 ROM selected completed without a
CPU fault but produced no UART0 output for this TS0001 image; the startup result
above uses the default host ROM traps. The cause of this ROM/application
difference is not isolated.

The 65 s UART trace emits local-time and stack-statistics reports at 30.305 s
and 60.305 s. The local time remains zero; the trace does not print
the reference relay `On`/`Off` reports by 65 s. It prints `key timer stop!`
at 3.519 s; the supplied log spells this as `ky timer stop!`.

The `*** On:%d, Off:%d, toggle:%d ***` format belongs to the XIP helper at
`0x1102ddd8`. Its disassembly increments counters for event values 1 and 2,
then passes the counters to the UART formatter. The trigger that reaches this
helper in the reference run is still unknown.

A 1,000,000-instruction standalone run found a runtime data block at
`0x1fff7f34`: the word `6`, followed by pointers `0x1102ddd9` and
`0x1102d299`. The first pointer targets the counter helper above, confirming
that its registration data is present in SRAM. The meaning of the word and the
event path that invokes this callback remain unknown.

Further disassembly of the saved-network startup path shows a generic record
directory at `0x1fff8f4c + 0x39c`, with 12 entries spaced `0xa4` bytes apart.
The image reads a 16-bit record type at entry `+4`, a 16-bit payload length at
`+6`, and a trailer word at `+0x0c + length`. After 30,000,000 instructions,
entries 9 and 10 have types 4 and 5, payload lengths of 72 bytes, and trailer
words `6` and `0x36`. The startup path also prints
`Restored nwk success on reboot,1,0,87DB`. These are useful bounds for further
NVM analysis, but the record payloads and the meaning of `0x87DB` remain
unidentified; none can yet be treated as the network PAN, channel or key.

A PC trace of the first `power report time` calculation shows the image combines
its stored fields with a byte returned by its RAM-resident helper at
`0x1fff7b0c`: `3 * 1000 + 4 * (5 - 3) * 218 = 4744`. The helper output is
`0xda` in this emulator run. The later `fix Report time` path similarly
computes `5 * 60000 + (6 - 5) * 235 * 114 = 326790`. These values are built
from image state and helper results, so the UART mismatch does not identify a
direct AP_TIM clock error; the reason those inputs differ from the supplied
run remains unknown. Keep the measured timer source model and do not tune it to
force these application log values to match.

The sampleGW data frames carry MAC PAN ID `0xA75C` (wire bytes `5c a7`). The
TS0001 startup log also prints `0x87DB` in the restored-network line, but its
periodic stack report labels the same value `devID`; interpreting it as a PAN
ID was unsupported. The saved PAN, channel and security parameters have not
been decoded, so a network-identity mismatch remains possible but unconfirmed.
It also cannot explain the absence of any outbound frame during first-boot
commissioning; that radio path remains unverified.

An instrumented 10 s saved-state probe confirms that the PHY receive path is
active: while the TS0001 was listening on channel 25, the emulator accepted the
gateway's Beacon Request at 4.298 s with RSSI -42 dBm (8-byte MAC payload). The
capture still contains only gateway frames and no TS0001 response or other
transmission. This shows that the missing outbound traffic occurs after basic
PHY reception; it does not identify the remaining Zigbee stack or network-state
failure.

A 120 s run reaches the three periodic stack reports by 90.305 s, but still
does not print the reference `On`/`Off` counters by the end of the run. Its
pcap contains 26 frames: 16 Beacon Requests and 10 broadcast data frames, all
sent by the coordinator, with no ACKs or TS0001 transmissions. This covers the
period represented by the supplied log's three stack reports without reaching
its relay-counter output.

The reusable `examples/phy6252-ts0001.yaml` probe also reproduces the reference
short key1 press on GPIO8 and key-level transitions on GPIO3. It does not print
the reference relay `On`/`Off` reports, and the paired pcap remains identical
to the gateway-only capture.

Rerunning that 15-second scenario after correcting `efuse_read` confirms the
GPIO path still reaches the firmware: UART reports key 1 down/up and key-level
transitions, but no relay `On`/`Off` counters. The pcap has 19 coordinator
frames (16 command, 3 data) and no TS0001 transmission. Compared with the
supplied hardware UART log, `power report time`, `set read local time`, and
`fixReport time` remain `4744`, `39`, and `326790` instead of `3288`, `44`, and
`330785`; the hardware log's relay-counter lines are absent as well. The user
confirms that the `On`/`Off` lines in the hardware log follow pressing the
relay's local toggle button; they are not evidence of a remote Zigbee command.
The emulator recognizes the GPIO8 key press and release, but its application
path still does not reach the local relay state changes that produce those
counters.

After adding the `0x3fc4` ROM wrapper, a 12-second run using the hardware's
full-flash dump and saved NVM reaches both GPIO8 key events and stays running.
It still has no local relay `On`/`Off` counters. Its 75-frame pcap contains
transmissions from the coordinator and sampleLight, but none from TS0001. This
removes the wrapper fault from the trace without changing the button or network
result.

The key path in the image samples the GPIO state in `0x110242ac`, reports the
Down/Up transition, and schedules key events (`0x1010` for key 1 down,
`0x1002` for key 1 up) through its timer wrapper when an application callback
is registered. The emulator produces both `keyid` reports for GPIO8. A PC trace
around the registered `On`/`Off` handler at `0x1102ddd8` shows that it is not
called during that press. Later runtime tracing corrected the earlier
interpretation that the missing step was before the local state update.

The image callback at `0x11032670` decodes the event, prints `keyid`, and calls
the key dispatcher at `0x11026660`: key 1 down arrives as `(1, 1, 0)` and key 1
up as `(1, 0, 2)`. In a two-second run with the saved hardware flash, the
down-event branch sees the configured key mask (`0x1fff8016 == 2`) and enabled
switch flag (`0x1fff80a2 == 1`). It changes the first relay state byte at
`0x1fff7fd4` from `1` to `0`. The up event follows a separate branch and leaves
that state unchanged. GPIO sampling, debounce, timer delivery, key dispatch,
and the local software state change therefore execute in the emulator.

The state-change helper `0x110219b4` then calls `0x11020b18`
(`app_device_SW_state_set`) with key 1 and state code 2. That function updates
the app state at `0x1fff7f2c` and posts event `0x7001` through the image's event
queue. The `0x1102ddd8` callback still does not run, so the UART `On`/`Off`
counters remain absent. The event-queue path from this posted state change to
the registered callback is unresolved; the saved hardware RAM dump does not
provide a reliable comparison for these runtime tables.

Separately, `0x110219b4` reaches `0x11021f34`, which builds a command for
cluster `0x0006`. Its lookup in `0x1102e1f4` finds the endpoint-1/cluster-6
entry in the table at `0x1fffc0f8` and selects descriptor `0x1fffc178` with
index 0 and marker 3. The special branch through `0x1102cb98` reads bytes
`+6..+7` from entry 0 of the per-index table at `0x1fffb408`; both are zero.
The caller consequently skips `0x11043bf0`, the send path. This explains why
that command path produces no TS0001 transmission, but it does not by itself
explain the absent local `On`/`Off` callback. The callback remains uncalled and
the pcap has no TS0001 transmission. Do not force relay state or callback
execution from GPIO input alone.

For a separate first-boot probe, a temporary copy of the TS0001 dump had its
`0x6000`-byte NVM range at `0x11079000` erased. The image reported
`Device started for the first time` with network state 0. Holding GPIO8 from
1 s to 8 s reports `KEY 1 longPress` and `zb bdb devNetOpt = 0`; with the
supplied NVM contents, the same input reports option 7. In the erased-NVM
case, the `devNetOpt = 0` path selects channel 11 (`0x400300b4 = 5`), arms
Timer1 with `set_timer(..., 1512)`, then its callback writes `0x70ee5763` to RF
register `0x4003004c` and reads `0x400300e4` and `0x400300e8`. The PHY+ SDK
example names `0x4003004c` as the syncword register; that register's hardware
effect is not verified here. The model samples shared-air energy at the `0xe8`
read. In a timing probe, a gateway beacon request on channel 11 ended 120 us
before the read and the model returned about -41 dBm. The first 20 ms RX timeout
now sets both `MD` and `RTO`: the SDK IRQ line is enabled in the image's NVIC,
IRQ4 becomes pending, and the ROM vector dispatches `JUMP_FUNCTION(228)` at
`0x1fff2985`. That handler rearms SRX with the 20 ms timeout about every 20 ms
before the BDB callback. With AP_TIM3 running, the callback completes its
`WaitUs()` polling and eight-sample RSSI loop, calls ROM `clear_timer`, and
successfully enqueues a BDB event through the image's queue wrapper at
`0x1fff7ad0`. Its event-group bit `0x20` wakes the LMAC task; the task sees a
non-null queue entry (`0x1fffe748`) through `0x1fff8b94` and calls the image's
callback dispatcher at `0x1fff4bb0`. A PC trace shows state 0 and event 0
selecting table row `(0, 0, 1, 0x1fff4a69)` at `0x1fff7e78`. The dispatcher
updates the state to 1 and invokes that callback. The callback calls ROM
`LL_Rand` and arms Timer1 for 1512 us with image callback `0x1fff49d9`. Timer1
IRQ20 reaches the registered image handler `0x1fff2565`, which calls that
callback; it sets event-group bit `0x20` and PendSV runs afterward. That path
changes channel and syncword, samples RSSI, and starts one more SRX around
6.411 s. The trace does not show the BDB dispatcher receiving event 1, so the
next expected table row `(1, 1, 2, 0x1fff4acd)` is not reached during the 7 s
probe. No later TX operation starts in the LL trace, and the 20 s pcap has 20
gateway frames but no TS0001 frame. The image resumes its main loop around
8.0 s. The saved-NVM 15 s run likewise has only 19 gateway frames and no
TS0001 frame. All runs continue without a CPU fault. The transition from the
commissioning callback path to an outbound Zigbee frame remains unknown. In a
separate 20 s run,
gateway startup was delayed so its first beacon request began at 6.416 s, about
4.5 ms after the TS0001 SRX start at 6.411 s. The pcap still contained 19
gateway frames and no TS0001 frame. Keep the cleared-NVM image outside the
repository with the other local test inputs. A further 20 s first-boot run
added `samplelight_tb03f.bin` at 5 s. The light completed MAC association with
the sampleGW (request at 7.694 s, response at 8.190 s) and exchanged NWK data
from short address `0x7054`. In the post-eFuse-fix rerun, the long-press TS0001 reached
`devNetOpt = 0` and the sampleLight associated at 9.045 s with short address
`0x2554`; it then exchanged 9 uplink NWK frames (5 unicast) and 6 downlink
frames. The pcap has 76 frames. A simultaneous `air0=file:` trace attributes
every frame to either `coordinator.mcu` or `sampleLight.mcu`; TS0001 transmitted
none. A joinable, active network was therefore present, but the transition from
the TS0001 steering callback to its first outbound frame remains unknown.
Replacing `samplegw_tb03f.bin` with the alternate `telink-gw.bin` produced the
same channel result: that gateway formed a network on channel 15 (PAN `0xA134`),
and sampleLight associated at 9.045 s with short address `0x4c74` and exchanged
NWK data. The radio-source trace again contains no TS0001 transmission after its
channel-11 steering path. This second gateway image changes the PAN but does not
provide a channel-11 network for the observed first-boot path.

A same-channel check used the SDK's `apps/sampleGW` sources, with only
`DEFAULT_CHANNEL` changed to 11 in a scratch build. The 20 s run formed an
active channel-11 network: `sampleLight` associated at 9.045 s as `0x3acc` and
exchanged 9 uplink NWK frames (5 unicast) and 6 downlink frames. The pcap again
has 76 frames, all sourced by the coordinator or sampleLight; TS0001 emitted
none. After correcting the `ll_hw_read_rfifo_zb` return/length outputs and SDK
RSSI footer, the rerun's pcap was byte-identical; these ROM-contract fixes do
not change this first-boot stall. The later `LL+0x54` pointer and streaming
FIFO implementation also leaves this 20-second pcap byte-identical to the
previous capture. The radio medium recorded six decodable
channel-11 frames at TS0001,
including the sampleLight beacon request and coordinator response at 6.202 s
and 6.221 s. Its UART reaches `zb bdb devNetOpt = 0`, but no association or
other transmission follows. Channel mismatch alone therefore does not explain
the missing TS0001 exchange; the first outbound frame after this steering path
and the stack's later RX state remain unknown. This test used the cleared-NVM
first-boot image (`network state=0`), unlike the supplied `log.txt`, which
restores state 12 and includes Tuya reports and relay `On`/`Off` counters.

A separate 12 s run used the saved-state TS0001 image and a coordinator built
with `DEFAULT_CHANNEL=25`, matching the channel observed during the relay's
saved-state receive window. The UART again reached `network state=12` and the
Tuya report setup; the derived values remained `4744`, `39`, and `326790`, and
the reference relay `On`/`Off` lines were absent. Its pcap has 75 frames:
sampleLight associated at 9.045 s as `0x30c4`, then exchanged 9 uplink and 7
downlink NWK data frames with the coordinator. All captured transmitters were
the coordinator or sampleLight; TS0001 transmitted none. This verifies that a
joinable network was active on channel 25, but does not establish that its PAN,
extended PAN, or security state matches the saved TS0001 network.

To test whether the trailing `0x87DB` in the UART line was the saved PAN ID, a
second scratch coordinator set both its MAC PIB and NWK NIB PAN ID to `0x87DB`
after formation, while remaining on channel 25. The pcap again has 75 frames;
the transmitted data frames use PAN `0x87DB`, sampleLight associated at 9.045 s
as `0x3d34`, and it exchanged 9 uplink and 7 downlink NWK data frames. TS0001
still transmitted no frame. Therefore channel 25 plus PAN `0x87DB` is not
sufficient to restore the relay's network path; the saved extended PAN,
security material, or another stack initialization state remains unresolved.

An LL helper trace of the same-channel first-boot run for 10 s saw 317
`ll_hw_set_srx` calls and 317 radio triggers: 302 on channel 11 and 15 on
channel 26. It saw no `STX`, `TRX`, or `RTX` helper calls, no TX FIFO data and
no TS0001 transmission. The observed stall therefore occurs before this image
sets up an LL transmit operation; implementing combined radio modes would not
change this tested path.

Run an image with `mcuemu run --model PHY6252 --firmware IMAGE --for 1s` or use
the YAML model name `phy6252` in `sim`. `--trace-pc-range START:END` enables
instruction tracing for a PC interval and prints registers on a fault.
`--trace-stack` reports when MSP or PSP leaves or returns to the PHY6252 SRAM range,
with the preceding instruction address. `--dump-memory ADDRESS:COUNT` prints
bytes from the requested range after the run. UART0 is exposed as `mcu.uart0`.
If a device eFuse dump is available, pass it with `--efuse DUMP` or the YAML
`efuse:` option; it must contain exactly four consecutive 8-byte blocks.
To execute or compare an external ROM, pass it with `--rom DUMP` or the YAML
`rom:` option. It must be 96 KiB or a 128 KiB dump with an erased 32 KiB tail;
the ROM bytes and vector table are mapped from that file. Firmware still starts
at its parsed application entry, because a complete cold boot from the ROM
reset vector is not established. The supplied PHY6222 dump has been exercised
with the PB-03F Zephyr image using that direct-image path; the Zephyr image joins
and exchanges NWK traffic. For a reproducible YAML setup, see
`examples/phy6252-zephyr-external-rom.yaml`; it expects local ROM, eFuse and RF
calibration files under `firmware/`.

`--boot-from-rom` or YAML `boot_from_rom: true` starts at the external ROM's
reset vector instead. This is an experimental ROM-analysis mode: with the
supplied dump and TS0001 image it remains in ROM (PC `0xac52`) for a one-second
run and emits no UART output. It does not yet hand off to the image, so use the
default direct-image path for the verified Zephyr compatibility scenario.
Firmware inputs can be the package beginning at physical flash offset `0x2000`
or a 512 KiB full flash dump; keep local images outside the repository.
For the TS0001 capture setup, place the full dump at
`firmware/phy6252-ts0001.bin` and run `examples/phy6252-ts0001.yaml` from a
scratch working directory. The scenario writes `air.pcap` there and probes
GPIO8 and GPIO3; inspect both the UART output and the captured frames.
