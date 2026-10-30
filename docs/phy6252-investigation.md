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
and RTC compare 0 can wake the modeled system-sleep path when both compare-0
interrupt and event enables are set. AON GPIO wake uses the related SDK's
`c_gpio_index[]` pad mapping for `REG_S9`/`REG_S10` masks, AP_GPIO edge mode,
and the IOCTL polarity bit for each pad; a matching edge sets the corresponding
`GPIO_WAKEUP_SRC` bit and wakes through the external ROM vector. Other AON
compare IRQ behavior is not modeled. The four retained AON sleep
words at `0x4000f0c0..0x4000f0cc` are read/write storage, as indicated by the
SDK's sleep/tracking fields. AON `REG_S11` at `0x4000f0a8` stores the SDK sleep
flag. With an external ROM, the `0xa5a55a5a` write to `PWRSLP` at `0x4000f004`
waits for an enabled RTC compare 0 or a configured GPIO wake edge, then resets
through the external ROM vector. These wake paths are inferred from the related
SDK and have not been checked on hardware. The watchdog uses the SDK's
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
`__ARM_common_switch8`, `LL_InitConnectContext`, `spif_config`, `spif_cmd`,
`spif_rddata`, `spif_write`, `spif_write_dma`, sector erase, `set_timer`, and
`clear_timer`.
`clear_timer` is modeled with the SDK's `hal_timer_stop_counter()` sequence.
Both flash write calls share the SDK's `(address, RAM source, byte count)`
contract and program flash with NOR 1-to-0 semantics. SPIF command completion
is synchronous; status reads report WIP clear. The host-trap APIs do not model
flash timing, protection ranges, page limits or most opcodes. Invalid flash
and crypto arguments return `-EINVAL`; `efuse_read` returns its separate SDK
status codes described below.

The supplied ROM dump's entry at `0x8900` enables the AON RTC and polls its
32.768 kHz counter until the requested number of ticks has elapsed. The host
model implements that wait as a virtual-time delay and services scheduled chip
events during it. This removes one missing host-ROM call seen in the closed
TS0001 startup trace; it does not yet explain the later point where that image
stops receiving or requesting radio transmission.

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

`LL_InitConnectContext` at `0x5044` is modeled from the supplied PHY6222 ROM
disassembly. It accepts up to eight context objects, clears `0x284` bytes per
object, and initializes pointer fields at offsets `0x16c`, `0x18c`, `0x1ac`,
and `0x1b0`; its globals begin at `0x1fff0914`. The two stack arguments select
the observed `0x30` or `0x10c` pointer stride, while other modes use incoming
`r5 + 6`. Invalid counts or memory ranges return `LL_STATUS_ERROR_BAD_PARAMETER`
(`0x12`). The TS0001 image reaches this routine during startup. This host
implementation reproduces its context setup only; it does not add BLE radio or
connection behavior.

The image also reaches unexported ROM entry `0xa2e0`, inside the SDK symbol
range for `ble_crc24_gen`. The SDK package has no body or contract for this
entry, so the current return-only approximation logs a warning. Although the
supplied ROM dump contains code at this address, its state changes and version
match have not been confirmed against the TS0001 image. Neither ROM entry
establishes Zigbee stack compatibility.

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
(`0x84cd`) vector targets. Headless Ghidra analysis confirms IRQ4 points to
`0x8871`: the wrapper reads `JUMP_FUNCTION(228)` from `0x1fff0390`, calls a
non-null entry with `BLX`, and returns; when the slot is null it calls the
stateful ROM handler at `0x4e24`. That fallback checks LL state and dispatches
different ROM routines, so it is not a no-op. In direct-image mode without an
external ROM, the model instead places a host trap at `0x4e25` in IRQ4's vector
slot and emulates the non-null callback path: it supplies the callback pointer
in `r0`, saves `r4` and `lr`, and returns through a small synthetic
`pop {r4,pc}` trampoline. A null slot still stops with an explicit error rather
than inventing the ROM's `0x4e24` fallback. The saved-state TS0001 baseband slot
points to `0x1fff2985`, an interior Thumb entry within a larger RAM routine
starting at `0x1fff28f1`. Its XIP initializer at `0x110148d0` writes that entry
to the callback slot. The surrounding routine uses `r0` as a mode value and the
block at `0x1fff2984` returns `0x12` for values other than 0 or 1. The dumped
ROM's wrapper leaves the callback address in `r0`, so this image and ROM dump
appear to have an incompatible IRQ callback ABI. This remains a concrete
version-mismatch observation, but the callback argument alone does not explain
the stalled radio path: changing it has not changed the tested execution. The
relay may use a different ROM revision, and no hardware register trace is
available. The dump remains an external analysis input; no ROM bytes or Ghidra
project files are stored in the repository.

A diagnostic experiment temporarily passed `r0=0` and `r0=1` to the saved-state
TS0001 callback instead of the callback address. In a stronger 20-second
first-boot run, all three variants (pointer, zero, one) produced byte-identical
UART logs and pcaps (SHA-256 `64874e20bdd2c8fdc7e886482af7e674e72527be622d19c9e6763f1400f8fa03`
and `67dad0098c9382feb206057edd518688aeb0be4c6c867fab42e4820837ebf2f7`,
respectively). Each pcap has 76 frames: the coordinator formed an active
channel-11 network, sampleLight joined as `0x3acc` and exchanged NWK data, but
TS0001 emitted no frame. Substituting either scalar therefore has no observable
effect in this scenario. Keep the ROM-disassembly-based pointer argument. This
comparison does not establish the callback ABI used by the relay's actual ROM
revision or identify why its application never requests transmission.

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
The emulator uses that ABI at the PHY6252 ROM trap: a valid programmed block is
copied to the output buffer and returns `0`; a missing block returns `24`
without changing the buffer; and an invalid block or output address returns
`6`. A supplied all-zero block is copied to the output buffer and returns `24`,
matching the supplied external ROM's behavior on the captured module. The ABI
and status codes come from the related SDK; the zero-block behavior is
confirmed by the ROM trace for block 1. A raw 32-byte dump can be
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
RAM snapshot. It produced no UART output. A PC trace shows the image reaches
ROM helper `0xafa0` with `r0=1` near 454.7 ms. The helper reads a null indirect
pointer at `0x1fff0100` and falls through to `0xafb8`; at 600 ms the CPU remains
in that fallback loop at `PC=0xafc6`. The SDK's `JUMP_FUNCTION` macro maps
`0x1fff0100` to slot 64, `ENTER_SLEEP_OFF_MODE`; the ROM map names `0xafa1`
`enter_sleep_off_mode` and `0xafb9` `enter_sleep_off_mode0`. The PHY62XX
power-manager source calls the former for RTC-reset sleep after preparing
retained state. The TS0001 initializer writes other entries around this jump
table slot but leaves slot 64 zero, as does the saved hardware RAM snapshot.
The trace therefore follows the documented null-slot fallback; it is not
evidence that a Zigbee callback failed to register. The fallback's infinite
loop and the cold-wake handoff still need hardware validation. The same trace
shows the dumped ROM's
`efuse_read` at `0xace0` reads zero words from `0x4000f168..0x4000f16c`, writes
them to the output buffer, and returns `PPlus_ERR_UNINITIALIZED` (24). The host
ROM trap now reproduces that result for a supplied all-zero block.

Follow-up Ghidra analysis of the TS0001 initializer at `0x110204e0` finds it
sets the separate ROM-dispatch slot at `0x1fff0118`. The external-ROM stall is
therefore in the low-power entry/wake path. It does not account for the
separate host-ROM commissioning run, whose first-boot trace still stops before
any LL transmit setup.

An AON snapshot while that ROM is asleep at 600 ms reads `RTCCTL=0x148001`,
`CMP0=0xba74`, and `RTC count=19493`; the saved compare corresponds to about
1.46 s at 32.768 kHz. At 1.5 s the count is 48951 and the PC has advanced to
`0x892e`, inside the ROM's elapsed-time wait routine. This confirms the
modeled compare0 wake occurs; the no-UART result remains in the ROM/application
startup after that wake.

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

The supplied ROM dump (`rom_10000000_20000.bin`, SHA-256
`430b26391488ad95451310526aaff9ec9df4321897282fc7cd9d1865611517f4`) is usable
as executable PHY62xx ROM for direct-image runs:
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
the low nibble of LL+`0x04`. The PHY+ example uses `TRX` for a transmit that
expects a response, setting an RX timeout before starting the operation. The
model now transmits the FIFO frame, then listens until it receives a frame or
the LL+`0x28` timeout expires; `MD` and `COK` or `RTO` report the result. `RTX`
still remains register-only; triggering it logs that its operation is not
modeled. `ll_hw_go()` clears the `0x3fff` W1C status mask,
selects `MD`-only interrupts, triggers LL+`0x00`, and updates RF+`0x94` from
the two RAM-resident TP calibration endpoints and the clamped RF channel index.
`ll_hw_trigger()` only strobes LL+`0x00`. The model implements these contracts,
IRQ status/mask/clear, FIFO reset, `ll_hw_get_irq_status`,
`ll_hw_read_rfifo_zb`, and `read_current_fine_time` against the modeled
baseband registers and FIFOs. Standalone TX/RX and the TRX response window
drive the shared air. The Zigbee FIFO helper
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

A standalone 35 s direct-image run with that ROM, the module eFuse dump and RF
calibration dump confirms this is not a missing UART sink: it emits no UART
bytes and ends at ROM PC `0x0000afc6` after 17,083,061 instructions and 560
million cycles. This is distinct from `boot_from_rom`, which starts at the ROM
reset vector and remains in its monitor path. The PHY6222 ROM is validated with
the PB-03F Zephyr image, but this run does not establish compatibility with the
TS0001 ROM calls, and its lack of traffic cannot be used to judge the relay's
Zigbee behavior.

A second 65 s run used the module's 512 KiB full-flash dump
`flash_11000000_80000.bin` (SHA-256
`def028f783aac9d3d648b32d7e7b250180e3aa8c7c04bd506858ce0109b5770a`), its
all-zero eFuse dump and the supplied four-byte RF calibration, with the same
host ROM traps and a GPIO8 press. With the coordinator alone, UART again reaches
`network state=12` and reports key 1 down/up. It reports `power report time =
4744`, `set read local time: 39`, and `fix Report time = 323030`; the reference
values are `3288`, `44`, and `330785`. At 30.305 s and 60.305 s it reports
`MiF=0x18C8`, `Tim=131`, and `lMac=210`, versus `MiF=0x18A0`, `Tim=122`, and
`lMac=154` in the supplied log. There are no local `On/Off` lines by 65 s.
The 23-frame pcap has no successful association response and no TS0001 frame;
the Zigbee air checker reports no joined device. A three-device replay with
sampleLight yields its association and NWK exchange, but those frames must not
be attributed to TS0001. The two-device capture confirms that the saved-network
relay path is still unverified with this coordinator.

The 65 s UART trace emits local-time and stack-statistics reports at 30.305 s
and 60.305 s. The local time remains zero; the trace does not print
the reference relay `On`/`Off` reports by 65 s. It prints `key timer stop!`
at 3.519 s; the supplied log spells this as `ky timer stop!`.

A 65 s replay with only one GPIO8 press moved to 6 s matches the log's event
order: the two Tuya report lines and `fix Report time` print at 5.203 s, then
`key 1 Down/Up` at 6.010/6.510 s. GPIO2 changes from high to low at 6.010 s,
confirming the local relay output follows the button. The UART still reports
`power report time = 4744`, `set read local time: 39`, and `fix Report time =
326790`, against `3288`, `44`, and `330785` in the supplied log. At 30.454 s
and 60.454 s it reports `MiF=0x1918`, `Tim=131`, and `lMac=210`, against
`MiF=0x18A0`, `Tim=122`, and `lMac=154` in the log. No local `On/Off` lines
appear. The pcap is unchanged: 23 coordinator frames, no successful
association response, and no TS0001 transmission. Moving the local button
after startup therefore fixes the UART ordering but does not resolve either
the missing diagnostic callback or Zigbee transmit path.

After matching the host `efuse_read` trap to the dumped ROM for blank blocks, a
65 s replay with the captured all-zero eFuse input produces the same UART
values and GPIO2 transition. Its 23-frame pcap still has no successful
association response and no TS0001 transmission. Supplying the blank eFuse
contents therefore does not account for the missing local diagnostic callback
or the network stall.

The `*** On:%d, Off:%d, toggle:%d ***` format belongs to the XIP helper at
`0x1102ddd8`. Its disassembly increments counters for event values 1 and 2,
then passes the counters to the UART formatter. The maintainer first associated
these reports with the local toggle button, then said the trigger is uncertain
and may require an over-the-air toggle. The UART log has no timestamped event
correlation, and the route from either input to this helper remains unresolved
in the emulator.

A four-second VCD probe of the module full-flash image confirms the physical
GPIO path. Startup logs `relay_LED[0] = 2,0`; GPIO2 is initially driven low,
then goes high when GPIO8 key 1 is pressed at 1.009 s. GPIO3 is the second key:
it reports key 2 transitions and changes GPIO2 low on press at 3.049 s and high
on release at 3.549 s. The `On/Off` UART helper remains uncalled. The TS0001
example now exposes GPIO2 as `relay_line` in `relay.vcd`, so local relay state
can be checked independently of the missing diagnostic callback.

A 1,000,000-instruction standalone run found a runtime data block at
`0x1fff7f34`. The saved-state image initializes it from file offset `0x17f0c`
(XIP address `0x11017f0c`) with two `(event ID, function)` pairs:
`(6, 0x1102ddd8)` and `(0xe000, 0x1102d298)`. The stored function pointers have
the Thumb bit set. Startup function `0x1102dc88` registers the table through
`0x11040940`, which stores the pointer and count in the registry at
`0x1fff81d0`; the registry also records callback pointer `0x11043375`.
Dispatcher `0x11041a90` compares the 16-bit event ID at offset
`+4` in its event object against this table and indirectly calls a matching
handler. Its only static direct call site in this image is `0x11045ae0` in the
message-processing routine at `0x11045a34`. A narrow PC trace of an offline
two-press GPIO8 run records no execution in `0x11041a90..0x11041ad0`. The
emulated local key path therefore does not reach this registered callback
dispatcher; the hardware route from button handling to the `On`/`Off` helper
remains unresolved.

Further disassembly of the saved-network startup path shows a generic record
directory at `0x1fff8f4c + 0x39c`, with 12 entries spaced `0xa4` bytes apart.
The image reads a 16-bit record type at entry `+4`, a 16-bit payload length at
`+6`, and a trailer word at `+0x0c + length`. After 30,000,000 instructions,
entries 9 and 10 have types 4 and 5, payload lengths of 72 bytes, and trailer
words `6` and `0x36`. The startup path also prints
`Restored nwk success on reboot,1,0,87DB`. These are useful bounds for further
NVM analysis. The ZB-info record header at flash offset `0x7923c` declares a
112-byte payload beginning at `0x79240` with bytes
`03 00 00 00 4a 01 00 00 d9 a9 db 87 00 f8 ff 07`. The
adjacent 16-bit values are consistent with PAN ID `0xa9d9` and short address
`0x87db`; the latter also matches the log's `devID` label. The following word
is the all-channel mask `0x07fff800`. Later in the same payload are the
candidate extended PAN ID `b2:06:0c:7a:9d:3a:16:2c` and 16-byte network key
`54:78:b1:4c:c6:f6:d7:ae:71:2e:63:38:0d:f0:ad:6e`. This field mapping is
inferred from the record layout and SDK structures; the saved operating
channel and complete security state are still unconfirmed.

A PC trace shows that the image's helper at `0x1fff7b0c` calls the ROM
`LL_Rand` entry (`0x5e04`) to fill a one-byte result. The first report computes
`3 * 1000 + 4 * (5 - 3) * random_byte`; with the original standalone default
seed, byte `0xda` gives `4744`, while the reference's `3288` corresponds to
byte `0x24`. The later fix-report calculation is
`5 * 60000 + (6 - 5) * 235 * random_byte`; byte `0x72` gives `326790`, while
the reference's `330785` corresponds to `0x83`. The
emulator's LL_Rand is deterministic, while the board ROM supplies
hardware-dependent values, so these timing differences are expected and do not
indicate an AP_TIM clock error. In `sim`, `world.seed` and the MCU instance name
now seed this deterministic sequence; standalone runs keep the fixed default
seed. This makes the generated UART values reproducible without treating the
board's captured random sequence as a timing requirement. The stack counters
still differ and have not been explained.

The `set read local time` helper also consumes one `LL_Rand` byte and maps it
to `35 + floor(10 * byte / 255)`: byte `0x72` gives 39, while the captured value
44 corresponds to a byte from `0xe6` through `0xff`. Its difference is random
input, not a timer discrepancy.

After the `world.seed` integration, the checked-in 10-second TS0001 UART test
with `seed: 1` and instance `relay.mcu` prints `power report time = 3040`,
`set read local time: 44`, and `fix Report time = 301410`. The local-time value
now matches the captured log; the two report intervals differ because they are
also seeded random values. This does not change the offline-only status of the
TS0001 radio path.

The 15-second network example, now using those same local board inputs, prints
`power report time = 3184`, `set read local time: 42`, and
`fix Report time = 356870`. Its pcap has 19 frames: 16 coordinator Beacon
Requests and three coordinator Data frames, with no TS0001 source frame. The
radio-attached run consumes the seeded generator along a different startup
path, so its UART values differ from the offline test while remaining
repeatable for that scenario.

The earlier sampleGW data frames carried MAC PAN ID `0xA75C` (wire bytes
`5c a7`). A later scratch coordinator used the candidate PAN ID `0xA9D9`,
short address `0x87DB`, candidate extended PAN ID and candidate NWK key. Its
pcap confirms that the coordinator transmitted with PAN ID `0xA9D9`, but the
TS0001 still emitted no frame during the 10-second run. The test does not
confirm the extended-PAN/key mapping or restore the module's parent state.
It also does not explain the absence of an outbound frame during first-boot
commissioning; that radio path remains unverified.

The saved ZB-info payload contains the same eight-byte value twice at offsets
`+0x20` and `+0x28` (`9e:75:d9:fe:ff:e2:0a:68`), a possible parent/coordinator
IEEE address. A separate scratch coordinator was built with that IEEE address
in addition to the candidate PAN ID, short address, extended PAN ID and NWK
key. In a 35-second run the relay printed `Restored nwk success on reboot`
and its GPIO8 press still changed the local relay output, but it transmitted no
802.15.4 frame. The 21-frame pcap contains 16 coordinator Beacon Requests and
five coordinator data frames; it has no successful association response, ACK,
or relay transmission. Thus matching this candidate IEEE address alone does
not restore radio communication. The repeated value's meaning and the other
conditions for the saved parent relationship remain unconfirmed.

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

The firmware also samples GPIO3 during startup (`GPIO:3,3` followed by
`High level` in the hardware log). This line is the second key input and must
idle high. The offline replay now gives GPIO3 an explicit board pull-up, which
reproduces that diagnostic; leaving the pad unconnected made the modeled input
read low even though GPIO8 was biased correctly.

An eight-second offline replay that presses GPIO3 at 6 s also changes GPIO2:
the UART reports `keyid = 2` and key-level transitions, and the relay output
goes low on press and high on release. It still emits no `On/Off` counters.
Both configured local inputs can therefore change relay state in this image;
neither replay reaches the registered diagnostic callback.

A separate 15-second offline replay held GPIO8 from 6 s to 10 s. It produced
the normal key 1 down/up messages and left GPIO2 low after the press, but still
did not print `On/Off` counters. A longer hold does not explain the missing
callback.

A 120-second replay of the checked-in offline scenario with the current image
reproduces startup, `High level`, Tuya report setup, both GPIO8 press/release
pairs, and the three periodic stack reports through 90 s. The relay output
returns high after the second press. Neither `On/Off` counter appears by 120 s.
The startup values remain `power report time = 4744`, `set read local time: 39`,
and `fix Report time = 326790`, versus `3288`, `44`, and `330785` in the
reference UART.

Rerunning the current 15-second radio scenario with the corrected GPIO3
pull-up prints `High level` and key 1 down/up, but no relay `On`/`Off` counters.
Its 19-frame pcap contains 16 coordinator Beacon Requests and three coordinator
data frames; it has no successful association response and no TS0001 source
frame (`0x87db`). Compared with the supplied hardware UART log,
`power report time`, `set read local time`, and `fix Report time` remain
`4744`, `39`, and `326790` instead of `3288`, `44`, and `330785`. The user
initially attributed the `On`/`Off` lines to the local toggle button, then said
the trigger is uncertain and might be an over-the-air toggle. The GPIO8 press
changes the relay state and GPIO2 output, while the diagnostic callback remains
unreached; this run does not identify what causes the reference lines.

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

Ghidra analysis of the supplied flash shows `app_device_SW_state_set` passes
`(0x7001, 0, 8, 0x1fff7f2c)` to the image wrapper at `0x1103d248`. In the
button trace that wrapper calls `0x1103d1ac`, which returns task id `0x20`,
then enters `0x11038668`. That routine prepares a 0x14-byte queue record using
the descriptor at `0x1fffc7cc`; its static path reaches the queue insertion
helper at `0x11038700` when allocation and copy succeed. A second runtime trace
reaches that helper from `0x11038668` and reads the queue descriptor at
`0x1fffc7cc`; it does not establish that the record remains queued or identify
the task that consumes it. The trace still shows no call to `0x1102ddd8`, so
the local button's hardware `On`/`Off` reports are not yet reproduced.

A four-second replay of `examples/phy6252-ts0001.yaml` with a post-run memory
snapshot identifies the queue backing store. The image's table at
`0x1fff83c0` contains base `0x11079000` and size `0x3000`; its descriptor at
`0x1fffc7cc` advances from offset `0xc78` to `0xc94` while the GPIO8 event is
posted. The resulting 28-byte record is visible at flash address
`0x11079c78`: sequence `0x41`, size `0x1c`, task id `0x20`, and an eight-byte
payload beginning `01 00 00 00`. The payload matches the changed relay-state
byte. This confirms the local button event is copied into the image's
flash-backed queue.

A PC trace through the enqueue helper `0x11038700` shows that
`FUN_00020054` returns slot 1 and the status byte at
`0x1fff8f4c + 4 + 0x388a` (`0x1fffc7da`) is 1. The branch at `0x11038762`
therefore skips `0x1103d734`. This is not evidence that the application event
callback was left asleep: `0x1103d734` walks the configured queue region in
4 KiB steps, calls ROM entry `0x1fff40d8` for each page, then runs
`0x1103d5b8`/`0x11037348` bookkeeping. The status bits guard that storage
operation. The ROM entry's exact symbol is unresolved, but the page loop makes
flash-queue maintenance a better fit than a callback notification. Do not
clear this status byte in the model or treat it as the missing callback wakeup.

The image call chain also shows that `app_device_SW_state_set` (`0x11020b18`)
ends by passing the changed application-state bytes to `0x1103d248`, which
creates the flash-backed record. Separately, the startup routine
`0x11020890` reads records through `0x1103d1f0`. These observations support
that the queue stores state for later restoration; they do not show that it
dispatches the runtime `0x1102ddd8` diagnostic callback. The callback's route
from the local button state change remains unresolved.

Forced Thumb disassembly of `0x1102ddd8` identifies the missing diagnostic
handler. It passes the event code at `param[1]` to `0x110221b0`, increments
one of three counters, then prints the format string at `0x1102de38`
(`On:%d, Off:%d, toggle:%d`). Event code 1 increments `On`, codes 0 and
`0x40` increment `Off`, and code 2 increments `toggle`. This matches the
counter labels in the supplied UART log and confirms the absent lines are
caused by this handler not running in the emulator; it does not identify what
dispatches the handler in the hardware run.

Headless Ghidra xrefs in the TS0001 image show the format string referenced
from `0x1102ddd8`, but no direct static callsite for that handler. This is
consistent with its registration in the event table and indirect dispatch;
the references do not establish whether the event originates from a local
button or an over-the-air state change.

The maintainer said the reference UART log was captured with no coordinator,
but later clarified that it is uncertain whether the `On`/`Off` lines follow a
physical button press or an over-the-air toggle. The log does not timestamp
those lines against an input event. A 65-second host-ROM replay without a
coordinator, using the saved-state image and GPIO8 presses at 6–6.5 s and
7–7.5 s, prints both `key 1 Down/Up` pairs and changes GPIO2 at each press. It
restores `network state=12` and repeats stack statistics at 30 s and 60 s, but
never prints the reference counters. No radio medium is present. This verifies
local button and output behavior, but does not establish the trigger for the
reference `On`/`Off` lines or resolve the callback path.

Disassembly of the external ROM's `0x27e8` wrapper initially looked like a
possible route to that callback: it calls `0x7f1c`, which loads a callback from
`0x1fff0038` and invokes it when non-null. The wrapper rejects `r0` values with
any bits above bit 7 set, then forwards that scalar in `r0`; `0x1102ddd8`
instead treats `r0` as a pointer and reads a record through offset `0x1c`.
These ABIs do not match. The firmware's separate event registry at
`0x1fff7f34` stores `0x1102ddd8` for event ID 6, but it is not the ROM slot at
`0x1fff0038`; the saved post-button RAM snapshot has zero in the latter slot.
The ROM wrapper therefore does not explain the missing diagnostic line, and
its actual purpose remains unidentified. Do not bridge it to the event
registry without a captured caller and compatible argument record.
PC traces of the same external-ROM, no-coordinator button scenario recorded no
execution at either `0x27e8` or `0x7f1c` during the first press window
(5.8–6.8 s) or the second (6.8–7.8 s). This rules out that ROM wrapper path for
the observed GPIO8 presses in this run; it does not identify the application's
separate callback dispatcher.

A 15-second run traces the receive wrapper at `0x1103d1f0`. The startup routine
`0x11020890` polls event IDs `0x7001`, `0x7002`, and `0x7003` successfully, then
finds `0x7005` empty. Those four reads all happen around cycle 14.46 million,
before the GPIO8 press; there are no later receive-wrapper calls during the
run. At 15 seconds the queue descriptor still has write offset `0xc94`, and
the callback at `0x1102ddd8` has not run. The startup poll reads before the
button press and does not establish how this persistent record is consumed at
runtime. The callback route remains a separate unresolved path.

The local button also posts three volatile task messages through
`0x11040b60` and `0x110425f8`. A post-press RAM snapshot at `0x1fffc574`
retains entries for message types `0x0c`, `0x0d`, and `0x0a`; each points to
`0x11043eb5`. These retained table entries are not evidence that the messages
remain unconsumed: a PC trace shows the scheduler invokes callback
`0x1102f915` for all three IDs. For the first message (`0x0a`), that callback
queues `0x11035cdd`, which runs at cycle 32,172,850 (about 1.01 seconds); it
builds a request using the message object and queues the follow-up callback
`0x1103a079`, which also runs. A four-second replay produces no TS0001 radio
frame: the pcap contains only the coordinator's beacon requests. The volatile
task-message callback sequence is running, but its work does not reach an
observed over-the-air transmission in this run.

Disassembly shows `0x1104263c` registers callback/context pointers in the
`0x1fffc574` table, and its only direct firmware callsite is the startup path
at `0x11040a02`. The adjacent routine at `0x11042674` scans a context table
and can call `0x11043ee4`, but a PC trace over `0x1104263c..0x11042712` from
0.9 to 4 seconds has no hits. It is not the runtime dispatcher for the
button's volatile task messages. This is separate from the flash-backed
`0x7001` event queue: its post-press consumer or wakeup path remains
unidentified, and the `0x1102ddd8` `On`/`Off` callback still does not run.

Separately, `0x110219b4` reaches `0x11021f34`, which builds a command for
cluster `0x0006`. Its lookup in `0x1102e1f4` finds the endpoint-1/cluster-6
entry in the table at `0x1fffc0f8` and selects descriptor `0x1fffc178` with
index 0 and marker 3. The special branch through `0x1102cb98` reads bytes
`+6..+7` from entry 0 of the per-index table at `0x1fffb408`; both are zero.
That `0x11021f34` route consequently skips `0x11043bf0`.

A separate PC trace of the physical-button route corrects the earlier broader
observation: `0x1102de60` does call `0x11043bf0`. It builds a local cluster-6
record, sets byte `+0x0b` to `1`, then passes that record to the common send
function. The function submits through `0x11040cb8` only when the high nibble
of byte `+0x0b` is nonzero; with the button's value `1`, it returns before that
submission. The related SDK does not document this private record flag, so the
trace explains where this firmware path stops but does not establish a missing
SoC or ROM contract. The registered `0x1102ddd8` diagnostic callback remains
unreached, and the pcap still contains no TS0001 transmission. Do not force
relay state or callback execution from GPIO input alone.

For a separate first-boot probe, a temporary copy of the TS0001 dump had its
`0x6000`-byte NVM range at `0x11079000` erased. The image reported
`Device started for the first time` with network state 0. Holding GPIO8 from
1 s to 8 s reports `KEY 1 longPress` and `zb bdb devNetOpt = 0`; with the
supplied NVM contents, the same input reports option 7. In the erased-NVM
case, the `devNetOpt = 0` path selects channel 11 (`0x400300b4 = 5`). An
earlier trace attributed a 1512-us `set_timer` call to this callback, but that
interpretation used preloaded SRAM bytes which are replaced by the runtime
stack image described below. A direct trace of ROM `set_timer` in the
8-second first-boot run records Timer1 with `time=0x25a` (602 us) and callback
`0x1fff49d9`. The callback's RF register effects still need to be checked
against the decoded runtime image. The PHY+ SDK
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
updates the state to 1 and invokes that callback. The callback's code was
initially analyzed from the package's preloaded SRAM bytes, which is incorrect
for this address at runtime. A write trace shows PC `0x1ffff52e` decoding a
16 KiB runtime stack image into SRAM `0x1fff4000..0x1fff7fff`, from the XIP
stream beginning at `0x11011808`. The captured image is a scratch analysis
artifact and is not part of the repository. It contains the dispatcher,
callback code and state table; its bytes at `0x1fff4a00..0x1fff4aff` happen to
match full-flash bytes `[0x149d0:0x14ad0]`, but they are part of the decoded
runtime image. Runtime callback `0x1fff4a69` schedules Timer1 for
`0x25a` microseconds with callback `0x1fff49d9`. That timer callback runs in
IRQ20; its current trace then queues event work through `0x1fff4cb4`. The
previous claim that the first callback itself calls `LL_Rand` and arms Timer1
with `0x1fff49d9` came from stale bytes and is withdrawn.

Timer1 IRQ20 reaches the registered image handler `0x1fff2565`; a trace shows it
setting event-group bit `0x20`, after which PendSV runs. The observed timer
callback changes channel and syncword, samples RSSI, and starts another SRX
around 6.411 s. Its event work does not lead to the next table row
`(1, 1, 2, 0x1fff4acd)` in the traced sequence.

The ROM `ll_hw_read_rfifo_zb` trap was entered three times by IRQ4 during the
8-second first-boot run, so received packets reached the closed stack's FIFO
read path. A capture of the helper's output contains a MAC Beacon Request
(command ID `0x07`), an encrypted Zigbee NWK broadcast on PAN `0xa75c`, then
another Beacon Request. The run's air pcap also contains a MAC beacon from that
PAN at 6.756687 s with `Association Permit` set. A 1 ms radio-mode trace sees
the PHY6252 receiving at 6.300 s, then `radio_receiving` turns off at 6.412 s;
direct samples around 6.756 s remain off. The beacon therefore arrives outside
the firmware's RX window and is not passed to the FIFO helper.

A follow-up first-boot run held GPIO8 from 1 s through 8 s, then sampled the
radio mode every millisecond through 20 s. After release it saw no further RX
window and no more `ll_hw_read_rfifo_zb` calls. The same capture contains live
coordinator/sampleLight traffic, including the light's association and NWK
exchange, while the TS0001 emits no frame. This narrows the failure to the
closed stack not scheduling or keeping its receive path active after the
initial commissioning callback; the shared air and another Zigbee node remain
active.

The same run has no TX FIFO writes and no `STX` or `TRX` mode request, so the
modeled transmitter never receives a request. The evidence points to the
closed stack's receive/commissioning progression: its next table row
`(1, 1, 2, 0x1fff4acd)` is not reached, RX has stopped by the time the
coordinator advertises association, and the stack never asks the PHY to send.
It does not yet distinguish a missing ROM callback contract from a stack-state
or timing issue earlier in the scan. No later TX operation starts in the LL
trace, and the 20 s pcap has 20 gateway frames but no TS0001 frame. The image
resumes its main loop around 8.0 s. The saved-NVM 15 s run likewise has only
19 gateway frames and no TS0001 frame. All runs continue without a CPU fault.
The transition from the commissioning callback path to an outbound Zigbee
frame remains unknown. In a separate 20 s run,
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
Repeating this first-boot network run with both local module eFuse and RF
calibration inputs still produced 76 frames: `sampleLight` joined at 9.045 s
and exchanged 9 uplink (5 unicast) and 6 downlink NWK frames, while the pcap
had no TS0001 source (`0x87db`). Missing calibration inputs therefore do not
explain the relay's lack of TX in this commissioning scenario.

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

An earlier scratch test treated the trailing `0x87DB` in the UART line as the
saved PAN ID and set both the MAC PIB and NWK NIB PAN ID to `0x87DB` after
formation, while remaining on channel 25. Later NVM inspection shows that this
value is more likely the relay's short address and identifies `0xA9D9` as the
candidate PAN ID. The earlier pcap had 75 frames;
the transmitted data frames use PAN `0x87DB`, sampleLight associated at 9.045 s
as `0x3d34`, and it exchanged 9 uplink and 7 downlink NWK data frames. TS0001
still transmitted no frame. That run did not match the decoded candidate PAN
and cannot establish the relay's network identity. The 10-second candidate
identity run above also had no TS0001 transmission, so its saved parent state
or another stack initialization issue remains unresolved.

A pre-ABI-fix LL trace of a same-channel first-boot run saw 317
`ll_hw_set_srx` calls and 317 radio triggers: 302 on channel 11 and 15 on
channel 26, with no `STX`, `TRX`, or `RTX` calls and no TX FIFO data. That run
used the earlier host trap, which preserved the interrupted `r0` instead of
matching the supplied ROM's callback call. Do not treat its repeated-receive
count as behavior of the current wrapper model. The stronger three-variant
comparison above confirms the current run also emits no TS0001 frame, but did
not capture an LL trigger trace for each argument variant. The remaining gap is
in the proprietary stack's transition from commissioning/RX state to its first
TX request; the tested callback argument and the modeled TX/ACK path have not
been shown to cause the stall.

After implementing the PHY+ `TRX` response window, a current 15-second
saved-state run with the sample gateway still captured only 19 gateway frames
and no TS0001 transmission. A temporary trace at the LL trigger found no `STX`
or `TRX` operation from the relay in this run. The missing frame therefore
occurs before the model's TX/ACK operation is requested; implementing `TRX`
alone does not unblock the saved-state network path.

An instruction trace initially appeared to show the proprietary stack losing
its FreeRTOS event mask. The LMAC task waits on event group `0x1fffd2a0`; the
deferred `0x20` event wakes it once and its BDB dispatcher moves from state 0
to state 1. Later, Timer1 runs the image callback `0x1fff49d9`, which calls
`0x1fff4cb4(1, 1)` and queues `0x1fff65d5(group, 0x20)`. The queued callback
runs, but the first trace showed the waiter still linked with item value `2`.

The follow-up identified a Cortex-M scheduling defect behind that stale item.
The image's FreeRTOS yield writes `ICSR.PENDSVSET`; the core had continued to
the next thread instruction before entering PendSV, so
`uxTaskResetEventItemValue` ran while the task was still linked on the event
list. The Cortex-M core now ends the translated block at that yield boundary.
A regression test verifies that PendSV observes the state before the next
thread instruction. A new TS0001 trace shows the second `0x20` event matching
the waiter value `0x82000020`, unlinking the waiter, and resuming the task;
the priority key is reset only after the wait returns.

This scheduler fix does not yet make the closed image transmit. A 20-second
first-boot run with an active channel-11 coordinator and sample light still
captured no TS0001 frame, with or without the supplied external ROM dump. The
other nodes exchanged 9 uplink and 6 downlink NWK frames. The current
no-ROM LL trace still has no TS0001 TX FIFO writes or `STX`/`TRX` request, so
the remaining failure is after the corrected task wake and before a transmit
request reaches the modeled radio. Do not attribute this remaining gap to the
PHY transmitter or claim TS0001 network support yet.

The external-ROM TS0001 run also produced no UART output, while the same ROM
is verified with the independent PB-03F Zephyr image. This does not establish
that the 6222 dump matches the closed image's ROM ABI; keep the host-trap TS0001
boot result and the external-ROM Zephyr result as separate compatibility
evidence.

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
When those local inputs and the sampleGW image are present, CMake registers
`firmware_phy6252_zephyr_join`. Run it with
`ctest --test-dir build -R firmware_phy6252_zephyr_join --output-on-failure`;
it checks the Zephyr boot/join console and successful association plus
bidirectional NWK traffic in the pcap. The binaries remain external local test
inputs and are not included in the repository.

`--boot-from-rom` or YAML `boot_from_rom: true` starts at the external ROM's
reset vector instead. This is an experimental ROM-analysis mode: with the
supplied dump and TS0001 image it remains in ROM (PC `0xac52`) for a one-second
run and emits no UART output. It does not yet hand off to the image, so use the
default direct-image path for the verified Zephyr compatibility scenario.
Firmware inputs can be the package beginning at physical flash offset `0x2000`
or a 512 KiB full flash dump; keep local images outside the repository.
For the TS0001 capture setup, place the full dump at
`firmware/phy6252-ts0001.bin`, with the captured module data at
`firmware/phy6222-efuse.bin` and `firmware/phy6222-rfcal.bin`. Run
`examples/phy6252-ts0001.yaml` from a scratch working directory. The scenario
writes `air.pcap` there and probes GPIO8 and GPIO3; inspect both the UART output
and the captured frames.
To replay the supplied no-coordinator button context, run
`examples/phy6252-ts0001-offline.yaml`; it presses GPIO8 twice and writes only
the relay GPIO2 transition to `relay.vcd`. The scenario keeps GPIO3, the
firmware's second key input, pulled high at idle. The firmware toggles the local
relay state, but the `On/Off` diagnostic callback seen in the reference UART
remains unreproduced.
When the TS0001 image and local eFuse/RF-calibration dumps are present, CMake
also registers `firmware_phy6252_ts0001_boot`. Run it with
`ctest --test-dir build -R firmware_phy6252_ts0001_boot --output-on-failure`.
It checks the ordered startup, Tuya and GPIO key UART messages and a GPIO2
transition after the button press. It does not assert the uncertain `On/Off`
trigger or Zigbee network support; the captured-network criterion remains
separate.
