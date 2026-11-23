# STM32WB55

`stm32wb55` models the STM32WB55's two application cores in `sim`: CPU1 uses
the shared Cortex-M4 core at 64 MHz, and CPU2 executes with the emulator's
Cortex-M0 profile at 32 MHz. The two cores have independent CPU state and cycle
clocks, and map the same flash and SRAM storage. CPU1 flash is also visible at
address zero for the STM32WB boot remap used by firmware that sets `VTOR = 0`.
CPU1 starts CPU2 by setting C2BOOT. CPU2's vector
table is found in its configured raw image, including images that use the M0+
zero-based code alias, or in the built-in model ROM when the modeled Thread RCP
is selected.
While CPU2 is running, CPU1 yields at one-microsecond service intervals so the
two instruction streams observe shared SRAM and IPCC changes on a common virtual
timeline. IPCC changes request an earlier service.

## CPU2 image

Configure the CPU2 image separately from a compatible CPU1 host application:

```yaml
devices:
  - id: wb55
    mcus:
      - id: main
        model: stm32wb55
        firmware: ../firmware/stm32wb55-cpu1.bin
        cpu2_firmware:
          - file: ../firmware/stm32wb5x_Phy_802_15_4_fw_1.25.0.bin
            offset: 0xDD000 # STM32WB5x PHY 802.15.4 v1.25.0, 1 MiB part
```

`cpu2_firmware` accepts the same `file`, `offset`, `from`, and `size` fields as
`firmware`. The configured bytes are stored in shared flash, and CPU2 begins
executing when CPU1 enables C2BOOT. For a conventional raw image, `offset` is
its physical flash offset and the vector table points into `0x08000000` flash.
Some STM32WB CPU2 images instead use the M0+ code alias at `0x00000000`: their
vector table contains low addresses even though the bytes are installed in the
secure flash region. Those images are mapped at zero in CPU2's address space;
SRAM2 is also available through its `0x10000000` alias. For example, the
STM32WB5x PHY 802.15.4 firmware v1.25.0 for a 1 MiB device is installed at
physical address `0x080DD000`, so configure `offset: 0xDD000`.
That PHY image is a coprocessor firmware, not a standalone application: ST pairs
it with STM32CubeMonitor-RF or a dedicated CPU1 application ([UM2288](https://www.st.com/resource/en/user_manual/dm00433646-stm32cubemonitorrf-software-tool-for-wireless-performance-measurements-stmicroelectronics.pdf)).
The decrypted Thread RCP v1.13.0 image for a 1 MiB STM32WB5xxG is installed at
`0x080DA000`, so configure `offset: 0xDA000`. Its reset vector uses CPU2's
zero-based code alias, which the loader maps to the installed bytes.
In a boot-only host experiment that merely sets C2BOOT, PHY v1.25.0 executes
through startup and then parks at `0x90ae`: its pointer validator rejects entry 0
of the empty shared reference table at `0x20030000` and writes the `0x3de96f61`
failure sentinel there. This confirms CPU2 alias execution, but not PHY
initialization; a compatible CPU1 host must populate the IPCC/mailbox table.

The PHY reads `FLASH_IPCCBR.IPCCDBA` and uses
`0x20030000 + 4 * IPCCDBA` as the table base. With the modeled default
`IPCCDBA = 0`, this is the usual CubeWB `TL_RefTable` location. Its ten entries
point, in order, to the device-info, BLE, Thread, system, memory-manager,
traces, 802.15.4 MAC, Zigbee, LLD-tests, and BLE LLD tables. During startup the
PHY validates the 0x20-byte device-info table and also consumes system command
and queue pointers, a memory pool and its size, a trace queue, and LLD-test
buffers. The CPU1 side must initialize the linked-list queue heads as well as
the pointers; a zero-filled table or just a valid entry 0 is insufficient.
CubeWB's transport does this through `TL_Init`, `TL_SYS_Init`, `TL_MM_Init`,
and the relevant stack-specific table initializers.

A scratch boot with a synthetic reference table, valid pool and self-linked
system queue advanced the raw PHY past `0x90ae` and into its low-power `WFI`
path near `0xf7e0`. This verifies the model's shared SRAM path and the PHY's
early mailbox-dependent startup. It does not initialize the 802.15.4 MAC or
provide the CPU1 command service needed for on-air operation; the matching
Thread/Zigbee host stack is still required for that.

The emulator expects executable bytes with a Cortex-M vector table. It does not
unpack encrypted STM32 wireless-firmware packages distributed for the Firmware
Upgrade Service (FUS); those packages do not contain directly executable CPU2
bytes. ST documents the wireless-stack images as encrypted and signed for FUS in
[AN5185](https://www.st.com/resource/en/application_note/dm00513965-st-firmware-upgrade-services-for-stm32wb-series-stmicroelectronics.pdf).

CPU2 runs at 32 MHz with independent CPU state and a cycle clock. Its physical
core is Cortex-M0+, while the current Unicorn profile is Cortex-M0; this covers
the image's common M0 instruction set but does not model M0+-specific core
behavior. CPU2 shares flash and SRAM with CPU1 and accesses the same IPCC status
and masks. While both cores are active, the simulator schedules CPU2 at
one-microsecond intervals against the same virtual timeline; IPCC updates can
request an earlier service. Different raw CPU2 firmware images can be loaded
per simulation and execute using the M0 profile. A synthetic zero-based image
is checked for C2BOOT startup and shared SRAM2 access. The
chip model also provides basic RCC, PWR, HSEM, RNG, and LPTIM register behavior.
The Thread RCP's IRQ19 handler reads `C1TOC2SR` and clears a received channel
through `C2SCR`. A low-half `C2SCR` write clears `C1TOC2SR`, while its high
half sets `C2TOC1SR`. CPU1's `C1SCR` performs the reciprocal operation: it
clears `C2TOC1SR` and sets `C1TOC2SR`. The two status directions are kept
independent, and synthetic firmware tests cover both receive-side clears.
The PHY-used PWR clear registers are modeled as write-one-to-clear commands:
`PWR_SCR` clears matching `PWR_SR1` flags, and `PWR_EXTSCR` clears CPU stop and
standby flags or the critical-radio-phase flag. The status bits themselves are
only raised by modeled hardware events; radio-originated PWR flags stay clear
until the RF event model can produce them.
RCC clock-prescaler writes set the corresponding `HPREF`, `C2HPREF`,
`SHDHPREF`, `PPRE1F`, and `PPRE2F` applied flags. This lets the STM32WB HAL
finish its clock setup; the model treats application as immediate and does not
simulate prescaler transition latency.
The CPU2 APB3 RF-control aperture is mapped. PHY v1.25.0's indirect register
command through `0x6000000c` reads or writes a byte register array using a
descriptor in SRAM2 and clears the command busy bit. This command completes
synchronously because its hardware latency is unknown. The official Thread RCP
polls bit 7 at `0x600013ec`; the model returns that ready bit while the 802.15.4
window is clocked. The decompiled PHY also probes the RF bus by writing `0xa5a5`
to `0x6000000c` after setting bit 0 at `0x60000404`; the model exposes the probe
value only while that bit is set. These readiness and power semantics are
inferred from firmware control flow.

The 16-bit MAC register window and its write-one SET/CLR aliases are available
only while `RCC_C2APB3ENR.802EN` is set. Disabling that clock hides stored values
and ignores writes, while preserving register contents. The current radio path
models the Thread RCP's TX byte FIFO and immediate/delayed transmit commands,
places transmitted MPDUs on the shared `VirtualAir`, and delivers completed
frames to the RX FIFO with calculated FCS, header fields, RSSI, event flags, and
IRQ8/IRQ22 mask behavior. `R008` reports the PSDU length including FCS; the FIFO
contains those PSDU/FCS bytes without the PHY length byte. The 16-us MAC counter
and compare flags are also modeled. `R080` and the built-in Spinel `PHY_RSSI`
property estimate instantaneous RSSI from the air noise floor and frames
currently in flight on the selected channel. CPU2 RSSI registers encode
`dBm + 100` in bits 11:4, matching the RCP's inferred raw-sample conversion;
`R07c` remains latched to the most recent received frame. This supports
software RSSI-based CCA probes, but RF calibration, receiver settling, hardware
CCA/CSMA, collision feedback to this MAC, and hardware-accurate SFD/header/end
timing are not yet modeled. RX event flags are raised together when a complete
air frame is polled.
The direct register mapping, state values, and these behaviors are inferred from
the decrypted firmware and `firmware-src/stm32wb55-cpu2/REGISTERS.md`, not from
an ST register specification. As observed in the PHY reset helper and confirmed
by the RCC register definitions,
`RCC_APB3RSTR.RFRST` resets the modeled RF register window, gates its accesses,
and drives `RCC_CSR.RFRSTS` until reset is released.
Unimplemented peripheral registers read as zero or retain writes.

### Inferred PHY register transport

RM0434 divides the CPU2 APB3 window into BLE control (`0x60000000`), radio
control (`0x60000400`), a reserved hole (`0x60000800..0x60000fff`), and
802.15.4 control (`0x60001000..0x60001fff`). The reference manual gives the
window boundaries but not the RF register map.

The decrypted PHY v1.25.0 code shows a descriptor command at `0x6000000c`:
bits 0–15 contain an offset from SRAM2A (`0x20030000`), and bit 16 is polled as
a busy flag. The descriptor contains a length/direction byte, a register index,
data bytes, and a trailing zero. Its register helper accepts indices below
`0x7d` and transfers up to `0x7d` bytes. The emulator models this indirect
register transport with a 126-byte register array. This mapping is inferred
from the decrypted image, not documented by ST. Direct register accesses at
`0x60001000 + 4 * index` use only their low 16 bits. The model also implements
the documented-in-`REGISTERS.md` observed SET/CLR aliases, command/FIFO ports,
event masks, and radio state dependency: indirect RF register `0x5c` reads as
`0x1a` when the RF bus is powered and direct register `R100.0` is set, `0x04`
when powered with RX disabled, and zero while unpowered. The names and state
mapping are inferred from the PHY/RCP code.

The PHY reads the FLASH option registers `OPTR`, `IPCCBR`, `SFR`, and `SRRVR`
while validating shared-memory pointers and locating the CPU2 reset vector.
For a raw CPU2 image installed at a nonzero flash offset, the model assumes the
image is in the secure wireless-firmware area: `OPTR.ESE` is set, `SFR.SFSA`
comes from the image's install page, `SRRVR.SBRV` points at its vector table,
`SRRVR.C2OPT` selects flash boot, and both SRAM2 banks are marked secure. The
image offset and these option-byte defaults are inferred model configuration;
they are not recovered from a board's option-byte dump. `IPCCBR` remains zero,
which places the host reference table at `0x20030000` as in the ST mailbox
initialization code.

The decompilation also shows these direct accesses, without enough context to
assign reliable hardware names:

| Address | Observed use in PHY v1.25.0 |
| --- | --- |
| `0x60001100` (index `0x40`) | Updates masks `0x2000`, `0x1000`, and `0x00c0`; another helper reads bit 15. |
| `0x60001194` (index `0x65`) | Stores a value in the low 14 bits and a separate 2-bit field at bits 14–15; one caller clamps a field to `0x26..0x32` and places another in bits 8–13. |
| `0x60000400` | Helpers set or clear bit 15 and bit 0 with read-modify-write operations. |
| `0x60001500`, `0x60001900` | A helper writes `0x8000` to one of these addresses depending on its argument. |

These are access patterns, not confirmed semantic names. In particular, the
`0x8000` writes may be commands or write-one-to-clear operations; generic
readback does not reproduce those side effects.

When `cpu2_firmware` is absent, CPU1 starts the built-in Thread RCP model. Its
IPCC and Spinel behavior is serviced in CPU2's 32 MHz time domain while the M0
core runs a wait loop; this mode lets the Zephyr CPU1 image join the example
network and exchange Zigbee data. When a raw CPU2 image is configured, the
built-in RCP model is disabled and that image executes on CPU2 instead. The
encrypted ST package remains unsupported by the raw-image loader, but a
decrypted Thread RCP v1.13.0 image loads as executable CPU2 code. Its Ghidra
decompilation confirms the same SRAM2 descriptor command used by PHY v1.25.0:
bit 16 starts the command, bit 7 of the descriptor header selects reads, and
the remaining bytes carry the register index and data. The model tests the
RCP's one-byte write/read sequence as well as multi-byte PHY transfers.

A 30-second simulation with this RCP image and the Zephyr CPU1 image captured
the association request and successful response at 7.30 s and 7.80 s. The
capture also contains NWK traffic in both directions. CPU1 then faults at
14.922 s on a halfword write to address `0x00000000` from PC `0x08029834`, so
long-running operation is not established. The RX FIFO length correction was
identified from the driver's Spinel `STREAM_RAW` format: the old model exposed
the PHR byte through `R3e8`, shifting the frame and leaving the final FCS byte
unread. The driver source is in the Zephyr checkout at
`drivers/ieee802154/ieee802154_stm32wb.c`.

The Zephyr source used for the join run also needed a null-index guard in
`subsys/zigbee/nwk/nwk_addr_map.c`: the end-device leave path calls
`zb_ExtAddrByShortAddr` with a null `idx`, while that function previously wrote
through it unconditionally. With that guard, the Zephyr CPU1 image joined the
TLSR8258 sample gateway at about 7.8 seconds and exchanged bidirectional NWK
traffic in the 24-second capture through the built-in CPU2 Thread RCP model. The example is
[`../examples/stm32wb55_zigbee.yaml`](../examples/stm32wb55_zigbee.yaml); it
needs the Zephyr CPU1 image and TLSR8258 coordinator image. Add
`cpu2_firmware` when testing a raw CPU2 image that can run without the
built-in Thread RCP compatibility model.
