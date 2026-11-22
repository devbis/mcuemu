# STM32WB55

`stm32wb55` models the STM32WB55's two application cores in `sim`: CPU1 uses
the shared Cortex-M4 core at 64 MHz, and CPU2 executes with the emulator's
Cortex-M0 profile at 32 MHz. The two cores have independent CPU state and cycle
clocks, and map the same flash and SRAM storage. CPU1 starts CPU2 by setting
C2BOOT. CPU2's vector
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
The PHY-used PWR clear registers are modeled as write-one-to-clear commands:
`PWR_SCR` clears matching `PWR_SR1` flags, and `PWR_EXTSCR` clears CPU stop and
standby flags or the critical-radio-phase flag. The status bits themselves are
only raised by modeled hardware events; radio-originated PWR flags stay clear
until the RF event model can produce them.
The CPU2 APB3 RF-control aperture is mapped. PHY v1.25.0's indirect register
command through `0x6000000c` is also emulated: it reads or writes a byte register
array using a descriptor in SRAM2 and clears the command busy bit. This command
completes synchronously because its hardware latency is unknown. Direct
802.15.4 control registers at `0x60001000..0x60001fff` retain masked writes, but
are accessible only while `RCC_C2APB3ENR.802EN` is set; disabling that clock hides
the stored values and ignores writes, while preserving register contents. Their
radio side effects, RF timing, and packet transfer are not modeled. As observed
in the PHY reset helper and confirmed by the RCC register definitions,
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
from the decrypted image, not documented by ST. The separate direct register
accesses at `0x60001000 + 4 * index` are preserved as ordinary masked
read-modify-write storage until their side effects can be identified.

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
official ST Thread RCP package was checked, but it is FUS-encrypted and is
rejected by the raw-image loader. CPU2's BLE, radio, and 802.15.4 control windows
are mapped at `0x60000000`; the inferred PHY indirect-register command is
modeled as described above. Other direct registers currently retain ordinary
writes and do not model RF timing or radio behavior. An arbitrary wireless
stack running from a raw CPU2 image cannot perform on-air operations without a
matching radio model.

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
