# TLSR826x (TLSR8269 / TLSR8267)

Support for Telink's older TC32 family, next to the TLSR8258. The model is
`mcuemu::Tlsr826x` (`emulator/src/tlsr826x.cpp`), driven by the same `Cpu`
wrapper through the `Tc32Device` interface (`emulator/include/mcuemu/tc32_device.hpp`);
the sim adapter is `sim::Tlsr826xMcu` and the models are `tlsr8269` (32 KB SRAM)
and `tlsr8267` (16 KB SRAM) in `mcuemu run --model TLSR8269|TLSR8267` and in `sim`.

There is no datasheet for this family in the project. Every register comes from
the Telink SDK headers (`platform/chip_826x/register.h` of `telink_zigbee_sdk`, also
`llvm-tc32-arm-based/drivers-8269/sdk/chip_826x/`), the SDK's boot files
(`platform/boot/826x/`) and the disassembly of its driver library
(`libdrivers_826x.a`, `drivers-8269/sdk/asm/*.asm`). Comments in the source label
each fact `SDK`, `DISASM`, `INFERRED` (same as the 8258, not seen for the 826x) or
`SIMPLIFIED`.

## What differs from the TLSR8258

| | TLSR826x | TLSR8258 |
|---|---|---|
| SRAM | 32 KB (8269) / 16 KB (8267) at `0x808000` | 64 KB at `0x840000` (32 KB retained) |
| Register window | `0x800000..0x807fff` (ends where the SRAM starts); no `0x1000000` alias | `0x800000`, 256 KB, plus the `0x1000000` alias |
| DMA block | `0x500..0x52b` (UART RX/TX = channels 0/1, RF RX/TX = 2/3) | `0xc00..0xc43` |
| DMA address registers | 16 bit (`0x500` UART RX, `0x504` UART TX, `0x508` RF RX, `0x50c` RF TX); the SRAM is `0x80xxxx` | 16 bit + high byte |
| TX trigger | bit 1 (UART) / bit 3 (RF) of `0x524` | `0xc24` |
| RX done | `0xf20` bit 0 (RF IRQ status) and `0x526` bit 2 (channel status, `RF_RxBufferRequest`) | `0xf20` bit 0 |
| RF frequency | 16-bit `0x4d6` = 2400 + 5 * (zigbee channel - 10) MHz | `0x1244` in quarter MHz |
| UART | DMA only (no data FIFO), 16-bit clock divider at `0x94`, control at `0x96`/`0x98`, status `0x9d`/`0x9e` | FIFOs at `0x90..0x93` plus DMA |
| 32 kHz tick | read through `0x754` after a `0x74f` bit 3 latch (`PM_Get32kTick`) | analog registers `0x40..0x43` |
| Hardware divider | `0x660..` as on the 8258, not in `register.h` but used by the SDK's `div_mod.S` in the 826x Zigbee build (INFERRED semantics) | `0x660..` |

Same as the 8258: the boot header ("KNLT" at flash offset 8, RAM-code size in
the next word, the head is copied to the SRAM bottom), the single IRQ vector at
`0x10`, IRQ numbering (timers 0..2, DMA 4, UART 6, ZB RT 13, GPIO 18, system
timer 20), GPIO ports at `0x580` with a stride of 8, timers at `0x620..`, system
timer at `0x740..`, MSPI flash at `0x0c/0x0d`, analog access at `0xb8..0xba`, and
the Zigbee TX buffer layout (`ZB_RADIO_DMA_HDR_BUILD` is identical in the SDK); the RX buffer differs (13-byte header, RSSI at p[4], MAC frame at p[13]).

## Modelled

IRQ mask/enable/source, timers 0..2 (mode 0: the timer stops at the capture value,
as the 8258 datasheet says; firmware restarts it by writing the tick), the 16 MHz
system timer with its compare IRQ and START/STOP, the 32 kHz tick and the 32 kHz
calibration handshake of `PM_WakeupInit`, GPIO ports A..E with interrupts and pull
resistors, the analog register interface (including the `rc_32k_cal` handshake),
the DMA UART (TX from a `u32 length + payload` buffer, RX closed by an idle
timeout), the MSPI flash (command set as the 8258 model, JEDEC `ef 40 13`), and
the Zigbee radio on the shared `VirtualAir` (TX and RX through the DMA buffers,
channel from `0x4d6`, RSSI, timestamp, CRC flag, RF IRQ and ZB RT IRQ).

## Not modelled

Deep sleep and wake by pad, watchdog, ADC, I2C, SPI, PWM, USB, audio, BLE
modes of the RF block, clock-tree effects of `0x66` (the core clock is the
`clock_hz` option, default 32 MHz), the fitted costs of the 8258's flash instruction-cache timing (`flash_timing`, on by default in `sim` as for the 8258, because the 826x link script reserves the same IC tag RAM; INFERRED, not refitted). The chip-ID registers (`0x7c..0x7e`) read 0.
The JEDEC id and the 512 KB flash are the 8258 model's values, not measured.

## Tests

`tests/machine/test_tlsr826x.cpp` (memory map, CPU, timers, GPIO, flash, UART DMA,
analog, radio exchange between two chips) and `tests/tlsr826x/` (a firmware built
from the SDK's `cstartup_826x.S` and `libdrivers_826x.a`: `uart_Init`, `uart_Send`,
a timer IRQ; ctest `firmware_tlsr826x_smoke`, skipped when the SDK or TC32 LLVM is
missing; run `tests/tlsr826x/build.sh` for the image). `examples/tlsr8269_smoke.yaml`
runs it in `sim`. No Zigbee firmware for the 826x has been run yet, so the RF path
is checked against the SDK's driver disassembly and the 8258 model, not against a
join.

## Toolchain notes

* This LLVM TC32 build encodes the SDK's `tmrcs` (read CPSR) as `tmrss` (read SSR);
  `div_mod.S` assembled with it puts the core into mode 0. `build.sh` assembles
  `cstartup_826x.S` with the vendor gcc 4.5 (`telink/tc32/bin/tc32-elf-gcc`) when
  present, and supplies its own software division helpers (`divmod.c`).
* `boot_826x.link` needs two parenthesised sums for `ld.lld`; `build.sh` patches a
  copy.

## Vendor SDK end device (joins the TLSR8258 coordinator)

The Telink Zigbee SDK `sampleSwitch` (end device) built for the TLSR8269 joins the
sampleGW coordinator in `sim`: scan, Association Request/Response, Transport Key, Device
Announce, then data requests and NWK traffic both ways, all ACKed. Build it with the CI
CMake (`tests/ci/telink`):

    cmake -S tests/ci/telink -B build-sw8269 -DAPP=sampleSwitch -DTELINK_PLATFORM=826x \
          -DSDK_PREFIX=<telink_zigbee_sdk>/tl_zigbee_sdk -DTOOLCHAIN_PREFIX=<tc32 gcc 4.5>
    cmake --build build-sw8269 --target sampleSwitch.bin
    cp build-sw8269/sampleSwitch.bin firmware/samplesw_8269.bin

`tests/firmware/scenarios/zigbee_tlsr8269_ed.yaml` and the ctest
`firmware_tlsr8269_zigbee_join` (needs that image and `samplegw_tb03f.bin`) check the join from
the air capture. Getting there needed, besides the radio, these blocks (all found by running
the image, evidence labels in the source): the hardware divider at `0x660` (the SDK's
`div_mod.S`), suspend sleep woken by the 32 kHz compare (`pm_sleep_wakeup`, `pm_start`:
`0x6f = 0x81`, compare at `0x754` latched by `0x74f` bit 3, wake status in AREG `0x44`),
a quiet-channel RSSI (`0x440`, otherwise the MAC's CCA is always busy and nothing is ever
sent), and the AES block (`0x540..0x55f`, same protocol as the 8258). Deep sleep and wake
by pad are not modelled; the image only suspends while a timer is pending.

## Zephyr end device (build-fp-ed-8269), not on hardware

`telink/llvm-tc32-arm-based/build-fp-ed-8269/zephyr/zephyr.bin` (Zephyr zigbee ED
with `ieee802154_tlsr8269.c`) boots and runs on the model: the kernel ticks from the
system-timer compare IRQ, the radio initialises, the device scans, receives the
sampleGW beacon, and completes the association (Association Response, short address
assigned) in a `sim` world with the TLSR8258 coordinator. It then polls with Data
Requests but does not get the coordinator's data frames: the driver advertises only
`IEEE802154_HW_FCS` (no hardware ACK handling) and the stack switches the radio off about
350 us after its own TX, before the coordinator's ACK has finished; the coordinator sends
its frame (Transport Key) without being heard and repeats it four times. The behaviour
varies with CPU timing (one poll in a 40 s run was answered). This is the image's driver
logic against the model's timing; it has not been compared with silicon.
