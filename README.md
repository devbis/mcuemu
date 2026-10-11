# TC32 / TLSR8258 emulator

This project layers a TLSR8258 board model over Unicorn. TC32 instruction
execution and register state live in Unicorn's `UC_ARCH_TC32` backend; the
wrapper maps the chip's flash, SRAM, MMIO, virtual time, and interrupts around
that engine. The Unicorn checkout under `engine/unicorn/` remains its own Git
repository. Board code and public headers live under `emulator/`.

## Build and run

Requirements: CMake 3.20+ and a C++20 compiler. The TC32-enabled Unicorn source
checkout must be present in `./engine/unicorn`.

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

If `firmware/samplegw_tb03f.bin` and `firmware/samplelight_tb03f.bin` are
present when CMake configures, CTest also registers firmware-level Zigbee
scenarios. Run the join test directly with:

```sh
ctest --test-dir build -R firmware_zigbee_join --output-on-failure
```

It starts the light five seconds after the coordinator, then checks that the
light's SDK joined flag is set and that both association MAC commands were
successfully received by the other node. The images remain local and ignored by
Git. Reconfigure with `cmake -S . -B build` after adding the images if CMake had
previously skipped this test. Python 3 is needed for this optional test.

`firmware_zigbee_button_light` extends the same boot and join sequence. It
presses sampleGW SW1 (active-low PD6) after the light joins, checks that the
light receives the resulting Zigbee traffic, and checks one complete
sampleLight PWM transition from off (`0x00`) to on (`0x07`) and back to off.
The test accounts for the SDK's startup configuration, which turns the light
on: the coordinator's first On command leaves it on, then later Off/On/Off
commands exercise the full output cycle. Run this scenario with:

```sh
ctest --test-dir build -R firmware_zigbee_button_light --output-on-failure
```

If `firmware/zephyr-pmtest.bin` is present when CMake configures, CTest also
registers `firmware_zephyr_pm_wake`. It checks four repeated Zephyr deep-sleep
entries and timer wakeups, using the local firmware image. Configure again
after copying the image into `firmware/`, then run:

```sh
ctest --test-dir build -R firmware_zephyr_pm_wake --output-on-failure
```

Run a raw flash image from address zero:

```sh
./build/tc32-emu run --model TLSR8258F512 --firmware firmware/sample.bin --steps 100000
```

The CLI can trace instructions, MMIO accesses, and stack or mode changes to
stderr. Ranges are inclusive, and each trace stream is capped at 10,000 events
by default. `--trace-after-step` delays traces until the selected instruction
count; `--fault-trace` prints a ring buffer of recent instructions if the CPU
faults. Network runs can restrict traces to one node with `--trace-node`:

```sh
./build/tc32-emu run --firmware firmware/sample.bin --steps 100000 \
  --trace-pc-range 0x0:0x200 --trace-mmio-range 0x8000b8:0x8000ba \
  --trace-limit 1000
```

For example, to trace the IRQ vector on one node and retain recent instructions
for a possible fault:

```sh
./build/tc32-emu network \
  --node coordinator=firmware/coordinator.bin \
  --node light=firmware/sampleLight.bin --steps 30000000 \
  --trace-node light --trace-pc-range 0x220:0x250 \
  --trace-stack --trace-after-step 1000000 --fault-trace 64
```

Run multiple raw images against one virtual Zigbee medium with a shared model
and channel. Each node advances in short instruction batches, and the command
prints transmitted frames and successful packet receptions:

```sh
./build/tc32-emu network \
  --node coordinator=firmware/coordinator.bin \
  --node light=firmware/sampleLight.bin \
  --model TLSR8258F512 --channel 15 --steps 1000000
```

Add `--radio-payloads` to include raw frame bytes in the output when examining
MAC discovery and commissioning traffic. `--dump-sram NAME=ADDRESS:LENGTH`
prints a selected range from one node's SRAM after execution, which is useful
for checking firmware state against local ELF symbols. `--dump-mmio` prints a
range from the TLSR8258 register window after execution. `--trace-outputs`
prints changes to GPIO output latches and the PWM enable-channel mask with
virtual timestamps. It reports register-level output state; PWM pulse waveforms
are not yet simulated.

Use `--start-at NAME=TIME` to power on a node at an absolute virtual time while
all nodes and the radio share one timeline. `--steps` sets the overall run
horizon; a delayed node receives only the remaining time after it starts. For
example, start the coordinator first and the light at 5 seconds:

```sh
./build/tc32-emu network \
  --node coordinator=firmware/samplegw_tb03f.bin \
  --node light=firmware/samplelight_tb03f.bin \
  --model TLSR8258F512 --channel 15 --steps 200000000 \
  --start-at light=5s --radio-payloads \
  --dump-sram coordinator=0x84801d:4 \
  --dump-sram light=0x84783d:4
```

Use `--gpio-event NAME=PORT:VALUE@TIME` to drive an input port at a virtual
time. Ports accept `A`..`E` (or `0`..`4`), byte values accept decimal or `0x`
hex, and time accepts `ns`, `us`, `ms`, or `s`. On the sampleGW 8258 dongle,
SW2 is active low on port D bit 5. The board model starts inputs high for the
sample's pulled-up idle buttons; GPIO events take effect before the next
execution quantum.

`firmware/` is local-only. Images and build products are ignored by git.
`tc32-emu models` lists the supported flash capacities.

## Current scope

- Unicorn has a TC32 CPU backend and public C API identifiers for registers and
  CPSR flags. Its unit tests cover arithmetic, flags, branches, stack/register
  operations, PC-relative loads, and CPSR reads.
- The wrapper uses `uc_open`, `uc_reg_*`, `uc_mem_*`, `uc_mmio_map`, and
  `uc_emu_start`; it does not execute firmware in a second CPU interpreter.
- TLSR8258 flash and SRAM are mapped into Unicorn, and the current wrapper
  connects GPIO, timers, UART, analog registers, AES-128 ECB, the hardware
  divider, MSPI flash identification, virtual time, and interrupt state to the
  board model. CPU timing uses a coarse estimate of three cycles per
  instruction.
- The low-frequency tick counter follows the 32 kHz RC/crystal selector and
  oscillator power-down bits. CPU suspend and deep sleep use the 32 kHz timer
  compare and timer wake-source gate; suspend resumes at the next instruction,
  while deep wake resets volatile state and restarts at the flash vector. If
  the selected 32 kHz source is powered down, the node remains asleep and the
  CLI reports that state. GPIO wake sources and selectable SRAM retention
  modes are not modeled yet.
- Zigbee RF TX/RX DMA is bridged to the packet-level virtual-air model, with
  RF completion flags and IRQ state. Firmware channel writes to the RF
  synthesizer registers select the Zigbee channel; the CLI channel remains the
  initial setting until firmware programs the synthesizer. BLE packet
  transport exists but is not wired to TLSR8258 RF.

This is an early emulator, not yet a complete vendor-firmware environment. TC32
instruction coverage and TLSR8258 peripheral behavior still need validation
against hardware. A two-node run with local `sampleLight` and an SDK-built
`sampleGW` coordinator executed without a CPU fault. With the light started at
5 seconds, the medium recorded 70 Zigbee transmissions and 36 successful
receptions. It delivered a coordinator Beacon Response, the light's association
request, the coordinator's association response, and subsequent bidirectional
traffic. At 11.25 seconds, the SDK `zb_isDeviceJoinedNwk()` flag was set for
both nodes: the light's `g_zbNwkCtx + 0x2d` byte was `0x04` and the
coordinator's was `0x96` (`permit_join` and `joinAccept` are also set there).
The RX DMA model now includes the two FCS bytes counted by the vendor SDK's
DMA length field; omitting them caused the firmware to reject its Beacon
Response. This verifies joining for this firmware pair and channel, not every
commissioning path or peripheral. BLE remains deferred.

## Project layout

- `engine/unicorn/` — separate Unicorn Git checkout with the TC32 backend.
- `emulator/include/tc32emu/` and `emulator/src/` — board model, CPU adapter,
  radio, C API, and CLI.
- `tests/tc32/`, `tests/machine/`, and `tests/radio/` — tests grouped by layer.
- `firmware/` — local-only vendor firmware images.
- `build/` and `tmp/` — local build output and temporary files.

## Unicorn checkout

The source is cloned into `engine/unicorn/` and is built as a TC32-only dependency by
the top-level CMake project. To bootstrap a new checkout:

```sh
./scripts/bootstrap-unicorn.sh
```

Without `TC32_UNICORN_FORK_URL`, the script clones official Unicorn and records
it as `upstream`, leaving `origin` unset. Set `TC32_UNICORN_FORK_URL` when a
writable fork is available; the fork becomes `origin` and official Unicorn is
kept as `upstream`. The engine checkout is ignored by the root project so its
changes stay in the Unicorn repository.

The implementation sequence and boundary between Unicorn and the board layer
are described in [`docs/unicorn-integration.md`](docs/unicorn-integration.md).
The local TC32 architecture reference is in
`../llvm-tc32-arm-based/tc32-architecture`.
