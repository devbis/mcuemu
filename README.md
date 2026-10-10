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

Run a raw flash image from address zero:

```sh
./build/tc32-emu run --model TLSR8258F512 --firmware firmware/sample.bin --steps 100000
```

The CLI can trace instructions and MMIO accesses to stderr. Ranges are
inclusive, and each trace stream is capped at 10,000 events by default:

```sh
./build/tc32-emu run --firmware firmware/sample.bin --steps 100000 \
  --trace-pc-range 0x0:0x200 --trace-mmio-range 0x8000b8:0x8000ba \
  --trace-limit 1000
```

Run multiple raw images against one virtual Zigbee medium with a shared model
and channel. Each node advances in short instruction batches, and the command
prints the frames observed in the medium:

```sh
./build/tc32-emu network \
  --node coordinator=firmware/coordinator.bin \
  --node light=firmware/sampleLight.bin \
  --model TLSR8258F512 --channel 15 --steps 1000000
```

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
- Zigbee RF TX/RX DMA is bridged to the packet-level virtual-air model, with
  RF completion flags and IRQ state. Firmware channel writes to the RF
  synthesizer registers select the Zigbee channel; the CLI channel remains the
  initial setting until firmware programs the synthesizer. BLE packet
  transport exists but is not wired to TLSR8258 RF.

This is an early emulator, not yet a complete vendor-firmware environment. TC32
instruction coverage and TLSR8258 peripheral behavior still need validation
against hardware. A local Zigbee `sampleLight` image ran 100 million
instructions without a CPU fault after adding the instruction-fetch alias for
its code in the SRAM cache window. This verifies continued execution, but does
not yet verify firmware-driven packet exchange. In a two-node run the image
emits 8-byte Zigbee frames on channels 11 and 12, then faults while returning
from a task-queue function with its stack pointer at the end of mapped SRAM.
The board models can also exchange a test MAC payload through RF DMA and
virtual air.

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
