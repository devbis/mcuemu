# mcuemu

A microcontroller and device emulator built on a [Unicorn](https://www.unicorn-engine.org/)
fork. It runs real firmware images for several MCU families and connects them
into whole systems: you describe devices, wiring, peripherals, and a shared
radio medium in YAML, and mcuemu runs them on one virtual timeline.

- **System layer (`sim`).** YAML-described systems of devices, nets, parts
  (built-in and Lua), a shared radio medium (Zigbee-style frames), endpoints and
  sinks (UART, VCD, pcap, files, TCP), and a scenario runner.
- **Control API.** `mcuemu serve` exposes a JSON-RPC 2.0 API over WebSocket for
  driving and inspecting a running system, with optional realtime pacing.
- **Chip models.** Each MCU is a separate model on top of a Unicorn CPU backend
  (TC32, MCS-51 and the Cortex-M interrupt machinery are provided by the fork). Models map flash, SRAM, MMIO,
  virtual time, and interrupts around the engine; adding a chip means adding a
  model, not changing the system layer.

## Tested MCUs

| MCU | Core | Status |
| --- | --- | --- |
| Telink TLSR8258 (`TLSR8258F512`, `TLSR8258F1K`) | TC32 | GPIO, timers, UART (incl. DMA), ADC, AES, sleep/wake, Zigbee RF; vendor Zigbee and Zephyr firmware tested |
| Cmsemicron CA51F253L3 | MCS-51 | CPU, Timer0, UART1, PWM; no GPIO, ADC or radio |
| Nordic nRF52840 (Pro Micro / nice!nano board) | Cortex-M4F | NVIC, SysTick, FPU, WFE; clocks, GPIO/GPIOTE, UARTE, TWIM, SAADC, TIMER, RTC, RNG, ECB, WDT, NVMC, PPI; 802.15.4 RADIO; the Embassy Zigbee sensor image joins a TLSR8258 coordinator; no BLE yet |

Support is partial and driven by the firmware tested so far. See
[`docs/status.md`](docs/status.md) and the per-chip docs.

## Documentation

- [`docs/system-config.md`](docs/system-config.md) — YAML system format.
- [`docs/control-api.md`](docs/control-api.md) — RPC methods and pacing.
- [`docs/scripting.md`](docs/scripting.md) — Lua parts.
- [`docs/architecture.md`](docs/architecture.md) — architecture notes.
- [`docs/tlsr8258.md`](docs/tlsr8258.md) (usage, firmware tests, status),
  [`docs/tlsr8258-register-model.md`](docs/tlsr8258-register-model.md),
  [`docs/mcs51.md`](docs/mcs51.md),
  [`docs/nrf52840.md`](docs/nrf52840.md) (Cortex-M core, peripherals, status) —
  chip models.
- [`examples/`](examples/) — ready-made systems (`mcuemu sim examples/world.yaml`).

## Running a system

```sh
./build/mcuemu sim examples/thermostat_only.yaml --for 10s
./build/mcuemu serve --root . --port 8080
```

Sink outputs (`*.log`, `air.pcap`, `*.vcd`) are written to the current
directory, so run experiments from a scratch directory.

## Build and test

Requirements: CMake 3.20+ and a C++20 compiler. The Unicorn fork checkout must
be present in `./engine/unicorn` (see [Unicorn checkout](#unicorn-checkout)).

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Firmware-level scenarios (Zigbee join, button/light, Zephyr PM, vendor sleep) are
registered by CMake when the local images are present in `firmware/`; see
[`docs/tlsr8258.md`](docs/tlsr8258.md). Run them after any change to CPU execution,
timing or peripherals. `firmware/` is local-only and ignored by git;
`mcuemu models` lists the supported flash capacities.

## Project layout

- `engine/unicorn/` — separate Unicorn fork checkout (TC32 and MCS-51 backends,
  Cortex-M NVIC).
- `emulator/include/` and `emulator/src/` — chip models, CPU adapters, radio,
  C API, and CLI; `mcs51/` holds the CA51F253L3 model, `cortexm/` the generic
  Cortex-M core, `nrf52/` the nRF52840 and `sim/` the system layer.
- `tests/` — tests grouped by layer (`tc32`, `machine`, `radio`, `mcs51`,
  `nrf52`, `sim`, `firmware`).
- `examples/` — system YAML, device descriptions, and Lua parts.
- `docs/` — reference documentation.
- `firmware/` — local-only vendor firmware images.
- `build/` and `tmp/` — local build output and temporary files.

## Unicorn checkout

The source is cloned into `engine/unicorn/` and is built as a dependency by
the top-level CMake project. To bootstrap a new checkout:

```sh
./scripts/bootstrap-unicorn.sh
```

The script clones the fork `https://github.com/devbis/unicorn.git` over HTTPS and
records it as `origin`; official Unicorn is kept as `upstream`. Maintainers can
push over SSH with
`git config --global url."git@github.com:".pushInsteadOf "https://github.com/"`.
The engine checkout is ignored by the root project so its
changes stay in the Unicorn repository.

The implementation sequence and boundary between Unicorn and the board layer
are described in [`docs/unicorn-integration.md`](docs/unicorn-integration.md).
The local TC32 architecture reference is in
`../llvm-tc32-arm-based/tc32-architecture`.
