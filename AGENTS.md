# AGENTS.md

Guidance for AI coding agents working in this repository. Read it before
changing anything; it records decisions and pitfalls that are not obvious from
the code. Human-facing docs are in `README.md` and `docs/`.

## What this project is

`mcuemu` is a microcontroller emulator built on a Unicorn fork. It models two
chips and a declarative system layer on top:

- **TLSR8258** (Telink, TC32 core, 2.4 GHz radio: Zigbee/BLE-style frames): board
  model in `emulator/src/` (`cpu.cpp`, `tlsr8258*.cpp`, `radio.cpp`).
- **CA51F253L3** (Cmsemicron, MCS-51 core): `emulator/src/mcs51/`.
- **`sim` layer** (`emulator/src/sim/`, `emulator/include/sim/`): YAML-described
  systems of devices, nets, parts (built-in and Lua), a shared radio medium,
  endpoints and sinks, a scenario runner, and a JSON-RPC 2.0 control API over
  WebSocket (`mcuemu serve`).

Naming: the executable is `mcuemu`; the TC32/TLSR8258 core library is
`mcuemu-core` (C++ namespace `mcuemu`, headers in `emulator/include/mcuemu/`,
C API prefix `mcuemu_`); the MCS-51 library is `mcs51emu` (namespace
`mcs51emu`); the system layer is `simcore` (namespace `sim`). The repository
folder may still be called `tc32-emu`; never hard-code absolute paths.

## Layout

```
emulator/include/{mcuemu,mcs51emu,sim}/   public and internal headers
emulator/src/                              TLSR8258 + CPU wrapper, CLI main, C API
emulator/src/mcs51/                        CA51F253L3 model and its own CLI (mcs51-emu)
emulator/src/sim/                          system layer (config, world, rpc, server, sinks, parts)
engine/unicorn/                            Unicorn fork: SEPARATE git repo, git-ignored here
tests/{tc32,machine,radio,mcs51,sim,firmware}/
examples/                                  world.yaml, thermostat_only.yaml, devices/, parts/
docs/                                      architecture, config reference, control API, mcs51, scripting
firmware/                                  local images only (ignored by git except .gitkeep)
scripts/bootstrap-unicorn.sh               fetches the Unicorn checkout
```

`docs/system-config.md` is the reference for the YAML format (devices, `include:`
with `params:`, nets, parts, links, attach, scenario, `firmware:` layouts).
`docs/control-api.md` documents the RPC methods and realtime pacing.
`docs/scripting.md` documents Lua parts. `docs/mcs51.md` documents the MCS-51
model and its execution engine. Keep docs in step with behaviour changes.

## Build and test

```sh
cmake -S . -B build        # needs engine/unicorn present; CMake errors out otherwise
cmake --build build -j8
ctest --test-dir build --output-on-failure
```

- Test binaries: `mcuemu-tests` (TC32 core, machine, radio), `mcs51emu-tests`,
  `simcore-tests` (the whole system layer, about 9 s). `ctest -E firmware` runs
  just those three.
- Firmware tests (`firmware_zigbee_join`, `firmware_zigbee_button_light`,
  `firmware_zephyr_pm_wake`, `firmware_vendor_sleep_1s`) exist only when the
  images are in `firmware/` at configure time (`samplegw_tb03f.bin`,
  `samplelight_tb03f.bin`, `zephyr-pmtest.bin`, `vendor_sleep_1s.bin`) and
  Python 3 is available. They take about 80 s together. **Run them after any
  change to CPU execution, timing or peripherals**: they are the only check that
  catches subtle timing regressions across nodes.
- Run ctest from the repository root with `--test-dir build`; running it from
  elsewhere creates a stray `Testing/` directory.
- After renaming or moving the folder, delete `build/` and reconfigure (the
  CMake cache stores the absolute source path).
- Firmware images are local and ignored; never commit `*.bin`, `*.hex`, `*.elf`.
  The examples expect `firmware/osrctherm_r02_zrd.bin`,
  `firmware/osrctherm_r02_v1.02_CA51F253L3.bin`, `firmware/samplegw_tb03f.bin`
  and `firmware/tuya_plug.bin` (the plug image is a placeholder path).
- `mcuemu sim` writes its sink outputs (`thermostat-*.log`, `air.pcap`,
  `*.vcd`) into the current directory. Run experiments from a scratch
  directory, or delete the files; do not commit them.

## The Unicorn fork (important)

`engine/unicorn/` is its own git repository and is listed in this repo's
`.gitignore`. Changes there are **not** captured by commits in this repo.
When you modify the fork (TC32 backend `qemu/target/tc32/`, MCS-51 backend
`qemu/target/mcs51/`, `include/unicorn/`), commit inside `engine/unicorn` as
well, and say so in your report: someone has to push that repo separately.

Facts about the fork that bit us:

- `uc_hook_add` does **not** flush translated blocks. Whether a block calls the
  code hook is baked in at translation time. If you add or remove a code hook
  while code is already translated, call `uc_ctl_flush_tb()`
  (`Ca51F253L3::set_code_hook_enabled` does).
- `uc_emu_start(..., count)` is unreliable after other runs have translated
  blocks: with `count=1` it executed dozens of instructions. The TC32 wrapper
  never uses `count`; it limits execution from its own code hook.
- On Apple Silicon each entry into translated code toggles JIT write protection
  (`pthread_jit_write_protect_np`), which is expensive. That is why both cores
  run many instructions per `uc_emu_start`.

## Architecture notes

### Time and scheduling (`sim/world.*`, `sim/scheduler.*`, `sim/mcu.hpp`)

`World::run_until(t)` advances in quanta (default 5 µs, `world.quantum` or
`--quantum`). Each MCU implements `Mcu::run_until(target_ns)` and may overshoot
the target by at most one instruction. Scheduled events (nets, parts, links,
radio, scenario) run between quanta. Virtual time is per-MCU cycle based; there
is no global instruction-count quantum in `sim` (the old `mcuemu network`
command still interleaves nodes in 256-instruction quanta; its behaviour must
not change, the zigbee firmware tests depend on it).

`Simulation::run`/`pump` implement realtime pacing (`speed` times wall clock as
an upper bound on virtual time; 0 means unthrottled; a lag over 50 ms of virtual
time is dropped).

### TC32 CPU wrapper (`emulator/src/cpu.cpp`)

`Cpu::run(limit, deadline_ns)` runs batches inside one `uc_emu_start`. The code
hook ticks the device (`device_.tick`, 3 cycles per instruction estimate) before
each instruction and, for every instruction after the first of a batch, stops the
batch **before** executing it when `batch_must_end()` says the wrapper has work:
a pending IRQ (not already in IRQ mode), the core leaving the awake state, a
halt, the instruction limit, or `network_time_ns() >= deadline`. This keeps the
order of "check IRQ, tick, execute" identical to single-stepping. Any change here
must keep the firmware tests green and `test_tlsr_batched_run_matches_single_steps`
passing.

### MCS-51 core (`emulator/src/mcs51/`, fork's `qemu/target/mcs51/`)

The interpreter lives in `helper.c`. The host passes pointers to code, IDATA and
SFR memory (`UC_MCS51_REG_FAST_MEMORY`), so ordinary register access bypasses
the soft MMU. Only SFRs flagged in `sfr_special_read_/write_` (UART buffers and
control, PWMUPD, Timer0 control/mode/counter, IE) go through the mapped SFR
region and its callbacks. **If you give an SFR a side effect, flag it as
special**, otherwise writes silently skip your callback.

With no code hook installed the translator emits `helper_uc_mcs51_run`, which
runs instructions until `UC_MCS51_REG_EVENT_CYCLES`,
`UC_MCS51_REG_INSTRUCTION_LIMIT` or `UC_MCS51_REG_BREAK_PC` is hit, then stops
the emulation. Timer0 and the UART1 transmitter are lazy: the host computes the
cycle of the next event (`schedule_events`), and `sync_peripherals` catches them
up at every stop and before any special SFR access. Trace mode
(`Ca51F253L3::set_trace`) uses the per-instruction hook path instead; a test
runs the same programs both ways and compares the full state, so keep the two
paths equivalent. Details in `docs/mcs51.md`.

### System layer (`emulator/src/sim/`)

- Config parsing is in `config.cpp` (yaml-cpp). `devices:` entries can be
  `{include: file.yaml, id?, params?}`; included files have `version: 1`,
  `params:` and a single `device:`; `${name}` is substituted at scalar level,
  unknown parameters are errors, errors are reported as `file: line N`, nesting
  is not supported. The server confines every included file, firmware, script and
  sink path to its `--root` (`Simulation::confine_path`).
- Sinks (`sink.cpp`): `stdout`, `text`, `hex[:GAP]`, `file:`, `tcp:`, `vcd:`,
  `pcap:`. `stdout`, `text` and `hex` are refused for remote clients.
  `mcuemu sim --trace-uart PATTERN[=hex|text]` attaches `hex` or `text` to
  matching UART endpoints.
- `firmware:` may be one file or a list of `{file, offset, from, size}`
  pieces (only `tlsr8258` accepts several). The Tuya thermostat example needs this
  because its ZT3L app image is linked for a bootloader the emulator does not run.
- Lua parts run in a sandbox with instruction and memory limits (`docs/scripting.md`).

### TLSR8258 model notes

Peripheral behaviour is documented in `docs/tlsr8258-register-model.md`; check
it before changing registers. Facts that mattered: UART DMA (rx/tx channels,
idle-timeout packet close) is modelled because SDK drivers use it; the ADC
supports the VBAT channel (`supply_mv`, default 3300) and pads driven by the chip
itself read as the rail; flash variants matter (`F1K` is needed for the ZT3L
image because the bootloader checks the JEDEC id). BLE register-level behaviour
is not implemented. The CA51F253L3 model has no GPIO, ADC or radio, so the
thermostat's display, keys, relay and NTC are not modelled.

## Conventions

- C++20, CMake. Match the surrounding style: no new dependencies, comments
  explain *why* (hardware evidence, inferred versus documented behaviour), not
  what. Existing source files state what is documented and what is inferred; keep
  doing that for register models.
- Code, comments, docs, tests and commit messages are in English. The maintainer
  has so far talked to agents in Russian.
- Tests use the small `CHECK` macro from `tests/test_support.hpp` (TC32 side) or a
  local one (mcs51, sim). New sim tests are plain functions registered in
  `tests/sim/test_sim.cpp`, declared before use.
- Commit messages: short imperative subject with an area prefix (`sim:`, `mcs51:`,
  `tc32:`), a body explaining the why. Commit only when asked.
- On macOS, BSD `sed -i` needs a suffix argument; prefer small Python scripts for
  bulk edits. `zsh` does not word-split variables, so use `bash -c` for command
  strings stored in variables.
- Do not report a change as done without having built it and run the relevant
  tests; say so explicitly when a test was skipped or could not run.

## Performance reference

Rough speed on an Apple Silicon laptop after the batching work: one CA51 runs
about 1.5x slower than real time; the full thermostat example (coordinator +
ZT3L + CA51) runs about 4x slower than real time. A busy-looping TLSR8258 at
48 MHz is still the dominant cost. Profile with `sample <pid> 3 -file out.txt`
rather than guessing.

## Known limitations

- No BLE at register level; CA51 has no GPIO/ADC/radio; no TLS on the control API.
- UART timing for the CA51F253L3 RX path is not bit-accurate.
- The smart plug example's firmware path and pin numbers are placeholders.
- Realtime pacing cannot reach 1x for busy firmware; use `speed` below 1.
- Stage 8 of the original plan (Python host/client, snapshot/restore) is not
  started and has not been approved.
