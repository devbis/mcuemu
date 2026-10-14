# AGENTS.md

Guidance for AI coding agents working in this repository. Read it before
changing anything; it records decisions and pitfalls that are not obvious from
the code. Human-facing docs are in `README.md` and `docs/`.

## What this project is

`mcuemu` is a microcontroller emulator built on a Unicorn fork. It models four
chips and a declarative system layer on top:

- **TLSR8258** (Telink, TC32 core, 2.4 GHz radio: Zigbee/BLE-style frames): board
  model in `emulator/src/` (`cpu.cpp`, `tlsr8258*.cpp`, `radio.cpp`).
- **CA51F253L3** (Cmsemicron, MCS-51 core): `emulator/src/mcs51/`.
- **nRF52840** (Nordic, Cortex-M4F): generic core glue in `emulator/src/cortexm/`
  (`cortexm::Core`), the chip in `emulator/src/nrf52/` (`nrf52emu::Nrf52840`).
  See `docs/nrf52840.md`.
- **EFR32MG21** (Silicon Labs, Cortex-M33): the chip in `emulator/src/efr32/`
  (`efr32emu::Efr32mg21`) on the same `cortexm::Core`. See `docs/efr32mg21.md`.
- **`sim` layer** (`emulator/src/sim/`, `emulator/include/sim/`): YAML-described
  systems of devices, nets, parts (built-in and Lua), a shared radio medium,
  endpoints and sinks, a scenario runner, and a JSON-RPC 2.0 control API over
  WebSocket (`mcuemu serve`).

Naming: the executable is `mcuemu`; the TC32/TLSR8258 core library is
`mcuemu-core` (C++ namespace `mcuemu`, headers in `emulator/include/mcuemu/`,
C API prefix `mcuemu_`); the MCS-51 library is `mcs51emu` (namespace
`mcs51emu`); the Cortex-M core is `cortexm` and the nRF52840 chip `nrf52emu`
(namespaces of the same names); the system layer is `simcore` (namespace `sim`). The repository
folder may still be called `tc32-emu`; never hard-code absolute paths.

## Layout

```
emulator/include/{mcuemu,mcs51emu,sim}/   public and internal headers
emulator/src/                              TLSR8258 + CPU wrapper, CLI main, C API
emulator/src/mcs51/                        CA51F253L3 model and its `mcuemu run` runner
emulator/src/cortexm/                      generic Cortex-M core (NVIC glue, SysTick, WFI/WFE, cycle clock)
emulator/src/nrf52/                        nRF52840: peripherals, FICR/UICR, `mcuemu run` runner
emulator/src/efr32/                        EFR32MG21: CMU/HFXO/GPIO/MSC blocks, the radio, `mcuemu run` runner
emulator/src/sim/                          system layer (config, world, rpc, server, sinks, parts)
engine/unicorn/                            Unicorn fork: SEPARATE git repo, git-ignored here
tests/{tc32,machine,radio,mcs51,nrf52,efr32,sim,firmware}/
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

- The build type defaults to `Release`; an unoptimised build is about 5x
  slower (the firmware tests take minutes). Pass `-DCMAKE_BUILD_TYPE=Debug` to debug.
- Test binaries: `mcuemu-tests` (TC32 core, machine, radio), `mcs51emu-tests`,
  `nrf52emu-tests` (Cortex-M core and the nRF52840 peripherals), `efr32emu-tests`
  (the EFR32MG21), `simcore-tests` (the whole system layer, about 9 s).
  `ctest -E firmware` runs just those five.
  The nrf52 test programs are assembly in `tests/nrf52/programs/*.s`; after
  editing one, run `tests/nrf52/programs/build.py` (needs `arm-none-eabi-as`) and
  commit the regenerated `tests/nrf52/programs.hpp`.
- Firmware tests (`firmware_zigbee_join`, `firmware_zigbee_button_light`,
  `firmware_zephyr_pm_wake`, `firmware_vendor_sleep_1s`) exist only when the
  images are in `firmware/` at configure time (`samplegw_tb03f.bin`,
  `samplelight_tb03f.bin`, `zephyr-pmtest.bin`, `vendor_sleep_1s.bin`) and
  Python 3 is available. They take about 80 s together. `firmware_nrf52840_boot`
  (image `nrf52840-sensor.bin`) takes under a second;
  `firmware_nrf52840_zigbee_join` and `firmware_efr32mg21_zigbee_join` (images
  `nrf52840-sensor-sht31.bin` / `efr32mg21-sensor.bin`, plus `samplegw_tb03f.bin`)
  run the two-chip networks in about 15 s each. **Run them after any
  change to CPU execution, timing or peripherals**: they are the only check that
  catches subtle timing regressions across nodes.
- **CI firmware tests** (`.github/workflows/firmware-tests.yml`, runs after
  "Build mcuemu" succeeds, Ubuntu): every image is built from source or
  downloaded, then one job per scenario runs the emulator; scenarios do not
  block each other. Details and the policy for new images are in
  `docs/firmware-ci.md`. Prefer building from source; commit an image to
  `tests/firmware-images/` only when it has no buildable source. Zigbee scenarios
  (`tests/firmware/scenarios/*.yaml`, `tests/firmware/zigbee_air_check.py`) pass
  when the device joins and the two sides exchange NWK data, judged from the
  pcap, not from RAM addresses. UART and RTT output must reach the job log
  (sinks `hex`/`text` or `--trace-uart`). These scenarios also run locally:
  put the images in `firmware/` under the names the scenario yaml uses.
- Run ctest from the repository root with `--test-dir build`; running it from
  elsewhere creates a stray `Testing/` directory.
- After renaming or moving the folder, delete `build/` and reconfigure (the
  CMake cache stores the absolute source path).
- Firmware images are local and ignored; never commit `*.bin`, `*.hex`, `*.elf`.
  The examples expect `firmware/osrctherm_r02_zrd.bin`,
  `firmware/osrctherm_r02_v1.02_CA51F253L3.bin`, `firmware/samplegw_tb03f.bin`,
  `firmware/tlsr8258-plug.bin` and `firmware/nrf52840-sensor.bin`.
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
- On Apple Silicon the JIT area is flipped between writable and executable
  with `pthread_jit_write_protect_np`, and that was 45% of a TLSR8258 run. It
  now stays executable between translation blocks (`cpu_tb_exec`), and
  `JIT_CALLBACK_GUARD` restores the mode only when a callback changed it. Every
  JIT writer must unlock the area itself and put back the state it found
  (`tb_gen_code`, `tb_add_jump`, `tb_flush`, `tb_phys_invalidate` do).
- `uc_emu_start(..., count)` is unreliable after other runs have translated
  blocks: with `count=1` it executed dozens of instructions. The TC32 wrapper
  never uses `count`; it limits execution from its own code hook.
- On Apple Silicon each entry into translated code toggles JIT write protection
  (`pthread_jit_write_protect_np`), which is expensive. That is why both cores
  run many instructions per `uc_emu_start`.

- The ARM backend is stock Unicorn plus our additions. Its Cortex-M exception
  model was dead code (`v7m_exception_taken` and `do_v7m_exception_exit` returned
  immediately, the NVIC was commented out), so `nvic.c` and the revived
  `m_helper.c` paths are ours, as are `uc_arm_nvic()` and `uc->guest_exception`.
  `UC_MODE_MCLASS` used to force Cortex-M33; an explicit M-profile model is now
  honoured. Without `uc->guest_exception` every M-profile exception (even SVC)
  would stop the engine with `UC_ERR_EXCEPTION`.
- The Cortex-M33 (EFR32MG21) is the stock model run without its Security
  Extension: `uc_arm_nvic(UC_ARM_NVIC_NO_SECURITY)` clears the feature and sets
  `NSACR`, and `v7m_exception_taken` sets `EXC_RETURN.ES` only for secure targets.
  Without these, FP stacking and the first exception return fault (HardFault with
  `CFSR` zero or `UFSR.INVPC`). The nvic models one (non-secure bank) state.
- On Cortex-M the code hook sees every instruction; WFI/WFE are decoded there
  and must never reach the engine (it would raise EXCP_YIELD, which the fork
  treats as an invalid instruction).

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

### Cortex-M core and nRF52840 (`emulator/src/cortexm/`, `emulator/src/nrf52/`)

`docs/nrf52840.md` is the reference. Rules that are easy to break:

- Time is core cycles. Peripherals compute their state from `core.cycles()` on
  demand (counters) and schedule only events: return the next due cycle from
  `service()` and call `request_service()` from register writes that move it
  earlier. A peripheral that ignores this works until the core sleeps and the
  clock jumps past the event.
- A new peripheral raises its interrupt level through `set_irq_level()` (the
  line is level-sensitive, like the hardware) and routes events through
  `event_raised()` so PPI sees them. Unimplemented peripherals are
  `StubPeripheral`s: they read back what was written and never raise events.
- Flash is read-only to the CPU; stores arrive in the write-protect handler
  (`Nvmc::on_store`) and are fixed up by `Core::defer`. Invalidate translated
  code (`Core::invalidate_code`) only when the program image itself changes.
- The generic core knows nothing about the nRF52840: keep chip facts in
  `nrf52/` so other Cortex-M chips can reuse `cortexm/`.

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

Rough speed on an Apple Silicon laptop: a TLSR8258 coordinator running the
Telink sample gateway (a busy main loop) simulates 30 s in about 15 s, twice as
fast as real time; one nRF52840 mostly asleep runs hundreds of times faster, and
a spinning Cortex-M core executes about 175 million instructions per second.
Profile with `sample <pid> 3 -file out.txt` rather than guessing. What mattered:

- **The TLSR8258 device clock is lazy inside a batch** (`Cpu::code_hook`,
  `Tlsr8258::cycles_until_event()`). Instructions only add to `owed_cycles_`;
  the device is ticked for the whole span when a conservative bound is reached,
  before every MMIO access and at the end of the batch. The result is the same as
  ticking every instruction (checked: pcap, VCD and RTT output of the two-chip
  example are byte-identical to the per-instruction version), but only if the
  bound never overestimates. **When a tick subsystem gets a new time-driven
  effect, add it to `cycles_until_event()`**, or return 0 while it is active.
  A state the batch checks before each instruction (power state, pending IRQ) must
  be re-read after the tick that can change it, which `code_hook` does.
- Instruction bytes are read from the host copy of flash/SRAM, not through
  `uc_mem_read`.
- The fork keeps the Apple JIT area executable between translation blocks and does
  not re-flip it around hooks; see the fork facts above.

## Known limitations

- No BLE at register level; CA51 has no GPIO/ADC/radio; no TLS on the control API.
- UART timing for the CA51F253L3 RX path is not bit-accurate.
- The plug example uses the pins of the Zbeacon TS011F board. Without a coordinator the plug firmware only scans and requests steering, so the relay does not react to the button.
- Realtime pacing cannot reach 1x for busy firmware; use `speed` below 1.
- nRF52840: the RADIO is 802.15.4 only (no BLE), no SPIM/PWM/CCM/USBD; one
  cycle per instruction; lazy FP stacking is not modelled (eager stacking is
  architecturally invisible).
- Stage 8 of the original plan (Python host/client, snapshot/restore) is not
  started and has not been approved.
