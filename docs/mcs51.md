# MCS-51 implementation notes

## Current model

`engine/unicorn` contains a separate `UC_ARCH_MCS51` CPU backend. The root
project wraps it with `mcs51emu::Ca51F253L3` and the `mcs51-emu` CLI.

The CA51F253L3 shell currently provides:

- 64 KiB 16-bit CODE window; firmware images are limited to 32 KiB and the
  unused flash bytes read as erased `0xff`.
- 64 KiB XDATA window and separate IDATA and SFR windows.
- 8051-compatible reset values for SP and the four standard port latches.
- Raw binary and Intel HEX loading with record checksums and flash bounds.
- Instruction count, architectural cycle count, PC, and optional instruction
  trace output.
- A 512-byte data flash region with the SDK-used read, write, erase, and lock
  commands at XDATA `0xFC01..0xFC06`.
- UART0 transmit capture from `S0BUF`, with immediate `TI0` completion so
  firmware using a polled UART debug routine can continue.
- `PWMUPD` write requests clear immediately after the modeled PWM shadow
  register update.
- Timer0 mode 1 counting from MCS-51 core cycles, setting `TF0`, and dispatching
  vector `0x000B` when `EA` and `ET0` are enabled. Timer0 counts only when
  `TR0=1`, `C/T0=0`, `GATE0=0`, and mode 1 is selected.

Most SFR bytes currently behave as storage. UART0 transmit and PWM updates
complete immediately; baud-rate and PWM timing are not modeled. UART receive,
other interrupt sources, other timer modes, GPIO electrical behavior, and sleep
are not modeled yet. Timer0 receives one increment per MCS-51 core cycle, which
matches the firmware's timer setup assumptions but is not a calibrated wall
clock. Other reset values and peripheral semantics still need CA51F253L3
documentation or hardware traces.

## Running

```sh
cmake -S . -B build
cmake --build build --target mcs51-emu
./build/mcs51-emu run --model CA51F253L3 \
  --firmware firmware/application.hex --steps 1000000 --trace --state
```

`--trace` writes each executed PC and its instruction bytes to stderr. The
runner reports executed instruction count, backend cycle count, and final PC.
`--state` prints CPU, timer, IDATA, and stack state. Captured UART0 output is
printed after the CPU summary.

## Extending toward CC2530

Keep CPU behavior in Unicorn and add a separate CC2530 machine model in the
root project. That wrapper should define the device's code banking, RAM and SFR
maps, interrupts, timers, radio, and clock behavior. Do not encode these SoC
details into `UC_ARCH_MCS51`; another 8051-compatible part should be able to
reuse the same core with a different memory and peripheral model.

The backend's instruction execution is based on the classic 8051 instruction
set. Enhanced or vendor-specific opcodes should be added only after identifying
the exact opcode semantics from the applicable device/compiler documentation.
