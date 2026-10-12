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

SFR bytes currently behave as memory. Peripheral side effects and reset values
specific to CA51F253L3 have not been added. Interrupts, timers, UART, GPIO
electrical behavior, and sleep are also future work. This keeps uncertain chip
behavior out of the CPU core until it can be tied to a manual or measurements.

## Running

```sh
cmake -S . -B build
cmake --build build --target mcs51-emu
./build/mcs51-emu run --model CA51F253L3 \
  --firmware firmware/application.hex --steps 1000000 --trace
```

`--trace` writes each executed PC and its instruction bytes to stderr. The
runner reports executed instruction count, backend cycle count, and final PC.

## Extending toward CC2530

Keep CPU behavior in Unicorn and add a separate CC2530 machine model in the
root project. That wrapper should define the device's code banking, RAM and SFR
maps, interrupts, timers, radio, and clock behavior. Do not encode these SoC
details into `UC_ARCH_MCS51`; another 8051-compatible part should be able to
reuse the same core with a different memory and peripheral model.

The backend's instruction execution is based on the classic 8051 instruction
set. Enhanced or vendor-specific opcodes should be added only after identifying
the exact opcode semantics from the applicable device/compiler documentation.
