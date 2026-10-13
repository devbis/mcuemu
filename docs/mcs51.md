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
- UART1 transmit capture from `S1BUF`, with frame timing derived from `S1REL`,
  `TI1` raised at transmit completion, write-one-to-clear status flags, and
  interrupt dispatch to the CA51F253L3 vector `0x0033`.
- `PWMUPD` write requests clear immediately after the modeled PWM shadow
  register update.
- Timer0 mode 1 counting from MCS-51 core cycles, setting `TF0`, and dispatching
  vector `0x000B` when `EA` and `ET0` are enabled. Timer0 counts only when
  `TR0=1`, `C/T0=0`, `GATE0=0`, and mode 1 is selected.

Most SFR bytes currently behave as storage. UART0 transmit and PWM updates
complete immediately; UART1 uses the documented-by-firmware `FOSC/32` reload
formula for its 8N1 frame duration. This timing is driven by emulated core
cycles, not calibrated wall time. UART1 receive is modeled: bytes pushed by the
host (a UART link in a system) queue up to 16 deep and are presented one at a
time through `S1BUF` with `RI1` set, raising the UART1 interrupt (vector
`0x0033`) when `EA` and `ES1` allow; writing `RI1` back (write-one-to-clear)
brings the next byte. There is no bit-level timing for reception: the sender's
byte rate is what spaces the bytes. UART0 receive, other interrupt sources, other
timer modes, GPIO electrical behavior, and sleep are not modeled yet. The CPU
backend accepts a part-specific interrupt vector from its SoC wrapper while
retaining the standard Timer0 vector. Timer0
receives one increment per MCS-51 core cycle, which
matches the firmware's timer setup assumptions but is not a calibrated wall
clock. Other reset values and peripheral semantics still need CA51F253L3
documentation or hardware traces.

## Execution engine

The interpreter (`helper.c` in the Unicorn fork) used to go through the soft MMU
for every register access and ended each translated block after one
instruction, which made it about 100x slower than real time. The host now hands
it direct pointers to the code, IDATA and SFR arrays (`UC_MCS51_REG_FAST_MEMORY`)
and it runs stretches of instructions inside one helper call:

- SFRs without side effects (ACC, B, PSW, SP, DPTR, ports, ...) are plain memory.
  Only the registers the host must see, listed in `sfr_special_read/write`
  (UART buffers and control, PWMUPD, Timer0 control, mode and counter, IE), go
  through the mapped SFR region and its callbacks.
- Timer0 and the UART1 transmitter are lazy. The host computes the cycle of the
  next event and sets `UC_MCS51_REG_EVENT_CYCLES`; the engine stops before the
  first instruction at or after it, the host catches the peripherals up and
  restarts. They are also caught up before any special SFR access, so a read of
  TL0/TH0 or a write to IE sees the state of the current cycle.
- `Ca51F253L3::run(instruction_limit, cycle_limit)` stops at a cycle limit, at
  most one instruction late, which is how `Ca51F253L3Mcu::run_until` follows the
  world clock. `UC_MCS51_REG_BREAK_PC` stops at the UART1 vector so the pending
  interrupt can be re-armed, as the per-instruction hook used to do.
- With a trace stream set, the old path is used: a code hook sees every
  instruction. A test runs the same programs on both paths and compares the
  complete state, so tracing does not change behaviour.

## Running

```sh
cmake -S . -B build
cmake --build build --target mcs51-emu
./build/mcs51-emu run --model CA51F253L3 \
  --firmware firmware/application.hex --steps 1000000 --trace --state
```

`--trace` writes each executed PC and its instruction bytes to stderr. The
runner reports executed instruction count, backend cycle count, and final PC.
`--state` prints CPU, timer, IDATA, and stack state. Captured UART0 and UART1
output is printed after the CPU summary.

## Extending toward CC2530

Keep CPU behavior in Unicorn and add a separate CC2530 machine model in the
root project. That wrapper should define the device's code banking, RAM and SFR
maps, interrupts, timers, radio, and clock behavior. Do not encode these SoC
details into `UC_ARCH_MCS51`; another 8051-compatible part should be able to
reuse the same core with a different memory and peripheral model.

The backend's instruction execution is based on the classic 8051 instruction
set. Enhanced or vendor-specific opcodes should be added only after identifying
the exact opcode semantics from the applicable device/compiler documentation.
