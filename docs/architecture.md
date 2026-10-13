# Architecture and implementation notes

## MCS-51 / CA51F253L3

MCS-51 instruction execution and registers live in the separate
`UC_ARCH_MCS51` backend in `engine/unicorn/`. The root project provides a
CA51F253L3 device shell and the `mcuemu run --model CA51F253L3`
runner. CODE, XDATA, IDATA, and SFR use distinct Unicorn address ranges so future device models can attach
register behavior without changing the CPU's 8051 address semantics.

The current CA51 wrapper loads raw or Intel HEX firmware into a 32 KiB flash
image and initializes common 8051 reset state. It is a CPU execution shell:
peripheral registers are storage only, and interrupt delivery, UART, timers,
GPIO electrical behavior, clocking, and sleep are not implemented yet. The
vendor [CA51F2 product overview](https://en.zhwssz.com/uploads/file/20221102/4e9369f760806204f362dcef2dc07175.pdf)
identifies the F253 variant's 32 KiB flash and 8051-compatible core. A complete
SFR manual is needed before assigning CA51-specific peripheral behavior.

The CPU API preserves a separate 16-bit CODE and 16-bit XDATA bus. The current
CA51 shell maps the full bus windows for deterministic access, while only the
first 32 KiB of CODE is loadable from firmware. Future CC2530 support should be
a separate device wrapper with its banked code mapping and SoC-specific
peripherals; it can reuse the CPU backend.

## Cortex-M / nRF52840

Cortex-M chips run on the fork's ARM backend through `cortexm::Core`, a generic
core wrapper (memory map, NVIC/SCS access, SysTick, WFI/WFE, cycle clock and a
service hook). A chip model implements `cortexm::Bus` for its peripheral windows
and calls the core with an absolute cycle limit; peripherals derive their state
from the cycle clock and ask for service only when an event is due, so an idle
chip costs nothing. Details and the fork changes are in
[`nrf52840.md`](nrf52840.md).

## Address space

The initial board exposes:

| Address range | Model |
| --- | --- |
| `0x000000` to flash size | Read-only program flash; 512 KiB or 1 MiB |
| `0x800000` to `0x83ffff` | 256 KiB register/MMIO window |
| `0x1000000` to `0x103ffff` | Firmware-observed compatibility alias for the register window |
| `0x840000` to `0x847fff` | 32 KiB SRAM retained in deep sleep |
| `0x848000` to `0x84ffff` | 32 KiB SRAM not retained in deep sleep |

These boundaries come from section 2.1.1 of the TLSR8258 datasheet. The model
maps the two SRAM retention classes contiguously; deep sleep preserves the
retained range and clears the non-retained range. The exact retention behavior
for every sleep mode still needs hardware validation.

The wrapper also configures a TC32 instruction-fetch alias starting at
`0x840000`, with a length equal to the selected flash capacity and source
address zero. Fetches in this range read the corresponding flash bytes; normal
data loads and stores continue to use SRAM. `sampleLight` returns from flash
to code addresses in this window, consistent with the chip's instruction
cache behavior. This is based on firmware execution and remains an inferred
hardware mapping.

The datasheet does not list the alias at `0x1000000`. It is mapped to the same
registers because the vendor firmware's `LoadTblCmdSet` computes addresses in
that range. Treat this as a compatibility inference until validated on silicon.

## CPU

The root project does not decode TC32 instructions. `Cpu` opens
`UC_ARCH_TC32` in Unicorn, maps board memory, accesses registers through
`uc_reg_*`, and executes through `uc_emu_start`. The Unicorn backend owns the
register state and implements the currently supported TC32 instruction
families, flags, branches, stack operations, memory forms, and status-register
operations. Invalid or unsupported encodings return a Unicorn error with the
faulting PC and instruction word.

The board adapter checks modeled interrupt state between Unicorn execution
steps, banks IRQ state through the TC32 register API, and enters the vector at
`0x10`. TC32 `treti {pc}` consumes the saved return PC from the active IRQ
stack, advances that stack, and restores CPSR from SPSR before resuming the
interrupted mode. Timer and UART are the only modeled interrupt sources so far.
The wrapper treats `0xcfxx` as a board-test semihosting stop marker; it is not
claimed as a TC32 ISA instruction.

Each executed instruction advances an estimated three modeled CPU cycles
through a Unicorn code hook. Unicorn exposes instruction hooks but not TC32
cycle counts; this estimate lets firmware pass its crystal-ready delay and has
not been calibrated against silicon. The board converts cycles to virtual
nanoseconds using its configured core clock, which defaults to the TLSR8258
maximum of 48 MHz. The system timer runs at its fixed 16 MHz rate. Timer compare
behavior is deliberately simple and should be refined against the datasheet
and firmware traces.

## Board and virtual radio

The register window defaults to storage for registers not modeled specially.
GPIO input bytes can be driven by the host; writes to GPIO output bytes update
observable port state. Timer tick registers advance while their enable bit is
set, set the status bit when a nonzero capture value is reached, and clear via
write-one-to-clear. UART models bounded RX/TX FIFOs, register counts, threshold
status, loopback, and transmit timing derived from its clock divider and frame
settings. Hosts can inject received bytes and drain transmitted bytes.
The model starts with TX_DONE set while idle to match the Telink SDK polling
API; the datasheet reset table lists zero for `0x9e`, so this initial status
needs confirmation against a hardware trace.

AES registers `0x540..0x55f` implement one-block AES-128 ECB encrypt/decrypt,
with the datasheet's 1000 system-cycle completion delay. CCM mode is not
implemented. The SDK-used divider register sequence at `0x660..0x66b` computes
signed and unsigned quotient/remainder and completes synchronously; its timing
is not modeled.

`VirtualAir` calculates packet duration from payload size, protocol overhead,
and bit rate. Receivers must match protocol and channel; overlapping
same-channel transmissions are discarded as collisions. The TLSR8258 Zigbee RF
path now reads TX buffers from DMA3, places received frames in DMA2 buffers,
updates RF completion status, and raises the modeled Zigbee IRQ. The DMA high
byte is relative to the `0x800000` memory window: a firmware value of `0x04`
with low address `0x29d4` resolves to SRAM address `0x8429d4`. The channel is
initialized by the host through `set_radio_channel`; firmware writes to
synthesizer offsets `0x1244` and `0x1245` then select the nearest Zigbee
5 MHz channel center. In the local vendor binary, `rf_set_channel` writes
quarter-MHz frequency units there (for example, `2450.25 MHz` selects channel
20). The `0x29` write to offset `0x0f16` is a separate RF control constant.
The medium does not parse Zigbee MAC/APS or BLE link-layer packets, and BLE is
not connected to TLSR8258 RF registers yet.

## Simulation core (`simcore`)

`emulator/include/sim/` holds the chip-independent layer that composes MCUs
into systems (see the declarative-configuration plan for the full roadmap):

- `Scheduler` — deterministic event queue over virtual nanoseconds.
- `Net`/`Pin` — electrical wiring. Each device terminal is a `Pin` attached to
  a `Net`; drives are `HiZ`/`Low`/`High`/`PullDown`/`PullUp`, nets are
  push-pull or open-drain (wired-AND for I2C/1-Wire), strong drivers that
  disagree are flagged as a conflict, and an analog source can set the net
  voltage for ADC inputs. Level changes carry the virtual time of the change.
- `Mcu` — interface the world uses (`run_until`, `connect_pin`, UART bytes,
  start offset). `Tlsr8258Mcu` wraps `Tlsr8258` + `Cpu` (pads `PA0..PE7`
  through the GPIO OUT/OEN registers, UART 0); `Ca51F253L3Mcu` wraps the
  mcs51 model (no GPIO yet, so no pads can be wired).
- `Endpoint`/`EndpointRegistry` — named observable points; every net is
  registered as a `NetEndpoint`. UART/RTT/LED/radio endpoints and sinks follow.
- `Session` and `Sink` — attach endpoints to stdout, files, TCP sockets,
  VCD waveforms and pcap captures. UARTs (`UartEndpoint`) and RTT
  (`RttEndpoint`, read from target memory) are endpoints too; UART links are
  built on them. The `led` and `button` parts give GPIO indicators and inputs
  a name.
- `Simulation` / `RpcSession` / `Server` — remote control. `Simulation` wraps a
  loaded system with run/pause, event subscriptions and a sandbox that confines
  files to a root directory; `RpcSession` is a transport-agnostic JSON-RPC 2.0
  dispatcher; `Server` is a single-threaded WebSocket server (hand-written
  RFC 6455 framing) with one isolated simulation per connection. See
  [control-api.md](control-api.md).
- `World` — owns MCUs, nets and the scheduler; `run_until` advances all MCUs
  in quanta (default 5 us) bounded by the next scheduled event, and jumps
  straight to the next event when no MCU is running.

- `Part`/`PartRegistry` — external peripherals attached to nets. Built in:
  `ds18b20` (1-Wire slave driven by timestamped edges), `ntc` (analog divider)
  and `i2c_regs` (register file behind the bit-level `I2cSlave` decoder), plus
  `script` parts whose behavior is a sandboxed Lua 5.4 file (`LuaPart`, see
  [scripting.md](scripting.md)).
  Part parameters are exposed as endpoints.

- Radio: `VirtualAir` is the medium. Besides the original ideal behavior it has
  a log-distance model with node positions, RSSI, sensitivity, SINR capture,
  half duplex and a shared frequency axis for BLE and IEEE 802.15.4 (so BLE
  traffic interferes with Zigbee when their spectra overlap). `World` owns the
  media and exposes each one as a `radio` endpoint; TLSR8258 attaches through
  `Mcu::attach_radio` and scripted parts through the Lua `radio` API.

A system is described in YAML and built by `build_system` (`System`,
`McuRegistry`; format in [system-config.md](system-config.md)). UART links are
polled by `World` after every slice. `mcuemu sim SYSTEM.yaml` runs one.

Pad timing: an MCU is executed one instruction at a time inside `run_until`,
so pad changes are timestamped to the executing instruction. Different MCUs
may be up to one quantum apart in time. Unwired pads keep the legacy behavior
(`set_gpio_input`, idle-high inputs). GPIO OEN resets to `0xff`.

## Source references

- See [TLSR8258 register model and evidence](tlsr8258-register-model.md) for
  the current sleep/wake and RF register semantics, evidence levels, and known
  model simplifications.
- Telink TLSR8258 datasheet, section 2.1.1, memory windows and SRAM split.
- Telink SDK `platform/chip_8258/register.h`, timer registers at offsets
  `0x620`–`0x638`, IRQ registers at `0x640`–`0x64b`, and GPIO ports at
  `0x580`–`0x5a7`.
- Telink TLSR825x series page for supported SoC variants and protocol support.
