# Architecture and implementation notes

## Address space

The initial board exposes:

| Address range | Model |
| --- | --- |
| `0x000000` to flash size | Read-only program flash; 512 KiB or 1 MiB |
| `0x800000` to `0x83ffff` | 256 KiB register/MMIO window |
| `0x1000000` to `0x103ffff` | Firmware-observed compatibility alias for the register window |
| `0x840000` to `0x847fff` | 32 KiB SRAM retained in deep sleep |
| `0x848000` to `0x84ffff` | 32 KiB SRAM not retained in deep sleep |

These boundaries come from section 2.1.1 of the TLSR8258 datasheet. This
baseline maps the two retention classes contiguously because power-state
behavior has not been implemented yet.

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

## Source references

- Telink TLSR8258 datasheet, section 2.1.1, memory windows and SRAM split.
- Telink SDK `platform/chip_8258/register.h`, timer registers at offsets
  `0x620`–`0x638`, IRQ registers at `0x640`–`0x64b`, and GPIO ports at
  `0x580`–`0x5a7`.
- Telink TLSR825x series page for supported SoC variants and protocol support.
