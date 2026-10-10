# Implementation status

The core-first boundary is now in place: TC32 execution is implemented in the
local Unicorn checkout, and the TLSR8258 wrapper calls that backend through the
Unicorn C API. End-to-end vendor-firmware acceptance is still outstanding.

| Work area | Current state |
| --- | --- |
| TC32 in Unicorn | Initial backend implements the covered instruction families, CPU state, CPSR/SPSR/SSR access, `tmrcs` CPSR reads, and C API dispatch. Ten backend tests pass, including an instruction-fetch alias test. Full TC32 vendor-binary coverage is not established. |
| TLSR8258 wrapper | Maps flash, board SRAM, the documented MMIO range, and a firmware-observed register alias into a TC32 `uc_engine`; configures Unicorn to fetch SRAM cache-alias instructions from flash while keeping data reads and writes on SRAM. Uses Unicorn's C API for registers and execution. Board code advances virtual time and injects modeled IRQs around Unicorn execution. |
| TLSR8258 peripherals | GPIO, timer compare/status, fixed 16 MHz system timer, UART FIFOs and timing, IRQ masks/sources, analog access, MSPI JEDEC ID, AES-128 ECB encrypt/decrypt, and the SDK-used integer divider helper are modeled. AREG `0x40..0x43` exposes a 32 kHz RC or 32.768 kHz crystal counter. AES completion uses the datasheet's 1000-cycle duration; divider operations complete synchronously. RF TX/RX DMA, including the DMA high-byte offset from `0x800000`, basic RF IRQ status, and packet delivery to the virtual medium are connected. Sleep, general flash commands, SPI/I²C transfers, and DMA channels beyond the RF path remain incomplete. |
| Virtual radio | BLE and Zigbee packet transport with channel matching, estimated duration, broadcast/unicast, and collision suppression. TLSR8258 bridges Zigbee RF TX/RX DMA to this medium and decodes Zigbee channel writes from the RF synthesizer frequency registers `0x801244..0x801245`. BLE is not connected to TLSR8258 RF. |
| CLI | Selects F512/F1K, loads raw images at address zero, runs one node or a multi-node Zigbee scenario, prints CPU/time and radio frame state, and optionally traces PC or filtered MMIO accesses. It does not parse ELF. |
| Zigbee `sampleLight` | Standalone, the local `samplelight_tb03f.bin` runs 100,000,000 instructions without a CPU fault after the instruction-fetch alias fix. In a two-node run it emits 8-byte frames on channels 11 and 12. It then faults at `PC=0xff8` on `tpop {r4-r7,pc}` with `SP=0x84fff8`, which reads 12 bytes beyond the mapped SRAM end. Two local `z03mmc_coord` images produced no frames, so firmware-driven packet reception and exchange remain unverified. SDK and binary artifacts remain outside this repository. |

Two behaviors are explicit inferences. The datasheet documents the register
window at `0x800000..0x83ffff`, while vendor `LoadTblCmdSet` code forms writes
at `0x1000000 + offset`; that observed address range is mapped as a compatibility
alias to the documented registers. The AREG `0x7f` cold-boot value is inferred
from the SDK wake path. Unicorn exposes instruction hooks but not TC32 cycle
counts, so board time currently advances by an estimated three cycles per
instruction; this has not been calibrated against hardware.

The RF bridge test exchanges an 802.15.4 payload between two TLSR8258 board
models through TX/RX DMA buffers and IRQ status. It also checks that firmware
synthesizer writes override the initial host-selected channel and that DMA
addresses use the vendor's high-byte encoding. The `network` CLI command loads
multiple firmware images onto a shared medium. Next work is to diagnose the
sample's stack overrun and obtain a coordinator image that responds on-air,
then verify a Zigbee exchange and connect BLE firmware to the TLSR8258 radio
model. Keep all CPU instruction decoding and execution inside Unicorn.
