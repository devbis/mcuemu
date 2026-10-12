# TLSR8258 register model and evidence

This document records what the emulator currently knows about the TLSR8258
registers used by its firmware scenarios. It separates documented behavior
from SDK behavior and emulator approximations so that an experiment does not
silently become a hardware fact.

## Evidence labels

| Label | Meaning |
| --- | --- |
| `DS` | Specified in the Telink TLSR8258 datasheet. |
| `SDK` | Named or exercised by Telink SDK code/documentation. This may describe a supported sequence without specifying the silicon implementation. |
| `TRACE` | Observed in a firmware or board trace. Record the image and scenario; do not generalize to all firmware without another check. |
| `INFERRED` | Emulator behavior inferred from firmware behavior or adjacent documentation. |
| `SIMPLIFIED` | A deliberate abstraction that is not cycle-accurate or register-complete. |

## Sleep, wake, and clocks

| Register | Current interpretation | Evidence and model status |
| --- | --- | --- |
| AREG `0x05` | Bit 0 powers down the 32 kHz RC oscillator; bit 1 powers down the 32 kHz crystal. Reset values are 0 and 1 respectively. | `DS`; model uses these bits to decide whether its selected 32 kHz clock advances. |
| AREG `0x26` | Bit 6 enables wake from the 32 kHz timer. | `DS`; represented by `AnalogWakeSource` / `AnalogWakeTimerEnable`. |
| AREG `0x44` | Bits 0–3 report the wake cause; a set status bit is cleared by writing 1. Bit 1 is the 32 kHz timer status. | `DS`; timer status is modeled as write-one-to-clear. This is a status register, not the wake-enable register. |
| AREG `0x40..0x43` | Firmware-visible 32-bit low-frequency tick counter. | `SDK`/`TRACE`; modeled from virtual time using the selected 32 kHz source. RC frequency is configurable; crystal uses 32768 Hz. |
| Digital `0x800740..0x800743` | 16 MHz system timer. Its low three bits are invalid, giving 0.5 us resolution. | `DS`; modeled. |
| Digital `0x80074c` | System timer control; bit 7 enables 32 kHz calibration. | `DS`; modeled for the calibration sequence and timer IRQ. |
| Digital `0x80074f` | System timer command/control strobes, including 32 kHz calibration and timer setup. | `DS`/`SDK`; the model clears command strobes synchronously. Real BUSY timing is not modeled yet. |
| Digital `0x800750..0x800751` | Calibration value produced by the 32 kHz measurement. | `SDK`/`TRACE`; the model emits a configured value after 16 low-frequency ticks. It is not an analog circuit simulation. |
| Digital `0x800754..0x800757` | 32 kHz wake compare used by the tested firmware. | `SDK`/`TRACE`; compare and wake are modeled. Verify against the applicable SDK/chip revision before treating the address as universal. |
| Digital `0x80006f` | Firmware writes `0x81` to request sleep. | `TRACE`; the exact sleep-entry delay is not specified by the register write alone. In the tested `count_power_c.bin`, the path reaches the `start_suspend + 0x58` boundary after 40 16-bit delay instructions. The emulator currently approximates this with 120 core cycles, using its estimated three cycles per instruction. |
| AREG `0x7e`, `0x7f` | Sleep-mode selection and PM wake/cold-boot status used by the SDK path. | `SDK`/`INFERRED`; these meanings and reset/retention details should be checked per wake mode. |

The datasheet identifies AREG `0x26[6]` as the 32 kHz timer wake enable and
AREG `0x44[1]` as the timer wake status. An older local register cheatsheet
labels `0x44` as the wake source; use the datasheet distinction above.

## Analog-register access path

The MCU accesses AREGs through digital registers `0x8000b8` (address),
`0x8000b9` (data), and `0x8000ba` (control). The model stores the 256-byte
analog map and gives special behavior to the clock, wake, tick, and calibration
registers. `tc32-emu run` and `network` can decode each completed transfer with
`--trace-analog`; each event includes virtual time, direction, AREG address,
and transferred value. The event buffer is bounded by `--trace-limit`, and in
`network` it follows the `--trace-node` filter. AREG transfers still complete
synchronously and BUSY always reads clear (`SIMPLIFIED`), so this trace records
the model's transaction boundary and does not establish real BUSY timing.

## Radio and DMA

The current RF bridge is a packet-level Zigbee model. It does not simulate the
analog RF chain or decode every RF/baseband register.

| Register group | Current model behavior | Evidence and limitation |
| --- | --- | --- |
| `0x800f02` | Bits 4 and 5 gate the modeled TX and RX paths. | Firmware-derived; keep as `INFERRED` until validated against more RF modes. |
| `0x800c08`, `0x800c42` | RX DMA buffer address, including the high-byte encoding. | Exercised by the Zigbee firmware and board-model tests. |
| `0x800c0c`, `0x800c43`, `0x800c24` | TX DMA buffer and transmit-ready trigger. | Firmware-derived; starts a packet transmission on the virtual medium. |
| `0x800c20[2]`, `0x800428[0]` | RX DMA and receiver enable gates. | Firmware-derived; packet reception is delivered only when both are enabled. |
| `0x800f1c..0x800f21` | RF interrupt mask/status; modeled RX/TX completion bits and IRQ propagation. | Tested at the board-model level; not a complete RF interrupt model. |
| `0x801244..0x801245` | Synthesizer frequency writes are decoded to a Zigbee channel. | Firmware-observed mapping. Other modulation modes are not implemented through these registers. |

The SDK guide documents supported RF modes and API sequences, but that does
not establish the undocumented register-level semantics of every mode. The
emulator currently maps the tested Zigbee TX/RX DMA sequence to packets on
`VirtualAir`; BLE is not connected to these TLSR8258 RF registers.

## Next accuracy steps

1. Compare the decoded AREG sequence and virtual timestamps against a captured
   hardware trace; use that evidence to decide whether BUSY duration needs a
   time-based model.
2. Add reset/retention tests for each sleep mode and compare register state
   before sleep, at wake, and after startup.
3. For RF, create small deterministic firmware probes for one TX, one RX, and
   one ACK exchange. Compare register/DMA/IRQ traces, then add only the
   observed side effects needed by those sequences.
4. Keep radio internals abstract at the packet level unless a firmware test
   depends on a lower-level behavior such as CCA, retry timing, ACK timing, or
   receive filtering.

## References

- [Telink TLSR8258 datasheet](https://wiki.telink-semi.cn/doc/ds/DS_TLSR8258-E_Datasheet%20for%20Telink%20BLE%2BIEEE802.15.4%20Multi-Standard%20Wireless%20SoC%20TLSR8258.pdf), especially sections 2.6–2.7 and 5.3.
- [Telink TLSR8258F512 SDK core features](https://wiki.telink-semi.cn/tools_and_sdk/Driver/doc/kite/html/md__project_1__table_of__content_03__t_s_i__core__features.html).
- [Telink TLSR8258F512 SDK RF features](https://wiki.telink-semi.cn/tools_and_sdk/Driver/doc/kite/html/md__project_1__table_of__content_14__t_s_i__r_f__features.html).
- Emulator implementation: [`tlsr8258.hpp`](../emulator/include/tc32emu/tlsr8258.hpp), [`tlsr8258.cpp`](../emulator/src/tlsr8258.cpp).
- Relevant tests: [`test_tlsr8258.cpp`](../tests/machine/test_tlsr8258.cpp), [`test_cpu.cpp`](../tests/tc32/test_cpu.cpp).
