# EFR32MG21 (Cortex-M33)

Silicon Labs EFR32MG21A020F512 (Series 2), as fitted on the BRD4181A radio
board: the second Cortex-M chip in mcuemu, on the same `cortexm::Core` as the
nRF52840 but with an ARMv8-M Mainline (Cortex-M33) CPU.

```sh
./build/mcuemu run --model EFR32MG21 --firmware firmware/efr32mg21-sensor.bin --for 2s
./build/mcuemu sim examples/efr32mg21_zigbee.yaml --for 30s
```

`mcuemu run` takes `--firmware IMAGE`, `--for DURATION`, `--steps N` and
`--trace-mmio`. In a system file use `model: efr32mg21`; pads are `PA0`..`PD15`.
The model has no variants, no `supply_mv`, and a fixed 38.4 MHz clock.

## What is and is not the silicon

| Part | Status |
| --- | --- |
| CPU, NVIC, SysTick, FPU, WFE | Cortex-M33 without the Security Extension (one security state, no SAU). 4 priority bits, 51 interrupt lines (`EMU`..`RTCC`). |
| Flash / RAM | 512 KiB at 0, 64 KiB at `0x20000000`. USERDATA (`0x0FE00000`) is erased, DEVINFO (`0x0FE08000`) holds the EUI-64 at `+0x1A0`. |
| Boot | The model has no Gecko Bootloader. An image whose reset vector lies at or above `0x4000` is an application-slot image: it is loaded at `0x4000` and started directly with `VTOR = 0x4000`. Any other image is loaded at 0. |
| Peripheral blocks | `0x40000000 + 0x4000 * n`, with the Series 2 aliases (`+0x1000` set, `+0x2000` clear, `+0x3000` toggle). |
| CMU (`0x40008000`), HFXO0 (`0x4000C000`) | Registers read back; `HFXO.STATUS` follows `CTRL` and the system clock selection (RDY/ENS, first-lock FSMLOCK) so the GSDK start-up sequence completes. The clock is the HFXO's 38.4 MHz from reset; the 20 MHz FSRCO is not modelled. |
| GPIO (`0x4003C000`) | Ports A..D: MODEL/MODEH modes (input, pull, push-pull, wired-or/and), DOUT, DIN, EXTI 0..7 (`EXTIPSEL/PINSEL`, rise/fall, IF/IEN) on `GPIO_EVEN`/`GPIO_ODD`. |
| MSC (`0x40030000`) | Page erase (8 KiB) and word programming (bits can only be cleared) by the register protocol of the zigbee-rs flash driver; instantaneous. |
| **RADIO** | **Not the silicon.** See below. |
| Everything else | Plain register stand-ins: values read back, no events, no interrupts (EMU, LDMA, TIMER, USART, I2C, IADC, CRYPTO, SE, ...). Firmware that waits for one of them stops at the wait; `--trace-mmio` shows where. |

## The radio

The real radio is a sequencer (RAC) running Silicon Labs' RAIL configuration;
it cannot be driven by a handful of register writes, and mcuemu does not model
it. What `radio.cpp` implements is the register interface of the zigbee-rs
`efr32s2` MAC driver (`zigbee-mac/src/efr32s2/driver.rs`), whose own header
calls it a scaffold with unverified register values: FRC, BUFC, SYNTH, RAC and
AGC blocks at `0x40090000`, TX and RX FIFOs, RAC commands and status, interrupt
`FRC_PRI` (IRQ 36), RSSI from the medium. It exists so that firmware can run in
a network. **Firmware built on RAIL or the GSDK radio stack will not work**, and
a driver whose register values were fixed to match the silicon would need this
block changed with it. The register map is in the header comment of
`emulator/src/efr32/radio.cpp`.

Frames go through the shared `mcuemu::VirtualAir` as 802.15.4 MAC frames
without the FCS (so a frame is 64 us shorter on air than on a real radio). A
40 us ramp precedes TX and RX; the receiver polls the medium every 16 us and can
lock on a frame whose start-of-frame delimiter has not passed (128 us), as in
the nRF52840 model. The radio stays in RX after a frame. All of this timing is
inferred.

## Board: BRD4181A

`examples/devices/efr32mg21_brd4181a.yaml`: LED0 on PB0 (active high), BTN0 on
PD2 (the WSTK pull-up, low = pressed), RTT on the control block of the image
(`rtt_address`, `0x20004a18` for the CI image this was developed with; it moves
with every build). `examples/efr32mg21_zigbee.yaml` puts the board on one medium
with the TLSR8258 sampleGW coordinator: a button press at 2 s starts the join,
and the sensor associates, receives the NWK key and completes the trust-center
link key exchange (about 23 s in, after the first scans find no network yet),
then reports temperature and humidity (synthetic) from about 83 s.

## The Unicorn fork

M33 needed three fork changes on top of the NVIC port (see
[`nrf52840.md`](nrf52840.md)):

- `uc_arm_nvic(UC_ARM_NVIC_NO_SECURITY)` makes the stock Cortex-M33 model a core
  without the Security Extension (clears the feature, SAU regions and the secure
  state) and sets `NSACR` so FP stacking is governed by `CPACR` alone. The NVIC
  models a single security state, so the secure banks would otherwise be
  unreachable.
- `v7m_exception_taken` set `EXC_RETURN.ES` whenever the Security Extension was
  absent, and the exception return of the same core then treated it as an invalid
  return (`UFSR.INVPC` escalating to HardFault on the first IRQ). ES is now set
  only for secure targets.
- `Core::reset(vector_table)` loads SP/PC from the application slot and sets
  `VTOR` there.

## Tests

- `efr32emu-tests`: the M33 exception path (the nRF52 `irq` program, with the
  FPU context), the application-slot boot, CMU/HFXO, GPIO and EXTI, MSC, and the
  radio (frame exchange, state, RSSI).
- `simcore-tests` (`tests/sim/test_efr32.cpp`): pads on nets, config rules.
- `firmware_efr32mg21_zigbee_join` (image `efr32mg21-sensor.bin` and
  `samplegw_tb03f.bin` in `firmware/`): the two-chip join, TCLK exchange and
  commit, judged from the RTT log.

## Not done

- A radio that matches the silicon (RAIL), BLE.
- EMU/EM2 sleep, LDMA, TIMER/LETIMER/RTCC/BURTC, USART, I2C, IADC, CRYPTO/SE
  (random numbers, AES) and the other stand-ins above.
- Other family members (MG1 is Series 1 and a different chip, MG24 is a larger
  Series 2 part): `Efr32mg21` is the only class.
- TrustZone: the model has one security state.
