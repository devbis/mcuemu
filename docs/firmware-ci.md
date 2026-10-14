# Firmware tests in CI

`.github/workflows/firmware-tests.yml` runs real firmware on the emulator. It
starts when "Build mcuemu" finishes successfully (or by hand: *Run workflow*,
optionally with the id of a build run) and uses the Linux binary from that build.
It runs on the latest Ubuntu runner.

## Images

Every image is an artifact (`fw-*`) of its own job, so a broken build fails only
the scenarios that need it.

| Image | Source | How |
| --- | --- | --- |
| `telink-gw.bin` (coordinator) | Telink Zigbee SDK `V3.7.2.0`, `apps/sampleGW` | `tests/ci/telink` (CMake, tc32 GCC from `devbis/tc32` branch `linux`), board `BOARD_8258_EVK_V1P2` |
| `telink-light.bin` | same SDK, `apps/sampleLight` | same |
| `z03mmc.bin` | `devbis/z03mmc` (default branch) | its own CMake, as in its workflow |
| `tlsr8258-plug.bin` | `faronov/zigbee-rs-plugs`, feature `zbeacon-ts011f-512k` | `scripts/tlsr8258-firmware.sh` with the pinned tc32 Rust toolchain (SHA-256 checked) |
| `nrf52840-sensor.bin`, `nrf52840-sensor-sht31.bin` | `devbis/zigbee-rs`, branch `nrf-experiment` (`ZIGBEE_RS_REF`) | `cargo build --release` in `examples/nrf52840-sensor` |
| `thermostat-ca51.bin`, `thermostat-zt3l.bin` | releases in `slacky1965/tuya_thermostat_full_custom_r02_zrd` (`bin/`, default branch) | downloaded |
| `zephyr-pmtest.bin`, `vendor_sleep_1s.bin` | no source available | committed in `tests/firmware-images/` |

Adding an image: build it from source if that is easy; otherwise download it
from a stable URL or a GitHub release; commit it to `tests/firmware-images/`
only as a last resort and record where it came from in that folder's README.
The downloaded thermostat files are not hash-pinned (the URL follows the
project's default branch); the job prints their SHA-256.

## Scenarios

One job per scenario, independent of each other. The scenario files are in
`tests/firmware/scenarios/`; they point at `firmware/<image>.bin`, so they run
locally too when you put the images there.

| Scenario | Passes when |
| --- | --- |
| `zigbee-light`, `zigbee-z03mmc`, `zigbee-plug` | the device gets an Association Response with status 0, then sends NWK data frames to the coordinator and the coordinator sends some back |
| `zigbee-nrf52840` | as above plus, in the defmt log, the scan, join, Transport-Key, Request Key, Confirm-Key SUCCESS and "network committed"; reports keep arriving after 75 s |
| `thermostat` | the ZT3L joins the coordinator and both UARTs of the two-chip board carry framed traffic |
| `nrf52840-boot`, `zephyr-pm-wake`, `vendor-sleep-1s` | the existing scripts in `tests/firmware/` |

`tests/firmware/zigbee_air_check.py` runs the system file and reads the pcap
sink (link type 230, 802.15.4 without FCS); it does not depend on RAM addresses
or pins. `--expect REGEX` checks the console, with RTT chunks re-joined into lines.

Some devices join late on purpose: the z03mmc scans once at power-up, before the
coordinator has formed its network, and steers again after 60 s, so its scenario
runs 80 s; the plug gets a short button press at 8 s to start steering.

## Logs

UART and RTT output is printed in the job log (`--trace-uart '*=text'`, or the
`hex`/`text` sinks in the system file) and saved with the pcap in the
`logs-<scenario>` artifact (14 days). Any new scenario that has a UART or RTT
channel must do the same.
