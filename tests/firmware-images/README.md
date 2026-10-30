# Prebuilt test images

Images that have no buildable source anywhere we can reach. Everything else the
firmware tests need is built or downloaded by `.github/workflows/firmware-tests.yml`.

| Image | Size | What it is | Used by |
| --- | --- | --- | --- |
| `zephyr-pmtest.bin` | 15 KB | Zephyr power-management test for the TLSR8258 (repeated 32 kHz deep-sleep wake cycles) | `tests/firmware/test_zephyr_pm_wake.py` |
| `vendor_sleep_1s.bin` | 11 KB | Vendor-SDK style 1 s deep-sleep loop for the TLSR8258 | `tests/firmware/test_vendor_sleep_1s.py` |
| `phy6252-ts0001.bin` | 512 KB | Closed Tuya TS0001 relay firmware, full flash of a PHY6222 module (no source; the NVM area is erased by the test before the run, so it joins as a first boot) | `tests/firmware/test_phy6252_ts0001_join.py` |
| `phy6222-efuse.bin`, `phy6222-rfcal.bin` | 32 B, 4 B | eFuse contents (blank) and RF calibration word the TS0001 expects | same |
| `phy6222-rom.bin` | 96 KB | PHY6222 mask ROM dump, trimmed from the 128 KB dump (the tail is erased `0xff`). Loaded with `rom:`; not used by the CI join test yet: with it the TS0001 first boot does not transmit (see `docs/phy6252-investigation.md`) | `examples/phy6252-zephyr-external-rom.yaml` |

Add an image here only when it cannot be built or downloaded; say where it came
from in this table.
