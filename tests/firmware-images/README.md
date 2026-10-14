# Prebuilt test images

Images that have no buildable source anywhere we can reach. Everything else the
firmware tests need is built or downloaded by `.github/workflows/firmware-tests.yml`.

| Image | Size | What it is | Used by |
| --- | --- | --- | --- |
| `zephyr-pmtest.bin` | 15 KB | Zephyr power-management test for the TLSR8258 (repeated 32 kHz deep-sleep wake cycles) | `tests/firmware/test_zephyr_pm_wake.py` |
| `vendor_sleep_1s.bin` | 11 KB | Vendor-SDK style 1 s deep-sleep loop for the TLSR8258 | `tests/firmware/test_vendor_sleep_1s.py` |

Add an image here only when it cannot be built or downloaded; say where it came
from in this table.
