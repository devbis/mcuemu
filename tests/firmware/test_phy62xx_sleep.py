#!/usr/bin/env python3
"""Check the sleeps of the PHY62x2 model with the sleep self test of the PB-03F.

The boot ROM starts the image (boot_from_rom). The self test runs three suspends to RAM of 500 ms, then a
deep sleep that keeps a retained-RAM counter. The negative image clears the RC32K wake-up bit of
AON->PMCTL0 before its first sleep: the chip must not wake, as it never did on the board.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

SYSTEM = """version: 1
world:
  seed: 1
  duration: {duration}
devices:
  - id: pb03f
    mcus:
      - id: mcu
        model: phy6252
        firmware: {firmware}
        boot_from_rom: true
        rom: {rom}
        efuse: {efuse}
        rf_calibration: {rf_calibration}
"""

# In order: the ROM starts the image cold, three suspends return, the deep sleep restarts the image
# through the ROM with the reset cause 5 and the retained counter at 2
EXPECTED = [
    r"SLEEPTEST boot 1 hwinfo 0x8 rom cause 0 SLEEP_R0 0x1",
    r"SLEEPTEST suspend 0 returned after (\d+) ticks",
    r"SLEEPTEST suspend 1 returned after (\d+) ticks",
    r"SLEEPTEST suspend 2 returned after (\d+) ticks",
    r"SLEEPTEST SUSPEND PASS",
    r"SLEEPTEST deep sleep 500 ms",
    r"SLEEPTEST boot 2 hwinfo 0x2 rom cause 5 SLEEP_R0 0x1",
    r"SLEEPTEST DEEP PASS",
    r"SLEEPTEST DONE",
]


def run(emulator: str, config: str, scratch: str) -> str:
    workdir = Path(scratch)
    system = workdir / "system.yaml"
    system.write_text(config)
    result = subprocess.run(
        [emulator, "sim", str(system), "--trace-uart", "*=text"],
        cwd=workdir,
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )
    output = result.stdout + result.stderr
    if result.returncode != 0:
        raise RuntimeError(f"emulator exited with status {result.returncode}\n{output}")
    return output


def main() -> int:
    if len(sys.argv) != 7:
        print(
            "usage: test_phy62xx_sleep.py EMULATOR FIRMWARE FIRMWARE_NORC ROM EFUSE RF_CALIBRATION",
            file=sys.stderr,
        )
        return 2
    emulator, firmware, norc, rom, efuse, rf_calibration = (
        str(Path(path).resolve()) for path in sys.argv[1:]
    )

    def config(image: str, duration: str) -> str:
        return SYSTEM.format(
            duration=duration,
            firmware=json.dumps(image),
            rom=json.dumps(rom),
            efuse=json.dumps(efuse),
            rf_calibration=json.dumps(rf_calibration),
        )

    with tempfile.TemporaryDirectory(prefix="mcuemu-phy62xx-sleep-") as scratch:
        output = run(emulator, config(firmware, "4s"), scratch)
        position = 0
        ticks = []
        for pattern in EXPECTED:
            match = re.search(pattern, output[position:])
            if not match:
                print(f"test_phy62xx_sleep: UART never reached /{pattern}/ in order", file=sys.stderr)
                print(output, file=sys.stderr)
                return 1
            position += match.end()
            if match.groups():
                ticks.append(int(match.group(1)))
        # 500 ms are 16384 ticks of the 32768 Hz RTC; the entry and the exit add a few hundred
        if any(not 16384 <= count <= 16384 + 3000 for count in ticks):
            print(f"test_phy62xx_sleep: suspend times out of range: {ticks}", file=sys.stderr)
            return 1

        output = run(emulator, config(norc, "3s"), scratch)
        if "SLEEPTEST boot 1" not in output:
            print("test_phy62xx_sleep: the negative image did not start", file=sys.stderr)
            print(output, file=sys.stderr)
            return 1
        if re.search(r"SLEEPTEST (suspend \d+ returned|boot 2|SUSPEND)", output):
            print(
                "test_phy62xx_sleep: the chip woke without the RC32K wake-up bit", file=sys.stderr
            )
            print(output, file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
