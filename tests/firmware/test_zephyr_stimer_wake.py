#!/usr/bin/env python3
"""Zephyr sleepy end device: the system timer after a timed deep-sleep wake.

A deep-sleep wake is a reset that keeps the analog registers, so the firmware
finds its marker in AREG 0x3a (0xa5). The "before" image ran a late post-wake
handler that wrote 0x74c = 0 and 0x74f = START after the system timer driver
had started the counter; on a TB03F that left 0x800740 constant while the CPU
kept running (the first busy wait never returned, so the device could not sleep
again). The "after" image does the recovery before the driver starts.

The model must reproduce both: the counter stands still when started without
its 16 MHz clock selected, and keeps counting in the fixed image.
"""

import re
import subprocess
import sys

MARKER = ["--areg", "0x3a=0xa5"]
TIMER = "0x800740:0x800743"
CONTROL = "0x80074f:0x80074f"


def sample(mcuemu, image, steps, extra):
    result = subprocess.run(
        [mcuemu, "run", "--model", "TLSR8258F512", "--firmware", image, "--steps", str(steps),
         "--print-mmio", TIMER, "--print-mmio", CONTROL] + extra,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=300, check=False)
    if result.returncode != 0:
        raise SystemExit(f"firmware_zephyr_stimer_wake: emulator failed:\n{result.stdout}")
    timer = re.search(rf"mmio 0x{int(TIMER.split(':')[0], 16):x}: (\w\w) (\w\w) (\w\w) (\w\w)", result.stdout)
    control = re.search(r"mmio 0x80074f: (\w\w)", result.stdout)
    if not timer or not control:
        raise SystemExit(f"firmware_zephyr_stimer_wake: no register dump:\n{result.stdout}")
    value = int("".join(reversed(timer.groups())), 16)
    return value, int(control.group(1), 16)


def main() -> int:
    if len(sys.argv) != 4:
        print("usage: test_zephyr_stimer_wake.py MCUEMU BEFORE.bin AFTER.bin", file=sys.stderr)
        return 2
    mcuemu, before, after = sys.argv[1:]
    errors = []

    t1, c1 = sample(mcuemu, before, 40_000_000, MARKER)
    t2, c2 = sample(mcuemu, before, 80_000_000, MARKER)
    if not (t1 == t2 and t1 != 0):
        errors.append(f"before image after a deep wake: counter should stand still, got {t1:#x} then {t2:#x}")
    if c1 & 2 or c2 & 2:
        errors.append(f"before image: 0x74f RUNNING should read 0, got {c1:#x}/{c2:#x}")

    t1, c1 = sample(mcuemu, after, 40_000_000, MARKER)
    t2, c2 = sample(mcuemu, after, 80_000_000, MARKER)
    if not (t2 > t1 > 0):
        errors.append(f"after image after a deep wake: counter should run, got {t1:#x} then {t2:#x}")
    if not (c1 & 2 and c2 & 2):
        errors.append(f"after image: 0x74f RUNNING should read 1, got {c1:#x}/{c2:#x}")

    # Without the marker there is no wake handler work: the old image runs too.
    t1, _ = sample(mcuemu, before, 40_000_000, [])
    t2, _ = sample(mcuemu, before, 80_000_000, [])
    if not (t2 > t1 > 0):
        errors.append(f"before image after a normal reset: counter should run, got {t1:#x} then {t2:#x}")

    if errors:
        for error in errors:
            print(f"firmware_zephyr_stimer_wake: {error}", file=sys.stderr)
        return 1
    print("firmware_zephyr_stimer_wake: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
