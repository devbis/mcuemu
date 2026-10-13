#!/usr/bin/env python3
"""Check repeated Zephyr 32 kHz deep-sleep timer wake cycles."""

import re
import subprocess
import sys


SLEEP_ENTRY = re.compile(
    r"^mmio W pc=0x[0-9a-f]+ addr=0x80006f size=1 value=0x81$",
    re.MULTILINE,
)
EXPECTED_CYCLES = 4
INSTRUCTION_LIMIT = "60000000"


def fail(message: str, output: str) -> int:
    print(f"firmware_zephyr_pm_wake: {message}", file=sys.stderr)
    if output:
        print("--- emulator output ---", file=sys.stderr)
        print(output, file=sys.stderr, end="" if output.endswith("\n") else "\n")
    return 1


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: test_zephyr_pm_wake.py MCUEMU FIRMWARE",
            file=sys.stderr,
        )
        return 2

    command = [
        sys.argv[1],
        "run",
        "--firmware",
        sys.argv[2],
        "--steps",
        INSTRUCTION_LIMIT,
        "--trace-mmio-range",
        "0x80006f:0x80006f",
        "--trace-limit",
        "100",
        "--fault-trace",
        "16",
    ]
    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=170,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        return fail("emulator timed out", output)

    if result.returncode != 0:
        return fail(f"emulator exited with status {result.returncode}", result.stdout)

    cycles = len(SLEEP_ENTRY.findall(result.stdout))
    if cycles < EXPECTED_CYCLES:
        return fail(
            f"expected at least {EXPECTED_CYCLES} deep-sleep entries, observed {cycles}",
            result.stdout,
        )
    if "CPU remains in" in result.stdout:
        return fail("firmware remained asleep instead of waking", result.stdout)

    time_match = re.search(r"^virtual time: (\d+) ns$", result.stdout, re.MULTILINE)
    virtual_time = int(time_match.group(1)) if time_match else 0
    print(
        "firmware_zephyr_pm_wake: "
        f"{cycles} deep-sleep entries, {virtual_time} ns virtual time"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
