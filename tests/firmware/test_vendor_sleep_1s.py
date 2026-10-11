#!/usr/bin/env python3
"""Check repeated 1-second deep-sleep timer wake cycles in Telink firmware."""

import re
import subprocess
import sys


SLEEP_ENTRY = re.compile(
    r"^mmio W pc=0x[0-9a-f]+ addr=0x80006f size=1 value=0x81$",
    re.MULTILINE,
)
EXPECTED_CYCLES = 4
INSTRUCTION_LIMIT = "500000"


def fail(message: str, output: str) -> int:
    print(f"firmware_vendor_sleep_1s: {message}", file=sys.stderr)
    if output:
        print("--- emulator output ---", file=sys.stderr)
        print(output, file=sys.stderr, end="" if output.endswith("\n") else "\n")
    return 1


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: test_vendor_sleep_1s.py TC32_EMU FIRMWARE",
            file=sys.stderr,
        )
        return 2

    command = [
        sys.argv[1],
        "run",
        "--model",
        "TLSR8258F512",
        "--firmware",
        sys.argv[2],
        "--steps",
        INSTRUCTION_LIMIT,
        "--trace-mmio-range",
        "0x80006f:0x80006f",
        "--trace-limit",
        "1000",
        "--fault-trace",
        "16",
    ]
    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=60,
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
            f"expected at least {EXPECTED_CYCLES} sleep entries, observed {cycles}",
            result.stdout,
        )
    if "CPU remains in" in result.stdout:
        return fail("firmware remained asleep instead of waking", result.stdout)

    time_match = re.search(r"^virtual time: (\d+) ns$", result.stdout, re.MULTILINE)
    if not time_match:
        return fail("emulator did not report virtual time", result.stdout)
    virtual_time_ns = int(time_match.group(1))
    average_sleep_ns = virtual_time_ns / cycles
    if not 900_000_000 <= average_sleep_ns <= 1_100_000_000:
        return fail(
            "average virtual time per sleep entry was "
            f"{average_sleep_ns:.0f} ns; expected about 1 second",
            result.stdout,
        )

    print(
        "firmware_vendor_sleep_1s: "
        f"{cycles} sleep/wake cycles, {virtual_time_ns} ns virtual time"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
