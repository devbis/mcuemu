#!/usr/bin/env python3
"""An ESP32-H2 (zigbee-rs sensor image: ESP-IDF 2nd stage bootloader + esp-hal
app) joins a network run by the TLSR8258 sampleGW coordinator: a RISC-V core
and a TC32 core on one radio medium.

The model runs the real bootloader on ROM stand-ins. The sensor scans,
associates, gets the NWK key and completes the trust-center link key exchange.
It first scans before the coordinator has formed its network, backs off and
reboots (a software reset the model restarts), then joins at the next scan.
"""

import re
import subprocess
import sys
import tempfile


def fail(message: str, output: str) -> int:
    print(f"firmware_esp32h2_zigbee_join: {message}", file=sys.stderr)
    if output:
        print("--- emulator output (tail) ---", file=sys.stderr)
        print("\n".join(output.splitlines()[-40:]), file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_esp32h2_zigbee_join.py MCUEMU EXAMPLE.yaml", file=sys.stderr)
        return 2
    with tempfile.TemporaryDirectory() as scratch:
        try:
            result = subprocess.run(
                [sys.argv[1], "sim", sys.argv[2], "--for", "30s"],
                cwd=scratch, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=300, check=False)
        except subprocess.TimeoutExpired as error:
            output = error.stdout or ""
            if isinstance(output, bytes):
                output = output.decode(errors="replace")
            return fail("emulator timed out", output)
    if result.returncode != 0:
        return fail(f"emulator exited with status {result.returncode}", result.stdout)
    log = result.stdout
    for pattern, what in [
        (r"Hardware AES KAT passed", "the hardware AES known-answer test"),
        (r"JoinedOrResumed", "the network join"),
        (r"Confirm-Key SUCCESS", "the trust-center link key exchange"),
        (r"CommissioningComplete \{ success: true \}", "commissioning"),
    ]:
        if not re.search(pattern, log):
            return fail(f"missing {what} ({pattern})", log)
    if re.search(r"PANIC", log):
        return fail("the firmware panicked", log)
    return 0


if __name__ == "__main__":
    sys.exit(main())
