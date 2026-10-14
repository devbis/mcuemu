#!/usr/bin/env python3
"""An EFR32MG21 (zigbee-rs sensor image, BRD4181A) joins a network run by the
TLSR8258 sampleGW coordinator: a Cortex-M33 and a TC32 core on one radio medium.

A button press at 2 s starts a join; the sensor scans, finds the coordinator's
beacon on channel 15, associates, receives the NWK key, then completes the
unique trust-center link key exchange (Request Key, Transport-Key, Verify Key,
Confirm Key) and commits the network. The radio model is the register
interface of the zigbee-rs `efr32s2` driver, not the silicon's (see
docs/efr32mg21.md).
"""

import os
import re
import subprocess
import sys
import tempfile


def fail(message: str, output: str) -> int:
    print(f"firmware_efr32mg21_zigbee_join: {message}", file=sys.stderr)
    if output:
        print("--- emulator output (tail) ---", file=sys.stderr)
        print("\n".join(output.splitlines()[-40:]), file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_efr32mg21_zigbee_join.py MCUEMU EXAMPLE.yaml", file=sys.stderr)
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
        pcap = os.path.join(scratch, "efr-air.pcap")
        pcap_size = os.path.getsize(pcap) if os.path.exists(pcap) else 0

    # The RTT text sink breaks lines at arbitrary chunk boundaries; join them.
    log = re.sub(r"\n?\[\s*[\d.]+ s\] board\.mcu\.rtt: ", "", result.stdout)
    for what, pattern in [
        ("the radio initialised", r"radio initialized in IEEE 802\.15\.4 mode"),
        ("an active scan that found the coordinator", r"Found 1 network\(s\)"),
        ("the join", r"Joined network as 0x[0-9A-F]+"),
        ("a unique TC link key request", r"Sending APSME-REQUEST-KEY to TC"),
        ("the Transport-Key with the TC link key", r"Transport-Key: TC Link Key from 0x0000"),
        ("the Confirm-Key", r"Confirm-Key SUCCESS from the Trust Center"),
        ("the committed network", r"Commissioning security complete.{1,16}network committed"),
    ]:
        if not re.search(pattern, log):
            return fail(f"never saw {what}", result.stdout)
    if pcap_size < 100:
        return fail("the pcap sink captured no frames", result.stdout)
    print("firmware_efr32mg21_zigbee_join: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
