#!/usr/bin/env python3
"""Boot the Silicon Labs EZSP NCP (Sonoff ZBDongle-E, EmberZNet 8.2.2) on the EFR32MG21 model and talk ASH/EZSP.

The image formats its NVM3 area, brings up RAIL and the ASH UART on USART0 (receive through LDMA) and
sends the power-on RSTACK 1A C1 02 02 9B 7B 7E. The test then plays the host: RST (the NCP reboots and
answers RSTACK with reset code 0x0B) and the EZSP `version` command, whose answer must report the
EmberZNet version of the release (protocol 0x12, stack 0x8220 = 8.2.2). The image comes from the
Nerivec/silabs-firmware-builder releases; extract the flat application with scripts/gbl2bin.py.
"""

import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
import ash_frame  # noqa: E402

RSTACK_POWER_ON = bytes([0x1A, 0xC1, 0x02, 0x02, 0x9B, 0x7B, 0x7E])


def fail(message: str) -> int:
    print(f"firmware_efr32mg21_ncp_rstack: {message}", file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_efr32mg21_ncp_rstack.py MCUEMU FIRMWARE", file=sys.stderr)
        return 2
    rst = (b"\x1a" + ash_frame.frame(0xC0)).hex()
    # A leading CANCEL byte: after a reset the NCP's ASH decoder drops bytes until it sees a flag or cancel.
    version = (b"\x1a" + ash_frame.data_frame(0, 0, bytes.fromhex("00000004"))).hex()
    command = [sys.argv[1], "run", "--model", "EFR32MG21", "--firmware", sys.argv[2], "--for", "6s",
               "--uart0-send", f"{rst}@2s", "--uart0-send", f"{version}@4.5s"]
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=300, check=False)
    except subprocess.TimeoutExpired:
        return fail("emulator timed out")
    if result.returncode != 0:
        return fail(f"emulator exited {result.returncode}")
    text = result.stdout.decode(errors="replace")
    line = next((l for l in text.splitlines() if l.startswith("UART0 hex:")), None)
    if line is None:
        return fail("the NCP sent nothing on UART0")
    capture = bytes.fromhex(line.split(":", 1)[1].replace(" ", ""))
    if not capture.startswith(RSTACK_POWER_ON):
        return fail("expected the power-on RSTACK first, got " + capture[:8].hex(" "))
    frames = list(ash_frame.parse(capture))
    kinds = [kind for kind, _ in frames]
    if kinds[:2] != ["RSTACK", "RSTACK"] or "reset code 0x0b" not in frames[1][1]:
        return fail("expected RSTACK after RST with reset code 0x0b, got " + repr(frames))
    data = [detail for kind, detail in frames if kind == "DATA"]
    if not data or not data[0].endswith("00 80 00 12 02 20 82"):
        return fail("expected the EZSP version response (protocol 0x12, stack 0x8220), got " + repr(frames))
    print("NCP: RSTACK, RST -> RSTACK(0x0b), EZSP version 0x12 / stack 8.2.2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
