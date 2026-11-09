#!/usr/bin/env python3
"""Boot the Silicon Labs EZSP NCP (Sonoff ZBDongle-E, EmberZNet 8.2.2) on the EFR32MG21 model.

The image formats its NVM3 area, brings up RAIL and the ASH UART on USART0, and sends the
power-on RSTACK frame: 1A C1 02 02 9B 7B 7E (cancel byte, RSTACK, version 2, reset code 2 = power
on, CRC, flag). The image comes from the Nerivec/silabs-firmware-builder releases; extract the flat
application with scripts/gbl2bin.py. Seeing exactly that frame (and not an ERROR frame C2 ..)
needs the DEVINFO fields, EMU RSTCAUSE and the RAIL init stand-ins of the model.
"""

import subprocess
import sys

RSTACK = bytes([0x1A, 0xC1, 0x02, 0x02, 0x9B, 0x7B, 0x7E])


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_efr32mg21_ncp_rstack.py MCUEMU FIRMWARE", file=sys.stderr)
        return 2
    command = [sys.argv[1], "run", "--model", "EFR32MG21", "--firmware", sys.argv[2], "--for", "3s"]
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=180, check=False)
    except subprocess.TimeoutExpired:
        print("firmware_efr32mg21_ncp_rstack: emulator timed out", file=sys.stderr)
        return 1
    if result.returncode != 0:
        print(f"firmware_efr32mg21_ncp_rstack: emulator exited {result.returncode}", file=sys.stderr)
        return 1
    marker = b"UART0 output ("
    start = result.stdout.find(marker)
    if start < 0:
        print("firmware_efr32mg21_ncp_rstack: the NCP sent nothing on UART0", file=sys.stderr)
        print(result.stdout.decode(errors="replace")[-1500:], file=sys.stderr)
        return 1
    start = result.stdout.index(b":\n", start) + 2
    body = result.stdout[start:]
    if not body.startswith(RSTACK):
        print("firmware_efr32mg21_ncp_rstack: expected RSTACK " + RSTACK.hex(" ") +
              ", got " + body[:16].hex(" "), file=sys.stderr)
        return 1
    print("NCP sent the power-on RSTACK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
