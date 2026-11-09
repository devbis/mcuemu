#!/usr/bin/env python3
"""The Silicon Labs EZSP NCP (EFR32MG21) runs an active scan next to a Telink coordinator on channel 11.

mcuemu sim puts both chips on one air; the host side is played by scenario events (UART bytes as
"hex:..."): RST, version, networkInit, then startScan (active, channels 11..26) at 8 s, when the
coordinator has formed its network. The NCP must transmit an 802.15.4 beacon request (hardware
listen-before-talk is emulated), the coordinator must answer with a beacon, and the NCP must report it
to the host as an EZSP networkFoundHandler callback (frame id 0x1B) for channel 11. The host polls with
`callback` (0x0006) frames; the callback also tells that the receive path (RAIL RX, BUFC) works.
"""

import os
import re
import struct
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
import ash_frame  # noqa: E402


def fail(message: str) -> int:
    print(f"firmware_efr32mg21_ncp_scan: {message}", file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) != 4:
        print("usage: test_efr32mg21_ncp_scan.py MCUEMU NCP_IMAGE COORDINATOR_CH11_IMAGE", file=sys.stderr)
        return 2
    mcuemu, ncp, coordinator = (os.path.abspath(p) for p in sys.argv[1:4])
    events = [(1.2, b"\x1a" + ash_frame.frame(0xC0)),
              (2.4, b"\x1a" + ash_frame.data_frame(0, 0, bytes.fromhex("00000012"))),
              (2.6, b"\x1a" + ash_frame.data_frame(1, 1, bytes.fromhex("01000117000000"))),
              (8.0, b"\x1a" + ash_frame.data_frame(2, 2, bytes.fromhex("0200011a000100f87f0703")))]
    seq, t = 3, 8.05
    while t < 9.0:
        events.append((t, b"\x1a" + ash_frame.data_frame(seq & 7, seq & 7, bytes([seq, 0, 1, 6, 0]))))
        seq, t = seq + 1, t + 0.05
    with tempfile.TemporaryDirectory() as directory:
        pcap = os.path.join(directory, "air.pcap")
        yaml = f"""version: 1
world:
  seed: 1
  radio:
    - {{ id: air0, model: log_distance, reference_loss_db: 40, path_loss_exponent: 2.5 }}
devices:
  - id: coordinator
    mcus:
      - {{ id: zb, model: tlsr8258, variant: F1K, firmware: {coordinator},
          radio: {{ air: air0, position: [0, 0, 1], tx_power_dbm: 10, protocols: [zigbee] }} }}
  - id: ncp
    mcus:
      - {{ id: mcu, model: efr32mg21, firmware: {ncp},
          radio: {{ air: air0, position: [4, 0, 1], tx_power_dbm: 0, protocols: [zigbee] }} }}
attach:
  - {{ endpoint: air0, sink: "pcap:{pcap}" }}
  - {{ endpoint: ncp.mcu.uart0, sink: "hex" }}
scenario:
  events:
""" + "".join(f'    - {{ at: {at:.3f}s, endpoint: ncp.mcu.uart0, set: "hex:{data.hex()}" }}\n' for at, data in events)
        path = os.path.join(directory, "scan.yaml")
        with open(path, "w") as handle:
            handle.write(yaml)
        try:
            result = subprocess.run([mcuemu, "sim", path, "--for", "9.5s"], stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=300, check=False)
        except subprocess.TimeoutExpired:
            return fail("emulator timed out")
        if result.returncode != 0:
            return fail(f"emulator exited {result.returncode}")
        text = result.stdout.decode(errors="replace")
        with open(pcap, "rb") as handle:
            capture = handle.read()
    # The NCP's beacon request: MAC command 0x07 to the broadcast address.
    frames, offset = [], 24
    while offset + 16 <= len(capture):
        _, _, length, _ = struct.unpack("<IIII", capture[offset:offset + 16])
        frames.append(capture[offset + 16:offset + 16 + length])
        offset += 16 + length
    if not any(len(f) == 8 and f[0] == 0x03 and f[-1] == 0x07 for f in frames):
        return fail("no beacon request on the air")
    if not any(len(f) > 8 and f[0] == 0x00 and f[1] & 0xC0 == 0x80 for f in frames):
        return fail("the coordinator sent no beacon")
    found = False
    for line in text.splitlines():
        match = re.match(r"\[\s*[\d.]+ s\] ncp\.mcu\.uart0\s+\d+ bytes: (.*)", line)
        if not match:
            continue
        for kind, detail in ash_frame.parse(bytes.fromhex(match.group(1).replace(" ", ""))):
            # networkFoundHandler: id 0x001B, then channel (0x0B) and the PAN ID.
            if kind == "DATA" and " 1b 00 0b " in detail:
                found = True
    if not found:
        return fail("the host never received a networkFoundHandler callback for channel 11")
    print("firmware_efr32mg21_ncp_scan: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
