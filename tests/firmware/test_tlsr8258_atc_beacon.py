#!/usr/bin/env python3
"""Check that the ATC firmware of the LYWSD03MMC (pvvx) advertises on the TLSR8258 model.

The image is the BLE firmware ATC_v59.bin of https://github.com/pvvx/ATC_MiThermometer. The test runs
it alone on the air for 6 s and reads the capture (LE_LL_WITH_PHDR): the firmware sends its
advertisement on the channels 37, 38 and 39 with the advertising access address, several times.
"""

from __future__ import annotations

import json
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

SYSTEM = """version: 1
world:
  seed: 1
  duration: 6s
  radio:
    - id: air0
devices:
  - id: atc
    mcus:
      - id: mcu
        model: tlsr8258
        variant: F512
        firmware: {firmware}
        radio: {{ air: air0, position: [0, 0, 1], tx_power_dbm: 4, protocols: [ble] }}
attach:
  - {{ endpoint: air0, sink: "pcap:atc-ble.pcap,ble" }}
"""

ADV_ACCESS_ADDRESS = 0x8E89BED6
LINKTYPE_BLE_LL_WITH_PHDR = 256


def fail(message: str, output: str = "") -> int:
    print(f"test_tlsr8258_atc_beacon: {message}", file=sys.stderr)
    if output:
        print(output, file=sys.stderr)
    return 1


def read_frames(path: Path) -> list[tuple[int, int, bytes]]:
    """(channel, access address, PDU without CRC) of every frame of the capture."""
    data = path.read_bytes()
    if len(data) < 24 or struct.unpack_from("<I", data, 20)[0] != LINKTYPE_BLE_LL_WITH_PHDR:
        raise ValueError("not a BLE link-layer capture")
    frames = []
    offset = 24
    while offset + 16 <= len(data):
        included = struct.unpack_from("<I", data, offset + 8)[0]
        packet = data[offset + 16 : offset + 16 + included]
        offset += 16 + included
        # 10 bytes of header (channel, power, noise, offenses, reference address, flags), the
        # access address, the PDU and the 3 bytes of CRC
        channel = packet[0]
        access_address = struct.unpack_from("<I", packet, 10)[0]
        frames.append((channel, access_address, packet[14:-3]))
    return frames


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_tlsr8258_atc_beacon.py EMULATOR FIRMWARE", file=sys.stderr)
        return 2
    emulator, firmware = (str(Path(path).resolve()) for path in sys.argv[1:])

    with tempfile.TemporaryDirectory(prefix="mcuemu-atc-") as scratch:
        workdir = Path(scratch)
        system = workdir / "system.yaml"
        system.write_text(SYSTEM.format(firmware=json.dumps(firmware)))
        try:
            result = subprocess.run(
                [emulator, "sim", str(system)],
                cwd=workdir,
                capture_output=True,
                text=True,
                timeout=300,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return fail("emulator exceeded the 300-second timeout")
        output = result.stdout + result.stderr
        if result.returncode != 0:
            return fail(f"emulator exited with status {result.returncode}", output)
        try:
            frames = read_frames(workdir / "atc-ble.pcap")
        except (OSError, ValueError) as error:
            return fail(f"cannot read the capture: {error}", output)

    if not frames:
        return fail("no advertisement on the air", output)
    per_channel = {channel: 0 for channel in (37, 38, 39)}
    for channel, access_address, pdu in frames:
        if channel not in per_channel:
            return fail(f"frame on channel {channel}, only 37, 38 and 39 are advertising channels")
        per_channel[channel] += 1
        if access_address != ADV_ACCESS_ADDRESS:
            return fail(f"access address {access_address:#010x} is not the advertising one")
        if len(pdu) < 8 or pdu[1] + 2 != len(pdu):
            return fail(f"the length of the PDU is wrong: {pdu.hex()}")
        # ADV_IND (0), ADV_NONCONN_IND (2) or ADV_SCAN_IND (6) with the advertiser address
        if pdu[0] & 0x0F not in (0, 2, 6):
            return fail(f"unexpected PDU type {pdu[0] & 0x0F:#x}: {pdu.hex()}")
    # The advertising event repeats on the three channels, with a short interval at the start
    if min(per_channel.values()) < 3:
        return fail(f"too few advertising events: {per_channel}")
    # The address of the device is that of Telink (A4:C1:38) in the PDU, least significant byte first
    if not all(pdu[5:8] == bytes((0x38, 0xC1, 0xA4)) for _, _, pdu in frames):
        return fail("the advertiser address is not in the Telink range A4:C1:38")
    return 0


if __name__ == "__main__":
    sys.exit(main())
