#!/usr/bin/env python3
"""Verify the closed TS0001 image joins a network and exchanges NWK data."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

from zigbee_air_check import nwk_data, read_pcap

TS0001_EUI = bytes.fromhex("f6897f9b7438c1a4")
NVM_START = 0x79000
NVM_SIZE = 0x6000


def mac_addresses(raw: bytes) -> tuple[bytes | None, bytes | None, bytes]:
    """Return extended destination/source addresses and the MAC payload."""
    if len(raw) < 3:
        return None, None, b""
    fcf = raw[0] | raw[1] << 8
    dst_mode = (fcf >> 10) & 3
    src_mode = (fcf >> 14) & 3
    pan_compress = bool(fcf & 0x40)
    pos = 3
    dst_eui = src_eui = None
    if dst_mode:
        pos += 2
        if dst_mode == 2:
            pos += 2
        elif dst_mode == 3:
            dst_eui = raw[pos : pos + 8]
            pos += 8
        else:
            return None, None, b""
    if src_mode:
        if not pan_compress:
            pos += 2
        if src_mode == 2:
            pos += 2
        elif src_mode == 3:
            src_eui = raw[pos : pos + 8]
            pos += 8
        else:
            return dst_eui, None, b""
    return dst_eui, src_eui, raw[pos:]


def fail(message: str, output: str = "") -> int:
    print(f"firmware_phy6252_ts0001_join: {message}", file=sys.stderr)
    if output:
        print("--- emulator output ---", file=sys.stderr)
        print(output, file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) not in (7, 8):
        print(
            "usage: test_phy6252_ts0001_join.py EMULATOR COORDINATOR SAMPLE_LIGHT "
            "FIRMWARE EFUSE RF_CALIBRATION [ROM]",
            file=sys.stderr,
        )
        return 2

    emulator, coordinator, sample_light, firmware, efuse, rf_calibration = (
        str(Path(path).resolve()) for path in sys.argv[1:7]
    )
    rom = str(Path(sys.argv[7]).resolve()) if len(sys.argv) == 8 else None

    image = bytearray(Path(firmware).read_bytes())
    if len(image) != 0x80000:
        return fail(f"expected a 512 KiB full-flash image, got {len(image)} bytes")
    image[NVM_START : NVM_START + NVM_SIZE] = b"\xff" * NVM_SIZE

    with tempfile.TemporaryDirectory(prefix="mcuemu-phy6252-ts0001-join-") as scratch:
        workdir = Path(scratch)
        firstboot_image = workdir / "ts0001-firstboot.bin"
        firstboot_image.write_bytes(image)
        rom_option = f"        rom: {json.dumps(rom)}\n" if rom else ""
        system = workdir / "system.yaml"
        system.write_text(
            f"""version: 1
world:
  seed: 1
  duration: 20s
  radio:
    - id: air0
      model: log_distance
      reference_loss_db: 40
      path_loss_exponent: 2.5
devices:
  - id: coordinator
    mcus:
      - id: mcu
        model: tlsr8258
        variant: F1K
        firmware: {json.dumps(coordinator)}
        radio: {{ air: air0, position: [0, 0, 1], tx_power_dbm: 10, protocols: [zigbee] }}
  - id: sampleLight
    mcus:
      - id: mcu
        model: tlsr8258
        variant: F1K
        firmware: {json.dumps(sample_light)}
        start_at: 5s
        radio: {{ air: air0, position: [2, 0, 1], tx_power_dbm: 10, protocols: [zigbee] }}
  - id: ts0001
    mcus:
      - id: mcu
        model: phy6252
        firmware: {json.dumps(str(firstboot_image))}
{rom_option}        efuse: {json.dumps(efuse)}
        rf_calibration: {json.dumps(rf_calibration)}
        radio: {{ air: air0, position: [3, 0, 1], tx_power_dbm: 0, protocols: [zigbee] }}
    nets:
      - {{ id: key8_line, pull: up, pins: [mcu.GPIO8] }}
    parts:
      - {{ id: key8, type: button, pins: {{ out: key8_line }} }}
attach:
  - {{ endpoint: air0, sink: "pcap:air.pcap" }}
scenario:
  events:
    - {{ at: 1s, endpoint: ts0001.key8.pressed, set: true }}
    - {{ at: 8s, endpoint: ts0001.key8.pressed, set: false }}
"""
        )

        try:
            result = subprocess.run(
                [emulator, "sim", str(system), "--trace-uart", "*=text"],
                cwd=workdir,
                capture_output=True,
                text=True,
                timeout=900,
                check=False,
            )
        except subprocess.TimeoutExpired as error:
            return fail("emulator exceeded the 900-second timeout", error.stdout or "")

        output = result.stdout + result.stderr
        if result.returncode != 0:
            return fail(f"emulator exited with status {result.returncode}", output)
        if "simulated 20000000000 ns" not in output:
            return fail("simulation did not reach its 20-second deadline", output)

        pcap = workdir / "air.pcap"
        if not pcap.exists():
            return fail("radio pcap was not written", output)
        frames = read_pcap(pcap)
        association_requests = []
        for frame in frames:
            _dst_eui, src_eui, payload = mac_addresses(frame.raw)
            if frame.kind == 3 and payload[:1] == b"\x01" and src_eui == TS0001_EUI:
                association_requests.append(frame)
        if not association_requests:
            return fail("TS0001 did not send a MAC Association Request", output)

        association = None
        short_address = None
        for frame in frames:
            dst_eui, _src_eui, payload = mac_addresses(frame.raw)
            if (
                frame.kind == 3
                and dst_eui == TS0001_EUI
                and payload[:1] == b"\x02"
                and len(payload) >= 4
                and payload[3] == 0
            ):
                association = frame
                short_address = payload[1] | payload[2] << 8
                break
        if association is None or short_address is None:
            return fail("coordinator did not successfully associate TS0001", output)

        uplink = unicast_uplink = downlink = 0
        for frame in frames:
            if frame.t_ns < association.t_ns:
                continue
            addresses = nwk_data(frame)
            if addresses is None:
                continue
            destination, source = addresses
            if source == short_address:
                uplink += 1
                if destination < 0xFFF8:
                    unicast_uplink += 1
            if destination == short_address and source == 0:
                downlink += 1
        if not uplink or not unicast_uplink or not downlink:
            return fail(
                "TS0001 associated but did not exchange bidirectional unicast NWK data "
                f"(up={uplink}, unicast={unicast_uplink}, down={downlink})",
                output,
            )

    print(
        "firmware_phy6252_ts0001_join: ok "
        f"(short=0x{short_address:04x}, up={uplink}, down={downlink})"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
