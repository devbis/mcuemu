#!/usr/bin/env python3
"""Verify the closed TS0001 image joins a coordinator's network on its own.

The optional ROM is the dumped PHY6222 ROM (without it the host stand-ins run).
COORDINATOR is the Telink sampleGW (it forms its network on channel 15; the relay
scans channels 11..26). Frames are attributed to a device by the air sink's
transmitter label, never by the EUI inside the frame: another node can carry the
same address.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

from zigbee_air_check import nwk_data, parse_mac

LINE = re.compile(
    r"^(?P<ns>\d+) air0 = \S+ ch(?P<channel>\d+) (?P<node>\S+) -> \S+ \d+B (?P<hex>[0-9a-f]+)"
)
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
    if len(sys.argv) not in (6, 7):
        print(
            "usage: test_phy6252_ts0001_join.py EMULATOR COORDINATOR "
            "FIRMWARE EFUSE RF_CALIBRATION [ROM]",
            file=sys.stderr,
        )
        return 2

    emulator, coordinator, firmware, efuse, rf_calibration = (
        str(Path(path).resolve()) for path in sys.argv[1:6]
    )
    rom = str(Path(sys.argv[6]).resolve()) if len(sys.argv) == 7 else None

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
  - {{ endpoint: air0, sink: "stdout" }}
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

        sent = []
        for line in result.stdout.splitlines():
            match = LINE.match(line)
            if match:
                frame = parse_mac(bytes.fromhex(match["hex"]), int(match["ns"]))
                if frame is not None:
                    sent.append((match["node"], int(match["channel"]), frame))
        own = [(channel, frame) for node, channel, frame in sent if node == "ts0001.mcu"]
        if not own:
            return fail("TS0001 transmitted nothing", output)

        requests = [
            frame for _channel, frame in own
            if frame.kind == 3 and frame.payload[:1] == b"\x01"
        ]
        if not requests:
            channels = sorted({channel for channel, _frame in own})
            return fail(
                f"TS0001 did not send a MAC Association Request (it transmitted "
                f"{len(own)} frame(s) on channel(s) {channels})",
                output,
            )
        _dst_eui, own_eui, _payload = mac_addresses(requests[0].raw)

        association = None
        short_address = None
        for node, _channel, frame in sent:
            dst_eui, _src_eui, payload = mac_addresses(frame.raw)
            if (
                node == "coordinator.mcu"
                and frame.kind == 3
                and dst_eui == own_eui
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
        for node, _channel, frame in sent:
            if frame.t_ns < association.t_ns:
                continue
            addresses = nwk_data(frame)
            if addresses is None:
                continue
            destination, source = addresses
            if node == "ts0001.mcu" and source == short_address:
                uplink += 1
                if destination < 0xFFF8:
                    unicast_uplink += 1
            if node == "coordinator.mcu" and destination == short_address and source == 0:
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
