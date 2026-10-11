#!/usr/bin/env python3
"""Run the local sampleGW/sampleLight images through the virtual Zigbee air."""

from __future__ import annotations

import re
import subprocess
import sys
from typing import Optional


STEPS = "180000000"
BUTTON_STEPS = "200000000"
JOIN_FLAG_ADDRESS = "0x84783d"
JOIN_FLAG_MASK = 0x04
PWM_ENABLE_ADDRESS = "0x800780"
PWM_ENABLE_MASK = 0x06

FRAME_RE = re.compile(
    r"^\s+(\d+) (\S+) -> (\S+) zigbee-802\.15\.4 ch=(\d+) "
    r"bytes=(\d+) t=(\d+)\.\.(\d+) ns\n\s+payload=([0-9a-f]+)\s*$",
    re.MULTILINE,
)
RECEPTION_RE = re.compile(r"^\s+frame (\d+) -> (\S+)\s*$", re.MULTILINE)
JOIN_FLAG_RE = re.compile(
    rf"^node light SRAM {JOIN_FLAG_ADDRESS} = ([0-9a-f]{{2}})$", re.MULTILINE
)
PWM_ENABLE_RE = re.compile(
    rf"^node light MMIO {PWM_ENABLE_ADDRESS} = ([0-9a-f]{{2}})$", re.MULTILINE
)
GPIO_EVENT_RE = re.compile(
    r"^gpio: node coordinator port D=0x([0-9a-f]+) at (\d+) ns$", re.MULTILINE
)


def mac_command_id(payload: bytes) -> Optional[int]:
    """Return the MAC command ID for a legacy IEEE 802.15.4 command frame."""
    if len(payload) < 3:
        return None

    frame_control = int.from_bytes(payload[:2], "little")
    if frame_control & 0x7 != 0x3:
        return None

    # This firmware pair emits 2003/2006-format MAC frames. Decode the MAC
    # header lengths so command IDs are checked semantically, not as byte blobs.
    frame_version = (frame_control >> 12) & 0x3
    if frame_version > 1:
        return None

    offset = 2
    if not (frame_control & (1 << 8)):
        offset += 1  # sequence number

    address_lengths = {0: 0, 2: 2, 3: 8}
    destination_mode = (frame_control >> 10) & 0x3
    source_mode = (frame_control >> 14) & 0x3
    if destination_mode not in address_lengths or source_mode not in address_lengths:
        return None

    if destination_mode:
        offset += 2  # destination PAN ID
        offset += address_lengths[destination_mode]
    if source_mode:
        if not (frame_control & (1 << 6)):
            offset += 2  # source PAN ID, unless compressed
        offset += address_lengths[source_mode]

    return payload[offset] if offset < len(payload) else None


def fail(message: str, output: str) -> None:
    print(f"firmware_zigbee_join: {message}", file=sys.stderr)
    if output:
        print("--- emulator output ---", file=sys.stderr)
        print(output, file=sys.stderr)
    raise SystemExit(1)


def main() -> int:
    if len(sys.argv) not in (4, 5):
        print(
            "usage: test_zigbee_join.py EMULATOR SAMPLE_GW SAMPLE_LIGHT "
            "[join|button]",
            file=sys.stderr,
        )
        return 2

    emulator, coordinator_firmware, light_firmware = sys.argv[1:4]
    scenario = sys.argv[4] if len(sys.argv) == 5 else "join"
    if scenario not in ("join", "button"):
        print(f"unknown scenario: {scenario}", file=sys.stderr)
        return 2

    steps = STEPS if scenario == "join" else BUTTON_STEPS
    command = [
        emulator,
        "network",
        "--node",
        f"coordinator={coordinator_firmware}",
        "--node",
        f"light={light_firmware}",
        "--model",
        "TLSR8258F512",
        "--channel",
        "15",
        "--steps",
        steps,
        "--start-at",
        "light=5s",
        "--radio-payloads",
        "--dump-sram",
        f"light={JOIN_FLAG_ADDRESS}:1",
    ]
    if scenario == "button":
        command.extend(
            [
                "--gpio-event",
                "coordinator=D:0xbf@11.3s",
                "--gpio-event",
                "coordinator=D:0xff@11.5s",
                "--dump-mmio",
                f"light={PWM_ENABLE_ADDRESS}:1",
            ]
        )

    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=590,
        )
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout or ""
        stderr = error.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors="replace")
        output = stdout + stderr
        fail("emulator exceeded the 590-second test timeout", output)

    output = result.stdout + result.stderr
    if result.returncode != 0:
        fail(f"emulator exited with status {result.returncode}", output)

    joined_match = JOIN_FLAG_RE.search(output)
    if not joined_match:
        fail(f"could not find the light's SRAM byte at {JOIN_FLAG_ADDRESS}", output)
    joined_flags = int(joined_match.group(1), 16)
    if joined_flags & JOIN_FLAG_MASK == 0:
        fail(
            f"SDK joined flag is clear: {JOIN_FLAG_ADDRESS}=0x{joined_flags:02x}",
            output,
        )

    frames = []
    for match in FRAME_RE.finditer(output):
        (
            frame_id,
            source,
            _destination,
            channel,
            byte_count,
            start_ns,
            _end_ns,
            payload_hex,
        ) = match.groups()
        payload = bytes.fromhex(payload_hex)
        if len(payload) != int(byte_count):
            fail(f"frame {frame_id} payload length does not match its summary", output)
        frames.append(
            {
                "id": int(frame_id),
                "source": source,
                "channel": int(channel),
                "start_ns": int(start_ns),
                "command_id": mac_command_id(payload),
            }
        )

    receptions = {
        (int(frame_id), receiver)
        for frame_id, receiver in RECEPTION_RE.findall(output)
    }
    association_requests = [
        frame
        for frame in frames
        if frame["source"] == "light"
        and frame["channel"] == 15
        and frame["command_id"] == 0x01
    ]
    if not association_requests:
        fail("light did not transmit an IEEE 802.15.4 association request", output)

    delivered_requests = [
        frame
        for frame in association_requests
        if (frame["id"], "coordinator") in receptions
    ]
    if not delivered_requests:
        fail("coordinator did not receive the light's association request", output)

    association_responses = [
        frame
        for frame in frames
        if frame["source"] == "coordinator"
        and frame["channel"] == 15
        and frame["command_id"] == 0x02
    ]
    if not association_responses:
        fail("coordinator did not transmit an IEEE 802.15.4 association response", output)

    delivered_responses = [
        frame
        for frame in association_responses
        if (frame["id"], "light") in receptions
    ]
    if not delivered_responses:
        fail("light did not receive the coordinator's association response", output)

    summary = (
        f"light joined flag=0x{joined_flags:02x}, association request frame "
        f"{delivered_requests[0]['id']} received by coordinator, association "
        f"response frame {delivered_responses[0]['id']} received by light"
    )

    if scenario == "button":
        button_events = {
            (int(value, 16), int(time_ns))
            for value, time_ns in GPIO_EVENT_RE.findall(output)
        }
        if not any(value == 0xBF and time_ns >= 11_300_000_000
                   for value, time_ns in button_events):
            fail("sampleGW SW1 press on PD6 was not applied", output)
        if not any(value == 0xFF and time_ns >= 11_500_000_000
                   for value, time_ns in button_events):
            fail("sampleGW SW1 release on PD6 was not applied", output)

        button_frames = [
            frame
            for frame in frames
            if frame["source"] == "coordinator"
            and frame["channel"] == 15
            and frame["start_ns"] >= 12_000_000_000
            and (frame["id"], "light") in receptions
        ]
        if not button_frames:
            fail("light did not receive the coordinator's post-button frame", output)

        pwm_match = PWM_ENABLE_RE.search(output)
        if not pwm_match:
            fail(f"could not find PWM enable register {PWM_ENABLE_ADDRESS}", output)
        pwm_enable = int(pwm_match.group(1), 16)
        if pwm_enable & PWM_ENABLE_MASK != PWM_ENABLE_MASK:
            fail(
                f"light PWM output channels are not enabled: "
                f"{PWM_ENABLE_ADDRESS}=0x{pwm_enable:02x}",
                output,
            )
        summary += (
            f", post-button frame {button_frames[0]['id']} received by light, "
            f"PWM enable=0x{pwm_enable:02x}"
        )

    print(f"firmware_zigbee_{scenario}: passed; {summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
