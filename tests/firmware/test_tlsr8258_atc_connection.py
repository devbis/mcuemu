#!/usr/bin/env python3
"""Check that the ATC firmware (pvvx) of the LYWSD03MMC keeps a BLE connection on the TLSR8258 model.

The firmware runs with a ble_host part next to it. The test, as a HCI controller host over TCP,
scans nothing and creates a connection to the address of the device (the ATC advertises with
A4:C1:38:02:34:60), so the firmware receives a CONNECT_IND, answers the connection events and sleeps
between them. The connection must come up and must still be there well after the supervision
timeout of 2 s that the test asks for: the stack drops a connection when it sees no valid packet,
which it did while the radio model wrote no CRC behind the received PDUs and returned a stale 32 kHz
counter to the sleep code. The model has no transmit queue yet, so the test does not expect an answer
to an ATT request.

The emulator runs at the speed of the wall clock (--speed 1) because the host is a program outside.
"""

from __future__ import annotations

import json
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

SYSTEM = """version: 1
world:
  seed: 1
  duration: 20s
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
    parts:
      - id: host
        type: ble_host
        radio: {{ air: air0, position: [1, 0, 1], protocols: [ble] }}
        params: {{ address: "C0:DE:00:00:00:01" }}
attach:
  - {{ endpoint: atc.host.hci, sink: "tcp:{port}" }}
"""

PEER = bytes.fromhex("a4c138023460")[::-1]
CONNECT_TIMEOUT_S = 12.0
HOLD_S = 3.5  # longer than the supervision timeout of 2 s


def fail(message: str, output: str = "") -> int:
    print(f"test_tlsr8258_atc_connection: {message}", file=sys.stderr)
    if output:
        print(output, file=sys.stderr)
    return 1


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def hci_command(sock: socket.socket, opcode: int, params: bytes = b"") -> None:
    sock.sendall(bytes([1]) + struct.pack("<HB", opcode, len(params)) + params)


def run_host(port: int, emulator: subprocess.Popen) -> str | None:
    """None when the connection came up and held, else the reason it did not."""
    deadline = time.time() + 20
    while True:
        try:
            sock = socket.create_connection(("127.0.0.1", port))
            break
        except OSError:
            if emulator.poll() is not None or time.time() > deadline:
                return "cannot connect to the HCI port of the host"
            time.sleep(0.02)
    sock.settimeout(0.2)
    hci_command(sock, 0x0C01, bytes([0xFF] * 7 + [0x3F]))  # Set Event Mask
    hci_command(sock, 0x2001, bytes([0xFF] * 8))  # LE Set Event Mask
    # LE Create Connection: interval 30 ms, latency 0, supervision timeout 2 s
    hci_command(
        sock,
        0x200D,
        struct.pack("<HHBB", 0x10, 0x10, 0, 0)
        + PEER
        + struct.pack("<BHHHHHH", 0, 0x18, 0x18, 0, 0xC8, 0, 0),
    )
    started = time.time()
    connected_at = None
    buffer = b""
    while True:
        now = time.time()
        if connected_at is None and now - started > CONNECT_TIMEOUT_S:
            return "the connection did not come up"
        if connected_at is not None and now - connected_at > HOLD_S:
            return None
        try:
            buffer += sock.recv(4096)
        except socket.timeout:
            pass
        while len(buffer) >= 3 and buffer[0] == 4 and len(buffer) >= 3 + buffer[2]:
            event, length = buffer[1], buffer[2]
            payload = buffer[3 : 3 + length]
            buffer = buffer[3 + length :]
            if event == 0x3E and payload[0] == 1:  # LE Connection Complete
                if payload[1] != 0:
                    return f"LE Connection Complete with status {payload[1]:#04x}"
                connected_at = time.time()
            elif event == 0x05:  # Disconnection Complete
                return f"disconnected with reason {payload[3]:#04x} after {now - (connected_at or now):.1f} s"
        if emulator.poll() is not None:
            return "the emulator exited"


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_tlsr8258_atc_connection.py EMULATOR FIRMWARE", file=sys.stderr)
        return 2
    emulator_path, firmware = (str(Path(path).resolve()) for path in sys.argv[1:])

    port = free_port()
    with tempfile.TemporaryDirectory(prefix="mcuemu-atc-conn-") as scratch:
        workdir = Path(scratch)
        system = workdir / "system.yaml"
        system.write_text(SYSTEM.format(firmware=json.dumps(firmware), port=port))
        emulator = subprocess.Popen(
            [emulator_path, "sim", str(system), "--speed", "1", "--for", "20s"],
            cwd=workdir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            problem = run_host(port, emulator)
        finally:
            if emulator.poll() is None:
                emulator.send_signal(signal.SIGINT)
            try:
                output, _ = emulator.communicate(timeout=30)
            except subprocess.TimeoutExpired:
                emulator.kill()
                output, _ = emulator.communicate()
    if problem is not None:
        return fail(problem, output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
