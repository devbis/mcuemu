#!/usr/bin/env python3
"""A host program for examples/tlsr8258-atc-gatt.yaml: it speaks HCI over TCP to the ble_host part.

usage: tlsr8258-atc-gatt.py PORT SECONDS

It creates a connection to the ATC firmware (A4:C1:38:02:34:60), then asks over ATT for the MTU, the
primary services, the characteristics and the Device Name, answers the L2CAP request of the firmware
to update the connection parameters and passes them on with LE Connection Update, and prints what
comes back with the time in seconds of the program's own clock.
"""

import socket
import struct
import sys
import time

PEER = bytes.fromhex("a4c138023460")[::-1]
ATT_REQUESTS = (
    ("Exchange MTU", bytes([0x02, 0x17, 0x00])),
    ("Read By Group Type (services)", bytes([0x10, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x28])),
    ("Read By Type (characteristics)", bytes([0x08, 0x01, 0x00, 0xFF, 0xFF, 0x03, 0x28])),
    ("Read (handle 3, the Device Name)", bytes([0x0A, 0x03, 0x00])),
)


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    port, seconds = int(sys.argv[1]), float(sys.argv[2])
    while True:
        try:
            sock = socket.create_connection(("127.0.0.1", port))
            break
        except OSError:
            time.sleep(0.02)
    sock.settimeout(0.2)
    started = time.time()

    def log(message: str) -> None:
        print(f"{time.time() - started:5.2f} {message}", flush=True)

    def command(opcode: int, params: bytes = b"") -> None:
        sock.sendall(bytes([1]) + struct.pack("<HB", opcode, len(params)) + params)

    def l2cap(handle: int, cid: int, data: bytes) -> None:
        sock.sendall(bytes([2]) + struct.pack("<HHHH", handle | 0x2000, len(data) + 4, len(data), cid) + data)

    command(0x0C01, bytes([0xFF] * 7 + [0x3F]))  # Set Event Mask
    command(0x2001, bytes([0xFF] * 8))  # LE Set Event Mask
    # LE Create Connection: interval 30 ms, latency 0, supervision timeout 2 s
    command(0x200D, struct.pack("<HHBB", 0x10, 0x10, 0, 0) + PEER + struct.pack("<BHHHHHH", 0, 0x18, 0x18, 0, 0xC8, 0, 0))

    handle = None
    connected_at = None
    pending = list(ATT_REQUESTS)
    names = []
    buffer = b""
    while time.time() - started < seconds:
        try:
            buffer += sock.recv(4096)
        except socket.timeout:
            pass
        while len(buffer) >= 3 and buffer[0] == 4 and len(buffer) >= 3 + buffer[2]:
            event, payload = buffer[1], buffer[3 : 3 + buffer[2]]
            buffer = buffer[3 + buffer[2] :]
            if event == 0x3E and payload[0] == 1 and payload[1] == 0:
                handle = struct.unpack("<H", payload[2:4])[0]
                connected_at = time.time()
                log("connected")
            elif event == 0x3E and payload[0] == 3:
                interval, latency, timeout = struct.unpack("<HHH", payload[4:10])
                log(f"LE Connection Update Complete: interval {interval * 1.25} ms, latency {latency}, timeout {timeout * 10} ms")
            elif event == 0x05:
                log(f"disconnected, reason {payload[3]:#04x}")
        while len(buffer) >= 5 and buffer[0] == 2 and len(buffer) >= 5 + struct.unpack("<H", buffer[3:5])[0]:
            size = struct.unpack("<H", buffer[3:5])[0]
            data, buffer = buffer[5 : 5 + size], buffer[5 + size :]
            cid, payload = struct.unpack("<H", data[2:4])[0], data[4:]
            if cid == 4:
                log(f"ATT {payload.hex()}" + (f"  ({payload[1:].decode(errors='replace')!r})" if payload[:1] == b"\x0b" else ""))
            elif cid == 5 and payload[0] == 0x12:
                low, high, latency, timeout = struct.unpack("<HHHH", payload[4:12])
                log(f"the firmware asks for interval {low * 1.25}..{high * 1.25} ms, latency {latency}, timeout {timeout * 10} ms")
                l2cap(handle, 5, bytes([0x13, payload[1], 2, 0, 0, 0]))  # accepted
                command(0x2013, struct.pack("<HHHHHHH", handle, low, high, latency, timeout, 0, 0))
        if handle is not None and pending and time.time() - connected_at > 0.1 + 0.5 * (len(ATT_REQUESTS) - len(pending)):
            title, request = pending.pop(0)
            l2cap(handle, 4, request)
            log(f"ATT request: {title}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
