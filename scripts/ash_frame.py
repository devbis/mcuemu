#!/usr/bin/env python3
"""Build and parse EZSP-UART (ASH) frames for `mcuemu run --uart0-send` and the NCP tests.

    ash_frame.py rst                 # the RST frame, hex
    ash_frame.py data SEQ ACK HEX    # a DATA frame (frame number SEQ, ack number ACK) carrying EZSP bytes HEX
    ash_frame.py parse HEX           # decode the frames in a UART capture
"""
import sys

FLAG, ESCAPE = 0x7E, 0x7D
RESERVED = {0x7E, 0x11, 0x13, 0x18, 0x1A, 0x7D}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def randomize(data: bytes) -> bytes:
    out, rand = bytearray(), 0x42
    for byte in data:
        out.append(byte ^ rand)
        rand = (rand >> 1) ^ 0xB8 if rand & 1 else rand >> 1
    return bytes(out)


def stuff(data: bytes) -> bytes:
    out = bytearray()
    for byte in data:
        if byte in RESERVED:
            out += bytes([ESCAPE, byte ^ 0x20])
        else:
            out.append(byte)
    return bytes(out)


def frame(control: int, payload: bytes = b"", randomized: bool = False) -> bytes:
    body = bytes([control]) + (randomize(payload) if randomized else payload)
    body += crc16(body).to_bytes(2, "big")
    return stuff(body) + bytes([FLAG])


def data_frame(seq: int, ack: int, ezsp: bytes) -> bytes:
    return frame((seq & 7) << 4 | (ack & 7), ezsp, randomized=True)


def parse(capture: bytes):
    """Yield (kind, detail) for each frame in a UART capture (0x1A cancels what came before)."""
    current = bytearray()
    for byte in capture:
        if byte == 0x1A:
            current.clear()
        elif byte == FLAG:
            raw, escaped, body = bytearray(), False, bytes(current)
            for b in body:
                if escaped:
                    raw.append(b ^ 0x20)
                    escaped = False
                elif b == ESCAPE:
                    escaped = True
                else:
                    raw.append(b)
            current.clear()
            if len(raw) < 3 or crc16(bytes(raw[:-2])) != int.from_bytes(raw[-2:], "big"):
                yield ("bad", bytes(raw).hex(" "))
                continue
            control, payload = raw[0], bytes(raw[1:-2])
            if control == 0xC1:
                yield ("RSTACK", f"version {payload[0]}, reset code 0x{payload[1]:02x}")
            elif control == 0xC2:
                yield ("ERROR", f"version {payload[0]}, code 0x{payload[1]:02x}")
            elif control == 0xC0:
                yield ("RST", "")
            elif control & 0x80 == 0:
                yield ("DATA", f"seq {control >> 4 & 7} ack {control & 7} retx {control >> 3 & 1}: "
                               + randomize(payload).hex(" "))
            elif control & 0xE0 == 0x80:
                yield ("ACK", f"ack {control & 7}")
            elif control & 0xE0 == 0xA0:
                yield ("NAK", f"ack {control & 7}")
            else:
                yield ("?", f"control 0x{control:02x}")
        else:
            current.append(byte)


def main() -> int:
    if len(sys.argv) >= 2 and sys.argv[1] == "rst":
        print((b"\x1a" + frame(0xC0)).hex())
    elif len(sys.argv) == 5 and sys.argv[1] == "data":
        print(data_frame(int(sys.argv[2]), int(sys.argv[3]), bytes.fromhex(sys.argv[4])).hex())
    elif len(sys.argv) >= 3 and sys.argv[1] == "parse":
        for kind, detail in parse(bytes.fromhex("".join(sys.argv[2:]))):
            print(f"{kind}: {detail}")
    else:
        print(__doc__, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
