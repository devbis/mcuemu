#!/usr/bin/env python3
"""Extract the application image from a Gecko Bootloader .gbl file.

    gbl2bin.py IN.gbl OUT.bin

The GBL is a list of tagged blocks (tag u32, length u32, payload). The program-data tag 0xFD0303FD
holds a flash address (u32) and the bytes for it; for the Silicon Labs firmware builds that is the
whole application, linked at 0x4000, which `mcuemu run --model EFR32MG21` places there and starts
(any image whose reset vector is at or above 0x4000 is treated as an application-slot image).
"""
import struct
import sys

PROGRAM = 0xFD0303FD
HEADER = 0x03A617EB


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    data = open(sys.argv[1], "rb").read()
    offset = 0
    image = bytearray()
    base = None
    while offset + 8 <= len(data):
        tag, length = struct.unpack_from("<II", data, offset)
        payload = data[offset + 8: offset + 8 + length]
        if offset == 0 and tag != HEADER:
            print("not a GBL file (bad header tag)", file=sys.stderr)
            return 1
        if tag == PROGRAM:
            address = struct.unpack_from("<I", payload)[0]
            chunk = payload[4:]
            if base is None:
                base = address
            if address - base != len(image):
                print(f"program data at 0x{address:x} is not contiguous", file=sys.stderr)
                return 1
            image += chunk
        offset += 8 + length
    if base is None:
        print("no program data in the GBL", file=sys.stderr)
        return 1
    open(sys.argv[2], "wb").write(image)
    print(f"{len(image)} bytes for 0x{base:x} -> {sys.argv[2]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
