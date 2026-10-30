#!/usr/bin/env python3
"""Inspect the experimental PHY6252 ROM segment table.

The Zephyr PB-03F packer emits a file whose first byte belongs at flash
offset 0x2000. A full flash dump instead contains that table at offset 0x2000.
Segment sources may be physical offsets or XIP addresses in the 0x11000000 map.
"""

import argparse
import json
import struct
from pathlib import Path

TABLE_OFFSET = 0x2000
RECORD_OFFSET = 0x100
FLASH_SIZE = 0x80000
RAM_BASE = 0x1FFF0000
RAM_END = 0x20000000
FLASH_BASE = 0x11000000


def inspect(raw, full_flash):
    table_offset = TABLE_OFFSET if full_flash else 0
    source_origin = 0 if full_flash else TABLE_OFFSET
    if len(raw) < table_offset + RECORD_OFFSET:
        raise ValueError("image is shorter than the segment table header")
    count, reserved, run_address = struct.unpack_from("<III", raw, table_offset)
    # The physical segment table must fit before the application slot at 0x10000.
    max_records = (0x10000 - TABLE_OFFSET - RECORD_OFFSET) // 16
    if not 1 <= count <= max_records:
        raise ValueError(f"invalid segment count {count}")
    if table_offset + RECORD_OFFSET + count * 16 > min(len(raw), 0x10000):
        raise ValueError("segment table extends beyond the image")
    if not RAM_BASE <= run_address <= RAM_END - 8:
        raise ValueError(f"run descriptor 0x{run_address:08x} is outside SRAM")
    records = []
    descriptor = None
    for index in range(count):
        src, size, dst, crc = struct.unpack_from(
            "<IIII", raw, table_offset + RECORD_OFFSET + index * 16)
        source_offset = src - FLASH_BASE if src >= FLASH_BASE else src
        file_offset = source_offset - source_origin
        if (source_offset < source_origin or file_offset + size > len(raw) or
                source_offset + size > FLASH_SIZE):
            raise ValueError(f"segment {index} source 0x{src:08x}+0x{size:x} is outside the image or flash")
        if RAM_BASE <= dst and dst + size <= RAM_END:
            kind = "sram"
        elif FLASH_BASE <= dst and dst + size <= FLASH_BASE + FLASH_SIZE:
            kind = "xip"
            if dst - FLASH_BASE != source_offset:
                raise ValueError(f"segment {index} XIP address does not match its flash offset")
        else:
            raise ValueError(f"segment {index} destination 0x{dst:08x}+0x{size:x} is outside SRAM and flash")
        if dst <= run_address and run_address + 8 <= dst + size:
            position = file_offset + run_address - dst
            descriptor = struct.unpack_from("<II", raw, position)
        records.append({"source_offset": source_offset, "size": size, "destination": dst,
                        "crc": crc, "destination_kind": kind})
    if descriptor is None:
        raise ValueError("no segment contains the run descriptor")
    stack, reset = descriptor
    if not RAM_BASE <= stack <= RAM_END or stack & 7:
        raise ValueError(f"invalid initial stack 0x{stack:08x}")
    reset_code_loaded = any(
        r["destination_kind"] in ("xip", "sram") and
        r["destination"] <= (reset & ~1) < r["destination"] + r["size"]
        for r in records)
    if not reset & 1 or not reset_code_loaded:
        raise ValueError(f"reset vector 0x{reset:08x} is not Thumb code in a loaded segment")
    return {"input_flash_offset": source_origin, "reserved": reserved, "run_address": run_address,
            "initial_sp": stack, "reset_vector": reset, "segments": records}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--full-flash", action="store_true", help="input starts at flash offset 0")
    args = parser.parse_args()
    try:
        raw = args.image.read_bytes()
        result = inspect(raw, args.full_flash)
    except (OSError, ValueError, struct.error) as error:
        parser.exit(1, f"{args.image}: {error}\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
