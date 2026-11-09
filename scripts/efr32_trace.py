#!/usr/bin/env python3
"""Annotate and summarise an `mcuemu run --model EFR32MG21 --trace-mmio` trace with register names.

    efr32_trace.py REGMAP_DIR TRACE [--from NS] [--to NS] [--blocks FRC,RAC,...] [--loops N]

REGMAP_DIR holds the *_register.py / *_field.py files of Silicon Labs' xG21 register maps
(simplicity_sdk: platform/radio/efr32_multiphy_configurator/host_py_rm_studio_internal/full/
efr32xg21/revA0/; not stored in this repository). Each distinct access is printed once with its count,
the register name and, for writes, the fields whose bits are set; a register read many times at one
pc (a wait loop) is reported with its value changes.
"""
import argparse
import collections
import glob
import os
import re
import sys

REG = re.compile(r"super\(RM_Register_\w+, self\).__init__\(rmio, label,\s*0x([0-9a-fA-F]+), 0x([0-9a-fA-F]+),\s*'(\w+)', '([\w.]+)'", re.S)
FLD = re.compile(r"super\(RM_Field_\w+, self\).__init__\(register,\s*'(\w+)', '([\w.]+)', '[\w-]+',\s*u\"[^\"]*\",\s*(\d+), (\d+)\)", re.S)
LINE = re.compile(r"^([RW]) 0x([0-9a-f]+) (?:->|<-) 0x([0-9a-f]+) pc=0x([0-9a-f]+) t=(\d+)")


def load(directory):
    regs, fields = {}, collections.defaultdict(list)
    for path in glob.glob(os.path.join(directory, "*_register.py")):
        for base, off, name, label in REG.findall(open(path).read()):
            regs[(int(base, 16) + int(off, 16)) & 0x0fffffff | 0xb0000000] = label
    for path in glob.glob(os.path.join(directory, "*_field.py")):
        for name, label, pos, width in FLD.findall(open(path).read()):
            reg = label.rsplit(".", 1)[0]
            fields[reg].append((name, int(pos), int(width)))
    return regs, fields


def canon(address):
    """Fold the secure/non-secure view and the set/clear/toggle aliases onto one register address."""
    address &= 0x0fffffff
    if (address >> 12) & 0x3 and (address & 0x0ffff000) >= 0x8000000:
        address &= ~0x3000
    return address | 0xb0000000


def describe(label, value, fields):
    parts = []
    for name, pos, width in sorted(fields.get(label, []), key=lambda f: f[1]):
        v = (value >> pos) & ((1 << width) - 1)
        if v:
            parts.append(f"{name}={v:#x}" if width > 1 else name)
    return " ".join(parts)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("regmap")
    p.add_argument("trace")
    p.add_argument("--from", dest="t0", type=int, default=0)
    p.add_argument("--to", dest="t1", type=int, default=1 << 62)
    p.add_argument("--blocks", default="")
    p.add_argument("--loops", type=int, default=50)
    args = p.parse_args()
    regs, fields = load(args.regmap)
    want = {b for b in args.blocks.split(",") if b}
    seen = collections.OrderedDict()
    loops = collections.Counter()
    last = {}
    for line in open(args.trace, errors="replace"):
        m = LINE.match(line)
        if not m:
            continue
        rw, addr, val, pc, t = m.group(1), int(m.group(2), 16), int(m.group(3), 16), int(m.group(4), 16), int(m.group(5))
        if not (args.t0 <= t <= args.t1):
            continue
        key = canon(addr)
        label = regs.get(key)
        if label is None:
            if (addr & 0x0f000000) not in (0x08000000, 0x00000000) or (addr & 0xf0000000) not in (0xa0000000, 0xb0000000, 0x40000000, 0x50000000, 0x44000000, 0x54000000):
                continue
            label = f"?{key:#010x}"
        block = label.split(".")[0].replace("_NS", "")
        if want and block not in want:
            continue
        loops[(rw, label, pc)] += 1
        entry = (rw, label, pc, val if rw == "W" else None)
        if rw == "R":
            if last.get((label, pc)) != val:
                last[(label, pc)] = val
                seen.setdefault((rw, label, pc, val), t)
        else:
            seen.setdefault(entry, t)
    for (rw, label, pc, val), t in seen.items():
        n = loops[(rw, label, pc)]
        if rw == "W":
            print(f"{t:>12} W {label:<28} <- {val:#010x} {describe(label, val, fields)}  pc={pc:#x}")
        else:
            print(f"{t:>12} R {label:<28} -> {val:#010x} {describe(label, val, fields)}  pc={pc:#x}" + (f"  [x{n}]" if n > 1 else ""))
    spin = [(k, c) for k, c in loops.items() if c >= args.loops and k[0] == "R"]
    if spin:
        print("\nregisters polled many times (rw, register, pc: count):")
        for (rw, label, pc), c in sorted(spin, key=lambda x: -x[1])[:15]:
            print(f"  {label:<28} pc={pc:#x}: {c}")


if __name__ == "__main__":
    sys.exit(main())
