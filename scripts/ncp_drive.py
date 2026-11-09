#!/usr/bin/env python3
"""Play an EZSP host for the Silicon Labs NCP in a `mcuemu sim` scenario.

    ncp_drive.py BASE.yaml OUT.yaml MCUEMU --cmd T=HEX ... [--poll-from T --poll-to T --poll-step S] [--for 20s]

BASE.yaml holds world/devices/attach (the NCP's UART endpoint must be `ncp.mcu.uart0` with a `hex` sink).
The host sends one DATA frame at a time and acknowledges the NCP's frames in the ack field of the next one. Each --cmd is an EZSP frame (hex, sequence byte included) sent at virtual time T seconds; `callback` polls
(EZSP 0x0006) are added in the poll window. The sim is deterministic, so the ASH sequence/ack numbers
are found by running it repeatedly: every pass reads the NCP's frames from the hex sink, then rebuilds
the scenario with the right ack field on each host frame, until it is stable.
Prints the NCP's EZSP frames (other than "no callbacks") with their times.
"""
import argparse
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
import ash_frame as ash


def ncp_frames(text):
    out = []
    for line in text.splitlines():
        m = re.match(r"\[\s*([\d.]+) s\] ncp\.mcu\.uart0\s+\d+ bytes: (.*)", line)
        if not m:
            continue
        for kind, detail in ash.parse(bytes.fromhex(m.group(2).replace(" ", ""))):
            out.append((float(m.group(1)), kind, detail))
    return out


def build(base, host, ncp_data, extra=()):
    """host: [(t, ezsp)], ncp_data: [(t, seq)] of first-time NCP DATA frames."""
    events = [(host[0][0], b"\x1a" + ash.frame(0xC0))]  # RST
    seq = 0
    for t, ezsp in host[1:]:
        before = [nseq for nt, nseq in ncp_data if nt < t]
        ack = ((before[-1] + 1) & 7) if before else 0  # next NCP frame we expect
        events.append((t, b"\x1a" + ash.data_frame(seq, ack, ezsp)))
        seq = (seq + 1) & 7
    events.sort(key=lambda e: e[0])
    return base + "scenario:\n  events:\n" + "".join(
        f'    - {{ at: {t:.4f}s, endpoint: ncp.mcu.uart0, set: "hex:{b.hex()}" }}\n' for t, b in events) + "".join(
        f"    - {{ {e} }}\n" for e in extra)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("base"); p.add_argument("out"); p.add_argument("mcuemu")
    p.add_argument("--cmd", action="append", default=[])
    p.add_argument("--poll-from", type=float, default=0); p.add_argument("--poll-to", type=float, default=0)
    p.add_argument("--poll-step", type=float, default=0.2)
    p.add_argument("--for", dest="duration", default="20s")
    p.add_argument("--rst-at", type=float, default=1.2)
    p.add_argument("--event", action="append", default=[], help="extra scenario event, e.g. 'at: 8s, endpoint: x.y, set: true'")
    a = p.parse_args()
    base = open(a.base).read()
    host = [(a.rst_at, b"")]
    for c in a.cmd:
        t, h = c.split("=", 1)
        host.append((float(t), bytes.fromhex(h)))
    cmd_times = [t for t, _ in host]
    t = a.poll_from
    while a.poll_to and t < a.poll_to:
        # One DATA frame in flight at a time: keep polls clear of the commands.
        if all(abs(t - c) > 0.06 for c in cmd_times):
            host.append((t, None))
        t += a.poll_step
    host.sort(key=lambda e: e[0])
    # Number the EZSP sequence byte of polls; commands carry their own.
    ez = 0x40
    fixed = []
    for t, e in host:
        if e is None:
            fixed.append((t, bytes([ez & 255, 0, 1, 6, 0]))); ez += 1
        else:
            fixed.append((t, e))
    ncp_data, observed = [], None
    for _ in range(30):
        with open(a.out, "w") as f:
            f.write(build(base, fixed, ncp_data, a.event))
        text = subprocess.run([a.mcuemu, "sim", a.out, "--for", a.duration], capture_output=True, timeout=900).stdout.decode(errors="replace")
        frames = ncp_frames(text)
        new = [(t, int(detail.split()[1])) for t, kind, detail in frames if kind == "DATA" and " retx 0" in detail]
        if new == observed and not any(k == 'ERROR' for _t, k, _d in frames):
            break
        if new == observed:
            print('ASH ERROR in the final pass: the host acks are not consistent', file=sys.stderr)
            break
        observed = new
        # Beyond the last frame seen, assume the usual one NCP frame per host frame, so a pass fixes
        # the acks of everything after its first surprise instead of just the next frame.
        ncp_data = list(new)
        last_t = new[-1][0] if new else 0.0
        seq = (new[-1][1] + 1) & 7 if new else 0
        for t, _e in fixed[1:]:
            if t > last_t:
                ncp_data.append((t + 0.001, seq)); seq = (seq + 1) & 7
        print(f"pass: {len(new)} NCP frames seen", file=sys.stderr)
    for t, kind, detail in frames:
        if kind == "DATA" and detail.endswith("07 00") and " retx 0" in detail:
            continue
        print(f"{t:9.4f} {kind} {detail}")


if __name__ == "__main__":
    main()
