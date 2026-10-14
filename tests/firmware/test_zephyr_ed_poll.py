#!/usr/bin/env python3
"""Zephyr sleepy end device on the shared air: join, then keep polling its parent.

Two builds of the same application (images from a TB03F investigation): the
"before" image restored its timers too late after a deep-sleep wake, so the
system timer stopped after the first wake and the device went silent; the
"after" image keeps polling. On the board the "after" device joined a
coordinator, received the Transport-Key and polled every 0.5 s.

What the model needs for this to come out as on the board:
* flash timing (`flash_timing: true`): the driver finishes a poll's TX, and the
  MAC asks for the ACK's Frame Pending bit only after the ACK has been taken
  in, because cold flash code runs about 10x slower than the cached estimate;
* the 16 MHz system timer that stops without START/clock select;
* frames that last as long on air as they do on a real radio (FCS included).

Usage: test_zephyr_ed_poll.py MCUEMU COORDINATOR.bin BEFORE.bin AFTER.bin
"""

import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import zigbee_air_check as air  # noqa: E402

DURATION = "50s"
LATE_NS = 35_000_000_000

TEMPLATE = """version: 1
world:
  seed: 1
  radio:
    - {{ id: air0, model: log_distance, reference_loss_db: 40, path_loss_exponent: 2.5 }}
devices:
  - id: coordinator
    mcus:
      - id: zb
        model: tlsr8258
        variant: F1K
        firmware: {coordinator}
        radio: {{ air: air0, position: [0, 0, 1], tx_power_dbm: 10, protocols: [zigbee] }}
  - id: ed
    mcus:
      - id: zb
        model: tlsr8258
        variant: F512
        start_at: 8s          # after the coordinator has formed its network
        flash_timing: true
        firmware: {ed}
        radio: {{ air: air0, position: [3, 0, 1], tx_power_dbm: 10, protocols: [zigbee] }}
attach:
  - {{ endpoint: air0, sink: "pcap:air.pcap" }}
"""


def start(mcuemu: str, coordinator: str, ed: str, workdir: Path) -> subprocess.Popen:
    workdir.mkdir(parents=True, exist_ok=True)
    system = workdir / "system.yaml"
    system.write_text(TEMPLATE.format(coordinator=coordinator, ed=ed))
    return subprocess.Popen(
        [mcuemu, "sim", str(system), "--for", DURATION],
        cwd=workdir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def polls_after(workdir: Path, short_out: list) -> tuple[bool, int]:
    frames = air.read_pcap(workdir / "air.pcap")
    short = None
    for frame in frames:
        if frame.kind == 3 and frame.payload[:1] == b"\x02" and len(frame.payload) >= 4 and frame.payload[3] == 0:
            short = frame.payload[1] | frame.payload[2] << 8
            break
    short_out.append(short)
    if short is None:
        return False, 0
    # MAC command 0x04 is the Data Request a polling end device sends.
    late = sum(1 for f in frames
               if f.kind == 3 and f.src == short and f.payload[:1] == b"\x04" and f.t_ns >= LATE_NS)
    return True, late


def main() -> int:
    if len(sys.argv) != 5:
        print(__doc__, file=sys.stderr)
        return 2
    mcuemu, coordinator, before, after = (str(Path(p).resolve()) for p in sys.argv[1:])
    with tempfile.TemporaryDirectory() as scratch:
        root = Path(scratch)
        runs = {"before": start(mcuemu, coordinator, before, root / "before"),
                "after": start(mcuemu, coordinator, after, root / "after")}
        for name, proc in runs.items():
            output, _ = proc.communicate(timeout=900)
            if proc.returncode != 0:
                print(f"firmware_zephyr_ed_poll: {name} run failed:\n{output}", file=sys.stderr)
                return 1
        errors = []
        shorts: list = []
        joined, late = polls_after(root / "after", shorts)
        if not joined:
            errors.append("after image: no successful association")
        elif late < 5:
            errors.append(f"after image: only {late} data requests after 35 s, expected it to keep polling")
        joined, late = polls_after(root / "before", shorts)
        if not joined:
            errors.append("before image: no successful association (the join itself must work)")
        elif late != 0:
            errors.append(f"before image: {late} data requests after 35 s, expected silence after the first deep wake")
    if errors:
        for error in errors:
            print(f"firmware_zephyr_ed_poll: {error}", file=sys.stderr)
        return 1
    print("firmware_zephyr_ed_poll: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
