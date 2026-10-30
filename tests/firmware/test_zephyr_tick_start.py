#!/usr/bin/env python3
"""Zephyr router whose system timer driver may or may not retry a START that is ignored.

On a TB03F the 16 MHz system timer ignored the START strobe that the Zephyr stimer
driver writes right after its STOP, in one image: the counter stayed at zero and the
first k_busy_wait() of the boot never returned. The driver now reads the counter back
and repeats START. The model (`tick_start_settle` / `tick_start_jitter` on a tlsr8258
MCU) ignores a START that comes too soon after a STOP.

Two builds of the same router (Zigbee shell, router.conf):
* "no retry": the driver of that build starts the timer once;
* "retry": the same build with the retrying driver.

Without the window both join the vendor sampleGW coordinator. With a window that is
longer than the driver's STOP-to-START gap (and jitter on top), the first stays silent and
the second still joins, as on the board.

Usage: test_zephyr_tick_start.py MCUEMU COORDINATOR.bin NO_RETRY.bin RETRY.bin
"""

import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import zigbee_air_check as air  # noqa: E402

DURATION = "30s"

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
  - id: node
    mcus:
      - id: zb
        model: tlsr8258
        variant: F512
        start_at: 8s          # after the coordinator has formed its network
        flash_timing: true
{window}        firmware: {node}
        radio: {{ air: air0, position: [3, 0, 1], tx_power_dbm: 10, protocols: [zigbee] }}
attach:
  - {{ endpoint: air0, sink: "pcap:air.pcap" }}
"""

WINDOW = "        tick_start_settle: 30us\n        tick_start_jitter: 15us\n        tick_start_seed: 3\n"


def start(mcuemu: str, coordinator: str, node: str, window: str, workdir: Path) -> subprocess.Popen:
    workdir.mkdir(parents=True, exist_ok=True)
    system = workdir / "system.yaml"
    system.write_text(TEMPLATE.format(coordinator=coordinator, node=node, window=window))
    return subprocess.Popen(
        [mcuemu, "sim", str(system), "--for", DURATION],
        cwd=workdir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def joined(workdir: Path) -> bool:
    frames = air.read_pcap(workdir / "air.pcap")
    for frame in frames:
        if frame.kind == 3 and frame.payload[:1] == b"\x02" and len(frame.payload) >= 4 and frame.payload[3] == 0:
            return True
    return False


def main() -> int:
    if len(sys.argv) != 5:
        print(__doc__, file=sys.stderr)
        return 2
    mcuemu, coordinator, no_retry, retry = (str(Path(p).resolve()) for p in sys.argv[1:])
    with tempfile.TemporaryDirectory() as scratch:
        root = Path(scratch)
        runs = {
            "no_retry_without_window": start(mcuemu, coordinator, no_retry, "", root / "a"),
            "retry_without_window": start(mcuemu, coordinator, retry, "", root / "b"),
            "no_retry_with_window": start(mcuemu, coordinator, no_retry, WINDOW, root / "c"),
            "retry_with_window": start(mcuemu, coordinator, retry, WINDOW, root / "d"),
        }
        for name, proc in runs.items():
            output, _ = proc.communicate(timeout=900)
            if proc.returncode != 0:
                print(f"firmware_zephyr_tick_start: {name} run failed:\n{output}", file=sys.stderr)
                return 1
        got = {name: joined(root / d) for name, d in
               (("no_retry_without_window", "a"), ("retry_without_window", "b"),
                ("no_retry_with_window", "c"), ("retry_with_window", "d"))}
    expect = {"no_retry_without_window": True, "retry_without_window": True,
              "no_retry_with_window": False, "retry_with_window": True}
    errors = [f"{name}: joined={got[name]}, expected {want}" for name, want in expect.items()
              if got[name] != want]
    if errors:
        for error in errors:
            print(f"firmware_zephyr_tick_start: {error}", file=sys.stderr)
        return 1
    print("firmware_zephyr_tick_start: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
