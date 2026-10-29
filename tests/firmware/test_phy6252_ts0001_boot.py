#!/usr/bin/env python3
"""Check TS0001 startup UART and the local relay output on the PB-03F model."""

from __future__ import annotations

import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path


SYSTEM = """version: 1
world:
  seed: 1
  duration: 10s
devices:
  - id: relay
    mcus:
      - id: mcu
        model: phy6252
        firmware: {firmware}
        efuse: {efuse}
        rf_calibration: {rf_calibration}
    nets:
      - {{ id: key1_line, pull: up, pins: [mcu.GPIO8] }}
      - {{ id: key2_line, pull: up, pins: [mcu.GPIO3] }}
      - {{ id: relay_line, pins: [mcu.GPIO2] }}
    parts:
      - {{ id: key1, type: button, pins: {{ out: key1_line }} }}
attach:
  - {{ endpoint: relay.relay_line, sink: "vcd:relay.vcd" }}
scenario:
  events:
    - {{ at: 6s, endpoint: relay.key1.pressed, set: true }}
    - {{ at: 6500ms, endpoint: relay.key1.pressed, set: false }}
"""

EXPECTED = [
    r"Stack Init",
    r"Zigbe(?:e)? stack initialized",
    r"zb_app_task_init",
    r"GPIO:8,0",
    r"GPIO:3,3",
    r"High level",
    r"network state=12",
    r"power report time\s*=\s*\d+",
    r"set read local time:\s*\d+",
    r"Tuya 10,D0,30,2",
    r"Tuya 30,D0,30,0",
    r"fix\s*Report time\s*=\s*\d+",
    r"key 1 Down",
    r"keyid = 1, 1, 0",
    r"key 1 up",
    r"keyid = 1, 0, 2",
]


def fail(message: str, output: str = "") -> int:
    print(f"firmware_phy6252_ts0001_boot: {message}", file=sys.stderr)
    if output:
        print("--- emulator output ---", file=sys.stderr)
        print(output, file=sys.stderr)
    return 1


def text_output(value: str | bytes | None) -> str:
    if isinstance(value, bytes):
        return value.decode(errors="replace")
    return value or ""


def main() -> int:
    if len(sys.argv) != 5:
        print(
            "usage: test_phy6252_ts0001_boot.py EMULATOR FIRMWARE EFUSE RF_CALIBRATION",
            file=sys.stderr,
        )
        return 2

    emulator, firmware, efuse, rf_calibration = (
        str(Path(path).resolve()) for path in sys.argv[1:]
    )
    config = SYSTEM.format(
        firmware=json.dumps(firmware),
        efuse=json.dumps(efuse),
        rf_calibration=json.dumps(rf_calibration),
    )

    with tempfile.TemporaryDirectory(prefix="mcuemu-phy6252-ts0001-") as scratch:
        workdir = Path(scratch)
        system = workdir / "system.yaml"
        system.write_text(config)
        try:
            result = subprocess.run(
                [emulator, "sim", str(system), "--trace-uart", "*=text"],
                cwd=workdir,
                capture_output=True,
                text=True,
                timeout=900,
                check=False,
            )
        except subprocess.TimeoutExpired as error:
            output = text_output(error.stdout) + text_output(error.stderr)
            return fail("emulator exceeded the 900-second timeout", output)

        output = result.stdout + result.stderr
        if result.returncode != 0:
            return fail(f"emulator exited with status {result.returncode}", output)
        mcu_time = re.search(
            r"mcu relay\.mcu \(phy6252\): running at (\d+) ns", output
        )
        if not mcu_time or int(mcu_time.group(1)) < 9_900_000_000:
            return fail("the simulation did not reach its 10-second deadline", output)

        position = 0
        for pattern in EXPECTED:
            match = re.search(pattern, output[position:])
            if not match:
                return fail(f"UART never reached /{pattern}/ in order", output)
            position += match.end()

        vcd_path = workdir / "relay.vcd"
        if not vcd_path.exists():
            return fail("relay GPIO VCD was not written", output)
        timestamp = 0
        saw_button_transition = False
        for line in vcd_path.read_text().splitlines():
            if line.startswith("#"):
                timestamp = int(line[1:])
            elif line in {"0!", "1!"} and 5_000_000_000 <= timestamp <= 7_000_000_000:
                saw_button_transition = True
                break
        if not saw_button_transition:
            return fail("GPIO2 did not change during the GPIO8 button press", output)

    print("firmware_phy6252_ts0001_boot: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
