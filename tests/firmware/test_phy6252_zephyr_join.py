#!/usr/bin/env python3
"""Run the PB-03F Zephyr image against the TLSR8258 sample gateway."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


SYSTEM = """version: 1
world:
  seed: 1
  duration: 10s
  radio:
    - {{ id: air0, model: log_distance, reference_loss_db: 40, path_loss_exponent: 2.5 }}
devices:
  - id: coordinator
    mcus:
      - id: mcu
        model: tlsr8258
        variant: F1K
        firmware: {coordinator}
        radio: {{ air: air0, position: [0, 0, 1], tx_power_dbm: 10, protocols: [zigbee] }}
  - id: pb03f
    mcus:
      - id: mcu
        model: phy6252
        firmware: {firmware}
        rom: {rom}
        efuse: {efuse}
        rf_calibration: {rf_calibration}
        radio: {{ air: air0, position: [4, 0, 1], tx_power_dbm: 0, protocols: [zigbee] }}
attach:
  - {{ endpoint: air0, sink: "pcap:air.pcap" }}
"""


def main() -> int:
    if len(sys.argv) != 7:
        print(
            "usage: test_phy6252_zephyr_join.py EMULATOR COORDINATOR "
            "FIRMWARE ROM EFUSE RF_CALIBRATION",
            file=sys.stderr,
        )
        return 2

    emulator, coordinator, firmware, rom, efuse, rf_calibration = (
        str(Path(path).resolve()) for path in sys.argv[1:]
    )
    config = SYSTEM.format(
        coordinator=json.dumps(coordinator),
        firmware=json.dumps(firmware),
        rom=json.dumps(rom),
        efuse=json.dumps(efuse),
        rf_calibration=json.dumps(rf_calibration),
    )

    with tempfile.TemporaryDirectory(prefix="mcuemu-phy6252-zephyr-") as scratch:
        workdir = Path(scratch)
        system = workdir / "system.yaml"
        system.write_text(config)
        checker = Path(__file__).with_name("zigbee_air_check.py")
        command = [
            sys.executable,
            str(checker),
            emulator,
            str(system),
            "--workdir",
            str(workdir),
            "--for",
            "10s",
            "--min-up",
            "1",
            "--min-unicast",
            "1",
            "--min-down",
            "1",
            "--expect",
            "Booting Zephyr OS",
            "--expect",
            r"zigbee_shell joined network \(bdb status: 0x00\)",
        ]
        result = subprocess.run(command, check=False, timeout=900)
        return result.returncode


if __name__ == "__main__":
    sys.exit(main())
