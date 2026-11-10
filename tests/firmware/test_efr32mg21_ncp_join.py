#!/usr/bin/env python3
"""A device from another chip family joins the network formed by the Silicon Labs EZSP NCP (EFR32MG21).

    test_efr32mg21_ncp_join.py MCUEMU NCP_IMAGE JOINER_IMAGE efr32|esp32h2

scripts/ncp_host.py plays the EZSP host (formNetwork, permitJoining); the NCP and the joiner share one air.
Passes when the NCP reports trustCenterJoinHandler (EZSP 0x24, a secured join) and, for the ESP32-H2, the
device's ZDO Device_annce as incomingMessageHandler (0x45; the Zephyr ED does not announce in the window). Runs long enough (about 25 s of simulated time) to cover the NCP's
UART/LDMA stability (it asserted at 11.6 s once) and, for the ESP32-H2, a RISC-V and an ARM engine in one process
(the Apple JIT write-protect tracker crashed there).
"""

import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

AIR = """version: 1
world:
  seed: 1
  radio:
    - { id: air0, model: log_distance, reference_loss_db: 40, path_loss_exponent: 2.5 }
devices:
  - id: ncp
    mcus:
      - { id: mcu, model: efr32mg21, firmware: ncp.bin,
          radio: { air: air0, position: [0, 0, 1], tx_power_dbm: 8, protocols: [zigbee] } }
  - id: board
    mcus:
      - { id: mcu, model: %s, %sfirmware: joiner.bin,
          radio: { air: air0, position: [4, 0, 1], tx_power_dbm: %d, protocols: [zigbee] } }
"""


def fail(message: str) -> int:
    print(f"firmware_efr32mg21_ncp_join: {message}", file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) != 5 or sys.argv[4] not in ("efr32", "esp32h2"):
        print(__doc__, file=sys.stderr)
        return 2
    mcuemu, ncp, joiner = (os.path.abspath(p) for p in sys.argv[1:4])
    model, power = ("efr32mg21", 0) if sys.argv[4] == "efr32" else ("esp32h2", 10)
    with tempfile.TemporaryDirectory() as root:
        shutil.copy(ncp, os.path.join(root, "ncp.bin"))
        shutil.copy(joiner, os.path.join(root, "joiner.bin"))
        with open(os.path.join(root, "join.yaml"), "w") as handle:
            handle.write(AIR % (model, "eui64: 0x84fd27fffe123456, " if model == "efr32mg21" else "", power))
        try:
            result = subprocess.run([sys.executable, "-u", os.path.join(HERE, "..", "..", "scripts", "ncp_host.py"),
                                     "join.yaml", "--root", root, "--mcuemu", mcuemu, "--for", "26s"],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=600, check=False)
        except subprocess.TimeoutExpired:
            return fail("timed out")
    text = result.stdout.decode(errors="replace")
    if result.returncode != 0:
        return fail(f"host script exited {result.returncode}:\n{text[-1500:]}")
    if "ASH ERROR" in text:
        return fail("the NCP reported an ASH ERROR (it reset itself):\n" + text[-1500:])
    if "networkInit" not in text or "stackStatusHandler" not in text:
        return fail("the NCP never formed a network:\n" + text[-1500:])
    if "trustCenterJoinHandler" not in text:
        return fail("the device did not join (no trustCenterJoinHandler):\n" + text[-1500:])
    if "incomingMessageHandler" not in text or "childJoinHandler" not in text:
        return fail("the join was not completed (no childJoinHandler / Device_annce at the host):\n" + text[-1500:])
    print(f"firmware_efr32mg21_ncp_join ({sys.argv[4]}): ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
