#!/usr/bin/env python3
"""Runs the Moes ZigBee "Smart Brightness Thermometer" (ZSS-ZK-THL, Tuya ZS3L, EFR32MG21) firmware next to the EZSP NCP and reports
what it does.

    moes_brightness_thermometer.py DUMP [--ncp firmware/zbdonglee-ncp.bin] [--mcuemu build/mcuemu] [--for 40s]
                        [--temperature 22.5] [--humidity 45] [--lux 120] [--channel 24] [--keep-nvm] [--no-press] [--no-check]

DUMP is the full-flash dump of pvvx's ZSS-ZK-THL page (0000000-00C0000.bin: Gecko bootloader at 0, application from 0x4000,
NVM3 at 0xB6000..0xBFFFF). Unless --keep-nvm is given the NVM3 pages are erased (the factory-new state), and a long press on
PB0 (7 s from t = 8 s) starts the pairing. The NCP (scripts/ncp_host.py) forms a network on --channel, which must be one the device
scans (14, 15, 19, 20, 24, 25, 11). The thermometer's CHT8305 (0x40) and OPT3001 (0x44) are Lua parts on PA0/PA3 and the e-ink
display is decoded from PD0/PC2/PB1 (examples/parts); the report lists the join events the NCP saw, how often the firmware read
each sensor, and the display refreshes with the last 18 segment-RAM bytes. The exit status is 1 when the device did not join
(trustCenterJoinHandler, childJoinHandler and the Device_annce at the NCP) or did not read both sensors and refresh the display, unless
--no-check is given. See docs/efr32mg21.md for the details.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

YAML = """version: 1
world:
  seed: 1
  radio:
    - {{ id: air0, model: log_distance, reference_loss_db: 40, path_loss_exponent: 2.5 }}
devices:
  - id: ncp
    mcus:
      - {{ id: mcu, model: efr32mg21, firmware: ncp.bin,
          radio: {{ air: air0, position: [0, 0, 1], tx_power_dbm: 8, protocols: [zigbee] }} }}
  - id: board
    mcus:
      - {{ id: mcu, model: efr32mg21, eui64: 0x84fd27fffe123456,
          firmware: [{{ file: thermometer.bin, offset: 0, size: 0x4000 }}, {{ file: thermometer.bin, offset: 0x4000, from: 0x4000 }}],
          radio: {{ air: air0, position: [4, 0, 1], tx_power_dbm: 0, protocols: [zigbee] }} }}
    nets:
      - {{ id: button, pull: up, pins: ["mcu.PB0"] }}
      - {{ id: sda, pull: up, pins: ["mcu.PA0"] }}
      - {{ id: scl, pull: up, pins: ["mcu.PA3"] }}
      - {{ id: sck, pins: ["mcu.PD0"] }}
      - {{ id: data, pins: ["mcu.PC2"] }}
      - {{ id: cs, pins: ["mcu.PB1"] }}
    parts:
      - {{ id: key, type: button, pins: {{ out: button }} }}
      - {{ id: sensor, type: script, script: cht8305.lua, pins: {{ sda: sda, scl: scl }},
          params: {{ temperature_c: {temperature}, humidity: {humidity} }} }}
      - {{ id: light, type: script, script: opt3001.lua, pins: {{ sda: sda, scl: scl }}, params: {{ lux: {lux} }} }}
      - {{ id: display, type: script, script: epd_zsszkthl.lua, pins: {{ sck: sck, data: data, cs: cs }} }}
"""


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("dump")
    p.add_argument("--ncp", default=os.path.join(ROOT, "firmware", "zbdonglee-ncp.bin"))
    p.add_argument("--mcuemu", default=os.path.join(ROOT, "build", "mcuemu"))
    p.add_argument("--for", dest="duration", default="40s")
    p.add_argument("--channel", type=int, default=24)
    p.add_argument("--temperature", type=float, default=22.5)
    p.add_argument("--humidity", type=float, default=45)
    p.add_argument("--lux", type=float, default=120)
    p.add_argument("--keep-nvm", action="store_true", help="keep the dump's NVM3 (the device then rejoins its old network)")
    p.add_argument("--no-press", action="store_true", help="do not press the pairing button")
    p.add_argument("--no-check", action="store_true", help="only report, never fail")
    a = p.parse_args()

    image = bytearray(open(a.dump, "rb").read())
    if len(image) < 0xC0000:
        print(f"{a.dump}: expected the 0xC0000-byte full-flash dump", file=sys.stderr)
        return 2
    if not a.keep_nvm:
        image[0xB6000:0xC0000] = b"\xff" * (0xC0000 - 0xB6000)
    with tempfile.TemporaryDirectory() as root:
        open(os.path.join(root, "thermometer.bin"), "wb").write(image)
        shutil.copy(a.ncp, os.path.join(root, "ncp.bin"))
        for part in ("cht8305.lua", "opt3001.lua", "epd_zsszkthl.lua"):
            shutil.copy(os.path.join(ROOT, "examples", "parts", part), os.path.join(root, part))
        with open(os.path.join(root, "system.yaml"), "w") as handle:
            handle.write(YAML.format(temperature=a.temperature, humidity=a.humidity, lux=a.lux))
        command = [sys.executable, "-u", os.path.join(HERE, "ncp_host.py"), "system.yaml", "--root", root, "--mcuemu",
                   os.path.abspath(a.mcuemu), "--for", a.duration, "--channel", str(a.channel)]
        if not a.no_press:
            command += ["--event", "at=8s endpoint=board.key.pressed value=true",
                        "--event", "at=15s endpoint=board.key.pressed value=false"]
        for name in ("sensor.reads", "sensor.writes", "light.reads", "light.writes", "display.refreshes", "display.ram"):
            command += ["--get", f"board.{name}"]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    text = result.stdout
    if result.returncode != 0:
        print(text[-2000:], file=sys.stderr)
        return result.returncode
    print("NCP events:")
    for line in text.splitlines():
        if re.search(r"EZSP 0x00(19|1b|23|24|45) ", line):
            print("  " + line.strip())
    print("Thermometer:")
    values = {}
    for line in text.splitlines():
        if line.startswith("get "):
            print("  " + line[4:])
            name, _, value = line[4:].partition(": ")
            values[name] = value
    if a.no_check:
        return 0
    problems = []
    for handler in ("trustCenterJoinHandler", "childJoinHandler", "incomingMessageHandler"):
        if handler not in text:
            problems.append(f"the NCP never reported {handler}")
    for name in ("board.sensor.reads", "board.light.reads", "board.display.refreshes"):
        if not values.get(name, "0").strip().isdigit() or int(values[name]) == 0:
            problems.append(f"{name} is {values.get(name, 'missing')}")
    for problem in problems:
        print("FAIL: " + problem, file=sys.stderr)
    if not problems:
        print("moes_brightness_thermometer: ok")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
