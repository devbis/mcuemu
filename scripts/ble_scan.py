#!/usr/bin/env python3
"""Scan with Bumble through the virtual BLE host of examples/ble_host.yaml.

Start the simulation first (it serves HCI on TCP port 9000):

    build/mcuemu sim examples/ble_host.yaml --speed 1 --for 10m

then, with `pip install bumble`:

    python3 scripts/ble_scan.py [tcp-client:127.0.0.1:9000] [seconds]
"""
import asyncio
import sys

from bumble.device import Device
from bumble.hci import Address
from bumble.transport import open_transport


async def main():
    transport_spec = sys.argv[1] if len(sys.argv) > 1 else "tcp-client:127.0.0.1:9000"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    async with await open_transport(transport_spec) as (source, sink):
        device = Device.with_hci("Bumble", Address("F0:F1:F2:F3:F4:F5"), source, sink)
        await device.power_on()
        print("controller up")
        seen = {}

        def on_advertisement(adv):
            kind = "scan response" if adv.is_scan_response else "advertisement"
            seen.setdefault((str(adv.address), kind), adv)
            print(kind, adv.address, "rssi", adv.rssi, adv.data)

        device.on("advertisement", on_advertisement)
        await device.start_scanning(active=True, filter_duplicates=True)
        await asyncio.sleep(seconds)
        await device.stop_scanning()
        print("reports:", len(seen))


asyncio.run(main())
