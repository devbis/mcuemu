# Examples

| file | what |
|---|---|
| `world.yaml` | A Zigbee network of three devices on one radio medium: a coordinator, a thermostat and a smart plug. The devices come from `devices/` through `include:`. |
| `devices/tuya_thermostat.yaml` | A Tuya-style thermostat: CA51F253L3 main board + ZT3L (TLSR8258) Zigbee module, joined by a UART running the "serial v2" protocol. |
| `devices/tuya_plug.yaml` | A Tuya-style smart plug: TLSR8258 module, relay, LED, button and a BL0937 energy meter on three GPIOs. |
| `parts/bl0937.lua` | The BL0937 as a scripted part (CF / CF1 pulse outputs, SEL input). |
| `parts/ds18b20.lua`, `parts/sht3x.lua` | Reference sensor scripts (see `docs/scripting.md`). |

```sh
build/mcuemu sim examples/world.yaml --list-endpoints --for 1ms
build/mcuemu sim examples/world.yaml --for 10s        # writes logs, a VCD and a pcap into the current directory
```

## Inserting a device

A world lists devices, and a device can live in its own file:

```yaml
devices:
  - include: devices/tuya_plug.yaml
    id: plug1                       # optional: rename the device
    params: { position: [3, 0, 0], power_w: 60 }
```

The device file is a YAML document with `version: 1`, an optional `params:`
mapping of defaults and one `device:`. A scalar such as `"${position}"` inside
`device:` is replaced by the parameter of that name (a lone reference keeps its
type: lists and numbers stay lists and numbers). A parameter the file does not
declare is an error, so typos do not pass silently. Paths in the file (firmware,
scripts) are relative to the file. The same file can be inserted several times
under different ids. See `docs/system-config.md`.

## Firmware

No firmware image is stored in the repository. Put the images where the
`params:` defaults point (or pass your own paths):

| device | image | default path |
|---|---|---|
| coordinator | the coordinator sample | `firmware/samplegw_tb03f.bin` |
| thermostat, ZT3L module | the application built by the thermostat project | `firmware/osrctherm_r02_zrd.bin` |
| thermostat, main board | the CA51F253L3 image | `firmware/osrctherm_r02_v1.02_CA51F253L3.bin` |
| plug | your plug firmware | `firmware/tuya_plug.bin` |

The pin numbers of the plug (`relay_pin`, `cf_pin`, ...) are typical
placeholders; take the real ones from your firmware or schematic.

## What is and is not modeled

* **Thermostat**: both chips run their real firmware and exchange real frames
  over the UART in both directions (`66 BB ...` in the two log files). The
  CA51F253L3 model has no GPIO, ADC or radio yet, so the display, keys, relay
  and NTC inputs of the main board are not connected to anything. The ZT3L
  application is linked behind a bootloader, which the emulator does not run;
  the file places the application's RAM code at address 0 and the rest at
  flash `0x8000` + offset (the two `zb_*` parameters come from the linker map).
* **Plug**: the BL0937 is a Lua part. CF pulses at `cf_hz_per_w` x watts, CF1
  carries the current or the voltage depending on SEL. Change the load with the
  `plug.meter.power_w` endpoint (also a scenario event or an API call). The
  calibration ratios are round numbers; use the ones your firmware expects.
* **Radio**: log-distance path loss with shadowing; BLE is not connected to
  the TLSR8258 registers, so only Zigbee devices are shown.
