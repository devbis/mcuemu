# System configuration (YAML)

`mcuemu sim SYSTEM.yaml [--for DURATION] [--quantum DURATION] [--trace-nets]
[--attach ENDPOINT=SINK]... [--list-endpoints]`
builds a system of devices from a declarative description and runs it on one
virtual timeline. `--for` overrides `world.duration`; `--quantum` overrides
`world.quantum`. Errors are reported with the YAML line. Keys for planned
features (parts, radio, attach, scenario scripts) are rejected explicitly
instead of being ignored. The roadmap is in
[declarative-config-plan.md](declarative-config-plan.md).

```yaml
version: 1                 # required
world:
  duration: 10ms           # run length unless --for is given
  quantum: 5us             # max skew between MCUs (default 5us)
  seed: 1
  radio:                   # radio media, see "Radio" below
    - { id: air0, model: log_distance, path_loss_exponent: 2.7 }
devices:
  - id: node               # letters, digits, '_' and '-'
    mcus:
      - id: main
        model: tlsr8258    # tlsr8258 | ca51f253l3 | nrf52840 | efr32mg21
        variant: F1K       # tlsr8258 only: F512 (default) | F1K
        firmware: fw/a.bin # relative to this file; or a list of flash pieces, see below
        clock_hz: 48MHz    # 48000000 | 48MHz | 32kHz
        supply_mv: 3300    # tlsr8258 only: supply seen by the ADC's VBAT input (default 3300)
        start_at: 100us    # power-on delay on the world timeline
        rtt: { address: 0x20000100 }   # optional: SEGGER RTT control block
        radio:             # optional: attach to a medium
          air: air0
          position: [0, 0, 1.5]    # meters
          tx_power_dbm: 0
          sensitivity_dbm: -97
          protocols: [zigbee]
    nets:                  # device scope: references are mcu.PAD
      - id: line
        kind: open_drain   # push_pull (default) | open_drain
        pull: up           # up | down | none
        pins: [main.PA0, aux.PB1]
    links:                 # byte-level UART cross-connect
      - { type: uart, a: main.uart0, b: aux.uart0, latency: 10us }
    parts:                 # external peripherals wired to this device's nets
      - id: temp1
        type: ds18b20
        pins: { dq: line }          # terminal: net
        params: { temperature_c: 23.5 }
      - id: custom                 # behavior from a Lua file, see scripting.md
        type: script
        script: parts/mysensor.lua  # relative to this file
        pins: { sda: i2c_sda, scl: i2c_scl }
nets:                      # world scope: references are device.mcu.PAD
  - { id: shared, pins: [a.m.PA0, b.m.PA0] }
links: []                  # world scope: device.mcu.uartN
attach:                    # endpoint -> sink (device scope: relative names)
  - { endpoint: node.main.uart0, sink: "tcp:5000" }
  - { endpoint: node.line, sink: "vcd:pins.vcd" }
scenario:
  events:                  # external stimulus on a net (full net name)
    - { at: 200us, net: node.line, drive: low }   # low|high|hiz|pullup|pulldown
    - { at: 2s, endpoint: node.temp1.temperature_c, set: 31.5 }
```

Durations: `ns`, `us`, `ms`, `s` (fractions allowed, `0` is accepted bare).

## Firmware layout

`firmware:` is one file loaded at flash address 0. For an image that is linked
for another layout (for instance an application that normally runs behind a
bootloader, so that part of it lives at an offset) give a list of pieces:

```yaml
firmware:
  - { file: app.bin, offset: 0, size: 0x1ad0 }                  # vectors + RAM code
  - { file: app.bin, offset: 0x9ad0, from: 0x1ad0 }             # the rest, at +0x8000
```

`offset` is where the piece goes in the flash, `from` the byte offset in the
file where it starts (default 0) and `size` its length (default: the rest of
the file); numbers may be decimal or `0x` hex. Only `tlsr8258` accepts more
than a single image at offset 0.

## Inserting devices from files

A `devices` entry may be an insertion instead of an inline device:

```yaml
devices:
  - include: devices/plug.yaml     # relative to this file
    id: plug2                      # optional: replaces the device id
    params: { position: [3, 0, 0], power_w: 60 }
```

The included file is a document of its own:

```yaml
version: 1
params: { position: [0, 0, 0], power_w: 0 }   # defaults for ${...} in the device
device:
  id: plug
  mcus:
    - { id: zb, model: tlsr8258, firmware: fw/plug.bin,
        radio: { air: air0, position: "${position}" } }
  parts:
    - { id: meter, type: script, script: bl0937.lua, pins: {cf: cf, cf1: cf1},
        params: { power_w: "${power_w}" } }
```

A scalar that is exactly `${name}` takes the parameter's whole value (lists
and numbers keep their type); `${name}` inside a longer string is spliced in
as text. Every parameter used must be declared in the file's `params:`, and the
inserting entry may only set declared parameters. Paths in the included file
(`firmware`, `script`) are relative to that file, and the same file can be
inserted several times under different ids. Errors inside it carry the file
name and its line (`devices/plug.yaml: line 7: ...`); a duplicate device id is
reported at the insertion. Insertions do not nest. When the system is loaded
through the control API, included files are confined to the server's `--root`
like every other file. `examples/` has a world built this way.

## Parts

A part has terminals (`pins:` maps terminal to net) and `params:`. Unknown
terminals or params are errors. Every controllable value is an endpoint named
`device.part.param`, settable from a scenario event.

| type | terminals | params (defaults) | endpoints |
|---|---|---|---|
| `ds18b20` | `dq` (open-drain net with pull-up) | `rom` (serial, default derived from the part name), `temperature_c` (25), `resolution` (12) | `temperature_c` |
| `ntc` | `out` (analog source on the net) | `r25` (10k), `beta` (3950), `r_series` (10k), `series` (`pullup`: fixed resistor to the supply, NTC to ground; or `pulldown`), `temp_c` (25), `supply_mv` (3300) | `temp_c`, `mv` (read-only) |
| `script` | whatever the script uses (all wired terminals) | any (`params` table in the script) | whatever the script `expose()`s |
| `i2c_regs` | `sda`, `scl` (open-drain nets with pull-ups) | `address` (7-bit, required), `size` (256), `init` (`"00=aa,01=bb"`) | `regNN` (hex register) |

`ds18b20` implements the ROM commands (Read, Match, Skip, Search, Alarm
Search) and Convert T / Read, Write Scratchpad with CRC and the datasheet
conversion times; EEPROM persistence and parasite power are not modeled. The
slave works from timestamped edges (reset detection at 480 us, presence 30 us
later, write slots sampled 30 us after the falling edge, read slots answered
immediately), so several sensors can share one line. `i2c_regs` is a generic
register file driven bit by bit by SDA/SCL edges, so it works with hardware
controllers and bit-banging firmware alike; the pointer auto-increments.

An `ntc` on a net wired to a TLSR8258 ADC pad (PB0..PB7, PC4, PC5) is read
by the chip's SAR ADC misc channel; `i2c_regs` is driven by the chip's I2C
master as well as by bit-banging firmware. The TLSR8258 model also applies the
internal pad pull resistors to the net, so a bus can rely on them instead of
an external `pull: up`. SPI is not modeled yet; `ca51f253l3` has no GPIO, ADC
or I2C model.

Scripted parts are described in [scripting.md](scripting.md).

`mcuemu serve` runs the same systems under remote control; see
[control-api.md](control-api.md).

## Endpoints and sinks

Everything observable or controllable is an endpoint with a name and a kind;
`--list-endpoints` prints them. A sink connects to an endpoint and is created
from a spec; the same specs work in `attach:` (device scope may use names
relative to the device, e.g. `m.uart0`) and in `--attach ENDPOINT=SINK`
(repeatable). Relative output paths are relative to the working directory.

To watch UART traffic without touching the YAML, `mcuemu sim` has
`--trace-uart PATTERN[=hex|text]` (repeatable): it attaches a `hex` (default) or
`text` sink to every UART endpoint whose name contains PATTERN (`'*'` is all
of them). Use `text` for a debug UART that prints messages:

    mcuemu sim examples/thermostat_only.yaml --for 3s \
        --trace-uart thermostat.main.uart1 --trace-uart thermostat.zb.uart0 \
        --trace-uart thermostat.main.uart0=text

Frames sent back to back (less than 1 ms apart) end up on one line; use
`--attach thermostat.main.uart1=hex:300us` to split them. Remote clients cannot
use `stdout`, `text` or `hex`.

| endpoint | kind | events | `set` |
|---|---|---|---|
| `device.mcu.uartN` | `uart` | bytes the UART transmitted | bytes/string to receive (queued until the FIFO has room) |
| `device.mcu.rtt`, `rttN` | `rtt` | bytes the firmware wrote to up buffer N | bytes/string for down buffer N |
| `device.net`, world nets | `net` | level changes (true = high) | - |
| `device.part.param` | `param` | the new value after a change | a new value |
| radio medium `air0` | `radio` | one text summary per frame | - |

| sink | for | does |
|---|---|---|
| `stdout` | any | bytes raw (a console), other values as `t_ns name = value` |
| `text` | uart, rtt | console text, one line per `\n`: `[ time s] endpoint: text`; control and non-ASCII bytes are shown as `\xNN`; a partial line is written at the end of the run |
| `hex` or `hex:GAP` | uart, rtt | hex lines on stdout: `[ time s] endpoint  N bytes: ..`. Bytes of one endpoint closer together than GAP (a duration, default `1ms`) share a line, so a frame is a line; lines from all `hex` sinks are merged in time order. A line appears once a later event (of any endpoint) shows the gap has passed, and at the end of the run |
| `file:PATH` | any | the same, into a file |
| `tcp:PORT` or `tcp:HOST:PORT` | any; useful for uart/rtt | listens (default `127.0.0.1`; `tcp:0` picks a port); one client gets the events and what it sends is fed into the endpoint; a new connection replaces the old one; a client that does not read loses data |
| `vcd:PATH` | `net` and numeric `param` | a waveform, written when the run ends; several endpoints can share one file |
| `pcap:PATH[,ble]` | `radio` | a capture of Zigbee frames (IEEE 802.15.4 without FCS, link type 230) or BLE (link type 251, with the advertising access address in front of the payload); timestamps are virtual time |

`rtt:` (on an `mcu`) reads the control block at `address` straight from target
memory; nothing happens until the firmware has initialized it. It uses the
standard layout for a 32-bit target (24-byte buffer descriptors). The host
advances `RdOff` of the up buffer and `WrOff` of the down buffer, polled every
`interval` (default 100 us of virtual time). `channels` creates `rtt`, `rtt1`,
... Only the TLSR8258 model exposes memory.

Parts `led` (terminal `in`, param `active: high|low`, endpoint `on`) and
`button` (terminal `out`, params `active`/`pressed`, endpoint `pressed`; it
releases the line when not pressed, so the net needs a pull) give indicators
and inputs a name.

## Radio

`world.radio` declares media; an `mcu` or `part` with `radio: {air: ...}`
becomes a node on one. Node names are the instance ids (`device.mcu`,
`device.part`). Every medium is an endpoint of kind `radio` named after it:
`get()` is the number of frames sent, subscribers receive one text summary per
frame (`zigbee-802.15.4 ch15 d.a -> * 3B 010203`) at its start time.

| key | default | meaning |
|---|---|---|
| `model` | `ideal` | `ideal` or `log_distance` |
| `reference_loss_db` | 40 | loss at 1 m (free space at 2.4 GHz) |
| `path_loss_exponent` | 2.0 | `n` in `loss = ref + 10 n log10(d / 1 m)` |
| `shadowing_sigma_db` | 0 | log-normal shadowing, fixed per node pair (seeded by `world.seed`) |
| `noise_floor_dbm` | -100 | |
| `sinr_threshold_db` | 6 | SINR needed to decode a frame |

`ideal` is the original behavior: everyone on the same protocol and channel
hears every frame (RSSI reads -50 dBm) and any overlap on that channel is a
collision. `log_distance` adds geometry:

- received power = TX power - loss; a frame below the receiver's sensitivity
  is not heard;
- a frame is decoded only if SINR >= threshold, where interference is every
  time-overlapping frame (any protocol or channel) scaled by the overlap of
  the occupied 2 MHz bands on the shared frequency axis: IEEE 802.15.4
  channel `n` is `2405 + 5 (n - 11)` MHz and BLE channels 37/38/39 are
  2402/2426/2480 MHz (data channels 0..10 and 11..36 at 2404 + 2i and
  2428 + 2(i - 11)). So BLE channel 0 (2404 MHz) half-overlaps Zigbee
  channel 11 (2405 MHz) while advertising channel 37 does not;
- a node does not receive while it transmits;
- the TLSR8258 reports the RSSI to firmware as `dBm + 110` in the RX
  descriptor.

Interference is counted for the whole frame once any overlap exists
(conservative), and a collision that overlaps only the last quantum of a frame
can be missed for MCU-driven receivers (parts decide at the frame end).
TX power comes from the node's `tx_power_dbm`, not from the chip's registers.

`protocols` lists what the node uses. The TLSR8258 model connects only the
Zigbee RF path; `ble` on it is rejected (its BLE registers are not modeled),
so BLE traffic comes from scripted parts, which can use the `radio` API in
[scripting.md](scripting.md). `nrf52840` and `efr32mg21` connect an 802.15.4 radio (`zigbee`)
and reject `ble`. `ca51f253l3` has no radio.

## Naming

- MCU instance: `device.mcu` (for example `node.main`).
- Net: `device.net` for device-scope nets, the plain id for world-scope nets.
  Every net is also an endpoint with the same name.
- Pads: `tlsr8258` has `PA0`..`PE7`. `nrf52840` has `P0.00`..`P0.31` and
  `P1.00`..`P1.15`. `efr32mg21` has `PA0`..`PD15`. `ca51f253l3` has no GPIO model yet, so no pad can be wired.
- UART: `uartN`. `tlsr8258` has `uart0`; `nrf52840` has `uart0`/`uart1` (UARTE0
  and UARTE1); `efr32mg21` has none yet; `ca51f253l3` has `uart0`/`uart1` (transmit only), so a link to
  it is one-way and reports a warning.

## Semantics

- Pads follow the GPIO OUT/OEN registers: OEN bit 0 drives OUT, 1 releases the
  pad. The line level is read back through the GPIO input register; an
  undriven line with no pull reads as high, like an unwired input.
- An open-drain net treats a `High` drive as released (wired-AND). A pull is a
  weak driver that loses against any strong one.
- A UART link moves bytes the transmitting UART has finished sending to the
  other side's RX FIFO after `latency`; bytes wait while the FIFO is full.
  `baud` is accepted but the wire timing comes from each chip model.
