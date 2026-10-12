# Scripting parts in Lua

A part of `type: script` takes its behavior from a Lua 5.4 file:

```yaml
parts:
  - id: temp1
    type: script
    script: parts/ds18b20.lua      # relative to the YAML file
    pins: { dq: ow }               # terminal: net, all of them go to the script
    params: { temperature_c: 23.5 }
```

`examples/parts/ds18b20.lua` (1-Wire) and `examples/parts/sht3x.lua` (I2C)
are complete references. The DS18B20 script passes the same tests as the
native C++ `ds18b20` part. Python scripts are planned and use the same API.

The script runs once at load time and registers what it needs; afterwards the
simulation calls it back. Each part has its own Lua state.

## Globals

| Name | Meaning |
|---|---|
| `pins.<terminal>` | A pin object for each wired terminal (see below). |
| `params` | The `params:` mapping. Values become integers, floats, booleans (`true`/`false`) or strings. |
| `US`, `MS`, `S` | `1000`, `1000000`, `1000000000`: nanoseconds per unit. All times are integer nanoseconds of virtual time. |
| `now()` | Time of the current callback. |
| `at(t, fn)` / `after(dt, fn)` | Run `fn(t)` at an absolute time / after a delay. Returns a timer id. |
| `cancel(id)` | Cancels a timer; returns whether it was still pending. |
| `expose(name, getter [, setter])` | Publishes `device.part.name` as an endpoint; without a setter it is read-only. Scenario events (`endpoint:` + `set:`) and clients call the setter. |
| `i2c_slave{...}` | Attaches an I2C slave (below). |
| `rand()` | Deterministic float in `[0, 1)`; seeded from `world.seed` and the part name. |
| `log(...)` | Prints to stderr with the part name and time. |
| `function on_edge(terminal, high, t)` | Optional: called when the net of a terminal changes level. `t` is the virtual time of the change. |

Pin objects: `pin:drive(state [, t])` with `state` one of `"low"`, `"high"`,
`"hiz"`, `"pullup"`, `"pulldown"` (`t` defaults to the current callback time);
`pin:release()`; `pin:level()` (`true`, `false`, or `nil` when floating);
`pin:level_at(t)` (level at an earlier time, from the last 64 edges: the way
to sample a slot by its timestamp rather than by when the callback happens to
run); `pin:mv()` (net millivolts); `pin:set_mv(mv or nil)` (analog source,
e.g. a thermistor divider); `pin:name()`.

Edges caused by the script's own `drive()`/`set_mv()` are not delivered to
`on_edge`, so releasing a line never looks like the master letting go.

## Radio

A part with a `radio:` section (see [system-config.md](system-config.md))
gets a `radio` table and may define `on_frame`:

| Call | Meaning |
|---|---|
| `radio.transmit{protocol=, channel=, payload=, [to=], [t=]}` | Sends a frame (`protocol` is `"zigbee"` or `"ble"`, `payload` a string or a list of bytes, `to` a node name or broadcast, `t` the start time, default now). Returns the frame id and its end time. |
| `radio.listen(protocol, channel)` / `radio.listen(nil)` | Tunes the receiver or turns it off. |
| `radio.set_power(dbm)` | Changes the node's TX power. |
| `radio.airtime(protocol, bytes)` | Time on air in ns. |
| `function on_frame(f)` | Called when a frame was received: `f.id`, `f.protocol`, `f.channel`, `f.source`, `f.destination` (nil for broadcast), `f.payload` (string), `f.rssi_dbm`, `f.start_ns`, `f.end_ns`. `now()` is the frame end. |

The protocols must be declared in the part's `radio.protocols`. Reception
follows the medium: collisions, sensitivity and half duplex apply, and a frame
is decided when it ends, after every overlapping frame is known.

## I2C slaves

```lua
i2c_slave {
  sda = "sda", scl = "scl",        -- terminals (default names)
  address = 0x44,                  -- 7-bit; or on_address(addr, read) -> boolean
  on_write = function(byte) return true end,   -- false = NACK
  on_read = function() return 0x12 end,        -- next byte for a master read
  on_start = function() end, on_stop = function() end,
}
```

The decoder handles START/STOP/repeated START, bit shifting and ACKs, so the
script only sees bytes. `now()` inside these callbacks is the scheduler time
(the net change notification carries no timestamp). Hardware controllers and
bit-banging firmware both work.

## Limits and safety

- Sandbox: only `base` (without `dofile`, `loadfile`, `print`,
  `collectgarbage`), `table`, `string`, `math` (without `random`),
  `utf8` and `coroutine`. No `io`, `os`, `package` or `debug`; the script
  cannot touch files, the clock or the process.
- A callback is stopped after 10 million VM instructions, and a script may
  use at most 64 MiB. The error is reported once on stderr with a traceback,
  that part stops reacting (its endpoints can still be read), and the
  simulation continues.
- Errors while loading the file (syntax, runtime, wrong file type) fail the
  build with the file and line.
