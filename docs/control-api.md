# Control API (JSON-RPC 2.0 over WebSocket)

```
mcuemu serve --root DIR [--port N] [--bind ADDR] [--token T]
               [--allow-origin ORIGIN]... [--max-sessions N] [--idle-timeout SEC]
```

`serve` listens on `ws://127.0.0.1:PORT/` (it prints the port; `--port 0`, the
default, picks a free one). **Each connection is its own isolated simulation.**
Messages are JSON-RPC 2.0 text frames (requests, notifications and batches).
`GET /health` answers `200 ok`. Everything is single-threaded; a running
simulation advances in short bursts between I/O.

## Safety model

- **Files are confined to `--root`** (required): firmware, Lua scripts and
  output files named in a loaded description (including the cc2530f256
  `flash:` `persist` and `log` files) or in `attach`. Relative paths
  resolve inside the root; `..`, absolute paths and symlinks that leave it are
  refused. `load` also takes `base_dir` (relative to the root) for resolving the
  YAML's relative paths.
- Remote clients cannot use the `stdout` sink; `tcp` sinks listen on loopback
  only and not below port 1024.
- Descriptions are limited to 1 MiB, messages to 4 MiB, memory reads to 1 MiB;
  Lua scripts are sandboxed and limited (see [scripting.md](scripting.md)).
- Binds to loopback by default; any other address requires `--token`. With a
  token, clients pass `?token=T` in the URL or `Authorization: Bearer T`.
- Browser pages are refused unless their `Origin` is listed with
  `--allow-origin` (`*` allows all). Clients that send no `Origin` (scripts,
  tools) are not affected.
- `--max-sessions` (default 8), `--idle-timeout` (default 600 s).

## Values

A value is `null`, a boolean, a number, a string, or `{"hex": "5a01"}` for
bytes (UART/RTT data). Times are nanoseconds as integers or strings such as
`"250us"`, `"10ms"`, `"2.5s"`. A `cc2530f256` UART frame endpoint
(`device.mcu.uartN.frames`) takes and reports frame tokens as a string, for
example `"0x41 0x42:perr 0x43:fe break"` (see
[system-config.md](system-config.md)). A bad token is error `-32001`.

## Methods

| method | params | result |
|---|---|---|
| `ping` | | `"pong"` |
| `load` | `yaml`, `base_dir?` | `{mcus: [{name, model}], endpoints, warnings, duration_ns?}`; the system starts paused. A bad description fails with code -32000 and the YAML line; the previous system stays |
| `unload` | | `true` |
| `status` | | `{loaded, running, speed, time_ns, mcus: [{name, model, state, time_ns, message?}]}` (`state`: running, sleeping, halted, fault) |
| `run` | `for?` or `until?`, `wait?`, `speed?` | `{running, time_ns, target_ns?}`; without a target it runs until `pause`; with `wait: true` the response is sent when the run ends |
| `set_speed` | `speed` (number, `0` or `"max"`) | `{speed}`; see Realtime pacing |
| `pause` | | `{reason: "paused", time_ns}` |
| `step` | `duration?` (default 1 ms, max 1 s) | `{reason: "step", time_ns}`; synchronous and leaves the simulation paused |
| `list_endpoints` | | `[{name, kind}]` |
| `get` | `endpoint` | the value |
| `set` | `endpoint`, `value`, `at?` | `{scheduled}`; with `at` it happens at that virtual time |
| `subscribe` | `endpoint` | `{subscription}`; events arrive as notifications |
| `unsubscribe` | `subscription` | `true` |
| `attach` | `endpoint`, `sink` | `{attachment}`; sink specs as in [system-config.md](system-config.md) |
| `detach` | `attachment` | `true` |
| `drive` | `net`, `state` (low, high, hiz, pullup, pulldown), `at?` | `{scheduled}` |
| `mem.read` | `mcu`, `address`, `length` | `{hex}` |
| `mem.write` | `mcu`, `address`, `hex` | `true` |
| `debug_if.enter` | `mcu` | `{time_ns}`; a cc2530f256 enters debug mode (RESET_N low, two DC falling edges, release) with the CPU halted at `0x0000` |
| `debug_if.command` | `mcu`, `hex` (command byte and its input bytes) | `{hex, accepted, time_ns}`; one SWRU191F Table 3-1 command; a refused command returns the status byte in every output byte with `accepted: false` |
| `debug_if.reset` | `mcu` | `{time_ns}`; a RESET_N pulse without DC edges: debug mode ends and the firmware runs |
| `debug_if.program` | `mcu`, `firmware`, `debug_lock?`, `verify?` (default true), `run?` (default true) | `{chip_id, chip_version, was_locked, pages, bytes_verified, commands, time_ns}`; CC-Debugger-style programming through the debug interface |
| `shutdown` | | `true`, then the server closes the connection |

For a cc2530f256 with `flash:`, `set` on `<mcu>.power` with `"cut"` or
`"reset"` cuts the power or resets the chip, at `at` if given (see
[system-config.md](system-config.md#flash-faults-power-cuts-and-persistence-cc2530f256)).
After a run, `unload` saves a `persist` flash image; call it before `load`
when the next system must start from that image.

The `debug_if.*` methods (cc2530f256 only; see
[cc2530.md](cc2530.md#debug-interface)) act like a debug probe on the chip's
RESET_N, DC and DD pins: each call pauses a running simulation and advances
the whole world synchronously (128 cycles per command byte; programming takes
seconds of virtual time). `firmware` is confined to the root like `load`.

Errors use the JSON-RPC codes (-32700 parse, -32600 invalid request, -32601
unknown method, -32602 invalid params) plus -32000 (the YAML is invalid) and
-32001 (the request cannot be honored: nothing loaded, unknown endpoint,
sandbox violation, read-only endpoint, ...).

## Notifications (server to client)

```json
{"jsonrpc":"2.0","method":"event","params":{"subscription":1,"endpoint":"d.m.uart0","time_ns":1000000,"value":{"hex":"48690a"}}}
{"jsonrpc":"2.0","method":"stopped","params":{"reason":"target","time_ns":5000000}}
{"jsonrpc":"2.0","method":"mcu_stopped","params":{"mcu":"d.m","state":"fault","message":"...","time_ns":123456}}
```

`stopped` is sent when a `run` reaches its target or is paused; `mcu_stopped`
when an MCU halts, faults or goes to sleep with no wake event. If a client
cannot keep up the simulation waits for it (and the connection is dropped if
it never reads).

## Example

```
-> {"jsonrpc":"2.0","id":1,"method":"load","params":{"yaml":"version: 1\n..."}}
<- {"jsonrpc":"2.0","id":1,"result":{"mcus":[{"name":"node.main","model":"tlsr8258"}],...}}
-> {"jsonrpc":"2.0","id":2,"method":"subscribe","params":{"endpoint":"node.main.uart0"}}
-> {"jsonrpc":"2.0","id":3,"method":"run","params":{"for":"1s","wait":true}}
<- {"jsonrpc":"2.0","method":"event","params":{...}}      (while it runs)
<- {"jsonrpc":"2.0","id":3,"result":{"reason":"target","time_ns":1000000000}}
-> {"jsonrpc":"2.0","id":4,"method":"set","params":{"endpoint":"node.temp1.temperature_c","value":31.5}}
```

## Realtime pacing

By default virtual time runs as fast as the host allows. `set_speed` (or the
`speed` parameter of `run`) paces a run to the wall clock: `1` is realtime,
`0.5` half speed, `10` ten times faster, `0`/`"max"` unpaced. The range is
0.001 to 1000. Pacing is an upper bound: virtual time never runs ahead of
`speed` times the wall clock, but a host that cannot keep up simply falls
behind (the backlog is dropped after 50 ms of virtual time rather than
replayed in a burst). The emulator executes a busy 48 MHz loop at roughly
0.1x realtime, so `speed: 1` pays off for firmware that sleeps most of the
time, which is the usual case for the devices modeled here.

Inputs given without `at` (`set`, `drive`, UART writes) land at the current
virtual time, i.e. when the user acted. The input is then part of the run;
runs with live inputs are not reproducible unless the inputs are replayed with
explicit `at` times. The server sleeps in `poll` while a paced run is ahead of
the wall clock, so an idle realtime session does not use a CPU.
Not modeled: authentication beyond the token, TLS (put a reverse proxy in
front for `wss://`), and binary frames (text only).
