# Bluetooth LE on the shared air

Status: work in progress. The pieces that exist today:

| Piece | Where | State |
| --- | --- | --- |
| Medium | `mcuemu::VirtualAir` (`radio.hpp`) | BLE frames carry an access address, a CRC seed and an `ll_framed` flag; receivers can filter by access address; airtime from the real frame layout (1M and 2M) |
| pcap | `pcap:PATH,ble` | link type 256 (`LE_LL_WITH_PHDR`): channel, access address, PDU and CRC-24, de-whitened |
| Link layer helpers | `emulator/include/ble/ll.hpp` (`bleemu`) | CRC-24, whitening, access address rules, advertising and data PDUs, CONNECT_IND, SN/NESN, channel selection #1, window widening, LL control PDUs |
| nRF52840 RADIO | [nrf52840.md](nrf52840.md) | BLE 1M/2M at register level |
| TLSR8258 RF | [tlsr8258-register-model.md](tlsr8258-register-model.md#ble-mode-of-the-rf) | BLE transmit and receive from the link layer commands: advertising, scan response and connection events of the ATC firmware (the slave answers from the transmit queue of the DMA channel 3, e.g. an ATT MTU response) |
| Virtual host | `sim::BleHost` (`sim/ble_host.hpp`) | HCI controller + central/scanner/initiator link layer (this page) |
| `ble_host` part and HCI transports | `parts/ble_host_part.cpp` | HCI as the endpoint `<part>.hci` (H4 bytes); TCP through the `tcp:` sink, the control API and the browser build through `set`/`subscribe` |

CRC and whitening were checked against independent implementations (scapy's
`BTLE.compute_crc` and the published channel 37 whitening sequence); the rest
follows the specification text and has no silicon to compare with.

## The virtual host

`BleHost` is the phone: a node on the air that scans, connects and exchanges
data with the firmware under test (a connectable peripheral). It speaks HCI over
H4 packets, so a host stack (BlueZ, Bumble) can sit on top through a transport,
and tests drive it directly:

```cpp
sim::BleHost host(air, scheduler, "host", {});
host.set_hci_sink([](const std::vector<uint8_t> &h4) { /* events and ACL data */ });
host.hci_send({0x01, 0x0C, 0x20, 0x02, 0x01, 0x00}); // LE Set Scan Enable
```

The host runs on the `sim::Scheduler`: connection events happen at exact virtual
times, replies go out `T_IFS` (150 us) after the end of the frame they answer,
and other nodes' frames reach it through the air's frame observer.

### As a part, over TCP

```yaml
parts:
  - { id: host, type: ble_host, radio: { air: air0, protocols: [ble] },
      params: { address: "C0:DE:00:00:00:01" } }
attach:
  - { endpoint: lab.host.hci, sink: "tcp:9000" }
```

The part's endpoint `lab.host.hci` carries H4 bytes: `set` takes what a host
stack writes (commands and ACL data, split or merged any way, reassembled by
`H4Framer`), subscribers get one event per controller packet. The `tcp:` sink is
therefore an HCI transport: a stack connects to the port and speaks H4 on the
byte stream (Bumble: `tcp-client:127.0.0.1:9000`). The control API's `set` and
`subscribe` carry the same bytes as `{"hex": ...}`, which is what the browser
build uses, so there are no extra RPC methods.

`mcuemu sim --speed X` paces virtual time at X times the wall clock (1 = real
time); an outside program needs it, because the default run ends as fast as it
can. `examples/ble_host.yaml` and `scripts/ble_scan.py` show the whole path:

```sh
build/mcuemu sim examples/ble_host.yaml --speed 1 --for 10m &
python3 scripts/ble_scan.py          # needs: pip install bumble
```

Checked with Bumble 0.0.236: the whole controller bring-up (`Device.power_on`)
succeeds, active scanning reports the advertisement and the scan response of the
scripted beacon (`examples/parts/ble_beacon.lua`), and `Device.connect()` gets
LE Connection Complete after a CONNECT_IND that carries the initiator address
Bumble set (the beacon does not accept connections, so the link then fails to
establish). BlueZ has not been tried (no Linux here); it should attach through
any H4-over-TCP bridge such as `btproxy`.

Scripted parts can advertise to the host: `radio.transmit{..., ll = true}` sends
a whole link-layer PDU (see [scripting.md](scripting.md)).

### What it does

- **HCI**: Reset, event masks (accepted), Read Local Version/Commands/Features/
  Extended Features (page 0), Read Buffer Size, Read BD_ADDR, LE Read Buffer
  Size (251 bytes, 8 packets), LE Read Local Supported Features, LE Set Random
  Address, LE Set Scan Parameters/Enable, LE Create Connection (+ Cancel), the
  filter accept list commands, LE Connection Update, LE Read Remote Features,
  Read Remote Version, LE Rand, Disconnect. Unknown opcodes get a Command Status
  with "unknown HCI command". `Read Local Supported Commands` lists exactly
  these, so a host stack does not ask for more. (The bit positions were written
  from memory of the specification table; a mismatch would show up when a real
  stack reads them.)
- **Scanning**: passive and active, rotating channels 37/38/39 every scan
  interval and listening for the first `window` of each (a pure function of
  time, no timers), duplicate filtering, accept list policy. An ADV_IND report is
  followed by a SCAN_REQ `T_IFS` later and a second report (type 4) for the
  SCAN_RSP. Reports carry the medium's RSSI.
- **Initiating**: the first connectable advertisement from the peer (or an
  accept-listed device) is answered with CONNECT_IND `T_IFS` later. Random
  access address (validated), CRC seed and hop increment; interval
  `interval_max` of the command; SCA field 7 (this clock is exact); transmit
  window offset 0, size 2. LE Connection Complete is reported when CONNECT_IND
  goes out.
- **Connection**: the first packet 1.25 ms after CONNECT_IND, then every
  interval exactly; channel selection algorithm #1; SN/NESN with
  retransmission; MD continues an event while it fits before the next anchor;
  ACL fragmentation into 27-byte LL PDUs and the reverse; Number of Completed
  Packets after the last fragment is acknowledged; LL control: feature and
  version exchange, ping, terminate (both directions), connection update with
  an instant eight events ahead, UNKNOWN_RSP for everything else (connection
  parameter request, length, PHY).
- **Failure**: no answer in the first six events gives Disconnection Complete
  0x3E, silence for the supervision timeout gives 0x08.

### Limits (v1)

One connection at a time; no encryption (LL or SMP is the host stack's, but LL
encryption procedures are refused); no data length extension, 2M/Coded PHY,
extended or periodic advertising, channel map update, privacy or CSA #2;
scanning pauses while connected; frames from Lua parts that do not set the
`ll_framed` flag are ignored; the peripheral is expected to reply within 16 us
of `T_IFS` (the specification allows 2 us).

Tests: `tests/ble/` (link layer), `tests/sim/test_ble_host.cpp` (host against a
scripted peripheral and against an nRF52840 RADIO driven through registers).
