#!/usr/bin/env python3
"""Reactive EZSP/ASH host for the Silicon Labs NCP (EFR32MG21) in a `mcuemu serve` simulation.

    ncp_host.py YAML --root DIR [--mcuemu PATH] [--for 40s] [--channel 11] [--pan a134]
                [--uart ncp.mcu.uart0] [--permit 255] [--events 'at=8s endpoint=board.key.pressed value=true' ...]

Loads the system description over the control API (docs/control-api.md), then plays the host: ASH reset,
EZSP version, stack profile / security level, networkInit, setInitialSecurityState (the Zigbee Alliance
09 trust-center link key), formNetwork, permitJoining, and then polls `callback` and prints what the
NCP reports. The ASH sequence and ack numbers follow the NCP's frames, so unlike a pre-computed scenario
the host never gets out of step. Needs only the standard library (a minimal WebSocket client is built in).
"""
import argparse
import base64
import json
import os
import socket
import struct
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
import ash_frame as ash  # noqa: E402


class WebSocket:
    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=120)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((f"GET / HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nUpgrade: websocket\r\n"
                           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        reply = b""
        while b"\r\n\r\n" not in reply:
            reply += self.sock.recv(1)
        if b" 101 " not in reply.split(b"\r\n")[0]:
            raise RuntimeError("WebSocket upgrade refused: " + reply.decode(errors="replace"))
        self.buffer = b""

    def send(self, text):
        data = text.encode()
        mask = os.urandom(4)
        header = bytes([0x81])
        if len(data) < 126:
            header += bytes([0x80 | len(data)])
        elif len(data) < 65536:
            header += bytes([0x80 | 126]) + struct.pack(">H", len(data))
        else:
            header += bytes([0x80 | 127]) + struct.pack(">Q", len(data))
        self.sock.sendall(header + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def _need(self, n):
        while len(self.buffer) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("connection closed")
            self.buffer += chunk

    def recv(self):
        payload = b""
        while True:
            self._need(2)
            first, second = self.buffer[0], self.buffer[1]
            length, pos = second & 0x7F, 2
            if length == 126:
                self._need(4); length, pos = struct.unpack(">H", self.buffer[2:4])[0], 4
            elif length == 127:
                self._need(10); length, pos = struct.unpack(">Q", self.buffer[2:10])[0], 10
            self._need(pos + length)
            body = self.buffer[pos:pos + length]
            self.buffer = self.buffer[pos + length:]
            opcode = first & 0x0F
            if opcode == 8:
                raise ConnectionError("server closed")
            if opcode in (0, 1, 2):
                payload += body
                if first & 0x80:
                    return payload.decode()


class Sim:
    def __init__(self, mcuemu, root, yaml_text, base_dir="."):
        self.proc = subprocess.Popen([mcuemu, "serve", "--root", root, "--port", "0"], stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True)
        port = None
        for line in self.proc.stdout:
            if "ws://" in line:
                port = int(line.rsplit(":", 1)[1].split("/")[0])
                break
        if port is None:
            raise RuntimeError("mcuemu serve did not report a port")
        self.ws, self.next_id, self.events = WebSocket(port), 1, []
        self.rpc("load", {"yaml": yaml_text, "base_dir": base_dir})

    def rpc(self, method, params=None):
        ident = self.next_id
        self.next_id += 1
        self.ws.send(json.dumps({"jsonrpc": "2.0", "id": ident, "method": method, "params": params or {}}))
        while True:
            message = json.loads(self.ws.recv())
            if message.get("id") == ident:
                if "error" in message:
                    raise RuntimeError(f"{method}: {message['error']}")
                return message["result"]
            if message.get("method") == "event":
                self.events.append(message["params"])

    def close(self):
        try:
            self.rpc("shutdown")
        except Exception:
            pass
        self.proc.terminate()


def duration_ns(text):
    for suffix, scale in (("ms", 1e6), ("us", 1e3), ("ns", 1), ("s", 1e9)):
        if text.endswith(suffix):
            return int(float(text[:-len(suffix)]) * scale)
    return int(text)


class Host:
    """ASH host: one DATA frame in flight, every NCP DATA frame acknowledged in the next frame we send."""

    def __init__(self, sim, uart):
        self.sim, self.uart = sim, uart
        self.stream = b""
        self.tx_seq = 0       # next host frame number
        self.rx_next = 0      # next NCP frame number expected (our ack)
        self.waiting = False  # a command is awaiting its response
        self.ezsp_seq = 0
        self.log = []

    def send_bytes(self, data):
        if os.environ.get("ASHDBG"):
            print("   host", [(k, d[:48]) for k, d in ash.parse(data if data.startswith(b"\x1a") else b"\x1a" + data)])
        self.sim.rpc("set", {"endpoint": self.uart, "value": {"hex": data.hex()}})

    def reset(self):
        self.tx_seq = self.rx_next = 0
        self.send_bytes(b"\x1a" + ash.frame(0xC0))

    def command(self, frame_id, payload=b"", legacy=False):
        """Sends one EZSP command (extended frame format unless legacy) and returns its sequence byte."""
        seq = self.ezsp_seq & 0xFF
        self.ezsp_seq += 1
        header = bytes([seq, 0x00, frame_id & 0xFF]) if legacy else bytes([seq, 0x00, 0x01, frame_id & 0xFF, frame_id >> 8])
        self.send_bytes(ash.data_frame(self.tx_seq, self.rx_next, header + payload))
        self.tx_seq = (self.tx_seq + 1) & 7
        self.waiting = True
        return seq

    def pump(self):
        """Takes the NCP's bytes from the event queue; returns [(kind, detail, ezsp_bytes_or_None)]."""
        out = []
        for event in self.sim.events:
            if event["endpoint"] == self.uart and "hex" in event["value"]:
                self.stream += bytes.fromhex(event["value"]["hex"])
        self.sim.events.clear()
        while 0x7E in self.stream:
            raw, self.stream = self.stream.split(b"\x7e", 1)
            for kind, detail in ash.parse(b"\x1a" + raw + b"\x7e"):
                ezsp = None
                if kind == "DATA":
                    parts = detail.split(": ")
                    ezsp = bytes.fromhex(parts[1].replace(" ", "")) if len(parts) > 1 and parts[1] else b""
                    seq = int(detail.split()[1])
                    if " retx 0" in detail or seq == self.rx_next:
                        self.rx_next = (seq + 1) & 7
                    self.send_bytes(ash.frame(0x80 | self.rx_next))
                if os.environ.get("ASHDBG"):
                    print(f"   ash {kind} {detail[:60]}")
                out.append((kind, detail, ezsp))
        return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("yaml"); p.add_argument("--root", required=True)
    p.add_argument("--mcuemu", default=os.path.join(os.path.dirname(__file__), "..", "build", "mcuemu"))
    p.add_argument("--for", dest="duration", default="40s")
    p.add_argument("--uart", default="ncp.mcu.uart0")
    p.add_argument("--channel", type=lambda v: int(v, 0), default=11)
    p.add_argument("--pan", default="a134"); p.add_argument("--permit", type=int, default=255)
    p.add_argument("--power", type=int, default=8)
    p.add_argument("--tc-key", default="5a6967426565416c6c69616e63653039")
    p.add_argument("--nwk-key", default="000102030405060708090a0b0c0d0e0f")
    p.add_argument("--permit-at", default="6s")
    p.add_argument("--poll", default="50ms", help="callback polling interval once the network is up")
    p.add_argument("--peek", action="append", default=[], help="MCU:ADDRESS:LENGTH, printed at the end (mem.read)")
    p.add_argument("--get", action="append", default=[], help="ENDPOINT, printed at the end (get)")
    p.add_argument("--watch", action="append", default=[],
                   help="MCU:ADDRESS:LENGTH@FROM-TO: print the bytes whenever they change inside that time window (1 ms steps)")
    p.add_argument("--event", action="append", default=[], help="at=T endpoint=NAME value=V (scenario-like set)")
    a = p.parse_args()

    root = os.path.abspath(a.root)
    yaml_path = os.path.join(root, a.yaml) if not os.path.isabs(a.yaml) else a.yaml
    sim = Sim(os.path.abspath(a.mcuemu), root, open(yaml_path).read(), base_dir=os.path.dirname(os.path.relpath(yaml_path, root)) or ".")
    host = Host(sim, a.uart)
    sim.rpc("subscribe", {"endpoint": a.uart})
    for e in a.event:
        kv = dict(x.split("=", 1) for x in e.split())
        value = {"true": True, "false": False}.get(kv["value"], kv["value"])
        sim.rpc("set", {"endpoint": kv["endpoint"], "value": value, "at": kv["at"]})

    watches = []
    for spec in a.watch:
        where, window = spec.rsplit("@", 1)
        mcu, address, length = where.rsplit(":", 2)
        lo, hi = window.split("-")
        watches.append([mcu, int(address, 0), int(length, 0), duration_ns(lo), duration_ns(hi), None])
    end = duration_ns(a.duration)
    permit_at, poll = duration_ns(a.permit_at), duration_ns(a.poll)
    step = 1_000_000
    plan = [  # (name, frame id, payload, legacy)
        ("version", 0x0000, bytes([0x12]), True),
        ("setConfigurationValue(stackProfile=2)", 0x0053, bytes([0x0C, 0x02, 0x00]), False),
        ("setConfigurationValue(securityLevel=5)", 0x0053, bytes([0x0D, 0x05, 0x00]), False),
        ("networkInit", 0x0017, bytes([0x00, 0x00]), False),
        ("setInitialSecurityState", 0x0068, struct.pack("<H", 0x0304) + bytes.fromhex(a.tc_key) + bytes.fromhex(a.nwk_key) + bytes([0]) + bytes(8), False),
        ("formNetwork", 0x001E, bytes.fromhex("aabbccddeeff0011") + bytes.fromhex(a.pan)[::-1] + bytes([a.power, a.channel, 0]) + struct.pack("<H", 0) + bytes([0]) + struct.pack("<I", 1 << a.channel), False),
        ("permitJoining", 0x0022, bytes([a.permit]), False),
    ]
    names = {0x0017: "networkInit", 0x001B: "networkFoundHandler", 0x0019: "stackStatusHandler",
             0x0023: "childJoinHandler", 0x0024: "trustCenterJoinHandler", 0x0045: "incomingMessageHandler",
             0x003F: "messageSentHandler", 0x0007: "noCallbacks"}
    state, step_index, last_poll, ready_at = "boot", 0, 0, None
    now = 0
    try:
        sim.rpc("step", {"duration": "1ms"})
        while now < end:
            sim.rpc("step", {"duration": f"{step}ns"})
            now += step
            for w in watches:
                if w[3] <= now <= w[4]:
                    value = sim.rpc("mem.read", {"mcu": w[0], "address": w[1], "length": w[2]})["hex"]
                    if value != w[5]:
                        print(f"{now/1e9:8.3f}s watch {w[0]}:{w[1]:#x} {value}")
                        w[5] = value
            for kind, detail, ezsp in host.pump():
                if kind == "RSTACK" and state == "boot":
                    state = "plan"; host.waiting = False
                    print(f"{now/1e9:8.3f}s RSTACK {detail}")
                elif kind == "DATA":
                    host.waiting = False
                    if len(ezsp) >= 5 and ezsp[3:5] == b"\x07\x00" and state == "run":
                        continue  # noCallbacks
                    fid = (ezsp[3] | ezsp[4] << 8) if len(ezsp) >= 5 else ezsp[2] if len(ezsp) >= 3 else -1
                    print(f"{now/1e9:8.3f}s EZSP 0x{fid:04x} {names.get(fid, '')} {ezsp.hex(' ')}")
                elif kind in ("ERROR", "bad", "NAK"):
                    print(f"{now/1e9:8.3f}s ASH {kind} {detail}")
            if state == "boot" and now == 1_200_000_000:
                host.reset()
            if state == "plan" and not host.waiting and now - (ready_at or 0) >= 20_000_000:
                if step_index < len(plan):
                    name, fid, payload, legacy = plan[step_index]
                    if name == "permitJoining" and now < permit_at:
                        pass
                    else:
                        print(f"{now/1e9:8.3f}s -> {name}")
                        host.command(fid, payload, legacy)
                        step_index += 1
                        ready_at = now
                else:
                    state = "run"
            elif state == "run" and not host.waiting and now - last_poll >= poll:
                host.command(0x0006); last_poll = now
        for endpoint in a.get:
            print(f"get {endpoint}: {sim.rpc('get', {'endpoint': endpoint})}")
        for spec in a.peek:
            mcu, address, length = spec.rsplit(":", 2)
            print(f"peek {spec}: {sim.rpc('mem.read', {'mcu': mcu, 'address': int(address, 0), 'length': int(length, 0)})['hex']}")
    finally:
        sim.close()


if __name__ == "__main__":
    main()
