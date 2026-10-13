#include "../test_support.hpp"

#include "sim/config.hpp"
#include "sim/system.hpp"
#include "sim/tlsr8258_mcu.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

namespace tc32emu_tests {

namespace {

using tc32emu::Tlsr8258;

const std::filesystem::path &scratch() {
    static const std::filesystem::path dir = [] {
        auto path = std::filesystem::temp_directory_path() / "simcore-attach-tests";
        std::filesystem::create_directories(path);
        std::ofstream out(path / "idle.bin", std::ios::binary);
        out.put(static_cast<char>(0xfe));
        out.put(static_cast<char>(0x87)); // b .
        return path;
    }();
    return dir;
}

std::string idle() { return (scratch() / "idle.bin").string(); }
std::string path_of(const std::string &name) { return (scratch() / name).string(); }

std::string read_file(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

std::string mcu_yaml(const std::string &extra = "", const std::string &mcu_extra = "") {
    return "version: 1\ndevices:\n  - id: d\n    mcus:\n"
           "      - {id: m, model: tlsr8258, firmware: " + idle() + mcu_extra + "}\n" + extra;
}

std::unique_ptr<sim::System> build(const std::string &yaml) {
    return sim::build_system(sim::parse_system_config(yaml));
}

template <typename Fn> std::string error_of(Fn &&fn) {
    try {
        fn();
    } catch (const sim::ConfigError &error) {
        return error.what();
    } catch (const std::exception &error) {
        return error.what();
    }
    return {};
}

bool contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

Tlsr8258 &device_of(sim::System &system) {
    return static_cast<sim::Tlsr8258Mcu &>(system.world().mcu("d.m")).device();
}

void transmit_uart(Tlsr8258 &device, std::uint8_t byte) {
    device.write8(Tlsr8258::UartControl0, 0x4f);
    device.write8(Tlsr8258::UartData, byte);
}

const sim::Bytes *bytes_of(const sim::Value &value) { return std::get_if<sim::Bytes>(&value); }

} // namespace

void test_uart_endpoint() {
    const auto system = build(mcu_yaml());
    sim::World &world = system->world();
    Tlsr8258 &device = device_of(*system);
    auto &uart = world.endpoints().at("d.m.uart0");
    CHECK(uart.kind() == "uart");

    std::vector<sim::EndpointEvent> events;
    uart.subscribe([&](const sim::EndpointEvent &e) { events.push_back(e); });
    transmit_uart(device, 0x5a);
    world.run_for(1000000);
    CHECK(events.size() == 1);
    CHECK(events.size() == 1 && bytes_of(events[0].value) &&
          *bytes_of(events[0].value) == sim::Bytes{0x5a});
    CHECK(std::get<std::int64_t>(uart.get()) == 1);

    // Bytes pushed into the endpoint reach the RX FIFO; the surplus waits.
    uart.set(sim::Value{sim::Bytes{1, 2, 3, 4, 5, 6}});
    world.run_for(100000);
    CHECK((device.read8(Tlsr8258::UartBufferCount) & 0x0fu) == 4); // 4-byte FIFO
    CHECK(device.read8(Tlsr8258::UartData + 3) == 1);              // drains one
    world.run_for(100000);
    CHECK((device.read8(Tlsr8258::UartBufferCount) & 0x0fu) == 4);
    uart.set(sim::Value{std::string("xy")});

    // A transmit-only UART refuses input.
    const auto ca51 = build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                            "      - {id: m, model: ca51f253l3, firmware: " + idle() + "}\n");
    CHECK(contains(error_of([&] {
                       ca51->world().endpoints().at("d.m.uart0").set(sim::Value{std::string("a")});
                   }),
                   "cannot receive"));
    CHECK(contains(error_of([&] { uart.set(sim::Value{3.5}); }), "bytes or a string"));
}

void test_rtt_endpoint() {
    const auto system = build(mcu_yaml("", ", rtt: {address: 0x840400, channels: 2, interval: 10us}"));
    sim::World &world = system->world();
    Tlsr8258 &device = device_of(*system);
    constexpr std::uint32_t cb = 0x840400, up_buf = 0x840500, down_buf = 0x840600;

    std::vector<std::string> up;
    world.endpoints().at("d.m.rtt").subscribe([&](const sim::EndpointEvent &e) {
        const auto *b = bytes_of(e.value);
        CHECK(b != nullptr);
        if (b)
            up.emplace_back(b->begin(), b->end());
    });
    CHECK(world.endpoints().find("d.m.rtt1") != nullptr);
    CHECK(world.endpoints().find("d.m.rtt2") == nullptr);

    auto w32 = [&](std::uint32_t at, std::uint32_t v) { device.write32(at, v); };
    auto put = [&](std::uint32_t at, const std::string &text) {
        for (std::size_t i = 0; i < text.size(); ++i)
            device.write8(at + static_cast<std::uint32_t>(i), static_cast<std::uint8_t>(text[i]));
    };
    // Before the firmware initializes the block nothing happens.
    world.run_for(100000);
    CHECK(up.empty());

    put(cb, std::string("SEGGER RTT\0\0\0\0\0\0", 16));
    w32(cb + 16, 1); // MaxNumUpBuffers
    w32(cb + 20, 1); // MaxNumDownBuffers
    w32(cb + 24 + 4, up_buf);  // up[0].pBuffer
    w32(cb + 24 + 8, 16);      // up[0].SizeOfBuffer
    w32(cb + 48 + 4, down_buf); // down[0].pBuffer (after one 24-byte up descriptor)
    w32(cb + 48 + 8, 8);
    put(up_buf, "hello");
    w32(cb + 24 + 12, 5); // WrOff
    world.run_for(100000);
    CHECK(up.size() == 1 && up[0] == "hello");
    CHECK(device.read32(cb + 24 + 16) == 5); // the host advanced RdOff

    // A write that wraps around the end of the buffer arrives in one piece.
    put(up_buf + 5, "ABCDEFGHIJK"); // offsets 5..15
    put(up_buf, "lm");              // offsets 0..1
    w32(cb + 24 + 12, 2);
    world.run_for(100000);
    CHECK(up.size() == 2 && up[1] == "ABCDEFGHIJKlm");
    CHECK(device.read32(cb + 24 + 16) == 2);
    CHECK(std::get<std::int64_t>(world.endpoints().at("d.m.rtt").get()) == 18);

    // Down buffer: 7 of 10 bytes fit, the rest follows once the firmware reads.
    world.endpoints().at("d.m.rtt").set(sim::Value{std::string("0123456789")});
    world.run_for(100000);
    std::string seen;
    for (unsigned i = 0; i < 7; ++i)
        seen.push_back(static_cast<char>(device.read8(down_buf + i)));
    CHECK(seen == "0123456");
    CHECK(device.read32(cb + 48 + 12) == 7); // WrOff
    w32(cb + 48 + 16, 7);                    // firmware consumed everything
    world.run_for(100000);
    CHECK(device.read8(down_buf + 7) == '7' && device.read8(down_buf) == '8' &&
          device.read8(down_buf + 1) == '9');
    CHECK(device.read32(cb + 48 + 12) == 2);

    // Garbage descriptors are ignored rather than trusted.
    w32(cb + 24 + 12, 99);
    world.run_for(100000);
    CHECK(up.size() == 2);

    CHECK(contains(error_of([] { build(mcu_yaml("", ", rtt: {address: 0x7000000}")); }),
                   "line 5"));
    CHECK(contains(error_of([] {
                       build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                             "      - {id: m, model: ca51f253l3, firmware: " + idle() +
                             ", rtt: {address: 0}}\n");
                   }),
                   "does not expose memory"));
    CHECK(contains(error_of([] { build(mcu_yaml("", ", rtt: {}")); }), "requires 'address'"));
    CHECK(contains(error_of([] { build(mcu_yaml("", ", rtt: {address: 0x840400, channels: 99}")); }),
                   "channels must be 1..16"));
}

void test_led_and_button() {
    const auto system = build(mcu_yaml(R"(    nets:
      - { id: line, pull: up }
    parts:
      - { id: btn, type: button, pins: { out: line } }
      - { id: led, type: led, pins: { in: line }, params: { active: low } }
scenario:
  events:
    - { at: 100us, endpoint: d.btn.pressed, set: true }
    - { at: 300us, endpoint: d.btn.pressed, set: false }
)"));
    sim::World &world = system->world();
    auto &led = world.endpoints().at("d.led.on");
    CHECK(std::get<bool>(led.get()) == false); // line idles high: active-low LED is off
    std::vector<std::pair<std::uint64_t, bool>> changes;
    led.subscribe([&](const sim::EndpointEvent &e) {
        changes.emplace_back(e.time_ns, std::get<bool>(e.value));
    });
    world.run_until(200000);
    CHECK(std::get<bool>(led.get()) == true);
    world.run_until(400000);
    CHECK(std::get<bool>(led.get()) == false);
    CHECK(changes.size() == 2);
    CHECK(changes.size() == 2 && changes[0].first >= 100000 && changes[0].first < 106000 &&
          changes[0].second && changes[1].first >= 300000 && !changes[1].second);
    CHECK(contains(error_of([&] { led.set(sim::Value{true}); }), "read-only"));
    CHECK(contains(error_of([] {
                       build(mcu_yaml("    nets:\n      - {id: n}\n    parts:\n"
                                      "      - {id: l, type: led, pins: {in: n}, params: {active: x}}\n"));
                   }),
                   "high or low"));
}

void test_sinks_file_stdout_vcd_pcap() {
    const auto system = build(mcu_yaml(R"(    nets:
      - { id: a, pull: up }
      - { id: b }
    parts:
      - { id: btn, type: button, pins: { out: a } }
      - { id: ntc, type: ntc, pins: { out: b } }
world: { radio: [ { id: air0 } ] }
)"));
    sim::World &world = system->world();
    sim::Session &session = system->session();

    // file: lines for non-byte values, raw bytes for UARTs.
    session.attach("d.a", "file:" + path_of("net.log"));
    session.attach("d.m.uart0", "file:" + path_of("uart.log"));
    // stdout: captured here.
    std::ostringstream captured;
    auto *old = std::cout.rdbuf(captured.rdbuf());
    session.attach("d.m.uart0", "stdout");
    transmit_uart(device_of(*system), 'o');
    world.endpoints().at("d.btn.pressed").set(sim::Value{true});
    world.run_for(100000);
    transmit_uart(device_of(*system), 'k');
    world.run_for(100000);
    world.endpoints().at("d.btn.pressed").set(sim::Value{false});
    std::cout.rdbuf(old);
    CHECK(captured.str() == "ok");
    CHECK(read_file(path_of("uart.log")) == "ok");
    const std::string net_log = read_file(path_of("net.log"));
    CHECK(contains(net_log, " d.a = 0\n") && contains(net_log, " d.a = 1\n"));

    // vcd: two signals (a boolean net and the NTC voltage) in one file.
    session.attach("d.a", "vcd:" + path_of("wave.vcd"));
    session.attach("d.ntc.mv", "vcd:" + path_of("wave.vcd"));
    world.run_for(1000);
    world.endpoints().at("d.btn.pressed").set(sim::Value{true});
    world.endpoints().at("d.ntc.temp_c").set(sim::Value{40.0});
    world.run_for(1000);
    session.finish();
    const std::string vcd = read_file(path_of("wave.vcd"));
    CHECK(contains(vcd, "$timescale 1 ns $end"));
    CHECK(contains(vcd, "$var wire 1 ! d_a $end"));
    CHECK(contains(vcd, "$var real 64 \" d_ntc_mv $end"));
    CHECK(contains(vcd, "$dumpvars\n1!\n"));
    CHECK(contains(vcd, "0!\n"));

    // pcap: Zigbee frames, 802.15.4 without FCS (link type 230).
    session.attach("air0", "pcap:" + path_of("air.pcap"));
    auto &air = world.radio("air0");
    air.transmit("x", "", tc32emu::RadioProtocol::Zigbee802154, 15, 1500000000, {1, 2, 3});
    air.transmit("x", "", tc32emu::RadioProtocol::Ble, 37, 1600000000, {9});
    air.transmit("y", "", tc32emu::RadioProtocol::Zigbee802154, 15, 2000000, {4, 5});
    session.finish();
    const std::string pcap = read_file(path_of("air.pcap"));
    auto u32 = [&](std::size_t at) {
        std::uint32_t v = 0;
        std::memcpy(&v, pcap.data() + at, 4);
        return v;
    };
    CHECK(pcap.size() == 24 + (16 + 3) + (16 + 2)); // the BLE frame is filtered
    CHECK(u32(0) == 0xa1b2c3d4u && u32(20) == 230u);
    CHECK(u32(24) == 1 && u32(28) == 500000); // 1.5 s since the epoch of the run
    CHECK(u32(32) == 3 && u32(36) == 3);
    CHECK(static_cast<std::uint8_t>(pcap[40]) == 1);

    // detach stops delivery.
    const auto id = session.attach("d.a", "file:" + path_of("detached.log"));
    CHECK(session.detach(id));
    CHECK(!session.detach(id));
    world.endpoints().at("d.btn.pressed").set(sim::Value{false});
    CHECK(read_file(path_of("detached.log")).empty());
}

void test_hex_sink_groups_frames() {
    const auto system = build(mcu_yaml(""));
    sim::World &world = system->world();
    sim::Session &session = system->session();
    std::ostringstream captured;
    auto *old = std::cout.rdbuf(captured.rdbuf());
    session.attach("d.m.uart0", "hex");        // default gap: 1 ms
    transmit_uart(device_of(*system), 'o');
    world.run_for(100000);
    transmit_uart(device_of(*system), 'k');    // within the gap: same line
    world.run_for(100000);
    session.finish();
    std::cout.rdbuf(old);
    CHECK(contains(captured.str(), "d.m.uart0  2 bytes: 6f 6b\n"));

    // A shorter gap splits the same traffic into two lines.
    const auto split = build(mcu_yaml(""));
    sim::Session &split_session = split->session();
    std::ostringstream split_captured;
    old = std::cout.rdbuf(split_captured.rdbuf());
    split_session.attach("d.m.uart0", "hex:50us");
    transmit_uart(device_of(*split), 'o');
    split->world().run_for(100000);
    transmit_uart(device_of(*split), 'k');
    split->world().run_for(100000);
    split_session.finish();
    std::cout.rdbuf(old);
    CHECK(contains(split_captured.str(), "d.m.uart0  1 byte: 6f\n"));
    CHECK(contains(split_captured.str(), "d.m.uart0  1 byte: 6b\n"));
}

void test_text_sink_makes_lines() {
    const auto system = build(mcu_yaml(""));
    sim::World &world = system->world();
    std::ostringstream captured;
    auto *old = std::cout.rdbuf(captured.rdbuf());
    system->session().attach("d.m.uart0", "text");
    for (const char c : std::string("hi\r\nx\x01y\n!")) {
        transmit_uart(device_of(*system), static_cast<std::uint8_t>(c));
        world.run_for(100000);
    }
    system->session().finish();
    std::cout.rdbuf(old);
    const std::string out = captured.str();
    CHECK(contains(out, " s] d.m.uart0: hi\n"));
    CHECK(contains(out, " s] d.m.uart0: x\\x01y\n"));
    CHECK(contains(out, " s] d.m.uart0: !\n")); // the partial line at the end
}

void test_tcp_sink() {
    const auto system = build(mcu_yaml());
    sim::World &world = system->world();
    sim::Session &session = system->session();
    const auto id = session.attach("d.m.uart0", "tcp:0");
    auto *tcp = dynamic_cast<sim::TcpSink *>(session.sink(id));
    CHECK(tcp != nullptr);
    if (!tcp)
        return;

    const int client = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(tcp->port());
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    CHECK(::connect(client, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0);
    session.poll(); // accept

    // Client -> UART RX FIFO.
    CHECK(::send(client, "xyz", 3, 0) == 3);
    for (int i = 0; i < 50 && (device_of(*system).read8(Tlsr8258::UartBufferCount) & 0x0f) < 3; ++i) {
        ::usleep(2000);
        session.poll();
        world.run_for(10000);
    }
    CHECK((device_of(*system).read8(Tlsr8258::UartBufferCount) & 0x0fu) == 3);
    CHECK(device_of(*system).read8(Tlsr8258::UartData + 3) == 'x');

    // UART TX -> client.
    transmit_uart(device_of(*system), 'Q');
    world.run_for(100000);
    pollfd waiting{client, POLLIN, 0};
    CHECK(::poll(&waiting, 1, 1000) == 1);
    char received = 0;
    CHECK(::recv(client, &received, 1, 0) == 1);
    CHECK(received == 'Q');
    ::close(client);
    session.poll(); // notices the disconnect
    transmit_uart(device_of(*system), 'R');
    world.run_for(100000); // no client: the byte is dropped, nothing breaks

    // Only loopback by default, and bad specs are rejected.
    CHECK(contains(error_of([&] { session.attach("d.m.uart0", "tcp:99999"); }), "0..65535"));
    CHECK(contains(error_of([&] { session.attach("d.m.uart0", "tcp:1.2.3:80"); }), "IPv4"));
    CHECK(contains(error_of([&] {
                       session.attach("d.m.uart0", "tcp:" + std::to_string(tcp->port()));
                   }),
                   "cannot listen"));
}

void test_attach_config_and_errors() {
    // `attach:` at device scope (relative names) and at world scope.
    const std::string out_a = path_of("cfg_uart.log");
    const std::string out_b = path_of("cfg_net.log");
    const auto system = build(mcu_yaml(R"(    nets:
      - { id: line, pull: up }
    parts:
      - { id: btn, type: button, pins: { out: line } }
    attach:
      - { endpoint: m.uart0, sink: "file:)" + out_a + R"(" }
attach:
  - { endpoint: d.line, sink: "file:)" + out_b + R"(" }
)"));
    transmit_uart(device_of(*system), 'z');
    system->world().endpoints().at("d.btn.pressed").set(sim::Value{true});
    system->world().run_for(100000);
    system->session().finish();
    CHECK(read_file(out_a) == "z");
    CHECK(contains(read_file(out_b), " d.line = 0\n"));
    CHECK(system->session().size() == 2);

    auto err = [&](const std::string &attach_yaml) {
        return error_of([&] { build(mcu_yaml() + attach_yaml); });
    };
    CHECK(contains(err("attach:\n  - { endpoint: d.nothing, sink: stdout }\n"),
                   "unknown endpoint 'd.nothing'"));
    CHECK(contains(err("attach:\n  - { endpoint: d.m.uart0, sink: smoke }\n"),
                   "unknown sink 'smoke'"));
    CHECK(contains(err("attach:\n  - { endpoint: d.m.uart0, sink: \"vcd:x.vcd\" }\n"),
                   "vcd needs a boolean or numeric endpoint"));
    CHECK(contains(err("attach:\n  - { endpoint: d.m.uart0, sink: \"pcap:x.pcap\" }\n"),
                   "pcap needs a radio endpoint"));
    CHECK(contains(err("attach:\n  - { endpoint: d.m.uart0 }\n"), "requires 'sink'"));
    CHECK(contains(err("attach:\n  - { endpoint: d.m.uart0, sink: stdout, x: 1 }\n"),
                   "unknown key 'x'"));
    CHECK(contains(err("attach:\n  - { endpoint: d.m.uart0, sink: \"file:/no/such/dir/x\" }\n"),
                   "cannot write"));
}

} // namespace tc32emu_tests
