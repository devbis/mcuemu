#include "../test_support.hpp"

#include "sim/rpc.hpp"
#include "sim/server.hpp"
#include "sim/websocket.hpp"

#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

namespace tc32emu_tests {

namespace {

using Json = nlohmann::json;
namespace fs = std::filesystem;

const fs::path &root_dir() {
    static const fs::path dir = [] {
        auto path = fs::temp_directory_path() / "simcore-control-tests";
        fs::remove_all(path);
        fs::create_directories(path / "sub");
        std::ofstream out(path / "idle.bin", std::ios::binary);
        out.put(static_cast<char>(0xfe));
        out.put(static_cast<char>(0x87)); // b .
        return fs::weakly_canonical(path);
    }();
    return dir;
}

const std::string &yaml_text() {
    static const std::string text =
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: tlsr8258, firmware: idle.bin}\n"
        "    nets:\n      - { id: line, pull: up }\n"
        "    parts:\n      - { id: btn, type: button, pins: { out: line } }\n";
    return text;
}

Json call(sim::RpcSession &rpc, const std::string &method, Json params = Json::object(),
          int id = 1) {
    const auto out = rpc.handle(Json{{"jsonrpc", "2.0"}, {"id", id}, {"method", method},
                                     {"params", params}}.dump());
    for (const std::string &message : out) {
        const Json json = Json::parse(message);
        if (json.contains("id") && json["id"] == id)
            return json;
    }
    return Json();
}

Json result_of(const Json &response) {
    CHECK(response.contains("result"));
    return response.contains("result") ? response["result"] : Json();
}

int code_of(const Json &response) {
    return response.contains("error") ? response["error"]["code"].get<int>() : 0;
}

std::string message_of(const Json &response) {
    return response.contains("error") ? response["error"]["message"].get<std::string>() : "";
}

bool contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

} // namespace

void test_websocket_primitives() {
    // Test vectors from RFC 3174 / RFC 4648 / RFC 6455.
    auto hex = [](const std::string &raw) {
        static const char digits[] = "0123456789abcdef";
        std::string text;
        for (unsigned char c : raw) {
            text.push_back(digits[c >> 4]);
            text.push_back(digits[c & 15]);
        }
        return text;
    };
    CHECK(hex(sim::ws::sha1("abc")) == "a9993e364706816aba3e25717850c26c9cd0d89d");
    CHECK(hex(sim::ws::sha1("")) == "da39a3ee5e6b4b0d3255bfef95601890afd80709");
    CHECK(hex(sim::ws::sha1(std::string(1000, 'a'))) ==
          "291e9a6c66994949b57ba5e650361e98fc36b1ba");
    CHECK(sim::ws::base64("") == "");
    CHECK(sim::ws::base64("f") == "Zg==");
    CHECK(sim::ws::base64("fo") == "Zm8=");
    CHECK(sim::ws::base64("foo") == "Zm9v");
    CHECK(sim::ws::base64("foobar") == "Zm9vYmFy");
    CHECK(sim::ws::accept_key("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");

    // Request head parsing.
    std::string head = "GET /api?token=x&a=b HTTP/1.1\r\nHost: h\r\nUpgrade: WebSocket\r\n"
                       "Connection: keep-alive, Upgrade\r\nSec-WebSocket-Version: 13\r\n"
                       "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\nrest";
    std::string partial = head.substr(0, 20);
    CHECK(!sim::ws::parse_request(partial).has_value());
    const auto request = sim::ws::parse_request(head);
    CHECK(request && request->path == "/api" && request->query == "token=x&a=b");
    CHECK(request && sim::ws::is_upgrade(*request));
    CHECK(head == "rest"); // only the head was consumed
    std::string junk = "NOT A REQUEST\r\n\r\n";
    bool threw = false;
    try { sim::ws::parse_request(junk); } catch (const std::invalid_argument &) { threw = true; }
    CHECK(threw);

    // Frames: the masked "Hello" of RFC 6455 section 5.7.
    sim::ws::FrameParser parser;
    const char hello[] = {char(0x81), char(0x85), char(0x37), char(0xfa), char(0x21), char(0x3d),
                          char(0x7f), char(0x9f), char(0x4d), char(0x51), char(0x58)};
    parser.feed(hello, 4);
    CHECK(!parser.next().has_value()); // incomplete
    parser.feed(hello + 4, sizeof hello - 4);
    const auto message = parser.next();
    CHECK(message && message->opcode == sim::ws::Opcode::Text && message->payload == "Hello");

    // Fragmentation with an interleaved ping (RFC 6455 section 5.4 / 5.5).
    auto masked = [](std::uint8_t b0, const std::string &payload) {
        std::string frame;
        frame.push_back(static_cast<char>(b0));
        frame.push_back(static_cast<char>(0x80 | payload.size()));
        const char key[4] = {1, 2, 3, 4};
        frame.append(key, 4);
        for (std::size_t i = 0; i < payload.size(); ++i)
            frame.push_back(static_cast<char>(payload[i] ^ key[i % 4]));
        return frame;
    };
    sim::ws::FrameParser fragments;
    const std::string stream = masked(0x01, "Hel") + masked(0x89, "p") + masked(0x80, "lo");
    fragments.feed(stream.data(), stream.size());
    const auto ping = fragments.next();
    CHECK(ping && ping->opcode == sim::ws::Opcode::Ping && ping->payload == "p");
    const auto whole = fragments.next();
    CHECK(whole && whole->opcode == sim::ws::Opcode::Text && whole->payload == "Hello");
    CHECK(!fragments.next().has_value());

    // Malformed input: unmasked client frame, oversize message.
    auto code_of_error = [](const std::string &bytes, std::size_t limit = 1024) {
        sim::ws::FrameParser p(limit);
        p.feed(bytes.data(), bytes.size());
        try {
            p.next();
        } catch (const sim::ws::ProtocolError &error) {
            return static_cast<int>(error.code());
        }
        return 0;
    };
    CHECK(code_of_error(std::string("\x81\x01x", 3)) == 1002);
    CHECK(code_of_error(masked(0x81, std::string(100, 'x')), 50) == 1009);
    CHECK(code_of_error(masked(0xc1, "x")) == 1002); // reserved bit

    // Server frames: 7-bit, 16-bit and 64-bit lengths.
    CHECK(sim::ws::encode_frame(sim::ws::Opcode::Text, "hi") == std::string("\x81\x02hi", 4));
    CHECK(sim::ws::encode_frame(sim::ws::Opcode::Text, std::string(125, 'a')).size() == 2 + 125);
    CHECK(sim::ws::encode_frame(sim::ws::Opcode::Text, std::string(126, 'a')).size() == 4 + 126);
    CHECK(sim::ws::encode_frame(sim::ws::Opcode::Text, std::string(65536, 'a')).size() == 10 + 65536);
    CHECK(sim::ws::encode_close(1000, "ok") == std::string("\x88\x04\x03\xe8ok", 6));
}

void test_rpc_basics() {
    sim::RpcSession rpc(root_dir().string());

    CHECK(result_of(call(rpc, "ping")) == "pong");
    CHECK(code_of(call(rpc, "no.such.method")) == -32601);
    // Malformed requests.
    CHECK(Json::parse(rpc.handle("{not json")[0])["error"]["code"] == -32700);
    CHECK(Json::parse(rpc.handle("[1]")[0])[0]["error"]["code"] == -32600);
    CHECK(Json::parse(rpc.handle(R"({"jsonrpc":"1.0","id":1,"method":"ping"})")[0])["error"]["code"] == -32600);
    CHECK(Json::parse(rpc.handle("[]")[0])["error"]["code"] == -32600);
    // A notification gets no answer; a batch gets one array.
    CHECK(rpc.handle(R"({"jsonrpc":"2.0","method":"ping"})").empty());
    const auto batch = rpc.handle(
        R"([{"jsonrpc":"2.0","id":1,"method":"ping"},{"jsonrpc":"2.0","method":"ping"},)"
        R"({"jsonrpc":"2.0","id":"x","method":"status"}])");
    CHECK(batch.size() == 1);
    const Json answers = Json::parse(batch[0]);
    CHECK(answers.size() == 2 && answers[0]["result"] == "pong" &&
          answers[1]["id"] == "x" && answers[1]["result"]["loaded"] == false);

    // Without a system most methods say so.
    CHECK(code_of(call(rpc, "list_endpoints")) == -32001);
    CHECK(contains(message_of(call(rpc, "run")), "no system is loaded"));
    CHECK(code_of(call(rpc, "load", Json::object())) == -32602); // missing yaml

    // Loading: the answer lists MCUs, a bad description reports its line.
    const Json loaded = result_of(call(rpc, "load", {{"yaml", yaml_text()}}));
    CHECK(loaded["mcus"].size() == 1 && loaded["mcus"][0]["name"] == "d.m" &&
          loaded["mcus"][0]["model"] == "tlsr8258");
    const Json bad = call(rpc, "load", {{"yaml", "version: 1\ndevices:\n  - id: d\n    mcux: []\n"}});
    CHECK(code_of(bad) == -32000 && contains(message_of(bad), "line 4"));
    // A failed load keeps the previous system.
    CHECK(result_of(call(rpc, "status"))["loaded"] == true);

    const Json status = result_of(call(rpc, "status"));
    CHECK(status["running"] == false && status["time_ns"] == 0);
    CHECK(status["mcus"][0]["state"] == "running");

    Json names = Json::array();
    for (const Json &entry : result_of(call(rpc, "list_endpoints")))
        names.push_back(entry["name"]);
    CHECK((names == Json::array({"d.btn.pressed", "d.line", "d.m.uart0"})));

    // get / set, with values of every type.
    CHECK(result_of(call(rpc, "get", {{"endpoint", "d.btn.pressed"}})) == false);
    CHECK(result_of(call(rpc, "set", {{"endpoint", "d.btn.pressed"}, {"value", true}}))["scheduled"] == false);
    CHECK(result_of(call(rpc, "get", {{"endpoint", "d.btn.pressed"}})) == true);
    CHECK(result_of(call(rpc, "get", {{"endpoint", "d.line"}})) == false); // pressed: line low
    CHECK(code_of(call(rpc, "get", {{"endpoint", "nope"}})) == -32001);
    CHECK(code_of(call(rpc, "get", Json::object())) == -32602);
    CHECK(code_of(call(rpc, "set", {{"endpoint", "d.line"}, {"value", true}})) == -32001); // read-only
    CHECK(code_of(call(rpc, "set", {{"endpoint", "d.btn.pressed"}, {"value", Json::array()}})) == -32602);

    // drive forces a net from outside.
    CHECK(result_of(call(rpc, "set", {{"endpoint", "d.btn.pressed"}, {"value", false}}))["scheduled"] == false);
    call(rpc, "drive", {{"net", "d.line"}, {"state", "low"}});
    CHECK(result_of(call(rpc, "get", {{"endpoint", "d.line"}})) == false);
    call(rpc, "drive", {{"net", "d.line"}, {"state", "hiz"}});
    CHECK(result_of(call(rpc, "get", {{"endpoint", "d.line"}})) == true);
    CHECK(code_of(call(rpc, "drive", {{"net", "d.line"}, {"state", "sideways"}})) == -32602);
    CHECK(code_of(call(rpc, "drive", {{"net", "nope"}, {"state", "low"}})) == -32001);

    CHECK(result_of(call(rpc, "unload")) == true);
    CHECK(result_of(call(rpc, "status"))["loaded"] == false);
    // Two sessions do not share state.
    sim::RpcSession other(root_dir().string());
    call(other, "load", {{"yaml", yaml_text()}});
    call(rpc, "load", {{"yaml", yaml_text()}});
    call(other, "set", {{"endpoint", "d.btn.pressed"}, {"value", true}});
    CHECK(result_of(call(rpc, "get", {{"endpoint", "d.btn.pressed"}})) == false);
}

void test_rpc_run_events_and_memory() {
    sim::RpcSession rpc(root_dir().string());
    call(rpc, "load", {{"yaml", yaml_text()}});

    // Time control: stepping is synchronous and pauses.
    CHECK(result_of(call(rpc, "step", {{"duration", "250us"}}))["time_ns"] == 250000);
    CHECK(result_of(call(rpc, "step"))["time_ns"] == 1250000); // default 1 ms
    CHECK(code_of(call(rpc, "step", {{"duration", "5s"}})) == -32602);

    // Subscribe to the UART, make the chip transmit through its registers
    // (UART control 0x800096 = 0x4f, data 0x800090), and run.
    const auto sub = result_of(call(rpc, "subscribe", {{"endpoint", "d.m.uart0"}}))["subscription"];
    CHECK(code_of(call(rpc, "subscribe", {{"endpoint", "nope"}})) == -32001);
    call(rpc, "mem.write", {{"mcu", "d.m"}, {"address", 0x800096}, {"hex", "4f"}});
    call(rpc, "mem.write", {{"mcu", "d.m"}, {"address", 0x800090}, {"hex", "5a"}});
    CHECK(result_of(call(rpc, "mem.read", {{"mcu", "d.m"}, {"address", 0x800096}, {"length", 1}}))["hex"] == "4f");

    // A free run is answered at once; pumping produces the events.
    CHECK(result_of(call(rpc, "run"))["running"] == true);
    std::vector<std::string> notes;
    for (int i = 0; i < 5; ++i)
        for (const auto &n : rpc.pump(std::chrono::milliseconds(2)))
            notes.push_back(n);
    bool saw_event = false;
    for (const std::string &note : notes) {
        const Json json = Json::parse(note);
        if (json.value("method", "") == "event" && json["params"]["subscription"] == sub) {
            saw_event = true;
            CHECK(json["params"]["endpoint"] == "d.m.uart0");
            CHECK(json["params"]["value"] == Json({{"hex", "5a"}}));
        }
    }
    CHECK(saw_event);
    CHECK(rpc.running());
    call(rpc, "pause");
    CHECK(!rpc.running());

    // run with `for` and `wait`: the response is deferred until the target.
    const std::uint64_t before = result_of(call(rpc, "status"))["time_ns"].get<std::uint64_t>();
    const auto immediate = rpc.handle(
        Json{{"jsonrpc", "2.0"}, {"id", 7}, {"method", "run"},
             {"params", {{"for", "5ms"}, {"wait", true}}}}.dump());
    CHECK(immediate.empty());
    std::vector<Json> after;
    for (int i = 0; i < 50 && rpc.running(); ++i)
        for (const auto &n : rpc.pump(std::chrono::milliseconds(5)))
            after.push_back(Json::parse(n));
    CHECK(!rpc.running());
    bool answered = false, stopped = false;
    for (const Json &json : after) {
        if (json.contains("id") && json["id"] == 7) {
            answered = true;
            CHECK(json["result"]["reason"] == "target");
            CHECK(json["result"]["time_ns"] == before + 5000000);
        }
        if (json.value("method", "") == "stopped")
            stopped = true;
    }
    CHECK(answered && stopped);
    CHECK(code_of(call(rpc, "run", {{"wait", true}})) == -32602);
    CHECK(code_of(call(rpc, "run", {{"for", "1ms"}, {"until", "2ms"}})) == -32602);

    // `until` in the past does nothing.
    CHECK(result_of(call(rpc, "run", {{"until", 1000}}))["running"] == false);

    // A scheduled set and a scheduled drive.
    const std::uint64_t now = result_of(call(rpc, "status"))["time_ns"].get<std::uint64_t>();
    call(rpc, "set", {{"endpoint", "d.btn.pressed"}, {"value", true}, {"at", now + 100000}});
    call(rpc, "drive", {{"net", "d.line"}, {"state", "low"}, {"at", now + 300000}});
    CHECK(result_of(call(rpc, "get", {{"endpoint", "d.btn.pressed"}})) == false);
    call(rpc, "step", {{"duration", "200us"}});
    CHECK(result_of(call(rpc, "get", {{"endpoint", "d.btn.pressed"}})) == true);
    call(rpc, "unsubscribe", {{"subscription", sub}});
    CHECK(code_of(call(rpc, "unsubscribe", {{"subscription", sub}})) == -32001);

    // Target memory: SRAM round trip, unmapped addresses and unknown MCUs.
    call(rpc, "mem.write", {{"mcu", "d.m"}, {"address", 0x840010}, {"hex", "deadbeef"}});
    CHECK(result_of(call(rpc, "mem.read", {{"mcu", "d.m"}, {"address", 0x840010}, {"length", 4}}))["hex"] == "deadbeef");
    CHECK(code_of(call(rpc, "mem.read", {{"mcu", "d.m"}, {"address", 0x7000000}, {"length", 4}})) == -32001);
    CHECK(code_of(call(rpc, "mem.read", {{"mcu", "x.y"}, {"address", 0}, {"length", 1}})) == -32001);
    CHECK(code_of(call(rpc, "mem.write", {{"mcu", "d.m"}, {"address", 0x840010}, {"hex", "abc"}})) == -32602);
    CHECK(code_of(call(rpc, "mem.read", {{"mcu", "d.m"}, {"address", 0}, {"length", 99999999}})) == -32001);
}

void test_rpc_sandbox() {
    sim::RpcSession rpc(root_dir().string());
    auto yaml_with = [](const std::string &firmware, const std::string &extra = "") {
        return "version: 1\ndevices:\n  - id: d\n    mcus:\n      - {id: m, model: tlsr8258, firmware: " +
               firmware + "}\n" + extra;
    };
    // Firmware and scripts must live under the root.
    CHECK(code_of(call(rpc, "load", {{"yaml", yaml_with("/etc/passwd")}})) == -32001);
    CHECK(contains(message_of(call(rpc, "load", {{"yaml", yaml_with("../idle.bin")}})), "outside"));
    CHECK(code_of(call(rpc, "load", {{"yaml", yaml_with("idle.bin")}, {"base_dir", "../"}})) == -32001);
    CHECK(code_of(call(rpc, "load", {{"yaml", yaml_with("idle.bin")}, {"base_dir", "sub"}})) == -32000); // sub/idle.bin is missing
    CHECK(code_of(call(rpc, "load", {{"yaml", yaml_with("../idle.bin")}, {"base_dir", "sub"}})) == 0); // inside the root

    // A symlink inside the root that points outside is refused too.
    std::error_code ignored;
    fs::create_symlink("/tmp", root_dir() / "escape", ignored);
    CHECK(code_of(call(rpc, "load", {{"yaml", yaml_with("escape/x.bin")}})) == -32001);
    CHECK(code_of(call(rpc, "load", {{"yaml", yaml_with("idle.bin",
                                                        "attach:\n  - {endpoint: d.m.uart0, sink: \"file:escape/x.log\"}\n")}})) == -32001);
    CHECK(code_of(call(rpc, "load", {{"yaml", yaml_with("idle.bin",
                                                        "attach:\n  - {endpoint: d.m.uart0, sink: stdout}\n")}})) == -32001);

    // Runtime attach: files stay in the root, stdout is refused, tcp only on loopback.
    call(rpc, "load", {{"yaml", yaml_with("idle.bin")}});
    const Json ok = call(rpc, "attach", {{"endpoint", "d.m.uart0"}, {"sink", "file:sub/uart.log"}});
    CHECK(code_of(ok) == 0 && fs::exists(root_dir() / "sub" / "uart.log"));
    CHECK(code_of(call(rpc, "attach", {{"endpoint", "d.m.uart0"}, {"sink", "file:../out.log"}})) == -32001);
    CHECK(code_of(call(rpc, "attach", {{"endpoint", "d.m.uart0"}, {"sink", "file:/tmp/out.log"}})) == -32001);
    CHECK(code_of(call(rpc, "attach", {{"endpoint", "d.m.uart0"}, {"sink", "stdout"}})) == -32001);
    CHECK(code_of(call(rpc, "attach", {{"endpoint", "d.m.uart0"}, {"sink", "tcp:0.0.0.0:9000"}})) == -32001);
    CHECK(code_of(call(rpc, "attach", {{"endpoint", "d.m.uart0"}, {"sink", "tcp:80"}})) == -32001);
    CHECK(contains(message_of(call(rpc, "attach", {{"endpoint", "d.m.uart0"}, {"sink", "smoke"}})), "unknown sink"));
    CHECK(code_of(call(rpc, "attach", {{"endpoint", "nope"}, {"sink", "file:sub/x"}})) == -32001);
    CHECK(code_of(call(rpc, "detach", {{"attachment", result_of(ok)["attachment"]}})) == 0);
    CHECK(code_of(call(rpc, "detach", {{"attachment", 999}})) == -32001);

    // A description larger than 1 MiB is refused up front.
    CHECK(code_of(call(rpc, "load", {{"yaml", std::string(1100000, '#')}})) == -32001);
    // The root itself must exist.
    bool threw = false;
    try { sim::RpcSession bad("/definitely/not/here"); } catch (const sim::SimulationError &) { threw = true; }
    CHECK(threw);
}

namespace {

// A WebSocket test client over a real loopback socket. The server runs in the
// same thread: every wait pumps Server::run_once().
struct WsClient {
    int fd{-1};
    std::string buffer;

    explicit WsClient(unsigned short port) {
        fd = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        CHECK(::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0);
    }
    ~WsClient() { if (fd >= 0) ::close(fd); }

    void send_raw(const std::string &bytes) { CHECK(::send(fd, bytes.data(), bytes.size(), 0) == static_cast<ssize_t>(bytes.size())); }

    std::string pump_until(sim::Server &server, const std::function<bool(const std::string &)> &done,
                           int rounds = 200) {
        for (int i = 0; i < rounds; ++i) {
            server.run_once(5);
            pollfd p{fd, POLLIN, 0};
            while (::poll(&p, 1, 0) == 1 && (p.revents & POLLIN)) {
                char chunk[65536];
                const ssize_t n = ::recv(fd, chunk, sizeof chunk, 0);
                if (n <= 0)
                    return buffer;
                buffer.append(chunk, static_cast<std::size_t>(n));
            }
            if (done(buffer))
                break;
        }
        return buffer;
    }

    // Opening handshake; returns the HTTP response head.
    std::string handshake(sim::Server &server, const std::string &path = "/",
                          const std::string &extra_headers = "") {
        send_raw("GET " + path + " HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                 "Connection: Upgrade\r\nSec-WebSocket-Version: 13\r\n"
                 "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n" + extra_headers + "\r\n");
        pump_until(server, [](const std::string &b) { return b.find("\r\n\r\n") != std::string::npos; });
        const std::size_t end = buffer.find("\r\n\r\n");
        std::string head = end == std::string::npos ? buffer : buffer.substr(0, end + 4);
        buffer.erase(0, head.size());
        return head;
    }

    static std::string masked_frame(sim::ws::Opcode opcode, const std::string &payload) {
        std::string frame;
        frame.push_back(static_cast<char>(0x80 | static_cast<std::uint8_t>(opcode)));
        if (payload.size() < 126) {
            frame.push_back(static_cast<char>(0x80 | payload.size()));
        } else {
            frame.push_back(static_cast<char>(0x80 | 126));
            frame.push_back(static_cast<char>(payload.size() >> 8));
            frame.push_back(static_cast<char>(payload.size()));
        }
        const char key[4] = {9, 8, 7, 6};
        frame.append(key, 4);
        for (std::size_t i = 0; i < payload.size(); ++i)
            frame.push_back(static_cast<char>(payload[i] ^ key[i % 4]));
        return frame;
    }

    void send_text(const std::string &text) { send_raw(masked_frame(sim::ws::Opcode::Text, text)); }

    // Decodes the server frames currently buffered.
    std::vector<std::pair<int, std::string>> frames() {
        std::vector<std::pair<int, std::string>> out;
        std::size_t at = 0;
        while (buffer.size() - at >= 2) {
            const int opcode = static_cast<std::uint8_t>(buffer[at]) & 0x0f;
            std::uint64_t length = static_cast<std::uint8_t>(buffer[at + 1]) & 0x7f;
            std::size_t header = 2;
            if (length == 126) {
                if (buffer.size() - at < 4) break;
                length = (static_cast<std::uint8_t>(buffer[at + 2]) << 8) | static_cast<std::uint8_t>(buffer[at + 3]);
                header = 4;
            } else if (length == 127) {
                if (buffer.size() - at < 10) break;
                length = 0;
                for (int i = 0; i < 8; ++i)
                    length = (length << 8) | static_cast<std::uint8_t>(buffer[at + 2 + i]);
                header = 10;
            }
            if (buffer.size() - at < header + length) break;
            out.emplace_back(opcode, buffer.substr(at + header, static_cast<std::size_t>(length)));
            at += header + static_cast<std::size_t>(length);
        }
        buffer.erase(0, at);
        return out;
    }

    // Sends a request and waits for its response; other messages are kept.
    Json request(sim::Server &server, const std::string &method, Json params, int id,
                 std::vector<Json> *others = nullptr) {
        send_text(Json{{"jsonrpc", "2.0"}, {"id", id}, {"method", method}, {"params", params}}.dump());
        Json response;
        for (int i = 0; i < 400 && response.is_null(); ++i) {
            server.run_once(5);
            pollfd p{fd, POLLIN, 0};
            while (::poll(&p, 1, 0) == 1 && (p.revents & POLLIN)) {
                char chunk[65536];
                const ssize_t n = ::recv(fd, chunk, sizeof chunk, 0);
                if (n <= 0) break;
                buffer.append(chunk, static_cast<std::size_t>(n));
            }
            for (auto &frame : frames()) {
                if (frame.first != 1) continue;
                const Json json = Json::parse(frame.second);
                if (json.contains("id") && json["id"] == id)
                    response = json;
                else if (others)
                    others->push_back(json);
            }
        }
        return response;
    }
};

} // namespace

void test_server_over_websocket() {
    sim::ServerOptions options;
    options.root = root_dir().string();
    options.max_sessions = 2;
    sim::Server server(options);
    CHECK(server.port() != 0);

    {
        // A plain request, a health check and a normal handshake.
        WsClient web(server.port());
        web.send_raw("GET / HTTP/1.1\r\nHost: x\r\n\r\n");
        const std::string answer = web.pump_until(server, [](const std::string &b) { return b.size() > 20; });
        CHECK(contains(answer, "426 Upgrade Required"));
        WsClient health(server.port());
        health.send_raw("GET /health HTTP/1.1\r\nHost: x\r\n\r\n");
        CHECK(contains(health.pump_until(server, [](const std::string &b) { return b.find("ok\n") != std::string::npos; }), "200 OK"));
        WsClient garbage(server.port());
        garbage.send_raw("???\r\n\r\n");
        CHECK(contains(garbage.pump_until(server, [](const std::string &b) { return b.size() > 20; }), "400"));
    }

    WsClient client(server.port());
    const std::string head = client.handshake(server);
    CHECK(contains(head, "101 Switching Protocols"));
    CHECK(contains(head, "Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo="));

    // Requests over the socket, including a message longer than 125 bytes.
    CHECK(client.request(server, "ping", Json::object(), 1)["result"] == "pong");
    const Json loaded = client.request(server, "load", {{"yaml", yaml_text()}}, 2);
    CHECK(loaded["result"]["mcus"][0]["name"] == "d.m");
    CHECK(client.request(server, "status", Json::object(), 3)["result"]["loaded"] == true);

    // Ping gets a pong; binary data is refused with close code 1003.
    client.send_raw(WsClient::masked_frame(sim::ws::Opcode::Ping, "hi"));
    client.pump_until(server, [&](const std::string &b) { return b.size() >= 4; }, 20);
    auto pong = client.frames();
    CHECK(pong.size() == 1 && pong[0].first == 0xa && pong[0].second == "hi");

    // A running simulation streams events while the server loop spins.
    client.request(server, "subscribe", {{"endpoint", "d.m.uart0"}}, 4);
    client.request(server, "mem.write", {{"mcu", "d.m"}, {"address", 0x800096}, {"hex", "4f"}}, 5);
    client.request(server, "mem.write", {{"mcu", "d.m"}, {"address", 0x800090}, {"hex", "41"}}, 6);
    std::vector<Json> others;
    const Json run = client.request(server, "run", {{"for", "3ms"}, {"wait", true}}, 7, &others);
    CHECK(run["result"]["reason"] == "target" && run["result"]["time_ns"] == 3000000);
    bool event = false;
    for (const Json &note : others)
        event |= note.value("method", "") == "event" && note["params"]["value"] == Json({{"hex", "41"}});
    CHECK(event);

    // Isolation: a second connection is a separate simulation.
    WsClient second(server.port());
    CHECK(contains(second.handshake(server), "101"));
    CHECK(second.request(server, "status", Json::object(), 1)["result"]["loaded"] == false);
    // The session limit applies (2 sessions are open).
    WsClient third(server.port());
    CHECK(contains(third.handshake(server), "503"));

    // Protocol errors close the connection with the right code.
    second.send_raw(std::string("\x81\x02hi", 4)); // unmasked
    second.pump_until(server, [](const std::string &b) { return b.size() >= 4; }, 40);
    auto closed = second.frames();
    CHECK(!closed.empty() && closed[0].first == 8 && closed[0].second.size() >= 2 &&
          ((static_cast<std::uint8_t>(closed[0].second[0]) << 8) | static_cast<std::uint8_t>(closed[0].second[1])) == 1002);
    client.send_raw(WsClient::masked_frame(sim::ws::Opcode::Binary, "x"));
    client.pump_until(server, [](const std::string &b) { return !b.empty(); }, 40);
    auto refused = client.frames();
    CHECK(!refused.empty() && refused.back().first == 8 &&
          ((static_cast<std::uint8_t>(refused.back().second[0]) << 8) | static_cast<std::uint8_t>(refused.back().second[1])) == 1003);

    // shutdown closes politely.
    WsClient last(server.port());
    last.handshake(server);
    last.request(server, "shutdown", Json::object(), 1);
    last.pump_until(server, [](const std::string &b) { return b.find("\x88") != std::string::npos; }, 40);
    server.run_once(5);
}

void test_rpc_realtime() {
    sim::RpcSession rpc(root_dir().string());
    call(rpc, "load", {{"yaml", yaml_text()}});
    CHECK(result_of(call(rpc, "status"))["speed"] == 0);
    CHECK(code_of(call(rpc, "set_speed", {{"speed", -1}})) == -32602);
    CHECK(code_of(call(rpc, "set_speed", {{"speed", 5000}})) == -32602);
    CHECK(code_of(call(rpc, "set_speed", {{"speed", "fast"}})) == -32602);
    CHECK(code_of(call(rpc, "set_speed")) == -32602);

    // Paced at 0.02x (the engine manages ~0.08x on a busy loop, so this is
    // reachable): 150 ms of wall time allow ~3 ms of virtual time.
    CHECK(result_of(call(rpc, "set_speed", {{"speed", 0.02}}))["speed"] == 0.02);
    call(rpc, "run");
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(150)) {
        rpc.pump(std::chrono::milliseconds(5));
        if (rpc.throttled())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const std::uint64_t paced = result_of(call(rpc, "status"))["time_ns"].get<std::uint64_t>();
    CHECK(paced > 1000000ull && paced <= 3600000ull);
    bool was_throttled = false; // caught up: the server may sleep instead of spinning
    for (int i = 0; i < 20 && !was_throttled; ++i) {
        rpc.pump(std::chrono::milliseconds(5));
        was_throttled = rpc.throttled();
    }
    CHECK(was_throttled);
    call(rpc, "pause");

    // 'max' removes the pacing: never throttled, and it advances.
    call(rpc, "run", {{"speed", "max"}});
    rpc.pump(std::chrono::milliseconds(20));
    CHECK(!rpc.throttled());
    CHECK(result_of(call(rpc, "status"))["time_ns"].get<std::uint64_t>() > paced);
    call(rpc, "pause");

    // A paced run with a target ends exactly at the target, and not early.
    call(rpc, "set_speed", {{"speed", 0.05}});
    const std::uint64_t now = result_of(call(rpc, "status"))["time_ns"].get<std::uint64_t>();
    call(rpc, "run", {{"for", "2ms"}});
    const auto target_start = std::chrono::steady_clock::now();
    bool stopped = false;
    for (int i = 0; i < 5000 && !stopped; ++i) {
        for (const std::string &message : rpc.pump(std::chrono::milliseconds(5)))
            stopped |= Json::parse(message).value("method", "") == "stopped";
        if (!stopped)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(stopped);
    CHECK(std::chrono::steady_clock::now() - target_start >= std::chrono::milliseconds(30));
    CHECK(result_of(call(rpc, "status"))["time_ns"] == now + 2000000ull);
}

void test_server_security_options() {
    // A token and the origin policy.
    sim::ServerOptions options;
    options.root = root_dir().string();
    options.token = "s3cret";
    options.allowed_origins = {"http://ui.local"};
    sim::Server server(options);
    {
        WsClient anonymous(server.port());
        CHECK(contains(anonymous.handshake(server), "401"));
    }
    {
        WsClient wrong(server.port());
        CHECK(contains(wrong.handshake(server, "/?token=nope"), "401"));
    }
    {
        WsClient query(server.port());
        CHECK(contains(query.handshake(server, "/?x=1&token=s3cret"), "101"));
    }
    {
        WsClient header(server.port());
        CHECK(contains(header.handshake(server, "/", "Authorization: Bearer s3cret\r\n"), "101"));
    }
    {
        // Browsers send Origin: only listed ones pass, even with the token.
        WsClient evil(server.port());
        CHECK(contains(evil.handshake(server, "/?token=s3cret", "Origin: http://evil.example\r\n"), "403"));
        WsClient good(server.port());
        CHECK(contains(good.handshake(server, "/?token=s3cret", "Origin: http://ui.local\r\n"), "101"));
    }
    // Exposing the server beyond loopback needs a token; the root must exist.
    sim::ServerOptions open;
    open.root = root_dir().string();
    open.bind = "0.0.0.0";
    bool threw = false;
    try { sim::Server exposed(open); } catch (const std::invalid_argument &) { threw = true; }
    CHECK(threw);
    open.bind = "127.0.0.1";
    open.root = "/definitely/not/here";
    threw = false;
    try { sim::Server nowhere(open); } catch (const sim::SimulationError &) { threw = true; }
    CHECK(threw);
}

} // namespace tc32emu_tests
