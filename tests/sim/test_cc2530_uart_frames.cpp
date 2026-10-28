// The CC2530 UART frame endpoint (<uart>.frames): frame tokens, YAML events,
// CC2530 <-> CC2530 links and the JSON-RPC surface. The USART's own error
// model is covered by tests/mcs51/test_cc2530_uart_errors.cpp.
#include "../test_support.hpp"

#include "sim/cc2530f256_mcu.hpp"
#include "sim/config.hpp"
#include "sim/endpoint.hpp"
#include "sim/rpc.hpp"
#include "sim/system.hpp"
#include "sim/uart_endpoint.hpp"
#include "sim/world.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using sim::Cc2530F256Mcu;
using sim::UartFrame;
using Json = nlohmann::json;

const std::filesystem::path &scratch() {
    static const std::filesystem::path path = [] {
        auto dir = std::filesystem::current_path() / "simcore-cc2530-uart-frames";
        std::filesystem::create_directories(dir);
        return std::filesystem::weakly_canonical(dir);
    }();
    return path;
}

std::string raw_image(const std::string &name, const std::vector<int> &bytes) {
    const auto path = scratch() / name;
    std::ofstream out(path, std::ios::binary);
    for (int byte : bytes)
        out.put(static_cast<char>(byte));
    return path.generic_string();
}

bool contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

std::uint8_t xdata(sim::Mcu &mcu, std::uint16_t address) {
    std::uint8_t value = 0;
    mcu.read_memory(Cc2530F256Mcu::XdataWindow + address, &value, 1);
    return value;
}

std::vector<std::uint8_t> xdata_block(sim::Mcu &mcu, std::uint16_t address, std::size_t size) {
    std::vector<std::uint8_t> block(size);
    mcu.read_memory(Cc2530F256Mcu::XdataWindow + address, block.data(), block.size());
    return block;
}

template <typename Fn> std::string error_of(Fn &&fn) {
    try {
        fn();
    } catch (const std::exception &error) {
        return error.what();
    }
    return {};
}

// USART0 (alternative 1) at 115203 baud on the 32 MHz XOSC with U0UCR =
// `ucr`. The loop waits for URX0IF (TCON.3, polled with the interrupt
// disabled), then stores U0CSR and U0DBUF at XDATA 0x0100 onwards; reading
// U0CSR first keeps the FE/ERR it clears (SWRU191F 17.8). With `echo` the
// byte is sent back.
std::vector<int> receiver_program(int ucr, bool echo) {
    std::vector<int> code{
        0x75, 0x81, 0x60,       // 0x00 MOV SP,#0x60
        0x75, 0xc6, 0x80,       // 0x03 MOV CLKCONCMD,#0x80
        0xe5, 0x9e,             // 0x06 MOV A,CLKCONSTA
        0xb4, 0x80, 0xfb,       // 0x08 CJNE A,#0x80,0x06
        0x75, 0xf1, 0x00,       // 0x0b MOV PERCFG,#0
        0x43, 0xf3, 0x0c,       // 0x0e ORL P0SEL,#0x0c
        0x75, 0xc2, 0xd8,       // 0x11 MOV U0BAUD,#216
        0x75, 0xc5, 0x0b,       // 0x14 MOV U0GCR,#11
        0x75, 0xc4, ucr,        // 0x17 MOV U0UCR,#ucr
        0x75, 0x86, 0xc0,       // 0x1a MOV U0CSR,#0xc0 (UART, RE)
        0x90, 0x01, 0x00,       // 0x1d MOV DPTR,#0x0100
        0x30, 0x8b, 0xfd,       // 0x20 JNB TCON.3,0x20
        0xc2, 0x8b,             // 0x23 CLR TCON.3
        0xe5, 0x86,             // 0x25 MOV A,U0CSR
        0xf0,                   // 0x27 MOVX @DPTR,A
        0xa3,                   // 0x28 INC DPTR
        0xe5, 0xc1,             // 0x29 MOV A,U0DBUF
        0xf0,                   // 0x2b MOVX @DPTR,A
        0xa3,                   // 0x2c INC DPTR
    };
    if (echo) {
        code.insert(code.end(), {0xf5, 0xc1,   // 0x2d MOV U0DBUF,A
                                 0x80, 0xef}); // 0x2f SJMP 0x20
    } else {
        code.insert(code.end(), {0x80, 0xf1}); // 0x2d SJMP 0x20
    }
    return code;
}

// Same setup; sends 0x41 and 0x43. After TX_BYTE it waits 2048 DJNZ
// loops (well over a frame) so the receiver is idle (UxCSR.ACTIVE = 0) when
// it reads U0CSR for the first byte.
std::vector<int> sender_program(int ucr) {
    return {
        0x75, 0x81, 0x60,       // 0x00 MOV SP,#0x60
        0x75, 0xc6, 0x80,       // 0x03 MOV CLKCONCMD,#0x80
        0xe5, 0x9e,             // 0x06 MOV A,CLKCONSTA
        0xb4, 0x80, 0xfb,       // 0x08 CJNE A,#0x80,0x06
        0x75, 0xf1, 0x00,       // 0x0b MOV PERCFG,#0
        0x43, 0xf3, 0x0c,       // 0x0e ORL P0SEL,#0x0c
        0x75, 0xc2, 0xd8,       // 0x11 MOV U0BAUD,#216
        0x75, 0xc5, 0x0b,       // 0x14 MOV U0GCR,#11
        0x75, 0xc4, ucr,        // 0x17 MOV U0UCR,#ucr
        0x75, 0x86, 0xc0,       // 0x1a MOV U0CSR,#0xc0
        0x75, 0xc1, 0x41,       // 0x1d MOV U0DBUF,#0x41
        0xe5, 0x86,             // 0x20 MOV A,U0CSR
        0x30, 0xe1, 0xfb,       // 0x22 JNB ACC.1,0x20 (TX_BYTE)
        0x53, 0x86, 0xfd,       // 0x25 ANL U0CSR,#0xfd
        0x7e, 0x08,             // 0x28 MOV R6,#8
        0xdf, 0xfe,             // 0x2a DJNZ R7,0x2a
        0xde, 0xfc,             // 0x2c DJNZ R6,0x2a
        0x75, 0xc1, 0x43,       // 0x2e MOV U0DBUF,#0x43
        0x80, 0xfe,             // 0x31 SJMP $
    };
}

std::string cc2530_device(const std::string &id, const std::string &image) {
    return "  - id: " + id + "\n    mcus:\n      - {id: m, model: cc2530f256, firmware: \"" +
           image + "\"}\n";
}

} // namespace

namespace mcuemu_tests {

void test_cc2530_uart_frame_tokens() {
    using Ninth = UartFrame::Ninth;
    const auto frames = sim::parse_uart_frames(
        " 0x41,66 0103:9=1  0x00:9=0:fe 255:perr:fe:fe2, break 7:fe2\n");
    CHECK(frames.size() == 7);
    CHECK((frames[0] == UartFrame{.data = 0x41}));
    CHECK((frames[1] == UartFrame{.data = 66}));
    CHECK((frames[2] == UartFrame{.data = 0x43, .ninth = Ninth::One}));
    CHECK((frames[3] == UartFrame{.data = 0x00, .ninth = Ninth::Zero, .stop1_error = true}));
    CHECK((frames[4] == UartFrame{.data = 0xff, .ninth = Ninth::BadParity,
                                  .stop1_error = true, .stop2_error = true}));
    CHECK((frames[5] == UartFrame{.line_break = true}));
    CHECK((frames[6] == UartFrame{.data = 7, .stop2_error = true}));
    CHECK(frames[0].plain() && frames[1].plain() && !frames[2].plain() && !frames[5].plain());
    const std::string text = sim::format_uart_frames(frames);
    CHECK(text == "0x41 0x42 0x43:9=1 0x00:9=0:fe 0xff:perr:fe:fe2 break 0x07:fe2");
    CHECK(sim::parse_uart_frames(text) == frames);
    CHECK(sim::parse_uart_frames("").empty() && sim::parse_uart_frames(" ,\t").empty());
    CHECK(sim::format_uart_frames({}).empty());

    const auto error = [](const std::string &tokens) {
        return error_of([&] { sim::parse_uart_frames(tokens); });
    };
    CHECK(contains(error("x"), "UART frame 'x': expected 'break' or a byte"));
    CHECK(contains(error("-1"), "expected 'break' or a byte"));
    CHECK(contains(error(":fe"), "expected 'break' or a byte"));
    CHECK(contains(error("256"), "UART frame '256': the byte must be 0..255"));
    CHECK(contains(error("0x41z"), "the byte must be 0..255"));
    CHECK(contains(error("0x41 0x41:9=0:perr"), "UART frame '0x41:9=0:perr': more than one 9th-bit flag"));
    CHECK(contains(error("0x41:fe:fe"), "flag 'fe' given twice"));
    CHECK(contains(error("0x41:bogus"), "unknown flag 'bogus'"));
    CHECK(contains(error("0x41:"), "unknown flag ''"));
    CHECK(contains(error("break:fe"), "expected 'break' or a byte"));
}

// YAML events inject errored frames; the firmware records U0CSR and U0DBUF
// and echoes. U0UCR = 0x3a: 9 bits, even parity (D9 = 1), one high stop bit.
void test_cc2530_uart_frames_system() {
    const std::string image = raw_image("echo_even.bin", receiver_program(0x3a, true));
    const std::string yaml = "version: 1\ndevices:\n" + cc2530_device("d", image) +
                             "scenario:\n  events:\n"
                             "    - { at: 2ms, endpoint: d.m.uart0.frames, set: \"0x41\" }\n"
                             "    - { at: 2500us, endpoint: d.m.uart0.frames, set: \"0x42:perr\" }\n"
                             "    - { at: 3ms, endpoint: d.m.uart0.frames, set: \"0x43:fe\" }\n"
                             "    - { at: 3500us, endpoint: d.m.uart0.frames, set: break }\n"
                             "    - { at: 4ms, endpoint: d.m.uart0.frames, set: \"0x44:9=0\" }\n"
                             "    - { at: 4500us, endpoint: d.m.uart0.frames, set: 0x45 }\n"
                             "    - { at: 5ms, endpoint: d.m.uart0, set: \"F\" }\n";
    const auto system = sim::build_system(sim::parse_system_config(yaml));
    sim::World &world = system->world();
    sim::Endpoint *frames = world.endpoints().find("d.m.uart0.frames");
    CHECK(frames && frames->kind() == "uart-frames");
    CHECK(world.endpoints().find("d.m.uart1.frames") != nullptr);
    if (!frames)
        return;
    std::string echoed_frames;
    std::vector<std::uint8_t> echoed_bytes;
    frames->subscribe([&](const sim::EndpointEvent &event) {
        CHECK(event.endpoint == "d.m.uart0.frames");
        if (!echoed_frames.empty())
            echoed_frames += ' ';
        echoed_frames += std::get<std::string>(event.value);
    });
    world.endpoints().at("d.m.uart0").subscribe([&](const sim::EndpointEvent &event) {
        const auto &bytes = std::get<sim::Bytes>(event.value);
        echoed_bytes.insert(echoed_bytes.end(), bytes.begin(), bytes.end());
    });
    world.run_until(6000000);
    sim::Mcu &mcu = world.mcu("d.m");
    // U0CSR: MODE | RE | RX_BYTE = 0xc4, TX_BYTE (0x02) from the first echo
    // on, ERR 0x08, FE 0x10. 0x44's explicit 9th bit 0 is its even parity;
    // the break (all zeros, even parity 0) only breaks the stop bit.
    const std::vector<std::uint8_t> expected{0xc4, 0x41, 0xce, 0x42, 0xd6, 0x43, 0xd6, 0x00,
                                             0xc6, 0x44, 0xc6, 0x45, 0xc6, 'F'};
    CHECK(xdata_block(mcu, 0x0100, expected.size()) == expected);
    CHECK(xdata(mcu, 0x0100 + static_cast<std::uint16_t>(expected.size())) == 0);
    // The echoes carry the transmitter's even parity bit.
    CHECK(echoed_frames == "0x41:9=0 0x42:9=0 0x43:9=1 0x00:9=0 0x44:9=0 0x45:9=1 0x46:9=1");
    CHECK((echoed_bytes == std::vector<std::uint8_t>{0x41, 0x42, 0x43, 0x00, 0x44, 0x45, 'F'}));
    CHECK(std::get<std::int64_t>(frames->get()) == 7);

    // Bad tokens in YAML name their line; values of other types are refused.
    const std::string bad = "version: 1\ndevices:\n" + cc2530_device("d", image) +
                            "scenario:\n  events:\n"
                            "    - { at: 1ms, endpoint: d.m.uart0.frames, set: \"0x41\" }\n"
                            "    - { at: 2ms, endpoint: d.m.uart0.frames, set: \"0x41:fe:fe\" }\n";
    CHECK(contains(error_of([&] { sim::build_system(sim::parse_system_config(bad)); }),
                   "line 9: UART frame '0x41:fe:fe': flag 'fe' given twice"));
    CHECK(contains(error_of([&] { frames->set(sim::Value{std::int64_t{256}}); }), "0..255"));
    CHECK(contains(error_of([&] { frames->set(sim::Value{true}); }), "frame tokens"));
    CHECK(contains(error_of([&] { frames->set(sim::Value{std::string("0x41 nope")}); }),
                   "UART frame 'nope'"));

    // Only MCUs that model frames get the endpoint.
    {
        std::ofstream idle(scratch() / "idle.bin", std::ios::binary);
        idle.put(static_cast<char>(0xfe));
        idle.put(static_cast<char>(0x87)); // b .
    }
    const auto other = sim::build_system(sim::parse_system_config(
        "version: 1\ndevices:\n  - id: t\n    mcus:\n      - {id: m, model: tlsr8258, firmware: \"" +
        (scratch() / "idle.bin").generic_string() + "\"}\n"));
    CHECK(other->world().endpoints().find("t.m.uart0") != nullptr);
    CHECK(other->world().endpoints().find("t.m.uart0.frames") == nullptr);
}

// Two CC2530s on a link: the sender's 9th bit reaches the receiver, which
// checks it against its own format.
void test_cc2530_uart_frames_link() {
    struct Case {
        int sender_ucr;
        int receiver_ucr;
        std::vector<std::uint8_t> stored; // U0CSR, U0DBUF per frame
    };
    const Case cases[] = {
        // Both 8 bits: plain frames, as without the frame endpoints.
        {0x02, 0x02, {0xc4, 0x41, 0xc4, 0x43}},
        // Both even parity: well formed.
        {0x3a, 0x3a, {0xc4, 0x41, 0xc4, 0x43}},
        // Even sender, odd receiver (D9 = 0): both parity bits are wrong.
        {0x3a, 0x1a, {0xcc, 0x41, 0xcc, 0x43}},
        // A 9-bit sender into an 8-bit receiver: the 9th bit is sampled as
        // the stop bit, so 0x41 (parity 0) has a framing error.
        {0x3a, 0x02, {0xd4, 0x41, 0xc4, 0x43}},
        // A 9-bit receiver without parity (PARITY = 0): the 9th bit is
        // data, never an error.
        {0x3a, 0x32, {0xc4, 0x41, 0xc4, 0x43}},
        // An 8-bit sender's frames carry no 9th bit: plain, so well formed
        // at a parity receiver (model-defined, docs/system-config.md).
        {0x02, 0x1a, {0xc4, 0x41, 0xc4, 0x43}},
    };
    for (const Case &c : cases) {
        const std::string tag = std::to_string(c.sender_ucr) + "_" + std::to_string(c.receiver_ucr);
        const std::string yaml =
            "version: 1\ndevices:\n" +
            cc2530_device("tx", raw_image("tx_" + tag + ".bin", sender_program(c.sender_ucr))) +
            cc2530_device("rx", raw_image("rx_" + tag + ".bin",
                                          receiver_program(c.receiver_ucr, false))) +
            "links:\n  - {type: uart, a: tx.m.uart0, b: rx.m.uart0, latency: 5us}\n";
        const auto system = sim::build_system(sim::parse_system_config(yaml));
        system->world().run_until(3000000);
        sim::Mcu &rx = system->world().mcu("rx.m");
        CHECK(xdata_block(rx, 0x0100, c.stored.size()) == c.stored);
        CHECK(xdata(rx, 0x0104) == 0);
    }
}

// JSON-RPC: a string of tokens through `set`, frame events through
// `subscribe`, and token errors as -32001. U0UCR = 0x1a: 9 bits, odd parity.
void test_cc2530_uart_frames_rpc() {
    const std::string image = raw_image("echo_odd.bin", receiver_program(0x1a, true));
    sim::RpcSession rpc(scratch().string());
    std::string events; // values of the frame events, space separated
    auto call = [&rpc, &events](const std::string &method, Json params) {
        Json response;
        for (const std::string &message :
             rpc.handle(Json{{"jsonrpc", "2.0"}, {"id", 1}, {"method", method},
                             {"params", params}}.dump())) {
            const Json json = Json::parse(message);
            if (json.contains("id") && json["id"] == 1)
                response = json;
            else if (json.value("method", "") == "event" &&
                     json["params"]["endpoint"] == "d.m.uart0.frames")
                events += (events.empty() ? "" : " ") + json["params"]["value"].get<std::string>();
        }
        return response;
    };
    CHECK(call("load", {{"yaml", "version: 1\ndevices:\n" + cc2530_device("d", image)}})
              .contains("result"));
    const Json listed = call("list_endpoints", Json::object());
    bool listed_frames = false;
    if (listed.contains("result"))
        for (const Json &entry : listed["result"])
            listed_frames |= entry["name"] == "d.m.uart0.frames" && entry["kind"] == "uart-frames";
    CHECK(listed_frames);
    CHECK(call("subscribe", {{"endpoint", "d.m.uart0.frames"}}).contains("result"));
    call("step", {{"duration", "2ms"}});
    // 0x41 has two ones: its odd parity bit is 1. 0x52 has three: 0, so
    // "perr" sends 1.
    CHECK(call("set", {{"endpoint", "d.m.uart0.frames"}, {"value", "0x41:9=1"}}).contains("result"));
    call("step", {{"duration", "500us"}});
    CHECK(events == "0x41:9=1");
    CHECK(call("set", {{"endpoint", "d.m.uart0.frames"}, {"value", "0x52:perr:fe"}}).contains("result"));
    const Json bad = call("set", {{"endpoint", "d.m.uart0.frames"}, {"value", "0x41:9=2"}});
    CHECK(bad.contains("error") && bad["error"]["code"] == -32001 &&
          contains(bad["error"]["message"].get<std::string>(), "unknown flag '9=2'"));
    call("step", {{"duration", "500us"}});
    CHECK(events == "0x41:9=1 0x52:9=0");
    // U0CSR 0xc4 for 0x41; 0xde = 0xc4 | FE | ERR | TX_BYTE (first echo).
    const Json memory = call("mem.read", {{"mcu", "d.m"},
                                          {"address", Cc2530F256Mcu::XdataWindow + 0x0100},
                                          {"length", 5}});
    CHECK(memory.contains("result") && memory["result"]["hex"] == "c441de5200");
}

} // namespace mcuemu_tests
