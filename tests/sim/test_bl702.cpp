// The BL702 adapter in a simulated system: the boot header of a flash image, UART0 out to the
// system, and the model's fixed-clock and variant rules.

#include "../test_support.hpp"

#include "sim/bl702_mcu.hpp"
#include "sim/config.hpp"
#include "sim/endpoint.hpp"
#include "sim/system.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <variant>

namespace mcuemu_tests {

namespace {

void put32(std::vector<std::uint8_t> &bytes, std::size_t offset, std::uint32_t value) {
    if (bytes.size() < offset + 4)
        bytes.resize(offset + 4, 0);
    std::memcpy(&bytes[offset], &value, 4);
}

// A flash image with no segments that starts at its code (flash offset 0x100), written to a temporary file.
std::string program_image(const std::string &name, const std::vector<std::uint32_t> &code) {
    std::vector<std::uint8_t> bytes(176, 0);
    put32(bytes, 0, 0x504e4642);   // "BFNP"
    put32(bytes, 116, 1u << 8);    // no segments: the code runs from flash
    put32(bytes, 124, 0x23000100); // entry
    bytes.resize(0x100, 0);
    for (std::uint32_t word : code)
        put32(bytes, bytes.size(), word);
    const auto file = std::filesystem::temp_directory_path() / name;
    std::ofstream(file, std::ios::binary).write(reinterpret_cast<const char *>(bytes.data()),
                                                static_cast<std::streamsize>(bytes.size()));
    return file.string();
}

// Writes "OK" to UART0, then waits: li a0,0x4000a088; li a1,'O'; sw a1,0(a0); li a1,'K'; sw a1,0(a0); wfi; j -4.
std::string uart_image_path() {
    static const std::string path = program_image(
        "simcore-bl702-uart.bin",
        {0x4000a537, 0x08850513, 0x04f00593, 0x00b52023, 0x04b00593, 0x00b52023, 0x10500073, 0xffdff06f});
    return path;
}

// Sends the five-byte frame 01 02 03 04 05 through the M154 registers, as the zigbee-rs driver does.
std::string radio_image_path() {
    static const std::string path = program_image(
        "simcore-bl702-radio.bin",
        {0x4c000537, 0x20050513, 0x040305b7, 0x20158593, 0x00b52023, 0x00500593, 0x00b52223, 0x4c000537,
         0x2fc50513, 0x070005b7, 0x00b52023, 0x4c000537, 0x0b450513, 0x00100593, 0x00b52023, 0x10500073,
         0xffdff06f});
    return path;
}

std::unique_ptr<sim::System> build(const std::string &mcu_extra, const std::string &image, const std::string &body = "") {
    return sim::build_system(sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: bl702, firmware: " + image + mcu_extra + "}\n" + body));
}

} // namespace

void test_bl702_runs_an_image_and_prints_on_uart() {
    const auto system = build("", uart_image_path());
    auto &mcu = static_cast<sim::Bl702Mcu &>(system->world().mcu("d.m"));
    CHECK(mcu.model() == "bl702");
    // The UART endpoint takes the bytes off the MCU and hands them to its subscribers.
    sim::World &world = system->world();
    std::string text;
    world.endpoints().at("d.m.uart0").subscribe([&text](const sim::EndpointEvent &event) {
        if (const auto *bytes = std::get_if<sim::Bytes>(&event.value))
            text.append(bytes->begin(), bytes->end());
    });
    world.run_for(1000000); // 1 ms: the program prints and then waits
    CHECK(mcu.state() == sim::McuState::Running);
    CHECK(text == "OK");
    CHECK(mcu.uart_count() == 1);
}

void test_bl702_transmits_on_the_world_radio() {
    const auto system = sim::build_system(sim::parse_system_config(
        "version: 1\nworld:\n  radio:\n    - id: air0\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: bl702, firmware: " + radio_image_path() +
        ", radio: {air: air0, position: [0, 0, 0], tx_power_dbm: 0, protocols: [zigbee]}}\n"));
    sim::World &world = system->world();
    world.run_for(1000000); // the frame is 416 us on air
    const auto &frames = world.radio("air0").frames();
    CHECK(frames.size() == 1);
    if (frames.size() == 1) {
        CHECK(frames[0].payload == std::vector<std::uint8_t>({1, 2, 3, 4, 5}));
        CHECK(frames[0].source == "d.m");
    }
}

void test_bl702_config_rules_and_missing_header() {
    auto rejected = [](auto make) {
        try {
            make();
        } catch (const std::exception &) {
            return true;
        }
        return false;
    };
    CHECK(rejected([] { build(", clock_hz: 16MHz", uart_image_path()); })); // the clock is fixed at 32 MHz
    CHECK(rejected([] { build(", variant: F1K", uart_image_path()); }));
    CHECK(rejected([] { build("", uart_image_path(), "    nets:\n      - { id: x, pins: [m.P0] }\n"); })); // no pads yet

    // A file without the 'BFNP' header does not boot: the boot ROM refuses it when the system is built.
    const auto blank = std::filesystem::temp_directory_path() / "simcore-bl702-blank.bin";
    std::ofstream(blank, std::ios::binary) << std::string(256, '\0');
    CHECK(rejected([&blank] { build("", blank.string()); }));
}

} // namespace mcuemu_tests
