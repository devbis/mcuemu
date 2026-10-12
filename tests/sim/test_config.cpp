#include "../test_support.hpp"

#include "sim/config.hpp"
#include "sim/system.hpp"
#include "sim/tlsr8258_mcu.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace tc32emu_tests {

namespace {

// Writes an idle-loop TC32 image next to the test binary's temp dir.
std::string idle_image() {
    static const std::string path = [] {
        const auto file =
            std::filesystem::temp_directory_path() / "simcore-test-idle.bin";
        std::ofstream out(file, std::ios::binary);
        out.put(static_cast<char>(0xfe));
        out.put(static_cast<char>(0x87)); // b .
        return file.string();
    }();
    return path;
}

std::string with_image(std::string yaml) {
    for (std::size_t at; (at = yaml.find("IMG")) != std::string::npos;)
        yaml.replace(at, 3, idle_image());
    return yaml;
}

template <typename Fn> std::string error_of(Fn &&fn) {
    try {
        fn();
    } catch (const sim::ConfigError &error) {
        return error.what();
    }
    return {};
}

bool contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

const char *two_mcu_yaml = R"(version: 1
world: { duration: 1ms, quantum: 2us }
devices:
  - id: node
    mcus:
      - { id: main, model: tlsr8258, variant: F1K, firmware: IMG, clock_hz: 32MHz }
      - { id: aux, model: tlsr8258, firmware: IMG, start_at: 100us }
    nets:
      - { id: line, kind: open_drain, pull: up, pins: [main.PA0, aux.PB1] }
    links:
      - { type: uart, a: main.uart0, b: aux.uart0, latency: 10us }
scenario:
  events:
    - { at: 200us, net: node.line, drive: low }
)";

} // namespace

void test_config_parsing() {
    const sim::SystemConfig config = sim::parse_system_config(
        with_image(two_mcu_yaml), "/base");
    CHECK(config.duration_ns == 1000000u);
    CHECK(config.quantum_ns == 2000u);
    CHECK(config.devices.size() == 1);
    const sim::DeviceConfig &device = config.devices[0];
    CHECK(device.mcus.size() == 2);
    CHECK(device.mcus[0].variant == "F1K");
    CHECK(device.mcus[0].clock_hz == 32000000u);
    CHECK(device.mcus[1].start_at_ns == 100000u);
    CHECK(device.nets.size() == 1);
    CHECK(device.nets[0].kind == sim::NetKind::OpenDrain);
    CHECK(device.nets[0].pull == sim::Drive::PullUp);
    CHECK(device.nets[0].pins.size() == 2);
    CHECK(device.links.size() == 1 && device.links[0].latency_ns == 10000u);
    CHECK(config.events.size() == 1 && config.events[0].drive == sim::Drive::Low);

    // Relative firmware paths resolve against the base directory.
    const sim::SystemConfig relative = sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: tlsr8258, firmware: fw/a.bin}\n",
        "/base/dir");
    CHECK(relative.devices[0].mcus[0].firmware == "/base/dir/fw/a.bin");

    CHECK(sim::parse_duration_ns("2.5s") == 2500000000ull);
    CHECK(sim::parse_duration_ns("750ns") == 750u);
    CHECK(sim::parse_duration_ns("0") == 0u);
    CHECK(sim::parse_frequency_hz("48MHz") == 48000000u);
    CHECK(sim::parse_frequency_hz("32kHz") == 32000u);
    CHECK(sim::parse_frequency_hz("24000000") == 24000000u);
}

void test_config_errors() {
    auto parse = [](const std::string &text) {
        return error_of([&] { sim::parse_system_config(text); });
    };
    CHECK(contains(parse("devices: []\n"), "'version' is required"));
    CHECK(contains(parse("version: 2\ndevices: []\n"), "unsupported version"));
    CHECK(contains(parse("version: 1\n"), "'devices' is required"));
    CHECK(contains(parse("version: 1\ndevices: [\n"), "line"));
    const std::string head = "version: 1\ndevices:\n  - id: d\n    mcus:\n";
    // Unknown key points at its line.
    const std::string typo =
        parse(head + "      - {id: m, model: tlsr8258, firmware: f, clok_hz: 1}\n");
    CHECK(contains(typo, "line 5") && contains(typo, "unknown key 'clok_hz'"));
    // Planned features are rejected explicitly, not ignored.
    CHECK(contains(parse(head + "      - {id: m, model: tlsr8258, firmware: f}\n"
                                "scenario:\n  script: x.lua\n"),
                   "planned but not supported yet"));
    CHECK(contains(parse(head + "      - {id: m, model: x, firmware: f, radio: {}}\n"),
                   "requires 'air'"));
    CHECK(contains(parse(head + "      - {id: m, model: tlsr8258}\n"),
                   "requires 'firmware'"));
    CHECK(contains(parse(head + "      - {id: 'a b', model: m, firmware: f}\n"),
                   "may only contain"));
    CHECK(contains(parse(head + "      - {id: m, model: t, firmware: f, start_at: 5}\n"),
                   "invalid duration"));
    CHECK(contains(parse(head + "      - {id: m, model: t, firmware: f}\n"
                                "      - {id: m, model: t, firmware: f}\n"),
                   "duplicate mcu id"));
    CHECK(contains(parse("version: 1\ndevices:\n  - {id: d, mcus: [{id: m, model: t, firmware: f}]}\n"
                         "  - {id: d, mcus: [{id: m, model: t, firmware: f}]}\n"),
                   "duplicate device id"));
    CHECK(contains(parse(head + "      - {id: m, model: t, firmware: f}\n"
                                "    nets:\n      - {id: n, kind: weird}\n"),
                   "push_pull or open_drain"));
    CHECK(contains(parse(head + "      - {id: m, model: t, firmware: f}\n"
                                "    links:\n      - {type: spi, a: x, b: y}\n"),
                   "not supported (only 'uart')"));
}

void test_system_build_and_run() {
    const sim::SystemConfig config = sim::parse_system_config(with_image(two_mcu_yaml));
    const auto system = sim::build_system(config);
    sim::World &world = system->world();
    CHECK(system->warnings().empty());

    auto &main_mcu = static_cast<sim::Tlsr8258Mcu &>(world.mcu("node.main"));
    auto &aux = static_cast<sim::Tlsr8258Mcu &>(world.mcu("node.aux"));
    CHECK(aux.time_ns() == 100000u); // start_at applied
    sim::Net &line = world.net("node.line");
    CHECK(line.level() == sim::Level::High); // open-drain with pull-up

    world.run_until(*system->duration_ns());
    // The scenario pulled the line low at 200 us and never released it.
    CHECK(line.level() == sim::Level::Low);
    CHECK((main_mcu.device().read8(tc32emu::Tlsr8258::GpioBase) & 1u) == 0u);
    CHECK((aux.device().read8(tc32emu::Tlsr8258::GpioBase + 8) & 2u) == 0u);
    CHECK(main_mcu.state() == sim::McuState::Running);
    CHECK(world.now() == 1000000u);
}

void test_system_build_errors() {
    auto build = [](const std::string &text) {
        return error_of([&] {
            sim::build_system(sim::parse_system_config(with_image(text)));
        });
    };
    const std::string head =
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: tlsr8258, firmware: IMG}\n";
    CHECK(contains(build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                         "      - {id: m, model: nope, firmware: IMG}\n"),
                   "unknown mcu model 'nope' (known: ca51f253l3, tlsr8258)"));
    CHECK(contains(build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                         "      - {id: m, model: tlsr8258, firmware: /no/such.bin}\n"),
                   "cannot open firmware image"));
    CHECK(contains(build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                         "      - {id: m, model: tlsr8258, variant: X, firmware: IMG}\n"),
                   "unknown tlsr8258 variant"));
    CHECK(contains(build(head + "    nets:\n      - {id: n, pins: [x.PA0]}\n"),
                   "unknown mcu 'd.x'"));
    CHECK(contains(build(head + "    nets:\n      - {id: n, pins: [m.PZ9]}\n"),
                   "no pad named 'PZ9'"));
    CHECK(contains(build(head + "    nets:\n      - {id: a, pins: [m.PA0]}\n"
                                "      - {id: b, pins: [m.PA0]}\n"),
                   "already connected"));
    CHECK(contains(build(head + "    nets:\n      - {id: a}\n      - {id: a}\n"),
                   "duplicate net 'd.a'"));
    CHECK(contains(build(head + "    links:\n      - {type: uart, a: m.uart0, b: m.uart0}\n"),
                   "to itself"));
    CHECK(contains(build(head + "    links:\n      - {type: uart, a: m.uart0, b: m.uart3}\n"),
                   "has 1 UART(s)"));
    CHECK(contains(build(head + "    links:\n      - {type: uart, a: m.uart0, b: m.serial}\n"),
                   "is not a UART"));
    CHECK(contains(build(head + "scenario:\n  events:\n"
                                "    - {at: 1us, net: nowhere, drive: low}\n"),
                   "unknown net 'nowhere'"));
    // World-level nets use full references and may span devices.
    CHECK(build("version: 1\ndevices:\n"
                "  - {id: a, mcus: [{id: m, model: tlsr8258, firmware: IMG}]}\n"
                "  - {id: b, mcus: [{id: m, model: tlsr8258, firmware: IMG}]}\n"
                "nets:\n  - {id: shared, pins: [a.m.PA0, b.m.PA0]}\n").empty());
}

void test_uart_link_between_mcus() {
    const auto system = sim::build_system(sim::parse_system_config(with_image(
        "version: 1\ndevices:\n"
        "  - id: d\n    mcus:\n"
        "      - {id: a, model: tlsr8258, firmware: IMG}\n"
        "      - {id: b, model: tlsr8258, firmware: IMG}\n"
        "      - {id: c, model: ca51f253l3, firmware: IMG}\n")));
    sim::World &world = system->world();
    (void)world;
    // CA51F253L3 cannot receive UART data yet: a link to it is one-way.
    const std::string yaml =
        "version: 1\ndevices:\n"
        "  - id: d\n    mcus:\n"
        "      - {id: a, model: tlsr8258, firmware: IMG}\n"
        "      - {id: b, model: tlsr8258, firmware: IMG}\n"
        "    links:\n      - {type: uart, a: a.uart0, b: b.uart0, latency: 5us}\n";
    const auto linked = sim::build_system(sim::parse_system_config(with_image(yaml)));
    sim::World &linked_world = linked->world();
    CHECK(linked->warnings().empty());

    auto &a = static_cast<sim::Tlsr8258Mcu &>(linked_world.mcu("d.a"));
    auto &b = static_cast<sim::Tlsr8258Mcu &>(linked_world.mcu("d.b"));
    // Firmware-free transmit: enable the UART and write a byte to A's TX FIFO.
    a.device().write8(tc32emu::Tlsr8258::UartControl0, 0x4f);
    a.device().write8(tc32emu::Tlsr8258::UartData, 0x5a);
    linked_world.run_until(2000000); // frame time at default baud << 2 ms
    // Byte arrived in B's RX FIFO (low nibble of the buffer count).
    CHECK((b.device().read8(tc32emu::Tlsr8258::UartBufferCount) & 0x0fu) == 1);
    CHECK(b.device().read8(tc32emu::Tlsr8258::UartData + 3) == 0x5a);
    CHECK(a.uart_take_tx(0).empty()); // the link consumed A's output
}

} // namespace tc32emu_tests
