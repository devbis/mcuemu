// The EFR32MG21 adapter in a simulated system: pads on nets, the model's
// fixed-clock and variant rules.

#include "../test_support.hpp"
#include "../nrf52/programs.hpp"

#include "sim/config.hpp"
#include "sim/efr32mg21_mcu.hpp"
#include "sim/system.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace mcuemu_tests {

namespace {

constexpr std::uint32_t Gpio = 0x4003C000;

std::string idle_image_path() {
    static const std::string path = [] {
        const auto file = std::filesystem::temp_directory_path() / "simcore-efr32-idle.bin";
        std::ofstream out(file, std::ios::binary);
        out.write(reinterpret_cast<const char *>(nrf52_programs::idle.data()),
                  static_cast<std::streamsize>(nrf52_programs::idle.size()));
        return file.string();
    }();
    return path;
}

std::unique_ptr<sim::System> build(const std::string &mcu_extra, const std::string &body) {
    return sim::build_system(sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: efr32mg21, firmware: " +
        idle_image_path() + mcu_extra + "}\n" + body));
}

} // namespace

void test_efr32_pads_follow_nets() {
    const auto system = build("", R"(    nets:
      - { id: led, pins: [m.PB0] }
      - { id: btn, pull: up, pins: [m.PD2] }
    parts:
      - { id: key, type: button, pins: { out: btn } }
)");
    sim::World &world = system->world();
    auto &mcu = static_cast<sim::Efr32mg21Mcu &>(world.mcu("d.m"));
    CHECK(mcu.model() == "efr32mg21");
    CHECK(world.net("d.led").level() == sim::Level::Floating);

    // PB0 push-pull: high through the DOUT set alias, low through the clear alias.
    mcu.chip().write(Gpio + 0x34, 4, 4);
    mcu.chip().write(Gpio + 0x1000 + 0x40, 1, 4);
    CHECK(world.net("d.led").level() == sim::Level::High);
    mcu.chip().write(Gpio + 0x2000 + 0x40, 1, 4);
    CHECK(world.net("d.led").level() == sim::Level::Low);

    // PD2 as an input: the button pulls the pulled-up net low.
    mcu.chip().write(Gpio + 0x94, 1u << 8, 4);
    CHECK((mcu.chip().read(Gpio + 0xa4, 4) >> 2 & 1u) == 1);
    world.endpoints().at("d.key.pressed").set(sim::Value{true});
    world.run_for(1000);
    CHECK(world.net("d.btn").level() == sim::Level::Low);
    CHECK((mcu.chip().read(Gpio + 0xa4, 4) >> 2 & 1u) == 0);
}

void test_efr32_config_rules() {
    bool rejected = false;
    try {
        build(", clock_hz: 80MHz", "");
    } catch (const std::exception &) {
        rejected = true;
    }
    CHECK(rejected); // the clock is fixed at the HFXO's 38.4 MHz
    rejected = false;
    try {
        build("", "    nets:\n      - { id: x, pins: [m.PE0] }\n");
    } catch (const std::exception &) {
        rejected = true;
    }
    CHECK(rejected); // the part has ports A..D only
}

} // namespace mcuemu_tests
