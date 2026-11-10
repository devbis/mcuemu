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

// I2C0 as a master on a routed pad pair, with the CHT8305 Lua part on the nets: the emlib sequence for a register
// read (address + pointer write, repeated START, address + read, two bytes, NACK, STOP).
void test_efr32_i2c_master_reads_cht8305() {
    const std::string parts = std::string(MCUEMU_SOURCE_DIR) + "/examples/parts/";
    const auto system = build("", "    nets:\n"
                                  "      - { id: scl, kind: open_drain, pull: up, pins: [m.PC1] }\n"
                                  "      - { id: sda, kind: open_drain, pull: up, pins: [m.PC2] }\n"
                                  "    parts:\n"
                                  "      - { id: sensor, type: script, script: " + parts + "cht8305.lua, pins: { sda: sda, scl: scl },\n"
                                  "          params: { temperature_c: 25, humidity: 50 } }\n"
                                  "      - { id: light, type: script, script: " + parts + "opt3001.lua, pins: { sda: sda, scl: scl },\n"
                                  "          params: { lux: 320 } }\n");
    sim::World &world = system->world();
    auto &mcu = static_cast<sim::Efr32mg21Mcu &>(world.mcu("d.m"));
    auto &chip = mcu.chip();
    constexpr std::uint32_t I2c = 0x4A010000, Set = 0x1000, Clear = 0x2000;
    // PC1 (SCL) and PC2 (SDA): wired-and with pull-up, DOUT high; I2C0 routed to them.
    chip.write(Gpio + 0x64, (0xAu << 4) | (0xAu << 8), 4);
    chip.write(Gpio + Set + 0x70, 6, 4);
    chip.write(Gpio + 0x49C, 2u | (1u << 16), 4);
    chip.write(Gpio + 0x4A0, 2u | (2u << 16), 4);
    chip.write(Gpio + 0x498, 3, 4);
    chip.write(I2c + 0x18, 46, 4);
    chip.write(I2c + 0x04, 1, 4);
    auto wait_for = [&](std::uint32_t flag) {
        for (int i = 0; i != 2000; ++i) {
            if (chip.read(I2c + 0x3C, 4) & flag)
                return true;
            world.run_for(5000);
        }
        return false;
    };
    auto clear = [&](std::uint32_t flags) { chip.write(I2c + Clear + 0x3C, flags, 4); };
    constexpr std::uint32_t Ack = 1u << 6, Nack = 1u << 7, Rx = 1u << 5, Stop = 1u << 8;
    chip.write(I2c + 0x34, 0x40u << 1, 4);
    chip.write(I2c + 0x0C, 1, 4); // START
    CHECK(wait_for(Ack | Nack));
    CHECK((chip.read(I2c + 0x3C, 4) & Ack) != 0);
    clear(Ack);
    chip.write(I2c + 0x34, 0x00, 4); // register pointer: temperature
    CHECK(wait_for(Ack | Nack));
    CHECK((chip.read(I2c + 0x3C, 4) & Ack) != 0);
    clear(Ack);
    chip.write(I2c + 0x34, (0x40u << 1) | 1u, 4);
    chip.write(I2c + 0x0C, 1, 4); // repeated START
    CHECK(wait_for(Ack | Nack));
    CHECK((chip.read(I2c + 0x3C, 4) & Ack) != 0);
    clear(Ack);
    CHECK(wait_for(Rx));
    const std::uint32_t high = chip.read(I2c + 0x24, 4);
    chip.write(I2c + 0x0C, 1u << 2, 4); // ACK
    CHECK(wait_for(Rx));
    const std::uint32_t low = chip.read(I2c + 0x24, 4);
    chip.write(I2c + 0x0C, (1u << 3) | (1u << 1), 4); // NACK, STOP
    CHECK(wait_for(Stop));
    CHECK(((high << 8) | low) == 0x64D8); // (25 + 40) / 165 * 65536 with the low two bits clear
    // The OPT3001 light sensor next to it on the same bus: manufacturer id, then the result for 320 lux (E = 3, M = 4000).
    clear(0x1FF);
    chip.write(I2c + 0x34, 0x44u << 1, 4);
    chip.write(I2c + 0x0C, 1, 4);
    CHECK(wait_for(Ack | Nack));
    CHECK((chip.read(I2c + 0x3C, 4) & Ack) != 0);
    clear(Ack);
    chip.write(I2c + 0x34, 0x7E, 4);
    CHECK(wait_for(Ack | Nack));
    clear(Ack);
    chip.write(I2c + 0x34, (0x44u << 1) | 1u, 4);
    chip.write(I2c + 0x0C, 1, 4);
    CHECK(wait_for(Ack | Nack));
    clear(Ack);
    CHECK(wait_for(Rx));
    const std::uint32_t id_high = chip.read(I2c + 0x24, 4);
    chip.write(I2c + 0x0C, 1u << 2, 4);
    CHECK(wait_for(Rx));
    const std::uint32_t id_low = chip.read(I2c + 0x24, 4);
    chip.write(I2c + 0x0C, (1u << 3) | (1u << 1), 4);
    CHECK(wait_for(Stop));
    CHECK(((id_high << 8) | id_low) == 0x5449);
    // A device that is not there is NACKed.
    clear(0x1FF);
    chip.write(I2c + 0x34, 0x55u << 1, 4);
    chip.write(I2c + 0x0C, 1, 4);
    CHECK(wait_for(Ack | Nack));
    CHECK((chip.read(I2c + 0x3C, 4) & Nack) != 0);
    chip.write(I2c + 0x0C, 1u << 1, 4);
    CHECK(wait_for(Stop));
}

// The e-ink display part decodes the 9-bit SPI frames the firmware bit-bangs on PD0 (SCK), PC2 (DATA) and PB1 (CS).
void test_efr32_epd_part_decodes_frames() {
    const std::string parts = std::string(MCUEMU_SOURCE_DIR) + "/examples/parts/";
    const auto system = build("", "    nets:\n"
                                  "      - { id: sck, pins: [m.PD0] }\n"
                                  "      - { id: sdi, pins: [m.PC2] }\n"
                                  "      - { id: cs, pins: [m.PB1] }\n"
                                  "    parts:\n"
                                  "      - { id: epd, type: script, script: " + parts + "epd_zsszkthl.lua, pins: { sck: sck, data: sdi, cs: cs } }\n");
    sim::World &world = system->world();
    auto &chip = static_cast<sim::Efr32mg21Mcu &>(world.mcu("d.m")).chip();
    chip.write(Gpio + 0x94, 4, 4);  // PD0 push-pull
    chip.write(Gpio + 0x64, 4u << 8, 4);  // PC2
    chip.write(Gpio + 0x34, 4u << 4, 4);  // PB1
    auto pin = [&](std::uint32_t port_dout, unsigned bit, bool high) {
        chip.write(Gpio + (high ? 0x1000u : 0x2000u) + port_dout, 1u << bit, 4);
        world.run_for(1000);
    };
    auto frame = [&](bool data, std::uint8_t value) {
        pin(0x40, 1, false); // CS low
        for (int i = 8; i >= 0; --i) {
            const bool bit = i == 8 ? data : ((value >> i) & 1u) != 0;
            pin(0x70, 2, bit);
            pin(0x90 + 0x10, 0, true); // SCK rises
            pin(0x90 + 0x10, 0, false);
        }
        pin(0x40, 1, true);
    };
    pin(0x40, 1, true);
    frame(false, 0x18);
    frame(true, 0xA5);
    frame(true, 0x3C);
    frame(false, 0x11);
    frame(true, 0xFF);
    CHECK(std::get<std::int64_t>(world.endpoints().at("d.epd.refreshes").get()) == 1);
    CHECK(std::get<std::string>(world.endpoints().at("d.epd.ram").get()) == "a53c");
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
