// The nRF52840 adapter in a simulated system: pads on nets, TWIM against an
// I2C register device, UARTE bytes through the Mcu interface.

#include "../test_support.hpp"
#include "../nrf52/programs.hpp"

#include "sim/config.hpp"
#include "sim/nrf52840_mcu.hpp"
#include "sim/system.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

namespace mcuemu_tests {

namespace {

constexpr std::uint32_t P0 = 0x50000000, Twim0 = 0x40003000, Uarte0 = 0x40002000;

std::string idle_image_path() {
    static const std::string path = [] {
        const auto file = std::filesystem::temp_directory_path() / "simcore-nrf52-idle.bin";
        std::ofstream out(file, std::ios::binary);
        out.write(reinterpret_cast<const char *>(nrf52_programs::idle.data()),
                  static_cast<std::streamsize>(nrf52_programs::idle.size()));
        return file.string();
    }();
    return path;
}

std::unique_ptr<sim::System> build(const std::string &body) {
    return sim::build_system(sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: nrf52840, firmware: " +
        idle_image_path() + "}\n" + body));
}

sim::Nrf52840Mcu &mcu_of(sim::System &system) {
    return static_cast<sim::Nrf52840Mcu &>(system.world().mcu("d.m"));
}

void write(sim::Nrf52840Mcu &mcu, std::uint32_t address, std::uint32_t value) {
    mcu.chip().write(address, value, 4);
}
std::uint32_t read(sim::Nrf52840Mcu &mcu, std::uint32_t address) {
    return mcu.chip().read(address, 4);
}

} // namespace

void test_nrf52_pads_follow_nets() {
    const auto system = build(R"(    nets:
      - { id: out, pins: [m.P0.05] }
      - { id: in, pins: [m.P1.03] }
      - { id: floating, pins: [m.P0.31] }
    parts:
      - { id: key, type: button, pins: { out: in } }
)");
    sim::World &world = system->world();
    sim::Nrf52840Mcu &mcu = mcu_of(*system);
    CHECK(world.net("d.out").level() == sim::Level::Floating);

    // P0.05 as an output driving high, then low.
    write(mcu, P0 + 0x514, 1u << 5);
    write(mcu, P0 + 0x508, 1u << 5);
    CHECK(world.net("d.out").level() == sim::Level::High);
    write(mcu, P0 + 0x50c, 1u << 5);
    CHECK(world.net("d.out").level() == sim::Level::Low);

    // P1.03 as an input with a pull-up (port 1 registers sit at +0x300), read
    // back through IN; the button pulls it low.
    write(mcu, P0 + 0x300 + 0x700 + 4 * 3, 1u << 2 | 3u << 2); // input connected, pull-up
    CHECK(world.net("d.in").level() == sim::Level::High);
    CHECK((read(mcu, P0 + 0x300 + 0x510) >> 3 & 1u) == 1);
    world.endpoints().at("d.key.pressed").set(sim::Value{true});
    world.run_for(1000);
    CHECK(world.net("d.in").level() == sim::Level::Low);
    CHECK((read(mcu, P0 + 0x300 + 0x510) >> 3 & 1u) == 0);

    // A bad pad name is a configuration error.
    bool failed = false;
    try {
        build("    nets:\n      - { id: x, pins: [m.P0.32] }\n");
    } catch (const std::exception &error) {
        failed = std::string(error.what()).find("P0.32") != std::string::npos;
    }
    CHECK(failed);
}

void test_nrf52_twim_talks_to_an_i2c_device() {
    const auto system = build(R"(    nets:
      - { id: sda, kind: open_drain, pull: up, pins: [m.P0.17] }
      - { id: scl, kind: open_drain, pull: up, pins: [m.P0.20] }
    parts:
      - id: sensor
        type: i2c_regs
        pins: { sda: sda, scl: scl }
        params: { address: 0x44, size: 32, init: "00=aa,01=bb,02=cc,03=dd" }
)");
    sim::World &world = system->world();
    sim::Nrf52840Mcu &mcu = mcu_of(*system);
    auto run_for = [&](std::uint64_t ns) { world.run_for(ns); };

    // The SDK leaves the pads as outputs with OUT = 0; once TWIM owns them an
    // idle bus is both lines released.
    write(mcu, P0 + 0x700 + 4 * 17, 0x601);
    write(mcu, P0 + 0x700 + 4 * 20, 0x601);
    CHECK(world.net("d.sda").level() == sim::Level::Low);
    write(mcu, Twim0 + 0x508, 20);        // PSEL.SCL = P0.20
    write(mcu, Twim0 + 0x50c, 17);        // PSEL.SDA = P0.17
    write(mcu, Twim0 + 0x524, 0x01980000); // 100 kHz
    write(mcu, Twim0 + 0x588, 0x44);      // ADDRESS
    write(mcu, Twim0 + 0x500, 6);         // ENABLE
    CHECK(world.net("d.sda").level() == sim::Level::High && world.net("d.scl").level() == sim::Level::High);

    // Write register 0x10 = 0x5c (two bytes: pointer, value), then STOP.
    std::uint8_t *ram = mcu.chip().ram();
    ram[0x100] = 0x10;
    ram[0x101] = 0x5c;
    write(mcu, Twim0 + 0x544, 0x20000100);
    write(mcu, Twim0 + 0x548, 2);
    write(mcu, Twim0 + 0x200, 1u << 9);   // SHORTS: LASTTX_STOP
    write(mcu, Twim0 + 0x008, 1);         // TASKS_STARTTX
    run_for(400000);
    CHECK(read(mcu, Twim0 + 0x104) == 1); // EVENTS_STOPPED
    CHECK(read(mcu, Twim0 + 0x4c4) == 0); // no error
    CHECK(read(mcu, Twim0 + 0x54c) == 2); // TXD.AMOUNT
    CHECK(std::get<std::int64_t>(world.endpoints().at("d.sensor.reg10").get()) == 0x5c);
    CHECK(world.net("d.sda").level() == sim::Level::High && world.net("d.scl").level() == sim::Level::High);

    // Read four bytes from register 0: write the pointer, repeated START, read
    // with ACK on all but the last byte, STOP.
    write(mcu, Twim0 + 0x104, 0);
    ram[0x100] = 0x00;
    write(mcu, Twim0 + 0x548, 1);
    write(mcu, Twim0 + 0x534, 0x20000200);
    write(mcu, Twim0 + 0x538, 4);
    write(mcu, Twim0 + 0x200, (1u << 7) | (1u << 12)); // LASTTX_STARTRX | LASTRX_STOP
    write(mcu, Twim0 + 0x008, 1);
    run_for(800000);
    CHECK(read(mcu, Twim0 + 0x104) == 1);
    CHECK(read(mcu, Twim0 + 0x53c) == 4);
    CHECK(ram[0x200] == 0xaa && ram[0x201] == 0xbb && ram[0x202] == 0xcc && ram[0x203] == 0xdd);

    // Nobody answers at another address: ANACK, ERROR, then STOPPED.
    write(mcu, Twim0 + 0x104, 0);
    write(mcu, Twim0 + 0x124, 0);
    write(mcu, Twim0 + 0x588, 0x45);
    write(mcu, Twim0 + 0x200, 1u << 9);
    write(mcu, Twim0 + 0x008, 1);
    run_for(400000);
    CHECK(read(mcu, Twim0 + 0x124) == 1);        // EVENTS_ERROR
    CHECK((read(mcu, Twim0 + 0x4c4) & 2u) != 0); // ERRORSRC.ANACK
    CHECK(read(mcu, Twim0 + 0x104) == 1);
    CHECK(world.net("d.sda").level() == sim::Level::High && world.net("d.scl").level() == sim::Level::High);
}

void test_nrf52_uart_bytes_through_the_mcu_interface() {
    const auto system = build("    nets: []\n    parts: []\n");
    sim::World &world = system->world();
    sim::Nrf52840Mcu &mcu = mcu_of(*system);
    CHECK(mcu.uart_count() == 2);
    CHECK(!mcu.uart_can_receive(0)); // not enabled yet

    std::memcpy(mcu.chip().ram() + 0x300, "ok", 2);
    write(mcu, Uarte0 + 0x500, 8);
    write(mcu, Uarte0 + 0x524, 0x01D7E000);
    write(mcu, Uarte0 + 0x544, 0x20000300);
    write(mcu, Uarte0 + 0x548, 2);
    write(mcu, Uarte0 + 0x008, 1);
    world.run_for(500000);
    // The world's UART endpoint collects what the chip transmitted.
    auto &uart = world.endpoints().at("d.m.uart0");
    CHECK(std::get<std::int64_t>(uart.get()) == 2);

    CHECK(mcu.uart_can_receive(0));
    write(mcu, Uarte0 + 0x534, 0x20000400);
    write(mcu, Uarte0 + 0x538, 1);
    write(mcu, Uarte0 + 0x000, 1);
    uart.set(sim::Value{std::string("Z")});
    world.run_for(1000);
    CHECK(mcu.chip().ram()[0x400] == 'Z');
    CHECK(read(mcu, Uarte0 + 0x110) == 1); // ENDRX
}

} // namespace mcuemu_tests
