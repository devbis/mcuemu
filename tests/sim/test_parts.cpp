#include "../test_support.hpp"

#include "sim/config.hpp"
#include "sim/parts/ds18b20.hpp"
#include "sim/system.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace mcuemu_tests {

namespace {

std::string idle_image_path() {
    static const std::string path = [] {
        const auto file =
            std::filesystem::temp_directory_path() / "simcore-parts-idle.bin";
        std::ofstream out(file, std::ios::binary);
        out.put(static_cast<char>(0xfe));
        out.put(static_cast<char>(0x87)); // b .
        return file.string();
    }();
    return path;
}

// Builds a system with one idle MCU, extra YAML for the device body, and
// returns it. `body` is indented under the device.
std::unique_ptr<sim::System> build(const std::string &body,
                                   const std::string &tail = "") {
    const std::string yaml = "version: 1\ndevices:\n  - id: d\n    mcus:\n"
                             "      - {id: m, model: tlsr8258, firmware: " +
                             idle_image_path() + "}\n" + body + tail;
    return sim::build_system(sim::parse_system_config(yaml));
}

template <typename Fn> std::string build_error(Fn &&fn) {
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

// Bit-banged 1-Wire master driving a Pin with explicit virtual timestamps.
struct OneWireMaster {
    sim::World &world;
    sim::Net &net;
    sim::Pin &pin;
    std::uint64_t t{1000000};

    void advance_us(unsigned us) {
        t += us * 1000ull;
        world.scheduler().advance_to(t);
    }
    bool reset() {
        pin.set_drive(sim::Drive::Low, t);
        advance_us(480);
        pin.set_drive(sim::Drive::HiZ, t);
        advance_us(70);
        const bool presence = net.level() == sim::Level::Low;
        advance_us(410);
        return presence;
    }
    void write_bit(bool bit) {
        pin.set_drive(sim::Drive::Low, t);
        advance_us(bit ? 6 : 60);
        pin.set_drive(sim::Drive::HiZ, t);
        advance_us(bit ? 64 : 10);
    }
    bool read_bit() {
        pin.set_drive(sim::Drive::Low, t);
        advance_us(3);
        pin.set_drive(sim::Drive::HiZ, t);
        advance_us(9);
        const bool bit = net.level() == sim::Level::High;
        advance_us(58);
        return bit;
    }
    void write_byte(std::uint8_t byte) {
        for (int i = 0; i < 8; ++i)
            write_bit(((byte >> i) & 1u) != 0);
    }
    std::uint8_t read_byte() {
        std::uint8_t value = 0;
        for (int i = 0; i < 8; ++i)
            value = static_cast<std::uint8_t>(value | (read_bit() ? 1u << i : 0u));
        return value;
    }
};

// A DS18B20 part definition, native or the Lua reference script.
std::string ds_part(const std::string &id, bool lua, const std::string &params) {
    std::string yaml = "      - id: " + id + "\n";
    if (lua)
        yaml += std::string("        type: script\n        script: ") +
                MCUEMU_SOURCE_DIR "/examples/parts/ds18b20.lua\n";
    else
        yaml += "        type: ds18b20\n";
    return yaml + "        pins: { dq: ow }\n        params: { " + params + " }\n";
}

std::string one_wire_device(bool lua) {
    return "    nets:\n      - { id: ow, kind: open_drain, pull: up }\n"
           "    parts:\n" +
           ds_part("t1", lua, "temperature_c: 23.5");
}

} // namespace

static void ds18b20_one_wire(bool lua) {
    const auto system = build(one_wire_device(lua));
    sim::World &world = system->world();
    sim::Net &net = world.net("d.ow");
    OneWireMaster master{world, net, net.attach("test")};

    CHECK(master.reset()); // presence pulse seen
    master.write_byte(0x33); // Read ROM
    std::uint8_t rom[8];
    for (auto &byte : rom)
        byte = master.read_byte();
    CHECK(rom[0] == 0x28);
    CHECK(sim::Ds18b20::crc8(rom, 8) == 0); // CRC over all 8 bytes is zero
    CHECK(sim::Ds18b20::crc8(rom, 7) == rom[7]);

    // Power-on scratchpad holds 85 C until the first conversion.
    CHECK(master.reset());
    master.write_byte(0xcc);
    master.write_byte(0xbe);
    CHECK(master.read_byte() == 0x50);
    CHECK(master.read_byte() == 0x05);

    // Convert: busy (reads 0) for 750 ms at 12 bits, then 1.
    CHECK(master.reset());
    master.write_byte(0xcc);
    master.write_byte(0x44);
    CHECK(!master.read_bit());
    master.advance_us(400000);
    CHECK(!master.read_bit()); // 400 ms: still converting
    master.advance_us(400000);
    CHECK(master.read_bit()); // 800 ms: done

    CHECK(master.reset());
    master.write_byte(0xcc);
    master.write_byte(0xbe);
    std::uint8_t pad[9];
    for (auto &byte : pad)
        byte = master.read_byte();
    CHECK(pad[0] == 0x78 && pad[1] == 0x01); // 23.5 C = 376 / 16
    CHECK(pad[4] == 0x7f);                   // 12-bit resolution
    CHECK(sim::Ds18b20::crc8(pad, 9) == 0);

    // Temperature is controllable from outside; negative values are
    // two's complement. Match ROM addresses the device by serial number.
    world.endpoints().at("d.t1.temperature_c").set(sim::Value{-10.125});
    CHECK(master.reset());
    master.write_byte(0x55);
    for (std::uint8_t byte : rom)
        master.write_byte(byte);
    master.write_byte(0x44);
    master.advance_us(800000);
    CHECK(master.reset());
    master.write_byte(0xcc);
    master.write_byte(0xbe);
    CHECK(master.read_byte() == 0x5e);
    CHECK(master.read_byte() == 0xff);

    // A wrong serial number leaves the device silent: reads return all ones.
    CHECK(master.reset());
    master.write_byte(0x55);
    for (int i = 0; i < 8; ++i)
        master.write_byte(0x00);
    master.write_byte(0xbe);
    CHECK(master.read_byte() == 0xff);

    // Reset pulses shorter than 480 us are ignored.
    master.pin.set_drive(sim::Drive::Low, master.t);
    master.advance_us(200);
    master.pin.set_drive(sim::Drive::HiZ, master.t);
    master.advance_us(100);
    CHECK(net.level() == sim::Level::High);
}

static void ds18b20_search_two_devices(bool lua) {
    const auto system = build(
        "    nets:\n      - { id: ow, kind: open_drain, pull: up }\n    parts:\n" +
        ds_part("a", lua, "rom: \"28-11-22-33-44-55-66\", resolution: 9") +
        ds_part("b", lua, "rom: \"28-11-22-33-44-55-77\""));
    sim::World &world = system->world();
    sim::Net &net = world.net("d.ow");
    OneWireMaster master{world, net, net.attach("test")};

    // Standard Maxim ROM search over the shared open-drain line.
    std::vector<std::array<std::uint8_t, 8>> found;
    int last_discrepancy = 0;
    std::array<std::uint8_t, 8> rom{};
    for (int guard = 0; guard < 4; ++guard) {
        if (!master.reset())
            break;
        master.write_byte(0xf0);
        int last_zero = 0;
        for (int id_bit = 1; id_bit <= 64; ++id_bit) {
            const bool bit = master.read_bit();
            const bool complement = master.read_bit();
            CHECK(!(bit && complement)); // at least one device is present
            bool direction;
            if (bit != complement) {
                direction = bit;
            } else {
                if (id_bit < last_discrepancy)
                    direction = ((rom[(id_bit - 1) / 8] >> ((id_bit - 1) % 8)) & 1u) != 0;
                else
                    direction = id_bit == last_discrepancy;
                if (!direction)
                    last_zero = id_bit;
            }
            auto &byte = rom[(id_bit - 1) / 8];
            const unsigned mask = 1u << ((id_bit - 1) % 8);
            byte = static_cast<std::uint8_t>(direction ? byte | mask : byte & ~mask);
            master.write_bit(direction);
        }
        found.push_back(rom);
        last_discrepancy = last_zero;
        if (last_discrepancy == 0)
            break;
    }
    CHECK(found.size() == 2);
    if (found.size() == 2) {
        CHECK(found[0][6] == 0x66 && found[1][6] == 0x77);
        CHECK(sim::Ds18b20::crc8(found[0].data(), 8) == 0);
        CHECK(sim::Ds18b20::crc8(found[1].data(), 8) == 0);
    }
}

void test_ds18b20_one_wire() { ds18b20_one_wire(false); }
void test_ds18b20_search_two_devices() { ds18b20_search_two_devices(false); }
void test_ds18b20_lua_matches_native() {
    ds18b20_one_wire(true);
    ds18b20_search_two_devices(true);
}

void test_ntc_divider() {
    const auto system = build(R"(    nets:
      - { id: adc }
    parts:
      - id: ntc1
        type: ntc
        pins: { out: adc }
        params: { r25: 10k, beta: 3950, r_series: 10k, temp_c: 25 }
)");
    sim::World &world = system->world();
    sim::Net &adc = world.net("d.adc");
    CHECK(adc.millivolts() >= 1649 && adc.millivolts() <= 1651); // R_ntc == R_series
    CHECK(adc.level() == sim::Level::High);                      // 1650 >= 1650

    world.endpoints().at("d.ntc1.temp_c").set(sim::Value{50.0});
    CHECK(adc.millivolts() >= 860 && adc.millivolts() <= 885); // ~3.59 kOhm
    CHECK(adc.level() == sim::Level::Low);
    const double mv = std::get<double>(world.endpoints().at("d.ntc1.mv").get());
    CHECK(std::fabs(mv - adc.millivolts()) < 1.0);

    world.endpoints().at("d.ntc1.temp_c").set(sim::Value{std::int64_t{0}});
    CHECK(adc.millivolts() > 2300); // colder -> larger R_ntc -> higher voltage
}

namespace {

struct I2cMaster {
    sim::Net &sda_net;
    sim::Net &scl_net;
    sim::Pin &sda;
    sim::Pin &scl;
    std::uint64_t t{1000};

    void tick() { t += 1000; }
    void set_sda(bool high) { sda.set_drive(high ? sim::Drive::HiZ : sim::Drive::Low, t); tick(); }
    void set_scl(bool high) { scl.set_drive(high ? sim::Drive::HiZ : sim::Drive::Low, t); tick(); }
    void start() { set_sda(true); set_scl(true); set_sda(false); set_scl(false); }
    void repeated_start() { set_sda(true); set_scl(true); set_sda(false); set_scl(false); }
    void stop() { set_sda(false); set_scl(true); set_sda(true); }
    // Returns true if the slave ACKed.
    bool write_byte(std::uint8_t byte) {
        for (int i = 7; i >= 0; --i) {
            set_sda(((byte >> i) & 1u) != 0);
            set_scl(true);
            set_scl(false);
        }
        set_sda(true); // release for ACK
        set_scl(true);
        const bool ack = sda_net.level() == sim::Level::Low;
        set_scl(false);
        return ack;
    }
    std::uint8_t read_byte(bool ack) {
        set_sda(true);
        std::uint8_t value = 0;
        for (int i = 7; i >= 0; --i) {
            set_scl(true);
            value = static_cast<std::uint8_t>(value | (sda_net.level() != sim::Level::Low ? 1u << i : 0u));
            set_scl(false);
        }
        set_sda(!ack); // ACK = pull low
        set_scl(true);
        set_scl(false);
        set_sda(true);
        return value;
    }
};

} // namespace

void test_i2c_register_device() {
    const auto system = build(R"(    nets:
      - { id: sda, kind: open_drain, pull: up }
      - { id: scl, kind: open_drain, pull: up }
    parts:
      - id: sensor
        type: i2c_regs
        pins: { sda: sda, scl: scl }
        params: { address: 0x44, size: 32, init: "00=aa,01=bb,02=cc" }
)");
    sim::World &world = system->world();
    sim::Net &sda_net = world.net("d.sda");
    sim::Net &scl_net = world.net("d.scl");
    I2cMaster m{sda_net, scl_net, sda_net.attach("test"), scl_net.attach("test")};

    // Write register pointer 0, repeated START, read three bytes.
    m.start();
    CHECK(m.write_byte(0x44 << 1));
    CHECK(m.write_byte(0x00));
    m.repeated_start();
    CHECK(m.write_byte((0x44 << 1) | 1));
    CHECK(m.read_byte(true) == 0xaa);
    CHECK(m.read_byte(true) == 0xbb);
    CHECK(m.read_byte(false) == 0xcc); // NACK ends the read
    m.stop();
    CHECK(sda_net.level() == sim::Level::High && scl_net.level() == sim::Level::High);

    // Write a register, then verify through the endpoint.
    m.start();
    CHECK(m.write_byte(0x44 << 1));
    CHECK(m.write_byte(0x10));
    CHECK(m.write_byte(0x5c));
    CHECK(m.write_byte(0x5d)); // auto-increment
    m.stop();
    CHECK(std::get<std::int64_t>(world.endpoints().at("d.sensor.reg10").get()) == 0x5c);
    CHECK(std::get<std::int64_t>(world.endpoints().at("d.sensor.reg11").get()) == 0x5d);

    // Another address is not ACKed, and the device ignores the rest.
    m.start();
    CHECK(!m.write_byte(0x45 << 1));
    m.stop();
    CHECK(std::get<std::int64_t>(world.endpoints().at("d.sensor.reg10").get()) == 0x5c);

    // Out-of-range register pointer is NACKed.
    m.start();
    CHECK(m.write_byte(0x44 << 1));
    CHECK(!m.write_byte(0x40));
    m.stop();

    // Registers can be changed from outside (a sensor reading, for example).
    world.endpoints().at("d.sensor.reg00").set(sim::Value{std::int64_t{0x12}});
    m.start();
    CHECK(m.write_byte(0x44 << 1));
    CHECK(m.write_byte(0x00));
    m.repeated_start();
    CHECK(m.write_byte((0x44 << 1) | 1));
    CHECK(m.read_byte(false) == 0x12);
    m.stop();
}

namespace {

// Sensirion CRC-8 (poly 0x31, init 0xff), independent of the script.
std::uint8_t sensirion_crc(std::uint8_t msb, std::uint8_t lsb) {
    std::uint8_t crc = 0xff;
    for (std::uint8_t byte : {msb, lsb}) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i)
            crc = static_cast<std::uint8_t>((crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1);
    }
    return crc;
}

} // namespace

void test_lua_sht3x_over_i2c() {
    CHECK(sensirion_crc(0xbe, 0xef) == 0x92); // datasheet example
    const auto system = build(std::string(R"(    nets:
      - { id: sda, kind: open_drain, pull: up }
      - { id: scl, kind: open_drain, pull: up }
    parts:
      - id: sht
        type: script
        script: )") + MCUEMU_SOURCE_DIR + R"(/examples/parts/sht3x.lua
        pins: { sda: sda, scl: scl }
        params: { address: 0x44, temperature_c: 25, humidity: 50 }
)");
    sim::World &world = system->world();
    sim::Net &sda_net = world.net("d.sda");
    sim::Net &scl_net = world.net("d.scl");
    I2cMaster m{sda_net, scl_net, sda_net.attach("test"), scl_net.attach("test")};

    auto measure = [&](std::uint8_t out[6]) {
        m.start();
        CHECK(m.write_byte(0x44 << 1));
        CHECK(m.write_byte(0x24));
        CHECK(m.write_byte(0x00));
        m.stop();
        m.start();
        CHECK(m.write_byte((0x44 << 1) | 1));
        for (int i = 0; i < 6; ++i)
            out[i] = m.read_byte(i < 5);
        m.stop();
    };
    std::uint8_t data[6];
    measure(data);
    CHECK(data[0] == 0x66 && data[1] == 0x66); // 25 C = (25 + 45) / 175 * 65535
    CHECK(data[2] == sensirion_crc(data[0], data[1]));
    CHECK(data[3] == 0x80 && data[4] == 0x00); // 50 %RH
    CHECK(data[5] == sensirion_crc(data[3], data[4]));

    // Parameters are endpoints; the next measurement reflects them.
    world.endpoints().at("d.sht.temperature_c").set(sim::Value{-45.0});
    world.endpoints().at("d.sht.humidity").set(sim::Value{std::int64_t{100}});
    measure(data);
    CHECK(data[0] == 0x00 && data[1] == 0x00);
    CHECK(data[3] == 0xff && data[4] == 0xff);

    // An unknown command is NACKed on its second byte; another address is not
    // acknowledged at all.
    m.start();
    CHECK(m.write_byte(0x44 << 1));
    CHECK(m.write_byte(0x12));
    CHECK(!m.write_byte(0x34));
    m.stop();
    m.start();
    CHECK(!m.write_byte(0x45 << 1));
    m.stop();
}

void test_part_errors_and_events() {
    const std::string nets =
        "    nets:\n      - { id: ow, kind: open_drain, pull: up }\n";
    auto err = [&](const std::string &parts) {
        return build_error([&] { build(nets + "    parts:\n" + parts); });
    };
    CHECK(contains(err("      - {id: p, type: nope}\n"),
                   "unknown part type 'nope' (known: button, ds18b20, i2c_regs, led, ntc, script)"));
    CHECK(contains(err("      - {id: p, type: ds18b20}\n"),
                   "terminal 'dq' is not wired"));
    CHECK(contains(err("      - {id: p, type: ds18b20, pins: {dq: nowhere}}\n"),
                   "unknown net 'nowhere'"));
    CHECK(contains(err("      - {id: p, type: ds18b20, pins: {dq: ow, extra: ow}}\n"),
                   "has no terminal 'extra'"));
    CHECK(contains(err("      - {id: p, type: ds18b20, pins: {dq: ow}, params: {temp: 1}}\n"),
                   "unknown param 'temp'"));
    CHECK(contains(err("      - {id: p, type: ds18b20, pins: {dq: ow}, params: {resolution: 13}}\n"),
                   "'resolution' must be 9..12"));
    CHECK(contains(err("      - {id: p, type: ds18b20, pins: {dq: ow}, params: {rom: 28-zz}}\n"),
                   "rom must look like"));
    CHECK(contains(err("      - {id: p, type: ds18b20, pins: {dq: ow}, params: {rom: 28-1-2-3-4-5-6-7}}\n"),
                   "CRC byte is wrong"));
    CHECK(contains(err("      - {id: p, type: ntc, pins: {out: ow}, params: {r25: abc}}\n"),
                   "positive resistance"));
    CHECK(contains(err("      - {id: p, type: i2c_regs, pins: {sda: ow, scl: ow}}\n"),
                   "'address'"));
    CHECK(contains(err("      - {id: p, type: ds18b20, pins: {dq: ow}}\n"
                       "      - {id: p, type: ds18b20, pins: {dq: ow}}\n"),
                   "duplicate part id"));
    CHECK(contains(build_error([&] {
                       build(nets, "scenario:\n  events:\n"
                                   "    - {at: 1ms, endpoint: d.nothing, set: 1}\n");
                   }),
                   "unknown endpoint 'd.nothing'"));
    CHECK(contains(build_error([&] {
                       sim::parse_system_config(
                           "version: 1\ndevices:\n  - {id: d, mcus: [{id: m, model: t, firmware: f}]}\n"
                           "scenario:\n  events:\n    - {at: 1ms, net: n, drive: low, set: 1}\n");
                   }),
                   "either 'net' + 'drive' or 'endpoint' + 'set'"));

    // Scenario events can set part parameters at a given time.
    const auto system = build(
        nets + "    parts:\n"
               "      - {id: t, type: ds18b20, pins: {dq: ow}, params: {temperature_c: 20}}\n",
        "scenario:\n  events:\n"
        "    - {at: 2ms, endpoint: d.t.temperature_c, set: 31.5}\n");
    sim::World &world = system->world();
    auto &endpoint = world.endpoints().at("d.t.temperature_c");
    CHECK(std::get<double>(endpoint.get()) == 20.0);
    std::vector<double> seen;
    endpoint.subscribe([&](const sim::EndpointEvent &e) {
        seen.push_back(std::get<double>(e.value));
    });
    world.run_until(1000000);
    CHECK(std::get<double>(endpoint.get()) == 20.0);
    world.run_until(3000000);
    CHECK(std::get<double>(endpoint.get()) == 31.5);
    CHECK(seen.size() == 1 && seen[0] == 31.5);
}

} // namespace mcuemu_tests
