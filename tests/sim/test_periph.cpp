#include "../test_support.hpp"

#include "sim/config.hpp"
#include "sim/system.hpp"
#include "sim/tlsr8258_mcu.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace mcuemu_tests {

namespace {

using mcuemu::Tlsr8258;

std::string idle_image_path() {
    static const std::string path = [] {
        const auto file =
            std::filesystem::temp_directory_path() / "simcore-periph-idle.bin";
        std::ofstream out(file, std::ios::binary);
        out.put(static_cast<char>(0xfe));
        out.put(static_cast<char>(0x87)); // b .
        return file.string();
    }();
    return path;
}

std::unique_ptr<sim::System> build(const std::string &body) {
    return sim::build_system(sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: tlsr8258, firmware: " +
        idle_image_path() + "}\n" + body));
}

sim::Tlsr8258Mcu &mcu_of(sim::System &system) {
    return static_cast<sim::Tlsr8258Mcu &>(system.world().mcu("d.m"));
}

void reg_write(Tlsr8258 &device, std::uint32_t offset, std::uint8_t value) {
    device.write8(Tlsr8258::RegisterBase + offset, value);
}
std::uint8_t reg_read(Tlsr8258 &device, std::uint32_t offset) {
    return device.read8(Tlsr8258::RegisterBase + offset);
}

// AREG access exactly as the SDK's analog_write() performs it.
void analog_write(Tlsr8258 &device, std::uint8_t address, std::uint8_t value) {
    reg_write(device, 0xb8, address);
    reg_write(device, 0xb9, value);
    reg_write(device, 0xba, 0x60);
}

} // namespace

void test_pad_pulls_and_functions() {
    const auto system = build(R"(    nets:
      - { id: a }
      - { id: b }
    parts: []
)");
    // `parts: []` is an empty list and must be accepted.
    sim::World &world = system->world();
    (void)world;

    const auto sys = build("    nets:\n      - { id: a, pins: [m.PA2] }\n");
    sim::Tlsr8258Mcu &mcu = mcu_of(*sys);
    Tlsr8258 &device = mcu.device();
    sim::Net &net = sys->world().net("d.a");
    CHECK(net.level() == sim::Level::Floating); // input, no pull

    // Internal 10K pull-up on PA2 (analog register 0x0e, bits 5:4).
    analog_write(device, 0x0e, 3u << 4);
    CHECK(net.level() == sim::Level::High);
    CHECK(device.gpio_pull(0, 2) == mcuemu::GpioPull::Up);
    // Pull-down instead.
    analog_write(device, 0x0e, 2u << 4);
    CHECK(net.level() == sim::Level::Low);
    CHECK((device.read8(Tlsr8258::GpioBase) & 0x04u) == 0);
    // An output overrides the pull.
    reg_write(device, 0x583, 0x04);
    reg_write(device, 0x582, 0xfb);
    CHECK(net.level() == sim::Level::High);
    // A pad handed to another function (ACT_AS_GPIO = 0) is not driven by OUT.
    reg_write(device, 0x586, 0xfb);
    CHECK(net.level() == sim::Level::Low); // only the pull-down acts
    CHECK(!device.gpio_is_gpio(0, 2));
}

void test_adc_reads_net_voltage() {
    const auto system = build(R"(    nets:
      - { id: adc, pins: [m.PB1] }
    parts:
      - id: ntc
        type: ntc
        pins: { out: adc }
        params: { r25: 10k, beta: 3950, r_series: 10k, temp_c: 25 }
)");
    sim::World &world = system->world();
    Tlsr8258 &device = mcu_of(*system).device();

    // Configure the misc channel the way adc_base_init()/adc_init() do for PB1.
    analog_write(device, 0xfc, 0x00); // SAR ADC powered
    analog_write(device, 0xf2, 0x24); // misc channel enabled, max state 2
    analog_write(device, 0xe8, 0x2e); // P = PB1 (selector 2), N = ground
    analog_write(device, 0xe7, 0x2a); // 1.2 V reference
    analog_write(device, 0xfa, 0xfd); // input prescaler 1/8
    analog_write(device, 0xec, 0x43); // differential, 14-bit
    analog_write(device, 0xef, 0xff); // R_max_mc = 1023, R_max_s = 15 (~23 kHz)
    analog_write(device, 0xf1, 0xcf);
    // 32-byte ring buffer at 0x841000 (size register counts 16-byte units).
    reg_write(device, 0xb08, 0x00);
    reg_write(device, 0xb09, 0x10);
    reg_write(device, 0xb0a, 0x01);
    reg_write(device, 0xb1e, 0x00);
    reg_write(device, 0xb1f, 0x00);
    reg_write(device, 0xb10, 0x04); // DFIFO2 input enable

    auto sample = [&](unsigned index) {
        return static_cast<unsigned>(device.read16(Tlsr8258::RetainedSramBase + 0x1000 + index * 2));
    };

    world.run_until(world.now() + 500000);
    // 1650 mV at 1/8 prescale and a 1175 mV reference: 1650 * 8192 / (8 * 1175).
    const unsigned expected = 1438;
    CHECK(sample(0) >= expected - 2 && sample(0) <= expected + 2);
    CHECK(sample(5) >= expected - 2 && sample(5) <= expected + 2);
    // ~11 samples in 500 us; the ring buffer wraps after 16.
    CHECK(sample(15) == 0);

    // A hotter thermistor lowers the divider voltage and the code follows.
    world.endpoints().at("d.ntc.temp_c").set(sim::Value{50.0});
    world.run_until(world.now() + 1000000);
    const double mv = std::get<double>(world.endpoints().at("d.ntc.mv").get());
    const unsigned code = static_cast<unsigned>(mv * 8192.0 / (8.0 * 1175.0) + 0.5);
    unsigned matching = 0;
    for (unsigned i = 0; i < 16; ++i)
        matching += (sample(i) + 2 >= code && sample(i) <= code + 2) ? 1u : 0u;
    CHECK(matching == 16); // the whole ring has been rewritten

    // Disabling the channel stops sampling: the buffer keeps its content.
    reg_write(device, 0xb10, 0x00);
    const unsigned frozen = sample(3);
    world.endpoints().at("d.ntc.temp_c").set(sim::Value{0.0});
    world.run_until(world.now() + 500000);
    CHECK(sample(3) == frozen);

    // 8-bit resolution scales the code down by 2^6.
    analog_write(device, 0xec, 0x40);
    reg_write(device, 0xb1e, 0x00);
    reg_write(device, 0xb10, 0x04);
    world.run_until(world.now() + 1000000);
    const double mv0 = std::get<double>(world.endpoints().at("d.ntc.mv").get());
    const unsigned code8 = static_cast<unsigned>(mv0 * 8192.0 / (8.0 * 1175.0)) >> 6;
    CHECK(sample(0) + 1 >= code8 && sample(0) <= code8 + 1);
}

namespace {

struct SdkI2c {
    sim::World &world;
    Tlsr8258 &device;

    void wait_idle() {
        for (int i = 0; i < 2000 && (reg_read(device, 0x02) & 1u); ++i)
            world.run_for(2000);
        CHECK((reg_read(device, 0x02) & 1u) == 0);
    }
    void command(std::uint8_t value) {
        reg_write(device, 0x07, value);
        wait_idle();
    }
    // i2c_write_byte(addr, 1, data)
    void write_byte(std::uint8_t reg, std::uint8_t data) {
        reg_write(device, 0x01, static_cast<std::uint8_t>(reg_read(device, 0x01) & ~1u));
        reg_write(device, 0x04, reg);
        command(0x01 | 0x02 | 0x10); // ID | ADDR | START
        reg_write(device, 0x06, data);
        command(0x08); // DI
        command(0x20); // STOP
    }
    // i2c_read_series(addr, 1, buf, n)
    std::vector<std::uint8_t> read_series(std::uint8_t reg, unsigned count) {
        reg_write(device, 0x01, static_cast<std::uint8_t>(reg_read(device, 0x01) & ~1u));
        reg_write(device, 0x04, reg);
        command(0x01 | 0x02 | 0x10);
        reg_write(device, 0x01, static_cast<std::uint8_t>(reg_read(device, 0x01) | 1u));
        command(0x01 | 0x10); // repeated START + ID(read)
        std::vector<std::uint8_t> data;
        for (unsigned i = 0; i + 1 < count; ++i) {
            command(0x08 | 0x40); // DI | READ_ID
            data.push_back(reg_read(device, 0x06));
        }
        command(0x08 | 0x40 | 0x80); // last byte: NACK
        data.push_back(reg_read(device, 0x06));
        command(0x20);
        return data;
    }
};

} // namespace

void test_i2c_master_with_sensor() {
    // The bus has no external pull-ups: the pads' internal 10K pull-ups are
    // enabled by the "firmware", as i2c_gpio_set() does.
    const auto system = build(R"(    nets:
      - { id: sda, kind: open_drain, pins: [m.PC0] }
      - { id: scl, kind: open_drain, pins: [m.PC1] }
    parts:
      - id: sensor
        type: i2c_regs
        pins: { sda: sda, scl: scl }
        params: { address: 0x44, size: 32, init: "00=aa,01=bb,02=cc,03=dd" }
)");
    sim::World &world = system->world();
    Tlsr8258 &device = mcu_of(*system).device();
    sim::Net &sda = world.net("d.sda");
    sim::Net &scl = world.net("d.scl");

    CHECK(sda.level() == sim::Level::Floating);
    // i2c_gpio_set(C0C1): pull-ups, then the pads leave GPIO mode.
    analog_write(device, 0x12, 0x0f);
    reg_write(device, 0x596, 0xfc);
    CHECK(sda.level() == sim::Level::High && scl.level() == sim::Level::High);

    // i2c_master_init(): slave id, speed divider, master enable.
    reg_write(device, 0x00, 20);
    reg_write(device, 0x01, 0x44 << 1);
    reg_write(device, 0x03, 0x02);

    SdkI2c i2c{world, device};
    std::vector<std::pair<std::uint64_t, bool>> sda_edges;
    world.endpoints().at("d.sda").subscribe([&](const sim::EndpointEvent &e) {
        sda_edges.emplace_back(e.time_ns, std::get<bool>(e.value));
    });
    const std::uint64_t t0 = world.now();
    i2c.write_byte(0x10, 0x5c);
    CHECK(std::get<std::int64_t>(world.endpoints().at("d.sensor.reg10").get()) == 0x5c);
    CHECK((reg_read(device, 0x02) & 4u) == 0);       // slave ACKed everything
    CHECK((reg_read(device, 0x02) & 2u) == 0);       // bus free after STOP
    CHECK(sda.level() == sim::Level::High && scl.level() == sim::Level::High);
    // The transfer took real bus time: START + 3 bytes (27 bit periods) +
    // STOP at 4 * 20 cycles per bit (48 MHz) is about 47 us.
    CHECK(!sda_edges.empty());
    CHECK(sda_edges.back().first - t0 > 40000);

    // Sequential read with ACK on all but the last byte.
    const auto bytes = i2c.read_series(0x00, 4);
    CHECK((bytes == std::vector<std::uint8_t>{0xaa, 0xbb, 0xcc, 0xdd}));
    CHECK((reg_read(device, 0x02) & 4u) == 0);

    // Wrong slave address: nobody ACKs, NAK is reported.
    reg_write(device, 0x01, 0x45 << 1);
    reg_write(device, 0x07, 0x01 | 0x10);
    i2c.wait_idle();
    CHECK((reg_read(device, 0x02) & 4u) != 0);
    CHECK((reg_read(device, 0x02) & 2u) != 0); // START sent, no STOP yet
    reg_write(device, 0x07, 0x20);
    i2c.wait_idle();
    CHECK((reg_read(device, 0x02) & 2u) == 0);
    // The NAK of the probe survives the STOP: address probing relies on it.
    CHECK((reg_read(device, 0x02) & 4u) != 0);
    // The next transfer clears it.
    reg_write(device, 0x01, 0x44 << 1);
    reg_write(device, 0x07, 0x01 | 0x10);
    i2c.wait_idle();
    CHECK((reg_read(device, 0x02) & 4u) == 0);
    reg_write(device, 0x07, 0x20);
    i2c.wait_idle();

    // The master is disabled without the enable bit.
    reg_write(device, 0x03, 0x00);
    reg_write(device, 0x01, 0x44 << 1);
    reg_write(device, 0x07, 0x01 | 0x10);
    CHECK((reg_read(device, 0x02) & 1u) == 0);
    CHECK(sda.level() == sim::Level::High);
}

} // namespace mcuemu_tests
