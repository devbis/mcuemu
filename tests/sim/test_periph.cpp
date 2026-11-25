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
    // 64-byte ring buffer of 16 entries at 0x841000 (size register counts 16-byte units).
    reg_write(device, 0xb08, 0x00);
    reg_write(device, 0xb09, 0x10);
    reg_write(device, 0xb0a, 0x03);
    reg_write(device, 0xb1e, 0x00);
    reg_write(device, 0xb1f, 0x00);
    reg_write(device, 0xb10, 0x04); // DFIFO2 input enable

    auto sample = [&](unsigned index) {
        return static_cast<unsigned>(device.read16(Tlsr8258::RetainedSramBase + 0x1000 + index * 4));
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

// The link layer command of the BLE mode: rf_start_stx2rx() of the vendor SDK writes the command to
// 0xf00 and then the address of the packet to the DMA channel 3, so the packet is read after the
// command; rf_set_ble_channel() leaves the channel number in 0x40d and the access address and the CRC
// seed are in 0x408 and 0x424.
void test_tlsr8258_sends_a_ble_advertisement() {
    const auto system = sim::build_system(sim::parse_system_config(
        "version: 1\nworld:\n  radio:\n    - id: air0\ndevices:\n  - id: d\n    mcus:\n"
        "      - id: m\n        model: tlsr8258\n        firmware: " + idle_image_path() + "\n"
        "        radio: { air: air0, protocols: [ble] }\n"));
    sim::World &world = system->world();
    Tlsr8258 &device = mcu_of(*system).device();

    // ADV_NONCONN_IND, 8 bytes: the advertiser address and two bytes of data
    const std::uint8_t packet[] = {0x0a, 0x00, 0x00, 0x00, 0x02, 0x08, 0xa4, 0xc1, 0x38, 0x01,
                                   0x02, 0x03, 0x02, 0x01};
    for (std::size_t i = 0; i < sizeof packet; ++i)
        device.write8(Tlsr8258::RetainedSramBase + 0x1000 + static_cast<std::uint32_t>(i), packet[i]);
    reg_write(device, 0x40d, 38);
    const std::uint8_t access_code[] = {0x8e, 0x89, 0xbe, 0xd6}; // rf_set_ble_access_code_adv()
    for (unsigned i = 0; i < 4; ++i)
        reg_write(device, 0x408 + i, access_code[i]);
    reg_write(device, 0x424, 0x55);
    reg_write(device, 0x425, 0x55);
    reg_write(device, 0x426, 0x55);

    // The command first, the address of the packet after it
    reg_write(device, 0xf00, 0x85);
    reg_write(device, 0xc0c, 0x00);
    reg_write(device, 0xc0d, 0x10);
    CHECK((reg_read(device, 0xf20) & 0x02) == 0);
    world.run_until(world.now() + 1000000);

    const auto &frames = world.radio("air0").frames();
    CHECK(frames.size() == 1);
    if (frames.size() == 1) {
        CHECK(frames[0].protocol == mcuemu::RadioProtocol::Ble);
        CHECK(frames[0].channel == 38);
        CHECK(frames[0].ble.access_address == 0x8e89bed6u);
        CHECK(frames[0].ble.crc_init == 0x555555u);
        CHECK(frames[0].ble.ll_framed);
        CHECK(frames[0].payload ==
              std::vector<std::uint8_t>(packet + 4, packet + sizeof packet));
    }
    CHECK((reg_read(device, 0xf20) & 0x02) != 0); // TX done

    // The stop command cancels a start that has not happened
    reg_write(device, 0xf00, 0x85);
    reg_write(device, 0xf00, 0x80);
    world.run_until(world.now() + 1000000);
    CHECK(world.radio("air0").frames().size() == 1);
}

namespace {

// A TLSR8258 on the air with a peer node that the test plays itself
struct BleRig {
    std::unique_ptr<sim::System> system;
    sim::World *world;
    Tlsr8258 *device;
    BleRig()
        : system(sim::build_system(sim::parse_system_config(
              "version: 1\nworld:\n  radio:\n    - id: air0\ndevices:\n  - id: d\n    mcus:\n"
              "      - id: m\n        model: tlsr8258\n        firmware: " + idle_image_path() +
              "\n        radio: { air: air0, protocols: [ble] }\n"))),
          world(&system->world()),
          device(&mcu_of(*system).device()) {
        reg_write(*device, 0x40d, 37);
        const std::uint8_t access_code[] = {0x8e, 0x89, 0xbe, 0xd6}; // the advertising one, swapped
        for (unsigned i = 0; i < 4; ++i)
            reg_write(*device, 0x408 + i, access_code[i]);
        reg_write(*device, 0xc08, 0x00); // RX buffer of the DMA channel 2 at 0x841200
        reg_write(*device, 0xc09, 0x12);
    }
    void sram(std::uint32_t offset, const std::vector<std::uint8_t> &bytes) {
        for (std::size_t i = 0; i < bytes.size(); ++i)
            device->write8(Tlsr8258::RetainedSramBase + offset + static_cast<std::uint32_t>(i), bytes[i]);
    }
    std::uint8_t sram(std::uint32_t offset) {
        return device->read8(Tlsr8258::RetainedSramBase + offset);
    }
    void peer_sends(std::uint64_t start_ns, std::vector<std::uint8_t> pdu) {
        mcuemu::BleLinkInfo link;
        link.ll_framed = true;
        world->radio("air0").transmit("peer", {}, mcuemu::RadioProtocol::Ble, 37, start_ns,
                                      std::move(pdu), 0, link);
    }
};

} // namespace

// STX2RX: the packet goes out, the receiver opens 10 us after it, and the packet that a peer sends
// 150 us after the end lands in the RX buffer in the format of the vendor stack
void test_tlsr8258_receives_after_stx2rx() {
    BleRig rig;
    rig.sram(0x1000, {0x0c, 0x00, 0x00, 0x00, 0x00, 0x0a, 0xa4, 0xc1, 0x38, 1, 2, 3, 4, 5, 6, 7, 8});
    reg_write(*rig.device, 0xf00, 0x87);
    reg_write(*rig.device, 0xc0c, 0x00);
    reg_write(*rig.device, 0xc0d, 0x10);
    rig.world->run_until(rig.world->now() + 300000);
    const auto &frames = rig.world->radio("air0").frames();
    CHECK(frames.size() == 1);
    const std::uint64_t tx_end = frames.at(0).end_ns;

    // SCAN_REQ-like PDU: type 0x03, 12 bytes
    const std::vector<std::uint8_t> request = {0x03, 0x0c, 1, 2, 3, 4, 5, 6, 6, 5, 4, 3, 2, 1};
    rig.peer_sends(tx_end + 150000, request);
    CHECK((reg_read(*rig.device, 0xf20) & 0x01) == 0);
    rig.world->run_until(tx_end + 1000000);

    CHECK((reg_read(*rig.device, 0xf20) & 0x01) != 0); // RX flag
    CHECK(rig.sram(0x1200) == 12 + 13);                // DMA length: payload + 13
    CHECK(rig.sram(0x1204) == 0x03);                   // PDU header
    CHECK(rig.sram(0x1205) == 0x0c);                   // payload length
    for (unsigned i = 0; i < 12; ++i)
        CHECK(rig.sram(0x1206 + i) == request[2 + i]);
    CHECK(rig.sram(0x1200 + 12 + 13 + 3) == 0);        // status: CRC good
    CHECK(rig.sram(0x1200 + 12 + 13 + 2) == 60);       // RSSI byte: -50 dBm + 110
    CHECK(frames.size() == 2);                         // no answer: STX2RX leaves it to the firmware
}

// BRX: the slave side of a connection event: it listens, and 150 us after a packet it sends the packet
// of the DMA channel 3 by itself
void test_tlsr8258_ble_slave_answers_a_packet() {
    BleRig rig;
    rig.sram(0x1000, {0x04, 0x00, 0x00, 0x00, 0x01, 0x00}); // an empty data PDU (LLID 1)
    reg_write(*rig.device, 0xf00, 0x82);
    reg_write(*rig.device, 0xc0c, 0x00);
    reg_write(*rig.device, 0xc0d, 0x10);
    rig.world->run_until(rig.world->now() + 100000);
    CHECK(rig.world->radio("air0").frames().empty());

    const std::uint64_t at = rig.world->now() + 50000;
    rig.peer_sends(at, {0x02, 0x00}); // LLID 2, no payload
    rig.world->run_until(at + 1000000);
    const auto &frames = rig.world->radio("air0").frames();
    CHECK(frames.size() == 2);
    CHECK((reg_read(*rig.device, 0xf20) & 0x01) != 0); // received
    if (frames.size() == 2) {
        CHECK(frames[1].source != "peer");
        CHECK(frames[1].start_ns == frames[0].end_ns + 150000); // T_IFS
        // The radio puts SN and NESN into the header: the first packet of the master (SN 0, NESN 0)
        // is new, so NESN becomes 1; it does not acknowledge anything of ours yet, so SN stays 0
        CHECK(frames[1].payload == (std::vector<std::uint8_t>{0x05, 0x00}));
    }
    CHECK((reg_read(*rig.device, 0xf20) & 0x02) != 0); // sent

    // The next event: the master acknowledged our packet (NESN 1) and sends new data (SN 1)
    reg_write(*rig.device, 0xf20, 0x03);
    reg_write(*rig.device, 0xf00, 0x82);
    const std::uint64_t again = rig.world->now() + 50000;
    rig.peer_sends(again, {0x0d, 0x00}); // LLID 1, SN 1, NESN 1
    rig.world->run_until(again + 1000000);
    const auto &all = rig.world->radio("air0").frames();
    CHECK(all.size() == 4);
    if (all.size() == 4) {
        // our SN toggles (acknowledged), the master's SN is the expected one so NESN toggles back
        CHECK(all[3].payload == (std::vector<std::uint8_t>{0x09, 0x00}));
    }
    // The master repeats its last packet (SN 1 again, NESN 1: it did not take ours as new): its SN is
    // not the one we expect, so NESN stays, and it acknowledges nothing, so SN stays
    reg_write(*rig.device, 0xf20, 0x03);
    reg_write(*rig.device, 0xf00, 0x82);
    const std::uint64_t retry = rig.world->now() + 50000;
    rig.peer_sends(retry, {0x0d, 0x00}); // LLID 1, SN 1, NESN 1 as before
    rig.world->run_until(retry + 1000000);
    const auto &after = rig.world->radio("air0").frames();
    CHECK(after.size() == 6);
    if (after.size() == 6)
        CHECK(after[5].payload == (std::vector<std::uint8_t>{0x09, 0x00}));
}

// The window of SRX with a timeout: the RX timeout flag comes when nothing arrives
void test_tlsr8258_ble_receive_times_out() {
    BleRig rig;
    reg_write(*rig.device, 0xf03, 0x04); // RX timeout enabled
    reg_write(*rig.device, 0xf0a, 0x2c); // 300 us
    reg_write(*rig.device, 0xf0b, 0x01);
    reg_write(*rig.device, 0xf00, 0x86);
    rig.world->run_until(rig.world->now() + 200000);
    CHECK((reg_read(*rig.device, 0xf20) & 0x04) == 0);
    rig.world->run_until(rig.world->now() + 400000);
    CHECK((reg_read(*rig.device, 0xf20) & 0x04) != 0);
    // A packet after the window is not received
    rig.peer_sends(rig.world->now() + 10000, {0x03, 0x00});
    rig.world->run_until(rig.world->now() + 500000);
    CHECK((reg_read(*rig.device, 0xf20) & 0x01) == 0);
}

// The vendor stack checks the CRC of every received packet in software, so the radio writes it behind
// the PDU: 0x424 holds CRCInit (here the bytes 5e 55 32 of a CONNECT_IND) and the stack's own copy is
// the bit-reversed value; for the empty data PDU 0d 00 the stack computes 7e 0b 55
void test_tlsr8258_ble_receive_writes_the_crc() {
    BleRig rig;
    reg_write(*rig.device, 0x424, 0x5e);
    reg_write(*rig.device, 0x425, 0x55);
    reg_write(*rig.device, 0x426, 0x32);
    reg_write(*rig.device, 0xf00, 0x86);
    rig.world->run_until(rig.world->now() + 100000);
    rig.peer_sends(rig.world->now() + 50000, {0x0d, 0x00});
    rig.world->run_until(rig.world->now() + 1000000);
    CHECK((reg_read(*rig.device, 0xf20) & 0x01) != 0);
    CHECK(rig.sram(0x1206) == 0x7e);
    CHECK(rig.sram(0x1207) == 0x0b);
    CHECK(rig.sram(0x1208) == 0x55);
}

// 0x754 is two registers: with 0x74c = 0x2c it takes the compare value of the wakeup timer, and with 0x28 (read
// mode) the command 0x08 of 0x74f latches the 32 kHz counter there, which the compare does not hide
void test_tlsr8258_32k_counter_read_mode() {
    BleRig rig;
    reg_write(*rig.device, 0x74c, 0x2c);
    reg_write(*rig.device, 0x754, 0x78);
    reg_write(*rig.device, 0x755, 0x56);
    reg_write(*rig.device, 0x756, 0x34);
    reg_write(*rig.device, 0x757, 0x12);
    CHECK(reg_read(*rig.device, 0x754) == 0x78);
    rig.world->run_until(rig.world->now() + 10000000);
    reg_write(*rig.device, 0x74c, 0x28);
    reg_write(*rig.device, 0x74f, 0x08);
    const std::uint32_t tick = reg_read(*rig.device, 0x754) | (reg_read(*rig.device, 0x755) << 8) |
        (reg_read(*rig.device, 0x756) << 16) | (std::uint32_t{reg_read(*rig.device, 0x757)} << 24);
    CHECK(tick != 0x12345678u);
    CHECK(tick > 0);
    reg_write(*rig.device, 0x74c, 0x2c);
    CHECK(reg_read(*rig.device, 0x754) == 0x78); // the compare value is still there
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
