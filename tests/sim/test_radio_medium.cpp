#include "../test_support.hpp"

#include "sim/config.hpp"
#include "sim/system.hpp"
#include "sim/tlsr8258_mcu.hpp"
#include "mcuemu/radio.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

namespace mcuemu_tests {

namespace {

using mcuemu::Propagation;
using mcuemu::RadioNodeConfig;
using mcuemu::RadioProtocol;
using mcuemu::Tlsr8258;
using mcuemu::VirtualAir;

Propagation log_distance(double sigma = 0.0, std::uint64_t seed = 1) {
    Propagation p;
    p.model = Propagation::Model::LogDistance;
    p.shadowing_sigma_db = sigma;
    p.seed = seed;
    return p;
}

RadioNodeConfig at(double x, double y = 0, double z = 0, double tx = 0) {
    RadioNodeConfig node;
    node.x = x;
    node.y = y;
    node.z = z;
    node.tx_power_dbm = tx;
    return node;
}

std::uint64_t airtime(RadioProtocol protocol, std::size_t bytes) {
    return VirtualAir::airtime_ns(protocol, bytes);
}

const std::filesystem::path &scratch() {
    static const std::filesystem::path dir = [] {
        auto path = std::filesystem::temp_directory_path() / "simcore-radio-tests";
        std::filesystem::create_directories(path);
        std::ofstream out(path / "idle.bin", std::ios::binary);
        out.put(static_cast<char>(0xfe));
        out.put(static_cast<char>(0x87)); // b .
        return path;
    }();
    return dir;
}

std::string write_file(const std::string &name, const std::string &text) {
    const auto path = scratch() / name;
    std::ofstream out(path);
    out << text;
    return path.string();
}

std::string idle() { return (scratch() / "idle.bin").string(); }

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

} // namespace

void test_air_ideal_is_unchanged() {
    VirtualAir air; // ideal by default
    air.set_node("a", at(0));
    air.set_node("b", at(100000)); // 100 km: irrelevant for the ideal model
    air.transmit("a", "", RadioProtocol::Zigbee802154, 15, 0, {1, 2, 3});
    const auto end = airtime(RadioProtocol::Zigbee802154, 3);
    const auto heard = air.receive("b", RadioProtocol::Zigbee802154, 15, end);
    CHECK(heard.size() == 1);
    CHECK(heard[0].rssi_dbm == -50);
    CHECK(air.path_loss_db("a", "b") == 0.0);
    // Same channel, any overlap collides, other protocols never interfere.
    VirtualAir collide;
    collide.transmit("x", "", RadioProtocol::Zigbee802154, 11, 0, {1});
    collide.transmit("y", "", RadioProtocol::Zigbee802154, 11, 100000, {2});
    CHECK(collide.receive("z", RadioProtocol::Zigbee802154, 11, 10000000).empty());
    VirtualAir cross;
    cross.transmit("x", "", RadioProtocol::Zigbee802154, 11, 0, {1});
    cross.transmit("y", "", RadioProtocol::Ble, 0, 0, {2});
    CHECK(cross.receive("z", RadioProtocol::Zigbee802154, 11, 10000000).size() == 1);
}

void test_air_path_loss_and_sensitivity() {
    VirtualAir air;
    air.set_propagation(log_distance());
    air.set_node("tx", at(0));
    air.set_node("near", at(10));
    air.set_node("mid", at(100));
    air.set_node("far", at(1000));
    CHECK(std::fabs(air.path_loss_db("tx", "near") - 60.0) < 1e-9);
    CHECK(std::fabs(air.path_loss_db("tx", "mid") - 80.0) < 1e-9);
    CHECK(air.path_loss_db("tx", "near") == air.path_loss_db("near", "tx"));

    air.transmit("tx", "", RadioProtocol::Zigbee802154, 15, 0, {1, 2, 3, 4, 5});
    const auto end = airtime(RadioProtocol::Zigbee802154, 5);
    const auto near_frames = air.receive("near", RadioProtocol::Zigbee802154, 15, end);
    const auto mid_frames = air.receive("mid", RadioProtocol::Zigbee802154, 15, end);
    CHECK(near_frames.size() == 1 && near_frames[0].rssi_dbm == -60);
    CHECK(mid_frames.size() == 1 && mid_frames[0].rssi_dbm == -80);
    // -100 dBm is below the -97 dBm default sensitivity.
    CHECK(air.receive("far", RadioProtocol::Zigbee802154, 15, end).empty());

    // TX power shifts the received level; a more sensitive receiver hears more.
    VirtualAir loud;
    loud.set_propagation(log_distance());
    loud.set_node("tx", at(0, 0, 0, 10));
    RadioNodeConfig sensitive = at(1000);
    sensitive.sensitivity_dbm = -105;
    loud.set_node("rx", sensitive);
    loud.transmit("tx", "", RadioProtocol::Ble, 37, 0, {1});
    const auto heard = loud.receive("rx", RadioProtocol::Ble, 37,
                                    airtime(RadioProtocol::Ble, 1));
    CHECK(heard.size() == 1 && heard[0].rssi_dbm == -90); // 10 - 100
}

void test_air_sinr_capture_and_cross_protocol() {
    // Capture effect: the stronger of two overlapping frames survives when it
    // beats the other by the SINR threshold; equal frames destroy each other.
    VirtualAir air;
    air.set_propagation(log_distance());
    air.set_node("rx", at(0));
    air.set_node("strong", at(5));
    air.set_node("weak", at(50));
    air.set_node("twin_a", at(0, 20));
    air.set_node("twin_b", at(0, -20));
    air.transmit("strong", "", RadioProtocol::Zigbee802154, 15, 0, {1, 2, 3});
    air.transmit("weak", "", RadioProtocol::Zigbee802154, 15, 20000, {4, 5, 6});
    const auto end = airtime(RadioProtocol::Zigbee802154, 3) + 20000;
    const auto survivors = air.receive("rx", RadioProtocol::Zigbee802154, 15, end);
    CHECK(survivors.size() == 1 && survivors[0].source == "strong");

    VirtualAir equal;
    equal.set_propagation(log_distance());
    equal.set_node("rx", at(0));
    equal.set_node("twin_a", at(0, 20));
    equal.set_node("twin_b", at(0, -20));
    equal.transmit("twin_a", "", RadioProtocol::Zigbee802154, 15, 0, {1});
    equal.transmit("twin_b", "", RadioProtocol::Zigbee802154, 15, 0, {2});
    CHECK(equal.receive("rx", RadioProtocol::Zigbee802154, 15, 10000000).empty());

    // Zigbee channel 11 is 2405 MHz; BLE channel 0 is 2404 MHz (half of its
    // 2 MHz band overlaps) while BLE advertising channel 37 at 2402 MHz does
    // not overlap at all.
    auto zigbee_with_ble = [](std::uint8_t ble_channel) {
        VirtualAir cross;
        cross.set_propagation(log_distance());
        cross.set_node("rx", at(0));
        cross.set_node("zb", at(10));
        cross.set_node("ble", at(1)); // 20 dB closer: far above the threshold
        cross.transmit("zb", "", RadioProtocol::Zigbee802154, 11, 0, {1, 2, 3, 4});
        cross.transmit("ble", "", RadioProtocol::Ble, ble_channel, 50000,
                       std::vector<std::uint8_t>(10, 0));
        return cross.receive("rx", RadioProtocol::Zigbee802154, 11, 10000000).size();
    };
    CHECK(zigbee_with_ble(0) == 0);  // overlapping spectrum: interference
    CHECK(zigbee_with_ble(37) == 1); // clear of it

    // Half duplex: a node that transmits during a frame does not receive it.
    VirtualAir duplex;
    duplex.set_propagation(log_distance());
    duplex.set_node("a", at(0));
    duplex.set_node("b", at(10));
    duplex.transmit("a", "", RadioProtocol::Zigbee802154, 20, 0, {1, 2, 3});
    duplex.transmit("b", "", RadioProtocol::Zigbee802154, 25, 30000, {9});
    CHECK(duplex.receive("b", RadioProtocol::Zigbee802154, 20, 10000000).empty());
}

void test_air_shadowing_and_frequency_map() {
    VirtualAir a, b, c;
    for (VirtualAir *air : {&a, &b, &c}) {
        air->set_node("x", at(0));
        air->set_node("y", at(10));
    }
    a.set_propagation(log_distance(8.0, 1));
    b.set_propagation(log_distance(8.0, 1));
    c.set_propagation(log_distance(8.0, 2));
    CHECK(a.path_loss_db("x", "y") == b.path_loss_db("x", "y")); // deterministic
    CHECK(a.path_loss_db("x", "y") == a.path_loss_db("y", "x")); // symmetric
    CHECK(a.path_loss_db("x", "y") != c.path_loss_db("x", "y")); // seed matters
    CHECK(a.path_loss_db("x", "y") != 60.0);

    CHECK(VirtualAir::frequency_mhz(RadioProtocol::Zigbee802154, 11) == 2405.0);
    CHECK(VirtualAir::frequency_mhz(RadioProtocol::Zigbee802154, 26) == 2480.0);
    CHECK(VirtualAir::frequency_mhz(RadioProtocol::Ble, 37) == 2402.0);
    CHECK(VirtualAir::frequency_mhz(RadioProtocol::Ble, 38) == 2426.0);
    CHECK(VirtualAir::frequency_mhz(RadioProtocol::Ble, 39) == 2480.0);
    CHECK(VirtualAir::frequency_mhz(RadioProtocol::Ble, 0) == 2404.0);
    CHECK(VirtualAir::frequency_mhz(RadioProtocol::Ble, 10) == 2424.0);
    CHECK(VirtualAir::frequency_mhz(RadioProtocol::Ble, 11) == 2428.0);
    CHECK(VirtualAir::frequency_mhz(RadioProtocol::Ble, 36) == 2478.0);
}

void test_radio_system_config() {
    auto build_yaml = [](const std::string &yaml) {
        return error_of([&] { sim::build_system(sim::parse_system_config(yaml)); });
    };
    const std::string head = "version: 1\n";
    const std::string device =
        "devices:\n  - id: d\n    mcus:\n      - {id: m, model: tlsr8258, firmware: " +
        idle() + ", radio: ";
    CHECK(contains(build_yaml(head + device + "{air: nowhere}}\n"),
                   "unknown radio medium 'nowhere'"));
    CHECK(contains(build_yaml(head + "world: { radio: [ {id: a}, {id: a} ] }\n" +
                              device + "{air: a}}\n"),
                   "duplicate radio medium id"));
    CHECK(contains(build_yaml(head + "world: { radio: [ {id: a, model: fancy} ] }\n" +
                              device + "{air: a}}\n"),
                   "ideal or log_distance"));
    CHECK(contains(build_yaml(head + "world: { radio: [ {id: a} ] }\n" + device +
                              "{air: a, position: [1, 2]}}\n"),
                   "[x, y, z]"));
    CHECK(contains(build_yaml(head + "world: { radio: [ {id: a} ] }\n" + device +
                              "{air: a, protocols: [lora]}}\n"),
                   "zigbee or ble"));
    CHECK(contains(build_yaml(head + "world: { radio: [ {id: a} ] }\n" + device +
                              "{air: a, protocols: [zigbee, ble]}}\n"),
                   "'ble' is not connected to the chip's RF registers"));
    CHECK(contains(build_yaml(head + "world: { radio: [ {id: a} ] }\n"
                              "devices:\n  - id: d\n    mcus:\n"
                              "      - {id: m, model: ca51f253l3, firmware: " + idle() +
                              ", radio: {air: a}}\n"),
                   "has no radio model"));
    CHECK(contains(build_yaml(head + "world: { radio: [ {id: a, path_loss_exponent: -1} ] }\n" +
                              device + "{air: a}}\n"),
                   "must be positive"));
    CHECK(build_yaml(head + "world: { radio: [ {id: a, model: log_distance} ] }\n" +
                     device + "{air: a, position: [0, 0, 1.5], tx_power_dbm: 4}}\n")
              .empty());
}

void test_tlsr_rssi_over_distance() {
    const auto system = sim::build_system(sim::parse_system_config(
        "version: 1\n"
        "world:\n  radio:\n"
        "    - { id: air0, model: log_distance, path_loss_exponent: 2.0 }\n"
        "devices:\n"
        "  - id: tx\n    mcus: [ {id: m, model: tlsr8258, firmware: " + idle() +
        ", radio: {air: air0, position: [0, 0, 0], tx_power_dbm: 0}} ]\n"
        "  - id: near\n    mcus: [ {id: m, model: tlsr8258, firmware: " + idle() +
        ", radio: {air: air0, position: [10, 0, 0]}} ]\n"
        "  - id: far\n    mcus: [ {id: m, model: tlsr8258, firmware: " + idle() +
        ", radio: {air: air0, position: [5000, 0, 0]}} ]\n"));
    sim::World &world = system->world();
    auto &sender = static_cast<sim::Tlsr8258Mcu &>(world.mcu("tx.m")).device();
    auto &near_rx = static_cast<sim::Tlsr8258Mcu &>(world.mcu("near.m")).device();
    auto &far_rx = static_cast<sim::Tlsr8258Mcu &>(world.mcu("far.m")).device();

    constexpr std::uint32_t tx_buffer = Tlsr8258::RetainedSramBase + 0x100;
    constexpr std::uint32_t rx_buffer = Tlsr8258::RetainedSramBase + 0x200;
    const std::uint8_t mac[6] = {0x61, 0x88, 0x01, 0x00, 0x00, 0x00};
    sender.write8(tx_buffer, 7);
    sender.write8(tx_buffer + 4, 8);
    for (unsigned i = 0; i < 6; ++i)
        sender.write8(tx_buffer + 5 + i, mac[i]);
    auto set_dma = [](Tlsr8258 &d, std::uint32_t low, std::uint32_t high,
                      std::uint32_t address) {
        d.write8(low, static_cast<std::uint8_t>(address));
        d.write8(low + 1, static_cast<std::uint8_t>(address >> 8));
        d.write8(high, static_cast<std::uint8_t>((address >> 16) & 0x7f));
    };
    set_dma(sender, Tlsr8258::RfTxDmaAddress, Tlsr8258::RfTxDmaAddressHigh, tx_buffer);
    for (Tlsr8258 *rx : {&near_rx, &far_rx}) {
        set_dma(*rx, Tlsr8258::RfRxDmaAddress, Tlsr8258::RfRxDmaAddressHigh, rx_buffer);
        rx->write8(Tlsr8258::RfDmaControl, 0x01);
        rx->write8(Tlsr8258::DmaChannelEnable, Tlsr8258::DmaRfRxEnable);
        rx->write8(Tlsr8258::RfMode, 0x65);
        rx->write8(Tlsr8258::RfIrqMask, 0x03);
    }
    sender.write8(Tlsr8258::RfMode, 0x55);
    sender.write8(Tlsr8258::DmaTxReady0, 0x08);
    world.run_for(2000000);

    CHECK(world.radio("air0").frames().size() == 1);
    CHECK((near_rx.read8(Tlsr8258::RfIrqStatus) & 1u) != 0);
    // 10 m, 0 dBm, free space: -60 dBm, which the SDK reads as byte - 110.
    const unsigned packet_length = 6 + 11;
    CHECK(near_rx.read8(rx_buffer + packet_length + 2) == 50);
    // 5 km is 114 dB of loss: below the sensitivity, nothing arrives.
    CHECK((far_rx.read8(Tlsr8258::RfIrqStatus) & 1u) == 0);

    // The medium shows up as an endpoint.
    CHECK(std::get<std::int64_t>(world.endpoints().at("air0").get()) == 1);
}

void test_lua_radio_parts() {
    auto make = [&](const std::string &adv_x, const std::string &ble_channel) {
        const std::string adv = write_file("adv.lua", R"(
local count = 0
local function beacon()
  count = count + 1
  radio.transmit{protocol = "ble", channel = params.channel,
                 payload = string.rep("b", 12)}
  if count < 3 then after(2 * MS, beacon) end
end
after(params.start, beacon)
)");
        const std::string zb_tx = write_file("zb_tx.lua", R"(
after(100 * US, function()
  radio.transmit{protocol = "zigbee", channel = 11, payload = "hello"}
end)
)");
        const std::string sniffer = write_file("sniffer.lua", R"(
radio.listen(params.protocol, params.channel)
local frames, last_rssi, bytes = 0, 0, ""
function on_frame(f)
  frames = frames + 1
  last_rssi = f.rssi_dbm
  bytes = f.payload
  assert(f.source ~= nil and f.end_ns > f.start_ns)
end
expose("frames", function() return frames end)
expose("rssi", function() return last_rssi end)
expose("payload", function() return bytes end)
)");
        return sim::build_system(sim::parse_system_config(
            "version: 1\nworld:\n  radio:\n"
            "    - { id: air0, model: log_distance }\n"
            "devices:\n  - id: d\n    mcus: [ {id: m, model: tlsr8258, firmware: " + idle() + "} ]\n"
            "    parts:\n"
            "      - id: adv\n        type: script\n        script: " + adv + "\n"
            "        radio: { air: air0, position: [" + adv_x + ", 0, 0], protocols: [ble] }\n"
            "        params: { channel: " + ble_channel + ", start: 150000 }\n"
            "      - id: zb\n        type: script\n        script: " + zb_tx + "\n"
            "        radio: { air: air0, position: [10, 0, 0], protocols: [zigbee] }\n"
            "      - id: ble_sniffer\n        type: script\n        script: " + sniffer + "\n"
            "        radio: { air: air0, position: [10, 0, 0], protocols: [ble] }\n"
            "        params: { protocol: ble, channel: " + ble_channel + " }\n"
            "      - id: zb_sniffer\n        type: script\n        script: " + sniffer + "\n"
            "        radio: { air: air0, position: [11, 0, 0], protocols: [zigbee] }\n"
            "        params: { protocol: zigbee, channel: 11 }\n"));
    };

    // Advertiser at the origin, BLE sniffer 10 m away: three beacons at
    // -60 dBm. Channel 37 (2402 MHz) does not touch Zigbee channel 11.
    {
        const auto system = make("0", "37");
        sim::World &world = system->world();
        world.run_until(10000000);
        auto number = [&](const char *name) {
            return sim::value_as_number(world.endpoints().at(name).get());
        };
        CHECK(number("d.ble_sniffer.frames") == 3);
        CHECK(number("d.ble_sniffer.rssi") == -60);
        CHECK(std::get<std::string>(world.endpoints().at("d.ble_sniffer.payload").get()) ==
              std::string(12, 'b'));
        // The Zigbee node is 1 m from its sniffer: -40 dBm. The radio endpoint
        // saw 4 frames (3 beacons and the Zigbee frame).
        CHECK(number("d.zb_sniffer.frames") == 1);
        CHECK(number("d.zb_sniffer.rssi") == -40);
        CHECK(std::get<std::int64_t>(world.endpoints().at("air0").get()) == 4);
    }
    // Coexistence: the first beacon (150 us) starts inside the Zigbee frame
    // (100 us). On BLE channel 0 (2404 MHz) it overlaps Zigbee channel 11
    // (2405 MHz) by half; placed 1 m from the Zigbee sniffer it is as loud as
    // the Zigbee signal, and with the 0.5 spectral coupling the 3 dB SINR is
    // below the threshold, so the frame is lost. On channel 37 the same
    // advertiser is harmless, and on channel 0 but 3 m away it is tolerated.
    auto zigbee_frames = [&](const std::string &adv_x, const std::string &channel) {
        const auto system = make(adv_x, channel);
        system->world().run_until(10000000);
        return sim::value_as_number(
            system->world().endpoints().at("d.zb_sniffer.frames").get());
    };
    CHECK(zigbee_frames("12", "0") == 0);
    CHECK(zigbee_frames("12", "37") == 1);
    CHECK(zigbee_frames("14", "0") == 1);
}

} // namespace mcuemu_tests
