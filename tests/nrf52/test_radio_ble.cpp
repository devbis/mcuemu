// RADIO in Bluetooth LE mode: two chips on one medium, driven through registers.

#include "../test_support.hpp"
#include "ble/ll.hpp"
#include "mcuemu/radio.hpp"
#include "nrf52emu/nrf52840.hpp"
#include "programs.hpp"

#include <cstring>

namespace mcuemu_tests {

namespace {

constexpr std::uint32_t Radio = 0x40001000;
constexpr std::uint64_t Us = 64;

// SHORTS bits.
constexpr std::uint32_t READY_START = 1u << 0, END_DISABLE = 1u << 1, DISABLED_TXEN = 1u << 2,
                        DISABLED_RXEN = 1u << 3, ADDRESS_RSSISTART = 1u << 4,
                        ADDRESS_BCSTART = 1u << 6;

struct BleNode {
    nrf52emu::Nrf52840 chip;

    BleNode(mcuemu::VirtualAir &air, const std::string &name, std::uint32_t access_address,
            std::uint32_t crc_init = 0x555555, unsigned frequency = 2) {
        chip.load_firmware(nrf52_programs::idle);
        chip.reset();
        chip.connect_radio(air, name);
        write(0x510, 3);                  // MODE: Ble_1Mbit
        write(0x508, frequency);          // FREQUENCY: 2 = 2402 MHz = advertising channel 37
        write(0x514, 8 | (1u << 8));      // PCNF0: LFLEN 8 bits, S0LEN 1 byte
        write(0x518, 255 | (3u << 16) | (1u << 25)); // PCNF1: MAXLEN 255, BALEN 3, WHITEEN
        write(0x51c, access_address << 8);           // BASE0
        write(0x524, access_address >> 24);          // PREFIX0.AP0
        write(0x52c, 0);                  // TXADDRESS: logical 0
        write(0x530, 1);                  // RXADDRESSES: logical 0
        write(0x534, 3 | (1u << 8));      // CRCCNF: 3 bytes, skip the address
        write(0x538, 0x00065B);           // CRCPOLY
        write(0x53c, crc_init);           // CRCINIT
        write(0x544, 150);                // TIFS
        write(0x554, 37 | 0x40);          // DATAWHITEIV
    }
    void write(std::uint32_t reg, std::uint32_t value) { chip.write(Radio + reg, value, 4); }
    std::uint32_t read(std::uint32_t reg) { return chip.read(Radio + reg, 4); }
    std::uint8_t *ram(std::uint32_t offset) { return chip.ram() + offset; }
    void put(std::uint32_t offset, const std::vector<std::uint8_t> &pdu) {
        std::memcpy(ram(offset), pdu.data(), pdu.size());
    }
};

void run_both(BleNode &a, BleNode &b, std::uint64_t until_us) {
    for (std::uint64_t t = 5; t <= until_us; t += 5) {
        a.chip.run(t * Us);
        b.chip.run(t * Us);
    }
}

constexpr std::uint32_t kAdv = 0x8E89BED6u;
const std::vector<std::uint8_t> kAdvPdu = {0x42, 0x09, 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6, 0x02, 0x01, 0x06};

// A transmits kAdvPdu at 0x20000200; B receives into 0x20000100.
void arm_exchange(BleNode &a, BleNode &b) {
    b.write(0x504, 0x20000100);
    b.write(0x200, READY_START | END_DISABLE | ADDRESS_RSSISTART);
    b.write(0x004, 1); // RXEN
    a.put(0x200, kAdvPdu);
    a.write(0x504, 0x20000200);
    a.write(0x200, READY_START | END_DISABLE);
    a.write(0x000, 1); // TXEN
}

} // namespace

void test_nrf52_ble_advertising_frame_exchange() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    BleNode a(air, "a", kAdv), b(air, "b", kAdv);
    arm_exchange(a, b);

    std::uint64_t address_us = 0, payload_us = 0, end_us = 0;
    for (std::uint64_t t = 5; t <= 700; t += 5) {
        a.chip.run(t * Us);
        b.chip.run(t * Us);
        if (!address_us && b.read(0x104)) address_us = t;
        if (!payload_us && b.read(0x108)) payload_us = t;
        if (!end_us && b.read(0x10c)) end_us = t;
    }
    CHECK(air.frames().size() == 1);
    const mcuemu::RadioFrame &frame = air.frames()[0];
    CHECK(frame.channel == 37 && frame.ble.access_address == kAdv && frame.ble.ll_framed);
    CHECK(frame.ble.crc_init == 0x555555u);
    CHECK(frame.payload == kAdvPdu);
    // 1 (preamble) + 4 (address) + 11 (PDU) + 3 (CRC) bytes at 8 us.
    CHECK(frame.end_ns - frame.start_ns == (1 + 4 + 11 + 3) * 8 * 1000u);
    // A started after the 130 us ramp-up (no TIFS applies to the first packet).
    CHECK(frame.start_ns >= 130000 && frame.start_ns < 140000);

    // The events come at their points in the packet, not all at the end.
    const std::uint64_t start_us = frame.start_ns / 1000, end_air_us = frame.end_ns / 1000;
    CHECK(address_us >= start_us + 40 && address_us <= start_us + 40 + 20);
    CHECK(payload_us > address_us && payload_us <= end_air_us - 24 + 5);
    CHECK(end_us >= end_air_us && end_us <= end_air_us + 20);
    CHECK(b.read(0x130) == 1 && b.read(0x134) == 0); // CRCOK
    CHECK(b.read(0x400) == 1);                       // CRCSTATUS
    CHECK(b.read(0x110) == 1);                       // DISABLED through END_DISABLE
    CHECK(b.read(0x408) == 0);                       // RXMATCH: logical address 0
    CHECK(b.read(0x550) == 0);
    // The PDU landed in RAM as S0 | LENGTH | payload.
    CHECK(std::memcmp(b.ram(0x100), kAdvPdu.data(), kAdvPdu.size()) == 0);
    // RXCRC is the CRC the sender appended.
    const auto crc = ble::crc24(0x555555, kAdvPdu);
    CHECK(b.read(0x40c) == (crc[0] | crc[1] << 8 | crc[2] << 16));
    // ADDRESS_RSSISTART sampled the signal: -50 dBm on the ideal medium.
    CHECK(b.read(0x11c) == 1 && b.read(0x548) == 50);
    // The sender saw its own packet's events.
    CHECK(a.read(0x104) == 1 && a.read(0x108) == 1 && a.read(0x10c) == 1 && a.read(0x110) == 1);
    CHECK(a.read(0x550) == 0);
}

void test_nrf52_ble_wrong_crc_init_and_wrong_address() {
    // A receiver seeded differently sees ADDRESS and END but a bad CRC.
    {
        mcuemu::VirtualAir air;
        air.set_node("a", {});
        air.set_node("b", {});
        BleNode a(air, "a", kAdv, 0xA1B2C3), b(air, "b", kAdv, 0x555555);
        arm_exchange(a, b);
        run_both(a, b, 700);
        CHECK(b.read(0x104) == 1 && b.read(0x10c) == 1);
        CHECK(b.read(0x130) == 0 && b.read(0x134) == 1); // CRCERROR
        CHECK(b.read(0x400) == 0);
        CHECK(b.read(0x110) == 1);                       // END_DISABLE still disables
    }
    // A different access address is not even noticed.
    {
        mcuemu::VirtualAir air;
        air.set_node("a", {});
        air.set_node("b", {});
        BleNode a(air, "a", kAdv), b(air, "b", 0x71764129u);
        arm_exchange(a, b);
        run_both(a, b, 700);
        CHECK(a.read(0x10c) == 1);
        CHECK(b.read(0x104) == 0 && b.read(0x10c) == 0 && b.read(0x130) == 0 && b.read(0x134) == 0);
        CHECK(b.read(0x550) == 3); // still receiving
    }
    // So is another channel.
    {
        mcuemu::VirtualAir air;
        air.set_node("a", {});
        air.set_node("b", {});
        BleNode a(air, "a", kAdv, 0x555555, 2), b(air, "b", kAdv, 0x555555, 26);
        arm_exchange(a, b);
        run_both(a, b, 700);
        CHECK(air.frames()[0].channel == 37);
        CHECK(b.read(0x104) == 0 && b.read(0x10c) == 0);
    }
}

// RX then TX exactly TIFS after the end of the received packet, using only
// SHORTS: END_DISABLE, DISABLED_TXEN, READY_START (the ramp-up takes less
// than TIFS, so the radio waits).
void test_nrf52_ble_tifs_reply() {
    mcuemu::VirtualAir air;
    air.set_node("central", {});
    air.set_node("peripheral", {});
    constexpr std::uint32_t aa = 0x71764129u;
    BleNode central(air, "central", aa), peripheral(air, "peripheral", aa);

    // The peripheral listens and echoes whatever it received.
    peripheral.write(0x504, 0x20000100);
    peripheral.write(0x200, READY_START | END_DISABLE | DISABLED_TXEN);
    peripheral.write(0x004, 1);
    // The central sends, then listens for the reply (DISABLED_RXEN, READY_START).
    const std::vector<std::uint8_t> pdu = {0x02, 0x03, 0xAA, 0xBB, 0xCC};
    central.put(0x200, pdu);
    central.write(0x504, 0x20000200);
    central.write(0x200, READY_START | END_DISABLE | DISABLED_RXEN);
    central.write(0x000, 1);
    // The central's receive buffer is separate, set once its transmit is done.
    std::uint64_t t = 5;
    for (; t <= 1500 && !central.read(0x10c); t += 5) {
        central.chip.run(t * Us);
        peripheral.chip.run(t * Us);
    }
    CHECK(central.read(0x10c) == 1); // END of its own transmission
    central.write(0x10c, 0);
    central.write(0x504, 0x20000300);
    // Let the exchange run until the reply is on the air, then stop the loop:
    // the shorts would otherwise keep the two radios answering each other.
    for (; t <= 2000 && air.frames().size() < 2; t += 5) {
        central.chip.run(t * Us);
        peripheral.chip.run(t * Us);
    }
    peripheral.write(0x200, END_DISABLE);
    central.write(0x200, END_DISABLE);
    for (const std::uint64_t until = t + 400; t <= until; t += 5) {
        central.chip.run(t * Us);
        peripheral.chip.run(t * Us);
    }
    CHECK(air.frames().size() == 2);
    if (air.frames().size() == 2) {
        // The reply starts exactly 150 us after the first packet ended.
        CHECK(air.frames()[1].start_ns == air.frames()[0].end_ns + 150000);
        CHECK(air.frames()[1].source == "peripheral" && air.frames()[1].payload == pdu);
        CHECK(air.frames()[1].ble.access_address == aa);
    }
    // The central heard the echo.
    CHECK(central.read(0x130) == 1);
    CHECK(std::memcmp(central.ram(0x300), pdu.data(), pdu.size()) == 0);
}

void test_nrf52_ble_device_match_and_bit_counter() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    BleNode a(air, "a", kAdv), b(air, "b", kAdv);
    // Entry 3 holds the advertiser address 11:22:33:44:55:C6 (TxAdd = random).
    b.write(0x600 + 4 * 3, 0x44332211);
    b.write(0x620 + 4 * 3, 0xC655);
    b.write(0x640, (1u << 3) | (1u << (8 + 3))); // DACNF: ENA3, TXADD3
    b.write(0x560, 24);                          // BCC: 24 bits
    arm_exchange(a, b);
    b.write(0x200, READY_START | END_DISABLE | ADDRESS_BCSTART);
    run_both(a, b, 700);
    CHECK(b.read(0x114) == 1 && b.read(0x118) == 0); // DEVMATCH
    CHECK(b.read(0x410) == 3);                       // DAI
    CHECK(b.read(0x128) == 1);                       // BCMATCH
    CHECK(b.read(0x130) == 1);

    // The same packet from a non-random address (TxAdd 0) does not match.
    mcuemu::VirtualAir air2;
    air2.set_node("a", {});
    air2.set_node("b", {});
    BleNode a2(air2, "a", kAdv), b2(air2, "b", kAdv);
    b2.write(0x600 + 4 * 3, 0x44332211);
    b2.write(0x620 + 4 * 3, 0xC655);
    b2.write(0x640, (1u << 3) | (1u << (8 + 3)));
    std::vector<std::uint8_t> other = kAdvPdu;
    other[0] &= ~0x40;
    b2.write(0x504, 0x20000100);
    b2.write(0x200, READY_START | END_DISABLE);
    b2.write(0x004, 1);
    a2.put(0x200, other);
    a2.write(0x504, 0x20000200);
    a2.write(0x200, READY_START | END_DISABLE);
    a2.write(0x000, 1);
    run_both(a2, b2, 700);
    CHECK(b2.read(0x114) == 0 && b2.read(0x118) == 1); // DEVMISS
    // A bit counter longer than the packet never matches.
    mcuemu::VirtualAir air3;
    air3.set_node("a", {});
    air3.set_node("b", {});
    BleNode a3(air3, "a", kAdv), b3(air3, "b", kAdv);
    b3.write(0x560, 5000);
    arm_exchange(a3, b3);
    b3.write(0x200, READY_START | END_DISABLE | ADDRESS_BCSTART);
    run_both(a3, b3, 700);
    CHECK(b3.read(0x128) == 0 && b3.read(0x10c) == 1);
}

void test_nrf52_ble_2m_phy_and_invalid_channel() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    BleNode a(air, "a", kAdv), b(air, "b", kAdv);
    a.write(0x510, 4); // Ble_2Mbit
    b.write(0x510, 4);
    arm_exchange(a, b);
    run_both(a, b, 700);
    CHECK(air.frames().size() == 1);
    // 2-byte preamble + 4 + 11 + 3 bytes at 4 us.
    CHECK(air.frames()[0].end_ns - air.frames()[0].start_ns == (2 + 4 + 11 + 3) * 8 * 500u);
    CHECK(b.read(0x10c) == 1 && b.read(0x130) == 1);

    // 2403 MHz is not a BLE channel: the state machine runs, nothing is sent.
    mcuemu::VirtualAir quiet;
    quiet.set_node("a", {});
    BleNode c(quiet, "c", kAdv, 0x555555, 3);
    c.put(0x200, kAdvPdu);
    c.write(0x504, 0x20000200);
    c.write(0x200, READY_START | END_DISABLE);
    c.write(0x000, 1);
    for (std::uint64_t t = 5; t <= 700; t += 5)
        c.chip.run(t * Us);
    CHECK(quiet.frames().empty());
    CHECK(c.read(0x10c) == 1 && c.read(0x110) == 1);
}

void test_nrf52_ble_disable_abandons_a_packet_in_progress() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    BleNode a(air, "a", kAdv), b(air, "b", kAdv);
    arm_exchange(a, b);
    b.write(0x200, READY_START); // no END_DISABLE: the firmware disables by hand
    std::uint64_t t = 5;
    for (; t <= 700 && !b.read(0x104); t += 5) {
        a.chip.run(t * Us);
        b.chip.run(t * Us);
    }
    CHECK(b.read(0x104) == 1 && b.read(0x10c) == 0);
    b.write(0x010, 1); // DISABLE while the frame is still on the air
    for (; t <= 900; t += 5) {
        a.chip.run(t * Us);
        b.chip.run(t * Us);
    }
    CHECK(b.read(0x110) == 1);
    CHECK(b.read(0x10c) == 0 && b.read(0x130) == 0 && b.read(0x108) == 0);
    CHECK(b.read(0x550) == 0);
}

} // namespace mcuemu_tests
