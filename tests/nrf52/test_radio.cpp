// RADIO in IEEE 802.15.4 mode: two chips on one medium.

#include "../test_support.hpp"
#include "mcuemu/radio.hpp"
#include "nrf52emu/nrf52840.hpp"
#include "programs.hpp"

#include <cstring>

namespace mcuemu_tests {

namespace {

constexpr std::uint32_t Radio = 0x40001000;
constexpr std::uint64_t Us = 64;

struct Node {
    nrf52emu::Nrf52840 chip;

    Node(mcuemu::VirtualAir &air, const std::string &name) {
        chip.load_firmware(nrf52_programs::idle);
        chip.reset();
        chip.connect_radio(air, name);
        write(0x510, 15);        // MODE: IEEE 802.15.4, 250 kbit/s
        write(0x508, 5 + 5 * 4); // FREQUENCY: channel 15 = 2425 MHz
        write(0x66c, 0x14 << 8 | 0); // CCACTRL: ED mode, threshold 20
    }
    void write(std::uint32_t reg, std::uint32_t value) { chip.write(Radio + reg, value, 4); }
    std::uint32_t read(std::uint32_t reg) { return chip.read(Radio + reg, 4); }
    std::uint8_t *ram(std::uint32_t offset) { return chip.ram() + offset; }
};

// Runs both chips to `us` in small slices so the shared medium sees them at
// about the same time, as the world's scheduler does.
void run_both(Node &a, Node &b, std::uint64_t until_us) {
    for (std::uint64_t t = 5; t <= until_us; t += 5) {
        a.chip.run(t * Us);
        b.chip.run(t * Us);
    }
}

} // namespace

void test_nrf52_radio_frame_exchange() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    Node a(air, "a"), b(air, "b");

    // B listens: RXEN, then START and DISABLE on frame end through SHORTS.
    b.write(0x504, 0x20000100);              // PACKETPTR
    b.write(0x200, (1u << 19) | (1u << 1));  // RXREADY_START | END_DISABLE
    b.write(0x004, 1);                       // TASKS_RXEN

    // A sends 5 MAC bytes: PHR counts the two FCS bytes the radio appends.
    const std::uint8_t mac[5] = {0x41, 0x88, 0x07, 0xaa, 0xbb};
    a.ram(0x200)[0] = 7;
    std::memcpy(a.ram(0x201), mac, 5);
    a.write(0x504, 0x20000200);
    a.write(0x200, (1u << 18) | (1u << 20)); // TXREADY_START | PHYEND_DISABLE
    a.write(0x000, 1);                       // TASKS_TXEN

    run_both(a, b, 1500);
    CHECK(a.read(0x16c) == 1);               // A: PHYEND
    CHECK(a.read(0x110) == 1);               // A: DISABLED
    CHECK(a.read(0x550) == 0);               // A: STATE disabled
    CHECK(b.read(0x10c) == 1);               // B: END
    CHECK(b.read(0x130) == 1);               // B: CRCOK
    CHECK(b.read(0x110) == 1);               // B: DISABLED
    CHECK(b.read(0x400) == 1);               // CRCSTATUS
    CHECK(b.ram(0x100)[0] == 7);
    CHECK(std::memcmp(b.ram(0x101), mac, 5) == 0);
    // The FCS is the CRC-16 of the frame (CRC-16/KERMIT of this sample).
    const std::uint16_t crc = static_cast<std::uint16_t>(b.ram(0x106)[0] | (b.ram(0x107)[0] << 8));
    CHECK(crc == b.read(0x40c));
    CHECK(b.ram(0x108)[0] > 0 && b.ram(0x108)[0] <= 63); // the ED/LQI byte follows the FCS
    // One frame reached the medium, 7 + 6 bytes at 32 us per byte on air.
    CHECK(air.frames().size() == 1);
    CHECK(air.frames()[0].channel == 15 && air.frames()[0].payload.size() == 5);
}

// ADDRESS (and SYNC, FRAMESTART) come when the preamble and SFD have gone by, not
// when the frame ends: the Nordic driver stops its ACK timeout on ADDRESS, and
// an ACK that is only "heard" at its end arrives after that timeout has fired.
void test_nrf52_radio_address_event_comes_before_the_frame_ends() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    Node a(air, "a"), b(air, "b");
    b.write(0x504, 0x20000100);
    b.write(0x200, (1u << 19) | (1u << 1));
    b.write(0x004, 1);
    const std::uint8_t mac[5] = {0x41, 0x88, 0x07, 0xaa, 0xbb};
    a.ram(0x200)[0] = 7;
    std::memcpy(a.ram(0x201), mac, 5);
    a.write(0x504, 0x20000200);
    a.write(0x200, (1u << 18) | (1u << 20));
    a.write(0x000, 1);

    std::uint64_t address_us = 0;
    for (std::uint64_t t = 5; t <= 1500 && address_us == 0; t += 5) {
        a.chip.run(t * Us);
        b.chip.run(t * Us);
        if (b.read(0x104) == 1)
            address_us = t;
    }
    CHECK(address_us != 0);
    CHECK(b.read(0x10c) == 0);               // END: the frame is still on the air
    CHECK(b.read(0x168) == 1);               // SYNC
    CHECK(b.read(0x138) == 1);               // FRAMESTART
    CHECK(air.frames().size() == 1);
    CHECK(address_us * 1000 < air.frames()[0].end_ns);
    b.write(0x104, 0);                       // clear ADDRESS: it must not come again at the end
    run_both(a, b, 1500);
    CHECK(b.read(0x10c) == 1);               // END
    CHECK(b.read(0x130) == 1);               // CRCOK
    CHECK(b.read(0x104) == 0);
}

void test_nrf52_radio_cca_sees_a_frame_on_air() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    Node a(air, "a"), b(air, "b");

    // A transmits a long frame without CCA.
    a.ram(0x200)[0] = 100;
    a.write(0x504, 0x20000200);
    a.write(0x200, (1u << 18) | (1u << 20));
    a.write(0x000, 1);
    // B runs CCA while it is on air (RXREADY_CCASTART, CCABUSY_DISABLE).
    b.write(0x504, 0x20000100);
    b.write(0x200, (1u << 11) | (1u << 13));
    b.write(0x004, 1);
    run_both(a, b, 1500);
    CHECK(b.read(0x148) == 1);               // CCABUSY
    CHECK(b.read(0x144) == 0);               // not CCAIDLE
    CHECK(b.read(0x110) == 1);               // DISABLED by the short

    // On an idle channel the CCA finds it idle and the short starts TX.
    mcuemu::VirtualAir quiet;
    quiet.set_node("c", {});
    Node c(quiet, "c");
    c.ram(0x200)[0] = 5;
    c.write(0x504, 0x20000200);
    c.write(0x200, (1u << 11) | (1u << 12) | (1u << 18) | (1u << 20)); // CCA-gated TX
    c.write(0x004, 1);
    for (std::uint64_t t = 5; t <= 1500; t += 5)
        c.chip.run(t * Us);
    CHECK(c.read(0x144) == 1);               // CCAIDLE
    CHECK(c.read(0x16c) == 1);               // PHYEND: the frame went out
    CHECK(quiet.frames().size() == 1);
}

void test_nrf52_radio_ramp_and_state() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    Node a(air, "a");
    CHECK(a.read(0x550) == 0);
    a.write(0x004, 1);                       // RXEN
    CHECK(a.read(0x550) == 1);               // RXRU
    a.chip.run(100 * Us);
    CHECK(a.read(0x100) == 0);               // READY after 130 us
    a.chip.run(140 * Us);
    CHECK(a.read(0x100) == 1 && a.read(0x158) == 1); // READY and RXREADY
    CHECK(a.read(0x550) == 2);               // RXIDLE
    a.write(0x010, 1);                       // DISABLE
    a.chip.run(160 * Us);
    CHECK(a.read(0x110) == 1 && a.read(0x550) == 0);
}

} // namespace mcuemu_tests
