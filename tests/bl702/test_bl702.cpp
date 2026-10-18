// The BL702 chip model: the boot header, the peripherals the sensor firmware drives, and the IEEE 802.15.4
// MAC on the shared air. Programs are a `wfi` loop (time passes, the blocks are serviced) or a few
// hand-encoded instructions.

#include "../test_support.hpp"
#include "bl702emu/bl702.hpp"
#include "mcuemu/radio.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

namespace mcuemu_tests {

namespace {
using bl702emu::Bl702;

constexpr std::uint32_t Wfi = 0x10500073;    // wfi
constexpr std::uint32_t JumpBack4 = 0xffdff06f; // j -4
constexpr std::uint32_t Ns1Ms = 1000000;
constexpr std::uint32_t CodeOffset = 0x100;  // the code in flash, XIP at Bl702::FlashBase + CodeOffset
constexpr std::uint32_t BootHeaderLength = 176;

void put32(std::vector<std::uint8_t> &bytes, std::size_t offset, std::uint32_t value) {
    if (bytes.size() < offset + 4)
        bytes.resize(offset + 4, 0);
    std::memcpy(&bytes[offset], &value, 4);
}

// A flash image: the boot header (flags, segment count, entry) and `code` at CodeOffset.
std::vector<std::uint8_t> image(std::uint32_t flags, std::uint32_t entry, std::vector<std::uint32_t> code = {Wfi, JumpBack4}) {
    std::vector<std::uint8_t> bytes(BootHeaderLength, 0);
    put32(bytes, 0, 0x504e4642); // "BFNP"
    put32(bytes, 116, flags);
    put32(bytes, 124, entry);
    bytes.resize(CodeOffset, 0);
    for (std::uint32_t word : code)
        put32(bytes, bytes.size(), word);
    return bytes;
}

std::uint32_t read32(Bl702 &chip, std::uint32_t address) { return chip.read(address, 4); }
void write32(Bl702 &chip, std::uint32_t address, std::uint32_t value) { chip.write(address, value, 4); }

constexpr std::uint32_t Entry = Bl702::FlashBase + CodeOffset;
constexpr std::uint32_t NoSegment = 1u << 8;
constexpr std::uint32_t M154 = 0x4c000000;
} // namespace

void test_bl702_boot_header_loads_segments_and_rejects_no_header() {
    // A header with one segment: 8 bytes that load into SRAM at 0x42010100. The code stays at CodeOffset.
    const std::uint8_t data[8] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};
    std::vector<std::uint8_t> bytes(BootHeaderLength, 0);
    put32(bytes, 0, 0x504e4642); // "BFNP"
    put32(bytes, 120, 1);        // segment count
    put32(bytes, 124, Entry);
    put32(bytes, BootHeaderLength, 0x42010100);
    put32(bytes, BootHeaderLength + 4, sizeof data);
    bytes.resize(BootHeaderLength + 16, 0); // the segment header is 16 bytes
    bytes.insert(bytes.end(), data, data + sizeof data);
    bytes.resize(CodeOffset, 0);
    put32(bytes, bytes.size(), Wfi);
    put32(bytes, bytes.size(), JumpBack4);

    Bl702 chip;
    chip.load_flash(bytes);
    chip.reset();
    std::uint8_t loaded[8] = {};
    CHECK(chip.core().read_memory(0x42010100, loaded, sizeof loaded));
    CHECK(std::memcmp(loaded, data, sizeof data) == 0);
    CHECK(chip.core().pc() == Entry);

    // An XIP segment: its load address is in the XIP window, its data follows the header in the flash. The
    // window maps the data there, so the entry runs from the flash.
    std::vector<std::uint8_t> xip(BootHeaderLength, 0);
    put32(xip, 0, 0x504e4642);
    put32(xip, 120, 1);
    put32(xip, 124, Bl702::FlashBase);
    put32(xip, BootHeaderLength, Bl702::FlashBase);
    put32(xip, BootHeaderLength + 4, 8);
    xip.resize(BootHeaderLength + 16, 0);
    put32(xip, xip.size(), Wfi);
    put32(xip, xip.size(), JumpBack4);
    Bl702 mapped;
    mapped.load_flash(xip);
    mapped.reset();
    std::uint8_t code[8] = {};
    CHECK(mapped.core().read_memory(Bl702::FlashBase, code, sizeof code));
    CHECK(std::memcmp(code, &xip[BootHeaderLength + 16], sizeof code) == 0);
    CHECK(mapped.run_until(100000).reason == riscv::StopReason::Deadline);

    Bl702 blank;
    blank.load_flash(std::vector<std::uint8_t>(BootHeaderLength, 0));
    bool refused = false;
    try {
        blank.reset();
    } catch (const std::runtime_error &) {
        refused = true;
    }
    CHECK(refused);
}

void test_bl702_uart_and_timer() {
    Bl702 chip;
    chip.load_flash(image(NoSegment, Entry));
    chip.reset();

    std::string text;
    chip.on_uart_tx = [&text](std::uint8_t byte) { text.push_back(static_cast<char>(byte)); };
    write32(chip, 0x4000a088, 'H');
    write32(chip, 0x4000a088, 'i');
    CHECK(text == "Hi");
    CHECK((read32(chip, 0x4000a084) & 0xff) != 0); // room in the transmit FIFO

    // TIMER0 at the SDK's 1 MHz setting: divider 31, counter enabled.
    const std::uint8_t zero = 0;
    CHECK(chip.core().write_memory(Bl702::ClicBase + 3, &zero, 1)); // the firmware clears an interrupt register
    write32(chip, 0x4000a5bc, 31u << 8);
    write32(chip, 0x4000a584, 1u << 1);
    chip.run_until(Ns1Ms);
    const std::uint32_t ticks = read32(chip, 0x4000a52c);
    CHECK(ticks >= 999 && ticks <= 1001);
}

void test_bl702_efuse_mac_and_aes_block() {
    Bl702::Config config;
    config.mac = {1, 2, 3, 4, 5, 6, 7, 8};
    Bl702 chip(config);
    chip.load_flash(image(NoSegment, Entry));
    chip.reset();
    // The SDK reads the chip ID from two little-endian words.
    CHECK(read32(chip, 0x40007014) == 0x04030201);
    CHECK(read32(chip, 0x40007018) == 0x08070605);

    // FIPS-197 C.1: AES-128, key 00..0f, plaintext 00112233..ff.
    std::uint8_t plain[16], key[16];
    for (unsigned i = 0; i < 16; ++i) {
        key[i] = static_cast<std::uint8_t>(i);
        plain[i] = static_cast<std::uint8_t>(i * 0x11);
    }
    CHECK(chip.core().write_memory(0x42010200, plain, sizeof plain));
    for (unsigned i = 0; i < 4; ++i) {
        std::uint32_t word;
        std::memcpy(&word, key + 4 * i, 4);
        write32(chip, 0x40004120 + 4 * i, word);
    }
    write32(chip, 0x40004104, 0x42010200); // MSA
    write32(chip, 0x40004108, 0x42010300); // MDA
    write32(chip, 0x40004100, (1u << 16) | (1u << 2) | (1u << 1)); // one block, enable, trigger
    CHECK((read32(chip, 0x40004100) & 1) == 0); // not busy, trigger cleared

    const std::uint8_t expected[16] = {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
                                       0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a};
    std::uint8_t cipher[16] = {};
    CHECK(chip.core().read_memory(0x42010300, cipher, sizeof cipher));
    CHECK(std::memcmp(cipher, expected, sizeof expected) == 0);
}

void test_bl702_radio_transmits_and_receives_on_the_air() {
    mcuemu::VirtualAir air;
    Bl702 chip;
    chip.connect_radio(air, "bl702");
    chip.load_flash(image(NoSegment, Entry));
    chip.reset();

    // Transmit five bytes: the buffer, PHR = length + 2 (the hardware adds the FCS), then the trigger.
    write32(chip, M154 + 0x200, 0x04030201);
    write32(chip, M154 + 0x204, 0x00000005);
    write32(chip, M154 + 0x2fc, (5 + 2) << 24);
    write32(chip, M154 + 0x0b4, 1);
    CHECK(read32(chip, M154 + 0x0b0) & (1u << 11)); // busy
    chip.run_until(Ns1Ms);
    CHECK(read32(chip, M154 + 0x030) & 0x1000'0000); // TX finished
    CHECK(!(read32(chip, M154 + 0x0b0) & (1u << 11)));
    write32(chip, M154 + 0x024, 0x1000'0000);       // write-one-to-clear
    CHECK(!(read32(chip, M154 + 0x030) & 0x1000'0000));
    CHECK(air.frames().size() == 1);
    if (air.frames().size() == 1) {
        CHECK(air.frames()[0].source == "bl702");
        CHECK(air.frames()[0].payload == std::vector<std::uint8_t>({1, 2, 3, 4, 5}));
    }

    // Receive: another node sends three bytes once the chip listens.
    write32(chip, M154 + 0x0e4, 1u << 12);
    air.transmit("other", {}, mcuemu::RadioProtocol::Zigbee802154, 11, chip.core().cycles_to_ns(chip.core().cycles()),
                 {0xaa, 0xbb, 0xcc});
    chip.run_until(2 * Ns1Ms);
    CHECK(read32(chip, M154 + 0x030) & 0x0100'0000); // RX done
    CHECK((read32(chip, M154 + 0x4fc) >> 24) == 3 + 2);
    CHECK((read32(chip, M154 + 0x400) & 0xffffff) == 0x00ccbbaa);
    CHECK(((read32(chip, 0x40001860) >> 8) & 0xff) == 55); // raw RSSI of -50 dBm: 5 - rssi
}

void test_bl702_cca_sees_a_frame_on_the_air() {
    mcuemu::VirtualAir air;
    Bl702 chip;
    chip.connect_radio(air, "bl702");
    chip.load_flash(image(NoSegment, Entry));
    chip.reset();

    // The driver's CCA defaults: CCA mode 2 and a threshold of 76 (dBm, negated).
    write32(chip, 0x40001834, (2u << 27) | (76u << 16));
    air.transmit("other", {}, mcuemu::RadioProtocol::Zigbee802154, 11, 0, {1, 2, 3, 4, 5});
    chip.run_until(100000); // 100 us into the frame (it lasts 416 us)

    auto assess = [&chip] {
        write32(chip, 0x40001834, read32(chip, 0x40001834) | (1u << 1)); // start, then clear
        write32(chip, 0x40001834, read32(chip, 0x40001834) & ~(1u << 1));
        write32(chip, 0x4000182c, 1u << 27);
        return read32(chip, 0x40001834);
    };
    const std::uint32_t busy = assess();
    CHECK(busy & (1u << 2));
    CHECK(busy & (1u << 7));
    CHECK(((busy >> 8) & 0x7f) == 55); // -50 dBm from the other node
    write32(chip, 0x4000182c, 0);

    chip.run_until(Ns1Ms); // the frame is over
    const std::uint32_t idle = assess();
    CHECK(idle & (1u << 2));
    CHECK(!(idle & (1u << 7)));
    CHECK(((idle >> 8) & 0x7f) == 105); // the noise floor, -100 dBm
}

void test_bl702_rom_flash_routines() {
    // The guest calls the ROM's XIP_WRITE (API entry 79) through the table to program four bytes at flash
    // offset 0x1000 from SRAM 0x42010400, then waits. Assembled from:
    //   li a0,0; li a1,0; li a2,0x1000; li a3,0x42010400; li a4,4; li t0,0x21018800
    //   lw t0,316(t0); jalr ra,0(t0); loop: wfi; j loop
    const std::vector<std::uint32_t> code = {0x513,        0x593,      0x1637,     0x420106b7,
                                             0x40068693,   0x400713,   0x210192b7, 0x80028293,
                                             0x13c2a283,   0x280e7,    Wfi,        JumpBack4};
    Bl702 chip;
    chip.load_flash(image(NoSegment, Entry, code));
    chip.reset();
    const std::uint8_t data[4] = {0x78, 0x56, 0x34, 0x12};
    CHECK(chip.core().write_memory(0x42010400, data, sizeof data));
    chip.run_until(100000);
    CHECK(std::memcmp(chip.flash() + 0x1000, data, sizeof data) == 0);
    CHECK(chip.core().reg(10) == 0); // the ROM's return value
}

void test_bl702_clic_takes_the_timer_interrupt() {
    // A program on the CLIC: mtvec in mode 3 (base 0x23000200), CLIC interrupt 7 (the machine timer) enabled at
    // level 0x10, mtimecmp = 1000 us, MIE, then wfi. The handler claims the interrupt with mnxti, moves the compare
    // out of the way, counts the entry at SRAM 0x42010500 and returns with mret. Assembled from clic.s and handler.s.
    const std::vector<std::uint32_t> main = {
        0x230002b7, 0x20328293, 0x30529073, 0x028002b7, 0x40728293, 0x00100313, 0x00628023, 0x028012b7,
        0x80728293, 0x01000313, 0x00628023, 0x020042b7, 0xfff00313, 0x0062a023, 0x0062a223, 0x3e800313,
        0x0062a023, 0x0002a223, 0x30046073, 0x10500073, 0xffdff06f};
    const std::vector<std::uint32_t> handler = {
        0x34502573, 0x02004337, 0xfff00393, 0x00732023, 0x00732223, 0x42010e37, 0x500e0e13, 0x000e2e83,
        0x001e8e93, 0x01de2023, 0x30200073};
    std::vector<std::uint32_t> code = main;
    code.resize(0x100 / 4, 0); // up to the handler at CodeOffset + 0x100
    code.insert(code.end(), handler.begin(), handler.end());

    Bl702 chip;
    chip.load_flash(image(NoSegment, Entry, code));
    chip.reset();
    chip.run_until(3 * Ns1Ms);
    std::uint8_t counter[4] = {};
    CHECK(chip.core().read_memory(0x42010500, counter, sizeof counter));
    std::uint32_t entries = 0;
    std::memcpy(&entries, counter, 4);
    CHECK(entries == 1); // taken once: the handler moved the compare away
    CHECK(read32(chip, 0x2004000) == 0xffffffff); // the handler wrote mtimecmp (low word)
}

void test_bl702_trng_fills_the_output_on_trigger() {
    // The SEC_ENG TRNG (CTRL at 0x40004200, output words from 0x40004208). The Zephyr driver triggers it and reads 32
    // bytes; sys_rand_get builds the EUI-64 from them, so an unmapped TRNG (all zeros) gives an all-zero address.
    Bl702 chip;
    chip.load_flash(image(NoSegment, Entry));
    chip.reset();
    write32(chip, 0x4000'4200, 1u << 1); // TRIG
    CHECK(read32(chip, 0x4000'4208) != 0 || read32(chip, 0x4000'420c) != 0);
    CHECK(read32(chip, 0x4000'4208) != read32(chip, 0x4000'420c));
}

void test_bl702_radio_timer_channel_fires_at_its_target() {
    // The MAC's timer channels: the counter (0x40) counts 16 us symbols and reaches the target of channel 2
    // (0x4c00006c) at 40 symbols, 640 us; the channel is enabled by bit 2 of 0x4c000020. Its status bit
    // (2 in 0x4c000030) stays until cleared.
    Bl702 chip;
    chip.load_flash(image(NoSegment, Entry));
    chip.reset();
    write32(chip, M154 + 0x06c, 40);
    write32(chip, M154 + 0x020, 1u << 2);
    chip.run_until(500000);
    CHECK(!(read32(chip, M154 + 0x030) & (1u << 2)));
    chip.run_until(700000);
    CHECK(read32(chip, M154 + 0x030) & (1u << 2));
    write32(chip, M154 + 0x024, 1u << 2);
    CHECK(!(read32(chip, M154 + 0x030) & (1u << 2)));
}

} // namespace mcuemu_tests
