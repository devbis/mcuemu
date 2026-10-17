// EFR32MG21: the Cortex-M33 core, the application-slot boot, CMU/HFXO, GPIO,
// MSC and the radio blocks.

#include "../test_support.hpp"
#include "../nrf52/programs.hpp"
#include "cortexm/core.hpp"
#include "efr32emu/efr32mg21.hpp"
#include "mcuemu/radio.hpp"

#include <cstring>

namespace mcuemu_tests {

namespace {

constexpr std::uint32_t Cmu = 0x40008000, Hfxo = 0x4000C000, Msc = 0x40030000, Gpio = 0x4003C000;
constexpr std::uint32_t Set = 0x1000, Clear = 0x2000, Toggle = 0x3000;
constexpr std::uint32_t Radio = 0x40090000;
constexpr std::uint64_t Us = 38; // cycles per microsecond at 38.4 MHz (rounded down)

struct Chip {
    efr32emu::Efr32mg21 chip;

    explicit Chip(const std::vector<std::uint8_t> &image = nrf52_programs::idle) {
        chip.load_firmware(image);
        chip.reset();
    }
    void w(std::uint32_t address, std::uint32_t value) { chip.write(address, value, 4); }
    std::uint32_t r(std::uint32_t address) { return chip.read(address, 4); }
};

} // namespace

// The nRF52 test programs (Thumb-2, FPU) on an M33 without the Security
// Extension: SVC, nested interrupts and the exception return with ES clear.
void test_efr32_m33_exceptions() {
    cortexm::Core::Config config;
    config.model = cortexm::Model::M33;
    config.priority_bits = 4;
    cortexm::Core core(config);
    std::uint8_t *flash = core.map_memory(0, 0x10000, false);
    core.map_memory(0x20000000, 0x10000, true);
    std::memcpy(flash, nrf52_programs::irq.data(), nrf52_programs::irq.size());
    core.reset();
    CHECK(core.run(2000).reason == cortexm::StopReason::Deadline);
    std::uint32_t value = 0;
    core.read_memory(0x2000010c, &value, 4);
    CHECK(value == 4); // SVC handler ran
    core.set_irq_line(3, true);
    core.set_irq_line(3, false);
    CHECK(core.run(2400).reason == cortexm::StopReason::Deadline);
    core.read_memory(0x20000108, &value, 4);
    CHECK(value == 3); // the more urgent nested handler ran
    core.read_memory(0x20000104, &value, 4);
    CHECK(value == 2);
    CHECK(core.reg(4) == 0x44);
    CHECK(core.reg(13) == 0x20001000); // stack balanced: every exception returned cleanly
}

// An application-only image (vector table in the 0x4000 slot) is loaded there
// and started directly, with VTOR pointing at it.
void test_efr32_application_slot_boot() {
    std::vector<std::uint8_t> image(0x20, 0xff);
    const std::uint32_t sp = 0x20010000, reset = 0x4009; // Thumb bit set
    std::memcpy(image.data(), &sp, 4);
    std::memcpy(image.data() + 4, &reset, 4);
    image[8] = 0xfe;
    image[9] = 0xe7; // b .
    Chip c(image);
    CHECK(c.chip.flash()[0x4000 + 8] == 0xfe);
    CHECK(c.chip.flash()[0] == 0xff);
    c.chip.run(100);
    CHECK(c.chip.core().pc() == 0x4008);
    CHECK(c.chip.core().reg(13) == 0x20010000);
    std::uint32_t vtor = 0;
    c.chip.core().read_memory(0xE000ED08, &vtor, 4);
    CHECK(vtor == 0x4000);
}

// The sequence of the zigbee-rs clock driver: FSRCO -> HFXO crystal start ->
// first-lock -> HFXO as system clock, using the set/clear aliases.
void test_efr32_clock_and_hfxo() {
    Chip c;
    CHECK((c.r(Cmu + 0x070) & 7) == 1);                 // FSRCO out of reset
    CHECK((c.r(Hfxo + 0x058) & 1) == 0);                // HFXO off
    c.w(Hfxo + Set + 0x028, 2);                         // DISONDEMAND
    c.w(Hfxo + Clear + 0x028, 1);
    CHECK((c.r(Hfxo + 0x058) & (1u << 16)) == 0);       // ENS clear
    c.w(Hfxo + 0x028, (c.r(Hfxo + 0x028) & ~3u) | 1);   // FORCEEN, on demand
    const std::uint32_t ready = 1u | 2u | (1u << 16) | (1u << 30);
    CHECK((c.r(Hfxo + 0x058) & ready) == ready);        // RDY, COREBIASOPTRDY, ENS, FSMLOCK
    c.w(Hfxo + Set + 0x028, 2);
    CHECK((c.r(Hfxo + 0x058) & (1u << 30)) == 0);       // first lock done
    c.w(Cmu + 0x070, 3);                                // HFXO selected
    CHECK((c.r(Cmu + 0x070) & 7) == 3);
    c.w(Hfxo + Clear + 0x028, 3);                       // FORCEEN and DISONDEMAND off
    CHECK((c.r(Hfxo + 0x058) & 0x10001) == 0x10001);    // on demand for the system clock
}

void test_efr32_gpio_aliases_and_exti() {
    Chip c;
    // PB0 push-pull, driven through the set/clear aliases of DOUT.
    c.w(Gpio + 0x34, 4);
    c.w(Gpio + Set + 0x40, 1);
    CHECK(c.r(Gpio + 0x40) == 1);
    CHECK(c.chip.pad_drive(16) == efr32emu::PadDrive::High);
    CHECK((c.r(Gpio + 0x44) & 1) == 1);                 // DIN follows the output
    c.w(Gpio + Clear + 0x40, 1);
    CHECK(c.chip.pad_drive(16) == efr32emu::PadDrive::Low);
    c.w(Gpio + Toggle + 0x40, 1);
    CHECK(c.chip.pad_drive(16) == efr32emu::PadDrive::High);

    // PD2 input with pull-up, EXTI2 on the falling edge: GPIO_EVEN (IRQ 3).
    c.w(Gpio + 0x94, 2u << 8);                          // mode 2 (input with pull)
    c.w(Gpio + Set + 0xa0, 4);                          // DOUT = 1: pull-up
    CHECK(c.chip.pad_drive(3 * 16 + 2) == efr32emu::PadDrive::PullUp);
    c.w(Gpio + 0x400, 3u << 8);                         // EXTIPSEL2 = port D
    c.w(Gpio + 0x408, 2u << 8);                         // EXTIPINSEL2 = pin 2
    c.w(Gpio + 0x414, 4);                               // EXTIFALL
    c.w(Gpio + Set + 0x424, 4);                         // IEN
    std::uint32_t enable = 1u << 3;
    c.chip.core().write_memory(0xE000E100, &enable, 4); // NVIC: enable IRQ 3
    c.chip.set_pad_input(3 * 16 + 2, true);
    CHECK(c.r(Gpio + 0x420) == 0);
    CHECK(!c.chip.core().interrupt_pending());
    c.chip.set_pad_input(3 * 16 + 2, false);            // button pressed
    CHECK(c.r(Gpio + 0x420) == 4);
    CHECK(c.chip.core().interrupt_pending());
    c.w(Gpio + Clear + 0x420, 4);
    CHECK(c.r(Gpio + 0x420) == 0);
    // The NVIC latched the edge: it stays pending until taken or cleared.
    CHECK(c.chip.core().interrupt_pending());
    std::uint32_t clear_pending = 1u << 3;
    c.chip.core().write_memory(0xE000E280, &clear_pending, 4);
    CHECK(!c.chip.core().interrupt_pending());
    // Rising edges are not selected.
    c.chip.set_pad_input(3 * 16 + 2, true);
    CHECK(c.r(Gpio + 0x420) == 0);

    unsigned pad = 0;
    CHECK(efr32emu::Efr32mg21::parse_pad("PD2", pad) && pad == 50);
    CHECK(efr32emu::Efr32mg21::parse_pad("PB0", pad) && pad == 16);
    CHECK(!efr32emu::Efr32mg21::parse_pad("PE0", pad) && !efr32emu::Efr32mg21::parse_pad("PA16", pad));
    CHECK(efr32emu::Efr32mg21::pad_name(50) == "PD2");
}

void test_efr32_msc_erase_and_program() {
    Chip c;
    const std::uint32_t page = 0x7C000;
    CHECK(c.chip.flash()[page] == 0xff);
    // Locked: nothing happens.
    c.w(Msc + 0x00c, 1);
    c.w(Msc + 0x014, page);
    c.w(Msc + 0x018, 0x12345678);
    CHECK(c.chip.flash()[page] == 0xff);
    CHECK((c.r(Msc + 0x01c) & (1u << 16)) != 0); // REGLOCK

    c.w(Msc + 0x03c, 0x1b71);
    CHECK((c.r(Msc + 0x01c) & (1u << 16)) == 0);
    c.w(Msc + 0x00c, 1);                          // WREN
    c.w(Msc + 0x014, page);
    CHECK((c.r(Msc + 0x01c) & 8) != 0);           // WDATAREADY
    c.w(Msc + 0x018, 0x12345678);
    c.w(Msc + 0x018, 0xf0f0f0f0);
    c.w(Msc + 0x010, 4);                          // WRITEEND
    std::uint32_t word = 0;
    std::memcpy(&word, c.chip.flash() + page, 4);
    CHECK(word == 0x12345678);
    std::memcpy(&word, c.chip.flash() + page + 4, 4);
    CHECK(word == 0xf0f0f0f0);
    // Programming only clears bits.
    c.w(Msc + 0x014, page);
    c.w(Msc + 0x018, 0x0f0f0f0f);
    std::memcpy(&word, c.chip.flash() + page, 4);
    CHECK(word == (0x12345678u & 0x0f0f0f0fu));
    // Erase restores the whole 8 KiB page.
    c.w(Msc + 0x014, page + 0x100);
    c.w(Msc + 0x010, 2);
    CHECK(c.chip.flash()[page] == 0xff && c.chip.flash()[page + 4] == 0xff);
    // An address beyond the flash is invalid.
    c.w(Msc + 0x014, 0x100000);
    CHECK((c.r(Msc + 0x01c) & 4) != 0);
    c.w(Msc + 0x03c, 0);
    CHECK((c.r(Msc + 0x01c) & (1u << 16)) != 0);
}

namespace {

void run_both(Chip &a, Chip &b, std::uint64_t until_us) {
    for (std::uint64_t t = 5; t <= until_us; t += 5) {
        a.chip.run(t * Us);
        b.chip.run(t * Us);
    }
}

} // namespace

void test_efr32_radio_frame_exchange() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    Chip a, b;
    a.chip.connect_radio(air, "a");
    b.chip.connect_radio(air, "b");
    for (Chip *chip : {&a, &b}) {
        chip->w(Radio + 0x200c, 4);           // SYNTH CHNO: channel 15
        chip->w(Radio + 0x301c, 3);           // RAC IEN: TX done and RX done
    }
    std::uint32_t enable = 1u << (36 - 32);
    for (Chip *chip : {&a, &b})
        chip->chip.core().write_memory(0xE000E104, &enable, 4); // ISER1: IRQ 36

    b.w(Radio + 0x1004, 2);                   // BUFC: clear the RX FIFO
    b.w(Radio + 0x3004, 2);                   // RXEN
    CHECK((b.r(Radio + 0x3008) & 0xf) == 2);

    const std::uint8_t mac[5] = {0x41, 0x88, 0x07, 0xaa, 0xbb};
    a.w(Radio + 0x1004, 1);
    a.w(Radio + 0x1040, 7);                   // PHR: 5 bytes + the 2-byte FCS
    for (std::uint8_t byte : mac)
        a.w(Radio + 0x1040, byte);
    a.w(Radio + 0x3004, 1);                   // TXEN
    CHECK((a.r(Radio + 0x3008) & 0xf) == 3);

    run_both(a, b, 1500);
    CHECK((a.r(Radio + 0x3010) & 1) == 1);    // TX done
    CHECK((a.r(Radio + 0x3008) & 0xf) == 1);  // idle again
    CHECK((b.r(Radio + 0x3010) & 2) == 2);    // RX done
    CHECK((b.r(Radio + 0x0030) & 0x10) != 0); // CRC ok
    CHECK(b.chip.core().interrupt_pending()); // FRC_PRI
    CHECK(b.r(Radio + 0x1084) == 5);          // frame length
    std::uint8_t got[5];
    for (std::uint8_t &byte : got)
        byte = static_cast<std::uint8_t>(b.r(Radio + 0x1080));
    CHECK(std::memcmp(got, mac, 5) == 0);
    b.w(Radio + 0x3018, 2);                   // IFC
    b.w(Radio + 0x0038, 0xffffffff);
    b.chip.core().write_memory(0xE000E284, &enable, 4); // ICPR1: the edge was latched
    CHECK(!b.chip.core().interrupt_pending());
    CHECK(air.frames().size() == 1 && air.frames()[0].channel == 15 && air.frames()[0].payload.size() == 5);
}

void test_efr32_radio_state_and_rssi() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    Chip a, b;
    a.chip.connect_radio(air, "a");
    b.chip.connect_radio(air, "b");
    CHECK((a.r(Radio + 0x3008) & 0xf) == 1);  // idle out of reset
    a.w(Radio + 0x3004, 2);
    CHECK((a.r(Radio + 0x3008) & 0xf) == 2);
    a.w(Radio + 0x3004, 8);                   // RXDIS
    CHECK((a.r(Radio + 0x3008) & 0xf) == 1);
    // A quiet channel reads far below the CCA threshold (-60 dBm).
    CHECK(static_cast<std::int8_t>(a.r(Radio + 0x5020) & 0xff) < -100);
    // A long frame on air makes the energy rise on the same channel.
    b.w(Radio + 0x1040, 100);
    for (int i = 0; i != 98; ++i)
        b.w(Radio + 0x1040, 0x55);
    b.w(Radio + 0x3004, 1);
    run_both(a, b, 500);
    CHECK(static_cast<std::int8_t>(a.r(Radio + 0x5020) & 0xff) > -60);
}

// The RTCC counts at 32768 Hz (inferred clock), reads the same through the
// secure and non-secure bases, and sets IF.CC1 at OCVALUE + 1 (the sleeptimer's
// convention) with the interrupt taken on the RTCC line.
void test_efr32_rtcc_counts_and_compares() {
    Chip c;
    c.w(0x58000004, 1);      // EN (non-secure alias)
    c.w(0x4800000c, 1);      // CMD.START
    c.chip.core().advance_to(19200000); // 0.5 s at 38.4 MHz
    CHECK(c.r(0x58000020) == 16384);
    CHECK(c.r(0x48000020) == 16384);
    c.w(0x48000020, 100);    // CNT = 100
    c.w(0x4800003c, 2);      // CC1.CTRL = OUTPUTCOMPARE
    c.w(0x48000040, 110);    // CC1.OCVALUE: match at tick 111
    c.w(0x48000018, 8);      // IEN.CC1
    const std::uint64_t start = c.chip.core().cycles();
    c.chip.run(start + 5000); // about 4 ticks
    CHECK((c.r(0x48000014) & 8) == 0);
    c.chip.run(start + 20000); // about 17 ticks
    CHECK((c.r(0x48000014) & 8) != 0);
    c.w(0x48002014, 8);      // IF_CLR: the clear alias (+0x2000) of IF
    CHECK((c.r(0x48000014) & 8) == 0);
}

// The SEMAILBOX host answers a command once the words its TX_HEADER announces
// have arrived (the handle word is not stored), with SUCCESS and no output.
void test_efr32_semailbox_answers_commands() {
    Chip c;
    CHECK((c.r(0x4C000040) & (1u << 20)) != 0);   // TX_STATUS.TXINT: room
    c.w(0x4C000050, 16);                          // TX_HEADER: 4 words
    c.w(0x4C000000, 1);                           // command
    c.w(0x4C000000, 2);
    CHECK((c.r(0x4C000044) & (1u << 20)) == 0);   // not yet: 2 of 3 words
    c.w(0x4C000000, 3);
    CHECK((c.r(0x4C000044) & (1u << 20)) != 0);   // RX_STATUS.RXINT
    CHECK((c.r(0x4C000054) & 0x000f0000) == 0);   // RX_HEADER: SLI_SE_RESPONSE_OK
    CHECK((c.r(0x5C000044) & (1u << 20)) != 0);   // the non-secure alias
}

} // namespace mcuemu_tests
