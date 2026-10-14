// nRF52840 peripherals, driven through the register interface with a CPU that
// only spins, so the timing under test is the peripheral's.

#include "../test_support.hpp"
#include "nrf52emu/nrf52840.hpp"
#include "programs.hpp"

#include <cstring>
#include <string>
#include <vector>

namespace mcuemu_tests {

namespace {

constexpr std::uint32_t Us = 64; // core cycles per microsecond
constexpr std::uint32_t Power = 0x40000000, Uarte0 = 0x40002000, Gpiote = 0x40006000,
                        Saadc = 0x40007000, Timer0 = 0x40008000, Timer1 = 0x40009000,
                        Rtc0 = 0x4000b000, Rng = 0x4000d000, Ecb = 0x4000e000,
                        Wdt = 0x40010000, Nvmc = 0x4001e000, Ppi = 0x4001f000,
                        P0 = 0x50000000;

struct Chip {
    nrf52emu::Nrf52840 chip;

    explicit Chip(const std::vector<std::uint8_t> &program = nrf52_programs::idle) {
        chip.load_firmware(program);
        chip.reset();
    }
    void write(std::uint32_t address, std::uint32_t value) { chip.write(address, value, 4); }
    std::uint32_t read(std::uint32_t address) { return chip.read(address, 4); }
    void run_to_us(std::uint64_t us) { chip.run(us * Us); }
    void run_to_cycle(std::uint64_t cycle) { chip.run(cycle); }
    std::uint32_t ram_word(std::uint32_t address) {
        std::uint32_t value = 0;
        std::memcpy(&value, chip.ram() + (address - nrf52emu::Nrf52840::RamBase), 4);
        return value;
    }
};

} // namespace

void test_nrf52_flash_ids_and_memory() {
    Chip c;
    std::uint32_t part = 0, codesize = 0;
    c.chip.core().read_memory(0x10000100, &part, 4);     // FICR.INFO.PART
    c.chip.core().read_memory(0x10000014, &codesize, 4); // FICR.CODESIZE
    CHECK(part == 0x52840);
    CHECK(codesize == 256);
    std::uint32_t ram = 0x11223344, back = 0;
    c.chip.core().write_memory(0x20000000, &ram, 4);
    c.chip.core().read_memory(0x20000000, &back, 4);
    CHECK(back == ram);
}

void test_nrf52_nvmc_programming_rules() {
    Chip c(nrf52_programs::nvmc);
    c.run_to_us(200);
    std::uint32_t word = 0, uicr = 0;
    c.chip.core().read_memory(0x80000, &word, 4);
    c.chip.core().read_memory(0x10001080, &uicr, 4);
    CHECK(word == 0x05050505u);   // two writes AND together; the write with WEN off is ignored
    CHECK(uicr == 0x12345678u);

    // A page erase needs CONFIG.EEN and sets the page to 0xFF.
    c.write(Nvmc + 0x504, 2);
    c.write(Nvmc + 0x508, 0x80000);
    c.chip.core().read_memory(0x80000, &word, 4);
    CHECK(word == 0xffffffffu);
    CHECK(c.read(Nvmc + 0x400) == 1);             // READY
    c.write(Nvmc + 0x514, 1);                     // ERASEUICR
    c.chip.core().read_memory(0x10001080, &uicr, 4);
    CHECK(uicr == 0xffffffffu);                   // CONFIG is EEN: this one erases
}

void test_nrf52_rtc_compare_timing_and_counter() {
    Chip c;
    c.write(Rtc0 + 0x540, 33);          // CC0: 33 ticks = 1.007 ms
    c.write(Rtc0 + 0x344, 1u << 16);    // EVTENSET: route COMPARE0
    c.write(Rtc0 + 0x000, 1);           // START
    c.run_to_us(1000);
    CHECK(c.read(Rtc0 + 0x140) == 0);
    c.run_to_us(1020);
    CHECK(c.read(Rtc0 + 0x140) == 1);
    c.write(Rtc0 + 0x140, 0);
    CHECK(c.read(Rtc0 + 0x140) == 0);
    c.run_to_us(2000);
    const std::uint32_t counter = c.read(Rtc0 + 0x504);
    CHECK(counter == 65 || counter == 64 || counter == 66); // 2 ms * 32768 Hz
    c.write(Rtc0 + 0x004, 1);           // STOP freezes it
    c.run_to_us(4000);
    CHECK(c.read(Rtc0 + 0x504) == counter);
}

void test_nrf52_rtc_interrupt_and_overflow() {
    Chip c;
    c.write(Rtc0 + 0x540, 10);
    c.write(Rtc0 + 0x304, 1u << 16);    // INTENSET COMPARE0
    c.write(Rtc0 + 0x000, 1);
    c.run_to_us(500);
    CHECK(c.read(Rtc0 + 0x140) == 1);   // events go to the interrupt line too
    // 24-bit overflow: TRIGOVRFLW puts the counter 15 ticks short of the wrap.
    c.write(Rtc0 + 0x344, 1u << 1);     // route OVRFLW
    c.write(Rtc0 + 0x00c, 1);
    c.run_to_us(1000);
    CHECK(c.read(Rtc0 + 0x104) == 1);
}

void test_nrf52_timer_compare_clear_and_ppi_count() {
    Chip c;
    // TIMER0 compares every 100 us (1600 ticks of 16 MHz) and clears itself;
    // PPI channel 0 counts those compares in TIMER1.
    c.write(Timer0 + 0x508, 3);         // BITMODE 32
    c.write(Timer0 + 0x510, 0);         // PRESCALER 0: 16 MHz
    c.write(Timer0 + 0x540, 1600);
    c.write(Timer0 + 0x200, 1);         // COMPARE0_CLEAR
    c.write(Timer1 + 0x504, 1);         // MODE: counter
    c.write(Timer1 + 0x000, 1);
    c.write(Ppi + 0x510, Timer0 + 0x140);   // CH0.EEP
    c.write(Ppi + 0x514, Timer1 + 0x008);   // CH0.TEP = COUNT
    c.write(Ppi + 0x504, 1);                // CHENSET
    c.write(Timer0 + 0x000, 1);
    c.run_to_us(1000);
    c.write(Timer1 + 0x040, 1);             // CAPTURE0
    const std::uint32_t count = c.read(Timer1 + 0x540);
    CHECK(count == 9 || count == 10);
}

// A channel whose fork task disables its own group fires once; this is how the
// Nordic 802.15.4 driver keeps the radio from ramping up again after a frame.
void test_nrf52_ppi_fork_disables_a_channel_group() {
    Chip c;
    c.write(Timer0 + 0x508, 3);             // BITMODE 32
    c.write(Timer0 + 0x510, 0);
    c.write(Timer0 + 0x540, 1600);
    c.write(Timer0 + 0x200, 1);             // COMPARE0_CLEAR: an event every 100 us
    c.write(Timer1 + 0x504, 1);             // counter mode
    c.write(Timer1 + 0x000, 1);
    c.write(Ppi + 0x510, Timer0 + 0x140);   // CH0.EEP
    c.write(Ppi + 0x514, Timer1 + 0x008);   // CH0.TEP = COUNT
    c.write(Ppi + 0x910, Ppi + 0x004);      // CH0 fork: TASKS_CHG[0].DIS
    c.write(Ppi + 0x800, 1);                // group 0 = channel 0
    c.write(Ppi + 0x504, 1);                // CHENSET
    c.write(Timer0 + 0x000, 1);
    c.run_to_us(1000);
    c.write(Timer1 + 0x040, 1);             // CAPTURE0
    CHECK(c.read(Timer1 + 0x540) == 1);     // counted once, then the channel was off
    c.write(Ppi + 0x000, 1);                // TASKS_CHG[0].EN
    c.run_to_us(1200);
    c.write(Timer1 + 0x040, 1);
    CHECK(c.read(Timer1 + 0x540) == 2);     // enabled again: one more, then off again
}

void test_nrf52_gpio_pads_and_sense() {
    Chip c;
    unsigned seen = 99;
    int observed = 0;
    c.chip.set_pad_observer([&](unsigned pad) {
        seen = pad;
        ++observed;
    });
    c.write(P0 + 0x514, 1u << 5);       // DIR: P0.05 output
    c.write(P0 + 0x508, 1u << 5);       // OUTSET
    CHECK(c.chip.pad_drive(5) == nrf52emu::PadDrive::High);
    CHECK(seen == 5 && observed > 0);
    c.write(P0 + 0x50c, 1u << 5);       // OUTCLR
    CHECK(c.chip.pad_drive(5) == nrf52emu::PadDrive::Low);

    // Open-drain output (S0D1): high releases the line, the pull-up takes it.
    c.write(P0 + 0x700 + 4 * 6, 1 | (6 << 8) | (3 << 2)); // output, S0D1, pull-up
    c.write(P0 + 0x508, 1u << 6);
    CHECK(c.chip.pad_drive(6) == nrf52emu::PadDrive::PullUp);

    // An input with a pull-up reads the net level; SENSE low raises PORT.
    c.write(P0 + 0x700 + 4 * 7, (3 << 2) | (3 << 16)); // input connected, pull-up, sense low
    CHECK(c.chip.pad_drive(7) == nrf52emu::PadDrive::PullUp);
    c.chip.set_pad_input(7, true);
    CHECK((c.read(P0 + 0x510) >> 7 & 1u) == 1);
    c.write(Gpiote + 0x17c, 0);
    c.chip.set_pad_input(7, false);
    CHECK((c.read(P0 + 0x510) >> 7 & 1u) == 0);
    CHECK(c.read(Gpiote + 0x17c) == 1);      // EVENTS_PORT
    CHECK((c.read(P0 + 0x520) >> 7 & 1u) == 1); // LATCH
}

void test_nrf52_gpiote_edge_event_and_output_task() {
    Chip c;
    // Channel 0: event on a falling edge of P0.07. Channel 1: toggle P0.05.
    c.write(P0 + 0x700 + 4 * 7, 0); // input connected
    c.write(Gpiote + 0x510, 1 | (7 << 8) | (2 << 16));
    c.write(Gpiote + 0x514, 3 | (5 << 8) | (3 << 16));
    c.chip.set_pad_input(7, true);
    CHECK(c.read(Gpiote + 0x100) == 0);
    c.chip.set_pad_input(7, false);
    CHECK(c.read(Gpiote + 0x100) == 1);
    CHECK(c.chip.pad_drive(5) == nrf52emu::PadDrive::Low);
    c.write(Gpiote + 0x004, 1);          // TASKS_OUT[1]
    CHECK(c.chip.pad_drive(5) == nrf52emu::PadDrive::High);
    c.write(Gpiote + 0x004, 1);
    CHECK(c.chip.pad_drive(5) == nrf52emu::PadDrive::Low);
    // The same task through PPI: the falling edge toggles the pad.
    c.write(Ppi + 0x510, Gpiote + 0x100);
    c.write(Ppi + 0x514, Gpiote + 0x004);
    c.write(Ppi + 0x504, 1);
    c.chip.set_pad_input(7, true);
    c.chip.set_pad_input(7, false);
    CHECK(c.chip.pad_drive(5) == nrf52emu::PadDrive::High);
}

void test_nrf52_ecb_encrypts_a_block() {
    Chip c;
    const std::uint8_t block[48] = {
        // key (FIPS-197 appendix B), cleartext, ciphertext slot
        0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
        0x32, 0x43, 0xf6, 0xa8, 0x88, 0x5a, 0x30, 0x8d, 0x31, 0x31, 0x98, 0xa2, 0xe0, 0x37, 0x07, 0x34};
    std::memcpy(c.chip.ram() + 0x100, block, 32);
    c.write(Ecb + 0x504, 0x20000100);
    c.write(Ecb + 0x000, 1);
    CHECK(c.read(Ecb + 0x100) == 0);
    c.run_to_us(20);
    CHECK(c.read(Ecb + 0x100) == 1);
    const std::uint8_t expected[16] = {0x39, 0x25, 0x84, 0x1d, 0x02, 0xdc, 0x09, 0xfb,
                                       0xdc, 0x11, 0x85, 0x97, 0x19, 0x6a, 0x0b, 0x32};
    CHECK(std::memcmp(c.chip.ram() + 0x100 + 32, expected, 16) == 0);
}

void test_nrf52_rng_is_deterministic_per_device() {
    auto sequence = [](std::uint64_t id) {
        Chip c;
        c.chip.set_device_id(id);
        std::vector<std::uint32_t> values;
        c.write(Rng + 0x000, 1);
        for (unsigned i = 1; i <= 4; ++i) {
            c.run_to_us(i * 100);
            CHECK(c.read(Rng + 0x100) == 1);
            values.push_back(c.read(Rng + 0x508));
            c.write(Rng + 0x100, 0);
        }
        return values;
    };
    CHECK(sequence(1) == sequence(1));
    CHECK(sequence(1) != sequence(2));
}

void test_nrf52_watchdog_resets_unless_fed() {
    Chip c;
    c.write(Wdt + 0x504, 32768);        // CRV: 1 s
    c.write(Wdt + 0x508, 1);            // RREN: RR0
    c.write(Wdt + 0x000, 1);            // START
    c.run_to_us(900000);
    c.write(Wdt + 0x600, 0x6e524635);   // feed
    c.run_to_us(1800000);
    CHECK((c.read(Power + 0x400) & 2u) == 0);   // no reset yet
    c.run_to_us(2000000);                // 1 s after the feed: the dog bites
    CHECK((c.read(Power + 0x400) & 2u) != 0);   // RESETREAS.DOG
}

void test_nrf52_uarte_transmit_and_receive() {
    Chip c;
    const char text[] = "hello";
    std::memcpy(c.chip.ram() + 0x200, text, 5);
    c.write(Uarte0 + 0x500, 8);               // ENABLE
    c.write(Uarte0 + 0x524, 0x01D7E000);      // 115200 baud
    c.write(Uarte0 + 0x544, 0x20000200);
    c.write(Uarte0 + 0x548, 5);
    c.write(Uarte0 + 0x008, 1);               // STARTTX
    c.run_to_us(300);
    CHECK(c.chip.uart_take_tx(0).size() == 3); // 86.8 us per byte
    c.run_to_us(1000);
    CHECK(c.read(Uarte0 + 0x128) == 1);        // ENDTX
    const std::vector<std::uint8_t> rest = c.chip.uart_take_tx(0);
    CHECK(std::string(rest.begin(), rest.end()) == "lo");

    c.write(Uarte0 + 0x534, 0x20000300);
    c.write(Uarte0 + 0x538, 3);
    c.write(Uarte0 + 0x000, 1);                // STARTRX
    CHECK(c.chip.uart_receive(0, 'a'));
    CHECK(c.chip.uart_receive(0, 'b'));
    CHECK(c.read(Uarte0 + 0x110) == 0);
    CHECK(c.chip.uart_receive(0, 'c'));
    CHECK(c.read(Uarte0 + 0x110) == 1);        // ENDRX
    CHECK(std::memcmp(c.chip.ram() + 0x300, "abc", 3) == 0);

    // UARTE1 lives at peripheral id 40, above the first 128 KiB of the window.
    CHECK(!c.chip.uart_can_receive(1));
    c.write(0x40028000 + 0x500, 8);
    CHECK(c.read(0x40028000 + 0x500) == 8);
    CHECK(c.chip.uart_can_receive(1));
}

void test_nrf52_saadc_reads_vdd() {
    Chip c;
    c.chip.set_supply_mv(3000);
    c.write(Saadc + 0x500, 1);                 // ENABLE
    c.write(Saadc + 0x5f0, 2);                 // 12-bit
    c.write(Saadc + 0x510, 9);                 // CH0.PSELP = VDD
    c.write(Saadc + 0x518, 0);                 // gain 1/6, internal 0.6 V reference
    c.write(Saadc + 0x62c, 0x20000400);
    c.write(Saadc + 0x630, 1);
    c.write(Saadc + 0x000, 1);                 // START
    c.write(Saadc + 0x004, 1);                 // SAMPLE
    c.run_to_us(100);
    CHECK(c.read(Saadc + 0x104) == 1);         // END
    std::int16_t sample = 0;
    std::memcpy(&sample, c.chip.ram() + 0x400, 2);
    CHECK(sample >= 3410 && sample <= 3414);   // 3.0 V * (1/6) / 0.6 V * 4096
}

void test_nrf52_clock_start_events() {
    Chip c;
    c.write(Power + 0x000, 1);                 // HFCLKSTART
    c.write(Power + 0x008, 1);                 // LFCLKSTART
    CHECK(c.read(Power + 0x100) == 0);
    c.run_to_us(500);
    CHECK(c.read(Power + 0x100) == 1);         // HFCLKSTARTED
    CHECK(c.read(Power + 0x104) == 1);         // LFCLKSTARTED
    CHECK((c.read(Power + 0x40c) >> 16 & 1u) == 1); // HFCLKSTAT.STATE
}

// Zephyr starts the LFCLK on the RC, then selects the crystal and starts it
// again without stopping; LFCLKSTAT must follow, or its spin-wait never ends.
void test_nrf52_lfclk_restart_switches_source() {
    Chip c;
    c.write(Power + 0x518, 0);                 // LFCLKSRC = RC
    c.write(Power + 0x008, 1);                 // LFCLKSTART
    c.run_to_us(200);
    CHECK(c.read(Power + 0x104) == 1);
    CHECK(c.read(Power + 0x418) == (1u << 16)); // running on the RC

    c.write(Power + 0x104, 0);
    c.write(Power + 0x518, 1);                 // LFCLKSRC = XTAL
    c.write(Power + 0x008, 1);
    CHECK(c.read(Power + 0x418) == (1u << 16)); // still the RC until the crystal is up
    c.run_to_us(2000);
    CHECK(c.read(Power + 0x104) == 1);
    CHECK(c.read(Power + 0x418) == ((1u << 16) | 1u));
}

} // namespace mcuemu_tests
