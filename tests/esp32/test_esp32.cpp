// ESP32-C6 model: ROM-loader style boot, ROM stand-ins, cache MMU, timers and
// the flash controller. The payloads are tests/riscv/programs/esp_*.s.

#include "../riscv/programs.hpp"
#include "../test_support.hpp"
#include "esp32emu/esp32.hpp"

#include <cstring>
#include <string>

namespace mcuemu_tests {

namespace {

// An ESP image: the 24-byte header (magic 0xE9, segment count, entry point, a
// chip id of 13 in the extended header) and one RAM segment.
std::vector<std::uint8_t> image(const std::vector<std::uint8_t> &payload,
                                std::uint32_t address = 0x40800000) {
    std::vector<std::uint8_t> out(24, 0);
    out[0] = 0xe9;
    out[1] = 1;
    const std::uint32_t entry = address;
    std::memcpy(&out[4], &entry, 4);
    out[12] = 13;
    const std::uint32_t length = static_cast<std::uint32_t>(payload.size());
    out.insert(out.end(), reinterpret_cast<const std::uint8_t *>(&address),
               reinterpret_cast<const std::uint8_t *>(&address) + 4);
    out.insert(out.end(), reinterpret_cast<const std::uint8_t *>(&length),
               reinterpret_cast<const std::uint8_t *>(&length) + 4);
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

} // namespace

void test_esp32c6_boots_an_image_and_stands_in_for_the_rom() {
    esp32emu::Esp32 chip;
    std::string console;
    chip.on_uart_tx = [&](std::uint8_t byte) { console.push_back(static_cast<char>(byte)); };
    chip.load_flash(image(riscv_programs::esp_uart_hello));
    chip.reset();
    CHECK(chip.run_until(1000000).reason == riscv::StopReason::Halted); // the payload's ebreak
    CHECK(console == "Hi!\n");
    CHECK(chip.core().reg(9) == 0x0a216948);        // memcpy in the ROM stand-in copied "Hi!\n"
    CHECK(chip.core().reg(18) == 1000000000u);      // __udivdi3: 10^12 / 1000
}

void test_esp32c6_cache_mmu_maps_flash_pages() {
    esp32emu::Esp32 chip;
    std::vector<std::uint8_t> data(0x10000, 0);
    const std::uint32_t marker = 0x12345678;
    std::memcpy(data.data(), &marker, 4);
    chip.load_flash(image(riscv_programs::esp_mmu));
    chip.load_flash(data, 0x20000); // flash page 2
    chip.reset();
    CHECK(chip.run_until(1000000).reason == riscv::StopReason::Halted);
    CHECK(chip.core().reg(9) == marker);  // read through the cache window at 0x42030000
    CHECK(chip.core().reg(18) == 0x202);  // the entry reads back
}

void test_esp32c6_systimer_counts_at_16_mhz() {
    esp32emu::Esp32 chip;
    chip.load_flash(image(riscv_programs::esp_spin));
    chip.reset();
    chip.run_until(1000000); // 1 ms
    chip.write(0x6000a004, 1u << 30, 4); // UNIT0_OP.UPDATE latches the counter
    CHECK(chip.read(0x6000a004, 4) & (1u << 29)); // VALUE_VALID
    const std::uint32_t low = chip.read(0x6000a044, 4);
    CHECK(chip.read(0x6000a040, 4) == 0);
    CHECK(low >= 15990 && low <= 16010); // 16 MHz for 1 ms
}

void test_esp32c6_lp_timer_and_rtc_calibration() {
    esp32emu::Esp32 chip;
    chip.load_flash(image(riscv_programs::esp_spin));
    chip.reset();
    chip.run_until(10000000); // 10 ms
    chip.write(0x600b0c10, 1u << 28, 4); // MAIN_TIMER_UPDATE
    const std::uint32_t slow = chip.read(0x600b0c14, 4); // MAIN_BUF0_LOW
    CHECK(slow >= 1355 && slow <= 1365); // 136 kHz for 10 ms

    // RTC slow clock calibration (TIMG0 RTCCALICFG): START with 100 cycles of RC_SLOW.
    chip.write(0x60008068, 1u << 31 | 100u << 16, 4);
    CHECK(chip.read(0x60008068, 4) & (1u << 15)); // RDY
    const std::uint32_t xtal_cycles = chip.read(0x6000806c, 4) >> 7;
    CHECK(xtal_cycles >= 29400 && xtal_cycles <= 29420); // 100 * 40 MHz / 136 kHz
}

void test_esp32c6_systimer_alarm_interrupts_a_sleeping_hart() {
    esp32emu::Esp32 chip;
    chip.load_flash(image(riscv_programs::esp_irq));
    chip.reset();
    CHECK(chip.run_until(1000000).reason == riscv::StopReason::Halted);
    CHECK(chip.core().reg(18) == 1);           // the handler ran once (the ack dropped the line)
    CHECK(chip.core().reg(19) == 0x80000005);  // mcause: CPU line 5
    // The hart slept from the first wfi to the alarm: 1600 ticks of 16 MHz = 100 us.
    CHECK(chip.time_ns() >= 100000 && chip.time_ns() < 101000);
    CHECK(chip.core().interrupt() == -1);
}

void test_esp32c6_pending_interrupt_is_taken_when_mie_returns_after_wfi() {
    esp32emu::Esp32 chip;
    chip.load_flash(image(riscv_programs::esp_irq_masked));
    chip.reset();
    CHECK(chip.run_until(1000000).reason == riscv::StopReason::Halted);
    CHECK(chip.core().reg(18) == 1);          // the handler ran
    CHECK(chip.core().reg(19) == 0x80000005);
}

void test_esp32c6_flash_controller_reports_jedec_id() {
    esp32emu::Esp32 chip; // 4 MiB flash
    chip.load_flash(image(riscv_programs::esp_spin));
    chip.reset();
    // The user-command path the bootloader uses: command 0x9f, read 3 bytes.
    chip.write(0x60003018, 1u << 28, 4);    // USER: USR_MISO
    chip.write(0x60003020, 0x7000009f, 4);  // USER2: 8-bit command 0x9f
    chip.write(0x60003000, 1u << 18, 4);    // CMD: USR
    CHECK((chip.read(0x60003000, 4) & (1u << 18)) == 0); // done at once
    CHECK((chip.read(0x60003058, 4) & 0xffffff) == (0xef | 0x40 << 8 | 22 << 16)); // 4 MiB = 2^22
}

} // namespace mcuemu_tests
