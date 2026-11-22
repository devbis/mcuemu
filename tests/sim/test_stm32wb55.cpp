#include "../test_support.hpp"

#include "stm32wb55emu/stm32wb55.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

namespace mcuemu_tests {
namespace {

void put32(std::vector<std::uint8_t> &bytes, std::size_t offset, std::uint32_t value) {
    std::memcpy(bytes.data() + offset, &value, sizeof(value));
}

} // namespace

void test_stm32wb55_cpu2_zero_based_flash_and_sram_aliases() {
    constexpr std::uint32_t install_offset = 0x000dd000;
    std::vector<std::uint8_t> image(32, 0);
    put32(image, 0, 0x2003d100); // CPU2 stack in SRAM2.
    put32(image, 4, 0x00000009); // reset handler at CPU2 code alias address 0x8.
    const auto code = words({0x4803, // ldr r0, [pc, #12] -> reference table at 0x10000000
                             0x6801, // ldr r1, [r0]
                             0x4803, // ldr r0, [pc, #12] -> alias SRAM at 0x10000010
                             0x6001, // str r1, [r0]
                             0xe7fe}); // b .
    std::memcpy(image.data() + 8, code.data(), code.size());
    put32(image, 24, 0x10000000);
    put32(image, 28, 0x10000010);

    stm32wb55emu::Stm32wb55 chip;
    chip.load_cpu2_firmware(image, install_offset);
    CHECK(chip.cpu2_has_vector_table());
    const std::uint32_t device_info_table = 0x200300b8;
    CHECK(chip.core().write_memory(0x20030000, &device_info_table,
                                   sizeof(device_info_table)));
    chip.write(0x5800040c, 1u << 15, 4); // CPU1 sets C2BOOT.
    const auto result = chip.run_cpu2_until(2000);
    CHECK(result.reason == cortexm::StopReason::Deadline);
    CHECK(chip.cpu2_time_ns() != 0);

    std::uint32_t value = 0;
    CHECK(chip.core().read_memory(0x20030010, &value, sizeof(value)));
    CHECK(value == device_info_table);

    std::uint32_t installed_word = 0;
    CHECK(chip.core().read_memory(stm32wb55emu::Stm32wb55::FlashBase + install_offset,
                                  &installed_word, sizeof(installed_word)));
    CHECK(installed_word == 0x2003d100);
}

void test_stm32wb55_pwr_clear_registers_do_not_read_back_commands() {
    stm32wb55emu::Stm32wb55 chip;
    chip.write(0x58000418, 1u << 10, 4); // PWR_SCR clears the 802.15.4 wake flag.
    chip.write(0x58000488, 1u << 2, 4);  // PWR_EXTSCR clears the critical radio phase flag.
    chip.write(0x58000410, 1u << 10, 4); // PWR_SR1 is hardware status, not a software latch.

    CHECK(chip.read(0x58000418, 4) == 0);
    CHECK(chip.read(0x58000488, 4) == 0);
    CHECK(chip.read(0x58000410, 4) == 0);
}

void test_stm32wb55_cpu2_802154_window_clock_gate() {
    stm32wb55emu::Stm32wb55 chip;
    chip.write(0x60001100, 0x12345678, 4);
    CHECK(chip.read(0x60001100, 4) == 0);

    chip.write(0x58000164, 1u << 1, 4); // RCC_C2APB3ENR.802EN.
    CHECK(chip.read(0x60001100, 4) == 0); // The disabled write was ignored.
    chip.write(0x60001100, 0x12345678, 4);
    CHECK(chip.read(0x60001100, 4) == 0x12345678);

    chip.write(0x58000164, 0, 4);
    CHECK(chip.read(0x60001100, 4) == 0);
    chip.write(0x58000164, 1u << 1, 4);
    CHECK(chip.read(0x60001100, 4) == 0x12345678);
}

void test_stm32wb55_cpu2_rf_reset_status_and_registers() {
    stm32wb55emu::Stm32wb55 chip;
    chip.write(0x58000164, 1u << 1, 4);
    chip.write(0x60001100, 0x12345678, 4);
    CHECK(chip.read(0x60001100, 4) == 0x12345678);

    chip.write(0x58000044, 1, 4); // RCC_APB3RSTR.RFRST.
    CHECK(chip.read(0x58000094, 4) & (1u << 16)); // RCC_CSR.RFRSTS.
    CHECK(chip.read(0x60001100, 4) == 0);
    chip.write(0x60001100, 0x87654321, 4); // Writes during reset are ignored.

    chip.write(0x58000044, 0, 4);
    CHECK((chip.read(0x58000094, 4) & (1u << 16)) == 0);
    CHECK(chip.read(0x60001100, 4) == 0);
    chip.write(0x60001100, 0x87654321, 4);
    CHECK(chip.read(0x60001100, 4) == 0x87654321);
}

} // namespace mcuemu_tests
