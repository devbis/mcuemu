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

void test_stm32wb55_cpu2_rf_indirect_register_transfer() {
    std::vector<std::uint8_t> image(0x34, 0);
    put32(image, 0, 0x2003d100); // CPU2 stack in SRAM2.
    put32(image, 4, 0x00000009); // reset handler at CPU2 code alias address 0x8.
    const auto code = words({0x4806, // ldr r0, [pc, #24] -> descriptor at 0x10000100
                             0x4907, // ldr r1, [pc, #28] -> PHY command register
                             0x4a07, // ldr r2, [pc, #28] -> busy + descriptor offset
                             0x600a, // str r2, [r1]: write the descriptor
                             0x2383, // movs r3, #0x83: read two bytes
                             0x7003, // strb r3, [r0]
                             0x600a, // str r2, [r1]: read the descriptor
                             0x7883, // ldrb r3, [r0, #2]
                             0x4c05, // ldr r4, [pc, #20] -> output at 0x10000120
                             0x7023, // strb r3, [r4]
                             0x78c3, // ldrb r3, [r0, #3]
                             0x7063, // strb r3, [r4, #1]
                             0xe7fe}); // b .
    std::memcpy(image.data() + 8, code.data(), code.size());
    put32(image, 0x24, 0x10000100);
    put32(image, 0x28, 0x6000000c);
    put32(image, 0x2c, 0x00010100);
    put32(image, 0x30, 0x10000120);

    stm32wb55emu::Stm32wb55 chip;
    chip.load_cpu2_firmware(image, 0x000dd000);
    const std::uint8_t descriptor[] = {3, 5, 0x12, 0x34, 0};
    CHECK(chip.core().write_memory(0x20030100, descriptor, sizeof(descriptor)));
    chip.write(0x5800040c, 1u << 15, 4); // CPU1 sets C2BOOT.

    const auto result = chip.run_cpu2_until(5000);
    CHECK(result.reason == cortexm::StopReason::Deadline);
    std::uint8_t readback[2]{};
    CHECK(chip.core().read_memory(0x20030120, readback, sizeof(readback)));
    CHECK(readback[0] == 0x12);
    CHECK(readback[1] == 0x34);
    CHECK((chip.read(0x6000000c, 4) & (1u << 16)) == 0);
}

void test_stm32wb55_thread_rcp_single_byte_rf_register_access() {
    std::vector<std::uint8_t> image(0x34, 0);
    put32(image, 0, 0x2003d100); // CPU2 stack in SRAM2.
    put32(image, 4, 0x00000009); // reset handler at CPU2 code alias address 0x8.
    const auto code = words({0x4806, // ldr r0, [pc, #24] -> descriptor at 0x10000100
                             0x4907, // ldr r1, [pc, #28] -> RF command register
                             0x4a07, // ldr r2, [pc, #28] -> busy + descriptor offset
                             0x600a, // str r2, [r1]: write one RF register byte
                             0x2382, // movs r3, #0x82: read one byte from RF register 0x2d
                             0x7003, // strb r3, [r0]
                             0x600a, // str r2, [r1]: submit read descriptor
                             0x7883, // ldrb r3, [r0, #2]
                             0x4c05, // ldr r4, [pc, #20] -> output at 0x10000120
                             0x7023, // strb r3, [r4]
                             0xe7fe}); // b .
    std::memcpy(image.data() + 8, code.data(), code.size());
    put32(image, 0x24, 0x10000100);
    put32(image, 0x28, 0x6000000c);
    put32(image, 0x2c, 0x00010100);
    put32(image, 0x30, 0x10000120);

    stm32wb55emu::Stm32wb55 chip;
    chip.load_cpu2_firmware(image, 0x000da000);
    const std::uint8_t descriptor[] = {2, 0x2d, 0x60, 0};
    CHECK(chip.core().write_memory(0x20030100, descriptor, sizeof(descriptor)));
    chip.write(0x5800040c, 1u << 15, 4); // CPU1 sets C2BOOT.

    const auto result = chip.run_cpu2_until(5000);
    CHECK(result.reason == cortexm::StopReason::Deadline);
    std::uint8_t readback = 0;
    CHECK(chip.core().read_memory(0x20030120, &readback, sizeof(readback)));
    CHECK(readback == 0x60);
    CHECK((chip.read(0x6000000c, 4) & (1u << 16)) == 0);
}

void test_stm32wb55_cpu2_ipcc_channel_directions() {
    std::vector<std::uint8_t> image(0x20, 0);
    put32(image, 0, 0x2003d100); // CPU2 stack in SRAM2.
    put32(image, 4, 0x00000009); // reset handler at CPU2 code alias address 0x8.
    const auto code = words({0x4803, // ldr r0, [pc, #12] -> C2SCR
                             0x4904, // ldr r1, [pc, #16] -> clear RX1 and set TX1
                             0x6001, // str r1, [r0]
                             0xe7fe}); // b .
    std::memcpy(image.data() + 8, code.data(), code.size());
    put32(image, 0x18, 0x58000c18);
    put32(image, 0x1c, 0x00010001);

    stm32wb55emu::Stm32wb55 chip;
    chip.load_cpu2_firmware(image, 0x000da000);
    chip.write(0x58000c08, 1u << 16, 4); // CPU1 sets channel 1 for CPU2.
    CHECK(chip.read(0x58000c0c, 4) & 1u);
    chip.write(0x5800040c, 1u << 15, 4); // CPU1 sets C2BOOT.

    const auto result = chip.run_cpu2_until(5000);
    CHECK(result.reason == cortexm::StopReason::Deadline);
    CHECK((chip.read(0x58000c0c, 4) & 1u) == 0);
    CHECK((chip.read(0x58000c1c, 4) & 1u) != 0);
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
