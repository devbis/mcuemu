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

void test_stm32wb55_cpu1_zero_based_vtor_flash_alias() {
    std::vector<std::uint8_t> image(0x12c, 0);
    put32(image, 0, 0x2003d100);
    put32(image, 4, 0x08000101);
    put32(image, (16 + 19) * 4, 0x08000121);

    const auto reset = words({0x4804, // ldr r0, [pc, #16] -> VTOR
                              0x2100, // movs r1, #0
                              0x6001, // str r1, [r0]: relocate VTOR to zero
                              0x4804, // ldr r0, [pc, #16] -> NVIC ISER
                              0x4904, // ldr r1, [pc, #16] -> IRQ19 bit
                              0x6001, // str r1, [r0]
                              0xe7fe}); // b .
    std::memcpy(image.data() + 0x100, reset.data(), reset.size());
    put32(image, 0x114, 0xe000ed08);
    put32(image, 0x118, 0xe000e100);
    put32(image, 0x11c, 1u << 19);

    const auto handler = words({0x4801, // ldr r0, [pc, #4] -> IRQ marker
                                0x215a, // movs r1, #0x5a
                                0x6001, // str r1, [r0]
                                0xe7fe}); // b .
    std::memcpy(image.data() + 0x120, handler.data(), handler.size());
    put32(image, 0x128, 0x20030120);

    stm32wb55emu::Stm32wb55 chip;
    chip.load_firmware(image);
    chip.reset();
    chip.core().set_irq_line(19, true);
    const auto result = chip.core().run(2000);
    CHECK(result.reason == cortexm::StopReason::Deadline);
    std::uint32_t marker = 0;
    CHECK(chip.core().read_memory(0x20030120, &marker, sizeof(marker)));
    CHECK(marker == 0x5a);
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
    std::vector<std::uint8_t> image(0x14c, 0);
    put32(image, 0, 0x2003d100); // CPU2 stack in SRAM2.
    put32(image, 4, 0x00000101); // Reset handler at 0x100.
    put32(image, (16 + 19) * 4, 0x00000131); // IPCC IRQ19 handler at 0x130.

    const auto reset = words({0x4805, // ldr r0, [pc, #20] -> IPCC C2CR
                              0x2101, // movs r1, #1: enable receive-channel IRQ
                              0x6001, // str r1, [r0]
                              0x4805, // ldr r0, [pc, #20] -> IPCC C2MR
                              0x2100, // movs r1, #0: unmask channel IRQs
                              0x6001, // str r1, [r0]
                              0x4805, // ldr r0, [pc, #20] -> NVIC ISER
                              0x4906, // ldr r1, [pc, #24] -> IRQ19 bit
                              0x6001, // str r1, [r0]
                              0xe7fe}); // b .
    std::memcpy(image.data() + 0x100, reset.data(), reset.size());
    put32(image, 0x118, 0x58000c10);
    put32(image, 0x11c, 0x58000c14);
    put32(image, 0x124, 0xe000e100);
    put32(image, 0x128, 1u << 19);

    const auto handler = words({0x4803, // ldr r0, [pc, #12] -> C2SCR
                                0x4904, // ldr r1, [pc, #16] -> clear RX1 and set TX1
                                0x6001, // str r1, [r0]
                                0x4804, // ldr r0, [pc, #16] -> IRQ marker
                                0x215a, // movs r1, #0x5a
                                0x6001, // str r1, [r0]
                                0x4770}); // bx lr
    std::memcpy(image.data() + 0x130, handler.data(), handler.size());
    put32(image, 0x140, 0x58000c18);
    put32(image, 0x144, 0x00010001);
    put32(image, 0x148, 0x20030120);

    stm32wb55emu::Stm32wb55 chip;
    chip.load_cpu2_firmware(image, 0x000da000);
    chip.write(0x58000c08, 1u << 16, 4); // CPU1 sets channel 1 for CPU2.
    CHECK(chip.read(0x58000c0c, 4) & 1u);
    chip.write(0x5800040c, 1u << 15, 4); // CPU1 sets C2BOOT.

    const auto result = chip.run_cpu2_until(5000);
    CHECK(result.reason == cortexm::StopReason::Deadline);
    CHECK((chip.read(0x58000c0c, 4) & 1u) == 0);
    CHECK((chip.read(0x58000c1c, 4) & 1u) != 0);
    std::uint32_t marker = 0;
    CHECK(chip.core().read_memory(0x20030120, &marker, sizeof(marker)));
    CHECK(marker == 0x5a);
}

void test_stm32wb55_cpu1_clears_cpu2_ipcc_receive_flag() {
    std::vector<std::uint8_t> cpu1_image(0x168, 0);
    put32(cpu1_image, 0, 0x2003d100);
    put32(cpu1_image, 4, 0x08000101);
    put32(cpu1_image, (16 + 44) * 4, 0x08000151); // CPU1 IPCC RX IRQ44.
    const auto cpu1_reset = words({0x480b, // ldr r0, [pc, #44] -> C1CR
                                   0x2101, // enable RX occupied IRQ
                                   0x6001,
                                   0x480b, // ldr r0, [pc, #44] -> C1MR
                                   0x490b, // ldr r1, [pc, #44] -> unmask channel 2
                                   0x6001,
                                   0x480b, // ldr r0, [pc, #44] -> NVIC ISER1
                                   0x490c, // ldr r1, [pc, #48] -> enable IRQ44
                                   0x6001,
                                   0xe7fe}); // wait for CPU2's receive flag
    std::memcpy(cpu1_image.data() + 0x100, cpu1_reset.data(), cpu1_reset.size());
    put32(cpu1_image, 0x130, 0x58000c00);
    put32(cpu1_image, 0x134, 0x58000c04);
    put32(cpu1_image, 0x138, 0xfffffffdu);
    put32(cpu1_image, 0x13c, 0xe000e104);
    put32(cpu1_image, 0x140, 1u << (44 - 32));
    const auto cpu1_handler = words({0x4803, // ldr r0, [pc, #12] -> C1SCR
                                     0x2102, // clear CPU2-to-CPU1 channel 2
                                     0x6001,
                                     0x4803, // ldr r0, [pc, #12] -> IRQ marker
                                     0x215a, // movs r1, #0x5a
                                     0x6001,
                                     0x4770}); // bx lr
    std::memcpy(cpu1_image.data() + 0x150, cpu1_handler.data(), cpu1_handler.size());
    put32(cpu1_image, 0x160, 0x58000c08);
    put32(cpu1_image, 0x164, 0x20030120);

    std::vector<std::uint8_t> cpu2_image(0x2c, 0);
    put32(cpu2_image, 0, 0x2003d100);
    put32(cpu2_image, 4, 0x00000009); // CPU2 reset handler at the zero alias.
    const auto cpu2_reset = words({0x4805, // ldr r0, [pc, #20] -> C2SCR
                                   0x4906, // ldr r1, [pc, #24] -> set TX channel 2
                                   0x6001,
                                   0x4806, // ldr r0, [pc, #24] -> CPU2 marker
                                   0x2133, // movs r1, #0x33
                                   0x6001,
                                   0xe7fe});
    std::memcpy(cpu2_image.data() + 8, cpu2_reset.data(), cpu2_reset.size());
    put32(cpu2_image, 0x20, 0x58000c18);
    put32(cpu2_image, 0x24, 1u << 17);
    put32(cpu2_image, 0x28, 0x20030124);

    stm32wb55emu::Stm32wb55 chip;
    chip.load_firmware(cpu1_image);
    chip.load_cpu2_firmware(cpu2_image, 0x000da000);
    chip.reset();
    CHECK(chip.core().run(2000).reason == cortexm::StopReason::Deadline);

    chip.write(0x5800040c, 1u << 15, 4); // CPU1 sets C2BOOT.
    const std::uint64_t cpu2_target_ns = chip.core().cycles_to_ns(chip.core().cycles()) + 5000;
    CHECK(chip.run_cpu2_until(cpu2_target_ns).reason == cortexm::StopReason::Deadline);
    std::uint32_t cpu2_marker = 0;
    CHECK(chip.core().read_memory(0x20030124, &cpu2_marker, sizeof(cpu2_marker)));
    CHECK(cpu2_marker == 0x33);
    CHECK(chip.read(0x58000c1c, 4) & (1u << 1));

    CHECK(chip.core().run(5000).reason == cortexm::StopReason::Deadline);
    CHECK((chip.read(0x58000c1c, 4) & (1u << 1)) == 0);
    std::uint32_t marker = 0;
    CHECK(chip.core().read_memory(0x20030120, &marker, sizeof(marker)));
    CHECK(marker == 0x5a);
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

void test_stm32wb55_rcc_prescaler_applied_flags() {
    stm32wb55emu::Stm32wb55 chip;

    chip.write(0x58000008, 0, 4); // HAL writes CFGR even when selecting DIV1.
    CHECK(chip.read(0x58000008, 4) & (1u << 16)); // HPREF.
    CHECK(chip.read(0x58000008, 4) & (1u << 17)); // PPRE1F.
    CHECK(chip.read(0x58000008, 4) & (1u << 18)); // PPRE2F.

    chip.write(0x58000108, 0, 4); // HCLK2 and shared HCLK4 both select DIV1.
    CHECK(chip.read(0x58000108, 4) & (1u << 16)); // SHDHPREF.
    CHECK(chip.read(0x58000108, 4) & (1u << 17)); // C2HPREF.

    chip.write(0x58000008, 0, 4);
    chip.write(0x58000108, 0, 4);
    CHECK(chip.read(0x58000008, 4) & (7u << 16));
    CHECK(chip.read(0x58000108, 4) & (3u << 16));
}

} // namespace mcuemu_tests
