#include "../test_support.hpp"
#include "cortexm/core.hpp"
#include "crypto/aes128.hpp"
#include "phy6252emu/image.hpp"
#include "phy6252emu/phy6252.hpp"
#include <unicorn/arm.h>
#include <unicorn/unicorn.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace {
void put(std::vector<std::uint8_t> &data, std::size_t offset, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i)
        data[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
}

class InvalidateOnWrite final : public cortexm::Bus {
public:
    explicit InvalidateOnWrite(cortexm::Core &core) : core_(core) {}
    std::uint32_t read(std::uint32_t, unsigned) override { return 0; }
    void write(std::uint32_t, std::uint32_t, unsigned) override {
        ++writes;
        core_.request_code_invalidation();
    }
    unsigned writes{0};

private:
    cortexm::Core &core_;
};

void test_aon_gpio_wakeup() {
    using namespace phy6252emu;
    std::vector<std::uint8_t> rom(0x18000, 0xff);
    put(rom, 0, 0x1fff1800); // initial SP
    put(rom, 4, 0x00000201); // Thumb reset vector
    rom[0x200] = 0x02; rom[0x201] = 0x48; // ldr r0, [pc, #8]
    rom[0x202] = 0x5a; rom[0x203] = 0x21; // movs r1, #0x5a
    rom[0x204] = 0x01; rom[0x205] = 0x60; // str r1, [r0]
    rom[0x206] = 0xfe; rom[0x207] = 0xe7; // b .
    put(rom, 0x20c, BootImage::RamBase + 0x100);

    Phy6252 chip;
    chip.load_rom_dump(rom);
    chip.core().reset(0); // initialize the Cortex-M engine before issuing a host MMIO reset
    chip.set_gpio_input(8, false);
    chip.set_gpio_input(3, true);
    chip.write(0x4000f00c, 1u << 12, 4); // GPIO8 falling-edge wake polarity (IOCTL bit 12)
    chip.write(0x4000f0a0, 1u << 14, 4); // GPIO8's AON pad index is 14
    chip.write(0x40008038, (1u << 8) | (1u << 3), 4); // AP_GPIO edge mode
    CHECK(chip.read(0x4000f00c, 4) == (1u << 12));
    CHECK(chip.read(0x4000f0a0, 4) == (1u << 14));
    CHECK(chip.read(0x40008038, 4) == ((1u << 8) | (1u << 3)));
    chip.write(0x4000f004, 0xa5a55a5a, 4); // enter system sleep through external ROM
    CHECK(chip.run(chip.core().cycles() + 16).reason == cortexm::StopReason::Deadline);
    CHECK(chip.read(0x4000f00c, 4) == (1u << 12));
    CHECK(chip.read(0x4000f0a0, 4) == (1u << 14));
    CHECK(chip.read(0x40008038, 4) == ((1u << 8) | (1u << 3)));
    CHECK(chip.read(0x4000f0b0, 4) == 0);

    chip.set_gpio_input(8, true); // rising edge does not match GPIO8's falling polarity
    CHECK(chip.read(0x4000f0b0, 4) == 0);
    CHECK(chip.run(chip.core().cycles() + 16).reason == cortexm::StopReason::Deadline);

    chip.set_gpio_input(3, false); // masked pad does not wake the sleeping core
    CHECK(chip.run(chip.core().cycles() + 16).reason == cortexm::StopReason::Deadline);
    CHECK(chip.read(0x4000f0b0, 4) == 0);

    chip.set_gpio_input(8, false);
    CHECK(chip.read(0x4000f0b0, 4) == (1u << 14));
    CHECK(chip.run(chip.core().cycles() + 16).reason == cortexm::StopReason::Deadline);
    std::uint32_t marker = 0;
    CHECK(chip.core().read_memory(BootImage::RamBase + 0x100, &marker, sizeof marker));
    CHECK(marker == 0x5a);

    Phy6252 upper_group_chip;
    upper_group_chip.load_rom_dump(rom);
    upper_group_chip.core().reset(0);
    upper_group_chip.set_gpio_input(20, false);
    upper_group_chip.write(0x4000f0a4, 1, 4); // logical GPIO20 maps to AON pad 32
    upper_group_chip.write(0x40008038, 1u << 20, 4);
    upper_group_chip.write(0x4000f004, 0xa5a55a5a, 4);
    CHECK(upper_group_chip.run(upper_group_chip.core().cycles() + 16).reason ==
          cortexm::StopReason::Deadline);
    upper_group_chip.set_gpio_input(20, true); // default IOCTL polarity is rising
    CHECK(upper_group_chip.read(0x4000f0b4, 4) == 1);
    CHECK(upper_group_chip.run(upper_group_chip.core().cycles() + 16).reason ==
          cortexm::StopReason::Deadline);
    CHECK(upper_group_chip.core().read_memory(BootImage::RamBase + 0x100,
                                               &marker, sizeof marker));
    CHECK(marker == 0x5a);
}

std::array<std::uint8_t, 8> random_bytes_for_seed(
    const std::vector<std::uint8_t> &image, std::uint32_t seed) {
    using namespace phy6252emu;
    Phy6252 chip;
    chip.load_firmware(image);
    chip.set_random_seed(seed);
    chip.reset();
    constexpr std::uint32_t buffer = BootImage::RamBase + 0x100;
    chip.core().set_reg(0, buffer);
    chip.core().set_reg(1, 8);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x5e04);
    CHECK(chip.run(chip.core().cycles() + 16).reason ==
          cortexm::StopReason::Deadline);
    CHECK(chip.core().reg(0) == 0);
    std::array<std::uint8_t, 8> bytes{};
    CHECK(chip.core().read_memory(buffer, bytes.data(), bytes.size()));
    return bytes;
}

void test_random_seed(const std::vector<std::uint8_t> &image) {
    const auto first = random_bytes_for_seed(image, 0x12345678);
    CHECK(first == random_bytes_for_seed(image, 0x12345678));
    CHECK(first != random_bytes_for_seed(image, 0x12345679));
    CHECK(random_bytes_for_seed(image, 0) ==
          random_bytes_for_seed(image, 0x6d2b79f5));
}

void test_direct_rom_baseband_wrapper(const std::vector<std::uint8_t> &image) {
    using namespace phy6252emu;
    Phy6252 chip;
    chip.load_firmware(image);
    chip.reset();
    CHECK(chip.run(100).reason == cortexm::StopReason::Deadline);

    constexpr std::uint32_t callback_address = BootImage::RamBase + 0x2200;
    constexpr std::uint32_t callback = callback_address | 1u;
    constexpr std::uint32_t marker_address = BootImage::RamBase + 0x2300;
    const std::array<std::uint8_t, 24> callback_code = {
        0x03, 0x49, // ldr r1, [pc, #12] ; marker address
        0x08, 0x60, // str r0, [r1]
        0x01, 0x22, // movs r2, #1
        0x03, 0x49, // ldr r1, [pc, #12] ; LL IRQ clear register
        0x0a, 0x60, // str r2, [r1]
        0x70, 0x47, // bx lr
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x23, 0xff, 0x1f, // marker address
        0x14, 0x10, 0x03, 0x40, // 0x40031014 LL IRQ clear
    };
    const std::uint32_t zero = 0;
    const std::uint32_t enable_radio_irq = 1u << 4;
    const std::uint32_t initial_r0 = 0x13579bdf;
    const std::uint32_t initial_r4 = 0x2468ace0;
    CHECK(chip.core().write_memory(0x1fff0390, &callback, sizeof callback));
    CHECK(chip.core().write_memory(callback_address, callback_code.data(),
                                   callback_code.size()));
    CHECK(chip.core().write_memory(marker_address, &zero, sizeof zero));
    CHECK(chip.core().write_memory(0xe000e100, &enable_radio_irq,
                                   sizeof enable_radio_irq));
    chip.core().set_reg(0, initial_r0);
    chip.core().set_reg(4, initial_r4);
    const std::uint32_t initial_sp = chip.core().reg(13);
    chip.core().set_irq_line(4, true);
    chip.core().set_interrupts_masked(false);
    CHECK(chip.run(chip.core().cycles() + 128).reason ==
          cortexm::StopReason::Deadline);

    std::uint32_t marker = 0;
    CHECK(chip.core().read_memory(marker_address, &marker, sizeof marker));
    CHECK(marker == callback); // ROM's BLX wrapper leaves the callback pointer in r0.
    CHECK(chip.core().reg(0) == initial_r0); // exception return restores interrupted state
    CHECK(chip.core().reg(4) == initial_r4);
    CHECK(chip.core().reg(13) == initial_sp);
}
}

int main() {
    using namespace phy6252emu;
    test_aon_gpio_wakeup();
    std::vector<std::uint8_t> image(0xe120, 0xff);
    put(image, 0, 2); put(image, 8, 0x1fff1838);
    put(image, 0x100, 0x10100); put(image, 0x104, 12); put(image, 0x108, 0x11010100);
    put(image, 0x110, 0x10110); put(image, 0x114, 8); put(image, 0x118, 0x1fff1838);
    put(image, 0xe100, 0x1fff2000); put(image, 0xe104, 0x11010109);
    image[0xe108] = 0xfe; image[0xe109] = 0xe7; // b .
    put(image, 0xe110, 0x1fff2000); put(image, 0xe114, 0x11010109);
    test_direct_rom_baseband_wrapper(image);
    test_random_seed(image);
    BootImage boot = load_boot_image(image);
    CHECK(boot.flash[0x10108] == 0xfe);
    CHECK(boot.ram[0x1838 + 1] == 0x20);
    std::vector<std::uint8_t> full_flash(BootImage::FlashSize, 0xff);
    std::copy(image.begin(), image.end(), full_flash.begin() + 0x2000);
    put(full_flash, 0x2100, BootImage::FlashBase + 0x10100); // absolute XIP source address
    BootImage dump = load_boot_image(full_flash);
    CHECK(dump.flash[0x10108] == 0xfe);
    CHECK(dump.ram[0x1838 + 1] == 0x20);
    cortexm::Core::Config config;
    config.model = cortexm::Model::M0;
    config.irq_count = 32;
    config.priority_bits = 2;
    config.track_last_instruction_pc = true;
    cortexm::Core core(config);
    auto *flash = core.map_memory(BootImage::FlashBase, BootImage::FlashSize, false);
    auto *ram = core.map_memory(BootImage::RamBase, BootImage::RamSize, true);
    std::memcpy(flash, boot.flash.data(), boot.flash.size());
    std::memcpy(ram, boot.ram.data(), boot.ram.size());
    core.reset(boot.run_address);
    CHECK(core.run(100).reason == cortexm::StopReason::Deadline);
    CHECK(core.pc() == 0x11010108);
    CHECK(core.last_instruction_pc() != 0);
    CHECK(core.reg(13) == 0x1fff2000);

    std::vector<std::uint8_t> rom_image(0x18000, 0xff);
    put(rom_image, 0x50, 0x12345679); // external ROM vectors are preserved
    put(rom_image, 0, 0x1fff1800);
    put(rom_image, 4, 0x00000201);
    rom_image[0x200] = 0x34;
    rom_image[0x201] = 0x12;
    Phy6252 dump_rom_chip;
    dump_rom_chip.load_rom_dump(rom_image);
    dump_rom_chip.load_firmware(image);
    dump_rom_chip.reset();
    std::uint32_t dumped_vector = 0;
    CHECK(dump_rom_chip.core().read_memory(0x50, &dumped_vector, sizeof dumped_vector));
    CHECK(dumped_vector == 0x12345679);
    std::array<std::uint8_t, 2> rom_code{};
    CHECK(dump_rom_chip.core().read_memory(0x200, rom_code.data(), rom_code.size()));
    CHECK((rom_code == std::array<std::uint8_t, 2>{0x34, 0x12}));
    CHECK(dump_rom_chip.core().read_memory(0x10000200, rom_code.data(), rom_code.size()));
    CHECK((rom_code == std::array<std::uint8_t, 2>{0x34, 0x12}));
    std::array<std::uint8_t, 2> aliased_flash_code{};
    CHECK(dump_rom_chip.core().read_memory(0x22010108, aliased_flash_code.data(),
                                           aliased_flash_code.size()));
    CHECK((aliased_flash_code == std::array<std::uint8_t, 2>{0xfe, 0xe7}));

    Phy6252 cold_boot_chip;
    cold_boot_chip.load_rom_dump(rom_image);
    cold_boot_chip.load_firmware(image);
    cold_boot_chip.set_boot_from_rom(true);
    cold_boot_chip.reset();
    CHECK(cold_boot_chip.core().reg(13) == 0x1fff1800);
    CHECK(cold_boot_chip.core().pc() == 0x200);
    CHECK(cold_boot_chip.core().read_memory(BootImage::RamBase + 0x1838,
                                           rom_code.data(), rom_code.size()));
    CHECK((rom_code == std::array<std::uint8_t, 2>{0, 0}));
    bool missing_rom_rejected = false;
    try {
        Phy6252 missing_rom_chip;
        missing_rom_chip.set_boot_from_rom(true);
    } catch (const std::invalid_argument &) {
        missing_rom_rejected = true;
    }
    CHECK(missing_rom_rejected);

    std::vector<std::uint8_t> padded_rom(0x20000, 0xff);
    std::copy(rom_image.begin(), rom_image.end(), padded_rom.begin());
    Phy6252 padded_rom_chip;
    padded_rom_chip.load_rom_dump(padded_rom);
    bool bad_rom_rejected = false;
    try {
        Phy6252 invalid_rom_chip;
        invalid_rom_chip.load_rom_dump(std::vector<std::uint8_t>(0x100, 0));
    } catch (const std::invalid_argument &) {
        bad_rom_rejected = true;
    }
    CHECK(bad_rom_rejected);
    padded_rom.back() = 0;
    bad_rom_rejected = false;
    try {
        Phy6252 invalid_rom_chip;
        invalid_rom_chip.load_rom_dump(padded_rom);
    } catch (const std::invalid_argument &) {
        bad_rom_rejected = true;
    }
    CHECK(bad_rom_rejected);

    cortexm::Core::Config m0_config;
    m0_config.model = cortexm::Model::M0;
    m0_config.irq_count = 32;
    m0_config.priority_bits = 2;
    cortexm::Core m0(m0_config);
    auto *m0_memory = m0.map_memory(0, 0x1000, true);
    m0.map_memory(BootImage::RamBase, BootImage::RamSize, true);
    std::uint32_t m0_vectors[16]{};
    m0_vectors[0] = 0x1fffce40;
    m0_vectors[1] = 0x201;
    m0_vectors[3] = 0x501;
    m0_vectors[14] = 0x401;
    std::memcpy(m0_memory + 0x100, m0_vectors, sizeof m0_vectors);
    m0_memory[0x200] = 0xfe; m0_memory[0x201] = 0xe7; // thread loop
    m0_memory[0x400] = 0xfe; m0_memory[0x401] = 0xe7; // PendSV loop
    m0_memory[0x500] = 0xfe; m0_memory[0x501] = 0xe7; // HardFault loop
    m0.reset(0x100);
    std::uint32_t m0_psp = 0x1fffe270, m0_control = 2;
    CHECK(uc_reg_write(m0.engine(), UC_ARM_REG_PSP, &m0_psp) == UC_ERR_OK);
    CHECK(uc_reg_write(m0.engine(), UC_ARM_REG_CONTROL, &m0_control) == UC_ERR_OK);
    const std::uint32_t pend_pendsv = 1u << 28;
    CHECK(m0.write_memory(0xe000ed04, &pend_pendsv, sizeof pend_pendsv));
    CHECK(m0.run(20).reason == cortexm::StopReason::Deadline);
    std::uint32_t m0_msp_after = 0, m0_psp_after = 0, m0_ipsr_after = 0;
    CHECK(uc_reg_read(m0.engine(), UC_ARM_REG_MSP, &m0_msp_after) == UC_ERR_OK);
    CHECK(uc_reg_read(m0.engine(), UC_ARM_REG_PSP, &m0_psp_after) == UC_ERR_OK);
    CHECK(uc_reg_read(m0.engine(), UC_ARM_REG_IPSR, &m0_ipsr_after) == UC_ERR_OK);
    CHECK(m0_msp_after == 0x1fffce40);
    CHECK(m0_psp_after == 0x1fffe250);
    CHECK(m0_ipsr_after == 14);

    cortexm::Core invalidation_core(m0_config);
    InvalidateOnWrite invalidation_bus(invalidation_core);
    auto *invalidation_memory = invalidation_core.map_memory(0x1000, 0x1000, true);
    invalidation_core.map_mmio(0x40000000, 0x1000, invalidation_bus);
    const std::uint32_t initial_sp = 0x3000, initial_pc = 0x1021;
    std::memcpy(invalidation_memory, &initial_sp, sizeof initial_sp);
    std::memcpy(invalidation_memory + 4, &initial_pc, sizeof initial_pc);
    const std::uint8_t invalidation_code[] = {0x01, 0x60, 0x01, 0x22, 0xfe, 0xe7};
    std::memcpy(invalidation_memory + 0x20, invalidation_code, sizeof invalidation_code);
    invalidation_core.reset(0x1000);
    invalidation_core.set_reg(0, 0x40000000);
    invalidation_core.set_reg(1, 0x12345678);
    CHECK(invalidation_core.run(invalidation_core.cycles() + 100).reason ==
          cortexm::StopReason::Deadline);
    CHECK(invalidation_bus.writes == 1);
    CHECK(invalidation_core.reg(2) == 1); // execution resumed after the MMIO callback flush.

    Phy6252 chip;
    chip.load_firmware(image);
    chip.reset();
    CHECK(chip.read(0x4000c858, 4) == 2); // SDK cache init locks the XIP write aperture
    constexpr std::uint32_t flash_store_code = BootImage::RamBase + 0x2f00;
    constexpr std::uint32_t flash_store_target = BootImage::FlashBase + 0x3dff0;
    const std::array<std::uint8_t, 4> flash_store_instructions{0x08, 0x70, 0xfe, 0xe7};
    CHECK(chip.core().write_memory(flash_store_code, flash_store_instructions.data(),
                                   flash_store_instructions.size()));
    auto *flash_store_byte = chip.core().host_pointer(flash_store_target, 1);
    CHECK(flash_store_byte != nullptr);
    *flash_store_byte = 0xff;
    chip.write(0x4000c858, 0, 4); // the ROM temporarily clears WR_PROTECTION
    chip.core().set_reg(0, 0x35);
    chip.core().set_reg(1, flash_store_target);
    chip.core().set_reg(15, flash_store_code);
    CHECK(chip.run(chip.core().cycles() + 8).reason == cortexm::StopReason::Deadline);
    CHECK(*flash_store_byte == 0x35);
    chip.core().set_reg(0, 0x0f);
    chip.core().set_reg(15, flash_store_code);
    CHECK(chip.run(chip.core().cycles() + 8).reason == cortexm::StopReason::Deadline);
    CHECK(*flash_store_byte == 0x05); // a second program can only clear more bits
    chip.write(0x4000c858, 2, 4);
    chip.core().set_reg(0, 0x00);
    chip.core().set_reg(15, flash_store_code);
    CHECK(chip.run(chip.core().cycles() + 8).reason == cortexm::StopReason::Deadline);
    CHECK(*flash_store_byte == 0x05); // a locked AHB store is acknowledged but ignored
    chip.reset(); // restore the timer epoch after exercising the flash programming loop
    CHECK(chip.read(0x4000102c, 4) == 0xffffffffu); // AP_TIM3 free-running CurrentCount
    const std::uint64_t wait_timer_start = chip.core().cycles();
    CHECK(chip.run(wait_timer_start + 16).reason == cortexm::StopReason::Deadline);
    CHECK(chip.read(0x4000102c, 4) == 0xfffffffbu); // one tick per four core cycles
    chip.reset();
    chip.core().set_clock_hz(32000000);
    const std::uint64_t fast_clock_start = chip.core().cycles();
    chip.core().advance_to(fast_clock_start + 32);
    CHECK(chip.read(0x4000102c, 4) == 0xfffffffbu); // still four AP_TIM ticks at 32 MHz HCLK
    chip.write(0x40001064, 3, 4); // Timer5 load count
    chip.write(0x4000106c, 3, 4); // enable one-shot
    const std::uint64_t fast_timer_start = chip.core().cycles();
    chip.core().advance_to(fast_timer_start + 24);
    CHECK(chip.read(0x40001068, 4) == 0);
    CHECK(chip.read(0x40001074, 4) == 1); // three 4 MHz ticks took 24 HCLK cycles
    chip.reset();
    CHECK(chip.read(0x4000f018, 4) == 0x01064040); // captured AON PMCTL1
    CHECK(chip.read(0x4000f054, 4) == 0x8000); // captured efuse_cfg
    CHECK(chip.read(0x4000f058, 4) == 0); // captured chip_state
    CHECK(chip.read(0x4000f05c, 4) == 0x1f2); // captured cal_rw
    CHECK(chip.read(0x4000f060, 4) == 0x10c0); // captured cal_ro0
    CHECK(chip.read(0x4000f064, 4) == 0); // captured cal_ro1
    CHECK(chip.read(0x4000f068, 4) == 0x100); // captured cal_ro2
    chip.write(0x4000f05c, 0x1234, 4);
    CHECK(chip.read(0x4000f05c, 4) == 0x1234);
    chip.write(0x4000f018, 0x010640c0, 4); // SDK RC32K cap-bank ownership strobe
    CHECK(chip.read(0x4000f018, 4) == 0x010640c0);
    chip.reset(); // reload the captured startup calibration values
    std::array<std::uint8_t, 2> rf_tp_cal{};
    CHECK(chip.core().read_memory(0x1fff0a44, rf_tp_cal.data(), rf_tp_cal.size()));
    CHECK(rf_tp_cal[0] == 0x2b); // ROM-owned g_rfPhyTpCal0
    CHECK(rf_tp_cal[1] == 0x23); // ROM-owned g_rfPhyTpCal1
    std::uint32_t vtor_value = 0x1fff8b00, handler_vector = 0;
    CHECK(chip.core().write_memory(0xe000ed08, &vtor_value, sizeof vtor_value));
    CHECK(chip.core().read_memory(0xe000ed08, &vtor_value, sizeof vtor_value));
    CHECK(vtor_value == 0);
    CHECK(chip.core().read_memory(0x0c, &handler_vector, sizeof handler_vector));
    CHECK(handler_vector == 0x28f1);
    CHECK(chip.core().read_memory(0x38, &handler_vector, sizeof handler_vector));
    CHECK(handler_vector == 0x84cd);
    CHECK(chip.core().read_memory(0x50, &handler_vector, sizeof handler_vector));
    CHECK(handler_vector == 0x4e25); // SDK LL_IRQHandler for BB_IRQn=4
    CHECK(chip.core().read_memory(0x80, &handler_vector, sizeof handler_vector));
    CHECK(handler_vector == 0x112d); // SDK GPIO_IRQHandler for GPIO_IRQn=16
    const std::uint32_t vector_word = 0x11010109;
    std::uint32_t vector_readback = 0;
    CHECK(chip.core().write_memory(0, &vector_word, sizeof vector_word));
    CHECK(chip.core().read_memory(0, &vector_readback, sizeof vector_readback));
    CHECK(vector_readback == vector_word);
    chip.reset();
    CHECK(chip.core().read_memory(0, &vector_readback, sizeof vector_readback));
    CHECK(vector_readback == 0);
    CHECK(chip.run(100).reason == cortexm::StopReason::Deadline);
    Phy6252 rom_test_chip;
    rom_test_chip.load_firmware(image);
    rom_test_chip.reset();
    CHECK(rom_test_chip.read(0x40031058, 4) == 0x0200);
    rom_test_chip.write(0x40031058, 0x0100, 4); // SDK setting: 1 KiB TX / 3 KiB RX.
    CHECK(rom_test_chip.read(0x40031058, 4) == 0x0100);
    CHECK(rom_test_chip.run(100).reason == cortexm::StopReason::Deadline);
    constexpr std::uint32_t random_buffer = 0x1fff2100;
    rom_test_chip.core().set_reg(0, random_buffer);
    rom_test_chip.core().set_reg(1, 8);
    rom_test_chip.core().set_reg(14, 0x11010109);
    rom_test_chip.core().set_reg(15, 0x5e04); // SDK LL_Rand entry
    CHECK(rom_test_chip.run(rom_test_chip.core().cycles() + 16).reason == cortexm::StopReason::Deadline);
    CHECK(rom_test_chip.core().reg(0) == 0); // LL_STATUS_SUCCESS
    std::array<std::uint8_t, 8> random_bytes{};
    CHECK(rom_test_chip.core().read_memory(random_buffer, random_bytes.data(), random_bytes.size()));
    CHECK(std::any_of(random_bytes.begin(), random_bytes.end(),
                      [](std::uint8_t byte) { return byte != 0; }));
    rom_test_chip.core().set_reg(0, random_buffer);
    rom_test_chip.core().set_reg(1, 0); // SDK LL_Rand rejects an empty request
    rom_test_chip.core().set_reg(14, 0x11010109);
    rom_test_chip.core().set_reg(15, 0x5e04);
    CHECK(rom_test_chip.run(rom_test_chip.core().cycles() + 16).reason == cortexm::StopReason::Deadline);
    CHECK(rom_test_chip.core().reg(0) == 0x12); // LL_STATUS_ERROR_BAD_PARAMETER
    const std::array<std::uint8_t, 4> unaligned_word{0x12, 0x34, 0x56, 0x78};
    CHECK(rom_test_chip.core().write_memory(random_buffer + 1, unaligned_word.data(), unaligned_word.size()));
    rom_test_chip.core().set_reg(0, random_buffer + 1);
    rom_test_chip.core().set_reg(14, 0x11010109);
    rom_test_chip.core().set_reg(15, 0x0f74); // ARM runtime __aeabi_uread4 entry
    CHECK(rom_test_chip.run(rom_test_chip.core().cycles() + 16).reason == cortexm::StopReason::Deadline);
    CHECK(rom_test_chip.core().reg(0) == 0x78563412);
    rom_test_chip.core().set_reg(0, 0x40001000); // AP_TIM1
    rom_test_chip.core().set_reg(1, 2); // set_timer() receives microseconds
    rom_test_chip.core().set_reg(14, 0x11010109);
    rom_test_chip.core().set_reg(15, 0x16c2c);
    CHECK(rom_test_chip.run(rom_test_chip.core().cycles() + 8).reason ==
          cortexm::StopReason::Deadline);
    CHECK(rom_test_chip.read(0x40001000, 4) == 8); // 4 MHz timer count per us
    CHECK(rom_test_chip.read(0x40001008, 4) == 3); // enabled one-shot
    CHECK(rom_test_chip.run(rom_test_chip.core().cycles() + 32).reason ==
          cortexm::StopReason::Deadline);
    CHECK(rom_test_chip.read(0x40001010, 4) == 1);
    CHECK(rom_test_chip.read(0x4000100c, 4) == 1); // EOI clears Timer1 pending
    CHECK(rom_test_chip.read(0x40001010, 4) == 0);
    rom_test_chip.write(0x40001000, 44, 4);
    rom_test_chip.write(0x40001008, 3, 4);
    rom_test_chip.core().set_reg(0, 0x40001000); // clear_timer(AP_TIM1)
    rom_test_chip.core().set_reg(14, 0x11010109);
    rom_test_chip.core().set_reg(15, 0xa5c0);
    CHECK(rom_test_chip.run(rom_test_chip.core().cycles() + 16).reason ==
          cortexm::StopReason::Deadline);
    CHECK(rom_test_chip.read(0x40001000, 4) == 0);
    CHECK(rom_test_chip.read(0x40001004, 4) == 0);
    CHECK(rom_test_chip.read(0x40001008, 4) == 0);
    auto invoke_radio_rom = [&](std::uint32_t address, std::uint32_t arg0 = 0) {
        rom_test_chip.core().set_reg(0, arg0);
        rom_test_chip.core().set_reg(14, 0x11010109);
        rom_test_chip.core().set_reg(15, address);
        CHECK(rom_test_chip.run(rom_test_chip.core().cycles() + 64).reason ==
              cortexm::StopReason::Deadline);
    };
    rom_test_chip.core().set_reg(0, 88); // BB settle
    rom_test_chip.core().set_reg(1, 8); // AFE settle
    rom_test_chip.core().set_reg(2, 40); // PLL settle
    rom_test_chip.core().set_reg(14, 0x11010109);
    rom_test_chip.core().set_reg(15, 0x113dc); // ll_hw_set_trx_settle
    CHECK(rom_test_chip.run(rom_test_chip.core().cycles() + 64).reason ==
          cortexm::StopReason::Deadline);
    CHECK(rom_test_chip.read(0x400300bc, 4) == 0x00580828);
    invoke_radio_rom(0x112a4); // ll_hw_set_srx
    CHECK(rom_test_chip.read(0x40031004, 4) == 0x010001);
    CHECK(rom_test_chip.read(0x40031038, 4) == 7);
    invoke_radio_rom(0x10cdc); // ll_hw_get_tr_mode
    CHECK(rom_test_chip.core().reg(0) == 1);
    invoke_radio_rom(0x112b8); // ll_hw_set_stx
    CHECK(rom_test_chip.read(0x40031004, 4) == 0x000100);
    invoke_radio_rom(0x113c8); // ll_hw_set_trx
    CHECK(rom_test_chip.read(0x40031004, 4) == 0x010102);
    invoke_radio_rom(0x11264); // ll_hw_set_rtx
    CHECK(rom_test_chip.read(0x40031004, 4) == 0x010103);
    invoke_radio_rom(0x11174, 1); // ll_hw_set_irq
    CHECK(rom_test_chip.read(0x4003100c, 4) == 1);
    rom_test_chip.write(0x40031400, 0x78563412, 4);
    invoke_radio_rom(0x110e8); // ll_hw_rst_tfifo
    CHECK(rom_test_chip.read(0x40031400, 4) == 0);
    rom_test_chip.write(0x40031c00, 0x78563412, 4);
    invoke_radio_rom(0x110b0); // ll_hw_rst_rfifo
    CHECK(rom_test_chip.read(0x40031c00, 4) == 0);

    mcuemu::VirtualAir phy_air;
    rom_test_chip.connect_radio(phy_air, "phy6252-rom-test");
    rom_test_chip.write(0x40031400, 5, 1); // PHR: three payload bytes and FCS
    rom_test_chip.write(0x40031401, 0x01, 1);
    rom_test_chip.write(0x40031402, 0x00, 1);
    rom_test_chip.write(0x40031403, 0x5a, 1);
    invoke_radio_rom(0x112b8); // ll_hw_set_stx
    invoke_radio_rom(0x10cf8); // ll_hw_go
    CHECK(rom_test_chip.read(0x4003100c, 4) == 1); // ll_hw_go uses MD-only IRQs
    CHECK((rom_test_chip.read(0x40030094, 4) & 0xffu) == 0x2b); // clamped channel 2 TP cap
    CHECK(phy_air.frames().size() == 1);
    if (!phy_air.frames().empty()) {
        CHECK(phy_air.frames()[0].payload == std::vector<std::uint8_t>({0x01, 0x00, 0x5a}));
        const std::uint64_t tx_done = rom_test_chip.core().ns_to_cycles(phy_air.frames()[0].end_ns);
        CHECK(rom_test_chip.run(tx_done + 64).reason == cortexm::StopReason::Deadline);
    }
    CHECK((rom_test_chip.read(0x40031010, 4) & 1u) != 0);
    invoke_radio_rom(0x10adc); // ll_hw_clr_irq
    CHECK(rom_test_chip.read(0x40031010, 4) == 0);

    invoke_radio_rom(0x112a4); // ll_hw_set_srx
    invoke_radio_rom(0x11420); // ll_hw_trigger
    CHECK(rom_test_chip.radio_receiving());
    const std::uint64_t phy_rx_start = rom_test_chip.core().cycles_to_ns(
        rom_test_chip.core().cycles()) + 1000;
    const std::uint64_t phy_rx_id = phy_air.transmit(
        "coordinator", {}, mcuemu::RadioProtocol::Zigbee802154, 11, phy_rx_start,
        {0x01, 0x00, 0xa5});
    const auto phy_rx_frame = std::find_if(phy_air.frames().begin(), phy_air.frames().end(),
        [phy_rx_id](const mcuemu::RadioFrame &frame) { return frame.id == phy_rx_id; });
    CHECK(phy_rx_frame != phy_air.frames().end());
    if (phy_rx_frame != phy_air.frames().end()) {
        const std::uint64_t rx_done = rom_test_chip.core().ns_to_cycles(phy_rx_frame->end_ns + 20000);
        CHECK(rom_test_chip.run(rx_done + 64).reason == cortexm::StopReason::Deadline);
    }
    const std::uint32_t radio_irq_status = rom_test_chip.read(0x40031010, 4);
    CHECK((radio_irq_status & 1u) != 0); // LIRQ_MD
    CHECK((radio_irq_status & (1u << 10)) != 0); // LIRQ_COK
    CHECK((radio_irq_status & (1u << 1)) != 0); // PB-03F Zephyr COK alias
    CHECK(static_cast<std::int8_t>(rom_test_chip.read(0x400300e4, 4) >> 24) == -50);
    CHECK((rom_test_chip.read(0x400300e8, 4) >> 24) == 50); // SDK RF RSSI is positive -dBm
    CHECK(rom_test_chip.read(0x40031054, 4) == (16u << 16)); // eight packet bytes plus two footers
    invoke_radio_rom(0x10bbc); // ll_hw_get_irq_status
    CHECK(rom_test_chip.core().reg(0) == radio_irq_status);
    constexpr std::uint32_t radio_packet = 0x1fff2300;
    constexpr std::uint32_t radio_length = 0x1fff2320;
    constexpr std::uint32_t radio_foot0 = 0x1fff2324;
    constexpr std::uint32_t radio_foot1 = 0x1fff2328;
    rom_test_chip.core().set_reg(1, radio_length);
    rom_test_chip.core().set_reg(2, radio_foot0);
    rom_test_chip.core().set_reg(3, radio_foot1);
    invoke_radio_rom(0x10f50, radio_packet); // ll_hw_read_rfifo_zb
    CHECK(rom_test_chip.core().reg(0) == 2); // two FIFO words copied by ll_hw_read_rfifo_zb
    std::array<std::uint8_t, 8> received_packet{};
    std::uint16_t received_length = 0;
    std::uint32_t received_foot0 = 0, received_foot1 = 0;
    CHECK(rom_test_chip.core().read_memory(radio_packet, received_packet.data(),
                                           received_packet.size()));
    CHECK(rom_test_chip.core().read_memory(radio_length, &received_length,
                                           sizeof received_length));
    CHECK(rom_test_chip.core().read_memory(radio_foot0, &received_foot0,
                                           sizeof received_foot0));
    CHECK(rom_test_chip.core().read_memory(radio_foot1, &received_foot1,
                                           sizeof received_foot1));
    CHECK(received_length == 6); // blen + 1 includes the leading length byte
    CHECK(received_packet[0] == 5 && received_packet[1] == 0x01 &&
          received_packet[2] == 0x00 && received_packet[3] == 0xa5);
    CHECK(received_packet[6] == 0 && received_packet[7] == 0);
    CHECK((received_foot0 & 0x00ffffffu) == 2);
    CHECK((received_foot0 >> 24) == 0);
    CHECK((received_foot1 >> 24) == 50);
    CHECK(rom_test_chip.read(0x40031054, 4) == ((16u << 16) | 16u));
    rom_test_chip.write(0x40031014, 1u << 1, 4); // Zephyr W1C clears the SDK COK alias too
    CHECK(rom_test_chip.read(0x40031010, 4) == 1u); // LIRQ_MD remains pending
    invoke_radio_rom(0x110b0); // ll_hw_rst_rfifo
    CHECK(rom_test_chip.read(0x40031c00, 1) == 0);
    invoke_radio_rom(0x10f50, radio_packet); // empty FIFO returns zero and clears outputs
    CHECK(rom_test_chip.core().reg(0) == 0);
    CHECK(rom_test_chip.core().read_memory(radio_packet, received_packet.data(),
                                           received_packet.size()));
    CHECK(rom_test_chip.core().read_memory(radio_length, &received_length,
                                           sizeof received_length));
    CHECK(rom_test_chip.core().read_memory(radio_foot0, &received_foot0,
                                           sizeof received_foot0));
    CHECK(rom_test_chip.core().read_memory(radio_foot1, &received_foot1,
                                           sizeof received_foot1));
    CHECK(received_packet[0] == 0 && received_length == 0 &&
          received_foot0 == 0 && received_foot1 == 0);
    CHECK(rom_test_chip.read(0x40031054, 4) == 0);

    invoke_radio_rom(0x10adc);
    invoke_radio_rom(0x112a4);
    invoke_radio_rom(0x11420);
    const std::uint64_t phy_rx_stream_start = rom_test_chip.core().cycles_to_ns(
        rom_test_chip.core().cycles()) + 1000;
    const std::uint64_t phy_rx_stream_id = phy_air.transmit(
        "coordinator", {}, mcuemu::RadioProtocol::Zigbee802154, 11, phy_rx_stream_start,
        {0x01, 0x00, 0xa5});
    const auto phy_rx_stream_frame = std::find_if(phy_air.frames().begin(), phy_air.frames().end(),
        [phy_rx_stream_id](const mcuemu::RadioFrame &frame) { return frame.id == phy_rx_stream_id; });
    CHECK(phy_rx_stream_frame != phy_air.frames().end());
    if (phy_rx_stream_frame != phy_air.frames().end()) {
        const std::uint64_t rx_done = rom_test_chip.core().ns_to_cycles(
            phy_rx_stream_frame->end_ns + 20000);
        CHECK(rom_test_chip.run(rx_done + 64).reason == cortexm::StopReason::Deadline);
    }
    CHECK(rom_test_chip.read(0x40031054, 4) == (16u << 16));
    const std::uint32_t rx_word0 = rom_test_chip.read(0x40031c00, 4);
    const std::uint32_t rx_word1 = rom_test_chip.read(0x40031c00, 4);
    const std::uint32_t rx_foot0 = rom_test_chip.read(0x40031c00, 4);
    const std::uint32_t rx_foot1 = rom_test_chip.read(0x40031c00, 4);
    CHECK(rx_word0 == 0xa5000105u);
    CHECK(rx_word1 == (static_cast<std::uint32_t>(received_packet[4]) |
                       (static_cast<std::uint32_t>(received_packet[5]) << 8)));
    CHECK(rx_foot0 == 2);
    CHECK(rx_foot1 == (50u << 24));
    CHECK(rom_test_chip.read(0x40031054, 4) == ((16u << 16) | 16u));
    rom_test_chip.write(0x40031000, 0, 4); // PHY_ABORT stops the receive operation
    CHECK(!rom_test_chip.radio_receiving());
    invoke_radio_rom(0x10adc); // ll_hw_clr_irq
    const std::uint32_t enable_radio_irq = 1u << 4;
    CHECK(rom_test_chip.core().write_memory(0xe000e280, &enable_radio_irq,
                                             sizeof enable_radio_irq)); // clear stale pending IRQ
    CHECK(rom_test_chip.core().write_memory(0xe000e100, &enable_radio_irq,
                                             sizeof enable_radio_irq));
    invoke_radio_rom(0x11174, 1u); // firmware waits for LIRQ_MD on receive completion
    invoke_radio_rom(0x11278, 33); // ll_hw_set_rx_timeout(33 us)
    CHECK(rom_test_chip.read(0x40031028, 4) == 33);
    invoke_radio_rom(0x112a4); // ll_hw_set_srx
    invoke_radio_rom(0x11420); // ll_hw_trigger
    CHECK(rom_test_chip.radio_receiving());
    rom_test_chip.core().set_interrupts_masked(true); // inspect NVIC pending without a test vector
    const std::uint64_t rx_timeout = rom_test_chip.core().cycles() + 33 * 16 + 128;
    CHECK(rom_test_chip.run(rx_timeout).reason == cortexm::StopReason::Deadline);
    CHECK(!rom_test_chip.radio_receiving());
    CHECK(rom_test_chip.read(0x40031010, 4) ==
          (1u | (1u << 2) | (1u << 3))); // MD completion plus SDK/Zephyr RTO aliases
    CHECK(rom_test_chip.core().interrupt_pending()); // MD mask raises the radio IRQ
    // SDK and PB-03F Zephyr expose RTO at bits 2 and 3 respectively.
    rom_test_chip.write(0x40031014, (1u << 0) | (1u << 3), 4);
    CHECK(rom_test_chip.read(0x40031010, 4) == 0);

    rom_test_chip.core().set_interrupts_masked(true);
    rom_test_chip.write(0x40031028, 33, 4); // external ROM writes ll_hw_set_rx_timeout directly
    invoke_radio_rom(0x112a4);
    invoke_radio_rom(0x11420);
    CHECK(rom_test_chip.radio_receiving());
    const std::uint64_t direct_rx_timeout = rom_test_chip.core().cycles() + 33 * 16 + 128;
    CHECK(rom_test_chip.run(direct_rx_timeout).reason == cortexm::StopReason::Deadline);
    CHECK(!rom_test_chip.radio_receiving());
    CHECK((rom_test_chip.read(0x40031010, 4) & ((1u << 0) | (1u << 2) | (1u << 3))) ==
          ((1u << 0) | (1u << 2) | (1u << 3)));
    rom_test_chip.write(0x40031014, (1u << 0) | (1u << 3), 4);
    rom_test_chip.core().set_interrupts_masked(false);

    Phy6252 cca_chip;
    cca_chip.load_firmware(image);
    cca_chip.reset();
    CHECK(cca_chip.run(100).reason == cortexm::StopReason::Deadline);
    mcuemu::VirtualAir cca_air;
    cca_chip.connect_radio(cca_air, "cca-probe");
    const std::uint64_t cca_sample_ns = cca_chip.core().cycles_to_ns(
        cca_chip.core().cycles());
    cca_air.transmit("cca-peer", {}, mcuemu::RadioProtocol::Zigbee802154, 11,
                     cca_sample_ns - 1000, {0x01, 0x00, 0x33});
    const std::int8_t cca_rssi = static_cast<std::int8_t>(
        cca_chip.read(0x400300e4, 4) >> 24);
    CHECK(cca_rssi == -50); // ideal shared-air propagation sample used by CCA

    Phy6252 irq_chip;
    irq_chip.load_firmware(image);
    irq_chip.reset();
    CHECK(irq_chip.run(100).reason == cortexm::StopReason::Deadline);
    constexpr std::uint32_t irq_callback = 0x1fff2201;
    constexpr std::uint32_t gpio_callback_argument = 0x1fff2310;
    CHECK(irq_chip.core().write_memory(0x1fff03c0, &irq_callback, sizeof irq_callback));
    const std::array<std::uint8_t, 36> gpio_irq_callback = {
        0x05, 0x49, // ldr r1, [pc, #20] ; incoming r0 marker
        0x08, 0x60, // str r0, [r1]
        0x07, 0x20, // movs r0, #7
        0x05, 0x49, // ldr r1, [pc, #20] ; marker address
        0x08, 0x60, // str r0, [r1]
        0x01, 0x20, // movs r0, #1
        0x04, 0x49, // ldr r1, [pc, #16] ; GPIO interrupt status
        0x08, 0x60, // str r0, [r1] ; clear edge status
        0x70, 0x47, // bx lr
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x10, 0x23, 0xff, 0x1f, // 0x1fff2310 incoming r0 marker
        0x00, 0x23, 0xff, 0x1f, // 0x1fff2300 marker
        0x4c, 0x80, 0x00, 0x40, // 0x4000804c GPIO interrupt status
    };
    CHECK(irq_chip.core().write_memory(0x1fff2200, gpio_irq_callback.data(),
                                       gpio_irq_callback.size()));
    irq_chip.write(0x40008038, 1, 4); // edge-triggered GPIO0
    irq_chip.write(0x4000803c, 1, 4); // rising edge
    irq_chip.write(0x40008034, ~1u, 4); // unmask GPIO0
    irq_chip.write(0x40008030, 1, 4); // enable GPIO0 interrupt
    const std::uint32_t enable_gpio_irq = 1u << 16;
    CHECK(irq_chip.core().write_memory(0xe000e100, &enable_gpio_irq,
                                       sizeof enable_gpio_irq));
    CHECK(irq_chip.run(irq_chip.core().cycles() + 20).reason ==
          cortexm::StopReason::Deadline);
    std::uint32_t irq_marker = 0;
    CHECK(irq_chip.core().read_memory(0x1fff2300, &irq_marker, sizeof irq_marker));
    CHECK(irq_marker == 0); // changing level mode to edge is not itself an edge
    CHECK(irq_chip.core().read_memory(gpio_callback_argument, &irq_marker,
                                       sizeof irq_marker));
    CHECK(irq_marker == 0);
    irq_chip.set_gpio_input(0, true);
    CHECK(irq_chip.run(irq_chip.core().cycles() + 100).reason ==
          cortexm::StopReason::Deadline);
    CHECK(irq_chip.core().read_memory(0x1fff2300, &irq_marker, sizeof irq_marker));
    CHECK(irq_marker == 7);
    CHECK(irq_chip.core().read_memory(gpio_callback_argument, &irq_marker,
                                       sizeof irq_marker));
    CHECK(irq_marker == irq_callback); // ROM's GPIO wrapper tail-branches with its pointer in r0.

    Phy6252 timer_irq_chip;
    timer_irq_chip.load_firmware(image);
    timer_irq_chip.reset();
    CHECK(timer_irq_chip.run(100).reason == cortexm::StopReason::Deadline);
    constexpr std::uint32_t timer_irq_callback = 0x1fff2201;
    CHECK(timer_irq_chip.core().write_memory(0x1fff03e4, &timer_irq_callback,
                                              sizeof timer_irq_callback)); // V25_IRQ_HANDLER
    const std::array<std::uint8_t, 20> timer_irq_handler = {
        0x07, 0x20, // movs r0, #7
        0x02, 0x49, // ldr r1, [pc, #8] ; marker address
        0x08, 0x60, // str r0, [r1]
        0x02, 0x49, // ldr r1, [pc, #8] ; Timer6 EOI
        0x08, 0x68, // ldr r0, [r1] ; SDK clears EOI by reading it
        0x70, 0x47, // bx lr
        0x00, 0x23, 0xff, 0x1f, // 0x1fff2300 marker
        0x70, 0x10, 0x00, 0x40, // 0x40001070 Timer6 EOI
    };
    CHECK(timer_irq_chip.core().write_memory(0x1fff2200, timer_irq_handler.data(),
                                              timer_irq_handler.size()));
    const std::uint32_t enable_timer6_irq = 1u << 25;
    CHECK(timer_irq_chip.core().write_memory(0xe000e100, &enable_timer6_irq,
                                              sizeof enable_timer6_irq));
    const std::uint64_t timer_irq_start = timer_irq_chip.core().cycles();
    timer_irq_chip.write(0x40001064, 3, 4); // Timer6 load count
    timer_irq_chip.write(0x4000106c, 3, 4); // enable one-shot with IRQ unmasked
    CHECK(timer_irq_chip.run(timer_irq_start + 100).reason ==
          cortexm::StopReason::Deadline);
    CHECK(timer_irq_chip.core().read_memory(0x1fff2300, &irq_marker, sizeof irq_marker));
    CHECK(irq_marker == 7);
    CHECK(timer_irq_chip.read(0x40001074, 4) == 0);

    Phy6252 timer1_irq_chip;
    timer1_irq_chip.load_firmware(image);
    timer1_irq_chip.reset();
    CHECK(timer1_irq_chip.run(100).reason == cortexm::StopReason::Deadline);
    CHECK(timer1_irq_chip.core().write_memory(0x1fff03d0, &timer_irq_callback,
                                               sizeof timer_irq_callback)); // V20_IRQ_HANDLER
    const std::array<std::uint8_t, 20> timer1_irq_handler = {
        0x07, 0x20, // movs r0, #7
        0x02, 0x49, // ldr r1, [pc, #8] ; marker address
        0x08, 0x60, // str r0, [r1]
        0x02, 0x49, // ldr r1, [pc, #8] ; Timer1 EOI
        0x08, 0x68, // ldr r0, [r1] ; SDK clears EOI by reading it
        0x70, 0x47, // bx lr
        0x00, 0x23, 0xff, 0x1f, // 0x1fff2300 marker
        0x0c, 0x10, 0x00, 0x40, // 0x4000100c Timer1 EOI
    };
    CHECK(timer1_irq_chip.core().write_memory(0x1fff2200, timer1_irq_handler.data(),
                                               timer1_irq_handler.size()));
    const std::uint32_t enable_timer1_irq = 1u << 20;
    CHECK(timer1_irq_chip.core().write_memory(0xe000e100, &enable_timer1_irq,
                                               sizeof enable_timer1_irq));
    timer1_irq_chip.core().set_reg(0, 0x40001000); // AP_TIM1
    timer1_irq_chip.core().set_reg(1, 1);
    timer1_irq_chip.core().set_reg(14, 0x11010109);
    timer1_irq_chip.core().set_reg(15, 0x16c2c);
    CHECK(timer1_irq_chip.run(timer1_irq_chip.core().cycles() + 8).reason ==
          cortexm::StopReason::Deadline);
    CHECK(timer1_irq_chip.run(timer1_irq_chip.core().cycles() + 100).reason ==
          cortexm::StopReason::Deadline);
    CHECK(timer1_irq_chip.core().read_memory(0x1fff2300, &irq_marker, sizeof irq_marker));
    CHECK(irq_marker == 7);
    CHECK(timer1_irq_chip.read(0x40001010, 4) == 0);

    const std::uint32_t fifo_probe = 0x11030000;
    auto *fifo_probe_code = chip.core().host_pointer(fifo_probe, 6);
    CHECK(fifo_probe_code != nullptr);
    // STR r0,[r1]; LDR r2,[r1]; B . — exercise both MMIO windows via the CPU.
    const std::uint8_t fifo_probe_instructions[] = {0x08, 0x60, 0x0a, 0x68, 0xfe, 0xe7};
    std::memcpy(fifo_probe_code, fifo_probe_instructions, sizeof fifo_probe_instructions);
    for (const auto [address, value] : {
             std::pair<std::uint32_t, std::uint32_t>{0x40031de8, 0x12345678},
             {0x400323fc, 0xaabbccdd},
         }) {
        chip.core().set_reg(0, value);
        chip.core().set_reg(1, address);
        chip.core().set_reg(15, fifo_probe);
        const auto fifo_probe_run = chip.run(chip.core().cycles() + 32);
        CHECK(fifo_probe_run.reason == cortexm::StopReason::Deadline);
        CHECK(chip.core().reg(2) == value);
    }
    chip.write(0x4000400c, 0x80, 4); // DLAB selects the divisor
    chip.write(0x40004000, 17, 4);
    CHECK(chip.read(0x40004000, 4) == 17);
    CHECK(chip.take_uart_tx().empty());
    chip.write(0x4000400c, 0, 4);
    chip.write(0x40004000, 'A', 4);
    CHECK(chip.take_uart_tx() == std::vector<std::uint8_t>{'A'});
    chip.uart_receive('B');
    CHECK((chip.read(0x40004014, 4) & 1) != 0);
    CHECK(chip.read(0x40004000, 4) == 'B');
    chip.core().set_reg(0, 2);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0xa5ec);
    CHECK(chip.run(200).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().reg(0) == 0);
    CHECK(chip.core().clock_hz() == 16000000);
    chip.core().set_reg(0, 1);
    chip.core().set_reg(15, 0xa5ec);
    CHECK(chip.run(250).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().clock_hz() == 32000000);
    const auto clock_change_ns = chip.core().cycles_to_ns(chip.core().cycles());
    chip.core().set_reg(0, 2);
    chip.core().set_reg(15, 0xa5ec);
    CHECK(chip.run(275).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().cycles_to_ns(chip.core().cycles()) >= clock_change_ns);
    chip.core().set_reg(15, 0xa5d0);
    CHECK(chip.run(300).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().reg(0) == 16000000);
    chip.core().set_reg(15, 0xa9c8);
    CHECK(chip.run(350).reason == cortexm::StopReason::Deadline);
    CHECK(!chip.core().interrupts_masked());
    chip.core().set_reg(15, 0xaeac);
    CHECK(chip.run(400).reason == cortexm::StopReason::Deadline);
    chip.core().set_reg(15, 0xa920);
    CHECK(chip.run(450).reason == cortexm::StopReason::Deadline);
    chip.core().set_reg(0, 1);
    chip.core().set_reg(15, 0x16b44);
    CHECK(chip.run(475).reason == cortexm::StopReason::Deadline);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0xa2e0);
    CHECK(chip.run(500).reason == cortexm::StopReason::Deadline);
    chip.core().set_reg(0, 0x001f6580);
    chip.core().set_reg(1, 0x0001c200);
    chip.core().set_reg(15, 0x0e08);
    CHECK(chip.run(501).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().reg(0) == 17);
    CHECK(chip.core().reg(1) == 0x18380);
    chip.core().set_reg(0, static_cast<std::uint32_t>(-100));
    chip.core().set_reg(1, 9);
    chip.core().set_reg(15, 0x0e34);
    CHECK(chip.run(502).reason == cortexm::StopReason::Deadline);
    CHECK(static_cast<std::int32_t>(chip.core().reg(0)) == -11);
    CHECK(static_cast<std::int32_t>(chip.core().reg(1)) == -1);
    chip.core().set_reg(15, 0xa000);
    const auto unknown = chip.run(503);
    CHECK(unknown.reason == cortexm::StopReason::Fault);
    CHECK(unknown.message.find("unimplemented PHY6252 ROM call") != std::string::npos);
    chip.core().set_reg(15, 0x15338);
    const auto crypto_invalid = chip.run(504);
    CHECK(crypto_invalid.reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().reg(0) == static_cast<std::uint32_t>(-22));

    CHECK(chip.read(0x4000c800, 4) == 0x80000000u);
    chip.write(0x4000c890, 0x05900001, 4);
    CHECK((chip.read(0x4000c890, 4) & 2) == 0);
    CHECK(chip.read(0x4000c8a0, 4) == 0);
    auto *spif_erase_bytes = chip.core().host_pointer(BootImage::FlashBase + 0x7f000, 4);
    CHECK(spif_erase_bytes != nullptr);
    spif_erase_bytes[0] = 0;
    spif_erase_bytes[1] = 0x55;
    spif_erase_bytes[2] = 0xaa;
    chip.write(0x4000c890, 0x06000001, 4); // ROM spif_cmd(FLASH_WREN).
    chip.write(0x4000c890, 0x05900001, 4);
    CHECK(chip.read(0x4000c8a0, 4) == 2); // status register WEL bit.
    chip.write(0x4000c894, 0x7f000, 4); // 4 KiB sector address is a flash offset.
    chip.write(0x4000c890, 0x200a0001, 4); // ROM spif_cmd(FLASH_SE, 3-byte address).
    CHECK(std::all_of(spif_erase_bytes, spif_erase_bytes + 4,
                      [](std::uint8_t byte) { return byte == 0xff; }));
    chip.write(0x4000c890, 0x05900001, 4);
    CHECK(chip.read(0x4000c8a0, 4) == 0); // erase consumes the write-enable latch.
    CHECK(chip.read(0x40002000, 1) == 0); // watchdog is disabled after reset
    chip.write(0x40002004, 3, 4); // SDK WDG_16S timeout selection
    CHECK(chip.read(0x40002004, 4) == 3);
    chip.write(0x40002000, 0x1d, 1); // enable, reset response, system reset
    CHECK(chip.read(0x40002000, 1) == 0x1d);
    CHECK(chip.read(0x40002008, 4) != 0);
    chip.write(0x4000200c, 0x76, 4); // feed/restart key
    chip.write(0x40002000, 0, 1);
    CHECK(chip.read(0x40002008, 4) == 0);
    chip.write(0x4000c000, 2, 4);
    CHECK(chip.read(0x4000c000, 4) == 2);
    chip.write(0x4000c824, 0x181c, 4); // AP_SPIF remap address register
    CHECK(chip.read(0x4000c824, 4) == 0x181c);
    const std::uint32_t dma_source = BootImage::RamBase + 0x5000;
    const std::uint32_t dma_destination = BootImage::RamBase + 0x5100;
    const std::uint8_t dma_bytes[] = {1, 2, 3, 4, 5, 6, 7, 8};
    CHECK(chip.core().write_memory(dma_source, dma_bytes, sizeof dma_bytes));
    chip.write(0x40010108, dma_source, 4); // DMA channel 3 SAR
    chip.write(0x40010110, dma_destination, 4); // DMA channel 3 DAR
    chip.write(0x40010124, 2, 4); // two 32-bit transfers
    chip.write(0x40010398, 1, 4); // DMA enabled
    chip.write(0x400103a0, 0x808, 4); // software request / enable channel 3
    std::uint8_t dma_result[sizeof dma_bytes]{};
    CHECK(chip.core().read_memory(dma_destination, dma_result, sizeof dma_result));
    CHECK(std::equal(dma_bytes, dma_bytes + sizeof dma_bytes, dma_result));
    CHECK(chip.read(0x400102c0, 4) == 8);
    chip.write(0x40010338, 8, 4); // clear transfer-complete status
    CHECK(chip.read(0x400102c0, 4) == 0);
    constexpr std::uint32_t dma0_source = BootImage::RamBase + 0x5200;
    constexpr std::uint32_t dma0_destination = BootImage::RamBase + 0x5300;
    const std::uint8_t dma0_bytes[] = {0x31, 0x42, 0x53, 0x64, 0x75, 0x86};
    CHECK(chip.core().write_memory(dma0_source, dma0_bytes, sizeof dma0_bytes));
    chip.write(0x40010000, dma0_source, 4); // DMA channel 0 SAR
    chip.write(0x40010008, dma0_destination, 4); // DMA channel 0 DAR
    chip.write(0x40010018, 1, 4); // byte-wide M2M, transfer-complete interrupt
    chip.write(0x4001001c, sizeof dma0_bytes, 4); // byte count
    chip.write(0x40010040, 0, 4); // CFG
    chip.write(0x40010044, 4, 4); // CFG_H from the observed ROM helper
    chip.write(0x40010398, 1, 4);
    chip.write(0x400103a0, 0x101, 4); // channel 0 write-enable + start
    std::uint8_t dma0_result[sizeof dma0_bytes]{};
    CHECK(chip.core().read_memory(dma0_destination, dma0_result, sizeof dma0_result));
    CHECK(std::equal(dma0_bytes, dma0_bytes + sizeof dma0_bytes, dma0_result));
    CHECK(chip.read(0x400102c0, 4) == 1);
    chip.write(0x40010338, 1, 4);
    CHECK(chip.read(0x400102c0, 4) == 0);

    constexpr std::uint32_t dma0_flash_destination = BootImage::FlashBase + 0x7fff0;
    const std::array<std::uint8_t, 4> dma0_flash_first{0xf0, 0x0f, 0xaa, 0x55};
    const std::array<std::uint8_t, 4> dma0_flash_second{0x0f, 0xf0, 0x55, 0xaa};
    CHECK(chip.core().write_memory(dma0_source, dma0_flash_first.data(), dma0_flash_first.size()));
    chip.write(0x40010008, dma0_flash_destination, 4);
    chip.write(0x4001001c, dma0_flash_first.size(), 4);
    chip.write(0x4000c858, 0, 4); // DMA flash writes use the unlocked SPIF aperture.
    chip.write(0x400103a0, 0x101, 4);
    auto *dma0_flash = chip.core().host_pointer(dma0_flash_destination, dma0_flash_first.size());
    CHECK(dma0_flash != nullptr);
    CHECK(std::equal(dma0_flash_first.begin(), dma0_flash_first.end(), dma0_flash));
    CHECK(chip.core().write_memory(dma0_source, dma0_flash_second.data(), dma0_flash_second.size()));
    chip.write(0x400103a0, 0x101, 4);
    CHECK(std::all_of(dma0_flash, dma0_flash + dma0_flash_first.size(),
                      [](std::uint8_t byte) { return byte == 0; })); // NOR cannot set bits.
    chip.write(0x4000c858, 2, 4);
    chip.write(0x40000024, 2, 4);
    CHECK(chip.read(0x40000024, 4) == 2);
    chip.write(0x40000008, 0x180, 4);
    CHECK(chip.read(0x40000008, 4) == 0x180);
    chip.write(0x40000014, 0x12345678, 4); // PCR SW_CLK1
    CHECK(chip.read(0x40000014, 4) == 0x12345678);
    chip.write(0x40000018, 0x87654321, 4); // PCR APB_CLK
    CHECK(chip.read(0x40000018, 4) == 0x87654321);
    chip.write(0x4000380c, 0x60, 4);
    CHECK(chip.read(0x4000380c, 4) == 0x60);
    chip.write(0x4000381c, 0x50400, 4);
    CHECK(chip.read(0x4000381c, 4) == 0x50400);
    chip.write(0x40003820, 0x12345678, 4);
    CHECK(chip.read(0x40003820, 4) == 0x12345678);
    chip.write(0x4000f008, 0x20000000, 4);
    CHECK(chip.read(0x4000f008, 4) == 0x20000000);
    chip.write(0x4000f054, 0x12, 4); // AON eFuse configuration
    CHECK(chip.read(0x4000f054, 4) == 0x12);
    chip.write(0x4000f01c, 0x78, 4); // AON PMCTL2_0
    CHECK(chip.read(0x4000f01c, 4) == 0x78);
    chip.write(0x4000f03c, 0x12, 4); // AON RTC clock selection
    CHECK(chip.read(0x4000f03c, 4) == 0x12);
    chip.write(0x4000f0c0, 0x04, 4); // retained AON sleep-state words
    chip.write(0x4000f0c4, 0x80, 4);
    chip.write(0x4000f0c8, 0x12345678, 4);
    chip.write(0x4000f0cc, 0x87654321, 4);
    CHECK(chip.read(0x4000f0c0, 4) == 0x04);
    CHECK(chip.read(0x4000f0c4, 4) == 0x80);
    CHECK(chip.read(0x4000f0c8, 4) == 0x12345678);
    CHECK(chip.read(0x4000f0cc, 4) == 0x87654321);
    chip.write(0x4000f0a0, 0x12345678, 4); // AON pad indices 0..31 wake mask
    chip.write(0x4000f0a4, 7, 4); // AON pad indices 32..34 wake mask
    CHECK(chip.read(0x4000f0a0, 4) == 0x12345678);
    CHECK(chip.read(0x4000f0a4, 4) == 7);
    CHECK(chip.read(0x4000f0a8, 4) == 0); // captured REG_S11 sleep flag
    chip.write(0x4000f0a8, 1, 4);
    CHECK(chip.read(0x4000f0a8, 4) == 1);
    chip.write(0x4000f0a8, 0, 4);
    CHECK(chip.read(0x4000f0a8, 4) == 0);
    CHECK(chip.read(0x4000f0b0, 4) == 0); // no modeled GPIO wake event
    CHECK(chip.read(0x4000f0b4, 4) == 0);
    chip.write(0x4000f140, 0x34, 4); // eFuse programming control, not OTP contents
    chip.write(0x4000f144, 0x56, 4);
    CHECK(chip.read(0x4000f140, 4) == 0x34);
    CHECK(chip.read(0x4000f144, 4) == 0x56);
    chip.write(0x40008008, 0x12345678, 4); // observed SWPORTA_CTL, pad effect unknown
    CHECK(chip.read(0x40008008, 4) == 0x12345678);
    chip.write(0x4000c850, 0, 4); // SPIF lower write-protection register
    chip.write(0x4000c854, 0x10, 4);
    chip.write(0x4000c858, 2, 4);
    CHECK(chip.read(0x4000c850, 4) == 0);
    CHECK(chip.read(0x4000c854, 4) == 0x10);
    CHECK(chip.read(0x4000c858, 4) == 2);

    auto invoke_rom = [&chip](std::uint32_t pc, std::uint32_t r0,
                              std::uint32_t r1, std::uint32_t r2,
                              std::uint64_t deadline, std::uint32_t r3 = 0) {
        chip.core().set_reg(0, r0);
        chip.core().set_reg(1, r1);
        chip.core().set_reg(2, r2);
        chip.core().set_reg(3, r3);
        chip.core().set_reg(14, 0x11010109);
        chip.core().set_reg(15, pc);
        const auto result = chip.run(deadline);
        CHECK(result.reason == cortexm::StopReason::Deadline);
    };
    invoke_rom(0x16b14, 1, 27, 300, chip.core().cycles() + 8);
    CHECK(chip.core().reg(0) == chip.read(0x4000f028, 4));
    const std::uint32_t source = BootImage::RamBase + 0x3000;
    const std::uint32_t destination = BootImage::FlashBase + 0x12010;
    invoke_rom(0xa974, 0, 0, 0, 526);
    CHECK(chip.core().reg(0) == 0 && chip.core().interrupts_masked());
    invoke_rom(0xa99c, 0, 0, 0, 527);
    CHECK(!chip.core().interrupts_masked());
    Phy6252 context_chip;
    context_chip.load_firmware(image);
    context_chip.reset();
    CHECK(context_chip.run(100).reason == cortexm::StopReason::Deadline);
    constexpr std::uint32_t context_buffer = BootImage::RamBase + 0x4000;
    constexpr std::uint32_t table_base = BootImage::RamBase + 0x4200;
    constexpr std::uint32_t context_stack = BootImage::RamBase + 0xef00;
    constexpr std::uint32_t context_state = 0x1fff0914;
    std::vector<std::uint8_t> context_bytes(0x284, 0xaa);
    const auto invoke_context = [&context_chip, context_buffer, table_base, context_stack](
        std::uint32_t context_count, std::uint32_t slot_count,
        std::array<std::uint32_t, 2> stack_args) {
        CHECK(context_chip.core().write_memory(context_stack, stack_args.data(),
                                                sizeof stack_args));
        context_chip.core().set_reg(0, context_buffer);
        context_chip.core().set_reg(1, table_base);
        context_chip.core().set_reg(2, context_count);
        context_chip.core().set_reg(3, slot_count);
        context_chip.core().set_reg(13, context_stack);
        context_chip.core().set_reg(14, 0x11010109);
        context_chip.core().set_reg(15, 0x5044);
        const auto result = context_chip.run(context_chip.core().cycles() + 64);
        CHECK(result.reason == cortexm::StopReason::Deadline);
    };
    CHECK(context_chip.core().write_memory(context_buffer, context_bytes.data(),
                                            context_bytes.size()));
    invoke_context(1, 2, {0, 0});
    CHECK(context_chip.core().reg(0) == 0);
    std::vector<std::uint8_t> expected_context(0x284, 0);
    put(expected_context, 0x16c, table_base);
    put(expected_context, 0x170, table_base + 0x30);
    put(expected_context, 0x1ac, table_base + 0x60);
    put(expected_context, 0x1b0, table_base + 0x90);
    put(expected_context, 0x1b4, table_base + 0xc0);
    CHECK(context_chip.core().read_memory(context_buffer, context_bytes.data(),
                                           context_bytes.size()));
    CHECK(context_bytes == expected_context);
    std::array<std::uint8_t, 4> context_state_bytes{};
    std::uint32_t context_buffer_pointer = 0;
    CHECK(context_chip.core().read_memory(context_state, context_state_bytes.data(),
                                           context_state_bytes.size()));
    CHECK(context_state_bytes == (std::array<std::uint8_t, 4>{1, 2, 0, 0}));
    CHECK(context_chip.core().read_memory(context_state + 0x20, &context_buffer_pointer,
                                           sizeof context_buffer_pointer));
    CHECK(context_buffer_pointer == context_buffer);

    context_bytes.assign(context_bytes.size(), 0xaa);
    CHECK(context_chip.core().write_memory(context_buffer, context_bytes.data(),
                                            context_bytes.size()));
    invoke_context(1, 1, {1, 1});
    CHECK(context_chip.core().reg(0) == 0);
    expected_context.assign(expected_context.size(), 0);
    put(expected_context, 0x16c, table_base);
    put(expected_context, 0x18c, table_base + 0x10c);
    put(expected_context, 0x1ac, table_base + 0x218);
    put(expected_context, 0x1b0, table_base + 0x324);
    CHECK(context_chip.core().read_memory(context_buffer, context_bytes.data(),
                                           context_bytes.size()));
    CHECK(context_bytes == expected_context);

    context_bytes.assign(context_bytes.size(), 0xaa);
    CHECK(context_chip.core().write_memory(context_buffer, context_bytes.data(),
                                            context_bytes.size()));
    invoke_context(9, 1, {0, 0});
    CHECK(context_chip.core().reg(0) == 0x12);
    CHECK(context_chip.core().read_memory(context_buffer, context_bytes.data(),
                                           context_bytes.size()));
    CHECK(context_bytes.front() == 0xaa);
    const std::uint32_t heap_start = BootImage::RamBase + 0x4000;
    invoke_rom(0x14cb4, heap_start, 0x100, 0, 530);
    CHECK(chip.core().reg(0) == 0);
    invoke_rom(0x14b3c, 13, 0, 0, 531);
    const std::uint32_t heap_first = chip.core().reg(0);
    CHECK(heap_first == heap_start);
    invoke_rom(0x14b3c, 32, 0, 0, 532);
    const std::uint32_t heap_second = chip.core().reg(0);
    CHECK(heap_second == heap_start + 16);
    invoke_rom(0x14c00, heap_first, 0, 0, 533);
    invoke_rom(0x14b3c, 8, 0, 0, 534);
    CHECK(chip.core().reg(0) == heap_first);
    invoke_rom(0x14c00, heap_first, 0, 0, 535);
    invoke_rom(0x14c00, heap_second, 0, 0, 536);
    const std::uint32_t efuse_buffer = BootImage::RamBase + 0x3020;
    const std::uint8_t untouched_efuse_data[] = {1, 2, 3, 4, 5, 6, 7, 8};
    CHECK(chip.core().write_memory(efuse_buffer, untouched_efuse_data,
                                  sizeof untouched_efuse_data));
    invoke_rom(0xace0, 1, efuse_buffer, 0x60, 539, 0x1fff2b29);
    CHECK(chip.core().reg(0) == 24); // no device dump: PPlus_ERR_UNINITIALIZED
    std::uint8_t efuse_data[8]{};
    CHECK(chip.core().read_memory(efuse_buffer, efuse_data, sizeof efuse_data));
    CHECK(std::equal(std::begin(untouched_efuse_data), std::end(untouched_efuse_data),
                     std::begin(efuse_data)));
    invoke_rom(0xace0, 4, efuse_buffer, 0, 540, 0x1fff2b29);
    CHECK(chip.core().reg(0) == 6); // PPlus_ERR_INVALID_PARAM
    std::vector<std::uint8_t> efuse_dump(32);
    for (std::size_t i = 0; i < efuse_dump.size(); ++i)
        efuse_dump[i] = static_cast<std::uint8_t>(0xa0 + i);
    chip.load_efuse_dump(efuse_dump);
    CHECK(chip.read(0x4000f180, 4) == 0x0f); // captured SECURTY_STATE
    chip.write(0x4000f054, 1u << 18, 4); // select eFuse block 2
    CHECK(chip.read(0x4000f170, 4) == 0xb3b2b1b0);
    CHECK(chip.read(0x4000f174, 4) == 0xb7b6b5b4);
    chip.write(0x4000f054, 0, 4); // clear the read strobe
    bool short_efuse_rejected = false;
    try {
        chip.load_efuse_dump(std::vector<std::uint8_t>(31));
    } catch (const std::invalid_argument &) {
        short_efuse_rejected = true;
    }
    CHECK(short_efuse_rejected);
    invoke_rom(0xace0, 2, efuse_buffer, 0xdeadbeef, 541, 0x1fff2b29);
    CHECK(chip.core().reg(0) == 0);
    CHECK(chip.core().read_memory(efuse_buffer, efuse_data, sizeof efuse_data));
    CHECK(std::equal(efuse_dump.begin() + 16, efuse_dump.begin() + 24,
                     std::begin(efuse_data)));
    invoke_rom(0xace0, 2, 0x100, 0, 542, 0x1fff2b29);
    CHECK(chip.core().reg(0) == 6); // invalid output pointer
    invoke_rom(0xace0, 1, efuse_buffer, 0x60, 543, 0x1fff2b29);
    CHECK(chip.core().reg(0) == 0);
    CHECK(chip.core().read_memory(efuse_buffer, efuse_data, sizeof efuse_data));
    CHECK(std::equal(efuse_dump.begin() + 8, efuse_dump.begin() + 16,
                     std::begin(efuse_data)));
    invoke_rom(0xace0, 3, efuse_buffer, 0x1fff0100, 544, 0x10);
    CHECK(chip.core().reg(0) == 0);
    CHECK(chip.core().read_memory(efuse_buffer, efuse_data, sizeof efuse_data));
    CHECK(std::equal(efuse_dump.begin() + 24, efuse_dump.begin() + 32,
                     std::begin(efuse_data)));
    chip.load_efuse_dump(std::vector<std::uint8_t>(32, 0));
    CHECK(chip.core().write_memory(efuse_buffer, untouched_efuse_data,
                                  sizeof untouched_efuse_data));
    invoke_rom(0xace0, 1, efuse_buffer, 0x60, 545, 0x1fff2b29);
    CHECK(chip.core().reg(0) == 24); // zero-valued device block is uninitialized
    CHECK(chip.core().read_memory(efuse_buffer, efuse_data, sizeof efuse_data));
    CHECK(std::all_of(std::begin(efuse_data), std::end(efuse_data),
                      [](std::uint8_t byte) { return byte == 0; }));
    const std::vector<std::uint8_t> rf_calibration = {0x2b, 0x21, 0x45, 0x43};
    chip.load_rf_calibration_dump(rf_calibration);
    chip.write(0x400300b4, 2, 4);
    chip.write(0x400300d8, 1u << 18, 4); // fDev 0
    chip.write(0x400300a4, 0, 4);
    chip.write(0x400300a4, 0x114, 4); // trigger rf_tp_cal
    CHECK(chip.read(0x400300f4, 4) == (0x2bu << 16));
    chip.write(0x400300b4, 66, 4);
    chip.write(0x400300a4, 0x114, 4);
    CHECK(chip.read(0x400300f4, 4) == (0x21u << 16));
    chip.write(0x400300b4, 2, 4);
    chip.write(0x400300d8, 2u << 18, 4); // fDev 1
    chip.write(0x400300a4, 0x114, 4);
    CHECK(chip.read(0x400300f4, 4) == (0x45u << 16));
    chip.write(0x400300b4, 66, 4);
    chip.write(0x400300a4, 0x114, 4);
    CHECK(chip.read(0x400300f4, 4) == (0x43u << 16));
    bool short_rf_calibration_rejected = false;
    try {
        chip.load_rf_calibration_dump(std::vector<std::uint8_t>(3));
    } catch (const std::invalid_argument &) {
        short_rf_calibration_rejected = true;
    }
    CHECK(short_rf_calibration_rejected);
    chip.write(0x400300a4, 0x140, 4); // opaque external-ROM RF init register
    CHECK(chip.read(0x400300a4, 4) == 0x140);
    chip.write(0x4003008c, 0x00800000, 4); // opaque external-ROM RF init register
    CHECK(chip.read(0x4003008c, 4) == 0x00800000);
    chip.write(0x40030124, 0x1, 4); // opaque external-ROM RF init register
    CHECK(chip.read(0x40030124, 4) == 0x1);
    chip.write(0x40030050, 0x22085580, 4); // opaque external-ROM RF init register
    CHECK(chip.read(0x40030050, 4) == 0x22085580);
    chip.write(0x40030054, 0x545c9ca4, 4); // opaque external-ROM PHY init table
    CHECK(chip.read(0x40030054, 4) == 0x545c9ca4);
    chip.write(0x40030074, 0x1c222e28, 4); // end of observed PHY init table
    CHECK(chip.read(0x40030074, 4) == 0x1c222e28);
    chip.write(0x400300a0, 0x0e, 4); // opaque PLL auto override
    CHECK(chip.read(0x400300a0, 4) == 0x0e);
    chip.write(0x400300a8, 0x40, 4); // SDK RX DCOC LO override
    chip.write(0x400300ac, 0x50, 4); // SDK RX LO and PLL buffer
    chip.write(0x40030078, 0x0d3564, 4); // SDK RX DCOC phase-search setup
    CHECK(chip.read(0x400300ec, 4) == 0); // captured calibration uses SDK default result
    CHECK(chip.read(0x400300a8, 4) == 0x40);
    chip.write(0x400300a8, 0, 4);
    CHECK(chip.read(0x400300a8, 4) == 0);
    chip.write(0x4003102c, 0x00010000, 4); // opaque external-ROM LL setup register
    CHECK(chip.read(0x4003102c, 4) == 0x00010000);
    chip.write(0x40031024, 0x1f4, 4); // opaque external-ROM LL timing register
    CHECK(chip.read(0x40031024, 4) == 0x1f4);
    chip.write(0x40031060, 0x000ea600, 4); // opaque external-ROM LL setup register
    CHECK(chip.read(0x40031060, 4) == 0x000ea600);
    chip.write(0x40031020, 0x01400020, 4); // opaque external-ROM LL setup register
    CHECK(chip.read(0x40031020, 4) == 0x01400020);
    chip.write(0x40031068, 7, 4); // opaque external-ROM LL setup register
    CHECK(chip.read(0x40031068, 4) == 7);
    CHECK(chip.core().write_memory(efuse_buffer, untouched_efuse_data,
                                  sizeof untouched_efuse_data));
    const crypto::Block aes_key = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                   0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
    const crypto::Block aes_input = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                     0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    const crypto::Block aes_expected = {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
                                        0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a};
    const std::uint32_t aes_input_address = BootImage::RamBase + 0x3040;
    const std::uint32_t aes_key_address = BootImage::RamBase + 0x3060;
    const std::uint32_t aes_output_address = BootImage::RamBase + 0x3080;
    CHECK(chip.core().write_memory(aes_input_address, aes_input.data(), aes_input.size()));
    CHECK(chip.core().write_memory(aes_key_address, aes_key.data(), aes_key.size()));
    invoke_rom(0x3fdc, aes_input_address, aes_key_address, aes_output_address, 550);
    crypto::Block aes_output{};
    CHECK(chip.core().read_memory(aes_output_address, aes_output.data(), aes_output.size()));
    CHECK(aes_output == aes_expected);
    const std::uint32_t copy_address = source + 0x100;
    invoke_rom(0x14ccc, efuse_buffer, efuse_buffer, sizeof efuse_data, 555);
    CHECK(chip.core().reg(0) == 0);
    invoke_rom(0x14ce8, copy_address, efuse_buffer, sizeof efuse_data, 560);
    std::uint8_t copied_data[8]{};
    CHECK(chip.core().read_memory(copy_address, copied_data, sizeof copied_data));
    CHECK(std::equal(std::begin(untouched_efuse_data), std::end(untouched_efuse_data),
                     std::begin(copied_data)));
    copied_data[0] ^= 1;
    CHECK(chip.core().write_memory(copy_address, copied_data, sizeof copied_data));
    invoke_rom(0x14ccc, efuse_buffer, copy_address, sizeof copied_data, 565);
    CHECK(static_cast<std::int32_t>(chip.core().reg(0)) > 0);
    constexpr char format[] = "v=%02x %d\r\n";
    const std::uint32_t format_address = source + 0x200;
    CHECK(chip.core().write_memory(format_address, format, sizeof format));
    invoke_rom(0x13ae4, format_address, 0x2b, 0xfffffffeu, 568);
    const auto uart_log = chip.take_uart_tx();
    CHECK(std::string(uart_log.begin(), uart_log.end()) == "v=2b -2\r\n");
    const std::uint8_t first_write[] = {0x0f, 0xf0, 0xaa};
    CHECK(chip.core().write_memory(source, first_write, sizeof first_write));
    invoke_rom(0x17394, destination, source, sizeof first_write, 570);
    CHECK(chip.core().reg(0) == 0);
    auto *chip_flash = chip.core().host_pointer(destination, sizeof first_write);
    CHECK(chip_flash != nullptr);
    CHECK(chip_flash[0] == 0x0f && chip_flash[1] == 0xf0 && chip_flash[2] == 0xaa);

    const std::uint8_t second_write[] = {0xff, 0x0f, 0x55};
    CHECK(chip.core().write_memory(source, second_write, sizeof second_write));
    invoke_rom(0x17394, destination, source, sizeof second_write, 600);
    CHECK(chip.core().reg(0) == 0);
    CHECK(chip_flash[0] == 0x0f && chip_flash[1] == 0x00 && chip_flash[2] == 0x00);
    chip.reset();
    chip_flash = chip.core().host_pointer(destination, sizeof second_write);
    CHECK(chip_flash != nullptr && chip_flash[0] == 0x0f && chip_flash[1] == 0 && chip_flash[2] == 0);

    invoke_rom(0x16fa8, BootImage::FlashBase + 0x12000, 0, 0, 700);
    CHECK(chip.core().reg(0) == 0);
    chip_flash = chip.core().host_pointer(destination, sizeof second_write);
    CHECK(chip_flash != nullptr && chip_flash[0] == 0xff && chip_flash[1] == 0xff && chip_flash[2] == 0xff);

    const std::uint8_t dma_write[] = {0xa5, 0x5a};
    const std::uint32_t flash_dma_destination = BootImage::FlashBase + 0x13020;
    CHECK(chip.core().write_memory(source, dma_write, sizeof dma_write));
    invoke_rom(0x1744c, flash_dma_destination, source, sizeof dma_write, 710);
    CHECK(chip.core().reg(0) == 0);
    auto *dma_flash = chip.core().host_pointer(flash_dma_destination, sizeof dma_write);
    CHECK(dma_flash != nullptr && dma_flash[0] == 0xa5 && dma_flash[1] == 0x5a);

    invoke_rom(0x1744c, BootImage::FlashBase + BootImage::FlashSize - 1,
               source, 2, 720);
    CHECK(chip.core().reg(0) == static_cast<std::uint32_t>(-22));

    invoke_rom(0x17394, BootImage::FlashBase + BootImage::FlashSize - 1,
               source, 2, 800);
    CHECK(chip.core().reg(0) == static_cast<std::uint32_t>(-22));
    invoke_rom(0x16fa8, BootImage::FlashBase + 1, 0, 0, 900);
    CHECK(chip.core().reg(0) == static_cast<std::uint32_t>(-22));

    chip.read(0x4000c81c, 4);
    const auto unknown_register = chip.run(1000);
    CHECK(unknown_register.reason == cortexm::StopReason::Fault);
    CHECK(unknown_register.message.find("unimplemented PHY6252 MMIO read at 0x4000c81c") != std::string::npos);

    chip.reset();
    const std::uint32_t rom_stack = BootImage::RamBase + 0xff00;
    const std::uint32_t zero = 0;
    CHECK(chip.core().write_memory(rom_stack, &zero, sizeof zero));
    chip.core().set_reg(13, rom_stack);
    chip.core().set_reg(3, 0);
    chip.write(0x4000c890, 0x9fa00001, 4); // RDID, three-byte response
    CHECK(chip.read(0x4000c890, 4) == 0x9fa00001);
    CHECK(chip.read(0x4000c8a0, 4) == 0x001364c8); // captured FlashID 0x1364c8
    invoke_rom(0x16dc4, 0, 1, 0x0801003b, 1001);
    CHECK(chip.core().reg(0) == 0);
    CHECK(chip.read(0x4000c804, 4) == 0x0801003b);
    CHECK(chip.read(0x4000c800, 4) & 0x80000000u);
    const std::uint32_t spif_command_args[] = {0, 0};
    CHECK(chip.core().write_memory(rom_stack, spif_command_args, sizeof spif_command_args));
    chip.core().set_reg(13, rom_stack);
    chip.core().set_reg(0, 5);
    chip.core().set_reg(1, 0);
    chip.core().set_reg(2, 2);
    chip.core().set_reg(3, 0);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x16d48);
    CHECK(chip.run(1002).reason == cortexm::StopReason::Deadline);
    CHECK(chip.read(0x4000c890, 4) == 0x00020005);
    const std::uint32_t status_destination = BootImage::RamBase + 0x3200;
    std::uint8_t status_byte = 0xff;
    CHECK(chip.core().write_memory(status_destination, &status_byte, 1));
    invoke_rom(0x1713c, status_destination, 1, 0, 1003);
    CHECK(chip.core().reg(0) == 0);
    CHECK(chip.core().read_memory(status_destination, &status_byte, 1));
    CHECK(status_byte == 0);
    CHECK(chip.core().write_memory(rom_stack, spif_command_args, sizeof spif_command_args));
    invoke_rom(0x16d48, 0x9f, 0, 3, 1004);
    CHECK(chip.core().reg(0) == 0);
    const std::uint32_t jedec_destination = BootImage::RamBase + 0x3210;
    std::array<std::uint8_t, 3> jedec_bytes{};
    invoke_rom(0x1713c, jedec_destination, jedec_bytes.size(), 0, 1005);
    CHECK(chip.core().read_memory(jedec_destination, jedec_bytes.data(), jedec_bytes.size()));
    CHECK((jedec_bytes == std::array<std::uint8_t, 3>{0xc8, 0x64, 0x13}));
    chip.write(0x4000f024, 2, 4); // RTC clear strobe
    CHECK(chip.read(0x4000f028, 4) == 0);
    chip.write(0x4000f02c, 100, 4);
    chip.write(0x4000f030, 200, 4);
    chip.write(0x4000f034, 300, 4);
    CHECK(chip.read(0x4000f034, 4) == 300);
    chip.write(0x4000f024, 1, 4); // run the 32.768 kHz RTC
    chip.core().advance_to(chip.core().cycles() + 16000); // approximately 1 ms at 16 MHz
    CHECK(chip.read(0x4000f028, 4) >= 31);
    CHECK(chip.read(0x4000f028, 4) <= 33);
    const std::uint32_t switch_table = BootImage::RamBase + 0x3300;
    const std::uint8_t switch_bytes[] = {2, 4, 6, 0, 0, 0, 0, 0, 0xfe, 0xe7};
    CHECK(chip.core().write_memory(switch_table, switch_bytes, sizeof switch_bytes));
    chip.core().set_reg(3, 0);
    chip.core().set_reg(14, switch_table + 1);
    chip.core().set_reg(15, 0x8960);
    const std::uint32_t switch_case = switch_table + 8;
    CHECK(chip.run(chip.core().cycles() + 8).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().pc() == switch_case);

    mcuemu::VirtualAir air;
    chip.connect_radio(air, "phy6252");
    chip.write(0x400300b4, 5, 4); // IEEE 802.15.4 channel 11
    chip.write(0x40031004, 0x000100, 4); // TX operation
    chip.write(0x4003100c, 1, 4); // TX done interrupt
    chip.write(0x40031000, 1, 4); // Empty command FIFO does not complete a TX.
    CHECK(air.frames().empty());
    CHECK((chip.read(0x40031010, 4) & 1u) == 0);
    const std::uint8_t tx_frame[] = {0x01, 0x00, 0x5a};
    chip.write(0x40031400, sizeof(tx_frame) + 2, 1); // PHR includes the FCS
    for (std::size_t i = 0; i < sizeof tx_frame; ++i)
        chip.write(0x40031401 + static_cast<std::uint32_t>(i), tx_frame[i], 1);
    chip.write(0x40031000, 1, 4);
    CHECK(air.frames().size() == 1);
    std::uint64_t tx_done = chip.core().cycles() + 1;
    if (!air.frames().empty()) {
        CHECK(air.frames()[0].channel == 11);
        CHECK(air.frames()[0].payload == std::vector<std::uint8_t>(tx_frame, tx_frame + sizeof tx_frame));
        tx_done = chip.core().ns_to_cycles(air.frames()[0].end_ns);
    }
    CHECK(chip.run(tx_done + 100).reason == cortexm::StopReason::Deadline);
    CHECK((chip.read(0x40031010, 4) & 1) != 0);
    chip.write(0x40031014, 0x1f, 4);

    chip.write(0x40031004, 0x010001, 4); // RX operation
    chip.write(0x4003100c, 0x0f, 4); // RX done, CRC, and timeout interrupts
    chip.write(0x40031000, 1, 4);
    const std::uint64_t rx_start = chip.core().cycles_to_ns(chip.core().cycles()) + 1000;
    const std::uint8_t rx_frame[] = {0x01, 0x00, 0xa5};
    const std::uint64_t rx_id = air.transmit("coordinator", {}, mcuemu::RadioProtocol::Zigbee802154,
                                              11, rx_start,
                                              std::vector<std::uint8_t>(rx_frame, rx_frame + sizeof rx_frame));
    const auto rx_it = std::find_if(air.frames().begin(), air.frames().end(),
                                    [rx_id](const auto &frame) { return frame.id == rx_id; });
    CHECK(rx_it != air.frames().end());
    if (rx_it != air.frames().end()) {
        const std::uint64_t rx_done = chip.core().ns_to_cycles(rx_it->end_ns + 20000);
        CHECK(chip.run(rx_done + 100).reason == cortexm::StopReason::Deadline);
    }
    CHECK((chip.read(0x40031010, 4) & ((1u << 0) | (1u << 10))) ==
          ((1u << 0) | (1u << 10)));
    CHECK(chip.read(0x40031c00, 1) == sizeof(rx_frame) + 2);
    for (std::size_t i = 0; i < sizeof rx_frame; ++i)
        CHECK(chip.read(0x40031c01 + static_cast<std::uint32_t>(i), 1) == rx_frame[i]);
    CHECK(chip.read(0x40031c01 + sizeof rx_frame, 1) == 0x7b);
    CHECK(chip.read(0x40031c02 + sizeof rx_frame, 1) == 0xa8);
    CHECK(static_cast<std::int8_t>(chip.read(0x400300e4, 4) >> 24) == -50);

    // PHY+ selects TRX when transmitting a packet that expects a response.
    // The controller completes only after the RX window receives a frame or
    // reaches LL+0x28's timeout.
    chip.write(0x40031014, 0x3fffu, 4);
    chip.write(0x40031028, 1500, 4);
    chip.write(0x40031004, 0x010102, 4); // TRX: transmit, then receive
    chip.write(0x4003100c, 0x0f, 4);
    chip.write(0x40031400, sizeof(tx_frame) + 2, 1);
    for (std::size_t i = 0; i < sizeof tx_frame; ++i)
        chip.write(0x40031401 + static_cast<std::uint32_t>(i), tx_frame[i], 1);
    const std::size_t frames_before_trx = air.frames().size();
    chip.write(0x40031000, 1, 4);
    CHECK(air.frames().size() == frames_before_trx + 1);
    const auto trx_tx = air.frames().back();
    CHECK(trx_tx.source == "phy6252");
    const std::uint64_t ack_start_ns = trx_tx.end_ns + 192000;
    const std::uint8_t ack_frame[] = {0x02, 0x00, 0xa6};
    const std::uint64_t ack_id = air.transmit(
        "coordinator", "phy6252", mcuemu::RadioProtocol::Zigbee802154,
        trx_tx.channel, ack_start_ns,
        std::vector<std::uint8_t>(ack_frame, ack_frame + sizeof ack_frame));
    const auto ack_it = std::find_if(air.frames().begin(), air.frames().end(),
        [ack_id](const auto &frame) { return frame.id == ack_id; });
    CHECK(ack_it != air.frames().end());
    CHECK(chip.run(chip.core().ns_to_cycles(trx_tx.end_ns + 100000)).reason ==
          cortexm::StopReason::Deadline);
    CHECK((chip.read(0x40031010, 4) & 1u) == 0); // TX alone does not finish TRX.
    if (ack_it != air.frames().end()) {
        CHECK(chip.run(chip.core().ns_to_cycles(ack_it->end_ns + 20000)).reason ==
              cortexm::StopReason::Deadline);
    }
    CHECK((chip.read(0x40031010, 4) & ((1u << 0) | (1u << 10))) ==
          ((1u << 0) | (1u << 10)));
    CHECK(chip.read(0x40031c00, 1) == sizeof(ack_frame) + 2);
    for (std::size_t i = 0; i < sizeof ack_frame; ++i)
        CHECK(chip.read(0x40031c01 + static_cast<std::uint32_t>(i), 1) == ack_frame[i]);

    chip.write(0x40031014, 0x3fffu, 4);
    chip.write(0x40031028, 500, 4); // No response: complete with MD and RTO.
    chip.write(0x40031004, 0x010102, 4);
    chip.write(0x40031400, sizeof(tx_frame) + 2, 1);
    for (std::size_t i = 0; i < sizeof tx_frame; ++i)
        chip.write(0x40031401 + static_cast<std::uint32_t>(i), tx_frame[i], 1);
    chip.write(0x40031000, 1, 4);
    const auto timeout_tx = air.frames().back();
    CHECK(chip.run(chip.core().ns_to_cycles(timeout_tx.end_ns + 550000)).reason ==
          cortexm::StopReason::Deadline);
    CHECK((chip.read(0x40031010, 4) & ((1u << 0) | (1u << 2))) ==
          ((1u << 0) | (1u << 2)));
    CHECK((chip.read(0x40031010, 4) & (1u << 10)) == 0);

    chip.write(0x40031f00, 0x1fff0a4a, 4);
    CHECK(chip.read(0x40031f00, 4) == 0x1fff0a4a);
    chip.write(0x40031fa0, 0x1fff0a4a, 4);
    CHECK(chip.read(0x40031fa0, 4) == 0x1fff0a4a);
    chip.write(0x40031fc0, 0x1fff0a4a, 4);
    CHECK(chip.read(0x40031fc0, 4) == 0x1fff0a4a);
    chip.write(0x40031fe0, 0x1fff0a4a, 4);
    CHECK(chip.read(0x40031fe0, 4) == 0x1fff0a4a);
    chip.write(0x40031fe4, 0x1fff4345, 4);
    CHECK(chip.read(0x40031fe4, 4) == 0x1fff4345);
    chip.write(0x40031ee8, 0x2580, 4);
    CHECK(chip.read(0x40031ee8, 4) == 0x2580);

    chip.reset();
    chip.set_gpio_input(2, true);
    CHECK((chip.read(0x40008050, 4) & (1u << 2)) != 0);
    chip.write(0x40008038, 1u << 2, 4); // edge-triggered
    chip.write(0x4000803c, 1u << 2, 4); // rising edge
    chip.write(0x40008034, 0, 4); // unmask
    chip.write(0x40008030, 1u << 2, 4); // enable
    chip.set_gpio_input(2, false);
    CHECK((chip.read(0x40008040, 4) & (1u << 2)) == 0);
    chip.set_gpio_input(2, true);
    CHECK((chip.read(0x40008040, 4) & (1u << 2)) != 0);
    chip.write(0x4000804c, 1u << 2, 4);
    CHECK((chip.read(0x40008040, 4) & (1u << 2)) == 0);
    chip.write(0x40008004, 1u << 3, 4);
    chip.write(0x40008000, 1u << 3, 4);
    CHECK(chip.gpio_drive(3) == GpioDrive::High);
    chip.write(0x4000380c, 1u << 3, 4); // select alternate pad function
    CHECK(chip.gpio_drive(3) == GpioDrive::HiZ);
    chip.write(0x4000380c, 0, 4);
    chip.write(0x4000f008, 2u << 1, 4); // GPIO0 pull-up
    CHECK(chip.gpio_drive(0) == GpioDrive::PullUp);

    chip.reset();
    const std::uint64_t timer_start = chip.core().cycles();
    chip.write(0x40001050, 5, 4); // Timer5 load count
    chip.write(0x40001058, 3, 4); // enable, user-defined one-shot, IRQ unmasked
    CHECK(chip.run(timer_start + 8).reason == cortexm::StopReason::Deadline);
    CHECK(chip.read(0x40001054, 4) == 3); // two ticks at 4 MHz
    CHECK(chip.run(timer_start + 20).reason == cortexm::StopReason::Deadline);
    CHECK(chip.read(0x40001054, 4) == 0);
    CHECK(chip.read(0x40001060, 4) == 1);
    CHECK(chip.read(0x4000105c, 4) == 1); // EOI returns and clears pending
    CHECK(chip.read(0x40001060, 4) == 0);
    chip.reset();
    const std::uint64_t timer6_start = chip.core().cycles();
    chip.write(0x40001064, 3, 4);
    chip.write(0x4000106c, 3, 4);
    CHECK(chip.run(timer6_start + 12).reason == cortexm::StopReason::Deadline);
    CHECK(chip.read(0x40001074, 4) == 1);
    CHECK(chip.read(0x40001070, 4) == 1);
    chip.write(0x40001068, 0, 4); // SDK hal_timer_stop_counter clears CurrentCount
    CHECK(chip.read(0x40001068, 4) == 0);

    constexpr char crc_vector[] = "123456789";
    const std::uint32_t crc_source = BootImage::RamBase + 0x4f00;
    const std::uint32_t crc_mic = BootImage::RamBase + 0x4f20;
    const std::uint32_t crc_plaintext = BootImage::RamBase + 0x4f40;
    const std::uint32_t crc_stack = BootImage::RamBase + 0x4f80;
    CHECK(chip.core().write_memory(crc_source, crc_vector, sizeof crc_vector - 1));
    const std::uint32_t crc_encrypt_args[] = {0, crc_mic};
    CHECK(chip.core().write_memory(crc_stack, crc_encrypt_args, sizeof crc_encrypt_args));
    chip.core().set_reg(0, 0);
    chip.core().set_reg(1, 0);
    chip.core().set_reg(2, crc_source);
    chip.core().set_reg(3, sizeof crc_vector - 1);
    chip.core().set_reg(13, crc_stack);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x15338);
    CHECK(chip.run(chip.core().cycles() + 10).reason == cortexm::StopReason::Deadline);
    std::uint32_t crc_value = 0;
    CHECK(chip.core().read_memory(crc_mic, &crc_value, sizeof crc_value));
    CHECK(crc_value == 0xbb3d); // CRC-16/ARC fallback for an erased secure-MIC area.
    const std::uint32_t crc_decrypt_args[] = {crc_mic, crc_plaintext};
    CHECK(chip.core().write_memory(crc_stack, crc_decrypt_args, sizeof crc_decrypt_args));
    chip.core().set_reg(0, 0);
    chip.core().set_reg(1, 0);
    chip.core().set_reg(2, crc_source);
    chip.core().set_reg(3, sizeof crc_vector - 1);
    chip.core().set_reg(13, crc_stack);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x1531c);
    CHECK(chip.run(chip.core().cycles() + 10).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().reg(0) == 0);
    std::array<char, sizeof crc_vector - 1> crc_round_trip{};
    CHECK(chip.core().read_memory(crc_plaintext, crc_round_trip.data(), crc_round_trip.size()));
    CHECK(std::equal(crc_round_trip.begin(), crc_round_trip.end(), crc_vector));

    auto *secure_area_mic = chip.core().host_pointer(BootImage::FlashBase + 0x2920, 16);
    CHECK(secure_area_mic != nullptr);
    if (secure_area_mic)
        std::memset(secure_area_mic, 0, 16); // Provisioned metadata selects the AES-CCM path.
    const crypto::Block sec_key = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                   0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
    const std::array<std::uint8_t, 13> sec_iv = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                                                 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c};
    std::array<std::uint8_t, 37> sec_plain{};
    for (std::size_t i = 0; i < sec_plain.size(); ++i)
        sec_plain[i] = static_cast<std::uint8_t>(i * 7 + 3);
    const std::uint32_t sec_key_address = BootImage::RamBase + 0x5000;
    const std::uint32_t sec_iv_address = BootImage::RamBase + 0x5020;
    const std::uint32_t sec_source = BootImage::RamBase + 0x5040;
    const std::uint32_t sec_ciphertext = BootImage::RamBase + 0x5080;
    const std::uint32_t sec_mic = BootImage::RamBase + 0x50c0;
    const std::uint32_t sec_plaintext = BootImage::RamBase + 0x5100;
    const std::uint32_t sec_stack = BootImage::RamBase + 0x5200;
    CHECK(chip.core().write_memory(sec_key_address, sec_key.data(), sec_key.size()));
    CHECK(chip.core().write_memory(sec_iv_address, sec_iv.data(), sec_iv.size()));
    CHECK(chip.core().write_memory(sec_source, sec_plain.data(), sec_plain.size()));
    const std::uint32_t encrypt_args[] = {sec_ciphertext, sec_mic};
    CHECK(chip.core().write_memory(sec_stack, encrypt_args, sizeof encrypt_args));
    chip.core().set_reg(0, sec_key_address);
    chip.core().set_reg(1, sec_iv_address);
    chip.core().set_reg(2, sec_source);
    chip.core().set_reg(3, sec_plain.size());
    chip.core().set_reg(13, sec_stack);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x15338);
    CHECK(chip.run(chip.core().cycles() + 10).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().reg(0) == 0);
    std::array<std::uint8_t, 4> sec_mic_bytes{};
    CHECK(chip.core().read_memory(sec_mic, sec_mic_bytes.data(), sec_mic_bytes.size()));
    CHECK((sec_mic_bytes == std::array<std::uint8_t, 4>{0xdb, 0x1f, 0x56, 0x78}));
    const std::uint32_t decrypt_args[] = {sec_mic, sec_plaintext};
    CHECK(chip.core().write_memory(sec_stack, decrypt_args, sizeof decrypt_args));
    chip.core().set_reg(0, sec_key_address);
    chip.core().set_reg(1, sec_iv_address);
    chip.core().set_reg(2, sec_ciphertext);
    chip.core().set_reg(3, sec_plain.size());
    chip.core().set_reg(13, sec_stack);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x1531c);
    CHECK(chip.run(chip.core().cycles() + 10).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().reg(0) == 0);
    std::array<std::uint8_t, sec_plain.size()> sec_round_trip{};
    CHECK(chip.core().read_memory(sec_plaintext, sec_round_trip.data(), sec_round_trip.size()));
    CHECK(sec_round_trip == sec_plain);
    sec_mic_bytes[0] ^= 1;
    CHECK(chip.core().write_memory(sec_mic, sec_mic_bytes.data(), sec_mic_bytes.size()));
    std::array<std::uint8_t, sec_plain.size()> untouched{};
    untouched.fill(0xa5);
    CHECK(chip.core().write_memory(sec_plaintext, untouched.data(), untouched.size()));
    CHECK(chip.core().write_memory(sec_stack, decrypt_args, sizeof decrypt_args));
    chip.core().set_reg(0, sec_key_address);
    chip.core().set_reg(1, sec_iv_address);
    chip.core().set_reg(2, sec_ciphertext);
    chip.core().set_reg(3, sec_plain.size());
    chip.core().set_reg(13, sec_stack);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x1531c);
    CHECK(chip.run(chip.core().cycles() + 10).reason == cortexm::StopReason::Deadline);
    CHECK(chip.core().reg(0) != 0);
    CHECK(chip.core().read_memory(sec_plaintext, sec_round_trip.data(), sec_round_trip.size()));
    CHECK(sec_round_trip == untouched);
    if (secure_area_mic)
        std::memset(secure_area_mic, 0xff, 16);

    put(image, 0x100, 0x40000);
    bool rejected = false;
    try { (void)load_boot_image(image); } catch (const std::runtime_error &) { rejected = true; }
    CHECK(rejected);
    invoke_rom(0x11278, 0x4e20, 0, 0, chip.core().cycles() + 8);
    invoke_rom(0x11284, 0x4e20, 0, 0, chip.core().cycles() + 8);
    invoke_rom(0x112a4, 0, 0, 0, chip.core().cycles() + 8);
    invoke_rom(0x113dc, 0x58, 8, 0x28, chip.core().cycles() + 8);
    invoke_rom(0x110b0, 0, 0, 0, chip.core().cycles() + 8);
    invoke_rom(0x110e8, 0, 0, 0, chip.core().cycles() + 8);
    invoke_rom(0x10cdc, 0, 0, 0, chip.core().cycles() + 8);
    CHECK(chip.core().reg(0) == 1); // ll_hw_get_tr_mode returns SRX selected above

    const std::uint32_t dispatch_slot = 0x1fff0118;
    const std::uint32_t callback_address = BootImage::RamBase + 0x5300;
    const std::uint32_t aes_dispatch_input = BootImage::RamBase + 0x3040;
    const std::uint32_t aes_dispatch_key = BootImage::RamBase + 0x3060;
    const std::uint32_t aes_dispatch_output = BootImage::RamBase + 0x3080;
    const std::uint32_t no_dispatch = 0;
    CHECK(chip.core().write_memory(aes_dispatch_input, aes_input.data(), aes_input.size()));
    CHECK(chip.core().write_memory(aes_dispatch_key, aes_key.data(), aes_key.size()));
    CHECK(chip.core().write_memory(dispatch_slot, &no_dispatch, sizeof no_dispatch));
    const std::uint32_t wrapper_sp = chip.core().reg(13);
    chip.core().set_reg(4, 0x76543210);
    invoke_rom(0x3fc4, aes_dispatch_input, aes_dispatch_key, aes_dispatch_output,
               chip.core().cycles() + 8);
    CHECK(chip.core().reg(0) == 0);
    CHECK(chip.core().reg(4) == 0x76543210);
    CHECK(chip.core().reg(13) == wrapper_sp);
    CHECK(chip.core().read_memory(aes_dispatch_output, aes_output.data(), aes_output.size()));
    CHECK(aes_output == aes_expected);

    const std::array<std::uint8_t, 4> callback_code = {0x42, 0x20, 0x70, 0x47};
    const std::uint32_t callback = callback_address | 1u;
    CHECK(chip.core().write_memory(callback_address, callback_code.data(), callback_code.size()));
    CHECK(chip.core().write_memory(dispatch_slot, &callback, sizeof callback));
    chip.core().set_reg(4, 0x89abcdef);
    invoke_rom(0x3fc4, 1, 2, 3, chip.core().cycles() + 8);
    CHECK(chip.core().reg(0) == 0x42);
    CHECK(chip.core().reg(4) == 0x89abcdef);
    CHECK(chip.core().reg(13) == wrapper_sp);
    const std::uint32_t no_callback = 0;
    CHECK(chip.core().write_memory(dispatch_slot, &no_callback, sizeof no_callback));

    chip.reset();
    CHECK((chip.read(0x4000f024, 4) & 1u) != 0);
    chip.core().advance_to(chip.core().cycles() + 16000);
    CHECK(chip.read(0x4000f028, 4) >= 31);
    CHECK(chip.read(0x4000f028, 4) <= 33);
    return mcuemu_tests::failures ? 1 : 0;
}
