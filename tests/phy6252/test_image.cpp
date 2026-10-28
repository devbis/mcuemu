#include "../test_support.hpp"
#include "cortexm/core.hpp"
#include "crypto/aes128.hpp"
#include "phy6252emu/image.hpp"
#include "phy6252emu/phy6252.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace {
void put(std::vector<std::uint8_t> &data, std::size_t offset, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i)
        data[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
}
}

int main() {
    using namespace phy6252emu;
    std::vector<std::uint8_t> image(0xe120, 0xff);
    put(image, 0, 2); put(image, 8, 0x1fff1838);
    put(image, 0x100, 0x10100); put(image, 0x104, 12); put(image, 0x108, 0x11010100);
    put(image, 0x110, 0x10110); put(image, 0x114, 8); put(image, 0x118, 0x1fff1838);
    put(image, 0xe100, 0x1fff2000); put(image, 0xe104, 0x11010109);
    image[0xe108] = 0xfe; image[0xe109] = 0xe7; // b .
    put(image, 0xe110, 0x1fff2000); put(image, 0xe114, 0x11010109);
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
    cortexm::Core core(config);
    auto *flash = core.map_memory(BootImage::FlashBase, BootImage::FlashSize, false);
    auto *ram = core.map_memory(BootImage::RamBase, BootImage::RamSize, true);
    std::memcpy(flash, boot.flash.data(), boot.flash.size());
    std::memcpy(ram, boot.ram.data(), boot.ram.size());
    core.reset(boot.run_address);
    CHECK(core.run(100).reason == cortexm::StopReason::Deadline);
    CHECK(core.pc() == 0x11010108);
    CHECK(core.reg(13) == 0x1fff2000);
    Phy6252 chip;
    chip.load_firmware(image);
    chip.reset();
    const std::uint32_t vector_word = 0x11010109;
    std::uint32_t vector_readback = 0;
    CHECK(chip.core().write_memory(0, &vector_word, sizeof vector_word));
    CHECK(chip.core().read_memory(0, &vector_readback, sizeof vector_readback));
    CHECK(vector_readback == vector_word);
    chip.reset();
    CHECK(chip.core().read_memory(0, &vector_readback, sizeof vector_readback));
    CHECK(vector_readback == 0);
    CHECK(chip.run(100).reason == cortexm::StopReason::Deadline);
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
    const auto crypto_unknown = chip.run(504);
    CHECK(crypto_unknown.reason == cortexm::StopReason::Fault);
    CHECK(crypto_unknown.message.find("phy_sec_encrypt") != std::string::npos);
    CHECK(crypto_unknown.message.find("lr=") != std::string::npos);

    CHECK(chip.read(0x4000c800, 4) == 0x80000000u);
    chip.write(0x4000c890, 0x05900001, 4);
    CHECK((chip.read(0x4000c890, 4) & 2) == 0);
    CHECK(chip.read(0x4000c8a0, 4) == 0);
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
    chip.write(0x4000f0a0, 0x12345678, 4); // GPIO0..31 wakeup mask
    chip.write(0x4000f0a4, 7, 4); // GPIO32..34 wakeup mask
    CHECK(chip.read(0x4000f0a0, 4) == 0x12345678);
    CHECK(chip.read(0x4000f0a4, 4) == 7);
    CHECK(chip.read(0x4000f0b0, 4) == 0); // no modeled GPIO wake event
    CHECK(chip.read(0x4000f0b4, 4) == 0);
    chip.write(0x4000f140, 0x34, 4); // eFuse programming control, not OTP contents
    chip.write(0x4000f144, 0x56, 4);
    CHECK(chip.read(0x4000f140, 4) == 0x34);
    CHECK(chip.read(0x4000f144, 4) == 0x56);

    auto invoke_rom = [&chip](std::uint32_t pc, std::uint32_t r0,
                              std::uint32_t r1, std::uint32_t r2,
                              std::uint64_t deadline) {
        chip.core().set_reg(0, r0);
        chip.core().set_reg(1, r1);
        chip.core().set_reg(2, r2);
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
    invoke_rom(0x5044, BootImage::RamBase + 0x4000,
               BootImage::RamBase + 0x4200, 1, 528);
    CHECK(chip.core().reg(0) == 0);
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
    invoke_rom(0xace0, 1, efuse_buffer, 0, 540);
    CHECK(chip.core().reg(0) == 0);
    std::uint8_t efuse_data[8]{};
    CHECK(chip.core().read_memory(efuse_buffer, efuse_data, sizeof efuse_data));
    CHECK(efuse_data[0] == 0x2b && efuse_data[1] == 0x23);
    CHECK(std::all_of(efuse_data + 2, efuse_data + sizeof efuse_data,
                      [](std::uint8_t value) { return value == 0; }));
    invoke_rom(0xace0, 2, efuse_buffer, 0, 545);
    CHECK(chip.core().reg(0) == static_cast<std::uint32_t>(-22));
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
    CHECK(std::equal(std::begin(efuse_data), std::end(efuse_data), std::begin(copied_data)));
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
    CHECK((chip.read(0x40031010, 4) & 3) == 3);
    CHECK(chip.read(0x40031c00, 1) == sizeof(rx_frame) + 2);
    for (std::size_t i = 0; i < sizeof rx_frame; ++i)
        CHECK(chip.read(0x40031c01 + static_cast<std::uint32_t>(i), 1) == rx_frame[i]);
    CHECK(static_cast<std::int8_t>(chip.read(0x400300e4, 4) >> 24) == -50);
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

    constexpr char crc_vector[] = "123456789";
    const std::uint32_t crc_source = BootImage::FlashBase + 0x20000;
    auto *crc_bytes = chip.core().host_pointer(crc_source, sizeof crc_vector - 1);
    CHECK(crc_bytes != nullptr);
    std::memcpy(crc_bytes, crc_vector, sizeof crc_vector - 1);
    const std::uint32_t crc_stack = BootImage::RamBase + 0x5000;
    const std::uint32_t crc_result = BootImage::RamBase + 0x5100;
    const std::uint32_t crc_stack_args[] = {0, crc_result};
    CHECK(chip.core().write_memory(crc_stack, crc_stack_args, sizeof crc_stack_args));
    chip.core().set_reg(0, 0x1fff9440);
    chip.core().set_reg(1, 0x11002830);
    chip.core().set_reg(2, crc_source);
    chip.core().set_reg(3, sizeof crc_vector - 1);
    chip.core().set_reg(13, crc_stack);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x15338);
    CHECK(chip.run(chip.core().cycles() + 10).reason == cortexm::StopReason::Deadline);
    std::uint32_t crc_value = 0;
    CHECK(chip.core().read_memory(crc_result, &crc_value, sizeof crc_value));
    CHECK(crc_value == 0xbb3d); // CRC-16/ARC for "123456789".

    const std::uint32_t clear_table_source = 0x11011000;
    auto *clear_table = chip.core().host_pointer(clear_table_source, 0x800);
    CHECK(clear_table != nullptr);
    std::memset(clear_table, 0, 0x800);
    for (const auto &[offset, value] : std::array<std::pair<std::uint32_t, std::uint32_t>, 5>{{
             {0x190, 0x1fff4519}, {0x194, 0x1fff4601}, {0x198, 0x1fff5ef9},
             {0x370, 0x1fff190d}, {0x3ac, 0x1fff2415},
         }})
        CHECK(chip.core().write_memory(clear_table_source + offset, &value, sizeof value));
    const std::uint32_t clear_table_record = 0x1ffff6d0;
    const std::uint16_t clear_table_mic = 0x0094; // CRC-16/ARC of this sparse vector table.
    CHECK(chip.core().write_memory(clear_table_record, &clear_table_mic, sizeof clear_table_mic));
    const std::uint32_t clear_table_stack = BootImage::RamBase + 0xf000;
    const std::uint32_t clear_table_args[] = {clear_table_record, BootImage::RamBase};
    CHECK(chip.core().write_memory(clear_table_stack, clear_table_args, sizeof clear_table_args));
    chip.core().set_reg(0, 0x1fff9440);
    chip.core().set_reg(1, 0x11002830);
    chip.core().set_reg(2, clear_table_source);
    chip.core().set_reg(3, 0x800);
    chip.core().set_reg(13, clear_table_stack);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x1531c);
    CHECK(chip.run(chip.core().cycles() + 10).reason == cortexm::StopReason::Deadline);
    std::uint32_t copied_vector = 0;
    CHECK(chip.core().read_memory(BootImage::RamBase + 0x194, &copied_vector, sizeof copied_vector));
    CHECK(copied_vector == 0x1fff4601);

    const std::uint32_t clear_code_source = 0x11011808;
    auto *clear_code = chip.core().host_pointer(clear_code_source, 0x4000);
    CHECK(clear_code != nullptr);
    std::memset(clear_code, 0, 0x4000);
    constexpr std::array<std::uint8_t, 8> thumb_trampoline = {
        0x04, 0x48, 0x80, 0x47, 0x04, 0x48, 0x00, 0x47};
    std::memcpy(clear_code + 0x17c, thumb_trampoline.data(), thumb_trampoline.size());
    const std::uint16_t clear_code_mic = 0xa2f9; // CRC-16/ARC of the sparse block.
    CHECK(chip.core().write_memory(clear_table_record, &clear_code_mic, sizeof clear_code_mic));
    const std::uint32_t clear_code_args[] = {clear_table_record, BootImage::RamBase + 0x1838};
    CHECK(chip.core().write_memory(clear_table_stack, clear_code_args, sizeof clear_code_args));
    chip.core().set_reg(0, 0x1fff9440);
    chip.core().set_reg(1, 0x11002830);
    chip.core().set_reg(2, clear_code_source);
    chip.core().set_reg(3, 0x4000);
    chip.core().set_reg(13, clear_table_stack);
    chip.core().set_reg(14, 0x11010109);
    chip.core().set_reg(15, 0x1531c);
    CHECK(chip.run(chip.core().cycles() + 10).reason == cortexm::StopReason::Deadline);
    std::uint8_t copied_trampoline[thumb_trampoline.size()]{};
    CHECK(chip.core().read_memory(BootImage::RamBase + 0x1838 + 0x17c,
                                  copied_trampoline, sizeof copied_trampoline));
    CHECK(std::equal(thumb_trampoline.begin(), thumb_trampoline.end(), copied_trampoline));

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
    CHECK(chip.core().reg(0) == 0);
    chip.reset();
    CHECK((chip.read(0x4000f024, 4) & 1u) != 0);
    chip.core().advance_to(chip.core().cycles() + 16000);
    CHECK(chip.read(0x4000f028, 4) >= 31);
    CHECK(chip.read(0x4000f028, 4) <= 33);
    return mcuemu_tests::failures ? 1 : 0;
}
