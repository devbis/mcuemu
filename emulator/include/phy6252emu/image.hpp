#pragma once

#include <cstdint>
#include <vector>

namespace phy6252emu {

struct BootImage {
    static constexpr std::uint32_t FlashSize = 0x80000;
    static constexpr std::uint32_t RamSize = 0x10000;
    static constexpr std::uint32_t RamBase = 0x1fff0000;
    static constexpr std::uint32_t FlashBase = 0x11000000;

    std::vector<std::uint8_t> flash;
    std::vector<std::uint8_t> ram;
    std::uint32_t run_address;
};

// Parse the experimental Zephyr PB-03F package, whose first byte is at
// flash offset 0x2000. Throws on an invalid table or run descriptor.
BootImage load_boot_image(const std::vector<std::uint8_t> &image);

} // namespace phy6252emu
