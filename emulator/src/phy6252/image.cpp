#include "phy6252emu/image.hpp"

#include <algorithm>
#include <stdexcept>

namespace phy6252emu {
namespace {
constexpr std::uint32_t ImageOffset = 0x2000, RecordsOffset = 0x100;
std::uint32_t word(const std::vector<std::uint8_t> &data, std::size_t offset) {
    if (offset > data.size() || data.size() - offset < 4)
        throw std::runtime_error("PHY6252 image has a truncated word");
    return static_cast<std::uint32_t>(data[offset]) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(data[offset + 3]) << 24);
}
} // namespace

BootImage load_boot_image(const std::vector<std::uint8_t> &image) {
    constexpr std::uint32_t MaxRecords = (0x10000 - ImageOffset - RecordsOffset) / 16;
    auto has_table = [&](std::size_t offset) {
        if (offset > image.size() || image.size() - offset < 12)
            return false;
        const std::uint32_t count = word(image, offset);
        return count != 0 && count <= MaxRecords &&
               RecordsOffset + static_cast<std::uint64_t>(count) * 16 <= image.size() - offset;
    };

    const bool package = has_table(0);
    const bool full_flash = !package && has_table(ImageOffset);
    if (!package && !full_flash)
        throw std::runtime_error("PHY6252 image has an invalid segment table");
    if (image.size() > (full_flash ? BootImage::FlashSize : BootImage::FlashSize - ImageOffset))
        throw std::runtime_error("PHY6252 image exceeds flash");

    const std::size_t table_offset = full_flash ? ImageOffset : 0;
    const std::uint32_t count = word(image, table_offset);
    const std::uint32_t run = word(image, table_offset + 8);
    if (run < BootImage::RamBase || static_cast<std::uint64_t>(run) + 8 > BootImage::RamBase + BootImage::RamSize)
        throw std::runtime_error("PHY6252 run descriptor is outside SRAM");
    BootImage result{std::vector<std::uint8_t>(BootImage::FlashSize, 0xff),
                     std::vector<std::uint8_t>(BootImage::RamSize, 0), run};
    const std::size_t image_flash_offset = full_flash ? 0 : ImageOffset;
    std::copy(image.begin(), image.end(), result.flash.begin() + image_flash_offset);
    bool descriptor = false;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> executable;
    for (std::uint32_t i = 0; i < count; ++i) {
        const std::size_t offset = table_offset + RecordsOffset + i * 16;
        const std::uint32_t src = word(image, offset);
        const std::uint32_t size = word(image, offset + 4);
        const std::uint32_t dst = word(image, offset + 8);
        const std::uint64_t source_offset64 = src >= BootImage::FlashBase ?
            static_cast<std::uint64_t>(src) - BootImage::FlashBase : src;
        if (source_offset64 + size > BootImage::FlashSize)
            throw std::runtime_error("PHY6252 segment source is outside the image");
        const std::uint32_t source_offset = static_cast<std::uint32_t>(source_offset64);
        if (full_flash && source_offset64 + size > image.size())
            throw std::runtime_error("PHY6252 segment source is outside the image");
        if (package && (source_offset < ImageOffset ||
                        static_cast<std::uint64_t>(source_offset) + size > ImageOffset + image.size()))
            throw std::runtime_error("PHY6252 segment source is outside the image");
        const std::size_t file_offset = full_flash ? source_offset : source_offset - ImageOffset;
        const auto begin = image.begin() + file_offset;
        if (dst >= BootImage::RamBase && static_cast<std::uint64_t>(dst) + size <= BootImage::RamBase + BootImage::RamSize) {
            std::copy_n(begin, size, result.ram.begin() + (dst - BootImage::RamBase));
            descriptor |= dst <= run && static_cast<std::uint64_t>(dst) + size >= static_cast<std::uint64_t>(run) + 8;
            executable.emplace_back(dst, size);
        } else if (dst >= BootImage::FlashBase && static_cast<std::uint64_t>(dst) + size <= BootImage::FlashBase + BootImage::FlashSize && dst - BootImage::FlashBase == source_offset) {
            std::copy_n(begin, size, result.flash.begin() + source_offset);
            executable.emplace_back(dst, size);
        } else {
            throw std::runtime_error("PHY6252 segment destination is invalid");
        }
    }
    if (!descriptor)
        throw std::runtime_error("PHY6252 run descriptor was not loaded");
    const std::uint32_t sp = word(result.ram, run - BootImage::RamBase);
    const std::uint32_t pc = word(result.ram, run - BootImage::RamBase + 4);
    const bool code = std::any_of(executable.begin(), executable.end(), [pc](const auto &segment) {
        return (pc & ~1u) >= segment.first && static_cast<std::uint64_t>(pc & ~1u) < static_cast<std::uint64_t>(segment.first) + segment.second;
    });
    if (sp < BootImage::RamBase || sp > BootImage::RamBase + BootImage::RamSize || (sp & 7u) || !(pc & 1u) || !code)
        throw std::runtime_error("PHY6252 run descriptor has an invalid stack or reset vector");
    return result;
}
} // namespace phy6252emu
