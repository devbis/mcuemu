#include "rom.hpp"

#include "bl702emu/bl702.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

namespace bl702emu {

namespace {
constexpr std::uint32_t Ebreak = 0x00100073;
constexpr std::uint32_t SectorSize = 4096;

// The indices of the SDK's ROM API table (the HAL's flash.rs and sf_ctrl calls).
constexpr unsigned XipErase = 78, XipWrite = 79, XipRead = 80, L1cCacheFlush = 121;
constexpr unsigned SfCtrlAesEnable = 144, SfCtrlAesDisable = 145, SfCtrlIsAesEnable = 146, LoadFlashConfig = 157;

// The flash configuration in the boot header: 84 bytes after the 'FCFG' magic (offset 8), so at 12.
constexpr std::uint32_t FlashConfigOffset = 12, FlashConfigLength = 84;
} // namespace

Rom::Rom(Bl702 &chip) : chip_(chip) {
    std::uint8_t *rom = chip_.core().host_pointer(Bl702::RomBase, Bl702::RomSize);
    for (std::uint32_t offset = 0; offset < Bl702::RomSize; offset += 4)
        std::memcpy(rom + offset, &Ebreak, 4);
    for (unsigned i = 0; i < Entries; ++i) {
        const std::uint32_t stub = StubBase + 4 * i;
        std::memcpy(rom + (stub - Bl702::RomBase), &Ebreak, 4);
        std::memcpy(rom + (Bl702::RomApiTable - Bl702::RomBase) + 4 * i, &stub, 4);
    }

    // Flash routines (the HAL runs them from RAM, through the table): the 84-byte flash configuration
    // is the boot header's, and the flash is an array the host writes. A write only clears bits, as NOR
    // flash does; an erase sets a whole 4 KiB sector to 0xff.
    add(LoadFlashConfig, [this](const std::array<std::uint32_t, 6> &args) {
        const std::uint8_t *header = chip_.flash() + FlashConfigOffset;
        return chip_.core().write_memory(args[1], header, FlashConfigLength) ? 0u : 1u;
    });
    add(XipErase, [this](const std::array<std::uint32_t, 6> &args) {
        // The end address is inclusive.
        const std::int64_t from = flash_offset(args[2], 1);
        const std::int64_t to = flash_offset(args[3], 1);
        if (from < 0 || to < from || to >= chip_.flash_size())
            return 1u;
        for (std::int64_t sector = from & ~static_cast<std::int64_t>(SectorSize - 1); sector <= to;
             sector += SectorSize) {
            const std::uint32_t count = static_cast<std::uint32_t>(
                std::min<std::int64_t>(SectorSize, chip_.flash_size() - sector));
            std::memset(chip_.flash() + sector, 0xff, count);
        }
        chip_.flash_modified();
        return 0u;
    });
    add(XipWrite, [this](const std::array<std::uint32_t, 6> &args) {
        const std::uint32_t length = args[4];
        const std::int64_t offset = flash_offset(args[2], length);
        if (offset < 0)
            return 1u;
        std::vector<std::uint8_t> data(length);
        if (length && !chip_.core().read_memory(args[3], data.data(), length))
            return 1u;
        for (std::uint32_t i = 0; i < length; ++i)
            chip_.flash()[offset + i] &= data[i];
        chip_.flash_modified();
        return 0u;
    });
    add(XipRead, [this](const std::array<std::uint32_t, 6> &args) {
        const std::uint32_t length = args[4];
        const std::int64_t offset = flash_offset(args[2], length);
        if (offset < 0)
            return 1u;
        return chip_.core().write_memory(args[3], chip_.flash() + offset, length) ? 0u : 1u;
    });
    // The cache is not modelled, and neither is the AES option of the flash controller: reads and writes
    // go to the array as they are.
    add(L1cCacheFlush, [](const std::array<std::uint32_t, 6> &) { return 0u; });
    add(SfCtrlAesEnable, [](const std::array<std::uint32_t, 6> &) { return 0u; });
    add(SfCtrlAesDisable, [](const std::array<std::uint32_t, 6> &) { return 0u; });
    add(SfCtrlIsAesEnable, [](const std::array<std::uint32_t, 6> &) { return 0u; });
}

bool Rom::call(std::uint32_t pc) {
    if (pc < StubBase || pc >= StubBase + 4 * Entries || (pc - StubBase) % 4 != 0)
        return false;
    const unsigned index = (pc - StubBase) / 4;
    std::array<std::uint32_t, 6> args{};
    for (unsigned i = 0; i < args.size(); ++i)
        args[i] = chip_.core().reg(10 + i);
    const auto handler = handlers_.find(index);
    const std::uint32_t result = handler == handlers_.end() ? 0 : handler->second(args);
    chip_.core().set_reg(10, result);
    chip_.core().set_reg(11, 0);
    chip_.core().set_reg(32, chip_.core().reg(1)); // pc = ra
    return true;
}

std::int64_t Rom::flash_offset(std::uint32_t address, std::uint32_t length) const {
    // The HAL passes physical flash offsets (the same ones its flash driver takes), not XIP addresses.
    if (static_cast<std::uint64_t>(address) + length > chip_.flash_size())
        return -1;
    return address;
}

} // namespace bl702emu
