#include "spi_mem.hpp"

#include <algorithm>
#include <cstring>
#include <cstdio>

namespace esp32emu {

namespace {
constexpr std::uint32_t Cmd = 0x00, User = 0x18, User2 = 0x20, MisoDlen = 0x28, W0 = 0x58;
constexpr std::uint32_t CmdUsr = 1u << 18;
constexpr std::uint32_t UserMiso = 1u << 28;

// Winbond-style JEDEC id: manufacturer, memory type, capacity as log2(bytes).
constexpr std::uint8_t Manufacturer = 0xef, MemoryType = 0x40;
} // namespace

SpiMem1::SpiMem1(std::uint32_t base, std::uint32_t flash_bytes)
    : Block(base, 0x200), flash_bytes_(flash_bytes) {}

std::uint32_t SpiMem1::read(std::uint32_t offset, unsigned) {
    return regs_[(offset >> 2) & 0x7f];
}

void SpiMem1::write(std::uint32_t offset, std::uint32_t value, unsigned) {
    const std::uint32_t index = (offset >> 2) & 0x7f;
    regs_[index] = value;
    if ((offset & ~3u) == Cmd) {
        if (value & CmdUsr)
            run_user_command();
        // Every other command bit (erase, program, read status, ...) completes
        // at once; nothing is stored in this flash through here.
        regs_[index] = 0;
    }
}

void SpiMem1::run_user_command() {
    const std::uint8_t command = static_cast<std::uint8_t>(regs_[User2 >> 2] & 0xff);
    std::uint32_t result = 0;
    switch (command) {
    case 0x9f: { // RDID
        unsigned capacity = 0;
        while ((1u << capacity) < flash_bytes_)
            ++capacity;
        result = Manufacturer | (MemoryType << 8) | (capacity << 16);
        break;
    }
    case 0x05: // RDSR: write-in-progress clear, WEL as set by WREN
        result = write_enabled_ ? 0x2 : 0;
        break;
    case 0x35: // RDSR2
    case 0x15: // RDSR3
        result = 0;
        break;
    case 0x06:
        write_enabled_ = true;
        break;
    case 0x04:
        write_enabled_ = false;
        break;
    default: // SFDP (0x5a) and anything else: an erased bus
        result = 0xffffffff;
        break;
    }
    if (regs_[User >> 2] & UserMiso) {
        regs_[W0 >> 2] = result;
        (void)MisoDlen;
    }
}

namespace {
constexpr std::uint32_t MmuItemContent = 0x37c, MmuItemIndex = 0x380;
constexpr std::uint32_t MmuValid = 1u << 9;
constexpr std::uint32_t MmuPageMask = 0x1ff;
} // namespace

SpiMem0::SpiMem0(std::uint32_t base, const std::vector<std::uint8_t> &flash, std::uint8_t *window,
                 std::function<void(unsigned page)> code_changed)
    : Block(base, 0x400), flash_(flash), window_(window), code_changed_(std::move(code_changed)) {}

std::uint32_t SpiMem0::read(std::uint32_t offset, unsigned) {
    offset &= ~3u;
    if (offset == MmuItemIndex)
        return index_;
    if (offset == MmuItemContent)
        return table_[index_ % Entries];
    return offset / 4 < regs_.size() ? regs_[offset / 4] : 0;
}

void SpiMem0::write(std::uint32_t offset, std::uint32_t value, unsigned) {
    offset &= ~3u;
    if (offset == MmuItemIndex) {
        index_ = value;
        return;
    }
    if (offset == MmuItemContent) {
        table_[index_ % Entries] = value;
        std::uint8_t *page = window_ + static_cast<std::uint64_t>(index_ % Entries) * PageSize;
        if (value & MmuValid) {
            const std::uint64_t source = static_cast<std::uint64_t>(value & MmuPageMask) * PageSize;
            std::memset(page, 0xff, PageSize);
            if (source < flash_.size())
                std::memcpy(page, flash_.data() + source,
                            std::min<std::uint64_t>(PageSize, flash_.size() - source));
        } else {
            std::memset(page, 0xff, PageSize);
        }
        code_changed_(index_ % Entries);
        return;
    }
    if (offset / 4 < regs_.size())
        regs_[offset / 4] = value;
}

void SpiMem0::refresh() {
    for (std::uint32_t i = 0; i < Entries; ++i) {
        if (!(table_[i] & MmuValid))
            continue;
        std::uint8_t *page = window_ + static_cast<std::uint64_t>(i) * PageSize;
        const std::uint64_t source = static_cast<std::uint64_t>(table_[i] & MmuPageMask) * PageSize;
        std::memset(page, 0xff, PageSize);
        if (source < flash_.size())
            std::memcpy(page, flash_.data() + source, std::min<std::uint64_t>(PageSize, flash_.size() - source));
    }
}

} // namespace esp32emu
