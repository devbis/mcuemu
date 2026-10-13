// nRF52840 NVMC: flash and UICR programming.
//
// Evidence: nRF52840 Product Specification, chapter "NVMC". A word write can
// only clear bits (the result is old AND new), a page erase sets a 4 KiB page
// to 0xFF. The model completes operations instantly.

#include "nrf52/peripherals.hpp"

#include <algorithm>
#include <cstring>

namespace nrf52emu {

namespace {
constexpr std::uint32_t PageSize = 4096;
constexpr std::uint32_t ConfigWen = 1, ConfigEen = 2;
} // namespace

std::uint32_t Nvmc::read(std::uint32_t offset) {
    switch (offset) {
    case 0x400: return 1; // READY
    case 0x408: return 1; // READYNEXT
    case 0x504: return config_;
    case 0x540: return icache_;
    default: return 0;
    }
}

void Nvmc::write(std::uint32_t offset, std::uint32_t value) {
    switch (offset) {
    case 0x504:
        config_ = value & 3u;
        break;
    case 0x508:   // ERASEPAGE
    case 0x510: { // ERASEPCR0 (the same operation)
        if (config_ != ConfigEen)
            break;
        const std::uint32_t page = value & ~(PageSize - 1);
        if (page < Nrf52840::FlashSize) {
            std::memset(chip_.flash() + page, 0xff, PageSize);
            if (page < chip_.code_end())
                chip_.core().invalidate_code();
        }
        break;
    }
    case 0x50c: // ERASEALL
        if (config_ == ConfigEen) {
            std::memset(chip_.flash(), 0xff, Nrf52840::FlashSize);
            std::memset(chip_.uicr(), 0xff, 0x1000);
            chip_.core().invalidate_code();
        }
        break;
    case 0x514: // ERASEUICR
        if (config_ == ConfigEen)
            std::memset(chip_.uicr(), 0xff, 0x1000);
        break;
    case 0x540: icache_ = value; break;
    default: break;
    }
}

std::uint8_t *Nvmc::target(std::uint32_t address, unsigned size) {
    if (address + size <= Nrf52840::FlashSize)
        return chip_.flash() + address;
    if (address >= Nrf52840::UicrBase && address + size <= Nrf52840::UicrBase + 0x1000)
        return chip_.uicr() + (address - Nrf52840::UicrBase);
    if (address >= Nrf52840::FicrBase && address + size <= Nrf52840::FicrBase + 0x1000)
        return chip_.ficr() + (address - Nrf52840::FicrBase);
    return nullptr;
}

bool Nvmc::on_store(std::uint32_t address, unsigned size, std::uint32_t) {
    std::uint8_t *where = target(address, size);
    if (!where)
        return true; // reserved space: the store is dropped by the bus
    std::uint8_t old[4];
    std::memcpy(old, where, size);
    const bool writable = config_ == ConfigWen &&
                          !(address >= Nrf52840::FicrBase && address < Nrf52840::UicrBase);
    // The engine stores the value after this handler returns; settle the
    // result of the hardware rule just before the next instruction.
    chip_.core().defer([where, size, old, writable, address, this] {
        for (unsigned i = 0; i != size; ++i)
            where[i] = writable ? static_cast<std::uint8_t>(old[i] & where[i]) : old[i];
        // Translated code is only stale when the program image itself changed.
        if (address < chip_.code_end())
            chip_.core().invalidate_code();
    });
    return true;
}

} // namespace nrf52emu
