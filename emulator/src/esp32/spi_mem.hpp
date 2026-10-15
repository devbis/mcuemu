#pragma once

#include "blocks.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace esp32emu {

// SPI1 (SPI_MEM1): the flash controller the CPU drives by hand. Only the
// "user command" path is modelled, which is what bootloaders and flash drivers
// use for JEDEC id, status, write enable and SFDP: the firmware loads the
// command, address and data registers, sets CMD.USR and waits for it to clear.
// Commands complete instantly. Documented in the TRM (SPI_MEM chapter); the flash
// itself is a generic NOR part: it answers RDID with the id below, reports not
// busy, and ignores writes (the CPU never writes the image through here).
class SpiMem1 final : public Block {
public:
    SpiMem1(std::uint32_t base, std::uint32_t flash_bytes);

    std::uint32_t read(std::uint32_t offset, unsigned size) override;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;

private:
    void run_user_command();

    std::uint32_t flash_bytes_;
    std::array<std::uint32_t, 0x80> regs_{}; // word index = offset / 4, up to 0x1ff
    bool write_enabled_{false};
};

// SPI0 (SPI_MEM0): the cache's side of the flash controller. Of its registers
// only the MMU table is modelled: ITEM_INDEX selects an entry, ITEM_CONTENT reads
// or writes it. An entry maps one 64 KiB page of the cache window
// (0x42000000 + 64 KiB * index) to a page of the flash: bits 0..8 are the flash
// page, bit 9 marks it valid, bit 10 "sensitive" (encryption; ignored). Layout
// from ESP-IDF (soc/esp32c6/include/soc/ext_mem_defs.h, hal/esp32c6/mmu_ll.h).
// The window is filled when an entry is written (and `code_changed` told which page); the flash is not written
// through the cache, so copies cannot go stale.
class SpiMem0 final : public Block {
public:
    static constexpr std::uint32_t Entries = 256;
    static constexpr std::uint32_t PageSize = 0x10000;

    SpiMem0(std::uint32_t base, const std::vector<std::uint8_t> &flash, std::uint8_t *window,
            std::function<void(unsigned page)> code_changed);

    std::uint32_t read(std::uint32_t offset, unsigned size) override;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;
    // Copies the flash again into every mapped page (after the flash was written or erased).
    void refresh();

private:
    const std::vector<std::uint8_t> &flash_;
    std::uint8_t *window_;
    std::function<void(unsigned page)> code_changed_;
    std::uint32_t index_{0};
    std::array<std::uint32_t, Entries> table_{};
    std::array<std::uint32_t, 0x100> regs_{}; // everything else reads back
};

} // namespace esp32emu
