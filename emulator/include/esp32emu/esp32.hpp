#pragma once

#include "mcuemu/radio.hpp"
#include "riscv/core.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace esp32emu {

class Block;
class Rom;
class Intc;
class Ieee802154;

enum class Variant { C6, H2 };

struct RomSymbol {
    std::uint32_t address;
    const char *name;
};

// What differs between the chips: sizes, the peripherals' base addresses, the
// interrupt-matrix source numbers and the ROM symbol table. Register layouts of
// the blocks that are modelled are the same on both (checked against the SVDs).
struct Layout {
    const char *name;
    std::uint32_t chip_id; // the image header's chip id / _rom_chip_id
    std::uint32_t rom_size, sram_size, lp_sram_size;
    std::uint32_t clock_hz, xtal_hz;
    std::uint32_t uart0, systimer, timg0, timg1, spi0, spi1, ieee802154, lp_timer, int_matrix;
    unsigned zb_mac_source, systimer_source;
    std::uint32_t int_status;   // INT_STATUS_REG_0 in the interrupt matrix
    std::uint32_t modem_analog; // the analog I2C master of the modem block
    bool phy_rom; // the ROM carries PHY helpers (phy_get_romfuncs, rom_i2c_*): C6 only
    std::vector<RomSymbol> rom_symbols;
};
const Layout &layout_of(Variant variant);

// ESP32-C6 and ESP32-H2 (HP core). Work in progress: the memory map, a ROM replaced by
// high-level stand-ins, the boot of an image the way the ROM loader starts it,
// and a growing set of peripheral blocks. Register behaviour is documented in
// docs/esp32-riscv.md; anything not modelled yet is logged with --trace-mmio.
class Esp32 final : public riscv::Bus {
public:
    static constexpr std::uint32_t RomBase = 0x40000000;
    static constexpr std::uint32_t SramBase = 0x40800000;
    static constexpr std::uint32_t LpSramBase = 0x50000000;
    static constexpr std::uint32_t CacheBase = 0x42000000, CacheSize = 0x2000000;
    static constexpr std::uint32_t PeriphBase = 0x60000000, PeriphSize = 0x100000;
    static constexpr std::uint32_t IntcBase = 0x20000000, IntcSize = 0x2000;
    static constexpr std::uint32_t RomReturnZero = RomBase + 0x8; // an ebreak slot nobody handles

    struct Config {
        Variant variant{Variant::C6};
        std::uint32_t flash_size{4u << 20};
        // Makes the two PHY functions that need the real RF front end return at once
        // (found in the image by their first instructions). Without them the PHY's
        // calibration overruns Zephyr's main stack; see docs/esp32-riscv.md.
        bool stub_phy{true};
    };

    Esp32() : Esp32(Config{}) {}
    explicit Esp32(const Config &config);
    ~Esp32();

    const Layout &layout() const noexcept { return layout_; }
    std::uint32_t sram_size() const noexcept { return layout_.sram_size; }
    std::uint32_t clock_hz() const noexcept { return layout_.clock_hz; }
    // ROM-owned data at the top of the SRAM that the C6's PHY blob is handed pointers to.
    std::uint32_t phy_rom_funcs() const noexcept { return SramBase + layout_.sram_size - 0x500; }
    std::uint32_t phy_params() const noexcept { return SramBase + layout_.sram_size - 0x318; } // phy_param_rom
    // Scratch slots in the ROM's last bytes, one per PHY table entry.
    std::uint32_t phy_rom_slots() const noexcept { return RomBase + layout_.rom_size - 0x200; }

    // Writes `data` into the flash at `offset`.
    void load_flash(const std::vector<std::uint8_t> &data, std::uint32_t offset = 0);
    // Makes the firmware function at `address` return `value` at once (a0; a1 is
    // cleared) without running. For code the model cannot run: the closed-source
    // PHY blob's calibration, chiefly. The address belongs to one build of the
    // firmware (find it in the build's ELF).
    void stub_function(std::uint32_t address, std::uint32_t value);
    // Puts the 802.15.4 MAC on the shared medium as `node`.
    void connect_radio(mcuemu::VirtualAir &air, const std::string &node);
    // Starts the way the ROM loader does: reads the image header at flash
    // offset 0, copies its RAM segments and jumps to the entry point.
    void reset();
    riscv::RunResult run_until(std::uint64_t target_ns);

    std::uint64_t time_ns() const { return core_.cycles_to_ns(core_.cycles()); }
    riscv::Core &core() noexcept { return core_; }

    // Bytes the firmware sends through UART0 (or the ROM's printf).
    std::function<void(std::uint8_t)> on_uart_tx;
    // Logs every access to an address no block claims, and every ROM call.
    bool trace_mmio{false};
    bool trace_rom{false}; // log every ROM stand-in call with its first arguments
    bool trace_irq{false}; // log every change of the interrupt line to the hart
    // Address ranges [begin, end) of modelled blocks whose accesses are logged too (--trace-block).
    std::vector<std::pair<std::uint32_t, std::uint32_t>> trace_blocks;
    // Stops logging register accesses after this many lines (a polling loop makes
    // gigabytes of them).
    std::uint64_t trace_limit{200000};

    // riscv::Bus
    std::uint32_t read(std::uint32_t address, unsigned size) override;
    void write(std::uint32_t address, std::uint32_t value, unsigned size) override;

    // For the blocks.
    void uart_tx(std::uint8_t byte);
    std::uint64_t cycles() const { return core_.cycles(); }
    std::vector<std::uint8_t> &flash() { return flash_; }
    std::uint8_t *cache_window() { return cache_; }
    // The flash contents changed behind the cache's back (the ROM's flash routines).
    void flash_modified();
    void invalidate_code() { core_.invalidate_code(); }
    void log(const std::string &line) const;

private:
    void trace_block(bool write, std::uint32_t address, std::uint32_t value, unsigned size);
    void stub_phy_in(std::uint32_t virtual_base, const std::uint8_t *data, std::size_t length);

    Config config_;
    const Layout &layout_;
    riscv::Core core_;
    std::vector<std::uint8_t> flash_;
    std::uint8_t *cache_{nullptr};
    // MPCER, MPCMR. The counter runs from reset (counting cycles): the H2 bootloader reads
    // it without ever enabling it (bootloader_fill_random's wait loop would not end).
    std::uint32_t pc_regs_[2]{1, 1};
    std::map<unsigned, std::uint32_t> pma_; // PMACFG/PMAADDR CSRs
    std::uint64_t pc_count_{0};  // MPCCR minus the cycles counted while enabled
    std::map<std::uint32_t, std::uint32_t> stubs_;
    std::unique_ptr<Rom> rom_;
    std::unique_ptr<Intc> intc_;
    Ieee802154 *mac_{nullptr};
    class SpiMem0 *spi0_{nullptr};
    std::vector<std::unique_ptr<Block>> blocks_;
    std::map<std::uint32_t, Block *> by_base_;
    // Registers no block claims read back what was written (zero before).
    std::map<std::uint32_t, std::uint32_t> scratch_;
};

} // namespace esp32emu
