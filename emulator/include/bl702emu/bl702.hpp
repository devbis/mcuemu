#pragma once

#include "mcuemu/radio.hpp"
#include "riscv/core.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace bl702emu {

class Block;
class Clic;
class Ieee802154;
class Rom;

// Bouffalo BL702 (a RISC-V core, built as riscv32imc by zigbee-rs, at 32 MHz on the XTAL; the module is the XT-ZB1). Work in
// progress: the memory map, a boot ROM replaced by host code, the boot header of a flash image, and the
// peripherals the zigbee-rs sensor firmware touches: GLB (scratch), UART0 (transmit), TIMER0 (1 MHz
// counter), eFuse (MAC), the AES accelerator of SEC_ENG, the RF/PHY calibration state and the IEEE
// 802.15.4 MAC (M154) on the shared air. Anything else in the peripheral window reads back what was
// written. The firmware is polled: no interrupt controller (CLIC) is modelled yet.
// Documented and inferred behaviour is listed in docs/bl702.md.
class Bl702 final : public riscv::Bus {
public:
    static constexpr std::uint32_t RomBase = 0x21000000, RomSize = 0x20000;
    static constexpr std::uint32_t RomApiTable = 0x21018800; // the ROM function table the HAL calls through
    static constexpr std::uint32_t FlashBase = 0x23000000;   // the XIP window (where reset() maps the code)
    static constexpr std::uint32_t SramBase = 0x42010000, SramSize = 0x20000;
    static constexpr std::uint32_t ItcmBase = 0x22014000, ItcmSize = 0x4000; // instruction TCM (Zephyr copies code here)
    static constexpr std::uint32_t PeriphBase = 0x40000000, PeriphSize = 0x10000;
    static constexpr std::uint32_t M154Base = 0x4c000000, M154Size = 0x1000;
    static constexpr std::uint32_t ClintBase = 0x02000000;     // machine timer (mtime, mtimecmp)
    static constexpr std::uint32_t ClicBase = 0x02800000;      // CLIC, hart 0 (the SoC's layout: +0x800000)
    static constexpr std::uint32_t ClockHz = 32000000;

    struct Config {
        std::uint32_t flash_size{1u << 20}; // the XT-ZB1 carries 1 MiB of SPI flash
        // The IEEE address the eFuse reports (the chip's MAC bytes, in transmission order).
        std::array<std::uint8_t, 8> mac{};
    };

    Bl702() : Bl702(Config{}) {}
    explicit Bl702(const Config &config);
    ~Bl702();
    Bl702(const Bl702 &) = delete;
    Bl702 &operator=(const Bl702 &) = delete;

    // Writes `data` into the flash at `offset` (the rest stays erased).
    void load_flash(const std::vector<std::uint8_t> &data, std::uint32_t offset = 0);
    // Starts the way the boot ROM does: checks the 'BFNP' header at flash offset 0, copies the segments that
    // load into SRAM, maps the XIP segment and jumps to the boot entry. Throws when there is no header.
    void reset();
    // Runs until `target_ns` or until the hart stops (Halted, Fault, Reset).
    riscv::RunResult run_until(std::uint64_t target_ns);

    std::uint64_t time_ns() const;
    riscv::Core &core() noexcept { return core_; }
    std::uint32_t flash_size() const noexcept { return config_.flash_size; }
    // Puts the IEEE 802.15.4 MAC on the shared medium as `node`.
    void connect_radio(mcuemu::VirtualAir &air, const std::string &node);

    // Bytes the firmware sends through UART0, as they leave the transmit FIFO.
    std::function<void(std::uint8_t)> on_uart_tx;

    // riscv::Bus: the peripheral window and the M154 window.
    std::uint32_t read(std::uint32_t address, unsigned size) override;
    void write(std::uint32_t address, std::uint32_t value, unsigned size) override;

    // For the blocks and the ROM.
    void uart_tx(std::uint8_t byte);
    // The physical flash (what the ROM's flash routines program), by flash offset.
    std::uint8_t *flash() noexcept { return flash_.data(); }
    // The flash changed: the XIP window follows it, and translated code is stale.
    void flash_modified();
    // The flash controller's XIP offset register (SF_CTRL_SF_ID0_OFFSET): its low 24 bits are the physical
    // flash offset the window's base shows (the SDK keeps the top byte). Firmware moves the window to read the
    // boot header and then restores the register.
    std::uint32_t xip_offset_register() const noexcept { return sf_offset_reg_; }
    void set_xip_offset_register(std::uint32_t value);

private:
    std::uint64_t service(std::uint64_t now);
    Block *find_block(std::uint32_t address) const;
    // Raises or drops the CLIC lines of the peripherals and wakes the hart if an interrupt is pending.
    void sync_interrupts();
    // Copies the flash into the XIP window: window byte i is flash byte i + window_shift_ (0xff outside).
    void sync_window();

    Config config_;
    riscv::Core core_;
    std::vector<std::uint8_t> flash_;
    std::uint8_t *window_{nullptr}; // the XIP window at FlashBase, mapped in the core
    std::int64_t window_shift_{0};  // set by the XIP segment of the boot header
    std::uint32_t sf_offset_reg_{0};
    std::unique_ptr<Rom> rom_;
    std::vector<std::unique_ptr<Block>> blocks_;
    Clic *clic_{nullptr};      // inside blocks_
    Ieee802154 *mac_{nullptr}; // the M154 block inside blocks_
    // Peripheral registers no block models read back what was written (zero before).
    std::map<std::uint32_t, std::uint32_t> scratch_;
};

} // namespace bl702emu
