#pragma once

#include "mcuemu/bus.hpp"

#include <cstdint>
#include <vector>

namespace mcuemu {

enum class Tlsr8258PowerState {
    Awake,
    Suspend,
    DeepSleep,
};

// Where a TC32 chip puts its memories. The Cpu maps the flash at 0, the SRAM
// and the register window(s) from this, so one wrapper runs every Telink TC32
// chip (TLSR8258 and the TLSR826x family).
struct Tc32Layout {
    std::uint32_t sram_base;
    std::uint32_t sram_size;
    std::uint32_t register_base;
    std::uint32_t register_size;
    // Second view of the register window (the TLSR8258 vendor alias); 0 = none.
    std::uint32_t register_alias_base;
};

// What Cpu needs from a chip model. Cpu drives a TC32 core and asks the device
// for timing, interrupts, sleep and the flash/SRAM host copies.
class Tc32Device : public Bus {
public:
    virtual const Tc32Layout &layout() const noexcept = 0;
    virtual std::uint32_t flash_size() const noexcept = 0;
    virtual const std::vector<std::uint8_t> &flash() const noexcept = 0;
    virtual std::uint8_t *sram_data() noexcept = 0;
    // Bytes at the bottom of the code address space fetched from SRAM (the
    // RAM code the startup copies there).
    virtual std::uint32_t ram_code_bytes() const noexcept = 0;
    virtual bool take_flash_dirty(std::uint32_t &begin, std::uint32_t &end) noexcept = 0;
    virtual Tlsr8258PowerState power_state() const noexcept = 0;
    // See Tlsr8258::cycles_until_event().
    virtual std::uint64_t cycles_until_event() const noexcept = 0;
    virtual std::uint64_t network_time_ns() const noexcept = 0;
    virtual std::uint32_t clock_hz() const noexcept = 0;
    virtual bool advance_sleep_to_wakeup() = 0;
    virtual bool consume_deep_sleep_wakeup() noexcept = 0;
};

} // namespace mcuemu
