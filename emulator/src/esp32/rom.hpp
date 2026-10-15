#pragma once

#include <cstdint>
#include <vector>
#include <map>
#include <functional>
#include <string>
#include <unordered_map>

namespace esp32emu {

class Esp32;

// Stand-ins for the ESP32-C6 mask ROM. There is no ROM image: the ROM region is
// filled with ebreak, the core stops there and the chip hands the call to this
// class, which runs host code for it (C ABI: arguments in a0..a7, result in a0
// and a1) and returns to ra. Names come from the ROM linker scripts
// (esp32c6_rom.hpp). A call nobody handles is logged and returns 0.
class Rom {
public:
    explicit Rom(Esp32 &chip);

    // True when `pc` was a ROM entry and has been serviced.
    bool call(std::uint32_t pc);
    const char *name_at(std::uint32_t address) const;

private:
    std::map<std::uint32_t, std::vector<std::uint8_t>> sha_data_; // by context address
    using Handler = std::function<void()>;
    void add(const char *name, Handler handler);
    void add_address(std::uint32_t address, std::string name, Handler handler);
    std::uint32_t analog_forced(std::uint32_t block, std::uint32_t reg) const;
    std::string format_printf(std::uint32_t format, unsigned first_arg);

    Esp32 &chip_;
    std::unordered_map<std::uint32_t, std::string> names_;
    std::unordered_map<std::uint32_t, Handler> handlers_;
    std::unordered_map<std::uint32_t, unsigned> unhandled_;
    std::unordered_map<std::uint32_t, std::uint32_t> analog_; // (block << 8 | register) -> value
    std::uint64_t result_{0}; // a0 (low) and a1 (high) on return
};

} // namespace esp32emu
