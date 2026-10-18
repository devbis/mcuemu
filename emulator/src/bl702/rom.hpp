#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <map>

namespace bl702emu {

class Bl702;

// Stand-ins for the BL702 boot ROM. There is no ROM image: the ROM region is filled with ebreak, and the
// ROM API table at Bl702::RomApiTable (which the HAL calls through) points at one stub per entry. The
// core stops on a stub, the chip hands the call to this class, which runs host code for it (C ABI:
// arguments in a0..a5, the result in a0, a1 cleared) and returns to ra. An entry nobody handles returns 0.
class Rom {
public:
    static constexpr std::uint32_t StubBase = 0x21010000;
    static constexpr unsigned Entries = 256;

    explicit Rom(Bl702 &chip);

    // True when `pc` is a stub; its host code has run and the core is back at the caller.
    bool call(std::uint32_t pc);

private:
    using Handler = std::function<std::uint32_t(const std::array<std::uint32_t, 6> &args)>;
    void add(unsigned index, Handler handler) { handlers_[index] = std::move(handler); }
    // The physical flash offset a caller passes, or -1 when [offset, offset + length) is outside the flash.
    std::int64_t flash_offset(std::uint32_t address, std::uint32_t length) const;

    Bl702 &chip_;
    std::map<unsigned, Handler> handlers_;
};

} // namespace bl702emu
