#ifndef MCS51EMU_CA51F253L3_HPP
#define MCS51EMU_CA51F253L3_HPP

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>

struct uc_struct;

namespace mcs51emu {

struct RunResult {
    std::size_t instructions{};
    std::uint16_t pc{};
    std::uint64_t cycles{};
    bool step_limit_reached{};
    std::string error;
};

// Initial CA51F253L3 memory shell. The core uses a 64 KiB 8051 code address
// space, while image loading enforces the device's 32 KiB flash capacity.
class Ca51F253L3 {
public:
    static constexpr std::size_t FlashSize = 32 * 1024;
    static constexpr std::uint64_t CodeSize = 64 * 1024;
    static constexpr std::uint64_t XdataBase = 0x00010000ULL;
    static constexpr std::uint64_t XdataSize = 64 * 1024;
    static constexpr std::uint64_t IdataBase = 0x00020000ULL;
    static constexpr std::uint64_t SfrBase = 0x00030000ULL;
    static constexpr std::uint64_t MappedInternalSize = 4096;

    Ca51F253L3();
    ~Ca51F253L3();
    Ca51F253L3(const Ca51F253L3 &) = delete;
    Ca51F253L3 &operator=(const Ca51F253L3 &) = delete;

    void load_firmware(const std::string &path);
    void reset();
    RunResult run(std::size_t instruction_limit);

    std::uint8_t read_sfr(std::uint8_t address) const;
    std::uint8_t read_idata(std::uint8_t address) const;
    std::uint8_t read_xdata(std::uint16_t address) const;
    void write_sfr(std::uint8_t address, std::uint8_t value);
    void set_trace(std::ostream *output) noexcept { trace_output_ = output; }

private:
    static void code_hook(uc_struct *uc, std::uint64_t address,
                          std::uint32_t size, void *user_data);
    void check_error(int error, const char *operation) const;

    uc_struct *uc_{};
    std::size_t code_hook_{};
    std::size_t executed_instructions_{};
    std::ostream *trace_output_{};
};

} // namespace mcs51emu

#endif
