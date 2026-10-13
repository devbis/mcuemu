#include <limits>
#include <algorithm>
#include "sim/ca51f253l3_mcu.hpp"

#include <stdexcept>

namespace sim {

Ca51F253L3Mcu::Ca51F253L3Mcu(std::string name, std::uint32_t clock_hz)
    : Mcu(std::move(name)), clock_hz_(clock_hz) {
    if (clock_hz == 0)
        throw std::invalid_argument("clock_hz must be positive");
}

void Ca51F253L3Mcu::load_firmware(const std::string &path) {
    device_.load_firmware(path);
    reset();
}

void Ca51F253L3Mcu::reset() {
    device_.reset();
    cycles_ = 0;
    state_ = McuState::Running;
    fault_.clear();
    uart_taken_[0] = uart_taken_[1] = 0;
}

std::uint64_t Ca51F253L3Mcu::time_ns() const {
    // 128-bit intermediate: cycles * 1e9 overflows 64 bits after ~18 s of
    // 1 GHz-equivalent time.
    return offset_ns_ + static_cast<std::uint64_t>(
                            static_cast<unsigned __int128>(cycles_) *
                            1000000000u / clock_hz_);
}

McuState Ca51F253L3Mcu::run_until(std::uint64_t target_ns) {
    while (state_ == McuState::Running && time_ns() < target_ns) {
        // time_ns() is the cycle count scaled by the clock, so the first cycle
        // whose time is at or past the target is the engine's limit.
        const unsigned __int128 scaled =
            static_cast<unsigned __int128>(target_ns) * clock_hz_;
        const std::uint64_t target_cycle =
            static_cast<std::uint64_t>((scaled + 999999999u) / 1000000000u);
        const mcs51emu::RunResult result =
            device_.run(std::numeric_limits<std::size_t>::max(), target_cycle);
        cycles_ = result.cycles;
        if (!result.error.empty()) {
            state_ = McuState::Fault;
            fault_ = result.error;
        }
    }
    return state_;
}

void Ca51F253L3Mcu::connect_pin(std::string_view pad, Pin &) {
    throw std::invalid_argument("CA51F253L3 model has no GPIO yet; cannot wire '" +
                                std::string(pad) + "'");
}

std::vector<std::uint8_t> Ca51F253L3Mcu::uart_take_tx(unsigned index) {
    if (index > 1)
        return {};
    const std::string &all =
        index == 0 ? device_.uart0_output() : device_.uart1_output();
    std::vector<std::uint8_t> bytes(all.begin() + uart_taken_[index], all.end());
    uart_taken_[index] = all.size();
    return bytes;
}

} // namespace sim
