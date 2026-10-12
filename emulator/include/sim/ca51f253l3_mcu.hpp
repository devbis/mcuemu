#pragma once

#include "mcs51emu/ca51f253l3.hpp"
#include "sim/mcu.hpp"

#include <cstdint>
#include <string>

namespace sim {

// CA51F253L3 adapter. The chip model has no GPIO yet, so no pads can be wired.
class Ca51F253L3Mcu final : public Mcu {
public:
    // The cycle counter ticks at FOSC; the clock is not yet derived from the
    // chip's clock configuration, so it is a parameter.
    explicit Ca51F253L3Mcu(std::string name,
                           std::uint32_t clock_hz = 24000000);

    std::string_view model() const override { return "ca51f253l3"; }
    void load_firmware(const std::string &path) override;
    void reset() override;
    void set_start_offset(std::uint64_t offset_ns) override {
        offset_ns_ = offset_ns;
    }
    std::uint64_t time_ns() const override;
    McuState run_until(std::uint64_t target_ns) override;
    McuState state() const noexcept override { return state_; }
    const std::string &fault_message() const noexcept override { return fault_; }

    void connect_pin(std::string_view pad, Pin &pin) override;
    // UART0/UART1 transmit output is captured; receive is not modeled yet.
    unsigned uart_count() const override { return 2; }
    std::vector<std::uint8_t> uart_take_tx(unsigned index) override;

    mcs51emu::Ca51F253L3 &device() noexcept { return device_; }

private:
    mcs51emu::Ca51F253L3 device_;
    std::uint32_t clock_hz_;
    std::uint64_t offset_ns_{0};
    std::uint64_t cycles_{0};
    McuState state_{McuState::Running};
    std::string fault_;
    std::size_t uart_taken_[2]{};
};

} // namespace sim
