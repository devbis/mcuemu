#pragma once

#include "sim/mcu.hpp"
#include "stm32wb55emu/stm32wb55.hpp"

#include <algorithm>
#include <string>

namespace sim {

class Stm32wb55Mcu final : public Mcu {
public:
    explicit Stm32wb55Mcu(std::string name);

    std::string_view model() const override { return "stm32wb55"; }
    void load_firmware(const std::string &path) override;
    void load_images(const std::vector<FirmwareImage> &images) override;
    void load_cpu2_images(const std::vector<FirmwareImage> &images);
    void reset() override;
    void set_start_offset(std::uint64_t offset_ns) override;
    std::uint64_t time_ns() const override {
        return std::max(chip_.time_ns(), chip_.cpu2_time_ns());
    }
    McuState run_until(std::uint64_t target_ns) override;
    McuState state() const noexcept override { return state_; }
    const std::string &fault_message() const noexcept override { return fault_; }

    std::vector<std::string> pin_names() const override { return {}; }
    void connect_pin(std::string_view pad, Pin &) override;
    void attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                      const std::vector<std::string> &protocols) override;
    void read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) override;
    void write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) override;
    unsigned uart_count() const override { return 1; }
    bool uart_can_receive(unsigned index) const override { return index == 0; }
    bool uart_rx(unsigned index, std::uint8_t byte) override {
        return index == 0 && chip_.uart_rx(byte);
    }
    std::vector<std::uint8_t> uart_take_tx(unsigned index) override;

    stm32wb55emu::Stm32wb55 &chip() noexcept { return chip_; }

private:
    stm32wb55emu::Stm32wb55 chip_;
    McuState state_{McuState::Running};
    std::string fault_;
};

} // namespace sim
