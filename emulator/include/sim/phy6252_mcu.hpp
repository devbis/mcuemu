#pragma once
#include "phy6252emu/phy6252.hpp"
#include "sim/mcu.hpp"

#include <array>

namespace sim {
class Phy6252Mcu final : public Mcu {
public:
    explicit Phy6252Mcu(std::string name) : Mcu(std::move(name)) {
        chip_.set_gpio_observer([this](unsigned pin) { drive_gpio(pin); });
    }
    std::string_view model() const override { return "phy6252"; }
    void load_firmware(const std::string &path) override;
    void load_images(const std::vector<FirmwareImage> &images) override;
    void reset() override;
    void set_start_offset(std::uint64_t ns) override { chip_.core().advance_to(chip_.core().ns_to_cycles(ns)); }
    std::uint64_t time_ns() const override { return chip_.core().cycles_to_ns(chip_.core().cycles()); }
    McuState run_until(std::uint64_t target_ns) override;
    McuState state() const noexcept override { return state_; }
    const std::string &fault_message() const noexcept override { return fault_; }
    std::vector<std::string> pin_names() const override;
    void connect_pin(std::string_view pad, Pin &) override;
    void attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                      const std::vector<std::string> &protocols) override;
    void read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) override;
    void write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) override;
    unsigned uart_count() const override { return 1; }
    bool uart_can_receive(unsigned index) const override { return index == 0; }
    bool uart_rx(unsigned index, std::uint8_t byte) override;
    std::vector<std::uint8_t> uart_take_tx(unsigned index) override;

    phy6252emu::Phy6252 &chip() noexcept { return chip_; }
private:
    static constexpr unsigned GpioCount = 23;
    phy6252emu::Phy6252 chip_;
    std::array<Pin *, GpioCount> pins_{};
    McuState state_{McuState::Running};
    std::string fault_;
    void drive_gpio(unsigned pin);
};
} // namespace sim
