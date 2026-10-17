#pragma once

#include "efr32emu/efr32mg21.hpp"
#include "sim/mcu.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace sim {

// EFR32MG21 adapter. Pads are named like the datasheet: PA0..PD15.
class Efr32mg21Mcu final : public Mcu {
public:
    explicit Efr32mg21Mcu(std::string name);

    std::string_view model() const override { return "efr32mg21"; }
    void load_firmware(const std::string &path) override;
    void load_images(const std::vector<FirmwareImage> &images) override;
    void reset() override;
    void set_start_offset(std::uint64_t offset_ns) override;
    std::uint64_t time_ns() const override { return chip_.time_ns(); }
    McuState run_until(std::uint64_t target_ns) override;
    McuState state() const noexcept override { return state_; }
    const std::string &fault_message() const noexcept override { return fault_; }

    std::vector<std::string> pin_names() const override;
    void connect_pin(std::string_view pad, Pin &pin) override;
    // IEEE 802.15.4 only: the model has no BLE radio.
    void attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                      const std::vector<std::string> &protocols) override;
    void read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) override;
    void write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) override;
    // The console (USART0) is the only UART; its output is taken as it grows.
    unsigned uart_count() const override { return 1; }
    std::vector<std::uint8_t> uart_take_tx(unsigned index) override;

    efr32emu::Efr32mg21 &chip() noexcept { return chip_; }

private:
    void drive_pad(unsigned pad);

    efr32emu::Efr32mg21 chip_;
    McuState state_{McuState::Running};
    std::size_t uart_taken_{0};
    std::string fault_;
    std::array<Pin *, efr32emu::Efr32mg21::PadCount> pins_{};
};

} // namespace sim
