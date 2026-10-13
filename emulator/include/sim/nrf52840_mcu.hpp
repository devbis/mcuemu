#pragma once

#include "nrf52emu/nrf52840.hpp"
#include "sim/mcu.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace sim {

// nRF52840 adapter. Pads are named like the datasheet: P0.00..P0.31, P1.00..P1.15.
class Nrf52840Mcu final : public Mcu {
public:
    explicit Nrf52840Mcu(std::string name);

    std::string_view model() const override { return "nrf52840"; }
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

    unsigned uart_count() const override { return nrf52emu::Nrf52840::UartCount; }
    bool uart_can_receive(unsigned index) const override {
        return const_cast<nrf52emu::Nrf52840 &>(chip_).uart_can_receive(index);
    }
    bool uart_rx(unsigned index, std::uint8_t byte) override { return chip_.uart_receive(index, byte); }
    std::vector<std::uint8_t> uart_take_tx(unsigned index) override { return chip_.uart_take_tx(index); }

    nrf52emu::Nrf52840 &chip() noexcept { return chip_; }

private:
    void drive_pad(unsigned pad);

    nrf52emu::Nrf52840 chip_;
    McuState state_{McuState::Running};
    std::string fault_;
    std::array<Pin *, nrf52emu::Nrf52840::PadCount> pins_{};
};

} // namespace sim
