#pragma once

#include "sim/mcu.hpp"
#include "tc32emu/cpu.hpp"
#include "tc32emu/tlsr8258.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace sim {

// Pads are named PA0..PE7 (port letter + bit).
class Tlsr8258Mcu final : public Mcu {
public:
    static constexpr unsigned PortCount = 5;

    Tlsr8258Mcu(std::string name,
                tc32emu::Tlsr8258Model model = tc32emu::Tlsr8258Model::F512,
                std::uint32_t clock_hz = 48000000);

    std::string_view model() const override { return "tlsr8258"; }
    void load_firmware(const std::string &path) override;
    void load_images(const std::vector<FirmwareImage> &images) override;
    void reset() override;
    void set_start_offset(std::uint64_t offset_ns) override;
    std::uint64_t time_ns() const override { return device_.network_time_ns(); }
    McuState run_until(std::uint64_t target_ns) override;
    McuState state() const noexcept override { return state_; }
    const std::string &fault_message() const noexcept override { return fault_; }

    std::vector<std::string> pin_names() const override;
    void connect_pin(std::string_view pad, Pin &pin) override;
    // Only the Zigbee RF path is connected to the chip's RF registers.
    void attach_radio(tc32emu::VirtualAir &air, const std::string &node,
                      const std::vector<std::string> &protocols) override;
    void read_memory(std::uint32_t address, std::uint8_t *out,
                     std::size_t length) override;
    void write_memory(std::uint32_t address, const std::uint8_t *data,
                      std::size_t length) override;
    unsigned uart_count() const override { return 1; }
    bool uart_can_receive(unsigned index) const override { return index == 0; }
    bool uart_rx(unsigned index, std::uint8_t byte) override;
    std::vector<std::uint8_t> uart_take_tx(unsigned index) override;

    // Escape hatch for chip-specific tests and tooling.
    tc32emu::Tlsr8258 &device() noexcept { return device_; }
    tc32emu::Cpu &cpu() noexcept { return cpu_; }

private:
    void drive_port(unsigned port);
    void drive_pad(unsigned port, unsigned bit);
    void sample_pin(unsigned port, unsigned bit, Level level);
    void on_i2c_pads(const tc32emu::I2cPads &pads, bool sda_low, bool scl_low);

    tc32emu::Tlsr8258 device_;
    tc32emu::Cpu cpu_;
    McuState state_{McuState::Running};
    std::string fault_;
    std::array<std::array<Pin *, 8>, PortCount> pins_{};
    // Pads currently pulled low by the I2C master.
    std::array<std::array<bool, 8>, PortCount> i2c_low_{};
};

} // namespace sim
