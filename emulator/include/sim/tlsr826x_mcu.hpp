#pragma once

#include "sim/mcu.hpp"
#include "mcuemu/cpu.hpp"
#include "mcuemu/tlsr826x.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace sim {

// TLSR8267/TLSR8269. Pads are named PA0..PE7 (port letter + bit).
class Tlsr826xMcu final : public Mcu {
public:
    static constexpr unsigned PortCount = 5;

    Tlsr826xMcu(std::string name,
                mcuemu::Tlsr826xModel model = mcuemu::Tlsr826xModel::T8269,
                std::uint32_t clock_hz = mcuemu::Tlsr826x::DefaultClockHz);

    std::string_view model() const override {
        return device_.model() == mcuemu::Tlsr826xModel::T8269 ? "tlsr8269" : "tlsr8267";
    }
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
    void attach_radio(mcuemu::VirtualAir &air, const std::string &node,
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
    mcuemu::Tlsr826x &device() noexcept { return device_; }
    mcuemu::Cpu &cpu() noexcept { return cpu_; }

private:
    void drive_port(unsigned port);
    void drive_pad(unsigned port, unsigned bit);
    void sample_pin(unsigned port, unsigned bit, Level level);

    mcuemu::Tlsr826x device_;
    mcuemu::Cpu cpu_;
    McuState state_{McuState::Running};
    std::string fault_;
    std::array<std::array<Pin *, 8>, PortCount> pins_{};
};

} // namespace sim
