#pragma once

#include "bl702emu/bl702.hpp"
#include "sim/mcu.hpp"

#include <string>
#include <vector>

namespace sim {

// BL702 adapter (the XT-ZB1 module): the IEEE 802.15.4 radio and UART0. The GPIO pads are not modelled yet,
// nor is USB.
class Bl702Mcu final : public Mcu {
public:
    explicit Bl702Mcu(std::string name);

    std::string_view model() const override { return "bl702"; }
    void load_firmware(const std::string &path) override;
    void load_images(const std::vector<FirmwareImage> &images) override;
    void reset() override;
    void set_start_offset(std::uint64_t offset_ns) override;
    std::uint64_t time_ns() const override { return chip_.time_ns(); }
    McuState run_until(std::uint64_t target_ns) override;
    McuState state() const noexcept override { return state_; }
    const std::string &fault_message() const noexcept override { return fault_; }

    void connect_pin(std::string_view pad, Pin &pin) override;
    void attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                      const std::vector<std::string> &protocols) override;
    void read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) override;
    void write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) override;

    unsigned uart_count() const override { return 1; }
    std::vector<std::uint8_t> uart_take_tx(unsigned index) override;

    bl702emu::Bl702 &chip() noexcept { return chip_; }

private:
    bl702emu::Bl702 chip_;
    McuState state_{McuState::Running};
    std::string fault_;
    std::vector<std::uint8_t> uart_tx_;
};

} // namespace sim
