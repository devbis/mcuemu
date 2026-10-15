#pragma once

#include "esp32emu/esp32.hpp"
#include "sim/mcu.hpp"

#include <array>
#include <string>
#include <vector>

namespace sim {

// ESP32-C6 / ESP32-H2 adapter: the radio (802.15.4 only), UART0 and the GPIO pads used as plain
// GPIO (named GPIO0.. : 31 pads on the C6, 28 on the H2).
class Esp32Mcu final : public Mcu {
public:
    Esp32Mcu(std::string name, esp32emu::Variant variant);

    std::string_view model() const override { return variant_ == esp32emu::Variant::C6 ? "esp32c6" : "esp32h2"; }
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
    void attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                      const std::vector<std::string> &protocols) override;
    void read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) override;
    void write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) override;

    unsigned uart_count() const override { return 1; }
    bool uart_can_receive(unsigned index) const override { return index == 0 && chip_.uart_can_receive(); }
    bool uart_rx(unsigned index, std::uint8_t byte) override { return index == 0 && chip_.uart_rx(byte); }
    std::vector<std::uint8_t> uart_take_tx(unsigned index) override;

    esp32emu::Esp32 &chip() noexcept { return chip_; }

private:
    void drive_pad(unsigned pad);

    std::array<Pin *, 32> pins_{};
    esp32emu::Variant variant_;
    esp32emu::Esp32 chip_;
    McuState state_{McuState::Running};
    std::string fault_;
    std::vector<std::uint8_t> uart_tx_;
};

} // namespace sim
