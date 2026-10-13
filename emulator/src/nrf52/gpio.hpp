#pragma once

#include "nrf52/peripheral.hpp"

#include <array>
#include <optional>

namespace nrf52emu {

class Gpiote;

// The two GPIO ports (P0 at 0x50000000, P1 at 0x50000300: 32 and 16 pins).
// Not an ordinary instance: one object serves both port register blocks and
// owns the pad state that everything else (GPIOTE, serial peripherals, the
// board adapter) reads and drives.
class Gpio final : public Peripheral {
public:
    explicit Gpio(Nrf52840 &chip) : Peripheral(chip, Nrf52840::GpioBase, -1) {}

    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void reset() override;

    // What the pad pushes onto its net.
    PadDrive drive(unsigned pad) const;
    // The level an external net holds on the pad.
    void set_external(unsigned pad, bool high);
    // Level the pad's input buffer sees (own drive and external circuit).
    bool level(unsigned pad) const;

    // A peripheral (serial, PWM) takes over the pad's output: it drives
    // `value` (open-drain style when `open_drain`), or releases when empty.
    struct Override {
        bool high;
        bool open_drain;
    };
    void set_override(unsigned pad, std::optional<Override> override_drive);
    // GPIOTE's output tasks are one such owner of a pad's output level.
    void set_task_output(unsigned pad, std::optional<bool> high);

    void attach_gpiote(Gpiote *gpiote) { gpiote_ = gpiote; }
    std::uint32_t pin_cnf(unsigned pad) const { return pin_cnf_[pad]; }
    bool is_output(unsigned pad) const { return pin_cnf_[pad] & 1u; }
    bool input_connected(unsigned pad) const { return !(pin_cnf_[pad] & 2u); }
    // The pull selected in PIN_CNF: 0 none, 1 down, 3 up.
    unsigned pull(unsigned pad) const { return (pin_cnf_[pad] >> 2) & 3u; }

private:
    static constexpr unsigned Ports = 2;
    static unsigned pins_in_port(unsigned port) { return port == 0 ? 32 : 16; }

    bool out_level(unsigned pad) const;
    void changed(unsigned pad);
    void refresh_input(unsigned pad);
    bool compute_level(unsigned pad) const;

    std::array<std::uint32_t, Ports> out_{};
    std::array<std::uint32_t, Ports> dir_{};
    std::array<std::uint32_t, Ports> latch_{};
    std::array<std::uint32_t, Ports> detectmode_{};
    std::array<std::uint32_t, Nrf52840::PadCount> pin_cnf_{};
    std::array<bool, Nrf52840::PadCount> external_{};
    std::array<bool, Nrf52840::PadCount> external_valid_{};
    std::array<bool, Nrf52840::PadCount> last_level_{};
    std::array<std::optional<Override>, Nrf52840::PadCount> override_{};
    std::array<std::optional<bool>, Nrf52840::PadCount> task_output_{};
    std::array<bool, Nrf52840::PadCount> sense_match_{};
    bool detect_{false};
    Gpiote *gpiote_{nullptr};
};

// GPIOTE: edge events and output tasks on pads, plus the PORT event raised by
// the pins' SENSE configuration.
class Gpiote final : public Peripheral {
public:
    static constexpr int Id = 6;
    Gpiote(Nrf52840 &chip, Gpio &gpio) : Peripheral(chip, Nrf52840::PeripheralBase + Id * 0x1000, Id), gpio_(gpio) {
        gpio_.attach_gpiote(this);
    }

    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    void reset() override;

    // Called by Gpio when a pad's input level changed, and when the aggregated
    // DETECT signal rises.
    void pad_changed(unsigned pad, bool level);
    void detect_rising();

private:
    static constexpr unsigned Channels = 8;
    void update_irq();
    void apply_task(unsigned channel, int kind); // 0 out, 1 set, 2 clr
    void reconfigure(unsigned channel);

    Gpio &gpio_;
    std::array<std::uint32_t, Channels> config_{};
    std::array<bool, Channels> in_event_{};
    bool port_event_{false};
    std::uint32_t inten_{0};
    std::array<bool, Channels> out_state_{};
};

} // namespace nrf52emu
