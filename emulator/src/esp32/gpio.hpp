#pragma once

#include "blocks.hpp"
#include "esp32emu/esp32.hpp"

#include <array>
#include <cstdint>
#include <functional>

namespace esp32emu {

// GPIO (0x60091000) and the IO_MUX pad registers (0x60090000), for pads used as
// plain GPIO. Written from the register layout in the SVDs and esp-hal's
// driver: OUT/ENABLE with their W1TS/W1TC aliases, IN, per-pin config (interrupt
// type and enable), the interrupt status with W1TC, and FUNCn_OUT_SEL_CFG, whose
// value 0x80 ("simple GPIO output") means the pad follows OUT; the pad is driven
// only then. IO_MUX supplies the pull-ups and pull-downs. Pins routed to other
// peripherals through the GPIO matrix are not modelled (their outputs stay
// high-impedance, their inputs read the pad). An input reads the level the board
// gives the pad (set_input), high until the board says otherwise (an idle pad with
// a pull-up, like a released active-low button).
// Interrupts (PINn.INT_TYPE: 1 rising, 2 falling, 3 any edge, 4 low level, 5 high
// level; PINn.INT_ENA bit 0 for the CPU) raise one line, `source`.
class IoMux final : public StatusBlock {
public:
    IoMux(std::uint32_t base, unsigned pins) : StatusBlock(base, 0x100), pins_(pins) {}
    bool pull_up(unsigned pad) const;
    bool pull_down(unsigned pad) const;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;
    std::function<void()> changed; // a pull resistor may have been switched

private:
    unsigned pins_;
};

class Gpio final : public StatusBlock {
public:
    static constexpr unsigned MaxPins = 32;
    Gpio(std::uint32_t base, unsigned pins, unsigned source, IoMux &mux, IrqSink irq);

    std::uint32_t read(std::uint32_t offset, unsigned size) override;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;
    void reset() override;

    unsigned pins() const { return pins_; }
    PadDrive drive(unsigned pad) const;
    void set_input(unsigned pad, bool high);
    // Called when the pad's drive may have changed (register writes).
    std::function<void(unsigned pad)> on_drive_changed;

private:
    std::uint32_t pin_config(unsigned pad) const;
    void evaluate_levels();
    void update_irq();

    unsigned pins_, source_;
    IoMux &mux_;
    IrqSink irq_;
    std::uint32_t out_{0}, enable_{0}, in_{0xffffffffu}, status_{0};
};

} // namespace esp32emu
