#pragma once

// CC2530 I/O controller (SWRU191F Chapter 7): ports P0-P2, direction,
// function select, pull configuration, peripheral pin mapping
// (PERCFG/APCFG/PxSEL/P2SEL priorities/PMUX/OBSSEL), port interrupts
// (PxIEN/PxIFG/PICTL, IRCON.P0IF, IRCON2.P1IF/P2IF) and the pad interface
// to the host (sim::Pin).

#include "cc2530_soc.hpp"

#include <array>
#include <functional>
#include <optional>

namespace mcs51emu::cc2530 {

// What the chip does to a pad.
using PinDrive = mcs51emu::Cc2530PinDrive;

class Io final : public Peripheral {
public:
    using Listener =
        std::function<void(unsigned port, unsigned pin, PinDrive drive, std::uint64_t cycle)>;

    explicit Io(Bus &bus);
    const char *name() const override { return "IOC"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    void cpu_flags_written() override;
    void power_mode_changed(PowerMode from, PowerMode to) override;

    // ---- host ----
    // External digital level applied to a pad (nullopt: nothing drives it;
    // the pad then follows its pull, or reads as the chip's own output).
    void set_input(unsigned port, unsigned pin, std::optional<bool> level);
    // External analog voltage on a pad (used by the ADC, comparator and
    // op-amp); nullopt removes it.
    void set_analog_mv(unsigned port, unsigned pin, std::optional<std::uint32_t> millivolts);
    std::optional<std::uint32_t> analog_mv(unsigned port, unsigned pin) const;
    PinDrive drive(unsigned port, unsigned pin) const;
    // Logic level the input buffer sees (external level, else own output,
    // else pull; floating without pull reads 1).
    bool level(unsigned port, unsigned pin) const;
    // Called whenever drive(port, pin) changes.
    void set_listener(Listener listener);

    // ---- peripherals ----
    // A peripheral output that may be routed to a pad changed (timer
    // compare outputs, USART TX/clock, observation outputs).
    void peripheral_outputs_changed();
    // The analog comparator output changed (SWRU191F 19: feeds the P0_5
    // interrupt edge detector).
    void comparator_changed(bool output);

private:
    friend class Adc;
    friend class AnalogComparator;
    friend class OpAmp;
    friend class Usart;
    friend struct IoAnalogUsartTestAccess; // white-box test access

    // PMUX.CKOEN drives a 32 kHz square wave: one event per half period.
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void clock_changed() override;

    PinDrive compute_drive(unsigned port, unsigned pin) const;
    // Peripheral function owning a pad with PxSEL = 1: nullopt when no
    // function is mapped there; otherwise the output level, or nullopt
    // inside when the winning function is an input.
    std::optional<std::optional<bool>> peripheral_pad(unsigned port, unsigned pin) const;
    bool level_of(unsigned port, unsigned pin, PinDrive drive) const;
    bool cko_level() const;
    // Effective pad voltage seen by the analog blocks (ADC, comparator).
    std::uint32_t pad_mv(unsigned port, unsigned pin) const;
    // Level of pad `role` (0 RX/MI, 1 TX/MO, 2 CT/SS, 3 RT/C) of USART
    // `index` at its PERCFG location.
    bool usart_pad_level(unsigned index, unsigned role) const;
    // Recomputes every pad until the configuration settles; `at` is the
    // cycle reported to the listener for drive changes.
    void update();
    void update(std::uint64_t at);
    void refresh(bool notify, std::uint64_t at);
    void port_edges(unsigned port, std::uint8_t rising, std::uint8_t falling);
    void assert_cpu_flags();
    void trigger_dma(unsigned trigger);

    std::array<std::array<std::optional<bool>, 8>, 3> external_{};
    std::array<std::array<std::optional<std::uint32_t>, 8>, 3> analog_{};
    std::array<std::array<PinDrive, 8>, 3> drive_{};
    std::array<std::uint8_t, 3> last_level_{};
    // Levels at the port interrupt edge detectors (P0_5 follows the
    // comparator output while CMPCTL.EN = 1, SWRU191F 19.1).
    std::array<std::uint8_t, 3> detector_{};
    bool comparator_output_{};
    bool updating_{};
    bool update_again_{};
    std::uint64_t cko_edge_lf_{Never};
    std::array<std::uint64_t, 32> dma_triggers_{};
    Listener listener_;
};

} // namespace mcs51emu::cc2530
