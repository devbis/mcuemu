#pragma once

// CC2530 analog blocks: ADC (SWRU191F Chapter 12) with the random number
// generator that shares ADCCON1 (Chapter 13), the analog comparator
// (Chapter 19) and the operational amplifier (Chapter 18).

#include "cc2530_soc.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace mcs51emu::cc2530 {

class Adc final : public Peripheral {
public:
    explicit Adc(Bus &bus);
    const char *name() const override { return "ADC"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;

    // ---- host ----
    void set_temperature_celsius(double celsius);
    void set_vdd_mv(std::uint32_t millivolts);
    // Seeds the deterministic noise source (ADC conversion noise, if any).
    void set_noise_seed(std::uint64_t seed);

    // ---- peripherals ----
    // Timer 1 channel 0 compare event: starts a conversion sequence when
    // ADCCON1.STSEL = 10 (SWRU191F 12.2.4). Called by Timer1.
    void timer1_ch0_event();

private:
    friend class Io;
    friend class AnalogComparator;
    friend class OpAmp;
    friend struct IoAnalogUsartTestAccess; // white-box test access

    struct Conversion {
        std::uint8_t channel;    // SCH/ECH coding
        std::uint8_t reference;  // SREF/EREF coding
        std::uint8_t decimation; // SDIV/EDIV coding
        bool single;             // triggered by ADCCON3
    };

    // Rising edge on P2_0 (ADCCON1.STSEL = 00 trigger), called by Io.
    void p2_0_rising();
    void request_sequence();
    void start_sequence(std::uint64_t at);
    // Starts the next conversion of the running sequence at hf time `at`,
    // or ends the sequence.
    void sequence_step(std::uint64_t at);
    void begin(Conversion conversion, std::uint64_t at);
    void complete();
    // Starts whatever waits for an idle ADC (single conversion, requested or
    // full-speed sequence) at hf time `at`.
    void start_pending(std::uint64_t at);
    std::uint64_t conversion_cycles(std::uint8_t decimation) const;
    // Input and reference voltages in units of 1/2048 uV.
    std::int64_t input_value(std::uint8_t channel, Reg selected_by);
    std::int64_t reference_value(std::uint8_t reference) const;
    void set_st_bit();
    void rng_shift(bool in);
    void rng_clock();
    void rng_sync();
    void trigger_dma(unsigned trigger);

    double temperature_c_{25.0};
    std::uint32_t vdd_mv_{3000};
    bool noise_{};
    std::uint64_t noise_state_{};
    std::uint16_t lfsr_{0xffff};

    std::optional<Conversion> current_;
    std::uint64_t done_hf_{Never};
    bool sequence_active_{};
    bool sequence_requested_{};
    Conversion sequence_{};       // configuration latched at the sequence start
    std::uint8_t sequence_next_{}; // next channel to convert
    std::uint8_t sch_written_{};   // ADCCON2.SCH as last written by software
    std::optional<std::uint8_t> single_pending_;
    // STSEL = 01 found nothing to convert (every channel of the sequence
    // disabled in APCFG): wait for the next ADCCON1/ADCCON2 write.
    bool full_speed_stalled_{};
    std::array<std::uint64_t, 32> dma_triggers_{};
};

class AnalogComparator final : public Peripheral {
public:
    explicit AnalogComparator(Bus &bus);
    const char *name() const override { return "CMP"; }
    void reset(ResetCause cause) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    // Pad voltages changed (called by Io).
    void inputs_changed();
    bool output() const;

private:
    friend struct IoAnalogUsartTestAccess; // white-box test access
    bool compute() const;
};

class OpAmp final : public Peripheral {
public:
    explicit OpAmp(Bus &bus);
    const char *name() const override { return "OPAMP"; }
    void reset(ResetCause cause) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;
    // Output voltage on P0_2 while enabled.
    std::optional<std::uint32_t> output_mv() const;

private:
    friend struct IoAnalogUsartTestAccess; // white-box test access

    std::uint64_t calibration_done_hf_{Never};
};

} // namespace mcs51emu::cc2530
