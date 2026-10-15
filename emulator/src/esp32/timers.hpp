#pragma once

#include "blocks.hpp"

#include <cstdint>
#include <functional>

namespace esp32emu {

using CycleSource = std::function<std::uint64_t()>;

// SYSTIMER: two 52-bit counters at 16 MHz (the C6's XTAL/2.5; ticks are derived
// from CPU cycles). Reading a counter is a latch: the driver sets
// UNITn_OP.UPDATE, waits for VALUE_VALID and then reads VALUE_HI/LO. The unit
// runs when its WORK_EN bit is set (the reset value of CONF has unit 0 running,
// the ROM leaves it so). Comparators: TARGETn_HI/LO stage a 52-bit target,
// COMPn_LOAD applies it, and with CONF.TARGETn_WORK_EN the comparator fires once
// the selected unit's count has reached it (inferred: also when the target is
// already behind, as ESP-IDF's "alarm miss" comment suggests). One-shot by
// default; in period mode (TARGETn_CONF.PERIOD_MODE) it re-arms by PERIOD ticks.
// A fire sets INT_RAW; the source line is INT_RAW & INT_ENA.
class Systimer final : public StatusBlock {
public:
    static constexpr std::uint64_t TickHz = 16000000;
    Systimer(std::uint32_t base, std::uint32_t cpu_hz, unsigned first_source, CycleSource cycles, IrqSink irq);

    std::uint32_t read(std::uint32_t offset, unsigned size) override;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    std::uint64_t ticks() const;
    std::uint64_t counter(unsigned unit) const;
    std::uint64_t cycle_of_tick(std::uint64_t tick) const;
    void latch(unsigned unit);
    void update_lines();

    IrqSink irq_;
    unsigned first_source_;
    struct Target {
        std::uint64_t value{0};
        bool armed{false};
    } targets_[3];
    std::uint32_t int_raw_{0}, int_ena_{0};

    std::uint32_t cpu_hz_;
    CycleSource cycles_;
    std::uint64_t latched_[2]{};
    // Tick count at the moment the unit last started or was loaded: a counter
    // value is its offset plus the ticks since.
    std::uint64_t origin_[2]{};
    std::uint64_t load_value_[2]{};
};

// LP_TIMER: the 48-bit RTC "main timer" on the slow clock (RC_SLOW, nominally
// 136 kHz, the same figure the TIMG calibration reports). UPDATE.MAIN_TIMER_UPDATE
// latches the count into MAIN_BUF0; the alarm registers and the interrupt are
// not modelled yet.
class LpTimer final : public StatusBlock {
public:
    static constexpr std::uint64_t SlowHz = 136000;
    LpTimer(std::uint32_t base, std::uint32_t cpu_hz, CycleSource cycles);

    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;

private:
    std::uint32_t cpu_hz_;
    CycleSource cycles_;
};

// A register that reads the time in microseconds since power-on (offset 0), the
// rest of the window reads back.
class MicrosecondCounter final : public StatusBlock {
public:
    MicrosecondCounter(std::uint32_t base, std::uint32_t cpu_hz, CycleSource cycles)
        : StatusBlock(base, 0x1000), cpu_hz_(cpu_hz), cycles_(std::move(cycles)) {}
    std::uint32_t read(std::uint32_t offset, unsigned size) override {
        return (offset & ~3u) == 0 ? static_cast<std::uint32_t>(cycles_() * 1000000ull / cpu_hz_)
                                   : StatusBlock::read(offset, size);
    }

private:
    std::uint32_t cpu_hz_;
    CycleSource cycles_;
};

} // namespace esp32emu
