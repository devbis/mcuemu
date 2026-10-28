#pragma once

// Shared helpers for the CC2530F256 peripheral tests: hand-encoded MCS-51
// programs in flash, register access, and the fast/step/trace equivalence
// runner. Every peripheral test executes real machine code on the Unicorn
// core or drives the documented host API; expected values come from
// SWRU191F, not from the implementation.

#include "mcs51emu/cc2530f256.hpp"

#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace cc2530test {

using mcs51emu::Cc2530F256;
using mcs51emu::Cc2530PowerMode;
using mcs51emu::Cc2530RunResult;

inline unsigned failures = 0;

#define CHECK(condition)                                                                       \
    do {                                                                                       \
        if (!(condition)) {                                                                    \
            std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition << '\n'; \
            ++cc2530test::failures;                                                            \
        }                                                                                      \
    } while (false)

#define CHECK_EQ(actual, expected)                                                             \
    do {                                                                                       \
        const auto check_actual_ = (actual);                                                   \
        const auto check_expected_ = (expected);                                               \
        if (!(check_actual_ == check_expected_)) {                                             \
            std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #actual " == " #expected \
                      << " (got " << +check_actual_ << ", want " << +check_expected_ << ")\n";  \
            ++cc2530test::failures;                                                            \
        }                                                                                      \
    } while (false)

// Physical flash image under construction.
struct Flash {
    std::vector<std::uint8_t> bytes = std::vector<std::uint8_t>(Cc2530F256::FlashSize, 0xff);
    std::uint32_t cursor = 0;

    Flash &at(std::uint32_t physical) {
        cursor = physical;
        return *this;
    }
    Flash &op(std::initializer_list<std::uint8_t> code) {
        for (const std::uint8_t byte : code)
            bytes.at(cursor++) = byte;
        return *this;
    }
    Flash &op(const std::vector<std::uint8_t> &code) {
        for (const std::uint8_t byte : code)
            bytes.at(cursor++) = byte;
        return *this;
    }
    std::uint16_t here() const { return static_cast<std::uint16_t>(cursor); }
};

inline void load(Cc2530F256 &device, const Flash &flash) { device.load_flash(0, flash.bytes); }

inline std::uint8_t hi(std::uint16_t value) { return static_cast<std::uint8_t>(value >> 8); }
inline std::uint8_t lo(std::uint16_t value) { return static_cast<std::uint8_t>(value); }

// Common encodings.
inline std::vector<std::uint8_t> mov_dir_imm(std::uint8_t dir, std::uint8_t value) {
    return {0x75, dir, value};
}
inline std::vector<std::uint8_t> mov_dptr(std::uint16_t address) {
    return {0x90, hi(address), lo(address)};
}
// MOV DPTR,#address; MOV A,#value; MOVX @DPTR,A
inline std::vector<std::uint8_t> movx_write(std::uint16_t address, std::uint8_t value) {
    return {0x90, hi(address), lo(address), 0x74, value, 0xf0};
}
// MOV DPTR,#address; MOVX A,@DPTR; MOV dir,A
inline std::vector<std::uint8_t> movx_read_to(std::uint16_t address, std::uint8_t dir) {
    return {0x90, hi(address), lo(address), 0xe0, 0xf5, dir};
}

// Everything a program can observe: CPU registers, counters, SFRs (peek),
// all SRAM, XREG 0x6000-0x63ff (peek).
inline std::vector<std::uint8_t> snapshot(const Cc2530F256 &device) {
    const auto regs = device.registers();
    std::vector<std::uint8_t> state{lo(regs.pc), hi(regs.pc), regs.a, regs.b, regs.psw,
                                    regs.sp, regs.dpl, regs.dph, regs.fmap, regs.memctr,
                                    regs.mpage, regs.dpl1, regs.dph1, regs.dps,
                                    regs.interrupt_levels, regs.interrupt_inhibit,
                                    lo(regs.posted_vector), hi(regs.posted_vector),
                                    regs.posted_level,
                                    static_cast<std::uint8_t>(regs.posted_source)};
    for (unsigned shift = 0; shift < 64; shift += 8) {
        state.push_back(static_cast<std::uint8_t>(regs.cycles >> shift));
        state.push_back(static_cast<std::uint8_t>(regs.instructions >> shift));
    }
    for (unsigned address = 0x80; address < 0x100; ++address)
        state.push_back(device.read_sfr(static_cast<std::uint8_t>(address)));
    for (unsigned address = 0; address < 0x2000; ++address)
        state.push_back(device.peek_xdata(static_cast<std::uint16_t>(address)));
    for (unsigned address = 0x6000; address < 0x6400; ++address)
        state.push_back(device.peek_xdata(static_cast<std::uint16_t>(address)));
    state.push_back(static_cast<std::uint8_t>(device.sleeping()));
    state.push_back(static_cast<std::uint8_t>(device.power_mode()));
    return state;
}

enum class Mode { Fast, Step, Trace };
inline const char *mode_name(Mode mode) {
    return mode == Mode::Fast ? "fast" : mode == Mode::Step ? "step" : "trace";
}

// Runs until the cycle counter reaches `cycle_limit` (or `max_instructions`)
// in the given mode. Step mode issues run(1) calls; trace mode installs the
// per-instruction hook. Returns the last run result.
inline Cc2530RunResult run_mode(Cc2530F256 &device, Mode mode, std::uint64_t cycle_limit,
                                std::size_t max_instructions = 10'000'000) {
    std::ostringstream sink;
    Cc2530RunResult result;
    if (mode == Mode::Trace)
        device.set_trace(&sink);
    if (mode == Mode::Step) {
        std::size_t executed = 0;
        for (;;) {
            result = device.run(1, cycle_limit);
            executed += result.instructions;
            if (!result.error.empty() || result.break_hit ||
                device.registers().cycles >= cycle_limit || executed >= max_instructions)
                break;
            if (result.instructions == 0 && result.sleeping)
                break; // asleep with nothing to wake it before the limit
        }
    } else {
        result = device.run(max_instructions, cycle_limit);
    }
    device.set_trace(nullptr);
    return result;
}

// Runs a fresh device per mode (prepared by `setup`) to `cycle_limit` and
// checks that the three final snapshots are identical. Returns the fast
// device's snapshot owner through `check` callback for further assertions.
template <class Setup, class Check>
void for_each_mode(const Flash &flash, std::uint64_t cycle_limit, Setup setup, Check check,
                   const std::string &name) {
    std::vector<std::uint8_t> reference;
    for (const Mode mode : {Mode::Fast, Mode::Step, Mode::Trace}) {
        Cc2530F256 device;
        load(device, flash);
        setup(device);
        const Cc2530RunResult result = run_mode(device, mode, cycle_limit);
        if (!result.error.empty()) {
            std::cerr << name << " [" << mode_name(mode) << "]: run error: " << result.error
                      << '\n';
            ++failures;
        }
        check(device, mode);
        const auto state = snapshot(device);
        if (mode == Mode::Fast) {
            reference = state;
        } else if (state != reference) {
            std::size_t i = 0;
            while (i < state.size() && state[i] == reference[i])
                ++i;
            std::cerr << name << ": " << mode_name(mode) << " differs from fast at snapshot byte "
                      << i << '\n';
            ++failures;
        }
    }
}

inline int finish(const char *suite) {
    if (failures != 0) {
        std::cerr << failures << " check(s) failed\n";
        return 1;
    }
    std::cout << suite << " tests passed\n";
    return 0;
}

} // namespace cc2530test
