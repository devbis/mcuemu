// Shared helpers for the EMU-04A-R2 CC2530 interrupt-scheduler tests.
//
// Programs are hand-encoded. Each instruction carries its cycle count under
// the documented CC2530 timing contract (contract_cycles from
// cc2530_table_2_3.hpp, transcribed from SWRU191F Table 2-3, not read from
// the emulator), so expected boundary cycles are computed independently of
// the model under test.
#pragma once

#include "cc2530_table_2_3.hpp"
#include "mcs51emu/cc2530f256.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

namespace r2 {

using mcs51emu::Cc2530F256;
using mcs51emu::Cc2530RunResult;

inline unsigned failures = 0;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::cerr << __FILE__ << ':' << __LINE__                           \
                      << ": check failed: " #condition << '\n';                \
            ++::r2::failures;                                                  \
        }                                                                      \
    } while (false)

#define CHECK_IN(condition, context)                                           \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::cerr << __FILE__ << ':' << __LINE__                           \
                      << ": check failed: " #condition << " [" << (context)    \
                      << "]\n";                                                \
            ++::r2::failures;                                                  \
        }                                                                      \
    } while (false)

constexpr std::uint8_t SP = 0x81, PCON = 0x87, TCON = 0x88, S0CON = 0x98, IEN2 = 0x9a,
                       S1CON = 0x9b, FMAP = 0x9f, IEN0 = 0xa8, IP0 = 0xa9, IEN1 = 0xb8,
                       IP1 = 0xb9, SLEEPCMD = 0xbe, IRCON = 0xc0, IRCON2 = 0xe8,
                       EA_BIT = 0xaf;

// SWRU191F Table 2-5 source numbers (vector = 0x03 + 8 * n).
constexpr unsigned SrcEnc = 4, SrcDma = 8, SrcT1 = 9, SrcT3 = 11, SrcT4 = 12, SrcP0 = 13,
                   SrcRf = 16;
constexpr std::uint16_t vector_of(unsigned source) {
    return static_cast<std::uint16_t>(0x03 + 8 * source);
}

inline std::uint8_t hi(std::uint16_t v) { return static_cast<std::uint8_t>(v >> 8); }
inline std::uint8_t lo(std::uint16_t v) { return static_cast<std::uint8_t>(v); }

using swru191f::AcallLow;
using swru191f::AjmpLow;
using swru191f::contract_cycles;
using swru191f::EntryCycles;
using swru191f::Table23Row;
using swru191f::table_2_3;
using swru191f::table_2_3_covers;
using swru191f::table_2_3_row;

struct Ins {
    std::vector<std::uint8_t> bytes;
    unsigned cycles; // contract cycles (for idle_pad: 1 = violate)
};
inline Ins op(std::vector<std::uint8_t> bytes) {
    const unsigned cycles = contract_cycles(bytes.at(0));
    return {std::move(bytes), cycles};
}

inline Ins nop() { return op({0x00}); }
inline Ins ljmp(std::uint16_t a) { return op({0x02, hi(a), lo(a)}); }
inline Ins lcall(std::uint16_t a) { return op({0x12, hi(a), lo(a)}); }
inline Ins ret() { return op({0x22}); }
inline Ins reti() { return op({0x32}); }
inline Ins sjmp_self() { return op({0x80, 0xfe}); }
inline Ins sjmp(std::int8_t rel) { return op({0x80, static_cast<std::uint8_t>(rel)}); }
inline Ins mov_imm(std::uint8_t d, std::uint8_t v) { return op({0x75, d, v}); }
inline Ins mov_rn_imm(unsigned n, std::uint8_t v) {
    return op({static_cast<std::uint8_t>(0x78 + n), v});
}
inline Ins mov_dir_dir(std::uint8_t dst, std::uint8_t src) { return op({0x85, src, dst}); }
inline Ins setb(std::uint8_t bit) { return op({0xd2, bit}); }
inline Ins clr(std::uint8_t bit) { return op({0xc2, bit}); }
inline Ins orl_imm(std::uint8_t d, std::uint8_t v) { return op({0x43, d, v}); }
inline Ins anl_imm(std::uint8_t d, std::uint8_t v) { return op({0x53, d, v}); }
inline Ins anl_dir_a(std::uint8_t d) { return op({0x52, d}); }
inline Ins inc_dir(std::uint8_t d) { return op({0x05, d}); }
inline Ins mul_ab() { return op({0xa4}); }
inline Ins mov_a_dir(std::uint8_t d) { return op({0xe5, d}); }
inline Ins mov_at_r1_a() { return op({0xf7}); }
inline Ins mov_at_r1_imm(std::uint8_t v) { return op({0x77, v}); }
inline Ins inc_r1() { return op({0x09}); }
inline Ins idle_write() { return op({0x75, PCON, 0x01}); } // MOV PCON,#1
// Pseudo-instruction for place(): NOP padding so that the instruction after
// the 3-byte idle_write() that follows starts off a 4-byte boundary
// (SWRU191F 4.2), or on one when `violate`.
inline Ins idle_pad(bool violate = false) { return {{}, violate ? 1u : 0u}; }

// Logs the ISR's return address (high, low: IDATA[SP], IDATA[SP-1]) at @R1:
// MOV R0,SP / MOV A,@R0 / MOV @R1,A / INC R1 / DEC R0 / MOV A,@R0 /
// MOV @R1,A / INC R1. Uses A, R0 and R1 of register bank 0.
inline std::vector<Ins> log_return() {
    return {op({0xa8, SP}), op({0xe6}), op({0xf7}), op({0x09}),
            op({0x18}),     op({0xe6}), op({0xf7}), op({0x09})};
}
// MOV A,direct / MOV @R1,A / INC R1.
inline std::vector<Ins> log_direct(std::uint8_t d) {
    return {mov_a_dir(d), mov_at_r1_a(), inc_r1()};
}
// MOV @R1,#v / INC R1.
inline std::vector<Ins> log_marker(std::uint8_t v) { return {mov_at_r1_imm(v), inc_r1()}; }

inline std::vector<Ins> cat(std::initializer_list<std::vector<Ins>> parts) {
    std::vector<Ins> all;
    for (const auto &part : parts)
        all.insert(all.end(), part.begin(), part.end());
    return all;
}
inline std::vector<Ins> repeat(const Ins &ins, unsigned n) { return std::vector<Ins>(n, ins); }

// Physical flash image under construction.
struct Flash {
    std::vector<std::uint8_t> bytes =
        std::vector<std::uint8_t>(Cc2530F256::FlashSize, 0xff);
    std::uint32_t cursor = 0;
};

// One placed straight-line sequence: the logical address of every
// instruction and the cycle offset at its start (relative to the first).
struct Placed {
    std::vector<std::uint16_t> pc;
    std::vector<unsigned> offset;
    std::vector<std::vector<std::uint8_t>> code;
    std::uint16_t end{}; // address after the last instruction
    unsigned total{};    // cycles of the whole sequence
    std::uint16_t at(std::size_t i) const { return pc.at(i); }
    std::size_t index_of(std::uint16_t address) const {
        for (std::size_t i = 0; i < pc.size(); ++i)
            if (pc[i] == address)
                return i;
        throw std::logic_error("address not in sequence");
    }
    // Index of the first instruction with exactly these bytes at or after `from`.
    std::size_t find(const std::vector<std::uint8_t> &bytes, std::size_t from = 0) const {
        for (std::size_t i = from; i < code.size(); ++i)
            if (code[i] == bytes)
                return i;
        throw std::logic_error("instruction not in sequence");
    }
};

// Places `code` at physical `physical` (logical `logical` for upper-bank code).
inline Placed place(Flash &flash, std::uint32_t physical, const std::vector<Ins> &code,
                    std::optional<std::uint16_t> logical = std::nullopt) {
    Placed placed;
    std::uint32_t address = physical;
    std::uint16_t pc = logical ? *logical : static_cast<std::uint16_t>(physical);
    unsigned cycles = 0;
    const auto emit = [&](const Ins &ins) {
        placed.pc.push_back(pc);
        placed.offset.push_back(cycles);
        placed.code.push_back(ins.bytes);
        for (const std::uint8_t byte : ins.bytes)
            flash.bytes.at(address++) = byte;
        pc = static_cast<std::uint16_t>(pc + ins.bytes.size());
        cycles += ins.cycles;
    };
    for (const Ins &ins : code) {
        if (!ins.bytes.empty()) {
            emit(ins);
            continue;
        }
        const bool violate = ins.cycles != 0;
        while ((((pc + 3) & 3u) == 0) != violate)
            emit(nop());
    }
    placed.end = pc;
    placed.total = cycles;
    return placed;
}

inline void load(Cc2530F256 &device, const Flash &flash) {
    device.load_flash(0, flash.bytes);
}

// Interrupt programs append bytes at @R1 (bank 0 R1 = IDATA 0x01) from 0x40.
inline std::vector<std::uint8_t> irq_log(const Cc2530F256 &device) {
    std::vector<std::uint8_t> log;
    for (unsigned address = 0x40; address < device.read_idata(0x01) && address < 0x100;
         ++address)
        log.push_back(device.read_idata(static_cast<std::uint8_t>(address)));
    return log;
}

enum class Mode { Fast, Trace, Step };
inline const char *mode_name(Mode mode) {
    return mode == Mode::Fast ? "fast" : mode == Mode::Trace ? "trace" : "step";
}
constexpr Mode AllModes[] = {Mode::Fast, Mode::Trace, Mode::Step};

// Runs to the first instruction boundary at or past `cycle_limit` (or until
// sleeping with nothing left to wake the CPU, or an error) in `mode`. Step
// mode issues run(1, cycle_limit) calls.
inline Cc2530RunResult run_mode(Cc2530F256 &device, Mode mode, std::uint64_t cycle_limit,
                                std::ostringstream &sink, std::size_t max_steps = 100000) {
    device.set_trace(mode == Mode::Trace ? &sink : nullptr);
    if (mode != Mode::Step)
        return device.run(max_steps, cycle_limit);
    Cc2530RunResult total;
    for (std::size_t i = 0; i < max_steps; ++i) {
        const Cc2530RunResult r = device.run(1, cycle_limit);
        total.instructions += r.instructions;
        total.pc = r.pc;
        total.cycles = r.cycles;
        total.fmap = r.fmap;
        total.sleeping = r.sleeping;
        total.error = r.error;
        if (!r.error.empty() || r.cycles >= cycle_limit || (r.instructions == 0 && r.sleeping))
            break;
    }
    return total;
}

// Runs until the CPU has just vectored to `vector` (the entry is done, the
// first ISR instruction is not), in `mode`. Fast and trace use a breakpoint.
inline Cc2530RunResult run_to_vector(Cc2530F256 &device, Mode mode, std::uint16_t vector,
                                     std::ostringstream &sink, std::size_t max_steps = 10000) {
    device.set_trace(mode == Mode::Trace ? &sink : nullptr);
    if (mode != Mode::Step) {
        device.set_breakpoint(vector);
        const Cc2530RunResult r = device.run(max_steps);
        device.set_breakpoint(std::nullopt);
        return r;
    }
    Cc2530RunResult total;
    for (std::size_t i = 0; i < max_steps; ++i) {
        const Cc2530RunResult r = device.run(1);
        total.instructions += r.instructions;
        total.pc = r.pc;
        total.cycles = r.cycles;
        total.error = r.error;
        total.sleeping = r.sleeping;
        if (!r.error.empty() || (r.pc == vector && device.in_interrupt()))
            break;
        if (r.instructions == 0 && r.sleeping)
            break;
    }
    total.break_hit = total.pc == vector;
    return total;
}

} // namespace r2
