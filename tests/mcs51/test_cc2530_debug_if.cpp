// CC2530F256 two-wire debug interface tests (SWRU191F Chapter 3): debug-mode
// entry, the DC/DD protocol on P2.2/P2.1, every Table 3-1 command, the
// configuration (Table 3-2) and status (Table 3-3) bytes, the four hardware
// breakpoints (3.3.3), DBGLOCK (3.4.1, Table 3-5), DBGDATA with DMA trigger
// DBG_BW (Table 8-1), CHIP_ERASE timing (2.2.3: 20 ms) and CC-Debugger-style
// flash programming through all of it.
//
// Programs are hand-encoded 8051 machine code executed by the real core.
// Expected values come from SWRU191F. Where the documents give no number the
// test relies on the model-defined rules of docs/cc2530.md "Debug
// interface": the command-level host port delivers one byte every 128
// cycles; a refused command answers its status byte in every output
// position; DEBUG_INSTR takes no virtual time and is not counted as a
// firmware instruction; DBGLOCK is latched at reset. No test depends on the
// cycle count of an instruction: execution is compared by instruction
// counts or against a reference run.

#include "cc2530_debug.hpp"
#include "cc2530_periph_support.hpp"
#include "mcs51emu/cc2530_debug_programmer.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using namespace cc2530test;

namespace {

using Bytes = std::vector<std::uint8_t>;
using mcs51emu::Cc2530DebugIfLink;
using mcs51emu::Cc2530DebugIfResponse;
using mcs51emu::Cc2530DebugProgramOptions;
using mcs51emu::Cc2530DebugProgramResult;
using mcs51emu::Cc2530FlashImage;
using mcs51emu::Cc2530PinDrive;

// Table 3-1 command codes (don't-care bits 0).
constexpr std::uint8_t CHIP_ERASE = 0x10;
constexpr std::uint8_t WR_CONFIG = 0x18;
constexpr std::uint8_t RD_CONFIG = 0x20;
constexpr std::uint8_t GET_PC = 0x28;
constexpr std::uint8_t READ_STATUS = 0x30;
constexpr std::uint8_t SET_HW_BRKPNT = 0x38;
constexpr std::uint8_t HALT = 0x40;
constexpr std::uint8_t RESUME = 0x48;
constexpr std::uint8_t DEBUG_INSTR = 0x50;
constexpr std::uint8_t STEP_INSTR = 0x58;
constexpr std::uint8_t GET_BM = 0x60;
constexpr std::uint8_t GET_CHIP_ID = 0x68;
constexpr std::uint8_t BURST_WRITE = 0x80;

// Table 3-3.
constexpr std::uint8_t ChipEraseBusy = 0x80;
constexpr std::uint8_t PconIdle = 0x40;
constexpr std::uint8_t CpuHalted = 0x20;
constexpr std::uint8_t PmActive = 0x10;
constexpr std::uint8_t HaltStatus = 0x08;
constexpr std::uint8_t DebugLocked = 0x04;
constexpr std::uint8_t OscillatorStable = 0x02;
constexpr std::uint8_t StackOverflow = 0x01;
// Debug mode entered: CPU halted, system clock oscillator stable.
constexpr std::uint8_t Halted = CpuHalted | OscillatorStable;

// Table 3-2: SOFT_POWER_MODE | TIMER_SUSPEND | DMA_PAUSE after entry.
constexpr std::uint8_t ConfigReset = 0x26;

// Registers.
constexpr std::uint8_t SP = 0x81;
constexpr std::uint8_t PCON = 0x87;
constexpr std::uint8_t SLEEPSTA = 0x9d;
constexpr std::uint8_t FMAP = 0x9f;
constexpr std::uint8_t P2 = 0xa0;
constexpr std::uint8_t IEN0 = 0xa8;
constexpr std::uint8_t SLEEPCMD = 0xbe;
constexpr std::uint8_t DMAIRQ = 0xd1;
constexpr std::uint8_t DMA0CFGL = 0xd4;
constexpr std::uint8_t DMA0CFGH = 0xd5;
constexpr std::uint8_t DMAARM = 0xd6;
constexpr std::uint8_t T1CNTL = 0xe2;
constexpr std::uint8_t T1CNTH = 0xe3;
constexpr std::uint8_t T1CTL = 0xe4;
constexpr std::uint8_t P2DIR = 0xff;
constexpr std::uint16_t CHVER = 0x6249;
constexpr std::uint16_t CHIPID = 0x624a;
constexpr std::uint16_t DBGDATA = 0x6260;

constexpr std::uint64_t HostByte = mcs51emu::cc2530::Debug::HostByteCycles;
constexpr std::uint64_t ChipEraseCycles = 640000; // 2.2.3 / 6: 20 ms at 32 MHz
constexpr std::uint32_t DbglockByte = 0x3ffff;    // Table 3-5: bit 7 of the last byte

// ---- host-port helpers ----

Cc2530DebugIfResponse send(Cc2530F256 &device, const Bytes &command) {
    return device.debug_if_command(command);
}

Bytes accepted(Cc2530F256 &device, const Bytes &command) {
    const Cc2530DebugIfResponse response = device.debug_if_command(command);
    CHECK(response.accepted);
    CHECK_EQ(response.bytes.size(), Cc2530F256::debug_if_output_length(command.front()));
    return response.bytes;
}

// A refused command shifts out the status byte in every output position.
void refused(Cc2530F256 &device, const Bytes &command, std::uint8_t status) {
    const Cc2530DebugIfResponse response = device.debug_if_command(command);
    CHECK(!response.accepted);
    CHECK(response.bytes == Bytes(Cc2530F256::debug_if_output_length(command.front()), status));
}

std::uint8_t status(Cc2530F256 &device) { return accepted(device, {READ_STATUS}).at(0); }

std::uint16_t get_pc(Cc2530F256 &device) {
    const Bytes pc = accepted(device, {GET_PC});
    return static_cast<std::uint16_t>(pc.at(0) << 8 | pc.at(1));
}

std::uint8_t instr(Cc2530F256 &device, const Bytes &code) {
    Bytes command{static_cast<std::uint8_t>(DEBUG_INSTR | code.size())};
    command.insert(command.end(), code.begin(), code.end());
    return accepted(device, command).at(0);
}

std::uint8_t step(Cc2530F256 &device) { return accepted(device, {STEP_INSTR}).at(0); }

void breakpoint(Cc2530F256 &device, unsigned number, bool enable, std::uint8_t bank,
                std::uint16_t address) {
    const auto control = static_cast<std::uint8_t>(number << 4 | (enable ? 0x08u : 0u) | bank);
    CHECK_EQ(accepted(device, {SET_HW_BRKPNT, control, hi(address), lo(address)}).at(0) & CpuHalted,
             CpuHalted);
}

// MOV DPTR,#address; then MOV A,#b; MOVX @DPTR,A; INC DPTR per byte.
void debug_write_xdata(Cc2530F256 &device, std::uint16_t address, const Bytes &bytes) {
    instr(device, {0x90, hi(address), lo(address)});
    for (const std::uint8_t byte : bytes) {
        instr(device, {0x74, byte});
        instr(device, {0xf0});
        instr(device, {0xa3});
    }
}

void debug_write_sfr(Cc2530F256 &device, std::uint8_t sfr, std::uint8_t value) {
    instr(device, {0x75, sfr, value});
}

// Trace mode is switched on for a device's whole life (`traced`): the
// debug commands then run traced as well. Switching one device from a
// traced to an untraced run is avoided: Unicorn keeps a deleted code hook
// until the end of the next emulation, which then runs without the
// fast-path stops.
std::ostringstream trace_sink;
void traced(Cc2530F256 &device, Mode mode) {
    trace_sink.str("");
    if (mode == Mode::Trace)
        device.set_trace(&trace_sink);
}

// Lets virtual time reach `limit`, single-stepping in Mode::Step.
void advance_to(Cc2530F256 &device, std::uint64_t limit, Mode mode = Mode::Fast) {
    while (device.registers().cycles < limit) {
        const Cc2530RunResult result = device.run(mode == Mode::Step ? 1 : SIZE_MAX, limit);
        if (!result.error.empty()) {
            std::cerr << "run error: " << result.error << '\n';
            ++failures;
            break;
        }
    }
}

void advance(Cc2530F256 &device, std::uint64_t cycles, Mode mode = Mode::Fast) {
    advance_to(device, device.registers().cycles + cycles, mode);
}

// Runs until the debug interface halts the CPU (a hardware breakpoint).
Cc2530RunResult run_to_halt(Cc2530F256 &device, Mode mode = Mode::Fast) {
    const std::uint64_t limit = device.registers().cycles + 1'000'000;
    Cc2530RunResult result;
    do {
        result = device.run(mode == Mode::Step ? 1 : SIZE_MAX, limit);
    } while (result.error.empty() && !result.debug_halted && device.registers().cycles < limit);
    CHECK(result.error.empty());
    CHECK(result.debug_halted);
    return result;
}

bool throws_logic(const std::function<void()> &fn) {
    try {
        fn();
    } catch (const std::logic_error &) {
        return true;
    }
    return false;
}

bool throws_invalid(const std::function<void()> &fn) {
    try {
        fn();
    } catch (const std::invalid_argument &) {
        return true;
    }
    return false;
}

// ---- programs ----

// Counter loop: A = rl(A + 3), R7 counts iterations, both mirrored to DATA
// 0x30/0x31.
Flash counter_program() {
    Flash flash;
    flash.at(0x0000).op({0x75, SP, 0x60,  // 0x0000 MOV SP,#0x60
                         0x74, 0x01,      // 0x0003 MOV A,#1
                         0x7f, 0x00,      // 0x0005 MOV R7,#0
                         0x24, 0x03,      // 0x0007 loop: ADD A,#3
                         0x23,            // 0x0009 RL A
                         0x0f,            // 0x000a INC R7
                         0xf5, 0x30,      // 0x000b MOV 0x30,A
                         0x8f, 0x31,      // 0x000d MOV 0x31,R7
                         0x80, 0xf6});    // 0x000f SJMP loop
    flash.at(0x0100).op({0x5c});          // a constant for MOVC
    return flash;
}

// Calls 0x8000 in bank 2 (A = 0x22) and bank 3 (A = 0x33) alternately.
Flash banked_program() {
    Flash flash;
    flash.at(0x0000).op({0x75, SP, 0x60,    // 0x0000 MOV SP,#0x60
                         0x75, FMAP, 0x02,  // 0x0003 MOV FMAP,#2
                         0x12, 0x80, 0x00,  // 0x0006 LCALL 0x8000
                         0xf5, 0x30,        // 0x0009 MOV 0x30,A
                         0x75, FMAP, 0x03,  // 0x000b MOV FMAP,#3
                         0x12, 0x80, 0x00,  // 0x000e LCALL 0x8000
                         0xf5, 0x31,        // 0x0011 MOV 0x31,A
                         0x80, 0xee});      // 0x0013 SJMP 0x0003
    flash.at(0x10000).op({0x74, 0x22, 0x22}); // bank 2: MOV A,#0x22; RET
    flash.at(0x18000).op({0x74, 0x33, 0x22}); // bank 3: MOV A,#0x33; RET
    return flash;
}

Flash spin_program() {
    Flash flash;
    flash.at(0x0000).op({0x80, 0xfe}); // SJMP $
    return flash;
}

void check_same_cpu(const Cc2530F256 &actual, const Cc2530F256 &expected) {
    const auto a = actual.registers();
    const auto e = expected.registers();
    CHECK_EQ(a.pc, e.pc);
    CHECK_EQ(a.a, e.a);
    CHECK_EQ(a.b, e.b);
    CHECK_EQ(a.psw, e.psw);
    CHECK_EQ(a.sp, e.sp);
    CHECK_EQ(a.dpl, e.dpl);
    CHECK_EQ(a.dph, e.dph);
    CHECK_EQ(a.fmap, e.fmap);
    CHECK(a.r == e.r);
    for (unsigned address = 0x08; address < 0x100; ++address)
        CHECK_EQ(actual.read_idata(static_cast<std::uint8_t>(address)),
                 expected.read_idata(static_cast<std::uint8_t>(address)));
}

// ---- tests ----

// Table 3-1 encodings: three don't-care LSBs, DEBUG_INSTR yy, BURST_WRITE
// kkk:length (0 = 2048), output byte counts; undefined codes are refused.
void test_encodings() {
    struct Length {
        std::uint8_t command;
        std::uint8_t second;
        unsigned in;
        unsigned out;
    };
    for (const Length &l : std::initializer_list<Length>{
             {0x10, 0, 1, 1}, {0x14, 0, 1, 1}, {0x18, 0, 2, 1}, {0x1d, 0, 2, 1},
             {0x20, 0, 1, 1}, {0x24, 0, 1, 1}, {0x28, 0, 1, 2}, {0x2f, 0, 1, 2},
             {0x30, 0, 1, 1}, {0x34, 0, 1, 1}, {0x38, 0, 4, 1}, {0x3f, 0, 4, 1},
             {0x40, 0, 1, 1}, {0x44, 0, 1, 1}, {0x48, 0, 1, 1}, {0x4c, 0, 1, 1},
             {0x51, 0, 2, 1}, {0x52, 0, 3, 1}, {0x53, 0, 4, 1}, {0x55, 0, 2, 1},
             {0x57, 0, 4, 1}, {0x58, 0, 1, 1}, {0x5c, 0, 1, 1}, {0x60, 0, 1, 1},
             {0x64, 0, 1, 1}, {0x68, 0, 1, 2}, {0x6c, 0, 1, 2}, {0x80, 0x00, 2050, 1},
             {0x80, 0x01, 3, 1}, {0x80, 0xff, 257, 1}, {0x81, 0x00, 258, 1},
             {0x84, 0x00, 1026, 1}, {0x87, 0xff, 2049, 1}}) {
        CHECK_EQ(Cc2530F256::debug_if_input_length(l.command, l.second), l.in);
        CHECK_EQ(Cc2530F256::debug_if_output_length(l.command), l.out);
    }

    Cc2530F256 device;
    load(device, spin_program());
    CHECK(!device.debug_if_active());
    CHECK(throws_logic([&] { device.debug_if_submit({READ_STATUS}); }));
    device.debug_if_enter();
    // Undefined codes and DEBUG_INSTR with yy = 0 (model-defined: one byte
    // in, one out, refused).
    for (const std::uint8_t command : {0x00, 0x08, 0x50, 0x54, 0x70, 0x78, 0x88, 0xf8})
        refused(device, {command}, Halted);
    // Malformed host commands.
    CHECK(throws_invalid([&] { device.debug_if_submit({}); }));
    CHECK(throws_invalid([&] { device.debug_if_submit({WR_CONFIG}); }));
    CHECK(throws_invalid([&] { device.debug_if_submit({READ_STATUS, 0x00}); }));
    CHECK(throws_invalid([&] { device.debug_if_submit({BURST_WRITE, 0x02, 0x11}); }));
    device.debug_if_submit({READ_STATUS});
    CHECK(throws_logic([&] { device.debug_if_submit({READ_STATUS}); })); // outstanding
    CHECK(!device.debug_if_take_response());
    advance_to(device, device.debug_if_due_cycle());
    const std::optional<Cc2530DebugIfResponse> response = device.debug_if_take_response();
    CHECK(response && response->accepted && response->bytes == Bytes{Halted});
}

// 3.1 entry state, the Table 3-3 status after entry, RD_CONFIG/WR_CONFIG,
// GET_CHIP_ID (CHIPID/CHVER), GET_PC, GET_BM and the host port timing.
void test_entry_and_status() {
    Cc2530F256 device;
    load(device, counter_program());
    advance(device, 2000);
    CHECK(device.registers().instructions > 0);
    device.debug_if_enter();
    CHECK(device.debug_if_active());
    CHECK(device.debug_if_halted());
    CHECK(!device.debug_if_locked());
    CHECK_EQ(device.pc(), 0x0000);
    CHECK_EQ(device.registers().sp, 0x07);
    CHECK_EQ(device.read_sfr(SLEEPSTA) & 0x18, 0x08); // RST = external reset
    CHECK_EQ(device.debug_if_status(), Halted);
    CHECK_EQ(device.debug_if_config(), ConfigReset);

    // Model-defined host port: byte i of a command arrives (i + 1) x 128
    // cycles after the port is free; the response shift-out occupies it for
    // another 128 cycles per output byte.
    const std::uint64_t t0 = device.registers().cycles;
    Cc2530DebugIfResponse response = send(device, {READ_STATUS});
    CHECK(response.accepted && response.bytes == Bytes{Halted});
    CHECK_EQ(response.cycle, t0 + HostByte);
    response = send(device, {RD_CONFIG});
    CHECK(response.bytes == Bytes{ConfigReset});
    CHECK_EQ(response.cycle, t0 + 3 * HostByte);
    response = send(device, {WR_CONFIG, 0x22});
    CHECK_EQ(response.cycle, t0 + 6 * HostByte);

    const Bytes id = accepted(device, {GET_CHIP_ID});
    CHECK_EQ(id.at(0), 0xa5); // CC2530 (Table 3-1, CHIPID)
    CHECK_EQ(id.at(0), device.peek_xdata(CHIPID));
    CHECK_EQ(id.at(1), device.peek_xdata(CHVER));
    CHECK_EQ(get_pc(device), 0x0000);
    CHECK_EQ(accepted(device, {GET_BM}).at(0), device.read_sfr(FMAP) & 0x07);
    CHECK_EQ(accepted(device, {GET_BM}).at(0), 1); // FMAP reset value

    // Reserved configuration bits read back 0 (Table 3-2).
    accepted(device, {WR_CONFIG, 0xff});
    CHECK_EQ(accepted(device, {RD_CONFIG}).at(0), 0x2e);
    CHECK_EQ(device.debug_if_config(), 0x2e);
    accepted(device, {0x1d, 0x00}); // WR_CONFIG with don't-care bits
    CHECK_EQ(accepted(device, {0x24}).at(0), 0x00);
    accepted(device, {WR_CONFIG, ConfigReset});
    CHECK_EQ(accepted(device, {0x34}).at(0), Halted); // READ_STATUS, don't-care bits
    CHECK(accepted(device, {0x6f}) == id);           // GET_CHIP_ID, don't-care bits

    // 3.3: the halted CPU executes nothing while time passes.
    const std::uint64_t instructions = device.registers().instructions;
    const std::uint64_t halted = device.diagnostics().debug_halted_cycles;
    advance(device, 100000);
    CHECK_EQ(device.registers().instructions, instructions);
    CHECK_EQ(device.pc(), 0x0000);
    CHECK(device.diagnostics().debug_halted_cycles >= halted + 100000);
}

// STEP_INSTR executes exactly one instruction and returns A; HALT/RESUME
// stop and continue the CPU. Both match an undisturbed reference run.
void test_halt_step_resume() {
    const Flash flash = counter_program();
    Cc2530F256 reference;
    load(reference, flash);
    Cc2530F256 device;
    load(device, flash);
    device.debug_if_enter();
    const std::uint64_t start = device.registers().instructions;
    for (unsigned i = 0; i < 20; ++i) {
        reference.run(1);
        CHECK_EQ(step(device), reference.registers().a);
        CHECK_EQ(get_pc(device), reference.pc());
        CHECK(device.debug_if_halted());
    }
    CHECK_EQ(device.registers().instructions - start, 20u);
    check_same_cpu(device, reference);

    // RESUME and HALT need no instruction count: compare against the
    // reference after as many instructions as the CPU executed.
    CHECK_EQ(accepted(device, {RESUME}).at(0), OscillatorStable);
    CHECK(!device.debug_if_halted());
    refused(device, {RESUME}, OscillatorStable);       // not halted
    refused(device, {STEP_INSTR}, OscillatorStable);   // not halted
    refused(device, {0x51, 0x00}, OscillatorStable);   // DEBUG_INSTR: not halted
    advance(device, 5000);
    CHECK_EQ(accepted(device, {HALT}).at(0), Halted); // HALT_STATUS = 0: HALT command
    CHECK_EQ(accepted(device, {HALT}).at(0), Halted); // already halted
    const std::uint64_t executed = device.registers().instructions - start;
    CHECK(executed > 20);
    reference.run(executed - 20);
    check_same_cpu(device, reference);
    CHECK_EQ(get_pc(device), reference.pc());

    // STEP_INSTR with an interrupt pending steps the interrupt entry
    // (model-defined): RFERR (Table 2-5 source 0, vector 0x0003).
    const std::uint8_t sp = device.registers().sp;
    const std::uint16_t pc = device.pc();
    instr(device, {0x75, IEN0, 0x81}); // EA | RFERRIE
    device.raise_interrupt_flag_for_test(0);
    step(device);
    CHECK_EQ(get_pc(device), 0x0003);
    CHECK_EQ(device.registers().sp, sp + 2);
    CHECK_EQ(device.read_idata(static_cast<std::uint8_t>(sp + 1)), lo(pc));
    CHECK_EQ(device.read_idata(static_cast<std::uint8_t>(sp + 2)), hi(pc));
}

// The same debug session in fast, single-step and trace execution ends in
// the same state.
void test_debug_session_mode_equivalence() {
    std::vector<std::uint8_t> reference;
    for (const Mode mode : {Mode::Fast, Mode::Step, Mode::Trace}) {
        Cc2530F256 device;
        traced(device, mode);
        load(device, banked_program());
        advance(device, 3000, mode);
        device.debug_if_enter();
        accepted(device, {WR_CONFIG, 0x22});
        for (unsigned i = 0; i < 5; ++i)
            step(device);
        accepted(device, {RESUME});
        advance(device, 3000, mode);
        accepted(device, {HALT});
        instr(device, {0x74, 0x5a});
        breakpoint(device, 1, true, 3, 0x8000);
        accepted(device, {RESUME});
        run_to_halt(device, mode);
        CHECK_EQ(status(device), Halted | HaltStatus);
        breakpoint(device, 2, true, 0, 0x0009);
        for (unsigned i = 0; i < 4; ++i) {
            accepted(device, {RESUME});
            run_to_halt(device, mode);
        }
        advance(device, 1000, mode);
        const auto state = snapshot(device);
        if (mode == Mode::Fast) {
            reference = state;
        } else if (state != reference) {
            std::cerr << "debug session: " << mode_name(mode) << " differs from fast\n";
            ++failures;
        }
    }
}

// 3.3.3: four breakpoints, enable/bank/address, halt with HALT_STATUS,
// RESUME leaves the breakpoint address; banked code compares FMAP.
void test_hw_breakpoints() {
    {
        Cc2530F256 device;
        load(device, counter_program());
        device.debug_if_enter();
        breakpoint(device, 0, true, 0, 0x000a); // INC R7
        const auto &slots = device.peripherals().debug->breakpoints();
        CHECK(slots[0].enabled && slots[0].bank == 0 && slots[0].address == 0x000a);
        accepted(device, {RESUME});
        Cc2530RunResult result = run_to_halt(device);
        CHECK_EQ(result.pc, 0x000a);
        CHECK_EQ(status(device), Halted | HaltStatus);
        CHECK_EQ(get_pc(device), 0x000a);
        CHECK_EQ(device.registers().r[7], 0);
        // RESUME executes the instruction at the breakpoint; the next arrival
        // halts again, one loop (6 instructions) later.
        std::uint64_t before = device.registers().instructions;
        accepted(device, {RESUME});
        result = run_to_halt(device);
        CHECK_EQ(result.pc, 0x000a);
        CHECK_EQ(device.registers().instructions - before, 6u);
        CHECK_EQ(device.registers().r[7], 1);

        // Breakpoint 3 at 0x000d as well: the first one reached halts.
        breakpoint(device, 3, true, 0, 0x000d);
        CHECK(slots[3].enabled && slots[3].address == 0x000d);
        accepted(device, {RESUME});
        CHECK_EQ(run_to_halt(device).pc, 0x000d);
        accepted(device, {RESUME});
        CHECK_EQ(run_to_halt(device).pc, 0x000a);
        // Disabled breakpoints do not halt.
        breakpoint(device, 0, false, 0, 0x000a);
        CHECK(!slots[0].enabled);
        before = device.registers().instructions;
        accepted(device, {RESUME});
        CHECK_EQ(run_to_halt(device).pc, 0x000d);
        CHECK_EQ(device.registers().instructions - before, 2u); // INC R7, MOV 0x30,A
        accepted(device, {RESUME});
        CHECK_EQ(run_to_halt(device).pc, 0x000d);
        breakpoint(device, 3, false, 0, 0x000d);
        accepted(device, {RESUME});
        advance(device, 20000);
        CHECK(!device.debug_if_halted());
        CHECK_EQ(accepted(device, {HALT}).at(0), Halted); // HALT_STATUS: HALT command
    }
    {
        // Banked code: only the bank-3 call halts at 0x8000.
        Cc2530F256 device;
        load(device, banked_program());
        device.debug_if_enter();
        breakpoint(device, 1, true, 3, 0x8000);
        accepted(device, {RESUME});
        run_to_halt(device);
        CHECK_EQ(get_pc(device), 0x8000);
        CHECK_EQ(accepted(device, {GET_BM}).at(0), 3);
        CHECK_EQ(device.read_idata(0x30), 0x22); // the bank-2 call completed
        CHECK_EQ(device.read_idata(0x31), 0x00);
        const std::uint64_t before = device.registers().instructions;
        accepted(device, {RESUME});
        run_to_halt(device);
        CHECK_EQ(get_pc(device), 0x8000);
        CHECK_EQ(accepted(device, {GET_BM}).at(0), 3);
        CHECK_EQ(device.read_idata(0x31), 0x33);
        CHECK_EQ(device.registers().instructions - before, 11u); // one loop
        // Bank 2 instead.
        breakpoint(device, 1, true, 2, 0x8000);
        accepted(device, {RESUME});
        run_to_halt(device);
        CHECK_EQ(get_pc(device), 0x8000);
        CHECK_EQ(accepted(device, {GET_BM}).at(0), 2);
        // Model-defined: in the common area (0x0000-0x7fff) the bank bits
        // are not compared.
        breakpoint(device, 1, false, 2, 0x8000);
        breakpoint(device, 0, true, 5, 0x0009);
        accepted(device, {RESUME});
        run_to_halt(device);
        CHECK_EQ(get_pc(device), 0x0009);
    }
    {
        // Several breakpoints (bank 2 RET, bank 3 entry, common code) give
        // the same arrivals in fast, single-step and trace execution.
        std::vector<std::tuple<std::uint16_t, std::uint8_t, std::uint64_t, std::uint64_t>> first;
        for (const Mode mode : {Mode::Fast, Mode::Step, Mode::Trace}) {
            Cc2530F256 device;
            traced(device, mode);
            load(device, banked_program());
            device.debug_if_enter();
            breakpoint(device, 0, true, 2, 0x8002);
            breakpoint(device, 1, true, 3, 0x8000);
            breakpoint(device, 2, true, 0, 0x0011);
            std::vector<std::tuple<std::uint16_t, std::uint8_t, std::uint64_t, std::uint64_t>> hits;
            for (unsigned i = 0; i < 9; ++i) {
                accepted(device, {RESUME});
                const Cc2530RunResult result = run_to_halt(device, mode);
                hits.emplace_back(result.pc, result.fmap, device.registers().instructions,
                                  result.cycles);
            }
            CHECK_EQ(std::get<0>(hits[0]), 0x8002);
            CHECK_EQ(std::get<1>(hits[0]), 2);
            CHECK_EQ(std::get<0>(hits[1]), 0x8000);
            CHECK_EQ(std::get<1>(hits[1]), 3);
            CHECK_EQ(std::get<0>(hits[2]), 0x0011);
            if (mode == Mode::Fast)
                first = hits;
            else if (hits != first) {
                std::cerr << "breakpoints: " << mode_name(mode) << " differs from fast\n";
                ++failures;
            }
        }
    }
}

// DEBUG_INSTR executes the supplied instruction on the real core without
// moving PC (except for control transfers) and returns A.
void test_debug_instr() {
    const Flash flash = counter_program();
    Cc2530F256 device;
    load(device, flash);
    device.debug_if_enter();
    const std::uint64_t instructions = device.registers().instructions;
    const std::uint64_t debug_instructions = device.diagnostics().debug_instructions;
    unsigned issued = 0;
    const auto run = [&](const Bytes &code) {
        ++issued;
        return instr(device, code);
    };

    // Arithmetic and flags: 0x80 + 0x80 = 0x100: A = 0, CY = OV = 1, P = 0.
    CHECK_EQ(run({0x74, 0x80}), 0x80); // MOV A,#0x80
    CHECK_EQ(run({0x24, 0x80}), 0x00); // ADD A,#0x80
    CHECK_EQ(device.registers().psw, 0x84);
    CHECK_EQ(device.pc(), 0x0000);
    // Registers and XDATA.
    run({0x7f, 0x42}); // MOV R7,#0x42
    CHECK_EQ(device.registers().r[7], 0x42);
    run({0x90, 0x01, 0x23}); // MOV DPTR,#0x0123
    run({0x74, 0x77});       // MOV A,#0x77
    run({0xf0});             // MOVX @DPTR,A
    CHECK_EQ(device.peek_xdata(0x0123), 0x77);
    CHECK_EQ(run({0xe4}), 0x00);       // CLR A
    CHECK_EQ(run({0xe0}), 0x77);       // MOVX A,@DPTR
    CHECK_EQ(run({0xe5, 0x9f}), 0x01); // MOV A,FMAP
    // MOVC reads the program memory, also at the address where the debug
    // interface fed the instruction (PC 0x0000).
    run({0x90, 0x00, 0x00}); // MOV DPTR,#0
    run({0xe4});
    CHECK_EQ(run({0x93}), flash.bytes[0]); // MOVC A,@A+DPTR
    run({0x74, 0xff});
    CHECK_EQ(run({0x83}), 0x5c); // MOVC A,@A+PC: 0x0000 + 1 + 0xff
    // FMAP through an SFR write; GET_BM follows.
    run({0x75, FMAP, 0x05});
    CHECK_EQ(accepted(device, {GET_BM}).at(0), 5);
    run({0x75, FMAP, 0x01});
    // 0xA5 (undefined opcode): model-defined no-op.
    run({0x74, 0x3c});
    CHECK_EQ(run({0xa5}), 0x3c);
    CHECK_EQ(device.pc(), 0x0000);
    // Control transfers change PC: LJMP, SJMP, a not-taken and a taken JZ,
    // LCALL and RET.
    run({0x02, 0x00, 0x07}); // LJMP 0x0007
    CHECK_EQ(get_pc(device), 0x0007);
    run({0x80, 0x02}); // SJMP +2
    CHECK_EQ(get_pc(device), 0x000b);
    run({0x74, 0x01});
    run({0x60, 0x05}); // JZ +5, not taken
    CHECK_EQ(get_pc(device), 0x000b);
    run({0xe4});
    run({0x60, 0x05}); // JZ +5, taken
    CHECK_EQ(get_pc(device), 0x0012);
    const std::uint8_t sp = device.registers().sp;
    run({0x12, 0x00, 0x40}); // LCALL 0x0040
    CHECK_EQ(get_pc(device), 0x0040);
    CHECK_EQ(device.registers().sp, sp + 2);
    run({0x22}); // RET
    CHECK_EQ(get_pc(device), 0x0015);
    CHECK_EQ(device.registers().sp, sp);

    // Not firmware instructions: the counter, the flash and virtual time
    // (apart from the host port) are untouched.
    CHECK_EQ(device.registers().instructions, instructions);
    CHECK_EQ(device.diagnostics().debug_instructions - debug_instructions, issued);
    for (std::uint32_t address = 0; address < 0x20; ++address)
        CHECK_EQ(device.read_flash(address), flash.bytes[address]);
    // The host port is free once the last response has been shifted out;
    // the two command bytes then take 2 x 128 cycles and the instruction
    // itself none.
    advance_to(device, device.peripherals().debug->host_port_free_at());
    const std::uint64_t before = device.registers().cycles;
    const Cc2530DebugIfResponse response = send(device, {0x51, 0x00}); // NOP
    CHECK_EQ(response.cycle, before + 2 * HostByte);
    CHECK_EQ(device.registers().cycles, response.cycle);
}

// BURST_WRITE puts each byte in DBGDATA and raises DMA trigger 31 (DBG_BW,
// Table 8-1); DBGDATA is read-only; DMA_PAUSE holds the transfers.
void test_burst_write_dma() {
    Cc2530F256 device;
    load(device, spin_program());
    device.debug_if_enter();
    accepted(device, {WR_CONFIG, 0x22}); // DMA_PAUSE off
    const auto arm = [&](std::uint16_t destination, unsigned length) {
        // Table 8-2: DBGDATA -> destination, LEN, single mode, TRIG 31,
        // DESTINC +1, high priority.
        debug_write_xdata(device, 0x0f00,
                          {hi(DBGDATA), lo(DBGDATA), hi(destination), lo(destination),
                           static_cast<std::uint8_t>(length >> 8),
                           static_cast<std::uint8_t>(length), 0x1f, 0x12});
        debug_write_sfr(device, DMA0CFGH, 0x0f);
        debug_write_sfr(device, DMA0CFGL, 0x00);
        debug_write_sfr(device, DMAIRQ, 0x00);
        debug_write_sfr(device, DMAARM, 0x01);
    };

    Bytes data;
    for (unsigned i = 0; i < 16; ++i)
        data.push_back(static_cast<std::uint8_t>(0xa0 ^ (i * 13)));
    arm(0x0100, 16);
    Bytes command{BURST_WRITE, 16};
    command.insert(command.end(), data.begin(), data.end());
    advance_to(device, device.peripherals().debug->host_port_free_at());
    const std::uint64_t start = device.registers().cycles;
    const Cc2530DebugIfResponse response = send(device, command);
    CHECK(response.accepted && response.bytes == Bytes{Halted});
    CHECK_EQ(response.cycle, start + 18 * HostByte);
    // The status byte is ready at the last data byte; its DMA transfer ends
    // within the response shift-out.
    advance(device, HostByte);
    for (unsigned i = 0; i < 16; ++i)
        CHECK_EQ(device.peek_xdata(static_cast<std::uint16_t>(0x0100 + i)), data[i]);
    CHECK_EQ(device.read_sfr(DMAIRQ) & 0x01, 0x01);
    CHECK_EQ(device.read_sfr(DMAARM) & 0x01, 0x00); // LEN transfers done
    CHECK_EQ(device.peek_xdata(DBGDATA), data.back());
    // DBGDATA is read-only (R).
    debug_write_xdata(device, DBGDATA, {0x00});
    CHECK_EQ(device.peek_xdata(DBGDATA), data.back());
    instr(device, {0x90, hi(DBGDATA), lo(DBGDATA)});
    CHECK_EQ(instr(device, {0xe0}), data.back());

    // A full 2048-byte burst (length field 0) and a kkk = 1 (256-byte) one.
    Bytes page;
    for (unsigned i = 0; i < 2048; ++i)
        page.push_back(static_cast<std::uint8_t>(i * 7 + (i >> 8)));
    arm(0x0000, 2048);
    command = {BURST_WRITE, 0x00};
    command.insert(command.end(), page.begin(), page.end());
    CHECK(send(device, command).accepted);
    advance(device, HostByte);
    bool same = true;
    for (unsigned i = 0; i < 2048; ++i)
        same = same && device.peek_xdata(static_cast<std::uint16_t>(i)) == page[i];
    CHECK(same);
    CHECK_EQ(device.read_sfr(DMAIRQ) & 0x01, 0x01);
    arm(0x0900, 256);
    command = {0x81, 0x00};
    command.insert(command.end(), page.begin(), page.begin() + 256);
    CHECK(send(device, command).accepted);
    advance(device, HostByte);
    CHECK_EQ(device.read_sfr(DMAIRQ) & 0x01, 0x01);
    CHECK_EQ(device.peek_xdata(0x09ff), page[255]);
    CHECK_EQ(device.peek_xdata(0x0a00), 0x00);

    // Not armed: DBGDATA still takes the bytes, no transfer happens.
    debug_write_sfr(device, DMAIRQ, 0x00);
    CHECK(send(device, {BURST_WRITE, 1, 0x99}).accepted);
    CHECK_EQ(device.peek_xdata(DBGDATA), 0x99);
    CHECK_EQ(device.read_sfr(DMAIRQ), 0x00);

    // DMA_PAUSE (Table 3-2): no transfer while set; the counted triggers
    // ("missed events", 8.1) are served once it is cleared, each reading
    // DBGDATA as it is then.
    arm(0x0a00, 4);
    accepted(device, {WR_CONFIG, 0x26});
    CHECK(send(device, {BURST_WRITE, 4, 0x01, 0x02, 0x03, 0x04}).accepted);
    advance(device, 1000);
    for (unsigned i = 0; i < 4; ++i)
        CHECK_EQ(device.peek_xdata(static_cast<std::uint16_t>(0x0a00 + i)), 0x00);
    CHECK_EQ(device.read_sfr(DMAIRQ) & 0x01, 0x00);
    CHECK_EQ(device.read_sfr(DMAARM) & 0x01, 0x01);
    accepted(device, {WR_CONFIG, 0x22});
    advance(device, 1000);
    for (unsigned i = 0; i < 4; ++i)
        CHECK_EQ(device.peek_xdata(static_cast<std::uint16_t>(0x0a00 + i)), 0x04);
    CHECK_EQ(device.read_sfr(DMAIRQ) & 0x01, 0x01);
}

// CHIP_ERASE: only right after entry or READ_STATUS; CHIP_ERASE_BUSY for
// the 20 ms chip-erase time (2.2.3); every byte, the lock bits included,
// becomes 0xff.
void test_chip_erase() {
    Flash flash = counter_program();
    flash.at(0x38000).op({0x12, 0x34});
    flash.at(Cc2530F256::LockBitsStart).op({0x00, 0xfe}); // pages 0-8 write-protected
    {
        // Another command first disables CHIP_ERASE until the next entry.
        Cc2530F256 device;
        load(device, flash);
        device.debug_if_enter();
        accepted(device, {RD_CONFIG});
        refused(device, {CHIP_ERASE}, Halted);
        device.debug_if_enter();
        accepted(device, {GET_CHIP_ID}); // model-defined: also disables it
        refused(device, {CHIP_ERASE}, Halted);
        advance(device, 2 * ChipEraseCycles);
        CHECK_EQ(device.read_flash(0), flash.bytes[0]);
    }
    Cc2530F256 device;
    load(device, flash);
    device.debug_if_enter();
    CHECK_EQ(status(device), Halted);
    CHECK_EQ(status(device), Halted);
    const Cc2530DebugIfResponse erase = send(device, {0x14}); // CHIP_ERASE, don't-care bits
    CHECK(erase.accepted);
    CHECK(erase.bytes == Bytes{ChipEraseBusy | Halted});
    const std::uint64_t t0 = erase.cycle;
    CHECK_EQ(device.debug_if_status(), ChipEraseBusy | Halted);
    // During the erase only READ_STATUS and GET_CHIP_ID are accepted.
    CHECK_EQ(status(device), ChipEraseBusy | Halted);
    CHECK_EQ(accepted(device, {GET_CHIP_ID}).at(0), 0xa5);
    refused(device, {HALT}, ChipEraseBusy | Halted);
    refused(device, {GET_PC}, ChipEraseBusy | Halted);
    refused(device, {CHIP_ERASE}, ChipEraseBusy | Halted);
    refused(device, {0x51, 0x00}, ChipEraseBusy | Halted);
    advance_to(device, t0 + ChipEraseCycles - 1);
    CHECK_EQ(device.debug_if_status(), ChipEraseBusy | Halted);
    CHECK_EQ(device.read_flash(0), flash.bytes[0]);
    advance_to(device, t0 + ChipEraseCycles);
    CHECK_EQ(device.debug_if_status(), Halted);
    bool erased = true;
    for (std::uint32_t address = 0; address < Cc2530F256::FlashSize; ++address)
        erased = erased && device.read_flash(address) == 0xff;
    CHECK(erased);
    refused(device, {CHIP_ERASE}, Halted); // GET_CHIP_ID was issued
    // The erased lock bits no longer protect page 0: a page erase works.
    CHECK_EQ(get_pc(device), 0x0000);
}

// 3.4.1: DBGLOCK = 0 (bit 7 of the last flash byte) leaves only
// CHIP_ERASE, READ_STATUS and GET_CHIP_ID; it is latched at reset, and a
// CHIP_ERASE unlocks the interface from the next reset on.
void test_dbglock() {
    Flash flash = counter_program();
    flash.at(DbglockByte).op({0x7f});
    Cc2530F256 device;
    load(device, flash);
    CHECK(device.debug_if_locked());
    advance(device, 1000); // the firmware runs as usual
    CHECK(device.registers().instructions > 0);
    device.debug_if_enter();
    const std::uint8_t locked = Halted | DebugLocked;
    CHECK_EQ(device.debug_if_status(), locked);
    CHECK_EQ(status(device), locked);
    for (const Bytes &command : {Bytes{WR_CONFIG, 0x22}, Bytes{RD_CONFIG}, Bytes{GET_PC},
                                 Bytes{SET_HW_BRKPNT, 0x08, 0x00, 0x07}, Bytes{HALT},
                                 Bytes{RESUME}, Bytes{0x51, 0x00}, Bytes{0x53, 0x02, 0x00, 0x40},
                                 Bytes{STEP_INSTR}, Bytes{GET_BM},
                                 Bytes{BURST_WRITE, 2, 0x55, 0x66}})
        refused(device, command, locked);
    CHECK_EQ(accepted(device, {GET_CHIP_ID}).at(0), 0xa5);
    CHECK_EQ(device.debug_if_config(), ConfigReset);
    CHECK(!device.peripherals().debug->breakpoints()[0].enabled);
    CHECK_EQ(device.peek_xdata(DBGDATA), 0x00);
    CHECK_EQ(device.pc(), 0x0000);

    device.debug_if_enter();
    CHECK_EQ(status(device), locked);
    CHECK(accepted(device, {CHIP_ERASE}).at(0) == (ChipEraseBusy | locked));
    advance(device, ChipEraseCycles);
    CHECK_EQ(device.read_flash(DbglockByte), 0xff);
    // Still locked until the next reset.
    CHECK_EQ(status(device), locked);
    refused(device, {HALT}, locked);
    device.debug_if_enter();
    CHECK(!device.debug_if_locked());
    CHECK_EQ(status(device), Halted);
    CHECK_EQ(accepted(device, {HALT}).at(0), Halted);
}

// Table 3-2 configuration effects and Table 3-3 power-mode/stack bits.
void test_config_effects() {
    {
        // TIMER_SUSPEND / TIMERS_OFF on Timer 1 (free-running, tick / 1).
        Flash flash;
        flash.at(0x0000).op({0x75, T1CTL, 0x01, // MOV T1CTL,#1
                             0x80, 0xfe});      // SJMP $
        Cc2530F256 device;
        load(device, flash);
        device.debug_if_enter();
        step(device); // the timer starts
        const auto count = [&] {
            const std::uint8_t low = instr(device, {0xe5, T1CNTL}); // latches T1CNTH
            return static_cast<unsigned>(instr(device, {0xe5, T1CNTH}) << 8 | low);
        };
        // Entry configuration 0x26: TIMER_SUSPEND, so a halted CPU stops it.
        unsigned before = count();
        advance(device, 10000);
        CHECK_EQ(count(), before);
        // Neither bit: the timer runs while the CPU is halted.
        accepted(device, {WR_CONFIG, 0x20});
        before = count();
        advance(device, 10000);
        CHECK(count() != before);
        // TIMERS_OFF: stopped even while the CPU runs.
        accepted(device, {WR_CONFIG, 0x28});
        before = count();
        accepted(device, {RESUME});
        advance(device, 10000);
        accepted(device, {HALT});
        CHECK_EQ(count(), before);
        // TIMER_SUSPEND: runs with the CPU, stops when it is halted.
        accepted(device, {WR_CONFIG, 0x22});
        before = count();
        accepted(device, {RESUME});
        advance(device, 10000);
        accepted(device, {HALT});
        const unsigned running = count();
        CHECK(running != before);
        advance(device, 10000);
        CHECK_EQ(count(), running);
    }
    // PCON entering PM2 (the next instruction on a 4-byte boundary, 4.2).
    Flash sleeper;
    sleeper.at(0x0000).op({0x75, SLEEPCMD, 0x06, // MOV SLEEPCMD,#6 (PM2)
                           0x00, 0x00,           // NOP NOP
                           0x75, PCON, 0x01,     // MOV PCON,#1
                           0x00,                 // 0x0008 NOP
                           0x80, 0xfe});         // SJMP $
    {
        // SOFT_POWER_MODE = 1: the chip enters PM2 and stays in debug mode;
        // only READ_STATUS, HALT and GET_CHIP_ID are accepted; HALT wakes it
        // up halted (3.5).
        Cc2530F256 device;
        load(device, sleeper);
        device.debug_if_enter();
        accepted(device, {RESUME});
        advance(device, 5000);
        CHECK(device.sleeping());
        CHECK(device.power_mode() == Cc2530PowerMode::Pm2);
        CHECK(device.debug_if_active());
        CHECK_EQ(status(device), PconIdle | PmActive);
        CHECK_EQ(accepted(device, {GET_CHIP_ID}).at(0), 0xa5);
        refused(device, {GET_PC}, PconIdle | PmActive);
        refused(device, {RD_CONFIG}, PconIdle | PmActive);
        CHECK_EQ(accepted(device, {HALT}).at(0), Halted);
        CHECK(device.power_mode() == Cc2530PowerMode::Active);
        CHECK(!device.sleeping());
        CHECK_EQ(get_pc(device), 0x0008);
        CHECK_EQ(step(device), 0x00); // the NOP after the PCON write
        CHECK_EQ(get_pc(device), 0x0009);
    }
    {
        // SOFT_POWER_MODE = 0: PM2 turns the digital regulator off, which
        // ends debug mode (Table 3-2).
        Cc2530F256 device;
        load(device, sleeper);
        device.debug_if_enter();
        accepted(device, {WR_CONFIG, 0x02});
        accepted(device, {RESUME});
        advance(device, 5000);
        CHECK(device.sleeping());
        CHECK(!device.debug_if_active());
        CHECK(throws_logic([&] { device.debug_if_submit({READ_STATUS}); }));
    }
    {
        // STACK_OVERFLOW: the CPU writes DATA 0xff.
        Flash flash;
        flash.at(0x0000).op({0x75, SP, 0xfe, // MOV SP,#0xfe
                             0xc0, 0xe0,     // PUSH ACC (to 0xff)
                             0x80, 0xfe});   // SJMP $
        Cc2530F256 device;
        load(device, flash);
        device.debug_if_enter();
        step(device);
        CHECK_EQ(status(device), Halted);
        step(device);
        CHECK_EQ(status(device), Halted | StackOverflow);
        instr(device, {0x75, SP, 0x60});
        CHECK_EQ(status(device), Halted | StackOverflow); // sticky until reset
        device.debug_if_enter();
        CHECK_EQ(status(device), Halted);
    }
}

// ---- the DC/DD wire (3.1, 3.2) ----

// A debugger on the pads: DC (P2.2) at 1 MHz (16 cycles per half period),
// DD (P2.1) set up while DC is low, sampled by the chip at the falling DC
// edge and by the debugger while DC is high.
struct Wire {
    Cc2530F256 &device;

    void settle() { device.run(SIZE_MAX, device.registers().cycles + 16); }
    void dc(bool level) {
        device.set_pin_input(2, 2, level);
        settle();
    }
    void pulse() {
        dc(true);
        dc(false);
    }
    void reset_n(bool level) {
        device.set_reset_pin(level);
        settle();
    }
    // 3.1: RESET_N low, `edges` falling DC edges, RESET_N released.
    void entry(unsigned edges) {
        dc(false);
        reset_n(false);
        for (unsigned i = 0; i < edges; ++i)
            pulse();
        reset_n(true);
    }
    void send_bit(bool bit) {
        device.set_pin_input(2, 1, bit);
        pulse();
    }
    void send(std::uint8_t byte) {
        for (int bit = 7; bit >= 0; --bit)
            send_bit(((byte >> bit) & 1u) != 0);
    }
    // Figure 3-4: DD low = response ready; otherwise 8 more clocks.
    unsigned wait_ready() {
        device.set_pin_input(2, 1, std::nullopt);
        unsigned rounds = 0;
        while (device.pin_level(2, 1) && rounds < 1000) {
            for (unsigned i = 0; i < 8; ++i)
                pulse();
            ++rounds;
        }
        CHECK(rounds < 1000);
        return rounds;
    }
    std::uint8_t receive() {
        std::uint8_t value = 0;
        for (unsigned i = 0; i < 8; ++i) {
            dc(true);
            value = static_cast<std::uint8_t>(value << 1 | (device.pin_level(2, 1) ? 1u : 0u));
            dc(false);
        }
        return value;
    }
    Bytes command(const Bytes &bytes, unsigned *rounds = nullptr) {
        for (const std::uint8_t byte : bytes)
            send(byte);
        const unsigned waited = wait_ready();
        if (rounds)
            *rounds = waited;
        Bytes response;
        for (unsigned i = 0; i < Cc2530F256::debug_if_output_length(bytes.front()); ++i)
            response.push_back(receive());
        CHECK(device.pin_drive(2, 1) == Cc2530PinDrive::HiZ); // DD back to input
        return response;
    }
};

void test_wire_protocol() {
    {
        // One falling edge, or edges before RESET_N goes low, are a plain
        // reset: the firmware starts.
        Cc2530F256 device;
        load(device, counter_program());
        Wire wire{device};
        wire.entry(1);
        CHECK(!device.debug_if_active());
        wire.pulse();
        wire.pulse();
        wire.reset_n(false);
        const std::uint64_t held = device.registers().instructions;
        const Cc2530RunResult result = device.run(SIZE_MAX, device.registers().cycles + 1000);
        CHECK(result.reset_held);
        CHECK_EQ(device.registers().instructions, held); // no instruction in reset
        CHECK(device.diagnostics().reset_pin_cycles >= 1000);
        wire.reset_n(true);
        CHECK(!device.debug_if_active());
        advance(device, 1000);
        CHECK(device.registers().instructions > held);
    }
    Cc2530F256 device;
    load(device, counter_program());
    Wire wire{device};
    advance(device, 1000);
    wire.entry(2);
    CHECK(device.debug_if_active());
    CHECK(device.debug_if_halted());
    CHECK_EQ(device.pc(), 0x0000);
    CHECK_EQ(device.read_sfr(SLEEPSTA) & 0x18, 0x08); // RST = external reset

    unsigned rounds = 99;
    CHECK(wire.command({READ_STATUS}, &rounds) == Bytes{Halted});
    CHECK_EQ(rounds, 0u);
    CHECK(wire.command({GET_CHIP_ID}) == (Bytes{0xa5, device.peek_xdata(CHVER)}));
    CHECK(wire.command({0x52, 0x74, 0x3c}) == Bytes{0x3c}); // DEBUG_INSTR MOV A,#0x3c
    CHECK_EQ(device.registers().a, 0x3c);
    // STEP_INSTR: the instruction executes after the command byte, so the
    // chip first answers "not ready" and the debugger clocks 8 more bits.
    CHECK(wire.command({STEP_INSTR}, &rounds) == Bytes{0x3c}); // MOV SP,#0x60
    CHECK_EQ(rounds, 1u);
    CHECK(wire.command({GET_PC}) == (Bytes{0x00, 0x03}));
    CHECK_EQ(device.registers().sp, 0x60);
    CHECK(wire.command({WR_CONFIG, 0x22}) == Bytes{Halted});
    CHECK(wire.command({RD_CONFIG}) == Bytes{0x22});
    CHECK(wire.command({BURST_WRITE, 2, 0x12, 0x34}) == Bytes{Halted});
    CHECK_EQ(device.peek_xdata(DBGDATA), 0x34);

    // In debug mode P2.1/P2.2 belong to the interface: port writes do not
    // drive them; P2.0 stays a GPIO.
    CHECK(wire.command({0x53, 0x75, P2DIR, 0x07}) == Bytes{0x3c});
    CHECK(wire.command({0x53, 0x75, P2, 0x00}) == Bytes{0x3c});
    CHECK(device.pin_drive(2, 0) == Cc2530PinDrive::Low);
    CHECK(device.pin_drive(2, 1) == Cc2530PinDrive::HiZ);
    CHECK(device.pin_drive(2, 2) == Cc2530PinDrive::HiZ);
    CHECK(wire.command({READ_STATUS}) == Bytes{Halted});

    // The host port refuses to interleave with a command on the wire.
    for (int bit = 7; bit >= 4; --bit)
        wire.send_bit(((READ_STATUS >> bit) & 1u) != 0);
    CHECK(throws_logic([&] { device.debug_if_submit({READ_STATUS}); }));
    for (int bit = 3; bit >= 0; --bit)
        wire.send_bit(((READ_STATUS >> bit) & 1u) != 0);
    CHECK_EQ(wire.wait_ready(), 0u);
    CHECK_EQ(wire.receive(), Halted);
    CHECK(device.pin_drive(2, 1) == Cc2530PinDrive::HiZ);
    // ... and works again once the wire is idle.
    CHECK(accepted(device, {READ_STATUS}) == Bytes{Halted});

    // RESUME over the wire; a RESET_N pulse without DC edges ends debug
    // mode and the pads are GPIOs again.
    CHECK(wire.command({RESUME}) == Bytes{OscillatorStable});
    advance(device, 1000);
    CHECK(!device.debug_if_halted());
    wire.reset_n(false);
    wire.reset_n(true);
    CHECK(!device.debug_if_active());
    advance(device, 1000);
    CHECK(device.registers().instructions > 0);
}

// ---- programming ----

// Sums a 16-byte table in page 1 with MOVC, calls bank 5 (MOVC of a bank
// constant) and stores both results in XDATA 0x1000/0x1001.
Flash programmed_image() {
    Flash flash;
    flash.at(0x00000).op({0x02, 0x00, 0x40});         // LJMP 0x0040
    flash.at(0x00040).op({0x75, SP, 0x60,             // 0x0040 MOV SP,#0x60
                          0x90, 0x08, 0x00,           // 0x0043 MOV DPTR,#0x0800
                          0x7f, 0x10,                 // 0x0046 MOV R7,#16
                          0x7e, 0x00,                 // 0x0048 MOV R6,#0
                          0xe4,                       // 0x004a CLR A
                          0x93,                       // 0x004b MOVC A,@A+DPTR
                          0x2e,                       // 0x004c ADD A,R6
                          0xfe,                       // 0x004d MOV R6,A
                          0xa3,                       // 0x004e INC DPTR
                          0xdf, 0xf9,                 // 0x004f DJNZ R7,0x004a
                          0x8e, 0x30,                 // 0x0051 MOV 0x30,R6
                          0x75, FMAP, 0x05,           // 0x0053 MOV FMAP,#5
                          0x12, 0x80, 0x00,           // 0x0056 LCALL 0x8000
                          0xf5, 0x31,                 // 0x0059 MOV 0x31,A
                          0x90, 0x10, 0x00,           // 0x005b MOV DPTR,#0x1000
                          0xe5, 0x30,                 // 0x005e MOV A,0x30
                          0xf0,                       // 0x0060 MOVX @DPTR,A
                          0xa3,                       // 0x0061 INC DPTR
                          0xe5, 0x31,                 // 0x0062 MOV A,0x31
                          0xf0,                       // 0x0064 MOVX @DPTR,A
                          0x80, 0xfe});               // 0x0065 SJMP $
    flash.at(0x00800);
    for (unsigned i = 0; i < 16; ++i)
        flash.op({static_cast<std::uint8_t>(0x11 * (i + 1))});
    flash.at(0x28000).op({0x90, 0x81, 0x00, // bank 5: MOV DPTR,#0x8100
                          0xe4,             // CLR A
                          0x93,             // MOVC A,@A+DPTR
                          0x22});           // RET
    flash.at(0x28100).op({0xc3});
    flash.at(0x3f000).op({0xde, 0xad, 0xbe, 0xef}); // page 126 (bank 7)
    return flash;
}

Cc2530FlashImage image_of(const Flash &flash) {
    Cc2530FlashImage image;
    image.bytes = flash.bytes;
    image.programmed.assign(flash.bytes.size(), false);
    for (std::size_t i = 0; i < flash.bytes.size(); ++i)
        image.programmed[i] = flash.bytes[i] != 0xff;
    return image;
}

// A chip loaded with a locked, write-protected image of other content.
Flash locked_garbage() {
    Flash flash;
    for (std::uint32_t address = 0; address < 0x2000; ++address)
        flash.bytes[address] = static_cast<std::uint8_t>(address * 7 + 3);
    for (std::uint32_t i = 0; i < 16; ++i)
        flash.bytes[Cc2530F256::LockBitsStart + i] = 0x00; // every page protected
    flash.bytes[DbglockByte] = 0x7f;                       // and DBGLOCK = 0
    return flash;
}

void check_program_result(Cc2530F256 &device) {
    advance(device, 200000);
    std::uint8_t sum = 0;
    for (unsigned i = 0; i < 16; ++i)
        sum = static_cast<std::uint8_t>(sum + 0x11 * (i + 1));
    CHECK_EQ(device.pc(), 0x0065);
    CHECK_EQ(device.read_idata(0x30), sum);
    CHECK_EQ(device.read_idata(0x31), 0xc3);
    CHECK_EQ(device.peek_xdata(0x1000), sum);
    CHECK_EQ(device.peek_xdata(0x1001), 0xc3);
}

// The CC-Debugger flow (CHIP_ERASE, BURST_WRITE -> DBGDATA -> DMA -> SRAM,
// DMA -> FWDATA with FCTL.WRITE, DEBUG_INSTR MOVX read-back) produces the
// image in flash; the chip then runs it like a directly loaded one.
void test_programming_round_trip() {
    const Flash flash = programmed_image();
    Cc2530F256 direct;
    load(direct, flash);
    check_program_result(direct);

    Cc2530F256 device;
    load(device, locked_garbage());
    CHECK(device.debug_if_locked());
    const std::uint64_t debug_instructions = device.diagnostics().debug_instructions;
    const Cc2530DebugProgramResult result =
        mcs51emu::cc2530_debug_program(mcs51emu::cc2530_debug_if_link(device), image_of(flash));
    CHECK_EQ(result.chip_id, 0xa5);
    CHECK_EQ(result.chip_version, device.peek_xdata(CHVER));
    CHECK(result.was_locked);
    CHECK_EQ(result.pages, 4u); // pages 0, 1, 80 and 126
    CHECK_EQ(result.bytes_verified, 4u * 2048u);
    CHECK(result.end_cycle > result.start_cycle);
    CHECK(device.diagnostics().debug_instructions > debug_instructions);
    bool same = true;
    for (std::uint32_t address = 0; address < Cc2530F256::FlashSize; ++address)
        same = same && device.read_flash(address) == flash.bytes[address];
    CHECK(same);
    CHECK(!device.debug_if_active());
    CHECK(!device.debug_if_locked());
    check_program_result(device);
    CHECK_EQ(device.registers().a, direct.registers().a);
    CHECK_EQ(device.registers().sp, direct.registers().sp);
    CHECK_EQ(device.registers().fmap, direct.registers().fmap);

    // With DBGLOCK programmed (the lock-bit page is written last): the
    // firmware runs, and the interface is locked from the reset on.
    Cc2530DebugProgramOptions lock;
    lock.debug_lock = true;
    const Cc2530DebugProgramResult locked =
        mcs51emu::cc2530_debug_program(mcs51emu::cc2530_debug_if_link(device), image_of(flash), lock);
    CHECK(!locked.was_locked);
    CHECK_EQ(locked.pages, 5u);
    CHECK_EQ(device.read_flash(DbglockByte), 0x7f);
    CHECK(device.debug_if_locked());
    check_program_result(device);
    device.debug_if_enter();
    CHECK_EQ(status(device), Halted | DebugLocked);
    refused(device, {GET_PC}, Halted | DebugLocked);
    // Programming without a run leaves the chip halted in debug mode.
    Cc2530DebugProgramOptions stay;
    stay.run = false;
    stay.verify = false;
    const Cc2530DebugProgramResult again =
        mcs51emu::cc2530_debug_program(mcs51emu::cc2530_debug_if_link(device), image_of(flash), stay);
    CHECK(again.was_locked);
    CHECK_EQ(again.bytes_verified, 0u);
    CHECK(device.debug_if_halted());
    CHECK_EQ(device.read_flash(DbglockByte), 0xff);
    CHECK_EQ(device.read_flash(0x28100), 0xc3);
}

// The same programming flow with every command bit-banged on DC/DD and the
// entry sequence on RESET_N/DC: a debugger on the pads can program the chip.
void test_programming_over_pads() {
    Flash flash;
    flash.at(0x0000).op({0x90, 0x10, 0x00, // MOV DPTR,#0x1000
                         0x74, 0x5e,       // MOV A,#0x5e
                         0xf0,             // MOVX @DPTR,A
                         0x80, 0xfe});     // SJMP $
    Cc2530F256 device;
    load(device, spin_program());
    Wire wire{device};
    Cc2530DebugIfLink link;
    link.enter = [&] { wire.entry(2); };
    link.command = [&](const Bytes &bytes) {
        // The wire does not tell a refused command apart; the programmer
        // checks the status bytes it reads.
        return Cc2530DebugIfResponse{wire.command(bytes), true, device.registers().cycles};
    };
    link.wait = [&](std::uint64_t cycles) { advance(device, cycles); };
    link.reset = [&] {
        wire.reset_n(false);
        wire.reset_n(true);
    };
    const Cc2530DebugProgramResult result = mcs51emu::cc2530_debug_program(link, image_of(flash));
    CHECK_EQ(result.chip_id, 0xa5);
    CHECK_EQ(result.pages, 1u);
    CHECK_EQ(result.bytes_verified, 2048u);
    for (std::uint32_t address = 0; address < 0x0800; ++address)
        CHECK_EQ(device.read_flash(address), flash.bytes[address]);
    CHECK(!device.debug_if_active());
    advance(device, 10000);
    CHECK_EQ(device.peek_xdata(0x1000), 0x5e);
    CHECK_EQ(device.pc(), 0x0006);
}

} // namespace

int main() {
    try {
        test_encodings();
        test_entry_and_status();
        test_halt_step_resume();
        test_debug_session_mode_equivalence();
        test_hw_breakpoints();
        test_debug_instr();
        test_burst_write_dma();
        test_chip_erase();
        test_dbglock();
        test_config_effects();
        test_wire_protocol();
        test_programming_round_trip();
        test_programming_over_pads();
    } catch (const std::exception &error) {
        std::cerr << "unexpected exception: " << error.what() << '\n';
        ++failures;
    }
    return finish("cc2530 debug interface");
}
