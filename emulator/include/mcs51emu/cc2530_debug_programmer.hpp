#pragma once

// CC-Debugger-style flash programming through the CC2530 two-wire debug
// interface (SWRU191F 3.4: "The external host must initially send
// instructions using the DEBUG_INSTR debug command to perform the flash
// programming with the flash controller"). Every byte reaches the flash the
// way TI's programmers put it there: BURST_WRITE -> DBGDATA -> DMA (trigger
// DBG_BW) -> XDATA buffer, then DMA (trigger FLASH) -> FWDATA with
// FCTL.WRITE, all set up with DEBUG_INSTR; read-back uses DEBUG_INSTR MOVX.

#include "mcs51emu/cc2530f256.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace mcs51emu {

// What the programmer needs from the debugger hardware.
struct Cc2530DebugIfLink {
    // Debug-mode entry (3.1: RESET_N low, two DC falling edges, RESET_N
    // released); the CPU is halted at 0x0000 afterwards.
    std::function<void()> enter;
    // One complete Table 3-1 command and its response.
    std::function<Cc2530DebugIfResponse(const std::vector<std::uint8_t> &)> command;
    // Lets virtual time pass (cycles of the 32 MHz base) without a command.
    std::function<void(std::uint64_t cycles)> wait;
    // A RESET_N pulse without DC edges: debug mode ends, the firmware starts.
    std::function<void()> reset;
};

// Link to a stand-alone chip: debug_if_enter(), debug_if_command(), run()
// and set_reset_pin().
Cc2530DebugIfLink cc2530_debug_if_link(Cc2530F256 &chip);

struct Cc2530DebugProgramOptions {
    bool verify{true};      // read every programmed page back
    bool debug_lock{false}; // program DBGLOCK = 0 (flash 0x3ffff bit 7)
    bool run{true};         // finish with a RESET_N pulse (the firmware starts)
};

struct Cc2530DebugProgramResult {
    std::uint8_t chip_id{};
    std::uint8_t chip_version{};
    bool was_locked{};        // DEBUG_LOCKED before the chip erase
    std::size_t pages{};      // 2 KiB pages written (pages that are all 0xff are skipped)
    std::size_t bytes_verified{};
    std::size_t commands{};   // debug commands issued
    std::uint64_t start_cycle{};
    std::uint64_t end_cycle{}; // response time of the last command
};

// Erases the chip (CHIP_ERASE), re-enters debug mode so the cleared lock
// bits take effect, checks GET_CHIP_ID, then writes every page of `image`
// that is not all 0xff in ascending order (the lock-bit page last), verifies
// it and resets the chip. Throws std::runtime_error naming the failing step.
Cc2530DebugProgramResult cc2530_debug_program(const Cc2530DebugIfLink &link,
                                              const Cc2530FlashImage &image,
                                              const Cc2530DebugProgramOptions &options = {});

} // namespace mcs51emu
