// CC2530F256 flash fault model tests: SWRU191F 6.2.2 multiple-write limits,
// flash operations cut by a reset or a power cut, power-cut injection, the
// per-page / per-operation instrumentation and the flash image round trip.
//
// Every scenario runs hand-encoded MCS-51 machine code that drives FCTL,
// FADDRH/L and FWDATA (from SRAM code, from flash code with the CPU stall of
// SWRU191F 6.3.1, or through DMA trigger FLASH) and checks flash, SRAM, SFRs
// and the documented diagnostics. Limits and timing come from SWRU191F 6.2.2
// and 2.2.3 (page erase 20 ms = 640000 cycles, word write 20 us = 640
// cycles). The outcome of an interrupted operation, the nondeterministic
// word value and the SRAM content after a power cut are model-defined
// (SWRU191F leaves them open); the expected values follow the documented
// rules (docs/cc2530.md "Flash fault injection") through fault_draw(), and a
// few golden values pin fault_draw() itself so seeded results stay
// reproducible across versions.

#include "cc2530_flash.hpp"
#include "cc2530_periph_support.hpp"

#include <bit>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cc2530test;
using mcs51emu::Cc2530FaultAction;
using mcs51emu::Cc2530FlashCut;
using mcs51emu::Cc2530FlashFaultConfig;
using mcs51emu::Cc2530FlashInterrupted;
using mcs51emu::Cc2530FlashLimitPolicy;
using mcs51emu::Cc2530FlashLogEntry;
using mcs51emu::Cc2530FlashOpFilter;
using mcs51emu::Cc2530FlashOpKind;
using mcs51emu::Cc2530FlashOutcome;
using mcs51emu::Cc2530FlashTrigger;
using mcs51emu::Cc2530ResetCause;
using mcs51emu::cc2530::fault_draw;

namespace {

// ---- registers ----
constexpr std::uint8_t SP = 0x81;
constexpr std::uint8_t PCON = 0x87;
constexpr std::uint8_t SLEEPSTA = 0x9d;
constexpr std::uint8_t SLEEPCMD = 0xbe;
constexpr std::uint8_t MEMCTR = 0xc7;
constexpr std::uint8_t DMAIRQ = 0xd1;
constexpr std::uint8_t DMA0CFGL = 0xd4;
constexpr std::uint8_t DMA0CFGH = 0xd5;
constexpr std::uint8_t DMAARM = 0xd6;
constexpr std::uint8_t AccBit7 = 0xe7;

constexpr std::uint16_t FCTL = 0x6270;
constexpr std::uint16_t FADDRL = 0x6271;
constexpr std::uint16_t FADDRH = 0x6272;
constexpr std::uint16_t FWDATA = 0x6273;

constexpr std::uint16_t Desc0 = 0x0f00;    // DMA channel 0 descriptor (XDATA)
constexpr std::uint16_t SramCode = 0x9000; // CODE address of SRAM 0x1000 (XMAP)

// SWRU191F 2.2.3 flash timing in 32 MHz cycles.
constexpr std::uint64_t EraseCycles = 640'000;
constexpr std::uint64_t WordCycles = 640;

constexpr std::uint32_t PageBytes = 2048;
constexpr std::uint32_t Page6 = 6 * PageBytes; // 0x3000, word address 0x0C00

// Documented draw index of the nondeterministic word value and SRAM stream.
constexpr std::uint64_t NondeterministicIndex = 32;
constexpr std::uint64_t SramStream = 0x5352414d'00000000ull;

// ---- program builder ----
struct Asm {
    std::uint16_t origin;
    std::vector<std::uint8_t> code;

    explicit Asm(std::uint16_t at) : origin(at) {}
    std::uint16_t here() const { return static_cast<std::uint16_t>(origin + code.size()); }
    Asm &op(std::initializer_list<std::uint8_t> bytes) {
        // push_back, not insert(): GCC 13 -O2 reports a false
        // -Wstringop-overflow for the inlined range insert.
        for (const std::uint8_t byte : bytes)
            code.push_back(byte);
        return *this;
    }
    Asm &op(const std::vector<std::uint8_t> &bytes) {
        code.insert(code.end(), bytes.begin(), bytes.end());
        return *this;
    }
    Asm &mov(std::uint8_t dir, std::uint8_t value) { return op({0x75, dir, value}); }
    Asm &movr(unsigned r, std::uint8_t value) {
        return op({static_cast<std::uint8_t>(0x78 + r), value});
    }
    Asm &dptr(std::uint16_t address) { return op(mov_dptr(address)); }
    Asm &xwrite(std::uint16_t address, std::uint8_t value) { return op(movx_write(address, value)); }
    Asm &xread_to(std::uint16_t address, std::uint8_t dir) { return op(movx_read_to(address, dir)); }
    Asm &xstore(std::uint16_t address, const std::vector<std::uint8_t> &bytes) {
        dptr(address);
        for (const std::uint8_t byte : bytes)
            op({0x74, byte, 0xf0, 0xa3}); // MOV A,#b; MOVX @DPTR,A; INC DPTR
        return *this;
    }
    // MOV A,#b; MOVX @DPTR,A per byte.
    Asm &xbytes(const std::vector<std::uint8_t> &bytes) {
        for (const std::uint8_t byte : bytes)
            op({0x74, byte, 0xf0});
        return *this;
    }
    // MOV DPTR,#code_address; CLR A; MOVC A,@A+DPTR; MOV dir,A
    Asm &movc_to(std::uint16_t code_address, std::uint8_t dir) {
        return dptr(code_address).op({0xe4, 0x93, 0xf5, dir});
    }
    Asm &nops(unsigned count) {
        code.insert(code.end(), count, 0x00);
        return *this;
    }
    Asm &halt() { return op({0x80, 0xfe}); } // SJMP $
    Asm &ljmp(std::uint16_t address) { return op({0x02, hi(address), lo(address)}); }
    std::uint8_t rel(std::uint16_t target, unsigned length) const {
        const int offset = static_cast<int>(target) - static_cast<int>(here() + length);
        if (offset < -128 || offset > 127)
            throw std::logic_error("branch out of range");
        return static_cast<std::uint8_t>(offset);
    }
    Asm &djnz(unsigned r, std::uint16_t target) {
        const std::uint8_t offset = rel(target, 2);
        return op({static_cast<std::uint8_t>(0xd8 + r), offset});
    }
    // MOV DPTR,#FCTL; L: MOVX A,@DPTR; JB ACC.7,L (wait for FCTL.BUSY = 0).
    Asm &wait_idle() {
        dptr(FCTL);
        const std::uint16_t loop = here();
        op({0xe0});
        const std::uint8_t offset = rel(loop, 3);
        return op({0x20, AccBit7, offset});
    }
    // One complete word write (SWRU191F 6.2.1) at `word_address`, data LSB
    // first, waiting until the controller is idle again.
    Asm &write_word(std::uint16_t word_address, std::uint32_t value) {
        xwrite(FADDRH, hi(word_address)).xwrite(FADDRL, lo(word_address)).xwrite(FCTL, 0x06);
        dptr(FWDATA).xbytes(le(value));
        return wait_idle();
    }
    // Page erase (6.3) of the page holding `word_address`, waiting for it.
    Asm &erase_page(std::uint16_t word_address) {
        xwrite(FADDRH, hi(word_address)).xwrite(FCTL, 0x05);
        return wait_idle();
    }
    static std::vector<std::uint8_t> le(std::uint32_t value) {
        return {static_cast<std::uint8_t>(value), static_cast<std::uint8_t>(value >> 8),
                static_cast<std::uint8_t>(value >> 16), static_cast<std::uint8_t>(value >> 24)};
    }
};

// DMA descriptor (SWRU191F Table 8-2): single mode, trigger FLASH (18),
// fixed destination FWDATA, high priority.
std::vector<std::uint8_t> flash_dma_descriptor(std::uint16_t src, std::uint16_t len) {
    return {hi(src), lo(src), hi(FWDATA), lo(FWDATA), static_cast<std::uint8_t>(len >> 8 & 0x1f),
            lo(len), 18, 0x42};
}

// Main program at 0x0100 after LJMP from the reset vector.
Flash program(const Asm &main) {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(main.origin).op(main.code);
    return flash;
}

// Main jumping to SRAM code at 0x9000 (MEMCTR.XMAP = 1).
Asm sram_entry() {
    Asm main(0x0100);
    main.mov(SP, 0x30).mov(MEMCTR, 0x08).ljmp(SramCode);
    return main;
}

void place_sram(Cc2530F256 &device, const Asm &code) {
    for (std::size_t i = 0; i < code.code.size(); ++i)
        device.write_sram(static_cast<std::uint16_t>(code.origin - 0x8000 + i), code.code[i]);
}

std::vector<std::uint8_t> filled(std::size_t count, std::uint8_t value) {
    return std::vector<std::uint8_t>(count, value);
}

std::vector<std::uint8_t> flash_bytes(const Cc2530F256 &device, std::uint32_t address,
                                      std::size_t count) {
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i < count; ++i)
        bytes.push_back(device.read_flash(static_cast<std::uint32_t>(address + i)));
    return bytes;
}

std::uint32_t flash_word(const Cc2530F256 &device, std::uint32_t address) {
    return static_cast<std::uint32_t>(device.read_flash(address)) |
           static_cast<std::uint32_t>(device.read_flash(address + 1)) << 8 |
           static_cast<std::uint32_t>(device.read_flash(address + 2)) << 16 |
           static_cast<std::uint32_t>(device.read_flash(address + 3)) << 24;
}

void fail(const std::source_location &loc, const std::string &message) {
    std::cerr << loc.file_name() << ':' << loc.line() << ": " << message << '\n';
    ++failures;
}

void expect(std::uint64_t actual, std::uint64_t expected, const char *what,
            const std::source_location loc = std::source_location::current()) {
    if (actual != expected)
        fail(loc, std::string(what) + ": got " + std::to_string(actual) + ", want " +
                      std::to_string(expected));
}

void expect_bytes(const std::vector<std::uint8_t> &actual,
                  const std::vector<std::uint8_t> &expected, const char *what,
                  const std::source_location loc = std::source_location::current()) {
    if (actual == expected)
        return;
    std::size_t i = 0;
    while (i < actual.size() && i < expected.size() && actual[i] == expected[i])
        ++i;
    fail(loc, std::string(what) + ": first difference at byte " + std::to_string(i));
}

std::uint64_t run_to_pc(Cc2530F256 &device, std::uint16_t pc,
                        const std::source_location loc = std::source_location::current()) {
    device.set_breakpoint(pc);
    const Cc2530RunResult result = device.run(50'000'000);
    device.set_breakpoint(std::nullopt);
    if (!result.error.empty())
        fail(loc, "run error: " + result.error);
    if (!result.break_hit || result.pc != pc)
        fail(loc, "breakpoint not reached");
    return device.registers().cycles;
}

// Runs to exactly `cycle` (CPU stalled, in a NOP sled or asleep there).
void run_to(Cc2530F256 &device, std::uint64_t cycle,
            const std::source_location loc = std::source_location::current()) {
    const Cc2530RunResult result = device.run(50'000'000, cycle);
    if (!result.error.empty())
        fail(loc, "run error: " + result.error);
    if (device.registers().cycles != cycle)
        fail(loc, "stopped at cycle " + std::to_string(device.registers().cycles) +
                      " instead of " + std::to_string(cycle));
}

unsigned rst_field(const Cc2530F256 &device) { return (device.read_sfr(SLEEPSTA) >> 3) & 3u; }

Cc2530FlashFaultConfig fault_config(Cc2530FlashInterrupted interrupted, std::uint64_t seed,
                                    Cc2530FlashLimitPolicy policy = Cc2530FlashLimitPolicy::Report) {
    Cc2530FlashFaultConfig config;
    config.interrupted = interrupted;
    config.limit_policy = policy;
    config.seed = seed;
    config.log = true;
    return config;
}

// ---- reference rules (docs/cc2530.md "Flash fault injection") ----

// Partial page erase: bit b (8 * byte offset + bit) of the page goes from 0
// to 1 when fault_draw(seed, serial, b) mod 640000 < elapsed.
std::vector<std::uint8_t> partial_erase(std::vector<std::uint8_t> page, std::uint64_t seed,
                                        std::uint64_t serial, std::uint64_t elapsed) {
    for (std::uint32_t i = 0; i < page.size(); ++i)
        for (unsigned bit = 0; bit < 8; ++bit)
            if (fault_draw(seed, serial, i * 8u + bit) % EraseCycles < elapsed)
                page[i] = static_cast<std::uint8_t>(page[i] | (1u << bit));
    return page;
}

// Partial word write: bit b of the word changes from the old to the
// programmed value when fault_draw(seed, serial, b) mod 640 < elapsed.
std::uint32_t partial_word(std::uint32_t old, std::uint32_t target, std::uint64_t seed,
                           std::uint64_t serial, std::uint64_t elapsed) {
    std::uint32_t result = old;
    for (unsigned bit = 0; bit < 32; ++bit) {
        const std::uint32_t mask = 1u << bit;
        if (((old ^ target) & mask) && fault_draw(seed, serial, bit) % WordCycles < elapsed)
            result = (result & ~mask) | (target & mask);
    }
    return result;
}

std::vector<std::uint8_t> sram_after_power_on(std::uint64_t seed, std::uint64_t power_ons) {
    std::vector<std::uint8_t> sram;
    for (std::uint32_t i = 0; i < Cc2530F256::SramSize; i += 8) {
        const std::uint64_t value = fault_draw(seed, SramStream + power_ons, i / 8);
        for (unsigned j = 0; j < 8; ++j)
            sram.push_back(static_cast<std::uint8_t>(value >> (8 * j)));
    }
    return sram;
}

std::vector<std::uint8_t> sram(const Cc2530F256 &device) {
    std::vector<std::uint8_t> bytes;
    for (std::uint32_t address = 0; address < Cc2530F256::SramSize; ++address)
        bytes.push_back(device.peek_xdata(static_cast<std::uint16_t>(address)));
    return bytes;
}

std::uint64_t power_ons(const Cc2530F256 &device) {
    return device.diagnostics().resets_by_cause[static_cast<unsigned>(Cc2530ResetCause::PowerOn)];
}

// ======================================================================
// fault_draw (model-defined generator, pinned)
// ======================================================================

// splitmix64 mixing: fixed values so seeded outcomes stay reproducible.
void test_fault_draw_golden() {
    expect(fault_draw(0, 0, 0), 0x238275bc38fcbe91ull, "draw(0,0,0)");
    expect(fault_draw(1, 2, 3), 0xe6f19ca578bd2189ull, "draw(1,2,3)");
    expect(fault_draw(0x1234, 1, 32), 0xee1521e4bdd0aa68ull, "draw(0x1234,1,32)");
    CHECK(fault_draw(5, 1, 0) != fault_draw(6, 1, 0));
    CHECK(fault_draw(5, 1, 0) != fault_draw(5, 2, 0));
    CHECK(fault_draw(5, 1, 0) != fault_draw(5, 1, 1));
}

// ======================================================================
// SWRU191F 6.2.2 multiple-write limits
// ======================================================================

// Bit limit: three writes of FE FF FF FF to word 0x3000 write a 0 to bit 0
// three times (allowed: two). Word limit: nine writes to word 0x3004, each
// zeroing a new bit (allowed: eight writes). CPU-driven from SRAM code.
struct LimitProgram {
    Asm code{SramCode};
    std::uint16_t after_two_zero_writes{};
    std::uint16_t after_eight_word_writes{};
    std::uint16_t end{};
};

LimitProgram limit_program() {
    LimitProgram p;
    p.code.write_word(0x0c00, 0xfffffffe).write_word(0x0c00, 0xfffffffe);
    p.after_two_zero_writes = p.code.here();
    p.code.write_word(0x0c00, 0xfffffffe);
    for (unsigned k = 0; k < 8; ++k)
        p.code.write_word(0x0c01, ~(1u << k));
    p.after_eight_word_writes = p.code.here();
    p.code.write_word(0x0c01, ~(1u << 8));
    p.end = p.code.here();
    p.code.halt();
    return p;
}

void limit_case(Cc2530FlashLimitPolicy policy) {
    const std::uint64_t seed = 0x5eed;
    const LimitProgram p = limit_program();
    Cc2530F256 device;
    device.set_flash_fault_config(fault_config(Cc2530FlashInterrupted::Abandon, seed, policy));
    load(device, program(sram_entry()));
    place_sram(device, p.code);

    run_to_pc(device, p.after_two_zero_writes);
    expect(device.diagnostics().flash_bit_limit_violations, 0, "two 0-writes to a bit: allowed");
    expect(flash_word(device, Page6), 0xfffffffe, "two writes: deterministic AND");
    run_to_pc(device, p.after_eight_word_writes);
    expect(device.diagnostics().flash_bit_limit_violations, 1, "third 0-write to bit 0");
    expect(device.diagnostics().flash_word_limit_violations, 0, "eight writes to a word: allowed");
    expect(flash_word(device, Page6 + 4), 0xffffff00, "eight writes");
    run_to_pc(device, p.end);
    const auto &d = device.diagnostics();
    expect(d.flash_bit_limit_violations, 1, "bit violations");
    expect(d.flash_word_limit_violations, 1, "ninth write to a word");
    expect(d.flash_page_limit_violations, 0, "page violations");
    expect(d.flash_page_writes[6], 12, "word writes to page 6");

    // Operation serials: the 3rd write is serial 3, the 9th write to the
    // second word serial 12.
    if (policy == Cc2530FlashLimitPolicy::Report) {
        expect(flash_word(device, Page6), 0xfffffffe, "report: AND kept");
        expect(flash_word(device, Page6 + 4), 0xfffffe00, "report: AND kept");
    } else {
        expect(flash_word(device, Page6),
               static_cast<std::uint32_t>(fault_draw(seed, 3, NondeterministicIndex)),
               "nondeterministic word value");
        expect(flash_word(device, Page6 + 4),
               static_cast<std::uint32_t>(fault_draw(seed, 12, NondeterministicIndex)),
               "nondeterministic word value");
        // Pinned: seed 0x5eed, serial 3.
        expect(flash_word(device, Page6), 0x1d3432fau, "golden nondeterministic value");
    }
    const auto &log = device.flash_log();
    expect(log.size(), 12, "log entries");
    if (log.size() == 12) {
        expect(log[1].violations, 0, "second write clean");
        expect(log[2].violations, mcs51emu::Cc2530FlashBitLimit, "third write: bit limit");
        expect(log[10].violations, 0, "eighth word write clean");
        expect(log[11].violations, mcs51emu::Cc2530FlashWordLimit, "ninth write: word limit");
        expect(static_cast<unsigned>(log[11].outcome),
               static_cast<unsigned>(Cc2530FlashOutcome::Completed), "completed");
        expect(log[11].address, Page6 + 4, "address");
        expect(log[11].page, 6, "page");
    }
}

// Page limit: 1024 word writes of FF FF FF FF (two per word, no 0 written)
// are allowed; the 1025th (00 00 00 00 to word 0) breaks only the page limit.
void page_limit_case(Cc2530FlashLimitPolicy policy) {
    const std::uint64_t seed = 77;
    Asm code(SramCode);
    code.movr(5, 2);
    const std::uint16_t pass = code.here();
    code.xwrite(FADDRH, 0x0c).xwrite(FADDRL, 0x00).movr(6, 2);
    const std::uint16_t outer = code.here();
    code.movr(7, 0);
    const std::uint16_t inner = code.here();
    // FADDR increments after each word (model-defined, cc2530_flash.cpp).
    code.xwrite(FCTL, 0x06).dptr(FWDATA).xbytes({0xff, 0xff, 0xff, 0xff}).wait_idle();
    code.djnz(7, inner).djnz(6, outer).djnz(5, pass);
    const std::uint16_t after_1024 = code.here();
    code.write_word(0x0c00, 0x00000000);
    const std::uint16_t end = code.here();
    code.halt();

    Cc2530F256 device;
    device.set_flash_fault_config(fault_config(Cc2530FlashInterrupted::Abandon, seed, policy));
    load(device, program(sram_entry()));
    place_sram(device, code);
    run_to_pc(device, after_1024);
    expect(device.diagnostics().flash_page_writes[6], 1024, "1024 writes");
    expect(device.diagnostics().flash_page_limit_violations, 0, "1024 writes: allowed");
    run_to_pc(device, end);
    const auto &d = device.diagnostics();
    expect(d.flash_page_writes[6], 1025, "1025 writes");
    expect(d.flash_page_limit_violations, 1, "1025th write: page limit");
    expect(d.flash_word_limit_violations, 0, "three writes to word 0");
    expect(d.flash_bit_limit_violations, 0, "one 0-write per bit");
    expect(device.flash_log().back().violations, mcs51emu::Cc2530FlashPageLimit, "log violations");
    const std::uint32_t expected =
        policy == Cc2530FlashLimitPolicy::Report
            ? 0u
            : static_cast<std::uint32_t>(fault_draw(seed, 1025, NondeterministicIndex));
    expect(flash_word(device, Page6), expected, "word 0 after the 1025th write");
    expect_bytes(flash_bytes(device, Page6 + 4, PageBytes - 4), filled(PageBytes - 4, 0xff),
                 "rest of page 6");
}

void test_write_limits() {
    limit_case(Cc2530FlashLimitPolicy::Report);
    limit_case(Cc2530FlashLimitPolicy::Nondeterministic);
    page_limit_case(Cc2530FlashLimitPolicy::Report);
    page_limit_case(Cc2530FlashLimitPolicy::Nondeterministic);
}

// 6.2.2 counts writes "subsequent to the last page erase": a page erase
// (FCTL.ERASE) and an erase recorded by another path (record_erase, e.g. a
// debug CHIP_ERASE) restart the tracking.
void test_erase_restarts_tracking() {
    Asm code(SramCode);
    code.write_word(0x0c00, 0xfffffffe).write_word(0x0c00, 0xfffffffe).erase_page(0x0c00);
    code.write_word(0x0c00, 0xfffffffe).write_word(0x0c00, 0xfffffffe);
    const std::uint16_t mid = code.here();
    code.write_word(0x0c00, 0xfffffffe).write_word(0x0c00, 0xfffffffe);
    const std::uint16_t end = code.here();
    code.halt();

    Cc2530F256 device;
    device.set_flash_fault_config(fault_config(Cc2530FlashInterrupted::Abandon, 1));
    load(device, program(sram_entry()));
    place_sram(device, code);
    run_to_pc(device, mid);
    expect(device.diagnostics().flash_bit_limit_violations, 0, "erase restarted the tracking");
    expect(device.diagnostics().flash_page_erases[6], 1, "one erase of page 6");
    device.peripherals().flash->record_erase(6, 1);
    run_to_pc(device, end);
    expect(device.diagnostics().flash_bit_limit_violations, 0, "record_erase restarted it");
    expect(device.diagnostics().flash_page_erases[6], 2, "recorded erase counted");
    const auto &log = device.flash_log();
    expect(log.size(), 8, "log: 2 writes, erase, 2 writes, recorded erase, 2 writes");
    if (log.size() == 8) {
        expect(static_cast<unsigned>(log[2].op), static_cast<unsigned>(Cc2530FlashOpKind::PageErase),
               "page erase entry");
        expect(log[2].elapsed_cycles, EraseCycles, "full erase time");
        expect(log[2].changed_bits, 1, "one bit erased");
        expect(static_cast<unsigned>(log[5].op), static_cast<unsigned>(Cc2530FlashOpKind::ChipErase),
               "recorded erase entry");
    }
    device.peripherals().flash->record_erase(120, 8);
    expect(device.diagnostics().flash_page_erases[127], 1, "range recorded");
    bool threw = false;
    try {
        device.peripherals().flash->record_erase(127, 2);
    } catch (const std::out_of_range &) {
        threw = true;
    }
    CHECK(threw);
}

// ======================================================================
// Interrupted page erase (CPU stalled in flash code, 6.3.1)
// ======================================================================

// main: SP, then a page 6 erase (FADDRH = 0x0C, FCTL = ERASE | CM=01) from
// flash, so the CPU stalls and virtual time reaches any cycle exactly; then
// the first page 6 byte via MOVC to IDATA 0x40 and a halt.
Asm erase_program(std::uint16_t &erase) {
    Asm main(0x0100);
    main.mov(SP, 0x30).xwrite(FADDRH, 0x0c).dptr(FCTL).op({0x74, 0x05});
    erase = main.here();
    main.op({0xf0});
    main.movc_to(0x3000, 0x40).halt();
    return main;
}

std::vector<std::uint8_t> page6_pattern() {
    std::vector<std::uint8_t> page(PageBytes);
    for (std::uint32_t i = 0; i < PageBytes; ++i)
        page[i] = static_cast<std::uint8_t>(i % 3 == 0 ? 0x00 : i & 0x5a);
    return page;
}

Flash erase_flash(const Asm &main) {
    Flash flash = program(main);
    flash.at(Page6).op(page6_pattern());
    return flash;
}

unsigned zero_bits(const std::vector<std::uint8_t> &bytes) {
    unsigned count = 0;
    for (const std::uint8_t byte : bytes)
        count += static_cast<unsigned>(std::popcount(static_cast<std::uint8_t>(~byte)));
    return count;
}

// Cuts the erase `elapsed` cycles after it started, by reset() or
// power_cut(); returns page 6.
std::vector<std::uint8_t> cut_erase(Cc2530FlashInterrupted mode, std::uint64_t seed,
                                    std::uint64_t elapsed, Cc2530FlashCut cut,
                                    Cc2530F256 &device) {
    std::uint16_t erase = 0;
    device.set_flash_fault_config(fault_config(mode, seed));
    load(device, erase_flash(erase_program(erase)));
    const std::uint64_t t = run_to_pc(device, erase);
    run_to(device, t + elapsed);
    expect(device.peek_xdata(FCTL), 0x85, "erasing (BUSY | CM=01 | ERASE)");
    if (cut == Cc2530FlashCut::Reset)
        device.reset();
    else
        device.power_cut();
    expect(device.peek_xdata(FCTL), 0x04, "FCTL reset value after the cut");
    const auto &log = device.flash_log();
    expect(log.size(), 1, "one log entry");
    if (!log.empty()) {
        expect(log[0].cycle, t + elapsed, "log cycle");
        expect(log[0].elapsed_cycles, elapsed, "log elapsed");
        expect(static_cast<unsigned>(log[0].cut), static_cast<unsigned>(cut), "log cut");
        expect(log[0].page, 6, "log page");
        expect(log[0].address, Page6, "log address");
    }
    expect(device.diagnostics().flash_interrupted_erases, 1, "interrupted erases");
    expect(rst_field(device), cut == Cc2530FlashCut::Reset ? 1u : 0u,
           "SLEEPSTA.RST: 01 external reset, 00 power-on (SWRU191F 4.3)");
    return flash_bytes(device, Page6, PageBytes);
}

void test_interrupted_erase() {
    const std::vector<std::uint8_t> original = page6_pattern();
    const unsigned original_zeros = zero_bits(original);
    for (const Cc2530FlashCut cut : {Cc2530FlashCut::Reset, Cc2530FlashCut::PowerCut}) {
        std::vector<std::uint8_t> previous = original;
        for (const std::uint64_t elapsed : {EraseCycles / 4, EraseCycles / 2, 3 * EraseCycles / 4}) {
            {
                Cc2530F256 device;
                const auto page =
                    cut_erase(Cc2530FlashInterrupted::Abandon, 9, elapsed, cut, device);
                expect_bytes(page, original, "abandon: page unchanged");
                expect(device.diagnostics().flash_page_erases[6], 0, "abandon: no erase");
                expect(static_cast<unsigned>(device.flash_log().at(0).outcome),
                       static_cast<unsigned>(Cc2530FlashOutcome::Abandoned), "abandoned");
            }
            {
                Cc2530F256 device;
                const auto page =
                    cut_erase(Cc2530FlashInterrupted::Partial, 9, elapsed, cut, device);
                expect_bytes(page, partial_erase(original, 9, 1, elapsed), "partial erase");
                const unsigned erased = original_zeros - zero_bits(page);
                expect(device.flash_log().at(0).changed_bits, erased, "changed bits");
                // The fraction of erased bits follows elapsed / 20 ms.
                const double fraction = static_cast<double>(erased) / original_zeros;
                const double want = static_cast<double>(elapsed) / EraseCycles;
                CHECK(fraction > want - 0.05 && fraction < want + 0.05);
                // Monotonic in the elapsed time (one threshold per bit).
                for (std::uint32_t i = 0; i < PageBytes; ++i)
                    CHECK((previous[i] & ~page[i]) == 0);
                previous = page;
                expect(device.diagnostics().flash_page_erases[6], 0, "partial: no erase");
                expect(static_cast<unsigned>(device.flash_log().at(0).outcome),
                       static_cast<unsigned>(Cc2530FlashOutcome::Partial), "partial");
                expect(device.read_code(0x3000), page[0], "CODE view refreshed");
            }
            {
                Cc2530F256 device;
                const auto page =
                    cut_erase(Cc2530FlashInterrupted::Complete, 9, elapsed, cut, device);
                expect_bytes(page, filled(PageBytes, 0xff), "complete: erased");
                expect(device.diagnostics().flash_page_erases[6], 1, "complete: erase counted");
                expect(device.flash_log().at(0).changed_bits, original_zeros, "changed bits");
                expect(static_cast<unsigned>(device.flash_log().at(0).outcome),
                       static_cast<unsigned>(Cc2530FlashOutcome::Forced), "forced");
            }
        }
    }
    // Pinned partial result: seed 9, serial 1, half the erase time.
    Cc2530F256 device;
    const auto page = cut_erase(Cc2530FlashInterrupted::Partial, 9, EraseCycles / 2,
                                Cc2530FlashCut::PowerCut, device);
    expect_bytes(std::vector<std::uint8_t>(page.begin(), page.begin() + 8),
                 {0x2c, 0x82, 0xa2, 0x9e, 0x2b, 0x9d, 0xf7, 0xea}, "golden first bytes");
    expect(device.flash_log().at(0).changed_bits, 6799, "golden changed bits");
}

// Same seed, same result; another seed, another result; a scheduled cut at
// the same cycle gives the same result as power_cut() there.
void test_interrupted_erase_determinism() {
    Cc2530F256 a, b, c;
    const auto first = cut_erase(Cc2530FlashInterrupted::Partial, 1234, EraseCycles / 2,
                                 Cc2530FlashCut::PowerCut, a);
    const auto second = cut_erase(Cc2530FlashInterrupted::Partial, 1234, EraseCycles / 2,
                                  Cc2530FlashCut::PowerCut, b);
    const auto other = cut_erase(Cc2530FlashInterrupted::Partial, 4321, EraseCycles / 2,
                                 Cc2530FlashCut::PowerCut, c);
    expect_bytes(first, second, "same seed");
    CHECK(first != other);

    std::uint16_t erase = 0;
    Cc2530F256 device;
    device.set_flash_fault_config(fault_config(Cc2530FlashInterrupted::Partial, 1234));
    load(device, erase_flash(erase_program(erase)));
    const std::uint64_t t = run_to_pc(device, erase);
    device.schedule_power_cut(t + EraseCycles / 2);
    expect(device.pending_power_faults(), 1, "scheduled");
    run_to(device, t + EraseCycles / 2);
    expect(device.pending_power_faults(), 0, "fired");
    expect(device.diagnostics().power_cuts, 1, "power cuts");
    expect(device.diagnostics().last_power_cut_cycle, t + EraseCycles / 2, "cut cycle");
    expect(rst_field(device), 0, "power-on reset cause");
    expect(device.pc(), 0, "at the reset vector");
    expect_bytes(flash_bytes(device, Page6, PageBytes), first, "scheduled cut = power_cut()");
}

// ======================================================================
// Interrupted word write: CPU from SRAM and DMA from flash code
// ======================================================================

// SRAM code writes 00 00 00 00 to word 0x3000 (FF FF FF FF) and runs into a
// NOP sled; the word write starts with the fourth FWDATA write at `fourth`.
std::uint32_t cut_cpu_word(Cc2530FlashInterrupted mode, std::uint64_t seed, std::uint64_t elapsed,
                           Cc2530FlashCut cut, Cc2530F256 &device) {
    Asm code(SramCode);
    code.xwrite(FADDRH, 0x0c).xwrite(FADDRL, 0x00).xwrite(FCTL, 0x06);
    code.dptr(FWDATA).xbytes({0x00, 0x00, 0x00}).op({0x74, 0x00});
    const std::uint16_t fourth = code.here();
    code.op({0xf0}).nops(1500).halt();
    device.set_flash_fault_config(fault_config(mode, seed));
    load(device, program(sram_entry()));
    place_sram(device, code);
    const std::uint64_t t = run_to_pc(device, fourth);
    run_to(device, t + elapsed);
    expect(device.peek_xdata(FCTL), 0x86, "programming (BUSY | CM=01 | WRITE)");
    if (cut == Cc2530FlashCut::Reset)
        device.reset();
    else
        device.power_cut();
    expect(device.diagnostics().flash_interrupted_writes, 1, "interrupted writes");
    const auto &log = device.flash_log();
    expect(log.size(), 1, "one log entry");
    if (!log.empty()) {
        expect(log[0].elapsed_cycles, elapsed, "log elapsed");
        expect(log[0].cycle, t + elapsed, "log cycle");
        expect(static_cast<unsigned>(log[0].op), static_cast<unsigned>(Cc2530FlashOpKind::WordWrite),
               "word write");
    }
    expect(rst_field(device), cut == Cc2530FlashCut::Reset ? 1u : 0u, "SLEEPSTA.RST");
    return flash_word(device, Page6);
}

void test_interrupted_cpu_word_write() {
    for (const Cc2530FlashCut cut : {Cc2530FlashCut::Reset, Cc2530FlashCut::PowerCut}) {
        for (const std::uint64_t elapsed : {WordCycles / 4, WordCycles / 2, 3 * WordCycles / 4}) {
            Cc2530F256 abandon, partial, complete;
            expect(cut_cpu_word(Cc2530FlashInterrupted::Abandon, 3, elapsed, cut, abandon),
                   0xffffffffu, "abandon: word unchanged");
            expect(abandon.diagnostics().flash_page_writes[6], 0, "abandon: not a write");
            const std::uint32_t word =
                cut_cpu_word(Cc2530FlashInterrupted::Partial, 3, elapsed, cut, partial);
            expect(word, partial_word(0xffffffffu, 0, 3, 1, elapsed), "partial word");
            expect(partial.flash_log().at(0).changed_bits,
                   static_cast<std::uint64_t>(std::popcount(~word)), "changed bits");
            expect(partial.diagnostics().flash_page_writes[6], 1, "partial: counts as a write");
            expect(cut_cpu_word(Cc2530FlashInterrupted::Complete, 3, elapsed, cut, complete), 0u,
                   "complete: programmed");
        }
    }
    Cc2530F256 device;
    expect(cut_cpu_word(Cc2530FlashInterrupted::Partial, 3, WordCycles / 2,
                        Cc2530FlashCut::PowerCut, device),
           0xd5d18132u, "golden partial word (seed 3, serial 1, 320 cycles)");
    Cc2530F256 other;
    CHECK(cut_cpu_word(Cc2530FlashInterrupted::Partial, 4, WordCycles / 2,
                       Cc2530FlashCut::PowerCut, other) != 0xd5d18132u);
}

// DMA (trigger FLASH) feeds two words to page 7 from flash code (6.2.3);
// the CPU stalls. Word 0 runs at t+8 .. t+648, word 1 (FULL meanwhile) at
// t+648 .. t+1288 (see test_cc2530_dma_flash_aes.cpp).
Flash dma_write_flash(std::uint16_t &write) {
    Asm main(0x0100);
    main.mov(SP, 0x30).xstore(Desc0, flash_dma_descriptor(0x0200, 8));
    main.mov(DMA0CFGH, hi(Desc0)).mov(DMA0CFGL, lo(Desc0));
    main.xwrite(FADDRH, 0x0e).mov(DMAARM, 0x01).nops(12).dptr(FCTL).op({0x74, 0x06});
    write = main.here();
    main.op({0xf0}).nops(4).halt();
    return program(main);
}

void test_interrupted_dma_word_write() {
    const std::vector<std::uint8_t> data{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    // Cut by a trigger 320 cycles into the first word: word 0 partial,
    // the buffered word 1 never starts.
    {
        std::uint16_t write = 0;
        Cc2530F256 device;
        device.set_flash_fault_config(fault_config(Cc2530FlashInterrupted::Partial, 11));
        load(device, dma_write_flash(write));
        for (std::uint16_t i = 0; i < data.size(); ++i)
            device.write_sram(static_cast<std::uint16_t>(0x0200 + i), data[i]);
        Cc2530FlashTrigger trigger;
        trigger.op = Cc2530FlashOpFilter::Write;
        trigger.delay_cycles = WordCycles / 2;
        device.add_flash_trigger(trigger);
        const std::uint64_t t = run_to_pc(device, write);
        run_to(device, t + 8 + WordCycles / 2);
        expect(device.diagnostics().power_cuts, 1, "trigger fired");
        expect(device.diagnostics().last_power_cut_cycle, t + 8 + WordCycles / 2, "cut cycle");
        expect(flash_word(device, 0x3800), partial_word(0xffffffffu, 0x04030201u, 11, 1,
                                                        WordCycles / 2),
               "word 0 partial");
        expect(flash_word(device, 0x3804), 0xffffffffu, "word 1 dropped");
        expect(device.flash_log().size(), 1, "one entry");
        expect(device.read_sfr(DMAIRQ), 0, "DMA state reset by the power cut");
    }
    // nth = 2 on page 7: word 0 completes, word 1 (serial 2) is cut.
    {
        std::uint16_t write = 0;
        Cc2530F256 device;
        device.set_flash_fault_config(fault_config(Cc2530FlashInterrupted::Partial, 11));
        load(device, dma_write_flash(write));
        for (std::uint16_t i = 0; i < data.size(); ++i)
            device.write_sram(static_cast<std::uint16_t>(0x0200 + i), data[i]);
        Cc2530FlashTrigger trigger;
        trigger.op = Cc2530FlashOpFilter::Write;
        trigger.nth = 2;
        trigger.page = 7;
        trigger.delay_cycles = 100;
        device.add_flash_trigger(trigger);
        const std::uint64_t t = run_to_pc(device, write);
        run_to(device, t + 8 + WordCycles + 100);
        expect(device.diagnostics().power_cuts, 1, "trigger fired");
        expect(flash_word(device, 0x3800), 0x04030201u, "word 0 complete");
        expect(flash_word(device, 0x3804), partial_word(0xffffffffu, 0x08070605u, 11, 2, 100),
               "word 1 partial");
        const auto &log = device.flash_log();
        expect(log.size(), 2, "two entries");
        if (log.size() == 2) {
            expect(static_cast<unsigned>(log[0].outcome),
                   static_cast<unsigned>(Cc2530FlashOutcome::Completed), "word 0");
            expect(static_cast<unsigned>(log[1].outcome),
                   static_cast<unsigned>(Cc2530FlashOutcome::Partial), "word 1");
            expect(log[1].cycle, t + 8 + WordCycles + 100, "cut time");
            expect(log[1].address, 0x3804, "word 1 address");
        }
        expect(device.diagnostics().flash_page_writes[7], 2, "both count as writes");
    }
}

// ======================================================================
// Triggers and scheduled cuts
// ======================================================================

// A trigger due at the cycle the erase completes: the erase completes
// first (no interrupted operation), then the power cut.
void test_trigger_at_completion() {
    std::uint16_t erase = 0;
    Cc2530F256 device;
    device.set_flash_fault_config(fault_config(Cc2530FlashInterrupted::Partial, 5));
    load(device, erase_flash(erase_program(erase)));
    Cc2530FlashTrigger trigger;
    trigger.op = Cc2530FlashOpFilter::Erase;
    trigger.page = 6;
    trigger.delay_cycles = EraseCycles;
    device.add_flash_trigger(trigger);
    const std::uint64_t t = run_to_pc(device, erase);
    run_to(device, t + EraseCycles);
    expect(device.diagnostics().power_cuts, 1, "cut at completion");
    expect(device.diagnostics().flash_interrupted_erases, 0, "nothing in flight");
    expect(device.diagnostics().flash_page_erases[6], 1, "erase completed");
    expect_bytes(flash_bytes(device, Page6, PageBytes), filled(PageBytes, 0xff), "erased");
    // A trigger on another page never fires.
    Cc2530FlashTrigger other;
    other.op = Cc2530FlashOpFilter::Erase;
    other.page = 9;
    device.add_flash_trigger(other);
    device.run(50'000'000, t + 3 * EraseCycles);
    expect(device.pending_power_faults(), 1, "page 9 trigger still armed");
    expect(device.diagnostics().power_cuts, 1, "no further cut");
}

// action = Reset: the trigger applies an external reset (SLEEPSTA.RST = 01)
// instead of a power cut; SRAM is kept.
void test_trigger_reset_action() {
    std::uint16_t erase = 0;
    Cc2530F256 device;
    device.set_flash_fault_config(fault_config(Cc2530FlashInterrupted::Partial, 5));
    load(device, erase_flash(erase_program(erase)));
    device.write_sram(0x0100, 0x42);
    Cc2530FlashTrigger trigger;
    trigger.op = Cc2530FlashOpFilter::Any;
    trigger.delay_cycles = 1000;
    trigger.action = Cc2530FaultAction::Reset;
    device.add_flash_trigger(trigger);
    const std::uint64_t t = run_to_pc(device, erase);
    run_to(device, t + 1000);
    expect(device.diagnostics().power_cuts, 0, "no power cut");
    expect(rst_field(device), 1, "external reset");
    expect(device.peek_xdata(0x0100), 0x42, "SRAM kept by a reset");
    const auto &log = device.flash_log();
    expect(log.size(), 1, "one entry");
    if (!log.empty()) {
        expect(static_cast<unsigned>(log[0].cut), static_cast<unsigned>(Cc2530FlashCut::Reset), "cut");
        expect(static_cast<unsigned>(log[0].reset_cause),
               static_cast<unsigned>(Cc2530ResetCause::External), "cause");
        expect(log[0].elapsed_cycles, 1000, "elapsed");
    }
    bool threw = false;
    try {
        Cc2530FlashTrigger bad;
        bad.nth = 0;
        device.add_flash_trigger(bad);
    } catch (const std::invalid_argument &) {
        threw = true;
    }
    CHECK(threw);
}

// A scheduled power cut reaches a chip asleep in PM2 (no wake source): the
// chip boots again; SRAM is lost, virtual time continues.
void test_power_cut_in_pm2() {
    Asm main(0x0100);
    main.mov(SP, 0x30).op({0x05, 0x50}); // INC 0x50
    main.mov(SLEEPCMD, 0x06).mov(PCON, 0x01).halt(); // PM2, no wake source
    Cc2530F256 device;
    load(device, program(main));
    Cc2530FlashFaultConfig config;
    config.sram_fill = 0x10; // after the load: only the power cut uses it
    device.set_flash_fault_config(config);
    device.schedule_power_cut(5'000'000);
    Cc2530RunResult result = device.run(1000, 1'000'000);
    CHECK(result.sleeping);
    expect(device.diagnostics().power_cuts, 0, "not yet cut");
    expect(static_cast<unsigned>(device.power_mode()),
           static_cast<unsigned>(Cc2530PowerMode::Pm2), "in PM2");
    CHECK(device.may_wake());
    result = device.run(1'000'000, 6'000'000);
    expect(device.diagnostics().power_cuts, 1, "power cut in PM2");
    expect(device.diagnostics().last_power_cut_cycle, 5'000'000, "exact cut time");
    expect(device.read_idata(0x50), 0x11, "SRAM filled with 0x10, then INC once");
    expect(rst_field(device), 0, "power-on reset");
    expect(power_ons(device), 2, "load + cut");
    CHECK(device.registers().cycles >= 5'000'000);
    CHECK(!device.may_wake());
}

// ======================================================================
// SRAM after a power cut
// ======================================================================

void test_sram_fill() {
    std::uint16_t erase = 0;
    {
        Cc2530F256 device;
        Cc2530FlashFaultConfig config;
        config.sram_fill = 0xa5;
        device.set_flash_fault_config(config);
        load(device, erase_flash(erase_program(erase)));
        expect_bytes(sram(device), filled(Cc2530F256::SramSize, 0xa5), "fill at the load");
        device.write_sram(0x0123, 0x00);
        device.reset();
        expect(device.peek_xdata(0x0123), 0x00, "reset keeps SRAM");
        device.power_cut();
        expect_bytes(sram(device), filled(Cc2530F256::SramSize, 0xa5), "fill after the cut");
    }
    {
        Cc2530F256 a, b, c;
        Cc2530FlashFaultConfig config;
        config.sram_fill = std::nullopt;
        config.seed = 99;
        a.set_flash_fault_config(config);
        b.set_flash_fault_config(config);
        config.seed = 100;
        c.set_flash_fault_config(config);
        for (Cc2530F256 *device : {&a, &b, &c})
            load(*device, erase_flash(erase_program(erase)));
        expect_bytes(sram(a), sram_after_power_on(99, 0), "seeded fill at the load");
        a.power_cut();
        b.power_cut();
        c.power_cut();
        expect(power_ons(a), 2, "two power-on resets");
        expect_bytes(sram(a), sram_after_power_on(99, 1), "seeded fill after the cut");
        expect_bytes(sram(a), sram(b), "same seed, same SRAM");
        CHECK(sram(a) != sram(c));
        CHECK(sram(a) != sram_after_power_on(99, 0));
        // Pinned: first SRAM bytes for seed 99 after the first power cut.
        const std::vector<std::uint8_t> after_cut = sram(a);
        expect_bytes(std::vector<std::uint8_t>(after_cut.begin(), after_cut.begin() + 4),
                     {0x50, 0x67, 0x9d, 0xaa}, "golden SRAM bytes");
    }
}

// ======================================================================
// Flash image round trip and instrumentation
// ======================================================================

void test_flash_image_round_trip() {
    Asm code(SramCode);
    code.write_word(0x0e00, 0x12345678).erase_page(0x0c00).write_word(0x0c10, 0xcafef00d);
    const std::uint16_t end = code.here();
    code.halt();
    Flash flash = program(sram_entry());
    flash.at(Page6).op(page6_pattern());
    Cc2530F256 writer;
    load(writer, flash);
    place_sram(writer, code);
    run_to_pc(writer, end);
    const std::vector<std::uint8_t> image = writer.flash_image();
    expect(image.size(), Cc2530F256::FlashSize, "image size");
    expect(flash_word(writer, 0x3800), 0x12345678u, "page 7 word");
    expect(flash_word(writer, 0x3040), 0xcafef00du, "page 6 word");

    // Survives a power cut and a reset within the run.
    writer.power_cut();
    writer.reset();
    CHECK(writer.flash_image() == image);

    Cc2530F256 reader;
    load(reader, program(sram_entry()));
    reader.write_sram(0x0100, 0x11);
    reader.restore_flash(image);
    CHECK(reader.flash_image() == image);
    expect(reader.read_code(0x3040), 0x0d, "CODE view rebuilt");
    expect(reader.peek_xdata(0x0100), 0x00, "power-on reset after the restore");
    expect(power_ons(reader), 1, "diagnostics restarted with the power-on reset");
    expect(reader.diagnostics().flash_page_writes[7], 0, "counters restart");
    bool threw = false;
    try {
        reader.restore_flash(std::vector<std::uint8_t>(1024, 0xff));
    } catch (const std::invalid_argument &) {
        threw = true;
    }
    CHECK(threw);
}

void test_instrumentation() {
    Asm code(SramCode);
    code.erase_page(0x0c00).write_word(0x0c00, 0x0).write_word(0x0c01, 0x0);
    code.erase_page(0x0e00).write_word(0x0e00, 0x0).erase_page(0x0c00);
    // Locked page 10: aborted erase, logged, not counted.
    code.erase_page(0x1400);
    const std::uint16_t end = code.here();
    code.halt();
    Flash flash = program(sram_entry());
    flash.at(Cc2530F256::LockBitsStart + 1).op({0xfb}); // PAGELOCK[10] = 0
    {
        Cc2530F256 device;
        device.set_flash_fault_config(fault_config(Cc2530FlashInterrupted::Abandon, 0));
        load(device, flash);
        place_sram(device, code);
        run_to_pc(device, end);
        const auto &d = device.diagnostics();
        expect(d.flash_page_erases[6], 2, "page 6 erases");
        expect(d.flash_page_erases[7], 1, "page 7 erases");
        expect(d.flash_page_erases[10], 0, "aborted erase not counted");
        expect(d.flash_page_writes[6], 2, "page 6 writes");
        expect(d.flash_page_writes[7], 1, "page 7 writes");
        const std::vector<Cc2530FlashLogEntry> log = device.take_flash_log();
        expect(log.size(), 7, "seven operations");
        CHECK(device.flash_log().empty());
        if (log.size() == 7) {
            for (std::size_t i = 1; i < log.size(); ++i)
                CHECK(log[i].cycle > log[i - 1].cycle);
            expect(static_cast<unsigned>(log[6].outcome),
                   static_cast<unsigned>(Cc2530FlashOutcome::Aborted), "aborted");
            expect(log[6].page, 10, "locked page");
            expect(log[1].changed_bits, 32, "word programmed from FF");
            expect(log[5].changed_bits, 64, "two programmed words erased");
            expect(log[0].elapsed_cycles, EraseCycles, "erase time");
            expect(log[1].elapsed_cycles, WordCycles, "word time");
        }
    }
    {
        Cc2530F256 device; // log disabled by default
        load(device, flash);
        place_sram(device, code);
        run_to_pc(device, end);
        CHECK(device.flash_log().empty());
        expect(device.diagnostics().flash_page_erases[6], 2, "counters without the log");
    }
}

// ======================================================================
// End to end: boot, erase, power cut mid-erase, reboot
// ======================================================================

// Boot: IDATA 0x40 = first page 6 byte as seen at boot (MOVC); erase page 6
// from flash (CPU stall); DMA-write 11 22 33 44 to its first word; copy the
// word to IDATA 0x41-0x44; halt. Power cut half way through the first
// erase (partial outcome): the second boot sees the partially erased byte,
// then erases and writes again.
Flash journal_flash() {
    Asm main(0x0100);
    main.mov(SP, 0x30).movc_to(0x3000, 0x40);
    main.xstore(Desc0, flash_dma_descriptor(0x0200, 4)).xstore(0x0200, {0x11, 0x22, 0x33, 0x44});
    main.mov(DMA0CFGH, hi(Desc0)).mov(DMA0CFGL, lo(Desc0));
    main.xwrite(FADDRH, 0x0c).xwrite(FADDRL, 0x00).xwrite(FCTL, 0x05); // erase (stall)
    main.mov(DMAARM, 0x01).nops(12).xwrite(FCTL, 0x06);                 // DMA write (stall)
    main.movc_to(0x3000, 0x41).movc_to(0x3001, 0x42).movc_to(0x3002, 0x43).movc_to(0x3003, 0x44);
    main.halt();
    Flash flash = program(main);
    flash.at(Page6).op(filled(PageBytes, 0x00));
    return flash;
}

void setup_journal_cut(Cc2530F256 &device) {
    Cc2530FlashFaultConfig config = fault_config(Cc2530FlashInterrupted::Partial, 2024);
    config.sram_fill = 0xee;
    device.set_flash_fault_config(config);
    // for_each_mode() calls this after the load: the load itself filled
    // SRAM with 0, only the power cut uses 0xEE.
    Cc2530FlashTrigger trigger;
    trigger.op = Cc2530FlashOpFilter::Erase;
    trigger.page = 6;
    trigger.delay_cycles = EraseCycles / 2;
    device.add_flash_trigger(trigger);
}

void test_end_to_end_power_cut_mid_erase() {
    const std::vector<std::uint8_t> half =
        partial_erase(filled(PageBytes, 0x00), 2024, 1, EraseCycles / 2);
    std::vector<std::uint8_t> final_page = filled(PageBytes, 0xff);
    final_page[0] = 0x11, final_page[1] = 0x22, final_page[2] = 0x33, final_page[3] = 0x44;
    for_each_mode(
        journal_flash(), 1'100'000, setup_journal_cut,
        [&](Cc2530F256 &device, Mode) {
            const auto &d = device.diagnostics();
            expect(d.power_cuts, 1, "one power cut");
            expect(d.flash_interrupted_erases, 1, "erase cut");
            expect(d.flash_page_erases[6], 1, "second erase completed");
            expect(d.flash_page_writes[6], 1, "one word written");
            expect(device.read_idata(0x40), half[0], "second boot saw the partial erase");
            expect(device.read_idata(0x41), 0x11, "word after reboot");
            expect(device.read_idata(0x44), 0x44, "word after reboot");
            expect_bytes(flash_bytes(device, Page6, PageBytes), final_page, "page 6 at the end");
            expect(rst_field(device), 0, "power-on reset cause");
            expect(device.read_idata(0x70), 0xee, "SRAM fill after the cut");
            const auto &log = device.flash_log();
            expect(log.size(), 3, "erase (partial), erase, word");
            if (log.size() == 3) {
                expect(static_cast<unsigned>(log[0].outcome),
                       static_cast<unsigned>(Cc2530FlashOutcome::Partial), "partial");
                expect(log[0].elapsed_cycles, EraseCycles / 2, "cut half way");
                expect(log[0].changed_bits, 16384 - zero_bits(half), "bits erased");
                expect(static_cast<unsigned>(log[1].outcome),
                       static_cast<unsigned>(Cc2530FlashOutcome::Completed), "erase");
                expect(static_cast<unsigned>(log[2].op),
                       static_cast<unsigned>(Cc2530FlashOpKind::WordWrite), "word");
                CHECK(log[1].cycle - log[0].cycle > EraseCycles); // reboot, then a full erase
            }
        },
        "journal power cut");
}

} // namespace

int main() {
    test_fault_draw_golden();
    test_write_limits();
    test_erase_restarts_tracking();
    test_interrupted_erase();
    test_interrupted_erase_determinism();
    test_interrupted_cpu_word_write();
    test_interrupted_dma_word_write();
    test_trigger_at_completion();
    test_trigger_reset_action();
    test_power_cut_in_pm2();
    test_sram_fill();
    test_flash_image_round_trip();
    test_instrumentation();
    test_end_to_end_power_cut_mid_erase();
    return finish("CC2530 flash fault model");
}
