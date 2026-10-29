// CC2530F256 Timer 1, Timer 3/Timer 4, Sleep Timer and Watchdog Timer tests.
//
// Every scenario runs hand-encoded MCS-51 machine code on the Unicorn core
// (or drives the documented host API) and checks register values, compare
// outputs and ISR entry cycles derived from SWRU191F, not from the model.
//
// Timing conventions used for the expected values (core contract,
// cc2530f256.hpp run()):
//  * A timer started (or re-anchored) by a register write counts from the
//    start of the writing instruction W; with tick period p the count reaches
//    k at W + k * p (Model-defined anchor, see cc2530_timers.cpp).
//  * Timer 1 tick = 32 MHz >> CLKCONSTA.TICKSPD, then T1CTL.DIV
//    (SWRU191F 9.1); reset TICKSPD = 1 gives 2 cycles per tick.
//  * An event at time E is applied at the first instruction boundary B >= E;
//    the ISR's first instruction executes at B + 2 (LCALL to the vector).
//    In the 0xFF sled (MOV R7,A, 1 cycle) every cycle is a boundary; in a
//    SJMP $ loop first reached at L, boundaries are at L + 2k.
//  * The 32 kHz clock: one period = 976.5625 cycles, CLK32K low in the first
//    half of each period, so period j starts at ceil(j * 15625 / 16) and its
//    rising edge is at ceil((2j + 1) * 15625 / 32).

#include "cc2530_periph_support.hpp"
#include "cc2530_timers.hpp"

#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <optional>
#include <vector>

namespace mcs51emu::cc2530 {
// TEST ACCESSOR (cc2530_timers.hpp): DMA trigger counts since construction.
struct TimersTestAccess {
    static std::uint64_t t1(const Timer1 &t, unsigned channel) { return t.dma_triggers_[channel]; }
    // 0 T3_CH0, 1 T3_CH1, 2 T4_CH0, 3 T4_CH1
    static std::uint64_t t34(const Timer34 &t, unsigned n) { return t.dma_triggers_[n]; }
    static std::uint64_t st(const SleepTimer &t) { return t.dma_triggers_; }
};
} // namespace mcs51emu::cc2530

namespace {

using namespace cc2530test;
using mcs51emu::cc2530::TimersTestAccess;

// SFRs (SWRU191F Table 2-1)
constexpr std::uint8_t SP = 0x81, PCON = 0x87, PICTL = 0x8c, ST0 = 0x95, ST1 = 0x96, ST2 = 0x97,
                       IEN2 = 0x9a, SLEEPSTA = 0x9d, CLKCONSTA = 0x9e, IEN0 = 0xa8,
                       STLOAD = 0xad, T1STAT = 0xaf, IEN1 = 0xb8, SLEEPCMD = 0xbe, IRCON = 0xc0,
                       CLKCONCMD = 0xc6, WDCTL = 0xc9, T3CNT = 0xca, T3CTL = 0xcb,
                       T3CCTL0 = 0xcc, T3CC0 = 0xcd, T3CCTL1 = 0xce, T3CC1 = 0xcf, TIMIF = 0xd8,
                       T1CC0L = 0xda, T1CC0H = 0xdb, T1CC1L = 0xdc, T1CC1H = 0xdd,
                       T1CC2L = 0xde, T1CC2H = 0xdf, T1CNTL = 0xe2, T1CNTH = 0xe3, T1CTL = 0xe4,
                       T1CCTL0 = 0xe5, T1CCTL1 = 0xe6, T1CCTL2 = 0xe7, IRCON2 = 0xe8,
                       T4CNT = 0xea, T4CTL = 0xeb, T4CCTL0 = 0xec, T4CC0 = 0xed, T4CCTL1 = 0xee,
                       T4CC1 = 0xef;
// Bit addresses
constexpr std::uint8_t BitEa = 0xaf, BitP0if = 0xc5, BitStif = 0xc7, BitT3ovfif = 0xd8,
                       BitT4ovfif = 0xdb, BitWdtif = 0xec, BitAcc0 = 0xe0;
// XREGs
constexpr std::uint16_t XIrctl = 0x6281, XT1cctl0 = 0x62a0, XT1cc0l = 0x62a6, XStcc = 0x62b0,
                        XStcs = 0x62b1, XStcv0 = 0x62b2, XPictl = 0x708c;
constexpr std::uint16_t xt1cctl(unsigned n) { return static_cast<std::uint16_t>(XT1cctl0 + n); }
constexpr std::uint16_t xt1ccl(unsigned n) { return static_cast<std::uint16_t>(XT1cc0l + 2 * n); }
// Interrupt vectors (SWRU191F Table 2-5)
constexpr std::uint16_t VSt = 0x2b, VT1 = 0x4b, VT3 = 0x5b, VT4 = 0x63, VP0 = 0x6b, VWdt = 0x8b;
// IRCON / IRCON2 flags
constexpr std::uint8_t IrconT1if = 0x02, IrconT3if = 0x08, IrconT4if = 0x10, IrconStif = 0x80,
                       Ircon2Wdtif = 0x10;
constexpr unsigned PmIndex2 = 3, PmIndex3 = 4; // Cc2530PowerMode Pm2, Pm3
constexpr unsigned CauseWatchdog = 2;          // Cc2530ResetCause::Watchdog

// Flash image: LJMP 0x0100 at the reset vector; main at 0x0100 starts with
// MOV SP,#0x30. Unwritten flash is 0xFF (MOV R7,A: a 1-cycle sled).
struct Program {
    Flash flash;
    Program() {
        flash.at(0).op({0x02, 0x01, 0x00});
        flash.at(0x0100).op(mov_dir_imm(SP, 0x30));
    }
    Program &op(std::initializer_list<std::uint8_t> code) {
        flash.op(code);
        return *this;
    }
    Program &op(const std::vector<std::uint8_t> &code) {
        flash.op(code);
        return *this;
    }
    Program &mov(std::uint8_t dir, std::uint8_t value) { return op(mov_dir_imm(dir, value)); }
    Program &xw(std::uint16_t address, std::uint8_t value) { return op(movx_write(address, value)); }
    Program &xr(std::uint16_t address, std::uint8_t dir) { return op(movx_read_to(address, dir)); }
    Program &copy(std::uint8_t src, std::uint8_t dst) { return op({0x85, src, dst}); }
    Program &setb(std::uint8_t bit) { return op({0xd2, bit}); }
    Program &nop() { return op({0x00}); }
    Program &sjmp_self() { return op({0x80, 0xfe}); }
    Program &at(std::uint32_t physical) {
        flash.at(physical);
        return *this;
    }
    // SLEEPCMD = mode; PCON = 1; NOP. The instruction after the PCON write is
    // kept off a 4-byte boundary (SWRU191F 4.2).
    Program &enter_power_mode(std::uint8_t sleepcmd) {
        mov(SLEEPCMD, sleepcmd);
        if (((here() + 3u) & 3u) == 0)
            nop();
        return mov(PCON, 0x01).nop();
    }
    std::uint16_t here() const { return flash.here(); }
    Program &isr(std::uint16_t vector, std::initializer_list<std::uint8_t> body) {
        const std::uint32_t saved = flash.cursor;
        flash.at(vector).op(body);
        flash.cursor = saved;
        return *this;
    }
};

std::uint64_t now(const Cc2530F256 &d) { return d.registers().cycles; }

// Runs until the CPU is about to execute `pc`; returns that cycle.
std::uint64_t run_to(Cc2530F256 &d, std::uint16_t pc, std::uint64_t limit, const char *what) {
    d.set_breakpoint(pc);
    for (;;) {
        const Cc2530RunResult r = d.run(10'000'000, limit);
        if (!r.error.empty()) {
            std::cerr << what << ": run error: " << r.error << '\n';
            ++failures;
            break;
        }
        if (r.break_hit)
            break;
        if (now(d) >= limit) {
            std::cerr << what << ": PC 0x" << std::hex << pc << std::dec << " not reached by cycle "
                      << limit << '\n';
            ++failures;
            break;
        }
    }
    d.set_breakpoint(std::nullopt);
    return now(d);
}

// Runs to the first instruction boundary at or after `cycle`.
void run_until(Cc2530F256 &d, std::uint64_t cycle) {
    while (now(d) < cycle) {
        const Cc2530RunResult r = d.run(10'000'000, cycle);
        if (!r.error.empty()) {
            std::cerr << "run error: " << r.error << '\n';
            ++failures;
            return;
        }
    }
}

// Runs to exactly `cycle` (which must be an instruction boundary).
bool at(Cc2530F256 &d, std::uint64_t cycle, const char *what) {
    run_until(d, cycle);
    if (now(d) != cycle) {
        std::cerr << what << ": wanted boundary " << cycle << ", landed at " << now(d) << '\n';
        ++failures;
        return false;
    }
    return true;
}

std::uint64_t loop_boundary(std::uint64_t loop, std::uint64_t event) {
    return event <= loop ? loop : loop + 2 * ((event - loop + 1) / 2);
}
std::uint64_t lf_ticks(std::uint64_t cycles) { return cycles * 16 / 15625; }
std::uint64_t period_start(std::uint64_t j) { return (j * 15625 + 15) / 16; }
std::uint64_t rising_edge(std::uint64_t j) { return ((2 * j + 1) * 15625 + 31) / 32; }

std::uint16_t word_sfr(const Cc2530F256 &d, std::uint8_t low) {
    return static_cast<std::uint16_t>(d.read_sfr(low) | d.read_sfr(static_cast<std::uint8_t>(low + 1)) << 8);
}
std::uint16_t word_xdata(const Cc2530F256 &d, std::uint16_t low) {
    return static_cast<std::uint16_t>(d.peek_xdata(low) |
                                      d.peek_xdata(static_cast<std::uint16_t>(low + 1)) << 8);
}

mcs51emu::cc2530::Timer1 &t1(Cc2530F256 &d) { return *d.peripherals().timer1; }
mcs51emu::cc2530::Timer34 &t34(Cc2530F256 &d) { return *d.peripherals().timer34; }
mcs51emu::cc2530::SleepTimer &st(Cc2530F256 &d) { return *d.peripherals().sleep_timer; }

// A compare output edge: channel output changes to `level` at tick `tick`.
struct Edge {
    unsigned channel;
    std::uint64_t tick;
    bool level;
};

// Checks each edge's output level one cycle before (W + tick * period - 1)
// and at (W + tick * period) the edge. Edges must be sorted by tick and the
// CPU must be in the 1-cycle sled at those times.
template <class Output>
void check_edges(Cc2530F256 &d, std::uint64_t w, std::uint64_t period,
                 const std::vector<Edge> &edges, Output output, const char *what) {
    for (std::size_t i = 0; i < edges.size();) {
        std::size_t j = i;
        while (j < edges.size() && edges[j].tick == edges[i].tick)
            ++j;
        const std::uint64_t t = w + edges[i].tick * period;
        if (at(d, t - 1, what))
            for (std::size_t k = i; k < j; ++k)
                if (output(edges[k].channel) == edges[k].level) {
                    std::cerr << what << ": channel " << edges[k].channel << " already "
                              << edges[k].level << " before tick " << edges[k].tick << '\n';
                    ++failures;
                }
        if (at(d, t, what))
            for (std::size_t k = i; k < j; ++k)
                if (output(edges[k].channel) != edges[k].level) {
                    std::cerr << what << ": channel " << edges[k].channel << " not "
                              << edges[k].level << " at tick " << edges[k].tick << '\n';
                    ++failures;
                }
        i = j;
    }
}

// ---------------------------------------------------------------------------
// Reset values and SFR/XREG aliases
// ---------------------------------------------------------------------------

void check_reset_values(Cc2530F256 &d) {
    // SWRU191F 9.12 Timer 1 registers
    for (std::uint8_t a : {T1CNTL, T1CNTH, T1CTL, T1STAT, T1CC0L, T1CC0H, T1CC1L, T1CC1H, T1CC2L,
                           T1CC2H})
        CHECK_EQ(d.read_sfr(a), 0);
    for (std::uint8_t a : {T1CCTL0, T1CCTL1, T1CCTL2})
        CHECK_EQ(d.read_sfr(a), 0x40);
    for (unsigned n = 0; n < 5; ++n) {
        CHECK_EQ(d.peek_xdata(xt1cctl(n)), 0x40);
        CHECK_EQ(word_xdata(d, xt1ccl(n)), 0);
    }
    CHECK_EQ(d.peek_xdata(XIrctl), 0);
    // SWRU191F 10.8 Timer 3/4 registers
    for (std::uint8_t base : {std::uint8_t{0xca}, std::uint8_t{0xea}}) {
        CHECK_EQ(d.read_sfr(base), 0);                                       // TxCNT
        CHECK_EQ(d.read_sfr(static_cast<std::uint8_t>(base + 1)), 0x08);     // TxCTL
        CHECK_EQ(d.read_sfr(static_cast<std::uint8_t>(base + 2)), 0x40);     // TxCCTL0
        CHECK_EQ(d.read_sfr(static_cast<std::uint8_t>(base + 3)), 0);        // TxCC0
        CHECK_EQ(d.read_sfr(static_cast<std::uint8_t>(base + 4)), 0x40);     // TxCCTL1
        CHECK_EQ(d.read_sfr(static_cast<std::uint8_t>(base + 5)), 0);        // TxCC1
    }
    CHECK_EQ(d.read_sfr(TIMIF), 0x40);
    // SWRU191F 11.4 Sleep Timer registers, 16.3 WDCTL
    CHECK_EQ(d.read_sfr(STLOAD), 0x01);
    CHECK_EQ(d.peek_xdata(XStcc), 0x1f);
    CHECK_EQ(d.peek_xdata(XStcs), 0);
    for (unsigned n = 0; n < 3; ++n)
        CHECK_EQ(d.peek_xdata(static_cast<std::uint16_t>(XStcv0 + n)), 0);
    CHECK_EQ(d.read_sfr(WDCTL), 0);
}

void test_reset_values_and_aliases() {
    Program p;
    p.xw(xt1cctl(1), 0x5c)      // XREG T1CCTL1 -> SFR 0xE6
        .mov(T1CCTL2, 0x13)     // SFR T1CCTL2 -> XREG 0x62A2
        .xw(xt1ccl(0), 0x34)    // XREG T1CC0L (buffered until T1CC0H)
        .copy(T1CC0L, 0x40)     // still the value in effect: 0
        .mov(T1CC0H, 0x12)      // SFR T1CC0H completes the write
        .xw(xt1ccl(3), 0x78)    // T1CC3L (XREG only)
        .xw(static_cast<std::uint16_t>(xt1ccl(3) + 1), 0x56)
        .xr(static_cast<std::uint16_t>(xt1ccl(3) + 1), 0x41)
        .copy(T1CC0L, 0x42);
    const std::uint16_t done = p.here();
    p.sjmp_self();
    Cc2530F256 d;
    load(d, p.flash);
    check_reset_values(d);
    run_to(d, done, 2000, "aliases");
    // SWRU191F 9.12: XREG 0x62A0-0x62A2 / 0x62A6-0x62AB are the SFR registers.
    CHECK_EQ(d.read_sfr(T1CCTL1), 0x5c);
    CHECK_EQ(d.peek_xdata(xt1cctl(2)), 0x13);
    CHECK_EQ(d.read_idata(0x40), 0x00); // SWRU191F 9.8: T1CCnL buffered until T1CCnH
    CHECK_EQ(d.read_idata(0x42), 0x34);
    CHECK_EQ(word_sfr(d, T1CC0L), 0x1234);
    CHECK_EQ(word_xdata(d, xt1ccl(0)), 0x1234);
    CHECK_EQ(d.read_idata(0x41), 0x56);
    CHECK_EQ(word_xdata(d, xt1ccl(3)), 0x5678);
    // A chip reset restores every reset value.
    d.reset();
    check_reset_values(d);
}

// ---------------------------------------------------------------------------
// Timer 1
// ---------------------------------------------------------------------------

// SWRU191F 9.4 modulo mode, 9.8 compare, 9.10 interrupts, 9.11 DMA.
void test_t1_modulo_compare_isr() {
    Program p;
    p.mov(T1CC0L, 100).mov(T1CC0H, 0).mov(T1CCTL0, 0x44).mov(IEN1, 0x02).setb(BitEa);
    const std::uint16_t w = p.here();
    p.mov(T1CTL, 0x02); // modulo, DIV 1
    p.isr(VT1, {0x05, 0x60, 0x53, T1STAT, 0xfe, 0x32}); // INC 0x60; ANL T1STAT,#0xFE; RETI
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 1000, "t1 modulo start");
    // Compare at count 100 (tick 100), then 0..100 again: tick 201.
    CHECK_EQ(run_to(d, VT1, W + 1000, "t1 modulo 1st"), W + 202);
    CHECK_EQ(d.read_sfr(T1STAT), 0x01);
    CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0); // cleared on vector entry
    CHECK_EQ(TimersTestAccess::t1(t1(d), 0), 1u);
    CHECK(t1(d).output(0)); // CMP 000: set on compare
    CHECK_EQ(run_to(d, VT1, W + 1000, "t1 modulo 2nd"), W + 404);
    CHECK_EQ(d.read_idata(0x60), 1);
    CHECK_EQ(d.read_sfr(T1STAT), 0x01); // no OVFIF in modulo mode below 0xFFFF
    CHECK_EQ(TimersTestAccess::t1(t1(d), 0), 2u);
    CHECK_EQ(d.read_sfr(T1CNTL), 0); // tick 202: wrapped past T1CC0
    CHECK_EQ(d.diagnostics().stale_interrupt_posts, 0u);
}

// SWRU191F 9.3 free-running, 9.12 T1CNTL/T1CNTH: reading T1CNTL latches
// T1CNTH; writing T1CNTL resets the counter.
void test_t1_free_running_latch() {
    struct Layout {
        std::uint16_t w, r1, h1, r2, x, r3, l;
    };
    auto build = [](bool pad, Layout &at_) {
        Program p;
        p.mov(IEN1, 0x02).setb(BitEa);
        at_.w = p.here();
        p.mov(T1CTL, 0x01);
        at_.r1 = p.here();
        p.op({0xe5, T1CNTL, 0xf5, 0x40});      // MOV A,T1CNTL; MOV 0x40,A
        p.op({0x7f, 0x00, 0xdf, 0xfe});        // MOV R7,#0; DJNZ R7,$
        at_.h1 = p.here();
        p.copy(T1CNTH, 0x41);                  // still the latched high byte
        at_.r2 = p.here();
        p.op({0xe5, T1CNTL, 0xf5, 0x42}).copy(T1CNTH, 0x43);
        if (pad)
            p.nop();
        at_.x = p.here();
        p.mov(T1CNTL, 0x00);
        at_.r3 = p.here();
        p.op({0xe5, T1CNTL, 0xf5, 0x44});
        at_.l = p.here();
        p.sjmp_self();
        // MOV 0x50,T1STAT; ANL T1STAT,#0xDF; RETI
        p.isr(VT1, {0x85, T1STAT, 0x50, 0x53, T1STAT, 0xdf, 0x32});
        return p;
    };
    Layout lay{};
    Program p = build(false, lay);
    {
        Cc2530F256 probe;
        load(probe, p.flash);
        const std::uint64_t w = run_to(probe, lay.w, 10000, "latch probe w");
        const std::uint64_t x = run_to(probe, lay.x, 10000, "latch probe x");
        if ((x - w) & 1u)
            p = build(true, lay); // keep X on a tick boundary of the W anchor
    }
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, lay.w, 10000, "latch w");
    const std::uint64_t R1 = run_to(d, lay.r1, 10000, "latch r1");
    const std::uint64_t H1 = run_to(d, lay.h1, 10000, "latch h1");
    const std::uint64_t R2 = run_to(d, lay.r2, 10000, "latch r2");
    const std::uint64_t X = run_to(d, lay.x, 10000, "latch x");
    CHECK_EQ((X - W) & 1u, 0u);
    const std::uint64_t R3 = run_to(d, lay.r3, 10000, "latch r3");
    const std::uint64_t L = run_to(d, lay.l, 10000, "latch l");
    CHECK_EQ(d.read_idata(0x40), lo(static_cast<std::uint16_t>((R1 - W) / 2)));
    CHECK((H1 - W) / 2 >= 256); // a live T1CNTH would be non-zero here
    CHECK_EQ(d.read_idata(0x41), 0);
    CHECK_EQ(d.read_idata(0x42), lo(static_cast<std::uint16_t>((R2 - W) / 2)));
    CHECK_EQ(d.read_idata(0x43), hi(static_cast<std::uint16_t>((R2 - W) / 2)));
    CHECK_EQ(d.read_idata(0x44), lo(static_cast<std::uint16_t>((R3 - X) / 2)));
    // Overflow 0xFFFF -> 0x0000 at tick 65536 after the T1CNTL write
    // (Model-defined: OVFIF on the step to 0x0000).
    CHECK_EQ(run_to(d, VT1, X + 140000, "t1 overflow"), loop_boundary(L, X + 131072) + 2);
    CHECK_EQ(d.read_sfr(T1STAT), 0x20); // ISR has not run yet
    CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0);
    run_to(d, lay.l, X + 140000, "t1 overflow return");
    CHECK_EQ(d.read_idata(0x50), 0x20);
    CHECK_EQ(d.read_sfr(T1STAT), 0x00);
}

// SWRU191F 9.5 up-and-down mode; Table 9-1 initial outputs; CMP 011/100 in
// up-and-down mode act on compare-up / compare-down.
void test_t1_up_down() {
    Program p;
    p.mov(T1CC0L, 40).mov(T1CC0H, 0).mov(T1CC1L, 10).mov(T1CC1H, 0).mov(T1CC2L, 10).mov(T1CC2H, 0);
    p.mov(T1CCTL0, 0x00).mov(T1CCTL1, 0x1c).mov(T1CCTL2, 0x24).mov(T1CNTL, 0);
    p.mov(IEN1, 0x02).setb(BitEa);
    const std::uint16_t w = p.here();
    p.mov(T1CTL, 0x03);
    p.isr(VT1, {0x85, T1STAT, 0x50, 0x53, T1STAT, 0xdf, 0x32});
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 1000, "updown w");
    CHECK(!t1(d).output(1)); // Table 9-1: 011 -> 0
    CHECK(t1(d).output(2));  // Table 9-1: 100 -> 1
    auto out = [&d](unsigned n) { return t1(d).output(n); };
    check_edges(d, W, 2, {{1, 10, true}, {2, 10, false}}, out, "updown up");
    if (at(d, W + 100, "updown count"))
        CHECK_EQ(d.read_sfr(T1CNTL), 30); // tick 50: 80 - 50
    check_edges(d, W, 2, {{1, 70, false}, {2, 70, true}}, out, "updown down");
    // OVFIF when the count reaches 0 (tick 80); T1OVFIM = 1 by reset.
    CHECK_EQ(run_to(d, VT1, W + 1000, "updown ovf"), W + 162);
    check_edges(d, W, 2, {{1, 90, true}, {2, 90, false}}, out, "updown up 2");
    CHECK_EQ(d.read_idata(0x50), 0x26);
    CHECK_EQ(d.read_sfr(T1STAT), 0x06);
    // Channel flags remain but their IM = 0: no re-assertion (9.10).
    CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0);
    CHECK_EQ(run_to(d, VT1, W + 1000, "updown ovf 2"), W + 322);
}

// SWRU191F 9.8 / Table 9-1: every compare output mode on all five channels.
void test_t1_output_modes() {
    auto out = [](Cc2530F256 &d) { return [&d](unsigned n) { return t1(d).output(n); }; };
    {
        Program p;
        p.mov(T1CC0L, 100).mov(T1CC0H, 0).mov(T1CCTL0, 0x14);          // toggle
        p.mov(T1CC1L, 20).mov(T1CC1H, 0).mov(T1CCTL1, 0x04);           // set
        p.mov(T1CC2L, 30).mov(T1CC2H, 0).mov(T1CCTL2, 0x0c);           // clear
        p.xw(xt1ccl(3), 40).xw(xt1ccl(3) + 1u, 0).xw(xt1cctl(3), 0x14); // toggle
        p.xw(xt1ccl(4), 50).xw(xt1ccl(4) + 1u, 0).xw(xt1cctl(4), 0x1c); // set, clear on 0
        p.mov(T1CNTL, 0);
        const std::uint16_t w = p.here();
        p.mov(T1CTL, 0x02);
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "modes A w");
        const bool initial[5] = {false, false, true, false, false};
        for (unsigned n = 0; n < 5; ++n)
            CHECK_EQ(t1(d).output(n), initial[n]);
        check_edges(d, W, 2,
                    {{1, 20, true}, {2, 30, false}, {3, 40, true}, {4, 50, true}, {0, 100, true},
                     {4, 101, false}, {3, 141, false}, {4, 151, true}, {0, 201, false},
                     {4, 202, false}},
                    out(d), "modes A");
        CHECK_EQ(d.read_sfr(T1STAT), 0x1f);
        // CMP 111 initializes the output and keeps CMP (SWRU191F 9.12).
        CHECK(t1(d).output(1));
        d.write_sfr(T1CCTL1, 0x7c);
        CHECK(!t1(d).output(1));
        CHECK_EQ(d.read_sfr(T1CCTL1), 0x44);
    }
    {
        Program p;
        p.mov(T1CC0L, 100).mov(T1CC0H, 0);
        p.mov(T1CC1L, 30).mov(T1CC1H, 0).mov(T1CCTL1, 0x2c);           // clr@CC0, set@CC1
        p.mov(T1CC2L, 60).mov(T1CC2H, 0).mov(T1CCTL2, 0x34);           // set@CC0, clr@CC2
        p.xw(xt1ccl(3), 70).xw(xt1ccl(3) + 1u, 0).xw(xt1cctl(3), 0x24); // clear, set on 0
        p.xw(xt1ccl(4), 80).xw(xt1ccl(4) + 1u, 0).xw(xt1cctl(4), 0x0c); // clear
        p.mov(T1CNTL, 0);
        const std::uint16_t w = p.here();
        p.mov(T1CTL, 0x02);
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "modes B w");
        CHECK(!t1(d).output(1));
        CHECK(t1(d).output(2));
        CHECK(t1(d).output(3));
        CHECK(t1(d).output(4));
        check_edges(d, W, 2,
                    {{1, 30, true}, {2, 60, false}, {3, 70, false}, {4, 80, false}, {1, 100, false},
                     {2, 100, true}, {3, 101, true}},
                    out(d), "modes B");
    }
}

// SWRU191F 9.8: T1CCnL is buffered until T1CCnH is written, and a compare
// value takes effect when the counter is 0x0000.
void test_t1_compare_buffering() {
    Program p;
    p.mov(T1CC0L, 100).mov(T1CC0H, 0).mov(T1CCTL0, 0x04);
    p.mov(T1CC1L, 20).mov(T1CC1H, 0).mov(T1CCTL1, 0x14).mov(T1CNTL, 0);
    const std::uint16_t w = p.here();
    p.mov(T1CTL, 0x02);
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 2000, "buffering w");
    auto out = [&d](unsigned n) { return t1(d).output(n); };
    check_edges(d, W, 2, {{1, 20, true}}, out, "buffering 1");
    if (at(d, W + 100, "buffering write")) {
        d.write_sfr(T1CC1L, 40);
        d.write_sfr(T1CC1H, 0);
        d.write_sfr(T1CC0L, 60);
        d.write_sfr(T1CC0H, 0);
        CHECK_EQ(word_sfr(d, T1CC1L), 20);
        CHECK_EQ(word_sfr(d, T1CC0L), 100);
    }
    if (at(d, W + 201, "buffering old")) {
        CHECK_EQ(word_sfr(d, T1CC1L), 20);
        CHECK_EQ(word_sfr(d, T1CC0L), 100);
    }
    // Tick 101: the counter wraps to 0 and the new values take effect.
    if (at(d, W + 202, "buffering new")) {
        CHECK_EQ(word_sfr(d, T1CC1L), 40);
        CHECK_EQ(word_sfr(d, T1CC0L), 60);
    }
    // Toggle at count 40 (tick 141), period 61 ticks: next toggle tick 202.
    check_edges(d, W, 2, {{1, 141, false}, {1, 202, true}}, out, "buffering 2");
    // Channel 2 is in capture mode: the value applies at the T1CC2H write.
    d.write_sfr(T1CC2L, 0x11);
    CHECK_EQ(d.read_sfr(T1CC2L), 0);
    d.write_sfr(T1CC2H, 0x22);
    CHECK_EQ(word_sfr(d, T1CC2L), 0x2211);
}

// SWRU191F 9.7 input capture (rising / falling / both edges), 9.10 flags.
// Model-defined: inputs arrive through Timer1::input_changed (pin routing
// is the I/O model's job); no capture while the timer is suspended.
void test_t1_capture() {
    Program p;
    p.xw(xt1ccl(4), 0x00).xw(xt1ccl(4) + 1u, 0x80);
    p.mov(T1CCTL1, 0x41).mov(T1CCTL2, 0x42).xw(xt1cctl(3), 0x43).xw(xt1cctl(4), 0x44);
    const std::uint16_t w = p.here();
    p.mov(T1CTL, 0x01);
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 2000, "capture w");
    t1(d).input_changed(1, true);
    CHECK_EQ(word_sfr(d, T1CC1L), 0);
    CHECK_EQ(d.read_sfr(T1STAT), 0);
    if (at(d, W + 101, "capture 1")) {
        t1(d).input_changed(1, true);
        CHECK_EQ(word_sfr(d, T1CC1L), 50);
        CHECK_EQ(d.read_sfr(T1STAT), 0x02);
        CHECK(d.read_sfr(IRCON) & IrconT1if);
        t1(d).input_changed(1, false); // falling edge not selected
        CHECK_EQ(word_sfr(d, T1CC1L), 50);
    }
    if (at(d, W + 200, "capture 2r")) {
        t1(d).input_changed(2, true); // rising edge not selected
        CHECK_EQ(word_sfr(d, T1CC2L), 0);
    }
    if (at(d, W + 300, "capture 2f")) {
        t1(d).input_changed(2, false);
        CHECK_EQ(word_sfr(d, T1CC2L), 150);
        t1(d).input_changed(3, true);
        CHECK_EQ(word_xdata(d, xt1ccl(3)), 150);
    }
    if (at(d, W + 1001, "capture 3f")) {
        t1(d).input_changed(3, false);
        CHECK_EQ(word_xdata(d, xt1ccl(3)), 0x01f4);
        t1(d).input_changed(4, true); // compare mode: no capture
        t1(d).input_changed(4, false);
        CHECK_EQ(word_xdata(d, xt1ccl(4)), 0x8000);
        t1(d).input_changed(0, true); // CAP = 00: no capture
        CHECK_EQ(word_sfr(d, T1CC0L), 0);
    }
    CHECK_EQ(d.read_sfr(T1STAT), 0x0e);
    // SWRU191F 9.11: Timer 1 DMA triggers are compare events only.
    for (unsigned n = 0; n < 3; ++n)
        CHECK_EQ(TimersTestAccess::t1(t1(d), n), 0u);
}

// SWRU191F 9.10, 9.12 T1STAT (R/W0) and the IRCON.T1IF re-assertion rule.
void test_t1_stat_semantics() {
    Program p;
    p.mov(TIMIF, 0x00); // T1OVFIM = 0
    p.mov(T1CC0L, 5).mov(T1CC0H, 0).mov(T1CC1L, 1).mov(T1CC1H, 0).mov(T1CC2L, 2).mov(T1CC2H, 0);
    p.mov(T1CCTL0, 0x00).mov(T1CCTL1, 0x04).mov(T1CCTL2, 0x04);
    const std::uint16_t w = p.here();
    p.mov(T1CTL, 0x03);
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 2000, "t1stat w");
    if (at(d, W + 22, "t1stat flags")) {
        CHECK_EQ(d.read_sfr(T1STAT), 0x26); // OVF (tick 10), CH2, CH1
        CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0); // every mask is 0
        d.write_sfr(T1CTL, 0x00);                    // suspend
        CHECK_EQ(d.read_sfr(T1CNTL), 1);
    }
    if (at(d, W + 222, "t1stat suspended"))
        CHECK_EQ(d.read_sfr(T1CNTL), 1);
    d.write_sfr(T1STAT, 0xff); // writing 1 has no effect
    CHECK_EQ(d.read_sfr(T1STAT), 0x26);
    CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0);
    d.write_sfr(T1CCTL2, 0x44); // unmasking alone does not set T1IF
    CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0);
    d.write_sfr(T1STAT, 0xfb); // clear CH2IF: CH1IF/OVFIF remain, masked
    CHECK_EQ(d.read_sfr(T1STAT), 0x22);
    CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0);
    d.write_sfr(TIMIF, 0x40); // T1OVFIM = 1
    CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0);
    d.write_sfr(T1STAT, 0xfd); // clear CH1IF while unmasked OVFIF remains
    CHECK_EQ(d.read_sfr(T1STAT), 0x20);
    CHECK(d.read_sfr(IRCON) & IrconT1if);
    d.write_sfr(IRCON, 0x00);
    d.write_sfr(T1STAT, 0x00); // nothing remains
    CHECK_EQ(d.read_sfr(T1STAT), 0);
    CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0);
}

// SWRU191F 9.11: T1_CH0..T1_CH2 DMA triggers on channel compare events.
void test_t1_dma_triggers() {
    Program p;
    p.mov(T1CC0L, 99).mov(T1CC0H, 0).mov(T1CCTL0, 0x04);
    p.mov(T1CC1L, 10).mov(T1CC1H, 0).mov(T1CCTL1, 0x04);
    p.mov(T1CC2L, 20).mov(T1CC2H, 0).mov(T1CCTL2, 0x04);
    p.xw(xt1ccl(3), 30).xw(xt1ccl(3) + 1u, 0).xw(xt1cctl(3), 0x04);
    p.xw(xt1ccl(4), 40).xw(xt1ccl(4) + 1u, 0).xw(xt1cctl(4), 0x04);
    const std::uint16_t w = p.here();
    p.mov(T1CTL, 0x02);
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 2000, "t1 dma w");
    if (at(d, W + 597, "t1 dma 597")) {
        CHECK_EQ(TimersTestAccess::t1(t1(d), 0), 2u); // ticks 99, 199
        CHECK_EQ(TimersTestAccess::t1(t1(d), 1), 3u); // ticks 10, 110, 210
        CHECK_EQ(TimersTestAccess::t1(t1(d), 2), 3u);
    }
    if (at(d, W + 598, "t1 dma 598"))
        CHECK_EQ(TimersTestAccess::t1(t1(d), 0), 3u); // tick 299
    CHECK_EQ(d.read_sfr(T1STAT), 0x1f);
}

// SWRU191F 4.4 CLKCONCMD.TICKSPD divides the timer tick (Timer 1, 3, 4).
void test_tickspd() {
    {
        Program p;
        p.mov(CLKCONCMD, 0xd9); // OSC32K 1, OSC 1, TICKSPD 011 (4 MHz), CLKSPD 001
        p.mov(T1CC0L, 50).mov(T1CC0H, 0).mov(T1CCTL0, 0x44).mov(IEN1, 0x0a).setb(BitEa);
        const std::uint16_t w = p.here();
        p.mov(T1CTL, 0x01);
        const std::uint16_t w3 = p.here();
        p.mov(T3CTL, 0x18);
        p.isr(VT1, {0x53, T1STAT, 0xfe, 0x32});
        p.isr(VT3, {0x32});
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "tickspd w");
        CHECK_EQ(d.read_sfr(CLKCONSTA), 0xd9);
        const std::uint64_t W3 = run_to(d, w3, 2000, "tickspd w3");
        CHECK_EQ(run_to(d, VT1, W + 4000, "tickspd t1"), W + 402);    // 50 ticks of 8 cycles
        CHECK_EQ(run_to(d, VT3, W3 + 4000, "tickspd t3"), W3 + 2050); // 256 ticks of 8
    }
    {
        Program p;
        p.mov(T1CC0L, 100).mov(T1CC0H, 0).mov(T1CCTL0, 0x44).mov(IEN1, 0x02).setb(BitEa);
        const std::uint16_t w = p.here();
        p.mov(T1CTL, 0x01);
        p.isr(VT1, {0x53, T1STAT, 0xfe, 0x32});
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "tickspd change w");
        if (at(d, W + 40, "tickspd change")) {
            CHECK_EQ(d.read_sfr(T1CNTL), 20);
            d.write_sfr(CLKCONCMD, 0xd9);
            CHECK_EQ(d.read_sfr(CLKCONSTA), 0xd9);
        }
        // Model-defined: the tick re-anchors at the clock change.
        if (at(d, W + 47, "tickspd 47"))
            CHECK_EQ(d.read_sfr(T1CNTL), 20);
        if (at(d, W + 48, "tickspd 48"))
            CHECK_EQ(d.read_sfr(T1CNTL), 21);
        CHECK_EQ(run_to(d, VT1, W + 2000, "tickspd change t1"), W + 40 + 80 * 8 + 2);
    }
}

// SWRU191F 9.9.2 IR generation: with IRCTL.IRGEN = 1 Timer 1 ticks on the
// Timer 3 channel 1 compare, and the channel 1 output is T1 ch1 AND T3 ch1.
// Model-defined: T1CTL.DIV still divides those ticks (DIV = 1 here).
void test_t1_ir_generation() {
    Program p;
    p.mov(T3CC0, 9).mov(T3CC1, 3).mov(T3CCTL0, 0x00).mov(T3CCTL1, 0x24);
    p.mov(T1CC0L, 4).mov(T1CC0H, 0).mov(T1CC1L, 2).mov(T1CC1H, 0);
    p.mov(T1CCTL0, 0x04).mov(T1CCTL1, 0x24);
    p.xw(XIrctl, 0x01).mov(T1CNTL, 0).mov(T1CTL, 0x02);
    const std::uint16_t w3 = p.here();
    p.mov(T3CTL, 0x16); // start, clear, modulo
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W3 = run_to(d, w3, 2000, "irgen w3");
    CHECK_EQ(d.peek_xdata(XIrctl), 0x01);
    CHECK_EQ(d.read_sfr(T1CNTL), 0);
    // T3 ch1 (clear on compare 3, set on 0): high for ticks 0-2 of 10.
    // T1 ch1 (clear on compare 2, set on 0) counts T3 ch1 compares at T3
    // ticks 3, 13, 23, 33, 43 (W3 + 6, 26, 46, 66, 86).
    struct Sample {
        std::uint64_t t;
        bool ir;
        int count; // -1: not checked
    };
    for (const Sample s : {Sample{5, true, 0}, Sample{6, false, 1}, Sample{20, true, -1},
                           Sample{26, false, 2}, Sample{40, false, -1}, Sample{66, false, 4},
                           Sample{86, false, 0}, Sample{100, true, -1}})
        if (at(d, W3 + s.t, "irgen")) {
            CHECK_EQ(t1(d).output(1), s.ir);
            if (s.count >= 0)
                CHECK_EQ(d.read_sfr(T1CNTL), s.count);
        }
}

// ---------------------------------------------------------------------------
// Timer 3 / Timer 4
// ---------------------------------------------------------------------------

// T3 free-running with overflow interrupt; ISR: CLR T3OVFIF; INC 0x60; RETI.
Program t3_overflow_program(std::uint16_t &w) {
    Program p;
    p.mov(IEN1, 0x08).setb(BitEa);
    w = p.here();
    p.mov(T3CTL, 0x18); // START, OVFIM, DIV 1, free-running
    p.isr(VT3, {0xc2, BitT3ovfif, 0x05, 0x60, 0x32});
    return p;
}

// SWRU191F 10.2.1 free-running, 10.6 interrupts, 10.8 TxCTL.DIV.
void test_t34_overflow_isr() {
    {
        std::uint16_t w = 0;
        const Program p = t3_overflow_program(w);
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "t3 ovf w");
        CHECK_EQ(run_to(d, VT3, W + 2000, "t3 ovf 1"), W + 514); // 256 ticks of 2
        CHECK_EQ(d.read_sfr(TIMIF), 0x41);
        CHECK_EQ(d.read_sfr(IRCON) & IrconT3if, 0); // cleared on vector entry
        if (at(d, W + 600, "t3 ovf isr done")) {
            CHECK_EQ(d.read_sfr(TIMIF), 0x40);
            CHECK_EQ(d.read_idata(0x60), 1);
        }
        CHECK_EQ(run_to(d, VT3, W + 2000, "t3 ovf 2"), W + 1026);
    }
    {
        Program p;
        p.mov(IEN1, 0x10).setb(BitEa);
        const std::uint16_t w = p.here();
        p.mov(T4CTL, 0x98); // DIV 100 (/16), START, OVFIM
        p.isr(VT4, {0xc2, BitT4ovfif, 0x05, 0x61, 0x32});
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "t4 ovf w");
        if (at(d, W + 8000, "t4 count"))
            CHECK_EQ(d.read_sfr(T4CNT), 250); // 8000 / 32
        CHECK_EQ(run_to(d, VT4, W + 20000, "t4 ovf"), W + 8194);
        CHECK_EQ(d.read_sfr(TIMIF), 0x48);
    }
}

// SWRU191F 10.2.2-10.2.4 down / modulo / up-and-down modes, 10.5 and
// Table 10-1 compare outputs (CLR initializes them), 10.7 DMA triggers.
void test_t34_modes() {
    {
        // Down mode: loads T4CC0 at start, counts down, OVFIF at 0x00.
        // Model-defined: the counter then stays at 0x00.
        Program p;
        p.mov(T4CC0, 50).mov(IEN1, 0x10).setb(BitEa);
        const std::uint16_t w = p.here();
        p.mov(T4CTL, 0x19);
        p.isr(VT4, {0xc2, BitT4ovfif, 0x05, 0x61, 0x32});
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "t4 down w");
        for (const auto &[t, v] : {std::pair{4u, 48}, std::pair{5u, 48}, std::pair{6u, 47},
                                  std::pair{99u, 1}})
            if (at(d, W + t, "t4 down"))
                CHECK_EQ(d.read_sfr(T4CNT), v);
        CHECK_EQ(run_to(d, VT4, W + 2000, "t4 down ovf"), W + 102);
        if (at(d, W + 2000, "t4 down halted")) {
            CHECK_EQ(d.read_sfr(T4CNT), 0);
            CHECK_EQ(d.read_idata(0x61), 1);
            CHECK_EQ(d.read_sfr(TIMIF) & 0x08, 0);
        }
    }
    auto out3 = [](Cc2530F256 &d) { return [&d](unsigned n) { return t34(d).output(3, n); }; };
    {
        // Modulo: 0..99. ch0 toggle at 99, ch1 clear at 50 / set at 0x00.
        Program p;
        p.mov(T3CC0, 99).mov(T3CC1, 50).mov(T3CCTL0, 0x14).mov(T3CCTL1, 0x34);
        const std::uint16_t w = p.here();
        p.mov(T3CTL, 0x16); // START, CLR, modulo, OVFIM 0
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "t3 modulo w");
        if (at(d, W + 4, "t3 modulo init")) {
            CHECK(!t34(d).output(3, 0)); // Table 10-1: 010 -> 0
            CHECK(t34(d).output(3, 1));  // Table 10-1: 110 -> 1
        }
        check_edges(d, W, 2,
                    {{1, 50, false}, {0, 99, true}, {1, 100, true}, {1, 150, false},
                     {0, 199, false}},
                    out3(d), "t3 modulo");
        if (at(d, W + 400, "t3 modulo flags")) {
            CHECK_EQ(d.read_sfr(TIMIF) & 0x01, 0); // no overflow below 0xFF
            CHECK_EQ(d.read_sfr(TIMIF) & 0x06, 0x06);
            CHECK_EQ(TimersTestAccess::t34(t34(d), 0), 2u);
            CHECK_EQ(TimersTestAccess::t34(t34(d), 1), 2u);
        }
    }
    {
        // Free-running: ch0 set on compare / clear on 0, ch1 set on
        // compare / clear on 0xFF.
        Program p;
        p.mov(T3CC0, 0x40).mov(T3CC1, 0x80).mov(T3CCTL0, 0x1c).mov(T3CCTL1, 0x2c);
        const std::uint16_t w = p.here();
        p.mov(T3CTL, 0x14);
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "t3 free w");
        check_edges(d, W, 2, {{0, 64, true}, {1, 128, true}, {1, 255, false}}, out3(d),
                    "t3 free");
        if (at(d, W + 511, "t3 free 511")) {
            CHECK_EQ(d.read_sfr(TIMIF) & 0x01, 0);
            CHECK(t34(d).output(3, 0));
        }
        if (at(d, W + 512, "t3 free 512")) {
            CHECK_EQ(d.read_sfr(TIMIF) & 0x01, 0x01); // 0xFF -> 0x00
            CHECK(!t34(d).output(3, 0));
            CHECK_EQ(d.read_sfr(IRCON) & IrconT3if, 0); // OVFIM = 0, IM = 0
        }
    }
    {
        // Up-and-down: 0..20..0; ch1 CMP 011 sets on compare-up, clears on
        // compare-down (Table 10-1); OVFIF at 0x00 (tick 40).
        Program p;
        p.mov(T3CC0, 20).mov(T3CC1, 5).mov(T3CCTL0, 0x00).mov(T3CCTL1, 0x1c);
        const std::uint16_t w = p.here();
        p.mov(T3CTL, 0x1f);
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t W = run_to(d, w, 2000, "t3 updown w");
        check_edges(d, W, 2, {{1, 5, true}, {1, 35, false}}, out3(d), "t3 updown");
        if (at(d, W + 79, "t3 updown 79"))
            CHECK_EQ(d.read_sfr(TIMIF) & 0x01, 0);
        if (at(d, W + 80, "t3 updown 80")) {
            CHECK_EQ(d.read_sfr(TIMIF) & 0x01, 0x01);
            CHECK(d.read_sfr(IRCON) & IrconT3if);
        }
    }
}

// SWRU191F 10.6, 10.8 TIMIF: flags R/W0, T1OVFIM R/W, bit 7 R0; IRCON.T3IF
// re-asserted when a flag is cleared while another unmasked one remains.
void test_timif_semantics() {
    Program p;
    p.mov(T3CC0, 0x10).mov(T3CCTL0, 0x44);
    const std::uint16_t w = p.here();
    p.mov(T3CTL, 0x10); // START, OVFIM 0, free-running
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 2000, "timif w");
    if (at(d, W + 600, "timif flags")) {
        CHECK_EQ(d.read_sfr(TIMIF), 0x43); // CH0 at tick 16, OVF at tick 256
        CHECK(d.read_sfr(IRCON) & IrconT3if);
    }
    d.write_sfr(IRCON, 0x00);
    d.write_sfr(T3CTL, 0x00);
    d.write_sfr(TIMIF, 0xff);
    CHECK_EQ(d.read_sfr(TIMIF), 0x43);
    CHECK_EQ(d.read_sfr(IRCON) & IrconT3if, 0);
    d.write_sfr(TIMIF, 0xc2); // clear T3OVFIF (masked); T3CH0IF (IM = 1) remains
    CHECK_EQ(d.read_sfr(TIMIF), 0x42);
    CHECK(d.read_sfr(IRCON) & IrconT3if);
    d.write_sfr(IRCON, 0x00);
    d.write_sfr(TIMIF, 0x00);
    CHECK_EQ(d.read_sfr(TIMIF), 0x00);
    CHECK_EQ(d.read_sfr(IRCON) & IrconT3if, 0);
    d.write_sfr(TIMIF, 0x80);
    CHECK_EQ(d.read_sfr(TIMIF), 0x00);
    d.write_sfr(TIMIF, 0x40);
    CHECK_EQ(d.read_sfr(TIMIF), 0x40);
}

// SWRU191F 10.4 input capture, 10.6 flags, 10.7 DMA on capture.
void test_t34_capture() {
    Program p;
    p.mov(T3CCTL0, 0x41).mov(T4CCTL1, 0x43);
    const std::uint16_t w3 = p.here();
    p.mov(T3CTL, 0x10);
    const std::uint16_t w4 = p.here();
    p.mov(T4CTL, 0x10);
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W3 = run_to(d, w3, 2000, "t34 capture w3");
    t34(d).input_changed(3, 0, true); // timer not started (Model-defined)
    CHECK_EQ(d.read_sfr(T3CC0), 0);
    CHECK_EQ(d.read_sfr(TIMIF), 0x40);
    const std::uint64_t W4 = run_to(d, w4, 2000, "t34 capture w4");
    const std::uint64_t B = W3 + 101;
    if (at(d, B, "t34 capture 1")) {
        t34(d).input_changed(3, 0, true);
        CHECK_EQ(d.read_sfr(T3CC0), 50);
        CHECK_EQ(d.read_sfr(TIMIF), 0x42);
        CHECK(d.read_sfr(IRCON) & IrconT3if);
        CHECK_EQ(TimersTestAccess::t34(t34(d), 0), 1u);
        t34(d).input_changed(3, 0, false); // falling edge not selected
        CHECK_EQ(d.read_sfr(T3CC0), 50);
        CHECK_EQ(TimersTestAccess::t34(t34(d), 0), 1u);
        t34(d).input_changed(4, 1, false);
        CHECK_EQ(d.read_sfr(T4CC1), lo(static_cast<std::uint16_t>((B - W4) / 2)));
        CHECK_EQ(d.read_sfr(TIMIF), 0x62);
        CHECK(d.read_sfr(IRCON) & IrconT4if);
        CHECK_EQ(TimersTestAccess::t34(t34(d), 3), 1u);
    }
    if (at(d, B + 600, "t34 capture 2")) {
        t34(d).input_changed(4, 1, true);
        CHECK_EQ(d.read_sfr(T4CC1), lo(static_cast<std::uint16_t>((B + 600 - W4) / 2)));
        CHECK_EQ(TimersTestAccess::t34(t34(d), 3), 2u);
    }
    CHECK_EQ(TimersTestAccess::t34(t34(d), 1), 0u);
    CHECK_EQ(TimersTestAccess::t34(t34(d), 2), 0u);
}

// ---------------------------------------------------------------------------
// Sleep timer
// ---------------------------------------------------------------------------

std::uint32_t xdata24(const Cc2530F256 &d, std::uint16_t low) {
    return static_cast<std::uint32_t>(d.peek_xdata(low)) |
           static_cast<std::uint32_t>(d.peek_xdata(static_cast<std::uint16_t>(low + 1))) << 8 |
           static_cast<std::uint32_t>(d.peek_xdata(static_cast<std::uint16_t>(low + 2))) << 16;
}

// SWRU191F 11.2 compare -> IRCON.STIF, 4.3/Table 4-1 the sleep timer wakes
// PM2; Timer 1 (HF clock) does not count in PM2 (4.1).
void test_st_wake_from_pm2() {
    Program p;
    const std::uint16_t w = p.here();
    p.mov(T1CTL, 0x01);
    p.mov(ST2, 0).mov(ST1, 0).mov(ST0, 10).copy(STLOAD, 0x40).mov(IEN0, 0xa0);
    p.enter_power_mode(0x06); // OSC_PD = 1, MODE = PM2
    p.isr(VSt, {0xc2, BitStif, 0x05, 0x60, 0x32});
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 2000, "st pm2 w");
    // Count 10 starts at period_start(10) = 9766; its rising edge (the
    // compare event) is at rising_edge(10) = 10254.
    if (at(d, 10253, "st pm2 before")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Pm2);
        CHECK_EQ(d.read_sfr(IRCON) & IrconStif, 0);
        CHECK_EQ(st(d).count(), 10u);
    }
    // Wake at 10254, NOP after the PCON write, then LCALL to the vector.
    CHECK_EQ(run_to(d, VSt, 20000, "st pm2 vector"), 10257u);
    CHECK(d.read_sfr(IRCON) & IrconStif); // STIF is not cleared by hardware
    CHECK_EQ(TimersTestAccess::st(st(d)), 1u);
    CHECK_EQ(st(d).count(), 10u);
    CHECK_EQ(d.read_idata(0x40), 0); // STLOAD.LDRDY = 0 right after the load
    const auto &diag = d.diagnostics();
    CHECK_EQ(diag.power_mode_entries_by_mode[PmIndex2], 1u);
    const std::uint64_t d2 = diag.power_mode_cycles_by_mode[PmIndex2];
    CHECK(d2 > 10000);
    CHECK_EQ(d.read_sfr(T1CNTL), lo(static_cast<std::uint16_t>((10257 - d2 - W) / 2)));
    // SWRU191F 11.1: after PM2, ST0 is valid only after the next positive
    // 32 kHz edge (edge 11 at 11231). Model-defined: until then it shows the
    // count at the PM2 entry (0).
    if (at(d, 11230, "st stale"))
        CHECK_EQ(d.read_sfr(ST0), 0);
    if (at(d, 11231, "st valid"))
        CHECK_EQ(d.read_sfr(ST0), 11);
    CHECK_EQ(d.read_idata(0x60), 1);
    CHECK_EQ(d.diagnostics().idle_alignment_violations, 0u);
}

// SWRU191F 4.1, 11.1: the sleep timer does not run in PM3 and cannot wake
// it; its value is not preserved. Model-defined: it restarts at 0 on wake.
void test_st_pm3() {
    Program p;
    p.mov(ST2, 0).mov(ST1, 0).mov(ST0, 10).mov(IEN1, 0x20).mov(IEN0, 0xa0);
    p.op({0x7e, 0x06, 0x7f, 0x00, 0xdf, 0xfe, 0xde, 0xfa}); // ~6 x 256 DJNZ
    p.enter_power_mode(0x07);
    const std::uint16_t ls = p.here();
    p.sjmp_self();
    p.isr(VP0, {0xc2, BitP0if, 0x05, 0x60, 0x32});
    p.isr(VSt, {0xc2, BitStif, 0x05, 0x61, 0x32});
    Cc2530F256 d;
    load(d, p.flash);
    std::uint32_t c0 = 0;
    if (at(d, 150000, "st pm3 150000")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Pm3);
        c0 = st(d).count();
        CHECK(c0 >= 1 && c0 < 10); // loaded (edge 0), entered PM3 before 10
        CHECK_EQ(d.read_sfr(IRCON) & IrconStif, 0);
    }
    if (at(d, 200000, "st pm3 200000")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Pm3);
        CHECK_EQ(st(d).count(), c0);
        CHECK_EQ(d.read_sfr(IRCON) & IrconStif, 0);
    }
    d.schedule_interrupt_flag_for_test(250000, 13); // P0IF wakes PM3
    CHECK_EQ(run_to(d, VP0, 300000, "st pm3 p0"), 250003u);
    const std::uint64_t d3 = d.diagnostics().power_mode_cycles_by_mode[PmIndex3];
    const std::uint64_t base = lf_ticks(250000 - d3);
    CHECK_EQ(base, c0);
    CHECK(st(d).count() <= 1u);
    const std::uint64_t L = run_to(d, ls, 300000, "st pm3 loop");
    CHECK_EQ(d.read_idata(0x60), 1);
    CHECK_EQ(run_to(d, VSt, 300000, "st pm3 compare"),
             loop_boundary(L, rising_edge(base + 10) + d3) + 2);
    CHECK_EQ(st(d).count(), 10u);
    CHECK_EQ(TimersTestAccess::st(st(d)), 1u);
}

// Compare 0x000123 loaded through ST2, ST1, ST0; a second load while
// LDRDY = 0; poll LDRDY (lb: after it), enable EA + STIE, SJMP $ (ls). The
// ST ISR stores ST0, ST1, ST2 (read in that order) at 0x50-0x52.
Program st_load_program(std::uint16_t &lb, std::uint16_t &ls) {
    Program p;
    p.mov(ST2, 0).mov(ST1, 1).mov(ST0, 0x23).copy(STLOAD, 0x40);
    p.mov(ST1, 0).mov(ST0, 5); // LDRDY = 0: ignored
    p.op({0xe5, STLOAD, 0x30, BitAcc0, 0xfb}); // wait for LDRDY
    lb = p.here();
    p.mov(IEN0, 0xa0);
    ls = p.here();
    p.sjmp_self();
    p.isr(VSt, {0x02, 0x08, 0x00});
    p.isr(0x0800, {0xe5, ST0, 0xf5, 0x50, 0x85, ST1, 0x51, 0x85, ST2, 0x52, 0xc2, BitStif, 0x05,
                   0x60, 0x32});
    return p;
}

// SWRU191F 11.1 ST0 read latches ST1/ST2; 11.2/11.4 compare load sequence
// ST2, ST1, ST0 and STLOAD.LDRDY. Model-defined: the load completes on the
// next positive 32 kHz edge; ST0 writes while LDRDY = 0 are ignored.
void test_st_load_and_latch() {
    std::uint16_t lb = 0, ls = 0;
    const Program p = st_load_program(lb, ls);
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t B = run_to(d, lb, 2000, "st load ready");
    CHECK(B > 489 && B <= 505); // rising edge 0 at 489, poll loop 6 cycles
    CHECK_EQ(d.read_idata(0x40), 0);
    CHECK_EQ(d.read_sfr(STLOAD), 0x01);
    const std::uint64_t L = run_to(d, ls, 2000, "st load loop");
    // Compare value 0x000123 = 291: rising edge 291 at 284668.
    CHECK_EQ(run_to(d, VSt, 400000, "st load compare"), loop_boundary(L, 284668) + 2);
    CHECK_EQ(TimersTestAccess::st(st(d)), 1u);
    run_until(d, 500000);
    const std::uint64_t t = now(d);
    CHECK_EQ(d.read_idata(0x60), 1);
    CHECK_EQ(d.read_idata(0x50), 0x23);
    CHECK_EQ(d.read_idata(0x51), 0x01);
    CHECK_EQ(d.read_idata(0x52), 0x00);
    CHECK_EQ(d.read_sfr(ST1), 0x01); // still the value latched by the ISR
    CHECK_EQ(d.read_sfr(ST0), lo(static_cast<std::uint16_t>(lf_ticks(t))));
    CHECK_EQ(st(d).count(), lf_ticks(t));
    CHECK_EQ(TimersTestAccess::st(st(d)), 1u);
}

// SWRU191F 11.3 capture: STCC selects the pin, the PICTL edge triggers,
// STCV = count + 1, valid (STCS.VALID) on the next positive 32 kHz edge;
// no new capture while VALID = 1; the pin cannot be switched while enabled.
void test_st_capture() {
    Program p;
    p.xw(XStcc, 0x02); // PORT 0, PIN 2
    p.at(0x1c00);
    p.xr(XStcs, 0x43).xw(XStcs, 0x01).xr(XStcs, 0x44).xw(XStcs, 0x00).xr(XStcs, 0x45);
    p.xw(XStcc, 0x0b).xr(XStcc, 0x46).xw(XStcc, 0x1f).xr(XStcc, 0x47);
    p.xw(XStcc, 0x0b).xr(XStcc, 0x48);
    Cc2530F256 d;
    load(d, p.flash);
    CHECK_EQ(d.peek_xdata(XStcc), 0x1f);
    if (at(d, 5000, "st capture 1"))
        st(d).input_changed(0, 2, true); // count 5 -> STCV 6
    if (at(d, 5371, "st capture 1 pending"))
        CHECK_EQ(d.peek_xdata(XStcs), 0);
    if (at(d, 5372, "st capture 1 valid")) { // rising edge 5
        CHECK_EQ(d.peek_xdata(XStcs), 1);
        CHECK_EQ(xdata24(d, XStcv0), 6u);
    }
    if (at(d, 5500, "st capture 2"))
        st(d).input_changed(0, 2, true); // VALID = 1: ignored
    const std::uint64_t bk = run_to(d, 0x1c00, 9000, "st capture block");
    CHECK(bk > 5500);
    CHECK_EQ(xdata24(d, XStcv0), 6u);
    if (at(d, 8000, "st capture block done")) {
        CHECK_EQ(d.read_idata(0x43), 1);
        CHECK_EQ(d.read_idata(0x44), 1);    // VALID is R/W0
        CHECK_EQ(d.read_idata(0x45), 0);
        CHECK_EQ(d.read_idata(0x46), 0x02); // switch while enabled: ignored
        CHECK_EQ(d.read_idata(0x47), 0x1f); // PORT 3 disables
        CHECK_EQ(d.read_idata(0x48), 0x0b); // PORT 1, PIN 3
        d.write_sfr(PICTL, 0x02);           // P1 pins 0-3: falling edge
        CHECK_EQ(d.peek_xdata(XPictl), 0x02);
        st(d).input_changed(0, 2, true); // not the selected pin
        st(d).input_changed(1, 3, true); // rising edge not selected
    }
    if (at(d, 9000, "st capture 3")) {
        CHECK_EQ(d.peek_xdata(XStcs), 0);
        st(d).input_changed(1, 3, false); // count 9 -> STCV 10
    }
    if (at(d, 9277, "st capture 3 pending"))
        CHECK_EQ(d.peek_xdata(XStcs), 0);
    if (at(d, 9278, "st capture 3 valid")) {
        CHECK_EQ(d.peek_xdata(XStcs), 1);
        CHECK_EQ(xdata24(d, XStcv0), 10u);
    }
    CHECK_EQ(TimersTestAccess::st(st(d)), 0u);
}

// ---------------------------------------------------------------------------
// Watchdog timer
// ---------------------------------------------------------------------------

// SWRU191F 16.1: watchdog mode resets the chip after the interval (64
// periods of the 32 kHz clock = 62500 cycles); 4.10 SLEEPSTA.RST = 10.
void test_wdt_reset() {
    Program p;
    p.mov(WDCTL, 0x0b); // MODE 10 (watchdog), INT 11 (64 periods)
    Cc2530F256 d;
    load(d, p.flash);
    if (at(d, 62499, "wdt before")) {
        CHECK_EQ(d.diagnostics().resets_by_cause[CauseWatchdog], 0u);
        CHECK_EQ(d.read_sfr(WDCTL), 0x0b);
    }
    CHECK_EQ(run_to(d, 0x0000, 70000, "wdt reset"), 62500u);
    CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, 0x10);
    CHECK_EQ(d.diagnostics().resets_by_cause[CauseWatchdog], 1u);
    CHECK_EQ(d.read_sfr(WDCTL), 0x00); // 16.1: disabled after a reset
}

// One clear-sequence scenario: 0xA at x1, optional cancelling CLR value,
// `gap` sled cycles, 0x5 at x2, then SJMP $. Returns X2 - X1.
std::uint64_t wdt_clear_case(unsigned gap, bool cancel, bool expect_clear) {
    Program p;
    p.mov(WDCTL, 0x0b);
    p.at(0x9c40); // reached at about 40000 cycles: before the 62500 timeout
    const std::uint16_t x1 = p.here();
    p.mov(WDCTL, 0xa0);
    if (cancel)
        p.mov(WDCTL, 0x30);
    p.at(p.here() + gap);
    const std::uint16_t x2 = p.here();
    p.mov(WDCTL, 0x50);
    const std::uint16_t l = p.here();
    p.sjmp_self();
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t X1 = run_to(d, x1, 50000, "wdt clear x1");
    const std::uint64_t X2 = run_to(d, x2, 50000, "wdt clear x2");
    const std::uint64_t L = run_to(d, l, 50000, "wdt clear loop");
    const std::uint64_t expected =
        expect_clear ? loop_boundary(L, period_start(lf_ticks(X2) + 64)) : loop_boundary(L, 62500);
    CHECK_EQ(run_to(d, 0x0000, 200000, "wdt clear reset"), expected);
    CHECK_EQ(d.diagnostics().resets_by_cause[CauseWatchdog], 1u);
    return X2 - X1;
}

// SWRU191F 16.1: 0xA then 0x5 in CLR within one watchdog clock period
// clears the counter. Model-defined: "within one period" means at most
// 976 cycles (one 32 kHz period, 976.5625 cycles) between the two writes,
// and any other CLR value written in between cancels the sequence.
void test_wdt_clear() {
    const std::uint64_t m = wdt_clear_case(0, false, true);
    CHECK(m >= 2 && m <= 4);
    CHECK_EQ(wdt_clear_case(static_cast<unsigned>(976 - m), false, true), 976u);
    CHECK_EQ(wdt_clear_case(static_cast<unsigned>(977 - m), false, false), 977u);
    wdt_clear_case(0, true, false);
}

// SWRU191F 16.2: timer mode requests IRCON2.WDTIF each interval.
void test_wdt_timer_mode() {
    Program p;
    p.mov(IEN2, 0x20).setb(BitEa).mov(WDCTL, 0x0f); // MODE 11, INT 11
    const std::uint16_t l = p.here();
    p.sjmp_self();
    p.isr(VWdt, {0xc2, BitWdtif, 0x05, 0x60, 0x32});
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t L = run_to(d, l, 2000, "wdt timer loop");
    CHECK_EQ(run_to(d, VWdt, 70000, "wdt timer 1"), loop_boundary(L, 62500) + 2);
    CHECK(d.read_sfr(IRCON2) & Ircon2Wdtif); // cleared by the ISR, not on entry
    const std::uint64_t L2 = run_to(d, l, 70000, "wdt timer loop 2");
    CHECK_EQ(d.read_idata(0x60), 1);
    CHECK_EQ(d.read_sfr(IRCON2) & Ircon2Wdtif, 0);
    CHECK_EQ(run_to(d, VWdt, 130000, "wdt timer 2"), loop_boundary(L2, 125000) + 2);
    CHECK_EQ(d.diagnostics().resets_by_cause[CauseWatchdog], 0u);
}

// SWRU191F 16.1, 16.3: MODE/INT locks; timer mode CLR[0] and MODE 00;
// WDCTL.INT reduced by the clock division (CLKSPD).
void test_wdt_locks() {
    {
        // Watchdog mode cannot be stopped or its interval changed.
        Program p;
        p.mov(WDCTL, 0x0b).mov(WDCTL, 0x00).mov(WDCTL, 0x08).copy(WDCTL, 0x40);
        const std::uint16_t l = p.here();
        p.sjmp_self();
        Cc2530F256 d;
        load(d, p.flash);
        const std::uint64_t L = run_to(d, l, 2000, "wdt lock a");
        CHECK_EQ(d.read_idata(0x40), 0x0b);
        CHECK_EQ(run_to(d, 0x0000, 200000, "wdt lock a reset"), loop_boundary(L, 62500));
    }
    {
        // Timer mode: MODE 10 and an INT change are ignored (Model-defined
        // for MODE 10: 16.3 requires stopping first); CLR[0] clears.
        Program p;
        p.mov(WDCTL, 0x0f).mov(WDCTL, 0x0b).mov(WDCTL, 0x0c).copy(WDCTL, 0x40);
        p.at(0xfc00); // reached at about 64500 cycles
        p.op({0xc2, BitWdtif});
        const std::uint16_t xc = p.here();
        p.mov(WDCTL, 0x1f); // CLR[0] = 1
        const std::uint16_t l2 = p.here();
        p.sjmp_self();
        Cc2530F256 d;
        load(d, p.flash);
        if (at(d, 62499, "wdt lock b 62499")) {
            CHECK_EQ(d.read_idata(0x40), 0x0f);
            CHECK_EQ(d.read_sfr(IRCON2) & Ircon2Wdtif, 0);
        }
        if (at(d, 62500, "wdt lock b 62500"))
            CHECK(d.read_sfr(IRCON2) & Ircon2Wdtif);
        const std::uint64_t Xc = run_to(d, xc, 70000, "wdt lock b clear");
        const std::uint64_t L2 = run_to(d, l2, 70000, "wdt lock b loop");
        CHECK_EQ(d.read_sfr(IRCON2) & Ircon2Wdtif, 0);
        const std::uint64_t T = period_start(lf_ticks(Xc) + 64);
        CHECK(T != 125000);
        const std::uint64_t B = loop_boundary(L2, T);
        if (at(d, B - 2, "wdt lock b before"))
            CHECK_EQ(d.read_sfr(IRCON2) & Ircon2Wdtif, 0);
        if (at(d, B, "wdt lock b after"))
            CHECK(d.read_sfr(IRCON2) & Ircon2Wdtif);
        d.write_sfr(WDCTL, 0x00); // MODE 00 stops and clears the timer
        d.write_sfr(IRCON2, 0x00);
        run_until(d, now(d) + 200000);
        CHECK_EQ(d.read_sfr(IRCON2) & Ircon2Wdtif, 0);
        CHECK_EQ(d.read_sfr(WDCTL), 0x00);
        CHECK_EQ(d.diagnostics().resets_by_cause[CauseWatchdog], 0u);
    }
    // CLKSPD division: 16 MHz RCOSC at 8 MHz (factor 2: 32 periods) and the
    // documented example 32 MHz XOSC at 4 MHz (factor 8: 8 periods).
    for (const auto &[clk, expected] : {std::pair{std::uint8_t{0xd2}, std::uint64_t{31250}},
                                       std::pair{std::uint8_t{0x9b}, std::uint64_t{7813}}}) {
        Program p;
        p.mov(CLKCONCMD, clk).mov(WDCTL, 0x0b);
        Cc2530F256 d;
        load(d, p.flash);
        CHECK_EQ(run_to(d, 0x0000, 70000, "wdt clkspd reset"), expected);
        CHECK_EQ(d.diagnostics().resets_by_cause[CauseWatchdog], 1u);
    }
}

// SWRU191F 16 (introduction): the counter is "retained during all power
// modes" and "continues counting when entering active mode again"; 4.1.3:
// the WDT is not among the blocks active in PM2. It does not advance in PM2.
void test_wdt_frozen_in_pm2() {
    Program p;
    const std::uint16_t w = p.here();
    p.mov(WDCTL, 0x0b);
    p.mov(ST2, 0).mov(ST1, 0).mov(ST0, 100).mov(IEN0, 0xa0);
    p.enter_power_mode(0x06);
    const std::uint16_t l = p.here();
    p.sjmp_self();
    p.isr(VSt, {0xc2, BitStif, 0x32});
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 2000, "wdt pm2 w");
    if (at(d, 70000, "wdt pm2 70000")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Pm2);
        CHECK_EQ(d.diagnostics().resets_by_cause[CauseWatchdog], 0u);
    }
    CHECK_EQ(run_to(d, VSt, 120000, "wdt pm2 st"), 98148u); // rising edge 100 + 3
    const std::uint64_t d2 = d.diagnostics().power_mode_cycles_by_mode[PmIndex2];
    const std::uint64_t before = lf_ticks(98145 - d2) - lf_ticks(W);
    const std::uint64_t L = run_to(d, l, 120000, "wdt pm2 loop");
    CHECK_EQ(run_to(d, 0x0000, 200000, "wdt pm2 reset"),
             loop_boundary(L, period_start(lf_ticks(98145) + 64 - before)));
    CHECK_EQ(d.diagnostics().resets_by_cause[CauseWatchdog], 1u);
}

// Reserved encodings are stored and reported (BRIEF: note_unmodeled).
void test_unmodeled_reporting() {
    Cc2530F256 d;
    d.write_sfr(WDCTL, 0x04); // MODE 01 reserved
    CHECK_EQ(d.diagnostics().unmodeled_feature_writes.count(0x7000u | WDCTL), 1u);
    CHECK_EQ(d.read_sfr(WDCTL), 0x04);
    d.write_sfr(T1CCTL0, 0x2c); // channel 0 CMP 101 reserved
    CHECK_EQ(d.diagnostics().unmodeled_feature_writes.count(0x7000u | T1CCTL0), 1u);
    CHECK_EQ(d.read_sfr(T1CCTL0), 0x2c);
    // RFIRQ capture (SWRU191F T1CCTLn.RFIRQ) is modelled now, so it is no
    // longer reported (this asserted the earlier "not modelled" rule).
    d.write_sfr(T1CCTL1, 0x80);
    CHECK_EQ(d.diagnostics().unmodeled_feature_writes.count(0x7000u | T1CCTL1), 0u);
    CHECK_EQ(d.read_sfr(T1CCTL1), 0x80);
}

// ---------------------------------------------------------------------------
// Fast / step / trace equivalence and idle_forever
// ---------------------------------------------------------------------------

void test_mode_equivalence() {
    auto no_setup = [](Cc2530F256 &) {};
    {
        // Timer 1 modulo compare interrupt every 100 ticks (200 cycles).
        Program p;
        p.mov(T1CC0L, 99).mov(T1CC0H, 0).mov(T1CCTL0, 0x44).mov(IEN1, 0x02).setb(BitEa);
        p.mov(T1CTL, 0x02).sjmp_self();
        p.isr(VT1, {0x53, T1STAT, 0xfe, 0x05, 0x60, 0x32});
        for_each_mode(
            p.flash, 5000, no_setup,
            [](Cc2530F256 &d, Mode) {
                CHECK(d.read_idata(0x60) >= 20);
                CHECK_EQ(TimersTestAccess::t1(t1(d), 0), d.read_idata(0x60) + 0u);
            },
            "t1 compare modes");
    }
    {
        std::uint16_t w = 0;
        const Program p = t3_overflow_program(w);
        for_each_mode(
            p.flash, 20000, no_setup,
            [](Cc2530F256 &d, Mode) { CHECK(d.read_idata(0x60) >= 30); }, "t3 overflow modes");
    }
    {
        std::uint16_t lb = 0, ls = 0;
        const Program p = st_load_program(lb, ls);
        for_each_mode(
            p.flash, 290000, no_setup,
            [](Cc2530F256 &d, Mode) {
                CHECK_EQ(d.read_idata(0x60), 1);
                CHECK_EQ(d.read_idata(0x50), 0x23);
                CHECK_EQ(d.read_idata(0x51), 0x01);
            },
            "st compare modes");
    }
    {
        Program p;
        p.mov(WDCTL, 0x0b);
        for_each_mode(
            p.flash, 70000, no_setup,
            [](Cc2530F256 &d, Mode) {
                CHECK_EQ(d.diagnostics().resets_by_cause[CauseWatchdog], 1u);
                CHECK_EQ(d.read_sfr(WDCTL), 0x0b); // set again after the reset
            },
            "wdt reset modes");
    }
    {
        // Sleep timer wake from PM2 (no fast-mode stop: run() returns on the
        // power mode entry, so all modes are compared at the same limit).
        Program p;
        p.mov(ST2, 0).mov(ST1, 0).mov(ST0, 10).mov(IEN0, 0xa0);
        p.enter_power_mode(0x06);
        p.sjmp_self();
        p.isr(VSt, {0xc2, BitStif, 0x05, 0x60, 0x32});
        for_each_mode(
            p.flash, 12000,
            [](Cc2530F256 &d) {
                // Reach the PM2 entry first so that each mode resumes from it.
                while (d.power_mode() != Cc2530PowerMode::Pm2 && now(d) < 2000)
                    d.run(1);
            },
            [](Cc2530F256 &d, Mode) { CHECK_EQ(d.read_idata(0x60), 1); }, "st pm2 modes");
    }
}

// The core's idle_forever() relies on may_interrupt(): a timer that can
// raise an enabled interrupt (or reset) must keep the CPU from idling.
void test_idle_forever() {
    auto idle = [](std::initializer_list<std::vector<std::uint8_t>> code) {
        Program p;
        for (const auto &c : code)
            p.op(c);
        const std::uint16_t l = p.here();
        p.sjmp_self();
        Cc2530F256 d;
        load(d, p.flash);
        run_to(d, l, 2000, "idle_forever");
        return d.idle_forever();
    };
    const std::vector<std::uint8_t> ea{0xd2, BitEa};
    CHECK(idle({}));
    CHECK(idle({mov_dir_imm(T1CTL, 0x01)}));                  // EA = 0
    CHECK(idle({mov_dir_imm(WDCTL, 0x0f)}));                  // WDTIE = 0
    CHECK(idle({mov_dir_imm(T3CTL, 0x18), mov_dir_imm(IEN1, 0x08)})); // EA = 0
    CHECK(!idle({mov_dir_imm(T1CCTL0, 0x44), mov_dir_imm(IEN1, 0x02), ea,
                 mov_dir_imm(T1CTL, 0x01)}));
    CHECK(!idle({mov_dir_imm(WDCTL, 0x08)}));     // watchdog reset pending
    CHECK(!idle({mov_dir_imm(IEN0, 0xa0)}));      // ST compare 0xFFFFFF
    CHECK(!idle({mov_dir_imm(T3CTL, 0x18), mov_dir_imm(IEN1, 0x08), ea}));
    CHECK(!idle({mov_dir_imm(T4CTL, 0x18), mov_dir_imm(IEN1, 0x10), ea}));
    CHECK(!idle({mov_dir_imm(IEN2, 0x20), ea, mov_dir_imm(WDCTL, 0x0f)}));
}

} // namespace

int main() {
    test_reset_values_and_aliases();
    test_t1_modulo_compare_isr();
    test_t1_free_running_latch();
    test_t1_up_down();
    test_t1_output_modes();
    test_t1_compare_buffering();
    test_t1_capture();
    test_t1_stat_semantics();
    test_t1_dma_triggers();
    test_tickspd();
    test_t1_ir_generation();
    test_t34_overflow_isr();
    test_t34_modes();
    test_timif_semantics();
    test_t34_capture();
    test_st_wake_from_pm2();
    test_st_pm3();
    test_st_load_and_latch();
    test_st_capture();
    test_wdt_reset();
    test_wdt_clear();
    test_wdt_timer_mode();
    test_wdt_locks();
    test_wdt_frozen_in_pm2();
    test_unmodeled_reporting();
    test_mode_equivalence();
    test_idle_forever();
    return finish("cc2530 timers");
}
