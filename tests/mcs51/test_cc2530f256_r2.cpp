// EMU-04A-R2 Gates A, B and C: interrupt delivery for asynchronous
// (virtual-time) events, independent ENCIF_x / RFIF_x flag bits, and PCON.IDLE
// wake ordering. Every expected value is computed here from SWRU191F
// (Tables 2-5, 2-7, 2-8, sections 4.1.1 and 4.2) and the documented MCUEMU
// cycle contract carried by each test instruction (checked by T0); nothing is read back from the model
// to build an expectation. Every scenario runs in fast, trace and
// single-step mode.
#include "cc2530_r2_support.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>

using namespace r2;
using mcs51emu::Cc2530Diagnostics;
using mcs51emu::Cc2530Registers;

namespace {

// SWRU191F Table 2-5 (flag and enable bits), Table 2-7 (priority group) and
// Table 2-5 footnote (1) (flag cleared by hardware when the ISR is called).
struct Spec {
    const char *name;
    std::uint8_t flag_sfr, flag_mask, enable_sfr, enable_mask, group;
    bool hardware_clear;
};
constexpr std::array<Spec, 18> Sources{{
    {"RFERR", TCON, 0x02, IEN0, 0x01, 0, true},
    {"ADC", TCON, 0x20, IEN0, 0x02, 1, true},
    {"URX0", TCON, 0x08, IEN0, 0x04, 2, true},
    {"URX1", TCON, 0x80, IEN0, 0x08, 3, true},
    {"ENC", S0CON, 0x03, IEN0, 0x10, 4, false},
    {"ST", IRCON, 0x80, IEN0, 0x20, 5, false},
    {"P2INT", IRCON2, 0x01, IEN2, 0x02, 1, false},
    {"UTX0", IRCON2, 0x02, IEN2, 0x04, 2, false},
    {"DMA", IRCON, 0x01, IEN1, 0x01, 0, false},
    {"T1", IRCON, 0x02, IEN1, 0x02, 1, true},
    {"T2", IRCON, 0x04, IEN1, 0x04, 2, true},
    {"T3", IRCON, 0x08, IEN1, 0x08, 3, true},
    {"T4", IRCON, 0x10, IEN1, 0x10, 4, true},
    {"P0INT", IRCON, 0x20, IEN1, 0x20, 5, false},
    {"UTX1", IRCON2, 0x04, IEN2, 0x08, 3, false},
    {"P1INT", IRCON2, 0x08, IEN2, 0x10, 4, false},
    {"RF", S1CON, 0x03, IEN2, 0x01, 0, false},
    {"WDT", IRCON2, 0x10, IEN2, 0x20, 5, false},
}};

// SWRU191F Table 2-6: level = IP1.x:IP0.x of the source's group.
unsigned level_of(unsigned source, std::uint8_t ip0, std::uint8_t ip1) {
    const unsigned g = Sources[source].group;
    return (((ip1 >> g) & 1u) << 1) | ((ip0 >> g) & 1u);
}

// SWRU191F Table 2-8, within one priority level.
constexpr std::array<unsigned, 18> PollSequence{0,  16, 8, 1,  9, 2,  10, 3,  11,
                                                4,  12, 5, 13, 6, 7, 14, 15, 17};
unsigned poll_position(unsigned source) {
    for (unsigned i = 0; i < PollSequence.size(); ++i)
        if (PollSequence[i] == source)
            return i;
    return 99;
}

constexpr std::uint16_t MainOrigin = 0x0800, HandlerOrigin = 0x0100, HandlerStride = 0x60;
constexpr std::uint32_t Bank3Physical = 3 * 0x8000;
constexpr std::uint16_t BankLogical = 0x8000;
// IDATA cells (bank 0 registers are 0x00-0x07; the stack starts at 0x20).
constexpr std::uint8_t Scratch = 0x30, ClearEnc = 0x38, ClearRf = 0x39, PostIdle = 0x3c,
                       AfterIdle = 0x3d, StackStart = 0x1f;
constexpr std::uint64_t Forever = std::numeric_limits<std::uint64_t>::max();

// Every handler starts with a 14-cycle, 13-instruction log entry:
// [0x80 | source, return PC high, return PC low, logged register].
constexpr std::size_t LogInstructions = 2 + 8 + 3;

struct HandlerSpec {
    std::uint8_t logged = 0; // direct address logged; 0 = the source's flag SFR
    std::vector<Ins> extra;  // after the log entry, before the flag clear and RETI
};

struct BenchSpec {
    std::vector<Ins> setup; // after the fixed prologue
    std::vector<Ins> body;  // followed by SJMP $
    std::map<unsigned, HandlerSpec> handlers;
    std::vector<Ins> bank3; // logical 0x8000 in bank 3
};

struct Bench {
    Flash flash;
    Placed main;
    std::size_t body_index{}, loop_index{};
    std::array<Placed, 18> handler;
    std::array<unsigned, 18> isr_cycles{};       // entry + vector LJMP + handler
    std::array<unsigned, 18> isr_instructions{}; // entry counts as one (R1-F4)
    Placed bank3;

    std::uint64_t main_cycle(std::size_t i) const {
        if (i < loop_index)
            return 2 + main.offset[i];
        return 2 + main.offset[loop_index] + 2 * (i - loop_index);
    }
    std::uint16_t main_pc(std::size_t i) const { return main.pc[std::min(i, loop_index)]; }
    std::size_t first_boundary(std::size_t from, std::uint64_t t) const {
        std::size_t i = from;
        while (main_cycle(i) < t)
            ++i;
        return i;
    }
    // Start cycle of handler instruction j of an ISR entered at `entry`.
    std::uint64_t handler_cycle(unsigned source, std::uint64_t entry, std::size_t j) const {
        return entry + 4 + handler[source].offset[j];
    }
};

std::vector<Ins> handler_code(unsigned source, const HandlerSpec &spec) {
    const Spec &s = Sources[source];
    std::vector<Ins> code = log_marker(static_cast<std::uint8_t>(0x80 | source));
    for (const auto &part : {log_return(), log_direct(spec.logged ? spec.logged : s.flag_sfr),
                             spec.extra})
        code.insert(code.end(), part.begin(), part.end());
    if (source == SrcEnc || source == SrcRf) {
        // Clear with the AND mask kept in IDATA, then clear everything on
        // later entries: tests choose which flag bit the first entry leaves.
        const std::uint8_t cell = source == SrcEnc ? ClearEnc : ClearRf;
        code.push_back(mov_a_dir(cell));
        code.push_back(anl_dir_a(s.flag_sfr));
        code.push_back(mov_imm(cell, 0xfc));
    } else if (!s.hardware_clear) {
        code.push_back(anl_imm(s.flag_sfr, static_cast<std::uint8_t>(~s.flag_mask)));
    }
    code.push_back(reti());
    return code;
}

Bench build(const BenchSpec &spec) {
    Bench b;
    place(b.flash, 0, {ljmp(MainOrigin)});
    for (unsigned k = 0; k < 18; ++k) {
        const auto address = static_cast<std::uint16_t>(HandlerOrigin + k * HandlerStride);
        place(b.flash, vector_of(k), {ljmp(address)});
        const auto it = spec.handlers.find(k);
        b.handler[k] = place(b.flash, address,
                             handler_code(k, it == spec.handlers.end() ? HandlerSpec{} : it->second));
        if (b.handler[k].end > address + HandlerStride)
            throw std::logic_error("handler too long");
        b.isr_cycles[k] = 2 + 2 + b.handler[k].total;
        b.isr_instructions[k] = 2 + static_cast<unsigned>(b.handler[k].pc.size());
    }
    std::vector<Ins> code{mov_imm(SP, StackStart), mov_rn_imm(1, 0x40), mov_imm(ClearEnc, 0xfc),
                          mov_imm(ClearRf, 0xfc)};
    code.insert(code.end(), spec.setup.begin(), spec.setup.end());
    for (const Ins &ins : code)
        if (ins.bytes.empty())
            throw std::logic_error("idle_pad is only allowed in the body");
    b.body_index = code.size();
    code.insert(code.end(), spec.body.begin(), spec.body.end());
    code.push_back(sjmp_self());
    b.main = place(b.flash, MainOrigin, code);
    // idle_pad expands to 0-3 NOPs: index the placed instructions.
    b.loop_index = b.main.pc.size() - 1;
    if (!spec.bank3.empty())
        b.bank3 = place(b.flash, Bank3Physical, spec.bank3, BankLogical);
    return b;
}

struct Entry {
    unsigned source;
    std::uint16_t ret;
    std::uint8_t value;
    bool operator==(const Entry &) const = default;
};

std::vector<Entry> decode(const std::vector<std::uint8_t> &log) {
    std::vector<Entry> entries;
    std::size_t i = 0;
    for (; i + 4 <= log.size(); i += 4)
        entries.push_back({(log[i] & 0x80u) ? log[i] & 0x7fu : 0xffu,
                           static_cast<std::uint16_t>((log[i + 1] << 8) | log[i + 2]), log[i + 3]});
    if (i != log.size())
        entries.push_back({0xfe, 0, 0});
    return entries;
}

std::string show(const std::vector<Entry> &entries) {
    std::ostringstream out;
    out << '[';
    for (const Entry &e : entries)
        out << ' ' << (e.source < 18 ? Sources[e.source].name : "?") << "@" << std::hex
            << e.ret << ":" << unsigned(e.value) << std::dec;
    out << " ]";
    return out.str();
}

struct Outcome {
    Cc2530RunResult result;
    Cc2530Registers regs;
    Cc2530Diagnostics diag;
    std::vector<Entry> log;
    std::array<std::uint8_t, 256> idata{};
    std::array<std::uint8_t, 256> sfr{};
    bool sleeping{}, in_interrupt{};
    std::size_t queued{};
    std::string error;
};

using Action = std::function<void(Cc2530F256 &)>;
struct Phase {
    std::uint64_t limit = Forever;
    Action before;
};

Outcome capture(Cc2530F256 &device) {
    Outcome out;
    out.regs = device.registers();
    out.diag = device.diagnostics();
    out.log = decode(irq_log(device));
    for (unsigned a = 0; a < 256; ++a) {
        out.idata[a] = device.read_idata(static_cast<std::uint8_t>(a));
        out.sfr[a] = a >= 0x80 ? device.read_sfr(static_cast<std::uint8_t>(a)) : 0;
    }
    out.sleeping = device.sleeping();
    out.in_interrupt = device.in_interrupt();
    out.queued = device.scheduled_event_count();
    return out;
}

Outcome execute(const Bench &bench, Mode mode, const std::vector<Phase> &phases) {
    auto device = std::make_unique<Cc2530F256>();
    load(*device, bench.flash);
    std::ostringstream sink;
    Cc2530RunResult last;
    std::string error;
    try {
        for (const Phase &phase : phases) {
            if (phase.before)
                phase.before(*device);
            last = run_mode(*device, mode, phase.limit, sink);
            if (!last.error.empty())
                error += last.error + "; ";
        }
    } catch (const std::exception &e) {
        error += std::string("exception: ") + e.what();
    }
    Outcome out = capture(*device);
    out.result = last;
    out.error = error;
    return out;
}

Outcome execute(const Bench &bench, Mode mode, std::uint64_t limit, Action before = {}) {
    return execute(bench, mode, std::vector<Phase>{{limit, std::move(before)}});
}

std::string context(const char *test, Mode mode, const std::string &detail) {
    return std::string(test) + " " + mode_name(mode) + " " + detail;
}

void expect_log(const Outcome &out, const std::vector<Entry> &expected, const std::string &ctx) {
    CHECK_IN(out.error.empty(), ctx + " error=" + out.error);
    CHECK_IN(out.log == expected, ctx + " log=" + show(out.log) + " want=" + show(expected));
    CHECK_IN(out.diag.stale_interrupt_posts == 0, ctx);
    CHECK_IN(out.diag.interrupts_taken == expected.size(), ctx);
}

// The run ended in the SJMP $ loop at the first boundary >= `limit` after
// the main timeline was delayed by `delay` cycles (ISRs, idle time, banked
// calls) and `extra_instructions` non-main instructions executed.
void expect_final(const Outcome &out, const Bench &b, std::uint64_t delay,
                  std::uint64_t extra_instructions, std::uint64_t limit, const std::string &ctx) {
    std::size_t fm = b.loop_index;
    while (b.main_cycle(fm) + delay < limit)
        ++fm;
    CHECK_IN(out.regs.cycles == b.main_cycle(fm) + delay,
             ctx + " cycles=" + std::to_string(out.regs.cycles) +
                 " want=" + std::to_string(b.main_cycle(fm) + delay));
    CHECK_IN(out.regs.instructions == 1 + fm + extra_instructions,
             ctx + " instructions=" + std::to_string(out.regs.instructions) +
                 " want=" + std::to_string(1 + fm + extra_instructions));
    CHECK_IN(out.regs.pc == b.main_pc(b.loop_index), ctx);
    CHECK_IN(out.regs.sp == StackStart, ctx);
    CHECK_IN(out.regs.interrupt_levels == 0, ctx);
    CHECK_IN(!out.sleeping, ctx);
    CHECK_IN(out.queued == 0, ctx);
}

template <typename E>
bool throws(const std::function<void()> &f) {
    try {
        f();
    } catch (const E &) {
        return true;
    } catch (...) {
        return false;
    }
    return false;
}

// ---------------------------------------------------------------- Gate A

// A1: an event that arrives while the CPU executes 1-, 2- and 4-cycle
// instructions, none of which writes an SFR, is taken at the first
// instruction boundary at which cycles >= its time, in every mode.
void test_async_event_without_sfr_writes() {
    BenchSpec spec;
    spec.setup = {mov_imm(IEN1, 0x02), setb(EA_BIT)};
    spec.body = {nop(),         mov_imm(Scratch, 1), mul_ab(),
                 inc_dir(Scratch + 1), mov_dir_dir(Scratch + 2, Scratch + 3), nop(),
                 mul_ab(),      nop(),               mov_imm(Scratch, 2),
                 inc_dir(Scratch + 1), mul_ab(),     mul_ab(),
                 nop()};
    const Bench b = build(spec);
    const std::uint64_t first = b.main_cycle(b.body_index);
    const std::uint64_t loop = b.main_cycle(b.loop_index);
    const std::uint64_t limit = loop + 80;
    for (std::uint64_t t = first; t <= loop + 5; ++t) {
        const std::size_t e = b.first_boundary(b.body_index, t);
        for (const Mode mode : AllModes) {
            const std::string ctx = context("A1", mode, "t=" + std::to_string(t));
            const Outcome out = execute(b, mode, limit, [t](Cc2530F256 &d) {
                d.schedule_interrupt_flag_for_test(t, SrcT1);
            });
            // T1IF is cleared by hardware at the vector (Table 2-5 note 1).
            expect_log(out, {{SrcT1, b.main_pc(e), 0x00}}, ctx);
            expect_final(out, b, b.isr_cycles[SrcT1], b.isr_instructions[SrcT1], limit, ctx);
            CHECK_IN(out.diag.scheduled_flag_events == 1, ctx);
            CHECK_IN(out.diag.test_injected_flags == 0, ctx);
        }
    }
}

// A2/A3: a request of higher priority arriving inside an ISR preempts it at
// the first ISR instruction boundary at or after its time, including the
// RETI boundary; one arriving during RETI, or of equal/lower priority, is
// taken only after RETI and one instruction of the interrupted code.
void test_event_inside_isr() {
    struct Config {
        std::uint8_t ip0, ip1;
    };
    const Config configs[] = {{0x08, 0x00}, {0x00, 0x00}, {0x02, 0x00}, {0x00, 0x0a},
                              {0x08, 0x0a}, {0x0a, 0x0a}, {0x02, 0x08}};
    for (const Config cfg : configs) {
        BenchSpec spec;
        spec.setup = {mov_imm(IP0, cfg.ip0), mov_imm(IP1, cfg.ip1), mov_imm(IEN1, 0x0a),
                      setb(EA_BIT)};
        spec.body = cat({repeat(nop(), 6), {mul_ab(), nop(), mul_ab(), nop()}});
        spec.handlers[SrcT1].extra = {nop(),    mul_ab(), nop(), mul_ab(),
                                      mul_ab(), nop(),    nop(), mul_ab()};
        const Bench b = build(spec);
        const Placed &h = b.handler[SrcT1];
        const std::size_t e1 = b.body_index + 2;
        const std::uint64_t entry1 = b.main_cycle(e1);
        const std::size_t reti_index = h.pc.size() - 1;
        const std::uint64_t reti_start = b.handler_cycle(SrcT1, entry1, reti_index);
        const std::uint64_t reti_end = entry1 + b.isr_cycles[SrcT1];
        const bool preempts = level_of(SrcT3, cfg.ip0, cfg.ip1) > level_of(SrcT1, cfg.ip0, cfg.ip1);
        const std::uint64_t limit = b.main_cycle(b.loop_index) + 120;
        for (std::uint64_t t2 = b.handler_cycle(SrcT1, entry1, LogInstructions); t2 <= reti_end;
             ++t2) {
            std::uint16_t ret = b.main_pc(e1 + 1);
            if (preempts && t2 <= reti_start) {
                std::size_t j = LogInstructions;
                while (b.handler_cycle(SrcT1, entry1, j) < t2)
                    ++j;
                ret = h.pc[j];
            }
            for (const Mode mode : AllModes) {
                const std::string ctx =
                    context("A2", mode,
                            "ip0=" + std::to_string(cfg.ip0) + " ip1=" + std::to_string(cfg.ip1) +
                                " t2=" + std::to_string(t2));
                const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &d) {
                    d.schedule_interrupt_flag_for_test(entry1, SrcT1);
                    d.schedule_interrupt_flag_for_test(t2, SrcT3);
                });
                expect_log(out, {{SrcT1, b.main_pc(e1), 0x00}, {SrcT3, ret, 0x00}}, ctx);
                expect_final(out, b, b.isr_cycles[SrcT1] + b.isr_cycles[SrcT3],
                             b.isr_instructions[SrcT1] + b.isr_instructions[SrcT3], limit, ctx);
                CHECK_IN(out.diag.scheduled_flag_events == 2, ctx);
            }
        }
    }
}

// A4: an event interrupting bank-3 code returns to the banked logical PC with
// FMAP intact.
void test_event_in_banked_code() {
    BenchSpec spec;
    spec.setup = {mov_imm(IEN1, 0x02), setb(EA_BIT), mov_imm(FMAP, 3), lcall(BankLogical),
                  mov_dir_dir(Scratch + 10, FMAP)};
    spec.bank3 = {nop(), mul_ab(), nop(), mov_imm(Scratch + 11, 0xb3), mul_ab(), nop(), ret()};
    spec.handlers[SrcT1].logged = FMAP;
    const Bench b = build(spec);
    const std::size_t call = b.main.find(lcall(BankLogical).bytes);
    const std::uint64_t bank_start = b.main_cycle(call) + 2;
    const std::uint64_t ret_end = bank_start + b.bank3.total;
    const std::uint64_t limit = b.main_cycle(b.loop_index) + b.bank3.total + 80;
    for (std::uint64_t t = bank_start; t <= ret_end; ++t) {
        std::uint16_t ret = b.main_pc(call + 1);
        for (std::size_t j = 0; j < b.bank3.pc.size(); ++j)
            if (bank_start + b.bank3.offset[j] >= t) {
                ret = b.bank3.pc[j];
                break;
            }
        for (const Mode mode : AllModes) {
            const std::string ctx = context("A4", mode, "t=" + std::to_string(t));
            const Outcome out = execute(b, mode, limit, [t](Cc2530F256 &d) {
                d.schedule_interrupt_flag_for_test(t, SrcT1);
            });
            expect_log(out, {{SrcT1, ret, 3}}, ctx);
            expect_final(out, b, b.isr_cycles[SrcT1] + b.bank3.total,
                         b.isr_instructions[SrcT1] + b.bank3.pc.size(), limit, ctx);
            CHECK_IN(out.idata[Scratch + 10] == 3, ctx);
            CHECK_IN(out.idata[Scratch + 11] == 0xb3, ctx);
            CHECK_IN(out.regs.fmap == 3, ctx);
        }
    }
}

// A5: events due at the same boundary are arbitrated like simultaneous
// flags: priority level first, then Table 2-8 polling order; scheduling
// order does not matter.
void test_same_time_events() {
    for (const std::uint8_t ip0 : {std::uint8_t{0x00}, std::uint8_t{0x08}}) {
        BenchSpec spec;
        spec.setup = {mov_imm(IP0, ip0), mov_imm(IEN1, 0x0a), setb(EA_BIT)};
        spec.body = repeat(nop(), 8);
        const Bench b = build(spec);
        const std::size_t e = b.body_index + 2;
        const std::uint64_t t = b.main_cycle(e);
        const bool t3_first = level_of(SrcT3, ip0, 0) > level_of(SrcT1, ip0, 0) ||
                              (level_of(SrcT3, ip0, 0) == level_of(SrcT1, ip0, 0) &&
                               poll_position(SrcT3) < poll_position(SrcT1));
        const unsigned first = t3_first ? SrcT3 : SrcT1, second = t3_first ? SrcT1 : SrcT3;
        const std::uint64_t limit = b.main_cycle(b.loop_index) + 100;
        for (const Mode mode : AllModes) {
            const std::string ctx = context("A5", mode, "ip0=" + std::to_string(ip0));
            const Outcome out = execute(b, mode, limit, [t](Cc2530F256 &d) {
                d.schedule_interrupt_flag_for_test(t + 40, SrcT4); // disabled: no entry
                d.schedule_interrupt_flag_for_test(t, SrcT3);
                d.schedule_interrupt_flag_for_test(t, SrcT1);
                d.schedule_interrupt_flag_for_test(t, SrcT1);
                CHECK(d.next_event_cycle() == t);
                CHECK(d.scheduled_event_count() == 4);
            });
            // The first ISR still sees the other source's flag pending.
            expect_log(out,
                       {{first, b.main_pc(e), Sources[second].flag_mask},
                        {second, b.main_pc(e + 1), 0x00}},
                       ctx);
            expect_final(out, b, b.isr_cycles[SrcT1] + b.isr_cycles[SrcT3],
                         b.isr_instructions[SrcT1] + b.isr_instructions[SrcT3], limit, ctx);
            CHECK_IN(out.diag.scheduled_flag_events == 4, ctx);
            CHECK_IN(out.sfr[IRCON] == Sources[SrcT4].flag_mask, ctx);
        }
    }
}

// A6: event API contract.
void test_event_api_contract() {
    BenchSpec spec;
    spec.setup = {mov_imm(IEN1, 0x02), setb(EA_BIT)};
    spec.body = repeat(nop(), 20);
    const Bench b = build(spec);
    Cc2530F256 d;
    load(d, b.flash);
    CHECK(throws<std::out_of_range>([&] { d.schedule_interrupt_flag_for_test(10, 18); }));
    CHECK(throws<std::invalid_argument>([&] { d.schedule_interrupt_flag_for_test(10, SrcEnc, 0); }));
    CHECK(throws<std::invalid_argument>(
        [&] { d.schedule_interrupt_flag_for_test(10, SrcEnc, 0x04); }));
    CHECK(throws<std::invalid_argument>(
        [&] { d.schedule_interrupt_flag_for_test(10, SrcT1, 0x01); }));
    CHECK(throws<std::invalid_argument>([&] { d.raise_interrupt_flag_for_test(SrcRf, 0x04); }));
    CHECK(throws<std::out_of_range>([&] { d.raise_interrupt_flag_for_test(18, 0x01); }));
    CHECK(d.scheduled_event_count() == 0);
    CHECK(!d.next_event_cycle());
    d.schedule_interrupt_flag_for_test(50, SrcRf, 0x02);
    d.schedule_interrupt_flag_for_test(20, SrcEnc, 0x01);
    CHECK(d.scheduled_event_count() == 2);
    CHECK(d.next_event_cycle() == 20u);

    // A reset discards scheduled events.
    d.reset();
    CHECK(d.scheduled_event_count() == 0);
    d.run(1000, 200);
    CHECK(d.read_sfr(S0CON) == 0 && d.read_sfr(S1CON) == 0);
    CHECK(d.diagnostics().scheduled_flag_events == 0);

    // An event scheduled in the past applies at the next boundary, before
    // the next instruction: run(1) performs the interrupt entry.
    load(d, b.flash);
    const std::uint64_t x = b.main_cycle(b.body_index + 5);
    d.run(1000, x);
    CHECK(d.registers().cycles == x);
    d.schedule_interrupt_flag_for_test(x - 3, SrcT1);
    const Cc2530RunResult r = d.run(1);
    CHECK(r.instructions == 1);
    CHECK(r.pc == vector_of(SrcT1));
    CHECK(d.registers().cycles == x + 2);
    CHECK(d.registers().interrupt_levels == 0x01);
    d.run(1000, x + 60);
    CHECK(decode(irq_log(d)) ==
          (std::vector<Entry>{{SrcT1, b.main_pc(b.body_index + 5), 0x00}}));

    // A disabled source's event sets its flag and nothing else; queued events
    // keep idle_forever() false.
    load(d, b.flash);
    d.run(10000, b.main_cycle(b.loop_index) + 10);
    CHECK(d.idle_forever());
    d.schedule_interrupt_flag_for_test(d.registers().cycles + 100, SrcT4);
    CHECK(!d.idle_forever());
    d.run(10000, d.registers().cycles + 200);
    CHECK(d.read_sfr(IRCON) == 0x10);
    CHECK(d.registers().posted_vector == 0);
    CHECK(d.diagnostics().interrupts_taken == 0);
    CHECK(d.idle_forever());
    CHECK(d.diagnostics().stale_interrupt_posts == 0);
}

// A7 (R1 R22, R17): a firmware write that makes an already-pending flag
// eligible (enable bit, EA, or a priority raise above the ISR in progress)
// is followed by the entry at the very next boundary, in a single fast run.
void test_enable_and_priority_writes() {
    struct Case {
        const char *name;
        std::vector<Ins> setup, body;
        std::size_t enabling; // body index of the enabling write
        unsigned source;
        std::uint8_t value;
    };
    const Case cases[] = {
        {"IEN1", {setb(EA_BIT)},
         {orl_imm(IRCON, 0x02), nop(), mul_ab(), mov_imm(IEN1, 0x02), nop(), nop()}, 3, SrcT1, 0},
        {"IEN2", {setb(EA_BIT)},
         {orl_imm(S1CON, 0x02), nop(), mul_ab(), mov_imm(IEN2, 0x01), nop(), nop()}, 3, SrcRf, 2},
        {"IEN0", {setb(EA_BIT)},
         {orl_imm(S0CON, 0x01), nop(), mul_ab(), orl_imm(IEN0, 0x10), nop(), nop()}, 3, SrcEnc, 1},
        {"EA", {},
         {mov_imm(IEN1, 0x02), orl_imm(IRCON, 0x02), nop(), mul_ab(), setb(EA_BIT), nop(), nop()},
         4, SrcT1, 0},
    };
    for (const Case &c : cases) {
        BenchSpec spec;
        spec.setup = c.setup;
        spec.body = c.body;
        const Bench b = build(spec);
        const std::uint64_t limit = b.main_cycle(b.loop_index) + 80;
        for (const Mode mode : AllModes) {
            const std::string ctx = context("A7", mode, c.name);
            const Outcome out = execute(b, mode, limit);
            expect_log(out, {{c.source, b.main_pc(b.body_index + c.enabling + 1), c.value}}, ctx);
            expect_final(out, b, b.isr_cycles[c.source], b.isr_instructions[c.source], limit, ctx);
        }
    }

    // R17: T1 and T3 pending at level 0, T1 taken first (Table 2-8); the T1
    // ISR raises IPG3 to level 1, so T3 preempts right after that write.
    BenchSpec spec;
    spec.setup = {mov_imm(IEN1, 0x0a), orl_imm(IRCON, 0x0a), setb(EA_BIT)};
    spec.body = repeat(nop(), 4);
    spec.handlers[SrcT1].extra = {nop(), mov_imm(IP0, 0x08), nop(), mul_ab(), nop()};
    const Bench b = build(spec);
    const std::uint64_t limit = b.main_cycle(b.loop_index) + 100;
    for (const Mode mode : AllModes) {
        const std::string ctx = context("A7", mode, "IP0 raise in ISR");
        const Outcome out = execute(b, mode, limit);
        expect_log(out,
                   {{SrcT1, b.main_pc(b.body_index), 0x08},
                    {SrcT3, b.handler[SrcT1].pc[LogInstructions + 2], 0x00}},
                   ctx);
        expect_final(out, b, b.isr_cycles[SrcT1] + b.isr_cycles[SrcT3],
                     b.isr_instructions[SrcT1] + b.isr_instructions[SrcT3], limit, ctx);
    }
}

// ---------------------------------------------------------------- Gate B

constexpr unsigned FlagSources[] = {SrcEnc, SrcRf};

std::uint8_t clear_cell(unsigned source) { return source == SrcEnc ? ClearEnc : ClearRf; }
std::uint8_t other_flag_sfr(unsigned source) { return source == SrcEnc ? S1CON : S0CON; }

enum class Raise { Firmware, Host, Scheduled };
const char *raise_name(Raise r) {
    return r == Raise::Firmware ? "firmware" : r == Raise::Host ? "host" : "scheduled";
}

// B1: each of the two flag bits requests the interrupt on its own
// (SWRU191F S0CON/S1CON: "Setting one of these flags requests interrupt
// service"), the flags are not cleared by hardware (no footnote (1)), and a
// bit left set after RETI requests the interrupt again.
void test_flag_bits_independently() {
    for (const unsigned src : FlagSources) {
        const Spec &s = Sources[src];
        for (const std::uint8_t m : {std::uint8_t{1}, std::uint8_t{2}, std::uint8_t{3}}) {
            for (const std::uint8_t clear_first : {std::uint8_t{0xfe}, std::uint8_t{0xfd}}) {
                for (const Raise raise : {Raise::Firmware, Raise::Host, Raise::Scheduled}) {
                    BenchSpec spec;
                    spec.setup = {mov_imm(clear_cell(src), clear_first),
                                  mov_imm(s.enable_sfr, s.enable_mask), setb(EA_BIT)};
                    spec.body = cat({repeat(nop(), 2),
                                     {raise == Raise::Firmware ? orl_imm(s.flag_sfr, m) : nop()},
                                     {nop(), mul_ab()},
                                     repeat(nop(), 3)});
                    const Bench b = build(spec);
                    std::size_t e = b.body_index; // host: pending since reset, taken after EA
                    if (raise == Raise::Firmware)
                        e = b.body_index + 3;
                    else if (raise == Raise::Scheduled)
                        e = b.body_index + 5; // event in the middle of MUL AB
                    const std::uint64_t t = b.main_cycle(b.body_index + 4) + 2;
                    std::vector<Entry> want{{src, b.main_pc(e), m}};
                    const std::uint8_t rest = m & clear_first;
                    if (rest)
                        want.push_back({src, b.main_pc(e + 1), rest});
                    const std::uint64_t limit = b.main_cycle(b.loop_index) + 120;
                    for (const Mode mode : AllModes) {
                        const std::string ctx =
                            context("B1", mode,
                                    std::string(s.name) + " m=" + std::to_string(m) +
                                        " clear=" + std::to_string(clear_first) + " " +
                                        raise_name(raise));
                        const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &d) {
                            if (raise == Raise::Host)
                                d.raise_interrupt_flag_for_test(src, m);
                            else if (raise == Raise::Scheduled)
                                d.schedule_interrupt_flag_for_test(t, src, m);
                        });
                        expect_log(out, want, ctx);
                        expect_final(out, b, want.size() * b.isr_cycles[src],
                                     want.size() * b.isr_instructions[src], limit, ctx);
                        CHECK_IN(out.sfr[s.flag_sfr] == 0, ctx);
                        CHECK_IN(out.sfr[other_flag_sfr(src)] == 0, ctx);
                        CHECK_IN(out.diag.test_injected_flags == (raise == Raise::Host ? 1u : 0u),
                                 ctx);
                        CHECK_IN(out.diag.scheduled_flag_events ==
                                     (raise == Raise::Scheduled ? 1u : 0u),
                                 ctx);
                    }
                }
            }
        }
    }
}

// B2: masking. Only the source's own enable bit (plus EA) makes a pending
// flag bit eligible; every other enable bit set leaves it pending. A flag
// cleared while disabled leaves nothing to dispatch.
void test_flag_masking() {
    for (const unsigned src : FlagSources) {
        const Spec &s = Sources[src];
        // EA plus every implemented enable bit except this source's.
        const std::uint8_t ien0 = static_cast<std::uint8_t>(
            0xbf & ~(s.enable_sfr == IEN0 ? s.enable_mask : 0));
        const std::uint8_t ien2 = static_cast<std::uint8_t>(
            0x3f & ~(s.enable_sfr == IEN2 ? s.enable_mask : 0));
        for (const std::uint8_t m : {std::uint8_t{1}, std::uint8_t{2}}) {
            {
                BenchSpec spec;
                spec.body = {orl_imm(s.flag_sfr, m), nop(),   mov_imm(IEN1, 0x3f),
                             mov_imm(IEN2, ien2),    mul_ab(), mov_imm(IEN0, ien0),
                             nop(),                  mul_ab(), orl_imm(s.enable_sfr, s.enable_mask),
                             nop(),                  nop()};
                const Bench b = build(spec);
                const std::uint64_t limit = b.main_cycle(b.loop_index) + 80;
                for (const Mode mode : AllModes) {
                    const std::string ctx =
                        context("B2", mode, std::string(s.name) + " own enable m=" +
                                                std::to_string(m));
                    const Outcome out = execute(b, mode, limit);
                    expect_log(out, {{src, b.main_pc(b.body_index + 9), m}}, ctx);
                    expect_final(out, b, b.isr_cycles[src], b.isr_instructions[src], limit, ctx);
                }
            }
            {
                // Pending while EA = 0; SETB EA dispatches at the next boundary.
                BenchSpec spec;
                spec.body = {mov_imm(s.enable_sfr, s.enable_mask), orl_imm(s.flag_sfr, m), nop(),
                             mul_ab(), setb(EA_BIT), nop(), nop()};
                const Bench b = build(spec);
                const std::uint64_t limit = b.main_cycle(b.loop_index) + 80;
                for (const Mode mode : AllModes) {
                    const std::string ctx =
                        context("B2", mode, std::string(s.name) + " EA m=" + std::to_string(m));
                    const Outcome out = execute(b, mode, limit);
                    expect_log(out, {{src, b.main_pc(b.body_index + 5), m}}, ctx);
                    expect_final(out, b, b.isr_cycles[src], b.isr_instructions[src], limit, ctx);
                }
            }
            {
                // Cleared while disabled: no request remains.
                BenchSpec spec;
                spec.setup = {setb(EA_BIT)};
                spec.body = {orl_imm(s.flag_sfr, m), nop(),
                             anl_imm(s.flag_sfr, static_cast<std::uint8_t>(~m)), nop(),
                             orl_imm(s.enable_sfr, s.enable_mask), nop(), nop()};
                const Bench b = build(spec);
                const std::uint64_t limit = b.main_cycle(b.loop_index) + 40;
                for (const Mode mode : AllModes) {
                    const std::string ctx = context(
                        "B2", mode, std::string(s.name) + " cleared m=" + std::to_string(m));
                    const Outcome out = execute(b, mode, limit);
                    expect_log(out, {}, ctx);
                    expect_final(out, b, 0, 0, limit, ctx);
                    CHECK_IN(out.regs.posted_vector == 0, ctx);
                }
            }
        }
    }

    // Pending-state calculation, host side (no instruction executes).
    Cc2530F256 d;
    load(d, build({}).flash);
    const auto posted = [&](int source, unsigned level) {
        const Cc2530Registers r = d.registers();
        if (source < 0)
            return r.posted_vector == 0 && r.posted_source == -1;
        return r.posted_source == source && r.posted_vector == vector_of(source) &&
               r.posted_level == level;
    };
    d.raise_interrupt_flag_for_test(SrcEnc, 0x02);
    CHECK(posted(-1, 0)); // EA and ENCIE clear
    d.write_sfr(IEN0, 0x10);
    CHECK(posted(-1, 0)); // EA clear
    d.write_sfr(IEN0, 0x90);
    CHECK(posted(SrcEnc, 0));
    d.write_sfr(IP0, 0x10);
    CHECK(posted(SrcEnc, 1));
    d.write_sfr(IP1, 0x10);
    CHECK(posted(SrcEnc, 3));
    d.raise_interrupt_flag_for_test(SrcRf, 0x02);
    CHECK(posted(SrcEnc, 3)); // RFIE clear
    d.write_sfr(IEN2, 0x01);
    CHECK(posted(SrcEnc, 3)); // RF at level 0
    d.write_sfr(IP0, 0x11);
    d.write_sfr(IP1, 0x11);
    CHECK(posted(SrcRf, 3)); // tie at level 3: RF polls before ENC
    d.write_sfr(IP0, 0x10);
    CHECK(posted(SrcEnc, 3)); // RF level 2
    d.write_sfr(S0CON, 0x01);
    CHECK(posted(SrcEnc, 3)); // ENCIF_0 alone still requests
    d.write_sfr(S0CON, 0x00);
    CHECK(posted(SrcRf, 2));
    d.write_sfr(S1CON, 0x01);
    CHECK(posted(SrcRf, 2)); // RFIF_0 alone still requests
    d.write_sfr(IEN2, 0x00);
    CHECK(posted(-1, 0));
    CHECK(d.diagnostics().interrupts_taken == 0);
}

// B3: ENC (IPG4) and RF (IPG0) pending together, every flag-bit combination,
// arbitrated by level and then by Table 2-8 (RF before ENC).
void test_flag_priority() {
    struct Config {
        std::uint8_t ip0, ip1;
    };
    const Config configs[] = {{0x00, 0x00}, {0x10, 0x00}, {0x20, 0x00}, {0x08, 0x00},
                              {0x00, 0x10}, {0x01, 0x10}, {0x11, 0x11}, {0x12, 0x00},
                              {0x11, 0x00}, {0x01, 0x00}, {0x02, 0x02}, {0x00, 0x01},
                              {0x10, 0x01}, {0x21, 0x20}};
    for (const Config cfg : configs) {
        for (const std::uint8_t me : {std::uint8_t{1}, std::uint8_t{2}}) {
            for (const std::uint8_t mr : {std::uint8_t{1}, std::uint8_t{2}}) {
                BenchSpec spec;
                spec.setup = {mov_imm(IP0, cfg.ip0),  mov_imm(IP1, cfg.ip1),
                              mov_imm(IEN0, 0x10),    mov_imm(IEN2, 0x01),
                              orl_imm(S0CON, me),     orl_imm(S1CON, mr),
                              setb(EA_BIT)};
                spec.body = cat({repeat(nop(), 4), {mul_ab()}, repeat(nop(), 3)});
                const Bench b = build(spec);
                const unsigned le = level_of(SrcEnc, cfg.ip0, cfg.ip1);
                const unsigned lr = level_of(SrcRf, cfg.ip0, cfg.ip1);
                const bool rf_first =
                    lr > le || (lr == le && poll_position(SrcRf) < poll_position(SrcEnc));
                const unsigned first = rf_first ? SrcRf : SrcEnc;
                const unsigned second = rf_first ? SrcEnc : SrcRf;
                const std::uint8_t mf = rf_first ? mr : me, ms = rf_first ? me : mr;
                const std::uint64_t limit = b.main_cycle(b.loop_index) + 120;
                for (const Mode mode : AllModes) {
                    const std::string ctx = context(
                        "B3", mode,
                        "ip0=" + std::to_string(cfg.ip0) + " ip1=" + std::to_string(cfg.ip1) +
                            " enc=" + std::to_string(me) + " rf=" + std::to_string(mr));
                    const Outcome out = execute(b, mode, limit);
                    expect_log(out,
                               {{first, b.main_pc(b.body_index), mf},
                                {second, b.main_pc(b.body_index + 1), ms}},
                               ctx);
                    expect_final(out, b, b.isr_cycles[SrcEnc] + b.isr_cycles[SrcRf],
                                 b.isr_instructions[SrcEnc] + b.isr_instructions[SrcRf], limit,
                                 ctx);
                }
            }
        }
    }
}

// B4: ENCIF_1 / RFIF_1 raised while a higher-priority ISR runs wait for it;
// RFIF_1 at a higher level preempts T1 and ENC ISRs.
void test_flags_during_isr() {
    const std::vector<Ins> stretch{nop(), mul_ab(), nop(), mul_ab(), nop(), nop(), mul_ab()};
    {
        // T1 at level 3; ENC level 2, RF level 0: both wait, then ENC, then RF.
        BenchSpec spec;
        spec.setup = {mov_imm(IP0, 0x02), mov_imm(IP1, 0x12), mov_imm(IEN0, 0x10),
                      mov_imm(IEN1, 0x02), mov_imm(IEN2, 0x01), setb(EA_BIT)};
        spec.body = repeat(nop(), 8);
        spec.handlers[SrcT1].extra = stretch;
        const Bench b = build(spec);
        const std::size_t e1 = b.body_index + 1;
        const std::uint64_t entry1 = b.main_cycle(e1);
        const std::uint64_t limit = b.main_cycle(b.loop_index) + 160;
        for (const Mode mode : AllModes) {
            const std::string ctx = context("B4", mode, "behind T1 level 3");
            const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &d) {
                d.schedule_interrupt_flag_for_test(entry1, SrcT1);
                d.schedule_interrupt_flag_for_test(
                    b.handler_cycle(SrcT1, entry1, LogInstructions) + 1, SrcEnc, 0x02);
                d.schedule_interrupt_flag_for_test(
                    b.handler_cycle(SrcT1, entry1, LogInstructions + 3), SrcRf, 0x02);
            });
            expect_log(out,
                       {{SrcT1, b.main_pc(e1), 0x00},
                        {SrcEnc, b.main_pc(e1 + 1), 0x02},
                        {SrcRf, b.main_pc(e1 + 2), 0x02}},
                       ctx);
            expect_final(out, b,
                         b.isr_cycles[SrcT1] + b.isr_cycles[SrcEnc] + b.isr_cycles[SrcRf],
                         b.isr_instructions[SrcT1] + b.isr_instructions[SrcEnc] +
                             b.isr_instructions[SrcRf],
                         limit, ctx);
        }
    }
    for (const unsigned victim : {SrcT1, SrcEnc}) {
        for (const std::uint8_t ip0 : {std::uint8_t{0x01}, std::uint8_t{0x11}}) {
            // RF (IPG0) level 1 vs the victim at level 0 (preempts) or, with
            // ENC's IPG4 at level 1 too, at an equal level (waits).
            BenchSpec spec;
            spec.setup = {mov_imm(IP0, ip0), mov_imm(IEN0, 0x10), mov_imm(IEN1, 0x02),
                          mov_imm(IEN2, 0x01), setb(EA_BIT)};
            spec.body = repeat(nop(), 8);
            spec.handlers[victim].extra = stretch;
            const Bench b = build(spec);
            const std::size_t e1 = b.body_index + 1;
            const std::uint64_t entry1 = b.main_cycle(e1);
            const bool preempts = level_of(SrcRf, ip0, 0) > level_of(victim, ip0, 0);
            const std::uint64_t limit = b.main_cycle(b.loop_index) + 160;
            const std::uint8_t victim_value = victim == SrcEnc ? 0x02 : 0x00;
            for (std::size_t j = LogInstructions; j < LogInstructions + stretch.size(); ++j) {
                const std::uint64_t t = b.handler_cycle(victim, entry1, j) + 1;
                std::size_t k = j + 1;
                const std::uint16_t ret = preempts ? b.handler[victim].pc[k] : b.main_pc(e1 + 1);
                for (const Mode mode : AllModes) {
                    const std::string ctx =
                        context("B4", mode,
                                std::string(Sources[victim].name) + " ip0=" + std::to_string(ip0) +
                                    " j=" + std::to_string(j));
                    const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &d) {
                        d.schedule_interrupt_flag_for_test(entry1, victim,
                                                           victim == SrcEnc ? 0x02 : 0x02);
                        d.schedule_interrupt_flag_for_test(t, SrcRf, 0x02);
                    });
                    expect_log(out, {{victim, b.main_pc(e1), victim_value}, {SrcRf, ret, 0x02}},
                               ctx);
                    expect_final(out, b, b.isr_cycles[victim] + b.isr_cycles[SrcRf],
                                 b.isr_instructions[victim] + b.isr_instructions[SrcRf], limit,
                                 ctx);
                }
            }
        }
    }
}

// B5: a flag bit raised and cleared inside a higher-priority ISR withdraws
// the request; a bit left set is taken after RETI + one instruction.
void test_flag_cleared_while_blocked() {
    for (const unsigned src : FlagSources) {
        const Spec &s = Sources[src];
        for (const std::uint8_t keep : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{2}}) {
            BenchSpec spec;
            spec.setup = {mov_imm(IP0, 0x02), mov_imm(IP1, 0x02), mov_imm(IEN0, 0x10),
                          mov_imm(IEN1, 0x02), mov_imm(IEN2, 0x01), setb(EA_BIT)};
            spec.body = cat({{orl_imm(IRCON, 0x02)}, repeat(nop(), 6)});
            spec.handlers[SrcT1].extra = {nop(), orl_imm(s.flag_sfr, 0x03), nop(), mul_ab(),
                                          anl_imm(s.flag_sfr, static_cast<std::uint8_t>(~(3 & ~keep))),
                                          nop()};
            const Bench b = build(spec);
            const std::size_t e1 = b.body_index + 1;
            std::vector<Entry> want{{SrcT1, b.main_pc(e1), 0x00}};
            if (keep)
                want.push_back({src, b.main_pc(e1 + 1), keep});
            const std::uint64_t limit = b.main_cycle(b.loop_index) + 120;
            for (const Mode mode : AllModes) {
                const std::string ctx = context(
                    "B5", mode, std::string(s.name) + " keep=" + std::to_string(keep));
                const Outcome out = execute(b, mode, limit);
                expect_log(out, want, ctx);
                expect_final(out, b,
                             b.isr_cycles[SrcT1] + (keep ? b.isr_cycles[src] : 0),
                             b.isr_instructions[SrcT1] + (keep ? b.isr_instructions[src] : 0),
                             limit, ctx);
                CHECK_IN(out.regs.posted_vector == 0, ctx);
                CHECK_IN(out.sfr[s.flag_sfr] == 0, ctx);
            }
        }
    }
}

// ---------------------------------------------------------------- Gate C

// Main body around the PCON.IDLE write: the post-IDLE instruction INC 0x3c
// and then INC 0x3d. ISRs log 0x3c, so 1 proves the SWRU191F 4.2 order.
std::vector<Ins> idle_sequence(bool violate = false) {
    return {idle_pad(violate), idle_write(), inc_dir(PostIdle), inc_dir(AfterIdle)};
}

struct IdleBench {
    Bench b;
    std::size_t p{};     // main index of MOV PCON,#1
    std::uint64_t c0{};  // its start cycle
    std::uint64_t c1{};  // cycle after it
};

IdleBench idle_bench(BenchSpec spec, bool violate = false) {
    spec.body.insert(spec.body.begin(), nop());
    const auto seq = idle_sequence(violate);
    spec.body.insert(spec.body.end(), seq.begin(), seq.end());
    spec.body.push_back(nop());
    for (unsigned k = 0; k < 18; ++k)
        if (!spec.handlers.count(k))
            spec.handlers[k].logged = PostIdle;
    IdleBench ib{build(spec)};
    ib.p = ib.b.main.find(idle_write().bytes);
    ib.c0 = ib.b.main_cycle(ib.p);
    ib.c1 = ib.c0 + 2;
    return ib;
}

void expect_idle_diag(const Outcome &out, std::uint64_t entries, std::uint64_t wakes,
                      std::uint64_t immediate, std::uint64_t idle, const std::string &ctx) {
    CHECK_IN(out.diag.power_mode_entries == entries, ctx);
    CHECK_IN(out.diag.power_mode_wakes == wakes, ctx);
    CHECK_IN(out.diag.power_mode_immediate_wakes == immediate, ctx);
    CHECK_IN(out.diag.idle_cycles == idle,
             ctx + " idle=" + std::to_string(out.diag.idle_cycles) + " want=" +
                 std::to_string(idle));
}

// C1: no request: the CPU sleeps after the PCON write; a finite limit
// advances virtual time without executing instructions.
void test_idle_without_request() {
    BenchSpec spec;
    spec.setup = {mov_imm(IEN1, 0x02), setb(EA_BIT)};
    const IdleBench ib = idle_bench(spec);
    const Bench &b = ib.b;
    for (const Mode mode : AllModes) {
        const std::string ctx = context("C1", mode, "forever");
        const Outcome out = execute(b, mode, Forever);
        expect_log(out, {}, ctx);
        CHECK_IN(out.result.sleeping && out.sleeping, ctx);
        CHECK_IN(out.regs.cycles == ib.c1, ctx);
        CHECK_IN(out.regs.pc == b.main_pc(ib.p + 1), ctx);
        CHECK_IN(out.regs.instructions == 1 + ib.p + 1, ctx);
        CHECK_IN(out.idata[PostIdle] == 0, ctx);
        CHECK_IN(out.diag.idle_alignment_violations == 0, ctx);
        expect_idle_diag(out, 1, 0, 0, 0, ctx);
    }
    for (const Mode mode : AllModes) {
        const std::string ctx = context("C1", mode, "finite limits");
        const Outcome out = execute(b, mode, {{ib.c1 + 100, {}}, {ib.c1 + 250, {}}});
        CHECK_IN(out.result.sleeping && out.sleeping, ctx);
        CHECK_IN(out.result.instructions == 0, ctx);
        CHECK_IN(out.regs.cycles == ib.c1 + 250, ctx);
        CHECK_IN(out.regs.instructions == 1 + ib.p + 1, ctx);
        expect_idle_diag(out, 1, 0, 0, 250, ctx);
    }
}

// C2/C8/C9/C10: an enabled request arriving while idle wakes the CPU at its
// time; the post-IDLE instruction executes first (SWRU191F 4.2), then the
// ISR, then execution continues after the ISR returns. The placement
// diagnostic does not change the behavior. Variant 2 enters PM1
// (SLEEPCMD.MODE = 1), which only an I/O port or sleep timer interrupt ends
// (SWRU191F 4.1), and is woken by the sleep timer flag; variant 3 shows that
// an enabled T1 request does not end PM1.
void test_idle_wake_by_event() {
    for (const int variant : {0, 1, 2, 3}) {
        const bool pm1 = variant >= 2;
        const unsigned source = variant == 2 ? 5u /* ST */ : SrcT1;
        BenchSpec spec;
        if (variant == 2)
            spec.setup = {mov_imm(IEN0, 0xa0)}; // EA, STIE
        else
            spec.setup = {mov_imm(IEN1, 0x02), setb(EA_BIT)};
        if (pm1)
            spec.setup.push_back(mov_imm(SLEEPCMD, 0x05)); // MODE = PM1
        const IdleBench ib = idle_bench(spec, variant == 1);
        const Bench &b = ib.b;
        for (const std::uint64_t d : {1u, 2u, 5u, 17u}) {
            const std::uint64_t limit = b.main_cycle(b.loop_index) + d + 80;
            for (const Mode mode : AllModes) {
                const std::string ctx = context(
                    "C2", mode, "variant=" + std::to_string(variant) + " d=" + std::to_string(d));
                const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &dev) {
                    dev.schedule_interrupt_flag_for_test(ib.c1 + d, source);
                });
                CHECK_IN(out.diag.power_mode_entries_by_mode[pm1 ? 2 : 1] == 1, ctx);
                CHECK_IN(out.diag.idle_alignment_violations == (variant == 1 ? 1u : 0u), ctx);
                if (variant == 3) {
                    expect_log(out, {}, ctx);
                    CHECK_IN(out.result.sleeping && out.sleeping, ctx);
                    CHECK_IN(out.regs.cycles == limit, ctx);
                    CHECK_IN(out.regs.pc == b.main_pc(ib.p + 1), ctx);
                    CHECK_IN(out.idata[PostIdle] == 0, ctx);
                    expect_idle_diag(out, 1, 0, 0, limit - ib.c1, ctx);
                    continue;
                }
                expect_log(out, {{source, b.main_pc(ib.p + 2), 1}}, ctx);
                expect_final(out, b, d + b.isr_cycles[source], b.isr_instructions[source], limit,
                             ctx);
                expect_idle_diag(out, 1, 1, 0, d, ctx);
                CHECK_IN(out.idata[PostIdle] == 1 && out.idata[AfterIdle] == 1, ctx);
                CHECK_IN(out.diag.power_mode_cycles_by_mode[pm1 ? 2 : 1] == d, ctx);
            }
        }
    }
}

// C3: a pending request that is not enabled (source enable or EA clear)
// does not wake the CPU (SWRU191F 4.1.1 "any enabled interrupt"); enabling
// it later wakes it.
void test_idle_disabled_request() {
    struct Case {
        const char *name;
        std::vector<Ins> setup;
        std::uint8_t enable_sfr, enable_value;
    };
    const Case cases[] = {
        {"source disabled", {setb(EA_BIT), orl_imm(IRCON, 0x02)}, IEN1, 0x02},
        {"EA clear", {mov_imm(IEN1, 0x02), orl_imm(IRCON, 0x02)}, IEN0, 0x80},
    };
    for (const Case &c : cases) {
        BenchSpec spec;
        spec.setup = c.setup;
        const IdleBench ib = idle_bench(spec);
        const Bench &b = ib.b;
        const std::uint64_t wake = ib.c1 + 50;
        const std::uint64_t limit = b.main_cycle(b.loop_index) + 50 + 80;
        for (const Mode mode : AllModes) {
            const std::string ctx = context("C3", mode, c.name);
            const Outcome out =
                execute(b, mode,
                        {{wake, [&](Cc2530F256 &d) {
                              // An enabled-source event of a disabled source
                              // while idle sets its flag only.
                              d.schedule_interrupt_flag_for_test(ib.c1 + 10, SrcP0, 0x20);
                          }},
                         {limit, [&](Cc2530F256 &d) {
                              CHECK_IN(d.sleeping(), ctx + " asleep at the first limit");
                              CHECK_IN(d.registers().cycles == wake, ctx);
                              CHECK_IN(d.read_sfr(IRCON) == 0x22, ctx);
                              d.write_sfr(c.enable_sfr, c.enable_value);
                          }}});
            expect_log(out, {{SrcT1, b.main_pc(ib.p + 2), 1}}, ctx);
            CHECK_IN(out.diag.scheduled_flag_events == 1, ctx);
            // P0INT stays pending (never enabled); the run ends in SJMP $.
            CHECK_IN(out.sfr[IRCON] == 0x20, ctx);
            expect_idle_diag(out, 1, 1, 0, 50, ctx);
            CHECK_IN(out.regs.cycles >= limit && out.regs.pc == b.main_pc(b.loop_index), ctx);
        }
    }
}

// C4: an enabled request already pending when PCON.IDLE is written wakes the
// CPU at once (no idle time), with the same order as a later request.
// (a) the PCON write is the post-RETI instruction of an ISR while an
//     equal-level request waits;
// (b) the PCON write is inside an ISR that blocks the request by priority:
//     the request before and after the write give the same instruction
//     stream (R1-F1), differing only in idle time.
void test_idle_request_pending_at_entry() {
    {
        BenchSpec spec;
        spec.setup = {mov_imm(IEN1, 0x0a), setb(EA_BIT)};
        const IdleBench ib = idle_bench(spec);
        const Bench &b = ib.b;
        const std::uint64_t limit = b.main_cycle(b.loop_index) + 120;
        for (const Mode mode : AllModes) {
            const std::string ctx = context("C4a", mode, "post-RETI PCON");
            const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &d) {
                d.schedule_interrupt_flag_for_test(ib.c0, SrcT3);
                d.schedule_interrupt_flag_for_test(ib.c0, SrcT1);
            });
            expect_log(out, {{SrcT1, b.main_pc(ib.p), 0}, {SrcT3, b.main_pc(ib.p + 2), 1}}, ctx);
            expect_final(out, b, b.isr_cycles[SrcT1] + b.isr_cycles[SrcT3],
                         b.isr_instructions[SrcT1] + b.isr_instructions[SrcT3], limit, ctx);
            expect_idle_diag(out, 1, 1, 1, 0, ctx);
        }
    }
    for (const bool t3_enabled : {true, false}) {
        BenchSpec spec;
        spec.setup = {mov_imm(IP0, 0x02), mov_imm(IEN1, t3_enabled ? 0x0a : 0x02), setb(EA_BIT)};
        spec.body = repeat(nop(), 6);
        spec.handlers[SrcT1] = {PostIdle, {nop(), idle_pad(), idle_write(), inc_dir(PostIdle),
                                           nop(), mul_ab()}};
        spec.handlers[SrcT3].logged = PostIdle;
        const Bench b = build(spec);
        const Placed &h = b.handler[SrcT1];
        const std::size_t e1 = b.body_index + 1;
        const std::uint64_t entry1 = b.main_cycle(e1);
        const std::size_t hp = h.find(idle_write().bytes);
        const std::uint64_t h_c1 = b.handler_cycle(SrcT1, entry1, hp) + 2;
        const std::uint64_t limit = b.main_cycle(b.loop_index) + 160;
        Outcome outs[2][3];
        for (const bool before : {true, false}) {
            const std::uint64_t t3 =
                before ? b.handler_cycle(SrcT1, entry1, LogInstructions) : h_c1 + 10;
            for (const Mode mode : AllModes) {
                const std::string ctx = context(
                    "C4b", mode,
                    std::string(before ? "request before" : "request after") +
                        (t3_enabled ? "" : " T3 disabled"));
                const Outcome out = execute(b, mode, t3_enabled ? limit : Forever,
                                            [&](Cc2530F256 &d) {
                                                d.schedule_interrupt_flag_for_test(entry1, SrcT1);
                                                d.schedule_interrupt_flag_for_test(t3, SrcT3);
                                            });
                if (t3_enabled) {
                    expect_log(out, {{SrcT1, b.main_pc(e1), 0}, {SrcT3, b.main_pc(e1 + 1), 1}},
                               ctx);
                    const std::uint64_t idle = before ? 0 : 10;
                    expect_final(out, b, idle + b.isr_cycles[SrcT1] + b.isr_cycles[SrcT3],
                                 b.isr_instructions[SrcT1] + b.isr_instructions[SrcT3], limit,
                                 ctx);
                    expect_idle_diag(out, 1, 1, before ? 1 : 0, idle, ctx);
                } else {
                    expect_log(out, {{SrcT1, b.main_pc(e1), 0}}, ctx);
                    CHECK_IN(out.sleeping && out.in_interrupt, ctx);
                    CHECK_IN(out.regs.pc == h.pc[hp + 1], ctx);
                    CHECK_IN(out.sfr[IRCON] == 0x08, ctx);
                    expect_idle_diag(out, 1, 0, 0, before ? 0 : 10, ctx);
                }
                outs[before][static_cast<int>(mode)] = out;
            }
        }
        // Same instruction stream whatever the order: equal IDATA (logs,
        // counters, stack) and instruction counts. With T3 enabled both runs
        // stop at the same cycle limit, so the later request's 10 idle cycles
        // replace exactly five 2-cycle SJMP $ iterations; with T3 disabled
        // both runs stop asleep after the same instructions.
        for (int m = 0; m < 3; ++m) {
            CHECK(outs[0][m].idata == outs[1][m].idata);
            CHECK(outs[1][m].regs.instructions ==
                  outs[0][m].regs.instructions + (t3_enabled ? 10 / 2 : 0));
        }
    }
}

// C5: several requests of different priorities while idle: one wake, the
// post-IDLE instruction, then the highest level first.
void test_idle_multiple_requests() {
    BenchSpec spec;
    spec.setup = {mov_imm(IP0, 0x08), mov_imm(IEN1, 0x0a), setb(EA_BIT)};
    const IdleBench ib = idle_bench(spec);
    const Bench &b = ib.b;
    const std::uint64_t limit = b.main_cycle(b.loop_index) + 200;
    struct Case {
        std::uint64_t t1, t3;
        std::vector<Entry> want;
    };
    const std::uint64_t w = ib.c1 + 7;
    const Case cases[] = {
        // Together: T3 (level 1) first, T1 after T3's RETI and INC 0x3d.
        {w, w, {{SrcT3, b.main_pc(ib.p + 2), 1}, {SrcT1, b.main_pc(ib.p + 3), 1}}},
        // T3 at the end of the post-IDLE instruction: both pending there.
        {w, w + 1, {{SrcT3, b.main_pc(ib.p + 2), 1}, {SrcT1, b.main_pc(ib.p + 3), 1}}},
        // T3 after T1's entry started: preempts at T1's vector instruction.
        {w, w + 2, {{SrcT3, vector_of(SrcT1), 1}, {SrcT1, b.main_pc(ib.p + 2), 1}}},
    };
    for (const Case &c : cases) {
        for (const Mode mode : AllModes) {
            const std::string ctx = context("C5", mode, "t3=w+" + std::to_string(c.t3 - w));
            const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &d) {
                d.schedule_interrupt_flag_for_test(c.t1, SrcT1);
                d.schedule_interrupt_flag_for_test(c.t3, SrcT3);
            });
            expect_log(out, c.want, ctx);
            expect_final(out, b, 7 + b.isr_cycles[SrcT1] + b.isr_cycles[SrcT3],
                         b.isr_instructions[SrcT1] + b.isr_instructions[SrcT3], limit, ctx);
            expect_idle_diag(out, 1, 1, 0, 7, ctx);
        }
    }
}

// C6: request and PCON.IDLE at the same virtual time. A request due at the
// PCON instruction's start boundary is taken before it executes; one due
// during it or at its end boundary wakes the CPU with no idle time.
void test_idle_same_time() {
    BenchSpec spec;
    spec.setup = {mov_imm(IEN1, 0x02), setb(EA_BIT)};
    const IdleBench ib = idle_bench(spec);
    const Bench &b = ib.b;
    for (const Mode mode : AllModes) {
        const std::string ctx = context("C6", mode, "t=c0");
        const Outcome out = execute(b, mode, Forever, [&](Cc2530F256 &d) {
            d.schedule_interrupt_flag_for_test(ib.c0, SrcT1);
        });
        expect_log(out, {{SrcT1, b.main_pc(ib.p), 0}}, ctx);
        CHECK_IN(out.sleeping, ctx);
        CHECK_IN(out.regs.pc == b.main_pc(ib.p + 1), ctx);
        CHECK_IN(out.regs.cycles == ib.c1 + b.isr_cycles[SrcT1], ctx);
        expect_idle_diag(out, 1, 0, 0, 0, ctx);
    }
    const std::uint64_t limit = b.main_cycle(b.loop_index) + 100;
    for (const std::uint64_t t : {ib.c0 + 1, ib.c1, ib.c1 + 1}) {
        const std::uint64_t idle = t > ib.c1 ? t - ib.c1 : 0;
        for (const Mode mode : AllModes) {
            const std::string ctx = context("C6", mode, "t=c0+" + std::to_string(t - ib.c0));
            const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &d) {
                d.schedule_interrupt_flag_for_test(t, SrcT1);
            });
            expect_log(out, {{SrcT1, b.main_pc(ib.p + 2), 1}}, ctx);
            expect_final(out, b, idle + b.isr_cycles[SrcT1], b.isr_instructions[SrcT1], limit,
                         ctx);
            expect_idle_diag(out, 1, 1, 0, idle, ctx);
        }
    }
}

// C7: after a wake, a higher-priority request preempts the waking ISR.
void test_idle_wake_then_preempt() {
    BenchSpec spec;
    spec.setup = {mov_imm(IP0, 0x08), mov_imm(IEN1, 0x0a), setb(EA_BIT)};
    spec.handlers[SrcT1] = {PostIdle, {nop(), mul_ab(), nop(), mul_ab(), nop()}};
    const IdleBench ib = idle_bench(spec);
    const Bench &b = ib.b;
    const std::uint64_t w = ib.c1 + 5;
    const std::uint64_t entry1 = w + 1;
    const std::uint64_t limit = b.main_cycle(b.loop_index) + 200;
    for (std::size_t j = LogInstructions; j < LogInstructions + 5; ++j) {
        const std::uint64_t t3 = b.handler_cycle(SrcT1, entry1, j) + 1;
        for (const Mode mode : AllModes) {
            const std::string ctx = context("C7", mode, "j=" + std::to_string(j));
            const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &d) {
                d.schedule_interrupt_flag_for_test(w, SrcT1);
                d.schedule_interrupt_flag_for_test(t3, SrcT3);
            });
            expect_log(out,
                       {{SrcT1, b.main_pc(ib.p + 2), 1}, {SrcT3, b.handler[SrcT1].pc[j + 1], 1}},
                       ctx);
            expect_final(out, b, 5 + b.isr_cycles[SrcT1] + b.isr_cycles[SrcT3],
                         b.isr_instructions[SrcT1] + b.isr_instructions[SrcT3], limit, ctx);
            expect_idle_diag(out, 1, 1, 0, 5, ctx);
        }
    }
}

// C14: the state at a run stop does not depend on why the run stopped. A
// run whose cycle limit falls inside or at the end of the PCON write stops
// at the write's end boundary (c1). A request due there (or during the
// write) wakes the CPU at that boundary (wake rule, C6), so every mode must
// return awake with the post-wake inhibit armed and the request posted;
// before the wake fix a run(1) step returned asleep with the request posted
// and woke only in the next call. A request due later leaves the CPU asleep
// with nothing posted. Either way "asleep" never coexists with a posted
// request at a stop.
void test_idle_state_at_stop() {
    BenchSpec spec;
    spec.setup = {mov_imm(IEN1, 0x02), setb(EA_BIT)};
    const IdleBench ib = idle_bench(spec);
    const Bench &b = ib.b;
    const std::uint64_t limit = b.main_cycle(b.loop_index) + 100;
    for (const std::uint64_t stop : {ib.c0 + 1, ib.c1}) {
        for (const std::uint64_t t : {ib.c0 + 1, ib.c1, ib.c1 + 1}) {
            const bool woken = t <= ib.c1;
            const std::uint64_t idle = woken ? 0 : t - ib.c1;
            for (const Mode mode : AllModes) {
                const std::string ctx =
                    context("C14", mode,
                            "stop=c0+" + std::to_string(stop - ib.c0) +
                                " t=c0+" + std::to_string(t - ib.c0));
                const Outcome out = execute(
                    b, mode,
                    {{stop, [&](Cc2530F256 &d) { d.schedule_interrupt_flag_for_test(t, SrcT1); }},
                     {limit, [&](Cc2530F256 &d) {
                          const Cc2530Registers r = d.registers();
                          CHECK_IN(r.cycles == ib.c1 && r.pc == b.main_pc(ib.p + 1), ctx);
                          CHECK_IN(d.sleeping() == !woken, ctx + " sleeping");
                          CHECK_IN(r.interrupt_inhibit == (woken ? 1 : 0), ctx + " inhibit");
                          CHECK_IN(r.posted_vector == (woken ? vector_of(SrcT1) : 0),
                                   ctx + " posted");
                          CHECK_IN(!(d.sleeping() && r.posted_vector != 0), ctx);
                          CHECK_IN(d.diagnostics().power_mode_wakes == (woken ? 1u : 0u),
                                   ctx + " wakes");
                          CHECK_IN(d.diagnostics().idle_cycles == 0, ctx);
                          CHECK_IN(d.scheduled_event_count() == (woken ? 0u : 1u), ctx);
                      }}});
                expect_log(out, {{SrcT1, b.main_pc(ib.p + 2), 1}}, ctx);
                expect_final(out, b, idle + b.isr_cycles[SrcT1], b.isr_instructions[SrcT1],
                             limit, ctx);
                expect_idle_diag(out, 1, 1, 0, idle, ctx);
            }
        }
    }
}

// C13: SWRU191F 4.2 "If this instruction is a global interrupt disable, it is
// possible to have it followed by code for execution after wakeup, but
// before the ISR is serviced." The post-IDLE instruction is CLR EA: the wake
// still happens, the code after it runs, and the ISR waits for EA.
// (a) SETB EA later: the ISR runs after both INCs (documented); that it is
//     taken at the boundary right after SETB EA is the MCUEMU rule for EA
//     writes (docs/cc2530.md, model-defined).
// (b) EA stays clear: no ISR, T1IF stays pending, the CPU stays awake.
void test_idle_post_clr_ea() {
    for (const bool reenable : {true, false}) {
        BenchSpec spec;
        spec.setup = {mov_imm(IEN1, 0x02), setb(EA_BIT)};
        spec.body = {nop(),           idle_pad(),          idle_write(), clr(EA_BIT),
                     inc_dir(PostIdle), inc_dir(AfterIdle)};
        if (reenable)
            spec.body.push_back(setb(EA_BIT));
        spec.body.push_back(nop());
        spec.handlers[SrcT1].logged = AfterIdle;
        const Bench b = build(spec);
        const std::size_t p = b.main.find(idle_write().bytes);
        const std::uint64_t c1 = b.main_cycle(p) + 2;
        for (const std::uint64_t d : {1u, 7u}) {
            const std::uint64_t limit = b.main_cycle(b.loop_index) + d + 80;
            for (const Mode mode : AllModes) {
                const std::string ctx = context("C13", mode,
                                                std::string(reenable ? "SETB EA" : "EA clear") +
                                                    " d=" + std::to_string(d));
                const Outcome out = execute(b, mode, limit, [&](Cc2530F256 &dev) {
                    dev.schedule_interrupt_flag_for_test(c1 + d, SrcT1);
                });
                CHECK_IN(out.idata[PostIdle] == 1 && out.idata[AfterIdle] == 1, ctx);
                expect_idle_diag(out, 1, 1, 0, d, ctx);
                CHECK_IN(!out.sleeping && out.regs.pc == b.main_pc(b.loop_index), ctx);
                CHECK_IN(out.regs.sp == StackStart && out.regs.interrupt_levels == 0, ctx);
                if (reenable) {
                    expect_log(out, {{SrcT1, b.main_pc(p + 5), 1}}, ctx);
                    CHECK_IN((out.sfr[IEN0] & 0x80) != 0, ctx);
                } else {
                    expect_log(out, {}, ctx);
                    CHECK_IN((out.sfr[IEN0] & 0x80) == 0, ctx);
                    CHECK_IN((out.sfr[IRCON] & 0x02) != 0, ctx);
                    CHECK_IN(out.regs.posted_vector == 0, ctx);
                }
            }
        }
    }
}

// C11/C12: repeated IDLE in a loop, and a reset while asleep.
void test_idle_loop_and_reset() {
    BenchSpec spec;
    spec.setup = {mov_imm(IEN1, 0x02), setb(EA_BIT)};
    // L: MOV PCON,#1 / INC 0x3c / SJMP L
    spec.body = {nop(), idle_pad(), idle_write(), inc_dir(PostIdle), {{0x80, 0xf9}, 2}};
    const Bench b = build(spec);
    const std::size_t p = b.main.find(idle_write().bytes);
    const std::uint64_t c1 = b.main_cycle(p) + 2;
    const std::uint64_t awake = 1 + b.isr_cycles[SrcT1] + 2 + 2; // INC, ISR, SJMP, PCON
    const std::uint64_t events[] = {c1 + 10, c1 + 100, c1 + 200};
    for (const Mode mode : AllModes) {
        const std::string ctx = context("C12", mode, "loop");
        const Outcome out = execute(b, mode, Forever, [&](Cc2530F256 &d) {
            for (const std::uint64_t t : events)
                d.schedule_interrupt_flag_for_test(t, SrcT1);
        });
        std::vector<Entry> want;
        for (std::uint8_t k = 0; k < 3; ++k)
            want.push_back({SrcT1, b.main_pc(p + 2), 0x00});
        expect_log(out, want, ctx);
        CHECK_IN(out.sleeping, ctx);
        CHECK_IN(out.idata[PostIdle] == 3, ctx);
        CHECK_IN(out.regs.cycles == events[2] + awake, ctx);
        CHECK_IN(out.regs.pc == b.main_pc(p + 1), ctx);
        expect_idle_diag(out, 4, 3, 0, 10 + (100 - 10 - awake) + (100 - awake), ctx);
    }
    for (const Mode mode : AllModes) {
        const std::string ctx = context("C11", mode, "reset while asleep");
        const Outcome out = execute(b, mode,
                                    {{c1 + 500, [&](Cc2530F256 &d) {
                                          d.schedule_interrupt_flag_for_test(c1 + 1000, SrcT1);
                                      }},
                                     {Forever, [&](Cc2530F256 &d) {
                                          CHECK_IN(d.sleeping(), ctx);
                                          CHECK_IN(d.registers().cycles == c1 + 500, ctx);
                                          d.reset();
                                          CHECK_IN(!d.sleeping(), ctx);
                                          CHECK_IN(d.scheduled_event_count() == 0, ctx);
                                          CHECK_IN(d.pc() == 0, ctx);
                                      }}});
        // The reset discarded the c1+1000 event; the program runs again from
        // the reset vector (the cycle counter keeps counting) and sleeps for
        // good with nothing left to wake it.
        CHECK_IN(out.error.empty(), ctx);
        CHECK_IN(out.sleeping, ctx);
        CHECK_IN(out.log.empty(), ctx);
        CHECK_IN(out.regs.cycles == 2 * c1 + 500, ctx);
        CHECK_IN(out.regs.pc == b.main_pc(p + 1), ctx);
        CHECK_IN(out.diag.scheduled_flag_events == 0, ctx);
        expect_idle_diag(out, 2, 0, 0, 500, ctx);
    }
}

// T0: the engine's per-opcode cycle count equals the documented MCUEMU
// timing contract (contract_cycles, written from the Intel MCS-51 table and
// the documented exception list). Each opcode executes once from reset with
// harmless operands (direct 0x30, bit 0x30, rel 0, DPTR 0).
void test_cycle_contract() {
    for (unsigned opcode = 0; opcode < 0x100; ++opcode) {
        if (opcode == 0xa5)
            continue; // reserved: rejected as an invalid instruction
        for (const Mode mode : {Mode::Fast, Mode::Trace}) {
            Cc2530F256 device;
            std::vector<std::uint8_t> image(0x100, 0x00);
            image[0] = static_cast<std::uint8_t>(opcode);
            image[1] = 0x30;
            image[2] = 0x00;
            device.load_flash(0, image);
            std::ostringstream sink;
            device.set_trace(mode == Mode::Trace ? &sink : nullptr);
            const Cc2530RunResult r = device.run(1);
            std::ostringstream ctx;
            ctx << "T0 opcode 0x" << std::hex << opcode << ' ' << mode_name(mode);
            CHECK_IN(r.error.empty(), ctx.str() + ": " + r.error);
            CHECK_IN(r.instructions == 1, ctx.str());
            CHECK_IN(r.cycles == contract_cycles(static_cast<std::uint8_t>(opcode)),
                     ctx.str() + " cycles " + std::to_string(r.cycles));
        }
    }
}

} // namespace

int main() {
    using Test = void (*)();
    const std::pair<const char *, Test> tests[] = {
        {"T0 cycle contract per opcode", test_cycle_contract},
        {"A1 async event without SFR writes", test_async_event_without_sfr_writes},
        {"A2/A3 event inside ISR", test_event_inside_isr},
        {"A4 event in banked code", test_event_in_banked_code},
        {"A5 same-time events", test_same_time_events},
        {"A6 event API contract", test_event_api_contract},
        {"A7 enable and priority writes", test_enable_and_priority_writes},
        {"B1 flag bits independently", test_flag_bits_independently},
        {"B2 flag masking", test_flag_masking},
        {"B3 flag priority", test_flag_priority},
        {"B4 flags during ISR", test_flags_during_isr},
        {"B5 flag cleared while blocked", test_flag_cleared_while_blocked},
        {"C1 idle without request", test_idle_without_request},
        {"C2 idle wake by event", test_idle_wake_by_event},
        {"C3 idle disabled request", test_idle_disabled_request},
        {"C4 idle request pending at entry", test_idle_request_pending_at_entry},
        {"C5 idle multiple requests", test_idle_multiple_requests},
        {"C6 idle same time", test_idle_same_time},
        {"C7 idle wake then preempt", test_idle_wake_then_preempt},
        {"C13 idle post-IDLE CLR EA", test_idle_post_clr_ea},
        {"C14 idle state at a run stop", test_idle_state_at_stop},
        {"C11/C12 idle loop and reset", test_idle_loop_and_reset},
    };
    for (const auto &[name, test] : tests) {
        const unsigned before = failures;
        try {
            test();
        } catch (const std::exception &e) {
            std::cerr << name << ": exception: " << e.what() << '\n';
            ++failures;
        }
        std::cout << (failures == before ? "PASS " : "FAIL ") << name << '\n';
    }
    if (failures) {
        std::cerr << failures << " check(s) failed\n";
        return 1;
    }
    std::cout << "all CC2530 R2 scheduler tests passed\n";
    return 0;
}
