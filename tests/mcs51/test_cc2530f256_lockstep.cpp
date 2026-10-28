// EMU-04A-R2 Gate D: fast / step / trace differential harness for the CC2530
// model.
//
// The same program and the same virtual-time interrupt events run on three
// devices:
//   fast  : run(N, sync) - the optimized engine loop, stopping only at the
//           engine's EVENT_CYCLES / BREAK_PC / instruction-limit stops;
//   step  : run(1, sync) repeated - run() re-evaluates events and interrupts
//           at every instruction boundary; nothing relies on engine stops;
//   trace : run(N, sync) with tracing - the per-instruction code hook
//           re-evaluates events and interrupts before every instruction.
// At every synchronization point (cycle limits; the first instruction
// boundary at or after each) the complete machine state is compared: all
// registers (PC, SP, PSW, A, B, R0-R7, DPTR0/DPTR1, DPS, FMAP, MEMCTR,
// MPAGE), cycles, instructions, in-progress interrupt levels, post-RETI
// inhibit, the posted request, all 128 SFRs, IDATA, the 8 KiB SRAM, the
// event queue, sleep state and every diagnostic counter. The firmware keeps
// an interrupt log in SRAM (source, SP, return address of every entry, and
// IRCON as read by the first ISR instruction), so a dispatch at a different
// boundary, or a hardware flag clear on vector entry that a lane performs
// late, differs even if the run ends at the same PC. Sync schedules: every
// instruction (step vs trace), dense (1..24 cycles), sparse (100..3000
// cycles), event edges (one cycle before, at and after every event) and one
// final stop; the final state must not depend on the schedule, and no lane
// may stop asleep with a posted request.
//
// Events: pseudo-random sets plus events aimed at exact boundaries taken from
// a reference run: PCON.IDLE writes, RETI and its post-RETI instruction,
// interrupt entries, FMAP writes, banked LCALL/RET and banked code, the
// middle of MUL/DIV, and time spent in IDLE. Coverage of each class is
// asserted.
//
// Optional: --image FILE.hex [--horizon CYCLES] runs a firmware image
// (no injected events) through the same comparison; --audit adds the
// per-step timing-contract, peak-SP/guard and reset-in-ISR checks
// (audit_image below).
#include "cc2530_r2_support.hpp"

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <map>
#include <set>

using mcs51emu::Cc2530Diagnostics;
using mcs51emu::Cc2530Registers;
using r2::Cc2530F256;
using r2::Cc2530RunResult;

namespace {

// ------------------------------------------------------------- assembler
class Asm {
public:
    explicit Asm(std::uint16_t origin) : origin_(origin) {}
    std::uint16_t pc() const { return static_cast<std::uint16_t>(origin_ + bytes_.size()); }
    void b(std::initializer_list<std::uint8_t> bytes) {
        bytes_.insert(bytes_.end(), bytes.begin(), bytes.end());
    }
    void label(const std::string &name) {
        if (!labels_.emplace(name, pc()).second)
            throw std::logic_error("duplicate label " + name);
    }
    // Opcode bytes followed by an 8-bit relative offset to `target`.
    void rel(std::initializer_list<std::uint8_t> bytes, const std::string &target) {
        b(bytes);
        fixes_.push_back({bytes_.size(), target, true});
        bytes_.push_back(0);
    }
    // Opcode followed by a 16-bit absolute address.
    void abs(std::uint8_t opcode, const std::string &target) {
        b({opcode});
        fixes_.push_back({bytes_.size(), target, false});
        bytes_.push_back(0);
        bytes_.push_back(0);
    }
    // NOPs until the 3-byte instruction that follows ends off a 4-byte
    // boundary (SWRU191F 4.2 placement of the PCON.IDLE write).
    void align_idle() {
        while (((pc() + 3) & 3u) == 0)
            b({0x00});
    }
    std::uint16_t at(const std::string &name) const { return labels_.at(name); }
    std::vector<std::uint8_t> finish() const {
        std::vector<std::uint8_t> out = bytes_;
        for (const Fix &fix : fixes_) {
            const std::uint16_t target = labels_.at(fix.target);
            if (fix.relative) {
                const int delta = int(target) - int(origin_ + fix.at + 1);
                if (delta < -128 || delta > 127)
                    throw std::logic_error("branch out of range to " + fix.target);
                out[fix.at] = static_cast<std::uint8_t>(delta);
            } else {
                out[fix.at] = static_cast<std::uint8_t>(target >> 8);
                out[fix.at + 1] = static_cast<std::uint8_t>(target);
            }
        }
        return out;
    }

private:
    struct Fix {
        std::size_t at;
        std::string target;
        bool relative;
    };
    std::uint16_t origin_;
    std::vector<std::uint8_t> bytes_;
    std::map<std::string, std::uint16_t> labels_;
    std::vector<Fix> fixes_;
};

constexpr std::uint8_t ACC = 0xe0, B = 0xf0, PSW = 0xd0, DPS = 0x92, MPAGE = 0x93, DPL1 = 0x84,
                       DPH1 = 0x85;
constexpr std::uint8_t DMAIF_BIT = 0xc0, P0IF_BIT = 0xc5;
constexpr unsigned SrcEnc = 4, SrcUrx0 = 2, SrcSt = 5, SrcDma = 8, SrcT1 = 9, SrcT3 = 11,
                   SrcP0 = 13, SrcRf = 16;
constexpr std::uint16_t LogBase = 0x1000; // SRAM interrupt log (8 bytes per entry)
constexpr std::size_t LogEntry = 8;

struct Program {
    std::vector<std::uint8_t> flash;
    std::set<std::uint16_t> vectors;
};

// Handler for source `k`: log (EA cleared around it), then `body`, then
// restore and RETI. The log entry is [0x80|k, SP, return high, return low,
// IRCON, 0, 0, 0] at MPAGE:R0 of register bank 1 (R7 = page). IRCON is read
// by the first ISR instruction, so a T1IF/T3IF that a lane has not yet
// cleared on vector entry (SWRU191F Table 2-5 "cleared when the CPU vectors
// to the ISR") changes the log even when the lanes agree again afterwards.
void handler(Asm &a, unsigned k, const std::function<void(Asm &)> &body) {
    a.label("h" + std::to_string(k));
    a.b({0xc0, r2::IRCON});      // PUSH IRCON: first ISR instruction
    a.b({0xc0, ACC, 0xc0, PSW}); // PUSH ACC; PUSH PSW
    a.b({0x75, PSW, 0x08});      // bank 1
    a.b({0xc2, 0xaf});           // CLR EA
    a.b({0x8f, MPAGE});          // MOV MPAGE,R7
    a.b({0x74, static_cast<std::uint8_t>(0x80 | k), 0xf2, 0x08}); // marker
    a.b({0xe5, r2::SP, 0xf2, 0x08});                              // SP
    a.b({0xe5, r2::SP, 0x24, 0xfd, 0xf9, 0xe7, 0xf2, 0x08});      // return high
    a.b({0x19, 0xe7, 0xf2, 0x08});                                // return low
    a.b({0x09, 0x09, 0xe7, 0xf2, 0x08});                          // IRCON at entry
    a.b({0x08, 0x08, 0x08});                                      // INC R0 x3
    a.rel({0xb8, 0x00}, "h" + std::to_string(k) + "_nopage");     // CJNE R0,#0
    a.b({0x0f});                                                  // INC R7
    a.label("h" + std::to_string(k) + "_nopage");
    a.b({0xd2, 0xaf}); // SETB EA
    body(a);
    a.b({0xd0, PSW, 0xd0, ACC, 0xd0, 0x3f, 0x32}); // POP PSW; POP ACC; POP 0x3F; RETI
}

// Main program: DPTR0/DPTR1/DPS, MOVX, MOVC, MUL/DIV, the 1-cycle exception
// opcodes, a banked call (FMAP 3), PCON.IDLE every fourth pass and an
// EA-off window with an IEN1 rewrite every eighth pass. Six enabled sources
// at four levels (IP0 = 0x29, IP1 = 0x11): DMA/RF level 3, ENC level 2,
// T3/P0INT level 1, T1 level 0. ISRs nest, change IP0 and IEN1, raise a
// flag by software, use DPTR1, call bank 4, and P0INT idles inside its ISR.
Program mixed_program() {
    Program p;
    p.flash.assign(Cc2530F256::FlashSize, 0xff);
    Asm a(0x0000);
    a.abs(0x02, "main");
    const unsigned sources[] = {SrcEnc, SrcDma, SrcT1, SrcT3, SrcP0, SrcRf};
    for (const unsigned k : sources) {
        while (a.pc() < r2::vector_of(k))
            a.b({0x00});
        a.abs(0x02, "h" + std::to_string(k));
        p.vectors.insert(r2::vector_of(k));
    }
    while (a.pc() < 0x0100)
        a.b({0x00});
    handler(a, SrcDma, [](Asm &h) {
        h.b({0xc2, DMAIF_BIT});                         // CLR DMAIF
        h.b({0xc0, DPS, 0xc0, DPL1, 0xc0, DPH1});       // save DPS, DPTR1
        h.b({0x75, DPS, 0x01, 0x90, 0x01, 0x00});       // DPS=1; DPTR1=#0x0100
        h.b({0xe0, 0x04, 0xf0, 0xa3, 0xf0});            // MOVX; INC A; MOVX; INC DPTR; MOVX
        h.b({0xd0, DPH1, 0xd0, DPL1, 0xd0, DPS});       // restore
    });
    handler(a, SrcT1, [](Asm &h) {
        h.b({0x63, r2::IP0, 0x08}); // XRL IP0,#0x08: T3 group level 1 <-> 0
        h.b({0xd2, DMAIF_BIT});     // SETB DMAIF: a level-3 request by software
        h.b({0x00, 0x00});
    });
    handler(a, SrcT3, [](Asm &h) {
        h.b({0x75, B, 0x07, 0xa4}); // MOV B,#7; MUL AB
        h.b({0x7a, 0x06});          // MOV R2,#6
        h.label("t3_spin");
        h.rel({0xda}, "t3_spin");   // DJNZ R2,$
    });
    handler(a, SrcP0, [](Asm &h) {
        h.b({0xc2, P0IF_BIT});         // CLR P0IF
        h.b({0x05, 0x36, 0xe5, 0x36}); // INC 0x36; MOV A,0x36
        h.b({0x54, 0x03});             // ANL A,#3
        h.rel({0x70}, "p0_noidle");    // JNZ
        h.align_idle();
        h.b({0x75, r2::PCON, 0x01});   // PCON.IDLE inside a level-1 ISR
        h.b({0x05, 0x3b});             // INC 0x3B (post-IDLE instruction)
        h.label("p0_noidle");
        h.b({0x00});
    });
    handler(a, SrcRf, [](Asm &h) {
        h.b({0x53, r2::S1CON, 0xfc});        // clear RFIF_1:0
        h.b({0xc0, r2::FMAP, 0x75, r2::FMAP, 0x04}); // PUSH FMAP; FMAP=4
        h.b({0x12, 0x80, 0x00});             // LCALL bank 4
        h.b({0xd0, r2::FMAP});               // POP FMAP
    });
    handler(a, SrcEnc, [](Asm &h) {
        h.b({0x53, r2::S0CON, 0xfc});  // clear ENCIF_1:0
        h.b({0x63, r2::IEN1, 0x20});   // XRL IEN1,#0x20: toggle P0INT enable
    });

    while (a.pc() < 0x0800)
        a.b({0x00});
    a.label("main");
    a.b({0x75, r2::SP, 0x40});
    a.b({0x75, 0x08, 0x00, 0x75, 0x0f, static_cast<std::uint8_t>(LogBase >> 8)});
    a.b({0x75, r2::IP0, 0x29, 0x75, r2::IP1, 0x11});
    a.b({0x75, r2::IEN1, 0x2b, 0x75, r2::IEN2, 0x01, 0x75, r2::IEN0, 0x90});
    a.b({0x7e, 0x00}); // MOV R6,#0
    a.label("loop");
    a.b({0x90, 0x02, 0x00, 0x75, DPS, 0x01, 0x90, 0x03, 0x00, 0x75, DPS, 0x00});
    a.b({0xee, 0xf0, 0xa3});                         // MOV A,R6; MOVX @DPTR,A; INC DPTR
    a.b({0x75, DPS, 0x01, 0xe0, 0x2e, 0xf0, 0xa3, 0x75, DPS, 0x00});
    a.b({0xee, 0x75, B, 0x07, 0xa4, 0xf5, 0x31});    // MUL AB -> 0x31
    a.b({0x75, B, 0x03, 0x84, 0xf5, 0x37});          // DIV AB -> 0x37
    a.b({0xee, 0x93, 0xf5, 0x38});                   // MOVC A,@A+DPTR -> 0x38
    a.b({0xc0, 0x31, 0xd0, 0x32});                   // PUSH 0x31; POP 0x32
    a.b({0xa8, 0x31, 0x88, 0x39});                   // MOV R0,0x31; MOV 0x39,R0
    a.b({0xa2, 0xe0, 0x72, 0xe1, 0x82, 0xe2, 0xa0, 0xe3, 0xb0, 0xe4, 0x92, 0x00});
    a.b({0x75, r2::FMAP, 0x03, 0x12, 0x80, 0x00, 0x75, r2::FMAP, 0x01});
    a.b({0xee, 0x54, 0x03});
    a.rel({0x70}, "no_idle");
    a.align_idle();
    a.b({0x75, r2::PCON, 0x01});
    a.b({0x05, 0x33}); // post-IDLE instruction
    a.label("no_idle");
    a.b({0xee, 0x54, 0x07});
    a.rel({0x70}, "no_window");
    a.b({0xc2, 0xaf, 0x00, 0x00, 0x75, r2::IEN1, 0x2b, 0xd2, 0xaf});
    a.label("no_window");
    a.b({0x0e}); // INC R6
    a.abs(0x02, "loop");
    const auto low = a.finish();
    std::copy(low.begin(), low.end(), p.flash.begin());

    Asm bank3(0x8000); // physical 0x18000
    bank3.b({0xe5, 0x31, 0x23, 0xf5, 0x31, 0x7d, 0x03});
    bank3.label("spin");
    bank3.rel({0xdd}, "spin"); // DJNZ R5,$
    bank3.b({0xee, 0x93, 0xf5, 0x3c, 0x22});
    const auto b3 = bank3.finish();
    std::copy(b3.begin(), b3.end(), p.flash.begin() + 0x18000);

    Asm bank4(0x8000); // physical 0x20000
    bank4.b({0x05, 0x34, 0xe5, 0x34, 0xf5, 0x3a, 0x22});
    const auto b4 = bank4.finish();
    std::copy(b4.begin(), b4.end(), p.flash.begin() + 0x20000);
    return p;
}

// --------------------------------------------------------------- snapshot
struct Snapshot {
    Cc2530Registers regs;
    std::array<std::uint8_t, 128> sfr{};
    std::array<std::uint8_t, 256> idata{};
    std::vector<std::uint8_t> sram;
    std::vector<std::uint64_t> counters;
    std::map<std::uint16_t, std::uint64_t> xreg_reads, xreg_writes;
    bool sleeping{}, in_interrupt{};
    std::size_t queued{};
    std::uint64_t next_event{};
    std::string error;
};

Snapshot snapshot(const Cc2530F256 &d, const std::string &error) {
    Snapshot s;
    s.regs = d.registers();
    for (unsigned i = 0; i < 128; ++i)
        s.sfr[i] = d.read_sfr(static_cast<std::uint8_t>(0x80 + i));
    for (unsigned i = 0; i < 256; ++i)
        s.idata[i] = d.read_idata(static_cast<std::uint8_t>(i));
    s.sram.resize(Cc2530F256::SramSize);
    for (unsigned i = 0; i < Cc2530F256::SramSize; ++i)
        s.sram[i] = d.peek_xdata(static_cast<std::uint16_t>(i));
    const Cc2530Diagnostics &g = d.diagnostics();
    s.counters = {g.unmapped_xdata_reads,     g.unmapped_xdata_writes,
                  g.read_only_xdata_writes,   g.read_only_sfr_writes,
                  g.fmap_writes,              g.memctr_writes,
                  g.interrupts_taken,         g.test_injected_flags,
                  g.scheduled_flag_events,    g.stale_interrupt_posts,
                  g.power_mode_entries,       g.power_mode_wakes,
                  g.power_mode_immediate_wakes, g.idle_cycles,
                  g.idle_alignment_violations, g.ircon_bit6_writes,
                  g.peripheral_services,      g.cpu_stalls,
                  g.cpu_stall_cycles};
    for (unsigned i = 0; i < 5; ++i) {
        s.counters.push_back(g.power_mode_entries_by_mode[i]);
        s.counters.push_back(g.power_mode_cycles_by_mode[i]);
    }
    for (unsigned i = 0; i < 4; ++i)
        s.counters.push_back(g.resets_by_cause[i]);
    s.xreg_reads = g.unmodeled_xreg_reads;
    s.xreg_writes = g.unmodeled_xreg_writes;
    s.sleeping = d.sleeping();
    s.in_interrupt = d.in_interrupt();
    s.queued = d.scheduled_event_count();
    s.next_event = d.next_event_cycle().value_or(UINT64_MAX);
    s.error = error;
    return s;
}

const char *const CounterNames[] = {
    "unmapped_xdata_reads", "unmapped_xdata_writes", "read_only_xdata_writes",
    "read_only_sfr_writes", "fmap_writes", "memctr_writes", "interrupts_taken",
    "test_injected_flags", "scheduled_flag_events", "stale_interrupt_posts",
    "power_mode_entries", "power_mode_wakes", "power_mode_immediate_wakes", "idle_cycles",
    "idle_alignment_violations", "ircon_bit6_writes", "peripheral_services", "cpu_stalls",
    "cpu_stall_cycles", "entries[Active]", "cycles[Active]", "entries[Idle]", "cycles[Idle]",
    "entries[PM1]", "cycles[PM1]", "entries[PM2]", "cycles[PM2]", "entries[PM3]",
    "cycles[PM3]", "resets[PowerOn]", "resets[External]", "resets[Watchdog]",
    "resets[ClockLoss]"};

std::string hx(unsigned v) {
    std::ostringstream o;
    o << "0x" << std::hex << v;
    return o.str();
}

// First difference, or "" when equal.
std::string difference(const Snapshot &x, const Snapshot &y) {
    const Cc2530Registers &a = x.regs, &b = y.regs;
#define FIELD(f)                                                                               \
    if (a.f != b.f)                                                                            \
        return std::string(#f " ") + std::to_string(+a.f) + " vs " + std::to_string(+b.f);
    FIELD(pc) FIELD(a) FIELD(b) FIELD(psw) FIELD(sp) FIELD(dpl) FIELD(dph) FIELD(fmap)
    FIELD(memctr) FIELD(mpage) FIELD(cycles) FIELD(instructions) FIELD(dpl1) FIELD(dph1)
    FIELD(dps) FIELD(interrupt_levels) FIELD(interrupt_inhibit) FIELD(posted_vector)
    FIELD(posted_level) FIELD(posted_source)
#undef FIELD
    for (unsigned i = 0; i < 8; ++i)
        if (a.r[i] != b.r[i])
            return "R" + std::to_string(i);
    for (unsigned i = 0; i < 128; ++i)
        if (x.sfr[i] != y.sfr[i])
            return "SFR " + hx(0x80 + i) + " " + hx(x.sfr[i]) + " vs " + hx(y.sfr[i]);
    for (unsigned i = 0; i < 256; ++i)
        if (x.idata[i] != y.idata[i])
            return "IDATA " + hx(i) + " " + hx(x.idata[i]) + " vs " + hx(y.idata[i]);
    for (unsigned i = 0; i < x.sram.size(); ++i)
        if (x.sram[i] != y.sram[i])
            return "SRAM " + hx(i) + " " + hx(x.sram[i]) + " vs " + hx(y.sram[i]);
    for (std::size_t i = 0; i < x.counters.size(); ++i)
        if (x.counters[i] != y.counters[i])
            return std::string("diagnostic ") + CounterNames[i] + " " +
                   std::to_string(x.counters[i]) + " vs " + std::to_string(y.counters[i]);
    if (x.xreg_reads != y.xreg_reads || x.xreg_writes != y.xreg_writes)
        return "unmodeled XREG counters";
    if (x.sleeping != y.sleeping)
        return "sleeping";
    if (x.in_interrupt != y.in_interrupt)
        return "in_interrupt";
    if (x.queued != y.queued || x.next_event != y.next_event)
        return "event queue";
    if (x.error != y.error)
        return "error '" + x.error + "' vs '" + y.error + "'";
    return {};
}

// ------------------------------------------------------------------ modes
enum class Mode { Fast, Step, Trace };
const char *name(Mode m) { return m == Mode::Fast ? "fast" : m == Mode::Step ? "step" : "trace"; }

struct Event {
    std::uint64_t at;
    unsigned source;
    std::uint8_t mask;
};

struct Lane {
    Mode mode;
    std::unique_ptr<Cc2530F256> device = std::make_unique<Cc2530F256>();
    std::ostringstream trace;
    std::string error;
    // Trace mode: (interrupted PC, vector) of every "irq ->" line.
    std::vector<std::pair<std::uint16_t, std::uint16_t>> irq_lines;
};

void parse_trace(Lane &lane) {
    const std::string text = lane.trace.str();
    std::size_t pos = 0;
    while ((pos = text.find("irq -> 0x", pos)) != std::string::npos) {
        const std::size_t line = text.rfind("pc=0x", pos);
        lane.irq_lines.emplace_back(
            static_cast<std::uint16_t>(std::stoul(text.substr(line + 5, 4), nullptr, 16)),
            static_cast<std::uint16_t>(std::stoul(text.substr(pos + 9, 4), nullptr, 16)));
        pos += 9;
    }
    lane.trace.str("");
    lane.trace.clear();
}

// Advances `lane` to the first instruction boundary at or after `limit`.
void advance(Lane &lane, std::uint64_t limit) {
    Cc2530F256 &d = *lane.device;
    // run() always executes at least one instruction, so a sync point the
    // lane has already reached (the previous run overshot it) is a no-op.
    if (!lane.error.empty() || d.registers().cycles >= limit)
        return;
    if (lane.mode == Mode::Step) {
        d.set_trace(nullptr);
        for (std::size_t i = 0; i < 10'000'000; ++i) {
            const Cc2530RunResult r = d.run(1, limit);
            if (!r.error.empty()) {
                lane.error = r.error;
                return;
            }
            if (r.cycles >= limit || (r.instructions == 0 && r.sleeping))
                return;
        }
        lane.error = "step budget";
        return;
    }
    d.set_trace(lane.mode == Mode::Trace ? &lane.trace : nullptr);
    const Cc2530RunResult r = d.run(10'000'000, limit);
    if (!r.error.empty())
        lane.error = r.error;
    if (lane.mode == Mode::Trace)
        parse_trace(lane);
}

// One instruction (or one entry, or one idle advance).
void single(Lane &lane) {
    Cc2530F256 &d = *lane.device;
    if (!lane.error.empty())
        return;
    d.set_trace(lane.mode == Mode::Trace ? &lane.trace : nullptr);
    const Cc2530RunResult r = d.run(1);
    if (!r.error.empty())
        lane.error = r.error;
    if (lane.mode == Mode::Trace)
        parse_trace(lane);
}

struct Totals {
    std::uint64_t scenarios{}, runs{}, sync_points{}, comparisons{}, instructions{},
        interrupts{}, events{}, idle_entries{}, wakes{}, max_depth{};
} totals;

using Loader = std::function<void(Cc2530F256 &)>;

struct Scenario {
    std::string name;
    Loader load;
    std::vector<Event> events;
    std::uint64_t horizon;
    bool log_check; // the program keeps the SRAM interrupt log
};

void prepare(Lane &lane, const Scenario &s) {
    s.load(*lane.device);
    for (const Event &e : s.events)
        lane.device->schedule_interrupt_flag_for_test(e.at, e.source, e.mask);
}

std::uint64_t popcount(std::uint8_t v) {
    std::uint64_t n = 0;
    for (; v; v &= v - 1)
        ++n;
    return n;
}

// Trace "irq ->" lines, the interrupts_taken counter and the firmware's
// own SRAM log describe the same entries: every logged entry has a trace
// line with the same interrupted PC and vector; entries not yet logged are
// ISRs still in their prologue (at most one per active level).
void check_event_sequence(const Lane &trace, const Snapshot &s, const Scenario &sc,
                          const std::string &ctx) {
    const std::uint64_t taken = s.counters[6];
    CHECK_IN(trace.irq_lines.size() == taken,
             ctx + " trace irq lines " + std::to_string(trace.irq_lines.size()) +
                 " taken " + std::to_string(taken));
    if (!sc.log_check)
        return;
    const unsigned r0 = s.idata[0x08], r7 = s.idata[0x0f];
    const std::size_t logged = ((r7 - (LogBase >> 8)) * 256u + r0) / LogEntry;
    CHECK_IN(logged <= taken && taken - logged <= popcount(s.regs.interrupt_levels),
             ctx + " logged " + std::to_string(logged) + " taken " + std::to_string(taken));
    CHECK_IN(LogBase + LogEntry * logged <= s.sram.size(),
             ctx + " log overflows SRAM: " + std::to_string(logged) + " entries");
    if (LogBase + LogEntry * logged > s.sram.size())
        return;
    std::multiset<std::pair<std::uint16_t, std::uint16_t>> lines(trace.irq_lines.begin(),
                                                                 trace.irq_lines.end());
    for (std::size_t i = 0; i < logged; ++i) {
        const std::size_t o = LogBase + LogEntry * i;
        const std::uint8_t marker = s.sram[o];
        const auto ret = static_cast<std::uint16_t>((s.sram[o + 2] << 8) | s.sram[o + 3]);
        const auto vector = static_cast<std::uint16_t>(r2::vector_of(marker & 0x7fu));
        const auto it = lines.find({ret, vector});
        CHECK_IN((marker & 0x80u) && it != lines.end(),
                 ctx + " log entry " + std::to_string(i) + " has no trace line");
        if (it != lines.end())
            lines.erase(it);
    }
}

// Runs the scenario on three lanes with the given sync schedule and returns
// the final (fast) snapshot.
Snapshot run_schedule(const Scenario &sc, const std::vector<std::uint64_t> &syncs,
                      const std::string &schedule) {
    Lane lanes[3] = {{Mode::Fast}, {Mode::Step}, {Mode::Trace}};
    for (Lane &lane : lanes)
        prepare(lane, sc);
    Snapshot last;
    for (const std::uint64_t limit : syncs) {
        for (Lane &lane : lanes) {
            advance(lane, limit);
            // Wake rule: a posted request wakes the core at the boundary it
            // is posted, so no stop may show a sleeping core with one.
            CHECK_IN(!(lane.device->sleeping() && lane.device->registers().posted_vector != 0),
                     sc.name + " [" + schedule + "] " + name(lane.mode) +
                         " asleep with a posted request at sync " + std::to_string(limit));
        }
        const Snapshot f = snapshot(*lanes[0].device, lanes[0].error);
        for (int m = 1; m < 3; ++m) {
            const std::string diff = difference(f, snapshot(*lanes[m].device, lanes[m].error));
            ++totals.comparisons;
            if (!diff.empty()) {
                std::cerr << sc.name << " [" << schedule << "] fast vs " << name(lanes[m].mode)
                          << " at sync " << limit << ": " << diff << '\n';
                ++r2::failures;
                return f;
            }
        }
        ++totals.sync_points;
        totals.max_depth = std::max(totals.max_depth, popcount(f.regs.interrupt_levels));
        last = f;
    }
    CHECK_IN(lanes[0].error.empty(), sc.name + " [" + schedule + "] " + lanes[0].error);
    CHECK_IN(last.counters[9] == 0, sc.name + " [" + schedule + "] stale interrupt posts");
    check_event_sequence(lanes[2], last, sc, sc.name + " [" + schedule + "]");
    totals.runs += 3;
    return last;
}

// Step vs trace at every instruction boundary.
void run_per_instruction(const Scenario &sc) {
    Lane lanes[2] = {{Mode::Step}, {Mode::Trace}};
    for (Lane &lane : lanes)
        prepare(lane, sc);
    while (lanes[0].device->registers().cycles < sc.horizon && lanes[0].error.empty()) {
        const bool asleep_for_good = lanes[0].device->sleeping() &&
                                     lanes[0].device->scheduled_event_count() == 0;
        if (asleep_for_good)
            break;
        for (Lane &lane : lanes)
            single(lane);
        const std::string diff = difference(snapshot(*lanes[0].device, lanes[0].error),
                                            snapshot(*lanes[1].device, lanes[1].error));
        ++totals.comparisons;
        if (!diff.empty()) {
            std::cerr << sc.name << " [per-instruction] step vs trace at cycle "
                      << lanes[0].device->registers().cycles << ": " << diff << '\n';
            ++r2::failures;
            return;
        }
        ++totals.sync_points;
    }
    totals.runs += 2;
}

std::uint64_t splitmix(std::uint64_t &state) {
    std::uint64_t z = (state += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

std::vector<std::uint64_t> schedule(std::uint64_t seed, std::uint64_t lo, std::uint64_t hi,
                                    std::uint64_t horizon) {
    std::vector<std::uint64_t> out;
    std::uint64_t t = 0;
    while (t < horizon) {
        t = std::min(horizon, t + lo + splitmix(seed) % (hi - lo + 1));
        out.push_back(t);
    }
    return out;
}

// Syncs one cycle before, at and after every event: stops that end exactly
// at, or one instruction around, the boundary where an event becomes due.
std::vector<std::uint64_t> event_edges(const Scenario &sc) {
    std::set<std::uint64_t> at;
    for (const Event &e : sc.events)
        for (const std::uint64_t t : {e.at - (e.at != 0), e.at, e.at + 1})
            if (t != 0 && t < sc.horizon)
                at.insert(t);
    std::vector<std::uint64_t> out(at.begin(), at.end());
    out.push_back(sc.horizon);
    return out;
}

void run_scenario(const Scenario &sc, std::uint64_t seed, bool per_instruction) {
    ++totals.scenarios;
    const Snapshot final_stop = run_schedule(sc, {sc.horizon}, "final");
    const Snapshot dense = run_schedule(sc, schedule(seed, 1, 24, sc.horizon), "dense");
    const Snapshot sparse = run_schedule(sc, schedule(seed + 1, 100, 3000, sc.horizon), "sparse");
    // (No events, as in firmware images: identical to the final stop.)
    const Snapshot edges =
        sc.events.empty() ? final_stop : run_schedule(sc, event_edges(sc), "event edges");
    // The final state must not depend on where the run was stopped.
    const std::string d1 = difference(final_stop, dense), d2 = difference(final_stop, sparse),
                      d3 = difference(final_stop, edges);
    CHECK_IN(d1.empty(), sc.name + " final vs dense schedule: " + d1);
    CHECK_IN(d2.empty(), sc.name + " final vs sparse schedule: " + d2);
    CHECK_IN(d3.empty(), sc.name + " final vs event-edge schedule: " + d3);
    if (per_instruction)
        run_per_instruction(sc);
    totals.instructions += final_stop.regs.instructions;
    totals.interrupts += final_stop.counters[6];
    totals.events += final_stop.counters[8];
    totals.idle_entries += final_stop.counters[10];
    totals.wakes += final_stop.counters[11];
}

// Event sources: the six enabled ones (ENC/RF with each flag bit alone and
// both) and two that are never enabled.
Event random_event(std::uint64_t &seed, std::uint64_t at) {
    static const std::pair<unsigned, std::uint8_t> kinds[] = {
        {SrcEnc, 0x01}, {SrcEnc, 0x02}, {SrcEnc, 0x03}, {SrcRf, 0x01},  {SrcRf, 0x02},
        {SrcRf, 0x03},  {SrcDma, 0x01}, {SrcT1, 0x02},  {SrcT3, 0x08},  {SrcP0, 0x20},
        {SrcUrx0, 0x08}, {SrcSt, 0x80}};
    const auto &k = kinds[splitmix(seed) % std::size(kinds)];
    return {at, k.first, k.second};
}

// ------------------------------------------------------- targeted events
enum Target {
    AtPcon, AtReti, AfterReti, AtEntry, AtFmapWrite, AtBankCall, InBank, AtBankRet, InMulDiv,
    WhileIdle, InIsr, TargetCount
};
const char *const TargetNames[] = {"PCON.IDLE write", "RETI", "post-RETI instruction",
                                   "interrupt entry", "FMAP write", "banked LCALL",
                                   "banked code", "banked RET", "inside MUL/DIV",
                                   "while idle", "inside an ISR"};

// Reference step run of the program with `base` events: candidate times for
// every target class (boundary cycles and, for multi-cycle work, the cycles
// inside it).
std::map<Target, std::vector<std::uint64_t>> find_targets(const Scenario &base) {
    std::map<Target, std::vector<std::uint64_t>> found;
    Cc2530F256 d;
    base.load(d);
    for (const Event &e : base.events)
        d.schedule_interrupt_flag_for_test(e.at, e.source, e.mask);
    bool after_reti = false;
    while (d.registers().cycles < base.horizon) {
        const Cc2530Registers before = d.registers();
        const std::uint16_t pc = before.pc;
        const std::uint8_t op = d.read_code(pc);
        const bool asleep = d.sleeping();
        const std::uint64_t taken = d.diagnostics().interrupts_taken;
        if (asleep && d.scheduled_event_count() == 0)
            break;
        const Cc2530RunResult r = d.run(1);
        if (!r.error.empty())
            break;
        const std::uint64_t c = before.cycles;
        auto add = [&](Target t, std::uint64_t at) { found[t].push_back(at); };
        if (asleep) {
            add(WhileIdle, c + 1);
            add(WhileIdle, (c + r.cycles) / 2);
            continue;
        }
        if (d.diagnostics().interrupts_taken != taken) {
            add(AtEntry, c);
            add(AtEntry, c + 1);
            after_reti = false;
            continue;
        }
        if (after_reti)
            add(AfterReti, c), add(AfterReti, c + 1);
        after_reti = op == 0x32;
        if (op == 0x75 && d.read_code(static_cast<std::uint16_t>(pc + 1)) == r2::PCON)
            for (unsigned k = 0; k < 4; ++k)
                add(AtPcon, c + k);
        if (op == 0x32)
            add(AtReti, c), add(AtReti, c + 1);
        if (op == 0x75 && d.read_code(static_cast<std::uint16_t>(pc + 1)) == r2::FMAP)
            add(AtFmapWrite, c), add(AtFmapWrite, c + 1);
        if (op == 0x12 && d.read_code(static_cast<std::uint16_t>(pc + 1)) >= 0x80)
            add(AtBankCall, c), add(AtBankCall, c + 1);
        if (pc >= 0x8000)
            add(op == 0x22 ? AtBankRet : InBank, c), add(op == 0x22 ? AtBankRet : InBank, c + 1);
        if (op == 0xa4 || op == 0x84)
            for (unsigned k = 1; k < 4; ++k)
                add(InMulDiv, c + k);
        if (before.interrupt_levels != 0)
            add(InIsr, c);
    }
    return found;
}

const char *source_name(unsigned source) {
    static const char *const names[] = {"RFERR", "ADC",  "URX0", "URX1", "ENC",   "ST",
                                        "P2INT", "UTX0", "DMA",  "T1",   "T2",    "T3",
                                        "T4",    "P0INT", "UTX1", "P1INT", "RF",  "WDT"};
    return names[source];
}

// Classifies, in a step run, the state in which each event was applied, so
// coverage is measured on the run itself rather than assumed. Events apply
// in (time, scheduling order): the k-th applied event is the k-th of the
// stably sorted list. A single step (run(1)) applies the events that are due
// at its closing boundary before returning, so an event at time `at` must be
// applied by the call whose interval (before, after] contains `at`; this is
// asserted. An event exactly at a boundary is classified by the instruction
// that follows that boundary (also when an interrupt entry is dispatched
// first: the entry step's PC/opcode is the deferred instruction), otherwise
// by the instruction or idle period in progress. "Interrupt entry" is counted
// only for an event that arrives during, or at the boundary before, the entry
// of another source's interrupt.
struct StepRecord {
    std::uint64_t before = 0, after = 0;
    std::uint16_t pc = 0, vector = 0;
    std::uint8_t op = 0, operand = 0, levels = 0;
    bool entry = false, asleep = false;
    std::uint64_t first_applied = 0, last_applied = 0; // [first, last)
};

std::set<Target> classify_applied(const Scenario &sc) {
    std::set<Target> hit;
    std::vector<Event> sorted = sc.events;
    std::stable_sort(sorted.begin(), sorted.end(),
                     [](const Event &a, const Event &b) { return a.at < b.at; });
    Cc2530F256 d;
    sc.load(d);
    for (const Event &e : sc.events)
        d.schedule_interrupt_flag_for_test(e.at, e.source, e.mask);
    std::vector<StepRecord> steps;
    while (d.registers().cycles < sc.horizon) {
        StepRecord s;
        const Cc2530Registers before = d.registers();
        s.asleep = d.sleeping();
        if (s.asleep && d.scheduled_event_count() == 0)
            break;
        s.before = before.cycles;
        s.pc = before.pc;
        s.op = d.read_code(s.pc);
        s.operand = d.read_code(static_cast<std::uint16_t>(s.pc + 1));
        s.levels = before.interrupt_levels;
        s.first_applied = d.diagnostics().scheduled_flag_events;
        const std::uint64_t taken = d.diagnostics().interrupts_taken;
        const Cc2530RunResult r = d.run(1);
        if (!r.error.empty())
            break;
        s.after = r.cycles;
        s.entry = d.diagnostics().interrupts_taken != taken;
        s.vector = s.entry ? r.pc : 0;
        s.last_applied = d.diagnostics().scheduled_flag_events;
        steps.push_back(s);
    }
    auto classify_instruction = [&](const StepRecord &n) {
        if (n.op == 0x32)
            hit.insert(AtReti);
        if (n.op == 0x75 && n.operand == r2::PCON)
            hit.insert(AtPcon);
        if (n.op == 0x75 && n.operand == r2::FMAP)
            hit.insert(AtFmapWrite);
        if (n.op == 0x12 && n.operand >= 0x80)
            hit.insert(AtBankCall);
        if (n.pc >= 0x8000)
            hit.insert(n.op == 0x22 ? AtBankRet : InBank);
        if (n.levels != 0)
            hit.insert(InIsr);
    };
    auto at_boundary_before = [&](const StepRecord &n, const StepRecord *previous,
                                  const Event &e) {
        if (n.asleep)
            return;
        if (n.entry && n.vector != r2::vector_of(e.source))
            hit.insert(AtEntry);
        if (previous && previous->op == 0x32 && !previous->entry && !previous->asleep)
            hit.insert(AfterReti);
        classify_instruction(n);
    };
    for (std::size_t i = 0; i < steps.size(); ++i) {
        const StepRecord &s = steps[i];
        for (std::uint64_t k = s.first_applied; k < s.last_applied && k < sorted.size(); ++k) {
            const std::uint64_t at = sorted[k].at;
            CHECK_IN(at > s.before || (i == 0 && at <= s.before),
                     sc.name + ": event at " + std::to_string(at) +
                         " applied late, in the step starting at " + std::to_string(s.before));
            CHECK_IN(at <= s.after, sc.name + ": event at " + std::to_string(at) +
                                        " applied early, in the step ending at " +
                                        std::to_string(s.after));
            if (s.asleep) {
                hit.insert(WhileIdle);
                continue;
            }
            if (at == s.after) {
                if (i + 1 < steps.size())
                    at_boundary_before(steps[i + 1], &s, sorted[k]);
                continue;
            }
            // Strictly inside the step's instruction (or interrupt entry).
            if (s.entry) {
                hit.insert(AtEntry);
                continue;
            }
            if (s.op == 0xa4 || s.op == 0x84)
                hit.insert(InMulDiv);
            classify_instruction(s);
        }
    }
    return hit;
}

int run_synthetic() {
    const Program program = mixed_program();
    const Loader load = [&program](Cc2530F256 &d) { d.load_flash(0, program.flash); };

    // 1. A run with no events: the program idles at the first PCON write
    //    (no request can wake it); all lanes must agree on that.
    run_scenario({"no events", load, {}, 4000, true}, 1, true);

    // 2. Pseudo-random event sets of increasing density.
    for (unsigned i = 0; i < 6; ++i) {
        std::uint64_t seed = 0x5eed0000ull + i;
        Scenario sc{"random-" + std::to_string(i), load, {}, 30000, true};
        const unsigned count = 40 + 40 * i;
        for (unsigned e = 0; e < count; ++e)
            sc.events.push_back(random_event(seed, splitmix(seed) % sc.horizon));
        // Bursts: several events at the same cycle.
        for (unsigned e = 0; e < 4; ++e) {
            const std::uint64_t at = splitmix(seed) % sc.horizon;
            for (unsigned j = 0; j < 3; ++j)
                sc.events.push_back(random_event(seed, at));
        }
        run_scenario(sc, seed, i < 2);
    }

    // 3. Targeted events: one extra event at an exact boundary (or inside a
    //    multi-cycle instruction or idle period) found in a reference run of
    //    the base scenario, with a level-3 (DMA) and a level-0 (T1) source.
    std::uint64_t seed = 0x7a26e7ull;
    Scenario base{"base", load, {}, 12000, true};
    for (unsigned e = 0; e < 30; ++e)
        base.events.push_back(random_event(seed, 200 + splitmix(seed) % 11000));
    const auto targets = find_targets(base);
    std::set<Target> covered;
    for (unsigned t = 0; t < TargetCount; ++t) {
        const auto it = targets.find(static_cast<Target>(t));
        CHECK_IN(it != targets.end() && !it->second.empty(),
                 std::string("reference run has no ") + TargetNames[t]);
        if (it == targets.end())
            continue;
        const auto &times = it->second;
        const std::size_t picks[] = {0, times.size() / 2, times.size() - 1};
        for (const std::size_t pick : picks) {
            for (const auto &[source, mask] :
                 {std::pair<unsigned, std::uint8_t>{SrcDma, 0x01}, {SrcT1, 0x02}}) {
                Scenario sc = base;
                sc.name = std::string("target ") + TargetNames[t] + " @" +
                          std::to_string(times[pick]) + " " + source_name(source);
                sc.events.push_back({times[pick], source, mask});
                const auto hit = classify_applied(sc);
                covered.insert(hit.begin(), hit.end());
                run_scenario(sc, seed + t * 7 + pick, false);
            }
        }
    }
    for (unsigned t = 0; t < TargetCount; ++t)
        CHECK_IN(covered.count(static_cast<Target>(t)),
                 std::string("no event was applied at: ") + TargetNames[t]);
    std::cout << "event classes applied:";
    for (const Target t : covered)
        std::cout << " [" << TargetNames[t] << "]";
    std::cout << '\n';
    return 0;
}

int run_image(const std::string &path, std::uint64_t horizon) {
    const Loader load = [path](Cc2530F256 &d) { d.load_firmware(path); };
    Scenario sc{"image " + path, load, {}, horizon, false};
    run_scenario(sc, 0x1a6eull, false);
    return 0;
}

// ------------------------------------------------------------ image audit
// --audit: per-step checks of a firmware image that do not depend on its
// expected results (those are in fw/tools/e4fw.py):
//  * timing contract (cc2530_r2_support.hpp: contract_cycles, the SWRU191F
//    Table 2-3 counts; interrupt entry = EntryCycles (7), 1 instruction, SP + 2);
//  * exact peak SP over every instruction/entry boundary, the spare bytes
//    below the guard and the guard bytes at the end (--guard ADDR:VALUE,
//    --min-headroom N);
//  * reset with interrupt levels in progress (R1 P4, R19/R20): at the deepest
//    nesting point and at a post-RETI-inhibit point inside an ISR, in fast and
//    step mode: the state before reset must agree, reset() must restore the
//    power-on interrupt state, the run after reset must reach the final
//    self-loop after exactly the cold run's instructions and cycles, ST0 must
//    equal the 32 kHz edge count since the reset, and all other state must
//    equal a cold run except SLEEPSTA.RST and the cumulative diagnostics.
struct AuditOptions {
    bool enabled = false;
    unsigned guard = 0x100; // first guard address (0x100: none)
    std::uint8_t guard_value = 0;
    unsigned min_headroom = 0;
};

constexpr std::uint8_t SleepstaSfr = 0x9d;
constexpr std::uint8_t St0Sfr = 0x95;

// Elapsed cycles/instructions and ST0 are checked separately (see below).
Snapshot normalized(Snapshot s) {
    s.regs.cycles = s.regs.instructions = 0;
    s.sfr[SleepstaSfr - 0x80] = 0;
    s.sfr[St0Sfr - 0x80] = 0;
    std::fill(s.counters.begin(), s.counters.end(), 0);
    s.xreg_reads.clear();
    s.xreg_writes.clear();
    return s;
}

// ST0 (SWRU191F 11.1: the live low byte of the 24-bit sleep timer count) at
// cycle `now` for a count restarted at 0 by a reset at cycle `reset`: the
// positive 32.768 kHz edges in (reset, now], with one cycle = 1/32 MHz.
// Model rule (SleepTimer::reset): a reset restarts the count, not the 32 kHz
// clock phase. Valid only without PM3, which stops the 32 kHz count base.
std::uint8_t expected_st0(std::uint64_t reset, std::uint64_t now) {
    const auto edges = [](std::uint64_t cycle) { return cycle * 32768u / 32000000u; };
    return static_cast<std::uint8_t>(edges(now) - edges(reset));
}

// Runs `d` after reset() for the cold run's instruction count, in fast mode
// (one run() per remaining count) or step mode (run(1), stopping at the
// final self-loop like the cold run). Stopping there exactly keeps the
// time-derived state comparable with the cold run.
void run_like_cold(Cc2530F256 &d, bool step, std::uint64_t instructions, std::uint64_t horizon,
                   const std::string &ctx) {
    const std::uint64_t target = d.registers().instructions + instructions;
    while (d.registers().instructions < target && d.registers().cycles < horizon &&
           !(step && d.idle_forever())) {
        const Cc2530RunResult r = d.run(step ? 1 : target - d.registers().instructions, horizon);
        if (!r.error.empty()) {
            CHECK_IN(false, ctx + ": " + r.error);
            return;
        }
    }
    CHECK_IN(d.idle_forever(), ctx + ": not in the final self-loop after the cold run's instructions");
}

void audit_image(const std::string &path, std::uint64_t horizon, const AuditOptions &opt) {
    Cc2530F256 d;
    d.load_firmware(path);
    const Cc2530Registers start = d.registers();
    unsigned peak = d.registers().sp, max_depth = 0;
    std::uint64_t steps = 0, entries = 0, idle_steps = 0, timing_bad = 0;
    std::uint64_t deepest_at = 0, inhibit_at = 0;
    std::uint8_t deepest_levels = 0, inhibit_levels = 0;
    while (!d.idle_forever() && d.registers().cycles < horizon) {
        const Cc2530Registers before = d.registers();
        const bool asleep = d.sleeping();
        const std::uint8_t op = d.read_code(before.pc);
        const std::uint64_t taken = d.diagnostics().interrupts_taken;
        // The cycle limit bounds an idle step: without it, a core asleep with
        // nothing able to wake it returns without advancing and the loop spins.
        const Cc2530RunResult r = d.run(1, horizon);
        if (!r.error.empty()) {
            CHECK_IN(false, path + ": " + r.error);
            break;
        }
        const Cc2530Registers after = d.registers();
        ++steps;
        if (asleep) {
            ++idle_steps;
            continue;
        }
        const bool entry = d.diagnostics().interrupts_taken != taken;
        const std::uint64_t dc = after.cycles - before.cycles;
        const std::uint64_t di = after.instructions - before.instructions;
        bool ok;
        if (entry) {
            ++entries;
            ok = dc == r2::EntryCycles && di == 1 &&
                 after.sp == static_cast<std::uint8_t>(before.sp + 2);
        } else {
            ok = dc == r2::contract_cycles(op) && di == 1;
        }
        if (!ok && timing_bad++ < 5)
            std::cerr << path << ": timing contract at pc " << hx(before.pc) << " op "
                      << hx(op) << (entry ? " (entry)" : "") << ": " << dc << " cycles, " << di
                      << " instructions\n";
        peak = std::max<unsigned>(peak, after.sp);
        const unsigned depth = static_cast<unsigned>(popcount(after.interrupt_levels));
        if (depth > max_depth) {
            max_depth = depth;
            deepest_at = after.instructions;
            deepest_levels = after.interrupt_levels;
        }
        if (!inhibit_at && after.interrupt_inhibit && after.interrupt_levels) {
            inhibit_at = after.instructions;
            inhibit_levels = after.interrupt_levels;
        }
    }
    CHECK_IN(d.idle_forever(), path + ": step run did not reach the final self-loop");
    CHECK_IN(timing_bad == 0, path + ": " + std::to_string(timing_bad) + " timing-contract violations");
    const int headroom = int(opt.guard) - 1 - int(peak);
    bool guard_ok = true;
    for (unsigned a = opt.guard; a < 0x100; ++a)
        guard_ok = guard_ok && d.read_idata(static_cast<std::uint8_t>(a)) == opt.guard_value;
    CHECK_IN(guard_ok, path + ": stack guard modified");
    CHECK_IN(headroom >= int(opt.min_headroom),
             path + ": stack headroom " + std::to_string(headroom) + " < " +
                 std::to_string(opt.min_headroom));
    std::cout << "audit " << path << ": steps=" << steps << " entries=" << entries
              << " idle_steps=" << idle_steps << " timing_violations=" << timing_bad
              << " cycles=" << d.registers().cycles << " peak_sp=" << hx(peak)
              << " guard=" << hx(opt.guard) << (guard_ok ? " intact" : " MODIFIED")
              << " headroom=" << headroom << " max_depth=" << max_depth << '\n';
    constexpr unsigned Pm3 = static_cast<unsigned>(mcs51emu::Cc2530PowerMode::Pm3);
    const Cc2530Registers end = d.registers();
    const std::uint64_t cold_cycles = end.cycles - start.cycles;
    const std::uint64_t cold_instructions = end.instructions - start.instructions;
    const Snapshot cold_raw = snapshot(d, "");
    CHECK_IN(d.diagnostics().power_mode_entries_by_mode[Pm3] == 0,
             path + ": PM3 entered; the ST0 expectation assumes a running 32 kHz count");
    CHECK_IN(cold_raw.sfr[St0Sfr - 0x80] == expected_st0(start.cycles, end.cycles),
             path + ": cold ST0 " + hx(cold_raw.sfr[St0Sfr - 0x80]) + " vs expected " +
                 hx(expected_st0(start.cycles, end.cycles)));
    const Snapshot cold = normalized(cold_raw);

    CHECK_IN(deepest_at && deepest_levels, path + ": no interrupt level in progress observed");
    CHECK_IN(inhibit_at, path + ": no post-RETI inhibit inside an ISR observed");
    const std::pair<const char *, std::uint64_t> points[] = {{"deepest nesting", deepest_at},
                                                             {"post-RETI inhibit", inhibit_at}};
    for (const auto &[what, at] : points) {
        if (!at)
            continue;
        Snapshot pre[2];
        for (int m = 0; m < 2; ++m) {
            const std::string ctx = path + " reset at " + what + " (instruction " +
                                    std::to_string(at) + ", " + (m ? "step" : "fast") + ")";
            Cc2530F256 x;
            x.load_firmware(path);
            // Advance to `at` instructions (idle advances do not count).
            for (std::size_t guard = 0; x.registers().instructions < at && guard < 10'000'000;
                 ++guard) {
                const std::uint64_t left = at - x.registers().instructions;
                const Cc2530RunResult r = x.run(m == 0 ? left : 1);
                if (!r.error.empty()) {
                    CHECK_IN(false, ctx + ": " + r.error);
                    break;
                }
            }
            pre[m] = snapshot(x, "");
            const Cc2530Registers p = x.registers();
            CHECK_IN(p.instructions == at, ctx + ": stopped at instruction " +
                                               std::to_string(p.instructions));
            if (std::string(what) == "deepest nesting")
                CHECK_IN(p.interrupt_levels == deepest_levels, ctx + ": levels " +
                                                                   hx(p.interrupt_levels));
            else
                CHECK_IN(p.interrupt_inhibit && p.interrupt_levels == inhibit_levels,
                         ctx + ": inhibit/levels not in progress");
            x.reset();
            const Cc2530Registers q = x.registers();
            CHECK_IN(q.pc == 0 && q.sp == 0x07 && q.interrupt_levels == 0 &&
                         !q.interrupt_inhibit && q.posted_vector == 0 && q.posted_level == 0 &&
                         q.posted_source == -1 && !x.sleeping() && !x.in_interrupt() &&
                         x.scheduled_event_count() == 0 && q.fmap == 1 && q.dps == 0,
                     ctx + ": state after reset() is not the power-on interrupt state");
            run_like_cold(x, m == 1, cold_instructions, horizon + q.cycles, ctx);
            const Cc2530Registers e = x.registers();
            CHECK_IN(e.instructions - q.instructions == cold_instructions &&
                         e.cycles - q.cycles == cold_cycles,
                     ctx + ": " + std::to_string(e.instructions - q.instructions) +
                         " instructions, " + std::to_string(e.cycles - q.cycles) +
                         " cycles after reset vs cold " + std::to_string(cold_instructions) +
                         ", " + std::to_string(cold_cycles));
            const Snapshot raw = snapshot(x, "");
            CHECK_IN(x.diagnostics().power_mode_entries_by_mode[Pm3] == 0,
                     ctx + ": PM3 entered; the ST0 expectation assumes a running 32 kHz count");
            CHECK_IN(raw.sfr[St0Sfr - 0x80] == expected_st0(q.cycles, e.cycles),
                     ctx + ": ST0 " + hx(raw.sfr[St0Sfr - 0x80]) + " vs expected " +
                         hx(expected_st0(q.cycles, e.cycles)));
            const std::string diff = difference(cold, normalized(raw));
            CHECK_IN(diff.empty(), ctx + ": completed run differs from a cold run: " + diff);
        }
        const std::string diff = difference(pre[0], pre[1]);
        CHECK_IN(diff.empty(), path + " reset point " + what + ": fast vs step: " + diff);
        std::cout << "audit " << path << ": reset at " << what << " (instruction " << at
                  << ", levels " << hx(std::string(what) == "deepest nesting" ? deepest_levels
                                                                               : inhibit_levels)
                  << ") fast+step: power-on interrupt state, completed run == cold run\n";
    }
}

} // namespace

int main(int argc, char **argv) {
    std::vector<std::string> images;
    std::uint64_t horizon = 250000;
    AuditOptions audit;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--image" && i + 1 < argc)
            images.push_back(argv[++i]);
        else if (arg == "--horizon" && i + 1 < argc)
            horizon = std::strtoull(argv[++i], nullptr, 0);
        else if (arg == "--audit")
            audit.enabled = true;
        else if (arg == "--guard" && i + 1 < argc) {
            const std::string g = argv[++i];
            const std::size_t colon = g.find(':');
            audit.guard = static_cast<unsigned>(std::stoul(g.substr(0, colon), nullptr, 0));
            audit.guard_value = colon == std::string::npos
                                    ? 0
                                    : static_cast<std::uint8_t>(
                                          std::stoul(g.substr(colon + 1), nullptr, 0));
        } else if (arg == "--min-headroom" && i + 1 < argc)
            audit.min_headroom = static_cast<unsigned>(std::strtoul(argv[++i], nullptr, 0));
        else {
            std::cerr << "usage: " << argv[0]
                      << " [--image FILE.hex]... [--horizon CYCLES] [--audit [--guard ADDR:VALUE]"
                         " [--min-headroom N]]\n";
            return 2;
        }
    }
    try {
        if (images.empty())
            run_synthetic();
        for (const std::string &image : images) {
            run_image(image, horizon);
            if (audit.enabled)
                audit_image(image, horizon, audit);
        }
    } catch (const std::exception &e) {
        std::cerr << "exception: " << e.what() << '\n';
        ++r2::failures;
    }
    std::cout << "scenarios=" << totals.scenarios << " lane_runs=" << totals.runs
              << " sync_points=" << totals.sync_points << " comparisons=" << totals.comparisons
              << " instructions(final)=" << totals.instructions
              << " interrupts=" << totals.interrupts << " events=" << totals.events
              << " idle_entries=" << totals.idle_entries << " wakes=" << totals.wakes
              << " max_nesting=" << totals.max_depth << '\n';
    if (r2::failures) {
        std::cerr << r2::failures << " check(s) failed\n";
        return 1;
    }
    std::cout << "fast/step/trace lockstep: all states equal\n";
    return 0;
}
