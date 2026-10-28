// CC2530 I/O controller (SWRU191F Chapter 7, register descriptions 7.11).
//
// Pads P0_0-P0_7, P1_0-P1_7, P2_0-P2_4. Each pad has a drive (what the chip
// does to it) computed from the registers in this priority order:
//   1. PMUX.CKOEN on P0[CKOPIN]: the 32 kHz clock ("overrides all other
//      configurations for the selected pin", PMUX).
//   2. PMUX.DREGSTA on P1[DREGSTAPIN]: 1 while the digital regulator is on
//      (active, idle, PM1), 0 in PM2/PM3 (Table 4-1).
//   3. OBSSELx.EN on P1_x (x = 0-5): the radio observation outputs
//      ("overwrites the standard GPIO behavior"); the observed radio signals
//      are not modeled (note_unmodeled), Model-defined: the pad drives low.
//   4. P2_3/P2_4 belong to the 32 kHz XOSC (analog, 3-state) while
//      CLKCONCMD.OSC32K = 0 (7.8).
//   5. APCFG[n] = 1 makes P0_n an analog pad: 3-state, no pull (7.6.6).
//   6. PxSEL = 1 (P2SEL.SELP2_0/3/4 for P2): the peripheral mapped to the pad
//      by PERCFG (Table 7-1) with the highest priority (P2DIR.PRIP0,
//      P2SEL.PRIxP1, 7.6.1-7.6.5) drives it if that function is an output;
//      inputs and pads without a mapped function are 3-state ("PxSEL = 1
//      overrides the pullup or pulldown setting", 7.6).
//   7. GPIO (7.3): PxDIR = 1 drives the latch; inputs get the pull of
//      PxINP/P2INP (MDPx = 1: 3-state; P2INP.PDUPx selects pullup or
//      pulldown per port). P1_0 and P1_1 have no pull (P1INP bits 1:0 are
//      reserved).
// The level of a pad is the host's external level, else the chip's own
// output or pull; a floating pad reads 1 (public header contract).
//
// Port interrupts (7.4, 2.5.1, Figure 2-4): an edge selected by PICTL on a
// pad level (outputs and peripheral pins included: "Even I/O pins configured
// as peripheral I/O or general-purpose outputs have interrupts generated")
// sets PxIFG whatever PxIEN is. The CPU flags IRCON.P0IF / IRCON2.P1IF /
// IRCON2.P2IF follow PxIFG & PxIEN as a level (no edge detect between PxIFG
// and PxINT): they are set while that is nonzero, also right after firmware
// clears the CPU flag (cpu_flags_written), which is why PxIFG must be cleared
// first. The P0_5 detector follows the analog comparator output while
// CMPCTL.EN = 1 (19.1, Figure 19-1).

#include "cc2530_io.hpp"

#include "cc2530_analog.hpp"
#include "cc2530_timers.hpp"
#include "cc2530_usart.hpp"

#include <utility>

namespace mcs51emu::cc2530 {
namespace {

constexpr std::uint8_t SfrP0 = 0x80, SfrP1 = 0x90, SfrP2 = 0xa0;
constexpr std::uint8_t SfrP0Ifg = 0x89, SfrP1Ifg = 0x8a, SfrP2Ifg = 0x8b, SfrPictl = 0x8c,
                       SfrP1Ien = 0x8d, SfrP0Inp = 0x8f, SfrP0Ien = 0xab, SfrP2Ien = 0xac,
                       SfrPmux = 0xae, SfrClkconcmd = 0xc6, SfrPercfg = 0xf1, SfrApcfg = 0xf2,
                       SfrP0Sel = 0xf3, SfrP1Sel = 0xf4, SfrP2Sel = 0xf5, SfrP1Inp = 0xf6,
                       SfrP2Inp = 0xf7, SfrP0Dir = 0xfd, SfrP1Dir = 0xfe, SfrP2Dir = 0xff;
constexpr std::uint8_t SfrPort[3] = {SfrP0, SfrP1, SfrP2};
constexpr std::uint8_t SfrIfg[3] = {SfrP0Ifg, SfrP1Ifg, SfrP2Ifg};
constexpr std::uint8_t SfrIen[3] = {SfrP0Ien, SfrP1Ien, SfrP2Ien};
constexpr std::uint8_t SfrDir[3] = {SfrP0Dir, SfrP1Dir, SfrP2Dir};
constexpr std::uint8_t PadMask[3] = {0xff, 0xff, 0x1f};
constexpr unsigned PadCount[3] = {8, 8, 5};
constexpr Irq PortIrq[3] = {Irq::P0Int, Irq::P1Int, Irq::P2Int};
constexpr Reg Obssel0 = 0x6243; // OBSSEL0-OBSSEL5: 0x6243-0x6248
constexpr unsigned ObsselCount = 6;
constexpr Reg Cmpctl = 0x62d0;

struct WritableBits {
    std::uint8_t sfr;
    std::uint8_t mask; // R/W bits; the others are reserved R0
};
constexpr WritableBits Registers[] = {
    {SfrP0, 0xff},     {SfrP1, 0xff},     {SfrP2, 0x1f},     {SfrP0Ifg, 0xff},
    {SfrP1Ifg, 0xff},  {SfrP2Ifg, 0x3f},  {SfrPictl, 0x8f},  {SfrP1Ien, 0xff},
    {SfrP0Inp, 0xff},  {SfrP0Ien, 0xff},  {SfrP2Ien, 0x3f},  {SfrPmux, 0xff},
    {SfrPercfg, 0x7f}, {SfrApcfg, 0xff},  {SfrP0Sel, 0xff},  {SfrP1Sel, 0xff},
    {SfrP2Sel, 0x7f},  {SfrP1Inp, 0xfc},  {SfrP2Inp, 0xff},  {SfrP0Dir, 0xff},
    {SfrP1Dir, 0xff},  {SfrP2Dir, 0xdf},
};

std::uint8_t writable(std::uint8_t sfr) {
    for (const WritableBits &r : Registers)
        if (r.sfr == sfr)
            return r.mask;
    return 0xff;
}

bool is_port(Reg reg) {
    return reg == sfr_reg(SfrP0) || reg == sfr_reg(SfrP1) || reg == sfr_reg(SfrP2);
}
unsigned port_of(Reg reg) { return reg == sfr_reg(SfrP0) ? 0 : reg == sfr_reg(SfrP1) ? 1 : 2; }

// ---- peripheral pin mapping, SWRU191F Table 7-1 ----
enum class Fn : std::uint8_t { U0, U1, T1, T3, T4 };
struct Use {
    Fn fn;
    std::uint8_t role; // USART: 0 RX/MI, 1 TX/MO, 2 CT/SS, 3 RT/C; timers: channel
};
struct Location {
    std::uint8_t port, pin;
};
// [alternative][role]
constexpr Location U0Loc[2][4] = {{{0, 2}, {0, 3}, {0, 4}, {0, 5}},
                                  {{1, 4}, {1, 5}, {1, 2}, {1, 3}}};
constexpr Location U1Loc[2][4] = {{{0, 5}, {0, 4}, {0, 2}, {0, 3}},
                                  {{1, 7}, {1, 6}, {1, 4}, {1, 5}}};
constexpr Location T1Loc[2][5] = {{{0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}},
                                  {{1, 2}, {1, 1}, {1, 0}, {0, 7}, {0, 6}}};
constexpr Location T3Loc[2][2] = {{{1, 3}, {1, 4}}, {{1, 6}, {1, 7}}};
constexpr Location T4Loc[2][2] = {{{1, 0}, {1, 1}}, {{2, 0}, {2, 3}}};

struct Uses {
    Use list[4]{};
    unsigned count = 0;
};

Uses uses_at(unsigned port, unsigned pin, std::uint8_t percfg) {
    Uses uses;
    auto scan = [&](Fn fn, const Location *location, unsigned roles) {
        for (unsigned role = 0; role < roles; ++role)
            if (location[role].port == port && location[role].pin == pin)
                uses.list[uses.count++] = {fn, static_cast<std::uint8_t>(role)};
    };
    scan(Fn::U0, U0Loc[percfg & 0x01u], 4);
    scan(Fn::U1, U1Loc[(percfg >> 1) & 1u], 4);
    scan(Fn::T1, T1Loc[(percfg >> 6) & 1u], 5);
    scan(Fn::T3, T3Loc[(percfg >> 5) & 1u], 2);
    scan(Fn::T4, T4Loc[(percfg >> 4) & 1u], 2);
    return uses;
}

// P2DIR.PRIP0 order of precedence on port 0 (lower rank wins).
unsigned port0_rank(Use use, unsigned prip0) {
    const bool t1_01 = use.fn == Fn::T1 && use.role <= 1;
    const bool t1_23 = use.fn == Fn::T1 && (use.role == 2 || use.role == 3);
    switch (prip0) {
    case 0: return use.fn == Fn::U0 ? 0 : use.fn == Fn::U1 ? 1 : 2;
    case 1: return use.fn == Fn::U1 ? 0 : use.fn == Fn::U0 ? 1 : 2;
    case 2: return t1_01 ? 0 : use.fn == Fn::U1 ? 1 : use.fn == Fn::U0 ? 2 : 3;
    default: return t1_23 ? 0 : use.fn == Fn::U0 ? 1 : use.fn == Fn::U1 ? 2 : 3;
    }
}

// Port 1 precedence of `a` over `b`, P2SEL.PRIxP1 (7.6.1-7.6.5): U0/U1
// PRI3P1, U1/T3 PRI2P1, U0/T1 PRI0P1, T1/T4 PRI1P1, and U0/T3: "The Timer 3
// channels have precedence when both bits [PRI2P1, PRI3P1] are set high"
// (7.6.2). No other pair shares a port 1 pad.
bool port1_beats(Fn a, Fn b, std::uint8_t p2sel) {
    const bool pri0 = p2sel & 0x08u, pri1 = p2sel & 0x10u, pri2 = p2sel & 0x20u,
               pri3 = p2sel & 0x40u;
    auto first_wins = [&](Fn high, Fn low, bool first) {
        if (a == high && b == low)
            return std::optional<bool>(first);
        if (a == low && b == high)
            return std::optional<bool>(!first);
        return std::optional<bool>();
    };
    for (const auto &rule : {first_wins(Fn::U0, Fn::U1, !pri3), first_wins(Fn::U1, Fn::T3, !pri2),
                             first_wins(Fn::U0, Fn::T3, !(pri2 && pri3)),
                             first_wins(Fn::U0, Fn::T1, !pri0), first_wins(Fn::T1, Fn::T4, !pri1)})
        if (rule)
            return *rule;
    return false;
}

// Number of 32.768 kHz half periods at 32 kHz-domain time `lf` (same phase as
// SLEEPSTA.CLK32K: low at 0, 65536 half periods per second).
std::uint64_t half_periods(std::uint64_t lf) { return lf / 15625 * 32 + lf % 15625 * 32 / 15625; }
// First 32 kHz-domain time after `lf` at which the clock level changes.
std::uint64_t next_half_period(std::uint64_t lf) {
    const std::uint64_t next = half_periods(lf) + 1;
    return next / 32 * 15625 + (next % 32 * 15625 + 31) / 32;
}

using PadUse = std::optional<std::optional<bool>>;
PadUse input_use() { return PadUse(std::in_place); }
PadUse output_use(bool level) { return PadUse(std::in_place, level); }

} // namespace

Io::Io(Bus &bus) : Peripheral(bus) {
    for (const WritableBits &r : Registers)
        bus.claim_sfr(r.sfr, this, r.sfr == SfrP0 || r.sfr == SfrP1 || r.sfr == SfrP2);
    for (unsigned i = 0; i < ObsselCount; ++i)
        bus.claim_xreg(static_cast<std::uint16_t>(Obssel0 + i), this);
}

void Io::reset(ResetCause) {
    // Register reset values (7.11): the port latches are all ones; every
    // other register resets to 0 (storage is zeroed by the core).
    bus_.storage(sfr_reg(SfrP0)) = 0xff;
    bus_.storage(sfr_reg(SfrP1)) = 0xff;
    bus_.storage(sfr_reg(SfrP2)) = 0x1f;
    comparator_output_ = false;
    cko_edge_lf_ = Never;
    // Pads settle to their reset configuration without generating edges or
    // notifying other blocks (they are reset as well); the host listener
    // still sees every drive change.
    updating_ = true;
    unsigned rounds = 0;
    do {
        update_again_ = false;
        refresh(false, bus_.now());
    } while (update_again_ && ++rounds < 8);
    update_again_ = false;
    updating_ = false;
}

std::uint8_t Io::read(Reg reg) {
    // SWRU191F 7.3: reading P0-P2 returns the pin levels, except in the read
    // half of a read-modify-write instruction, which uses the latch.
    if (is_port(reg) && bus_.rmw_access())
        return bus_.storage(reg);
    return peek(reg);
}

std::uint8_t Io::peek(Reg reg) const {
    if (is_port(reg))
        return last_level_[port_of(reg)];
    return bus_.storage(reg);
}

void Io::write(Reg reg, std::uint8_t value) {
    if (!is_sfr(reg)) {
        // OBSSEL0-5. The rfc_obs_sig radio signals are not modeled.
        bus_.storage(reg) = value;
        if (value & 0x80u)
            bus_.note_unmodeled(reg, true);
        update();
        return;
    }
    const std::uint8_t sfr = sfr_of(reg);
    const std::uint8_t mask = writable(sfr);
    switch (sfr) {
    case SfrP0Ifg:
    case SfrP1Ifg:
    case SfrP2Ifg:
        // R/W0: writing 0 clears a flag, writing 1 has no effect.
        bus_.storage(reg) = static_cast<std::uint8_t>(bus_.storage(reg) & value & mask);
        assert_cpu_flags();
        return;
    case SfrP0Ien:
    case SfrP1Ien:
    case SfrP2Ien:
        bus_.storage(reg) = static_cast<std::uint8_t>(value & mask);
        assert_cpu_flags();
        return;
    case SfrPictl:
        // PADSC (output drive strength) has no observable effect in a
        // logic-level pad model.
        bus_.storage(reg) = static_cast<std::uint8_t>(value & mask);
        return;
    default:
        bus_.storage(reg) = static_cast<std::uint8_t>(value & mask);
        update();
        return;
    }
}

void Io::cpu_flags_written() { assert_cpu_flags(); }

void Io::power_mode_changed(PowerMode, PowerMode to) {
    // PMUX note: OBSSEL0-OBSSEL5 do not retain data in PM2 and PM3.
    if (to == PowerMode::Pm2 || to == PowerMode::Pm3)
        for (unsigned i = 0; i < ObsselCount; ++i)
            bus_.storage(static_cast<Reg>(Obssel0 + i)) = 0;
    update();
}

void Io::clock_changed() { update(); } // CLKCONCMD.OSC32K owns P2_3/P2_4 (7.8)

std::uint64_t Io::deadline() const {
    return cko_edge_lf_ == Never ? Never : bus_.lf_deadline(cko_edge_lf_);
}

void Io::service(std::uint64_t now) {
    // The CKO pad changes at the 32 kHz edge itself; report that time.
    std::uint64_t at = now;
    const std::uint64_t lf = bus_.lf_now();
    if (cko_edge_lf_ != Never && lf >= cko_edge_lf_ && now >= lf - cko_edge_lf_)
        at = now - (lf - cko_edge_lf_);
    update(at);
}

// ---- host ----

void Io::set_input(unsigned port, unsigned pin, std::optional<bool> level) {
    external_[port][pin] = level;
    update();
}

void Io::set_analog_mv(unsigned port, unsigned pin, std::optional<std::uint32_t> millivolts) {
    analog_[port][pin] = millivolts;
    if (port == 0)
        bus_.soc().comparator->inputs_changed();
}

std::optional<std::uint32_t> Io::analog_mv(unsigned port, unsigned pin) const {
    return analog_[port][pin];
}

PinDrive Io::drive(unsigned port, unsigned pin) const { return drive_[port][pin]; }

bool Io::level(unsigned port, unsigned pin) const { return level_of(port, pin, drive_[port][pin]); }

void Io::set_listener(Listener listener) { listener_ = std::move(listener); }

// ---- peripherals ----

void Io::peripheral_outputs_changed() { update(); }

void Io::comparator_changed(bool output) {
    comparator_output_ = output;
    update();
}

// ---- implementation ----

bool Io::cko_level() const { return (half_periods(bus_.lf_now()) & 1u) != 0; }

PinDrive Io::compute_drive(unsigned port, unsigned pin) const {
    auto reg = [this](std::uint8_t sfr) { return bus_.storage(sfr_reg(sfr)); };
    const std::uint8_t pmux = reg(SfrPmux);
    if (port == 0 && (pmux & 0x80u) && ((pmux >> 4) & 7u) == pin)
        return cko_level() ? PinDrive::High : PinDrive::Low;
    if (port == 1 && (pmux & 0x08u) && (pmux & 7u) == pin) {
        const PowerMode mode = bus_.power_mode();
        return mode == PowerMode::Pm2 || mode == PowerMode::Pm3 ? PinDrive::Low : PinDrive::High;
    }
    // Model-defined: an enabled observation output drives low (the radio
    // observation signals are not modeled).
    if (port == 1 && pin < ObsselCount && (bus_.storage(static_cast<Reg>(Obssel0 + pin)) & 0x80u))
        return PinDrive::Low;
    if (port == 2 && (pin == 3 || pin == 4) && !(reg(SfrClkconcmd) & 0x80u))
        return PinDrive::HiZ;
    if (port == 0 && ((reg(SfrApcfg) >> pin) & 1u))
        return PinDrive::HiZ;

    bool selected = false;
    if (port == 2) {
        const std::uint8_t p2sel = reg(SfrP2Sel);
        selected = (pin == 0 && (p2sel & 0x01u)) || (pin == 3 && (p2sel & 0x02u)) ||
                   (pin == 4 && (p2sel & 0x04u));
    } else {
        selected = (reg(port == 0 ? SfrP0Sel : SfrP1Sel) >> pin) & 1u;
    }
    if (selected) {
        // Model-defined: a selected pad without a mapped function, or whose
        // function is an input, is 3-state (PxSEL = 1 disables the pull).
        const PadUse use = peripheral_pad(port, pin);
        if (use && *use)
            return **use ? PinDrive::High : PinDrive::Low;
        return PinDrive::HiZ;
    }

    if ((reg(SfrDir[port]) >> pin) & 1u)
        return ((bus_.storage(sfr_reg(SfrPort[port])) >> pin) & 1u) ? PinDrive::High
                                                                     : PinDrive::Low;
    if (port == 1 && pin < 2)
        return PinDrive::HiZ; // P1_0/P1_1: no pullup or pulldown (P1INP)
    const std::uint8_t mode = port == 0 ? reg(SfrP0Inp) : port == 1 ? reg(SfrP1Inp) : reg(SfrP2Inp);
    if ((mode >> pin) & 1u)
        return PinDrive::HiZ; // 3-state
    return ((reg(SfrP2Inp) >> (5 + port)) & 1u) ? PinDrive::PullDown : PinDrive::PullUp;
}

std::optional<std::optional<bool>> Io::peripheral_pad(unsigned port, unsigned pin) const {
    Soc &soc = bus_.soc();
    const Uses uses = uses_at(port, pin, bus_.storage(sfr_reg(SfrPercfg)));
    const std::uint8_t p2dir = bus_.storage(sfr_reg(SfrP2Dir));
    const std::uint8_t p2sel = bus_.storage(sfr_reg(SfrP2Sel));
    auto timer_pad = [&](std::uint16_t cctl_address, bool output) {
        // CCTLn.MODE (bit 2) = 1: compare mode, the channel drives its pad.
        return (bus_.xdata_peek(cctl_address) & 0x04u) ? output_use(output) : input_use();
    };
    PadUse best;
    Use best_use{};
    for (unsigned i = 0; i < uses.count; ++i) {
        const Use use = uses.list[i];
        PadUse pad;
        switch (use.fn) {
        case Fn::U0:
        case Fn::U1:
            pad = soc.usart[use.fn == Fn::U0 ? 0 : 1]->pad_use(use.role);
            break;
        case Fn::T1: {
            static constexpr std::uint16_t Cctl[5] = {0x70e5, 0x70e6, 0x70e7, 0x62a3, 0x62a4};
            pad = timer_pad(Cctl[use.role], soc.timer1->output(use.role));
            break;
        }
        case Fn::T3:
            pad = timer_pad(use.role == 0 ? 0x70cc : 0x70ce, soc.timer34->output(3, use.role));
            break;
        case Fn::T4:
            pad = timer_pad(use.role == 0 ? 0x70ec : 0x70ee, soc.timer34->output(4, use.role));
            break;
        }
        if (!pad)
            continue; // not present on the pad (UART RTS/CTS without FLOW, SPI master SSN)
        bool wins = !best;
        if (!wins && port == 0)
            wins = port0_rank(use, p2dir >> 6) < port0_rank(best_use, p2dir >> 6);
        else if (!wins && port == 1)
            wins = port1_beats(use.fn, best_use.fn, p2sel);
        if (wins) {
            best = pad;
            best_use = use;
        }
    }
    return best;
}

bool Io::level_of(unsigned port, unsigned pin, PinDrive drive) const {
    if (external_[port][pin])
        return *external_[port][pin];
    return drive != PinDrive::Low && drive != PinDrive::PullDown;
}

// Model-defined pad voltage: the host's analog voltage; else the op-amp
// output on P0_2 (analog pad, APCFG[2] = 1); else the digital level as 0 V /
// VDD; a floating pad without any source reads 0 V.
std::uint32_t Io::pad_mv(unsigned port, unsigned pin) const {
    if (analog_[port][pin])
        return *analog_[port][pin];
    Soc &soc = bus_.soc();
    if (port == 0 && pin == 2 && (bus_.storage(sfr_reg(SfrApcfg)) & 0x04u))
        if (const auto output = soc.opamp->output_mv())
            return *output;
    const PinDrive drive = drive_[port][pin];
    if (!external_[port][pin] && drive == PinDrive::HiZ)
        return 0;
    return level_of(port, pin, drive) ? soc.adc->vdd_mv_ : 0;
}

bool Io::usart_pad_level(unsigned index, unsigned role) const {
    const unsigned alternative = (bus_.storage(sfr_reg(SfrPercfg)) >> index) & 1u;
    const Location pad = index == 0 ? U0Loc[alternative][role] : U1Loc[alternative][role];
    return level(pad.port, pad.pin);
}

void Io::update() { update(bus_.now()); }

void Io::update(std::uint64_t at) {
    if (updating_) {
        update_again_ = true;
        return;
    }
    struct Guard {
        bool &flag;
        explicit Guard(bool &f) : flag(f) { flag = true; }
        ~Guard() { flag = false; }
    } guard(updating_);
    // Pads settle within a bounded number of rounds unless peripherals feed
    // back on each other combinatorially.
    unsigned rounds = 0;
    do {
        update_again_ = false;
        refresh(true, at);
    } while (update_again_ && ++rounds < 64);
    if (update_again_) {
        update_again_ = false;
        bus_.fault("CC2530 I/O pads do not settle");
    }
    bus_.reschedule();
}

void Io::refresh(bool notify, std::uint64_t at) {
    std::pair<std::uint8_t, std::uint8_t> drive_changes[21];
    unsigned drive_change_count = 0;
    bool port0_drive_changed = false;
    for (unsigned port = 0; port < 3; ++port) {
        for (unsigned pin = 0; pin < PadCount[port]; ++pin) {
            const PinDrive drive = compute_drive(port, pin);
            if (drive == drive_[port][pin])
                continue;
            drive_[port][pin] = drive;
            drive_changes[drive_change_count++] = {static_cast<std::uint8_t>(port),
                                                   static_cast<std::uint8_t>(pin)};
            port0_drive_changed = port0_drive_changed || port == 0;
        }
    }
    cko_edge_lf_ = (bus_.storage(sfr_reg(SfrPmux)) & 0x80u) ? next_half_period(bus_.lf_now()) : Never;

    std::uint8_t level_changes[3]{}, rising[3]{}, falling[3]{};
    const bool comparator_enabled = (bus_.storage(Cmpctl) & 0x02u) != 0;
    for (unsigned port = 0; port < 3; ++port) {
        std::uint8_t levels = 0;
        for (unsigned pin = 0; pin < PadCount[port]; ++pin)
            if (level_of(port, pin, drive_[port][pin]))
                levels = static_cast<std::uint8_t>(levels | (1u << pin));
        level_changes[port] = static_cast<std::uint8_t>(levels ^ last_level_[port]);
        last_level_[port] = levels;
        std::uint8_t detector = levels;
        if (port == 0 && comparator_enabled)
            detector = static_cast<std::uint8_t>((detector & ~0x20u) | (comparator_output_ ? 0x20u : 0u));
        rising[port] = static_cast<std::uint8_t>(detector & ~detector_[port]);
        falling[port] = static_cast<std::uint8_t>(~detector & detector_[port] & PadMask[port]);
        detector_[port] = detector;
    }

    if (listener_)
        for (unsigned i = 0; i < drive_change_count; ++i) {
            const auto [port, pin] = drive_changes[i];
            listener_(port, pin, drive_[port][pin], at);
        }
    if (!notify)
        return;

    // Peripheral inputs read their pads whatever PxSEL is (7.6).
    Soc &soc = bus_.soc();
    const std::uint8_t percfg = bus_.storage(sfr_reg(SfrPercfg));
    for (unsigned port = 0; port < 3; ++port) {
        for (unsigned pin = 0; pin < PadCount[port]; ++pin) {
            if (!((level_changes[port] >> pin) & 1u))
                continue;
            const bool level = (last_level_[port] >> pin) & 1u;
            soc.sleep_timer->input_changed(port, pin, level);
            const Uses uses = uses_at(port, pin, percfg);
            for (unsigned i = 0; i < uses.count; ++i) {
                const Use use = uses.list[i];
                if (use.fn == Fn::T1)
                    soc.timer1->input_changed(use.role, level);
                else if (use.fn == Fn::T3 || use.fn == Fn::T4)
                    soc.timer34->input_changed(use.fn == Fn::T3 ? 3 : 4, use.role, level);
            }
            if (port == 2 && pin == 0 && level)
                soc.adc->p2_0_rising(); // ADCCON1.STSEL = 00 trigger (7.6.6)
        }
    }
    if (level_changes[0] || level_changes[1]) {
        soc.usart[0]->pad_levels_changed();
        soc.usart[1]->pad_levels_changed();
    }
    if (port0_drive_changed || level_changes[0])
        soc.comparator->inputs_changed();

    for (unsigned port = 0; port < 3; ++port)
        port_edges(port, rising[port], falling[port]);
}

void Io::port_edges(unsigned port, std::uint8_t rising, std::uint8_t falling) {
    // PICTL: 0 = rising edge, 1 = falling edge (P0ICON all of port 0,
    // P1ICONL P1_3-P1_0, P1ICONH P1_7-P1_4, P2ICON all of port 2).
    const std::uint8_t pictl = bus_.storage(sfr_reg(SfrPictl));
    std::uint8_t falling_selected = 0;
    if (port == 0)
        falling_selected = (pictl & 0x01u) ? 0xff : 0x00;
    else if (port == 1)
        falling_selected = static_cast<std::uint8_t>(((pictl & 0x02u) ? 0x0fu : 0u) |
                                                     ((pictl & 0x04u) ? 0xf0u : 0u));
    else
        falling_selected = (pictl & 0x08u) ? 0x1f : 0x00;
    const auto events = static_cast<std::uint8_t>(
        ((rising & ~falling_selected) | (falling & falling_selected)) & PadMask[port]);
    if (!events)
        return;
    std::uint8_t &flags = bus_.storage(sfr_reg(SfrIfg[port]));
    flags = static_cast<std::uint8_t>(flags | events);
    // 7.5 and Table 8-1 note (1): IOC_0/IOC_1 fire on an interrupt of an
    // interrupt-enabled port 0/port 1 pin.
    if (port < 2 && (events & bus_.storage(sfr_reg(SfrIen[port]))))
        trigger_dma(port == 0 ? trigger::Ioc0 : trigger::Ioc1);
    assert_cpu_flags();
}

void Io::assert_cpu_flags() {
    for (unsigned port = 0; port < 3; ++port) {
        const std::uint8_t pending = bus_.storage(sfr_reg(SfrIfg[port])) &
                                     bus_.storage(sfr_reg(SfrIen[port])) & PadMask[port];
        if (pending && !bus_.irq_flag(PortIrq[port]))
            bus_.raise_irq(PortIrq[port]);
    }
}

void Io::trigger_dma(unsigned trigger) {
    ++dma_triggers_[trigger];
    bus_.dma_trigger(trigger);
}

} // namespace mcs51emu::cc2530
