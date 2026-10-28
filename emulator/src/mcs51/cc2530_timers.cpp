// CC2530 Timer 1 (SWRU191F Chapter 9), Timer 3/Timer 4 (Chapter 10), sleep
// timer (Chapter 11) and watchdog timer (Chapter 16).
//
// Timer 1, 3 and 4 count ticks of the HF domain (hf_now()): the tick is
// 32 MHz >> CLKCONSTA.TICKSPD (SWRU191F 4.4, 9.1, 10.1), divided by the
// timer's prescaler. Between events the counters are not stepped one by one:
// a counter keeps the HF time of its last step (anchor) and is advanced in
// closed form up to the next tick at which something can happen (a wrap, a
// direction change, or a count equal to 0, the terminal value, TxCC0 or a
// compare value). Events are processed tick by tick with their own times.
//
// The sleep timer and the watchdog count the 32 kHz domain (lf_now()).

#include "cc2530_timers.hpp"

#include "cc2530_analog.hpp"
#include "cc2530_io.hpp"

#include <algorithm>

namespace mcs51emu::cc2530 {

namespace {

// Core SFRs read by the timers (owned by the core or other peripherals).
constexpr std::uint8_t SfrIen0 = 0xa8, SfrIen1 = 0xb8, SfrIen2 = 0x9a;
constexpr std::uint8_t Ien0Ea = 0x80, Ien0Stie = 0x20;
constexpr std::uint8_t Ien1T1ie = 0x02, Ien1T3ie = 0x08, Ien1T4ie = 0x10;
constexpr std::uint8_t Ien2Wdtie = 0x20;
constexpr std::uint16_t XdataPictl = 0x708c, XdataDmaarm = 0x70d6;

// Timer 1 (SWRU191F 9.12, 9.13).
constexpr std::uint8_t SfrT1cc0l = 0xda, SfrT1cntl = 0xe2, SfrT1cnth = 0xe3, SfrT1ctl = 0xe4,
                       SfrT1cctl0 = 0xe5, SfrT1stat = 0xaf;
constexpr std::uint16_t XregIrctl = 0x6281, XregT1cctl0 = 0x62a0, XregT1cc0l = 0x62a6;

// Timer 3/4 (SWRU191F 10.8). Timer 4 registers are Timer 3's + 0x20.
constexpr std::uint8_t SfrT3cnt = 0xca, SfrT3ctl = 0xcb, SfrT3cctl0 = 0xcc, SfrT3cc0 = 0xcd,
                       SfrT3cctl1 = 0xce, SfrT3cc1 = 0xcf, SfrTimif = 0xd8;
constexpr std::uint8_t TimifT1ovfim = 0x40;

// Sleep timer (SWRU191F 11.4).
constexpr std::uint8_t SfrSt0 = 0x95, SfrSt1 = 0x96, SfrSt2 = 0x97, SfrStload = 0xad;
constexpr std::uint16_t XregStcc = 0x62b0, XregStcs = 0x62b1, XregStcv0 = 0x62b2;
constexpr std::uint32_t StMask = 0xffffff;

// Watchdog (SWRU191F 16.3).
constexpr std::uint8_t SfrWdctl = 0xc9;

// SWRU191F Table 9-1 and Table 10-1: initial compare output per CMP[2:0].
constexpr bool InitialOutput[8] = {false, true, false, false, true, false, true, false};

bool hf_stopped(PowerMode mode) {
    return mode == PowerMode::Pm1 || mode == PowerMode::Pm2 || mode == PowerMode::Pm3;
}

std::uint8_t sfr_storage(const Bus &bus, std::uint8_t sfr) { return bus.storage(sfr_reg(sfr)); }

bool dma_armed(const Bus &bus) { return (bus.xdata_peek(XdataDmaarm) & 0x1fu) != 0; }

// The 32 kHz clock as SLEEPSTA.CLK32K shows it (core): 65536 half periods
// per second, half period h = floor(lf * 32 / 15625), low in even halves.
// lf_ticks() counts whole periods, so period j holds halves 2j (low) and
// 2j + 1 (high): rising edge j lies inside period j.
std::uint64_t half_periods(std::uint64_t lf) { return lf / 15625 * 32 + lf % 15625 * 32 / 15625; }
std::uint64_t rising_edge_lf(std::uint64_t edge) {
    const std::uint64_t half = 2 * edge + 1;
    return half / 32 * 15625 + (half % 32 * 15625 + 31) / 32;
}
// First rising edge strictly after `lf`.
std::uint64_t first_edge_after(std::uint64_t lf) { return (half_periods(lf) + 1) / 2; }

} // namespace

// ===========================================================================
// Shared 8/16-bit counter engine (Timer 1, Timer 3, Timer 4)
// ===========================================================================

// Ticks until the next step that is not a plain +1/-1 or that lands on a
// value something reacts to; 0 when the counter does not run on HF ticks.
std::uint64_t Timer1::ticks_to_event(const Unit &unit) {
    const Counter &c = unit.c;
    if (c.mode == CountMode::Stopped || c.halted || c.external)
        return 0;
    const unsigned value = c.count;
    const unsigned cc0 = unit.ch[0].value;
    const bool going_down = c.mode == CountMode::Down || (c.mode == CountMode::UpDown && c.down);
    if (!going_down) {
        if (value == c.max)
            return 1;
        if ((c.mode == CountMode::Modulo || c.mode == CountMode::UpDown) && value == cc0)
            return 1;
        std::uint64_t best = c.max - value;
        const auto consider = [&](unsigned target) {
            if (target > value && target - value < best)
                best = target - value;
        };
        consider(cc0);
        for (unsigned n = 0; n < unit.channels; ++n)
            if (unit.ch[n].ctl & 0x04u)
                consider(unit.ch[n].value);
        return best;
    }
    if (value == 0)
        return 1;
    std::uint64_t best = value;
    for (unsigned n = 0; n < unit.channels; ++n) {
        const unsigned target = unit.ch[n].value;
        if ((unit.ch[n].ctl & 0x04u) && target < value && value - target < best)
            best = value - target;
    }
    return best;
}

void Timer1::step_linear(Unit &unit, std::uint64_t ticks) {
    Counter &c = unit.c;
    const bool going_down = c.mode == CountMode::Down || (c.mode == CountMode::UpDown && c.down);
    c.count = static_cast<std::uint16_t>(going_down ? c.count - ticks : c.count + ticks);
}

void Timer1::load_pending(Unit &unit) {
    for (unsigned n = 0; n < unit.channels; ++n) {
        Channel &ch = unit.ch[n];
        if (ch.has_pending) {
            ch.value = ch.pending;
            ch.has_pending = false;
        }
    }
}

// CMP = 7 is never stored (the write keeps CMP), so ctl holds 0-6.
void Timer1::initialize_output(Channel &channel) {
    channel.out = InitialOutput[(channel.ctl >> 3) & 7u];
}

// One counter step and its compare output actions.
std::uint8_t Timer1::step(Unit &unit) {
    Counter &c = unit.c;
    const unsigned cc0 = unit.ch[0].value;
    std::uint8_t events = 0;
    bool up = true;
    switch (c.mode) {
    case CountMode::Stopped:
        return 0;
    case CountMode::Free:
        // SWRU191F 9.3, 10.2.1: 0x0000..max, overflow when max wraps to 0.
        if (c.count == c.max) {
            c.count = 0;
            events |= EventOverflow;
        } else {
            ++c.count;
        }
        break;
    case CountMode::Modulo:
        // SWRU191F 9.4, 10.2.3: 0..T1CC0, then 0. OVFIF is documented only
        // for the terminal count 0xFFFF/0xFF reached when the count started
        // above T1CC0 (Figure 9-2 shows no overflow at T1CC0).
        if (c.count == c.max) {
            c.count = 0;
            events |= EventOverflow;
        } else if (c.count == cc0) {
            c.count = 0;
        } else {
            ++c.count;
        }
        break;
    case CountMode::UpDown:
        // SWRU191F 9.5, 10.2.4: up to T1CC0, down to 0; OVFIF at 0.
        if (c.down && c.count == 0)
            c.down = false; // not reachable through the documented paths
        if (!c.down) {
            if (c.count == cc0 && cc0 == 0) {
                // Model-defined: with T1CC0 = 0 the counter stays at 0 and
                // reaches 0 (overflow) on every tick.
                events |= EventOverflow;
            } else if (c.count == cc0) {
                c.down = true;
                up = false;
                if (--c.count == 0) {
                    events |= EventOverflow;
                    c.down = false;
                }
            } else if (c.count == c.max) {
                // Model-defined: started above T1CC0, wraps at max.
                c.count = 0;
                events |= EventOverflow;
            } else {
                ++c.count;
            }
        } else {
            up = false;
            if (--c.count == 0) {
                events |= EventOverflow;
                c.down = false;
            }
        }
        break;
    case CountMode::Down:
        // SWRU191F 10.2.2: from TxCC0 down to 0x00, OVFIF at 0x00.
        // Model-defined: the counter then stays at 0x00 (halted) until it
        // is restarted; the document does not say what follows.
        up = false;
        if (c.count > 0)
            --c.count;
        if (c.count == 0) {
            events |= EventOverflow;
            c.halted = true;
        }
        break;
    }
    // SWRU191F 9.8, 10.5: compare register writes take effect at count 0.
    // Model-defined: the new values are compared from this step on.
    if (c.count == 0)
        load_pending(unit);
    const unsigned value = c.count;
    const bool updown = c.mode == CountMode::UpDown;
    for (unsigned n = 0; n < unit.channels; ++n) {
        Channel &ch = unit.ch[n];
        if (!(ch.ctl & 0x04u))
            continue; // capture mode
        const unsigned cmp = (ch.ctl >> 3) & 7u;
        const bool match = value == ch.value;
        // Model-defined: when the count matches both the 0/0xFF/T1CC0
        // action and the channel's compare value, the compare action wins.
        if (unit.timer1) {
            // SWRU191F 9.12 T1CCTLn.CMP.
            const bool split = updown && n != 0; // compare-up / compare-down
            if ((cmp == 3 || cmp == 4) && !split && value == 0)
                ch.out = cmp == 4;
            if ((cmp == 5 || cmp == 6) && n != 0 && value == cc0)
                ch.out = cmp == 6;
            if (match) {
                switch (cmp) {
                case 0: ch.out = true; break;
                case 1: ch.out = false; break;
                case 2: ch.out = !ch.out; break;
                case 3: ch.out = split ? up : true; break;
                case 4: ch.out = split ? !up : false; break;
                case 5: if (n != 0) ch.out = true; break;  // channel 0: reserved
                case 6: if (n != 0) ch.out = false; break; // channel 0: reserved
                default: break;
                }
            }
        } else {
            // SWRU191F 10.8 TxCCTLn.CMP (channel 0 has no compare-down).
            const bool split = updown && n == 1;
            if ((cmp == 3 || cmp == 4) && !split && value == 0)
                ch.out = cmp == 4;
            if (cmp == 5 && value == c.max)
                ch.out = false;
            if (cmp == 6 && value == 0)
                ch.out = true;
            if (match) {
                switch (cmp) {
                case 0: ch.out = true; break;
                case 1: ch.out = false; break;
                case 2: ch.out = !ch.out; break;
                case 3: ch.out = split ? up : true; break;
                case 4: ch.out = split ? !up : false; break;
                case 5: ch.out = true; break;
                case 6: ch.out = false; break;
                default: break;
                }
            }
        }
        if (match)
            events |= static_cast<std::uint8_t>(1u << n);
    }
    return events;
}

// Steps `unit` through every tick at or before HF time `hf`; emit(events)
// runs after each step that lands on an event value.
template <class Emit>
void Timer1::advance(Unit &unit, std::uint64_t hf, Emit &&emit) {
    Counter &c = unit.c;
    for (;;) {
        const std::uint64_t ticks = ticks_to_event(unit);
        if (ticks == 0 || hf < c.anchor)
            return;
        const std::uint64_t at = c.anchor + ticks * c.period;
        if (at > hf) {
            const std::uint64_t whole = (hf - c.anchor) / c.period;
            step_linear(unit, whole);
            c.anchor += whole * c.period;
            return;
        }
        step_linear(unit, ticks - 1);
        c.anchor = at;
        emit(step(unit));
    }
}

std::uint64_t Timer1::unit_deadline(const Bus &bus, const Unit &unit) {
    const std::uint64_t ticks = ticks_to_event(unit);
    if (ticks == 0)
        return Never;
    return bus.hf_deadline(unit.c.anchor + ticks * unit.c.period);
}

// ===========================================================================
// Timer 1
// ===========================================================================

namespace {

enum class T1Kind { None, CcLow, CcHigh, Cctl, Cntl, Cnth, Ctl, Stat, Irctl };
struct T1Reg {
    T1Kind kind = T1Kind::None;
    unsigned n = 0;
};

// SWRU191F 9.13: T1CCTL0-4 at 0x62A0-0x62A4 and T1CC0L..T1CC4H at
// 0x62A6-0x62AF; channels 0-2 alias the SFRs.
T1Reg decode_t1(Reg reg) {
    if (is_sfr(reg)) {
        const std::uint8_t sfr = sfr_of(reg);
        if (sfr >= SfrT1cc0l && sfr <= SfrT1cc0l + 5)
            return {(sfr - SfrT1cc0l) & 1u ? T1Kind::CcHigh : T1Kind::CcLow,
                    (sfr - SfrT1cc0l) / 2u};
        if (sfr >= SfrT1cctl0 && sfr <= SfrT1cctl0 + 2)
            return {T1Kind::Cctl, static_cast<unsigned>(sfr - SfrT1cctl0)};
        switch (sfr) {
        case SfrT1cntl: return {T1Kind::Cntl, 0};
        case SfrT1cnth: return {T1Kind::Cnth, 0};
        case SfrT1ctl: return {T1Kind::Ctl, 0};
        case SfrT1stat: return {T1Kind::Stat, 0};
        default: return {};
        }
    }
    if (reg >= XregT1cctl0 && reg <= XregT1cctl0 + 4)
        return {T1Kind::Cctl, static_cast<unsigned>(reg - XregT1cctl0)};
    if (reg >= XregT1cc0l && reg <= XregT1cc0l + 9)
        return {(reg - XregT1cc0l) & 1u ? T1Kind::CcHigh : T1Kind::CcLow,
                static_cast<unsigned>(reg - XregT1cc0l) / 2u};
    if (reg == XregIrctl)
        return {T1Kind::Irctl, 0};
    return {};
}

// Storage of channel n's registers: the SFR for channels 0-2, else XREG.
Reg t1_cctl_reg(unsigned n) {
    return n < 3 ? sfr_reg(static_cast<std::uint8_t>(SfrT1cctl0 + n))
                 : static_cast<Reg>(XregT1cctl0 + n);
}
Reg t1_cc_reg(unsigned n, bool high) {
    const unsigned offset = 2 * n + (high ? 1 : 0);
    return n < 3 ? sfr_reg(static_cast<std::uint8_t>(SfrT1cc0l + offset))
                 : static_cast<Reg>(XregT1cc0l + offset);
}

constexpr std::uint64_t T1Dividers[4] = {1, 8, 32, 128};

} // namespace

Timer1::Timer1(Bus &bus) : Peripheral(bus) {
    for (std::uint8_t sfr = SfrT1cc0l; sfr <= SfrT1cc0l + 5; ++sfr)
        bus_.claim_sfr(sfr, this, false);
    bus_.claim_sfr(SfrT1cntl, this, true); // reading latches T1CNTH
    bus_.claim_sfr(SfrT1cnth, this, false);
    bus_.claim_sfr(SfrT1ctl, this, false);
    for (std::uint8_t sfr = SfrT1cctl0; sfr <= SfrT1cctl0 + 2; ++sfr)
        bus_.claim_sfr(sfr, this, false);
    bus_.claim_sfr(SfrT1stat, this, false);
    for (std::uint16_t xreg = XregT1cctl0; xreg <= XregT1cctl0 + 4; ++xreg)
        bus_.claim_xreg(xreg, this);
    for (std::uint16_t xreg = XregT1cc0l; xreg <= XregT1cc0l + 9; ++xreg)
        bus_.claim_xreg(xreg, this);
    bus_.claim_xreg(XregIrctl, this);
}

void Timer1::reset(ResetCause) {
    unit_ = Unit{};
    unit_.c.max = 0xffff;
    unit_.channels = 5;
    unit_.timer1 = true;
    unit_.c.period = tick_period();
    unit_.c.anchor = bus_.hf_now();
    ir_prescale_ = 0;
    // SWRU191F 9.12: T1CCTLn reset 0x40 (IM = 1), everything else 0x00.
    for (unsigned n = 0; n < 5; ++n)
        bus_.storage(t1_cctl_reg(n)) = 0x40;
}

std::uint64_t Timer1::tick_period() const {
    const unsigned tickspd = (bus_.clkconsta() >> 3) & 7u;
    const unsigned div = (sfr_storage(bus_, SfrT1ctl) >> 2) & 3u;
    return (std::uint64_t{1} << tickspd) * T1Dividers[div];
}

void Timer1::handle_events(std::uint8_t events) {
    if (events == 0)
        return;
    // SWRU191F 9.10: source flags are set regardless of the masks; IRCON.T1IF
    // is set when an event occurs whose mask bit is 1.
    std::uint8_t &stat = bus_.storage(sfr_reg(SfrT1stat));
    bool request = false;
    if (events & EventOverflow) {
        stat |= 0x20u;
        request = request || (sfr_storage(bus_, SfrTimif) & TimifT1ovfim);
    }
    for (unsigned n = 0; n < 5; ++n) {
        if (!(events & (1u << n)))
            continue;
        stat |= static_cast<std::uint8_t>(1u << n);
        request = request || (unit_.ch[n].ctl & 0x40u);
    }
    if (request)
        bus_.raise_irq(Irq::T1);
    // SWRU191F 9.11: T1_CH0..T1_CH2 on channel 0-2 compare events.
    for (unsigned n = 0; n < 3; ++n) {
        if (events & (1u << n)) {
            ++dma_triggers_[n];
            bus_.dma_trigger(trigger::T1Ch0 + n);
        }
    }
    // SWRU191F 12.2.4 / ADCCON1.STSEL = 10: a Timer 1 channel 0 compare event
    // starts an ADC sequence.
    if (events & 1u)
        bus_.soc().adc->timer1_ch0_event();
}

void Timer1::sync_storage() {
    for (unsigned n = 0; n < 5; ++n) {
        bus_.storage(t1_cc_reg(n, false)) = static_cast<std::uint8_t>(unit_.ch[n].value);
        bus_.storage(t1_cc_reg(n, true)) = static_cast<std::uint8_t>(unit_.ch[n].value >> 8);
    }
}

void Timer1::settle() {
    advance(unit_, bus_.hf_now(), [this](std::uint8_t events) { handle_events(events); });
    sync_storage();
}

std::uint8_t Timer1::outputs() const {
    std::uint8_t bits = 0;
    for (unsigned n = 0; n < 5; ++n)
        if (output(n))
            bits |= static_cast<std::uint8_t>(1u << n);
    return bits;
}

void Timer1::notify_outputs(std::uint8_t before) {
    if (outputs() != before)
        bus_.soc().io->peripheral_outputs_changed();
}

std::uint16_t Timer1::count_now() const {
    Unit copy = unit_;
    advance(copy, bus_.hf_now(), [](std::uint8_t) {});
    return copy.c.count;
}

std::uint8_t Timer1::read(Reg reg) {
    if (reg == sfr_reg(SfrT1cntl)) {
        // SWRU191F 9.1: reading T1CNTL buffers the high byte in T1CNTH.
        const std::uint8_t before = outputs();
        settle();
        notify_outputs(before);
        bus_.storage(sfr_reg(SfrT1cnth)) = static_cast<std::uint8_t>(unit_.c.count >> 8);
        return static_cast<std::uint8_t>(unit_.c.count);
    }
    return peek(reg);
}

std::uint8_t Timer1::peek(Reg reg) const {
    const T1Reg r = decode_t1(reg);
    switch (r.kind) {
    case T1Kind::CcLow: return bus_.storage(t1_cc_reg(r.n, false));
    case T1Kind::CcHigh: return bus_.storage(t1_cc_reg(r.n, true));
    case T1Kind::Cctl: return bus_.storage(t1_cctl_reg(r.n));
    case T1Kind::Cntl: return static_cast<std::uint8_t>(count_now());
    default: return bus_.storage(reg);
    }
}

void Timer1::write(Reg reg, std::uint8_t value) {
    const std::uint8_t before = outputs();
    settle();
    const T1Reg r = decode_t1(reg);
    Counter &c = unit_.c;
    switch (r.kind) {
    case T1Kind::CcLow:
        // SWRU191F 9.12 T1CCnL: buffered until T1CCnH is written.
        unit_.ch[r.n].low = value;
        break;
    case T1Kind::CcHigh: {
        // SWRU191F 9.12 T1CCnH: in compare mode the new value takes effect
        // when T1CNT = 0x0000 (at once if the count is 0 now).
        Channel &ch = unit_.ch[r.n];
        const auto full = static_cast<std::uint16_t>(value << 8 | ch.low);
        if ((ch.ctl & 0x04u) && c.count != 0) {
            ch.pending = full;
            ch.has_pending = true;
        } else {
            ch.value = full;
            ch.has_pending = false;
        }
        break;
    }
    case T1Kind::Cctl: {
        Channel &ch = unit_.ch[r.n];
        std::uint8_t ctl = value;
        if (((value >> 3) & 7u) == 7u) {
            // SWRU191F Table 9-1: CMP = 111 initializes the output for the
            // current CMP and leaves CMP unchanged.
            ctl = static_cast<std::uint8_t>((value & ~0x38u) | (ch.ctl & 0x38u));
            ch.ctl = ctl;
            initialize_output(ch);
        }
        // RFIRQ (capture from the RF interrupt) is not modelled; channel 0
        // CMP 101/110 are reserved.
        const unsigned cmp = (ctl >> 3) & 7u;
        if ((ctl & 0x80u) || (r.n == 0 && (cmp == 5 || cmp == 6)))
            bus_.note_unmodeled(reg, true);
        ch.ctl = ctl;
        bus_.storage(t1_cctl_reg(r.n)) = ctl;
        break;
    }
    case T1Kind::Cntl:
        // SWRU191F 9.1, 9.12: any write clears the counter and initializes
        // all channel outputs (Table 9-1). Model-defined: the prescaler
        // phase restarts at the write and the count direction is up.
        c.count = 0;
        c.down = false;
        c.anchor = bus_.hf_now();
        ir_prescale_ = 0;
        load_pending(unit_);
        for (unsigned n = 0; n < 5; ++n)
            initialize_output(unit_.ch[n]);
        break;
    case T1Kind::Cnth:
        break; // read-only
    case T1Kind::Ctl: {
        // SWRU191F 9.12 T1CTL: DIV 3:2, MODE 1:0 (00 suspends at the
        // present value), bits 7:4 R0.
        const std::uint8_t ctl = value & 0x0fu;
        static constexpr CountMode Modes[4] = {CountMode::Stopped, CountMode::Free,
                                               CountMode::Modulo, CountMode::UpDown};
        const CountMode mode = Modes[ctl & 3u];
        bus_.storage(reg) = ctl;
        const std::uint64_t period = tick_period();
        // Model-defined: starting or changing the prescaler restarts the
        // tick phase at the write.
        if ((c.mode == CountMode::Stopped && mode != CountMode::Stopped) || period != c.period)
            c.anchor = bus_.hf_now();
        if (mode == CountMode::UpDown && c.mode != CountMode::UpDown)
            c.down = false;
        c.mode = mode;
        c.period = period;
        break;
    }
    case T1Kind::Stat: {
        // SWRU191F 9.12: OVFIF and CHnIF are R/W0, bits 7:6 R0.
        std::uint8_t &stat = bus_.storage(reg);
        const std::uint8_t old = stat;
        stat = static_cast<std::uint8_t>(old & value & 0x3fu);
        // SWRU191F 9.10: clearing a source flag while another one with its
        // mask set is still pending sets IRCON.T1IF again.
        std::uint8_t masks = (sfr_storage(bus_, SfrTimif) & TimifT1ovfim) ? 0x20u : 0u;
        for (unsigned n = 0; n < 5; ++n)
            if (unit_.ch[n].ctl & 0x40u)
                masks |= static_cast<std::uint8_t>(1u << n);
        if ((old & ~stat) != 0 && (stat & masks) != 0)
            bus_.raise_irq(Irq::T1);
        break;
    }
    case T1Kind::Irctl: {
        // SWRU191F 9.9.2, 9.12 IRCTL: IRGEN = 1 clocks Timer 1 with Timer 3
        // channel 1 compare events. Bits 7:1 are R/W (reserved), kept.
        const bool irgen = value & 1u;
        bus_.storage(reg) = value;
        if (irgen != c.external) {
            c.external = irgen;
            c.anchor = bus_.hf_now();
            ir_prescale_ = 0;
        }
        break;
    }
    case T1Kind::None:
        bus_.storage(reg) = value;
        break;
    }
    sync_storage();
    notify_outputs(before);
}

std::uint64_t Timer1::deadline() const { return unit_deadline(bus_, unit_); }

void Timer1::service(std::uint64_t) {
    const std::uint8_t before = outputs();
    settle();
    notify_outputs(before);
}

void Timer1::power_mode_changed(PowerMode, PowerMode) {
    // The HF clock domain (hf_now/hf_deadline) stops in PM1-PM3; the count
    // and the tick phase resume where they stopped.
    bus_.reschedule();
}

void Timer1::cpu_flags_written() {
    // IRCON.T1IF is set on events and on T1STAT clears (SWRU191F 9.10),
    // not re-asserted when firmware clears it.
}

void Timer1::clock_changed() {
    // SWRU191F 4.4: CLKCONSTA.TICKSPD sets the tick. Model-defined: a new
    // tick period restarts the tick phase now.
    const std::uint8_t before = outputs();
    settle();
    const std::uint64_t period = tick_period();
    if (period != unit_.c.period) {
        unit_.c.period = period;
        unit_.c.anchor = bus_.hf_now();
    }
    notify_outputs(before);
}

bool Timer1::may_interrupt() const {
    if (deadline() == Never)
        return false;
    const bool enabled = (sfr_storage(bus_, SfrIen0) & Ien0Ea) &&
                         (sfr_storage(bus_, SfrIen1) & Ien1T1ie);
    return enabled || dma_armed(bus_);
}

void Timer1::ir_tick() {
    if (!unit_.c.external || unit_.c.mode == CountMode::Stopped)
        return;
    // Model-defined: the T1CTL.DIV prescaler also divides the IR tick.
    const unsigned div = static_cast<unsigned>(T1Dividers[(sfr_storage(bus_, SfrT1ctl) >> 2) & 3u]);
    if (++ir_prescale_ < div)
        return;
    ir_prescale_ = 0;
    const std::uint8_t before = outputs();
    handle_events(step(unit_));
    sync_storage();
    notify_outputs(before);
}

bool Timer1::output(unsigned channel) const {
    if (channel >= 5)
        return false;
    const bool level = unit_.ch[channel].out;
    // SWRU191F 9.9.2: with IRGEN the channel 1 pin carries Timer 1 channel 1
    // ANDed with Timer 3 channel 1.
    if (channel == 1 && unit_.c.external)
        return level && bus_.soc().timer34->output(3, 1);
    return level;
}

void Timer1::input_changed(unsigned channel, bool level) {
    if (channel >= 5)
        return;
    const std::uint8_t before = outputs();
    settle();
    Channel &ch = unit_.ch[channel];
    // SWRU191F 9.7: after the timer has been started, the selected edge
    // captures the counter. Model-defined: no capture while suspended or in
    // PM1-PM3 (no tick clock), nor with RFIRQ = 1 (not modelled).
    const unsigned cap = ch.ctl & 3u;
    const bool edge_selected = level ? (cap & 1u) : (cap & 2u);
    if ((ch.ctl & 0x04u) || (ch.ctl & 0x80u) || !edge_selected ||
        unit_.c.mode == CountMode::Stopped || hf_stopped(bus_.power_mode())) {
        notify_outputs(before);
        return;
    }
    ch.value = unit_.c.count;
    ch.has_pending = false;
    sync_storage();
    bus_.storage(sfr_reg(SfrT1stat)) |= static_cast<std::uint8_t>(1u << channel);
    if (ch.ctl & 0x40u)
        bus_.raise_irq(Irq::T1);
    notify_outputs(before);
    bus_.reschedule();
}

// ===========================================================================
// Timer 3 / Timer 4
// ===========================================================================

namespace {

enum class T34Kind { None, Cnt, Ctl, Cctl, Cc, Timif };
struct T34Reg {
    T34Kind kind = T34Kind::None;
    unsigned index = 0; // 0 = Timer 3, 1 = Timer 4
    unsigned n = 0;
};

T34Reg decode_t34(Reg reg) {
    if (!is_sfr(reg))
        return {};
    const std::uint8_t sfr = sfr_of(reg);
    if (sfr == SfrTimif)
        return {T34Kind::Timif, 0, 0};
    const unsigned index = sfr >= 0xe0 ? 1u : 0u;
    switch (static_cast<std::uint8_t>(sfr - index * 0x20u)) {
    case SfrT3cnt: return {T34Kind::Cnt, index, 0};
    case SfrT3ctl: return {T34Kind::Ctl, index, 0};
    case SfrT3cctl0: return {T34Kind::Cctl, index, 0};
    case SfrT3cc0: return {T34Kind::Cc, index, 0};
    case SfrT3cctl1: return {T34Kind::Cctl, index, 1};
    case SfrT3cc1: return {T34Kind::Cc, index, 1};
    default: return {};
    }
}

std::uint8_t t34_sfr(std::uint8_t timer3_sfr, unsigned index) {
    return static_cast<std::uint8_t>(timer3_sfr + index * 0x20u);
}
std::uint8_t t34_cc(unsigned n) { return n == 0 ? SfrT3cc0 : SfrT3cc1; }

} // namespace

Timer34::Timer34(Bus &bus) : Peripheral(bus) {
    for (unsigned index = 0; index < 2; ++index) {
        bus_.claim_sfr(t34_sfr(SfrT3cnt, index), this, true); // count is time-derived
        for (const std::uint8_t sfr : {SfrT3ctl, SfrT3cctl0, SfrT3cc0, SfrT3cctl1, SfrT3cc1})
            bus_.claim_sfr(t34_sfr(sfr, index), this, false);
    }
    bus_.claim_sfr(SfrTimif, this, false);
}

void Timer34::reset(ResetCause) {
    for (unsigned index = 0; index < 2; ++index) {
        Unit &unit = unit_[index];
        unit = Unit{};
        unit.c.max = 0xff;
        unit.channels = 2;
        unit.timer1 = false;
        // SWRU191F 10.8: TxCTL 0x08 (OVFIM = 1), TxCCTLn 0x40 (IM = 1).
        bus_.storage(sfr_reg(t34_sfr(SfrT3ctl, index))) = 0x08;
        bus_.storage(sfr_reg(t34_sfr(SfrT3cctl0, index))) = 0x40;
        bus_.storage(sfr_reg(t34_sfr(SfrT3cctl1, index))) = 0x40;
        unit.c.period = tick_period(index);
        unit.c.anchor = bus_.hf_now();
    }
    // SWRU191F 10.8: TIMIF 0x40 (T1OVFIM = 1).
    bus_.storage(sfr_reg(SfrTimif)) = TimifT1ovfim;
}

std::uint64_t Timer34::tick_period(unsigned index) const {
    const unsigned tickspd = (bus_.clkconsta() >> 3) & 7u;
    const unsigned div = sfr_storage(bus_, t34_sfr(SfrT3ctl, index)) >> 5;
    return std::uint64_t{1} << (tickspd + div);
}

void Timer34::handle_events(unsigned index, std::uint8_t events) {
    if (events == 0)
        return;
    // SWRU191F 10.6: TIMIF bits 2:0 (Timer 3) and 5:3 (Timer 4) are OVFIF,
    // CH0IF, CH1IF; IRCON.TxIF on an event whose mask is 1.
    const unsigned shift = index * 3;
    const std::uint8_t ctl = sfr_storage(bus_, t34_sfr(SfrT3ctl, index));
    std::uint8_t flags = 0;
    bool request = false;
    if (events & Timer1::EventOverflow) {
        flags |= 1u;
        request = request || (ctl & 0x08u);
    }
    for (unsigned n = 0; n < 2; ++n) {
        if (events & (1u << n)) {
            flags |= static_cast<std::uint8_t>(2u << n);
            request = request || (unit_[index].ch[n].ctl & 0x40u);
        }
    }
    bus_.storage(sfr_reg(SfrTimif)) |= static_cast<std::uint8_t>(flags << shift);
    if (request)
        bus_.raise_irq(index == 0 ? Irq::T3 : Irq::T4);
    // SWRU191F 10.7: T3_CH0/T3_CH1/T4_CH0/T4_CH1 on capture or compare.
    for (unsigned n = 0; n < 2; ++n) {
        if (events & (1u << n)) {
            ++dma_triggers_[index * 2 + n];
            bus_.dma_trigger((index == 0 ? trigger::T3Ch0 : trigger::T4Ch0) + n);
        }
    }
    // SWRU191F 9.9.2: Timer 3 channel 1 compare is the IR-mode T1 tick.
    if (index == 0 && (events & 0x02u))
        bus_.soc().timer1->ir_tick();
}

void Timer34::sync_storage() {
    for (unsigned index = 0; index < 2; ++index)
        for (unsigned n = 0; n < 2; ++n)
            bus_.storage(sfr_reg(t34_sfr(t34_cc(n), index))) =
                static_cast<std::uint8_t>(unit_[index].ch[n].value);
}

void Timer34::settle() {
    for (unsigned index = 0; index < 2; ++index)
        Timer1::advance(unit_[index], bus_.hf_now(),
                        [this, index](std::uint8_t events) { handle_events(index, events); });
    sync_storage();
}

std::uint8_t Timer34::outputs() const {
    std::uint8_t bits = 0;
    for (unsigned index = 0; index < 2; ++index)
        for (unsigned n = 0; n < 2; ++n)
            if (unit_[index].ch[n].out)
                bits |= static_cast<std::uint8_t>(1u << (index * 2 + n));
    return bits;
}

void Timer34::notify_outputs(std::uint8_t before) {
    if (outputs() != before)
        bus_.soc().io->peripheral_outputs_changed();
}

std::uint8_t Timer34::read(Reg reg) {
    const T34Reg r = decode_t34(reg);
    if (r.kind == T34Kind::Cnt) {
        const std::uint8_t before = outputs();
        settle();
        notify_outputs(before);
        return static_cast<std::uint8_t>(unit_[r.index].c.count);
    }
    return peek(reg);
}

std::uint8_t Timer34::peek(Reg reg) const {
    const T34Reg r = decode_t34(reg);
    if (r.kind == T34Kind::Cnt) {
        Unit copy = unit_[r.index];
        Timer1::advance(copy, bus_.hf_now(), [](std::uint8_t) {});
        return static_cast<std::uint8_t>(copy.c.count);
    }
    return bus_.storage(reg);
}

void Timer34::write(Reg reg, std::uint8_t value) {
    using CountMode = Timer1::CountMode;
    const std::uint8_t before = outputs();
    settle();
    const T34Reg r = decode_t34(reg);
    switch (r.kind) {
    case T34Kind::Cnt:
        break; // SWRU191F 10.8: TxCNT is read-only
    case T34Kind::Ctl: {
        // SWRU191F 10.8 TxCTL: DIV 7:5, START 4, OVFIM 3, CLR 2 (R0/W1),
        // MODE 1:0 (free-running, down, modulo, up-and-down).
        Unit &unit = unit_[r.index];
        Timer1::Counter &c = unit.c;
        static constexpr CountMode Modes[4] = {CountMode::Free, CountMode::Down,
                                               CountMode::Modulo, CountMode::UpDown};
        const CountMode mode = (value & 0x10u) ? Modes[value & 3u] : CountMode::Stopped;
        bus_.storage(reg) = value & static_cast<std::uint8_t>(~0x04u);
        const std::uint64_t period = tick_period(r.index);
        const bool starting = c.mode == CountMode::Stopped && mode != CountMode::Stopped;
        if (value & 0x04u) {
            // SWRU191F 10.2.4, 10.5: CLR clears the count, sets the
            // direction to up and initializes all channel outputs.
            c.count = 0;
            c.down = false;
            c.anchor = bus_.hf_now();
            Timer1::load_pending(unit);
            for (unsigned n = 0; n < 2; ++n)
                Timer1::initialize_output(unit.ch[n]);
        }
        // Model-defined: starting or changing the prescaler restarts the
        // tick phase at the write.
        if (starting || period != c.period)
            c.anchor = bus_.hf_now();
        if (mode == CountMode::Down && (starting || c.mode != CountMode::Down)) {
            // SWRU191F 10.2.2: once started, the counter is loaded from TxCC0.
            c.count = unit.ch[0].value;
            c.halted = false;
        }
        if (mode != CountMode::Down)
            c.halted = false;
        if (mode == CountMode::UpDown && c.mode != CountMode::UpDown)
            c.down = false;
        c.mode = mode;
        c.period = period;
        break;
    }
    case T34Kind::Cctl: {
        Timer1::Channel &ch = unit_[r.index].ch[r.n];
        std::uint8_t ctl = value & 0x7fu; // bit 7 R0
        if (((ctl >> 3) & 7u) == 7u) {
            // SWRU191F Table 10-1: CMP = 111 initializes the output.
            ctl = static_cast<std::uint8_t>((ctl & ~0x38u) | (ch.ctl & 0x38u));
            ch.ctl = ctl;
            Timer1::initialize_output(ch);
        }
        ch.ctl = ctl;
        bus_.storage(reg) = ctl;
        break;
    }
    case T34Kind::Cc: {
        // SWRU191F 10.8 TxCCn: in compare mode the value takes effect when
        // TxCNT = 0x00 (at once if the count is 0 now).
        Unit &unit = unit_[r.index];
        Timer1::Channel &ch = unit.ch[r.n];
        if ((ch.ctl & 0x04u) && unit.c.count != 0) {
            ch.pending = value;
            ch.has_pending = true;
        } else {
            ch.value = value;
            ch.has_pending = false;
        }
        break;
    }
    case T34Kind::Timif: {
        // SWRU191F 10.8 TIMIF: bit 7 R0, T1OVFIM R/W, flags R/W0.
        std::uint8_t &timif = bus_.storage(reg);
        const std::uint8_t old = timif;
        timif = static_cast<std::uint8_t>((old & value & 0x3fu) | (value & TimifT1ovfim));
        const std::uint8_t cleared = old & static_cast<std::uint8_t>(~timif) & 0x3fu;
        // SWRU191F 10.6: clearing a flag while another flag of the same
        // timer with its mask set is pending sets IRCON.TxIF again.
        for (unsigned index = 0; index < 2; ++index) {
            const unsigned shift = index * 3;
            std::uint8_t masks = 0;
            if (sfr_storage(bus_, t34_sfr(SfrT3ctl, index)) & 0x08u)
                masks |= 1u;
            for (unsigned n = 0; n < 2; ++n)
                if (unit_[index].ch[n].ctl & 0x40u)
                    masks |= static_cast<std::uint8_t>(2u << n);
            if (((cleared >> shift) & 7u) != 0 && ((timif >> shift) & masks) != 0)
                bus_.raise_irq(index == 0 ? Irq::T3 : Irq::T4);
        }
        break;
    }
    case T34Kind::None:
        bus_.storage(reg) = value;
        break;
    }
    sync_storage();
    notify_outputs(before);
}

std::uint64_t Timer34::deadline() const {
    return std::min(Timer1::unit_deadline(bus_, unit_[0]), Timer1::unit_deadline(bus_, unit_[1]));
}

void Timer34::service(std::uint64_t) {
    const std::uint8_t before = outputs();
    settle();
    notify_outputs(before);
}

void Timer34::power_mode_changed(PowerMode, PowerMode) { bus_.reschedule(); }

void Timer34::cpu_flags_written() {
    // IRCON.T3IF/T4IF are set on events and on TIMIF clears (SWRU191F 10.6).
}

void Timer34::clock_changed() {
    const std::uint8_t before = outputs();
    settle();
    for (unsigned index = 0; index < 2; ++index) {
        const std::uint64_t period = tick_period(index);
        if (period != unit_[index].c.period) {
            unit_[index].c.period = period;
            unit_[index].c.anchor = bus_.hf_now();
        }
    }
    notify_outputs(before);
}

bool Timer34::may_interrupt() const {
    const bool ea = sfr_storage(bus_, SfrIen0) & Ien0Ea;
    const std::uint8_t ien1 = sfr_storage(bus_, SfrIen1);
    const bool dma = dma_armed(bus_);
    for (unsigned index = 0; index < 2; ++index) {
        if (Timer1::unit_deadline(bus_, unit_[index]) == Never)
            continue;
        if (dma || (ea && (ien1 & (index == 0 ? Ien1T3ie : Ien1T4ie))))
            return true;
        // Timer 3 channel 1 compares tick Timer 1 in IR generation mode.
        if (index == 0 && (bus_.storage(XregIrctl) & 1u) && ea && (ien1 & Ien1T1ie))
            return true;
    }
    return false;
}

bool Timer34::output(unsigned timer, unsigned channel) const {
    if ((timer != 3 && timer != 4) || channel >= 2)
        return false;
    return unit_[timer - 3].ch[channel].out;
}

void Timer34::input_changed(unsigned timer, unsigned channel, bool level) {
    if ((timer != 3 && timer != 4) || channel >= 2)
        return;
    const unsigned index = timer - 3;
    const std::uint8_t before = outputs();
    settle();
    Unit &unit = unit_[index];
    Timer1::Channel &ch = unit.ch[channel];
    // SWRU191F 10.4. Model-defined: no capture while the timer is stopped or
    // in PM1-PM3.
    const unsigned cap = ch.ctl & 3u;
    const bool edge_selected = level ? (cap & 1u) : (cap & 2u);
    if ((ch.ctl & 0x04u) || !edge_selected || unit.c.mode == Timer1::CountMode::Stopped ||
        hf_stopped(bus_.power_mode())) {
        notify_outputs(before);
        return;
    }
    ch.value = static_cast<std::uint8_t>(unit.c.count);
    ch.has_pending = false;
    sync_storage();
    handle_events(index, static_cast<std::uint8_t>(1u << channel));
    notify_outputs(before);
    bus_.reschedule();
}

// ===========================================================================
// Sleep timer
// ===========================================================================

SleepTimer::SleepTimer(Bus &bus) : Peripheral(bus) {
    bus_.claim_sfr(SfrSt0, this, true); // count is time-derived; latches ST1/ST2
    bus_.claim_sfr(SfrSt1, this, false);
    bus_.claim_sfr(SfrSt2, this, false);
    bus_.claim_sfr(SfrStload, this, false);
    for (std::uint16_t xreg = XregStcc; xreg <= XregStcv0 + 2; ++xreg)
        bus_.claim_xreg(xreg, this);
}

void SleepTimer::reset(ResetCause) {
    // SWRU191F 11.1: the timer starts running immediately after a reset.
    // Model-defined: every reset restarts the count at 0.
    const std::uint64_t lf = bus_.lf_now();
    base_ = lf_ticks(lf);
    next_edge_ = first_edge_after(lf);
    compare_ = StMask; // SWRU191F 11.2: compare value 0xFFFFFF after reset
    write_st1_ = write_st2_ = 0;
    loading_ = capturing_ = stale_ = false;
    load_value_ = capture_value_ = stale_value_ = 0;
    load_edge_ = capture_edge_ = stale_edge_ = 0;
    bus_.storage(sfr_reg(SfrStload)) = 0x01; // LDRDY
    bus_.storage(XregStcc) = 0x1f;           // PORT = 3, PIN = 7: capture disabled
}

std::uint32_t SleepTimer::count_at(std::uint64_t lf) const {
    return static_cast<std::uint32_t>((lf_ticks(lf) - base_) & StMask);
}

std::uint32_t SleepTimer::count() const { return count_at(bus_.lf_now()); }

// SWRU191F 11.1: after PM1/PM2 the ST2:ST1:ST0 value is not up to date
// until a positive 32 kHz edge has been seen. Model-defined: until then the
// registers show the count at the power mode entry.
std::uint32_t SleepTimer::visible_count() const {
    const std::uint64_t lf = bus_.lf_now();
    if (stale_ && lf < rising_edge_lf(stale_edge_))
        return stale_value_;
    return count_at(lf);
}

// SWRU191F 11.2: a compare event happens on a positive 32 kHz edge while the
// count equals the compare value. The count of edge e is e - base_.
std::uint64_t SleepTimer::compare_edge() const {
    const std::uint64_t from = next_edge_;
    const std::uint64_t ahead = (compare_ - ((from - base_) & StMask)) & StMask;
    return from + ahead;
}

std::uint64_t SleepTimer::next_event_edge() const {
    std::uint64_t edge = compare_edge();
    if (loading_)
        edge = std::min(edge, load_edge_);
    if (capturing_)
        edge = std::min(edge, capture_edge_);
    return edge;
}

void SleepTimer::process_edge(std::uint64_t edge) {
    next_edge_ = edge + 1;
    if (loading_ && load_edge_ == edge) {
        // Model-defined: a load takes until the next positive 32 kHz edge;
        // the new value is compared from that edge on.
        compare_ = load_value_;
        loading_ = false;
        bus_.storage(sfr_reg(SfrStload)) = 0x01;
    }
    if (capturing_ && capture_edge_ == edge) {
        // SWRU191F 11.3: the capture is detected by the 32 kHz clock.
        capturing_ = false;
        bus_.storage(XregStcv0) = static_cast<std::uint8_t>(capture_value_);
        bus_.storage(XregStcv0 + 1) = static_cast<std::uint8_t>(capture_value_ >> 8);
        bus_.storage(XregStcv0 + 2) = static_cast<std::uint8_t>(capture_value_ >> 16);
        bus_.storage(XregStcs) |= 0x01u;
    }
    if (((edge - base_) & StMask) == compare_) {
        // SWRU191F 11.2: IRCON.STIF and DMA trigger 11 (ST).
        ++dma_triggers_;
        bus_.dma_trigger(trigger::St);
        bus_.raise_irq(Irq::St);
    }
}

void SleepTimer::settle() {
    const std::uint64_t lf = bus_.lf_now();
    for (;;) {
        const std::uint64_t edge = next_event_edge();
        if (rising_edge_lf(edge) > lf)
            break;
        process_edge(edge);
    }
    // Edges up to `lf` without events need no processing.
    next_edge_ = std::max(next_edge_, first_edge_after(lf));
}

std::uint8_t SleepTimer::read(Reg reg) {
    if (reg == sfr_reg(SfrSt0)) {
        // SWRU191F 11.1: reading ST0 latches the count into ST1/ST2.
        settle();
        const std::uint32_t value = visible_count();
        bus_.storage(sfr_reg(SfrSt1)) = static_cast<std::uint8_t>(value >> 8);
        bus_.storage(sfr_reg(SfrSt2)) = static_cast<std::uint8_t>(value >> 16);
        return static_cast<std::uint8_t>(value);
    }
    return peek(reg);
}

std::uint8_t SleepTimer::peek(Reg reg) const {
    if (reg == sfr_reg(SfrSt0))
        return static_cast<std::uint8_t>(visible_count());
    return bus_.storage(reg);
}

namespace {
// SWRU191F 11.4 STCC: PORT 0-2, PIN 0-7 (ports 0, 1) or 0-5 (port 2).
bool capture_selected(std::uint8_t stcc) {
    const unsigned port = (stcc >> 3) & 3u, pin = stcc & 7u;
    return port < 2 || (port == 2 && pin <= 5);
}
} // namespace

void SleepTimer::write(Reg reg, std::uint8_t value) {
    settle();
    switch (reg) {
    case sfr_reg(SfrSt0):
        // SWRU191F 11.2, 11.4: an ST0 write with LDRDY = 1 loads
        // ST2:ST1:ST0 as the compare value; ignored while LDRDY = 0.
        if (bus_.storage(sfr_reg(SfrStload)) & 0x01u) {
            load_value_ = static_cast<std::uint32_t>(write_st2_) << 16 |
                          static_cast<std::uint32_t>(write_st1_) << 8 | value;
            loading_ = true;
            load_edge_ = first_edge_after(bus_.lf_now());
            bus_.storage(sfr_reg(SfrStload)) = 0x00;
        }
        break;
    case sfr_reg(SfrSt1):
        write_st1_ = value; // read value stays the latched count
        break;
    case sfr_reg(SfrSt2):
        write_st2_ = value;
        break;
    case sfr_reg(SfrStload):
        break; // R0 / R
    case XregStcc: {
        // SWRU191F 11.3: the capture pin cannot be switched while capture
        // is enabled; selecting PORT = 3 (or an invalid pin) disables it.
        // Model-defined: such a switch is ignored. STCC is documented as
        // R, but 11.3 requires firmware to write PORT and PIN.
        std::uint8_t &stcc = bus_.storage(reg);
        const auto next = static_cast<std::uint8_t>(value & 0x1fu);
        if (capture_selected(stcc) && capture_selected(next) && next != stcc)
            break;
        stcc = next;
        if (!capture_selected(stcc))
            capturing_ = false;
        break;
    }
    case XregStcs: {
        std::uint8_t &stcs = bus_.storage(reg); // VALID R/W0
        stcs = static_cast<std::uint8_t>(stcs & value & 0x01u);
        break;
    }
    default:
        break; // STCV0-2 read-only
    }
}

std::uint64_t SleepTimer::deadline() const {
    return bus_.lf_deadline(rising_edge_lf(next_event_edge()));
}

void SleepTimer::service(std::uint64_t) { settle(); }

void SleepTimer::power_mode_changed(PowerMode from, PowerMode to) {
    if (to == PowerMode::Pm1 || to == PowerMode::Pm2 || to == PowerMode::Pm3) {
        settle();
        stale_value_ = visible_count();
    } else if (to == PowerMode::Active && (from == PowerMode::Pm1 || from == PowerMode::Pm2)) {
        settle();
        stale_ = true;
        stale_edge_ = first_edge_after(bus_.lf_now());
    } else if (to == PowerMode::Active && from == PowerMode::Pm3) {
        // SWRU191F 11.1: the timer does not run in PM3 and its value is not
        // preserved. Model-defined: it restarts from 0 at the wake.
        const std::uint64_t lf = bus_.lf_now();
        base_ = lf_ticks(lf);
        next_edge_ = first_edge_after(lf);
        load_edge_ = std::max(load_edge_, next_edge_);
        capture_edge_ = std::max(capture_edge_, next_edge_);
        stale_ = false;
    }
    bus_.reschedule();
}

bool SleepTimer::may_interrupt() const {
    if (deadline() == Never)
        return false;
    const std::uint8_t ien0 = sfr_storage(bus_, SfrIen0);
    return ((ien0 & Ien0Ea) && (ien0 & Ien0Stie)) || dma_armed(bus_);
}

void SleepTimer::input_changed(unsigned port, unsigned pin, bool level) {
    settle();
    const std::uint8_t stcc = bus_.storage(XregStcc);
    if (!capture_selected(stcc) || port != ((stcc >> 3) & 3u) || pin != (stcc & 7u))
        return;
    if ((bus_.storage(XregStcs) & 0x01u) || capturing_ || bus_.power_mode() == PowerMode::Pm3)
        return;
    // SWRU191F 7.x PICTL: the pin's interrupt edge (0 rising, 1 falling)
    // sets the pin flag that triggers the capture. Model-defined: the edge
    // itself triggers it (PxIFG is not consulted).
    const std::uint8_t pictl = bus_.xdata_peek(XdataPictl);
    const unsigned bit = port == 0 ? 0u : port == 1 ? (pin < 4 ? 1u : 2u) : 3u;
    const bool falling = (pictl >> bit) & 1u;
    if (level == falling)
        return;
    // SWRU191F 11.3: the captured value is 1 more than the count at the
    // event; it becomes valid on the next positive 32 kHz edge.
    const std::uint64_t lf = bus_.lf_now();
    capture_value_ = (count_at(lf) + 1) & StMask;
    capture_edge_ = first_edge_after(lf);
    capturing_ = true;
    bus_.reschedule();
}

// ===========================================================================
// Watchdog timer
// ===========================================================================

Watchdog::Watchdog(Bus &bus) : Peripheral(bus) { bus_.claim_sfr(SfrWdctl, this, false); }

void Watchdog::reset(ResetCause) {
    // SWRU191F 16.1: the WDT is disabled after a system reset.
    mode_ = WdtMode::Idle;
    interval_ = 0;
    count_ = 0;
    last_tick_ = lf_ticks(bus_.lf_now());
    frozen_ = false;
    armed_ = false;
    armed_lf_ = 0;
    reset_requested_ = false;
}

// SWRU191F 16.3 WDCTL.INT: 32768, 8192, 512 or 64 periods of the 32 kHz
// clock, reduced by (oscillator frequency / CLKSPD clock) for the CC2530.
// Model-defined: a reduced interval below one period is one period.
std::uint64_t Watchdog::limit() const {
    static constexpr std::uint64_t Intervals[4] = {32768, 8192, 512, 64};
    const std::uint8_t clkconsta = bus_.clkconsta();
    const unsigned clkspd = clkconsta & 7u;
    const unsigned osc = (clkconsta >> 6) & 1u; // 1: 16 MHz RCOSC
    const unsigned shift = clkspd > osc ? clkspd - osc : 0;
    return std::max<std::uint64_t>(1, Intervals[interval_] >> shift);
}

void Watchdog::settle() {
    const std::uint64_t tick = lf_ticks(bus_.lf_now());
    if (mode_ != WdtMode::Idle && !frozen_)
        count_ += tick - last_tick_;
    last_tick_ = tick;
}

std::uint8_t Watchdog::peek(Reg reg) const { return bus_.storage(reg); }

void Watchdog::write(Reg reg, std::uint8_t value) {
    settle();
    const unsigned clr = value >> 4;
    const unsigned mode = (value >> 2) & 3u;
    std::uint8_t &wdctl = bus_.storage(reg);
    switch (mode_) {
    case WdtMode::Idle:
        // SWRU191F 16.3: CLR writes have no effect while IDLE; MODE 10
        // starts watchdog mode, 11 timer mode, counting from 0 with INT.
        wdctl = value & 0x0fu; // CLR reads 0
        if (mode == 1)
            bus_.note_unmodeled(reg, true); // reserved: stays stopped
        if (mode >= 2) {
            mode_ = mode == 2 ? WdtMode::Watchdog : WdtMode::Timer;
            interval_ = value & 3u;
            count_ = 0;
            armed_ = false;
        }
        break;
    case WdtMode::Watchdog: {
        // SWRU191F 16.1: MODE and INT cannot be changed. The counter is
        // cleared by 0xA then 0x5 in CLR within one 32 kHz period.
        // Model-defined: any other CLR value written in between (including
        // 0x0) cancels the sequence.
        const std::uint64_t lf = bus_.lf_now();
        if (clr == 0xa) {
            armed_ = true;
            armed_lf_ = lf;
        } else {
            if (clr == 0x5 && armed_ && (lf - armed_lf_) * 16 <= 15625)
                count_ = 0;
            armed_ = false;
        }
        break;
    }
    case WdtMode::Timer:
        // SWRU191F 16.2, 16.3: MODE 00 stops and clears; CLR[0] clears;
        // INT cannot be changed while running. Model-defined: MODE 10 is
        // ignored (16.3: stop first), reserved MODE 01 stops like IDLE.
        if (mode < 2) {
            if (mode == 1)
                bus_.note_unmodeled(reg, true);
            mode_ = WdtMode::Idle;
            count_ = 0;
            wdctl = value & 0x0fu;
        } else if (clr & 1u) {
            count_ = 0;
        }
        break;
    }
}

std::uint64_t Watchdog::deadline() const {
    if (mode_ == WdtMode::Idle || frozen_ || reset_requested_)
        return Never;
    const std::uint64_t lim = limit();
    const std::uint64_t tick = count_ >= lim ? last_tick_ + 1 : last_tick_ + (lim - count_);
    return bus_.lf_deadline(lf_tick_cycle(tick));
}

void Watchdog::service(std::uint64_t) {
    settle();
    if (mode_ == WdtMode::Idle || frozen_ || count_ < limit())
        return;
    if (mode_ == WdtMode::Watchdog) {
        // SWRU191F 16.1: the interval elapsed without a clear: reset.
        reset_requested_ = true;
        bus_.request_reset(ResetCause::Watchdog);
        return;
    }
    // SWRU191F 16.2: timer mode requests IRCON2.WDTIF each interval.
    count_ = 0;
    bus_.raise_irq(Irq::Wdt);
}

void Watchdog::power_mode_changed(PowerMode, PowerMode to) {
    // SWRU191F 16: the counter is retained in all power modes and continues
    // counting in active mode. Model-defined: it counts in active mode and
    // Idle (PM0), stops in PM1-PM3 and resumes at the next period boundary.
    const bool stop = hf_stopped(to);
    if (stop && !frozen_) {
        settle();
        frozen_ = true;
    } else if (!stop && frozen_) {
        frozen_ = false;
        last_tick_ = lf_ticks(bus_.lf_now());
    }
    bus_.reschedule();
}

void Watchdog::clock_changed() {
    // The interval depends on CLKSPD; the count so far is kept.
    settle();
}

bool Watchdog::may_interrupt() const {
    if (deadline() == Never)
        return false;
    if (mode_ == WdtMode::Watchdog)
        return true; // will reset the chip
    return (sfr_storage(bus_, SfrIen0) & Ien0Ea) && (sfr_storage(bus_, SfrIen2) & Ien2Wdtie);
}

} // namespace mcs51emu::cc2530
