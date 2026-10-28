// CC2530 Timer 2 (MAC timer), SWRU191F Chapter 22.
//
// The 16-bit up-counter t2tim increments on every 32 MHz cycle of the HF
// clock domain (Bus::hf_now, frozen in PM1-PM3). Its value is computed
// arithmetically from an anchor; the scheduler is woken exactly at the next
// period wrap or compare match, so every event is processed with its own
// timestamp. The 24-bit overflow counter t2ovf changes only at period
// wraps.

#include "cc2530_mactimer.hpp"

#include "cc2530_radio.hpp"
#include "cc2530_timers.hpp"

namespace mcs51emu::cc2530 {

namespace {

constexpr std::uint8_t T2CTRL = 0x94, T2EVTCFG = 0x9C, T2IRQF = 0xA1, T2M0 = 0xA2, T2M1 = 0xA3,
                       T2MOVF0 = 0xA4, T2MOVF1 = 0xA5, T2MOVF2 = 0xA6, T2IRQM = 0xA7,
                       T2MSEL = 0xC3;

// T2CTRL bits (22.5).
constexpr std::uint8_t CtrlRun = 0x01, CtrlSync = 0x02, CtrlState = 0x04, CtrlLatch = 0x08;
// T2IRQF / T2IRQM bits (22.5).
constexpr std::uint8_t FlagPer = 0x01, FlagCmp1 = 0x02, FlagCmp2 = 0x04, FlagOvfPer = 0x08,
                       FlagOvfCmp1 = 0x10, FlagOvfCmp2 = 0x20, FlagMask = 0x3F;
// T2EVTCFG event codes (22.5): the event number equals the flag bit index.
constexpr unsigned EventNone = 7;

constexpr std::uint32_t OvfMask = 0xFFFFFF;
// SWRU191F 22.4.3: TOH, cycles from the sampled 32 kHz edge to RUN mode.
constexpr std::uint64_t SyncOverhead = 86;

// Model-defined: a timer period t2_per = 0 makes the timer wrap (PERF) when
// it would count from 0xFFFF to 0x0000 = t2_per, i.e. a 65536-cycle period;
// SWRU191F 22.1.3 defines the wrap as "counts to a value equal to the
// period", which for 0 happens only at the 16-bit roll-over. Likewise
// t2ovf_per = 0 gives a 2^24 overflow period (22.1.8).
//
// Model-defined: when the counter is at or above a nonzero period (written
// while idle or a period lowered below the count) it counts up to 0xFFFF and
// rolls over to 0 without a period event, then wraps at the period.

// Value after `k` cycles of counting from `v`.
std::uint16_t advance(std::uint16_t v, std::uint64_t k, std::uint16_t per) {
    if (per == 0)
        return static_cast<std::uint16_t>((v + k) & 0xFFFF);
    if (v < per)
        return static_cast<std::uint16_t>((v + k) % per);
    const std::uint64_t to_roll = 0x10000u - v;
    if (k < to_roll)
        return static_cast<std::uint16_t>(v + k);
    return static_cast<std::uint16_t>((k - to_roll) % per);
}

// Cycles from `v` until the period wrap (TIMER2_PERF).
std::uint64_t cycles_to_wrap(std::uint16_t v, std::uint16_t per) {
    if (per == 0)
        return 0x10000u - v;
    if (v < per)
        return static_cast<std::uint64_t>(per) - v;
    return 0x10000u - v + per;
}

// Cycles (>= 1) from `v` until the counter counts to `x`, or Never.
// Model-defined: the counter "counts to" a compare value whenever it takes
// that value by counting, including 0 reached by a period wrap or by the
// roll-over (SWRU191F 22.1.5 does not distinguish); a compare value the
// counter never takes (>= nonzero period) never matches.
std::uint64_t cycles_to_value(std::uint16_t v, std::uint16_t x, std::uint16_t per) {
    if (per == 0) {
        const std::uint64_t d = static_cast<std::uint16_t>(x - v);
        return d == 0 ? 0x10000u : d;
    }
    if (v < per) {
        if (x >= per)
            return Never;
        const std::uint64_t d = (static_cast<std::uint64_t>(x) + per - v) % per;
        return d == 0 ? per : d;
    }
    if (x > v)
        return static_cast<std::uint64_t>(x) - v;
    if (x < per)
        return 0x10000u - v + x;
    return Never;
}

bool in_sleep(PowerMode mode) {
    return mode == PowerMode::Pm1 || mode == PowerMode::Pm2 || mode == PowerMode::Pm3;
}

bool rf_loop_active = false;

} // namespace

void advance_rf_domain(Soc &soc, std::uint64_t hf) {
    // Not re-entrant: a nested call (e.g. a DMA transfer started from an
    // event reading RFD) sees the state of the event being processed.
    if (rf_loop_active)
        return;
    rf_loop_active = true;
    for (;;) {
        const std::uint64_t t = soc.mac_timer->next_event_hf();
        const std::uint64_t r = soc.radio->next_event_hf();
        if (t <= r) {
            if (t > hf)
                break;
            soc.mac_timer->process_next_event();
        } else {
            if (r > hf)
                break;
            soc.radio->process_next_event();
        }
    }
    rf_loop_active = false;
}

MacTimer::MacTimer(Bus &bus) : Peripheral(bus) {
    bus_.claim_sfr(T2CTRL, this, true);
    bus_.claim_sfr(T2EVTCFG, this, false);
    bus_.claim_sfr(T2IRQF, this, true); // RMW snapshot (R/W0 rule)
    bus_.claim_sfr(T2M0, this, true);
    bus_.claim_sfr(T2M1, this, true);
    bus_.claim_sfr(T2MOVF0, this, true);
    bus_.claim_sfr(T2MOVF1, this, true);
    bus_.claim_sfr(T2MOVF2, this, true);
    bus_.claim_sfr(T2IRQM, this, false);
    bus_.claim_sfr(T2MSEL, this, false);
}

void MacTimer::reset(ResetCause) {
    // SWRU191F 22.5 / Table 22-1: every register and internal register
    // resets to 0 except T2CTRL.SYNC = 1; the timer is IDLE (22.1.1).
    phase_ = Phase::Idle;
    anchor_ = count_from_ = bus_.hf_now();
    tim_ = 0;
    ovf_ = 0;
    per_ = cmp1_ = cmp2_ = cap_ = 0;
    ovf_per_ = ovf_cmp1_ = ovf_cmp2_ = ovf_cap_ = 0;
    tim_high_latch_ = 0;
    ovf_latch_ = 0;
    m0_buffer_ = 0;
    movf_buffer_[0] = movf_buffer_[1] = 0;
    irqf_ = 0;
    rmw_time_ = Never;
    rmw_snapshot_ = 0;
    edge_hf_ = load_hf_ = Never;
    load_tim_ = 0;
    load_ovf_ = 0;
    stored_valid_ = false;
    stored_sleep_ = 0;
    pulses_[0] = pulses_[1] = 0;
    segment_size_ = 0;
    note_segment(anchor_);
    bus_.storage(sfr_reg(T2CTRL)) = CtrlSync;
}

std::uint64_t MacTimer::to_now(std::uint64_t hf) const {
    // Valid for hf <= hf_now(): the HF clock ran continuously since `hf`
    // because pending events are processed before every power-mode change.
    return bus_.now() - (bus_.hf_now() - hf);
}

void MacTimer::catch_up() { advance_rf_domain(bus_.soc(), bus_.hf_now()); }

std::uint16_t MacTimer::timer_at(std::uint64_t hf) const {
    if (!counting() || hf <= count_from_)
        return tim_;
    return advance(tim_, hf - count_from_, per_);
}

void MacTimer::rebase(std::uint64_t hf) {
    if (counting() && hf > count_from_) {
        tim_ = advance(tim_, hf - count_from_, per_);
        count_from_ = hf;
    }
    if (hf > anchor_)
        anchor_ = hf;
}

std::uint64_t MacTimer::next_timer_event() const {
    if (!counting())
        return Never;
    std::uint64_t d = cycles_to_wrap(tim_, per_);
    const std::uint64_t c1 = cycles_to_value(tim_, cmp1_, per_);
    const std::uint64_t c2 = cycles_to_value(tim_, cmp2_, per_);
    if (c1 < d)
        d = c1;
    if (c2 < d)
        d = c2;
    return count_from_ + d;
}

std::uint64_t MacTimer::next_event_hf() const {
    std::uint64_t t = next_timer_event();
    if (phase_ == Phase::StartEdge || phase_ == Phase::StopEdge)
        t = edge_hf_ < t ? edge_hf_ : t;
    if (phase_ == Phase::Starting)
        t = load_hf_ < t ? load_hf_ : t;
    return t;
}

void MacTimer::process_next_event() {
    const std::uint64_t timer = next_timer_event();
    const std::uint64_t edge =
        (phase_ == Phase::StartEdge || phase_ == Phase::StopEdge) ? edge_hf_ : Never;
    const std::uint64_t load = phase_ == Phase::Starting ? load_hf_ : Never;
    if (timer != Never && timer <= edge && timer <= load) {
        timer_event(timer);
        note_segment(timer);
    } else if (edge != Never && edge <= load) {
        edge_event(edge);
        note_segment(edge);
    } else if (load != Never) {
        start_loaded(load);
        note_segment(load);
    }
}

void MacTimer::note_segment(std::uint64_t since) {
    if (segment_size_ == 0 || segments_[segment_last_].since != since) {
        segment_last_ = (segment_last_ + 1) % SegmentCount;
        if (segment_size_ < SegmentCount)
            ++segment_size_;
    }
    segments_[segment_last_] = {since, count_from_, ovf_, tim_, per_, counting()};
}

void MacTimer::timer_event(std::uint64_t hf) {
    const std::uint64_t k = hf - count_from_;
    const bool wrap = k == cycles_to_wrap(tim_, per_);
    tim_ = advance(tim_, k, per_);
    count_from_ = anchor_ = hf;
    std::uint8_t bits = 0;
    if (wrap) {
        // SWRU191F 22.1.3/22.1.6/22.1.8: period wrap, overflow count + 1,
        // overflow period wrap to 0 and overflow compares.
        bits |= FlagPer;
        const std::uint32_t next = (ovf_ + 1) & OvfMask;
        if (next == ovf_per_) {
            ovf_ = 0;
            bits |= FlagOvfPer;
        } else {
            ovf_ = next;
        }
        if (ovf_ == ovf_cmp1_)
            bits |= FlagOvfCmp1;
        if (ovf_ == ovf_cmp2_)
            bits |= FlagOvfCmp2;
    }
    if (tim_ == cmp1_)
        bits |= FlagCmp1;
    if (tim_ == cmp2_)
        bits |= FlagCmp2;
    set_flags(bits);
    if (wrap) // SWRU191F 23.14.9 WAIT/WAITX, 23.14.2 CSPT: MAC timer overflow.
        bus_.soc().radio->mac_timer_overflow(hf);
    for (unsigned e = 0; e < 6; ++e)
        if (bits & (1u << e))
            fire(e, hf);
}

void MacTimer::fire(unsigned event, std::uint64_t hf) {
    // SWRU191F 22.3: T2_EVENT1/T2_EVENT2 pulse on the configured event;
    // Table 8-1 DMA triggers 5/6; 23.14.9 WEVENT1/WEVENT2.
    const std::uint8_t cfg = bus_.storage(sfr_reg(T2EVTCFG));
    for (unsigned which = 1; which <= 2; ++which) {
        const unsigned code = which == 1 ? (cfg & 7u) : ((cfg >> 4) & 7u);
        if (code == EventNone || code != event)
            continue;
        ++pulses_[which - 1];
        bus_.dma_trigger(which == 1 ? trigger::T2Event1 : trigger::T2Event2);
        bus_.soc().radio->mac_timer_event(which, hf);
    }
}

void MacTimer::set_flags(std::uint8_t bits) {
    if (!bits)
        return;
    const std::uint8_t mask = bus_.storage(sfr_reg(T2IRQM));
    const std::uint8_t rising = static_cast<std::uint8_t>(bits & ~irqf_);
    irqf_ = static_cast<std::uint8_t>(irqf_ | bits);
    // SWRU191F 22.1.3/22.2: flags are set regardless of the mask; an
    // interrupt request (IRCON.T2IF, Figure 2-4) is generated when the mask
    // bit is set. Model-defined: the request is generated when a flag sets
    // with its mask bit 1 (a flag that is already 1 does not request again).
    if (rising & mask)
        bus_.raise_irq(Irq::T2);
}

std::uint64_t MacTimer::next_lf_edge_hf() const {
    // SWRU191F 22.4.1: the 32 kHz rising edge, synchronous with the 32 MHz
    // clock. Model-defined: the edge is the first 32.768 kHz tick boundary
    // (lf_tick_cycle) strictly after the current time, detected without
    // extra sampling delay.
    const std::uint64_t lf = bus_.lf_now();
    const std::uint64_t edge_lf = lf_tick_cycle(lf_ticks(lf) + 1);
    return bus_.hf_now() + (edge_lf - lf);
}

std::uint32_t MacTimer::sleep_count_at_hf(std::uint64_t hf) const {
    // Sleep Timer value at the edge = count() now minus the 32 kHz ticks
    // elapsed since the edge (mod 2^24). The edge lies on a tick boundary.
    const std::uint64_t lf_now = bus_.lf_now();
    const std::uint64_t lf_at = lf_now - (bus_.hf_now() - hf);
    const std::uint64_t elapsed = lf_ticks(lf_now) - lf_ticks(lf_at);
    const std::uint32_t count = bus_.soc().sleep_timer->count();
    return static_cast<std::uint32_t>((count - elapsed) & OvfMask);
}

void MacTimer::edge_event(std::uint64_t hf) {
    edge_hf_ = Never;
    if (phase_ == Phase::StopEdge) {
        // SWRU191F 22.4.2: at the edge the timer stops, the Sleep Timer
        // value is stored and STATE goes 1 -> 0.
        rebase(hf);
        phase_ = Phase::Idle;
        stored_sleep_ = sleep_count_at_hf(hf);
        stored_valid_ = true;
        reconcile(hf);
        return;
    }
    // SWRU191F 22.4.3: synchronous start.
    if (bus_.clkconsta() & 0x40)
        // The 16 MHz RCOSC offset of 22.4.3 is not documented: not modeled.
        bus_.note_unmodeled(sfr_reg(T2CTRL), true);
    if (!stored_valid_) {
        // Model-defined: a synchronous start without a preceding synchronous
        // stop loads "unpredictable values" (22.4.3); the model keeps the
        // current timer and overflow values and counts as unmodeled.
        bus_.note_unmodeled(sfr_reg(T2CTRL), true);
        load_tim_ = tim_;
        load_ovf_ = ovf_;
    } else {
        const std::uint32_t nc = sleep_count_at_hf(hf);
        std::uint64_t nd = (nc - stored_sleep_) & OvfMask; // Nt <= 0 -> 2^24 + Nt
        if (nd == 0)
            nd = 0x1000000u;
        // C = Nd * 976.5625 + tST + TOH, rounded to nearest (Model-defined:
        // halves round up; Nd * 15625 / 16 has a 1/16 resolution).
        const std::uint64_t c = (nd * 15625u + 8u) / 16u + tim_ + SyncOverhead;
        const std::uint64_t pt = per_ ? per_ : 0x10000u;
        const std::uint64_t povf = ovf_per_ ? ovf_per_ : 0x1000000u;
        const std::uint64_t t = c % pt;
        const std::uint64_t oticks = (c - t) / pt;
        load_tim_ = static_cast<std::uint16_t>(t);
        load_ovf_ = static_cast<std::uint32_t>((oticks + ovf_) % povf);
    }
    phase_ = Phase::Starting;
    load_hf_ = hf + SyncOverhead;
}

void MacTimer::start_loaded(std::uint64_t hf) {
    // SWRU191F 22.4.3: values loaded, RUN mode, STATE = 1, 86 cycles after
    // the edge.
    load_hf_ = Never;
    tim_ = load_tim_;
    ovf_ = load_ovf_;
    phase_ = Phase::Running;
    anchor_ = count_from_ = hf;
    reconcile(hf);
}

void MacTimer::reconcile(std::uint64_t hf) {
    // Brings the run state in line with T2CTRL.RUN (22.1.1, 22.4).
    // Model-defined: SYNC is sampled when the transition is requested; a RUN
    // write that reverts a pending synchronized transition cancels it, and
    // clearing SYNC while a transition waits for the edge completes it now.
    const std::uint8_t ctrl = bus_.storage(sfr_reg(T2CTRL));
    const bool run = ctrl & CtrlRun;
    const bool sync = ctrl & CtrlSync;
    switch (phase_) {
    case Phase::Idle:
        if (!run)
            break;
        if (sync) {
            phase_ = Phase::StartEdge;
            edge_hf_ = next_lf_edge_hf();
        } else {
            phase_ = Phase::Running;
            anchor_ = count_from_ = hf;
        }
        break;
    case Phase::StartEdge:
        if (!run) {
            phase_ = Phase::Idle;
            edge_hf_ = Never;
        } else if (!sync) {
            phase_ = Phase::Running;
            edge_hf_ = Never;
            anchor_ = count_from_ = hf;
        }
        break;
    case Phase::Starting:
        break; // completes first, then reconciles again
    case Phase::Running:
        if (run)
            break;
        if (sync) {
            phase_ = Phase::StopEdge;
            edge_hf_ = next_lf_edge_hf();
        } else {
            rebase(hf);
            phase_ = Phase::Idle;
            stored_valid_ = false; // asynchronous stop: no stored value
        }
        break;
    case Phase::StopEdge:
        if (run) {
            phase_ = Phase::Running;
            edge_hf_ = Never;
        } else if (!sync) {
            rebase(hf);
            phase_ = Phase::Idle;
            edge_hf_ = Never;
            stored_valid_ = false;
        }
        break;
    }
}

std::uint8_t MacTimer::ctrl_value() const {
    std::uint8_t v = bus_.storage(sfr_reg(T2CTRL)) & (CtrlRun | CtrlSync | CtrlLatch);
    if (counting())
        v |= CtrlState;
    return v;
}

std::uint8_t MacTimer::mux_read(Reg reg, bool side_effects) {
    const std::uint8_t msel = bus_.storage(sfr_reg(T2MSEL));
    const unsigned tsel = msel & 7u;
    const unsigned osel = (msel >> 4) & 7u;
    const bool latch_mode = bus_.storage(sfr_reg(T2CTRL)) & CtrlLatch;
    const std::uint64_t hf = bus_.hf_now();
    const std::uint8_t sfr = sfr_of(reg);
    if (sfr == T2M0 || sfr == T2M1) {
        std::uint16_t value = 0;
        switch (tsel) {
        case 0: {
            const std::uint16_t t = timer_at(hf);
            if (sfr == T2M1)
                return tim_high_latch_; // 22.1.2: latched by the T2M0 read
            if (side_effects) {
                tim_high_latch_ = static_cast<std::uint8_t>(t >> 8);
                if (latch_mode)
                    ovf_latch_ = ovf_; // 22.1.6, T2CTRL.LATCH_MODE = 1
            }
            return static_cast<std::uint8_t>(t);
        }
        case 1: value = cap_; break;
        case 2: value = per_; break;
        case 3: value = cmp1_; break;
        case 4: value = cmp2_; break;
        default:
            // Model-defined: reserved T2MSEL codes 101-111 read 0.
            if (side_effects)
                bus_.note_unmodeled(reg, false);
            return 0;
        }
        // Model-defined: only t2tim is latched; the other internal registers
        // change only by writes or capture and are read directly.
        return static_cast<std::uint8_t>(sfr == T2M0 ? value : value >> 8);
    }
    const unsigned shift = (sfr - T2MOVF0) * 8u;
    std::uint32_t value = 0;
    switch (osel) {
    case 0:
        if (latch_mode) {
            // Model-defined: with LATCH_MODE = 1 all three bytes come from the
            // latch loaded by the T2M0 read (22.1.6 "the rest of the
            // timestamp is ready to be read").
            return static_cast<std::uint8_t>(ovf_latch_ >> shift);
        }
        if (sfr == T2MOVF0) {
            if (side_effects)
                ovf_latch_ = ovf_; // 22.1.6: latches the two MSBs
            return static_cast<std::uint8_t>(ovf_);
        }
        return static_cast<std::uint8_t>(ovf_latch_ >> shift);
    case 1: value = ovf_cap_; break;
    case 2: value = ovf_per_; break;
    case 3: value = ovf_cmp1_; break;
    case 4: value = ovf_cmp2_; break;
    default:
        if (side_effects)
            bus_.note_unmodeled(reg, false);
        return 0;
    }
    return static_cast<std::uint8_t>(value >> shift);
}

std::uint8_t MacTimer::read(Reg reg) {
    catch_up();
    switch (sfr_of(reg)) {
    case T2CTRL:
        return ctrl_value();
    case T2IRQF:
        if (bus_.rmw_access()) {
            rmw_time_ = bus_.now();
            rmw_snapshot_ = irqf_;
        }
        return irqf_;
    case T2M0:
    case T2M1:
    case T2MOVF0:
    case T2MOVF1:
    case T2MOVF2:
        return mux_read(reg, true);
    default:
        return bus_.storage(reg);
    }
}

std::uint8_t MacTimer::peek(Reg reg) const {
    switch (sfr_of(reg)) {
    case T2CTRL:
        return ctrl_value();
    case T2IRQF:
        return irqf_;
    case T2M0:
    case T2M1:
    case T2MOVF0:
    case T2MOVF1:
    case T2MOVF2:
        return const_cast<MacTimer *>(this)->mux_read(reg, false);
    default:
        return bus_.storage(reg);
    }
}

void MacTimer::write(Reg reg, std::uint8_t value) {
    catch_up();
    const std::uint64_t hf = bus_.hf_now();
    const std::uint8_t msel = bus_.storage(sfr_reg(T2MSEL));
    switch (sfr_of(reg)) {
    case T2CTRL:
        // 7:4 R0, STATE read-only; RUN reads back the last written value.
        bus_.storage(reg) = value & (CtrlRun | CtrlSync | CtrlLatch);
        reconcile(hf);
        break;
    case T2EVTCFG:
        bus_.storage(reg) = value & 0x77;
        if ((value & 7u) == 6u || ((value >> 4) & 7u) == 6u)
            bus_.note_unmodeled(reg, true); // reserved code 110
        break;
    case T2IRQF: {
        // R/W0 (22.5): writing 0 clears, writing 1 leaves the flag. On the
        // write half of a read-modify-write instruction only flags that were
        // 1 at the read can be cleared (a flag set after the read survives).
        std::uint8_t clear = static_cast<std::uint8_t>(~value & FlagMask);
        if (rmw_time_ == bus_.now()) // same instruction as the RMW read
            clear &= rmw_snapshot_;
        rmw_time_ = Never;
        irqf_ = static_cast<std::uint8_t>(irqf_ & ~clear);
        break;
    }
    case T2IRQM: {
        const std::uint8_t old = bus_.storage(reg);
        bus_.storage(reg) = value & FlagMask;
        // Model-defined: unmasking a standing flag requests the interrupt
        // (the OR of T2IRQF & T2IRQM feeds T2IF, SWRU191F Figure 2-4).
        if (irqf_ & value & ~old & FlagMask)
            bus_.raise_irq(Irq::T2);
        break;
    }
    case T2MSEL:
        bus_.storage(reg) = value & 0x77;
        break;
    case T2M0:
        // Model-defined: the low byte is buffered and committed together
        // with the T2M1 write (22.1.2 "T2M0 must be written first").
        m0_buffer_ = value;
        break;
    case T2M1: {
        const std::uint16_t v = static_cast<std::uint16_t>(value << 8 | m0_buffer_);
        rebase(hf);
        switch (msel & 7u) {
        case 0:
            if (counting()) {
                // 22.1.4 delta increment: the timer halts at its current
                // value while a delta counter counts v cycles down to zero.
                // Model-defined: a new delta while halted restarts the delta
                // counter; v = 0 does not halt.
                count_from_ = hf + v;
            } else {
                tim_ = v; // 22.1.2: modifiable while idle
            }
            break;
        case 1:
            break; // t2_cap is read-only (Table 22-1)
        case 2: per_ = v; break;
        case 3: cmp1_ = v; break;
        case 4: cmp2_ = v; break;
        default:
            bus_.note_unmodeled(reg, true);
            break;
        }
        break;
    }
    case T2MOVF0:
        movf_buffer_[0] = value;
        break;
    case T2MOVF1:
        movf_buffer_[1] = value;
        break;
    case T2MOVF2: {
        // 22.1.7: LSB first, all three bytes, effective on the MSB write.
        const std::uint32_t v = static_cast<std::uint32_t>(value) << 16 |
                                static_cast<std::uint32_t>(movf_buffer_[1]) << 8 |
                                movf_buffer_[0];
        rebase(hf);
        switch ((msel >> 4) & 7u) {
        case 0: ovf_ = v; break;
        case 1: break; // t2ovf_cap is read-only
        case 2: ovf_per_ = v; break;
        case 3: ovf_cmp1_ = v; break;
        case 4: ovf_cmp2_ = v; break;
        default:
            bus_.note_unmodeled(reg, true);
            break;
        }
        break;
    }
    default:
        bus_.storage(reg) = value;
        break;
    }
    note_segment(hf);
    bus_.storage(sfr_reg(T2IRQF)) = irqf_;
}

std::uint64_t MacTimer::deadline() const {
    const std::uint64_t hf = next_event_hf();
    return hf == Never ? Never : bus_.hf_deadline(hf);
}

void MacTimer::service(std::uint64_t) {
    catch_up();
    bus_.storage(sfr_reg(T2IRQF)) = irqf_;
}

void MacTimer::power_mode_changed(PowerMode from, PowerMode to) {
    // Events up to the change were processed by catch_up() in the caller's
    // order; PM1-PM3 freeze the HF domain (deadlines become Never).
    catch_up();
    if (in_sleep(from) && !in_sleep(to) &&
        (phase_ == Phase::StartEdge || phase_ == Phase::StopEdge)) {
        // Model-defined: the 32 kHz edge is sampled by the 32 MHz clock
        // (22.4.1), so a synchronized start/stop pending across PM1-PM3
        // completes at the first edge after the HF clock is back.
        edge_hf_ = next_lf_edge_hf();
    }
    bus_.storage(sfr_reg(T2IRQF)) = irqf_;
}

bool MacTimer::may_interrupt() const { return next_event_hf() != Never; }

void MacTimer::sfd_capture(std::uint64_t at) {
    // `at` is in now() time.
    sfd_capture_hf(bus_.hf_now() - (bus_.now() - at));
}

void MacTimer::sfd_capture_hf(std::uint64_t hf) {
    // SWRU191F 22.1.10: timer and overflow count captured when the radio
    // SFD status goes high. Model-defined: capture also happens while the
    // timer is idle (it captures the held value).
    const Segment &last = segments_[segment_last_];
    if (segment_size_ == 0 || hf >= last.since) {
        cap_ = timer_at(hf);
        ovf_cap_ = ovf_;
        return;
    }
    // Model-defined: an SFD older than the recorded history (64 counter
    // changes) uses the oldest recorded state.
    unsigned index = segment_last_;
    for (unsigned n = 1; n < segment_size_; ++n) {
        const unsigned prev = (index + SegmentCount - 1) % SegmentCount;
        index = prev;
        if (segments_[index].since <= hf)
            break;
    }
    const Segment &s = segments_[index];
    cap_ = s.counting && hf > s.count_from ? advance(s.tim, hf - s.count_from, s.per) : s.tim;
    ovf_cap_ = s.ovf;
}

bool MacTimer::running() const { return counting(); }

void MacTimer::set_compare1_from_timer(std::uint64_t hf) {
    rebase(hf);
    cmp1_ = timer_at(hf);
}

std::uint64_t MacTimer::event_pulses(unsigned which) const {
    return which == 1 || which == 2 ? pulses_[which - 1] : 0;
}

} // namespace mcs51emu::cc2530
