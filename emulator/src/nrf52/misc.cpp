// nRF52840 RNG, TEMP, WDT, EGU, PPI and the register-only stand-in.
//
// Evidence: nRF52840 Product Specification, chapters of the same names.
// Inferred, not documented: RNG bytes come from a seeded PRNG at a fixed rate
// (30 us per byte, doubled with the bias-correction bit); TEMP reports a
// configurable constant after 36 us; the WDT resets the chip two 32 kHz ticks
// after its timeout event, as the specification says.

#include "nrf52/peripherals.hpp"

namespace nrf52emu {

// ---- RNG -----------------------------------------------------------------

void Rng::reset() {
    running_ = false;
    valrdy_ = false;
    shorts_ = inten_ = config_ = value_ = 0;
    ready_at_ = Never;
    update_irq();
}

void Rng::update_irq() { set_irq_level(valrdy_ && (inten_ & 1u)); }

std::uint64_t Rng::next_random() {
    state_ ^= state_ >> 12;
    state_ ^= state_ << 25;
    state_ ^= state_ >> 27;
    return state_ * 0x2545f4914f6cdd1dull;
}

void Rng::task(std::uint32_t offset) {
    if (offset == 0x000) {
        running_ = true;
        ready_at_ = now() + (config_ & 1u ? 60u : 30u) * (clock_hz() / 1000000u);
        request_service(ready_at_);
    } else if (offset == 0x004) {
        running_ = false;
        ready_at_ = Never;
    }
}

std::uint64_t Rng::service(std::uint64_t t) {
    if (running_ && ready_at_ <= t) {
        value_ = static_cast<std::uint32_t>(next_random() >> 56) & 0xffu;
        valrdy_ = true;
        event_raised(0x100);
        if (shorts_ & 1u) {
            running_ = false;
            ready_at_ = Never;
        } else {
            ready_at_ = t + (config_ & 1u ? 60u : 30u) * (clock_hz() / 1000000u);
        }
        update_irq();
    }
    return running_ ? ready_at_ : Never;
}

std::uint32_t Rng::read(std::uint32_t offset) {
    switch (offset) {
    case 0x100: return valrdy_ ? 1 : 0;
    case 0x200: return shorts_;
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x504: return config_;
    case 0x508: return value_;
    default: return 0;
    }
}

void Rng::write(std::uint32_t offset, std::uint32_t value) {
    switch (offset) {
    case 0x000:
    case 0x004:
        if (value & 1u)
            task(offset);
        break;
    case 0x100:
        valrdy_ = (value & 1u) != 0;
        update_irq();
        break;
    case 0x200: shorts_ = value & 1u; break;
    case 0x304: inten_ |= value & 1u; update_irq(); break;
    case 0x308: inten_ &= ~value; update_irq(); break;
    case 0x504: config_ = value & 1u; break;
    default: break;
    }
}

// ---- TEMP ----------------------------------------------------------------

void Temp::reset() {
    ready_ = false;
    inten_ = 0;
    done_at_ = Never;
    set_irq_level(false);
}

void Temp::task(std::uint32_t offset) {
    if (offset == 0x000) {
        done_at_ = now() + 36u * (clock_hz() / 1000000u);
        request_service(done_at_);
    } else if (offset == 0x004) {
        done_at_ = Never;
    }
}

std::uint64_t Temp::service(std::uint64_t t) {
    if (done_at_ <= t) {
        done_at_ = Never;
        ready_ = true;
        event_raised(0x100);
        set_irq_level(inten_ & 1u);
    }
    return done_at_;
}

std::uint32_t Temp::read(std::uint32_t offset) {
    switch (offset) {
    case 0x100: return ready_ ? 1 : 0;
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x508: return static_cast<std::uint32_t>(quarter_degrees_);
    default: return 0;
    }
}

void Temp::write(std::uint32_t offset, std::uint32_t value) {
    switch (offset) {
    case 0x000:
    case 0x004:
        if (value & 1u)
            task(offset);
        break;
    case 0x100:
        ready_ = (value & 1u) != 0;
        set_irq_level(ready_ && (inten_ & 1u));
        break;
    case 0x304: inten_ |= value & 1u; set_irq_level(ready_ && (inten_ & 1u)); break;
    case 0x308: inten_ &= ~value; set_irq_level(ready_ && (inten_ & 1u)); break;
    default: break;
    }
}

// ---- WDT -----------------------------------------------------------------

void Wdt::reset() {
    // The watchdog is not reset by a soft reset in hardware unless it caused it;
    // the model starts stopped after every reset, which a running firmware
    // re-arms as part of its start-up.
    running_ = timeout_ = false;
    inten_ = 0;
    crv_ = 0xffffffffu;
    rren_ = 1;
    config_ = 1;
    reload_pending_ = 0;
    deadline_ = reset_at_ = Never;
    update_irq();
}

void Wdt::update_irq() { set_irq_level(timeout_ && (inten_ & 1u)); }

void Wdt::arm(std::uint64_t cycle) {
    const std::uint64_t cycles_per_tick = clock_hz() / 32768u;
    deadline_ = cycle + (static_cast<std::uint64_t>(crv_) + 1) * cycles_per_tick;
    reload_pending_ = rren_;
    request_service(deadline_);
}

void Wdt::task(std::uint32_t offset) {
    if (offset == 0x000 && !running_) {
        running_ = true;
        arm(now());
    }
}

std::uint64_t Wdt::service(std::uint64_t t) {
    if (running_ && deadline_ <= t) {
        deadline_ = Never;
        timeout_ = true;
        event_raised(0x100);
        update_irq();
        reset_at_ = t + 2u * (clock_hz() / 32768u);
    }
    if (reset_at_ <= t) {
        reset_at_ = Never;
        chip_.request_reset(1u << 1); // RESETREAS.DOG
    }
    return reset_at_ != Never ? reset_at_ : (running_ ? deadline_ : Never);
}

std::uint32_t Wdt::read(std::uint32_t offset) {
    switch (offset) {
    case 0x100: return timeout_ ? 1 : 0;
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x400: return running_ ? 1 : 0;
    case 0x404: return reload_pending_ & rren_;
    case 0x504: return crv_;
    case 0x508: return rren_;
    case 0x50c: return config_;
    default: return 0;
    }
}

void Wdt::write(std::uint32_t offset, std::uint32_t value) {
    if (offset == 0x000) {
        if (value & 1u)
            task(offset);
        return;
    }
    if (offset == 0x100) {
        timeout_ = (value & 1u) != 0;
        update_irq();
        return;
    }
    if (offset >= 0x600 && offset < 0x620) {
        if (value == 0x6e524635u && running_) {
            reload_pending_ &= ~(1u << ((offset - 0x600) / 4));
            if ((reload_pending_ & rren_) == 0)
                arm(now());
        }
        return;
    }
    if (running_ && (offset == 0x504 || offset == 0x508 || offset == 0x50c))
        return; // the configuration is locked once started
    switch (offset) {
    case 0x304: inten_ |= value & 1u; update_irq(); break;
    case 0x308: inten_ &= ~value; update_irq(); break;
    case 0x504: crv_ = value < 15 ? 15 : value; break;
    case 0x508: rren_ = value & 0xffu; break;
    case 0x50c: config_ = value & 0x9u; break;
    default: break;
    }
}

// ---- EGU -----------------------------------------------------------------

void Egu::reset() {
    events_ = inten_ = 0;
    update_irq();
}

void Egu::update_irq() { set_irq_level((events_ & inten_ & 0xffffu) != 0); }

void Egu::task(std::uint32_t offset) {
    if (offset < 0x40) {
        const unsigned n = offset / 4;
        events_ |= 1u << n;
        event_raised(0x100 + 4 * n);
        update_irq();
    }
}

std::uint32_t Egu::read(std::uint32_t offset) {
    if (offset >= 0x100 && offset < 0x140)
        return (events_ >> ((offset - 0x100) / 4)) & 1u;
    if (offset == 0x300 || offset == 0x304 || offset == 0x308)
        return inten_;
    return 0;
}

void Egu::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x40) {
        if (value & 1u)
            task(offset);
        return;
    }
    if (offset >= 0x100 && offset < 0x140) {
        const unsigned n = (offset - 0x100) / 4;
        if (value & 1u)
            events_ |= 1u << n;
        else
            events_ &= ~(1u << n);
        update_irq();
        return;
    }
    if (offset == 0x304) {
        inten_ |= value & 0xffffu;
        update_irq();
    } else if (offset == 0x308) {
        inten_ &= ~value;
        update_irq();
    }
}

// ---- PPI -----------------------------------------------------------------

namespace {
struct FixedChannel {
    std::uint32_t eep, tep;
};
constexpr std::uint32_t Timer0 = 0x40008000, Radio = 0x40001000, Rtc0 = 0x4000b000, Ccm = 0x4000f000;
// Channels 20..31, nRF52840 Product Specification "Pre-programmed channels".
constexpr FixedChannel fixed[12] = {
    {Timer0 + 0x140, Radio + 0x000}, {Timer0 + 0x140, Radio + 0x004},
    {Timer0 + 0x144, Radio + 0x010}, {Radio + 0x128, Ccm + 0x000 /* AAR START */},
    {Radio + 0x100, Ccm + 0x000},    {Radio + 0x104, Ccm + 0x004},
    {Radio + 0x104, Timer0 + 0x044}, {Radio + 0x10c, Timer0 + 0x048},
    {Rtc0 + 0x140, Radio + 0x000},   {Rtc0 + 0x140, Radio + 0x004},
    {Rtc0 + 0x140, Timer0 + 0x00c},  {Rtc0 + 0x140, Timer0 + 0x000},
};
} // namespace

void Ppi::reset() {
    chen_ = 0;
    for (auto &v : eep_) v = 0;
    for (auto &v : tep_) v = 0;
    for (auto &v : fork_) v = 0;
    for (auto &v : chg_) v = 0;
}

std::uint32_t Ppi::eep(unsigned ch) const { return ch < 20 ? eep_[ch] : fixed[ch - 20].eep; }
std::uint32_t Ppi::tep(unsigned ch) const { return ch < 20 ? tep_[ch] : fixed[ch - 20].tep; }

void Ppi::route(std::uint32_t address) {
    if (chen_ == 0 || depth_ > 8)
        return;
    ++depth_;
    for (unsigned ch = 0; ch != 32; ++ch) {
        if (!((chen_ >> ch) & 1u) || eep(ch) != address)
            continue;
        if (tep(ch))
            chip_.trigger_task(tep(ch));
        if (fork_[ch])
            chip_.trigger_task(fork_[ch]);
    }
    --depth_;
}

// The nRF 802.15.4 radio driver relies on this: its ramp-up channel forks to
// TASKS_CHG[n].DIS of its own group, so only the first trigger ramps the radio up.
void Ppi::task(std::uint32_t offset) {
    if (offset < 0x30 && (offset & 3) == 0)
        set_group(offset / 8, (offset & 4) == 0);
}

void Ppi::set_group(unsigned group, bool enable) {
    if (group >= 6)
        return;
    if (enable)
        chen_ |= chg_[group];
    else
        chen_ &= ~chg_[group];
}

std::uint32_t Ppi::read(std::uint32_t offset) {
    switch (offset) {
    case 0x500:
    case 0x504:
    case 0x508: return chen_;
    default: break;
    }
    if (offset >= 0x510 && offset < 0x510 + 8 * 20) {
        const unsigned ch = (offset - 0x510) / 8;
        return (offset & 4) ? tep_[ch] : eep_[ch];
    }
    if (offset >= 0x800 && offset < 0x800 + 4 * 6)
        return chg_[(offset - 0x800) / 4];
    if (offset >= 0x910 && offset < 0x910 + 4 * 32)
        return fork_[(offset - 0x910) / 4];
    return 0;
}

void Ppi::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x30) { // TASKS_CHG[n].EN/DIS
        if (value & 1u)
            set_group(offset / 8, (offset & 4) == 0);
        return;
    }
    switch (offset) {
    case 0x500: chen_ = value; return;
    case 0x504: chen_ |= value; return;
    case 0x508: chen_ &= ~value; return;
    default: break;
    }
    if (offset >= 0x510 && offset < 0x510 + 8 * 20) {
        const unsigned ch = (offset - 0x510) / 8;
        if (offset & 4)
            tep_[ch] = value;
        else
            eep_[ch] = value;
        return;
    }
    if (offset >= 0x800 && offset < 0x800 + 4 * 6) {
        chg_[(offset - 0x800) / 4] = value;
        return;
    }
    if (offset >= 0x910 && offset < 0x910 + 4 * 32)
        fork_[(offset - 0x910) / 4] = value;
}

// ---- stand-in ------------------------------------------------------------

std::uint32_t StubPeripheral::read(std::uint32_t offset) { return registers_[(offset / 4) & 1023]; }

void StubPeripheral::write(std::uint32_t offset, std::uint32_t value) {
    registers_[(offset / 4) & 1023] = value;
}

} // namespace nrf52emu
