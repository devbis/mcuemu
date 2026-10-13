// nRF52840 TIMER0..4.
//
// Evidence: nRF52840 Product Specification, chapter "TIMER". Counting is
// derived from the cycle clock like the RTC: the model only wakes up for
// compare events. The base clock is 16 MHz (HFCLK 64 MHz / 4) before the
// prescaler. Inferred: the timer runs whether or not HFXO was started.

#include "nrf52/peripherals.hpp"

namespace nrf52emu {

std::uint64_t Timer::cycles_per_tick() const {
    const std::uint64_t base = clock_hz() / 16000000u;
    return (base ? base : 1) << (prescaler_ > 9 ? 9 : prescaler_);
}

std::uint32_t Timer::mask() const {
    switch (bitmode_) {
    case 0: return 0xffffu;
    case 1: return 0xffu;
    case 2: return 0xffffffu;
    default: return 0xffffffffu;
    }
}

std::uint32_t Timer::counter_at(std::uint64_t cycle) const {
    if (!running_ || mode_ != 0)
        return base_counter_ & mask();
    const std::uint64_t ticks = (cycle - base_cycle_) / cycles_per_tick();
    return static_cast<std::uint32_t>((base_counter_ + ticks) & mask());
}

void Timer::reset() {
    running_ = false;
    mode_ = bitmode_ = 0;
    prescaler_ = 4;
    for (auto &cc : cc_)
        cc = 0;
    shorts_ = inten_ = events_ = 0;
    base_cycle_ = now();
    base_counter_ = 0;
    last_counter_ = 0;
    update_irq();
}

void Timer::update_irq() { set_irq_level((events_ & inten_) != 0); }

void Timer::restart_from(std::uint64_t cycle, std::uint32_t counter) {
    base_cycle_ = cycle;
    base_counter_ = counter & mask();
    last_counter_ = base_counter_;
    synced_ticks_ = 0;
}

void Timer::freeze(std::uint64_t cycle) {
    base_counter_ = counter_at(cycle);
    running_ = false;
    last_counter_ = base_counter_;
    synced_ticks_ = 0;
}

void Timer::compare_hit(unsigned ch, std::uint64_t) {
    events_ |= 1u << (16 + ch);
    event_raised(0x140 + 4 * ch);
}

std::uint64_t Timer::next_event_cycle() const {
    if (!running_ || mode_ != 0)
        return Never;
    const std::uint64_t cpt = cycles_per_tick();
    const std::uint32_t m = mask();
    const std::uint64_t ticks_done = synced_ticks_;
    // Ticks from the last processed counter value to the nearest CC match.
    std::uint64_t best = Never;
    for (unsigned ch = 0; ch != channels_; ++ch) {
        std::uint64_t distance = (static_cast<std::uint64_t>(cc_[ch]) - last_counter_) & m;
        if (distance == 0)
            distance = static_cast<std::uint64_t>(m) + 1;
        if (distance < best)
            best = distance;
    }
    if (best == Never)
        return Never;
    return base_cycle_ + (ticks_done + best) * cpt;
}

std::uint64_t Timer::service(std::uint64_t t) {
    const std::uint32_t m = mask();
    while (running_ && mode_ == 0) {
        const std::uint64_t cpt = cycles_per_tick();
        const std::uint64_t ticks_done = synced_ticks_;
        const std::uint64_t ticks_now = (t - base_cycle_) / cpt;
        if (ticks_now <= ticks_done)
            break;
        const std::uint64_t passed = ticks_now - ticks_done;
        std::uint64_t best = Never;
        for (unsigned ch = 0; ch != channels_; ++ch) {
            std::uint64_t distance = (static_cast<std::uint64_t>(cc_[ch]) - last_counter_) & m;
            if (distance == 0)
                distance = static_cast<std::uint64_t>(m) + 1;
            if (distance <= passed && distance < best)
                best = distance;
        }
        if (best == Never) {
            synced_ticks_ += passed;
            last_counter_ = static_cast<std::uint32_t>((last_counter_ + passed) & m);
            break;
        }
        synced_ticks_ += best;
        last_counter_ = static_cast<std::uint32_t>((last_counter_ + best) & m);
        const std::uint64_t event_cycle = base_cycle_ + (ticks_done + best) * cpt;
        bool clear = false, stop = false;
        for (unsigned ch = 0; ch != channels_; ++ch) {
            if ((cc_[ch] & m) != last_counter_)
                continue;
            compare_hit(ch, event_cycle);
            clear = clear || (shorts_ >> ch) & 1u;
            stop = stop || (shorts_ >> (8 + ch)) & 1u;
        }
        if (stop) {
            running_ = false;
            base_counter_ = last_counter_;
            synced_ticks_ = 0;
            break;
        }
        if (clear)
            restart_from(event_cycle, 0);
    }
    update_irq();
    return next_event_cycle();
}

void Timer::task(std::uint32_t offset) {
    const std::uint64_t t = now();
    if (offset >= 0x040 && offset < 0x040 + 4 * channels_) {
        service(t);
        cc_[(offset - 0x040) / 4] = counter_at(t);
        return;
    }
    switch (offset) {
    case 0x000: // START
        if (!running_) {
            running_ = true;
            restart_from(t, base_counter_);
        }
        break;
    case 0x004: // STOP
        if (running_) {
            service(t);
            freeze(t);
        }
        break;
    case 0x008: { // COUNT
        if (mode_ == 0)
            break;
        base_counter_ = (base_counter_ + 1) & mask();
        last_counter_ = base_counter_;
        synced_ticks_ = 0;
        bool clear = false;
        for (unsigned ch = 0; ch != channels_; ++ch) {
            if ((cc_[ch] & mask()) == base_counter_) {
                compare_hit(ch, t);
                clear = clear || (shorts_ >> ch) & 1u;
            }
        }
        if (clear)
            base_counter_ = last_counter_ = 0;
        update_irq();
        break;
    }
    case 0x00c: // CLEAR
        service(t);
        restart_from(t, 0);
        break;
    case 0x010: // SHUTDOWN
        running_ = false;
        restart_from(t, 0);
        break;
    default: return;
    }
    request_service(next_event_cycle());
}

std::uint32_t Timer::read(std::uint32_t offset) {
    if (offset >= 0x140 && offset < 0x140 + 4 * channels_)
        return (events_ >> (16 + (offset - 0x140) / 4)) & 1u;
    switch (offset) {
    case 0x200: return shorts_;
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x504: return mode_;
    case 0x508: return bitmode_;
    case 0x510: return prescaler_;
    default: break;
    }
    if (offset >= 0x540 && offset < 0x540 + 4 * channels_)
        return cc_[(offset - 0x540) / 4];
    return 0;
}

void Timer::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x100) {
        if (value & 1u)
            task(offset);
        return;
    }
    if (offset >= 0x140 && offset < 0x140 + 4 * channels_) {
        const unsigned bit = 16 + (offset - 0x140) / 4;
        if (value & 1u)
            events_ |= 1u << bit;
        else
            events_ &= ~(1u << bit);
        update_irq();
        return;
    }
    const std::uint64_t t = now();
    switch (offset) {
    case 0x200: shorts_ = value & 0x3f3fu; break;
    case 0x304: inten_ |= value; update_irq(); break;
    case 0x308: inten_ &= ~value; update_irq(); break;
    case 0x504: mode_ = value & 3u; break;
    case 0x508: bitmode_ = value & 3u; break;
    case 0x510:
        service(t);
        if (running_)
            restart_from(t, counter_at(t));
        prescaler_ = value & 0xfu;
        break;
    default:
        if (offset >= 0x540 && offset < 0x540 + 4 * channels_) {
            service(t);
            cc_[(offset - 0x540) / 4] = value;
        }
        break;
    }
    request_service(next_event_cycle());
}

} // namespace nrf52emu
