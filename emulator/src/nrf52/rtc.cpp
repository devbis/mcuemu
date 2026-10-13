// nRF52840 RTC0..2.
//
// Evidence: nRF52840 Product Specification, chapter "RTC". The counter is
// derived from the cycle clock instead of being stepped: counter(cycle) =
// base + ticks elapsed since the base cycle, so reads are exact and the
// peripheral only wakes up for compare events.
// Inferred: the 32.768 kHz clock is exact and does not depend on LFCLK being
// started; TICK events are generated every tick when enabled (rarely used).

#include "nrf52/peripherals.hpp"

namespace nrf52emu {

namespace {
constexpr std::uint32_t CounterMask = 0xffffffu;
constexpr std::uint64_t LfHz = 32768;
constexpr unsigned BitTick = 0, BitOverflow = 1, BitCompare0 = 16;
} // namespace

void Rtc::reset() {
    running_ = false;
    prescaler_ = 0;
    for (auto &cc : cc_)
        cc = 0;
    evten_ = inten_ = events_ = 0;
    base_cycle_ = now();
    base_counter_ = 0;
    synced_counter_ = 0;
    synced_ticks_ = 0;
    update_irq();
}

std::uint64_t Rtc::cycles_for_ticks(std::uint64_t ticks) const {
    // Exact for 64 MHz: cycles per tick = clock_hz * (prescaler + 1) / 32768.
    const std::uint64_t numerator = ticks * clock_hz() * (prescaler_ + 1);
    return (numerator + LfHz - 1) / LfHz;
}

std::uint32_t Rtc::counter_at(std::uint64_t cycle) const {
    if (!running_)
        return base_counter_;
    const std::uint64_t elapsed = cycle - base_cycle_;
    const std::uint64_t ticks = elapsed * LfHz / (clock_hz() * (prescaler_ + 1));
    return static_cast<std::uint32_t>((base_counter_ + ticks) & CounterMask);
}

void Rtc::rebase(std::uint64_t cycle, std::uint32_t counter) {
    base_cycle_ = cycle;
    base_counter_ = counter & CounterMask;
    synced_counter_ = base_counter_;
    synced_ticks_ = 0;
}

void Rtc::raise(unsigned bit) {
    events_ |= 1u << bit;
    std::uint32_t offset = bit == BitTick ? 0x100 : bit == BitOverflow ? 0x104 : 0x140 + 4 * (bit - BitCompare0);
    event_raised(offset);
}

void Rtc::update_irq() { set_irq_level((events_ & inten_) != 0); }

// Raises the events for the counter values passed since the last sync.
void Rtc::sync(std::uint64_t cycle) {
    if (!running_)
        return;
    const std::uint64_t elapsed = cycle - base_cycle_;
    const std::uint64_t ticks = elapsed * LfHz / (clock_hz() * (prescaler_ + 1));
    if (ticks == synced_ticks_)
        return;
    const std::uint64_t passed = ticks - synced_ticks_;
    // The values reached are synced_counter_+1 .. synced_counter_+passed.
    if (event_enabled(BitTick))
        raise(BitTick);
    // Overflow: the counter wrapped from 0xFFFFFF to 0.
    const std::uint64_t to_wrap = (CounterMask + 1ull) - synced_counter_;
    if (passed >= to_wrap && event_enabled(BitOverflow))
        raise(BitOverflow);
    for (unsigned ch = 0; ch != channels_; ++ch) {
        if (!event_enabled(BitCompare0 + ch))
            continue;
        // Distance from the last synced value to CC[ch], 1..2^24.
        std::uint64_t distance = (static_cast<std::uint64_t>(cc_[ch]) - synced_counter_) & CounterMask;
        if (distance == 0)
            distance = CounterMask + 1ull;
        if (distance <= passed)
            raise(BitCompare0 + ch);
    }
    synced_ticks_ = ticks;
    synced_counter_ = static_cast<std::uint32_t>((base_counter_ + ticks) & CounterMask);
    update_irq();
}

std::uint64_t Rtc::next_event_cycle() const {
    if (!running_)
        return Never;
    std::uint64_t next_ticks = Never;
    auto consider = [&](std::uint64_t distance) {
        if (distance < next_ticks)
            next_ticks = distance;
    };
    if (event_enabled(BitTick))
        consider(1);
    if (event_enabled(BitOverflow))
        consider((CounterMask + 1ull) - synced_counter_);
    for (unsigned ch = 0; ch != channels_; ++ch) {
        if (!event_enabled(BitCompare0 + ch))
            continue;
        std::uint64_t distance = (static_cast<std::uint64_t>(cc_[ch]) - synced_counter_) & CounterMask;
        if (distance == 0)
            distance = CounterMask + 1ull;
        consider(distance);
    }
    if (next_ticks == Never)
        return Never;
    const std::uint64_t at = base_cycle_ + cycles_for_ticks(synced_ticks_ + next_ticks);
    return at;
}

std::uint64_t Rtc::service(std::uint64_t t) {
    sync(t);
    return next_event_cycle();
}

void Rtc::task(std::uint32_t offset) {
    const std::uint64_t t = now();
    switch (offset) {
    case 0x000: // START
        if (!running_) {
            sync(t);
            running_ = true;
            rebase(t, base_counter_);
        }
        break;
    case 0x004: // STOP
        if (running_) {
            sync(t);
            const std::uint32_t value = counter_at(t);
            running_ = false;
            rebase(t, value);
        }
        break;
    case 0x008: // CLEAR
        sync(t);
        rebase(t, 0);
        break;
    case 0x00c: // TRIGOVRFLW
        sync(t);
        rebase(t, CounterMask - 15);
        break;
    default: return;
    }
    request_service(next_event_cycle());
}

std::uint32_t Rtc::read(std::uint32_t offset) {
    if (offset == 0x100)
        return (events_ >> BitTick) & 1u;
    if (offset == 0x104)
        return (events_ >> BitOverflow) & 1u;
    if (offset >= 0x140 && offset < 0x140 + 4 * channels_)
        return (events_ >> (BitCompare0 + (offset - 0x140) / 4)) & 1u;
    switch (offset) {
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x340:
    case 0x344:
    case 0x348: return evten_;
    case 0x504: return counter_at(now());
    case 0x508: return prescaler_;
    default: break;
    }
    if (offset >= 0x540 && offset < 0x540 + 4 * channels_)
        return cc_[(offset - 0x540) / 4];
    return 0;
}

void Rtc::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x100) {
        if (value & 1u)
            task(offset);
        return;
    }
    auto event_bit_of = [&](std::uint32_t o) -> int {
        if (o == 0x100) return BitTick;
        if (o == 0x104) return BitOverflow;
        if (o >= 0x140 && o < 0x140 + 4 * channels_) return BitCompare0 + (o - 0x140) / 4;
        return -1;
    };
    if (const int bit = event_bit_of(offset); bit >= 0) {
        if (value & 1u)
            events_ |= 1u << bit;
        else
            events_ &= ~(1u << bit);
        update_irq();
        return;
    }
    const std::uint64_t t = now();
    switch (offset) {
    case 0x304:
        sync(t);
        inten_ |= value & (0x3u | (((1u << channels_) - 1) << 16));
        update_irq();
        break;
    case 0x308:
        sync(t);
        inten_ &= ~value;
        update_irq();
        break;
    case 0x344:
        sync(t);
        evten_ |= value & (0x3u | (((1u << channels_) - 1) << 16));
        break;
    case 0x348:
        sync(t);
        evten_ &= ~value;
        break;
    case 0x508:
        if (!running_) {
            prescaler_ = value & 0xfffu;
            rebase(t, base_counter_);
        }
        break;
    default:
        if (offset >= 0x540 && offset < 0x540 + 4 * channels_) {
            sync(t);
            cc_[(offset - 0x540) / 4] = value & CounterMask;
        }
        break;
    }
    request_service(next_event_cycle());
}

} // namespace nrf52emu
