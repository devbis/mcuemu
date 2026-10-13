// nRF52840 SAADC.
//
// Evidence: nRF52840 Product Specification, chapter "SAADC": the result is
// (V(P) - V(N)) * GAIN / REFERENCE * 2^(RESOLUTION - m) with m = 0 for
// single-ended and 1 for differential inputs, written as 16-bit samples to the
// EasyDMA buffer. Inferred: timing (see peripherals.hpp), oversampling and
// burst mode only scale the conversion time, not the value.

#include "nrf52/peripherals.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace nrf52emu {

namespace {

// Event indices (also the INTEN bit positions).
enum Event : unsigned { Started = 0, End = 1, Done = 2, ResultDone = 3, CalibrateDone = 4, Stopped = 5 };

std::uint32_t event_offset(unsigned index) {
    switch (index) {
    case Started: return 0x100;
    case End: return 0x104;
    case Done: return 0x108;
    case ResultDone: return 0x10c;
    case CalibrateDone: return 0x110;
    case Stopped: return 0x114;
    default: return 0;
    }
}

constexpr double GainTable[8] = {1.0 / 6, 1.0 / 5, 1.0 / 4, 1.0 / 3, 1.0 / 2, 1.0, 2.0, 4.0};

} // namespace

void Saadc::reset() {
    events_ = inten_ = enable_ = 0;
    channels_.fill(Channel{});
    resolution_ = 1;
    oversample_ = samplerate_ = 0;
    ptr_ = maxcnt_ = amount_ = 0;
    started_ = busy_ = running_ = stopped_pending_ = false;
    next_channel_ = sample_total_ = 0;
    done_at_ = calibrate_at_ = next_sample_at_ = Never;
    result_index_ = 0;
    update_irq();
}

void Saadc::update_irq() { set_irq_level((events_ & inten_) != 0); }

void Saadc::raise(unsigned index) {
    events_ |= 1u << index;
    event_raised(event_offset(index));
    update_irq();
}

unsigned Saadc::enabled_channels() const {
    unsigned count = 0;
    for (const Channel &channel : channels_)
        if (channel.pselp != 0)
            ++count;
    return count;
}

std::uint32_t Saadc::input_mv(std::uint32_t psel) const {
    static constexpr unsigned ain_pad[8] = {2, 3, 4, 5, 28, 29, 30, 31};
    if (psel == 0)
        return 0;
    if (psel == 9 || psel == 0x0d) // VDD, VDDHDIV5 (both from the supply the model has)
        return chip_.supply_mv();
    if (psel >= 1 && psel <= 8)
        return chip_.analog_input_mv(ain_pad[psel - 1]);
    return 0;
}

std::int16_t Saadc::convert(unsigned channel) {
    const Channel &config = channels_[channel];
    const bool differential = (config.config >> 20) & 1u;
    const double gain = GainTable[(config.config >> 8) & 7u];
    const bool vdd_reference = (config.config >> 12) & 1u;
    const double reference_mv = vdd_reference ? chip_.supply_mv() / 4.0 : 600.0;
    const unsigned bits = 8 + 2 * std::min(resolution_, 3u);
    double volts_p = input_mv(config.pselp);
    double volts_n = differential ? input_mv(config.pseln) : 0.0;
    const double scale = static_cast<double>(1u << (bits - (differential ? 1 : 0)));
    double value = (volts_p - volts_n) * gain / reference_mv * scale;
    const double maximum = scale - 1;
    const double minimum = differential ? -scale : 0.0;
    value = std::min(std::max(value, minimum), maximum);
    return static_cast<std::int16_t>(value);
}

void Saadc::start_sample(std::uint64_t t) {
    busy_ = true;
    next_channel_ = 0;
    // Acquisition times from CONFIG.TACQ (3, 5, 10, 15, 20, 40 us) plus 2 us.
    static constexpr unsigned tacq_us[6] = {3, 5, 10, 15, 20, 40};
    std::uint64_t us = 0;
    for (const Channel &channel : channels_)
        if (channel.pselp != 0)
            us += tacq_us[std::min<std::uint32_t>((channel.config >> 16) & 7u, 5u)] + 2;
    done_at_ = t + std::max<std::uint64_t>(us, 2) * (clock_hz() / 1000000u);
    request_service(done_at_);
}

void Saadc::finish_sample(std::uint64_t) {
    busy_ = false;
    done_at_ = Never;
    for (unsigned ch = 0; ch != channels_.size(); ++ch) {
        if (channels_[ch].pselp == 0)
            continue;
        if (amount_ + 2 <= maxcnt_ * 2 && ptr_ >= Nrf52840::RamBase) {
            const std::int16_t sample = convert(ch);
            const std::uint32_t address = ptr_ + amount_ * 2;
            if (address >= Nrf52840::RamBase && address + 2 <= Nrf52840::RamBase + Nrf52840::RamSize)
                std::memcpy(chip_.ram() + (address - Nrf52840::RamBase), &sample, 2);
            ++amount_;
            raise(ResultDone);
        }
    }
    raise(Done);
    if (amount_ >= maxcnt_ && maxcnt_ != 0) {
        raise(End);
        started_ = false;
    }
}

void Saadc::task(std::uint32_t offset) {
    const std::uint64_t t = now();
    switch (offset) {
    case 0x000: // START: latch the buffer and begin
        if (enable_ == 0)
            break;
        started_ = true;
        amount_ = 0;
        raise(Started);
        if ((samplerate_ >> 12) & 1u) { // timer mode: continuous sampling
            running_ = true;
            const std::uint32_t cc = std::max<std::uint32_t>(samplerate_ & 0x7ffu, 80u);
            next_sample_at_ = t + static_cast<std::uint64_t>(cc) * (clock_hz() / 16000000u);
            request_service(next_sample_at_);
        }
        break;
    case 0x004: // SAMPLE
        if (enable_ != 0 && started_ && !busy_)
            start_sample(t);
        break;
    case 0x008: // STOP
        running_ = false;
        started_ = false;
        busy_ = false;
        done_at_ = next_sample_at_ = Never;
        raise(Stopped);
        break;
    case 0x00c: // CALIBRATEOFFSET
        if (enable_ != 0) {
            calibrate_at_ = t + 100u * (clock_hz() / 1000000u);
            request_service(calibrate_at_);
        }
        break;
    default: break;
    }
}

std::uint64_t Saadc::service(std::uint64_t t) {
    if (calibrate_at_ <= t) {
        calibrate_at_ = Never;
        raise(CalibrateDone);
    }
    if (busy_ && done_at_ <= t)
        finish_sample(t);
    if (running_ && next_sample_at_ <= t && !busy_) {
        const std::uint32_t cc = std::max<std::uint32_t>(samplerate_ & 0x7ffu, 80u);
        next_sample_at_ = t + static_cast<std::uint64_t>(cc) * (clock_hz() / 16000000u);
        start_sample(t);
    }
    std::uint64_t next = std::min(calibrate_at_, busy_ ? done_at_ : Never);
    if (running_)
        next = std::min(next, next_sample_at_);
    return next;
}

std::uint32_t Saadc::read(std::uint32_t offset) {
    if (offset >= 0x100 && offset <= 0x114)
        for (unsigned i = 0; i != 6; ++i)
            if (event_offset(i) == offset)
                return (events_ >> i) & 1u;
    switch (offset) {
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x400: return busy_ ? 1 : 0; // STATUS
    case 0x500: return enable_;
    case 0x5f0: return resolution_;
    case 0x5f4: return oversample_;
    case 0x5f8: return samplerate_;
    case 0x62c: return ptr_;
    case 0x630: return maxcnt_;
    case 0x634: return amount_;
    default: break;
    }
    if (offset >= 0x510 && offset < 0x510 + 16 * 8) {
        const Channel &channel = channels_[(offset - 0x510) / 16];
        switch ((offset - 0x510) % 16) {
        case 0: return channel.pselp;
        case 4: return channel.pseln;
        case 8: return channel.config;
        case 12: return channel.limit;
        default: return 0;
        }
    }
    return 0;
}

void Saadc::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x100) {
        if (value & 1u)
            task(offset);
        return;
    }
    if (offset >= 0x100 && offset <= 0x114) {
        for (unsigned i = 0; i != 6; ++i) {
            if (event_offset(i) == offset) {
                if (value & 1u)
                    events_ |= 1u << i;
                else
                    events_ &= ~(1u << i);
                update_irq();
                return;
            }
        }
        return;
    }
    switch (offset) {
    case 0x304: inten_ |= value; update_irq(); return;
    case 0x308: inten_ &= ~value; update_irq(); return;
    case 0x500: enable_ = value & 1u; return;
    case 0x5f0: resolution_ = value & 3u; return;
    case 0x5f4: oversample_ = value & 0xfu; return;
    case 0x5f8: samplerate_ = value & 0x17ffu; return;
    case 0x62c: ptr_ = value; return;
    case 0x630: maxcnt_ = value & 0x7fffu; return;
    default: break;
    }
    if (offset >= 0x510 && offset < 0x510 + 16 * 8) {
        Channel &channel = channels_[(offset - 0x510) / 16];
        switch ((offset - 0x510) % 16) {
        case 0: channel.pselp = value & 0x1f; break;
        case 4: channel.pseln = value & 0x1f; break;
        case 8: channel.config = value; break;
        case 12: channel.limit = value; break;
        default: break;
        }
    }
}

} // namespace nrf52emu
