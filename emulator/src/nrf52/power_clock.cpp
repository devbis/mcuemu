// nRF52840 POWER and CLOCK (instance 0).
//
// Evidence: nRF52840 Product Specification, chapters "CLOCK" and "POWER".
// Inferred, not documented: start-up and calibration delays are short fixed
// values (the real crystal needs up to hundreds of milliseconds), so firmware
// that waits for them proceeds quickly; no brown-out or USB events are ever
// raised; SYSTEMOFF is not modelled beyond leaving the CPU asleep.

#include "nrf52/peripherals.hpp"

namespace nrf52emu {

namespace {
constexpr std::uint64_t HfclkStartUs = 20;
constexpr std::uint64_t LfclkRcStartUs = 100;
constexpr std::uint64_t LfclkXtalStartUs = 1000;
constexpr std::uint64_t CalibrationUs = 100;
} // namespace

void PowerClock::reset() {
    events_ = 0;
    inten_ = 0;
    hf_requested_ = hf_running_ = hf_xtal_ = false;
    lf_requested_ = lf_running_ = false;
    lf_source_ = lf_source_running_ = 0;
    hf_debounce_ = 0x10;
    ctiv_ = 0;
    dcdcen_ = dcdcen0_ = pofcon_ = 0;
    hf_ready_at_ = lf_ready_at_ = cal_done_at_ = ct_next_ = Never;
    ct_running_ = false;
    update_irq();
}

std::uint32_t PowerClock::event_bit(std::uint32_t offset) const {
    switch (offset) {
    case 0x100: return HfclkStarted;
    case 0x104: return LfclkStarted;
    case 0x108: return PofWarn;
    case 0x10c: return Done;
    case 0x110: return Ctto;
    case 0x114: return SleepEnter;
    case 0x118: return SleepExit;
    case 0x11c: return UsbDetected;
    case 0x120: return UsbRemoved;
    case 0x124: return UsbPwrRdy;
    case 0x128: return CtStarted;
    case 0x12c: return CtStopped;
    default: return 0;
    }
}

void PowerClock::raise(std::uint32_t bit) {
    events_ |= bit;
    for (std::uint32_t offset = 0x100; offset <= 0x12c; offset += 4)
        if (event_bit(offset) == bit)
            event_raised(offset);
    update_irq();
}

void PowerClock::update_irq() { set_irq_level((events_ & inten_) != 0); }

void PowerClock::task(std::uint32_t offset) {
    const std::uint64_t cycles_per_us = clock_hz() / 1000000u;
    switch (offset) {
    case 0x000: // HFCLKSTART
        hf_requested_ = true;
        if (!hf_running_ && hf_ready_at_ == Never) {
            hf_ready_at_ = now() + HfclkStartUs * cycles_per_us;
            request_service(hf_ready_at_);
        }
        break;
    case 0x004: // HFCLKSTOP
        hf_requested_ = hf_running_ = hf_xtal_ = false;
        hf_ready_at_ = Never;
        break;
    case 0x008: // LFCLKSTART
        lf_requested_ = true;
        if (!lf_running_ && lf_ready_at_ == Never) {
            lf_ready_at_ = now() + (lf_source_ == 0 ? LfclkRcStartUs : LfclkXtalStartUs) * cycles_per_us;
            request_service(lf_ready_at_);
        }
        break;
    case 0x00c: // LFCLKSTOP
        lf_requested_ = lf_running_ = false;
        lf_ready_at_ = Never;
        break;
    case 0x010: // CAL
        cal_done_at_ = now() + CalibrationUs * cycles_per_us;
        request_service(cal_done_at_);
        break;
    case 0x014: // CTSTART: calibration timer, period CTIV * 0.25 s
        ct_running_ = true;
        ct_next_ = now() + static_cast<std::uint64_t>(ctiv_ == 0 ? 1 : ctiv_) * clock_hz() / 4;
        request_service(ct_next_);
        raise(CtStarted);
        break;
    case 0x018: // CTSTOP
        ct_running_ = false;
        ct_next_ = Never;
        raise(CtStopped);
        break;
    default: break;
    }
}

std::uint64_t PowerClock::service(std::uint64_t t) {
    if (hf_ready_at_ <= t) {
        hf_ready_at_ = Never;
        hf_running_ = true;
        hf_xtal_ = true;
        raise(HfclkStarted);
    }
    if (lf_ready_at_ <= t) {
        lf_ready_at_ = Never;
        lf_running_ = true;
        lf_source_running_ = lf_source_;
        raise(LfclkStarted);
    }
    if (cal_done_at_ <= t) {
        cal_done_at_ = Never;
        raise(Done);
    }
    if (ct_running_ && ct_next_ <= t) {
        ct_next_ = t + static_cast<std::uint64_t>(ctiv_ == 0 ? 1 : ctiv_) * clock_hz() / 4;
        raise(Ctto);
    }
    std::uint64_t next = Never;
    for (std::uint64_t at : {hf_ready_at_, lf_ready_at_, cal_done_at_, ct_next_})
        if (at < next)
            next = at;
    return next;
}

std::uint32_t PowerClock::read(std::uint32_t offset) {
    if (const std::uint32_t bit = event_bit(offset))
        return (events_ & bit) ? 1 : 0;
    switch (offset) {
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x400: return reset_reason_;
    case 0x408: return hf_requested_ ? 1 : 0;                       // HFCLKRUN
    case 0x40c: return (hf_xtal_ ? 1u : 0u) | (hf_running_ ? 1u << 16 : 0u); // HFCLKSTAT
    case 0x414: return lf_requested_ ? 1 : 0;                       // LFCLKRUN
    case 0x418: return lf_source_running_ | (lf_running_ ? 1u << 16 : 0u); // LFCLKSTAT
    case 0x41c: return lf_source_running_;                          // LFCLKSRCCOPY
    case 0x428: return 0xffffffffu;                                 // RAMSTATUS: all sections on
    case 0x438: return 0;                                           // USBREGSTATUS: no USB power
    case 0x510: return pofcon_;
    case 0x518: return lf_source_;
    case 0x51c: return gpregret_[0];
    case 0x520: return gpregret_[1];
    case 0x528: return hf_debounce_;
    case 0x538: return ctiv_;
    case 0x578: return dcdcen_;
    case 0x580: return dcdcen0_;
    case 0x640: return 1; // MAINREGSTATUS: normal voltage mode
    default: return 0;
    }
}

void PowerClock::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x100) {
        if (value & 1u)
            task(offset);
        return;
    }
    if (const std::uint32_t bit = event_bit(offset)) {
        if (value & 1u)
            events_ |= bit;
        else
            events_ &= ~bit;
        update_irq();
        return;
    }
    switch (offset) {
    case 0x304: inten_ |= value; update_irq(); break;
    case 0x308: inten_ &= ~value; update_irq(); break;
    case 0x400: reset_reason_ &= ~value; break;
    case 0x500: // SYSTEMOFF
        if (value & 1u)
            chip_.request_system_off();
        break;
    case 0x510: pofcon_ = value; break;
    case 0x518: lf_source_ = value & 3u; break;
    case 0x51c: gpregret_[0] = value & 0xffu; break;
    case 0x520: gpregret_[1] = value & 0xffu; break;
    case 0x528: hf_debounce_ = value & 0xffu; break;
    case 0x538: ctiv_ = value & 0x7fu; break;
    case 0x578: dcdcen_ = value & 1u; break;
    case 0x580: dcdcen0_ = value & 1u; break;
    default: break;
    }
}

} // namespace nrf52emu
