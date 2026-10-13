// nRF52840 GPIO and GPIOTE.
//
// Evidence: nRF52840 Product Specification, chapters "GPIO" and "GPIOTE".
// Inferred, not documented: a pad whose input buffer is disconnected reads 0 in
// IN; a pad nothing is connected to reads its own drive, then its pull, then 0.

#include "nrf52/gpio.hpp"

#include <cstdio>
#include <ostream>

namespace nrf52emu {

namespace {

constexpr std::uint32_t RegOut = 0x504, RegOutSet = 0x508, RegOutClr = 0x50c, RegIn = 0x510,
                        RegDir = 0x514, RegDirSet = 0x518, RegDirClr = 0x51c, RegLatch = 0x520,
                        RegDetectMode = 0x524, RegPinCnf = 0x700;

} // namespace

void Gpio::reset() {
    out_.fill(0);
    dir_.fill(0);
    latch_.fill(0);
    detectmode_.fill(0);
    pin_cnf_.fill(2u); // input, buffer disconnected
    external_.fill(false);
    external_valid_.fill(false);
    override_.fill(std::nullopt);
    task_output_.fill(std::nullopt);
    sense_match_.fill(false);
    detect_ = false;
    for (unsigned pad = 0; pad != Nrf52840::PadCount; ++pad) {
        last_level_[pad] = false;
        chip_.notify_pad(pad);
    }
}

bool Gpio::out_level(unsigned pad) const {
    if (task_output_[pad])
        return *task_output_[pad];
    return (out_[pad / 32] >> (pad % 32)) & 1u;
}

PadDrive Gpio::drive(unsigned pad) const {
    auto pulled = [&]() {
        switch (pull(pad)) {
        case 1: return PadDrive::PullDown;
        case 3: return PadDrive::PullUp;
        default: return PadDrive::HiZ;
        }
    };
    if (override_[pad]) {
        if (override_[pad]->open_drain)
            return override_[pad]->high ? pulled() : PadDrive::Low;
        return override_[pad]->high ? PadDrive::High : PadDrive::Low;
    }
    const bool output = task_output_[pad].has_value() || is_output(pad);
    if (!output)
        return pulled();
    const bool high = out_level(pad);
    switch ((pin_cnf_[pad] >> 8) & 7u) {
    case 4: // D0S1
    case 5: // D0H1
        return high ? PadDrive::High : pulled();
    case 6: // S0D1
    case 7: // H0D1
        return high ? pulled() : PadDrive::Low;
    default:
        return high ? PadDrive::High : PadDrive::Low;
    }
}

bool Gpio::compute_level(unsigned pad) const {
    if (external_valid_[pad])
        return external_[pad];
    // An open-drain line (I2C) that is released idles high: assume the external
    // pull-up the bus needs.
    if (override_[pad] && override_[pad]->open_drain && override_[pad]->high)
        return true;
    switch (drive(pad)) {
    case PadDrive::High:
    case PadDrive::PullUp: return true;
    default: return false;
    }
}

bool Gpio::level(unsigned pad) const { return compute_level(pad); }

void Gpio::set_external(unsigned pad, bool high) {
    if (pad >= Nrf52840::PadCount)
        return;
    external_valid_[pad] = true;
    external_[pad] = high;
    refresh_input(pad);
}

void Gpio::set_override(unsigned pad, std::optional<Override> override_drive) {
    override_[pad] = override_drive;
    changed(pad);
}

void Gpio::set_task_output(unsigned pad, std::optional<bool> high) {
    task_output_[pad] = high;
    changed(pad);
}

void Gpio::changed(unsigned pad) {
    chip_.notify_pad(pad);
    refresh_input(pad);
}

void Gpio::refresh_input(unsigned pad) {
    const bool lvl = level(pad);
    const unsigned sense = (pin_cnf_[pad] >> 16) & 3u;
    const bool match = (sense == 2 && lvl) || (sense == 3 && !lvl);
    if (match && !sense_match_[pad])
        latch_[pad / 32] |= 1u << (pad % 32);
    sense_match_[pad] = match;
    bool any = false;
    for (bool m : sense_match_)
        any = any || m;
    const bool rising = any && !detect_;
    detect_ = any;

    if (lvl != last_level_[pad]) {
        last_level_[pad] = lvl;
        if (gpiote_)
            gpiote_->pad_changed(pad, lvl);
    }
    if (rising && gpiote_)
        gpiote_->detect_rising();
}

std::uint32_t Gpio::read(std::uint32_t offset) {
    const unsigned port = offset >= 0x800 ? 1 : 0;
    const std::uint32_t reg = offset - 0x300 * port;
    const unsigned first = port * 32;
    auto bits_in = [&](auto value_of) {
        std::uint32_t result = 0;
        for (unsigned bit = 0; bit != pins_in_port(port); ++bit)
            if (value_of(first + bit))
                result |= 1u << bit;
        return result;
    };
    switch (reg) {
    case RegOut:
    case RegOutSet:
    case RegOutClr: return out_[port];
    case RegIn:
        return bits_in([&](unsigned pad) { return input_connected(pad) && level(pad); });
    case RegDir:
    case RegDirSet:
    case RegDirClr: return dir_[port];
    case RegLatch: return latch_[port];
    case RegDetectMode: return detectmode_[port];
    default: break;
    }
    if (reg >= RegPinCnf && reg < RegPinCnf + 4 * pins_in_port(port))
        return pin_cnf_[first + (reg - RegPinCnf) / 4];
    return 0;
}

void Gpio::write(std::uint32_t offset, std::uint32_t value) {
    const unsigned port = offset >= 0x800 ? 1 : 0;
    const std::uint32_t reg = offset - 0x300 * port;
    const unsigned first = port * 32;
    auto each_changed = [&](std::uint32_t mask) {
        for (unsigned bit = 0; bit != pins_in_port(port); ++bit)
            if (mask & (1u << bit))
                changed(first + bit);
    };
    auto sync_dir = [&](std::uint32_t previous) {
        for (unsigned bit = 0; bit != pins_in_port(port); ++bit) {
            const unsigned pad = first + bit;
            const std::uint32_t bitmask = 1u << bit;
            // DIR mirrors PIN_CNF.DIR.
            if (dir_[port] & bitmask)
                pin_cnf_[pad] |= 1u;
            else
                pin_cnf_[pad] &= ~1u;
        }
        each_changed(previous ^ dir_[port]);
    };
    switch (reg) {
    case RegOut: {
        const std::uint32_t before = out_[port];
        out_[port] = value;
        each_changed(before ^ value);
        return;
    }
    case RegOutSet: {
        const std::uint32_t before = out_[port];
        out_[port] |= value;
        each_changed(before ^ out_[port]);
        return;
    }
    case RegOutClr: {
        const std::uint32_t before = out_[port];
        out_[port] &= ~value;
        each_changed(before ^ out_[port]);
        return;
    }
    case RegDir: {
        const std::uint32_t before = dir_[port];
        dir_[port] = value;
        sync_dir(before);
        return;
    }
    case RegDirSet: {
        const std::uint32_t before = dir_[port];
        dir_[port] |= value;
        sync_dir(before);
        return;
    }
    case RegDirClr: {
        const std::uint32_t before = dir_[port];
        dir_[port] &= ~value;
        sync_dir(before);
        return;
    }
    case RegLatch:
        latch_[port] &= ~value;
        return;
    case RegDetectMode:
        detectmode_[port] = value;
        return;
    default: break;
    }
    if (reg >= RegPinCnf && reg < RegPinCnf + 4 * pins_in_port(port)) {
        const unsigned pad = first + (reg - RegPinCnf) / 4;
        pin_cnf_[pad] = value & 0x00030f0fu;
        const std::uint32_t bitmask = 1u << (pad % 32);
        if (value & 1u)
            dir_[port] |= bitmask;
        else
            dir_[port] &= ~bitmask;
        changed(pad);
    }
}

// ---- GPIOTE ---------------------------------------------------------------

void Gpiote::reset() {
    config_.fill(0);
    in_event_.fill(false);
    port_event_ = false;
    inten_ = 0;
    out_state_.fill(false);
    update_irq();
}

void Gpiote::update_irq() {
    std::uint32_t pending = 0;
    for (unsigned ch = 0; ch != Channels; ++ch)
        if (in_event_[ch])
            pending |= 1u << ch;
    if (port_event_)
        pending |= 1u << 31;
    set_irq_level((pending & inten_) != 0);
}

std::uint32_t Gpiote::read(std::uint32_t offset) {
    if (offset >= 0x100 && offset < 0x100 + 4 * Channels)
        return in_event_[(offset - 0x100) / 4] ? 1 : 0;
    if (offset == 0x17c)
        return port_event_ ? 1 : 0;
    if (offset == 0x300 || offset == 0x304 || offset == 0x308)
        return inten_;
    if (offset >= 0x510 && offset < 0x510 + 4 * Channels)
        return config_[(offset - 0x510) / 4];
    return 0;
}

void Gpiote::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x100) {
        if (value & 1u)
            task(offset);
        return;
    }
    if (offset >= 0x100 && offset < 0x100 + 4 * Channels) {
        in_event_[(offset - 0x100) / 4] = (value & 1u) != 0;
        update_irq();
        return;
    }
    if (offset == 0x17c) {
        port_event_ = (value & 1u) != 0;
        update_irq();
        return;
    }
    if (offset == 0x304) {
        inten_ |= value;
        update_irq();
        return;
    }
    if (offset == 0x308) {
        inten_ &= ~value;
        update_irq();
        return;
    }
    if (offset >= 0x510 && offset < 0x510 + 4 * Channels) {
        const unsigned ch = (offset - 0x510) / 4;
        config_[ch] = value & 0x00133f03u;
        reconfigure(ch);
    }
}

void Gpiote::reconfigure(unsigned ch) {
    const std::uint32_t cfg = config_[ch];
    const unsigned pad = ((cfg >> 13) & 1u) * 32 + ((cfg >> 8) & 31u);
    if (pad >= Nrf52840::PadCount)
        return;
    if ((cfg & 3u) == 3u) { // task
        out_state_[ch] = ((cfg >> 20) & 1u) != 0;
        gpio_.set_task_output(pad, out_state_[ch]);
    } else {
        gpio_.set_task_output(pad, std::nullopt);
    }
}

void Gpiote::apply_task(unsigned ch, int kind) {
    const std::uint32_t cfg = config_[ch];
    if ((cfg & 3u) != 3u)
        return;
    const unsigned pad = ((cfg >> 13) & 1u) * 32 + ((cfg >> 8) & 31u);
    if (pad >= Nrf52840::PadCount)
        return;
    switch (kind) {
    case 1: out_state_[ch] = true; break;
    case 2: out_state_[ch] = false; break;
    default:
        switch ((cfg >> 16) & 3u) {
        case 1: out_state_[ch] = true; break;
        case 2: out_state_[ch] = false; break;
        case 3: out_state_[ch] = !out_state_[ch]; break;
        default: break;
        }
    }
    gpio_.set_task_output(pad, out_state_[ch]);
}

void Gpiote::task(std::uint32_t offset) {
    if (offset < 0x20)
        apply_task((offset / 4) % Channels, 0);
    else if (offset >= 0x30 && offset < 0x50)
        apply_task(((offset - 0x30) / 4) % Channels, 1);
    else if (offset >= 0x60 && offset < 0x80)
        apply_task(((offset - 0x60) / 4) % Channels, 2);
}

void Gpiote::pad_changed(unsigned pad, bool level) {
    for (unsigned ch = 0; ch != Channels; ++ch) {
        const std::uint32_t cfg = config_[ch];
        if ((cfg & 3u) != 1u)
            continue;
        if (((cfg >> 13) & 1u) * 32 + ((cfg >> 8) & 31u) != pad)
            continue;
        const unsigned polarity = (cfg >> 16) & 3u;
        const bool hit = polarity == 3 || (polarity == 1 && level) || (polarity == 2 && !level);
        if (!hit)
            continue;
        in_event_[ch] = true;
        event_raised(0x100 + 4 * ch);
    }
    update_irq();
}

void Gpiote::detect_rising() {
    port_event_ = true;
    event_raised(0x17c);
    update_irq();
}

} // namespace nrf52emu
