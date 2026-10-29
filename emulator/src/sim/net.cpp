#include "sim/net.hpp"

#include <stdexcept>

namespace sim {

void Pin::set_drive(Drive drive, std::uint64_t time_ns) {
    if (drive_ == drive)
        return;
    drive_ = drive;
    net_.resolve(time_ns);
}

void Pin::set_analog_mv(std::optional<std::uint32_t> millivolts,
                        std::uint64_t time_ns) {
    if (analog_mv_ == millivolts)
        return;
    analog_mv_ = millivolts;
    net_.resolve(time_ns);
}

Level Pin::level() const noexcept { return net_.level(); }

void Net::set_pull(Drive pull) {
    if (pull != Drive::HiZ && pull != Drive::PullUp && pull != Drive::PullDown)
        throw std::invalid_argument("net pull must be PullUp, PullDown or HiZ");
    pull_ = pull;
    resolve(0);
}

Pin &Net::attach(std::string owner) {
    pins_.push_back(std::unique_ptr<Pin>(new Pin(*this, std::move(owner))));
    return *pins_.back();
}

void Net::resolve(std::uint64_t time_ns) {
    bool strong_low = false, strong_high = false, strong_pull_up = false;
    bool weak_low = pull_ == Drive::PullDown, weak_high = pull_ == Drive::PullUp;
    std::optional<std::uint32_t> analog;
    for (const auto &pin : pins_) {
        Drive drive = pin->drive();
        if (kind_ == NetKind::OpenDrain && drive == Drive::High)
            drive = Drive::HiZ;
        switch (drive) {
        case Drive::Low: strong_low = true; break;
        case Drive::High: strong_high = true; break;
        case Drive::PullDown: weak_low = true; break;
        case Drive::PullUp: weak_high = true; break;
        case Drive::PullUpStrong: strong_pull_up = true; break;
        case Drive::HiZ: break;
        }
        if (pin->analog_mv())
            analog = pin->analog_mv();
    }

    Level level;
    bool conflict = false;
    if (strong_low && strong_high) {
        level = Level::Low; // shorted drivers: report as low, flag conflict
        conflict = true;
    } else if (strong_low) {
        level = Level::Low;
    } else if (strong_high) {
        level = Level::High;
    } else if (strong_pull_up) {
        level = Level::High;
    } else if (pull_ == Drive::PullUp) {
        // A board resistor is assumed to dominate an opposing on-chip weak
        // pull; a driven GPIO output still wins in the branches above.
        level = Level::High;
    } else if (pull_ == Drive::PullDown) {
        level = Level::Low;
    } else if (weak_low && weak_high) {
        level = Level::Floating;
    } else if (weak_low) {
        level = Level::Low;
    } else if (weak_high) {
        level = Level::High;
    } else {
        level = Level::Floating;
    }

    std::uint32_t millivolts = 0;
    if (analog) {
        millivolts = *analog;
        // A strong digital driver wins over an analog source.
        if (level != Level::Floating && (strong_low || strong_high))
            millivolts = level == Level::High ? supply_mv_ : 0;
        else if (level == Level::Floating || (!strong_low && !strong_high))
            level = millivolts * 2 >= supply_mv_ ? Level::High : Level::Low;
    } else if (level == Level::High) {
        millivolts = supply_mv_;
    }

    if (conflict && !conflict_)
        ++conflict_count_;
    conflict_ = conflict;
    if (level == level_ && millivolts == millivolts_)
        return;
    level_ = level;
    millivolts_ = millivolts;
    const NetChange change{time_ns, level_, millivolts_};
    // Listeners may attach pins or subscribe; iterate by index.
    for (std::size_t i = 0; i < listeners_.size(); ++i)
        listeners_[i](change);
}

} // namespace sim
