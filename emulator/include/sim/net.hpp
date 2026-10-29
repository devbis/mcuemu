#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim {

enum class Drive {
    HiZ,
    Low,      // strong
    High,     // strong
    PullDown, // weak
    PullUp,   // weak
    PullUpStrong, // strong pull resistor; still loses to an actively driven output
};

enum class Level {
    Low,
    High,
    Floating,
};

enum class NetKind {
    PushPull,
    // A High drive releases the line (wired-AND), as on I2C and 1-Wire.
    OpenDrain,
};

class Net;

// One electrical connection of a device (MCU pad or part terminal) to a net.
// Owned by its net; the reference stays valid for the net's lifetime.
class Pin {
public:
    const std::string &owner() const noexcept { return owner_; }
    Net &net() noexcept { return net_; }
    Drive drive() const noexcept { return drive_; }
    // `time_ns` is the virtual time at which the change takes effect; it is
    // reported to net listeners.
    void set_drive(Drive drive, std::uint64_t time_ns);
    // Analog source (e.g. an NTC divider). Overrides the digital level.
    void set_analog_mv(std::optional<std::uint32_t> millivolts,
                       std::uint64_t time_ns);
    std::optional<std::uint32_t> analog_mv() const noexcept { return analog_mv_; }
    Level level() const noexcept;

private:
    friend class Net;
    Pin(Net &net, std::string owner) : net_(net), owner_(std::move(owner)) {}
    Net &net_;
    std::string owner_;
    Drive drive_{Drive::HiZ};
    std::optional<std::uint32_t> analog_mv_;
};

struct NetChange {
    std::uint64_t time_ns;
    Level level;
    std::uint32_t millivolts;
};

class Net {
public:
    using Listener = std::function<void(const NetChange &)>;

    explicit Net(std::string name, NetKind kind = NetKind::PushPull)
        : name_(std::move(name)), kind_(kind) {}
    Net(const Net &) = delete;
    Net &operator=(const Net &) = delete;

    const std::string &name() const noexcept { return name_; }
    NetKind kind() const noexcept { return kind_; }
    void set_supply_mv(std::uint32_t millivolts) noexcept { supply_mv_ = millivolts; }
    // Passive pull resistor on the net (weak, like Drive::PullUp/PullDown).
    void set_pull(Drive pull);

    Pin &attach(std::string owner);
    Level level() const noexcept { return level_; }
    std::uint32_t millivolts() const noexcept { return millivolts_; }
    // Strong drivers that disagree on a push-pull net.
    bool conflict() const noexcept { return conflict_; }
    std::size_t conflict_count() const noexcept { return conflict_count_; }

    void subscribe(Listener listener) { listeners_.push_back(std::move(listener)); }

private:
    friend class Pin;
    void resolve(std::uint64_t time_ns);

    std::string name_;
    NetKind kind_;
    std::uint32_t supply_mv_{3300};
    Drive pull_{Drive::HiZ};
    std::vector<std::unique_ptr<Pin>> pins_;
    std::vector<Listener> listeners_;
    Level level_{Level::Floating};
    std::uint32_t millivolts_{0};
    bool conflict_{false};
    std::size_t conflict_count_{0};
};

} // namespace sim
