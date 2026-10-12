#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <utility>

namespace sim {

using EventId = std::uint64_t;

// Deterministic discrete-event queue over virtual time. Events with equal
// timestamps run in scheduling order.
class Scheduler {
public:
    using Callback = std::function<void()>;

    std::uint64_t now() const noexcept { return now_ns_; }

    // Schedules `fn` at absolute virtual time `time_ns`. A time in the past is
    // clamped to now().
    EventId at(std::uint64_t time_ns, Callback fn);
    EventId after(std::uint64_t delay_ns, Callback fn) {
        return at(now_ns_ + delay_ns, std::move(fn));
    }
    bool cancel(EventId id);

    std::optional<std::uint64_t> next_event_time() const noexcept;
    bool empty() const noexcept { return queue_.empty(); }

    // Moves time forward to `time_ns` (never backwards) and runs every event
    // due at or before it, in order. Events scheduled by callbacks that fall
    // inside the window run too.
    void advance_to(std::uint64_t time_ns);

private:
    using Key = std::pair<std::uint64_t, EventId>;
    std::uint64_t now_ns_{0};
    EventId next_id_{1};
    std::map<Key, Callback> queue_;
    std::map<EventId, std::uint64_t> times_;
};

} // namespace sim
