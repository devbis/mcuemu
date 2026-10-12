#include "sim/scheduler.hpp"

#include <algorithm>

namespace sim {

EventId Scheduler::at(std::uint64_t time_ns, Callback fn) {
    time_ns = std::max(time_ns, now_ns_);
    const EventId id = next_id_++;
    queue_.emplace(Key{time_ns, id}, std::move(fn));
    times_.emplace(id, time_ns);
    return id;
}

bool Scheduler::cancel(EventId id) {
    const auto time = times_.find(id);
    if (time == times_.end())
        return false;
    queue_.erase(Key{time->second, id});
    times_.erase(time);
    return true;
}

std::optional<std::uint64_t> Scheduler::next_event_time() const noexcept {
    if (queue_.empty())
        return std::nullopt;
    return queue_.begin()->first.first;
}

void Scheduler::advance_to(std::uint64_t time_ns) {
    while (!queue_.empty() && queue_.begin()->first.first <= time_ns) {
        const auto first = queue_.begin();
        now_ns_ = std::max(now_ns_, first->first.first);
        const EventId id = first->first.second;
        Callback fn = std::move(first->second);
        queue_.erase(first);
        times_.erase(id);
        fn();
    }
    now_ns_ = std::max(now_ns_, time_ns);
}

} // namespace sim
