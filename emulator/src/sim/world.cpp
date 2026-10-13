#include "sim/world.hpp"

#include <algorithm>
#include <stdexcept>

namespace sim {

void World::set_quantum_ns(std::uint64_t quantum_ns) {
    if (quantum_ns == 0)
        throw std::invalid_argument("quantum must be positive");
    quantum_ns_ = quantum_ns;
}

Mcu &World::add_mcu(std::unique_ptr<Mcu> mcu, std::uint64_t start_at_ns) {
    for (const auto &existing : mcus_)
        if (existing->name() == mcu->name())
            throw std::invalid_argument("duplicate MCU: " + mcu->name());
    mcu->set_start_offset(start_at_ns);
    mcus_.push_back(std::move(mcu));
    return *mcus_.back();
}

Mcu &World::mcu(std::string_view name) const {
    for (const auto &mcu : mcus_)
        if (mcu->name() == name)
            return *mcu;
    throw std::out_of_range("unknown MCU: " + std::string(name));
}

std::vector<std::string> World::mcu_names() const {
    std::vector<std::string> names;
    for (const auto &mcu : mcus_)
        names.push_back(mcu->name());
    return names;
}

Net &World::add_net(std::string name, NetKind kind) {
    for (const auto &existing : nets_)
        if (existing->name() == name)
            throw std::invalid_argument("duplicate net: " + name);
    nets_.push_back(std::make_unique<Net>(name, kind));
    Net &net = *nets_.back();
    endpoints_.add(std::move(name), std::make_unique<NetEndpoint>(net));
    return net;
}

Net &World::net(std::string_view name) const {
    for (const auto &net : nets_)
        if (net->name() == name)
            return *net;
    throw std::out_of_range("unknown net: " + std::string(name));
}

mcuemu::VirtualAir &World::add_radio(std::string name) {
    for (const auto &entry : airs_)
        if (entry.first == name)
            throw std::invalid_argument("duplicate radio medium: " + name);
    auto air = std::make_unique<mcuemu::VirtualAir>();
    mcuemu::VirtualAir &ref = *air;
    endpoints_.add(name, std::make_unique<RadioEndpoint>(name, ref));
    airs_.emplace_back(std::move(name), std::move(air));
    return ref;
}

mcuemu::VirtualAir &World::radio(std::string_view name) const {
    for (const auto &entry : airs_)
        if (entry.first == name)
            return *entry.second;
    throw std::out_of_range("unknown radio medium: " + std::string(name));
}

void World::connect(Mcu &mcu, std::string_view pad, Net &net) {
    Pin &pin = net.attach(mcu.name() + "." + std::string(pad));
    mcu.connect_pin(pad, pin);
}

void World::run_until(std::uint64_t time_ns) {
    scheduler_.advance_to(scheduler_.now()); // events already due
    while (scheduler_.now() < time_ns) {
        std::uint64_t target =
            std::min(time_ns, scheduler_.now() + quantum_ns_);
        if (const auto event = scheduler_.next_event_time();
            event && *event > scheduler_.now())
            target = std::min(target, *event);
        bool any_running = false;
        for (const auto &mcu : mcus_)
            any_running |= mcu->state() == McuState::Running;
        if (!any_running) {
            // Nothing executes: jump straight to the next event (or the end).
            const auto event = scheduler_.next_event_time();
            target = event && *event > scheduler_.now()
                         ? std::min(time_ns, *event)
                         : time_ns;
        }
        for (const auto &mcu : mcus_) {
            if (mcu->state() == McuState::Running)
                mcu->run_until(target);
        }
        for (const Poller &poller : pollers_)
            poller(target);
        scheduler_.advance_to(target);
    }
}

} // namespace sim
