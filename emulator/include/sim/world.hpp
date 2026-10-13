#pragma once

#include "sim/endpoint.hpp"
#include "sim/mcu.hpp"
#include "sim/net.hpp"
#include "sim/scheduler.hpp"
#include "mcuemu/radio.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace sim {

// Owns MCUs, nets, endpoints and the global event queue, and advances them
// together on one virtual timeline.
class World {
public:
    Scheduler &scheduler() noexcept { return scheduler_; }
    EndpointRegistry &endpoints() noexcept { return endpoints_; }
    std::uint64_t now() const noexcept { return scheduler_.now(); }

    // Upper bound for how far one MCU runs ahead of the others between
    // synchronisation points. Smaller is more accurate for cross-MCU and
    // bit-banged interaction, larger is faster.
    void set_quantum_ns(std::uint64_t quantum_ns);

    Mcu &add_mcu(std::unique_ptr<Mcu> mcu, std::uint64_t start_at_ns = 0);
    Mcu &mcu(std::string_view name) const;
    std::vector<std::string> mcu_names() const;
    // Creates a net and registers a NetEndpoint named after it.
    Net &add_net(std::string name, NetKind kind = NetKind::PushPull);
    Net &net(std::string_view name) const;
    // Attaches MCU pad `pad` to `net`.
    void connect(Mcu &mcu, std::string_view pad, Net &net);

    // Called after every synchronisation slice with the slice end time, before
    // the scheduler runs events due at that time. Used by byte-level links.
    using Poller = std::function<void(std::uint64_t now_ns)>;
    void add_poller(Poller poller) { pollers_.push_back(std::move(poller)); }

    // Creates a radio medium and registers an endpoint of kind "radio" with
    // the same name: get() is the number of frames sent so far, subscribers get
    // one event per frame (a text summary) at its start time.
    mcuemu::VirtualAir &add_radio(std::string name);
    mcuemu::VirtualAir &radio(std::string_view name) const;

    void run_until(std::uint64_t time_ns);
    void run_for(std::uint64_t duration_ns) { run_until(now() + duration_ns); }

private:
    Scheduler scheduler_;
    EndpointRegistry endpoints_;
    std::uint64_t quantum_ns_{5000};
    std::vector<std::unique_ptr<Mcu>> mcus_;
    std::vector<std::unique_ptr<Net>> nets_;
    std::vector<Poller> pollers_;
    std::vector<std::pair<std::string, std::unique_ptr<mcuemu::VirtualAir>>> airs_;
};

} // namespace sim
