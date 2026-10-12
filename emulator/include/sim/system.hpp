#pragma once

#include "sim/config.hpp"
#include "sim/mcu.hpp"
#include "sim/part.hpp"
#include "sim/sink.hpp"
#include "sim/rtt_endpoint.hpp"
#include "sim/uart_endpoint.hpp"
#include "sim/world.hpp"

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim {

// Maps a model name from the configuration to an MCU implementation.
class McuRegistry {
public:
    // `name` is the full instance id ("device.mcu"). Factories validate their
    // model-specific parameters and throw std::invalid_argument.
    using Factory = std::function<std::unique_ptr<Mcu>(const McuConfig &config,
                                                       const std::string &name)>;

    void add(std::string model, Factory factory);
    const Factory *find(const std::string &model) const;
    std::vector<std::string> models() const;

    // tlsr8258, ca51f253l3.
    static const McuRegistry &builtin();

private:
    std::map<std::string, Factory> factories_;
};

// A running simulation assembled from a SystemConfig.
class System {
public:
    System();
    ~System();
    System(const System &) = delete;
    System &operator=(const System &) = delete;

    World &world() noexcept { return world_; }
    // Sinks attached to endpoints (from `attach:` and added at run time).
    Session &session() noexcept { return session_; }
    const std::optional<std::uint64_t> &duration_ns() const noexcept {
        return duration_ns_;
    }
    // Forces a net from outside (like a scenario event, but right now). Throws
    // std::out_of_range for an unknown net.
    void drive_net(const std::string &net, Drive drive);
    // Non-fatal findings from construction (e.g. a one-way UART link).
    const std::vector<std::string> &warnings() const noexcept { return warnings_; }

private:
    friend std::unique_ptr<System> build_system(const SystemConfig &,
                                                const McuRegistry &,
                                                const PartRegistry &);
    World world_;
    Session session_{world_.endpoints()};
    std::optional<std::uint64_t> duration_ns_;
    std::vector<std::string> warnings_;
    // Owned by the endpoint registry; polled after every slice.
    std::vector<UartEndpoint *> uarts_;
    std::vector<RttEndpoint *> rtts_;
    unsigned session_ticks_{0};
    std::vector<std::unique_ptr<Part>> parts_;
    // Pins used to apply scenario events, one per driven net.
    std::map<std::string, Pin *> scenario_pins_;
};

// Instantiates MCUs, nets, links and scenario events and loads firmware.
// Throws ConfigError (with the offending YAML line) on invalid references.
std::unique_ptr<System> build_system(
    const SystemConfig &config,
    const McuRegistry &registry = McuRegistry::builtin(),
    const PartRegistry &parts = PartRegistry::builtin());

} // namespace sim
