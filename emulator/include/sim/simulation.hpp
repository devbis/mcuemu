#pragma once

#include "sim/system.hpp"

#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim {

// Raised for requests the simulation cannot honor (no system loaded, path
// outside the sandbox, unknown subscription...). The RPC layer maps it to an
// error response.
class SimulationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// One event delivered to a subscriber.
struct SimEvent {
    std::size_t subscription;
    EndpointEvent event;
};

struct McuStop {
    std::string mcu;
    McuState state;
    std::string message;
    std::uint64_t time_ns;
};

// A loaded system plus the control state a remote client needs: run/pause,
// event subscriptions and a sandbox that confines files to a root directory.
// Everything runs on the caller's thread; pump() advances virtual time in
// small slices so that the caller can keep serving I/O.
class Simulation {
public:
    // `root` confines every file the configuration or the client names
    // (firmware, scripts, output files); relative paths resolve inside it.
    explicit Simulation(std::string root);
    ~Simulation();
    Simulation(const Simulation &) = delete;
    Simulation &operator=(const Simulation &) = delete;

    // Builds a system from YAML text. `base_dir` is relative to the root.
    // Throws ConfigError for a bad description and SimulationError for a
    // sandbox violation. Replaces any loaded system; starts paused.
    void load(const std::string &yaml, const std::string &base_dir = "");
    void unload();
    bool loaded() const noexcept { return system_ != nullptr; }
    System &system();
    const std::string &root() const noexcept { return root_; }

    std::uint64_t time_ns();
    bool running() const noexcept { return running_; }
    // Starts running; with a target the run stops when virtual time gets
    // there, without one it runs until pause().
    void run(std::optional<std::uint64_t> target_ns);
    void pause();
    // Virtual nanoseconds executed per slice of pump(); defaults to 1 ms.
    void set_slice_ns(std::uint64_t slice_ns);

    struct PumpResult {
        bool reached_target{false};
        std::vector<McuStop> stops; // MCUs that left the Running state
    };
    // Executes slices until `wall_budget` has elapsed, the target is reached
    // or the run was paused. Does nothing while paused.
    PumpResult pump(std::chrono::milliseconds wall_budget);

    // Event subscriptions; events queue up until taken.
    std::size_t subscribe(const std::string &endpoint);
    void unsubscribe(std::size_t subscription);
    std::vector<SimEvent> take_events();
    std::size_t pending_events() const noexcept { return events_.size(); }

    // Sinks, with paths confined to the root; `stdout` is refused.
    std::size_t attach(const std::string &endpoint, const std::string &spec);
    void detach(std::size_t id);

    // Target memory of an MCU.
    Bytes read_memory(const std::string &mcu, std::uint32_t address, std::size_t length);
    void write_memory(const std::string &mcu, std::uint32_t address, const Bytes &data);

    // Validates a sink spec against the sandbox and returns the spec with
    // paths made absolute. Throws SimulationError.
    std::string confine_sink(const std::string &spec) const;
    // Resolves `path` inside the root; throws SimulationError if it escapes.
    std::string confine_path(const std::string &path, const std::string &base = "") const;

private:
    void check_config(const SystemConfig &config) const;
    std::vector<McuStop> collect_stops();

    std::string root_;
    std::unique_ptr<System> system_;
    bool running_{false};
    std::optional<std::uint64_t> target_ns_;
    std::uint64_t slice_ns_{1000000};
    struct Subscription {
        Endpoint *endpoint;
        std::size_t endpoint_subscription;
    };
    std::map<std::size_t, Subscription> subscriptions_;
    std::size_t next_subscription_{1};
    std::deque<SimEvent> events_;
    std::map<std::string, McuState> last_states_;
};

} // namespace sim
