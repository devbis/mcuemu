#pragma once

#include "sim/simulation.hpp"

#include <nlohmann/json_fwd.hpp>

#include <chrono>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace sim {

// JSON-RPC 2.0 control of one Simulation. The class is transport-agnostic:
// feed it request text, send back what it returns. See docs/control-api.md for
// the method list.
class RpcSession {
public:
    explicit RpcSession(std::string root);
    ~RpcSession();
    RpcSession(const RpcSession &) = delete;
    RpcSession &operator=(const RpcSession &) = delete;

    // Handles one message (a request, a notification or a batch) and returns
    // the messages to send back, in order. Never throws.
    std::vector<std::string> handle(const std::string &text);
    // Advances a running simulation for at most `wall_budget` and returns the
    // notifications and deferred responses produced (subscription events,
    // "stopped", "mcu_stopped").
    std::vector<std::string> pump(std::chrono::milliseconds wall_budget);
    // Subscription events that appeared outside pump() (for example from a
    // `set`) are included in handle()'s output; this returns any leftovers.
    std::vector<std::string> drain();

    bool running() const noexcept { return simulation_.running(); }
    bool close_requested() const noexcept { return close_requested_; }
    Simulation &simulation() noexcept { return simulation_; }

private:
    using Json = nlohmann::json;
    using Method = std::function<Json(const Json &params)>;

    std::string process(const Json &request, bool &has_response);
    Json dispatch(const std::string &method, const Json &params);
    void flush_events(std::vector<std::string> &out);
    Json stop_result(const char *reason);
    void register_methods();

    Simulation simulation_;
    std::map<std::string, Method> methods_;
    bool close_requested_{false};
    // A `run` with "wait": its request id, answered when the run ends.
    std::optional<std::string> waiting_id_;
    std::vector<std::string> deferred_;
};

} // namespace sim
