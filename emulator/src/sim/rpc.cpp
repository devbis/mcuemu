#include "sim/rpc.hpp"

#include "sim/config.hpp"

#include <nlohmann/json.hpp>

namespace sim {

using Json = nlohmann::json;

namespace {

// Error codes: the JSON-RPC ones plus -32000 (configuration) and -32001
// (request cannot be honored in the current state).
struct RpcError {
    int code;
    std::string message;
};

const char *state_name(McuState state) {
    switch (state) {
    case McuState::Running: return "running";
    case McuState::Sleeping: return "sleeping";
    case McuState::Halted: return "halted";
    case McuState::Fault: return "fault";
    }
    return "unknown";
}

std::string to_hex(const Bytes &bytes) {
    static const char digits[] = "0123456789abcdef";
    std::string text;
    for (std::uint8_t byte : bytes) {
        text.push_back(digits[byte >> 4]);
        text.push_back(digits[byte & 15]);
    }
    return text;
}

Bytes from_hex(const std::string &text) {
    if (text.size() % 2 != 0)
        throw RpcError{-32602, "hex data must have an even number of digits"};
    Bytes bytes;
    auto digit = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    for (std::size_t i = 0; i < text.size(); i += 2) {
        const int hi = digit(text[i]), lo = digit(text[i + 1]);
        if (hi < 0 || lo < 0)
            throw RpcError{-32602, "invalid hex digit"};
        bytes.push_back(static_cast<std::uint8_t>(hi * 16 + lo));
    }
    return bytes;
}

Json value_to_json(const Value &value) {
    if (const auto *b = std::get_if<bool>(&value)) return *b;
    if (const auto *i = std::get_if<std::int64_t>(&value)) return *i;
    if (const auto *d = std::get_if<double>(&value)) return *d;
    if (const auto *s = std::get_if<std::string>(&value)) return *s;
    if (const auto *bytes = std::get_if<Bytes>(&value)) return Json{{"hex", to_hex(*bytes)}};
    return nullptr;
}

Value json_to_value(const Json &json) {
    if (json.is_null()) return Value{};
    if (json.is_boolean()) return Value{json.get<bool>()};
    if (json.is_number_integer()) return Value{json.get<std::int64_t>()};
    if (json.is_number_unsigned()) return Value{static_cast<std::int64_t>(json.get<std::uint64_t>())};
    if (json.is_number_float()) return Value{json.get<double>()};
    if (json.is_string()) return Value{json.get<std::string>()};
    if (json.is_object() && json.size() == 1 && json.contains("hex") && json["hex"].is_string())
        return Value{from_hex(json["hex"].get<std::string>())};
    throw RpcError{-32602, "a value is null, a boolean, a number, a string or {\"hex\": \"..\"}"};
}

const Json &require(const Json &params, const char *key) {
    if (!params.is_object() || !params.contains(key))
        throw RpcError{-32602, std::string("missing parameter '") + key + "'"};
    return params[key];
}

std::string require_string(const Json &params, const char *key) {
    const Json &value = require(params, key);
    if (!value.is_string())
        throw RpcError{-32602, std::string("'") + key + "' must be a string"};
    return value.get<std::string>();
}

// A time: an integer of nanoseconds or a string like "10ms".
std::uint64_t to_duration(const Json &value, const char *name) {
    if (value.is_number_unsigned() || (value.is_number_integer() && value.get<std::int64_t>() >= 0))
        return value.get<std::uint64_t>();
    if (value.is_string()) {
        try {
            return parse_duration_ns(value.get<std::string>());
        } catch (const std::exception &error) {
            throw RpcError{-32602, std::string("'") + name + "': " + error.what()};
        }
    }
    throw RpcError{-32602, std::string("'") + name + "' must be nanoseconds or a string like \"10ms\""};
}

std::optional<std::uint64_t> optional_duration(const Json &params, const char *key) {
    if (!params.is_object() || !params.contains(key) || params[key].is_null())
        return std::nullopt;
    return to_duration(params[key], key);
}

Json error_object(int code, const std::string &message) {
    return Json{{"code", code}, {"message", message}};
}

} // namespace

RpcSession::RpcSession(std::string root) : simulation_(std::move(root)) {
    register_methods();
}

RpcSession::~RpcSession() = default;

Json RpcSession::stop_result(const char *reason) {
    return Json{{"reason", reason}, {"time_ns", simulation_.time_ns()}};
}

void RpcSession::register_methods() {
    methods_["ping"] = [](const Json &) { return Json("pong"); };

    methods_["load"] = [this](const Json &params) {
        const std::string yaml = require_string(params, "yaml");
        const std::string base = params.contains("base_dir") && params["base_dir"].is_string()
                                     ? params["base_dir"].get<std::string>()
                                     : "";
        try {
            simulation_.load(yaml, base);
        } catch (const ConfigError &error) {
            throw RpcError{-32000, error.what()};
        }
        System &system = simulation_.system();
        Json mcus = Json::array();
        for (const std::string &name : system.world().mcu_names())
            mcus.push_back(Json{{"name", name},
                                {"model", std::string(system.world().mcu(name).model())}});
        Json result{{"mcus", mcus},
                    {"endpoints", system.world().endpoints().names().size()},
                    {"warnings", system.warnings()}};
        if (system.duration_ns())
            result["duration_ns"] = *system.duration_ns();
        return result;
    };
    methods_["unload"] = [this](const Json &) {
        simulation_.unload();
        return Json(true);
    };

    methods_["status"] = [this](const Json &) {
        Json status{{"loaded", simulation_.loaded()}, {"running", simulation_.running()}};
        if (simulation_.loaded()) {
            status["time_ns"] = simulation_.time_ns();
            Json mcus = Json::array();
            World &world = simulation_.system().world();
            for (const std::string &name : world.mcu_names()) {
                Mcu &mcu = world.mcu(name);
                Json entry{{"name", name}, {"model", std::string(mcu.model())},
                           {"state", state_name(mcu.state())}, {"time_ns", mcu.time_ns()}};
                if (!mcu.fault_message().empty())
                    entry["message"] = mcu.fault_message();
                mcus.push_back(entry);
            }
            status["mcus"] = mcus;
        }
        return status;
    };

    // run: {for | until: time} or nothing (free run); {wait: true} defers the
    // response until the run ends.
    methods_["run"] = [this](const Json &params) {
        std::optional<std::uint64_t> target;
        if (const auto duration = optional_duration(params, "for"))
            target = simulation_.time_ns() + *duration;
        if (const auto until = optional_duration(params, "until")) {
            if (target)
                throw RpcError{-32602, "give 'for' or 'until', not both"};
            target = *until;
        }
        simulation_.run(target);
        const bool wait = params.is_object() && params.contains("wait") &&
                          params["wait"].is_boolean() && params["wait"].get<bool>();
        if (wait && !target)
            throw RpcError{-32602, "'wait' needs 'for' or 'until'"};
        Json result{{"running", simulation_.running()}, {"time_ns", simulation_.time_ns()}};
        if (target)
            result["target_ns"] = *target;
        return result;
    };
    methods_["pause"] = [this](const Json &) {
        const bool was_running = simulation_.running();
        simulation_.pause();
        if (waiting_id_) {
            Json response{{"jsonrpc", "2.0"}, {"id", Json::parse(*waiting_id_)},
                          {"result", stop_result("paused")}};
            deferred_.push_back(response.dump());
            waiting_id_.reset();
        }
        if (was_running)
            deferred_.push_back(Json{{"jsonrpc", "2.0"}, {"method", "stopped"},
                                     {"params", stop_result("paused")}}.dump());
        return stop_result("paused");
    };
    // step: run synchronously for a short virtual time (default 1 ms, at most 1 s).
    methods_["step"] = [this](const Json &params) {
        const std::uint64_t duration = optional_duration(params, "duration").value_or(1000000);
        if (duration > 1000000000ull)
            throw RpcError{-32602, "a step is limited to 1 s of virtual time"};
        World &world = simulation_.system().world();
        simulation_.pause();
        world.run_until(world.now() + duration);
        return stop_result("step");
    };

    methods_["list_endpoints"] = [this](const Json &) {
        Json list = Json::array();
        EndpointRegistry &registry = simulation_.system().world().endpoints();
        for (const std::string &name : registry.names())
            list.push_back(Json{{"name", name}, {"kind", registry.at(name).kind()}});
        return list;
    };
    methods_["get"] = [this](const Json &params) {
        const std::string name = require_string(params, "endpoint");
        Endpoint *endpoint = simulation_.system().world().endpoints().find(name);
        if (!endpoint)
            throw RpcError{-32001, "unknown endpoint '" + name + "'"};
        return value_to_json(endpoint->get());
    };
    // set: {endpoint, value, [at]}; `at` schedules it for a virtual time.
    methods_["set"] = [this](const Json &params) {
        const std::string name = require_string(params, "endpoint");
        const Value value = json_to_value(require(params, "value"));
        World &world = simulation_.system().world();
        Endpoint *endpoint = world.endpoints().find(name);
        if (!endpoint)
            throw RpcError{-32001, "unknown endpoint '" + name + "'"};
        if (const auto at = optional_duration(params, "at")) {
            world.scheduler().at(*at, [endpoint, value] {
                try { endpoint->set(value); } catch (const std::exception &) {}
            });
            return Json{{"scheduled", true}};
        }
        try {
            endpoint->set(value);
        } catch (const std::exception &error) {
            throw RpcError{-32001, error.what()};
        }
        return Json{{"scheduled", false}};
    };
    methods_["subscribe"] = [this](const Json &params) {
        return Json{{"subscription", simulation_.subscribe(require_string(params, "endpoint"))}};
    };
    methods_["unsubscribe"] = [this](const Json &params) {
        simulation_.unsubscribe(static_cast<std::size_t>(require(params, "subscription").get<std::uint64_t>()));
        return Json(true);
    };
    methods_["attach"] = [this](const Json &params) {
        return Json{{"attachment", simulation_.attach(require_string(params, "endpoint"),
                                                      require_string(params, "sink"))}};
    };
    methods_["detach"] = [this](const Json &params) {
        simulation_.detach(static_cast<std::size_t>(require(params, "attachment").get<std::uint64_t>()));
        return Json(true);
    };
    // drive: force a net from outside; {net, state: low|high|hiz|pullup|pulldown, [at]}
    methods_["drive"] = [this](const Json &params) {
        const std::string net = require_string(params, "net");
        const std::string state = require_string(params, "state");
        Drive drive;
        if (state == "low") drive = Drive::Low;
        else if (state == "high") drive = Drive::High;
        else if (state == "hiz") drive = Drive::HiZ;
        else if (state == "pullup") drive = Drive::PullUp;
        else if (state == "pulldown") drive = Drive::PullDown;
        else throw RpcError{-32602, "state must be low, high, hiz, pullup or pulldown"};
        System &system = simulation_.system();
        try {
            (void)system.world().net(net);
        } catch (const std::out_of_range &) {
            throw RpcError{-32001, "unknown net '" + net + "'"};
        }
        if (const auto at = optional_duration(params, "at")) {
            System *sys = &system;
            system.world().scheduler().at(*at, [sys, net, drive] { sys->drive_net(net, drive); });
            return Json{{"scheduled", true}};
        }
        system.drive_net(net, drive);
        return Json{{"scheduled", false}};
    };

    methods_["mem.read"] = [this](const Json &params) {
        const std::uint64_t address = require(params, "address").get<std::uint64_t>();
        const std::uint64_t length = require(params, "length").get<std::uint64_t>();
        if (address > 0xffffffffull)
            throw RpcError{-32602, "address must fit 32 bits"};
        return Json{{"hex", to_hex(simulation_.read_memory(require_string(params, "mcu"),
                                                          static_cast<std::uint32_t>(address),
                                                          static_cast<std::size_t>(length)))}};
    };
    methods_["mem.write"] = [this](const Json &params) {
        const std::uint64_t address = require(params, "address").get<std::uint64_t>();
        if (address > 0xffffffffull)
            throw RpcError{-32602, "address must fit 32 bits"};
        simulation_.write_memory(require_string(params, "mcu"), static_cast<std::uint32_t>(address),
                                 from_hex(require_string(params, "hex")));
        return Json(true);
    };
    methods_["shutdown"] = [this](const Json &) {
        close_requested_ = true;
        return Json(true);
    };
}

Json RpcSession::dispatch(const std::string &method, const Json &params) {
    const auto it = methods_.find(method);
    if (it == methods_.end())
        throw RpcError{-32601, "method '" + method + "' not found"};
    // Everything except these needs a loaded system; the simulation says so.
    return it->second(params);
}

std::string RpcSession::process(const Json &request, bool &has_response) {
    has_response = false;
    if (!request.is_object() || request.value("jsonrpc", "") != "2.0" ||
        !request.contains("method") || !request["method"].is_string()) {
        has_response = true;
        const Json id = request.is_object() && request.contains("id") ? request["id"] : Json();
        return Json{{"jsonrpc", "2.0"}, {"id", id},
                    {"error", error_object(-32600, "not a JSON-RPC 2.0 request")}}.dump();
    }
    const bool notification = !request.contains("id");
    const Json id = notification ? Json() : request["id"];
    if (!notification && !(id.is_string() || id.is_number() || id.is_null())) {
        has_response = true;
        return Json{{"jsonrpc", "2.0"}, {"id", nullptr},
                    {"error", error_object(-32600, "id must be a string, a number or null")}}.dump();
    }
    Json response{{"jsonrpc", "2.0"}, {"id", id}};
    try {
        const Json params = request.contains("params") ? request["params"] : Json::object();
        if (!params.is_object() && !params.is_null())
            throw RpcError{-32602, "params must be an object"};
        Json result = dispatch(request["method"].get<std::string>(),
                               params.is_null() ? Json::object() : params);
        // A `run` with wait defers its response until the run ends.
        if (request["method"] == "run" && params.is_object() && params.contains("wait") &&
            params["wait"] == true && simulation_.running()) {
            waiting_id_ = id.dump();
            return {};
        }
        if (request["method"] == "run" && params.is_object() && params.contains("wait") &&
            params["wait"] == true)
            result = stop_result("target"); // already past the target
        response["result"] = std::move(result);
    } catch (const RpcError &error) {
        response["error"] = error_object(error.code, error.message);
    } catch (const ConfigError &error) {
        response["error"] = error_object(-32000, error.what());
    } catch (const SimulationError &error) {
        response["error"] = error_object(-32001, error.what());
    } catch (const Json::exception &error) {
        response["error"] = error_object(-32602, std::string("invalid parameters: ") + error.what());
    } catch (const std::exception &error) {
        response["error"] = error_object(-32603, error.what());
    }
    has_response = !notification;
    return has_response ? response.dump() : std::string();
}

void RpcSession::flush_events(std::vector<std::string> &out) {
    if (!simulation_.loaded())
        return;
    for (const SimEvent &item : simulation_.take_events())
        out.push_back(Json{{"jsonrpc", "2.0"}, {"method", "event"},
                           {"params", {{"subscription", item.subscription},
                                       {"endpoint", item.event.endpoint},
                                       {"time_ns", item.event.time_ns},
                                       {"value", value_to_json(item.event.value)}}}}.dump());
}

std::vector<std::string> RpcSession::handle(const std::string &text) {
    std::vector<std::string> out;
    Json message;
    try {
        message = Json::parse(text);
    } catch (const Json::exception &error) {
        out.push_back(Json{{"jsonrpc", "2.0"}, {"id", nullptr},
                           {"error", error_object(-32700, "parse error")}}.dump());
        return out;
    }
    if (message.is_array()) {
        if (message.empty()) {
            out.push_back(Json{{"jsonrpc", "2.0"}, {"id", nullptr},
                               {"error", error_object(-32600, "empty batch")}}.dump());
            return out;
        }
        Json responses = Json::array();
        for (const Json &item : message) {
            bool has = false;
            const std::string reply = process(item, has);
            if (has)
                responses.push_back(Json::parse(reply));
        }
        if (!responses.empty())
            out.push_back(responses.dump());
    } else {
        bool has = false;
        const std::string reply = process(message, has);
        if (has)
            out.push_back(reply);
    }
    for (std::string &deferred : deferred_)
        out.push_back(std::move(deferred));
    deferred_.clear();
    flush_events(out);
    return out;
}

std::vector<std::string> RpcSession::pump(std::chrono::milliseconds wall_budget) {
    std::vector<std::string> out;
    if (!simulation_.loaded())
        return out;
    const Simulation::PumpResult result = simulation_.pump(wall_budget);
    flush_events(out);
    for (const McuStop &stop : result.stops)
        out.push_back(Json{{"jsonrpc", "2.0"}, {"method", "mcu_stopped"},
                           {"params", {{"mcu", stop.mcu}, {"state", state_name(stop.state)},
                                       {"message", stop.message}, {"time_ns", stop.time_ns}}}}.dump());
    if (result.reached_target) {
        if (waiting_id_) {
            out.push_back(Json{{"jsonrpc", "2.0"}, {"id", Json::parse(*waiting_id_)},
                               {"result", stop_result("target")}}.dump());
            waiting_id_.reset();
        }
        out.push_back(Json{{"jsonrpc", "2.0"}, {"method", "stopped"},
                           {"params", stop_result("target")}}.dump());
    }
    for (std::string &deferred : deferred_)
        out.push_back(std::move(deferred));
    deferred_.clear();
    return out;
}

std::vector<std::string> RpcSession::drain() {
    std::vector<std::string> out;
    flush_events(out);
    return out;
}

} // namespace sim
