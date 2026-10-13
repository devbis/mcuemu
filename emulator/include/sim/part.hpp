#pragma once

#include "sim/endpoint.hpp"
#include "sim/net.hpp"
#include "sim/scheduler.hpp"
#include "mcuemu/radio.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace sim {

// Typed access to the `params:` mapping of a part. Every key must be consumed;
// finish() reports leftovers so that typos fail loudly. Errors are
// std::invalid_argument.
class ParamReader {
public:
    explicit ParamReader(const std::map<std::string, std::string> &params)
        : params_(params) {}

    std::optional<std::string> string(const std::string &key);
    double number(const std::string &key, double fallback);
    std::int64_t integer(const std::string &key, std::int64_t fallback);
    // "4700", "4.7k", "10K", "1M" (ohms).
    double resistance(const std::string &key, double fallback);
    void finish() const;

private:
    const std::map<std::string, std::string> &params_;
    std::set<std::string> used_;
};

// Everything a part needs from the simulation while it is being built.
class PartContext {
public:
    using PinProvider = std::function<Pin &(const std::string &terminal)>;

    PartContext(std::string name, Scheduler &scheduler,
                const std::map<std::string, std::string> &params,
                PinProvider provider, std::vector<std::string> terminals = {},
                std::string script = {}, std::uint64_t seed = 1)
        : name_(std::move(name)), scheduler_(scheduler), params_(params),
          raw_params_(params), provider_(std::move(provider)),
          terminals_(std::move(terminals)), script_(std::move(script)),
          seed_(seed) {}

    // Full instance id, "device.part".
    const std::string &name() const noexcept { return name_; }
    Scheduler &scheduler() noexcept { return scheduler_; }
    ParamReader &params() noexcept { return params_; }
    // Terminals wired in the configuration (scripts take all of them).
    const std::vector<std::string> &terminals() const noexcept { return terminals_; }
    // Raw `params:` as written, for parts that accept arbitrary keys.
    const std::map<std::string, std::string> &raw_params() const noexcept {
        return raw_params_;
    }
    // Script file (resolved path); empty for native parts.
    const std::string &script() const noexcept { return script_; }
    // The radio medium this part is attached to (null when the configuration
    // has no `radio:`), its node name on it and the protocols it declared.
    mcuemu::VirtualAir *air() const noexcept { return air_; }
    const std::string &radio_node() const noexcept { return radio_node_; }
    const std::vector<std::string> &radio_protocols() const noexcept {
        return radio_protocols_;
    }
    void set_radio(mcuemu::VirtualAir *air, std::string node,
                   std::vector<std::string> protocols) {
        air_ = air;
        radio_node_ = std::move(node);
        radio_protocols_ = std::move(protocols);
    }
    // World seed, for deterministic randomness.
    std::uint64_t seed() const noexcept { return seed_; }
    // Attaches to the net wired to `terminal` in the configuration. Throws
    // std::invalid_argument if the terminal is not wired.
    Pin &pin(const std::string &terminal) { return provider_(terminal); }

private:
    std::string name_;
    Scheduler &scheduler_;
    ParamReader params_;
    const std::map<std::string, std::string> &raw_params_;
    PinProvider provider_;
    std::vector<std::string> terminals_;
    std::string script_;
    std::uint64_t seed_;
    mcuemu::VirtualAir *air_{nullptr};
    std::string radio_node_;
    std::vector<std::string> radio_protocols_;
};

// A controllable value of a part (a temperature, a resistance, ...), exposed
// as the endpoint "<device>.<part>.<param>". get()/set() use doubles for
// numbers, bools for flags.
class ParamEndpoint final : public Endpoint {
public:
    using Getter = std::function<Value()>;
    using Setter = std::function<void(const Value &)>;

    ParamEndpoint(std::string name, Getter getter, Setter setter,
                  std::function<std::uint64_t()> clock)
        : name_(std::move(name)), getter_(std::move(getter)),
          setter_(std::move(setter)), clock_(std::move(clock)) {}

    std::string kind() const override { return "param"; }
    Value get() const override { return getter_(); }
    void set(const Value &value) override;
    std::size_t subscribe(EndpointSink sink) override;
    void unsubscribe(std::size_t id) override { sinks_.erase(id); }
    // Reports the current value to subscribers (for values that change on
    // their own, like an LED following a net).
    void notify(std::uint64_t time_ns);

private:
    std::string name_;
    Getter getter_;
    Setter setter_;
    std::function<std::uint64_t()> clock_;
    std::map<std::size_t, EndpointSink> sinks_;
    std::size_t next_id_{1};
};

// Numeric view of a Value (bool, integer or double); throws
// std::invalid_argument otherwise.
double value_as_number(const Value &value);

class Part {
public:
    explicit Part(std::string name) : name_(std::move(name)) {}
    virtual ~Part() = default;
    Part(const Part &) = delete;
    Part &operator=(const Part &) = delete;

    const std::string &name() const noexcept { return name_; }
    virtual std::string type() const = 0;
    // Registers "<name>.<param>" endpoints. `now` returns the world time.
    virtual void register_endpoints(
        EndpointRegistry &registry,
        const std::function<std::uint64_t()> &now) {
        (void)registry;
        (void)now;
    }

private:
    std::string name_;
};

class PartRegistry {
public:
    using Factory = std::function<std::unique_ptr<Part>(PartContext &)>;

    void add(std::string type, Factory factory);
    const Factory *find(const std::string &type) const;
    std::vector<std::string> types() const;

    // ds18b20, ntc, i2c_regs, script (Lua).
    static const PartRegistry &builtin();

private:
    std::map<std::string, Factory> factories_;
};

} // namespace sim
