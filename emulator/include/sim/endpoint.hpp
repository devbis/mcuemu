#pragma once

#include "sim/net.hpp"
#include "tc32emu/radio.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace sim {

using Bytes = std::vector<std::uint8_t>;
using Value = std::variant<std::monostate, bool, std::int64_t, double,
                           std::string, Bytes>;

struct EndpointEvent {
    std::uint64_t time_ns;
    std::string endpoint;
    Value value;
};

using EndpointSink = std::function<void(const EndpointEvent &)>;

// A named observable/controllable point of the simulated system (a net, a
// UART, an LED, ...). Attachments (CLI, YAML, remote API) all go through this
// one interface.
class Endpoint {
public:
    virtual ~Endpoint() = default;
    virtual std::string kind() const = 0;
    virtual Value get() const { return {}; }
    virtual void set(const Value &) {
        throw std::logic_error("endpoint is read-only");
    }
    // Registers a sink for change events; returns an id for unsubscribe().
    virtual std::size_t subscribe(EndpointSink sink) = 0;
    virtual void unsubscribe(std::size_t id) = 0;
};

class EndpointRegistry {
public:
    void add(std::string name, std::unique_ptr<Endpoint> endpoint);
    Endpoint *find(const std::string &name) const;
    Endpoint &at(const std::string &name) const;
    std::vector<std::string> names() const;

private:
    std::map<std::string, std::unique_ptr<Endpoint>> endpoints_;
};

// Digital view of a net: get() -> logic level (bool, floating reads as false),
// subscribe() -> events on every level change. set() is not supported here;
// forcing a level goes through a dedicated driver part.
class NetEndpoint final : public Endpoint {
public:
    explicit NetEndpoint(Net &net);
    std::string kind() const override { return "net"; }
    Value get() const override;
    std::size_t subscribe(EndpointSink sink) override;
    void unsubscribe(std::size_t id) override;

private:
    Net &net_;
    std::map<std::size_t, EndpointSink> sinks_;
    std::size_t next_id_{1};
};

// A radio medium: get() counts frames, subscribers see a summary per frame
// ("zigbee ch15 d.a -> * 3B 010203") at the frame's start time.
class RadioEndpoint final : public Endpoint {
public:
    RadioEndpoint(std::string name, tc32emu::VirtualAir &air);
    tc32emu::VirtualAir &air() const noexcept { return air_; }
    std::string kind() const override { return "radio"; }
    Value get() const override {
        return Value{static_cast<std::int64_t>(frame_count_)};
    }
    std::size_t subscribe(EndpointSink sink) override;
    void unsubscribe(std::size_t id) override { sinks_.erase(id); }

private:
    std::string name_;
    tc32emu::VirtualAir &air_;
    std::size_t frame_count_{0};
    std::map<std::size_t, EndpointSink> sinks_;
    std::size_t next_id_{1};
};

} // namespace sim
