#include "sim/endpoint.hpp"

namespace sim {

void EndpointRegistry::add(std::string name,
                           std::unique_ptr<Endpoint> endpoint) {
    if (!endpoints_.emplace(name, std::move(endpoint)).second)
        throw std::invalid_argument("duplicate endpoint: " + name);
}

Endpoint *EndpointRegistry::find(const std::string &name) const {
    const auto it = endpoints_.find(name);
    return it == endpoints_.end() ? nullptr : it->second.get();
}

Endpoint &EndpointRegistry::at(const std::string &name) const {
    Endpoint *endpoint = find(name);
    if (!endpoint)
        throw std::out_of_range("unknown endpoint: " + name);
    return *endpoint;
}

std::vector<std::string> EndpointRegistry::names() const {
    std::vector<std::string> result;
    result.reserve(endpoints_.size());
    for (const auto &entry : endpoints_)
        result.push_back(entry.first);
    return result;
}

NetEndpoint::NetEndpoint(Net &net) : net_(net) {
    net_.subscribe([this](const NetChange &change) {
        const EndpointEvent event{change.time_ns, net_.name(),
                                  Value{change.level == Level::High}};
        // Sinks may unsubscribe while being notified.
        const auto snapshot = sinks_;
        for (const auto &entry : snapshot)
            entry.second(event);
    });
}

Value NetEndpoint::get() const { return Value{net_.level() == Level::High}; }

std::size_t NetEndpoint::subscribe(EndpointSink sink) {
    const std::size_t id = next_id_++;
    sinks_.emplace(id, std::move(sink));
    return id;
}

void NetEndpoint::unsubscribe(std::size_t id) { sinks_.erase(id); }

RadioEndpoint::RadioEndpoint(std::string name, tc32emu::VirtualAir &air)
    : name_(std::move(name)) {
    air.add_observer([this](const tc32emu::RadioFrame &frame) {
        ++frame_count_;
        std::string text = tc32emu::protocol_name(frame.protocol);
        text += frame.protocol == tc32emu::RadioProtocol::Zigbee802154 ? " ch" : " ble-ch";
        text += std::to_string(frame.channel) + " " + frame.source + " -> " +
                (frame.destination.empty() ? "*" : frame.destination) + " " +
                std::to_string(frame.payload.size()) + "B ";
        static const char digits[] = "0123456789abcdef";
        for (std::uint8_t byte : frame.payload) {
            text.push_back(digits[byte >> 4]);
            text.push_back(digits[byte & 15]);
        }
        const EndpointEvent event{frame.start_ns, name_, Value{std::move(text)}};
        const auto snapshot = sinks_;
        for (const auto &entry : snapshot)
            entry.second(event);
    });
}

std::size_t RadioEndpoint::subscribe(EndpointSink sink) {
    const std::size_t id = next_id_++;
    sinks_.emplace(id, std::move(sink));
    return id;
}

} // namespace sim
