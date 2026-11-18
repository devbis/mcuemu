// The `ble_host` part: a sim::BleHost on the part's radio node, with its HCI
// stream as the endpoint "<device>.<part>.hci".
//
// The endpoint carries H4 bytes both ways. set() takes the bytes a host stack
// writes (commands, ACL data; split or merged any way), and subscribers get
// one event per controller packet. With the existing sinks that is a transport:
// `attach: {endpoint: hci, sink: "tcp:9000"}` makes the host reachable over TCP
// (Bumble: tcp-client:127.0.0.1:9000), and the control API's `set`/`subscribe`
// carry the same bytes as {"hex": ...} (the browser build).

#include "sim/ble_host.hpp"
#include "sim/h4.hpp"
#include "sim/part.hpp"

#include <cstdio>
#include <memory>

namespace sim {

namespace {

// "AA:BB:CC:DD:EE:FF" (most significant byte first) to on-air order.
ble::Address parse_address(const std::string &text) {
    unsigned b[6];
    char tail = 0;
    if (std::sscanf(text.c_str(), "%x:%x:%x:%x:%x:%x%c", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &tail) != 6)
        throw std::invalid_argument("param 'address' must look like C0:DE:00:00:00:01, got '" + text + "'");
    ble::Address address{};
    for (int i = 0; i < 6; ++i) {
        if (b[i] > 0xFF)
            throw std::invalid_argument("param 'address' must look like C0:DE:00:00:00:01, got '" + text + "'");
        address[5 - i] = static_cast<std::uint8_t>(b[i]);
    }
    return address;
}

class HciEndpoint final : public Endpoint {
public:
    HciEndpoint(std::string name, std::shared_ptr<BleHost> host, std::function<std::uint64_t()> clock)
        : name_(std::move(name)), host_(std::move(host)), clock_(std::move(clock)) {
        host_->set_hci_sink([this](const std::vector<std::uint8_t> &packet) { deliver(packet); });
    }
    ~HciEndpoint() override { host_->set_hci_sink({}); }

    std::string kind() const override { return "hci"; }
    // Number of controller packets sent so far.
    Value get() const override { return Value{static_cast<std::int64_t>(packets_)}; }
    void set(const Value &value) override {
        const auto *bytes = std::get_if<Bytes>(&value);
        if (!bytes)
            throw std::invalid_argument("the hci endpoint takes bytes (H4 packets)");
        framer_.feed(bytes->data(), bytes->size(),
                     [this](const H4Framer::Packet &packet) { host_->hci_send(packet); });
    }
    std::size_t subscribe(EndpointSink sink) override {
        sinks_[next_id_] = std::move(sink);
        return next_id_++;
    }
    void unsubscribe(std::size_t id) override { sinks_.erase(id); }

private:
    void deliver(const std::vector<std::uint8_t> &packet) {
        ++packets_;
        const EndpointEvent event{clock_(), name_, Value{Bytes(packet)}};
        // A sink may unsubscribe while it is called.
        const auto sinks = sinks_;
        for (const auto &entry : sinks)
            entry.second(event);
    }

    std::string name_;
    std::shared_ptr<BleHost> host_;
    std::function<std::uint64_t()> clock_;
    H4Framer framer_;
    std::uint64_t packets_{0};
    std::map<std::size_t, EndpointSink> sinks_;
    std::size_t next_id_{1};
};

class BleHostPart final : public Part {
public:
    explicit BleHostPart(PartContext &ctx) : Part(ctx.name()) {
        if (!ctx.air())
            throw std::invalid_argument("part type 'ble_host' needs a 'radio:' section");
        bool ble = false;
        for (const std::string &protocol : ctx.radio_protocols())
            ble = ble || protocol == "ble";
        if (!ble)
            throw std::invalid_argument("part type 'ble_host' needs 'ble' in its radio protocols");
        ParamReader &p = ctx.params();
        BleHost::Config config;
        if (const auto address = p.string("address"))
            config.address = parse_address(*address);
        config.seed = ctx.seed() + static_cast<std::uint64_t>(p.integer("seed", 0));
        p.finish();
        host_ = std::make_shared<BleHost>(*ctx.air(), ctx.scheduler(), ctx.radio_node(), config);
    }

    std::string type() const override { return "ble_host"; }

    void register_endpoints(EndpointRegistry &registry, const std::function<std::uint64_t()> &now) override {
        registry.add(name() + ".hci", std::make_unique<HciEndpoint>(name() + ".hci", host_, now));
    }

private:
    std::shared_ptr<BleHost> host_;
};

} // namespace

std::unique_ptr<Part> make_ble_host_part(PartContext &context) {
    return std::make_unique<BleHostPart>(context);
}

} // namespace sim
