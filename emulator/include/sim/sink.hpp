#pragma once

#include "sim/endpoint.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace sim {

class VcdWriter;

// Where an endpoint's events go. A sink is created from a spec string:
//   stdout            bytes (UART, RTT) raw, other values as "t_ns name = value"
//   file:PATH         the same into a file
//   tcp:PORT          listen on 127.0.0.1:PORT (or tcp:HOST:PORT); a client gets
//                     the events and what it sends is fed back into the endpoint
//   vcd:PATH          boolean/numeric endpoints as a waveform, written at finish
//   pcap:PATH[,ble]   a radio endpoint as a capture (IEEE 802.15.4 without FCS,
//                     or BLE LL with the advertising access address)
class Sink {
public:
    virtual ~Sink() = default;
    virtual void consume(const EndpointEvent &event) = 0;
    // Housekeeping that may block briefly or touch the network; called
    // periodically by the session, not per event.
    virtual void poll() {}
    // Flushes buffered output (idempotent).
    virtual void finish() {}
};

// Collects signals from several endpoints into one VCD file.
class VcdWriter {
public:
    explicit VcdWriter(std::string path) : path_(std::move(path)) {}
    std::size_t add_signal(const std::string &name, bool real, const Value &initial);
    void record(std::size_t signal, std::uint64_t time_ns, const Value &value);
    void write();

private:
    struct Signal {
        std::string name;
        bool real;
        Value initial;
    };
    struct Change {
        std::uint64_t time;
        std::size_t signal;
        Value value;
    };
    std::string path_;
    std::vector<Signal> signals_;
    std::vector<Change> changes_;
    bool written_{false};
};

class TcpSink;

// Owns the sinks attached to the endpoints of a registry.
class Session {
public:
    explicit Session(EndpointRegistry &registry) : registry_(registry) {}
    ~Session();
    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

    // Throws std::out_of_range for an unknown endpoint and
    // std::invalid_argument for a bad spec or one the endpoint cannot feed.
    std::size_t attach(const std::string &endpoint, const std::string &spec);
    bool detach(std::size_t id);
    // Network housekeeping and file flushing; call it regularly.
    void poll();
    // Writes out buffered files (VCD); safe to call more than once.
    void finish();
    Sink *sink(std::size_t id);
    std::size_t size() const noexcept { return attachments_.size(); }

private:
    struct Attachment {
        std::size_t id;
        Endpoint *endpoint;
        std::size_t subscription;
        std::unique_ptr<Sink> sink;
    };
    EndpointRegistry &registry_;
    std::vector<Attachment> attachments_;
    std::map<std::string, std::shared_ptr<VcdWriter>> vcd_files_;
    std::size_t next_id_{1};
};

class TcpSink final : public Sink {
public:
    TcpSink(const std::string &address, Endpoint &endpoint);
    ~TcpSink() override;
    void consume(const EndpointEvent &event) override;
    void poll() override;
    unsigned short port() const noexcept { return port_; }

private:
    void drop_client();
    Endpoint &endpoint_;
    int listener_{-1};
    int client_{-1};
    unsigned short port_{0};
    bool warned_{false};
};

} // namespace sim
