#pragma once

#include "sim/endpoint.hpp"
#include "sim/mcu.hpp"

#include <deque>

namespace sim {

// Host side of SEGGER RTT, read straight from target memory (no CPU
// involvement). The control block address is given; nothing happens until the
// firmware has initialized it (the "SEGGER RTT" id string is present).
//
// Layout (32-bit target): id[16], MaxNumUpBuffers, MaxNumDownBuffers, then
// the up buffer descriptors followed by the down ones, each {sName, pBuffer,
// SizeOfBuffer, WrOff, RdOff, Flags} of 24 bytes.
//   events: bytes the firmware wrote to up buffer `channel` (the host
//           advances RdOff);
//   set():  bytes (Bytes or a string) for down buffer `channel`, delivered as
//           space becomes available; get() is the number of bytes read.
class RttEndpoint final : public Endpoint {
public:
    RttEndpoint(std::string name, Mcu &mcu, std::uint32_t address,
                unsigned channel, std::uint64_t interval_ns)
        : name_(std::move(name)), mcu_(mcu), address_(address),
          channel_(channel), interval_ns_(interval_ns) {}

    std::string kind() const override { return "rtt"; }
    Value get() const override { return Value{static_cast<std::int64_t>(received_)}; }
    void set(const Value &value) override;
    std::size_t subscribe(EndpointSink sink) override;
    void unsubscribe(std::size_t id) override { sinks_.erase(id); }

    // Reads and writes at most once per `interval_ns` of virtual time.
    void poll(std::uint64_t now_ns);

private:
    struct Descriptor {
        std::uint32_t buffer, size, write_offset, read_offset;
        std::uint32_t address; // of the descriptor itself
    };
    bool find(bool up, Descriptor &out);
    std::uint32_t load32(std::uint32_t address);

    std::string name_;
    Mcu &mcu_;
    std::uint32_t address_;
    unsigned channel_;
    std::uint64_t interval_ns_;
    std::uint64_t next_poll_ns_{0};
    std::uint64_t received_{0};
    std::deque<std::uint8_t> down_pending_;
    std::map<std::size_t, EndpointSink> sinks_;
    std::size_t next_id_{1};
};

} // namespace sim
