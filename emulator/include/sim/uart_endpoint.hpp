#pragma once

#include "sim/endpoint.hpp"
#include "sim/mcu.hpp"

#include <deque>

namespace sim {

// One UART of an MCU as an endpoint (kind "uart").
//   events:  the bytes the UART transmitted (Bytes), stamped with the time of
//            the poll that collected them;
//   set():   bytes (Bytes or a string) to receive; they wait until the UART's
//            FIFO accepts them;
//   get():   total bytes transmitted so far.
// poll() must run regularly (World does so after every slice).
class UartEndpoint final : public Endpoint {
public:
    UartEndpoint(std::string name, Mcu &mcu, unsigned index)
        : name_(std::move(name)), mcu_(mcu), index_(index) {}

    std::string kind() const override { return "uart"; }
    Value get() const override { return Value{static_cast<std::int64_t>(sent_)}; }
    void set(const Value &value) override;
    std::size_t subscribe(EndpointSink sink) override;
    void unsubscribe(std::size_t id) override { sinks_.erase(id); }

    bool can_receive() const { return mcu_.uart_can_receive(index_); }
    void poll(std::uint64_t now_ns);

private:
    std::string name_;
    Mcu &mcu_;
    unsigned index_;
    std::uint64_t sent_{0};
    std::deque<std::uint8_t> rx_pending_;
    std::map<std::size_t, EndpointSink> sinks_;
    std::size_t next_id_{1};
};

} // namespace sim
