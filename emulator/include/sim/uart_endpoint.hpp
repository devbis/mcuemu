#pragma once

#include "sim/endpoint.hpp"
#include "sim/mcu.hpp"

#include <deque>
#include <string>
#include <string_view>
#include <vector>

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
    // Queues one frame behind the pending bytes (UartFramesEndpoint).
    void queue(const UartFrame &frame) { rx_pending_.push_back(frame); }
    void poll(std::uint64_t now_ns);

private:
    std::string name_;
    Mcu &mcu_;
    unsigned index_;
    std::uint64_t sent_{0};
    std::deque<UartFrame> rx_pending_;
    std::map<std::size_t, EndpointSink> sinks_;
    std::size_t next_id_{1};
};

// Frame view of a UART whose MCU models the 9th bit and line errors (kind
// "uart-frames", named <uart>.frames; see docs/system-config.md). Frames
// are written as tokens (parse_uart_frames):
//   events:  the frames the UART transmitted (a string of tokens), stamped
//            like the byte events; only while subscribed (the MCU records
//            frames only then);
//   set():   a string of tokens, an integer 0..255 or bytes (plain frames);
//            they share the byte endpoint's queue, in order;
//   get():   frames reported so far.
// The byte endpoint keeps reporting every transmitted byte.
class UartFramesEndpoint final : public Endpoint {
public:
    UartFramesEndpoint(std::string name, Mcu &mcu, unsigned index, UartEndpoint &bytes)
        : name_(std::move(name)), mcu_(mcu), index_(index), bytes_(bytes) {}

    std::string kind() const override { return "uart-frames"; }
    Value get() const override { return Value{static_cast<std::int64_t>(sent_)}; }
    void set(const Value &value) override;
    std::size_t subscribe(EndpointSink sink) override;
    void unsubscribe(std::size_t id) override;

    bool can_receive() const { return bytes_.can_receive(); }
    void poll(std::uint64_t now_ns);

private:
    std::string name_;
    Mcu &mcu_;
    unsigned index_;
    UartEndpoint &bytes_;
    std::uint64_t sent_{0};
    std::map<std::size_t, EndpointSink> sinks_;
    std::size_t next_id_{1};
};

// Frame tokens, separated by spaces or commas: "break", or a byte (C
// notation: 65, 0x41, 0101) followed by flags, each ":flag": "9=0" / "9=1"
// (explicit 9th bit), "perr" (the inverse of the expected 9th bit), "fe"
// (first stop bit wrong), "fe2" (second stop bit wrong). Example:
// "0x41 0x42:perr 0x43:9=1:fe2 break". Throws std::invalid_argument.
std::vector<UartFrame> parse_uart_frames(std::string_view text);
// The inverse; a frame without a 9th bit is the byte alone ("0x41").
std::string format_uart_frames(const std::vector<UartFrame> &frames);

} // namespace sim
