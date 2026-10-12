#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace sim::ws {

// The parts of RFC 6455 the control server needs: the opening handshake and
// frame encoding/decoding (text messages, ping/pong, close, fragmentation).

std::string sha1(const std::string &data); // 20 raw bytes
std::string base64(const std::string &data);
std::string accept_key(const std::string &client_key);

struct Request {
    std::string method;
    std::string path;
    std::string query;
    std::map<std::string, std::string> headers; // lower-case names
};

// Consumes a complete HTTP request head from `buffer` (up to the blank line).
// Returns nullopt while it is incomplete; throws std::invalid_argument when it
// is malformed or larger than 16 KiB.
std::optional<Request> parse_request(std::string &buffer);

// True for a valid WebSocket upgrade request (version 13, key present).
bool is_upgrade(const Request &request);
std::string handshake_response(const Request &request);
std::string http_response(int status, const std::string &reason, const std::string &body = "");

enum class Opcode : std::uint8_t {
    Continuation = 0x0,
    Text = 0x1,
    Binary = 0x2,
    Close = 0x8,
    Ping = 0x9,
    Pong = 0xa,
};

// Server-to-client frames are never masked.
std::string encode_frame(Opcode opcode, const std::string &payload);
std::string encode_close(std::uint16_t code, const std::string &reason = "");

class ProtocolError : public std::runtime_error {
public:
    ProtocolError(std::uint16_t code, const std::string &message)
        : std::runtime_error(message), code_(code) {}
    std::uint16_t code() const noexcept { return code_; }

private:
    std::uint16_t code_;
};

struct Message {
    Opcode opcode; // Text, Binary, Close, Ping or Pong
    std::string payload;
};

// Decodes client frames. Fragmented data messages are reassembled; control
// frames may arrive between fragments. Throws ProtocolError (1002 protocol
// violation, 1009 too big) for malformed input.
class FrameParser {
public:
    explicit FrameParser(std::size_t max_message = 4u << 20) : max_message_(max_message) {}
    void feed(const char *data, std::size_t length) { buffer_.append(data, length); }
    // Next complete message, or nullopt if more bytes are needed.
    std::optional<Message> next();

private:
    std::size_t max_message_;
    std::string buffer_;
    std::string assembling_;
    Opcode assembling_opcode_{Opcode::Text};
    bool in_fragments_{false};
};

} // namespace sim::ws
