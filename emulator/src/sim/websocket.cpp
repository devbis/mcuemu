#include "sim/websocket.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>

namespace sim::ws {

namespace {

std::uint32_t rotl(std::uint32_t value, unsigned bits) {
    return (value << bits) | (value >> (32 - bits));
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

std::string trim(const std::string &text) {
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos)
        return "";
    return text.substr(first, text.find_last_not_of(" \t") - first + 1);
}

} // namespace

std::string sha1(const std::string &data) {
    std::uint32_t h[5] = {0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0};
    std::string message = data;
    message.push_back(static_cast<char>(0x80));
    while (message.size() % 64 != 56)
        message.push_back(0);
    const std::uint64_t bits = static_cast<std::uint64_t>(data.size()) * 8;
    for (int i = 7; i >= 0; --i)
        message.push_back(static_cast<char>(bits >> (i * 8)));
    for (std::size_t chunk = 0; chunk < message.size(); chunk += 64) {
        std::uint32_t w[80];
        for (int i = 0; i < 16; ++i)
            w[i] = (static_cast<std::uint32_t>(static_cast<std::uint8_t>(message[chunk + i * 4])) << 24) |
                   (static_cast<std::uint32_t>(static_cast<std::uint8_t>(message[chunk + i * 4 + 1])) << 16) |
                   (static_cast<std::uint32_t>(static_cast<std::uint8_t>(message[chunk + i * 4 + 2])) << 8) |
                   static_cast<std::uint32_t>(static_cast<std::uint8_t>(message[chunk + i * 4 + 3]));
        for (int i = 16; i < 80; ++i)
            w[i] = rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int i = 0; i < 80; ++i) {
            std::uint32_t f, k;
            if (i < 20) { f = (b & c) | (~b & d); k = 0x5a827999; }
            else if (i < 40) { f = b ^ c ^ d; k = 0x6ed9eba1; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8f1bbcdc; }
            else { f = b ^ c ^ d; k = 0xca62c1d6; }
            const std::uint32_t temp = rotl(a, 5) + f + e + k + w[i];
            e = d; d = c; c = rotl(b, 30); b = a; a = temp;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
    }
    std::string digest;
    for (std::uint32_t word : h)
        for (int i = 3; i >= 0; --i)
            digest.push_back(static_cast<char>(word >> (i * 8)));
    return digest;
}

std::string base64(const std::string &data) {
    static const char table[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (std::size_t i = 0; i < data.size(); i += 3) {
        const std::uint32_t chunk =
            (static_cast<std::uint32_t>(static_cast<std::uint8_t>(data[i])) << 16) |
            (i + 1 < data.size() ? static_cast<std::uint32_t>(static_cast<std::uint8_t>(data[i + 1])) << 8 : 0) |
            (i + 2 < data.size() ? static_cast<std::uint32_t>(static_cast<std::uint8_t>(data[i + 2])) : 0);
        out.push_back(table[(chunk >> 18) & 63]);
        out.push_back(table[(chunk >> 12) & 63]);
        out.push_back(i + 1 < data.size() ? table[(chunk >> 6) & 63] : '=');
        out.push_back(i + 2 < data.size() ? table[chunk & 63] : '=');
    }
    return out;
}

std::string accept_key(const std::string &client_key) {
    return base64(sha1(client_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));
}

std::optional<Request> parse_request(std::string &buffer) {
    constexpr std::size_t MaxHead = 16 * 1024;
    const std::size_t end = buffer.find("\r\n\r\n");
    if (end == std::string::npos) {
        if (buffer.size() > MaxHead)
            throw std::invalid_argument("request head too large");
        return std::nullopt;
    }
    if (end > MaxHead)
        throw std::invalid_argument("request head too large");
    const std::string head = buffer.substr(0, end);
    buffer.erase(0, end + 4);

    Request request;
    std::size_t line_end = head.find("\r\n");
    const std::string line = head.substr(0, line_end);
    const std::size_t sp1 = line.find(' ');
    const std::size_t sp2 = line.rfind(' ');
    if (sp1 == std::string::npos || sp2 == sp1 || line.compare(sp2 + 1, 5, "HTTP/") != 0)
        throw std::invalid_argument("malformed request line");
    request.method = line.substr(0, sp1);
    std::string target = line.substr(sp1 + 1, sp2 - sp1 - 1);
    if (const std::size_t q = target.find('?'); q != std::string::npos) {
        request.query = target.substr(q + 1);
        target.resize(q);
    }
    request.path = target;
    while (line_end != std::string::npos) {
        const std::size_t start = line_end + 2;
        line_end = head.find("\r\n", start);
        const std::string header =
            head.substr(start, line_end == std::string::npos ? std::string::npos : line_end - start);
        if (header.empty())
            continue;
        const std::size_t colon = header.find(':');
        if (colon == std::string::npos)
            throw std::invalid_argument("malformed header");
        request.headers[lower(trim(header.substr(0, colon)))] = trim(header.substr(colon + 1));
    }
    return request;
}

bool is_upgrade(const Request &request) {
    const auto header = [&](const char *name) {
        const auto it = request.headers.find(name);
        return it == request.headers.end() ? std::string() : lower(it->second);
    };
    return request.method == "GET" && header("upgrade") == "websocket" &&
           header("connection").find("upgrade") != std::string::npos &&
           header("sec-websocket-version") == "13" &&
           request.headers.count("sec-websocket-key") != 0;
}

std::string handshake_response(const Request &request) {
    return "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
           "Connection: Upgrade\r\nSec-WebSocket-Accept: " +
           accept_key(request.headers.at("sec-websocket-key")) + "\r\n\r\n";
}

std::string http_response(int status, const std::string &reason, const std::string &body) {
    return "HTTP/1.1 " + std::to_string(status) + " " + reason +
           "\r\nContent-Type: text/plain\r\nContent-Length: " + std::to_string(body.size()) +
           "\r\nConnection: close\r\n\r\n" + body;
}

std::string encode_frame(Opcode opcode, const std::string &payload) {
    std::string frame;
    frame.push_back(static_cast<char>(0x80 | static_cast<std::uint8_t>(opcode)));
    if (payload.size() < 126) {
        frame.push_back(static_cast<char>(payload.size()));
    } else if (payload.size() <= 0xffff) {
        frame.push_back(126);
        frame.push_back(static_cast<char>(payload.size() >> 8));
        frame.push_back(static_cast<char>(payload.size()));
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; --i)
            frame.push_back(static_cast<char>(static_cast<std::uint64_t>(payload.size()) >> (i * 8)));
    }
    frame += payload;
    return frame;
}

std::string encode_close(std::uint16_t code, const std::string &reason) {
    std::string payload;
    payload.push_back(static_cast<char>(code >> 8));
    payload.push_back(static_cast<char>(code));
    payload += reason.substr(0, 100);
    return encode_frame(Opcode::Close, payload);
}

std::optional<Message> FrameParser::next() {
    for (;;) {
        if (buffer_.size() < 2)
            return std::nullopt;
        const auto b0 = static_cast<std::uint8_t>(buffer_[0]);
        const auto b1 = static_cast<std::uint8_t>(buffer_[1]);
        const bool fin = (b0 & 0x80) != 0;
        const auto opcode = static_cast<Opcode>(b0 & 0x0f);
        if ((b0 & 0x70) != 0)
            throw ProtocolError(1002, "reserved bits set");
        if ((b1 & 0x80) == 0)
            throw ProtocolError(1002, "client frames must be masked");
        std::uint64_t length = b1 & 0x7f;
        std::size_t offset = 2;
        if (length == 126) {
            if (buffer_.size() < 4)
                return std::nullopt;
            length = (static_cast<std::uint64_t>(static_cast<std::uint8_t>(buffer_[2])) << 8) |
                     static_cast<std::uint8_t>(buffer_[3]);
            offset = 4;
        } else if (length == 127) {
            if (buffer_.size() < 10)
                return std::nullopt;
            length = 0;
            for (int i = 0; i < 8; ++i)
                length = (length << 8) | static_cast<std::uint8_t>(buffer_[2 + i]);
            offset = 10;
        }
        const bool control = (static_cast<std::uint8_t>(opcode) & 0x8) != 0;
        if (control && (length > 125 || !fin))
            throw ProtocolError(1002, "invalid control frame");
        if (length > max_message_ || assembling_.size() + length > max_message_)
            throw ProtocolError(1009, "message too large");
        if (buffer_.size() < offset + 4 + length)
            return std::nullopt;
        const char *mask = buffer_.data() + offset;
        std::string payload(buffer_.data() + offset + 4, static_cast<std::size_t>(length));
        for (std::size_t i = 0; i < payload.size(); ++i)
            payload[i] = static_cast<char>(static_cast<std::uint8_t>(payload[i]) ^
                                           static_cast<std::uint8_t>(mask[i % 4]));
        buffer_.erase(0, offset + 4 + static_cast<std::size_t>(length));

        switch (opcode) {
        case Opcode::Ping:
        case Opcode::Pong:
        case Opcode::Close:
            return Message{opcode, std::move(payload)};
        case Opcode::Text:
        case Opcode::Binary:
            if (in_fragments_)
                throw ProtocolError(1002, "new message inside a fragmented one");
            if (fin)
                return Message{opcode, std::move(payload)};
            in_fragments_ = true;
            assembling_opcode_ = opcode;
            assembling_ = std::move(payload);
            break;
        case Opcode::Continuation:
            if (!in_fragments_)
                throw ProtocolError(1002, "unexpected continuation");
            assembling_ += payload;
            if (fin) {
                in_fragments_ = false;
                Message message{assembling_opcode_, std::move(assembling_)};
                assembling_.clear();
                return message;
            }
            break;
        default:
            throw ProtocolError(1002, "unknown opcode");
        }
    }
}

} // namespace sim::ws
