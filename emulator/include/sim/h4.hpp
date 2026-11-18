#pragma once

// Reassembles HCI H4 packets (type byte, header, payload) from a byte stream
// that may split or merge them arbitrarily, as a socket does.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <vector>

namespace sim {

class H4Framer {
public:
    using Packet = std::vector<std::uint8_t>;

    // Appends `size` bytes and calls `deliver` for every packet completed by
    // them. An unknown packet type (or an isochronous packet, which this
    // controller does not have) discards the buffered bytes and throws
    // std::invalid_argument: the stream cannot be resynchronised.
    void feed(const std::uint8_t *data, std::size_t size,
              const std::function<void(const Packet &)> &deliver) {
        buffer_.insert(buffer_.end(), data, data + size);
        for (;;) {
            if (buffer_.empty())
                return;
            std::size_t header = 0, length_at = 0, length_size = 0;
            switch (buffer_[0]) {
            case 0x01: header = 4; length_at = 3; length_size = 1; break; // command
            case 0x02: header = 5; length_at = 3; length_size = 2; break; // ACL data
            case 0x03: header = 4; length_at = 3; length_size = 1; break; // SCO data
            case 0x04: header = 3; length_at = 2; length_size = 1; break; // event
            default: {
                const std::uint8_t type = buffer_[0];
                buffer_.clear();
                throw std::invalid_argument("unsupported H4 packet type " + std::to_string(type));
            }
            }
            if (buffer_.size() < header)
                return;
            std::size_t length = buffer_[length_at];
            if (length_size == 2)
                length |= static_cast<std::size_t>(buffer_[length_at + 1]) << 8;
            if (buffer_.size() < header + length)
                return;
            Packet packet(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(header + length));
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(header + length));
            deliver(packet);
        }
    }

    std::size_t pending() const noexcept { return buffer_.size(); }
    void clear() noexcept { buffer_.clear(); }

private:
    Packet buffer_;
};

} // namespace sim
