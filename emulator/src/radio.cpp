#include "tc32emu/radio.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace tc32emu {

std::uint64_t VirtualAir::transmit(std::string source, std::string destination,
                                  RadioProtocol protocol, std::uint8_t channel,
                                  std::uint64_t start_ns,
                                  std::vector<std::uint8_t> payload,
                                  std::uint32_t bit_rate) {
    if (source.empty())
        throw std::invalid_argument("radio sender must have a node name");
    if ((protocol == RadioProtocol::Zigbee802154 &&
         (channel < 11 || channel > 26)) ||
        (protocol == RadioProtocol::Ble && channel > 39))
        throw std::invalid_argument("radio channel is invalid for the selected protocol");
    if (bit_rate == 0)
        bit_rate = protocol == RadioProtocol::Zigbee802154 ? 250000u : 1000000u;
    if (bit_rate == 0)
        throw std::invalid_argument("radio bit rate must be nonzero");

    const std::uint64_t overhead_bytes =
        protocol == RadioProtocol::Zigbee802154 ? 6u : 10u;
    constexpr std::uint64_t nanos_per_second = 1000000000u;
    const std::uint64_t max_value = std::numeric_limits<std::uint64_t>::max();
    if (payload.size() > max_value / 8u - overhead_bytes)
        throw std::overflow_error("radio payload is too large to time");
    const std::uint64_t bits =
        (static_cast<std::uint64_t>(payload.size()) + overhead_bytes) * 8u;
    const std::uint64_t whole_seconds = bits / bit_rate;
    const std::uint64_t remainder_bits = bits % bit_rate;
    if (whole_seconds > max_value / nanos_per_second)
        throw std::overflow_error("radio frame duration overflows virtual clock");
    const std::uint64_t whole_ns = whole_seconds * nanos_per_second;
    const std::uint64_t fractional_ns =
        (remainder_bits * nanos_per_second + bit_rate - 1u) / bit_rate;
    if (fractional_ns > max_value - whole_ns)
        throw std::overflow_error("radio frame duration overflows virtual clock");
    const std::uint64_t duration_ns = whole_ns + fractional_ns;
    if (duration_ns > std::numeric_limits<std::uint64_t>::max() - start_ns)
        throw std::overflow_error("radio frame end time overflows virtual clock");

    RadioFrame frame;
    frame.id = next_id_++;
    frame.source = std::move(source);
    frame.destination = std::move(destination);
    frame.protocol = protocol;
    frame.channel = channel;
    frame.start_ns = start_ns;
    frame.end_ns = start_ns + duration_ns;
    frame.payload = std::move(payload);
    frames_.push_back(std::move(frame));
    return frames_.back().id;
}

std::vector<RadioFrame> VirtualAir::receive(const std::string &node,
                                           RadioProtocol protocol,
                                           std::uint8_t channel,
                                           std::uint64_t at_ns) {
    std::vector<RadioFrame> received;
    for (const RadioFrame &frame : frames_) {
        if (frame.end_ns > at_ns || frame.source == node ||
            frame.protocol != protocol || frame.channel != channel ||
            (!frame.destination.empty() && frame.destination != node))
            continue;

        const bool already_delivered = std::any_of(
            delivered_.begin(), delivered_.end(), [&](const auto &item) {
                return item.first == frame.id && item.second == node;
            });
        if (already_delivered)
            continue;

        const bool collided = std::any_of(
            frames_.begin(), frames_.end(), [&](const RadioFrame &other) {
                if (other.id == frame.id || other.protocol != frame.protocol ||
                    other.channel != frame.channel)
                    return false;
                return other.start_ns < frame.end_ns && frame.start_ns < other.end_ns;
            });
        delivered_.emplace_back(frame.id, node);
        if (!collided) {
            receptions_.push_back({frame.id, node});
            received.push_back(frame);
        }
    }
    std::sort(received.begin(), received.end(), [](const auto &left, const auto &right) {
        if (left.end_ns != right.end_ns)
            return left.end_ns < right.end_ns;
        return left.id < right.id;
    });
    return received;
}

VirtualRadioNode::VirtualRadioNode(VirtualAir &air, std::string name)
    : air_(air), name_(std::move(name)) {
    if (name_.empty())
        throw std::invalid_argument("virtual radio node must have a name");
}

std::uint64_t VirtualRadioNode::transmit(
    std::string destination, RadioProtocol protocol, std::uint8_t channel,
    std::uint64_t start_ns, std::vector<std::uint8_t> payload,
    std::uint32_t bit_rate) {
    return air_.transmit(name_, std::move(destination), protocol, channel,
                         start_ns, std::move(payload), bit_rate);
}

std::vector<RadioFrame> VirtualRadioNode::receive(RadioProtocol protocol,
                                                   std::uint8_t channel,
                                                   std::uint64_t at_ns) {
    return air_.receive(name_, protocol, channel, at_ns);
}

const char *protocol_name(RadioProtocol protocol) noexcept {
    return protocol == RadioProtocol::Zigbee802154 ? "zigbee-802.15.4" : "ble";
}

} // namespace tc32emu
