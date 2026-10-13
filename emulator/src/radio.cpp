#include "mcuemu/radio.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mcuemu {

std::uint64_t VirtualAir::airtime_ns(RadioProtocol protocol,
                                     std::size_t payload_bytes,
                                     std::uint32_t bit_rate) {
    if (bit_rate == 0)
        bit_rate = protocol == RadioProtocol::Zigbee802154 ? 250000u : 1000000u;
    const std::uint64_t overhead_bytes =
        protocol == RadioProtocol::Zigbee802154 ? 6u : 10u;
    constexpr std::uint64_t nanos_per_second = 1000000000u;
    const std::uint64_t max_value = std::numeric_limits<std::uint64_t>::max();
    if (payload_bytes > max_value / 8u - overhead_bytes)
        throw std::overflow_error("radio payload is too large to time");
    const std::uint64_t bits =
        (static_cast<std::uint64_t>(payload_bytes) + overhead_bytes) * 8u;
    const std::uint64_t whole_seconds = bits / bit_rate;
    const std::uint64_t remainder_bits = bits % bit_rate;
    if (whole_seconds > max_value / nanos_per_second)
        throw std::overflow_error("radio frame duration overflows virtual clock");
    const std::uint64_t whole_ns = whole_seconds * nanos_per_second;
    const std::uint64_t fractional_ns =
        (remainder_bits * nanos_per_second + bit_rate - 1u) / bit_rate;
    if (fractional_ns > max_value - whole_ns)
        throw std::overflow_error("radio frame duration overflows virtual clock");
    return whole_ns + fractional_ns;
}

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
    const std::uint64_t duration_ns = airtime_ns(protocol, payload.size(), bit_rate);
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
    if (const RadioNodeConfig *sender = node(frame.source))
        frame.tx_power_dbm = sender->tx_power_dbm;
    frames_.push_back(std::move(frame));
    // Observers may transmit or schedule work; give them a stable copy.
    const RadioFrame registered = frames_.back();
    for (const FrameObserver &observer : observers_)
        observer(registered);
    return registered.id;
}

const RadioNodeConfig *VirtualAir::node(const std::string &name) const {
    const auto it = nodes_.find(name);
    return it == nodes_.end() ? nullptr : &it->second;
}

double VirtualAir::frequency_mhz(RadioProtocol protocol, std::uint8_t channel) {
    if (protocol == RadioProtocol::Zigbee802154)
        return 2405.0 + 5.0 * (static_cast<int>(channel) - 11);
    if (channel == 37) return 2402.0;
    if (channel == 38) return 2426.0;
    if (channel == 39) return 2480.0;
    return channel <= 10 ? 2404.0 + 2.0 * channel : 2428.0 + 2.0 * (channel - 11);
}

namespace {

std::uint64_t splitmix(std::uint64_t &state) {
    std::uint64_t z = (state += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

} // namespace

double VirtualAir::path_loss_db(const std::string &from,
                                const std::string &to) const {
    if (propagation_.model == Propagation::Model::Ideal)
        return 0.0;
    const RadioNodeConfig origin;
    const RadioNodeConfig *a = node(from);
    const RadioNodeConfig *b = node(to);
    if (!a) a = &origin;
    if (!b) b = &origin;
    const double dx = a->x - b->x, dy = a->y - b->y, dz = a->z - b->z;
    const double distance = std::max(std::sqrt(dx * dx + dy * dy + dz * dz), 0.1);
    double loss = propagation_.reference_loss_db +
                  10.0 * propagation_.exponent * std::log10(distance);
    if (propagation_.shadowing_sigma_db > 0.0) {
        // One fixed Gaussian draw per unordered pair of nodes.
        std::uint64_t state = propagation_.seed;
        for (char c : std::min(from, to) + '\n' + std::max(from, to))
            state = (state ^ static_cast<std::uint8_t>(c)) * 1099511628211ull;
        const double u1 = (static_cast<double>(splitmix(state) >> 11) + 1.0) / 9007199254740993.0;
        const double u2 = static_cast<double>(splitmix(state) >> 11) / 9007199254740992.0;
        loss += propagation_.shadowing_sigma_db *
                std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.14159265358979323846 * u2);
    }
    return loss;
}

double VirtualAir::rx_power_dbm(const RadioFrame &frame,
                                const std::string &node_name) const {
    return frame.tx_power_dbm - path_loss_db(frame.source, node_name);
}

// LogDistance reception test; `rssi` is the received power of `frame`.
bool VirtualAir::decodable(const RadioFrame &frame, const std::string &node_name,
                           double &rssi) const {
    constexpr double occupied_mhz = 2.0;
    rssi = rx_power_dbm(frame, node_name);
    const RadioNodeConfig defaults;
    const RadioNodeConfig *receiver = node(node_name);
    if (rssi < (receiver ? receiver : &defaults)->sensitivity_dbm)
        return false;
    const double f0 = frequency_mhz(frame.protocol, frame.channel);
    double interference_mw = std::pow(10.0, propagation_.noise_floor_dbm / 10.0);
    for (const RadioFrame &other : frames_) {
        if (other.id == frame.id || !(other.start_ns < frame.end_ns &&
                                      frame.start_ns < other.end_ns))
            continue;
        if (other.source == node_name)
            return false; // half duplex: the node was transmitting
        const double f1 = frequency_mhz(other.protocol, other.channel);
        const double overlap =
            std::min(f0, f1) + occupied_mhz - std::max(f0, f1);
        if (overlap <= 0.0)
            continue;
        interference_mw += (std::min(overlap, occupied_mhz) / occupied_mhz) *
                           std::pow(10.0, rx_power_dbm(other, node_name) / 10.0);
    }
    const double sinr_db = rssi - 10.0 * std::log10(interference_mw);
    return sinr_db >= propagation_.sinr_threshold_db;
}

std::vector<RadioFrame> VirtualAir::receive(const std::string &node,
                                           RadioProtocol protocol,
                                           std::uint8_t channel,
                                           std::uint64_t at_ns,
                                           std::uint64_t after_ns) {
    std::vector<RadioFrame> received;
    for (const RadioFrame &frame : frames_) {
        if (frame.end_ns > at_ns || frame.end_ns <= after_ns ||
            frame.source == node ||
            frame.protocol != protocol || frame.channel != channel ||
            (!frame.destination.empty() && frame.destination != node))
            continue;

        const bool already_delivered = std::any_of(
            delivered_.begin(), delivered_.end(), [&](const auto &item) {
                return item.first == frame.id && item.second == node;
            });
        if (already_delivered)
            continue;

        if (propagation_.model == Propagation::Model::LogDistance) {
            delivered_.emplace_back(frame.id, node);
            double rssi = 0;
            if (decodable(frame, node, rssi)) {
                receptions_.push_back({frame.id, node});
                RadioFrame heard = frame;
                heard.rssi_dbm = static_cast<int>(std::lround(rssi));
                received.push_back(std::move(heard));
            }
            continue;
        }
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
                                                   std::uint64_t at_ns,
                                                   std::uint64_t after_ns) {
    return air_.receive(name_, protocol, channel, at_ns, after_ns);
}

const char *protocol_name(RadioProtocol protocol) noexcept {
    return protocol == RadioProtocol::Zigbee802154 ? "zigbee-802.15.4" : "ble";
}

} // namespace mcuemu
