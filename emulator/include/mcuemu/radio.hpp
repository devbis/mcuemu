#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mcuemu {

enum class RadioProtocol {
    Zigbee802154,
    Ble,
};

struct RadioFrame {
    std::uint64_t id{0};
    std::string source;
    std::string destination; // Empty means broadcast.
    RadioProtocol protocol{RadioProtocol::Zigbee802154};
    std::uint8_t channel{11};
    std::uint64_t start_ns{0};
    std::uint64_t end_ns{0};
    std::vector<std::uint8_t> payload;
    // Physical layer. tx_power_dbm is set from the sender's node settings;
    // rssi_dbm is filled in per receiver on the copies returned by receive().
    // The ideal model reports a fixed -50 dBm.
    double tx_power_dbm{0};
    int rssi_dbm{-50};
};

// Where a node is and how it transmits/receives (used by the loss model).
struct RadioNodeConfig {
    double x{0}, y{0}, z{0}; // meters
    double tx_power_dbm{0};
    double sensitivity_dbm{-97};
};

// Radio propagation. Ideal: any receiver on the same protocol and channel
// hears every frame, and any time overlap on that channel is a collision (the
// original behavior). LogDistance: loss = reference + 10 n log10(d / 1 m) plus
// optional per-link shadowing; a frame is received if it is above the
// receiver's sensitivity and its SINR (signal over noise plus the interference
// of every time-overlapping frame, scaled by the overlap of the occupied
// 2 MHz channels on the shared frequency axis) reaches the threshold; a node
// does not receive while it transmits.
struct Propagation {
    enum class Model { Ideal, LogDistance };
    Model model{Model::Ideal};
    double reference_loss_db{40.0}; // at 1 m, free space at 2.4 GHz
    double exponent{2.0};
    double shadowing_sigma_db{0.0}; // log-normal, fixed per node pair
    double noise_floor_dbm{-100.0};
    double sinr_threshold_db{6.0};
    std::uint64_t seed{1};
};

struct RadioReception {
    std::uint64_t frame_id{0};
    std::string receiver;
};

class VirtualAir {
public:
    using FrameObserver = std::function<void(const RadioFrame &)>;

    void set_propagation(const Propagation &propagation) {
        propagation_ = propagation;
    }
    const Propagation &propagation() const noexcept { return propagation_; }
    void set_node(const std::string &name, const RadioNodeConfig &config) {
        nodes_[name] = config;
    }
    const RadioNodeConfig *node(const std::string &name) const;
    // Loss between two nodes in dB (symmetric); 0 for the ideal model.
    double path_loss_db(const std::string &from, const std::string &to) const;
    // Called for every transmitted frame, as soon as it is registered.
    void add_observer(FrameObserver observer) {
        observers_.push_back(std::move(observer));
    }
    // Center frequency on the shared 2.4 GHz axis: IEEE 802.15.4 channels
    // 11..26 are 2405 + 5 (n - 11) MHz; BLE channel indexes 0..39 map to
    // 2402..2480 MHz (advertising 37/38/39 at 2402/2426/2480).
    static double frequency_mhz(RadioProtocol protocol, std::uint8_t channel);
    // Time on air of `payload_bytes` including protocol overhead.
    static std::uint64_t airtime_ns(RadioProtocol protocol,
                                    std::size_t payload_bytes,
                                    std::uint32_t bit_rate = 0);

    std::uint64_t transmit(std::string source, std::string destination,
                           RadioProtocol protocol, std::uint8_t channel,
                           std::uint64_t start_ns,
                           std::vector<std::uint8_t> payload,
                           std::uint32_t bit_rate = 0);
    std::vector<RadioFrame> receive(const std::string &node,
                                    RadioProtocol protocol,
                                    std::uint8_t channel,
                                    std::uint64_t at_ns,
                                    std::uint64_t after_ns = 0);
    const std::vector<RadioFrame> &frames() const noexcept { return frames_; }
    const std::vector<RadioReception> &receptions() const noexcept {
        return receptions_;
    }

private:
    bool decodable(const RadioFrame &frame, const std::string &node,
                   double &rssi) const;
    double rx_power_dbm(const RadioFrame &frame, const std::string &node) const;

    Propagation propagation_;
    std::map<std::string, RadioNodeConfig> nodes_;
    std::vector<FrameObserver> observers_;
    std::uint64_t next_id_{1};
    std::vector<RadioFrame> frames_;
    std::vector<RadioReception> receptions_;
    std::vector<std::pair<std::uint64_t, std::string>> delivered_;
};

class VirtualRadioNode {
public:
    VirtualRadioNode(VirtualAir &air, std::string name);

    std::uint64_t transmit(std::string destination, RadioProtocol protocol,
                           std::uint8_t channel, std::uint64_t start_ns,
                           std::vector<std::uint8_t> payload,
                           std::uint32_t bit_rate = 0);
    std::vector<RadioFrame> receive(RadioProtocol protocol,
                                    std::uint8_t channel,
                                    std::uint64_t at_ns,
                                    std::uint64_t after_ns = 0);
    const std::string &name() const noexcept { return name_; }

private:
    VirtualAir &air_;
    std::string name_;
};

const char *protocol_name(RadioProtocol protocol) noexcept;

} // namespace mcuemu
