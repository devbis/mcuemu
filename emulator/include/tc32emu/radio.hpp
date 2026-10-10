#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace tc32emu {

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
};

struct RadioReception {
    std::uint64_t frame_id{0};
    std::string receiver;
};

class VirtualAir {
public:
    std::uint64_t transmit(std::string source, std::string destination,
                           RadioProtocol protocol, std::uint8_t channel,
                           std::uint64_t start_ns,
                           std::vector<std::uint8_t> payload,
                           std::uint32_t bit_rate = 0);
    std::vector<RadioFrame> receive(const std::string &node,
                                    RadioProtocol protocol,
                                    std::uint8_t channel,
                                    std::uint64_t at_ns);
    const std::vector<RadioFrame> &frames() const noexcept { return frames_; }
    const std::vector<RadioReception> &receptions() const noexcept {
        return receptions_;
    }

private:
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
                                    std::uint64_t at_ns);
    const std::string &name() const noexcept { return name_; }

private:
    VirtualAir &air_;
    std::string name_;
};

const char *protocol_name(RadioProtocol protocol) noexcept;

} // namespace tc32emu
