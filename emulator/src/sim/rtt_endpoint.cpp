#include "sim/rtt_endpoint.hpp"

#include <cstring>

namespace sim {

namespace {

constexpr std::uint32_t DescriptorSize = 24;
constexpr std::uint32_t FirstDescriptor = 24; // after id[16] and the two counts
constexpr std::uint32_t MaxBufferSize = 1u << 20;

} // namespace

std::uint32_t RttEndpoint::load32(std::uint32_t address) {
    std::uint8_t raw[4];
    mcu_.read_memory(address, raw, 4);
    return static_cast<std::uint32_t>(raw[0]) | (static_cast<std::uint32_t>(raw[1]) << 8) |
           (static_cast<std::uint32_t>(raw[2]) << 16) |
           (static_cast<std::uint32_t>(raw[3]) << 24);
}

// Locates the up/down descriptor of `channel`; false while the control block
// is missing or implausible.
bool RttEndpoint::find(bool up, Descriptor &out) {
    try {
        char id[16];
        mcu_.read_memory(address_, reinterpret_cast<std::uint8_t *>(id), sizeof id);
        if (std::memcmp(id, "SEGGER RTT", 10) != 0)
            return false;
        const std::uint32_t max_up = load32(address_ + 16);
        const std::uint32_t max_down = load32(address_ + 20);
        if (max_up > 16 || max_down > 16 || channel_ >= (up ? max_up : max_down))
            return false;
        out.address = address_ + FirstDescriptor +
                      (up ? 0 : max_up * DescriptorSize) + channel_ * DescriptorSize;
        out.buffer = load32(out.address + 4);
        out.size = load32(out.address + 8);
        out.write_offset = load32(out.address + 12);
        out.read_offset = load32(out.address + 16);
        return out.size != 0 && out.size <= MaxBufferSize &&
               out.write_offset < out.size && out.read_offset < out.size;
    } catch (const std::exception &) {
        return false; // memory not mapped (yet): try again later
    }
}

void RttEndpoint::set(const Value &value) {
    if (const auto *bytes = std::get_if<Bytes>(&value))
        down_pending_.insert(down_pending_.end(), bytes->begin(), bytes->end());
    else if (const auto *text = std::get_if<std::string>(&value))
        down_pending_.insert(down_pending_.end(), text->begin(), text->end());
    else
        throw std::invalid_argument("RTT takes bytes or a string");
}

std::size_t RttEndpoint::subscribe(EndpointSink sink) {
    const std::size_t id = next_id_++;
    sinks_.emplace(id, std::move(sink));
    return id;
}

void RttEndpoint::poll(std::uint64_t now_ns) {
    if (now_ns < next_poll_ns_)
        return;
    next_poll_ns_ = now_ns + interval_ns_;
    Descriptor desc;
    try {
        if (find(true, desc) && desc.write_offset != desc.read_offset) {
            Bytes data;
            std::uint32_t at = desc.read_offset;
            while (at != desc.write_offset) {
                const std::uint32_t run = (at < desc.write_offset ? desc.write_offset
                                                                  : desc.size) - at;
                const std::size_t first = data.size();
                data.resize(first + run);
                mcu_.read_memory(desc.buffer + at, data.data() + first, run);
                at = (at + run) % desc.size;
            }
            std::uint8_t raw[4] = {static_cast<std::uint8_t>(at),
                                   static_cast<std::uint8_t>(at >> 8),
                                   static_cast<std::uint8_t>(at >> 16),
                                   static_cast<std::uint8_t>(at >> 24)};
            mcu_.write_memory(desc.address + 16, raw, 4); // RdOff
            received_ += data.size();
            const EndpointEvent event{now_ns, name_, Value{std::move(data)}};
            const auto snapshot = sinks_;
            for (const auto &entry : snapshot)
                entry.second(event);
        }
        if (!down_pending_.empty() && find(false, desc)) {
            const std::uint32_t free_bytes =
                desc.read_offset > desc.write_offset
                    ? desc.read_offset - desc.write_offset - 1
                    : desc.size - (desc.write_offset - desc.read_offset) - 1;
            std::uint32_t at = desc.write_offset;
            std::uint32_t written = 0;
            while (written < free_bytes && !down_pending_.empty()) {
                const std::uint8_t byte = down_pending_.front();
                down_pending_.pop_front();
                mcu_.write_memory(desc.buffer + at, &byte, 1);
                at = (at + 1) % desc.size;
                ++written;
            }
            if (written != 0) {
                std::uint8_t raw[4] = {static_cast<std::uint8_t>(at),
                                       static_cast<std::uint8_t>(at >> 8),
                                       static_cast<std::uint8_t>(at >> 16),
                                       static_cast<std::uint8_t>(at >> 24)};
                mcu_.write_memory(desc.address + 12, raw, 4); // WrOff
            }
        }
    } catch (const std::exception &) {
        // The firmware moved or unmapped the buffer; try again next time.
    }
}

} // namespace sim
