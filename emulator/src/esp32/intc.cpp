#include "intc.hpp"

#include <algorithm>

namespace esp32emu {

namespace {
constexpr std::uint32_t PlicEnable = 0x0, PlicType = 0x4, PlicClear = 0x8, PlicEmipStatus = 0xc,
                        PlicPri0 = 0x10, PlicThresh = 0x90, PlicConf = 0x3fc;

} // namespace

Intc::Intc(std::function<void(int)> deliver) : deliver_(std::move(deliver)) {}

std::uint32_t Intc::line_levels() const {
    std::uint32_t lines = 0;
    for (unsigned source = 0; source < Sources; ++source)
        if (level_[source] && route_[source] != 0)
            lines |= 1u << route_[source];
    return lines;
}

void Intc::evaluate() {
    const std::uint32_t lines = line_levels();
    // Edge-typed lines latch a rising edge until software clears it.
    edge_pending_ |= lines & ~last_lines_ & type_;
    last_lines_ = lines;
    const std::uint32_t pending = (lines & ~type_) | (edge_pending_ & type_);

    int best = -1;
    std::uint32_t best_priority = 0;
    for (unsigned line = 1; line < CpuLines; ++line) {
        if (!(pending >> line & 1u) || !(enable_ >> line & 1u))
            continue;
        const std::uint32_t priority = priority_[line];
        if (priority >= std::max<std::uint32_t>(threshold_, 1) && priority >= best_priority) {
            best = static_cast<int>(line); // ">=": the higher line wins a tie
            best_priority = priority;
        }
    }
    if (best != delivered_) {
        delivered_ = best;
        deliver_(best);
    }
}

void Intc::set_source(unsigned source, bool level) {
    if (source >= Sources || level_[source] == level)
        return;
    level_[source] = level;
    evaluate();
}

std::uint32_t Intc::matrix_read(std::uint32_t offset) const {
    offset &= ~3u;
    if (offset / 4 < Sources && offset < status_offset_)
        return route_[offset / 4];
    if (offset >= status_offset_ && offset < status_offset_ + 12) { // raw source status, 32 per register
        std::uint32_t bits = 0;
        const unsigned first = (offset - status_offset_) / 4 * 32;
        for (unsigned i = 0; i < 32 && first + i < Sources; ++i)
            bits |= static_cast<std::uint32_t>(level_[first + i]) << i;
        return bits;
    }
    return 0;
}

void Intc::matrix_write(std::uint32_t offset, std::uint32_t value) {
    offset &= ~3u;
    if (offset / 4 < Sources && offset < status_offset_) {
        route_[offset / 4] = static_cast<std::uint8_t>(value & 0x1f);
        evaluate();
    }
}

std::uint32_t Intc::plic_read(std::uint32_t offset) const {
    offset &= ~3u;
    switch (offset) {
    case PlicEnable: return enable_;
    case PlicType: return type_;
    case PlicClear: return 0;
    case PlicEmipStatus: return (line_levels() & ~type_) | (edge_pending_ & type_);
    case PlicThresh: return threshold_;
    case PlicConf: return conf_;
    default:
        if (offset >= PlicPri0 && offset < PlicPri0 + 4 * CpuLines)
            return priority_[(offset - PlicPri0) / 4];
        return 0;
    }
}

void Intc::plic_write(std::uint32_t offset, std::uint32_t value) {
    offset &= ~3u;
    switch (offset) {
    case PlicEnable: enable_ = value; break;
    case PlicType: type_ = value; break;
    case PlicClear: edge_pending_ &= ~value; break;
    case PlicThresh: threshold_ = value & 0xff; break;
    case PlicConf: conf_ = value; break;
    default:
        if (offset >= PlicPri0 && offset < PlicPri0 + 4 * CpuLines)
            priority_[(offset - PlicPri0) / 4] = value & 0xf;
        break;
    }
    evaluate();
}

} // namespace esp32emu
