#include "blocks.hpp"

#include <algorithm>
#include <cstring>

namespace bl702emu {

namespace {
constexpr std::uint32_t MtimeOffset = 0xbff8, MtimecmpOffset = 0x4000, MsipOffset = 0x0;
constexpr std::uint64_t CyclesPerTick = Bl702::ClockHz / Clint::TimebaseHz;
} // namespace

// ---- CLIC -------------------------------------------------------------------------------------------------

Clic::Clic(Bl702 &chip) : Block(chip, Bl702::ClicBase, 0x1000) {}

std::uint32_t Clic::read(std::uint32_t offset, unsigned size) {
    std::uint32_t value = 0;
    for (unsigned b = 0; b < size && offset + b < sizeof regs_; ++b)
        value |= static_cast<std::uint32_t>(regs_[offset + b]) << (8 * b);
    return value;
}

void Clic::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    for (unsigned b = 0; b < size && offset + b < sizeof regs_; ++b)
        regs_[offset + b] = static_cast<std::uint8_t>(value >> (8 * b));
}

void Clic::set_line(unsigned irq, bool asserted) {
    if (irq < Interrupts)
        line_[irq] = asserted;
}

int Clic::pick(std::uint32_t mil, std::uint32_t thresh, std::uint32_t &level, bool claim) {
    // An interrupt is taken above the active level (any level when none is active, mil 0) and at least the
    // threshold; the highest level wins and the lowest number breaks a tie. Level 0 is taken when nothing is
    // active: the BFLB SoC gives its machine timer level 0 (inferred from the Zephyr image, which takes it).
    int best = -1;
    std::uint32_t best_level = 0;
    for (unsigned irq = 0; irq < Interrupts; ++irq) {
        const bool enabled = (regs_[IntieOffset + irq] & 1) != 0;
        const bool pending = (regs_[IntipOffset + irq] & 1) != 0 || line_[irq];
        const std::uint32_t irq_level = regs_[IntcfgOffset + irq];
        const bool eligible = (mil == 0 || irq_level > mil) && irq_level >= thresh;
        if (!enabled || !pending || !eligible)
            continue;
        if (best < 0 || irq_level > best_level) {
            best = static_cast<int>(irq);
            best_level = irq_level;
        }
    }
    if (best >= 0) {
        level = best_level;
        if (claim)
            regs_[IntipOffset + best] &= ~1u;
    }
    return best;
}

// ---- CLINT ------------------------------------------------------------------------------------------------

Clint::Clint(Bl702 &chip, Clic &clic) : Block(chip, Bl702::ClintBase, 0x10000), clic_(clic) {}

void Clint::reset() {
    mtimecmp_ = ~std::uint64_t{0};
    mtime_offset_ = 0;
    msip_ = false;
    clic_.set_line(SoftwareIrq, false);
    clic_.set_line(TimerIrq, false);
}

std::uint64_t Clint::mtime(std::uint64_t now) const { return now / CyclesPerTick + mtime_offset_; }

namespace {
// A 64-bit register read or written 32 bits at a time.
std::uint32_t half(std::uint64_t value, std::uint32_t offset, unsigned size) {
    const std::uint64_t shifted = value >> (8 * (offset & 4u));
    return static_cast<std::uint32_t>(shifted) & (size == 1 ? 0xffu : size == 2 ? 0xffffu : 0xffffffffu);
}
} // namespace

std::uint32_t Clint::read(std::uint32_t offset, unsigned size) {
    if (offset == MsipOffset)
        return msip_ ? 1 : 0;
    if (offset >= MtimecmpOffset && offset < MtimecmpOffset + 8)
        return half(mtimecmp_, offset - MtimecmpOffset, size);
    if (offset >= MtimeOffset && offset < MtimeOffset + 8)
        return half(mtime(chip_.core().cycles()), offset - MtimeOffset, size);
    return 0;
}

void Clint::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    const std::uint64_t now = chip_.core().cycles();
    if (offset == MsipOffset) {
        msip_ = (value & 1) != 0;
        clic_.set_line(SoftwareIrq, msip_);
        return;
    }
    if (offset >= MtimecmpOffset && offset < MtimecmpOffset + 8) {
        const std::uint32_t shift = 8 * ((offset - MtimecmpOffset) & 4u);
        const std::uint64_t mask = std::uint64_t(size == 1 ? 0xffu : size == 2 ? 0xffffu : 0xffffffffu) << shift;
        mtimecmp_ = (mtimecmp_ & ~mask) | ((std::uint64_t(value) << shift) & mask);
        chip_.core().request_service(now); // the line may fall or rise now
        return;
    }
    if (offset >= MtimeOffset && offset < MtimeOffset + 8) {
        const std::uint32_t shift = 8 * ((offset - MtimeOffset) & 4u);
        const std::uint64_t mask = std::uint64_t(size == 1 ? 0xffu : size == 2 ? 0xffffu : 0xffffffffu) << shift;
        const std::uint64_t current = mtime(now);
        const std::uint64_t mtime_now = (current & ~mask) | ((std::uint64_t(value) << shift) & mask);
        mtime_offset_ += mtime_now - current;
        chip_.core().request_service(now);
    }
}

std::uint64_t Clint::service(std::uint64_t now) {
    const std::uint64_t ticks = mtime(now);
    clic_.set_line(TimerIrq, ticks >= mtimecmp_);
    if (mtimecmp_ <= ticks)
        return NoEvent;
    return now + (mtimecmp_ - ticks) * CyclesPerTick;
}

} // namespace bl702emu
