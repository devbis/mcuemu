#include "timers.hpp"

#include "esp32emu/esp32c6_regs.hpp"

#include <algorithm>

namespace esp32emu {

namespace {
namespace st = esp32c6_regs::systimer;
namespace lp = esp32c6_regs::lp_timer;

constexpr std::uint64_t Mask52 = (1ull << 52) - 1;
constexpr std::uint64_t Mask48 = (1ull << 48) - 1;

// cycles * hz / cpu_hz without overflowing 64 bits for hours of virtual time.
std::uint64_t scale(std::uint64_t cycles, std::uint64_t hz, std::uint64_t cpu_hz) {
    return cycles / cpu_hz * hz + cycles % cpu_hz * hz / cpu_hz;
}
} // namespace

Systimer::Systimer(std::uint32_t base, std::uint32_t cpu_hz, unsigned first_source, CycleSource cycles, IrqSink irq)
    : StatusBlock(base, 0x100), irq_(std::move(irq)), first_source_(first_source), cpu_hz_(cpu_hz), cycles_(std::move(cycles)) {
    StatusBlock::write(st::reg::CONF, st::reset::CONF, 4);
}

void Systimer::reset() {
    // A chip reset restarts the counters at zero (the model's clock keeps running across it).
    StatusBlock::reset();
    StatusBlock::write(st::reg::CONF, st::reset::CONF, 4);
    for (unsigned unit = 0; unit < 2; ++unit) {
        origin_[unit] = ticks();
        load_value_[unit] = 0;
        latched_[unit] = 0;
    }
    for (Target &target : targets_)
        target = Target{};
    int_raw_ = int_ena_ = 0;
    update_lines();
}

std::uint64_t Systimer::ticks() const { return scale(cycles_(), TickHz, cpu_hz_); }

std::uint64_t Systimer::counter(unsigned unit) const {
    return (load_value_[unit] + ticks() - origin_[unit]) & Mask52;
}

// First CPU cycle at which `tick` systimer ticks have elapsed.
std::uint64_t Systimer::cycle_of_tick(std::uint64_t tick) const {
    return (tick * cpu_hz_ + TickHz - 1) / TickHz;
}

void Systimer::latch(unsigned unit) { latched_[unit] = counter(unit); }

void Systimer::update_lines() {
    for (unsigned n = 0; n < 3; ++n)
        irq_(first_source_ + n, (int_raw_ & int_ena_) >> n & 1u);
}

std::uint64_t Systimer::service(std::uint64_t now) {
    const std::uint32_t conf = StatusBlock::read(st::reg::CONF, 4);
    std::uint64_t next = NoEvent;
    bool changed = false;
    for (unsigned n = 0; n < 3; ++n) {
        Target &target = targets_[n];
        const unsigned work_en_shift = st::field::CONF_TARGET0_WORK_EN_shift - n; // T0:24 T1:23 T2:22
        if (!target.armed || !(conf >> work_en_shift & 1u))
            continue;
        const std::uint32_t tconf = StatusBlock::read(st::reg::TARGET0_CONF + 4 * n, 4);
        const unsigned unit = tconf >> 31 & 1u;
        const std::uint64_t value = counter(unit);
        if (value >= target.value) {
            int_raw_ |= 1u << n;
            changed = true;
            if (tconf >> 30 & 1u) { // period mode
                target.value = (target.value + (tconf & 0x3ffffff)) & Mask52;
                if (!(tconf & 0x3ffffff))
                    target.armed = false;
            } else {
                target.armed = false;
            }
        }
        if (target.armed) {
            // The tick at which the unit's count reaches the target.
            const std::uint64_t ahead = target.value - counter(unit);
            next = std::min(next, std::max(now + 1, cycle_of_tick(ticks() + ahead)));
        }
    }
    if (changed)
        update_lines();
    return next;
}

std::uint32_t Systimer::read(std::uint32_t offset, unsigned size) {
    switch (offset & ~3u) {
    case st::reg::UNIT0_VALUE_HI: return static_cast<std::uint32_t>(latched_[0] >> 32);
    case st::reg::UNIT0_VALUE_LO: return static_cast<std::uint32_t>(latched_[0]);
    case st::reg::UNIT1_VALUE_HI: return static_cast<std::uint32_t>(latched_[1] >> 32);
    case st::reg::UNIT1_VALUE_LO: return static_cast<std::uint32_t>(latched_[1]);
    case st::reg::INT_RAW: return int_raw_;
    case st::reg::INT_ENA: return int_ena_;
    case st::reg::INT_ST: return int_raw_ & int_ena_;
    case st::reg::UNIT0_OP:
    case st::reg::UNIT1_OP:
        // The latch completes at once: VALUE_VALID is always set.
        return StatusBlock::read(offset, size) | (1u << st::field::UNIT0_OP_TIMER_UNIT0_VALUE_VALID_shift);
    default: return StatusBlock::read(offset, size);
    }
}

void Systimer::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    const std::uint32_t reg = offset & ~3u;
    if (reg == st::reg::UNIT0_OP || reg == st::reg::UNIT1_OP) {
        const unsigned unit = reg == st::reg::UNIT0_OP ? 0 : 1;
        if (value >> st::field::UNIT0_OP_TIMER_UNIT0_UPDATE_shift & 1u)
            latch(unit);
        return;
    }
    if (reg == st::reg::UNIT0_LOAD_HI || reg == st::reg::UNIT0_LOAD_LO ||
        reg == st::reg::UNIT1_LOAD_HI || reg == st::reg::UNIT1_LOAD_LO) {
        StatusBlock::write(offset, value, size);
        return;
    }
    if (reg == st::reg::UNIT0_LOAD || reg == st::reg::UNIT1_LOAD) {
        const unsigned unit = reg == st::reg::UNIT0_LOAD ? 0 : 1;
        const std::uint32_t hi = StatusBlock::read(unit ? st::reg::UNIT1_LOAD_HI : st::reg::UNIT0_LOAD_HI, 4);
        const std::uint32_t lo = StatusBlock::read(unit ? st::reg::UNIT1_LOAD_LO : st::reg::UNIT0_LOAD_LO, 4);
        load_value_[unit] = (static_cast<std::uint64_t>(hi) << 32 | lo) & Mask52;
        origin_[unit] = ticks();
        return;
    }
    if (reg >= st::reg::COMP0_LOAD && reg <= st::reg::COMP2_LOAD) {
        const unsigned n = (reg - st::reg::COMP0_LOAD) / 4;
        const std::uint32_t hi = StatusBlock::read(st::reg::TARGET0_HI + 8 * n, 4);
        const std::uint32_t lo = StatusBlock::read(st::reg::TARGET0_LO + 8 * n, 4);
        targets_[n].value = (static_cast<std::uint64_t>(hi & 0xfffff) << 32 | lo) & Mask52;
        targets_[n].armed = true;
        return;
    }
    switch (reg) {
    case st::reg::INT_ENA:
        int_ena_ = value & 7u;
        update_lines();
        return;
    case st::reg::INT_RAW: // write-one-to-set (software trigger)
        int_raw_ |= value & 7u;
        update_lines();
        return;
    case st::reg::INT_CLR:
        int_raw_ &= ~value;
        update_lines();
        return;
    default:
        break;
    }
    StatusBlock::write(offset, value, size);
}

LpTimer::LpTimer(std::uint32_t base, std::uint32_t cpu_hz, CycleSource cycles)
    : StatusBlock(base, 0x400), cpu_hz_(cpu_hz), cycles_(std::move(cycles)) {}

void LpTimer::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    if ((offset & ~3u) == lp::reg::UPDATE) {
        if (value >> lp::field::UPDATE_MAIN_TIMER_UPDATE_shift & 1u) {
            const std::uint64_t count = scale(cycles_(), SlowHz, cpu_hz_) & Mask48;
            StatusBlock::write(lp::reg::MAIN_BUF0_LOW, static_cast<std::uint32_t>(count), 4);
            StatusBlock::write(lp::reg::MAIN_BUF0_HIGH, static_cast<std::uint32_t>(count >> 32), 4);
        }
        // UPDATE is a strobe; only the stall/reset configuration bits stay.
        value &= ~(1u << lp::field::UPDATE_MAIN_TIMER_UPDATE_shift);
    }
    StatusBlock::write(offset, value, size);
}

} // namespace esp32emu
