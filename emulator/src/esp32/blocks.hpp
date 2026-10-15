#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace esp32emu {

// A peripheral raises or drops its interrupt source line (the numbers of the
// SVD's `irq::` constants, the sources of the interrupt matrix).
using IrqSink = std::function<void(unsigned source, bool level)>;

inline constexpr std::uint64_t NoEvent = std::numeric_limits<std::uint64_t>::max();

// A peripheral block: a register window the chip routes accesses to. Offsets
// are relative to the block's base address.
class Block {
public:
    Block(std::uint32_t base, std::uint32_t size) : base_(base), size_(size) {}
    virtual ~Block() = default;

    std::uint32_t base() const noexcept { return base_; }
    std::uint32_t size() const noexcept { return size_; }

    // Back to the power-on state (a software reset of the chip). Blocks whose state the
    // firmware can observe across a restart override it.
    virtual void reset() {}

    virtual std::uint32_t read(std::uint32_t offset, unsigned size) = 0;
    virtual void write(std::uint32_t offset, std::uint32_t value, unsigned size) = 0;

    // Brings time-driven state up to `now` (CPU cycles) and returns the cycle of
    // the next event, NoEvent when there is none. Called by the chip's service
    // pass, which runs after every register write and when the hart is due.
    virtual std::uint64_t service(std::uint64_t now) {
        (void)now;
        return NoEvent;
    }

private:
    std::uint32_t base_;
    std::uint32_t size_;
};

// Registers that read back what was written, with some bits forced on read. Used
// for blocks that are only written and polled for "ready" (PLL calibration done,
// clock switched, ...). Every forced bit is evidence from a firmware poll loop;
// the comment at the use site says which.
class StatusBlock : public Block {
public:
    struct Forced {
        std::uint32_t offset;
        std::uint32_t set;   // bits that read 1
        std::uint32_t clear; // bits that read 0
    };
    StatusBlock(std::uint32_t base, std::uint32_t size, std::vector<Forced> forced = {})
        : Block(base, size), forced_(std::move(forced)) {}

    // Power-on contents: what the model preloads (a factory MAC, reset values) survives reset().
    void keep_as_power_on_state() { initial_ = words_; }
    void reset() override { words_ = initial_; }
    std::uint32_t read(std::uint32_t offset, unsigned) override {
        auto it = words_.find(offset & ~3u);
        std::uint32_t value = it == words_.end() ? 0 : it->second;
        for (const Forced &f : forced_)
            if (f.offset == (offset & ~3u))
                value = (value | f.set) & ~f.clear;
        return value;
    }
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override {
        std::uint32_t &word = words_[offset & ~3u];
        if (size == 4) {
            word = value;
        } else {
            const unsigned shift = (offset & 3u) * 8;
            const std::uint32_t mask = (size == 1 ? 0xffu : 0xffffu) << shift;
            word = (word & ~mask) | ((value << shift) & mask);
        }
    }

private:
    std::vector<Forced> forced_;
    std::map<std::uint32_t, std::uint32_t> words_, initial_;
};

} // namespace esp32emu
