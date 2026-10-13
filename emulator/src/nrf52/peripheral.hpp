#pragma once

#include "nrf52emu/nrf52840.hpp"

#include <cstdint>
#include <limits>

namespace nrf52emu {

constexpr std::uint64_t Never = std::numeric_limits<std::uint64_t>::max();

// One memory-mapped peripheral instance. Offsets are relative to the
// instance's base address.
//
// nRF peripherals share a register layout convention: TASKS_* at 0x000, EVENTS_*
// at 0x100, SHORTS at 0x200, INTENSET/INTENCLR at 0x304/0x308. An event is a
// register that reads 1 once raised and is cleared by writing 0; a task is
// triggered by writing 1. A task can also be triggered through PPI, which
// arrives at task().
class Peripheral {
public:
    Peripheral(Nrf52840 &chip, std::uint32_t base, int irq) : chip_(chip), base_(base), irq_(irq) {}
    virtual ~Peripheral() = default;

    virtual std::uint32_t read(std::uint32_t offset) = 0;
    virtual void write(std::uint32_t offset, std::uint32_t value) = 0;
    // A PPI (or fork) task trigger: `offset` is the TASKS_* register offset.
    virtual void task(std::uint32_t offset) { (void)offset; }
    // Advances time-driven state to `now` (core cycles) and returns the next
    // cycle at which the peripheral needs attention.
    virtual std::uint64_t service(std::uint64_t now) {
        (void)now;
        return Never;
    }
    virtual void reset() {}

    std::uint32_t base() const noexcept { return base_; }
    int irq() const noexcept { return irq_; }

protected:
    std::uint64_t now() const;
    std::uint64_t clock_hz() const;
    // Drives the interrupt line (the peripheral's id) to the NVIC.
    void set_irq_level(bool level);
    // An event register went to 1: routes it through PPI.
    void event_raised(std::uint32_t event_offset);
    // Asks the chip to call service() no later than `at_cycle`.
    void request_service(std::uint64_t at_cycle);
    Nrf52840 &chip() noexcept { return chip_; }

    Nrf52840 &chip_;
    std::uint32_t base_;
    int irq_;
};

// Peripherals that the model does not implement still answer: registers read
// back what was written, so initialisation code that does not wait for hardware
// proceeds. Anything that polls a status bit or waits for an event will hang
// there, which the trace makes visible.
class StubPeripheral final : public Peripheral {
public:
    StubPeripheral(Nrf52840 &chip, std::uint32_t base, int irq) : Peripheral(chip, base, irq) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;

private:
    std::uint32_t registers_[1024]{};
};

} // namespace nrf52emu
