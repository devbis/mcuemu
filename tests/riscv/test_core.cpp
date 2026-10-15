// riscv::Core: memory map, MMIO, the cycle clock, WFI sleep, the interrupt line,
// service callbacks and stop reasons.

#include "../test_support.hpp"
#include "programs.hpp"
#include "riscv/core.hpp"

#include <unicorn/unicorn.h>

#include <cstring>
#include <functional>
#include <limits>

namespace mcuemu_tests {

namespace {

constexpr std::uint32_t Flash = 0x42000000;
constexpr std::uint32_t Periph = 0x60000000;

struct CountingBus final : riscv::Bus {
    unsigned writes{0};
    std::uint32_t last_write{0};
    std::function<void()> on_ack; // the interrupt acknowledge register
    std::uint32_t read(std::uint32_t address, unsigned size) override {
        return address == Periph + 4 && size == 4 ? 0xabcd : 0;
    }
    void write(std::uint32_t address, std::uint32_t value, unsigned size) override {
        if (address == Periph + 8 && on_ack)
            on_ack();
        if (address == Periph && size == 4) {
            ++writes;
            last_write = value;
        }
    }
};

struct Fixture {
    riscv::Core core;
    CountingBus bus;

    explicit Fixture(const std::vector<std::uint8_t> &program) : core(riscv::Core::Config{}) {
        std::uint8_t *flash = core.map_memory(Flash, 0x10000, false);
        std::memcpy(flash, program.data(), program.size());
        core.map_mmio(Periph, 0x1000, bus);
        bus.on_ack = [this] { core.set_interrupt(-1); };
        core.reset(Flash);
    }

    std::uint32_t csr(int id) {
        std::uint32_t value = 0;
        uc_reg_read(core.engine(), id, &value);
        return value;
    }
};

} // namespace

void test_riscv_core_runs_to_a_deadline_and_reaches_mmio() {
    Fixture f(riscv_programs::mmio_loop);
    auto result = f.core.run(1000);
    CHECK(result.reason == riscv::StopReason::Deadline);
    CHECK(f.core.cycles() >= 1000 && f.core.cycles() < 1010);
    CHECK(f.core.instructions() == f.core.cycles()); // one cycle per instruction
    // Four instructions per iteration after the two that set up the loop.
    CHECK(f.bus.writes >= 240 && f.bus.writes <= 251);
    CHECK(f.bus.last_write == f.bus.writes);
    CHECK(f.core.reg(7) == 0xabcd); // t2 holds what the bus returned
}

void test_riscv_core_sleeps_in_wfi_until_the_host_raises_an_interrupt() {
    Fixture f(riscv_programs::trap_irq);
    auto result = f.core.run(1000);
    CHECK(result.reason == riscv::StopReason::Deadline);
    CHECK(f.core.sleeping());
    CHECK(f.core.cycles() == 1000);      // the clock jumped over the sleep
    CHECK(f.core.instructions() < 20);   // and only the few real instructions counted
    CHECK(f.core.reg(8) == 1);           // the ecall trap ran in the guest

    f.core.set_interrupt(20);
    result = f.core.run(2000);
    CHECK(result.reason == riscv::StopReason::Halted); // program ended at ebreak
    CHECK(!f.core.sleeping());
    CHECK(f.core.reg(9) == 0x80000014);
    CHECK(f.core.interrupt() == -1); // the handler's ack dropped the level
    CHECK(f.core.instructions() < 40);
}

void test_riscv_core_calls_the_service_handler_at_its_deadlines() {
    Fixture f(riscv_programs::mmio_loop);
    unsigned calls = 0;
    f.core.set_service_handler([&](std::uint64_t now) {
        ++calls;
        return now + 100;
    });
    CHECK(f.core.run(1000).reason == riscv::StopReason::Deadline);
    CHECK(calls >= 10 && calls <= 12); // at the start and every 100 cycles
}

void test_riscv_core_stops_at_the_instruction_limit_and_on_faults() {
    Fixture f(riscv_programs::mmio_loop);
    f.core.set_instruction_limit(50);
    CHECK(f.core.run(1000000).reason == riscv::StopReason::StepLimit);
    CHECK(f.core.instructions() == 50);

    riscv::Core lost(riscv::Core::Config{});
    lost.map_memory(Flash, 0x1000, false);
    lost.reset(0x10000000); // nothing is mapped there
    auto result = lost.run(100);
    CHECK(result.reason == riscv::StopReason::Fault);
}

void test_riscv_core_has_no_fpu() {
    Fixture f(riscv_programs::mmio_loop);
    const std::uint32_t misa = f.csr(UC_RISCV_REG_MISA);
    auto has = [&](char letter) { return (misa & (1u << (letter - 'A'))) != 0; };
    CHECK(has('I') && has('M') && has('A') && has('C'));
    CHECK(!has('F') && !has('D'));
}

} // namespace mcuemu_tests
