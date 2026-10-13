// cortexm::Core: exceptions through the fork's NVIC, SysTick, WFE sleep, resets.

#include "../test_support.hpp"
#include "cortexm/core.hpp"
#include "programs.hpp"

#include <cstring>
#include <limits>

namespace mcuemu_tests {

namespace {

constexpr std::uint64_t Never = std::numeric_limits<std::uint64_t>::max();

struct Fixture {
    cortexm::Core core;
    std::uint8_t *flash;

    explicit Fixture(const std::vector<std::uint8_t> &program)
        : core(cortexm::Core::Config{}) {
        flash = core.map_memory(0, 0x10000, false);
        core.map_memory(0x20000000, 0x40000, true);
        std::memcpy(flash, program.data(), program.size());
        core.reset();
    }

    std::uint32_t word(std::uint32_t address) {
        std::uint32_t value = 0;
        core.read_memory(address, &value, 4);
        return value;
    }
    void poke(std::uint32_t address, std::uint32_t value) { core.write_memory(address, &value, 4); }
    void pulse(unsigned irq) {
        core.set_irq_line(irq, true);
        core.set_irq_line(irq, false);
    }
};

} // namespace

void test_core_svc_nested_irq_and_fpu_context() {
    Fixture f(nrf52_programs::irq);
    CHECK(f.core.run(2000).reason == cortexm::StopReason::Deadline);
    CHECK(f.word(0x2000010c) == 4); // SVC handler ran
    CHECK(f.word(0x20000110) == 1); // and returned to the code after the svc
    CHECK(f.word(0x20000114) == 0x40600000); // 1.5 + 2.0

    f.pulse(3); // IRQ3 (priority 0x40); its handler pends IRQ4 (priority 0x20)
    CHECK(f.core.run(2400).reason == cortexm::StopReason::Deadline);
    CHECK(f.word(0x20000100) == 1);
    CHECK(f.word(0x20000108) == 3); // the more urgent handler ran
    CHECK(f.word(0x20000104) == 2); // and IRQ3's handler finished
    // The handler clobbered s0 (10.0); the exception return restored it.
    CHECK(f.word(0x20000114) == 0x40600000);
    CHECK(f.core.reg(4) == 0x44);
    CHECK(f.core.reg(13) == 0x20001000); // stack balanced
}

void test_core_wfe_sleeps_until_an_interrupt() {
    Fixture f(nrf52_programs::wfe);
    bool fired = false;
    f.core.set_service_handler([&](std::uint64_t now) -> std::uint64_t {
        if (!fired && now >= 10000) {
            fired = true;
            f.pulse(0);
            return Never;
        }
        return fired ? Never : 10000;
    });
    CHECK(f.core.run(100000).reason == cortexm::StopReason::Deadline);
    CHECK(f.word(0x20000100) == 1);       // the ISR ran once
    CHECK(f.core.reg(4) == 1);            // the loop woke up once
    CHECK(f.core.cycles() >= 100000);
    CHECK(f.core.instructions() < 100);   // the rest was spent asleep
    CHECK(f.core.sleeping());
}

void test_core_systick_interrupt() {
    Fixture f(nrf52_programs::systick);
    CHECK(f.core.run(10500).reason == cortexm::StopReason::Deadline);
    CHECK(f.word(0x20000100) == 10); // every 1000 cycles from about cycle 10
}

void test_core_basepri_masks_lower_priority() {
    Fixture f(nrf52_programs::basepri);
    f.core.run(500);
    f.pulse(3); // priority 0x40 is not more urgent than BASEPRI 0x30
    f.core.run(2000);
    CHECK(f.word(0x20000100) == 0);
    f.poke(0x20000200, 1); // main lifts BASEPRI
    f.core.run(4000);
    CHECK(f.word(0x20000100) == 1); // the pending interrupt is taken now
}

void test_core_level_interrupt_stays_pending() {
    Fixture f(nrf52_programs::wfe);
    f.core.run(500);
    f.core.set_irq_line(0, true); // a level that is never lowered
    f.core.run(2000);
    // A level-sensitive line is taken again as soon as the handler returns.
    CHECK(f.word(0x20000100) > 5);
}

void test_core_reset_and_breakpoint() {
    Fixture reset(nrf52_programs::sysreset);
    const cortexm::RunResult result = reset.core.run(1000);
    CHECK(result.reason == cortexm::StopReason::Reset);

    Fixture bkpt(nrf52_programs::bkpt);
    const cortexm::RunResult stopped = bkpt.core.run(1000);
    CHECK(stopped.reason == cortexm::StopReason::Halted);
    CHECK(bkpt.core.reg(0) == 7); // stopped at the breakpoint, before the next move
}

void test_core_store_to_flash_reaches_the_handler() {
    Fixture f(nrf52_programs::flashstore);
    unsigned address = 0, size = 0;
    std::uint32_t value = 0;
    f.core.set_write_protect_handler([&](std::uint32_t a, unsigned s, std::uint32_t v) {
        address = a;
        size = s;
        value = v;
        std::uint8_t *where = f.core.host_pointer(a, s);
        f.core.defer([where] { where[0] = static_cast<std::uint8_t>(where[0] & 0x0f); });
        return true;
    });
    f.core.run(300);
    CHECK(address == 0x8000 && size == 4 && value == 0x12345678);
    CHECK(f.word(0x8000) == 0x12345608u); // the stored word, with the deferred fix-up applied to byte 0
}

} // namespace mcuemu_tests
