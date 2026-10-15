#include "../test_support.hpp"

#include <iostream>

namespace mcuemu_tests {
void test_riscv_ecall_reaches_the_guest_trap_handler();
void test_riscv_host_interrupt_wakes_wfi_with_its_own_cause();
void test_riscv_interrupt_waits_for_mstatus_mie();
void test_riscv_vectored_mtvec_enters_at_base_plus_4n();
void test_riscv_core_runs_to_a_deadline_and_reaches_mmio();
void test_riscv_core_sleeps_in_wfi_until_the_host_raises_an_interrupt();
void test_riscv_core_calls_the_service_handler_at_its_deadlines();
void test_riscv_core_stops_at_the_instruction_limit_and_on_faults();
void test_riscv_core_has_no_fpu();
} // namespace mcuemu_tests

#define RUN(test)                                                                \
    do {                                                                         \
        std::cerr << "[ run ] " #test "\n" << std::flush;                        \
        test();                                                                  \
    } while (0)

int main() {
    using namespace mcuemu_tests;
    RUN(test_riscv_ecall_reaches_the_guest_trap_handler);
    RUN(test_riscv_host_interrupt_wakes_wfi_with_its_own_cause);
    RUN(test_riscv_interrupt_waits_for_mstatus_mie);
    RUN(test_riscv_vectored_mtvec_enters_at_base_plus_4n);
    RUN(test_riscv_core_runs_to_a_deadline_and_reaches_mmio);
    RUN(test_riscv_core_sleeps_in_wfi_until_the_host_raises_an_interrupt);
    RUN(test_riscv_core_calls_the_service_handler_at_its_deadlines);
    RUN(test_riscv_core_stops_at_the_instruction_limit_and_on_faults);
    RUN(test_riscv_core_has_no_fpu);
    if (failures != 0) {
        std::cerr << failures << " check(s) failed\n";
        return 1;
    }
    return 0;
}
