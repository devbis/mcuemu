#include "test_support.hpp"

namespace mcuemu_tests {
void test_cpu_arithmetic_and_branch();
void test_cpu_load_store();
void test_cpu_load_multiple_preserves_loaded_base_register();
void test_cpu_vendor_mmio_alias();
void test_cpu_pc_add_uses_word_aligned_pc();
void test_cpu_suspend_wake_from_32k_timer();
void test_cpu_sleep_stays_asleep_when_rc32k_is_powered_down();
void test_cpu_deep_sleep_wake_resets_to_flash_vector();
void test_cpu_asr_by_32();
void test_cpu_low_register_move_and_three_operand_immediates();
void test_cpu_long_branches_and_call();
void test_cpu_status_modes_and_exception_return();
void test_tlsr_memory_gpio_and_timer();
void test_tlsr_analog_registers_and_32k_tick();
void test_tlsr_rc32k_calibration_ready_after_virtual_time();
void test_tlsr_deep_sleep_sram_retention();
void test_tlsr_system_timer();
void test_tlsr_system_timer_start_stop_and_clock_select();
void test_tlsr_deep_sleep_wake_gate();
void test_tlsr_mspi_jedec_id();
void test_tlsr_mspi_flash_read_program_erase();
void test_tlsr_aes128_encrypt_decrypt();
void test_tlsr_divider_registers();
void test_tlsr_uart_fifo_and_virtual_timing();
void test_tlsr_uart_irq_enters_and_returns_from_irq_mode();
void test_tlsr_batched_run_matches_single_steps();
void test_tlsr_flash_timing_charges_cache_misses();
void test_virtual_air();
void test_tlsr_radio_dma_packet_exchange();
void test_c_api_register_access();
}

// Names each test on stderr before it runs, so a crash in CI shows where.
#define RUN(test)                                                                \
    do {                                                                         \
        std::cerr << "[ run ] " #test "\n" << std::flush;                        \
        test();                                                                  \
    } while (0)

int main() {
    using namespace mcuemu_tests;
    RUN(test_cpu_arithmetic_and_branch);
    RUN(test_cpu_load_store);
    RUN(test_cpu_load_multiple_preserves_loaded_base_register);
    RUN(test_cpu_vendor_mmio_alias);
    RUN(test_cpu_pc_add_uses_word_aligned_pc);
    RUN(test_cpu_suspend_wake_from_32k_timer);
    RUN(test_cpu_sleep_stays_asleep_when_rc32k_is_powered_down);
    RUN(test_cpu_deep_sleep_wake_resets_to_flash_vector);
    RUN(test_cpu_asr_by_32);
    RUN(test_cpu_low_register_move_and_three_operand_immediates);
    RUN(test_cpu_long_branches_and_call);
    RUN(test_cpu_status_modes_and_exception_return);
    RUN(test_tlsr_memory_gpio_and_timer);
    RUN(test_tlsr_analog_registers_and_32k_tick);
    RUN(test_tlsr_rc32k_calibration_ready_after_virtual_time);
    RUN(test_tlsr_deep_sleep_sram_retention);
    RUN(test_tlsr_system_timer);
    RUN(test_tlsr_system_timer_start_stop_and_clock_select);
    RUN(test_tlsr_deep_sleep_wake_gate);
    RUN(test_tlsr_mspi_jedec_id);
    RUN(test_tlsr_mspi_flash_read_program_erase);
    RUN(test_tlsr_aes128_encrypt_decrypt);
    RUN(test_tlsr_divider_registers);
    RUN(test_tlsr_uart_fifo_and_virtual_timing);
    RUN(test_tlsr_uart_irq_enters_and_returns_from_irq_mode);
    RUN(test_tlsr_batched_run_matches_single_steps);
    RUN(test_tlsr_flash_timing_charges_cache_misses);
    RUN(test_virtual_air);
    RUN(test_tlsr_radio_dma_packet_exchange);
    RUN(test_c_api_register_access);
    if (mcuemu_tests::failures) {
        std::cerr << mcuemu_tests::failures << " test check(s) failed\n";
        return 1;
    }
    std::cout << "all mcuemu checks passed\n";
    return 0;
}
