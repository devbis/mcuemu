#include "test_support.hpp"

namespace tc32emu_tests {
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
void test_tlsr_mspi_jedec_id();
void test_tlsr_aes128_encrypt_decrypt();
void test_tlsr_divider_registers();
void test_tlsr_uart_fifo_and_virtual_timing();
void test_tlsr_uart_irq_enters_and_returns_from_irq_mode();
void test_virtual_air();
void test_tlsr_radio_dma_packet_exchange();
void test_c_api_register_access();
}

int main() {
    using namespace tc32emu_tests;
    test_cpu_arithmetic_and_branch();
    test_cpu_load_store();
    test_cpu_load_multiple_preserves_loaded_base_register();
    test_cpu_vendor_mmio_alias();
    test_cpu_pc_add_uses_word_aligned_pc();
    test_cpu_suspend_wake_from_32k_timer();
    test_cpu_sleep_stays_asleep_when_rc32k_is_powered_down();
    test_cpu_deep_sleep_wake_resets_to_flash_vector();
    test_cpu_asr_by_32();
    test_cpu_low_register_move_and_three_operand_immediates();
    test_cpu_long_branches_and_call();
    test_cpu_status_modes_and_exception_return();
    test_tlsr_memory_gpio_and_timer();
    test_tlsr_analog_registers_and_32k_tick();
    test_tlsr_rc32k_calibration_ready_after_virtual_time();
    test_tlsr_deep_sleep_sram_retention();
    test_tlsr_system_timer();
    test_tlsr_mspi_jedec_id();
    test_tlsr_aes128_encrypt_decrypt();
    test_tlsr_divider_registers();
    test_tlsr_uart_fifo_and_virtual_timing();
    test_tlsr_uart_irq_enters_and_returns_from_irq_mode();
    test_virtual_air();
    test_tlsr_radio_dma_packet_exchange();
    test_c_api_register_access();
    if (tc32emu_tests::failures) {
        std::cerr << tc32emu_tests::failures << " test check(s) failed\n";
        return 1;
    }
    std::cout << "all tc32-emu checks passed\n";
    return 0;
}
