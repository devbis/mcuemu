#include "../test_support.hpp"

#include <iostream>

namespace mcuemu_tests {
void test_core_svc_nested_irq_and_fpu_context();
void test_core_wfe_sleeps_until_an_interrupt();
void test_core_systick_interrupt();
void test_core_exception_from_process_stack();
void test_core_basepri_masks_lower_priority();
void test_core_level_interrupt_stays_pending();
void test_core_reset_and_breakpoint();
void test_core_store_to_flash_reaches_the_handler();
void test_nrf52_flash_ids_and_memory();
void test_nrf52_nvmc_programming_rules();
void test_nrf52_rtc_compare_timing_and_counter();
void test_nrf52_rtc_interrupt_and_overflow();
void test_nrf52_timer_compare_clear_and_ppi_count();
void test_nrf52_gpio_pads_and_sense();
void test_nrf52_ppi_fork_disables_a_channel_group();
void test_nrf52_gpiote_edge_event_and_output_task();
void test_nrf52_ecb_encrypts_a_block();
void test_nrf52_rng_is_deterministic_per_device();
void test_nrf52_watchdog_resets_unless_fed();
void test_nrf52_uarte_transmit_and_receive();
void test_nrf52_saadc_reads_vdd();
void test_nrf52_clock_start_events();
void test_nrf52_lfclk_restart_switches_source();
void test_nrf52_radio_frame_exchange();
void test_nrf52_radio_cca_sees_a_frame_on_air();
void test_nrf52_radio_ramp_and_state();
} // namespace mcuemu_tests

// Names each test on stderr before it runs, so a crash in CI shows where.
#define RUN(test)                                                                \
    do {                                                                         \
        std::cerr << "[ run ] " #test "\n" << std::flush;                        \
        test();                                                                  \
    } while (0)

int main() {
    using namespace mcuemu_tests;
    RUN(test_core_svc_nested_irq_and_fpu_context);
    RUN(test_core_wfe_sleeps_until_an_interrupt);
    RUN(test_core_systick_interrupt);
    RUN(test_core_exception_from_process_stack);
    RUN(test_core_basepri_masks_lower_priority);
    RUN(test_core_level_interrupt_stays_pending);
    RUN(test_core_reset_and_breakpoint);
    RUN(test_core_store_to_flash_reaches_the_handler);
    RUN(test_nrf52_flash_ids_and_memory);
    RUN(test_nrf52_nvmc_programming_rules);
    RUN(test_nrf52_rtc_compare_timing_and_counter);
    RUN(test_nrf52_rtc_interrupt_and_overflow);
    RUN(test_nrf52_timer_compare_clear_and_ppi_count);
    RUN(test_nrf52_gpio_pads_and_sense);
    RUN(test_nrf52_ppi_fork_disables_a_channel_group);
    RUN(test_nrf52_gpiote_edge_event_and_output_task);
    RUN(test_nrf52_ecb_encrypts_a_block);
    RUN(test_nrf52_rng_is_deterministic_per_device);
    RUN(test_nrf52_watchdog_resets_unless_fed);
    RUN(test_nrf52_uarte_transmit_and_receive);
    RUN(test_nrf52_saadc_reads_vdd);
    RUN(test_nrf52_clock_start_events);
    RUN(test_nrf52_lfclk_restart_switches_source);
    RUN(test_nrf52_radio_frame_exchange);
    RUN(test_nrf52_radio_cca_sees_a_frame_on_air);
    RUN(test_nrf52_radio_ramp_and_state);
    if (failures != 0) {
        std::cerr << failures << " check(s) failed\n";
        return 1;
    }
    return 0;
}
