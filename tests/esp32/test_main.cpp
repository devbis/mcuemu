#include "../test_support.hpp"

#include <iostream>

namespace mcuemu_tests {
void test_esp32c6_boots_an_image_and_stands_in_for_the_rom();
void test_esp32c6_cache_mmu_maps_flash_pages();
void test_esp32c6_systimer_counts_at_16_mhz();
void test_esp32c6_lp_timer_and_rtc_calibration();
void test_esp32c6_systimer_alarm_interrupts_a_sleeping_hart();
void test_esp32c6_pending_interrupt_is_taken_when_mie_returns_after_wfi();
void test_esp32c6_flash_controller_reports_jedec_id();
void test_esp32_mac_delivers_a_unicast_frame_and_gets_the_hardware_ack();
void test_esp32_mac_filters_foreign_frames_and_times_the_ack_out();
void test_esp32_aes_block_and_register_interface();
} // namespace mcuemu_tests

#define RUN(test)                                                                \
    do {                                                                         \
        std::cerr << "[ run ] " #test "\n" << std::flush;                        \
        test();                                                                  \
    } while (0)

int main() {
    using namespace mcuemu_tests;
    RUN(test_esp32c6_boots_an_image_and_stands_in_for_the_rom);
    RUN(test_esp32c6_cache_mmu_maps_flash_pages);
    RUN(test_esp32c6_systimer_counts_at_16_mhz);
    RUN(test_esp32c6_lp_timer_and_rtc_calibration);
    RUN(test_esp32c6_systimer_alarm_interrupts_a_sleeping_hart);
    RUN(test_esp32c6_pending_interrupt_is_taken_when_mie_returns_after_wfi);
    RUN(test_esp32c6_flash_controller_reports_jedec_id);
    RUN(test_esp32_mac_delivers_a_unicast_frame_and_gets_the_hardware_ack);
    RUN(test_esp32_mac_filters_foreign_frames_and_times_the_ack_out);
    RUN(test_esp32_aes_block_and_register_interface);
    if (failures != 0) {
        std::cerr << failures << " check(s) failed\n";
        return 1;
    }
    return 0;
}
