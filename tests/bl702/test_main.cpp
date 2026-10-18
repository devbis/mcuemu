#include "../test_support.hpp"

#include <iostream>

namespace mcuemu_tests {
void test_bl702_boot_header_loads_segments_and_rejects_no_header();
void test_bl702_uart_and_timer();
void test_bl702_efuse_mac_and_aes_block();
void test_bl702_trng_fills_the_output_on_trigger();
void test_bl702_radio_transmits_and_receives_on_the_air();
void test_bl702_cca_sees_a_frame_on_the_air();
void test_bl702_rom_flash_routines();
void test_bl702_clic_takes_the_timer_interrupt();
void test_bl702_radio_timer_channel_fires_at_its_target();
} // namespace mcuemu_tests

#define RUN(test)                                                                \
    do {                                                                         \
        std::cerr << "[ run ] " #test "\n" << std::flush;                        \
        test();                                                                  \
    } while (0)

int main() {
    using namespace mcuemu_tests;
    RUN(test_bl702_boot_header_loads_segments_and_rejects_no_header);
    RUN(test_bl702_uart_and_timer);
    RUN(test_bl702_efuse_mac_and_aes_block);
    RUN(test_bl702_trng_fills_the_output_on_trigger);
    RUN(test_bl702_radio_transmits_and_receives_on_the_air);
    RUN(test_bl702_cca_sees_a_frame_on_the_air);
    RUN(test_bl702_rom_flash_routines);
    RUN(test_bl702_clic_takes_the_timer_interrupt);
    RUN(test_bl702_radio_timer_channel_fires_at_its_target);

    if (failures) {
        std::cerr << failures << " check(s) failed\n";
        return 1;
    }
    std::cerr << "all bl702emu tests passed\n";
    return 0;
}
