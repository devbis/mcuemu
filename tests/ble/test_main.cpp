#include "../test_support.hpp"

#include <iostream>

namespace mcuemu_tests {
void test_ble_crc24();
void test_ble_whitening();
void test_ble_access_address_rules();
void test_ble_adv_pdus_and_connect_ind();
void test_ble_data_pdus_and_sequence_numbers();
void test_ble_channel_selection_1();
void test_ble_timing_and_air_packet();
void test_ble_control_pdus();
} // namespace mcuemu_tests

#define RUN(test)                                                                \
    do {                                                                         \
        std::cerr << "[ run ] " #test "\n" << std::flush;                        \
        test();                                                                  \
    } while (0)

int main() {
    using namespace mcuemu_tests;
    RUN(test_ble_crc24);
    RUN(test_ble_whitening);
    RUN(test_ble_access_address_rules);
    RUN(test_ble_adv_pdus_and_connect_ind);
    RUN(test_ble_data_pdus_and_sequence_numbers);
    RUN(test_ble_channel_selection_1);
    RUN(test_ble_timing_and_air_packet);
    RUN(test_ble_control_pdus);
    if (failures) {
        std::cerr << failures << " check(s) failed\n";
        return 1;
    }
    std::cerr << "all ble checks passed\n";
    return 0;
}
