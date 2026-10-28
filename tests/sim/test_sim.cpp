#include "../test_support.hpp"

#include "sim/ca51f253l3_mcu.hpp"

#include <filesystem>
#include <fstream>
#include "sim/endpoint.hpp"
#include "sim/net.hpp"
#include "sim/phy6252_mcu.hpp"
#include "sim/scheduler.hpp"
#include "sim/tlsr8258_mcu.hpp"
#include "sim/world.hpp"

#include <string>
#include <vector>

using mcuemu_tests::words;
using mcuemu::Tlsr8258;

namespace {

void test_scheduler_order_and_cancel() {
    sim::Scheduler sched;
    std::vector<int> order;
    sched.at(200, [&] { order.push_back(3); });
    sched.at(100, [&] { order.push_back(1); });
    sched.at(100, [&] { order.push_back(2); });
    const auto cancelled = sched.at(150, [&] { order.push_back(99); });
    CHECK(sched.cancel(cancelled));
    CHECK(!sched.cancel(cancelled));
    CHECK(sched.next_event_time() == 100);

    sched.advance_to(150);
    CHECK((order == std::vector<int>{1, 2}));
    CHECK(sched.now() == 150);
    // Callback scheduling inside the window runs in the same advance.
    sched.at(160, [&] { sched.at(170, [&] { order.push_back(4); }); });
    sched.advance_to(300);
    CHECK((order == std::vector<int>{1, 2, 4, 3}));
    CHECK(sched.now() == 300);
    // Past times clamp to now().
    sched.at(5, [&] { order.push_back(5); });
    CHECK(sched.next_event_time() == 300);
    sched.advance_to(300);
    CHECK(order.back() == 5);
}

void test_net_push_pull_resolution() {
    sim::Net net("n");
    sim::Pin &a = net.attach("a");
    sim::Pin &b = net.attach("b");
    CHECK(net.level() == sim::Level::Floating);

    std::vector<sim::NetChange> changes;
    net.subscribe([&](const sim::NetChange &c) { changes.push_back(c); });

    a.set_drive(sim::Drive::High, 10);
    CHECK(net.level() == sim::Level::High);
    CHECK(net.millivolts() == 3300);
    CHECK(changes.size() == 1 && changes[0].time_ns == 10);
    a.set_drive(sim::Drive::High, 11); // no change, no event
    CHECK(changes.size() == 1);

    b.set_drive(sim::Drive::Low, 20); // short circuit
    CHECK(net.conflict());
    CHECK(net.conflict_count() == 1);
    CHECK(net.level() == sim::Level::Low);
    a.set_drive(sim::Drive::HiZ, 30);
    CHECK(!net.conflict());
    b.set_drive(sim::Drive::HiZ, 40);
    CHECK(net.level() == sim::Level::Floating);

    // Weak pulls lose against strong drivers and settle a floating line.
    net.set_pull(sim::Drive::PullUp);
    CHECK(net.level() == sim::Level::High);
    b.set_drive(sim::Drive::Low, 50);
    CHECK(net.level() == sim::Level::Low);

    // The board's explicit resistor dominates a conflicting weak pad pull.
    sim::Net gpio("gpio");
    gpio.set_pull(sim::Drive::PullUp);
    sim::Pin &pad = gpio.attach("pad");
    pad.set_drive(sim::Drive::PullDown, 60);
    CHECK(gpio.level() == sim::Level::High);
    pad.set_drive(sim::Drive::Low, 70);
    CHECK(gpio.level() == sim::Level::Low);
}

void test_net_open_drain_is_wired_and() {
    sim::Net bus("ow", sim::NetKind::OpenDrain);
    bus.set_pull(sim::Drive::PullUp);
    sim::Pin &master = bus.attach("master");
    sim::Pin &slave = bus.attach("slave");
    CHECK(bus.level() == sim::Level::High);
    master.set_drive(sim::Drive::High, 1); // released
    slave.set_drive(sim::Drive::High, 1);
    CHECK(bus.level() == sim::Level::High);
    CHECK(!bus.conflict());
    slave.set_drive(sim::Drive::Low, 2);
    CHECK(bus.level() == sim::Level::Low);
    master.set_drive(sim::Drive::Low, 3);
    slave.set_drive(sim::Drive::HiZ, 4);
    CHECK(bus.level() == sim::Level::Low);
    master.set_drive(sim::Drive::HiZ, 5);
    CHECK(bus.level() == sim::Level::High);
}

void test_net_analog_source() {
    sim::Net net("ntc");
    sim::Pin &divider = net.attach("ntc");
    divider.set_analog_mv(900, 0);
    CHECK(net.millivolts() == 900);
    CHECK(net.level() == sim::Level::Low); // below half of 3.3 V
    divider.set_analog_mv(2500, 1);
    CHECK(net.level() == sim::Level::High);
    divider.set_analog_mv(std::nullopt, 2);
    CHECK(net.level() == sim::Level::Floating);
}

void test_endpoint_registry_and_net_events() {
    sim::World world;
    sim::Net &net = world.add_net("led");
    sim::Pin &pin = net.attach("test");
    auto &endpoint = world.endpoints().at("led");
    CHECK(endpoint.kind() == "net");
    CHECK(std::get<bool>(endpoint.get()) == false);

    std::vector<sim::EndpointEvent> events;
    const auto id = endpoint.subscribe(
        [&](const sim::EndpointEvent &e) { events.push_back(e); });
    pin.set_drive(sim::Drive::High, 42);
    CHECK(events.size() == 1);
    CHECK(events[0].time_ns == 42 && std::get<bool>(events[0].value));
    endpoint.unsubscribe(id);
    pin.set_drive(sim::Drive::Low, 43);
    CHECK(events.size() == 1);

    CHECK(world.endpoints().find("missing") == nullptr);
    bool duplicate = false;
    try {
        world.add_net("led");
    } catch (const std::invalid_argument &) {
        duplicate = true;
    }
    CHECK(duplicate);
}

// A TC32 image that spins on `b .` so the MCU just advances time.
void load_idle_loop(sim::Tlsr8258Mcu &mcu) {
    mcu.device().load_firmware(words({0x87fe}));
    mcu.cpu().sync_flash();
    mcu.reset();
}

void test_tlsr_pad_drives_net_and_reads_back() {
    sim::World world;
    auto &mcu = static_cast<sim::Tlsr8258Mcu &>(
        world.add_mcu(std::make_unique<sim::Tlsr8258Mcu>("a")));
    load_idle_loop(mcu);
    sim::Net &net = world.add_net("pa0");
    world.connect(mcu, "PA0", net);

    // Reset: all pads are inputs, so the line is not driven.
    CHECK(net.level() == sim::Level::Floating);
    // Output enabled (OEN bit = 0) with OUT = 1 drives the line high.
    mcu.device().write8(Tlsr8258::GpioBase + 3, 0x01);
    CHECK(net.level() == sim::Level::Floating);
    mcu.device().write8(Tlsr8258::GpioBase + 2, 0xfe);
    CHECK(net.level() == sim::Level::High);
    CHECK((mcu.device().read8(Tlsr8258::GpioBase) & 1u) == 1u);
    mcu.device().write8(Tlsr8258::GpioBase + 3, 0x00);
    CHECK(net.level() == sim::Level::Low);
    CHECK((mcu.device().read8(Tlsr8258::GpioBase) & 1u) == 0u);
    // Back to input: released.
    mcu.device().write8(Tlsr8258::GpioBase + 2, 0xff);
    CHECK(net.level() == sim::Level::Floating);

    bool unknown = false;
    try {
        world.connect(mcu, "PF0", net);
    } catch (const std::invalid_argument &) {
        unknown = true;
    }
    CHECK(unknown);
    bool duplicate = false;
    try {
        world.connect(mcu, "PA0", net);
    } catch (const std::invalid_argument &) {
        duplicate = true;
    }
    CHECK(duplicate);
}

void test_phy6252_pads_follow_nets() {
    sim::World world;
    auto &mcu = static_cast<sim::Phy6252Mcu &>(
        world.add_mcu(std::make_unique<sim::Phy6252Mcu>("phy")));
    CHECK(mcu.pin_names().size() == 23);
    sim::Net &output_net = world.add_net("phy_output");
    world.connect(mcu, "GPIO3", output_net);
    CHECK(output_net.level() == sim::Level::Floating);
    mcu.chip().write(0x40008004, 1u << 3, 4);
    mcu.chip().write(0x40008000, 1u << 3, 4);
    CHECK(output_net.level() == sim::Level::High);
    mcu.chip().write(0x40008000, 0, 4);
    CHECK(output_net.level() == sim::Level::Low);
    mcu.chip().write(0x40008004, 0, 4);
    CHECK(output_net.level() == sim::Level::Floating);

    sim::Pin &switch_pin = output_net.attach("switch");
    switch_pin.set_drive(sim::Drive::High, 1);
    CHECK((mcu.chip().read(0x40008050, 4) & (1u << 3)) != 0);
    switch_pin.set_drive(sim::Drive::Low, 2);
    CHECK((mcu.chip().read(0x40008050, 4) & (1u << 3)) == 0);

    sim::Net &pull_net = world.add_net("phy_pull");
    world.connect(mcu, "GPIO0", pull_net);
    mcu.chip().write(0x4000f008, 2u << 1, 4);
    CHECK(mcu.chip().gpio_drive(0) == phy6252emu::GpioDrive::PullUp);
    CHECK(pull_net.level() == sim::Level::High);
    mcu.chip().write(0x4000f008, 1u << 1, 4); // SDK GPIO_PULL_UP_S
    CHECK(mcu.chip().gpio_drive(0) == phy6252emu::GpioDrive::PullUpStrong);
    CHECK(pull_net.level() == sim::Level::High);

    bool unknown = false;
    try {
        world.connect(mcu, "GPIO23", pull_net);
    } catch (const std::invalid_argument &) {
        unknown = true;
    }
    CHECK(unknown);
    bool duplicate = false;
    try {
        world.connect(mcu, "GPIO0", output_net);
    } catch (const std::invalid_argument &) {
        duplicate = true;
    }
    CHECK(duplicate);
}

void test_two_mcus_and_a_button_on_one_net() {
    sim::World world;
    auto &a = static_cast<sim::Tlsr8258Mcu &>(
        world.add_mcu(std::make_unique<sim::Tlsr8258Mcu>("dev.a")));
    auto &b = static_cast<sim::Tlsr8258Mcu &>(
        world.add_mcu(std::make_unique<sim::Tlsr8258Mcu>("dev.b")));
    load_idle_loop(a);
    load_idle_loop(b);
    sim::Net &line = world.add_net("line", sim::NetKind::OpenDrain);
    line.set_pull(sim::Drive::PullUp);
    world.connect(a, "PA0", line);
    world.connect(b, "PB1", line);
    sim::Pin &button = line.attach("button");

    // Pull-up holds the line high; both MCUs sample it.
    CHECK((a.device().read8(Tlsr8258::GpioBase) & 0x01u) == 0x01u);
    CHECK((b.device().read8(Tlsr8258::GpioBase + 8) & 0x02u) == 0x02u);

    std::vector<std::pair<std::uint64_t, bool>> edges;
    world.endpoints().at("line").subscribe([&](const sim::EndpointEvent &e) {
        edges.emplace_back(e.time_ns, std::get<bool>(e.value));
    });

    // Button pressed at 100 us, released at 300 us.
    world.scheduler().at(100000, [&] {
        button.set_drive(sim::Drive::Low, world.scheduler().now());
    });
    world.scheduler().at(300000, [&] {
        button.set_drive(sim::Drive::HiZ, world.scheduler().now());
    });
    // MCU A pulls the line low at 200 us by writing its GPIO registers from
    // an event, while the button is still held.
    world.scheduler().at(200000, [&] {
        a.device().write8(Tlsr8258::GpioBase + 3, 0x00);
        a.device().write8(Tlsr8258::GpioBase + 2, 0xfe);
    });

    world.run_until(150000);
    CHECK(line.level() == sim::Level::Low);
    CHECK((b.device().read8(Tlsr8258::GpioBase + 8) & 0x02u) == 0);
    CHECK(a.state() == sim::McuState::Running);
    CHECK(a.time_ns() >= 150000 && a.time_ns() < 150000 + 1000);

    world.run_until(350000);
    // Button released but MCU A keeps pulling low (wired-AND).
    CHECK(line.level() == sim::Level::Low);
    a.device().write8(Tlsr8258::GpioBase + 2, 0xff); // A releases
    CHECK(line.level() == sim::Level::High);
    CHECK((b.device().read8(Tlsr8258::GpioBase + 8) & 0x02u) == 0x02u);

    // Press (low), then the line only rises once both the button and MCU A
    // have let go; A's pull-down at 200 us and the button release at 300 us
    // cause no edge of their own.
    CHECK(edges.size() == 2);
    CHECK(edges[0].first == 100000 && !edges[0].second);
    CHECK(edges[1].second && edges[1].first >= 350000);
}

void test_start_offset_delays_mcu() {
    sim::World world;
    auto &late = static_cast<sim::Tlsr8258Mcu &>(
        world.add_mcu(std::make_unique<sim::Tlsr8258Mcu>("late"), 1000000));
    load_idle_loop(late);
    CHECK(late.time_ns() == 1000000);
    world.run_until(500000);
    CHECK(late.time_ns() == 1000000); // not started yet
    world.run_until(1500000);
    CHECK(late.time_ns() >= 1500000 && late.time_ns() < 1501000);
}

void test_world_jumps_when_nothing_runs() {
    sim::World world;
    auto &mcu = static_cast<sim::Tlsr8258Mcu &>(
        world.add_mcu(std::make_unique<sim::Tlsr8258Mcu>("halt")));
    mcu.device().load_firmware(words({0xcf00})); // halt
    mcu.cpu().sync_flash();
    mcu.reset();
    bool fired = false;
    world.scheduler().at(3600ull * 1000000000ull, [&] { fired = true; });
    world.run_until(7200ull * 1000000000ull);
    CHECK(mcu.state() == sim::McuState::Halted);
    CHECK(fired);
    CHECK(world.now() == 7200ull * 1000000000ull);
}

void test_ca51_adapter_basics() {
    sim::World world;
    auto &mcu = world.add_mcu(std::make_unique<sim::Ca51F253L3Mcu>("c"));
    CHECK(mcu.model() == "ca51f253l3");
    CHECK(mcu.time_ns() == 0);
    bool unsupported = false;
    sim::Net net("n");
    try {
        mcu.connect_pin("P0.0", net.attach("c.P0.0"));
    } catch (const std::invalid_argument &) {
        unsupported = true;
    }
    CHECK(unsupported);
    CHECK(mcu.uart_take_tx(1).empty());
}

void test_ca51_uart1_receive() {
    // LJMP main / vector 0x33: LJMP isr / main: IE=EA|ES1, spin /
    // isr: store S1BUF in IDATA 0x30, count in 0x31, clear RI1 (write-one), RETI.
    std::string image(0x210, '\0');
    auto put = [&](std::size_t at, std::initializer_list<int> bytes) {
        for (int byte : bytes)
            image[at++] = static_cast<char>(byte);
    };
    put(0x0000, {0x02, 0x01, 0x00});
    put(0x0033, {0x02, 0x02, 0x00});
    put(0x0100, {0x75, 0xa8, 0xc0, 0x80, 0xfe});
    put(0x0200, {0xe5, 0x9b, 0xf5, 0x30, 0x05, 0x31, 0x75, 0x9a, 0x01, 0x32});
    const auto path = std::filesystem::temp_directory_path() / "simcore-ca51-rx.bin";
    std::ofstream(path, std::ios::binary) << image;

    sim::Ca51F253L3Mcu mcu("c");
    mcu.load_firmware(path.string());
    CHECK(mcu.run_until(20000) == sim::McuState::Running);
    CHECK(!mcu.uart_can_receive(0) && mcu.uart_can_receive(1));
    CHECK(!mcu.uart_rx(0, 1)); // UART0 is transmit-only
    CHECK(mcu.uart_rx(1, 0x42));
    mcu.run_until(60000);
    CHECK(mcu.device().read_idata(0x30) == 0x42 && mcu.device().read_idata(0x31) == 1);
    // Several bytes at once queue up and arrive one by one, in order.
    CHECK(mcu.uart_rx(1, 1) && mcu.uart_rx(1, 2) && mcu.uart_rx(1, 3));
    mcu.run_until(200000);
    CHECK(mcu.device().read_idata(0x30) == 3 && mcu.device().read_idata(0x31) == 4);
    // The queue is bounded.
    int accepted = 0;
    for (int i = 0; i < 100; ++i)
        accepted += mcu.uart_rx(1, 9) ? 1 : 0;
    CHECK(accepted >= 16 && accepted < 100);
}

} // namespace

namespace mcuemu_tests {
void test_cc2530_system_runs_banked_firmware();
void test_cc2530_config_errors_and_pins();
void test_cc2530_fault_and_sleep_states();
void test_cc2530_time_offset_and_quanta();
void test_cc2530_debugger_writes();
void test_cc2530_debugger_config();
void test_cc2530_debugger_report();
void test_cc2530_uart_host_link();
void test_cc2530_system_gpio_adc_parts();
void test_cc2530_system_supply_mv();
void test_cc2530_system_spi_loopback();
void test_cc2530_uart_frame_tokens();
void test_cc2530_uart_frames_system();
void test_cc2530_uart_frames_link();
void test_cc2530_uart_frames_rpc();
void test_config_parsing();
void test_config_errors();
void test_system_build_and_run();
void test_system_build_errors();
void test_uart_link_between_mcus();
void test_ds18b20_one_wire();
void test_ds18b20_lua_matches_native();
void test_lua_sht3x_over_i2c();
void test_lua_api_and_sandbox();
void test_lua_errors();
void test_air_ideal_is_unchanged();
void test_air_path_loss_and_sensitivity();
void test_air_sinr_capture_and_cross_protocol();
void test_air_shadowing_and_frequency_map();
void test_radio_system_config();
void test_tlsr_rssi_over_distance();
void test_lua_radio_parts();
void test_uart_endpoint();
void test_rtt_endpoint();
void test_led_and_button();
void test_sinks_file_stdout_vcd_pcap();
void test_tcp_sink();
void test_attach_config_and_errors();
void test_hex_sink_groups_frames();
void test_text_sink_makes_lines();
void test_websocket_primitives();
void test_rpc_basics();
void test_rpc_run_events_and_memory();
void test_rpc_sandbox();
void test_server_over_websocket();
void test_server_security_options();
void test_rpc_realtime();
void test_config_include();
void test_shipped_examples();
void test_bl0937_example_part();
void test_ds18b20_search_two_devices();
void test_ntc_divider();
void test_i2c_register_device();
void test_part_errors_and_events();
void test_pad_pulls_and_functions();
void test_adc_reads_net_voltage();
void test_i2c_master_with_sensor();
void test_efr32_pads_follow_nets();
void test_efr32_config_rules();
void test_bl702_runs_an_image_and_prints_on_uart();
void test_bl702_transmits_on_the_world_radio();
void test_bl702_config_rules_and_missing_header();
void test_nrf52_pads_follow_nets();
void test_nrf52_twim_talks_to_an_i2c_device();
void test_nrf52_uart_bytes_through_the_mcu_interface();
}

// Names each test on stderr before it runs, so a crash in CI shows where.
#define RUN(test)                                                                \
    do {                                                                         \
        std::cerr << "[ run ] " #test "\n" << std::flush;                        \
        test();                                                                  \
    } while (0)

int main() {
    using namespace mcuemu_tests;
    RUN(test_config_parsing);
    RUN(test_config_errors);
    RUN(test_system_build_and_run);
    RUN(test_system_build_errors);
    RUN(test_uart_link_between_mcus);
    RUN(test_ds18b20_one_wire);
    RUN(test_ds18b20_lua_matches_native);
    RUN(test_lua_sht3x_over_i2c);
    RUN(test_lua_api_and_sandbox);
    RUN(test_lua_errors);
    RUN(test_air_ideal_is_unchanged);
    RUN(test_air_path_loss_and_sensitivity);
    RUN(test_air_sinr_capture_and_cross_protocol);
    RUN(test_air_shadowing_and_frequency_map);
    RUN(test_radio_system_config);
    RUN(test_tlsr_rssi_over_distance);
    RUN(test_lua_radio_parts);
    RUN(test_uart_endpoint);
    RUN(test_rtt_endpoint);
    RUN(test_led_and_button);
    RUN(test_sinks_file_stdout_vcd_pcap);
    RUN(test_hex_sink_groups_frames);
    RUN(test_text_sink_makes_lines);
    RUN(test_tcp_sink);
    RUN(test_attach_config_and_errors);
    RUN(test_websocket_primitives);
    RUN(test_rpc_basics);
    RUN(test_rpc_run_events_and_memory);
    RUN(test_rpc_sandbox);
    RUN(test_server_over_websocket);
    RUN(test_server_security_options);
    RUN(test_rpc_realtime);
    RUN(test_config_include);
    RUN(test_shipped_examples);
    RUN(test_bl0937_example_part);
    RUN(test_ds18b20_search_two_devices);
    RUN(test_ntc_divider);
    RUN(test_i2c_register_device);
    RUN(test_part_errors_and_events);
    RUN(test_pad_pulls_and_functions);
    RUN(test_adc_reads_net_voltage);
    RUN(test_i2c_master_with_sensor);
    RUN(test_efr32_pads_follow_nets);
    RUN(test_bl702_runs_an_image_and_prints_on_uart);
    RUN(test_bl702_transmits_on_the_world_radio);
    RUN(test_bl702_config_rules_and_missing_header);
    RUN(test_efr32_config_rules);
    RUN(test_nrf52_pads_follow_nets);
    RUN(test_nrf52_twim_talks_to_an_i2c_device);
    RUN(test_nrf52_uart_bytes_through_the_mcu_interface);
    RUN(test_scheduler_order_and_cancel);
    RUN(test_net_push_pull_resolution);
    RUN(test_net_open_drain_is_wired_and);
    RUN(test_net_analog_source);
    RUN(test_endpoint_registry_and_net_events);
    RUN(test_tlsr_pad_drives_net_and_reads_back);
    RUN(test_phy6252_pads_follow_nets);
    RUN(test_two_mcus_and_a_button_on_one_net);
    RUN(test_start_offset_delays_mcu);
    RUN(test_world_jumps_when_nothing_runs);
    RUN(test_ca51_adapter_basics);
    RUN(test_ca51_uart1_receive);
    RUN(test_cc2530_system_runs_banked_firmware);
    RUN(test_cc2530_config_errors_and_pins);
    RUN(test_cc2530_fault_and_sleep_states);
    RUN(test_cc2530_time_offset_and_quanta);
    RUN(test_cc2530_debugger_writes);
    RUN(test_cc2530_debugger_config);
    RUN(test_cc2530_debugger_report);
    RUN(test_cc2530_uart_host_link);
    RUN(test_cc2530_system_gpio_adc_parts);
    RUN(test_cc2530_system_supply_mv);
    RUN(test_cc2530_system_spi_loopback);
    RUN(test_cc2530_uart_frame_tokens);
    RUN(test_cc2530_uart_frames_system);
    RUN(test_cc2530_uart_frames_link);
    RUN(test_cc2530_uart_frames_rpc);
    if (mcuemu_tests::failures) {
        std::cerr << mcuemu_tests::failures << " sim check(s) failed\n";
        return 1;
    }
    std::cout << "all simcore checks passed\n";
    return 0;
}
