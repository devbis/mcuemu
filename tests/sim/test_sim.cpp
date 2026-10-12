#include "../test_support.hpp"

#include "sim/ca51f253l3_mcu.hpp"
#include "sim/endpoint.hpp"
#include "sim/net.hpp"
#include "sim/scheduler.hpp"
#include "sim/tlsr8258_mcu.hpp"
#include "sim/world.hpp"

#include <string>
#include <vector>

using tc32emu_tests::words;
using tc32emu::Tlsr8258;

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

} // namespace

namespace tc32emu_tests {
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
void test_ds18b20_search_two_devices();
void test_ntc_divider();
void test_i2c_register_device();
void test_part_errors_and_events();
void test_pad_pulls_and_functions();
void test_adc_reads_net_voltage();
void test_i2c_master_with_sensor();
}

int main() {
    using namespace tc32emu_tests;
    test_config_parsing();
    test_config_errors();
    test_system_build_and_run();
    test_system_build_errors();
    test_uart_link_between_mcus();
    test_ds18b20_one_wire();
    test_ds18b20_lua_matches_native();
    test_lua_sht3x_over_i2c();
    test_lua_api_and_sandbox();
    test_lua_errors();
    test_air_ideal_is_unchanged();
    test_air_path_loss_and_sensitivity();
    test_air_sinr_capture_and_cross_protocol();
    test_air_shadowing_and_frequency_map();
    test_radio_system_config();
    test_tlsr_rssi_over_distance();
    test_lua_radio_parts();
    test_ds18b20_search_two_devices();
    test_ntc_divider();
    test_i2c_register_device();
    test_part_errors_and_events();
    test_pad_pulls_and_functions();
    test_adc_reads_net_voltage();
    test_i2c_master_with_sensor();
    test_scheduler_order_and_cancel();
    test_net_push_pull_resolution();
    test_net_open_drain_is_wired_and();
    test_net_analog_source();
    test_endpoint_registry_and_net_events();
    test_tlsr_pad_drives_net_and_reads_back();
    test_two_mcus_and_a_button_on_one_net();
    test_start_offset_delays_mcu();
    test_world_jumps_when_nothing_runs();
    test_ca51_adapter_basics();
    if (tc32emu_tests::failures) {
        std::cerr << tc32emu_tests::failures << " sim check(s) failed\n";
        return 1;
    }
    std::cout << "all simcore checks passed\n";
    return 0;
}
