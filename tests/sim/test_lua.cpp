#include "../test_support.hpp"

#include "sim/config.hpp"
#include "sim/system.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

namespace tc32emu_tests {

namespace {

const std::filesystem::path &scratch_dir() {
    static const std::filesystem::path dir = [] {
        auto path = std::filesystem::temp_directory_path() / "simcore-lua-tests";
        std::filesystem::create_directories(path);
        std::ofstream out(path / "idle.bin", std::ios::binary);
        out.put(static_cast<char>(0xfe));
        out.put(static_cast<char>(0x87)); // b .
        return path;
    }();
    return dir;
}

std::string write_script(const std::string &name, const std::string &text) {
    const auto path = scratch_dir() / name;
    std::ofstream out(path);
    out << text;
    return path.string();
}

// One idle MCU plus a script part on two nets.
std::unique_ptr<sim::System> build(const std::string &script_path,
                                   const std::string &params = "",
                                   std::uint64_t seed = 1) {
    return sim::build_system(sim::parse_system_config(
        "version: 1\nworld: { seed: " + std::to_string(seed) +
        " }\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: tlsr8258, firmware: " +
        (scratch_dir() / "idle.bin").string() + "}\n"
        "    nets:\n      - { id: a, kind: open_drain, pull: up }\n"
        "      - { id: b }\n"
        "    parts:\n      - id: s\n        type: script\n        script: " +
        script_path + "\n        pins: { a: a, b: b }\n        params: { " + params +
        " }\n"));
}

template <typename Fn> std::string error_of(Fn &&fn) {
    try {
        fn();
    } catch (const sim::ConfigError &error) {
        return error.what();
    }
    return {};
}

bool contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

double number(sim::World &world, const std::string &endpoint) {
    return sim::value_as_number(world.endpoints().at(endpoint).get());
}

} // namespace

void test_lua_api_and_sandbox() {
    // Params keep their types, the sandbox hides io/os, timers run in order
    // and can be cancelled, and pins can be driven and sampled.
    const std::string script = write_script("api.lua", R"(
assert(io == nil and os == nil and dofile == nil and loadfile == nil
       and package == nil and debug == nil and print == nil, "sandbox")
assert(math.random == nil and math.randomseed == nil, "no unseeded random")
assert(math.type(params.count) == "integer" and params.count == 42)
assert(math.type(params.ratio) == "float" and params.ratio == 0.5)
assert(params.flag == true and params.name == "abc" and params.hex == 255)
assert(US == 1000 and MS == 1000000 and S == 1000000000)

local fired = {}
at(5 * US, function(t) fired[#fired + 1] = "5:" .. t end)
local victim = at(7 * US, function() fired[#fired + 1] = "cancelled" end)
after(9 * US, function(t) fired[#fired + 1] = "9:" .. t end)
assert(cancel(victim) == true and cancel(victim) == false)

local edges = 0
function on_edge(name, high, t)
  edges = edges + 1
  last_edge = name .. (high and "^" or "v") .. t
end

expose("fired", function() return #fired end)
expose("edges", function() return edges end)
expose("level", function() return pins.a:level() end)
expose("mv", function() return pins.b:mv() end)
expose("drive", function() return 0 end,
       function(v) pins.b:drive(v and "high" or "low") end)
expose("analog", function() return 0 end, function(v) pins.b:set_mv(v) end)
expose("first", function() return fired[1] or "" end)
expose("second", function() return fired[2] or "" end)
)");
    const auto system = build(script,
        "count: 42, ratio: 0.5, flag: true, name: abc, hex: 0xff");
    sim::World &world = system->world();

    CHECK(number(world, "d.s.fired") == 0);
    world.run_until(20000);
    CHECK(number(world, "d.s.fired") == 2);
    CHECK(std::get<std::string>(world.endpoints().at("d.s.first").get()) == "5:5000");
    CHECK(std::get<std::string>(world.endpoints().at("d.s.second").get()) == "9:9000");

    // on_edge sees changes made by others, with their timestamps; the script's
    // own drive does not call back.
    sim::Pin &outside = world.net("d.a").attach("test");
    CHECK(number(world, "d.s.edges") == 0);
    outside.set_drive(sim::Drive::Low, 25000);
    outside.set_drive(sim::Drive::HiZ, 26000);
    CHECK(number(world, "d.s.edges") == 2);
    CHECK(std::get<bool>(world.endpoints().at("d.s.level").get()) == true);
    world.endpoints().at("d.s.drive").set(sim::Value{true});
    CHECK(world.net("d.b").level() == sim::Level::High);
    CHECK(number(world, "d.s.edges") == 2); // own edge, not delivered
    world.endpoints().at("d.s.analog").set(sim::Value{std::int64_t{900}});
    CHECK(number(world, "d.s.mv") == 900 || number(world, "d.s.mv") == 3300);

    // Scenario events can set exposed values.
    CHECK(contains(error_of([&] {
                       sim::build_system(sim::parse_system_config(
                           "version: 1\ndevices:\n  - id: d\n    mcus:\n"
                           "      - {id: m, model: tlsr8258, firmware: " +
                           (scratch_dir() / "idle.bin").string() + "}\n"
                           "    parts:\n      - {id: s, type: script}\n"));
                   }),
                   "requires 'script'"));
}

void test_lua_errors() {
    // A syntax error is reported with file and line at build time.
    const std::string broken = write_script("broken.lua", "local x = \nfunction(\n");
    const std::string message = error_of([&] { build(broken); });
    CHECK(contains(message, "broken.lua:"));
    CHECK(contains(message, "part 'd.s'"));

    // A runtime error while loading fails the build as well.
    const std::string load_error =
        write_script("load_error.lua", "error('boom at load')\n");
    CHECK(contains(error_of([&] { build(load_error); }), "boom at load"));

    // Sandbox violation at load.
    CHECK(!error_of([&] {
        build(write_script("io.lua", "io.open('/etc/passwd')\n"));
    }).empty());

    // Wrong file type and unknown script file.
    CHECK(contains(error_of([&] {
                       build(write_script("part.py", "x = 1\n"));
                   }),
                   "must be a .lua file"));
    CHECK(contains(error_of([&] { build((scratch_dir() / "missing.lua").string()); }),
                   "cannot open"));

    // An error inside a callback stops that part only; the simulation goes on.
    const std::string flaky = write_script("flaky.lua", R"(
local calls = 0
function on_edge(name, high, t)
  calls = calls + 1
  error("callback failed")
end
expose("calls", function() return calls end)
)");
    const auto system = build(flaky);
    sim::World &world = system->world();
    sim::Pin &outside = world.net("d.a").attach("test");
    outside.set_drive(sim::Drive::Low, 1000);
    outside.set_drive(sim::Drive::HiZ, 2000);
    outside.set_drive(sim::Drive::Low, 3000);
    CHECK(number(world, "d.s.calls") == 1); // stopped after the first failure
    world.run_until(1000000);
    CHECK(world.now() == 1000000);

    // A runaway loop hits the instruction limit instead of hanging.
    const std::string spin = write_script("spin.lua", R"(
at(1 * US, function() while true do end end)
expose("alive", function() return 1 end)
)");
    const auto spinning = build(spin);
    spinning->world().run_until(1000000);
    CHECK(spinning->world().now() == 1000000);

    // Memory is bounded.
    const std::string hog = write_script("hog.lua", R"(
at(1 * US, function() local t = {} for i = 1, 1e9 do t[i] = ("x"):rep(1000) .. i end end)
)");
    const auto hogging = build(hog);
    hogging->world().run_until(1000000);
    CHECK(hogging->world().now() == 1000000);
}

} // namespace tc32emu_tests

namespace tc32emu_tests {

// Counts falling edges of a net over a window, from the net's change log.
void test_bl0937_example_part() {
    const std::string script =
        std::string(TC32EMU_SOURCE_DIR) + "/examples/parts/bl0937.lua";
    auto system = sim::build_system(sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: tlsr8258, firmware: " + (scratch_dir() / "idle.bin").string() + "}\n"
        "    nets:\n      - { id: cf }\n      - { id: cf1 }\n      - { id: sel }\n"
        "    parts:\n      - id: meter\n        type: script\n        script: " + script + "\n"
        "        pins: { cf: cf, cf1: cf1, sel: sel }\n"
        "        params: { power_w: 460, voltage_v: 230, cf_hz_per_w: 0.5, cf1_hz_per_a: 100, cf1_hz_per_v: 8 }\n"
        "  - id: sel_driver\n    mcus:\n"
        "      - {id: m, model: tlsr8258, firmware: " + (scratch_dir() / "idle.bin").string() + "}\n"));
    sim::World &world = system->world();
    sim::Net &cf = world.net("d.cf");
    sim::Net &cf1 = world.net("d.cf1");
    sim::Net &sel = world.net("d.sel");
    auto falls = [](sim::Net &net, int &count) {
        net.subscribe([&count](const sim::NetChange &change) {
            if (change.level == sim::Level::Low)
                ++count;
        });
    };
    int cf_falls = 0, cf1_falls = 0;
    falls(cf, cf_falls);
    falls(cf1, cf1_falls);

    // 460 W at 0.5 Hz/W = 230 Hz on CF; 2 A at 100 Hz/A = 200 Hz on CF1 (SEL low).
    CHECK(std::abs(number(world, "d.meter.cf_hz") - 230.0) < 1e-6);
    CHECK(std::abs(number(world, "d.meter.cf1_hz") - 200.0) < 1e-6);
    world.run_until(1000000000ull);
    CHECK(cf_falls >= 229 && cf_falls <= 231);
    CHECK(cf1_falls >= 199 && cf1_falls <= 201);

    // SEL high switches CF1 to the voltage: 230 V * 8 Hz/V = 1840 Hz.
    sim::Pin &driver = sel.attach("test");
    driver.set_drive(sim::Drive::High, world.now());
    CHECK(std::abs(number(world, "d.meter.cf1_hz") - 1840.0) < 1e-6);
    cf1_falls = 0;
    world.run_until(world.now() + 100000000ull);
    CHECK(cf1_falls >= 183 && cf1_falls <= 185);

    // A new load takes effect at once; zero power stops the pulses.
    world.endpoints().at("d.meter.power_w").set(sim::Value{920.0});
    CHECK(std::abs(number(world, "d.meter.cf_hz") - 460.0) < 1e-6);
    world.endpoints().at("d.meter.power_w").set(sim::Value{0.0});
    cf_falls = 0;
    world.run_until(world.now() + 100000000ull);
    CHECK(cf_falls == 0);
    CHECK(cf.level() == sim::Level::High);
}

} // namespace tc32emu_tests
