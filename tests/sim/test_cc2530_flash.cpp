// System-level CC2530 flash fault injection: the `flash:` YAML block, the
// operation log, the <mcu>.power endpoint and the flash persistence file.
// The chip-level rules are covered by tests/mcs51/test_cc2530_flash_powercut.cpp.
#include "../test_support.hpp"

#include "sim/cc2530f256_mcu.hpp"
#include "sim/config.hpp"
#include "sim/endpoint.hpp"
#include "sim/system.hpp"
#include "sim/world.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using sim::Cc2530F256Mcu;

// Under the working directory (the build tree when run by ctest).
const std::filesystem::path &scratch() {
    static const std::filesystem::path path = [] {
        auto dir = std::filesystem::current_path() / "simcore-cc2530-flash-tests";
        std::filesystem::create_directories(dir);
        return dir;
    }();
    return path;
}

std::string path_of(const std::string &name) { return (scratch() / name).generic_string(); }

std::vector<std::uint8_t> read_file(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::string read_text(const std::string &path) {
    std::ifstream input(path);
    std::ostringstream text;
    text << input.rdbuf();
    return text.str();
}

bool contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

unsigned count_of(const std::string &text, const std::string &part) {
    unsigned count = 0;
    for (std::size_t at = text.find(part); at != std::string::npos; at = text.find(part, at + 1))
        ++count;
    return count;
}

// Boot: copy the first page 6 byte (MOVC 0x3000) to IDATA 0x40; erase page 6
// from flash (the CPU stalls 20 ms) only if that byte reads 0x00; spin. A
// raw binary loaded at address 0, page 6 programmed to 0x00.
std::string erase_on_boot_image() {
    std::vector<std::uint8_t> image(0x3000 + 2048, 0xff);
    const std::vector<std::uint8_t> code{
        0x02, 0x00, 0x40}; // LJMP 0x0040
    const std::vector<std::uint8_t> main{
        0x75, 0x81, 0x60, // 0x40 MOV SP,#0x60
        0x90, 0x30, 0x00, // 0x43 MOV DPTR,#0x3000
        0xe4,             // 0x46 CLR A
        0x93,             // 0x47 MOVC A,@A+DPTR
        0xf5, 0x40,       // 0x48 MOV 0x40,A
        0x70, 0x0c,       // 0x4a JNZ 0x58
        0x90, 0x62, 0x72, // 0x4c MOV DPTR,#FADDRH
        0x74, 0x0c,       // 0x4f MOV A,#0x0c (page 6)
        0xf0,             // 0x51 MOVX @DPTR,A
        0x90, 0x62, 0x70, // 0x52 MOV DPTR,#FCTL
        0x74, 0x05,       // 0x55 MOV A,#0x05 (CM=01, ERASE)
        0xf0,             // 0x57 MOVX @DPTR,A
        0x80, 0xfe};      // 0x58 SJMP 0x58
    std::copy(code.begin(), code.end(), image.begin());
    std::copy(main.begin(), main.end(), image.begin() + 0x40);
    std::fill(image.begin() + 0x3000, image.end(), 0x00);
    const std::string path = path_of("erase_on_boot.bin");
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char *>(image.data()),
              static_cast<std::streamsize>(image.size()));
    return path;
}

std::string system_yaml(const std::string &mcu_extra) {
    return "version: 1\ndevices:\n  - id: d\n    mcus:\n"
           "      - id: m\n        model: cc2530f256\n        firmware: \"" +
           erase_on_boot_image() + "\"\n" + mcu_extra;
}

std::string error_of_build(const std::string &yaml) {
    try {
        sim::build_system(sim::parse_system_config(yaml));
    } catch (const std::exception &error) {
        return error.what();
    }
    return {};
}

std::uint8_t idata(sim::Mcu &mcu, std::uint8_t address) {
    std::uint8_t value = 0;
    mcu.read_memory(Cc2530F256Mcu::IdataWindow + address, &value, 1);
    return value;
}

// SLEEPSTA.RST (bits 4:3): 00 power-on/brown-out, 01 external (SWRU191F 4.3).
unsigned reset_field(sim::Mcu &mcu) {
    std::uint8_t value = 0;
    mcu.read_memory(Cc2530F256Mcu::SfrWindow + 0x9d, &value, 1);
    return (value >> 3) & 3u;
}

std::uint8_t flash_byte(sim::Mcu &mcu, std::uint32_t address) {
    std::uint8_t value = 0;
    mcu.read_memory(Cc2530F256Mcu::FlashWindow + address, &value, 1);
    return value;
}

} // namespace

namespace mcuemu_tests {

void test_cc2530_flash_config() {
    const std::string bad_limits = error_of_build(system_yaml("        flash: {limits: maybe}\n"));
    CHECK(contains(bad_limits, "limits must be report or nondeterministic"));
    CHECK(contains(error_of_build(system_yaml("        flash: {interrupted: half}\n")),
                   "interrupted must be abandon, partial or complete"));
    CHECK(contains(error_of_build(system_yaml(
                       "        flash: {power_cuts: [{at: 1ms, op: erase}]}\n")),
                   "takes no 'op'"));
    CHECK(contains(error_of_build(system_yaml("        flash: {power_cuts: [{nth: 2}]}\n")),
                   "requires 'at'"));
    CHECK(contains(error_of_build(system_yaml(
                       "        flash: {power_cuts: [{op: erase, page: 128}]}\n")),
                   "page must be 0-127"));
    CHECK(contains(error_of_build(system_yaml(
                       "        flash: {power_cuts: [{op: write, nth: 0}]}\n")),
                   "nth counts operations from 1"));
    CHECK(contains(error_of_build(system_yaml(
                       "        flash: {power_cuts: [{op: any, action: brownout}]}\n")),
                   "action must be power_cut or reset"));
    CHECK(contains(error_of_build(system_yaml("        flash: {sram_fill: 256}\n")),
                   "sram_fill must be a byte"));
    CHECK(contains(error_of_build(system_yaml(
                       "        flash: {persist: same.bin, log: same.bin}\n")),
                   "persist and log must be different"));
    CHECK(contains(error_of_build(system_yaml("        start_at: 2ms\n"
                                              "        flash: {power_cuts: [{at: 1ms}]}\n")),
                   "comes before the MCU starts"));
    CHECK(contains(error_of_build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                                  "      - {id: c, model: ca51f253l3, firmware: x.hex,"
                                  " flash: {}}\n"),
                   "'flash' is only supported for cc2530f256"));

    const sim::SystemConfig config = sim::parse_system_config(system_yaml(
        "        flash:\n"
        "          limits: nondeterministic\n"
        "          interrupted: partial\n"
        "          seed: 0x1234\n"
        "          sram_fill: random\n"
        "          power_cuts:\n"
        "            - {at: 3ms}\n"
        "            - {op: erase, nth: 2, page: 6, after: 5ms, action: reset}\n"));
    const sim::FlashConfig &flash = *config.devices[0].mcus[0].flash;
    CHECK(flash.limits == "nondeterministic" && flash.interrupted == "partial");
    CHECK(flash.seed == 0x1234 && !flash.sram_fill);
    CHECK(flash.power_cuts.size() == 2);
    CHECK(flash.power_cuts[0].at_ns == 3'000'000u);
    CHECK(!flash.power_cuts[1].at_ns && flash.power_cuts[1].op == "erase");
    CHECK(flash.power_cuts[1].nth == 2 && flash.power_cuts[1].page == 6u);
    CHECK(flash.power_cuts[1].after_ns == 5'000'000u && flash.power_cuts[1].reset);

    // Without `flash:` the endpoint set is unchanged.
    auto plain = sim::build_system(sim::parse_system_config(system_yaml("")));
    CHECK(!plain->world().endpoints().find("d.m.power"));
    auto enabled = sim::build_system(sim::parse_system_config(system_yaml("        flash: {}\n")));
    sim::Endpoint *power = enabled->world().endpoints().find("d.m.power");
    CHECK(power && power->kind() == "power");
}

void test_cc2530_flash_power_cut_log() {
    const std::string log = path_of("cut.log");
    std::filesystem::remove(log);
    std::vector<sim::EndpointEvent> events;
    std::uint8_t seen = 0;
    {
        auto system = sim::build_system(sim::parse_system_config(system_yaml(
            "        flash:\n"
            "          interrupted: partial\n"
            "          seed: 7\n"
            "          log: \"" + log + "\"\n"
            "          power_cuts: [{at: 10ms}]\n")));
        sim::World &world = system->world();
        sim::Mcu &mcu = world.mcu("d.m");
        sim::Endpoint &power = world.endpoints().at("d.m.power");
        power.subscribe([&](const sim::EndpointEvent &event) { events.push_back(event); });
        world.run_until(50'000'000);
        CHECK(std::get<std::int64_t>(power.get()) == 1);
        CHECK(reset_field(mcu) == 0); // power-on
        // The second boot read the partially erased byte and erased again
        // only if it still read 0x00.
        seen = idata(mcu, 0x40);
        CHECK(flash_byte(mcu, 0x3000) == (seen == 0 ? 0xff : seen));
        const std::string text = read_text(log);
        // The entry is logged at the cut: 20 cycles of boot code, then the
        // erase ran until cycle 320000 (10 ms at 32 MHz).
        CHECK(contains(text, "time_ns=10000000 cycle=320000 op=erase page=6 address=0x03000 "
                             "outcome=partial cut=power-cut elapsed=319980"));
        CHECK(contains(text, "cycle=320000 event=power-cut count=1"));
        CHECK(contains(text, "time_ns=10000000 cycle=320000 event=power-cut"));
        CHECK(count_of(text, "outcome=completed") == (seen == 0 ? 1u : 0u));
    }
    CHECK(events.size() == 1);
    if (events.size() == 1) {
        CHECK(events[0].time_ns == 10'000'000);
        CHECK(events[0].endpoint == "d.m.power");
        CHECK(std::get<std::int64_t>(events[0].value) == 1);
    }
    const std::string text = read_text(log);
    CHECK(contains(text, "summary power_cuts=1 interrupted_erases=1 interrupted_writes=0"));
    // A partial erase is not counted as an erase.
    CHECK(contains(text, "summary page=6 erases=1 writes=0") == (seen == 0));
}

void test_cc2530_flash_power_endpoint() {
    auto system = sim::build_system(sim::parse_system_config(system_yaml("        flash: {}\n")));
    sim::World &world = system->world();
    sim::Mcu &mcu = world.mcu("d.m");
    sim::Endpoint &power = world.endpoints().at("d.m.power");
    world.run_until(30'000'000);
    // Erased page 6 at boot, then spins: halted at the self-loop.
    CHECK(mcu.state() == sim::McuState::Halted);
    CHECK(flash_byte(mcu, 0x3000) == 0xff && idata(mcu, 0x40) == 0x00);
    CHECK(reset_field(mcu) == 0);
    power.set(sim::Value{std::string("reset")});
    CHECK(mcu.state() == sim::McuState::Running);
    CHECK(reset_field(mcu) == 1); // external
    world.run_until(31'000'000);
    CHECK(idata(mcu, 0x40) == 0xff); // flash kept, no second erase
    CHECK(std::get<std::int64_t>(power.get()) == 0);
    power.set(sim::Value{std::string("cut")});
    CHECK(std::get<std::int64_t>(power.get()) == 1);
    CHECK(reset_field(mcu) == 0);
    // A halted MCU is brought to the world time before the cut.
    CHECK(mcu.time_ns() >= 31'000'000);
    world.run_until(32'000'000);
    CHECK(idata(mcu, 0x40) == 0xff);
    bool rejected = false;
    try {
        power.set(sim::Value{std::int64_t{1}});
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    CHECK(rejected);
}

void test_cc2530_flash_persist() {
    const std::string persist = path_of("flash.bin");
    const std::string log = path_of("persist.log");
    std::filesystem::remove(persist);
    const std::string yaml = system_yaml("        flash: {persist: \"" + persist +
                                         "\", log: \"" + log + "\"}\n");
    {
        // Built but never run: no file is created.
        auto system = sim::build_system(sim::parse_system_config(yaml));
    }
    CHECK(!std::filesystem::exists(persist));
    {
        auto system = sim::build_system(sim::parse_system_config(yaml));
        system->world().run_until(30'000'000);
        CHECK(idata(system->world().mcu("d.m"), 0x40) == 0x00);
    }
    const std::vector<std::uint8_t> saved = read_file(persist);
    CHECK(saved.size() == 262144);
    if (saved.size() == 262144) {
        CHECK(saved[0] == 0x02 && saved[1] == 0x00 && saved[2] == 0x40);
        bool erased = true;
        for (std::uint32_t i = 0x3000; i < 0x3800; ++i)
            erased &= saved[i] == 0xff;
        CHECK(erased);
    }
    CHECK(contains(read_text(log), "op=erase page=6 address=0x03000 outcome=completed"));
    {
        // The second run starts from the saved flash, not from the image.
        auto system = sim::build_system(sim::parse_system_config(yaml));
        sim::Mcu &mcu = system->world().mcu("d.m");
        CHECK(flash_byte(mcu, 0x3000) == 0xff);
        system->world().run_until(30'000'000);
        CHECK(idata(mcu, 0x40) == 0xff);
    }
    const std::string second = read_text(log);
    CHECK(!contains(second, "op=erase"));
    CHECK(contains(second, "summary power_cuts=0 interrupted_erases=0"));

    // A file that is not a whole CC2530F256 flash is rejected.
    std::ofstream(persist, std::ios::binary | std::ios::trunc) << "short";
    CHECK(contains(error_of_build(yaml), "not the 262144"));
    std::filesystem::remove(persist);
}

} // namespace mcuemu_tests
