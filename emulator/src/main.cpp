#include "sim/cli.hpp"
#include "mcuemu/cpu.hpp"
#include "mcuemu/radio.hpp"
#include "mcuemu/tlsr8258.hpp"
#include "mcs51emu/run_command.hpp"
#include "efr32emu/run_command.hpp"
#include "esp32emu/run_command.hpp"
#include "nrf52emu/run_command.hpp"
#include "phy6252emu/run_command.hpp"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

const char *power_state_name(mcuemu::Tlsr8258PowerState state) {
    switch (state) {
    case mcuemu::Tlsr8258PowerState::Awake:
        return "awake";
    case mcuemu::Tlsr8258PowerState::Suspend:
        return "suspend";
    case mcuemu::Tlsr8258PowerState::DeepSleep:
        return "deep-sleep";
    }
    return "unknown";
}

void usage(std::ostream &out) {
    out << "Usage: mcuemu run --model MODEL --firmware IMAGE [--steps N]\n"
           "       [--rc32k-hz HZ] [--rc32k-calibration VALUE]\n"
           "       [--trace-pc | --trace-pc-range START:END]\n"
           "       [--trace-mmio | --trace-mmio-range START:END]\n"
           "       [--trace-analog]\n"
           "       [--trace-stack] [--trace-after-step N]\n"
           "       [--trace-limit N] [--fault-trace N]\n"
           "       [--areg ADDRESS=VALUE]... [--print-mmio START:END]...\n"
           "       mcuemu network --node NAME=IMAGE --node NAME=IMAGE\n"
           "       --model MODEL [--channel 11..26] [--steps N]\n"
           "       [--rc32k-hz HZ] [--rc32k-calibration VALUE]\n"
           "       [--trace-pc-range START:END] [--trace-mmio-range START:END]\n"
           "       [--trace-analog]\n"
           "       [--trace-stack] [--trace-node NAME] [--trace-after-step N]\n"
           "       [--trace-limit N] [--fault-trace N] [--radio-payloads]\n"
           "       [--gpio-event NAME=PORT:VALUE@TIME]\n"
           "       [--trace-outputs]\n"
           "       [--start-at NAME=TIME]\n"
           "       [--dump-mmio NAME=ADDRESS:LENGTH]\n"
           "       [--dump-sram NAME=ADDRESS:LENGTH]\n"
           "       mcuemu models\n"
           "       mcuemu sim SYSTEM.yaml [--for DURATION] [--quantum DURATION]\n"
           "       [--trace-nets]\n"
           "       mcuemu serve --root DIR [--port N] [--bind ADDR] [--token T]\n"
           "       [--allow-origin ORIGIN]... [--max-sessions N] [--idle-timeout SEC]\n\n"
           "Models: TLSR8258F512, TLSR8258F1K, CA51F253L3 and CC2530F256 (MCS-51), NRF52840 (Cortex-M4),\n"
           "        EFR32MG21 (Cortex-M33), ESP32C6, ESP32H2 (RISC-V), PHY6252 (Cortex-M0; experimental)\n"
           "Firmware is loaded as a raw image at flash address 0.\n"
           "Trace ranges are inclusive; addresses accept decimal or 0x hex.\n";
}

void print_analog_trace(std::ostream &out, const mcuemu::Tlsr8258 &device,
                        const std::string *node_name = nullptr) {
    const auto &events = device.analog_register_events();
    for (const mcuemu::AnalogRegisterEvent &event : events) {
        if (node_name)
            out << '[' << *node_name << "] ";
        out << "areg t=" << event.time_ns << " ns op="
            << (event.direction ==
                        mcuemu::AnalogRegisterEvent::Direction::Read
                    ? 'R'
                    : 'W')
            << " addr=0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(event.address) << " value=0x"
            << std::setw(2) << static_cast<unsigned>(event.value) << std::dec
            << std::setfill(' ') << '\n';
    }
    const std::uint64_t total = device.analog_register_event_count();
    if (total > events.size()) {
        if (node_name)
            out << '[' << *node_name << "] ";
        out << "areg trace limit: recorded " << events.size() << " of "
            << total << " events\n";
    }
}

std::size_t parse_count(const std::string &text) {
    std::size_t value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size())
        throw std::invalid_argument("invalid instruction count: " + text);
    return value;
}

std::uint32_t parse_positive_u32(const std::string &text,
                                 const char *description) {
    const std::size_t value = parse_count(text);
    if (value == 0 || value > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument(std::string(description) +
                                    " must fit a nonzero 32-bit value");
    return static_cast<std::uint32_t>(value);
}

std::uint16_t parse_positive_u16(const std::string &text,
                                 const char *description) {
    const std::size_t value = parse_count(text);
    if (value == 0 || value > std::numeric_limits<std::uint16_t>::max())
        throw std::invalid_argument(std::string(description) +
                                    " must fit a nonzero 16-bit value");
    return static_cast<std::uint16_t>(value);
}

std::uint32_t parse_address(const std::string &text) {
    int base = 10;
    const char *begin = text.data();
    const char *end = begin + text.size();
    if (text.size() > 2 && text[0] == '0' &&
        (text[1] == 'x' || text[1] == 'X')) {
        base = 16;
        begin += 2;
    }
    std::uint32_t value = 0;
    const auto result = std::from_chars(begin, end, value, base);
    if (begin == end || result.ec != std::errc{} || result.ptr != end)
        throw std::invalid_argument("invalid address: " + text);
    return value;
}

mcuemu::AddressRange parse_range(const std::string &text) {
    const std::size_t separator = text.find(':');
    if (separator == std::string::npos ||
        text.find(':', separator + 1) != std::string::npos)
        throw std::invalid_argument("address range must be START:END: " + text);
    mcuemu::AddressRange range{parse_address(text.substr(0, separator)),
                                parse_address(text.substr(separator + 1))};
    if (range.first > range.last)
        throw std::invalid_argument("address range start exceeds end: " + text);
    return range;
}

std::uint64_t parse_duration_ns(const std::string &text) {
    std::uint64_t unit_scale = 0;
    std::size_t suffix_length = 0;
    if (text.ends_with("ns")) {
        unit_scale = 1;
        suffix_length = 2;
    } else if (text.ends_with("us")) {
        unit_scale = 1000;
        suffix_length = 2;
    } else if (text.ends_with("ms")) {
        unit_scale = 1000000;
        suffix_length = 2;
    } else if (text.ends_with('s')) {
        unit_scale = 1000000000;
        suffix_length = 1;
    } else {
        throw std::invalid_argument("time must end in ns, us, ms, or s: " + text);
    }

    const std::string number = text.substr(0, text.size() - suffix_length);
    const std::size_t point = number.find('.');
    if (number.empty() ||
        (point != std::string::npos && number.find('.', point + 1) !=
                                           std::string::npos))
        throw std::invalid_argument("invalid time: " + text);
    const std::string whole_text = number.substr(0, point);
    const std::string fraction_text = point == std::string::npos
                                          ? std::string{}
                                          : number.substr(point + 1);
    if (whole_text.empty() ||
        (point != std::string::npos && fraction_text.empty()))
        throw std::invalid_argument("invalid time: " + text);

    std::uint64_t whole = 0;
    const auto whole_result = std::from_chars(
        whole_text.data(), whole_text.data() + whole_text.size(), whole);
    if (whole_result.ec != std::errc{} ||
        whole_result.ptr != whole_text.data() + whole_text.size())
        throw std::invalid_argument("invalid time: " + text);
    if (whole > std::numeric_limits<std::uint64_t>::max() / unit_scale)
        throw std::invalid_argument("time is too large: " + text);
    std::uint64_t result = whole * unit_scale;

    std::size_t precision = 0;
    while (precision < fraction_text.size() &&
           fraction_text[fraction_text.size() - precision - 1] == '0')
        ++precision;
    const std::size_t significant_length = fraction_text.size() - precision;
    const std::size_t max_precision = unit_scale == 1000000000
                                          ? 9
                                          : unit_scale == 1000000 ? 6
                                          : unit_scale == 1000  ? 3
                                                                : 0;
    if (significant_length > max_precision)
        throw std::invalid_argument("time has sub-nanosecond precision: " + text);
    if (significant_length != 0) {
        std::uint64_t fraction = 0;
        const auto fraction_result = std::from_chars(
            fraction_text.data(), fraction_text.data() + significant_length,
            fraction);
        if (fraction_result.ec != std::errc{} ||
            fraction_result.ptr != fraction_text.data() + significant_length)
            throw std::invalid_argument("invalid time: " + text);
        for (std::size_t i = significant_length; i < max_precision; ++i)
            fraction *= 10;
        if (fraction > std::numeric_limits<std::uint64_t>::max() - result)
            throw std::invalid_argument("time is too large: " + text);
        result += fraction;
    }
    return result;
}

std::uint64_t instructions_for_time(std::uint64_t time_ns,
                                   std::uint32_t clock_hz) {
    constexpr std::uint64_t nanoseconds_per_instruction_denominator =
        3u * 1000000000u;
    const std::uint64_t whole_periods =
        time_ns / nanoseconds_per_instruction_denominator;
    const std::uint64_t remainder_ns =
        time_ns % nanoseconds_per_instruction_denominator;
    if (clock_hz != 0 &&
        whole_periods > std::numeric_limits<std::uint64_t>::max() / clock_hz)
        return std::numeric_limits<std::uint64_t>::max();
    const std::uint64_t whole_instructions = whole_periods * clock_hz;
    const std::uint64_t remainder_cycles = remainder_ns * clock_hz;
    const std::uint64_t partial_instructions =
        remainder_cycles / nanoseconds_per_instruction_denominator +
        (remainder_cycles % nanoseconds_per_instruction_denominator != 0);
    if (partial_instructions >
        std::numeric_limits<std::uint64_t>::max() - whole_instructions)
        return std::numeric_limits<std::uint64_t>::max();
    return whole_instructions + partial_instructions;
}

struct GpioEvent {
    std::string node_name;
    unsigned port;
    std::uint8_t value;
    std::uint64_t time_ns;
    bool applied{false};
};

GpioEvent parse_gpio_event(const std::string &text) {
    const std::size_t equals = text.find('=');
    const std::size_t colon = text.find(':', equals == std::string::npos
                                                 ? 0
                                                 : equals + 1);
    const std::size_t at = text.find('@', colon == std::string::npos
                                             ? 0
                                             : colon + 1);
    if (equals == std::string::npos || equals == 0 || colon != equals + 2 ||
        at == std::string::npos || at == colon + 1 || at + 1 == text.size())
        throw std::invalid_argument(
            "GPIO event must be NAME=PORT:VALUE@TIME: " + text);

    const std::string port_text = text.substr(equals + 1, 1);
    unsigned port = 0;
    const char port_char = static_cast<char>(
        std::toupper(static_cast<unsigned char>(port_text[0])));
    if (port_char >= 'A' && port_char <= 'E')
        port = static_cast<unsigned>(port_char - 'A');
    else if (port_char >= '0' && port_char <= '4')
        port = static_cast<unsigned>(port_char - '0');
    else
        throw std::invalid_argument("GPIO port must be A..E or 0..4: " + text);

    const std::uint32_t value = parse_address(text.substr(colon + 1,
                                                          at - colon - 1));
    if (value > 0xff)
        throw std::invalid_argument("GPIO value must fit in one byte: " + text);
    return {text.substr(0, equals), port, static_cast<std::uint8_t>(value),
            parse_duration_ns(text.substr(at + 1))};
}

// `run` dispatches on --model: the CA51 family has its own runner, everything
// else is a TLSR8258 variant.
void require_model(const std::string &model) {
    if (model.empty())
        throw std::invalid_argument(
            "--model is required (TLSR8258F512, TLSR8258F1K, CA51F253L3, CC2530F256, NRF52840; "
            "see `mcuemu models`)");
}

// The TLSR8258 parser only knows the TLSR variants; `run` also dispatches
// the MCS-51 and nRF52 models before it gets here.
mcuemu::Tlsr8258Model parse_run_model(const std::string &model) {
    try {
        return mcuemu::parse_model(model);
    } catch (const std::invalid_argument &) {
        throw std::invalid_argument("unknown model '" + model +
                                    "'; use TLSR8258F512, TLSR8258F1K, CA51F253L3, "
                                    "CC2530F256 or NRF52840 (see `mcuemu models`)");
    }
}

bool is_mcs51_model(const std::string &model) {
    return model.rfind("CA51", 0) == 0 || model == "CC2530F256";
}
bool is_nrf52_model(const std::string &model) { return model.rfind("NRF52", 0) == 0; }
bool is_efr32_model(const std::string &model) { return model.rfind("EFR32", 0) == 0; }
bool is_esp32_model(const std::string &model) { return model.rfind("ESP32", 0) == 0; }

int run_command(int argc, char **argv) {
    for (int i = 2; i + 1 < argc; ++i)
        if (std::string(argv[i]) == "--model" && is_mcs51_model(argv[i + 1]))
            return mcs51emu::run_command(argc, argv);
        else if (std::string(argv[i]) == "--model" && is_nrf52_model(argv[i + 1]))
            return nrf52emu::run_command(argc, argv);
        else if (std::string(argv[i]) == "--model" && is_efr32_model(argv[i + 1]))
            return efr32emu::run_command(argc, argv);
        else if (std::string(argv[i]) == "--model" && is_esp32_model(argv[i + 1]))
            return esp32emu::run_command(argc, argv);
        else if (std::string(argv[i]) == "--model" && std::string(argv[i + 1]) == "PHY6252")
            return phy6252emu::run_command(argc, argv);

    std::string firmware;
    std::string model;
    std::uint32_t rc32k_hz = mcuemu::Tlsr8258::DefaultRc32kFrequencyHz;
    std::uint16_t rc32k_calibration =
        mcuemu::Tlsr8258::DefaultRc32kCalibration;
    std::size_t steps = 100000;
    std::size_t trace_limit = 10000;
    std::size_t trace_after_step = 0;
    std::size_t fault_trace = 0;
    bool trace_pc = false;
    bool trace_mmio = false;
    bool trace_stack = false;
    bool trace_analog = false;
    std::optional<mcuemu::AddressRange> pc_range;
    std::optional<mcuemu::AddressRange> mmio_range;
    std::vector<std::pair<std::uint8_t, std::uint8_t>> analog_presets;
    std::vector<mcuemu::AddressRange> print_ranges;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--firmware" && i + 1 < argc)
            firmware = argv[++i];
        else if (arg == "--model" && i + 1 < argc)
            model = argv[++i];
        else if (arg == "--rc32k-hz" && i + 1 < argc)
            rc32k_hz = parse_positive_u32(argv[++i], "RC 32 kHz frequency");
        else if (arg == "--rc32k-calibration" && i + 1 < argc)
            rc32k_calibration =
                parse_positive_u16(argv[++i], "RC 32 kHz calibration");
        else if (arg == "--steps" && i + 1 < argc)
            steps = parse_count(argv[++i]);
        else if (arg == "--trace-pc")
            trace_pc = true;
        else if (arg == "--trace-pc-range" && i + 1 < argc) {
            pc_range = parse_range(argv[++i]);
            trace_pc = true;
        } else if (arg == "--trace-mmio")
            trace_mmio = true;
        else if (arg == "--trace-stack")
            trace_stack = true;
        else if (arg == "--trace-analog")
            trace_analog = true;
        else if (arg == "--trace-mmio-range" && i + 1 < argc) {
            mmio_range = parse_range(argv[++i]);
            trace_mmio = true;
        } else if (arg == "--areg" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const std::size_t eq = spec.find('=');
            if (eq == std::string::npos)
                throw std::invalid_argument("--areg expects ADDRESS=VALUE: " + spec);
            analog_presets.emplace_back(
                static_cast<std::uint8_t>(std::stoul(spec.substr(0, eq), nullptr, 0)),
                static_cast<std::uint8_t>(std::stoul(spec.substr(eq + 1), nullptr, 0)));
        } else if (arg == "--print-mmio" && i + 1 < argc) {
            print_ranges.push_back(parse_range(argv[++i]));
        } else if (arg == "--trace-limit" && i + 1 < argc)
            trace_limit = parse_count(argv[++i]);
        else if (arg == "--trace-after-step" && i + 1 < argc)
            trace_after_step = parse_count(argv[++i]);
        else if (arg == "--fault-trace" && i + 1 < argc)
            fault_trace = parse_count(argv[++i]);
        else if (arg == "--help" || arg == "-h") {
            usage(std::cout);
            return 0;
        } else {
            throw std::invalid_argument("unknown or incomplete option: " + arg);
        }
    }
    if (firmware.empty())
        throw std::invalid_argument("--firmware is required");
    require_model(model);

    mcuemu::Tlsr8258 device(parse_run_model(model));
    device.set_rc32k_characteristics(rc32k_hz, rc32k_calibration);
    device.set_analog_trace_enabled(trace_analog, trace_limit);
    std::ifstream input(firmware, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot open firmware image: " + firmware);
    std::vector<std::uint8_t> image(
        (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    device.load_firmware(image);

    // Analog registers survive a deep-sleep wake: presetting AREG 0x3a to the
    // marker the firmware leaves makes the start a timed deep wake.
    for (const auto &[address, value] : analog_presets) {
        device.write8(0x8000b8, address);
        device.write8(0x8000b9, value);
        device.write8(0x8000ba, 0x60); // CYC0 | write
    }
    mcuemu::Cpu cpu(device);
    cpu.set_trace_limit(trace_limit);
    cpu.set_trace_after_step(trace_after_step);
    cpu.set_recent_trace_capacity(fault_trace);
    if (trace_pc)
        cpu.set_pc_trace(&std::cerr, pc_range);
    if (trace_mmio)
        cpu.set_mmio_trace(&std::cerr, mmio_range);
    if (trace_stack)
        cpu.set_stack_trace(&std::cerr);
    const mcuemu::RunResult result = cpu.run(steps);
    if (trace_analog)
        print_analog_trace(std::cerr, device);
    std::cout << "model: " << mcuemu::model_name(mcuemu::parse_model(model)) << '\n'
              << "flash image: " << image.size() << " bytes\n"
              << "RC 32 kHz: " << rc32k_hz << " Hz, calibration "
              << rc32k_calibration << '\n'
              << "instructions: " << result.instructions << '\n'
              << "pc: 0x" << std::hex << result.pc << std::dec << '\n'
              << "virtual time: " << device.time_ns() << " ns\n";
    for (unsigned reg = 0; reg != 16; ++reg) {
        std::cout << "r" << reg << "=0x" << std::hex << cpu.reg(reg)
                  << ((reg & 3u) == 3u ? '\n' : ' ');
    }
    for (const mcuemu::AddressRange &range : print_ranges) {
        std::cout << "mmio 0x" << std::hex << range.first << ":";
        for (std::uint32_t address = range.first; address <= range.last; ++address)
            std::cout << ' ' << std::setw(2) << std::setfill('0')
                      << static_cast<unsigned>(device.read8(address));
        std::cout << std::dec << std::setfill(' ') << '\n';
    }
    if (result.reason == mcuemu::StopReason::Fault) {
        std::cerr << "CPU fault: " << result.message << '\n';
        cpu.dump_recent_trace(std::cerr);
        return 1;
    }
    if (result.reason == mcuemu::StopReason::Halted)
        std::cout << "stopped on tserv software-service trap\n";
    else if (result.reason == mcuemu::StopReason::Sleeping)
        std::cout << "CPU remains in "
                  << power_state_name(device.power_state()) << ": "
                  << result.message << '\n';
    else
        std::cout << "instruction limit reached\n";
    return 0;
}

struct NetworkNode {
    std::string name;
    std::string firmware;
    std::unique_ptr<mcuemu::Tlsr8258> device;
    std::unique_ptr<mcuemu::Cpu> cpu;
    std::size_t instructions{0};
    bool stopped{false};
    std::uint64_t start_at_ns{0};
    bool started{true};
    bool start_time_set{false};
    std::size_t instruction_limit{0};
};

struct SramDump {
    std::string node_name;
    std::uint32_t address;
    std::size_t length;
};

struct MmioDump {
    std::string node_name;
    std::uint32_t address;
    std::size_t length;
};

struct OutputEventLine {
    std::string node_name;
    mcuemu::OutputEvent event;
};

int network_command(int argc, char **argv) {
    std::string model;
    std::uint32_t rc32k_hz = mcuemu::Tlsr8258::DefaultRc32kFrequencyHz;
    std::uint16_t rc32k_calibration =
        mcuemu::Tlsr8258::DefaultRc32kCalibration;
    std::uint8_t channel = 11;
    std::size_t steps = 1000000;
    std::size_t trace_limit = 10000;
    std::size_t trace_after_step = 0;
    std::size_t fault_trace = 0;
    std::optional<std::string> trace_node;
    std::optional<mcuemu::AddressRange> pc_range;
    std::optional<mcuemu::AddressRange> mmio_range;
    bool trace_stack = false;
    bool trace_analog = false;
    bool radio_payloads = false;
    bool trace_outputs = false;
    std::vector<SramDump> sram_dumps;
    std::vector<MmioDump> mmio_dumps;
    std::vector<std::pair<std::string, std::string>> node_specs;
    std::vector<GpioEvent> gpio_events;
    std::vector<std::pair<std::string, std::uint64_t>> node_start_times;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--node" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const std::size_t separator = spec.find('=');
            if (separator == std::string::npos || separator == 0 ||
                separator + 1 == spec.size())
                throw std::invalid_argument("node must be NAME=IMAGE: " + spec);
            node_specs.emplace_back(spec.substr(0, separator),
                                    spec.substr(separator + 1));
        } else if (arg == "--model" && i + 1 < argc) {
            model = argv[++i];
        } else if (arg == "--rc32k-hz" && i + 1 < argc) {
            rc32k_hz = parse_positive_u32(argv[++i], "RC 32 kHz frequency");
        } else if (arg == "--rc32k-calibration" && i + 1 < argc) {
            rc32k_calibration =
                parse_positive_u16(argv[++i], "RC 32 kHz calibration");
        } else if (arg == "--channel" && i + 1 < argc) {
            const std::size_t parsed = parse_count(argv[++i]);
            if (parsed < 11 || parsed > 26)
                throw std::invalid_argument("Zigbee channel must be 11..26");
            channel = static_cast<std::uint8_t>(parsed);
        } else if (arg == "--steps" && i + 1 < argc) {
            steps = parse_count(argv[++i]);
        } else if (arg == "--trace-pc-range" && i + 1 < argc) {
            pc_range = parse_range(argv[++i]);
        } else if (arg == "--trace-mmio-range" && i + 1 < argc) {
            mmio_range = parse_range(argv[++i]);
        } else if (arg == "--trace-stack") {
            trace_stack = true;
        } else if (arg == "--trace-analog") {
            trace_analog = true;
        } else if (arg == "--radio-payloads") {
            radio_payloads = true;
        } else if (arg == "--trace-outputs") {
            trace_outputs = true;
        } else if (arg == "--gpio-event" && i + 1 < argc) {
            gpio_events.push_back(parse_gpio_event(argv[++i]));
        } else if (arg == "--start-at" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const std::size_t equals = spec.find('=');
            if (equals == std::string::npos || equals == 0 ||
                equals + 1 == spec.size())
                throw std::invalid_argument("node start must be NAME=TIME: " +
                                            spec);
            node_start_times.emplace_back(spec.substr(0, equals),
                                          parse_duration_ns(spec.substr(equals + 1)));
        } else if (arg == "--dump-sram" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const std::size_t equals = spec.find('=');
            const std::size_t colon = spec.find(':', equals == std::string::npos
                                                         ? 0
                                                         : equals + 1);
            if (equals == std::string::npos || equals == 0 ||
                colon == std::string::npos || colon == equals + 1 ||
                colon + 1 == spec.size())
                throw std::invalid_argument(
                    "SRAM dump must be NAME=ADDRESS:LENGTH: " + spec);
            const std::string name = spec.substr(0, equals);
            const std::uint32_t address =
                parse_address(spec.substr(equals + 1, colon - equals - 1));
            const std::size_t length = parse_count(spec.substr(colon + 1));
            if (length == 0 || address < mcuemu::Tlsr8258::RetainedSramBase ||
                address >= mcuemu::Tlsr8258::RetainedSramBase +
                               mcuemu::Tlsr8258::SramSize ||
                length > mcuemu::Tlsr8258::RetainedSramBase +
                             mcuemu::Tlsr8258::SramSize - address)
                throw std::invalid_argument(
                    "SRAM dump range must fit mapped TLSR8258 SRAM: " + spec);
            sram_dumps.push_back({name, address, length});
        } else if (arg == "--dump-mmio" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const std::size_t equals = spec.find('=');
            const std::size_t colon = spec.find(':', equals == std::string::npos
                                                         ? 0
                                                         : equals + 1);
            if (equals == std::string::npos || equals == 0 ||
                colon == std::string::npos || colon == equals + 1 ||
                colon + 1 == spec.size())
                throw std::invalid_argument(
                    "MMIO dump must be NAME=ADDRESS:LENGTH: " + spec);
            const std::string name = spec.substr(0, equals);
            const std::uint32_t address =
                parse_address(spec.substr(equals + 1, colon - equals - 1));
            const std::size_t length = parse_count(spec.substr(colon + 1));
            const bool documented_range =
                address >= mcuemu::Tlsr8258::RegisterBase &&
                address < mcuemu::Tlsr8258::RegisterBase +
                              mcuemu::Tlsr8258::RegisterSize &&
                length <= mcuemu::Tlsr8258::RegisterBase +
                              mcuemu::Tlsr8258::RegisterSize - address;
            const bool vendor_alias_range =
                address >= mcuemu::Tlsr8258::VendorRegisterAliasBase &&
                address < mcuemu::Tlsr8258::VendorRegisterAliasBase +
                              mcuemu::Tlsr8258::RegisterSize &&
                length <= mcuemu::Tlsr8258::VendorRegisterAliasBase +
                              mcuemu::Tlsr8258::RegisterSize - address;
            if (length == 0 || (!documented_range && !vendor_alias_range))
                throw std::invalid_argument(
                    "MMIO dump range must fit the TLSR8258 register window: " +
                    spec);
            mmio_dumps.push_back({name, address, length});
        } else if (arg == "--trace-limit" && i + 1 < argc) {
            trace_limit = parse_count(argv[++i]);
        } else if (arg == "--trace-after-step" && i + 1 < argc) {
            trace_after_step = parse_count(argv[++i]);
        } else if (arg == "--fault-trace" && i + 1 < argc) {
            fault_trace = parse_count(argv[++i]);
        } else if (arg == "--trace-node" && i + 1 < argc) {
            trace_node = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            usage(std::cout);
            return 0;
        } else {
            throw std::invalid_argument("unknown or incomplete option: " + arg);
        }
    }
    if (node_specs.size() < 2)
        throw std::invalid_argument("network requires at least two --node entries");
    require_model(model);
    const mcuemu::Tlsr8258Model selected_model = mcuemu::parse_model(model);

    mcuemu::VirtualAir air;
    std::vector<NetworkNode> nodes;
    nodes.reserve(node_specs.size());
    for (auto &[name, firmware] : node_specs) {
        if (std::any_of(nodes.begin(), nodes.end(), [&](const NetworkNode &node) {
                return node.name == name;
            }))
            throw std::invalid_argument("duplicate virtual node name: " + name);

        NetworkNode node;
        node.name = std::move(name);
        node.firmware = std::move(firmware);
        node.instruction_limit = steps;
        node.device = std::make_unique<mcuemu::Tlsr8258>(selected_model);
        node.device->set_rc32k_characteristics(rc32k_hz, rc32k_calibration);
        node.device->set_analog_trace_enabled(
            trace_analog && (!trace_node || *trace_node == node.name),
            trace_limit);
        node.device->set_output_trace_enabled(trace_outputs);
        std::ifstream input(node.firmware, std::ios::binary);
        if (!input)
            throw std::runtime_error("cannot open firmware image: " + node.firmware);
        std::vector<std::uint8_t> image(
            (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        node.device->load_firmware(image);
        node.device->connect_radio(air, node.name);
        node.device->set_radio_channel(channel);
        node.cpu = std::make_unique<mcuemu::Cpu>(*node.device);
        if (!trace_node || *trace_node == node.name) {
            node.cpu->set_trace_prefix("[" + node.name + "] ");
            node.cpu->set_trace_limit(trace_limit);
            node.cpu->set_trace_after_step(trace_after_step);
            node.cpu->set_recent_trace_capacity(fault_trace);
            if (pc_range)
                node.cpu->set_pc_trace(&std::cerr, pc_range);
            if (mmio_range)
                node.cpu->set_mmio_trace(&std::cerr, mmio_range);
            if (trace_stack)
                node.cpu->set_stack_trace(&std::cerr);
        }
        nodes.push_back(std::move(node));
    }

    for (const SramDump &dump : sram_dumps) {
        if (std::none_of(nodes.begin(), nodes.end(), [&](const NetworkNode &node) {
                return node.name == dump.node_name;
            }))
            throw std::invalid_argument("unknown node in SRAM dump: " +
                                        dump.node_name);
    }
    for (const MmioDump &dump : mmio_dumps) {
        if (std::none_of(nodes.begin(), nodes.end(), [&](const NetworkNode &node) {
                return node.name == dump.node_name;
            }))
            throw std::invalid_argument("unknown node in MMIO dump: " +
                                        dump.node_name);
    }
    for (const GpioEvent &event : gpio_events) {
        if (std::none_of(nodes.begin(), nodes.end(), [&](const NetworkNode &node) {
                return node.name == event.node_name;
            }))
            throw std::invalid_argument("unknown node in GPIO event: " +
                                        event.node_name);
    }
    for (const auto &[node_name, start_at_ns] : node_start_times) {
        auto node = std::find_if(nodes.begin(), nodes.end(),
                                 [&](const NetworkNode &candidate) {
                                     return candidate.name == node_name;
                                 });
        if (node == nodes.end())
            throw std::invalid_argument("unknown node in start time: " +
                                        node_name);
        if (node->start_time_set)
            throw std::invalid_argument("duplicate start time for node: " +
                                        node_name);
        node->start_time_set = true;
        node->start_at_ns = start_at_ns;
        node->started = (start_at_ns == 0);
        const std::uint64_t skipped_instructions = instructions_for_time(
            start_at_ns, node->device->clock_hz());
        node->instruction_limit =
            skipped_instructions >= steps
                ? 0
                : steps - static_cast<std::size_t>(skipped_instructions);
        node->device->set_radio_time_offset(start_at_ns);
    }

    // Advance each CPU in short, equal instruction quanta. This keeps virtual
    // clocks close enough that packet airtime and collisions are meaningful.
    constexpr std::size_t quantum = 256;
    bool fault = false;
    std::uint64_t global_time_ns = 0;
    std::cout << "network: model=" << mcuemu::model_name(selected_model)
              << " zigbee-channel=" << static_cast<unsigned>(channel)
              << " RC32k=" << rc32k_hz << "Hz"
              << " calibration=" << rc32k_calibration << '\n';
    while (true) {
        bool any_running = false;
        std::optional<std::uint64_t> next_start_ns;
        for (NetworkNode &node : nodes) {
            if (node.stopped || node.instructions >= node.instruction_limit)
                continue;
            if (!node.started) {
                if (global_time_ns < node.start_at_ns) {
                    if (!next_start_ns || node.start_at_ns < *next_start_ns)
                        next_start_ns = node.start_at_ns;
                    continue;
                }
                node.started = true;
                std::cout << "node " << node.name << " starts at "
                          << node.start_at_ns << " ns\n";
            }
            any_running = true;
            const std::uint64_t node_time_ns = node.device->network_time_ns();
            for (GpioEvent &event : gpio_events) {
                if (!event.applied && event.node_name == node.name &&
                    node_time_ns >= event.time_ns) {
                    node.device->set_gpio_input(event.port, event.value);
                    event.applied = true;
                    std::cout << "gpio: node " << node.name << " port "
                              << static_cast<char>('A' + event.port) << "=0x"
                              << std::hex << static_cast<unsigned>(event.value)
                              << std::dec << " at " << node_time_ns
                              << " ns\n";
                }
            }
            const std::size_t count =
                std::min(quantum,
                         node.instruction_limit - node.instructions);
            const mcuemu::RunResult result = node.cpu->run(count);
            node.instructions += result.instructions;
            global_time_ns = std::max(global_time_ns,
                                      node.device->network_time_ns());
            if (result.reason == mcuemu::StopReason::Halted) {
                node.stopped = true;
            } else if (result.reason == mcuemu::StopReason::Sleeping) {
                std::cout << "node " << node.name << " remains in "
                          << power_state_name(node.device->power_state())
                          << ": " << result.message << '\n';
                node.stopped = true;
            } else if (result.reason == mcuemu::StopReason::Fault) {
                std::cerr << "node " << node.name << " CPU fault at 0x"
                          << std::hex << result.pc << std::dec << ": "
                          << result.message << '\n';
                std::cerr << "  SP=0x" << std::hex << node.cpu->reg(13)
                          << " LR=0x" << node.cpu->reg(14)
                          << " CPSR=0x" << node.cpu->cpsr() << std::dec << '\n';
                node.cpu->dump_recent_trace(std::cerr);
                node.stopped = true;
                fault = true;
                break;
            }
        }
        if (fault)
            break;
        if (!any_running) {
            if (next_start_ns) {
                global_time_ns = *next_start_ns;
                continue;
            }
            break;
        }
    }

    for (const NetworkNode &node : nodes) {
        std::cout << "node " << node.name << ": " << node.instructions
                  << " instructions, " << node.device->network_time_ns()
                  << " ns, pc=0x"
                  << std::hex << node.cpu->reg(15) << std::dec
                  << ", power=" << power_state_name(node.device->power_state())
                  << '\n';
    }
    if (trace_analog) {
        for (const NetworkNode &node : nodes) {
            if (!trace_node || *trace_node == node.name)
                print_analog_trace(std::cerr, *node.device, &node.name);
        }
    }
    for (const SramDump &dump : sram_dumps) {
        const auto node = std::find_if(nodes.begin(), nodes.end(),
                                       [&](const NetworkNode &candidate) {
                                           return candidate.name == dump.node_name;
                                       });
        if (node == nodes.end())
            throw std::invalid_argument("unknown node in SRAM dump: " +
                                        dump.node_name);
        std::cout << "node " << dump.node_name << " SRAM 0x" << std::hex
                  << dump.address << " =";
        for (std::size_t i = 0; i < dump.length; ++i) {
            const unsigned byte = node->device->read8(
                dump.address + static_cast<std::uint32_t>(i));
            std::cout << ' ' << (byte < 16 ? "0" : "") << byte;
        }
        std::cout << std::dec << '\n';
    }
    for (const MmioDump &dump : mmio_dumps) {
        const auto node = std::find_if(nodes.begin(), nodes.end(),
                                       [&](const NetworkNode &candidate) {
                                           return candidate.name == dump.node_name;
                                       });
        if (node == nodes.end())
            throw std::invalid_argument("unknown node in MMIO dump: " +
                                        dump.node_name);
        std::cout << "node " << dump.node_name << " MMIO 0x" << std::hex
                  << dump.address << " =";
        for (std::size_t i = 0; i < dump.length; ++i) {
            const unsigned byte = node->device->read8(
                dump.address + static_cast<std::uint32_t>(i));
            std::cout << ' ' << (byte < 16 ? "0" : "") << byte;
        }
        std::cout << std::dec << '\n';
    }
    if (trace_outputs) {
        std::vector<OutputEventLine> output_events;
        for (const NetworkNode &node : nodes) {
            for (const mcuemu::OutputEvent &event :
                 node.device->output_events())
                output_events.push_back({node.name, event});
        }
        std::sort(output_events.begin(), output_events.end(),
                  [](const OutputEventLine &lhs, const OutputEventLine &rhs) {
                      if (lhs.event.time_ns != rhs.event.time_ns)
                          return lhs.event.time_ns < rhs.event.time_ns;
                      if (lhs.node_name != rhs.node_name)
                          return lhs.node_name < rhs.node_name;
                      return static_cast<unsigned>(lhs.event.kind) <
                             static_cast<unsigned>(rhs.event.kind);
                  });
        for (const OutputEventLine &line : output_events) {
            std::cout << "output: node " << line.node_name << ' ';
            if (line.event.kind == mcuemu::OutputEvent::Kind::GpioPort) {
                std::cout << "GPIO port "
                          << static_cast<char>('A' + line.event.index)
                          << " latch=0x";
            } else {
                std::cout << "PWM enable-mask=0x";
            }
            std::cout << std::hex << static_cast<unsigned>(line.event.value)
                      << std::dec << " at " << line.event.time_ns << " ns\n";
        }
    }
    std::cout << "radio frames: " << air.frames().size() << '\n';
    for (const mcuemu::RadioFrame &frame : air.frames()) {
        std::cout << "  " << frame.id << ' ' << frame.source << " -> "
                  << (frame.destination.empty() ? "*" : frame.destination)
                  << ' ' << mcuemu::protocol_name(frame.protocol) << " ch="
                  << static_cast<unsigned>(frame.channel) << " bytes="
                  << frame.payload.size() << " t=" << frame.start_ns << ".."
                  << frame.end_ns << " ns\n";
        if (radio_payloads) {
            std::cout << "    payload=";
            constexpr char hex[] = "0123456789abcdef";
            for (std::uint8_t byte : frame.payload)
                std::cout << hex[byte >> 4] << hex[byte & 0x0fu];
            std::cout << '\n';
        }
    }
    std::cout << "radio receptions: " << air.receptions().size() << '\n';
    for (const mcuemu::RadioReception &reception : air.receptions()) {
        std::cout << "  frame " << reception.frame_id << " -> "
                  << reception.receiver << '\n';
    }
    return fault ? 1 : 0;
}

} // namespace

int main(int argc, char **argv) {
    try {
        if (argc < 2 || std::string(argv[1]) == "--help" ||
            std::string(argv[1]) == "-h") {
            usage(std::cout);
            return argc < 2 ? 2 : 0;
        }
        if (std::string(argv[1]) == "models") {
            std::cout << "TLSR8258F512 (512 KiB flash, 64 KiB SRAM)\n"
                         "TLSR8258F1K (1 MiB flash, 64 KiB SRAM)\n"
                         "CA51F253L3 (32 KiB code flash, MCS-51; `run` and `sim`)\n"
                         "CC2530F256 (256 KiB banked flash, 8 KiB SRAM, MCS-51; `run` and `sim`)\n"
                         "NRF52840 (1 MiB flash, 256 KiB RAM, Cortex-M4F; `run`)\n"
                         "EFR32MG21 (512 KiB flash, 64 KiB RAM, Cortex-M33; `run` and `sim`)\n"
                         "PHY6252 (512 KiB flash, 64 KiB RAM, Cortex-M0; experimental)\n"
                         "ESP32C6 (4 MiB flash, 512 KiB SRAM) and ESP32H2 (320 KiB SRAM), RISC-V RV32IMAC; work in progress\n";
            return 0;
        }
        if (std::string(argv[1]) == "network")
            return network_command(argc, argv);
        if (std::string(argv[1]) == "sim")
            return sim::sim_command(argc, argv);
        if (std::string(argv[1]) == "serve")
            return sim::serve_command(argc, argv);
        if (std::string(argv[1]) != "run") {
            usage(std::cerr);
            return 2;
        }
        return run_command(argc, argv);
    } catch (const std::exception &error) {
        std::cerr << "error: " << error.what() << '\n';
        usage(std::cerr);
        return 2;
    }
}
