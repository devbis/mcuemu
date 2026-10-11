#include "tc32emu/cpu.hpp"
#include "tc32emu/radio.hpp"
#include "tc32emu/tlsr8258.hpp"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cstdint>
#include <fstream>
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

const char *power_state_name(tc32emu::Tlsr8258PowerState state) {
    switch (state) {
    case tc32emu::Tlsr8258PowerState::Awake:
        return "awake";
    case tc32emu::Tlsr8258PowerState::Suspend:
        return "suspend";
    case tc32emu::Tlsr8258PowerState::DeepSleep:
        return "deep-sleep";
    }
    return "unknown";
}

void usage(std::ostream &out) {
    out << "Usage: tc32-emu run --firmware IMAGE [--model MODEL] [--steps N]\n"
           "       [--trace-pc | --trace-pc-range START:END]\n"
           "       [--trace-mmio | --trace-mmio-range START:END]\n"
           "       [--trace-stack] [--trace-after-step N]\n"
           "       [--trace-limit N] [--fault-trace N]\n"
           "       tc32-emu network --node NAME=IMAGE --node NAME=IMAGE\n"
           "       [--model MODEL] [--channel 11..26] [--steps N]\n"
           "       [--trace-pc-range START:END] [--trace-mmio-range START:END]\n"
           "       [--trace-stack] [--trace-node NAME] [--trace-after-step N]\n"
           "       [--trace-limit N] [--fault-trace N] [--radio-payloads]\n"
           "       [--gpio-event NAME=PORT:VALUE@TIME]\n"
           "       [--trace-outputs]\n"
           "       [--start-at NAME=TIME]\n"
           "       [--dump-mmio NAME=ADDRESS:LENGTH]\n"
           "       [--dump-sram NAME=ADDRESS:LENGTH]\n"
           "       tc32-emu models\n\n"
           "Models: TLSR8258F512, TLSR8258F1K\n"
           "Firmware is loaded as a raw image at flash address 0.\n"
           "Trace ranges are inclusive; addresses accept decimal or 0x hex.\n";
}

std::size_t parse_count(const std::string &text) {
    std::size_t value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size())
        throw std::invalid_argument("invalid instruction count: " + text);
    return value;
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

tc32emu::AddressRange parse_range(const std::string &text) {
    const std::size_t separator = text.find(':');
    if (separator == std::string::npos ||
        text.find(':', separator + 1) != std::string::npos)
        throw std::invalid_argument("address range must be START:END: " + text);
    tc32emu::AddressRange range{parse_address(text.substr(0, separator)),
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

int run_command(int argc, char **argv) {
    std::string firmware;
    std::string model = "TLSR8258F512";
    std::size_t steps = 100000;
    std::size_t trace_limit = 10000;
    std::size_t trace_after_step = 0;
    std::size_t fault_trace = 0;
    bool trace_pc = false;
    bool trace_mmio = false;
    bool trace_stack = false;
    std::optional<tc32emu::AddressRange> pc_range;
    std::optional<tc32emu::AddressRange> mmio_range;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--firmware" && i + 1 < argc)
            firmware = argv[++i];
        else if (arg == "--model" && i + 1 < argc)
            model = argv[++i];
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
        else if (arg == "--trace-mmio-range" && i + 1 < argc) {
            mmio_range = parse_range(argv[++i]);
            trace_mmio = true;
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

    tc32emu::Tlsr8258 device(tc32emu::parse_model(model));
    std::ifstream input(firmware, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot open firmware image: " + firmware);
    std::vector<std::uint8_t> image(
        (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    device.load_firmware(image);

    tc32emu::Cpu cpu(device);
    cpu.set_trace_limit(trace_limit);
    cpu.set_trace_after_step(trace_after_step);
    cpu.set_recent_trace_capacity(fault_trace);
    if (trace_pc)
        cpu.set_pc_trace(&std::cerr, pc_range);
    if (trace_mmio)
        cpu.set_mmio_trace(&std::cerr, mmio_range);
    if (trace_stack)
        cpu.set_stack_trace(&std::cerr);
    const tc32emu::RunResult result = cpu.run(steps);
    std::cout << "model: " << tc32emu::model_name(tc32emu::parse_model(model)) << '\n'
              << "flash image: " << image.size() << " bytes\n"
              << "instructions: " << result.instructions << '\n'
              << "pc: 0x" << std::hex << result.pc << std::dec << '\n'
              << "virtual time: " << device.time_ns() << " ns\n";
    for (unsigned reg = 0; reg != 16; ++reg) {
        std::cout << "r" << reg << "=0x" << std::hex << cpu.reg(reg)
                  << ((reg & 3u) == 3u ? '\n' : ' ');
    }
    if (result.reason == tc32emu::StopReason::Fault) {
        std::cerr << "CPU fault: " << result.message << '\n';
        cpu.dump_recent_trace(std::cerr);
        return 1;
    }
    if (result.reason == tc32emu::StopReason::Halted)
        std::cout << "stopped on tserv software-service trap\n";
    else if (result.reason == tc32emu::StopReason::Sleeping)
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
    std::unique_ptr<tc32emu::Tlsr8258> device;
    std::unique_ptr<tc32emu::Cpu> cpu;
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
    tc32emu::OutputEvent event;
};

int network_command(int argc, char **argv) {
    std::string model = "TLSR8258F512";
    std::uint8_t channel = 11;
    std::size_t steps = 1000000;
    std::size_t trace_limit = 10000;
    std::size_t trace_after_step = 0;
    std::size_t fault_trace = 0;
    std::optional<std::string> trace_node;
    std::optional<tc32emu::AddressRange> pc_range;
    std::optional<tc32emu::AddressRange> mmio_range;
    bool trace_stack = false;
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
            if (length == 0 || address < tc32emu::Tlsr8258::RetainedSramBase ||
                address >= tc32emu::Tlsr8258::RetainedSramBase +
                               tc32emu::Tlsr8258::SramSize ||
                length > tc32emu::Tlsr8258::RetainedSramBase +
                             tc32emu::Tlsr8258::SramSize - address)
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
                address >= tc32emu::Tlsr8258::RegisterBase &&
                address < tc32emu::Tlsr8258::RegisterBase +
                              tc32emu::Tlsr8258::RegisterSize &&
                length <= tc32emu::Tlsr8258::RegisterBase +
                              tc32emu::Tlsr8258::RegisterSize - address;
            const bool vendor_alias_range =
                address >= tc32emu::Tlsr8258::VendorRegisterAliasBase &&
                address < tc32emu::Tlsr8258::VendorRegisterAliasBase +
                              tc32emu::Tlsr8258::RegisterSize &&
                length <= tc32emu::Tlsr8258::VendorRegisterAliasBase +
                              tc32emu::Tlsr8258::RegisterSize - address;
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
    const tc32emu::Tlsr8258Model selected_model = tc32emu::parse_model(model);

    tc32emu::VirtualAir air;
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
        node.device = std::make_unique<tc32emu::Tlsr8258>(selected_model);
        node.device->set_output_trace_enabled(trace_outputs);
        std::ifstream input(node.firmware, std::ios::binary);
        if (!input)
            throw std::runtime_error("cannot open firmware image: " + node.firmware);
        std::vector<std::uint8_t> image(
            (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        node.device->load_firmware(image);
        node.device->connect_radio(air, node.name);
        node.device->set_radio_channel(channel);
        node.cpu = std::make_unique<tc32emu::Cpu>(*node.device);
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
    std::cout << "network: model=" << tc32emu::model_name(selected_model)
              << " zigbee-channel=" << static_cast<unsigned>(channel) << '\n';
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
            const tc32emu::RunResult result = node.cpu->run(count);
            node.instructions += result.instructions;
            global_time_ns = std::max(global_time_ns,
                                      node.device->network_time_ns());
            if (result.reason == tc32emu::StopReason::Halted) {
                node.stopped = true;
            } else if (result.reason == tc32emu::StopReason::Sleeping) {
                std::cout << "node " << node.name << " remains in "
                          << power_state_name(node.device->power_state())
                          << ": " << result.message << '\n';
                node.stopped = true;
            } else if (result.reason == tc32emu::StopReason::Fault) {
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
            for (const tc32emu::OutputEvent &event :
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
            if (line.event.kind == tc32emu::OutputEvent::Kind::GpioPort) {
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
    for (const tc32emu::RadioFrame &frame : air.frames()) {
        std::cout << "  " << frame.id << ' ' << frame.source << " -> "
                  << (frame.destination.empty() ? "*" : frame.destination)
                  << ' ' << tc32emu::protocol_name(frame.protocol) << " ch="
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
    for (const tc32emu::RadioReception &reception : air.receptions()) {
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
                         "TLSR8258F1K (1 MiB flash, 64 KiB SRAM)\n";
            return 0;
        }
        if (std::string(argv[1]) == "network")
            return network_command(argc, argv);
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
