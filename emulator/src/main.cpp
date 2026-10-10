#include "tc32emu/cpu.hpp"
#include "tc32emu/radio.hpp"
#include "tc32emu/tlsr8258.hpp"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void usage(std::ostream &out) {
    out << "Usage: tc32-emu run --firmware IMAGE [--model MODEL] [--steps N]\n"
           "       [--trace-pc | --trace-pc-range START:END]\n"
           "       [--trace-mmio | --trace-mmio-range START:END]\n"
           "       [--trace-limit N]\n"
           "       tc32-emu network --node NAME=IMAGE --node NAME=IMAGE\n"
           "       [--model MODEL] [--channel 11..26] [--steps N]\n"
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

int run_command(int argc, char **argv) {
    std::string firmware;
    std::string model = "TLSR8258F512";
    std::size_t steps = 100000;
    std::size_t trace_limit = 10000;
    bool trace_pc = false;
    bool trace_mmio = false;
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
        else if (arg == "--trace-mmio-range" && i + 1 < argc) {
            mmio_range = parse_range(argv[++i]);
            trace_mmio = true;
        } else if (arg == "--trace-limit" && i + 1 < argc)
            trace_limit = parse_count(argv[++i]);
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
    if (trace_pc)
        cpu.set_pc_trace(&std::cerr, pc_range);
    if (trace_mmio)
        cpu.set_mmio_trace(&std::cerr, mmio_range);
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
        return 1;
    }
    if (result.reason == tc32emu::StopReason::Halted)
        std::cout << "stopped on tserv software-service trap\n";
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
};

int network_command(int argc, char **argv) {
    std::string model = "TLSR8258F512";
    std::uint8_t channel = 11;
    std::size_t steps = 1000000;
    std::vector<std::pair<std::string, std::string>> node_specs;
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
        node.device = std::make_unique<tc32emu::Tlsr8258>(selected_model);
        std::ifstream input(node.firmware, std::ios::binary);
        if (!input)
            throw std::runtime_error("cannot open firmware image: " + node.firmware);
        std::vector<std::uint8_t> image(
            (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        node.device->load_firmware(image);
        node.device->connect_radio(air, node.name);
        node.device->set_radio_channel(channel);
        node.cpu = std::make_unique<tc32emu::Cpu>(*node.device);
        nodes.push_back(std::move(node));
    }

    // Advance each CPU in short, equal instruction quanta. This keeps virtual
    // clocks close enough that packet airtime and collisions are meaningful.
    constexpr std::size_t quantum = 256;
    bool fault = false;
    std::cout << "network: model=" << tc32emu::model_name(selected_model)
              << " zigbee-channel=" << static_cast<unsigned>(channel) << '\n';
    while (true) {
        bool any_running = false;
        for (NetworkNode &node : nodes) {
            if (node.stopped || node.instructions >= steps)
                continue;
            any_running = true;
            const std::size_t count =
                std::min(quantum, steps - node.instructions);
            const tc32emu::RunResult result = node.cpu->run(count);
            node.instructions += result.instructions;
            if (result.reason == tc32emu::StopReason::Halted) {
                node.stopped = true;
            } else if (result.reason == tc32emu::StopReason::Fault) {
                std::cerr << "node " << node.name << " CPU fault at 0x"
                          << std::hex << result.pc << std::dec << ": "
                          << result.message << '\n';
                std::cerr << "  SP=0x" << std::hex << node.cpu->reg(13)
                          << " LR=0x" << node.cpu->reg(14)
                          << " CPSR=0x" << node.cpu->cpsr() << std::dec << '\n';
                node.stopped = true;
                fault = true;
                break;
            }
        }
        if (!any_running || fault)
            break;
    }

    for (const NetworkNode &node : nodes) {
        std::cout << "node " << node.name << ": " << node.instructions
                  << " instructions, " << node.device->time_ns() << " ns, pc=0x"
                  << std::hex << node.cpu->reg(15) << std::dec << '\n';
    }
    std::cout << "radio frames: " << air.frames().size() << '\n';
    for (const tc32emu::RadioFrame &frame : air.frames()) {
        std::cout << "  " << frame.id << ' ' << frame.source << " -> "
                  << (frame.destination.empty() ? "*" : frame.destination)
                  << ' ' << tc32emu::protocol_name(frame.protocol) << " ch="
                  << static_cast<unsigned>(frame.channel) << " bytes="
                  << frame.payload.size() << " t=" << frame.start_ns << ".."
                  << frame.end_ns << " ns\n";
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
