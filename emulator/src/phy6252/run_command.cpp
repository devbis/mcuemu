#include "phy6252emu/run_command.hpp"
#include "phy6252emu/phy6252.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace phy6252emu {
namespace {
std::uint32_t parse_address(const std::string &value) {
    std::size_t end = 0;
    const unsigned long long address = std::stoull(value, &end, 0);
    if (end != value.size() || address > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("invalid address: " + value);
    return static_cast<std::uint32_t>(address);
}
}

int run_command(int argc, char **argv) {
    std::string firmware;
    std::uint64_t steps = 0, duration_ns = 0;
    bool trace_stack = false;
    std::optional<std::pair<std::uint32_t, std::uint32_t>> trace_range;
    for (int i = 2; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--firmware" && i + 1 < argc) firmware = argv[++i];
        else if (option == "--model" && i + 1 < argc) ++i;
        else if (option == "--steps" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
                throw std::invalid_argument("invalid instruction count: " + value);
            std::size_t end = 0; steps = std::stoull(value, &end);
            if (!steps || end != value.size()) throw std::invalid_argument("invalid instruction count: " + value);
        } else if (option == "--trace-pc-range" && i + 1 < argc) {
            const std::string range = argv[++i];
            const std::size_t separator = range.find(':');
            if (separator == std::string::npos || range.find(':', separator + 1) != std::string::npos)
                throw std::invalid_argument("--trace-pc-range expects START:END");
            const std::uint32_t first = parse_address(range.substr(0, separator));
            const std::uint32_t last = parse_address(range.substr(separator + 1));
            if (first > last)
                throw std::invalid_argument("--trace-pc-range start must not exceed end");
            trace_range = std::pair{first, last};
        } else if (option == "--for" && i + 1 < argc) {
            const std::string value = argv[++i];
            std::size_t end = 0; const double number = std::stod(value, &end);
            const std::string unit = value.substr(end);
            const double scale = unit == "ns" ? 1 : unit == "us" ? 1e3 : unit == "ms" ? 1e6 : unit == "s" ? 1e9 : 0;
            if (!std::isfinite(number) || number <= 0 || scale == 0 || number * scale > 1e18) throw std::invalid_argument("invalid duration: " + value);
            duration_ns = static_cast<std::uint64_t>(number * scale);
            if (!duration_ns) throw std::invalid_argument("duration is shorter than one nanosecond");
        } else if (option == "--trace-stack") {
            trace_stack = true;
        } else if (option == "--help" || option == "-h") {
            std::cout << "Usage: mcuemu run --model PHY6252 --firmware IMAGE [--steps N] [--for DURATION]\n"
                         "       [--trace-pc-range START:END] [--trace-stack]\n";
            return 0;
        } else throw std::invalid_argument("unknown or incomplete option: " + option);
    }
    if (firmware.empty()) throw std::invalid_argument("--firmware is required");
    std::ifstream input(firmware, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open firmware image: " + firmware);
    std::vector<std::uint8_t> image((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    Phy6252 chip;
    chip.load_firmware(image);
    chip.reset();
    if (trace_range)
        chip.core().set_pc_trace(&std::cerr, trace_range->first, trace_range->second, true);
    if (trace_stack)
        chip.core().set_stack_trace(&std::cerr, BootImage::RamBase,
                                    BootImage::RamBase + BootImage::RamSize);
    if (steps) chip.core().set_instruction_limit(steps);
    if (!steps && !duration_ns) duration_ns = 1000000000;
    cortexm::RunResult result;
    if (duration_ns) {
        constexpr std::uint64_t RunQuantumNs = 1000000; // Recompute cycle deadlines after clk_init().
        while (chip.core().cycles_to_ns(chip.core().cycles()) < duration_ns) {
            const std::uint64_t now_ns = chip.core().cycles_to_ns(chip.core().cycles());
            const std::uint64_t next_ns = std::min(duration_ns, now_ns + RunQuantumNs);
            result = chip.run(chip.core().ns_to_cycles(next_ns));
            if (result.reason != cortexm::StopReason::Deadline)
                break;
        }
    } else {
        result = chip.run(~std::uint64_t{0} >> 1);
    }
    const auto output = chip.take_uart_tx();
    if (!output.empty()) std::cout.write(reinterpret_cast<const char *>(output.data()), output.size());
    std::cout << "\nmodel: PHY6252\ninstructions: " << chip.core().instructions()
              << "\ncycles: " << chip.core().cycles() << "\npc: 0x" << std::hex << chip.core().pc() << std::dec << '\n';
    if (result.reason == cortexm::StopReason::Fault) { std::cerr << "CPU fault: " << result.message << '\n'; return 1; }
    if (result.reason == cortexm::StopReason::Halted) std::cout << "stop: " << result.message << '\n';
    else if (result.reason == cortexm::StopReason::StepLimit) std::cout << "stop: instruction limit\n";
    else std::cout << "stop: time limit\n";
    return 0;
}
} // namespace phy6252emu
