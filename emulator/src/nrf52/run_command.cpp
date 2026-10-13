#include "nrf52emu/run_command.hpp"

#include "nrf52emu/nrf52840.hpp"

#include <cctype>
#include <charconv>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace nrf52emu {

namespace {

void usage(std::ostream &out) {
    out << "Usage: mcuemu run --model NRF52840 --firmware IMAGE [--steps N] [--for DURATION]\n"
           "       [--trace-mmio]\n"
           "\n"
           "IMAGE is a raw binary placed at flash address 0. Without --steps and --for the\n"
           "run lasts 1 s of virtual time. DURATION is a number with ns, us, ms or s.\n"
           "--trace-mmio logs every peripheral register access to stderr.\n";
}

std::uint64_t parse_number(const std::string &text, const char *what) {
    std::uint64_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value == 0)
        throw std::invalid_argument(std::string("invalid ") + what + ": " + text);
    return value;
}

std::uint64_t parse_duration_ns(const std::string &text) {
    std::size_t digits = 0;
    while (digits < text.size() && (std::isdigit(static_cast<unsigned char>(text[digits])) || text[digits] == '.'))
        ++digits;
    if (digits == 0)
        throw std::invalid_argument("invalid duration: " + text);
    const double number = std::stod(text.substr(0, digits));
    const std::string unit = text.substr(digits);
    double scale = 0;
    if (unit == "ns") scale = 1;
    else if (unit == "us") scale = 1e3;
    else if (unit == "ms") scale = 1e6;
    else if (unit == "s") scale = 1e9;
    else throw std::invalid_argument("duration needs a unit (ns, us, ms, s): " + text);
    return static_cast<std::uint64_t>(number * scale);
}

} // namespace

int run_command(int argc, char **argv) {
    std::string firmware;
    std::uint64_t steps = 0;
    std::uint64_t duration_ns = 0;
    bool trace_mmio = false;

    for (int i = 2; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--firmware" && i + 1 < argc)
            firmware = argv[++i];
        else if (option == "--model" && i + 1 < argc)
            ++i; // already used to pick this runner
        else if (option == "--steps" && i + 1 < argc)
            steps = parse_number(argv[++i], "instruction count");
        else if (option == "--for" && i + 1 < argc)
            duration_ns = parse_duration_ns(argv[++i]);
        else if (option == "--trace-mmio")
            trace_mmio = true;
        else if (option == "--help" || option == "-h") {
            usage(std::cout);
            return 0;
        } else {
            throw std::invalid_argument("unknown or incomplete option: " + option);
        }
    }
    if (firmware.empty())
        throw std::invalid_argument("--firmware is required");
    if (steps == 0 && duration_ns == 0)
        duration_ns = 1000000000ull;

    std::ifstream input(firmware, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot open firmware image: " + firmware);
    const std::vector<std::uint8_t> image((std::istreambuf_iterator<char>(input)),
                                          std::istreambuf_iterator<char>());

    Nrf52840 chip;
    chip.load_firmware(image);
    if (trace_mmio)
        chip.set_trace(&std::cerr);
    chip.reset();
    if (steps != 0)
        chip.core().set_instruction_limit(steps);
    const std::uint64_t until =
        duration_ns != 0 ? chip.core().ns_to_cycles(duration_ns) : ~std::uint64_t{0} >> 1;
    const cortexm::RunResult result = chip.run(until);

    cortexm::Core &core = chip.core();
    std::cout << "model: NRF52840\n"
              << "flash image: " << image.size() << " bytes\n"
              << "instructions: " << core.instructions() << '\n'
              << "cycles: " << core.cycles() << '\n'
              << "virtual time: " << chip.time_ns() << " ns\n"
              << "pc: 0x" << std::hex << core.pc() << std::dec << '\n';
    for (unsigned reg = 0; reg != 16; ++reg)
        std::cout << (reg == 13 ? "sp" : reg == 14 ? "lr" : reg == 15 ? "pc" : "r" + std::to_string(reg))
                  << "=0x" << std::hex << core.reg(reg) << std::dec << ((reg & 3u) == 3u ? '\n' : ' ');
    switch (result.reason) {
    case cortexm::StopReason::Fault:
        std::cerr << "CPU fault: " << result.message << '\n';
        return 1;
    case cortexm::StopReason::Halted:
        std::cout << "stop: " << result.message << '\n';
        break;
    case cortexm::StopReason::StepLimit:
        std::cout << "stop: instruction limit\n";
        break;
    default:
        std::cout << (chip.system_off() ? "stop: system off\n"
                                        : core.sleeping() ? "stop: time limit (core asleep)\n"
                                                           : "stop: time limit\n");
        break;
    }
    return 0;
}

} // namespace nrf52emu
