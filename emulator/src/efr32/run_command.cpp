#include "efr32emu/run_command.hpp"

#include "efr32emu/efr32mg21.hpp"

#include <cctype>
#include <charconv>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace efr32emu {

namespace {

void usage(std::ostream &out) {
    out << "Usage: mcuemu run --model EFR32MG21 --firmware IMAGE [--steps N] [--for DURATION]\n"
           "       [--trace-mmio]\n"
           "\n"
           "IMAGE is a raw binary. An application image (reset vector at or above 0x4000) is\n"
           "placed at 0x4000 and started directly; any other image is placed at 0. Without\n"
           "--steps and --for the run lasts 1 s of virtual time. DURATION is a number with\n"
           "ns, us, ms or s. --trace-mmio logs every peripheral register access to stderr.\n"
           "--dump-memory ADDRESS:COUNT prints COUNT bytes from the end of the run.\n"
           "--trace-pc-range START:END prints each instruction executed in that address range.\n";
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
    std::uint64_t steps = 0, duration_ns = 0;
    bool trace_mmio = false;
    std::vector<std::string> dumps;
    std::uint32_t pc_first = 0, pc_last = 0;
    bool trace_pc = false, trace_pc_regs = false;
    for (int i = 2; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--firmware" && i + 1 < argc)
            firmware = argv[++i];
        else if (option == "--model" && i + 1 < argc)
            ++i;
        else if (option == "--steps" && i + 1 < argc)
            steps = parse_number(argv[++i], "instruction count");
        else if (option == "--for" && i + 1 < argc)
            duration_ns = parse_duration_ns(argv[++i]);
        else if (option == "--trace-mmio")
            trace_mmio = true;
        else if (option == "--dump-memory" && i + 1 < argc)
            dumps.push_back(argv[++i]);
        else if (option == "--trace-pc-regs")
            trace_pc_regs = true;
        else if (option == "--trace-pc-range" && i + 1 < argc) {
            const std::string range = argv[++i];
            const std::size_t colon = range.find(':');
            if (colon == std::string::npos)
                throw std::invalid_argument("--trace-pc-range expects START:END: " + range);
            pc_first = static_cast<std::uint32_t>(std::stoul(range.substr(0, colon), nullptr, 0));
            pc_last = static_cast<std::uint32_t>(std::stoul(range.substr(colon + 1), nullptr, 0));
            trace_pc = true;
        }
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

    Efr32mg21 chip;
    chip.load_firmware(image);
    if (trace_mmio)
        chip.set_trace(&std::cerr);
    if (trace_pc)
        chip.core().set_pc_trace(&std::cerr, pc_first, pc_last, trace_pc_regs);
    chip.reset();
    if (steps != 0)
        chip.core().set_instruction_limit(steps);
    const std::uint64_t until = duration_ns != 0 ? chip.core().ns_to_cycles(duration_ns) : ~std::uint64_t{0} >> 1;
    const cortexm::RunResult result = chip.run(until);

    cortexm::Core &core = chip.core();
    std::cout << "model: EFR32MG21\n"
              << "flash image: " << image.size() << " bytes\n"
              << "instructions: " << core.instructions() << '\n'
              << "cycles: " << core.cycles() << '\n'
              << "virtual time: " << chip.time_ns() << " ns\n"
              << "pc: 0x" << std::hex << core.pc() << std::dec << '\n';
    for (unsigned reg = 0; reg != 16; ++reg)
        std::cout << (reg == 13 ? "sp" : reg == 14 ? "lr" : reg == 15 ? "pc" : "r" + std::to_string(reg))
                  << "=0x" << std::hex << core.reg(reg) << std::dec << ((reg & 3u) == 3u ? '\n' : ' ');
    if (!chip.uart0_output().empty()) {
        std::cout << "UART0 output (" << chip.uart0_output().size() << " bytes):\n";
        std::cout.write(chip.uart0_output().data(), static_cast<std::streamsize>(chip.uart0_output().size()));
        std::cout << '\n';
    }
    for (const std::string &dump : dumps) {
        const std::size_t colon = dump.find(':');
        if (colon == std::string::npos)
            throw std::invalid_argument("--dump-memory expects ADDRESS:COUNT: " + dump);
        const auto address = static_cast<std::uint32_t>(std::stoul(dump.substr(0, colon), nullptr, 0));
        const std::size_t count = std::stoul(dump.substr(colon + 1), nullptr, 0);
        std::vector<std::uint8_t> bytes(count);
        const bool ok = core.read_memory(address, bytes.data(), bytes.size());
        std::cout << "memory 0x" << std::hex << address << std::dec << ':';
        if (!ok)
            std::cout << " unreadable";
        for (std::uint8_t byte : bytes)
            std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte)
                      << std::setfill(' ') << std::dec;
        std::cout << std::dec << '\n';
    }
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
        std::cout << (core.sleeping() ? "stop: time limit (core asleep)\n" : "stop: time limit\n");
        break;
    }
    return 0;
}

} // namespace efr32emu
