#include "mcs51emu/ca51f253l3.hpp"

#include <charconv>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace {

void usage(std::ostream &out) {
    out << "Usage: mcs51-emu run --firmware IMAGE [--model CA51F253L3] "
           "[--steps N] [--trace]\n"
           "\n"
           "IMAGE may be a raw binary or Intel HEX file.\n"
           "Model: CA51F253L3 (32 KiB flash, 256-byte IDATA, 2 KiB XDATA).\n";
}

std::size_t parse_count(const std::string &text) {
    std::size_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        value == 0)
        throw std::invalid_argument("invalid positive instruction count: " + text);
    return value;
}

int run(int argc, char **argv) {
    std::string firmware;
    std::string model = "CA51F253L3";
    std::size_t steps = 1'000'000;
    bool trace = false;

    for (int i = 2; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--firmware" && i + 1 < argc)
            firmware = argv[++i];
        else if (option == "--model" && i + 1 < argc)
            model = argv[++i];
        else if (option == "--steps" && i + 1 < argc)
            steps = parse_count(argv[++i]);
        else if (option == "--trace")
            trace = true;
        else if (option == "--help" || option == "-h") {
            usage(std::cout);
            return 0;
        } else {
            throw std::invalid_argument("unknown or incomplete option: " + option);
        }
    }

    if (model != "CA51F253L3")
        throw std::invalid_argument("unsupported MCS-51 model: " + model);
    if (firmware.empty())
        throw std::invalid_argument("--firmware is required");

    mcs51emu::Ca51F253L3 device;
    if (trace)
        device.set_trace(&std::cerr);
    device.load_firmware(firmware);
    const mcs51emu::RunResult result = device.run(steps);

    std::cout << "model: " << model << '\n'
              << "instructions: " << result.instructions << '\n'
              << "cycles: " << result.cycles << '\n'
              << "pc: 0x" << std::hex << std::setw(4) << std::setfill('0')
              << result.pc << std::dec << std::setfill(' ') << '\n';
    if (!result.error.empty()) {
        std::cerr << "CPU fault: " << result.error << '\n';
        return 1;
    }
    if (result.step_limit_reached)
        std::cout << "stop: instruction limit\n";
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    if (argc == 1 || std::string(argv[1]) == "--help" ||
        std::string(argv[1]) == "-h") {
        usage(std::cout);
        return 0;
    }
    if (std::string(argv[1]) != "run") {
        usage(std::cerr);
        return 2;
    }
    try {
        return run(argc, argv);
    } catch (const std::exception &error) {
        std::cerr << "mcs51-emu: " << error.what() << '\n';
        return 2;
    }
}
