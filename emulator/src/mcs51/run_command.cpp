#include "mcs51emu/run_command.hpp"

#include "mcs51emu/ca51f253l3.hpp"

#include <charconv>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace mcs51emu {

namespace {

void usage(std::ostream &out) {
    out << "Usage: mcuemu run --model CA51F253L3 --firmware IMAGE "
           "[--steps N] [--trace] [--state]\n"
           "\n"
           "IMAGE may be a raw binary or Intel HEX file.\n"
           "Model: CA51F253L3 (32 KiB code flash, 512-byte data flash, "
           "256-byte IDATA, 2 KiB XDATA).\n"
           "UART0/S0BUF and UART1/S1BUF output is printed after execution.\n";
}

std::size_t parse_count(const std::string &text) {
    std::size_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        value == 0)
        throw std::invalid_argument("invalid positive instruction count: " + text);
    return value;
}

} // namespace

int run_command(int argc, char **argv) {
    std::string firmware;
    std::string model = "CA51F253L3";
    std::size_t steps = 1'000'000;
    bool trace = false;
    bool state = false;

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
        else if (option == "--state")
            state = true;
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
    if (state) {
        std::cout << "core: SP=0x" << std::hex
                  << static_cast<unsigned>(device.read_sfr(0x81))
                  << " PSW=0x" << static_cast<unsigned>(device.read_sfr(0xd0))
                  << " ACC=0x" << static_cast<unsigned>(device.read_sfr(0xe0))
                  << " DPH:DPL=0x" << static_cast<unsigned>(device.read_sfr(0x83))
                  << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned>(device.read_sfr(0x82))
                  << std::dec << std::setfill(' ')
                  << " TCON=0x" << std::hex
                  << static_cast<unsigned>(device.read_sfr(0x88))
                  << " TMOD=0x" << static_cast<unsigned>(device.read_sfr(0x89))
                  << " TH0:TL0=0x" << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned>(device.read_sfr(0x8c))
                  << std::setw(2)
                  << static_cast<unsigned>(device.read_sfr(0x8a))
                  << " IE=0x" << static_cast<unsigned>(device.read_sfr(0xa8))
                  << std::dec << std::setfill(' ') << "\nIRAM 0x00..0x0f:";
        for (std::uint8_t address = 0x00; address <= 0x0f; ++address)
            std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<unsigned>(device.read_idata(address));
        std::cout << "\nIRAM 0x30..0x3f:";
        for (std::uint8_t address = 0x30; address <= 0x3f; ++address)
            std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<unsigned>(device.read_idata(address));
        const unsigned sp = device.read_sfr(0x81);
        std::cout << "\nstack:";
        const unsigned first = sp > 15 ? sp - 15 : 0;
        for (unsigned address = first; address <= sp; ++address)
            std::cout << " 0x" << std::hex << address << '=' << std::setw(2)
                      << std::setfill('0')
                      << static_cast<unsigned>(device.read_idata(
                             static_cast<std::uint8_t>(address)));
        std::cout << std::dec << std::setfill(' ') << '\n';
    }
    if (!device.uart0_output().empty()) {
        std::cout << "UART0 output (" << device.uart0_output().size()
                  << " bytes):\n";
        std::cout.write(device.uart0_output().data(),
                        static_cast<std::streamsize>(device.uart0_output().size()));
        if (device.uart0_output().back() != '\n')
            std::cout << '\n';
    }
    if (!device.uart1_output().empty()) {
        std::cout << "UART1 output (" << device.uart1_output().size()
                  << " bytes):\n";
        std::cout.write(device.uart1_output().data(),
                        static_cast<std::streamsize>(device.uart1_output().size()));
        if (device.uart1_output().back() != '\n')
            std::cout << '\n';
    }
    return 0;
}

} // namespace mcs51emu
