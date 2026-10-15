#include "esp32emu/run_command.hpp"

#include "elf_symbols.hpp"
#include "esp32emu/esp32.hpp"

#include <unicorn/unicorn.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <memory>
#include <charconv>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace esp32emu {

namespace {

void usage(std::ostream &out) {
    out << "Usage: mcuemu run --model ESP32C6|ESP32H2 --firmware IMAGE [--steps N] [--for DURATION]\n"
           "       [--restart] [--trace-mmio [--trace-limit N]] [--trace-block BEGIN:END]... [--last-pcs] [--break ADDR]... [--watch BEGIN:END]... [--dump ADDR[:LEN]]... [--log-at ADDR]... [--elf FILE] [--stub NAME|0xADDR[=VALUE]]...\n"
           "\n"
           "IMAGE is the flash contents from offset 0 (an ESP image with the 0xE9 header, as\n"
           "the ROM loader finds it). Without --steps and --for the run lasts 1 s of virtual\n"
           "time. DURATION is a number with ns, us, ms or s. UART0 output goes to stdout;\n"
           "--trace-mmio logs every access to a register no block models to stderr.\n";
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
    Variant variant = Variant::C6;
    bool restart_on_reset = false;
    std::uint64_t steps = 0, duration_ns = 0;
    bool trace_mmio = false, last_pcs = false, trace_irq = false, trace_rom = false;
    std::uint64_t trace_limit = 200000;
    std::vector<std::uint32_t> breakpoints, logpoints;
    std::string elf_path;
    std::vector<std::string> stubs, logpoint_specs, breakpoint_specs;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> dumps;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> watches, trace_blocks;
    for (int i = 2; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--firmware" && i + 1 < argc)
            firmware = argv[++i];
        else if (option == "--model" && i + 1 < argc) {
            std::string model = argv[++i];
            for (char &c : model)
                c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            if (model == "ESP32H2")
                variant = Variant::H2;
            else if (model != "ESP32C6")
                throw std::invalid_argument("unknown ESP32 model: " + model);
        }
        else if (option == "--steps" && i + 1 < argc)
            steps = parse_number(argv[++i], "instruction count");
        else if (option == "--for" && i + 1 < argc)
            duration_ns = parse_duration_ns(argv[++i]);
        else if (option == "--trace-mmio")
            trace_mmio = true;
        else if (option == "--trace-block" && i + 1 < argc) {
            const std::string range = argv[++i];
            const auto colon = range.find(':');
            if (colon == std::string::npos)
                throw std::invalid_argument("--trace-block needs BEGIN:END");
            trace_blocks.emplace_back(std::stoul(range.substr(0, colon), nullptr, 0),
                                      std::stoul(range.substr(colon + 1), nullptr, 0));
        } else if (option == "--trace-limit" && i + 1 < argc)
            trace_limit = parse_number(argv[++i], "trace limit");
        else if (option == "--break" && i + 1 < argc)
            breakpoint_specs.push_back(argv[++i]);
        else if (option == "--restart")
            restart_on_reset = true;
        else if (option == "--trace-rom")
            trace_rom = true;
        else if (option == "--trace-irq")
            trace_irq = true;
        else if (option == "--watch" && i + 1 < argc) {
            const std::string range = argv[++i];
            const auto colon = range.find(':');
            if (colon == std::string::npos)
                throw std::invalid_argument("--watch needs BEGIN:END");
            watches.emplace_back(std::stoul(range.substr(0, colon), nullptr, 0),
                                 std::stoul(range.substr(colon + 1), nullptr, 0));
        } else if (option == "--elf" && i + 1 < argc) {
            elf_path = argv[++i];
        } else if (option == "--stub" && i + 1 < argc) {
            stubs.push_back(argv[++i]); // NAME[=VALUE] or 0xADDRESS[=VALUE]
        } else if (option == "--log-at" && i + 1 < argc) {
            logpoint_specs.push_back(argv[++i]);
        } else if (option == "--dump" && i + 1 < argc) {
            const std::string range = argv[++i];
            const auto colon = range.find(':');
            dumps.emplace_back(std::stoul(range.substr(0, colon), nullptr, 0),
                               colon == std::string::npos ? 64 : std::stoul(range.substr(colon + 1), nullptr, 0));
        } else if (option == "--last-pcs")
            last_pcs = true;
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

    Esp32::Config config;
    config.variant = variant;
    Esp32 chip(config);
    chip.load_flash(image);
    chip.trace_mmio = trace_mmio;
    chip.trace_limit = trace_limit;
    chip.trace_blocks = trace_blocks;
    chip.trace_irq = trace_irq;
    chip.trace_rom = trace_rom;
    chip.on_uart_tx = [](std::uint8_t byte) { std::cout.put(static_cast<char>(byte)).flush(); };
    // A medium of its own, so frames the firmware sends are visible (hex on stderr).
    mcuemu::VirtualAir air;
    chip.connect_radio(air, "esp32");
    air.add_observer([](const mcuemu::RadioFrame &frame) {
        char head[96];
        std::snprintf(head, sizeof head, "[air] %llu ns ch%u %zu bytes:", static_cast<unsigned long long>(frame.start_ns),
                      frame.channel, frame.payload.size());
        std::string line = head;
        for (std::uint8_t byte : frame.payload) {
            char hex[4];
            std::snprintf(hex, sizeof hex, " %02x", byte);
            line += hex;
        }
        std::cerr << line << '\n';
    });
    chip.reset();
    std::unique_ptr<ElfSymbols> elf;
    if (!elf_path.empty())
        elf = std::make_unique<ElfSymbols>(elf_path);
    for (const std::string &spec : stubs) {
        const auto equals = spec.find('=');
        const std::string target = spec.substr(0, equals);
        const std::uint32_t value =
            equals == std::string::npos ? 0 : static_cast<std::uint32_t>(std::stoul(spec.substr(equals + 1), nullptr, 0));
        std::uint32_t address = 0;
        if (target.rfind("0x", 0) == 0) {
            address = static_cast<std::uint32_t>(std::stoul(target, nullptr, 0));
        } else {
            if (!elf)
                throw std::invalid_argument("--stub NAME needs --elf; use an 0x address without it");
            address = elf->address_of(target);
            if (address == 0)
                throw std::invalid_argument("symbol not found in the ELF: " + target);
        }
        chip.stub_function(address, value);
    }
    for (const auto &range : watches)
        chip.core().watch_writes(range.first, range.second,
                                 [&chip](std::uint32_t, std::uint32_t address, unsigned size,
                                         std::uint32_t value) {
                                     const auto pcs = chip.core().recent_pcs();
                                     std::cerr << "[watch] W" << size << " 0x" << std::hex << address
                                               << " <- 0x" << value << " after pc 0x"
                                               << (pcs.empty() ? 0 : pcs.back()) << std::dec << " at "
                                               << chip.time_ns() << " ns\n";
                                 });
    auto resolve = [&](const std::string &spec) {
        if (spec.rfind("0x", 0) == 0)
            return static_cast<std::uint32_t>(std::stoul(spec, nullptr, 0));
        if (!elf)
            throw std::invalid_argument("a symbol name needs --elf: " + spec);
        const std::uint32_t address = elf->address_of(spec);
        if (address == 0)
            throw std::invalid_argument("symbol not found in the ELF: " + spec);
        return address;
    };
    for (const std::string &spec : breakpoint_specs)
        breakpoints.push_back(resolve(spec));
    for (const std::string &spec : logpoint_specs)
        logpoints.push_back(resolve(spec));
    for (std::uint32_t address : breakpoints)
        chip.core().add_breakpoint(address);
    for (std::uint32_t address : logpoints)
        chip.core().add_breakpoint(address);
    if (steps != 0)
        chip.core().set_instruction_limit(steps);
    riscv::RunResult result;
    for (;;) {
        result = chip.run_until(duration_ns != 0 ? duration_ns : ~std::uint64_t{0} >> 8);
        if (result.reason == riscv::StopReason::Reset && restart_on_reset) {
            chip.reset(); // a software reset: the ROM loader starts the image again
            continue;
        }
        if (result.reason != riscv::StopReason::Halted)
            break;
        // --log-at: print the registers that matter for a calling convention and go on.
        if (std::find(logpoints.begin(), logpoints.end(), chip.core().pc()) != logpoints.end()) {
            riscv::Core &c = chip.core();
            std::cout << "[at] " << (elf ? elf->describe(c.pc()) : std::string()) << " pc=0x" << std::hex
                      << c.pc() << " ra=0x" << c.reg(1) << (elf ? " (" + elf->describe(c.reg(1)) + ")" : std::string()) << " sp=0x"
                      << c.reg(2) << " a0=0x" << c.reg(10) << " a1=0x" << c.reg(11) << " s0=0x"
                      << c.reg(8) << std::dec << " t=" << chip.time_ns() << "\n  [sp..+0x50]:";
            for (std::uint32_t offset = 0; offset < 0x50; offset += 4) {
                std::uint32_t word = 0;
                c.read_memory(c.reg(2) + offset, &word, 4);
                std::cout << ' ' << std::hex << word << std::dec;
            }
            {
                std::cout << "\n  [0x4081c278]:";
                for (std::uint32_t offset = 0; offset < 0x10; offset += 4) {
                    std::uint32_t word = 0;
                    c.read_memory(0x4081c278 + offset, &word, 4);
                    std::cout << ' ' << std::hex << word << std::dec;
                }
            }
            std::cout << "\n  last pcs:";
            const auto pcs = c.recent_pcs();
            for (std::size_t k = pcs.size() > 40 ? pcs.size() - 40 : 0; k < pcs.size(); ++k)
                std::cout << ' ' << std::hex << pcs[k] << std::dec;
            std::cout << "\n";
            continue;
        }
        if (breakpoints.empty())
            break;
        // A breakpoint on the trap vector also catches interrupts: only an
        // exception (mcause bit 31 clear) stops the run.
        std::uint32_t mcause = 0;
        uc_reg_read(chip.core().engine(), UC_RISCV_REG_MCAUSE, &mcause);
        if (chip.core().pc() != breakpoints.front() || !(mcause >> 31))
            break;
    }

    riscv::Core &core = chip.core();
    std::cout << "\nmodel: " << chip.layout().name << "\n"
              << "flash image: " << image.size() << " bytes\n"
              << "instructions: " << core.instructions() << '\n'
              << "cycles: " << core.cycles() << '\n'
              << "virtual time: " << chip.time_ns() << " ns\n"
              << "pc: 0x" << std::hex << core.pc() << std::dec << '\n';
    static const char *const names[] = {"zero", "ra", "sp", "gp", "tp",  "t0",  "t1", "t2", "s0", "s1", "a0",
                                        "a1",   "a2", "a3", "a4", "a5",  "a6",  "a7", "s2", "s3", "s4", "s5",
                                        "s6",   "s7", "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"};
    for (unsigned reg = 0; reg != 32; ++reg)
        std::cout << names[reg] << "=0x" << std::hex << core.reg(reg) << std::dec
                  << ((reg & 3u) == 3u ? '\n' : ' ');
    {
        std::uint32_t mcause = 0, mepc = 0, mtval = 0, mstatus = 0, mtvec = 0;
        uc_reg_read(core.engine(), UC_RISCV_REG_MCAUSE, &mcause);
        uc_reg_read(core.engine(), UC_RISCV_REG_MEPC, &mepc);
        uc_reg_read(core.engine(), UC_RISCV_REG_MTVAL, &mtval);
        uc_reg_read(core.engine(), UC_RISCV_REG_MSTATUS, &mstatus);
        uc_reg_read(core.engine(), UC_RISCV_REG_MTVEC, &mtvec);
        std::cout << std::hex << "mcause=0x" << mcause << " mepc=0x" << mepc << " mtval=0x" << mtval
                  << " mstatus=0x" << mstatus << " mtvec=0x" << mtvec << std::dec << '\n';
    }
    for (const auto &range : dumps) {
        std::cout << "memory 0x" << std::hex << range.first << ':';
        for (std::uint32_t offset = 0; offset < range.second; offset += 4) {
            std::uint32_t word = 0;
            core.read_memory(range.first + offset, &word, 4);
            std::cout << ((offset & 15u) == 0 ? "\n  " : " ") << word;
        }
        std::cout << std::dec << '\n';
    }
    if (last_pcs) {
        std::uint32_t value = 0;
        std::cout << std::hex << "pmpcfg0-3:";
        for (unsigned csr = 0x3a0; csr != 0x3a4; ++csr) {
            uc_reg_read(core.engine(), UC_RISCV_REG_CSR(csr), &value);
            std::cout << ' ' << value;
        }
        std::cout << "\npmpaddr0-15:";
        for (unsigned csr = 0x3b0; csr != 0x3c0; ++csr) {
            uc_reg_read(core.engine(), UC_RISCV_REG_CSR(csr), &value);
            std::cout << ' ' << value;
        }
        std::cout << std::dec << '\n';
        std::cout << "last instructions:";
        const auto pcs = core.recent_pcs();
        for (std::size_t i = pcs.size() > 64 ? pcs.size() - 64 : 0; i < pcs.size(); ++i)
            std::cout << ' ' << std::hex << pcs[i];
        std::cout << std::dec << '\n';
        if (elf && !pcs.empty()) {
            std::cout << "in:";
            std::string last;
            for (std::size_t i = pcs.size() > 64 ? pcs.size() - 64 : 0; i < pcs.size(); ++i) {
                std::string name = elf->describe(pcs[i]);
                name = name.substr(0, name.find('+'));
                if (name != last)
                    std::cout << ' ' << name;
                last = name;
            }
            std::cout << '\n';
        }
    }
    switch (result.reason) {
    case riscv::StopReason::Fault:
        std::cerr << "CPU fault: " << result.message << "\nlast instructions:";
        for (std::uint32_t pc : core.recent_pcs())
            std::cerr << ' ' << std::hex << pc;
        std::cerr << std::dec << '\n';
        return 1;
    case riscv::StopReason::Halted:
    case riscv::StopReason::Reset:
        std::cout << "stop: " << result.message << '\n';
        break;
    case riscv::StopReason::StepLimit:
        std::cout << "stop: instruction limit\n";
        break;
    default:
        std::cout << (core.sleeping() ? "stop: time limit (core asleep)\n" : "stop: time limit\n");
        break;
    }
    return 0;
}

} // namespace esp32emu
