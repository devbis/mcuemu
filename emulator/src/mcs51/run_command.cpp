#include "mcs51emu/run_command.hpp"

#include "mcs51emu/ca51f253l3.hpp"
#include "mcs51emu/cc2530_hit_report.hpp"
#include "mcs51emu/cc2530f256.hpp"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mcs51emu {

namespace {

void usage(std::ostream &out) {
    out << "Usage: mcuemu run --model CA51F253L3 --firmware IMAGE "
           "[--steps N] [--trace] [--state]\n"
           "       mcuemu run --model CC2530F256 --firmware IMAGE [--steps N]\n"
           "       [--trace] [--state] [--until-pc ADDR [--hits N] [--report-hits]\n"
           "       [--write-xdata HIT:ADDR:HEX]...] [--dump-xdata ADDR:LEN]...\n"
           "       [--ieee-address HEX]\n"
           "\n"
           "IMAGE may be a raw binary or Intel HEX file.\n"
           "Model: CA51F253L3 (32 KiB code flash, 512-byte data flash, "
           "256-byte IDATA, 2 KiB XDATA).\n"
           "UART0/S0BUF and UART1/S1BUF output is printed after execution.\n"
           "Model: CC2530F256 (256 KiB flash in eight FMAP banks, 8 KiB SRAM).\n"
           "Intel HEX addresses are physical flash addresses (bank N at N*0x8000);\n"
           "convert ELF files with 'llvm-objcopy -O ihex'. The run stops at the\n"
           "step limit, a fault, PCON power mode, --until-pc (before executing it)\n"
           "or an idle self-loop with no deliverable interrupt. --hits N stops at the\n"
           "N-th arrival at --until-pc instead of the first; --report-hits prints a\n"
           "'hit:' block (with the --dump-xdata regions and, with --state, IDATA and\n"
           "SFRs) at every arrival. --write-xdata stores HEX bytes to XDATA SRAM at\n"
           "ADDR on arrival HIT, after its report, as a halted debugger would.\n";
}

std::size_t parse_count(const std::string &text) {
    std::size_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        value == 0)
        throw std::invalid_argument("invalid positive instruction count: " + text);
    return value;
}

std::uint64_t parse_number(const std::string &text, std::uint64_t max, const char *what) {
    std::uint64_t value = 0;
    const bool is_hex = text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X');
    const char *first = text.data() + (is_hex ? 2 : 0);
    const auto parsed =
        std::from_chars(first, text.data() + text.size(), value, is_hex ? 16 : 10);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value > max)
        throw std::invalid_argument(std::string("invalid ") + what + ": " + text);
    return value;
}

struct Cc2530Options {
    std::string firmware;
    std::size_t steps{};
    bool trace{};
    bool state{};
    std::optional<std::uint16_t> until_pc;
    std::size_t hits{1};
    bool report_hits{};
    struct XdataWrite {
        std::size_t hit;
        std::uint16_t address;
        std::vector<std::uint8_t> bytes;
    };
    std::vector<XdataWrite> write_xdata;
    std::vector<std::pair<std::uint16_t, std::uint32_t>> dump_xdata;
    std::optional<std::uint64_t> ieee_address;
};

mcs51emu::Cc2530HitDump hit_dump(const Cc2530Options &options) {
    return {options.state, options.dump_xdata};
}

void print_memory(std::ostream &out, const mcs51emu::Cc2530F256 &device,
                  const Cc2530Options &options) {
    mcs51emu::write_cc2530_memory(out, device, hit_dump(options));
}

void print_hit(std::ostream &out, const mcs51emu::Cc2530F256 &device,
               const Cc2530Options &options, std::size_t hit, std::size_t executed) {
    mcs51emu::write_cc2530_hit(out, device, hit, executed, hit_dump(options));
}

int run_cc2530(const Cc2530Options &options) {
    mcs51emu::Cc2530F256 device;
    device.load_firmware(options.firmware);
    if (options.ieee_address)
        device.set_ieee_address(*options.ieee_address);
    if (options.trace)
        device.set_trace(&std::cerr);
    device.set_breakpoint(options.until_pc);

    // Run in slices so an idle self-loop (the usual end of a test image)
    // stops the run instead of spinning to the step limit.
    constexpr std::size_t Slice = 100000;
    std::size_t executed = 0;
    std::size_t hits = 0;
    mcs51emu::Cc2530RunResult result;
    std::string stop = "instruction limit";
    while (executed < options.steps) {
        result = device.run(std::min(Slice, options.steps - executed));
        executed += result.instructions;
        if (!result.error.empty()) {
            stop = "fault";
            break;
        }
        if (result.break_hit) {
            ++hits;
            if (options.report_hits)
                print_hit(std::cout, device, options, hits, executed);
            for (const Cc2530Options::XdataWrite &write : options.write_xdata)
                if (write.hit == hits)
                    for (std::size_t i = 0; i < write.bytes.size(); ++i)
                        device.write_sram(static_cast<std::uint16_t>(write.address + i),
                                          write.bytes[i]);
            if (hits < options.hits)
                continue; // the next run() executes the breakpoint instruction
            stop = "until-pc";
            break;
        }
        if (result.sleeping) {
            stop = "PCON power mode (no wake source modeled)";
            break;
        }
        if (device.idle_forever()) {
            stop = "idle self-loop";
            break;
        }
    }
    const mcs51emu::Cc2530Registers regs = device.registers();
    std::cout << "model: CC2530F256\n"
              << "instructions: " << executed << '\n'
              << "cycles: " << regs.cycles << '\n'
              << "pc: 0x" << std::hex << std::setw(4) << std::setfill('0') << regs.pc
              << std::dec << std::setfill(' ') << '\n'
              << "fmap: " << unsigned(regs.fmap) << '\n';
    if (const auto physical = device.code_physical(regs.pc))
        std::cout << "pc-physical: 0x" << std::hex << std::setw(5) << std::setfill('0')
                  << *physical << std::dec << std::setfill(' ') << '\n';
    else
        std::cout << "pc-physical: SRAM (XMAP)\n";
    std::cout << "stop: " << stop << '\n';
    if (options.until_pc)
        std::cout << "until-pc-hits: " << hits << '\n';
    const mcs51emu::Cc2530Diagnostics &diag = device.diagnostics();
    for (const std::string &warning : diag.load_warnings)
        std::cout << "load warning: " << warning << '\n';
    std::cout << "interrupts-taken: " << diag.interrupts_taken << '\n'
              << "fmap-writes: " << diag.fmap_writes << '\n'
              << "unmapped-xdata: reads=" << diag.unmapped_xdata_reads
              << " writes=" << diag.unmapped_xdata_writes << '\n'
              << "read-only-writes: xdata=" << diag.read_only_xdata_writes
              << " sfr=" << diag.read_only_sfr_writes << '\n';
    for (const auto &[address, count] : diag.unmodeled_xreg_reads)
        std::cout << "unmodeled-xreg-read: 0x" << std::hex << address << std::dec
                  << " x" << count << '\n';
    for (const auto &[address, count] : diag.unmodeled_xreg_writes)
        std::cout << "unmodeled-xreg-write: 0x" << std::hex << address << std::dec
                  << " x" << count << '\n';
    if (options.state) {
        std::cout << "core: A=0x" << std::hex << unsigned(regs.a) << " B=0x"
                  << unsigned(regs.b) << " PSW=0x" << unsigned(regs.psw) << " SP=0x"
                  << unsigned(regs.sp) << " DPTR=0x" << std::setw(2) << std::setfill('0')
                  << unsigned(regs.dph) << std::setw(2) << unsigned(regs.dpl)
                  << " MEMCTR=0x" << unsigned(regs.memctr) << " MPAGE=0x"
                  << unsigned(regs.mpage) << " R0-R7:";
        for (const std::uint8_t r : regs.r)
            std::cout << ' ' << std::setw(2) << unsigned(r);
        std::cout << std::dec << std::setfill(' ') << '\n';
        std::cout << "cpu-ext: DPTR1=0x" << std::hex << std::setw(2) << std::setfill('0')
                  << unsigned(regs.dph1) << std::setw(2) << unsigned(regs.dpl1)
                  << " DPS=0x" << std::setw(2) << unsigned(regs.dps)
                  << " IRQ-LEVELS=0x" << std::setw(2) << unsigned(regs.interrupt_levels)
                  << " IRQ-INHIBIT=" << unsigned(regs.interrupt_inhibit)
                  << " POSTED=0x" << std::setw(4) << unsigned(regs.posted_vector)
                  << std::dec << std::setfill(' ') << '\n';
    }
    print_memory(std::cout, device, options);
    if (!result.error.empty()) {
        std::cerr << "CPU fault: " << result.error << '\n';
        return 1;
    }
    return 0;
}

} // namespace

int run_command(int argc, char **argv) {
    std::string firmware;
    std::string model = "CA51F253L3";
    std::size_t steps = 1'000'000;
    bool trace = false;
    bool state = false;
    Cc2530Options cc2530;
    bool cc2530_only = false;

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
        else if (option == "--until-pc" && i + 1 < argc) {
            cc2530.until_pc =
                static_cast<std::uint16_t>(parse_number(argv[++i], 0xffff, "--until-pc"));
            cc2530_only = true;
        } else if (option == "--hits" && i + 1 < argc) {
            cc2530.hits = parse_count(argv[++i]);
            cc2530_only = true;
        } else if (option == "--report-hits") {
            cc2530.report_hits = true;
            cc2530_only = true;
        } else if (option == "--write-xdata" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const auto first = spec.find(':');
            const auto second = first == std::string::npos ? first : spec.find(':', first + 1);
            if (second == std::string::npos)
                throw std::invalid_argument("--write-xdata expects HIT:ADDR:HEX");
            Cc2530Options::XdataWrite write;
            write.hit = parse_count(spec.substr(0, first));
            const auto address = parse_number(spec.substr(first + 1, second - first - 1),
                                              mcs51emu::Cc2530F256::SramSize - 1,
                                              "--write-xdata address");
            const std::string hex = spec.substr(second + 1);
            if (hex.empty() || hex.size() % 2 != 0 ||
                hex.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos)
                throw std::invalid_argument("--write-xdata expects an even number of hex digits");
            for (std::size_t k = 0; k < hex.size(); k += 2)
                write.bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(k, 2), nullptr, 16)));
            if (address + write.bytes.size() > mcs51emu::Cc2530F256::SramSize)
                throw std::invalid_argument("--write-xdata range leaves XDATA SRAM 0x0000-0x1fff");
            write.address = static_cast<std::uint16_t>(address);
            cc2530.write_xdata.push_back(std::move(write));
            cc2530_only = true;
        } else if (option == "--dump-xdata" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const auto colon = spec.find(':');
            if (colon == std::string::npos)
                throw std::invalid_argument("--dump-xdata expects ADDR:LEN");
            const auto start = parse_number(spec.substr(0, colon), 0xffff, "XDATA address");
            const auto length =
                parse_number(spec.substr(colon + 1), 0x10000 - start, "XDATA length");
            cc2530.dump_xdata.emplace_back(static_cast<std::uint16_t>(start),
                                           static_cast<std::uint32_t>(length));
            cc2530_only = true;
        } else if (option == "--ieee-address" && i + 1 < argc) {
            std::string text = argv[++i];
            if (text.rfind("0x", 0) != 0 && text.rfind("0X", 0) != 0)
                text = "0x" + text;
            cc2530.ieee_address = parse_number(text, UINT64_MAX, "--ieee-address");
            cc2530_only = true;
        }
        else if (option == "--help" || option == "-h") {
            usage(std::cout);
            return 0;
        } else {
            throw std::invalid_argument("unknown or incomplete option: " + option);
        }
    }

    if (model != "CA51F253L3" && model != "CC2530F256")
        throw std::invalid_argument("unsupported MCS-51 model: " + model);
    if (firmware.empty())
        throw std::invalid_argument("--firmware is required");
    if (model == "CC2530F256") {
        if ((cc2530.hits != 1 || cc2530.report_hits || !cc2530.write_xdata.empty()) &&
            !cc2530.until_pc)
            throw std::invalid_argument("--hits, --report-hits and --write-xdata need --until-pc");
        for (const Cc2530Options::XdataWrite &write : cc2530.write_xdata)
            if (write.hit > cc2530.hits)
                throw std::invalid_argument("--write-xdata hit " + std::to_string(write.hit) +
                                            " is after the last arrival (--hits " +
                                            std::to_string(cc2530.hits) + ")");
        cc2530.firmware = firmware;
        cc2530.steps = steps;
        cc2530.trace = trace;
        cc2530.state = state;
        return run_cc2530(cc2530);
    }
    if (cc2530_only)
        throw std::invalid_argument(
            "--until-pc, --hits, --report-hits, --write-xdata, --dump-xdata and "
            "--ieee-address need --model CC2530F256");

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
