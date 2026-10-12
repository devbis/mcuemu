#include "mcs51emu/ca51f253l3.hpp"

#include "unicorn/mcs51.h"
#include "unicorn/unicorn.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace mcs51emu {
namespace {

constexpr int SfrStackPointer = 0x81;
constexpr int SfrPort0 = 0x80;
constexpr int SfrPort1 = 0x90;
constexpr int SfrPort2 = 0xa0;
constexpr int SfrPort3 = 0xb0;

std::uint8_t hex_byte(std::string_view text, std::size_t offset,
                      const std::string &path) {
    if (offset + 2 > text.size())
        throw std::runtime_error(path + ": truncated Intel HEX record");
    unsigned value = 0;
    for (std::size_t i = offset; i < offset + 2; ++i) {
        const unsigned char ch = static_cast<unsigned char>(text[i]);
        const int digit = std::isdigit(ch) ? ch - '0'
                          : std::toupper(ch) >= 'A' && std::toupper(ch) <= 'F'
                                ? std::toupper(ch) - 'A' + 10
                                : -1;
        if (digit < 0)
            throw std::runtime_error(path + ": invalid Intel HEX digit");
        value = (value << 4) | static_cast<unsigned>(digit);
    }
    return static_cast<std::uint8_t>(value);
}

std::vector<std::uint8_t> load_intel_hex(const std::string &path) {
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("cannot open firmware image: " + path);

    std::vector<std::uint8_t> flash(Ca51F253L3::FlashSize, 0xff);
    std::string line;
    std::uint32_t upper_address = 0;
    bool eof_seen = false;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty())
            continue;
        if (eof_seen)
            throw std::runtime_error(path + ": data follows Intel HEX EOF record");
        if (line[0] != ':' || line.size() < 11 || (line.size() & 1u) == 0)
            throw std::runtime_error(path + ": malformed Intel HEX record at line " +
                                     std::to_string(line_number));

        const std::string_view record(line.data() + 1, line.size() - 1);
        const std::uint8_t count = hex_byte(record, 0, path);
        const std::uint8_t address_hi = hex_byte(record, 2, path);
        const std::uint8_t address_lo = hex_byte(record, 4, path);
        const std::uint8_t type = hex_byte(record, 6, path);
        if (record.size() != 10 + static_cast<std::size_t>(count) * 2)
            throw std::runtime_error(path + ": Intel HEX byte count mismatch at line " +
                                     std::to_string(line_number));

        unsigned sum = count + address_hi + address_lo + type;
        std::vector<std::uint8_t> data;
        data.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            data.push_back(hex_byte(record, 8 + i * 2, path));
            sum += data.back();
        }
        const std::uint8_t checksum = hex_byte(record, 8 + count * 2, path);
        sum += checksum;
        if ((sum & 0xffu) != 0)
            throw std::runtime_error(path + ": Intel HEX checksum mismatch at line " +
                                     std::to_string(line_number));

        const std::uint16_t offset = static_cast<std::uint16_t>(
            (static_cast<unsigned>(address_hi) << 8) | address_lo);
        switch (type) {
        case 0x00: {
            const std::uint64_t absolute = upper_address + offset;
            if (absolute + count > flash.size())
                throw std::runtime_error(path +
                                         ": firmware exceeds CA51F253L3 32 KiB flash");
            std::copy(data.begin(), data.end(), flash.begin() + absolute);
            break;
        }
        case 0x01:
            if (count != 0 || offset != 0)
                throw std::runtime_error(path + ": malformed Intel HEX EOF record");
            eof_seen = true;
            break;
        case 0x02:
            if (count != 2 || offset != 0)
                throw std::runtime_error(path + ": malformed segment address record");
            upper_address = static_cast<std::uint32_t>(
                ((static_cast<unsigned>(data[0]) << 8) | data[1]) << 4);
            break;
        case 0x04:
            if (count != 2 || offset != 0)
                throw std::runtime_error(path + ": malformed linear address record");
            upper_address = static_cast<std::uint32_t>(
                ((static_cast<unsigned>(data[0]) << 8) | data[1]) << 16);
            break;
        case 0x03: case 0x05:
            // Start-address records are metadata; reset entry is address zero
            // for the initial CA51F253L3 model.
            break;
        default:
            throw std::runtime_error(path + ": unsupported Intel HEX record type " +
                                     std::to_string(type));
        }
    }
    if (!eof_seen)
        throw std::runtime_error(path + ": missing Intel HEX EOF record");
    return flash;
}

std::vector<std::uint8_t> load_firmware_file(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot open firmware image: " + path);
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)),
                                    std::istreambuf_iterator<char>());
    if (bytes.empty())
        throw std::runtime_error("firmware image is empty: " + path);

    const auto first = std::find_if_not(bytes.begin(), bytes.end(), [](std::uint8_t ch) {
        return std::isspace(static_cast<unsigned char>(ch));
    });
    if (first != bytes.end() && *first == ':')
        return load_intel_hex(path);
    if (bytes.size() > Ca51F253L3::FlashSize)
        throw std::runtime_error("firmware exceeds CA51F253L3 32 KiB flash");
    bytes.resize(Ca51F253L3::FlashSize, 0xff);
    return bytes;
}

} // namespace

Ca51F253L3::Ca51F253L3() {
    uc_engine *engine = nullptr;
    const uc_err open_error = uc_open(UC_ARCH_MCS51, UC_MODE_LITTLE_ENDIAN, &engine);
    if (open_error != UC_ERR_OK)
        throw std::runtime_error(std::string("uc_open(MCS-51): ") +
                                 uc_strerror(open_error));
    uc_ = reinterpret_cast<uc_struct *>(engine);
    try {
        check_error(uc_mem_map(engine, 0, CodeSize,
                               UC_PROT_READ | UC_PROT_EXEC),
                    "map code space");
        std::vector<std::uint8_t> erased_code(CodeSize, 0xff);
        check_error(uc_mem_write(engine, 0, erased_code.data(), erased_code.size()),
                    "initialize erased flash");
        check_error(uc_mem_map(engine, XdataBase, XdataSize, UC_PROT_ALL),
                    "map XDATA");
        check_error(uc_mem_map(engine, IdataBase, MappedInternalSize, UC_PROT_ALL),
                    "map IDATA");
        check_error(uc_mem_map(engine, SfrBase, MappedInternalSize, UC_PROT_ALL),
                    "map SFR");
        check_error(uc_hook_add(engine, reinterpret_cast<uc_hook *>(&code_hook_),
                                UC_HOOK_CODE,
                                reinterpret_cast<void *>(
                                    static_cast<uc_cb_hookcode_t>(Ca51F253L3::code_hook)),
                                this, 1, 0),
                    "install code hook");
        reset();
    } catch (...) {
        uc_close(engine);
        uc_ = nullptr;
        throw;
    }
}

Ca51F253L3::~Ca51F253L3() {
    if (uc_)
        uc_close(reinterpret_cast<uc_engine *>(uc_));
}

void Ca51F253L3::check_error(int error, const char *operation) const {
    const auto result = static_cast<uc_err>(error);
    if (result != UC_ERR_OK)
        throw std::runtime_error(std::string("Unicorn ") + operation + ": " +
                                 uc_strerror(result));
}

void Ca51F253L3::load_firmware(const std::string &path) {
    const std::vector<std::uint8_t> image = load_firmware_file(path);
    check_error(uc_mem_write(reinterpret_cast<uc_engine *>(uc_), 0,
                             image.data(), image.size()),
                "load firmware");
    reset();
}

void Ca51F253L3::reset() {
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    std::array<std::uint8_t, MappedInternalSize> zeros{};
    check_error(uc_mem_write(engine, IdataBase, zeros.data(), zeros.size()),
                "reset IDATA");
    check_error(uc_mem_write(engine, SfrBase, zeros.data(), zeros.size()),
                "reset SFR");

    // 8051-compatible reset values. CA51-specific SFR reset behavior and
    // peripherals are added as their register documentation is modeled.
    write_sfr(SfrStackPointer, 0x07);
    write_sfr(SfrPort0, 0xff);
    write_sfr(SfrPort1, 0xff);
    write_sfr(SfrPort2, 0xff);
    write_sfr(SfrPort3, 0xff);
    std::uint16_t pc = 0;
    std::uint64_t cycles = 0;
    check_error(uc_reg_write(engine, UC_MCS51_REG_PC, &pc), "reset PC");
    check_error(uc_reg_write(engine, UC_MCS51_REG_CYCLES, &cycles),
                "reset cycle counter");
    executed_instructions_ = 0;
}

void Ca51F253L3::code_hook(uc_struct *opaque_uc, std::uint64_t address,
                           std::uint32_t size, void *user_data) {
    auto *self = static_cast<Ca51F253L3 *>(user_data);
    ++self->executed_instructions_;
    if (!self->trace_output_)
        return;

    std::array<std::uint8_t, 3> bytes{};
    auto *engine = reinterpret_cast<uc_engine *>(opaque_uc);
    const std::size_t read_size = std::min<std::size_t>(size, bytes.size());
    if (uc_mem_read(engine, address, bytes.data(), read_size) != UC_ERR_OK)
        return;
    auto &out = *self->trace_output_;
    out << "pc=0x" << std::hex << std::setw(4) << std::setfill('0') << address
        << "  ";
    for (std::size_t i = 0; i < read_size; ++i)
        out << std::setw(2) << static_cast<unsigned>(bytes[i]) << ' ';
    out << std::dec << std::setfill(' ') << '\n';
}

RunResult Ca51F253L3::run(std::size_t instruction_limit) {
    RunResult result;
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    const std::uint16_t start_pc = [&] {
        std::uint16_t value = 0;
        check_error(uc_reg_read(engine, UC_MCS51_REG_PC, &value), "read PC");
        return value;
    }();
    const std::size_t before = executed_instructions_;
    const uc_err error = uc_emu_start(engine, start_pc, CodeSize, 0,
                                      instruction_limit);
    result.instructions = executed_instructions_ - before;
    check_error(uc_reg_read(engine, UC_MCS51_REG_PC, &result.pc), "read PC");
    check_error(uc_reg_read(engine, UC_MCS51_REG_CYCLES, &result.cycles),
                "read cycle counter");
    result.step_limit_reached = error == UC_ERR_OK &&
                                result.instructions >= instruction_limit;
    if (error != UC_ERR_OK)
        result.error = uc_strerror(error);
    return result;
}

std::uint8_t Ca51F253L3::read_sfr(std::uint8_t address) const {
    std::uint8_t value = 0;
    const uc_err error = uc_mem_read(reinterpret_cast<uc_engine *>(uc_),
                                     SfrBase + address, &value, sizeof(value));
    if (error != UC_ERR_OK)
        throw std::runtime_error(std::string("Unicorn read SFR: ") +
                                 uc_strerror(error));
    return value;
}

std::uint8_t Ca51F253L3::read_idata(std::uint8_t address) const {
    std::uint8_t value = 0;
    const uc_err error = uc_mem_read(reinterpret_cast<uc_engine *>(uc_),
                                     IdataBase + address, &value, sizeof(value));
    if (error != UC_ERR_OK)
        throw std::runtime_error(std::string("Unicorn read IDATA: ") +
                                 uc_strerror(error));
    return value;
}

std::uint8_t Ca51F253L3::read_xdata(std::uint16_t address) const {
    std::uint8_t value = 0;
    const uc_err error = uc_mem_read(reinterpret_cast<uc_engine *>(uc_),
                                     XdataBase + address, &value, sizeof(value));
    if (error != UC_ERR_OK)
        throw std::runtime_error(std::string("Unicorn read XDATA: ") +
                                 uc_strerror(error));
    return value;
}

void Ca51F253L3::write_sfr(std::uint8_t address, std::uint8_t value) {
    check_error(uc_mem_write(reinterpret_cast<uc_engine *>(uc_), SfrBase + address,
                             &value, sizeof(value)),
                "write SFR");
}

} // namespace mcs51emu
