#include "mcs51emu/cc2530_hit_report.hpp"

#include <iomanip>

namespace mcs51emu {

namespace {

void print_hex_line(std::ostream &out, const char *label, unsigned address,
                    const std::vector<std::uint8_t> &bytes) {
    out << label << " 0x" << std::hex << std::setw(4) << std::setfill('0') << address
        << ':';
    for (const std::uint8_t byte : bytes)
        out << ' ' << std::setw(2) << static_cast<unsigned>(byte);
    out << std::dec << std::setfill(' ') << '\n';
}

} // namespace

void write_cc2530_memory(std::ostream &out, const Cc2530F256 &device, const Cc2530HitDump &dump) {
    if (dump.state) {
        for (unsigned row = 0; row < 0x100; row += 16) {
            std::vector<std::uint8_t> bytes;
            for (unsigned i = 0; i < 16; ++i)
                bytes.push_back(device.read_idata(static_cast<std::uint8_t>(row + i)));
            print_hex_line(out, "idata", row, bytes);
        }
        for (unsigned row = 0x80; row < 0x100; row += 16) {
            std::vector<std::uint8_t> bytes;
            for (unsigned i = 0; i < 16; ++i)
                bytes.push_back(device.read_sfr(static_cast<std::uint8_t>(row + i)));
            print_hex_line(out, "sfr", row, bytes);
        }
    }
    for (const auto &[start, length] : dump.xdata) {
        for (std::uint32_t row = 0; row < length; row += 16) {
            std::vector<std::uint8_t> bytes;
            for (std::uint32_t i = row; i < length && i < row + 16; ++i)
                bytes.push_back(device.peek_xdata(static_cast<std::uint16_t>(start + i)));
            print_hex_line(out, "xdata", start + row, bytes);
        }
    }
}

void write_cc2530_hit(std::ostream &out, const Cc2530F256 &device, std::uint64_t hit,
                      std::uint64_t instructions, const Cc2530HitDump &dump) {
    const Cc2530Registers regs = device.registers();
    out << "hit: " << hit << " instructions=" << instructions << " cycles=" << regs.cycles
        << std::hex << std::setfill('0') << " pc=0x" << std::setw(4) << regs.pc
        << " sp=0x" << std::setw(2) << unsigned(regs.sp) << " dps=0x" << std::setw(2)
        << unsigned(regs.dps) << " fmap=0x" << std::setw(2) << unsigned(regs.fmap)
        << " irq-levels=0x" << std::setw(2) << unsigned(regs.interrupt_levels) << std::dec
        << std::setfill(' ') << '\n';
    write_cc2530_memory(out, device, dump);
    out << "end-hit: " << hit << '\n';
}

} // namespace mcs51emu
