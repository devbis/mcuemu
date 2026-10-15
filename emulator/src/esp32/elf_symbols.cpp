#include "elf_symbols.hpp"

#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace esp32emu {

namespace {
template <typename T> T load(const std::vector<std::uint8_t> &data, std::size_t offset) {
    T value{};
    if (offset + sizeof(T) > data.size())
        throw std::runtime_error("truncated ELF file");
    std::memcpy(&value, data.data() + offset, sizeof(T));
    return value;
}
} // namespace

ElfSymbols::ElfSymbols(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot open ELF file: " + path);
    const std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(input)),
                                         std::istreambuf_iterator<char>());
    if (data.size() < 52 || std::memcmp(data.data(), "\x7f" "ELF", 4) != 0 || data[4] != 1 || data[5] != 1)
        throw std::runtime_error(path + " is not a 32-bit little-endian ELF file");
    const auto section_offset = load<std::uint32_t>(data, 32);
    const auto section_size = load<std::uint16_t>(data, 46);
    const auto section_count = load<std::uint16_t>(data, 48);
    for (unsigned i = 0; i < section_count; ++i) {
        const std::size_t header = section_offset + static_cast<std::size_t>(i) * section_size;
        if (load<std::uint32_t>(data, header + 4) != 2) // SHT_SYMTAB
            continue;
        const auto symbols = load<std::uint32_t>(data, header + 16);
        const auto bytes = load<std::uint32_t>(data, header + 20);
        const auto strtab_index = load<std::uint32_t>(data, header + 24);
        const auto entry_size = load<std::uint32_t>(data, header + 36);
        const std::size_t strtab_header = section_offset + static_cast<std::size_t>(strtab_index) * section_size;
        const auto strings = load<std::uint32_t>(data, strtab_header + 16);
        for (std::uint32_t at = 0; entry_size != 0 && at + entry_size <= bytes; at += entry_size) {
            const auto name_offset = load<std::uint32_t>(data, symbols + at);
            const auto value = load<std::uint32_t>(data, symbols + at + 4);
            const auto info = load<std::uint8_t>(data, symbols + at + 12);
            const unsigned type = info & 0xf;
            if (name_offset == 0 || (type > 2) || value == 0) // objects, functions and assembly labels
                continue;
            const char *name = reinterpret_cast<const char *>(data.data() + strings + name_offset);
            if (name[0] == '$' || name[0] == '.') // RISC-V mapping symbols and local labels
                continue;
            by_name_.emplace(name, value & ~1u);
            by_address_.emplace(value & ~1u, name);
        }
    }
}

std::uint32_t ElfSymbols::address_of(const std::string &name) const {
    auto it = by_name_.find(name);
    return it == by_name_.end() ? 0 : it->second;
}

std::string ElfSymbols::describe(std::uint32_t address) const {
    auto it = by_address_.upper_bound(address);
    if (it == by_address_.begin())
        return {};
    --it;
    if (address - it->first >= 0x100000)
        return {};
    char buffer[24];
    std::snprintf(buffer, sizeof buffer, "+0x%x", address - it->first);
    return it->second + buffer;
}

} // namespace esp32emu
