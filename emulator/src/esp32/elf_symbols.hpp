#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace esp32emu {

// Function and object symbols of an ELF32 little-endian image (the build's
// zephyr.elf / app.elf), for naming addresses in traces and for stubbing a
// firmware function by name. Reads .symtab only; throws std::runtime_error on
// anything that is not a 32-bit little-endian ELF with a symbol table.
class ElfSymbols {
public:
    explicit ElfSymbols(const std::string &path);

    // Address of `name`, 0 when absent.
    std::uint32_t address_of(const std::string &name) const;
    // "name+0xoff" for the symbol at or below `address`, empty when none within 1 MiB.
    std::string describe(std::uint32_t address) const;
    bool starts_symbol(std::uint32_t address) const { return by_address_.count(address) != 0; }
    std::size_t size() const { return by_address_.size(); }

private:
    std::map<std::uint32_t, std::string> by_address_;
    std::map<std::string, std::uint32_t> by_name_;
};

} // namespace esp32emu
