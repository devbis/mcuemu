#pragma once

#include <cstdint>
#include <stdexcept>

namespace mcuemu {

class Bus {
public:
    virtual ~Bus() = default;
    virtual std::uint8_t read8(std::uint32_t address) = 0;
    virtual void write8(std::uint32_t address, std::uint8_t value) = 0;
    virtual void tick(std::uint32_t cycles) = 0;
    virtual bool interrupt_pending() const noexcept { return false; }

    std::uint16_t read16(std::uint32_t address);
    std::uint32_t read32(std::uint32_t address);
    void write16(std::uint32_t address, std::uint16_t value);
    void write32(std::uint32_t address, std::uint32_t value);
};

class BusFault : public std::runtime_error {
public:
    BusFault(std::uint32_t address, const char *operation);
    std::uint32_t address() const noexcept { return address_; }

private:
    std::uint32_t address_;
};

} // namespace mcuemu
