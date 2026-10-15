#pragma once

#include "blocks.hpp"

#include <array>
#include <cstdint>
#include <functional>

namespace esp32emu {

// The ESP32-C6/H2 interrupt path in its PLIC mode (SOC_INT_PLIC_SUPPORTED, the
// mode ESP-IDF and Zephyr use): peripherals raise source lines; the interrupt
// matrix (INTMTX, 0x60010000) routes each source to one of the hart's 31 CPU
// interrupt lines; the machine PLIC (PLIC_MX, 0x20001000) enables, types
// (level or edge), prioritises and thresholds them. The winner is handed to the
// hart as mcause = its number (riscv::Core::set_interrupt).
//
// Layout from ESP-IDF (soc/esp32c6/register/soc/plic_reg.h, interrupt_matrix_reg.h).
// Inferred: among equal priorities the higher line number wins (the documented
// rule of the ESP32-C3 interrupt controller); an interrupt is taken when its
// priority is greater than the threshold.
class Intc {
public:
    // Offset of INT_STATUS_REG_0 in the matrix (the raw source status the dispatcher reads): 0x134 on the C6,
    // 0x104 on the H2.
    void set_status_offset(std::uint32_t offset) { status_offset_ = offset; }
    static constexpr unsigned Sources = 77;   // C6; INTMTX map registers 0..76
    static constexpr unsigned CpuLines = 32;  // line 0 is "not routed"

    explicit Intc(std::function<void(int cause)> deliver);

    // Source line level from a peripheral.
    void set_source(unsigned source, bool level);

    // INTMTX window (offsets from 0x60010000).
    std::uint32_t matrix_read(std::uint32_t offset) const;
    void matrix_write(std::uint32_t offset, std::uint32_t value);
    // PLIC_MX window (offsets from 0x20001000).
    std::uint32_t plic_read(std::uint32_t offset) const;
    void plic_write(std::uint32_t offset, std::uint32_t value);

private:
    void evaluate();
    std::uint32_t line_levels() const;

    std::function<void(int)> deliver_;
    std::uint32_t status_offset_{0x134};
    std::array<std::uint8_t, Sources> route_{};
    std::array<bool, Sources> level_{};
    std::uint32_t enable_{0}, type_{0}, edge_pending_{0}, last_lines_{0};
    std::array<std::uint32_t, CpuLines> priority_{};
    std::uint32_t threshold_{0};
    std::uint32_t conf_{0};
    int delivered_{-1};
};

class IntMatrixBlock final : public Block {
public:
    IntMatrixBlock(std::uint32_t base, Intc &intc) : Block(base, 0x1000), intc_(intc) {}
    std::uint32_t read(std::uint32_t offset, unsigned) override { return intc_.matrix_read(offset); }
    void write(std::uint32_t offset, std::uint32_t value, unsigned) override {
        intc_.matrix_write(offset, value);
    }

private:
    Intc &intc_;
};

class PlicBlock final : public Block {
public:
    PlicBlock(std::uint32_t base, Intc &intc) : Block(base, 0x400), intc_(intc) {}
    std::uint32_t read(std::uint32_t offset, unsigned) override { return intc_.plic_read(offset); }
    void write(std::uint32_t offset, std::uint32_t value, unsigned) override {
        intc_.plic_write(offset, value);
    }

private:
    Intc &intc_;
};

} // namespace esp32emu
