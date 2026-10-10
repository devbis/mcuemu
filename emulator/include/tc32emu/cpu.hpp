#pragma once

#include "tc32emu/bus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>

struct uc_struct;

namespace tc32emu {

class Tlsr8258;

struct AddressRange {
    std::uint32_t first;
    std::uint32_t last;

    bool contains(std::uint32_t address) const noexcept {
        return address >= first && address <= last;
    }
};

enum class StopReason {
    StepLimit,
    Halted,
    Fault,
};

struct RunResult {
    StopReason reason{StopReason::StepLimit};
    std::size_t instructions{0};
    std::uint32_t pc{0};
    std::uint16_t instruction{0};
    std::string message;
};

// Board-facing CPU adapter. Instruction execution and architectural state are
// provided by Unicorn's UC_ARCH_TC32 backend; this class only maps the board
// memory and drives virtual-device timing and interrupts.
class Cpu {
public:
    static constexpr std::uint32_t IrqVector = 0x10;
    static constexpr std::uint32_t FlagN = 1u << 31;
    static constexpr std::uint32_t FlagZ = 1u << 30;
    static constexpr std::uint32_t FlagC = 1u << 29;
    static constexpr std::uint32_t FlagV = 1u << 28;

    explicit Cpu(Tlsr8258 &device);
    ~Cpu();
    Cpu(const Cpu &) = delete;
    Cpu &operator=(const Cpu &) = delete;

    void reset(std::uint32_t pc = 0);
    void sync_flash();
    RunResult run(std::size_t instruction_limit);
    bool step(std::string *fault = nullptr,
              std::uint16_t *instruction = nullptr);
    void set_pc_trace(std::ostream *output,
                      std::optional<AddressRange> range = std::nullopt);
    void set_mmio_trace(std::ostream *output,
                        std::optional<AddressRange> range = std::nullopt);
    void set_trace_limit(std::size_t limit) noexcept { trace_limit_ = limit; }

    std::uint32_t reg(unsigned index) const;
    void set_reg(unsigned index, std::uint32_t value);
    std::uint32_t cpsr() const;
    void set_cpsr(std::uint32_t value);
    std::uint32_t spsr() const;
    void set_spsr(std::uint32_t value);
    std::uint32_t pc() const;
    bool halted() const noexcept { return halted_; }

private:
    struct MmioBinding {
        Cpu *cpu;
        std::uint32_t base;
    };

    static void code_hook(uc_struct *uc, std::uint64_t address,
                          std::uint32_t size, void *user_data);
    static std::uint64_t mmio_read(uc_struct *uc, std::uint64_t offset,
                                   unsigned size, void *user_data);
    static void mmio_write(uc_struct *uc, std::uint64_t offset,
                           unsigned size, std::uint64_t value,
                           void *user_data);

    void check_error(int error, const char *operation) const;
    bool enter_pending_irq();
    void set_callback_fault(const std::string &message);
    void emit_trace(std::ostream *output,
                    const std::optional<AddressRange> &range,
                    std::size_t &event_count, bool &limit_reported,
                    std::uint32_t address, const std::string &message);

    Tlsr8258 &device_;
    uc_struct *uc_{nullptr};
    std::array<MmioBinding, 2> mmio_bindings_{};
    std::size_t code_hook_{0};
    bool halted_{false};
    bool instruction_seen_{false};
    bool has_previous_instruction_{false};
    std::uint32_t current_pc_{0};
    std::uint16_t current_instruction_{0};
    std::uint32_t previous_pc_{0};
    std::uint16_t previous_instruction_{0};
    std::size_t executed_instructions_{0};
    std::string callback_fault_;
    std::ostream *pc_trace_output_{nullptr};
    std::ostream *mmio_trace_output_{nullptr};
    std::optional<AddressRange> pc_trace_range_;
    std::optional<AddressRange> mmio_trace_range_;
    std::size_t trace_limit_{10000};
    std::size_t pc_trace_count_{0};
    std::size_t mmio_trace_count_{0};
    bool pc_trace_limit_reported_{false};
    bool mmio_trace_limit_reported_{false};
};

} // namespace tc32emu
