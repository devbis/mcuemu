#pragma once

#include "mcuemu/bus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

struct uc_struct;

namespace mcuemu {

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
    Sleeping,
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
    // Executes up to `instruction_limit` instructions. Instructions are run in
    // batches inside one Unicorn call; a batch ends before the instruction that
    // follows an event the wrapper must handle (a pending IRQ, a sleep request,
    // a halt) or once the device's network time reaches `deadline_ns`. The
    // deadline is only checked between instructions, like the step limit, so
    // the last instruction may finish just past it.
    RunResult run(std::size_t instruction_limit,
                  std::uint64_t deadline_ns = UINT64_MAX);
    bool step(std::string *fault = nullptr,
              std::uint16_t *instruction = nullptr);
    void set_pc_trace(std::ostream *output,
                      std::optional<AddressRange> range = std::nullopt);
    void set_mmio_trace(std::ostream *output,
                        std::optional<AddressRange> range = std::nullopt);
    void set_stack_trace(std::ostream *output);
    void set_trace_prefix(std::string prefix);
    void set_trace_limit(std::size_t limit) noexcept { trace_limit_ = limit; }
    void set_trace_after_step(std::size_t step) noexcept {
        trace_after_step_ = step;
    }
    void set_recent_trace_capacity(std::size_t capacity);
    void dump_recent_trace(std::ostream &output) const;

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

    struct TraceSnapshot {
        std::uint32_t pc;
        std::uint32_t sp;
        std::uint32_t cpsr;
        std::uint16_t instruction;
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
    bool batch_must_end();
    bool run_batch(std::size_t limit, std::string *fault,
                   std::uint16_t *instruction, std::size_t *executed);
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
    bool batch_active_{false};
    bool batch_first_{false};
    std::size_t batch_remaining_{0};
    std::uint64_t batch_deadline_ns_{UINT64_MAX};
    // Lazy device clock inside a batch. Instructions whose cycles cannot change
    // anything yet are only counted (`owed_cycles_`); the device is ticked for
    // the whole span when the bound `slack_cycles_` is reached, before any
    // register access, and at the end of the batch. See Tlsr8258::cycles_until_event().
    std::uint64_t owed_cycles_{0};
    std::uint64_t slack_cycles_{0};
    bool full_next_{true};
    void flush_owed();
    void refresh_slack();
    bool has_previous_instruction_{false};
    std::uint32_t current_pc_{0};
    std::uint16_t current_instruction_{0};
    std::uint32_t previous_pc_{0};
    std::uint16_t previous_instruction_{0};
    std::size_t executed_instructions_{0};
    std::string callback_fault_;
    std::ostream *pc_trace_output_{nullptr};
    std::ostream *mmio_trace_output_{nullptr};
    std::ostream *stack_trace_output_{nullptr};
    std::string trace_prefix_;
    std::optional<AddressRange> pc_trace_range_;
    std::optional<AddressRange> mmio_trace_range_;
    std::size_t trace_limit_{10000};
    std::size_t trace_after_step_{0};
    std::vector<TraceSnapshot> recent_trace_;
    std::size_t recent_trace_head_{0};
    std::size_t recent_trace_count_{0};
    std::size_t pc_trace_count_{0};
    std::size_t mmio_trace_count_{0};
    std::size_t stack_trace_count_{0};
    bool pc_trace_limit_reported_{false};
    bool mmio_trace_limit_reported_{false};
    bool stack_trace_limit_reported_{false};
    bool has_stack_snapshot_{false};
    std::uint32_t last_stack_pointer_{0};
    std::uint32_t last_stack_cpsr_{0};
};

} // namespace mcuemu
