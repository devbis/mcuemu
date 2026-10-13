#include "tc32emu/cpu.hpp"

#include "tc32emu/tlsr8258.hpp"
#include "unicorn/unicorn.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace tc32emu {
namespace {

constexpr std::uint32_t irq_mode = 0x12;
constexpr std::uint32_t svc_mode = 0x13;
constexpr std::uint32_t mode_mask = 0x1f;
constexpr std::uint64_t emulation_end = std::uint64_t{1} << 32;
constexpr std::uint32_t estimated_cycles_per_instruction = 3;

std::string instruction_text(std::uint32_t pc, std::uint16_t instruction) {
    std::ostringstream out;
    out << "TC32 instruction 0x" << std::hex << std::setw(4)
        << std::setfill('0') << instruction << " at 0x" << pc;
    return out.str();
}

} // namespace

Cpu::Cpu(Tlsr8258 &device) : device_(device) {
    uc_engine *engine = nullptr;
    const uc_err open_error = uc_open(UC_ARCH_TC32, UC_MODE_LITTLE_ENDIAN, &engine);
    if (open_error != UC_ERR_OK)
        throw std::runtime_error(std::string("uc_open(TC32): ") +
                                 uc_strerror(open_error));
    uc_ = reinterpret_cast<uc_struct *>(engine);

    try {
        mmio_bindings_[0] = {this, Tlsr8258::RegisterBase};
        mmio_bindings_[1] = {this, Tlsr8258::VendorRegisterAliasBase};
        check_error(uc_mem_map(engine, 0, device_.flash_size(),
                               UC_PROT_READ | UC_PROT_EXEC),
                    "map flash");
        check_error(uc_mem_map_ptr(engine, Tlsr8258::RetainedSramBase,
                                   Tlsr8258::SramSize, UC_PROT_ALL,
                                   device_.sram_data()),
                    "map SRAM");
        check_error(uc_mmio_map(engine, Tlsr8258::RegisterBase,
                                Tlsr8258::RegisterSize, mmio_read,
                                &mmio_bindings_[0], mmio_write,
                                &mmio_bindings_[0]),
                    "map MMIO");
        check_error(uc_mmio_map(engine, Tlsr8258::VendorRegisterAliasBase,
                                Tlsr8258::RegisterSize, mmio_read,
                                &mmio_bindings_[1], mmio_write,
                                &mmio_bindings_[1]),
                    "map vendor register alias");
        check_error(uc_tc32_set_fetch_alias(engine,
                                            Tlsr8258::RetainedSramBase,
                                            device_.flash_size(), 0),
                    "set TC32 flash fetch alias");
        check_error(uc_hook_add(engine, reinterpret_cast<uc_hook *>(&code_hook_),
                                UC_HOOK_CODE,
                                reinterpret_cast<void *>(
                                    static_cast<uc_cb_hookcode_t>(Cpu::code_hook)),
                                this, 1, 0),
                    "install code hook");
        sync_flash();
        reset();
    } catch (...) {
        uc_close(engine);
        uc_ = nullptr;
        throw;
    }
}

Cpu::~Cpu() {
    if (uc_)
        uc_close(reinterpret_cast<uc_engine *>(uc_));
}

void Cpu::check_error(int error, const char *operation) const {
    const auto result = static_cast<uc_err>(error);
    if (result != UC_ERR_OK)
        throw std::runtime_error(std::string("Unicorn ") + operation + ": " +
                                 uc_strerror(result));
}

void Cpu::sync_flash() {
    check_error(uc_mem_write(reinterpret_cast<uc_engine *>(uc_), 0,
                             device_.flash().data(), device_.flash().size()),
                "load flash image");
}

void Cpu::reset(std::uint32_t pc_value) {
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    std::uint32_t cpsr_value = svc_mode;
    for (std::uint32_t mode = 0; mode < 32; ++mode) {
        cpsr_value = mode;
        check_error(uc_reg_write(engine, UC_TC32_REG_CPSR, &cpsr_value),
                    "reset CPSR");
        std::uint32_t zero = 0;
        check_error(uc_reg_write(engine, UC_TC32_REG_SP, &zero), "reset SP bank");
        check_error(uc_reg_write(engine, UC_TC32_REG_LR, &zero), "reset LR bank");
        check_error(uc_reg_write(engine, UC_TC32_REG_SPSR, &zero), "reset SPSR bank");
    }

    cpsr_value = svc_mode;
    check_error(uc_reg_write(engine, UC_TC32_REG_CPSR, &cpsr_value),
                "set reset CPSR");
    for (int regid = UC_TC32_REG_R0; regid <= UC_TC32_REG_R12; ++regid) {
        std::uint32_t zero = 0;
        check_error(uc_reg_write(engine, regid, &zero), "reset register");
    }
    std::uint32_t zero = 0;
    check_error(uc_reg_write(engine, UC_TC32_REG_SSR, &zero), "reset SSR");
    check_error(uc_reg_write(engine, UC_TC32_REG_PC, &pc_value), "set reset PC");
    halted_ = false;
    callback_fault_.clear();
    current_instruction_ = 0;
    current_pc_ = pc_value;
    has_previous_instruction_ = false;
    previous_pc_ = 0;
    previous_instruction_ = 0;
    executed_instructions_ = 0;
    recent_trace_head_ = 0;
    recent_trace_count_ = 0;
    instruction_seen_ = false;
}

std::uint32_t Cpu::reg(unsigned index) const {
    if (index > 15)
        throw std::out_of_range("TC32 register index must be 0..15");
    std::uint32_t value = 0;
    check_error(uc_reg_read(reinterpret_cast<uc_engine *>(uc_),
                            UC_TC32_REG_R0 + static_cast<int>(index), &value),
                "read register");
    return value;
}

void Cpu::set_reg(unsigned index, std::uint32_t value) {
    if (index > 15)
        throw std::out_of_range("TC32 register index must be 0..15");
    check_error(uc_reg_write(reinterpret_cast<uc_engine *>(uc_),
                             UC_TC32_REG_R0 + static_cast<int>(index), &value),
                "write register");
}

std::uint32_t Cpu::cpsr() const {
    std::uint32_t value = 0;
    check_error(uc_reg_read(reinterpret_cast<uc_engine *>(uc_), UC_TC32_REG_CPSR,
                            &value),
                "read CPSR");
    return value;
}

void Cpu::set_cpsr(std::uint32_t value) {
    check_error(uc_reg_write(reinterpret_cast<uc_engine *>(uc_), UC_TC32_REG_CPSR,
                             &value),
                "write CPSR");
}

std::uint32_t Cpu::spsr() const {
    std::uint32_t value = 0;
    check_error(uc_reg_read(reinterpret_cast<uc_engine *>(uc_), UC_TC32_REG_SPSR,
                            &value),
                "read SPSR");
    return value;
}

void Cpu::set_spsr(std::uint32_t value) {
    check_error(uc_reg_write(reinterpret_cast<uc_engine *>(uc_), UC_TC32_REG_SPSR,
                             &value),
                "write SPSR");
}

std::uint32_t Cpu::pc() const { return reg(15); }

void Cpu::set_callback_fault(const std::string &message) {
    if (callback_fault_.empty())
        callback_fault_ = message;
    uc_emu_stop(reinterpret_cast<uc_engine *>(uc_));
}

void Cpu::set_pc_trace(std::ostream *output,
                       std::optional<AddressRange> range) {
    pc_trace_output_ = output;
    pc_trace_range_ = range;
    pc_trace_count_ = 0;
    pc_trace_limit_reported_ = false;
}

void Cpu::set_mmio_trace(std::ostream *output,
                         std::optional<AddressRange> range) {
    mmio_trace_output_ = output;
    mmio_trace_range_ = range;
    mmio_trace_count_ = 0;
    mmio_trace_limit_reported_ = false;
}

void Cpu::set_stack_trace(std::ostream *output) {
    stack_trace_output_ = output;
    stack_trace_count_ = 0;
    stack_trace_limit_reported_ = false;
    has_stack_snapshot_ = false;
}

void Cpu::set_trace_prefix(std::string prefix) {
    trace_prefix_ = std::move(prefix);
}

void Cpu::set_recent_trace_capacity(std::size_t capacity) {
    recent_trace_.assign(capacity, {});
    recent_trace_head_ = 0;
    recent_trace_count_ = 0;
    has_stack_snapshot_ = false;
}

void Cpu::dump_recent_trace(std::ostream &output) const {
    if (recent_trace_.empty())
        return;
    output << trace_prefix_ << "recent instructions (oldest first):\n";
    const std::size_t oldest =
        (recent_trace_head_ + recent_trace_.size() - recent_trace_count_) %
        recent_trace_.size();
    for (std::size_t i = 0; i < recent_trace_count_; ++i) {
        const TraceSnapshot &entry =
            recent_trace_[(oldest + i) % recent_trace_.size()];
        output << trace_prefix_ << "  pc=0x" << std::hex << entry.pc
               << " opcode=0x" << std::setw(4) << std::setfill('0')
               << entry.instruction << " sp=0x" << entry.sp
               << " cpsr=0x" << entry.cpsr << std::dec << '\n';
    }
}

void Cpu::emit_trace(std::ostream *output,
                     const std::optional<AddressRange> &range,
                     std::size_t &event_count, bool &limit_reported,
                     std::uint32_t address, const std::string &message) {
    if (!output || (range && !range->contains(address)))
        return;
    if (event_count >= trace_limit_) {
        if (!limit_reported) {
            *output << "trace limit reached at 0x" << std::hex << address
                    << std::dec << "; further events suppressed\n";
            limit_reported = true;
        }
        return;
    }
    *output << message << '\n';
    ++event_count;
}

std::uint64_t Cpu::mmio_read(uc_struct *opaque, std::uint64_t offset,
                             unsigned size, void *user_data) {
    auto *binding = static_cast<MmioBinding *>(user_data);
    auto *self = binding->cpu;
    std::uint64_t value = 0;
    try {
        if (size > sizeof(value))
            throw std::runtime_error("unsupported MMIO read width");
        for (unsigned i = 0; i < size; ++i) {
            const std::uint64_t address = binding->base + offset + i;
            if (address > UINT32_MAX)
                throw std::runtime_error("MMIO read address overflow");
            value |= static_cast<std::uint64_t>(
                         self->device_.read8(static_cast<std::uint32_t>(address)))
                     << (i * 8u);
        }
        const auto trace_address = static_cast<std::uint32_t>(binding->base + offset);
        if (self->mmio_trace_output_ &&
            self->executed_instructions_ >= self->trace_after_step_ &&
            (!self->mmio_trace_range_ ||
             self->mmio_trace_range_->contains(trace_address))) {
            std::string message;
            if (self->mmio_trace_count_ < self->trace_limit_) {
                std::ostringstream trace;
                trace << "mmio R pc=0x" << std::hex << self->current_pc_
                      << " addr=0x" << trace_address << std::dec << " size="
                      << size << " value=0x" << std::hex << value;
                message = trace.str();
            }
            self->emit_trace(self->mmio_trace_output_, self->mmio_trace_range_,
                             self->mmio_trace_count_,
                             self->mmio_trace_limit_reported_, trace_address,
                             message);
        }
    } catch (const std::exception &error) {
        self->set_callback_fault(error.what());
        (void)opaque;
    }
    return value;
}

void Cpu::mmio_write(uc_struct *opaque, std::uint64_t offset, unsigned size,
                     std::uint64_t value, void *user_data) {
    auto *binding = static_cast<MmioBinding *>(user_data);
    auto *self = binding->cpu;
    try {
        if (size > sizeof(value))
            throw std::runtime_error("unsupported MMIO write width");
        for (unsigned i = 0; i < size; ++i) {
            const std::uint64_t address = binding->base + offset + i;
            if (address > UINT32_MAX)
                throw std::runtime_error("MMIO write address overflow");
            self->device_.write8(static_cast<std::uint32_t>(address),
                                 static_cast<std::uint8_t>(value >> (i * 8u)));
        }
        const auto trace_address = static_cast<std::uint32_t>(binding->base + offset);
        if (self->mmio_trace_output_ &&
            self->executed_instructions_ >= self->trace_after_step_ &&
            (!self->mmio_trace_range_ ||
             self->mmio_trace_range_->contains(trace_address))) {
            std::string message;
            if (self->mmio_trace_count_ < self->trace_limit_) {
                std::ostringstream trace;
                trace << "mmio W pc=0x" << std::hex << self->current_pc_
                      << " addr=0x" << trace_address << std::dec << " size="
                      << size << " value=0x" << std::hex << value;
                message = trace.str();
            }
            self->emit_trace(self->mmio_trace_output_, self->mmio_trace_range_,
                             self->mmio_trace_count_,
                             self->mmio_trace_limit_reported_, trace_address,
                             message);
        }
    } catch (const std::exception &error) {
        self->set_callback_fault(error.what());
        (void)opaque;
    }
}

void Cpu::code_hook(uc_struct *opaque, std::uint64_t address,
                    std::uint32_t size, void *user_data) {
    auto *self = static_cast<Cpu *>(user_data);
    if (self->batch_active_) {
        // The first instruction of a batch always runs: the caller has just
        // handled whatever was pending. Later ones stop the batch instead of
        // starting when something needs the wrapper (see batch_must_end()).
        if (self->batch_first_)
            self->batch_first_ = false;
        else if (self->batch_must_end()) {
            const std::uint32_t resume_pc = static_cast<std::uint32_t>(address);
            auto *engine = reinterpret_cast<uc_engine *>(opaque);
            uc_reg_write(engine, UC_TC32_REG_PC, &resume_pc);
            uc_emu_stop(engine);
            return;
        }
        --self->batch_remaining_;
    }
    const std::uint32_t previous_pc = self->previous_pc_;
    const std::uint16_t previous_instruction = self->previous_instruction_;
    const bool has_previous_instruction = self->has_previous_instruction_;
    self->instruction_seen_ = true;
    // Unicorn exposes instruction hooks but no per-instruction cycle count.
    // Use a coarse TC32 estimate so device timers advance in core cycles.
    self->device_.tick(estimated_cycles_per_instruction);
    self->current_pc_ = static_cast<std::uint32_t>(address);
    ++self->executed_instructions_;

    if (self->stack_trace_output_ &&
        self->executed_instructions_ >= self->trace_after_step_) {
        std::uint32_t stack_pointer = 0;
        std::uint32_t status = 0;
        auto *engine = reinterpret_cast<uc_engine *>(opaque);
        if (uc_reg_read(engine, UC_TC32_REG_SP, &stack_pointer) == UC_ERR_OK &&
            uc_reg_read(engine, UC_TC32_REG_CPSR, &status) == UC_ERR_OK) {
            if (self->has_stack_snapshot_ &&
                (stack_pointer != self->last_stack_pointer_ ||
                 (status & mode_mask) != (self->last_stack_cpsr_ & mode_mask))) {
                std::ostringstream trace;
                trace << self->trace_prefix_ << "stack pc=0x" << std::hex
                      << address << " sp=0x" << stack_pointer << " cpsr=0x"
                      << status << " previous_sp=0x"
                      << self->last_stack_pointer_ << " previous_cpsr=0x"
                      << self->last_stack_cpsr_;
                self->emit_trace(self->stack_trace_output_, std::nullopt,
                                 self->stack_trace_count_,
                                 self->stack_trace_limit_reported_,
                                 static_cast<std::uint32_t>(address),
                                 trace.str());
            }
            self->last_stack_pointer_ = stack_pointer;
            self->last_stack_cpsr_ = status;
            self->has_stack_snapshot_ = true;
        }
    }

    std::array<std::uint8_t, 2> bytes{};
    bool have_instruction = false;
    const std::uint64_t flash_offset =
        address >= Tlsr8258::RetainedSramBase
            ? static_cast<std::uint64_t>(address) -
                  Tlsr8258::RetainedSramBase
            : UINT64_MAX;
    const auto &flash = self->device_.flash();
    if (size >= bytes.size() && flash_offset < flash.size() &&
        bytes.size() <= flash.size() - flash_offset) {
        bytes[0] = flash[static_cast<std::size_t>(flash_offset)];
        bytes[1] = flash[static_cast<std::size_t>(flash_offset + 1)];
        have_instruction = true;
    } else if (size >= bytes.size() &&
               uc_mem_read(reinterpret_cast<uc_engine *>(opaque), address,
                           bytes.data(), bytes.size()) == UC_ERR_OK) {
        have_instruction = true;
    }
    if (have_instruction) {
        self->current_instruction_ = static_cast<std::uint16_t>(bytes[0]) |
                                     (static_cast<std::uint16_t>(bytes[1]) << 8);
        self->previous_pc_ = static_cast<std::uint32_t>(address);
        self->previous_instruction_ = self->current_instruction_;
        self->has_previous_instruction_ = true;
        if (!self->recent_trace_.empty()) {
            std::uint32_t stack_pointer = 0;
            std::uint32_t status = 0;
            auto *engine = reinterpret_cast<uc_engine *>(opaque);
            if (uc_reg_read(engine, UC_TC32_REG_SP, &stack_pointer) ==
                    UC_ERR_OK &&
                uc_reg_read(engine, UC_TC32_REG_CPSR, &status) == UC_ERR_OK) {
                self->recent_trace_[self->recent_trace_head_] = {
                    static_cast<std::uint32_t>(address), stack_pointer, status,
                    self->current_instruction_};
                self->recent_trace_head_ =
                    (self->recent_trace_head_ + 1) % self->recent_trace_.size();
                self->recent_trace_count_ = std::min(
                    self->recent_trace_count_ + 1, self->recent_trace_.size());
            }
        }
        const auto trace_address = static_cast<std::uint32_t>(address);
        if (self->pc_trace_output_ &&
            self->executed_instructions_ >= self->trace_after_step_ &&
            (!self->pc_trace_range_ ||
             self->pc_trace_range_->contains(trace_address))) {
            std::string message;
            if (self->pc_trace_count_ < self->trace_limit_) {
                std::ostringstream trace;
                trace << self->trace_prefix_ << "pc=0x" << std::hex << address
                      << " opcode=0x"
                      << std::setw(4) << std::setfill('0')
                      << self->current_instruction_ << " size=" << std::dec
                      << size << " step=" << self->executed_instructions_;
                std::uint32_t stack_pointer = 0;
                std::uint32_t status = 0;
                auto *engine = reinterpret_cast<uc_engine *>(opaque);
                if (uc_reg_read(engine, UC_TC32_REG_SP, &stack_pointer) ==
                        UC_ERR_OK &&
                    uc_reg_read(engine, UC_TC32_REG_CPSR, &status) ==
                        UC_ERR_OK) {
                    trace << " sp=0x" << std::hex << stack_pointer
                          << " cpsr=0x" << status;
                }
                if (has_previous_instruction) {
                    trace << " from=0x" << std::hex << previous_pc
                          << " previous_opcode=0x" << std::setw(4)
                          << std::setfill('0') << previous_instruction;
                }
                message = trace.str();
            }
            self->emit_trace(self->pc_trace_output_, self->pc_trace_range_,
                             self->pc_trace_count_,
                             self->pc_trace_limit_reported_, trace_address,
                             message);
        }
        // 0xcfxx is retained as a board-test semihosting stop marker. It is
        // deliberately handled by the wrapper; it is not an ISA instruction
        // implemented by Unicorn's TC32 backend.
        if ((self->current_instruction_ & 0xff00u) == 0xcf00u) {
            auto *engine = reinterpret_cast<uc_engine *>(opaque);
            const std::uint32_t next_pc = static_cast<std::uint32_t>(address + 2);
            uc_reg_write(engine, UC_TC32_REG_PC, &next_pc);
            self->halted_ = true;
            uc_emu_stop(engine);
        }
    }
}

bool Cpu::enter_pending_irq() {
    if (!device_.interrupt_pending())
        return false;

    const std::uint32_t old_cpsr = cpsr();
    if ((old_cpsr & mode_mask) == irq_mode)
        return false;

    const std::uint32_t return_pc = pc();
    const std::uint32_t irq_cpsr = (old_cpsr & ~mode_mask) | irq_mode;
    set_cpsr(irq_cpsr);
    check_error(uc_reg_write(reinterpret_cast<uc_engine *>(uc_), UC_TC32_REG_SPSR,
                             &old_cpsr),
                "save IRQ status");
    const std::uint32_t irq_lr = return_pc;
    check_error(uc_reg_write(reinterpret_cast<uc_engine *>(uc_), UC_TC32_REG_LR,
                             &irq_lr),
                "set IRQ return address");
    const std::uint32_t vector = IrqVector;
    check_error(uc_reg_write(reinterpret_cast<uc_engine *>(uc_), UC_TC32_REG_PC,
                             &vector),
                "set IRQ vector");
    device_.tick(1);
    return true;
}

bool Cpu::batch_must_end() {
    if (halted_ || batch_remaining_ == 0 ||
        device_.power_state() != Tlsr8258PowerState::Awake ||
        device_.network_time_ns() >= batch_deadline_ns_)
        return true;
    if (!device_.interrupt_pending())
        return false;
    // enter_pending_irq() ignores an IRQ raised while one is being served.
    std::uint32_t status = 0;
    uc_reg_read(reinterpret_cast<uc_engine *>(uc_), UC_TC32_REG_CPSR, &status);
    return (status & mode_mask) != irq_mode;
}

bool Cpu::run_batch(std::size_t limit, std::string *fault,
                    std::uint16_t *instruction, std::size_t *executed) {
    *executed = 0;
    if (halted_)
        return false;

    callback_fault_.clear();
    instruction_seen_ = false;
    current_instruction_ = 0;
    if (enter_pending_irq()) {
        if (instruction)
            *instruction = 0;
        *executed = 1;
        return true;
    }

    const std::uint32_t start_pc = pc();
    const std::size_t before = executed_instructions_;
    batch_active_ = true;
    batch_first_ = true;
    batch_remaining_ = limit;
    const uc_err error =
        uc_emu_start(reinterpret_cast<uc_engine *>(uc_), start_pc, emulation_end,
                     0, 0);
    batch_active_ = false;
    *executed = executed_instructions_ - before;
    if (instruction)
        *instruction = current_instruction_;
    if (!callback_fault_.empty()) {
        if (fault)
            *fault = callback_fault_;
        return false;
    }
    if (error != UC_ERR_OK) {
        if (fault) {
            std::ostringstream out;
            out << uc_strerror(error) << " while executing "
                << instruction_text(pc(), current_instruction_);
            *fault = out.str();
        }
        return false;
    }
    if (!instruction_seen_ && !halted_) {
        if (fault)
            *fault = "Unicorn stopped without executing a TC32 instruction";
        return false;
    }
    return true;
}

bool Cpu::step(std::string *fault, std::uint16_t *instruction) {
    std::size_t executed = 0;
    return run_batch(1, fault, instruction, &executed);
}

RunResult Cpu::run(std::size_t instruction_limit, std::uint64_t deadline_ns) {
    RunResult result;
    while (result.instructions < instruction_limit) {
        if (halted_) {
            result.reason = StopReason::Halted;
            result.pc = pc();
            return result;
        }

        if (device_.power_state() != Tlsr8258PowerState::Awake) {
            if (!device_.advance_sleep_to_wakeup()) {
                result.reason = StopReason::Sleeping;
                result.pc = pc();
                result.message =
                    "32 kHz timer did not produce a wake event; check the "
                    "wake-source enable, compare value, and selected clock power";
                return result;
            }
            if (device_.consume_deep_sleep_wakeup())
                reset();
            continue;
        }

        if (result.instructions != 0 && device_.network_time_ns() >= deadline_ns)
            break;

        std::string fault;
        std::uint16_t instruction = 0;
        std::size_t executed = 0;
        batch_deadline_ns_ = deadline_ns;
        const bool ok = run_batch(instruction_limit - result.instructions,
                                  &fault, &instruction, &executed);
        result.instructions += executed;
        if (!ok) {
            result.reason = halted_ ? StopReason::Halted : StopReason::Fault;
            result.pc = pc();
            result.instruction = instruction;
            result.message = std::move(fault);
            return result;
        }
        result.instruction = instruction;
        if (halted_) {
            result.reason = StopReason::Halted;
            result.pc = pc();
            return result;
        }
    }

    result.reason = StopReason::StepLimit;
    result.pc = pc();
    return result;
}

} // namespace tc32emu
