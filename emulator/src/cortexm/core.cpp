#include "cortexm/core.hpp"

#include <unicorn/arm.h>
#include <unicorn/unicorn.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <limits>
#include <ostream>
#include <stdexcept>

namespace cortexm {

namespace {

constexpr std::uint32_t ScsBase = 0xe000e000u;
constexpr std::uint32_t PrivateBase = 0xe0000000u; // ITM, DWT, SCS, debug ROM
constexpr std::uint32_t PrivateSize = 0x00100000u;
constexpr std::uint32_t DwtBase = 0xe0001000u;
constexpr std::uint64_t Never = std::numeric_limits<std::uint64_t>::max();

// Thumb hint instructions the core must not hand to the engine: WFI/WFE put
// the model to sleep, SEV sets the event register, YIELD/SEVL are no-ops.
enum class Hint { None, Yield, Wfe, Wfi, Sev, Sevl };

Hint decode_hint(std::uint16_t first, std::uint16_t second, unsigned size) {
    if (size == 2) {
        switch (first) {
        case 0xbf10: return Hint::Yield;
        case 0xbf20: return Hint::Wfe;
        case 0xbf30: return Hint::Wfi;
        case 0xbf40: return Hint::Sev;
        case 0xbf50: return Hint::Sevl;
        default: return Hint::None;
        }
    }
    if (first == 0xf3af) {
        switch (second) {
        case 0x8001: return Hint::Yield;
        case 0x8002: return Hint::Wfe;
        case 0x8003: return Hint::Wfi;
        case 0x8004: return Hint::Sev;
        case 0x8005: return Hint::Sevl;
        default: return Hint::None;
        }
    }
    return Hint::None;
}

} // namespace

struct Core::Region {
    std::uint32_t base;
    std::uint32_t size;
    bool writable;
    std::vector<std::uint8_t> storage;
};

enum class Cause { None, Deadline, StepLimit, Hint, Halted, Reset };

struct Core::Impl final : Bus {
    Core &core;
    std::vector<std::unique_ptr<Region>> regions;
    WriteProtectHandler write_protect;
    ServiceHandler service;
    std::vector<std::function<void()>> deferred;
    uc_hook hooks[3]{};
    uc_hook invalid_hook{};
    std::uint64_t invalid_address{0};
    std::uint64_t invalid_value{0};
    unsigned invalid_size{0};
    bool invalid_write{false};
    std::uint64_t deadline{0};
    std::uint64_t service_deadline{Never};
    Cause cause{Cause::None};
    Hint hint{Hint::None};
    unsigned hint_size{0};
    bool event{false};
    bool sleep_wfe{false};
    bool reset_pending{false};

    // SysTick.
    std::uint32_t st_ctrl{0}, st_reload{0}, st_value{0};
    std::uint64_t st_cycle{0}; // cycle at which st_value was last brought up to date
    bool st_countflag{false};
    // DWT.
    std::uint32_t dwt_ctrl{0};
    std::uint64_t dwt_origin{0}; // cycles when CYCCNT was zero

    explicit Impl(Core &c) : core(c) {}

    // ---- engine setup ----------------------------------------------------
    static std::uint64_t mmio_read(uc_engine *, std::uint64_t offset, unsigned size, void *user) {
        auto *entry = static_cast<MmioEntryUser *>(user);
        return entry->bus->read(entry->base + static_cast<std::uint32_t>(offset), size);
    }
    static void mmio_write(uc_engine *, std::uint64_t offset, unsigned size,
                           std::uint64_t value, void *user) {
        auto *entry = static_cast<MmioEntryUser *>(user);
        entry->bus->write(entry->base + static_cast<std::uint32_t>(offset),
                          static_cast<std::uint32_t>(value), size);
    }
    struct MmioEntryUser {
        std::uint32_t base;
        std::uint32_t size;
        Bus *bus;
    };
    std::vector<std::unique_ptr<MmioEntryUser>> mmio_users;

    static void code_hook(uc_engine *, std::uint64_t address, std::uint32_t size, void *user) {
        static_cast<Impl *>(user)->on_instruction(static_cast<std::uint32_t>(address), size);
    }
    static void intr_hook(uc_engine *, std::uint32_t intno, void *user) {
        auto *self = static_cast<Impl *>(user);
        // 7 is EXCP_BKPT in the ARM backend; the host decides what it means.
        (void)intno;
        self->cause = Cause::Halted;
        uc_emu_stop(self->core.uc_);
    }
    static bool write_prot_hook(uc_engine *, uc_mem_type, std::uint64_t address, int size,
                                std::int64_t value, void *user) {
        auto *self = static_cast<Impl *>(user);
        if (!self->write_protect)
            return false;
        return self->write_protect(static_cast<std::uint32_t>(address),
                                   static_cast<unsigned>(size),
                                   static_cast<std::uint32_t>(value));
    }

    static bool invalid_access(uc_engine *, uc_mem_type type, std::uint64_t address, int size,
                               std::int64_t value, void *user) {
        auto *self = static_cast<Impl *>(user);
        self->invalid_address = address;
        self->invalid_size = static_cast<unsigned>(size);
        self->invalid_value = static_cast<std::uint64_t>(value);
        self->invalid_write = type == UC_MEM_WRITE_UNMAPPED || type == UC_MEM_WRITE_PROT;
        return false; // Preserve the engine fault; record details for the host diagnostic.
    }

    void build() {
        if (core.uc_) {
            uc_close(core.uc_);
            core.uc_ = nullptr;
        }
        uc_engine *uc = nullptr;
        if (uc_open(UC_ARCH_ARM, static_cast<uc_mode>(UC_MODE_THUMB | UC_MODE_MCLASS), &uc) != UC_ERR_OK)
            throw std::runtime_error("cannot create the ARM engine");
        core.uc_ = uc;
        const uc_cpu_arm model = core.config_.model == Model::M0    ? UC_CPU_ARM_CORTEX_M0
                                 : core.config_.model == Model::M3    ? UC_CPU_ARM_CORTEX_M3
                                 : core.config_.model == Model::M33 ? UC_CPU_ARM_CORTEX_M33
                                                                    : UC_CPU_ARM_CORTEX_M4;
        check(uc_ctl_set_cpu_model(uc, model), "set CPU model");

        for (auto &region : regions) {
            const uc_prot perms = region->writable
                                      ? static_cast<uc_prot>(UC_PROT_READ | UC_PROT_WRITE | UC_PROT_EXEC)
                                      : static_cast<uc_prot>(UC_PROT_READ | UC_PROT_EXEC);
            check(uc_mem_map_ptr(uc, region->base, region->size, perms, region->storage.data()),
                  "map memory");
        }
        for (auto &user : mmio_users)
            check(uc_mmio_map(uc, user->base, user->size, &Impl::mmio_read, user.get(),
                              &Impl::mmio_write, user.get()),
                  "map peripherals");
        // The private peripheral bus (SCS, SysTick, DWT, ...) is the core's own.
        check(uc_mmio_map(uc, PrivateBase, PrivateSize, &Impl::private_read_thunk, this,
                          &Impl::private_write_thunk, this),
              "map the private peripheral bus");
        check(uc_hook_add(uc, &hooks[0], UC_HOOK_CODE, reinterpret_cast<void *>(&Impl::code_hook),
                          this, 1, 0),
              "add code hook");
        check(uc_hook_add(uc, &hooks[1], UC_HOOK_INTR, reinterpret_cast<void *>(&Impl::intr_hook),
                          this, 1, 0),
              "add exception hook");
        check(uc_hook_add(uc, &hooks[2], UC_HOOK_MEM_WRITE_PROT,
                          reinterpret_cast<void *>(&Impl::write_prot_hook), this, 1, 0),
              "add write-protect hook");
        check(uc_hook_add(uc, &invalid_hook, UC_HOOK_MEM_INVALID,
                          reinterpret_cast<void *>(&Impl::invalid_access), this, 1, 0),
              "add invalid memory hook");
        std::uint32_t priority_bits = core.config_.priority_bits;
        check(uc_arm_nvic(uc, UC_ARM_NVIC_CONFIG, core.config_.irq_count, &priority_bits, 0),
              "configure NVIC");
        if (core.config_.model == Model::M33)
            check(uc_arm_nvic(uc, UC_ARM_NVIC_NO_SECURITY, 0, nullptr, 0),
                  "drop the Security Extension");
    }

    static std::uint64_t private_read_thunk(uc_engine *, std::uint64_t offset, unsigned size,
                                            void *user) {
        return static_cast<Impl *>(user)->read(PrivateBase + static_cast<std::uint32_t>(offset), size);
    }
    static void private_write_thunk(uc_engine *, std::uint64_t offset, unsigned size,
                                    std::uint64_t value, void *user) {
        static_cast<Impl *>(user)->write(PrivateBase + static_cast<std::uint32_t>(offset),
                                         static_cast<std::uint32_t>(value), size);
    }

    static void check(uc_err error, const char *what) {
        if (error != UC_ERR_OK)
            throw std::runtime_error(std::string("cortexm: ") + what + ": " + uc_strerror(error));
    }

    // ---- per-instruction hook ------------------------------------------
    void on_instruction(std::uint32_t address, unsigned size) {
        if (!deferred.empty()) {
            auto actions = std::move(deferred);
            deferred.clear();
            for (auto &action : actions)
                action();
        }
        if (core.cycles_ >= deadline) {
            cause = Cause::Deadline;
            uc_emu_stop(core.uc_);
            return;
        }
        if (core.stack_trace_ != nullptr) {
            std::uint32_t msp = 0, psp = 0;
            if (uc_reg_read(core.uc_, UC_ARM_REG_MSP, &msp) == UC_ERR_OK &&
                uc_reg_read(core.uc_, UC_ARM_REG_PSP, &psp) == UC_ERR_OK) {
                auto trace_bank = [&](const char *name, std::uint32_t value,
                                      std::uint32_t &last, bool &last_valid,
                                      bool &initialized) {
                    const bool valid = value >= core.stack_lower_bound_ &&
                                       value <= core.stack_upper_bound_;
                    if ((!initialized && value != 0 && !valid) ||
                        (initialized && valid != last_valid)) {
                        *core.stack_trace_ << name << " trace: 0x" << std::hex << last
                                           << " -> 0x" << value;
                        if (core.last_instruction_pc_ != 0)
                            *core.stack_trace_ << " after 0x" << core.last_instruction_pc_;
                        *core.stack_trace_ << " before 0x" << address << " ("
                                           << (valid ? "valid" : "outside SRAM") << ")\n" << std::dec;
                    }
                    last = value;
                    last_valid = valid;
                    initialized = true;
                };
                trace_bank("MSP", msp, core.last_msp_, core.last_msp_valid_,
                           core.msp_trace_initialized_);
                trace_bank("PSP", psp, core.last_psp_, core.last_psp_valid_,
                           core.psp_trace_initialized_);
            }
        }
        if (core.config_.track_last_instruction_pc || core.pc_trace_ != nullptr ||
            core.stack_trace_ != nullptr)
            core.last_instruction_pc_ = address;
        if (core.pc_trace_ != nullptr && address >= core.pc_first_ && address <= core.pc_last_) {
            *core.pc_trace_ << "PC 0x" << std::hex << address << std::dec << " cycles=" << core.cycles_;
            if (core.pc_trace_regs_) {
                for (unsigned reg = 0; reg != 13; ++reg)
                    *core.pc_trace_ << " r" << reg << "=0x" << std::hex << core.reg(reg) << std::dec;
                *core.pc_trace_ << " sp=0x" << std::hex << core.reg(13)
                                << " lr=0x" << core.reg(14);
                std::uint32_t msp = 0, psp = 0, control = 0, ipsr = 0;
                uc_reg_read(core.uc_, UC_ARM_REG_MSP, &msp);
                uc_reg_read(core.uc_, UC_ARM_REG_PSP, &psp);
                uc_reg_read(core.uc_, UC_ARM_REG_CONTROL, &control);
                uc_reg_read(core.uc_, UC_ARM_REG_IPSR, &ipsr);
                *core.pc_trace_ << " msp=0x" << msp << " psp=0x" << psp
                                << " control=0x" << control << " ipsr=0x" << ipsr << std::dec;
            }
            *core.pc_trace_ << '\n';
        }
        if (core.instruction_limit_ != 0 && core.instructions_ >= core.instruction_limit_) {
            cause = Cause::StepLimit;
            uc_emu_stop(core.uc_);
            return;
        }
        // Hints are decoded from the host copy of the code (flash or RAM).
        if (const std::uint8_t *code = core.host_pointer(address, size)) {
            const std::uint16_t first = static_cast<std::uint16_t>(code[0] | (code[1] << 8));
            if ((first & 0xff00) == 0xbf00 || first == 0xf3af) {
                const std::uint16_t second =
                    size == 4 ? static_cast<std::uint16_t>(code[2] | (code[3] << 8)) : 0;
                const Hint found = decode_hint(first, second, size);
                if (found != Hint::None) {
                    hint = found;
                    hint_size = size;
                    cause = Cause::Hint;
                    uc_emu_stop(core.uc_);
                    return;
                }
            }
        }
        ++core.instructions_;
        core.cycles_ += core.cpi_;
    }

    // ---- private peripheral bus ----------------------------------------
    // The counter is modelled as the number of core cycles left until it next
    // reaches zero; zero in the register means it reloads on the next tick.
    void systick_sync(std::uint64_t now) {
        if (!(st_ctrl & 1u) || st_reload == 0) {
            st_cycle = now;
            return;
        }
        std::uint64_t elapsed = now - st_cycle;
        st_cycle = now;
        const std::uint64_t period = static_cast<std::uint64_t>(st_reload) + 1;
        const std::uint64_t remaining = st_value ? st_value : period;
        if (elapsed < remaining) {
            st_value = static_cast<std::uint32_t>(remaining - elapsed);
            return;
        }
        elapsed -= remaining;
        st_value = static_cast<std::uint32_t>(period - elapsed % period);
        st_countflag = true;
        if (st_ctrl & 2u) {
            std::uint32_t level = 1;
            uc_arm_nvic(core.uc_, UC_ARM_NVIC_SET_IRQ, 15, &level, 0);
            level = 0;
            uc_arm_nvic(core.uc_, UC_ARM_NVIC_SET_IRQ, 15, &level, 0);
        }
    }

    std::uint64_t systick_next() const {
        if ((st_ctrl & 3u) != 3u || st_reload == 0)
            return Never;
        return st_cycle + (st_value ? st_value : static_cast<std::uint64_t>(st_reload) + 1);
    }

    std::uint32_t read(std::uint32_t address, unsigned size) override {
        if (address >= ScsBase && address < ScsBase + 0x1000) {
            const std::uint32_t offset = address - ScsBase;
            if (offset >= 0x10 && offset < 0x20)
                return systick_read(offset);
            std::uint32_t value = 0;
            uc_arm_nvic(core.uc_, UC_ARM_NVIC_SCS_READ, offset, &value, size);
            // Sub-word reads select their byte lane of the 32-bit register.
            return size == 4 ? value : (value >> ((offset & 3u) * 8)) & (size == 1 ? 0xffu : 0xffffu);
        }
        if (address == DwtBase)
            return dwt_ctrl;
        if (address == DwtBase + 4)
            return (dwt_ctrl & 1u) ? static_cast<std::uint32_t>(core.cycles_ - dwt_origin) : 0;
        return 0;
    }

    std::uint32_t systick_read(std::uint32_t offset) {
        systick_sync(core.cycles_);
        switch (offset) {
        case 0x10: {
            const std::uint32_t value = st_ctrl | (st_countflag ? 0x10000u : 0u);
            st_countflag = false;
            return value;
        }
        case 0x14: return st_reload;
        case 0x18: return std::min(st_value, st_reload);
        default: return 0; // CALIB: no reference clock, no TENMS value
        }
    }

    void write(std::uint32_t address, std::uint32_t value, unsigned size) override {
        if (address >= ScsBase && address < ScsBase + 0x1000) {
            const std::uint32_t offset = address - ScsBase;
            if (offset >= 0x10 && offset < 0x20) {
                systick_write(offset, value);
                return;
            }
            uc_arm_nvic(core.uc_, UC_ARM_NVIC_SCS_WRITE, offset, &value, size);
            std::uint32_t reset = 0;
            uc_arm_nvic(core.uc_, UC_ARM_NVIC_TAKE_RESET, 0, &reset, 0);
            if (reset) {
                cause = Cause::Reset;
                uc_emu_stop(core.uc_);
            }
            return;
        }
        if (address == DwtBase) {
            dwt_ctrl = value & 1u;
            return;
        }
        if (address == DwtBase + 4) {
            dwt_origin = core.cycles_ - value;
            return;
        }
    }

    void systick_write(std::uint32_t offset, std::uint32_t value) {
        systick_sync(core.cycles_);
        switch (offset) {
        case 0x10:
            st_ctrl = value & 7u;
            break;
        case 0x14:
            st_reload = value & 0xffffffu;
            break;
        case 0x18:
            st_value = 0;
            st_countflag = false;
            break;
        default:
            break;
        }
        if (offset == 0x10 || offset == 0x18)
            core.request_service(core.cycles_ + 1);
    }

    void run_service() {
        systick_sync(core.cycles_);
        std::uint64_t next = systick_next();
        // A peripheral serviced early in the pass can be armed by an event a later one raises
        // (a timer compare triggering a radio task through PPI): that request_service() call
        // lands in service_deadline while the pass runs and must not be overwritten.
        service_deadline = std::numeric_limits<std::uint64_t>::max();
        if (service)
            next = std::min(next, service(core.cycles_));
        next = std::min(next, service_deadline);
        service_deadline = next <= core.cycles_ ? core.cycles_ + 1 : next;
    }
};

// ---- Core ------------------------------------------------------------------

Core::Core(const Config &config) : config_(config), impl_(std::make_unique<Impl>(*this)) {
    // Memory is mapped by the chip model, which then calls reset(); the engine
    // is built there so the maps exist before the CPU is created.
}

Core::~Core() {
    if (uc_)
        uc_close(uc_);
}

std::uint8_t *Core::map_memory(std::uint32_t base, std::uint32_t size, bool writable) {
    auto region = std::make_unique<Region>();
    region->base = base;
    region->size = size;
    region->writable = writable;
    // Erased flash reads 0xFF; RAM comes up zeroed (real SRAM is undefined).
    region->storage.assign(size, writable ? 0x00 : 0xff);
    std::uint8_t *data = region->storage.data();
    impl_->regions.push_back(std::move(region));
    return data;
}

void Core::map_mmio(std::uint32_t base, std::uint32_t size, Bus &bus) {
    auto user = std::make_unique<Impl::MmioEntryUser>();
    user->base = base;
    user->size = size;
    user->bus = &bus;
    impl_->mmio_users.push_back(std::move(user));
}

void Core::set_write_protect_handler(WriteProtectHandler handler) {
    impl_->write_protect = std::move(handler);
}

void Core::defer(std::function<void()> action) { impl_->deferred.push_back(std::move(action)); }

std::uint8_t *Core::host_pointer(std::uint32_t address, std::size_t length) {
    for (auto &region : impl_->regions) {
        if (address >= region->base &&
            static_cast<std::uint64_t>(address) + length <=
                static_cast<std::uint64_t>(region->base) + region->size)
            return region->storage.data() + (address - region->base);
    }
    return nullptr;
}

bool Core::read_memory(std::uint32_t address, void *out, std::size_t length) {
    if (const std::uint8_t *data = host_pointer(address, length)) {
        std::memcpy(out, data, length);
        return true;
    }
    return uc_ && uc_mem_read(uc_, address, out, length) == UC_ERR_OK;
}

bool Core::write_memory(std::uint32_t address, const void *data, std::size_t length) {
    if (std::uint8_t *target = host_pointer(address, length)) {
        std::memcpy(target, data, length);
        return true;
    }
    return uc_ && uc_mem_write(uc_, address, data, length) == UC_ERR_OK;
}

void Core::reset(std::uint32_t vector_table) {
    impl_->build();
    impl_->event = false;
    impl_->deferred.clear();
    impl_->st_ctrl = impl_->st_reload = impl_->st_value = 0;
    impl_->st_countflag = false;
    impl_->st_cycle = cycles_;
    impl_->dwt_ctrl = 0;
    sleeping_ = false;
    std::uint32_t vector[2] = {0, 0};
    read_memory(vector_table, vector, sizeof vector);
    if (vector_table != 0)
        uc_arm_nvic(uc_, UC_ARM_NVIC_SCS_WRITE, 0xd08, &vector_table, 4);
    std::uint32_t sp = vector[0];
    std::uint32_t pc = vector[1] & ~1u;
    uc_reg_write(uc_, UC_ARM_REG_SP, &sp);
    uc_reg_write(uc_, UC_ARM_REG_PC, &pc);
    impl_->service_deadline = 0; // service once before the first instruction
    impl_->deadline = 0;
    impl_->reset_pending = false;
    last_instruction_pc_ = 0;
    msp_trace_initialized_ = psp_trace_initialized_ = false;
}

void Core::request_reset() {
    impl_->reset_pending = true;
    if (uc_) {
        impl_->cause = Cause::Reset;
        uc_emu_stop(uc_);
    }
}

void Core::set_vtor(std::uint32_t address) {
    if (uc_)
        uc_arm_nvic(uc_, UC_ARM_NVIC_SCS_WRITE, 0xd08, &address, 4);
}

void Core::invalidate_code() {
    if (uc_)
        uc_ctl_flush_tb(uc_);
}

void Core::set_service_handler(ServiceHandler handler) { impl_->service = std::move(handler); }

void Core::request_service(std::uint64_t at_cycle) noexcept {
    // Only ever pulls the deadline in; the run loop recomputes it exactly.
    if (at_cycle < impl_->deadline)
        impl_->deadline = std::max(at_cycle, cycles_ + 1);
    if (at_cycle < impl_->service_deadline)
        impl_->service_deadline = at_cycle;
}

std::uint64_t Core::cycles_to_ns(std::uint64_t cycles) const noexcept {
    const std::uint64_t hz = config_.clock_hz;
    if (cycles >= clock_epoch_cycles_) {
        const std::uint64_t elapsed = cycles - clock_epoch_cycles_;
        return clock_epoch_ns_ + elapsed / hz * 1000000000ull +
               elapsed % hz * 1000000000ull / hz;
    }
    const std::uint64_t elapsed = clock_epoch_cycles_ - cycles;
    const std::uint64_t ns = elapsed / hz * 1000000000ull +
                             elapsed % hz * 1000000000ull / hz;
    return ns > clock_epoch_ns_ ? 0 : clock_epoch_ns_ - ns;
}

std::uint64_t Core::ns_to_cycles(std::uint64_t ns) const noexcept {
    const std::uint64_t hz = config_.clock_hz;
    if (ns >= clock_epoch_ns_) {
        const std::uint64_t elapsed = ns - clock_epoch_ns_;
        return clock_epoch_cycles_ + elapsed / 1000000000ull * hz +
               elapsed % 1000000000ull * hz / 1000000000ull;
    }
    const std::uint64_t elapsed = clock_epoch_ns_ - ns;
    const std::uint64_t cycles = elapsed / 1000000000ull * hz +
                                 elapsed % 1000000000ull * hz / 1000000000ull;
    return cycles > clock_epoch_cycles_ ? 0 : clock_epoch_cycles_ - cycles;
}

void Core::set_clock_hz(std::uint32_t clock_hz) {
    if (clock_hz == 0)
        throw std::invalid_argument("Cortex-M clock must be nonzero");
    if (clock_hz == config_.clock_hz)
        return;
    clock_epoch_ns_ = cycles_to_ns(cycles_);
    clock_epoch_cycles_ = cycles_;
    config_.clock_hz = clock_hz;
}

void Core::advance_to(std::uint64_t cycle) noexcept {
    if (cycle > cycles_)
        cycles_ = cycle;
}

void Core::set_irq_line(unsigned irq, bool level) {
    if (!uc_)
        return;
    std::uint32_t value = level ? 1 : 0;
    uc_arm_nvic(uc_, UC_ARM_NVIC_SET_IRQ, 16 + irq, &value, 0);
}

bool Core::interrupts_masked() {
    std::uint32_t value = 0;
    if (uc_)
        uc_reg_read(uc_, UC_ARM_REG_PRIMASK, &value);
    return (value & 1u) != 0;
}

void Core::set_interrupts_masked(bool masked) {
    if (!uc_)
        return;
    std::uint32_t value = masked ? 1u : 0u;
    uc_reg_write(uc_, UC_ARM_REG_PRIMASK, &value);
}

bool Core::interrupt_pending() {
    std::uint32_t pending = 0;
    uc_arm_nvic(uc_, UC_ARM_NVIC_PENDING, 0, &pending, 0);
    return pending != 0;
}

namespace {
int arm_register(unsigned index) {
    switch (index) {
    case 13: return UC_ARM_REG_SP;
    case 14: return UC_ARM_REG_LR;
    case 15: return UC_ARM_REG_PC;
    default: return UC_ARM_REG_R0 + static_cast<int>(index);
    }
}
} // namespace

std::uint32_t Core::reg(unsigned index) {
    std::uint32_t value = 0;
    uc_reg_read(uc_, arm_register(index), &value);
    return value;
}

void Core::set_reg(unsigned index, std::uint32_t value) {
    uc_reg_write(uc_, arm_register(index), &value);
}

std::uint32_t Core::pc() {
    std::uint32_t value = 0;
    uc_reg_read(uc_, UC_ARM_REG_PC, &value);
    return value;
}

RunResult Core::run(std::uint64_t until_cycle) {
    Impl &impl = *impl_;
    RunResult result;
    if (!uc_)
        reset();

    while (cycles_ < until_cycle) {
        if (impl.service_deadline <= cycles_)
            impl.run_service();
        if (impl.reset_pending) {
            impl.reset_pending = false;
            result.reason = StopReason::Reset;
            result.message = "reset requested";
            return result;
        }

        if (sleeping_) {
            if ((impl.sleep_wfe && impl.event) || interrupt_pending()) {
                if (impl.sleep_wfe)
                    impl.event = false;
                sleeping_ = false;
            } else {
                // Nothing can happen before the next scheduled event.
                const std::uint64_t target = std::min(until_cycle, impl.service_deadline);
                cycles_ = std::max(cycles_ + 1, target);
                continue;
            }
        }

        impl.deadline = std::min(until_cycle, impl.service_deadline);
        if (impl.deadline <= cycles_)
            impl.deadline = cycles_ + 1;
        impl.cause = Cause::None;
        impl.invalid_address = 0;
        impl.invalid_value = 0;
        impl.invalid_size = 0;
        impl.invalid_write = false;

        std::uint32_t pc_value = pc() | 1u;
        const uc_err error = uc_emu_start(uc_, pc_value, 0x100000000ull, 0, 0);

        switch (impl.cause) {
        case Cause::Deadline:
            break;
        case Cause::StepLimit:
            result.reason = StopReason::StepLimit;
            return result;
        case Cause::Hint: {
            std::uint32_t next = (pc() & ~1u) + impl.hint_size;
            uc_reg_write(uc_, UC_ARM_REG_PC, &next);
            ++instructions_;
            cycles_ += cpi_;
            switch (impl.hint) {
            case Hint::Wfi:
                sleeping_ = true;
                impl.sleep_wfe = false;
                break;
            case Hint::Wfe:
                if (impl.event) {
                    impl.event = false;
                } else {
                    sleeping_ = true;
                    impl.sleep_wfe = true;
                }
                break;
            case Hint::Sev:
            case Hint::Sevl:
                impl.event = true;
                break;
            default:
                break;
            }
            break;
        }
        case Cause::Halted:
            result.reason = StopReason::Halted;
            result.message = "BKPT at pc 0x" + [&] {
                char buffer[16];
                std::snprintf(buffer, sizeof buffer, "%x", pc());
                return std::string(buffer);
            }();
            return result;
        case Cause::Reset:
            impl.reset_pending = false;
            result.reason = StopReason::Reset;
            result.message = "system reset requested";
            return result;
        case Cause::None:
            if (error != UC_ERR_OK) {
                result.reason = StopReason::Fault;
                char buffer[192];
                if (impl.invalid_size != 0) {
                    std::snprintf(buffer, sizeof buffer,
                                  "%s at pc 0x%x (invalid %s 0x%llx size=%u value=0x%llx)",
                                  uc_strerror(error), pc(), impl.invalid_write ? "write" : "read",
                                  static_cast<unsigned long long>(impl.invalid_address),
                                  impl.invalid_size,
                                  static_cast<unsigned long long>(impl.invalid_value));
                } else {
                    std::snprintf(buffer, sizeof buffer, "%s at pc 0x%x", uc_strerror(error), pc());
                }
                result.message = buffer;
                return result;
            }
            result.reason = StopReason::Fault;
            result.message = "the engine stopped without a reason at pc 0x" + [&] {
                char buffer[16];
                std::snprintf(buffer, sizeof buffer, "%x", pc());
                return std::string(buffer);
            }();
            return result;
        }
    }
    return result;
}

} // namespace cortexm
