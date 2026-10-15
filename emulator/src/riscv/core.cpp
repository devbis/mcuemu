#include "riscv/core.hpp"

#include <unicorn/unicorn.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace riscv {

namespace {

constexpr std::uint64_t Never = std::numeric_limits<std::uint64_t>::max();

constexpr std::uint32_t WfiInstruction = 0x10500073;
constexpr std::uint32_t EbreakInstruction = 0x00100073;
constexpr std::uint16_t CEbreakInstruction = 0x9002;

// Instructions the core must not hand to the engine: WFI puts the model to
// sleep (it would halt the engine), ebreak stops the run for the host.
enum class Special { None, Wfi, Ebreak };

} // namespace

struct Core::Region {
    std::uint32_t base;
    std::uint32_t size;
    bool writable;
    std::vector<std::uint8_t> storage;
};

enum class Cause { None, Deadline, StepLimit, Special, Reset, Flush, Breakpoint };

struct Core::Impl {
    Core &core;
    std::vector<std::unique_ptr<Region>> regions;
    ServiceHandler service;

    struct MmioUser {
        std::uint32_t base;
        std::uint32_t size;
        Bus *bus;
    };
    std::vector<std::unique_ptr<MmioUser>> mmio_users;

    uc_hook hook{};
    uc_hook invalid_hook{};
    std::uint64_t invalid_address{0};
    bool invalid_write{false};
    std::uint64_t deadline{0};
    std::uint64_t service_deadline{Never};
    Cause cause{Cause::None};
    Special special{Special::None};
    unsigned special_size{0};
    bool reset_pending{false};
    bool flush_pending{false};
    bool skip_breakpoint{false}; // resuming at a breakpoint executes it once
    static constexpr unsigned RingSize = 1024;
    std::uint32_t ring[RingSize]{};
    std::uint64_t ring_count{0};

    explicit Impl(Core &c) : core(c) {}

    static void check(uc_err error, const char *what) {
        if (error != UC_ERR_OK)
            throw std::runtime_error(std::string("riscv: ") + what + ": " + uc_strerror(error));
    }

    static std::uint64_t mmio_read(uc_engine *, std::uint64_t offset, unsigned size, void *user) {
        auto *entry = static_cast<MmioUser *>(user);
        return entry->bus->read(entry->base + static_cast<std::uint32_t>(offset), size);
    }
    static void mmio_write(uc_engine *, std::uint64_t offset, unsigned size, std::uint64_t value,
                           void *user) {
        auto *entry = static_cast<MmioUser *>(user);
        entry->bus->write(entry->base + static_cast<std::uint32_t>(offset),
                          static_cast<std::uint32_t>(value), size);
    }
    struct Watch {
        std::uint32_t begin, end;
        WatchCallback callback;
    };
    std::vector<std::unique_ptr<Watch>> watches;
    std::vector<uc_hook> watch_hooks;

    static void watch_hook(uc_engine *, uc_mem_type, std::uint64_t address, int size,
                           std::int64_t value, void *user) {
        auto *watch = static_cast<Watch *>(user);
        std::uint32_t pc = 0;
        // The pc register is only exact at block boundaries; the hook order gives
        // the last executed instruction instead.
        (void)pc;
        watch->callback(0, static_cast<std::uint32_t>(address), static_cast<unsigned>(size),
                        static_cast<std::uint32_t>(value));
    }

    CsrHandler csr_handler;
    unsigned csr_first{1}, csr_last{0};

    static int csr_thunk(unsigned csr, int is_write, std::uint32_t *value, void *user) {
        auto *self = static_cast<Impl *>(user);
        // Not served by the host: the core's own CSR (or illegal instruction) takes over.
        return self->csr_handler && self->csr_handler(csr, is_write != 0, *value) ? 0 : 1;
    }

    static void code_hook(uc_engine *, std::uint64_t address, std::uint32_t size, void *user) {
        static_cast<Impl *>(user)->on_instruction(static_cast<std::uint32_t>(address), size);
    }

    static bool invalid_access(uc_engine *, uc_mem_type type, std::uint64_t address, int, std::int64_t,
                               void *user) {
        auto *self = static_cast<Impl *>(user);
        self->invalid_address = address;
        self->invalid_write = type == UC_MEM_WRITE_UNMAPPED || type == UC_MEM_WRITE_PROT;
        return false; // not handled: the run stops with the engine's error
    }

    void build() {
        if (core.uc_) {
            uc_close(core.uc_);
            core.uc_ = nullptr;
        }
        uc_engine *uc = nullptr;
        if (uc_open(UC_ARCH_RISCV, UC_MODE_RISCV32, &uc) != UC_ERR_OK)
            throw std::runtime_error("cannot create the RISC-V engine");
        core.uc_ = uc;
        // SiFive E31 is RV32IMAC with M and U modes and no FPU, like the HP core
        // of the ESP32-C6 and the ESP32-H2 (the default model also has F and D).
        check(uc_ctl_set_cpu_model(uc, UC_CPU_RISCV32_SIFIVE_E31), "set CPU model");
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
        check(uc_hook_add(uc, &hook, UC_HOOK_CODE, reinterpret_cast<void *>(&Impl::code_hook), this,
                          1, 0),
              "add code hook");
        for (auto &watch : watches) {
            uc_hook handle{};
            check(uc_hook_add(uc, &handle, UC_HOOK_MEM_WRITE,
                              reinterpret_cast<void *>(&Impl::watch_hook), watch.get(), watch->begin,
                              watch->end - 1),
                  "add write watch");
        }
        if (csr_handler)
            check(uc_riscv_csr_handler(uc, csr_first, csr_last, &Impl::csr_thunk, this),
                  "install the CSR handler");
        check(uc_hook_add(uc, &invalid_hook, UC_HOOK_MEM_INVALID,
                          reinterpret_cast<void *>(&Impl::invalid_access), this, 1, 0),
              "add access fault hook");
    }

    // ---- per-instruction hook ------------------------------------------
    void on_instruction(std::uint32_t address, unsigned size) {
        if (flush_pending) {
            // Translated code cannot be dropped from inside a running block (a
            // peripheral write is a callback of one): stop before this
            // instruction, flush and resume.
            cause = Cause::Flush;
            uc_emu_stop(core.uc_);
            return;
        }
        if (core.cycles_ >= deadline) {
            cause = Cause::Deadline;
            uc_emu_stop(core.uc_);
            return;
        }
        if (core.instruction_limit_ != 0 && core.instructions_ >= core.instruction_limit_) {
            cause = Cause::StepLimit;
            uc_emu_stop(core.uc_);
            return;
        }
        if (!core.breakpoints_.empty() && !skip_breakpoint &&
            std::find(core.breakpoints_.begin(), core.breakpoints_.end(), address) !=
                core.breakpoints_.end()) {
            cause = Cause::Breakpoint;
            uc_emu_stop(core.uc_);
            return;
        }
        skip_breakpoint = false;
        if (core.trace_hook_)
            core.trace_hook_(address);
        // Decoded from the host copy of the code (flash or RAM).
        if (const std::uint8_t *code = core.host_pointer(address, size)) {
            std::uint32_t word = code[0] | (code[1] << 8);
            if (size == 4)
                word |= static_cast<std::uint32_t>(code[2] << 16 | code[3] << 24);
            Special found = Special::None;
            if (size == 4 && word == WfiInstruction)
                found = Special::Wfi;
            else if ((size == 4 && word == EbreakInstruction) ||
                     (size == 2 && word == CEbreakInstruction))
                found = Special::Ebreak;
            if (found != Special::None) {
                special = found;
                special_size = size;
                cause = Cause::Special;
                uc_emu_stop(core.uc_);
                return;
            }
        }
        ring[ring_count++ % RingSize] = address;
        ++core.instructions_;
        core.cycles_ += core.cpi_;
    }

    void run_service() {
        std::uint64_t next = Never;
        // A request_service() made while the pass runs lands in service_deadline
        // and must not be overwritten (same rule as cortexm::Core).
        service_deadline = Never;
        if (service)
            next = service(core.cycles_);
        next = std::min(next, service_deadline);
        service_deadline = next <= core.cycles_ ? core.cycles_ + 1 : next;
    }
};

// ---- Core ------------------------------------------------------------------

Core::Core(const Config &config) : config_(config), impl_(std::make_unique<Impl>(*this)) {}

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
    auto user = std::make_unique<Impl::MmioUser>();
    user->base = base;
    user->size = size;
    user->bus = &bus;
    impl_->mmio_users.push_back(std::move(user));
}

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
    for (auto &watch : impl_->watches)
        if (address < watch->end && address + length > watch->begin) // a host-side store (ROM stand-ins)
            watch->callback(0xffffffff, address, static_cast<unsigned>(length),
                            length >= 4 ? *static_cast<const std::uint32_t *>(data) : 0);
    if (std::uint8_t *target = host_pointer(address, length)) {
        std::memcpy(target, data, length);
        return true;
    }
    return uc_ && uc_mem_write(uc_, address, data, length) == UC_ERR_OK;
}

void Core::reset(std::uint32_t entry) {
    impl_->build();
    sleeping_ = false;
    interrupt_ = -1;
    std::uint32_t pc = entry;
    uc_reg_write(uc_, UC_RISCV_REG_PC, &pc);
    impl_->service_deadline = 0; // service once before the first instruction
    impl_->deadline = 0;
    impl_->reset_pending = false;
}

void Core::request_reset() {
    impl_->reset_pending = true;
    if (uc_) {
        impl_->cause = Cause::Reset;
        uc_emu_stop(uc_);
    }
}

void Core::invalidate_code() { impl_->flush_pending = true; }

void Core::watch_writes(std::uint32_t begin, std::uint32_t end, WatchCallback callback) {
    impl_->watches.push_back(std::make_unique<Impl::Watch>(Impl::Watch{begin, end, std::move(callback)}));
}

void Core::set_csr_handler(unsigned first, unsigned last, CsrHandler handler) {
    impl_->csr_first = first;
    impl_->csr_last = last;
    impl_->csr_handler = std::move(handler);
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
    return cycles / hz * 1000000000ull + cycles % hz * 1000000000ull / hz;
}

std::uint64_t Core::ns_to_cycles(std::uint64_t ns) const noexcept {
    const std::uint64_t hz = config_.clock_hz;
    return ns / 1000000000ull * hz + ns % 1000000000ull * hz / 1000000000ull;
}

void Core::advance_to(std::uint64_t cycle) noexcept {
    if (cycle > cycles_)
        cycles_ = cycle;
}

void Core::set_interrupt(int cause) {
    interrupt_ = cause < 0 ? -1 : cause;
    if (uc_)
        uc_riscv_irq(uc_, interrupt_);
}

std::uint32_t Core::reg(unsigned index) {
    std::uint32_t value = 0;
    uc_reg_read(uc_, index >= 32 ? UC_RISCV_REG_PC : UC_RISCV_REG_X0 + static_cast<int>(index),
                &value);
    return value;
}

void Core::set_reg(unsigned index, std::uint32_t value) {
    uc_reg_write(uc_, index >= 32 ? UC_RISCV_REG_PC : UC_RISCV_REG_X0 + static_cast<int>(index),
                 &value);
}

std::uint32_t Core::pc() { return reg(32); }

std::vector<std::uint32_t> Core::recent_pcs() const {
    std::vector<std::uint32_t> pcs;
    const std::uint64_t count = impl_->ring_count;
    for (std::uint64_t i = count > Impl::RingSize ? count - Impl::RingSize : 0; i < count; ++i)
        pcs.push_back(impl_->ring[i % Impl::RingSize]);
    return pcs;
}

RunResult Core::run(std::uint64_t until_cycle) {
    Impl &impl = *impl_;
    RunResult result;
    if (!uc_)
        throw std::logic_error("riscv::Core::run before reset");

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
            if (interrupt_ >= 0) {
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

        const std::uint32_t pc_value = pc();
        const uc_err error = uc_emu_start(uc_, pc_value, 0x100000000ull, 0, 0);

        switch (impl.cause) {
        case Cause::Deadline:
            break;
        case Cause::StepLimit:
            result.reason = StopReason::StepLimit;
            return result;
        case Cause::Special:
            if (impl.special == Special::Wfi) {
                set_reg(32, pc() + impl.special_size);
                ++instructions_;
                cycles_ += cpi_;
                sleeping_ = true;
                break;
            }
            result.reason = StopReason::Halted;
            {
                char buffer[48];
                std::snprintf(buffer, sizeof buffer, "ebreak at pc 0x%x", pc());
                result.message = buffer;
            }
            return result;
        case Cause::Breakpoint: {
            impl.skip_breakpoint = true;
            result.reason = StopReason::Halted;
            char buffer[48];
            std::snprintf(buffer, sizeof buffer, "breakpoint at pc 0x%x", pc());
            result.message = buffer;
            return result;
        }
        case Cause::Flush:
            impl.flush_pending = false;
            uc_ctl_flush_tb(uc_);
            break;
        case Cause::Reset:
            impl.reset_pending = false;
            result.reason = StopReason::Reset;
            result.message = "system reset requested";
            return result;
        case Cause::None: {
            result.reason = StopReason::Fault;
            char buffer[128];
            if (error != UC_ERR_OK)
                std::snprintf(buffer, sizeof buffer, "%s at pc 0x%x (address 0x%llx)",
                              uc_strerror(error), pc(),
                              static_cast<unsigned long long>(impl.invalid_address));
            else
                std::snprintf(buffer, sizeof buffer,
                              "the engine stopped without a reason at pc 0x%x", pc());
            result.message = buffer;
            return result;
        }
        }
    }
    return result;
}

} // namespace riscv
