#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct uc_struct;

namespace riscv {

// Peripheral window: a chip model implements this for each MMIO range it
// maps. Addresses are absolute; `size` is 1, 2 or 4 bytes.
class Bus {
public:
    virtual ~Bus() = default;
    virtual std::uint32_t read(std::uint32_t address, unsigned size) = 0;
    virtual void write(std::uint32_t address, std::uint32_t value, unsigned size) = 0;
};

enum class StopReason {
    Deadline,  // the requested cycle was reached
    StepLimit, // the instruction limit was reached
    Halted,    // ebreak executed
    Fault,     // the engine reported an error (unmapped access, ...)
    Reset,     // a peripheral asked for a system reset
};

struct RunResult {
    StopReason reason{StopReason::Deadline};
    std::string message;
};

// A 32-bit RISC-V hart (RV32IMAC, no FPU: what the ESP32-C6 HP core and the
// ESP32-H2 are) on the Unicorn fork's RISC-V backend: memory map, WFI sleep,
// the interrupt line, and the cycle clock a chip model hangs its peripherals
// on. Traps (ecall, faults) run in the guest through mtvec; the interrupt
// controller is the chip model's, which tells the hart the winner with
// set_interrupt().
//
// Time is counted in core cycles. Without a pipeline model every instruction
// costs `cycles_per_instruction` (inferred, not measured).
class Core {
public:
    struct Config {
        std::uint32_t clock_hz{160000000};
        // The default model has no FPU (the SiFive E31). With `fpu` the hart is the SiFive U34 model, which
        // adds F and D (and the S mode and an MMU that firmware in M mode leaves off).
        bool fpu{false};
    };

    explicit Core(const Config &config);
    ~Core();
    Core(const Core &) = delete;
    Core &operator=(const Core &) = delete;

    // ---- memory ----------------------------------------------------------
    // Plain memory backed by a host buffer the core owns. A store to
    // non-writable memory (flash) faults.
    std::uint8_t *map_memory(std::uint32_t base, std::uint32_t size, bool writable);
    void map_mmio(std::uint32_t base, std::uint32_t size, Bus &bus);

    // Host-side access to mapped memory without side effects. Returns nullptr
    // outside the host-backed regions.
    std::uint8_t *host_pointer(std::uint32_t address, std::size_t length = 1);
    // Reads through the memory map; MMIO ranges call the chip model.
    bool read_memory(std::uint32_t address, void *out, std::size_t length);
    bool write_memory(std::uint32_t address, const void *data, std::size_t length);

    // ---- execution -------------------------------------------------------
    // Builds the engine and starts at `entry` in machine mode with interrupts
    // off. Memory must be mapped first.
    void reset(std::uint32_t entry);
    // Runs until `until_cycle` (absolute core cycles) or a stop condition.
    // While the hart sleeps in WFI the clock jumps to the next event.
    RunResult run(std::uint64_t until_cycle);

    // Stops the run (StopReason::Halted, "breakpoint") before the instruction at `address` executes.
    void add_breakpoint(std::uint32_t address) { breakpoints_.push_back(address); }
    // Called with the pc before every instruction (debugging: slows the run down). Empty disables it.
    void set_trace_hook(std::function<void(std::uint32_t pc)> hook) { trace_hook_ = std::move(hook); }

    // Calls `callback(pc, address, size, value)` for every store to [begin, end),
    // from inside the running block (debugging aid; it slows the hart down).
    using WatchCallback = std::function<void(std::uint32_t pc, std::uint32_t address, unsigned size,
                                             std::uint32_t value)>;
    void watch_writes(std::uint32_t begin, std::uint32_t end, WatchCallback callback);

    // Stops a run once this many instructions have executed in total (0: no limit).
    void set_instruction_limit(std::uint64_t limit) noexcept { instruction_limit_ = limit; }

    std::uint64_t cycles() const noexcept { return cycles_; }
    std::uint64_t instructions() const noexcept { return instructions_; }
    std::uint32_t clock_hz() const noexcept { return config_.clock_hz; }
    std::uint64_t cycles_to_ns(std::uint64_t cycles) const noexcept;
    std::uint64_t ns_to_cycles(std::uint64_t ns) const noexcept;
    // Moves the clock forward without executing (a chip model's start offset).
    void advance_to(std::uint64_t cycle) noexcept;
    bool sleeping() const noexcept { return sleeping_; }

    // Chip-model service: called whenever the clock reaches the deadline it
    // returned last (and once at the start of every run). It advances the
    // peripherals to `now` and returns the next cycle at which it needs to run
    // again (UINT64_MAX when idle). A model that schedules something sooner
    // from an MMIO write calls `request_service`.
    using ServiceHandler = std::function<std::uint64_t(std::uint64_t now)>;
    void set_service_handler(ServiceHandler handler);
    void request_service(std::uint64_t at_cycle) noexcept;

    // Stops the current or next run with StopReason::Reset.
    void request_reset();
    // Drops translated code after memory it was translated from changed behind
    // the CPU's back. Safe to call from a peripheral write: the flush happens
    // before the next instruction.
    void invalidate_code();

    // The interrupt the chip's controller picked, or -1 for none. It is taken
    // as mcause = 1<<31 | cause once mstatus.MIE allows it, and stays asserted
    // until it is changed (level behaviour). It also wakes a sleeping hart,
    // whatever MIE says. The hart takes it at the next translation block
    // boundary, not mid-block.
    void set_interrupt(int cause);
    int interrupt() const noexcept { return interrupt_; }

    // Serves vendor CSRs first..last from the host. The handler returns false for
    // an illegal access; for a read it fills `value`, for a write it receives the
    // new value. It runs inside translated code, so it may read the core's cycle
    // and instruction counters (kept exact per instruction) and nothing else of the engine.
    using CsrHandler = std::function<bool(unsigned csr, bool write, std::uint32_t &value)>;
    void set_csr_handler(unsigned first, unsigned last, CsrHandler handler);

    // CLIC (mtvec mode 3): the host's choice of the interrupt the hart takes. `pick(mil, thresh, level, claim)`
    // returns the number of the highest pending interrupt whose level is above `mil` and `thresh` (its
    // level in `level`), or -1. claim is true when mnxti takes it. The hart keeps mintstatus.mil, mintthresh
    // and mtvt itself (see uc_riscv_clic()). Kept across resets.
    using ClicPick = std::function<int(std::uint32_t mil, std::uint32_t thresh, std::uint32_t &level, bool claim)>;
    void set_clic(ClicPick pick);

    // Registers (x0..x31, 32 is pc), for tests and tooling.
    std::uint32_t reg(unsigned index);
    void set_reg(unsigned index, std::uint32_t value);
    std::uint32_t pc();

    // Addresses of the last executed instructions, oldest first (a debugging aid
    // for faults: the pc register is only exact at block boundaries).
    std::vector<std::uint32_t> recent_pcs() const;

    // Escape hatch for tests (CSRs are read with uc_reg_read(UC_RISCV_REG_M...)).
    uc_struct *engine() noexcept { return uc_; }

    // The cost model; the default of 1 keeps cycles equal to instructions.
    void set_cycles_per_instruction(std::uint32_t cycles) noexcept { cpi_ = cycles; }

private:
    struct Region;
    struct Impl;
    friend struct Impl;

    Config config_;
    uc_struct *uc_{nullptr};
    std::unique_ptr<Impl> impl_;
    std::uint64_t cycles_{0};
    std::uint64_t instructions_{0};
    std::uint64_t instruction_limit_{0};
    std::uint32_t cpi_{1};
    bool sleeping_{false};
    int interrupt_{-1};
    std::vector<std::uint32_t> breakpoints_;
    std::function<void(std::uint32_t)> trace_hook_;
};

} // namespace riscv
