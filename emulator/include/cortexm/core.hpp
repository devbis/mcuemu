#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct uc_struct;

namespace cortexm {

// M33 is ARMv8-M Mainline without the Security Extension (one security state).
enum class Model { M3, M4, M33 };

// Peripheral window: a chip model implements this for each MMIO range it
// maps. Addresses are absolute; `size` is 1, 2 or 4 bytes.
class Bus {
public:
    virtual ~Bus() = default;
    virtual std::uint32_t read(std::uint32_t address, unsigned size) = 0;
    virtual void write(std::uint32_t address, std::uint32_t value, unsigned size) = 0;
};

enum class StopReason {
    Deadline, // the requested cycle was reached
    StepLimit, // the instruction limit was reached
    Halted,   // BKPT executed
    Fault,    // the engine reported an error (unmapped access, ...)
    Reset,    // AIRCR.SYSRESETREQ
};

struct RunResult {
    StopReason reason{StopReason::Deadline};
    std::string message;
};

// A Cortex-M3/M4 core on the Unicorn fork's ARM backend: memory map, the NVIC
// and System Control Space (in the engine), SysTick and the DWT cycle counter
// (here), WFI/WFE, and the cycle clock that a chip model hangs its peripherals
// on.
//
// Time is counted in core cycles. Without a cycle-accurate pipeline model every
// instruction costs `cycles_per_instruction` (inferred, not measured).
class Core {
public:
    struct Config {
        Model model{Model::M4};
        std::uint32_t clock_hz{64000000};
        unsigned irq_count{48};       // external interrupt lines
        unsigned priority_bits{3};
    };

    explicit Core(const Config &config);
    ~Core();
    Core(const Core &) = delete;
    Core &operator=(const Core &) = delete;

    // ---- memory ----------------------------------------------------------
    // Plain memory backed by a host buffer the core owns. Writes by the CPU
    // are refused when `writable` is false and reported to the write-protect
    // handler (flash).
    std::uint8_t *map_memory(std::uint32_t base, std::uint32_t size, bool writable);
    void map_mmio(std::uint32_t base, std::uint32_t size, Bus &bus);
    // Called for a CPU store to non-writable memory; returning true lets the
    // store complete (the chip decides what it means), false faults.
    using WriteProtectHandler =
        std::function<bool(std::uint32_t address, unsigned size, std::uint32_t value)>;
    void set_write_protect_handler(WriteProtectHandler handler);
    // Runs `action` before the next instruction executes (to fix up after a
    // store that the write-protect handler allowed).
    void defer(std::function<void()> action);

    // Host-side access to mapped memory (flash, RAM) without side effects.
    // Returns nullptr outside the host-backed regions.
    std::uint8_t *host_pointer(std::uint32_t address, std::size_t length = 1);
    // Reads through the memory map; MMIO ranges call the chip model.
    bool read_memory(std::uint32_t address, void *out, std::size_t length);
    bool write_memory(std::uint32_t address, const void *data, std::size_t length);

    // ---- execution -------------------------------------------------------
    // Loads SP and PC from the vector table at `vector_table` (VTOR is set to
    // it too) and clears the NVIC. A chip with a boot loader in front of the
    // application starts the application directly this way.
    void reset(std::uint32_t vector_table = 0);
    // Runs until `until_cycle` (absolute core cycles) or a stop condition.
    // While the core sleeps in WFI/WFE the clock jumps to the next event.
    RunResult run(std::uint64_t until_cycle);

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

    // Stops the current or next run with StopReason::Reset (watchdog, system
    // reset request from a peripheral). The caller restarts with reset().
    void request_reset();
    // Drops translated code after flash was rewritten behind the CPU's back.
    void invalidate_code();

    // External interrupt line `irq` (0-based; exception number is 16 + irq).
    void set_irq_line(unsigned irq, bool level);
    // True when an enabled exception is pending (a sleeping core wakes).
    bool interrupt_pending();

    // Registers, for tests and tooling.
    std::uint32_t reg(unsigned index);
    void set_reg(unsigned index, std::uint32_t value);
    std::uint32_t pc();

    // Escape hatch for tests.
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
};

} // namespace cortexm
