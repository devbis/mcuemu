#pragma once

// CC2530 two-wire debug interface (SWRU191F Chapter 3): debug-mode entry,
// the DC/DD bit-serial protocol on P2.2/P2.1, the Table 3-1 command set, the
// debug configuration and status bytes, four hardware breakpoints, DBGLOCK
// and the DBGDATA register with DMA trigger DBG_BW. Not to be confused with
// the sim YAML `debugger:` helper (Cc2530F256::set_breakpoint), which is the
// emulator's own breakpoint and has nothing to do with this chip block.

#include "cc2530_io.hpp"
#include "cc2530_soc.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace mcs51emu::cc2530 {

class Debug final : public Peripheral {
public:
    // Table 3-2 (reserved bits read back 0).
    static constexpr std::uint8_t ConfigReset = 0x26;
    static constexpr std::uint8_t ConfigMask = 0x2e;
    static constexpr std::uint8_t ConfigSoftPowerMode = 0x20;
    static constexpr std::uint8_t ConfigTimersOff = 0x08;
    static constexpr std::uint8_t ConfigDmaPause = 0x04;
    static constexpr std::uint8_t ConfigTimerSuspend = 0x02;
    // Table 3-3.
    static constexpr std::uint8_t StatusChipEraseBusy = 0x80;
    static constexpr std::uint8_t StatusPconIdle = 0x40;
    static constexpr std::uint8_t StatusCpuHalted = 0x20;
    static constexpr std::uint8_t StatusPmActive = 0x10;
    static constexpr std::uint8_t StatusHaltStatus = 0x08;
    static constexpr std::uint8_t StatusDebugLocked = 0x04;
    static constexpr std::uint8_t StatusOscillatorStable = 0x02;
    static constexpr std::uint8_t StatusStackOverflow = 0x01;

    // Model-defined host port rate: one byte every 128 cycles of the 32 MHz
    // base, i.e. DC at 2 MHz (8 bits x 16 cycles), about the CC Debugger's
    // rate. Only the host port uses it; the wire runs at the host's edges.
    static constexpr std::uint64_t HostByteCycles = 128;

    struct Breakpoint {
        bool enabled{};
        std::uint8_t bank{};
        std::uint16_t address{};
    };
    struct Response {
        std::vector<std::uint8_t> bytes;
        bool accepted{};
        std::uint64_t at{};
    };

    explicit Debug(Bus &bus);
    const char *name() const override { return "DBG"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;
    // The host (or a wire peer) can always act on a chip in debug mode.
    bool may_interrupt() const override { return active_ || !inbox_.empty(); }

    // Encoding helpers (Table 3-1). input_length() counts every byte of the
    // command including the command byte; for BURST_WRITE it needs the
    // first two bytes (`second`) and returns 2 + burst length. Undefined
    // commands (and DEBUG_INSTR with yy = 0) are one byte with one output
    // byte (model-defined).
    static unsigned input_length(std::uint8_t command, std::uint8_t second = 0);
    static unsigned output_length(std::uint8_t command);

    // ---- debug mode ----
    // SWRU191F 3.1: entered at the release of RESET_N after two DC falling
    // edges; the CPU is halted at PC 0x0000.
    void enter();
    // Called by the core before every reset, ahead of the peripheral
    // resets: a power-on or RESET_N reset leaves debug mode, a watchdog or
    // clock-loss reset keeps it (model-defined).
    void before_reset(ResetCause cause);
    bool active() const noexcept { return active_; }
    bool halted() const noexcept { return active_ && halted_; }
    bool locked() const noexcept { return locked_; }
    std::uint8_t config() const noexcept { return config_; }
    // Table 3-3 status byte (side-effect free).
    std::uint8_t status() const;
    // The CPU must not execute (halted and no STEP_INSTR pending).
    bool cpu_held() const noexcept { return active_ && halted_ && !step_pending_; }
    bool stepping() const noexcept { return active_ && step_pending_; }
    // DC falling edges seen while RESET_N was held low since the last reset.
    unsigned entry_edges() const noexcept { return entry_edges_; }

    // ---- hardware breakpoints (3.3.3) ----
    const std::array<Breakpoint, 4> &breakpoints() const noexcept { return breakpoints_; }
    // True when an enabled breakpoint matches the instruction about to
    // execute at logical `pc` with FMAP `fmap`, and the breakpoints are not
    // suppressed (stepping, or the first instruction after RESUME).
    bool breakpoint_at(std::uint16_t pc, std::uint8_t fmap) const;
    bool breakpoints_armed() const noexcept;
    void breakpoint_hit();
    // The CPU executed at least one instruction (or interrupt entry).
    void progressed() noexcept { skip_breakpoint_once_ = false; }
    void step_completed(std::uint8_t accumulator);

    // ---- pads ----
    // P2.1/P2.2 levels changed (Io::refresh); `at` is the virtual time.
    void pads_changed(std::uint64_t at);
    // What the chip drives on P2.1 (DD) while in debug mode.
    PinDrive dd_drive() const noexcept { return dd_drive_; }

    // ---- host port (command-level access, model-defined) ----
    // Queues one complete command; its bytes arrive HostByteCycles apart
    // starting at max(now, end of the previous response). Throws
    // std::logic_error when not in debug mode or a host command is
    // outstanding, std::invalid_argument for a malformed command.
    void host_submit(const std::vector<std::uint8_t> &bytes);
    bool host_busy() const noexcept { return host_busy_; }
    // Time at which the last byte of the outstanding command is processed.
    std::uint64_t host_due() const noexcept { return host_due_; }
    // End of the response shift-out of the last completed host command.
    std::uint64_t host_port_free_at() const noexcept { return host_port_free_at_; }
    std::optional<Response> host_take();

private:
    enum class Source : std::uint8_t { Wire, Host };
    enum class WireState : std::uint8_t { Input, Wait, Dummy, Output };
    struct Byte {
        std::uint8_t value{};
        std::uint64_t at{};
        Source source{};
    };

    void process_byte(const Byte &byte);
    void drain(std::uint64_t limit);
    void execute(std::uint64_t at);
    bool accepted(std::uint8_t command) const;
    void note_issued(std::uint8_t command);
    void respond(std::vector<std::uint8_t> bytes, bool accepted, std::uint64_t at);
    void leave(std::uint64_t at, bool resetting);
    void apply_config();
    bool stack_hit() const;
    void check_stack();
    // Samples STACK_OVERFLOW, then returns status().
    std::uint8_t status_byte();
    void set_dd(PinDrive drive);
    void latch_ready();
    void drive_output_bit();
    void wire_reset();

    // Debug mode and CPU control.
    bool active_ = false;
    bool halted_ = false;
    bool halt_by_breakpoint_ = false;
    bool step_pending_ = false;
    bool skip_breakpoint_once_ = false;
    bool locked_ = false;
    // CHIP_ERASE is accepted only until another command than READ_STATUS
    // has been issued since debug-mode entry (Table 3-1).
    bool erase_enabled_ = false;
    bool stack_overflow_ = false;
    std::uint8_t stack_shadow_ = 0;
    std::uint8_t config_ = ConfigReset;
    std::array<Breakpoint, 4> breakpoints_{};
    unsigned entry_edges_ = 0;

    // Command parser (shared by the wire and the host port).
    std::deque<Byte> inbox_;
    std::vector<std::uint8_t> command_; // first bytes (all but BURST_WRITE data)
    unsigned received_ = 0;
    unsigned expected_ = 0;
    Source source_ = Source::Wire;
    bool burst_accepted_ = false;
    bool waiting_for_step_ = false;

    // Wire (SWRU191F 3.2).
    WireState wire_ = WireState::Input;
    bool last_dc_ = true;
    unsigned bits_ = 0;
    std::uint8_t shift_ = 0;
    std::uint8_t wire_command_ = 0; // first byte of the command being shifted in
    unsigned wire_received_ = 0;
    unsigned wire_expected_ = 0;
    std::optional<std::vector<std::uint8_t>> wire_response_;
    std::vector<std::uint8_t> output_;
    unsigned output_bit_ = 0;
    unsigned dummy_bits_ = 0;
    PinDrive dd_drive_ = PinDrive::HiZ;

    // Host port.
    bool host_busy_ = false;
    std::uint64_t host_due_ = 0;
    std::uint64_t host_port_free_at_ = 0;
    std::optional<Response> host_response_;
};

} // namespace mcs51emu::cc2530
