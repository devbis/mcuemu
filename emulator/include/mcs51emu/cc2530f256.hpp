#ifndef MCS51EMU_CC2530F256_HPP
#define MCS51EMU_CC2530F256_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

struct uc_struct;

namespace mcuemu {
class VirtualAir;
}

namespace mcs51emu {

namespace cc2530 {
class Peripheral;
struct Soc;
} // namespace cc2530

// Active: CPU running. Idle: PCON.IDLE with SLEEPCMD.MODE = 0 (all clocks
// run, CPU stopped). Pm1-Pm3: PCON.IDLE with SLEEPCMD.MODE 1-3 (SWRU191F
// 4.1): Pm1/Pm2 stop the 32 MHz XOSC and 16 MHz RCOSC (the 32 kHz domain
// keeps running), Pm3 also stops the 32 kHz domain.
enum class Cc2530PowerMode : std::uint8_t { Active, Idle, Pm1, Pm2, Pm3 };
// SLEEPSTA.RST encoding (SWRU191F 4.3).
enum class Cc2530ResetCause : std::uint8_t { PowerOn = 0, External = 1, Watchdog = 2, ClockLoss = 3 };
// What the chip does to a pad.
enum class Cc2530PinDrive : std::uint8_t { HiZ, Low, High, PullUp, PullDown };

struct Cc2530RunResult {
    std::size_t instructions{};
    std::uint16_t pc{};
    std::uint8_t fmap{};
    std::uint64_t cycles{};
    bool step_limit_reached{};
    bool break_hit{};
    // The CPU is in the power mode entered through PCON.IDLE: no enabled
    // interrupt request is pending and no scheduled event within the cycle
    // limit has woken it (see Cc2530F256::run).
    bool sleeping{};
    std::string error; // CPU exception or unsupported SoC behavior
};

struct Cc2530Registers {
    std::uint16_t pc{};
    std::uint8_t a{}, b{}, psw{}, sp{}, dpl{}, dph{};
    std::array<std::uint8_t, 8> r{}; // the bank selected by PSW.RS
    std::uint8_t fmap{}, memctr{}, mpage{};
    std::uint64_t cycles{}, instructions{};
    // DPTR1 and the data pointer select (DPL/DPH above are DPTR0).
    std::uint8_t dpl1{}, dph1{}, dps{};
    // Interrupt controller: bit L of interrupt_levels is set while an ISR of
    // priority level L is in progress; interrupt_inhibit is the one
    // instruction after RETI during which no interrupt is taken. The posted
    // request is the one the core takes at the next boundary if its level
    // allows (vector 0 and source -1: none).
    std::uint8_t interrupt_levels{};
    bool interrupt_inhibit{};
    std::uint16_t posted_vector{};
    std::uint8_t posted_level{};
    int posted_source{-1};
};

// Counters for accesses the model accepts but does not emulate, so a run can
// show which unmodeled hardware the firmware touched.
struct Cc2530Diagnostics {
    std::uint64_t unmapped_xdata_reads{};
    std::uint64_t unmapped_xdata_writes{};
    std::uint64_t read_only_xdata_writes{}; // info page, XBANK, read-only XREGs, P0-P2 alias
    std::uint64_t read_only_sfr_writes{};   // CLKCONSTA, SLEEPSTA
    std::uint64_t fmap_writes{};
    std::uint64_t memctr_writes{};
    std::uint64_t interrupts_taken{};
    // Flags set through raise_interrupt_flag_for_test (not by a peripheral).
    std::uint64_t test_injected_flags{};
    // Flags set by the virtual-time event queue (schedule_interrupt_flag_for_test).
    std::uint64_t scheduled_flag_events{};
    // Peripheral service() calls made by the deadline scheduler.
    std::uint64_t peripheral_services{};
    // Interrupt-delivery contract check (R1-F10). Counts instruction
    // boundaries at which the request posted to the core differed from the
    // one recomputed from the flag, enable and priority SFRs, i.e. a change
    // that bypassed update_interrupts(). The boundary check repairs the post,
    // so trace mode and single-stepping would otherwise mask such a defect.
    // Must stay 0.
    std::uint64_t stale_interrupt_posts{};
    // PCON.IDLE. Entries count every write of PCON.IDLE = 1; wakes include
    // immediate ones (a waking request already pending at the write).
    // idle_cycles is the virtual time spent in any power mode.
    std::uint64_t power_mode_entries{};
    std::uint64_t power_mode_wakes{};
    std::uint64_t power_mode_immediate_wakes{};
    std::uint64_t idle_cycles{};
    // Per Cc2530PowerMode (index 0, Active, unused): entries and virtual time.
    std::array<std::uint64_t, 5> power_mode_entries_by_mode{};
    std::array<std::uint64_t, 5> power_mode_cycles_by_mode{};
    // Resets per Cc2530ResetCause, including the power-on reset of a load.
    std::array<std::uint64_t, 4> resets_by_cause{};
    // CPU stalls requested by the flash controller and their virtual time.
    std::uint64_t cpu_stalls{};
    std::uint64_t cpu_stall_cycles{};
    // Entries that break the SWRU191F 4.2 placement rule: the instruction
    // after the one setting PCON.IDLE starts on a 4-byte boundary.
    std::uint64_t idle_alignment_violations{};
    // Writes of 1 to IRCON bit 6 ("must be written 0"; effect not modeled).
    std::uint64_t ircon_bit6_writes{};
    // XREGs no peripheral model owns (plain latches).
    std::map<std::uint16_t, std::uint64_t> unmodeled_xreg_reads;
    std::map<std::uint16_t, std::uint64_t> unmodeled_xreg_writes;
    // Accesses a peripheral model accepts without emulating the selected
    // function (key: XREG address, or 0x7000 | SFR address).
    std::map<std::uint16_t, std::uint64_t> unmodeled_feature_reads;
    std::map<std::uint16_t, std::uint64_t> unmodeled_feature_writes;
    std::vector<std::string> load_warnings;
};

// TI CC2530F256 SoC around the generic Unicorn MCS-51 core (SWRU191F).
//
// CODE has three identities that must not be confused: the linker's virtual
// address, the 16-bit CPU logical address, and the physical flash offset. The
// lower CODE half (0x0000-0x7fff) is always physical bank 0; the upper half
// shows bank FMAP, i.e. physical FMAP * 0x8000 + (logical - 0x8000). With
// MEMCTR.XMAP set, logical 0x8000-0x9fff is the 8 KiB SRAM instead.
//
// Modeled: flash, SRAM and its DATA/IDATA/XDATA aliases, FMAP, MEMCTR
// (XBANK and XMAP), MPAGE (MOVX @Ri page), the dual data pointer (DPS,
// DPTR0/DPTR1), the XDATA map (XREG, SFR alias, info page, XBANK), the
// 18-source interrupt controller with its four priority levels, priority
// groups, polling order, nesting and the post-RETI instruction rule, the
// CLKCONCMD -> CLKCONSTA mirror, power modes PM0-PM3 (PCON.IDLE with
// SLEEPCMD.MODE), resets, a deterministic virtual-time interrupt-event queue
// and the peripherals below (docs/cc2530.md lists them and their limits).
//
// Timing contract (EMU-04A-R2, see emu04a-r2-interrupt-scheduler.md):
// registers().cycles is the virtual time base. An instruction advances it by
// the MCS51 core's cycle table (helper.c): the classic Intel machine-cycle
// count (1, 2 or 4) except that MOV direct,Rn (0x88-0x8F), MOV Rn,direct
// (0xA8-0xAF), ORL/ANL C,bit and C,/bit (0x72, 0x82, 0xA0, 0xB0) and MOV
// bit,C (0x92) take 1 instead of 2. This is a model-defined contract: it is
// neither the classic table nor the CC2530 clock counts of SWRU191F
// Table 2-3. An interrupt entry advances it by 2 (TI: at least 7) and counts
// as one instruction. Time spent in a power mode advances cycles but not
// instructions. CLKCONCMD does not change the rate.
//
// Interrupt-delivery contract: every change to an interrupt flag, enable or
// priority SFR, whether by firmware, host, test hook or scheduled event, goes
// through sfr_write()/set_interrupt_flags() and update_interrupts(), which
// post the resulting request to the core before the next instruction
// boundary. The core evaluates the posted request at every instruction
// boundary in every execution mode.
//
// Peripherals (src/mcs51/cc2530_*.hpp) own their SFRs and XREGs; the core
// dispatches CPU, DMA and host accesses to the owner and runs a deadline
// scheduler: a peripheral's deadline() is serviced at the first instruction
// boundary at which cycles >= deadline, in every execution mode, exactly
// like the test events above (test events first at equal times, then
// peripherals in construction order). XREGs no model owns stay plain
// latches and are counted in diagnostics().unmodeled_xreg_*.
class Cc2530F256 {
public:
    static constexpr std::size_t FlashSize = 256 * 1024;
    static constexpr std::size_t BankSize = 0x8000;
    static constexpr unsigned BankCount = 8;
    static constexpr std::size_t SramSize = 8 * 1024;
    static constexpr std::uint16_t IdataXdataAlias = 0x1f00;
    // cc2530-zigbee keeps NV pages 125/126 and the lock/config page 127 out of
    // CODE images (docs/BANKED_CODE.md); the lock bits are the last 16 bytes.
    static constexpr std::uint32_t ReservedFlashStart = 0x3e800;
    static constexpr std::uint32_t LockBitsStart = 0x3fff0;
    static constexpr std::uint16_t InfoPageStart = 0x7800;
    static constexpr std::uint16_t IeeeAddressOffset = 0x780c;

    Cc2530F256();
    ~Cc2530F256();
    Cc2530F256(const Cc2530F256 &) = delete;
    Cc2530F256 &operator=(const Cc2530F256 &) = delete;

    // Intel HEX with physical flash addresses (type 02 or 04 extended address
    // records for banks 1-7; llvm-objcopy emits 02 below 1 MiB) or a raw binary
    // that starts at physical address 0. ELF images are rejected: convert them
    // with llvm-objcopy -O ihex, which uses the LMAs.
    // Erases the flash first, then performs a power-on reset.
    void load_firmware(const std::string &path);
    // Erases the flash, programs `image` at physical `address` and performs a
    // power-on reset (used by tests).
    void load_flash(std::uint32_t address, const std::vector<std::uint8_t> &image);

    // Power-on reset: virtual time and SRAM cleared (SRAM power-on content is
    // undefined), then a reset. reset() keeps SRAM content and virtual time,
    // like a watchdog or external reset (SLEEPSTA.RST = 01).
    void power_on_reset();
    void reset();

    // Runs at most `instruction_limit` instructions, stopping at the first
    // instruction boundary where the cycle counter has reached `cycle_limit`,
    // before executing a breakpoint (except as the first instruction of this
    // call), when PCON enters a power mode, or on a fault.
    //
    // Scheduled events are applied at the first instruction boundary at which
    // cycles >= their time, before the core evaluates interrupts there; all
    // events due at the boundary where run() returns have been applied.
    //
    // In a power mode no instruction executes. Virtual time advances to the
    // next scheduled event, or to `cycle_limit` if that comes first (a finite
    // limit only), and every enabled pending request (IEN0.EA, its enable bit
    // and its flag set, whatever its priority) wakes the CPU. The instruction
    // after the one that set PCON.IDLE then executes before any ISR
    // (SWRU191F 4.2), also when the request was already pending at the
    // PCON write. If virtual time advanced in the power mode during this
    // call, run() may return at `cycle_limit` without executing an
    // instruction.
    Cc2530RunResult run(std::size_t instruction_limit,
                        std::uint64_t cycle_limit = UINT64_MAX);
    void set_breakpoint(std::optional<std::uint16_t> logical_pc) noexcept {
        breakpoint_ = logical_pc;
    }

    // An unconditional branch to itself with no interrupt deliverable and no
    // scheduled event left: nothing can make the CPU leave it.
    bool idle_forever() const;
    // A scheduled event, pending reset or peripheral may still raise an
    // interrupt or reset; false: nothing on the chip can end a power mode.
    bool may_wake() const;

    // Side-effect-free inspection.
    std::uint8_t read_sfr(std::uint8_t address) const;
    std::uint8_t read_idata(std::uint8_t address) const;
    std::uint8_t peek_xdata(std::uint16_t address) const;
    std::uint8_t read_code(std::uint16_t logical) const;
    std::uint8_t read_flash(std::uint32_t physical) const;
    // Physical flash offset behind a logical CODE address, or nullopt when
    // XMAP maps it to SRAM.
    std::optional<std::uint32_t> code_physical(std::uint16_t logical) const;
    Cc2530Registers registers() const;
    std::uint16_t pc() const;
    std::uint8_t fmap() const noexcept { return sfr_[0x9f]; }
    std::uint8_t memctr() const noexcept { return sfr_[0xc7]; }
    // In a power mode entered through PCON.IDLE (see Cc2530RunResult).
    bool sleeping() const noexcept { return sleep_requested_; }
    // The power mode in effect (Active while running; the mode requested by
    // the PCON write becomes effective at the following instruction boundary).
    Cc2530PowerMode power_mode() const noexcept { return power_mode_; }
    // Between an interrupt vector entry and its RETI (any priority level).
    bool in_interrupt() const { return engine_levels() != 0; }

    // Host-side writes. write_sfr applies the register's side effects.
    void write_sfr(std::uint8_t address, std::uint8_t value);
    void write_sram(std::uint16_t address, std::uint8_t value);
    void set_ieee_address(std::uint64_t address);

    // TEST ONLY. Sets the CPU interrupt flag of SWRU191F Table 2-5 source
    // `source` (0..17) exactly as a firmware write to that flag register
    // would, and nothing else: no timer, DMA, radio or other peripheral state
    // is touched, so it does not pretend a peripheral raised the event.
    // Counted in diagnostics().test_injected_flags. Not reachable through
    // the system layer, JSON-RPC or MCP. Throws std::out_of_range for
    // source > 17.
    void raise_interrupt_flag_for_test(unsigned source);
    // TEST ONLY. As above, but sets only the bits of `flag_mask` (a non-empty
    // subset of the source's flag bits, e.g. ENCIF_1 = 0x02 of S0CON).
    // Throws std::invalid_argument for any other mask.
    void raise_interrupt_flag_for_test(unsigned source, std::uint8_t flag_mask);

    // TEST ONLY until peripherals exist. Deterministic asynchronous interrupt
    // event: at the first instruction boundary at which the cycle counter is
    // >= `at_cycle` (immediately at the next boundary if it already is), the
    // flag bits `flag_mask` of `source` are set through the same path a
    // peripheral uses, without any firmware SFR write. Events due at the same
    // boundary apply in (at_cycle, scheduling order). A reset discards all
    // events. Counted in diagnostics().scheduled_flag_events.
    void schedule_interrupt_flag_for_test(std::uint64_t at_cycle, unsigned source);
    void schedule_interrupt_flag_for_test(std::uint64_t at_cycle, unsigned source,
                                          std::uint8_t flag_mask);
    std::size_t scheduled_event_count() const noexcept { return events_.size(); }
    std::optional<std::uint64_t> next_event_cycle() const noexcept {
        if (events_.empty())
            return std::nullopt;
        return events_.front().at;
    }

    // Earliest virtual time at which a scheduled test event or a peripheral
    // needs service (UINT64_MAX: none).
    std::uint64_t next_deadline() const;

    // ---- host side of the peripheral models ----
    // Pads: `port` 0-2, `pin` 0-7 (P2 has pins 0-4).
    // External digital level on a pad; nullopt: not driven by the host.
    void set_pin_input(unsigned port, unsigned pin, std::optional<bool> level);
    // External analog voltage on a pad (ADC, comparator, op-amp inputs).
    void set_pin_analog_mv(unsigned port, unsigned pin, std::optional<std::uint32_t> millivolts);
    Cc2530PinDrive pin_drive(unsigned port, unsigned pin) const;
    // Level seen by the pad's input buffer.
    bool pin_level(unsigned port, unsigned pin) const;
    // Called on every change of a pad's drive, with the virtual time.
    void set_pin_listener(
        std::function<void(unsigned port, unsigned pin, Cc2530PinDrive drive, std::uint64_t cycle)>
            listener);
    void set_temperature_celsius(double celsius);
    void set_vdd_mv(std::uint32_t millivolts);
    // USART `index` 0/1 in UART mode.
    bool uart_can_receive(unsigned index) const;
    void uart_receive(unsigned index, std::uint8_t byte);
    std::vector<std::uint8_t> uart_take_transmitted(unsigned index);
    void set_spi_device(unsigned index, std::function<std::uint8_t(std::uint8_t)> device);
    // Seeds deterministic noise sources (ADC analog noise, radio RFRND and
    // CSP RANDXY).
    void set_noise_seed(std::uint64_t seed);
    // Connects the IEEE 802.15.4 radio to a shared air as node `node`.
    // Virtual time maps to air time as ns = ns_offset + cycles * 1000 / 32.
    void attach_air(mcuemu::VirtualAir &air, std::string node, std::uint64_t ns_offset = 0);
    // Internal peripheral objects (src/mcs51/cc2530_*.hpp), for white-box
    // tests and the system layer.
    cc2530::Soc &peripherals() noexcept { return *soc_; }

    void set_trace(std::ostream *output) noexcept { trace_output_ = output; }
    const Cc2530Diagnostics &diagnostics() const noexcept { return diagnostics_; }

    static unsigned instruction_size(std::uint8_t opcode) noexcept;

private:
    class BusImpl;
    static std::uint64_t sfr_mmio_read(uc_struct *uc, std::uint64_t offset,
                                       unsigned size, void *user_data);
    static void sfr_mmio_write(uc_struct *uc, std::uint64_t offset,
                               unsigned size, std::uint64_t value,
                               void *user_data);
    static std::uint64_t xdata_mmio_read(uc_struct *uc, std::uint64_t offset,
                                         unsigned size, void *user_data);
    static void xdata_mmio_write(uc_struct *uc, std::uint64_t offset,
                                 unsigned size, std::uint64_t value,
                                 void *user_data);
    static void code_hook(uc_struct *uc, std::uint64_t address,
                          std::uint32_t size, void *user_data);

    std::uint8_t xdata_value(std::uint16_t address) const;
    std::uint8_t xdata_read(std::uint16_t address);
    void xdata_write(std::uint16_t address, std::uint8_t value);
    std::uint8_t sfr_read(std::uint8_t address);
    std::uint8_t sfr_peek(std::uint8_t address) const;
    std::uint8_t sleepsta_value() const;
    void sfr_write(std::uint8_t address, std::uint8_t value);
    // After a CPU/DMA/host access reached a peripheral: a lowered deadline
    // must stop the fast interpreter at the right boundary.
    void peripheral_accessed();
    void refresh_flash(std::uint32_t begin, std::uint32_t end);
    void rebuild_views();
    void install_code_view();
    struct Request {
        int source{-1};
        std::uint8_t level{};
    };
    Request pending_interrupt() const;
    void update_interrupts();
    // update_interrupts() at an instruction boundary where no input may have
    // changed without a post: counts a stale post first (R1-F10).
    void check_interrupts_at_boundary();
    bool sync_interrupt_consumption();
    // The single path for a non-firmware flag change (events, test hook,
    // future peripherals).
    void set_interrupt_flags(unsigned source, std::uint8_t flag_mask);
    std::uint8_t checked_flag_mask(unsigned source, std::uint8_t flag_mask) const;
    void apply_due_events();
    std::uint64_t engine_cycles() const;
    void wake_from_power_mode();
    // At an instruction boundary: makes a requested power mode effective,
    // then wakes if the mode's wake rule holds. True if it woke.
    void settle_power_mode();
    bool wake_request(Cc2530PowerMode mode) const;
    void notify_power_mode(Cc2530PowerMode from, Cc2530PowerMode to);
    bool apply_pending_reset();
    std::uint64_t hf_now() const;
    std::uint64_t lf_now() const;
    // Mirrors the core's decision at the next instruction boundary; the
    // post-RETI inhibit is ignored where only eventual delivery matters.
    bool interrupt_deliverable(bool honor_inhibit) const;
    void enter_power_mode();
    void request_stop(std::string error);
    void check_error(int error, const char *operation) const;
    void set_code_hook_enabled(bool enabled);
    void trace_instruction(std::uint16_t pc);
    std::uint16_t engine_irq_vector() const;
    std::uint8_t engine_levels() const;
    bool engine_inhibit() const;
    void reset_core(Cc2530ResetCause cause);
    void create_peripherals();

    uc_struct *uc_{};
    std::size_t code_hook_{};
    bool code_hook_installed_{};

    // Physical flash, and one zero-copy 64 KiB CODE view per FMAP value:
    // view N = flash[0, 0x8000) followed by flash[N * 0x8000, +0x8000).
    std::vector<std::uint8_t> flash_;
    std::vector<std::uint8_t> views_;
    // The CODE view while MEMCTR.XMAP is set. Bytes 0x8000-0x9fff of this
    // buffer *are* the SRAM: XDATA 0x0000-0x1fff and IDATA (SRAM 0x1f00-
    // 0x1fff) are mapped onto them, so code written to SRAM and executed
    // through XMAP is always coherent.
    std::vector<std::uint8_t> xmap_view_;
    std::uint8_t *sram_{};
    std::uint8_t *idata_{};
    std::array<std::uint8_t, 256> sfr_{};
    std::array<std::uint8_t, 256> sfr_special_read_{};
    std::array<std::uint8_t, 256> sfr_special_write_{};
    std::array<std::uint8_t, 0x400> xreg_{};
    std::array<std::uint8_t, 0x800> info_page_{};

    std::unique_ptr<BusImpl> bus_;
    std::unique_ptr<cc2530::Soc> soc_;
    // Construction order = scheduler tie-break order.
    std::vector<std::unique_ptr<cc2530::Peripheral>> peripherals_;
    std::array<cc2530::Peripheral *, 256> sfr_owner_{};
    std::array<cc2530::Peripheral *, 0x400> xreg_owner_{};
    Cc2530PowerMode power_mode_{Cc2530PowerMode::Active};
    Cc2530PowerMode requested_mode_{Cc2530PowerMode::Active};
    std::uint64_t power_mode_since_{};
    std::uint64_t hf_stopped_{}; // completed time in PM1-PM3
    std::uint64_t lf_stopped_{}; // completed time in PM3
    std::optional<Cc2530ResetCause> pending_reset_;
    std::uint64_t stall_until_{};
    std::uint64_t fast_event_stop_{UINT64_MAX};

    std::uint16_t posted_vector_{};
    std::uint8_t posted_level_{};
    int posted_source_{-1};
    struct Event {
        std::uint64_t at{};
        std::uint64_t sequence{};
        unsigned source{};
        std::uint8_t flag_mask{};
    };
    // Sorted by (at, sequence).
    std::vector<Event> events_;
    std::uint64_t event_sequence_{};
    std::optional<std::uint16_t> breakpoint_;
    bool break_hit_{};
    std::uint64_t hook_calls_{};
    std::uint64_t trace_cycle_limit_{UINT64_MAX};
    bool sleep_requested_{};
    bool running_fast_{};
    std::string stop_error_;
    Cc2530Diagnostics diagnostics_;
    std::ostream *trace_output_{};
};

} // namespace mcs51emu

#endif
