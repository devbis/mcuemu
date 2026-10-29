#pragma once

// Internal CC2530F256 peripheral framework (SWRU191F). Not a public header:
// the public surface is mcs51emu::Cc2530F256.
//
// Every SoC block that owns SFRs or XREGs is a Peripheral. The core
// (Cc2530F256) keeps the register storage, dispatches CPU, DMA and host
// accesses of claimed registers to their owner, runs the deadline scheduler
// and the power modes, and owns the CPU interrupt controller. Peripherals talk
// to the core and to each other only through Bus.

#include "mcs51emu/cc2530f256.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace mcuemu {
class VirtualAir;
}

namespace mcs51emu::cc2530 {

inline constexpr std::uint64_t Never = UINT64_MAX;

// Virtual time is counted in ticks of the 32 MHz system clock reference
// (one Unicorn cycle = 1/32 us), whatever CLKCONCMD selects. The 32.768 kHz
// clock that drives the sleep timer and the watchdog advances one tick every
// 15625/16 = 976.5625 cycles (exact rational, no drift).
inline constexpr std::uint64_t CyclesPerSecond = 32000000;
// Floor of the number of 32.768 kHz ticks elapsed at `cycles`.
constexpr std::uint64_t lf_ticks(std::uint64_t cycles) { return cycles / 15625 * 16 + cycles % 15625 * 16 / 15625; }
// First cycle at which lf_ticks() reaches `ticks`.
constexpr std::uint64_t lf_tick_cycle(std::uint64_t ticks) {
    return ticks / 16 * 15625 + (ticks % 16 * 15625 + 15) / 16;
}

// Register identity. An XREG is its XDATA address (0x6000-0x63ff); an SFR is
// its XDATA alias address 0x7000 | sfr (0x7080-0x70ff).
using Reg = std::uint16_t;
constexpr Reg sfr_reg(std::uint8_t sfr) { return static_cast<Reg>(0x7000u | sfr); }
constexpr bool is_sfr(Reg reg) { return (reg & 0xff00u) == 0x7000u; }
constexpr std::uint8_t sfr_of(Reg reg) { return static_cast<std::uint8_t>(reg); }

// CPU interrupt sources, SWRU191F Table 2-5 order (vector 0x03 + 8 * n).
enum class Irq : unsigned {
    RfErr = 0, Adc, Urx0, Urx1, Enc, St, P2Int, Utx0, Dma, T1, T2, T3, T4,
    P0Int, Utx1, P1Int, Rf, Wdt,
};

// DMA trigger numbers, SWRU191F Table 8-1.
namespace trigger {
inline constexpr unsigned None = 0, Prev = 1, T1Ch0 = 2, T1Ch1 = 3, T1Ch2 = 4,
                          T2Event1 = 5, T2Event2 = 6, T3Ch0 = 7, T3Ch1 = 8, T4Ch0 = 9,
                          T4Ch1 = 10, St = 11, Ioc0 = 12, Ioc1 = 13, Urx0 = 14, Utx0 = 15,
                          Urx1 = 16, Utx1 = 17, Flash = 18, Radio = 19, AdcChAll = 20,
                          AdcCh11 = 21, /* ... AdcCh84 = 28 */ EncDw = 29, EncUp = 30,
                          DbgBw = 31;
}

using PowerMode = mcs51emu::Cc2530PowerMode;
using ResetCause = mcs51emu::Cc2530ResetCause;

class Peripheral;
class Io;
class Adc;
class AnalogComparator;
class OpAmp;
class Usart;
class Timer1;
class Timer34;
class SleepTimer;
class Watchdog;
class MacTimer;
class Dma;
class FlashController;
class Aes;
class Radio;
class Debug;

// Typed access between peripherals (all non-null after construction).
struct Soc {
    Io *io{};
    Adc *adc{}; // also the RNG (ADCCON1.RCTRL, RNDL/RNDH)
    AnalogComparator *comparator{};
    OpAmp *opamp{};
    Usart *usart[2]{};
    Timer1 *timer1{};
    Timer34 *timer34{}; // Timer 3, Timer 4 and the shared TIMIF
    SleepTimer *sleep_timer{};
    Watchdog *watchdog{};
    MacTimer *mac_timer{};
    Dma *dma{};
    FlashController *flash{};
    Aes *aes{};
    Radio *radio{};
    Debug *debug{}; // the debug interface (Chapter 3)
};

class Bus {
public:
    virtual ~Bus() = default;

    // ---- registration (peripheral constructors only) ----
    // Claims an SFR: every CPU/DMA/host write goes to owner->write(). With
    // `computed_read`, CPU and DMA reads go to owner->read() (needed when a
    // read has side effects or the value is derived from time/state);
    // otherwise the CPU reads the storage byte directly, so the owner must
    // keep storage(reg) current. One owner per register.
    virtual void claim_sfr(std::uint8_t sfr, Peripheral *owner, bool computed_read) = 0;
    // Claims an XREG (0x6000-0x63ff). XREG accesses always reach the owner.
    virtual void claim_xreg(std::uint16_t address, Peripheral *owner) = 0;

    // ---- time ----
    // Current virtual time. Inside a register callback this is the start of
    // the accessing CPU instruction; inside service() it is the instruction
    // boundary being serviced (>= the deadline that triggered it).
    virtual std::uint64_t now() const = 0;
    // Time of the high-frequency clock domain: now() minus the time spent in
    // PM1-PM3, during which the 32 MHz XOSC and 16 MHz RCOSC are off. Frozen
    // while in PM1-PM3.
    virtual std::uint64_t hf_now() const = 0;
    // now()-based deadline at which hf_now() reaches `hf`, or Never while the
    // HF clocks are stopped (re-query after power_mode_changed()).
    virtual std::uint64_t hf_deadline(std::uint64_t hf) const = 0;
    // Time of the 32 kHz domain: now() minus the time spent in PM3.
    virtual std::uint64_t lf_now() const = 0;
    virtual std::uint64_t lf_deadline(std::uint64_t lf) const = 0;
    virtual PowerMode power_mode() const = 0;
    // Effective CLKCONSTA (TICKSPD in bits 5:3 selects the T1/T3/T4 tick
    // 32 MHz >> TICKSPD).
    virtual std::uint8_t clkconsta() const = 0;

    // ---- register storage ----
    virtual std::uint8_t &storage(Reg reg) = 0;
    virtual std::uint8_t storage(Reg reg) const = 0;
    // True while the CPU executes the read half of a read-modify-write
    // instruction (ANL/ORL/XRL/INC/DEC/DJNZ direct, CPL/CLR/SETB bit, JBC,
    // MOV bit,C), as reported by UC_MCS51_REG_RMW_ACCESS.
    virtual bool rmw_access() const = 0;

    // ---- interrupts, DMA, scheduling ----
    // Sets CPU interrupt flag bits of `source` (TCON/S0CON/S1CON/IRCON/
    // IRCON2; a subset of the source's flag mask; 0 = all) and re-evaluates
    // the interrupt controller. Flags are only set, never cleared.
    virtual void raise_irq(Irq source, std::uint8_t mask = 0) = 0;
    virtual bool irq_flag(Irq source) const = 0;
    // Clears CPU flag bits (for peripherals whose CPU flag mirrors a level,
    // e.g. TIMIF/T1STAT summaries; use sparingly, firmware owns R/W0 flags).
    virtual void clear_irq(Irq source, std::uint8_t mask = 0) = 0;
    // Asserts DMA trigger `trigger` (see namespace trigger) now.
    virtual void dma_trigger(unsigned trigger) = 0;
    // A peripheral's deadline() changed; the scheduler re-reads it before
    // the CPU executes past it.
    virtual void reschedule() = 0;

    // ---- memory ----
    // XDATA accesses with full side effects, as a DMA bus master performs
    // them (SRAM, XREG, SFR alias, info page, XBANK flash).
    virtual std::uint8_t xdata_read(std::uint16_t address) = 0;
    virtual void xdata_write(std::uint16_t address, std::uint8_t value) = 0;
    virtual std::uint8_t xdata_peek(std::uint16_t address) const = 0;
    // Physical flash (256 KiB). After modifying it call flash_changed() so
    // CODE views and XBANK reads see the new content.
    virtual std::vector<std::uint8_t> &flash() = 0;
    virtual void flash_changed(std::uint32_t begin, std::uint32_t end) = 0;
    // True while the CPU fetches from flash (not from SRAM through XMAP).
    virtual bool cpu_executing_from_flash() const = 0;

    // ---- system ----
    // Stalls the CPU (no instruction, no interrupt entry) until now() >=
    // `until`; peripherals keep running. Used for flash erase/write
    // initiated by code executing from flash (SWRU191F 6.3.1). The stall
    // starts at the next instruction boundary.
    virtual void stall_cpu_until(std::uint64_t until) = 0;
    // Resets the whole chip with `cause` at the next instruction boundary.
    virtual void request_reset(ResetCause cause) = 0;
    // Stops the run with an error (unsupported or illegal behavior).
    virtual void fault(std::string message) = 0;
    // Counts an access the model accepts but does not emulate.
    virtual void note_unmodeled(Reg reg, bool write) = 0;

    // ---- debug interface (Chapter 3) ----
    // RESET_N is held low: the chip is in reset and the CPU does not run.
    virtual bool reset_held() const = 0;
    // PCON.IDLE was set and the CPU is not executing (power mode requested
    // or in effect).
    virtual bool sleep_requested() const = 0;
    virtual std::uint16_t cpu_pc() const = 0;
    // True inside the CPU engine (between instruction boundaries).
    virtual bool engine_running() const = 0;
    // DEBUG_INSTR: executes the 1-3 byte instruction `code` on the halted
    // CPU at its current PC without advancing the PC, in zero virtual time,
    // and returns A.
    virtual std::uint8_t debug_execute(const std::uint8_t *code, unsigned length) = 0;
    // HALT in a power mode: wake the chip (it stays halted).
    virtual void wake_for_debug_halt() = 0;
    // Timer 1/3/4 time base: hf_now() minus the time the debug interface
    // held the timers (TIMER_SUSPEND / TIMERS_OFF). Timer deadlines are
    // Never while suspended.
    virtual void set_timers_suspended(bool suspended) = 0;
    virtual bool timers_suspended() const = 0;
    virtual std::uint64_t timer_hf_now() const = 0;
    virtual std::uint64_t timer_hf_deadline(std::uint64_t hf) const = 0;
    virtual Soc &soc() = 0;
};

class Peripheral {
public:
    explicit Peripheral(Bus &bus) : bus_(bus) {}
    virtual ~Peripheral() = default;
    Peripheral(const Peripheral &) = delete;
    Peripheral &operator=(const Peripheral &) = delete;

    virtual const char *name() const = 0;
    // Called after the core has zeroed all SFR/XREG storage on any reset;
    // must write the register reset values into storage and reset internal
    // state. now() is valid (0 after a power-on reset).
    virtual void reset(ResetCause cause) = 0;
    // CPU/DMA read of a claimed register (side effects allowed).
    virtual std::uint8_t read(Reg reg) { return peek(reg); }
    // Side-effect-free value of a claimed register (debugger/host view).
    virtual std::uint8_t peek(Reg reg) const { return bus_.storage(reg); }
    // CPU/DMA/host write of a claimed register.
    virtual void write(Reg reg, std::uint8_t value) { bus_.storage(reg) = value; }
    // Earliest absolute time at which service() must run, or Never.
    virtual std::uint64_t deadline() const { return Never; }
    // Called at the first instruction boundary at which now() >= deadline().
    // Must process every internal event due at or before `now` using each
    // event's own scheduled time (not `now`) for state, and move deadline()
    // past `now` (or to Never).
    virtual void service(std::uint64_t now) { (void)now; }
    virtual void power_mode_changed(PowerMode from, PowerMode to) {
        (void)from;
        (void)to;
    }
    // Firmware wrote one of the CPU interrupt flag SFRs (TCON, S0CON, S1CON,
    // IRCON, IRCON2), e.g. to clear a flag: level-sensitive sources (port
    // interrupts) re-assert here.
    virtual void cpu_flags_written() {}
    // CLKCONSTA changed (system clock source, CLKSPD or TICKSPD): tick-based
    // peripherals re-anchor their counters at now().
    virtual void clock_changed() {}
    // May this peripheral, without any CPU or host action, still set an
    // interrupt flag whose delivery is currently enabled, request a reset,
    // or change other CPU-visible state that could do so? Used only to
    // prove that a self-loop can never be left (Cc2530F256::idle_forever);
    // answering true is always safe.
    virtual bool may_interrupt() const { return deadline() != Never; }

protected:
    Bus &bus_;
};

} // namespace mcs51emu::cc2530
