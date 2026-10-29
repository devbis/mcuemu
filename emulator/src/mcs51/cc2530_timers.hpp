#pragma once

// CC2530 Timer 1 (SWRU191F Chapter 9), Timer 3/Timer 4 (Chapter 10), the
// sleep timer (Chapter 11) and the watchdog timer (Chapter 16).

#include "cc2530_soc.hpp"

#include <cstdint>

namespace mcs51emu::cc2530 {

class Timer1 final : public Peripheral {
public:
    explicit Timer1(Bus &bus);
    const char *name() const override { return "T1"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;
    void cpu_flags_written() override;

    // Compare output level of channel 0-4 (for Io pin routing).
    bool output(unsigned channel) const;
    // Input capture: level of the pad routed to channel 0-4 changed.
    void input_changed(unsigned channel, bool level);
    // Input capture from the RF interrupt (T1CCTLn.RFIRQ = 1): an RF
    // interrupt request (rising) or the RF interrupt line going low.
    void rf_interrupt_edge(bool rising);

private:
    friend class Timer34;
    friend struct TimersTestAccess;

    enum class CountMode : std::uint8_t { Stopped, Free, Down, Modulo, UpDown };
    // Counter shared by Timer 1 (16 bits) and Timer 3/Timer 4 (8 bits).
    struct Counter {
        std::uint16_t max = 0xffff;
        CountMode mode = CountMode::Stopped;
        std::uint16_t count = 0;
        bool down = false;     // up-and-down mode, counting down
        bool halted = false;   // T3/T4 down mode reached 0x00
        bool external = false; // T1 ticked by T3 channel 1 compares (IRCTL.IRGEN)
        std::uint64_t period = 1; // HF cycles per counter step
        std::uint64_t anchor = 0; // hf_now() of the last step (or of the start)
    };
    struct Channel {
        std::uint8_t ctl = 0x40;   // TxCCTLn
        std::uint16_t value = 0;   // T1CCn / TxCCn in effect
        std::uint16_t pending = 0; // compare value waiting for count 0
        bool has_pending = false;
        std::uint8_t low = 0; // T1CCnL write buffer
        bool out = false;     // compare output level
    };
    struct Unit {
        Counter c;
        Channel ch[5];
        unsigned channels = 5;
        bool timer1 = true;
    };
    // step() result: bit n = channel n compare event, EventOverflow = overflow.
    static constexpr std::uint8_t EventOverflow = 0x80;
    static std::uint64_t ticks_to_event(const Unit &unit);
    static void step_linear(Unit &unit, std::uint64_t ticks);
    static std::uint8_t step(Unit &unit);
    static void load_pending(Unit &unit);
    static void initialize_output(Channel &channel);
    template <class Emit>
    static void advance(Unit &unit, std::uint64_t hf, Emit &&emit);
    static std::uint64_t unit_deadline(const Bus &bus, const Unit &unit);

    void clock_changed() override;
    bool may_interrupt() const override;
    void settle();
    void handle_events(std::uint8_t events);
    void sync_storage();
    std::uint8_t outputs() const;
    void notify_outputs(std::uint8_t before);
    std::uint64_t tick_period() const;
    std::uint16_t count_now() const;
    // One Timer 3 channel 1 compare event while IRCTL.IRGEN = 1.
    void ir_tick();
    // Captures channel `channel` on an edge of its selected input if its
    // mode and CAP allow (sets T1STAT.CHnIF); true if it captured.
    bool capture(unsigned channel, bool rising);

    Unit unit_;
    unsigned ir_prescale_ = 0;
    // TEST ACCESSOR (TimersTestAccess): T1_CH0..T1_CH2 DMA triggers asserted
    // since construction.
    std::uint64_t dma_triggers_[3]{};
};

// Timer 3 and Timer 4 share TIMIF; one peripheral models both.
class Timer34 final : public Peripheral {
public:
    explicit Timer34(Bus &bus);
    const char *name() const override { return "T3/T4"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;
    void cpu_flags_written() override;

    // `timer` 3 or 4, `channel` 0 or 1.
    bool output(unsigned timer, unsigned channel) const;
    void input_changed(unsigned timer, unsigned channel, bool level);

private:
    friend class Timer1;
    friend struct TimersTestAccess;
    using Unit = Timer1::Unit;

    void clock_changed() override;
    bool may_interrupt() const override;
    void settle();
    void handle_events(unsigned index, std::uint8_t events);
    void sync_storage();
    std::uint8_t outputs() const;
    void notify_outputs(std::uint8_t before);
    std::uint64_t tick_period(unsigned index) const;

    Unit unit_[2]; // Timer 3, Timer 4
    // TEST ACCESSOR (TimersTestAccess): T3_CH0, T3_CH1, T4_CH0, T4_CH1 DMA
    // triggers asserted since construction.
    std::uint64_t dma_triggers_[4]{};
};

class SleepTimer final : public Peripheral {
public:
    explicit SleepTimer(Bus &bus);
    const char *name() const override { return "ST"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;

    // 24-bit sleep timer count at the current time.
    std::uint32_t count() const;
    // A port pad changed level (capture input selection is STCS/STCC).
    void input_changed(unsigned port, unsigned pin, bool level);

private:
    friend struct TimersTestAccess;

    bool may_interrupt() const override;
    void settle();
    void process_edge(std::uint64_t edge);
    std::uint64_t compare_edge() const;
    std::uint64_t next_event_edge() const;
    std::uint32_t count_at(std::uint64_t lf) const;
    std::uint32_t visible_count() const;

    // 32 kHz periods are counted as lf_ticks(); rising CLK32K edge j lies in
    // period j (see cc2530_timers.cpp).
    std::uint64_t base_ = 0;      // lf tick at which the count was 0
    std::uint64_t next_edge_ = 0; // first rising edge not processed yet
    std::uint32_t compare_ = 0xffffff;
    std::uint8_t write_st1_ = 0, write_st2_ = 0;
    bool loading_ = false;
    std::uint32_t load_value_ = 0;
    std::uint64_t load_edge_ = 0;
    bool capturing_ = false;
    std::uint32_t capture_value_ = 0;
    std::uint64_t capture_edge_ = 0;
    bool stale_ = false;
    std::uint32_t stale_value_ = 0;
    std::uint64_t stale_edge_ = 0;
    // TEST ACCESSOR (TimersTestAccess): ST DMA triggers since construction.
    std::uint64_t dma_triggers_ = 0;
};

class Watchdog final : public Peripheral {
public:
    explicit Watchdog(Bus &bus);
    const char *name() const override { return "WDT"; }
    void reset(ResetCause cause) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;

private:
    friend struct TimersTestAccess;
    enum class WdtMode : std::uint8_t { Idle, Watchdog, Timer };

    void clock_changed() override;
    bool may_interrupt() const override;
    void settle();
    std::uint64_t limit() const;

    WdtMode mode_ = WdtMode::Idle;
    std::uint8_t interval_ = 0;   // WDCTL.INT in effect
    std::uint64_t count_ = 0;     // 32 kHz periods counted
    std::uint64_t last_tick_ = 0; // lf_ticks() up to which count_ is current
    bool frozen_ = false;         // PM1-PM3
    bool armed_ = false;          // CLR = 0xA written
    std::uint64_t armed_lf_ = 0;
    bool reset_requested_ = false;
};

} // namespace mcs51emu::cc2530
