#pragma once

// CC2530 Timer 2, the MAC timer (SWRU191F Chapter 22): 16-bit timer with
// period, 24-bit overflow counter, compare 1/2 and overflow compare 1/2,
// SFD capture, the T2 interrupt (T2IRQF/T2IRQM), the T2_EVENT1/T2_EVENT2
// outputs to the DMA and the CSP, and sleep-timer synchronized start/stop.

#include "cc2530_soc.hpp"

#include <cstdint>

namespace mcs51emu::cc2530 {

class MacTimer final : public Peripheral {
public:
    explicit MacTimer(Bus &bus);
    const char *name() const override { return "T2"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;
    bool may_interrupt() const override;

    // The radio detected a start-of-frame delimiter (transmitted or
    // received) at `at` (<= now): captures the timer and overflow count.
    void sfd_capture(std::uint64_t at);
    // Same, at HF time `hf` (<= hf_now()). `hf` may precede timer events
    // already processed (a frame the shared air reported late); the value
    // is then taken from the recorded counter history.
    void sfd_capture_hf(std::uint64_t hf);
    // True while the timer runs.
    bool running() const;

    // ---- RF timing domain (shared with Radio) ----
    // The MAC timer and the radio are both clocked by the 32 MHz XOSC and
    // exchange events (timer overflow / T2_EVENTn -> CSP, SFD -> capture,
    // CSP SETCMP1 -> compare 1). Their internal events are kept in HF time
    // (Bus::hf_now) and processed together in time order by
    // advance_rf_domain() so that each sees the other's state at the exact
    // event time.
    // HF time of the next internal timer event, or Never.
    std::uint64_t next_event_hf() const;
    // Processes the event returned by next_event_hf().
    void process_next_event();
    // CSP instruction SETCMP1 executed at HF time `hf` (SWRU191F 23.14.9):
    // t2_cmp1 = timer value at `hf`.
    void set_compare1_from_timer(std::uint64_t hf);

    // Test-only observation (DMA skeleton cannot record triggers): number of
    // T2_EVENT1 (which = 1) / T2_EVENT2 (which = 2) pulses since reset.
    std::uint64_t event_pulses(unsigned which) const;

private:
    enum class Phase : std::uint8_t {
        Idle,      // stopped (STATE = 0)
        StartEdge, // RUN = 1 with SYNC: waiting for the 32 kHz edge
        Starting,  // edge seen, values computed, loading 86 cycles later
        Running,   // STATE = 1
        StopEdge,  // RUN = 0 with SYNC: running until the 32 kHz edge
    };

    void catch_up();
    std::uint64_t to_now(std::uint64_t hf) const;
    std::uint64_t next_lf_edge_hf() const;
    std::uint32_t sleep_count_at_hf(std::uint64_t hf) const;
    bool counting() const { return phase_ == Phase::Running || phase_ == Phase::StopEdge; }
    std::uint16_t timer_at(std::uint64_t hf) const;
    void rebase(std::uint64_t hf);
    std::uint64_t next_timer_event() const;
    void timer_event(std::uint64_t hf);
    void edge_event(std::uint64_t hf);
    void start_loaded(std::uint64_t hf);
    void reconcile(std::uint64_t hf);
    void set_flags(std::uint8_t bits);
    void fire(unsigned event, std::uint64_t hf);
    std::uint8_t ctrl_value() const;
    std::uint8_t mux_read(Reg reg, bool side_effects);

    Phase phase_{Phase::Idle};
    // Counter: holds `tim_` from `anchor_` and counts from `count_from_`
    // (> anchor_ while the delta counter of 22.1.4 halts the timer).
    std::uint64_t anchor_{0};
    std::uint64_t count_from_{0};
    std::uint16_t tim_{0};
    std::uint32_t ovf_{0};
    std::uint16_t per_{0}, cmp1_{0}, cmp2_{0}, cap_{0};
    std::uint32_t ovf_per_{0}, ovf_cmp1_{0}, ovf_cmp2_{0}, ovf_cap_{0};
    // Read latches (22.1.2, 22.1.6) and write buffers (22.1.2, 22.1.7).
    std::uint8_t tim_high_latch_{0};
    std::uint32_t ovf_latch_{0};
    std::uint8_t m0_buffer_{0};
    std::uint8_t movf_buffer_[2]{};
    std::uint8_t irqf_{0};
    std::uint64_t rmw_time_{Never};
    std::uint8_t rmw_snapshot_{0};
    // Synchronized start/stop (22.4).
    std::uint64_t edge_hf_{Never};
    std::uint64_t load_hf_{Never};
    std::uint16_t load_tim_{0};
    std::uint32_t load_ovf_{0};
    bool stored_valid_{false};
    std::uint32_t stored_sleep_{0};
    std::uint64_t pulses_[2]{};
    // Counter history for sfd_capture_hf(): state valid from `since`.
    struct Segment {
        std::uint64_t since;
        std::uint64_t count_from;
        std::uint32_t ovf;
        std::uint16_t tim, per;
        bool counting;
    };
    static constexpr unsigned SegmentCount = 64;
    void note_segment(std::uint64_t since);
    Segment segments_[SegmentCount]{};
    unsigned segment_last_{0};
    unsigned segment_size_{0};
};

// Processes the MAC timer and radio events due at or before HF time `hf`
// in time order (timer first on ties). Not re-entrant: nested calls return.
void advance_rf_domain(Soc &soc, std::uint64_t hf);

} // namespace mcs51emu::cc2530
