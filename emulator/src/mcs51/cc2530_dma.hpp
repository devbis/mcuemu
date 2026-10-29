#pragma once

// CC2530 five-channel DMA controller (SWRU191F Chapter 8).

#include "cc2530_soc.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace mcs51emu::cc2530 {

class Dma final : public Peripheral {
public:
    explicit Dma(Bus &bus);
    const char *name() const override { return "DMA"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;

    // A DMA trigger (namespace trigger) occurred now.
    void trigger(unsigned number);
    // Debug configuration DMA_PAUSE (SWRU191F Table 3-2): no data slot is
    // granted while paused; a slot in progress completes, descriptor fetches
    // and triggers are still recorded and served after the pause.
    void set_paused(bool paused);

private:
    // The flash controller and the AES coprocessor are co-simulated with the
    // DMA (see advance()): their internal events and the DMA events are
    // processed in one time-ordered loop so that FLASH/ENC_DW/ENC_UP
    // triggers and the DMA writes to FWDATA/ENCDI and reads of ENCDO happen
    // at their exact cycles in every run mode.
    friend class FlashController;
    friend class Aes;

    static constexpr unsigned Channels = 5;
    enum class State : std::uint8_t { Idle, Configuring, Armed };

    struct Channel {
        State state = State::Idle;
        std::uint64_t ready_at = 0; // end of the descriptor fetch (Configuring)
        std::uint64_t armed_at = 0; // time the descriptor was loaded
        bool loaded = false;        // a descriptor was loaded since reset
        // Descriptor fields (SWRU191F Table 8-2), as last loaded.
        std::uint16_t src = 0, dst = 0, len = 0;
        std::uint8_t vlen = 0, tmode = 0, trig = 0, srcinc = 0, destinc = 0, priority = 0;
        bool word = false, irqmask = false, m8 = false;
        // Transfer state.
        std::uint16_t src_ptr = 0, dst_ptr = 0;
        std::uint32_t done = 0;  // transfers performed in this transfer count
        std::uint32_t total = 0; // transfer count (valid when total_known)
        bool total_known = false;
        bool in_block = false; // a block (TMODE 01/11) is being transferred
        std::vector<std::uint64_t> requests;                    // trigger times, sorted
        std::vector<std::pair<std::uint64_t, unsigned>> early; // triggers while configuring
        std::uint64_t dmareq_at = 0; // time DMAREQn was set
        std::uint32_t generation = 0; // bumped on abort/rearm/reset
    };

    // Time of a register access or trigger: the DMA's own event time while
    // it processes events, otherwise the HF-clock time of the access.
    std::uint64_t access_time() const;
    // Processes every DMA, flash-controller and AES event due at or before
    // HF time `limit`, earliest first.
    void advance(std::uint64_t limit);
    std::uint64_t own_next_event() const;
    std::uint64_t arbitration_time() const;
    std::uint64_t eligible_at(unsigned index) const;
    void record(unsigned index, unsigned number, std::uint64_t at);
    void start_fetch(std::uint8_t channels, std::uint64_t at);
    void load_descriptor(unsigned index, std::uint64_t at);
    void arbitrate(std::uint64_t at);
    void finish_slot(std::uint64_t at);
    void complete(unsigned index, std::uint64_t at);
    void abort(std::uint8_t channels, std::uint64_t at);
    void write_arm(std::uint8_t value, std::uint64_t at);
    void set_arm_bit(unsigned index, bool armed);
    static unsigned slot_cycles(const Channel &channel);

    Channel channels_[Channels];
    std::uint64_t fetch_free_ = 0;  // the descriptor fetch unit is busy until then
    std::uint64_t engine_free_ = 0; // end of the last data slot
    bool slot_active_ = false;
    unsigned slot_channel_ = 0;
    std::uint64_t slot_end_ = 0;
    std::uint32_t slot_generation_ = 0;
    unsigned last_granted_ = Channels - 1; // round-robin pointer
    std::uint64_t watermark_ = 0;          // every event up to here is processed
    std::uint64_t cursor_ = 0;             // time of the event being processed
    bool in_service_ = false;
    bool paused_ = false; // set by the debug interface, not by a reset
};

} // namespace mcs51emu::cc2530
