#pragma once

// CC2530F256 flash controller (SWRU191F Chapter 6): FCTL, FADDRH/L, FWDATA,
// page erase, word programming (CPU from SRAM or DMA trigger FLASH),
// write-protection lock bits, plus the flash fault model: SWRU191F 6.2.2
// write-limit tracking, outcomes of operations cut by a reset or power cut,
// power-cut injection and the per-page / per-operation instrumentation.

#include "cc2530_soc.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace mcs51emu::cc2530 {

// Deterministic pseudo-random value for the model-defined fault outcomes
// (splitmix64 mixing): the same (seed, stream, index) always gives the same
// value, different seeds give independent sequences.
std::uint64_t fault_draw(std::uint64_t seed, std::uint64_t stream, std::uint64_t index);

class FlashController final : public Peripheral {
public:
    explicit FlashController(Bus &bus);
    const char *name() const override { return "FLASH"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;

    // ---- fault model (docs/cc2530.md "Flash fault injection") ----
    // Counters go to `diagnostics`; due power cuts / trigger actions are
    // handed to `request` (the chip applies them at the instruction
    // boundary like any reset request).
    void bind(Cc2530Diagnostics &diagnostics, std::function<void(Cc2530FaultAction)> request);
    void configure(const Cc2530FlashFaultConfig &config) { config_ = config; }
    const Cc2530FlashFaultConfig &config() const noexcept { return config_; }
    // A reset (`cut` Reset, with `cause`) or power cut ends the operation in
    // flight with the configured outcome. Must run before reset(), while
    // virtual time still shows the cut.
    void interrupt_operation(Cc2530FlashCut cut, ResetCause cause);
    // The flash content was replaced (image load): the operation in flight,
    // the 6.2.2 tracking, the log and the operation serials are dropped.
    void flash_replaced();
    // Virtual time restarts at 0 (power_on_reset()): matched triggers whose
    // fire time is anchored in the old timeline are dropped.
    void time_restarted();
    // Bookkeeping of an erase of pages [first_page, first_page + count) that
    // another path (e.g. a debug-interface CHIP_ERASE) has applied to the
    // flash: per-page erase counters, 6.2.2 tracking restart and a log entry.
    // Does not touch the flash content.
    void record_erase(unsigned first_page, unsigned count,
                      Cc2530FlashOpKind op = Cc2530FlashOpKind::ChipErase,
                      Cc2530FlashOutcome outcome = Cc2530FlashOutcome::Completed,
                      std::uint64_t elapsed_cycles = 0);
    void schedule_power_cut(std::uint64_t at_cycle);
    void add_trigger(const Cc2530FlashTrigger &trigger);
    std::size_t pending_faults() const noexcept { return cuts_.size() + triggers_.size(); }
    const std::vector<Cc2530FlashLogEntry> &log() const noexcept { return log_; }
    std::vector<Cc2530FlashLogEntry> take_log();

private:
    // Events are processed by Dma::advance() together with the DMA events
    // (the FLASH DMA trigger and DMA writes to FWDATA interleave exactly).
    friend class Dma;

    enum class Op : std::uint8_t { None, Erase, Write };

    std::uint64_t next_event() const;
    void process_until(std::uint64_t limit);
    std::uint64_t access_time() const;
    bool page_locked(unsigned page) const;
    std::uint16_t faddr() const;
    void set_faddr(std::uint16_t word_address);
    void start_erase(std::uint64_t at, bool then_write);
    void finish_erase(std::uint64_t at);
    void start_write(std::uint64_t at);
    void start_word(std::uint64_t at, const std::uint8_t *data);
    void finish_word(std::uint64_t at);
    void end_write();
    void stall_if_executing_from_flash(std::uint64_t until);
    void request_data();

    // 6.2.2 bookkeeping since the last erase of one page.
    struct PageWear {
        std::array<std::uint32_t, 512> zero_once{};  // bit written 0 at least once
        std::array<std::uint32_t, 512> zero_twice{}; // ... at least twice
        std::array<std::uint8_t, 512> word_writes{}; // saturates at 255
        std::uint32_t page_writes{};
    };
    struct Trigger {
        Cc2530FlashTrigger spec;
        std::uint64_t matched{};    // matching operation starts so far
        std::uint64_t fire_hf{Never}; // HF time of the action once matched
    };
    // Programs `data` into the word at `word_address` with a model-defined
    // fraction (`elapsed` of WordWriteCycles; all of it when complete),
    // updates the 6.2.2 tracking and returns the violated limits.
    std::uint8_t program_word(std::uint16_t word_address, const std::uint8_t *data,
                              std::uint64_t serial, std::uint64_t elapsed,
                              std::uint32_t &changed);
    void op_started(Cc2530FlashOpKind op, unsigned page, std::uint64_t at);
    // Per-page erase counters and 6.2.2 tracking restart of an applied erase.
    void erase_applied(unsigned first_page, unsigned count);
    void log_op(std::uint64_t at_hf, Cc2530FlashOpKind op, std::uint32_t address,
                Cc2530FlashOutcome outcome, Cc2530FlashCut cut, ResetCause cause,
                std::uint64_t elapsed, std::uint32_t changed, std::uint8_t violations);
    std::uint64_t fault_deadline() const;
    void fire_due_faults(std::uint64_t now);

    Op op_ = Op::None;
    bool write_after_erase_ = false; // ERASE and WRITE set together
    std::uint64_t erase_end_ = 0;
    unsigned erase_page_ = 0;
    std::uint64_t window_end_ = 0; // the write sequence times out here
    bool programming_ = false;
    std::uint64_t program_end_ = 0;
    std::uint16_t program_address_ = 0; // word address being programmed
    std::uint8_t program_data_[4]{};
    std::uint8_t buffer_[4]{}; // FWDATA write buffer
    unsigned buffered_ = 0;
    bool full_ = false;
    bool abort_ = false;
    std::uint8_t cache_mode_ = 1;

    // Fault model state. Kept across resets; flash_replaced() clears the
    // tracking, the log and the serials.
    Cc2530FlashFaultConfig config_;
    Cc2530Diagnostics *diagnostics_{};
    std::function<void(Cc2530FaultAction)> request_fault_;
    std::uint64_t op_serial_ = 0; // erases and word writes started
    std::uint64_t erase_serial_ = 0;
    std::uint64_t program_serial_ = 0;
    std::array<std::unique_ptr<PageWear>, Cc2530F256::FlashSize / 2048> wear_;
    std::vector<Cc2530FlashLogEntry> log_;
    std::vector<std::uint64_t> cuts_; // scheduled power cuts (now() domain), sorted
    std::vector<Trigger> triggers_;
};

} // namespace mcs51emu::cc2530
