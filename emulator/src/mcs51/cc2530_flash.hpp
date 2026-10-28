#pragma once

// CC2530F256 flash controller (SWRU191F Chapter 6): FCTL, FADDRH/L, FWDATA,
// page erase, word programming (CPU from SRAM or DMA trigger FLASH),
// write-protection lock bits.

#include "cc2530_soc.hpp"

#include <cstdint>

namespace mcs51emu::cc2530 {

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
};

} // namespace mcs51emu::cc2530
