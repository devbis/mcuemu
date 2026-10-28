#pragma once

// CC2530 AES coprocessor (SWRU191F Chapter 15): ENCCS/ENCDI/ENCDO, key and
// IV loading, ECB/CBC/CFB/OFB/CTR/CBC-MAC, ENC_DW/ENC_UP DMA triggers and
// the ENC interrupt.

#include "cc2530_soc.hpp"

#include <cstdint>

namespace mcs51emu::cc2530 {

class Aes final : public Peripheral {
public:
    explicit Aes(Bus &bus);
    const char *name() const override { return "AES"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;

private:
    // Events are processed by Dma::advance() together with the DMA events
    // (ENC_DW/ENC_UP triggers and DMA accesses to ENCDI/ENCDO interleave
    // exactly); the DMA also samples the ENC_DW/ENC_UP request levels.
    friend class Dma;

    enum class Command : std::uint8_t { Encrypt = 0, Decrypt = 1, LoadKey = 2, LoadIv = 3 };

    std::uint64_t next_event() const;
    void process_until(std::uint64_t limit);
    bool dma_level(unsigned number) const;
    bool needs_input() const;
    std::uint64_t access_time() const;
    void clear_state();
    void start_command(std::uint8_t enccs, std::uint64_t at);
    void accept_input(std::uint8_t value, std::uint64_t at);
    void try_start_processing(std::uint64_t at);
    void finish_processing(std::uint64_t at);
    void set_ready(bool ready);
    std::uint8_t output_value() const;

    // AES-128, FIPS-197.
    void expand_key();
    void encrypt_block(const std::uint8_t *in, std::uint8_t *out) const;
    void decrypt_block(const std::uint8_t *in, std::uint8_t *out) const;

    bool active_ = false; // a command is in progress (ENCCS.RDY = 0)
    Command command_ = Command::Encrypt;
    std::uint8_t mode_ = 0; // ENCCS.MODE sampled at ST
    unsigned chunk_ = 0;    // 16-byte block or 4-byte sub-block being handled
    unsigned chunk_size_ = 16;
    unsigned chunks_ = 1;
    std::uint8_t input_[16]{};
    unsigned input_count_ = 0;
    bool input_complete_ = false; // chunk loaded (waiting or processing)
    bool processing_ = false;
    std::uint64_t processing_end_ = 0;
    std::uint8_t output_[16]{};
    unsigned output_pos_ = 0;
    unsigned output_len_ = 0;
    std::uint8_t key_[16]{};
    std::uint8_t round_keys_[176]{};
    std::uint8_t iv_[16]{};
    std::uint8_t keystream_[16]{}; // CFB/OFB/CTR: E(IV) of the current block
    std::uint8_t feedback_[16]{};  // CFB: ciphertext of the current block
};

} // namespace mcs51emu::cc2530
