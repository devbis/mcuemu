// CC2530 AES coprocessor (SWRU191F Chapter 15).
//
// Model summary
// - ENCCS (0xB3), ENCDI (0xB1), ENCDO (0xB2) (15.10). A 1 written to
//   ENCCS.ST starts the command selected by ENCCS.CMD with the mode in
//   ENCCS.MODE (both sampled at ST): encrypt, decrypt, load key, load IV.
// - Key and IV (16 bytes each) are downloaded through ENCDI (15.2, DN108).
// - Data: ECB, CBC and CBC-MAC work on 128-bit blocks; CFB, OFB and CTR
//   divide the block into four 32-bit sub-blocks that are each loaded and
//   read out in turn (15.5). Input via ENCDI, output via ENCDO.
// - CBC-MAC produces no output; the MAC is the output of the last block,
//   processed in CBC mode (15.6).
// - ENC_DW / ENC_UP DMA triggers (15.9), the ENC interrupt on block
//   completion (15.8, S0CON.ENCIF_1/ENCIF_0 both set, Table 2-x) and on
//   key/IV load completion. 15.8 names only blocks; 15.4 says "After each
//   block is processed, an interrupt is generated". Load interrupts follow
//   recorded physical observations (cc2530-zigbee docs/DEBUGGING.md,
//   2026-09-17 LG AES runs: ENCCS=4C, S0CON=03 after KEY; fresh ENCIF pair
//   after both KEY and IV), not an explicit SWRU191F guarantee.
// - AES-128 cipher, inverse cipher and key expansion follow FIPS-197.
// - Registers, key and IV return to reset values when PM2/PM3 is entered
//   (15.2, 15.10, 4.6).
//
// Internal times are HF-clock times; the events are processed by
// Dma::advance() so ENC_DW/ENC_UP and the DMA accesses to ENCDI/ENCDO
// interleave at exact cycles.

#include "cc2530_aes.hpp"

#include "cc2530_dma.hpp"

#include <algorithm>
#include <array>

namespace mcs51emu::cc2530 {

namespace {

constexpr std::uint8_t SfrEncdi = 0xb1;
constexpr std::uint8_t SfrEncdo = 0xb2;
constexpr std::uint8_t SfrEnccs = 0xb3;

constexpr std::uint8_t EnccsMode = 0x70;
constexpr std::uint8_t EnccsRdy = 0x08;
constexpr std::uint8_t EnccsCmd = 0x06;
constexpr std::uint8_t EnccsSt = 0x01;
constexpr std::uint8_t EnccsReset = EnccsRdy; // 15.10: RDY resets to 1

// ENCCS.MODE (15.10).
constexpr std::uint8_t ModeCbc = 0;
constexpr std::uint8_t ModeCfb = 1;
constexpr std::uint8_t ModeOfb = 2;
constexpr std::uint8_t ModeCtr = 3;
constexpr std::uint8_t ModeEcb = 4;
constexpr std::uint8_t ModeCbcMac = 5;

// Model-defined: SWRU191F and the CC2530 data sheet (SWRS081) give no AES
// processing time. A block (or 32-bit sub-block, or key/IV load) is
// processed 40 system clocks after its last input byte. TI's Z-Stack HAL
// inserts a software delay before non-DMA ENCDO reads, so the latency is
// non-zero on the real chip; software that polls ENCCS.RDY or uses the DMA
// triggers is independent of the value.
constexpr std::uint64_t ProcessingCycles = 40;

// ---- AES-128 (FIPS-197) ----

constexpr std::uint8_t xtime(std::uint8_t a) {
    return static_cast<std::uint8_t>((a << 1) ^ ((a & 0x80) ? 0x1b : 0x00)); // 4.2.1
}

constexpr std::uint8_t gmul(std::uint8_t a, std::uint8_t b) {
    std::uint8_t product = 0;
    while (b) {
        if (b & 1)
            product = static_cast<std::uint8_t>(product ^ a);
        a = xtime(a);
        b = static_cast<std::uint8_t>(b >> 1);
    }
    return product;
}

constexpr std::uint8_t rotl8(std::uint8_t value, unsigned shift) {
    return static_cast<std::uint8_t>(value << shift | value >> (8 - shift));
}

struct SBoxes {
    std::array<std::uint8_t, 256> forward{};
    std::array<std::uint8_t, 256> inverse{};
};

// FIPS-197 5.1.1: multiplicative inverse in GF(2^8) ({00} maps to {00})
// followed by the affine transformation with c = {63}.
constexpr SBoxes make_sboxes() {
    SBoxes boxes;
    for (unsigned x = 0; x < 256; ++x) {
        std::uint8_t inverse = 0;
        if (x != 0)
            for (unsigned y = 1; y < 256; ++y)
                if (gmul(static_cast<std::uint8_t>(x), static_cast<std::uint8_t>(y)) == 1) {
                    inverse = static_cast<std::uint8_t>(y);
                    break;
                }
        const auto s = static_cast<std::uint8_t>(inverse ^ rotl8(inverse, 1) ^ rotl8(inverse, 2) ^
                                                 rotl8(inverse, 3) ^ rotl8(inverse, 4) ^ 0x63);
        boxes.forward[x] = s;
        boxes.inverse[s] = static_cast<std::uint8_t>(x);
    }
    return boxes;
}

constexpr SBoxes Boxes = make_sboxes();
static_assert(Boxes.forward[0x00] == 0x63 && Boxes.forward[0x53] == 0xed, "FIPS-197 Figure 7");
static_assert(Boxes.inverse[0xed] == 0x53, "FIPS-197 Figure 14");

// The state is kept as 16 bytes in input order: state[r][c] = s[r + 4c]
// (FIPS-197 3.4).
void add_round_key(std::uint8_t *s, const std::uint8_t *round_key) {
    for (unsigned i = 0; i < 16; ++i)
        s[i] = static_cast<std::uint8_t>(s[i] ^ round_key[i]);
}

void sub_bytes(std::uint8_t *s, const std::array<std::uint8_t, 256> &box) {
    for (unsigned i = 0; i < 16; ++i)
        s[i] = box[s[i]];
}

void shift_rows(std::uint8_t *s) { // 5.1.2: row r rotates left by r
    std::uint8_t t[16];
    for (unsigned r = 0; r < 4; ++r)
        for (unsigned c = 0; c < 4; ++c)
            t[r + 4 * c] = s[r + 4 * ((c + r) % 4)];
    std::copy(t, t + 16, s);
}

void inv_shift_rows(std::uint8_t *s) { // 5.3.1
    std::uint8_t t[16];
    for (unsigned r = 0; r < 4; ++r)
        for (unsigned c = 0; c < 4; ++c)
            t[r + 4 * ((c + r) % 4)] = s[r + 4 * c];
    std::copy(t, t + 16, s);
}

void mix_columns(std::uint8_t *s, std::uint8_t a, std::uint8_t b, std::uint8_t c, std::uint8_t d) {
    // Column multiplication by the circulant matrix (a b c d) (5.1.3/5.3.3).
    for (unsigned col = 0; col < 4; ++col) {
        std::uint8_t *x = s + 4 * col;
        const std::uint8_t x0 = x[0], x1 = x[1], x2 = x[2], x3 = x[3];
        x[0] = static_cast<std::uint8_t>(gmul(x0, a) ^ gmul(x1, b) ^ gmul(x2, c) ^ gmul(x3, d));
        x[1] = static_cast<std::uint8_t>(gmul(x0, d) ^ gmul(x1, a) ^ gmul(x2, b) ^ gmul(x3, c));
        x[2] = static_cast<std::uint8_t>(gmul(x0, c) ^ gmul(x1, d) ^ gmul(x2, a) ^ gmul(x3, b));
        x[3] = static_cast<std::uint8_t>(gmul(x0, b) ^ gmul(x1, c) ^ gmul(x2, d) ^ gmul(x3, a));
    }
}

constexpr unsigned Rounds = 10; // Nr for AES-128

} // namespace

Aes::Aes(Bus &bus) : Peripheral(bus) {
    bus_.claim_sfr(SfrEnccs, this, false);
    bus_.claim_sfr(SfrEncdi, this, false);
    bus_.claim_sfr(SfrEncdo, this, true); // a read pops the next output byte
}

void Aes::reset(ResetCause) {
    // 15.2: "Both the key and IV values are cleared by a reset".
    clear_state();
}

void Aes::clear_state() {
    active_ = false;
    command_ = Command::Encrypt;
    mode_ = 0;
    chunk_ = 0;
    chunk_size_ = 16;
    chunks_ = 1;
    std::fill(std::begin(input_), std::end(input_), 0);
    input_count_ = 0;
    input_complete_ = false;
    processing_ = false;
    processing_end_ = 0;
    std::fill(std::begin(output_), std::end(output_), 0);
    output_pos_ = 0;
    output_len_ = 0;
    std::fill(std::begin(key_), std::end(key_), 0);
    expand_key();
    std::fill(std::begin(iv_), std::end(iv_), 0);
    std::fill(std::begin(keystream_), std::end(keystream_), 0);
    std::fill(std::begin(feedback_), std::end(feedback_), 0);
    bus_.storage(sfr_reg(SfrEnccs)) = EnccsReset;
    bus_.storage(sfr_reg(SfrEncdi)) = 0;
    bus_.storage(sfr_reg(SfrEncdo)) = 0;
}

std::uint8_t Aes::read(Reg reg) {
    if (sfr_of(reg) != SfrEncdo || output_pos_ >= output_len_)
        return bus_.storage(reg);
    // ENCDO: the next byte of the processed (sub-)block. Model-defined: with
    // no output pending, the last value read (or written) is returned.
    const std::uint8_t value = output_[output_pos_++];
    bus_.storage(reg) = value;
    const std::uint64_t at = access_time();
    if (output_pos_ < output_len_)
        bus_.dma_trigger(trigger::EncUp); // more output to upload (15.9)
    else
        try_start_processing(at); // output drained: the next chunk may run
    return value;
}

std::uint8_t Aes::peek(Reg reg) const {
    if (sfr_of(reg) == SfrEncdo)
        return output_value();
    return bus_.storage(reg);
}

std::uint8_t Aes::output_value() const {
    return output_pos_ < output_len_ ? output_[output_pos_] : bus_.storage(sfr_reg(SfrEncdo));
}

void Aes::write(Reg reg, std::uint8_t value) {
    const std::uint64_t at = access_time();
    switch (sfr_of(reg)) {
    case SfrEnccs: {
        // Bit 7 R0, RDY (bit 3) read-only, ST (bit 0) R/W1 H0: the command
        // starts at once, so ST always reads 0.
        std::uint8_t &enccs = bus_.storage(reg);
        enccs = static_cast<std::uint8_t>((value & (EnccsMode | EnccsCmd)) | (enccs & EnccsRdy));
        if (value & EnccsSt)
            start_command(value, at);
        break;
    }
    case SfrEncdi:
        bus_.storage(reg) = value;
        accept_input(value, at);
        break;
    default: // ENCDO is R/W (15.10); a write only changes the register byte
        bus_.storage(reg) = value;
        break;
    }
}

std::uint64_t Aes::deadline() const { return bus_.hf_deadline(next_event()); }

void Aes::service(std::uint64_t) { bus_.soc().dma->advance(bus_.hf_now()); }

void Aes::power_mode_changed(PowerMode, PowerMode to) {
    // 15.10: "The registers return to their reset value when the chip enters
    // PM2 or PM3"; 15.2: key and IV are cleared when PM2 or PM3 is entered.
    // In PM1 the AES halts with the HF clock and keeps its state.
    if (to == PowerMode::Pm2 || to == PowerMode::Pm3)
        clear_state();
}

std::uint64_t Aes::access_time() const { return bus_.soc().dma->access_time(); }

std::uint64_t Aes::next_event() const { return processing_ ? processing_end_ : Never; }

void Aes::process_until(std::uint64_t limit) {
    while (processing_ && processing_end_ <= limit)
        finish_processing(processing_end_);
}

bool Aes::needs_input() const { return active_ && !input_complete_; }

bool Aes::dma_level(unsigned number) const {
    // 15.9: ENC_DW "is active when input data must be downloaded", ENC_UP
    // "when output data must be uploaded".
    if (number == trigger::EncDw)
        return needs_input();
    if (number == trigger::EncUp)
        return output_pos_ < output_len_;
    return false;
}

void Aes::set_ready(bool ready) {
    std::uint8_t &enccs = bus_.storage(sfr_reg(SfrEnccs));
    enccs = static_cast<std::uint8_t>(ready ? enccs | EnccsRdy : enccs & ~EnccsRdy);
}

void Aes::start_command(std::uint8_t enccs, std::uint64_t at) {
    const auto command = static_cast<Command>((enccs & EnccsCmd) >> 1);
    const auto mode = static_cast<std::uint8_t>((enccs & EnccsMode) >> 4);
    const bool load = command == Command::LoadKey || command == Command::LoadIv;
    if (!load && mode > ModeCbcMac) {
        // MODE 110/111 are reserved (15.10): the start is ignored.
        bus_.note_unmodeled(sfr_reg(SfrEnccs), true);
        return;
    }
    // Model-defined: a new start abandons a command in progress (its
    // partial input and a running computation). 15.2: "A key load or IV
    // load operation aborts any processing that could be running" - a load
    // also discards output not yet read; a data start keeps it (it belongs
    // to the previous, completed block).
    processing_ = false;
    input_count_ = 0;
    input_complete_ = false;
    if (load) {
        output_pos_ = 0;
        output_len_ = 0;
    }
    active_ = true;
    command_ = command;
    mode_ = mode;
    chunk_ = 0;
    const bool sub_blocks = !load && (mode == ModeCfb || mode == ModeOfb || mode == ModeCtr);
    chunk_size_ = sub_blocks ? 4 : 16; // 15.5: four 32-bit blocks
    chunks_ = sub_blocks ? 4 : 1;
    set_ready(false);
    // 15.4: "Writing a start command generates a DMA trigger".
    bus_.dma_trigger(trigger::EncDw);
    (void)at;
}

void Aes::accept_input(std::uint8_t value, std::uint64_t at) {
    if (!needs_input())
        return; // Model-defined: input without a pending command is ignored
    input_[chunk_ * chunk_size_ % 16 + input_count_] = value;
    if (++input_count_ < chunk_size_) {
        bus_.dma_trigger(trigger::EncDw); // more input to download (15.9)
        return;
    }
    input_complete_ = true;
    try_start_processing(at);
}

void Aes::try_start_processing(std::uint64_t at) {
    // 15.1: "the result must be read out before the next block can be
    // processed" - a chunk starts once its input is complete and the output
    // of the previous chunk has been read.
    if (!input_complete_ || processing_ || output_pos_ < output_len_)
        return;
    processing_ = true;
    processing_end_ = at + ProcessingCycles;
}

void Aes::finish_processing(std::uint64_t) {
    processing_ = false;
    input_complete_ = false;
    input_count_ = 0;
    switch (command_) {
    case Command::LoadKey:
    case Command::LoadIv:
        // Observed on hardware (see the model summary): a completed load
        // sets RDY and both ENCIF flags like a processed block.
        std::copy(std::begin(input_), std::end(input_), command_ == Command::LoadKey ? key_ : iv_);
        if (command_ == Command::LoadKey)
            expand_key();
        active_ = false;
        set_ready(true);
        bus_.raise_irq(Irq::Enc);
        return;
    default:
        break;
    }
    const bool decrypt = command_ == Command::Decrypt;
    std::uint8_t block[16];
    unsigned produced = 0;
    switch (mode_) {
    case ModeEcb:
        if (decrypt)
            decrypt_block(input_, output_);
        else
            encrypt_block(input_, output_);
        produced = 16;
        break;
    case ModeCbc:
        if (decrypt) { // P = D(C) xor IV, IV = C
            decrypt_block(input_, block);
            for (unsigned i = 0; i < 16; ++i)
                output_[i] = static_cast<std::uint8_t>(block[i] ^ iv_[i]);
            std::copy(std::begin(input_), std::end(input_), iv_);
        } else { // C = E(P xor IV), IV = C
            for (unsigned i = 0; i < 16; ++i)
                block[i] = static_cast<std::uint8_t>(input_[i] ^ iv_[i]);
            encrypt_block(block, output_);
            std::copy(output_, output_ + 16, iv_);
        }
        produced = 16;
        break;
    case ModeCbcMac: // 15.6: chaining only; the MAC comes from the last
                     // block processed in CBC mode. Decryption is "similar".
        for (unsigned i = 0; i < 16; ++i)
            block[i] = static_cast<std::uint8_t>(input_[i] ^ iv_[i]);
        encrypt_block(block, iv_);
        break;
    default: { // CFB, OFB, CTR: one 32-bit sub-block (15.5)
        if (chunk_ == 0)
            encrypt_block(iv_, keystream_);
        const unsigned base = chunk_ * 4;
        for (unsigned i = 0; i < 4; ++i) {
            const std::uint8_t in = input_[base + i];
            const auto out = static_cast<std::uint8_t>(in ^ keystream_[base + i]);
            output_[i] = out;
            // CFB feeds back the ciphertext (encrypt: output, decrypt: input).
            feedback_[base + i] = decrypt ? in : out;
        }
        produced = 4;
        if (chunk_ + 1 == chunks_) {
            if (mode_ == ModeCfb) {
                std::copy(std::begin(feedback_), std::end(feedback_), iv_);
            } else if (mode_ == ModeOfb) {
                std::copy(std::begin(keystream_), std::end(keystream_), iv_);
            } else {
                // Model-defined: the counter is the last 32 bits of the
                // IV/nonce, incremented big-endian per block (15.7: "the
                // automatic generation of CTR works on 32 bits").
                for (unsigned i = 16; i-- > 12;)
                    if (++iv_[i] != 0)
                        break;
            }
        }
        break;
    }
    }
    output_pos_ = 0;
    output_len_ = produced;
    if (produced)
        bus_.dma_trigger(trigger::EncUp); // 15.9: output must be uploaded
    if (++chunk_ < chunks_) {
        bus_.dma_trigger(trigger::EncDw); // next 32-bit sub-block
        return;
    }
    // 15.8: "The AES interrupt, ENC, is produced when encryption or
    // decryption of a block is completed" (S0CON.ENCIF_1 and ENCIF_0).
    active_ = false;
    set_ready(true);
    bus_.raise_irq(Irq::Enc);
}

void Aes::expand_key() {
    // FIPS-197 5.2, Nk = 4: w[i] = w[i-4] xor SubWord(RotWord(w[i-1])) xor
    // Rcon[i/4] for i % 4 == 0, else w[i-4] xor w[i-1].
    std::copy(std::begin(key_), std::end(key_), round_keys_);
    std::uint8_t rcon = 1;
    for (unsigned i = 4; i < 4 * (Rounds + 1); ++i) {
        std::uint8_t temp[4];
        std::copy(round_keys_ + 4 * (i - 1), round_keys_ + 4 * i, temp);
        if (i % 4 == 0) {
            const std::uint8_t first = temp[0];
            temp[0] = static_cast<std::uint8_t>(Boxes.forward[temp[1]] ^ rcon);
            temp[1] = Boxes.forward[temp[2]];
            temp[2] = Boxes.forward[temp[3]];
            temp[3] = Boxes.forward[first];
            rcon = xtime(rcon);
        }
        for (unsigned j = 0; j < 4; ++j)
            round_keys_[4 * i + j] = static_cast<std::uint8_t>(round_keys_[4 * (i - 4) + j] ^ temp[j]);
    }
}

void Aes::encrypt_block(const std::uint8_t *in, std::uint8_t *out) const {
    // FIPS-197 5.1, Figure 5.
    std::uint8_t s[16];
    std::copy(in, in + 16, s);
    add_round_key(s, round_keys_);
    for (unsigned round = 1; round < Rounds; ++round) {
        sub_bytes(s, Boxes.forward);
        shift_rows(s);
        mix_columns(s, 0x02, 0x03, 0x01, 0x01);
        add_round_key(s, round_keys_ + 16 * round);
    }
    sub_bytes(s, Boxes.forward);
    shift_rows(s);
    add_round_key(s, round_keys_ + 16 * Rounds);
    std::copy(s, s + 16, out);
}

void Aes::decrypt_block(const std::uint8_t *in, std::uint8_t *out) const {
    // FIPS-197 5.3, Figure 12 (InvCipher).
    std::uint8_t s[16];
    std::copy(in, in + 16, s);
    add_round_key(s, round_keys_ + 16 * Rounds);
    for (unsigned round = Rounds - 1; round >= 1; --round) {
        inv_shift_rows(s);
        sub_bytes(s, Boxes.inverse);
        add_round_key(s, round_keys_ + 16 * round);
        mix_columns(s, 0x0e, 0x0b, 0x0d, 0x09);
    }
    inv_shift_rows(s);
    sub_bytes(s, Boxes.inverse);
    add_round_key(s, round_keys_);
    std::copy(s, s + 16, out);
}

} // namespace mcs51emu::cc2530
