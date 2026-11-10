#include "efr32emu/efr32mg21.hpp"

#include "efr32/blocks.hpp"
#include "efr32/rail_map.hpp"
#include "mcuemu/aes.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

// RADIOAES (0x44000000, secure; the Zigbee stack's AES): a Silicon Labs CRYPTOACC-style engine that is fed through
// scatter-gather descriptors. Registers (efr32mg21_aes.h, confirmed by the Tuya ZS3L image's accesses): FETCHADDR +0x04,
// PUSHADDR +0x14, CTRL +0x34, CMD +0x38 (bit 0 start fetcher, bit 1 start pusher), STATUS +0x3C (busy bits; the operation
// completes at once here). A descriptor is {address, next (bit 0 stops), length | flags, tag}; the formats and the way
// the SDK drives the engine come from sli_protocol_crypto_radioaes.c (simplicity_sdk). Modes: ECB, CBC, CTR and CCM
// (the NWK security of the stack); CMAC, DES, hash and ChaCha-Poly are not modelled.

namespace efr32emu {

namespace {

constexpr std::uint32_t FlagConst = 0x10000000u, FlagDiscard = 0x40000000u, LengthMask = 0x0FFFFFFFu;
constexpr std::uint32_t TagConfig = 0x10, TagHeader = 0x40;
constexpr std::uint32_t ModeEcb = 0x100, ModeCbc = 0x200, ModeCtr = 0x400, ModeCcm = 0x2000, ModeMask = 0x1FF00;
constexpr unsigned OffsetKey = 8, OffsetIv = 40;

struct Descriptor {
    std::uint32_t address, next, length, tag;
};

struct Element {
    std::vector<std::uint8_t> bytes; // valid bytes
    bool header;
    bool last;
};

using Aes16 = std::array<std::uint8_t, 16>;

} // namespace

void Efr32mg21::run_radioaes() {
    auto read_descriptor = [&](std::uint32_t at, Descriptor &d) {
        return core_->read_memory(at, &d, sizeof d);
    };
    std::array<std::uint8_t, 128> cfg{};
    std::vector<Element> elements;
    // Fetcher.
    std::uint32_t at = rail_regs_->peek(rail::RadioAesFetchAddr);
    for (unsigned guard = 0; at > 1 && guard != 64; ++guard) {
        Descriptor d{};
        if (!read_descriptor(at, d))
            break;
        const std::uint32_t length = d.length & LengthMask;
        std::vector<std::uint8_t> data(length);
        if (d.length & FlagConst) {
            std::uint8_t word[4] = {};
            core_->read_memory(d.address, word, 4);
            for (std::uint32_t i = 0; i != length; ++i)
                data[i] = word[i & 3u];
        } else if (length != 0 && !core_->read_memory(d.address, data.data(), length)) {
            break;
        }
        if (d.tag & TagConfig) {
            const unsigned offset = (d.tag >> 8) & 0xFFu;
            for (std::uint32_t i = 0; i != length && offset + i < cfg.size(); ++i)
                cfg[offset + i] = data[i];
        } else {
            elements.push_back({std::move(data), (d.tag & TagHeader) != 0, (d.tag & 0x20u) != 0});
        }
        if (d.next & 1u)
            break;
        at = d.next & ~3u;
    }

    std::uint32_t mode_word = 0;
    std::memcpy(&mode_word, cfg.data(), 4);
    const bool decrypt = (mode_word & 1u) != 0;
    const unsigned key_bytes = (mode_word & 0xCu) == 0x4u ? 32u : (mode_word & 0xCu) == 0x8u ? 24u : 16u;
    const std::uint8_t *key = cfg.data() + OffsetKey;
    const std::uint8_t *iv = cfg.data() + OffsetIv;
    auto aes = [&](const Aes16 &in, bool dec = false) {
        Aes16 out{};
        mcuemu::aes_block(key, key_bytes, in.data(), out.data(), dec);
        return out;
    };
    auto padded = [](std::vector<std::uint8_t> bytes) {
        bytes.resize((bytes.size() + 15u) & ~15u, 0);
        return bytes;
    };

    std::vector<std::uint8_t> out; // what the pusher takes, in order
    const std::uint32_t mode = mode_word & ModeMask;
    if (mode == ModeEcb || mode == ModeCbc || mode == ModeCtr) {
        Aes16 chain{}, counter{};
        std::memcpy(chain.data(), iv, 16);
        counter = chain;
        for (const Element &e : elements) {
            const std::vector<std::uint8_t> bytes = padded(e.bytes);
            for (std::size_t offset = 0; offset < bytes.size(); offset += 16) {
                Aes16 in{}, result{};
                std::copy_n(bytes.begin() + static_cast<std::ptrdiff_t>(offset), 16, in.begin());
                if (mode == ModeEcb) {
                    result = aes(in, decrypt);
                } else if (mode == ModeCbc) {
                    if (decrypt) {
                        result = aes(in, true);
                        for (unsigned i = 0; i != 16; ++i)
                            result[i] ^= chain[i];
                        chain = in;
                    } else {
                        for (unsigned i = 0; i != 16; ++i)
                            in[i] ^= chain[i];
                        result = aes(in);
                        chain = result;
                    }
                } else {
                    const Aes16 stream = aes(counter);
                    for (unsigned i = 0; i != 16; ++i)
                        result[i] = in[i] ^ stream[i];
                    for (int i = 15; i >= 0 && ++counter[static_cast<unsigned>(i)] == 0; --i) {
                    }
                }
                out.insert(out.end(), result.begin(), result.end());
            }
        }
        if (mode == ModeCtr && (mode_word & 0x20u))
            out.insert(out.end(), counter.begin(), counter.end()); // CX_SAVE: the counter after the last block
    } else if (mode == ModeCcm) {
        // Header elements (B0 first, then the encoded AAD), then payload, then, when decrypting with a tag, the tag.
        std::vector<std::uint8_t> header;
        std::vector<const Element *> payload;
        for (const Element &e : elements) {
            if (e.header) {
                const std::vector<std::uint8_t> bytes = padded(e.bytes);
                header.insert(header.end(), bytes.begin(), bytes.end());
            } else {
                payload.push_back(&e);
            }
        }
        if (header.size() >= 16) {
            const std::uint8_t flags = header[0];
            const unsigned l = (flags & 7u) + 1u; // length field size
            const unsigned m_code = (flags >> 3) & 7u;
            const unsigned m = m_code != 0 ? 2u * m_code + 2u : 0u;
            std::uint32_t n = 0;
            for (unsigned i = 0; i != l; ++i)
                n = (n << 8) | header[15u - l + 1u + i];
            auto counter_block = [&](std::uint32_t index) {
                Aes16 a{};
                a[0] = static_cast<std::uint8_t>(l - 1u);
                std::copy_n(header.begin() + 1, 15 - l, a.begin() + 1);
                for (unsigned i = 0; i != l; ++i)
                    a[15u - i] = static_cast<std::uint8_t>(index >> (8 * i));
                return a;
            };
            std::vector<std::uint8_t> data;
            std::vector<std::uint8_t> tag_in;
            for (std::size_t i = 0; i != payload.size(); ++i) {
                const bool is_tag = decrypt && m != 0 && i + 1 == payload.size() && payload.size() > 1;
                (is_tag ? tag_in : data).insert((is_tag ? tag_in : data).end(), payload[i]->bytes.begin(),
                                                payload[i]->bytes.end());
            }
            if (data.size() > n)
                data.resize(n);
            data.resize(n, 0);
            // CTR over the payload with counters 1.., on the way in for decrypt and out for encrypt.
            std::vector<std::uint8_t> processed(n);
            for (std::uint32_t offset = 0; offset < n; offset += 16) {
                const Aes16 stream = aes(counter_block(offset / 16u + 1u));
                for (unsigned i = 0; i != 16 && offset + i < n; ++i)
                    processed[offset + i] = data[offset + i] ^ stream[i];
            }
            const std::vector<std::uint8_t> &plain = decrypt ? processed : data;
            // CBC-MAC: B0, the rest of the header, then the plaintext padded with zeros.
            std::vector<std::uint8_t> mac_input = header;
            mac_input.insert(mac_input.end(), plain.begin(), plain.end());
            mac_input.resize((mac_input.size() + 15u) & ~15u, 0);
            Aes16 x{};
            for (std::size_t offset = 0; offset < mac_input.size(); offset += 16) {
                for (unsigned i = 0; i != 16; ++i)
                    x[i] ^= mac_input[offset + i];
                x = aes(x);
            }
            const Aes16 s0 = aes(counter_block(0));
            Aes16 tag{};
            for (unsigned i = 0; i != 16; ++i)
                tag[i] = x[i] ^ s0[i];
            // Stream out: the header reflected, the processed payload padded to a block, the tag block.
            out = header;
            std::vector<std::uint8_t> payload_out = padded(processed);
            if (payload_out.empty())
                payload_out.assign(16, 0);
            out.insert(out.end(), payload_out.begin(), payload_out.end());
            Aes16 verdict{};
            if (decrypt) {
                bool ok = true;
                for (unsigned i = 0; i != m; ++i)
                    ok = ok && i < tag_in.size() && tag_in[i] == tag[i];
                if (!ok)
                    verdict.fill(0xFF);
            } else {
                std::copy_n(tag.begin(), m, verdict.begin());
            }
            out.insert(out.end(), verdict.begin(), verdict.end());
        }
    }

    // Pusher.
    at = rail_regs_->peek(rail::RadioAesPushAddr);
    std::size_t taken = 0;
    for (unsigned guard = 0; at > 1 && guard != 64; ++guard) {
        Descriptor d{};
        if (!read_descriptor(at, d))
            break;
        const std::uint32_t length = d.length & LengthMask;
        if (!(d.length & FlagDiscard) && d.address != 0 && length != 0) {
            std::vector<std::uint8_t> chunk(length, 0);
            for (std::uint32_t i = 0; i != length && taken + i < out.size(); ++i)
                chunk[i] = out[taken + i];
            core_->write_memory(d.address, chunk.data(), chunk.size());
        }
        taken += length;
        if (d.next & 1u)
            break;
        at = d.next & ~3u;
    }
}

} // namespace efr32emu
