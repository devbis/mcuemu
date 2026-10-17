#include "aes.hpp"

#include <cstring>

namespace esp32emu {

namespace {

struct Tables {
    std::uint8_t sbox[256], inv[256];
    Tables() {
        std::uint8_t p = 1, q = 1;
        do {
            p = static_cast<std::uint8_t>(p ^ (p << 1) ^ (p & 0x80 ? 0x1b : 0));
            q ^= q << 1;
            q ^= q << 2;
            q ^= q << 4;
            if (q & 0x80)
                q ^= 0x09;
            const std::uint8_t x =
                static_cast<std::uint8_t>(q ^ (q << 1 | q >> 7) ^ (q << 2 | q >> 6) ^ (q << 3 | q >> 5) ^ (q << 4 | q >> 4));
            sbox[p] = x ^ 0x63;
        } while (p != 1);
        sbox[0] = 0x63;
        for (unsigned i = 0; i < 256; ++i)
            inv[sbox[i]] = static_cast<std::uint8_t>(i);
    }
};

const Tables &tables() {
    static const Tables t;
    return t;
}

std::uint8_t xtime(std::uint8_t x) { return static_cast<std::uint8_t>(x << 1 ^ (x & 0x80 ? 0x1b : 0)); }
std::uint8_t mul(std::uint8_t a, std::uint8_t b) {
    std::uint8_t r = 0;
    for (; b; b >>= 1, a = xtime(a))
        if (b & 1)
            r ^= a;
    return r;
}

} // namespace

void aes_block(const std::uint8_t *key, unsigned key_bytes, const std::uint8_t in[16], std::uint8_t out[16],
               bool decrypt) {
    const Tables &t = tables();
    const unsigned nk = key_bytes / 4, rounds = nk + 6;
    std::uint8_t w[240];
    std::memcpy(w, key, key_bytes);
    std::uint8_t rcon = 1;
    for (unsigned i = nk; i < 4 * (rounds + 1); ++i) {
        std::uint8_t temp[4];
        std::memcpy(temp, w + 4 * (i - 1), 4);
        if (i % nk == 0) {
            const std::uint8_t first = temp[0];
            temp[0] = t.sbox[temp[1]] ^ rcon;
            temp[1] = t.sbox[temp[2]];
            temp[2] = t.sbox[temp[3]];
            temp[3] = t.sbox[first];
            rcon = xtime(rcon);
        } else if (nk > 6 && i % nk == 4) {
            for (auto &b : temp)
                b = t.sbox[b];
        }
        for (unsigned j = 0; j < 4; ++j)
            w[4 * i + j] = w[4 * (i - nk) + j] ^ temp[j];
    }
    std::uint8_t s[16];
    std::memcpy(s, in, 16);
    auto add = [&](unsigned round) {
        for (unsigned i = 0; i < 16; ++i)
            s[i] ^= w[16 * round + i];
    };
    auto sub = [&](const std::uint8_t *box) {
        for (auto &b : s)
            b = box[b];
    };
    auto shift = [&](bool inverse) {
        std::uint8_t c[16];
        for (unsigned col = 0; col < 4; ++col)
            for (unsigned row = 0; row < 4; ++row)
                c[4 * ((col + (inverse ? row : 4 - row)) % 4) + row] = s[4 * col + row];
        std::memcpy(s, c, 16);
    };
    auto mix = [&](bool inverse) {
        for (unsigned col = 0; col < 4; ++col) {
            std::uint8_t *a = s + 4 * col;
            const std::uint8_t a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
            if (!inverse) {
                a[0] = mul(a0, 2) ^ mul(a1, 3) ^ a2 ^ a3;
                a[1] = a0 ^ mul(a1, 2) ^ mul(a2, 3) ^ a3;
                a[2] = a0 ^ a1 ^ mul(a2, 2) ^ mul(a3, 3);
                a[3] = mul(a0, 3) ^ a1 ^ a2 ^ mul(a3, 2);
            } else {
                a[0] = mul(a0, 14) ^ mul(a1, 11) ^ mul(a2, 13) ^ mul(a3, 9);
                a[1] = mul(a0, 9) ^ mul(a1, 14) ^ mul(a2, 11) ^ mul(a3, 13);
                a[2] = mul(a0, 13) ^ mul(a1, 9) ^ mul(a2, 14) ^ mul(a3, 11);
                a[3] = mul(a0, 11) ^ mul(a1, 13) ^ mul(a2, 9) ^ mul(a3, 14);
            }
        }
    };
    if (!decrypt) {
        add(0);
        for (unsigned r = 1; r < rounds; ++r) {
            sub(t.sbox);
            shift(false);
            mix(false);
            add(r);
        }
        sub(t.sbox);
        shift(false);
        add(rounds);
    } else {
        add(rounds);
        for (unsigned r = rounds - 1; r > 0; --r) {
            shift(true);
            sub(t.inv);
            add(r);
            mix(true);
        }
        shift(true);
        sub(t.inv);
        add(0);
    }
    std::memcpy(out, s, 16);
}

namespace {
constexpr std::uint32_t Key = 0x00, TextIn = 0x20, TextOut = 0x30, Mode = 0x40, Endian = 0x44, Trigger = 0x48, State = 0x4c;

// Registers -> byte stream under the ENDIAN swaps (`word_swap`, `byte_swap`).
void words_to_bytes(const std::uint32_t *words, unsigned count, bool word_swap, bool byte_swap, std::uint8_t *out) {
    for (unsigned i = 0; i < count; ++i) {
        std::uint32_t w = words[word_swap ? count - 1 - i : i];
        if (byte_swap)
            w = ((w >> 24) | ((w >> 8) & 0xff00u) | ((w << 8) & 0xff0000u) | (w << 24));
        std::memcpy(out + 4 * i, &w, 4);
    }
}
} // namespace

std::uint32_t Aes::read(std::uint32_t offset, unsigned size) {
    if ((offset & ~3u) == State)
        return 0; // idle: the transform is instant
    return StatusBlock::read(offset, size);
}

void Aes::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    StatusBlock::write(offset, value, size);
    if ((offset & ~3u) == Trigger && (value & 1u))
        run();
}

void Aes::run() {
    const std::uint32_t mode = StatusBlock::read(Mode, 4), endian = StatusBlock::read(Endian, 4);
    const unsigned key_bytes = (mode & 3) == 0 ? 16 : (mode & 3) == 1 ? 24 : 32;
    std::uint32_t key_words[8], in_words[4], out_words[4];
    for (unsigned i = 0; i < 8; ++i)
        key_words[i] = StatusBlock::read(Key + 4 * i, 4);
    for (unsigned i = 0; i < 4; ++i)
        in_words[i] = StatusBlock::read(TextIn + 4 * i, 4);
    std::uint8_t key[32], in[16], out[16];
    words_to_bytes(key_words, key_bytes / 4, endian >> 1 & 1, endian & 1, key);
    words_to_bytes(in_words, 4, endian >> 3 & 1, endian >> 2 & 1, in);
    aes_block(key, key_bytes, in, out, mode & 4);
    for (unsigned i = 0; i < 4; ++i)
        std::memcpy(&out_words[i], out + 4 * i, 4);
    // The output swaps are the same transform in the other direction.
    std::uint32_t result[4];
    for (unsigned i = 0; i < 4; ++i) {
        std::uint32_t w = out_words[i];
        if (endian >> 4 & 1)
            w = ((w >> 24) | ((w >> 8) & 0xff00u) | ((w << 8) & 0xff0000u) | (w << 24));
        result[endian >> 5 & 1 ? 3 - i : i] = w;
    }
    for (unsigned i = 0; i < 4; ++i)
        StatusBlock::write(TextOut + 4 * i, result[i], 4);
}

} // namespace esp32emu
