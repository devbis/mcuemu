#pragma once

#include <array>
#include <cstdint>

namespace crypto {

// AES-128 encryption of one block (FIPS-197), for peripheral models that need
// a cipher (nRF ECB/CCM/AAR). Not constant-time and not for real secrets.
using Block = std::array<std::uint8_t, 16>;

namespace detail {

inline const std::array<std::uint8_t, 256> &sbox() {
    static const std::array<std::uint8_t, 256> table = [] {
        std::array<std::uint8_t, 256> t{};
        // Generate the S-box from the multiplicative inverse in GF(2^8) and the
        // affine transform instead of listing it.
        std::uint8_t p = 1, q = 1;
        do {
            p = static_cast<std::uint8_t>(p ^ (p << 1) ^ ((p & 0x80) ? 0x1b : 0));
            q = static_cast<std::uint8_t>(q ^ (q << 1));
            q = static_cast<std::uint8_t>(q ^ (q << 2));
            q = static_cast<std::uint8_t>(q ^ (q << 4));
            if (q & 0x80)
                q ^= 0x09;
            const auto rotl = [](std::uint8_t x, int s) {
                return static_cast<std::uint8_t>((x << s) | (x >> (8 - s)));
            };
            t[p] = static_cast<std::uint8_t>(q ^ rotl(q, 1) ^ rotl(q, 2) ^ rotl(q, 3) ^ rotl(q, 4) ^ 0x63);
        } while (p != 1);
        t[0] = 0x63;
        return t;
    }();
    return table;
}

inline std::uint8_t xtime(std::uint8_t x) {
    return static_cast<std::uint8_t>((x << 1) ^ ((x & 0x80) ? 0x1b : 0));
}

} // namespace detail

inline Block aes128_encrypt(const Block &key, const Block &plain) {
    const auto &sbox = detail::sbox();
    std::array<std::uint8_t, 176> round_keys{};
    for (int i = 0; i != 16; ++i)
        round_keys[i] = key[i];
    std::uint8_t rcon = 1;
    for (int i = 16; i != 176; i += 4) {
        std::uint8_t t[4] = {round_keys[i - 4], round_keys[i - 3], round_keys[i - 2], round_keys[i - 1]};
        if (i % 16 == 0) {
            const std::uint8_t first = t[0];
            t[0] = static_cast<std::uint8_t>(sbox[t[1]] ^ rcon);
            t[1] = sbox[t[2]];
            t[2] = sbox[t[3]];
            t[3] = sbox[first];
            rcon = detail::xtime(rcon);
        }
        for (int j = 0; j != 4; ++j)
            round_keys[i + j] = static_cast<std::uint8_t>(round_keys[i - 16 + j] ^ t[j]);
    }
    Block s = plain;
    auto add_key = [&](int round) {
        for (int i = 0; i != 16; ++i)
            s[i] = static_cast<std::uint8_t>(s[i] ^ round_keys[round * 16 + i]);
    };
    add_key(0);
    for (int round = 1; round <= 10; ++round) {
        for (auto &b : s)
            b = sbox[b];
        // ShiftRows (column-major state)
        Block r = s;
        for (int c = 0; c != 4; ++c)
            for (int row = 0; row != 4; ++row)
                s[c * 4 + row] = r[((c + row) % 4) * 4 + row];
        if (round != 10) {
            for (int c = 0; c != 4; ++c) {
                const std::uint8_t a0 = s[c * 4], a1 = s[c * 4 + 1], a2 = s[c * 4 + 2], a3 = s[c * 4 + 3];
                const std::uint8_t all = static_cast<std::uint8_t>(a0 ^ a1 ^ a2 ^ a3);
                s[c * 4] = static_cast<std::uint8_t>(a0 ^ all ^ detail::xtime(static_cast<std::uint8_t>(a0 ^ a1)));
                s[c * 4 + 1] = static_cast<std::uint8_t>(a1 ^ all ^ detail::xtime(static_cast<std::uint8_t>(a1 ^ a2)));
                s[c * 4 + 2] = static_cast<std::uint8_t>(a2 ^ all ^ detail::xtime(static_cast<std::uint8_t>(a2 ^ a3)));
                s[c * 4 + 3] = static_cast<std::uint8_t>(a3 ^ all ^ detail::xtime(static_cast<std::uint8_t>(a3 ^ a0)));
            }
        }
        add_key(round);
    }
    return s;
}

} // namespace crypto
