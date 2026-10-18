#include "aes.hpp"

#include <cstring>

namespace esp32emu {

using mcuemu::aes_block;

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
