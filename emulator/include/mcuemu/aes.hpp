#pragma once

#include <cstdint>

namespace mcuemu {

// AES on one 16-byte block with a 16, 24 or 32-byte key (FIPS-197, bytes in order).
void aes_block(const std::uint8_t *key, unsigned key_bytes, const std::uint8_t in[16], std::uint8_t out[16],
               bool decrypt);

} // namespace mcuemu
