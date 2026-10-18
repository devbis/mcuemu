#pragma once

#include "blocks.hpp"
#include "mcuemu/aes.hpp"

#include <array>
#include <cstdint>

namespace esp32emu {

// The AES accelerator (0x60088000 on the C6 and the H2), CPU-driven ECB mode only, which is
// what esp-hal's `Aes` does for single blocks (and what the zigbee-rs known-answer
// test and CCM* build on): the driver writes the key (KEY_0..7), the mode (MODE:
// bits 1:0 select 128/192/256-bit key, bit 2 decrypt), the block (TEXT_IN_0..3),
// sets TRIGGER and polls STATE until it is 0, then reads TEXT_OUT_0..3. The
// transform is instant. ENDIAN: with 0 the registers hold the byte stream as
// little-endian words; bits 0/1 (key byte/word order), 2/3 (input) and 4/5
// (output) swap bytes within words / reverse the word order. The DMA and block
// modes (GCM, CTR in hardware) are not modelled.
class Aes final : public StatusBlock {
public:
    explicit Aes(std::uint32_t base) : StatusBlock(base, 0x100) {}
    std::uint32_t read(std::uint32_t offset, unsigned size) override;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;

private:
    void run();
};

// The block transform lives in mcuemu-core (mcuemu/aes.hpp).
using mcuemu::aes_block;

} // namespace esp32emu
