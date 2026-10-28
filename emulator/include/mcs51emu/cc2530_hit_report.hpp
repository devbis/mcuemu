#pragma once

#include "mcs51emu/cc2530f256.hpp"

#include <cstdint>
#include <ostream>
#include <utility>
#include <vector>

namespace mcs51emu {

// What a CC2530F256 checkpoint report dumps besides the register line: all
// of IDATA and the SFR page when `state` is set, then each XDATA range
// (start, length) in order, 16 bytes per row.
struct Cc2530HitDump {
    bool state{true};
    std::vector<std::pair<std::uint16_t, std::uint32_t>> xdata;
};

// The block `mcuemu run --model CC2530F256 --report-hits` prints at each
// breakpoint arrival:
//
//   hit: N instructions=I cycles=C pc=0xPPPP sp=0xSS dps=0xDD fmap=0xFF irq-levels=0xLL
//   idata 0x0000: ...   (rows of `dump`)
//   end-hit: N
//
// Shared by the CLI and the `mcuemu sim` debugger report so the two formats
// cannot drift apart.
void write_cc2530_memory(std::ostream &out, const Cc2530F256 &device, const Cc2530HitDump &dump);
void write_cc2530_hit(std::ostream &out, const Cc2530F256 &device, std::uint64_t hit,
                      std::uint64_t instructions, const Cc2530HitDump &dump);

} // namespace mcs51emu
