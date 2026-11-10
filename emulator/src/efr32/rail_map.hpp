#pragma once

#include <cstdint>

// Addresses of the radio registers RAIL drives directly, in the secure view (the
// 0x5/0xB non-secure aliases fold onto these). Offsets inside a block follow the
// image's layout where it differs from the SDK header (see docs/efr32mg21.md).
namespace efr32emu::rail {

// BUFC_S: buffer n has its registers at +0x30*n from Buf0.
constexpr std::uint32_t BufcBase = 0x44004000u;
constexpr std::uint32_t Buf0Addr = BufcBase + 0x00C, Buf0WriteOffset = BufcBase + 0x010,
                        Buf0WriteData = BufcBase + 0x020, Buf0Cmd = BufcBase + 0x030;
constexpr std::uint32_t Buf0ReadOffset = BufcBase + 0x014, Buf0Status = BufcBase + 0x028;
constexpr std::uint32_t Buf2Addr = BufcBase + 0x06C;
constexpr std::uint32_t Buf1Addr = BufcBase + 0x03C, Buf1WriteOffset = BufcBase + 0x040,
                        Buf1ReadOffset = BufcBase + 0x044;
constexpr std::uint32_t Buf2Status = BufcBase + 0x088;
// BUFC IF/IEN as the firmware writes them (the IF_CLR alias is +0x2000).
constexpr std::uint32_t BufcIf = 0x440040E0u, BufcIen = 0x440040ECu, BufcIfClear = 0x440060E0u;
constexpr std::uint32_t Buf2Threshold = 1u << 18;

// RAC.
constexpr std::uint32_t RacBase = 0xA8020000u;
constexpr std::uint32_t RacRxEnable = RacBase + 0x008, RacStatus = RacBase + 0x00C, RacCmd = RacBase + 0x010,
                        RacIf = RacBase + 0x01C, RacIen = RacBase + 0x020;
constexpr std::uint32_t RacRxenRequest = 1u << 1;

// FRC.
// AGC (non-secure 0xB800C000, secure view 0xA800C000): STATUS1 (+0x0C), RSSI (+0x10).
constexpr std::uint32_t AgcStatus1 = 0xA800C00Cu, AgcRssi = 0xA800C010u;
constexpr std::uint32_t FrcIf = 0xA8004068u, FrcIen = 0xA800406Cu;

// SYNTH channel number.
constexpr std::uint32_t SynthChannel = 0xA8018044u;
constexpr std::uint32_t SynthCarrier = 0xA8018034u;

// RADIOAES (0x44000000): the engine's descriptor chain heads, CMD (+0x38) starts it.
constexpr std::uint32_t RadioAesBase = 0x44000000u, RadioAesFetchAddr = RadioAesBase + 0x04, RadioAesPushAddr = RadioAesBase + 0x14,
                        RadioAesCmd = RadioAesBase + 0x38;

// PROTIMER (image layout).
constexpr std::uint32_t ProtimerBase = 0xA801C000u;
constexpr std::uint32_t ProtimerCmd = ProtimerBase + 0x00C, ProtimerStatus = ProtimerBase + 0x014,
                        ProtimerWrapCount = ProtimerBase + 0x020, ProtimerPrecntTop = ProtimerBase + 0x030,
                        ProtimerIf = ProtimerBase + 0x064, ProtimerIen = ProtimerBase + 0x070;

} // namespace efr32emu::rail
