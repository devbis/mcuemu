#pragma once

#include "blocks.hpp"

namespace esp32emu {

// Timer group (TIMG0/TIMG1), so far only the RTC slow-clock calibration
// ("rtc_clk_cal" in ESP-IDF and Zephyr's boot code): setting RTCCALICFG.START
// counts XTAL cycles over CALI_MAX cycles of the selected slow clock and
// reports them in RTCCALICFG1.VALUE with RDY set. The measurement is instant and
// deterministic, with nominal frequencies: RC_SLOW 136 kHz, RC_FAST/256
// ~68.4 kHz, 32 kHz crystal 32.768 kHz, XTAL 40 MHz (inferred from ESP-IDF's
// nominal values, not measured on a board). The other registers read back.
// Watchdogs and the general-purpose timers are not modelled yet.
class Timg final : public StatusBlock {
public:
    // `xtal_hz` is the crystal the calibration counts (40 MHz on the C6, 32 MHz on the H2).
    Timg(std::uint32_t base, std::uint32_t xtal_hz) : StatusBlock(base, 0x400), xtal_hz_(xtal_hz) {}

    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;

private:
    std::uint32_t xtal_hz_;
};

} // namespace esp32emu
