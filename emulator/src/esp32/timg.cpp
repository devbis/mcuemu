#include "timg.hpp"

#include "esp32emu/esp32c6_regs.hpp"

namespace esp32emu {

namespace {
namespace t = esp32c6_regs::timg0;
constexpr double SlowClockHz[4] = {136000.0, 17.5e6 / 256, 32768.0, 136000.0};
} // namespace

void Timg::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    if ((offset & ~3u) == t::reg::RTCCALICFG) {
        const bool start = value >> t::field::RTCCALICFG_RTC_CALI_START_shift & 1u;
        // RDY is a status bit: the hardware sets it, the driver does not write it.
        value &= ~(1u << t::field::RTCCALICFG_RTC_CALI_RDY_shift);
        if (start) {
            const unsigned select = value >> t::field::RTCCALICFG_RTC_CALI_CLK_SEL_shift & 3u;
            const std::uint32_t max = value >> t::field::RTCCALICFG_RTC_CALI_MAX_shift & 0x7fff;
            const std::uint32_t xtal_cycles =
                static_cast<std::uint32_t>(max * double(xtal_hz_) / SlowClockHz[select] + 0.5);
            StatusBlock::write(t::reg::RTCCALICFG1, xtal_cycles
                                                        << t::field::RTCCALICFG1_RTC_CALI_VALUE_shift,
                               4);
            value |= 1u << t::field::RTCCALICFG_RTC_CALI_RDY_shift;
        }
    }
    StatusBlock::write(offset, value, size);
}

} // namespace esp32emu
