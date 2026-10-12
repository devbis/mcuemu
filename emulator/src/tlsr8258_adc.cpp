// TLSR8258 SAR ADC, misc channel into DFIFO2.
//
// Evidence: datasheet section 11 and the SDK driver (platform/chip_8258/adc.c,
// adc.h, dfifo.h). The ADC is configured through analog registers: power
// afe_0xfc<5> (0 = on), channel enable afe_0xf2<2>, inputs afe_0xe8
// (<7:4> positive, <3:0> negative; 1..8 = PB0..PB7, 9/10 = PC4/PC5), reference
// afe_0xe7<5:4>, resolution afe_0xec<1:0>, input prescaler afe_0xfa<7:6>
// (divide by 1<<n). Sample data goes to a ring buffer in SRAM selected by
// 0xb08/0xb0a/0xb0b once DFIFO2 input is enabled in 0xb10<2>.
//
// The sample period follows from the state lengths, (R_max_mc + R_max_s)/24 MHz
// (which reproduces the 23/96/192 kHz rates in the SDK). The conversion
// inverts the SDK's mv = code * prescale * vref >> 13 for the differential
// P-N voltage with the 1.2 V reference at a calibrated 1175 mV.
//
// Inferred, not documented: samples are stored as 16-bit entries (the SDK
// reads the buffer as 32-bit words and uses only the low 13 bits plus the sign
// at bit 13, which works with either layout); the first sample lands one
// period after the channel is enabled; the left/right channels, PGA and the
// temperature sensor are not modeled (they read as 0 mV).

#include "tc32emu/tlsr8258.hpp"

#include <algorithm>
#include <cmath>

namespace tc32emu {

namespace {

constexpr std::uint8_t AnalogMiscInput = 0xe8;
constexpr std::uint8_t AnalogVref = 0xe7;
constexpr std::uint8_t AnalogResolution = 0xec;
constexpr std::uint8_t AnalogMaxMc = 0xef;
constexpr std::uint8_t AnalogMaxS = 0xf1;
constexpr std::uint8_t AnalogChannelEnable = 0xf2;
constexpr std::uint8_t AnalogInputScale = 0xfa;
constexpr std::uint8_t AnalogPgaControl = 0xfc;

constexpr std::uint32_t Dfifo2Address = 0xb08;
constexpr std::uint32_t Dfifo2Size = 0xb0a;
constexpr std::uint32_t Dfifo2AddressHigh = 0xb0b;
constexpr std::uint32_t DfifoMode = 0xb10;
constexpr std::uint8_t Dfifo2InEnable = 1u << 2;

} // namespace

std::uint32_t Tlsr8258::adc_channel_mv(unsigned selector) const {
    if (!adc_input_provider_)
        return 0;
    if (selector >= 1 && selector <= 8)
        return adc_input_provider_(1, selector - 1); // PB0..PB7
    if (selector == 9 || selector == 10)
        return adc_input_provider_(2, selector - 5); // PC4, PC5
    return 0; // no input, ground, PGA and temperature sensor
}

std::uint16_t Tlsr8258::adc_sample() const {
    const std::uint8_t input = analog_registers_[AnalogMiscInput];
    const double diff_mv = static_cast<double>(adc_channel_mv(input >> 4)) -
                           static_cast<double>(adc_channel_mv(input & 0x0f));
    const unsigned prescale_shift = analog_registers_[AnalogInputScale] >> 6;
    const unsigned reference = (analog_registers_[AnalogVref] >> 4) & 3u;
    const double vref_mv = reference == 0   ? 600.0
                           : reference == 1 ? 900.0
                                            : static_cast<double>(adc_reference_mv_);
    // 14-bit signed code: 13 magnitude bits plus the sign at bit 13.
    double code = std::floor(diff_mv * 8192.0 /
                                 (static_cast<double>(1u << prescale_shift) * vref_mv) +
                             0.5);
    code = std::clamp(code, -8192.0, 8191.0);
    const unsigned bits = 8u + 2u * (analog_registers_[AnalogResolution] & 3u);
    const int shifted = static_cast<int>(code) >> (14u - bits);
    return static_cast<std::uint16_t>(shifted & ((1 << bits) - 1));
}

void Tlsr8258::adc_advance() {
    const bool powered = (analog_registers_[AnalogPgaControl] & (1u << 5)) == 0;
    const bool misc = (analog_registers_[AnalogChannelEnable] & (1u << 2)) != 0;
    const bool dfifo = (registers_[DfifoMode] & Dfifo2InEnable) != 0;
    if (!powered || !misc || !dfifo) {
        adc_next_sample_ns_ = 0;
        return;
    }
    const std::uint32_t capture =
        analog_registers_[AnalogMaxMc] |
        (static_cast<std::uint32_t>(analog_registers_[AnalogMaxS] >> 6) << 8);
    const std::uint32_t set_length = analog_registers_[AnalogMaxS] & 0x0fu;
    const std::uint64_t cycles = capture + set_length;
    if (cycles == 0)
        return;
    const std::uint64_t period_ns = std::max<std::uint64_t>(cycles * 125u / 3u, 1u);
    if (adc_next_sample_ns_ == 0)
        adc_next_sample_ns_ = time_ns_ + period_ns;

    const std::uint32_t base = dma_address(Dfifo2Address, Dfifo2AddressHigh);
    const std::uint32_t size =
        (static_cast<std::uint32_t>(registers_[Dfifo2Size]) + 1u) * 16u;
    // After a long jump (sleep) only the newest buffer's worth of samples is
    // observable, so do not replay the whole gap.
    const std::uint64_t pending = (time_ns_ - adc_next_sample_ns_) / period_ns;
    if (time_ns_ >= adc_next_sample_ns_ && pending > size / 2u)
        adc_next_sample_ns_ = time_ns_ - (size / 2u) * period_ns;
    while (adc_next_sample_ns_ <= time_ns_) {
        const std::uint16_t sample = adc_sample();
        const std::uint32_t at = base + dfifo2_write_pointer_;
        try {
            write8(at, static_cast<std::uint8_t>(sample & 0xff));
            write8(at + 1, static_cast<std::uint8_t>(sample >> 8));
        } catch (const BusFault &) {
            // A buffer address outside mapped memory: the hardware would write
            // into the void, so the sample is dropped.
        }
        dfifo2_write_pointer_ = static_cast<std::uint16_t>(
            (dfifo2_write_pointer_ + 2u) % size);
        adc_next_sample_ns_ += period_ns;
    }
}

} // namespace tc32emu
