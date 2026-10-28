// CC2530 analog blocks: ADC (SWRU191F Chapter 12), random-number generator
// (Chapter 14), operational amplifier (Chapter 18) and analog comparator
// (Chapter 19). Pad voltages come from Io::pad_mv(); characteristic values
// cite the CC2530 datasheet SWRS081B.

#include "cc2530_analog.hpp"

#include "cc2530_io.hpp"

#include <algorithm>
#include <cmath>

namespace mcs51emu::cc2530 {
namespace {

constexpr std::uint8_t SfrAdccon1 = 0xb4, SfrAdccon2 = 0xb5, SfrAdccon3 = 0xb6, SfrAdcl = 0xba,
                       SfrAdch = 0xbb, SfrRndl = 0xbc, SfrRndh = 0xbd;
constexpr Reg Adccon1 = sfr_reg(SfrAdccon1), Adccon2 = sfr_reg(SfrAdccon2),
              Adccon3 = sfr_reg(SfrAdccon3), Adcl = sfr_reg(SfrAdcl), Adch = sfr_reg(SfrAdch),
              Rndl = sfr_reg(SfrRndl), Rndh = sfr_reg(SfrRndh);
constexpr Reg Tr0 = 0x624b;
constexpr std::uint16_t Atest = 0x61bd; // SWRU191F 23.15.3, owned by the radio
constexpr Reg Cmpctl = 0x62d0;
constexpr Reg Opampc = 0x62c0, Opamps = 0x62c1, Opampmc = 0x61a6;

constexpr std::uint8_t Eoc = 0x80, St = 0x40;
constexpr std::uint8_t ApcfgSfr = 0xf2;

// Internal reference voltage 1.15 V (SWRS081B, ADC characteristics).
constexpr std::int64_t InternalReferenceUv = 1150000;
// Voltages are handled in units of 1/2048 uV so that the temperature sensor
// (defined in 12-bit ADC codes against the internal reference) is exact.
constexpr std::int64_t Scale = 2048;

// Resolution of the two's-complement result per decimation rate
// (SWRU191F ADCCON2.SDIV: 7/9/10/12 bits ENOB setting; 12.2.5: with 512
// the 12 MSBs span -2048..2047).
constexpr unsigned ResultBits[4] = {7, 9, 10, 12};

std::int64_t floor_div(std::int64_t a, std::int64_t b) {
    std::int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0)))
        --q;
    return q;
}

std::uint64_t splitmix64(std::uint64_t &state) {
    std::uint64_t z = (state += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

} // namespace

// ============================================================== ADC + RNG

Adc::Adc(Bus &bus) : Peripheral(bus) {
    for (std::uint8_t sfr : {SfrAdccon1, SfrAdccon2, SfrAdccon3, SfrAdcl, SfrAdch, SfrRndl, SfrRndh})
        bus.claim_sfr(sfr, this, sfr == SfrAdch); // reading ADCH clears ADCCON1.EOC
    bus.claim_xreg(Tr0, this);
}

void Adc::reset(ResetCause) {
    // Reset values, SWRU191F 12.2.10 and 14.3. The host environment
    // (temperature, VDD, noise) is not part of the chip and persists.
    bus_.storage(Adccon1) = 0x33;
    bus_.storage(Adccon2) = 0x10;
    bus_.storage(Adccon3) = 0x00;
    bus_.storage(Adcl) = 0x00;
    bus_.storage(Adch) = 0x00;
    bus_.storage(Tr0) = 0x00;
    lfsr_ = 0xffff;
    rng_sync();
    current_.reset();
    done_hf_ = Never;
    sequence_active_ = false;
    sequence_requested_ = false;
    sequence_ = {};
    sequence_next_ = 0;
    sch_written_ = 0;
    single_pending_.reset();
    full_speed_stalled_ = false;
}

std::uint8_t Adc::read(Reg reg) {
    const std::uint8_t value = bus_.storage(reg);
    if (reg == Adch)
        bus_.storage(Adccon1) = static_cast<std::uint8_t>(bus_.storage(Adccon1) & ~Eoc);
    return value;
}

std::uint8_t Adc::peek(Reg reg) const { return bus_.storage(reg); }

void Adc::write(Reg reg, std::uint8_t value) {
    switch (reg) {
    case Adccon1: {
        // EOC is R/H0 and ST reads back the sequence state; STSEL, RCTRL
        // and the reserved bits 1:0 are R/W.
        std::uint8_t &adccon1 = bus_.storage(Adccon1);
        adccon1 = static_cast<std::uint8_t>((adccon1 & (Eoc | St)) | (value & 0x3fu));
        const unsigned rctrl = (value >> 2) & 3u;
        if (rctrl == 1) {
            // 14.2.1: clock the LFSR once (13x unrolling); RCTRL returns to
            // 00 when done (immediately at this level of timing).
            rng_clock();
            adccon1 = static_cast<std::uint8_t>(adccon1 & ~0x0cu);
        } else if (rctrl == 2) {
            bus_.note_unmodeled(reg, true); // RCTRL = 10 is reserved
        }
        full_speed_stalled_ = false;
        const unsigned stsel = (value >> 4) & 3u;
        if ((value & St) && stsel == 3)
            request_sequence();
        else if (stsel == 1)
            start_pending(bus_.hf_now());
        return;
    }
    case Adccon2:
        bus_.storage(reg) = value;
        sch_written_ = value & 0x0fu;
        full_speed_stalled_ = false;
        if (((bus_.storage(Adccon1) >> 4) & 3u) == 1)
            start_pending(bus_.hf_now());
        return;
    case Adccon3:
        // 12.2.3: the single conversion starts at once, or after the running
        // sequence. Model-defined: a write during a running single
        // conversion is queued behind it (the last write wins).
        bus_.storage(reg) = value;
        if (current_ || sequence_active_)
            single_pending_ = value;
        else
            begin({static_cast<std::uint8_t>(value & 0x0fu), static_cast<std::uint8_t>(value >> 6),
                   static_cast<std::uint8_t>((value >> 4) & 3u), true},
                  bus_.hf_now());
        return;
    case Adcl:
    case Adch:
        return; // read-only
    case Rndl:
        // 14.2.2: the 8 LSBs move to the 8 MSBs, the written byte becomes the
        // 8 LSBs. Model-defined: ignored while the generator is stopped
        // (RCTRL = 11, "turned off").
        if (((bus_.storage(Adccon1) >> 2) & 3u) != 3) {
            lfsr_ = static_cast<std::uint16_t>((lfsr_ << 8) | value);
            rng_sync();
        }
        return;
    case Rndh:
        // 14.2.3: CRC16 of the written byte, processed from the MSB.
        if (((bus_.storage(Adccon1) >> 2) & 3u) != 3) {
            for (int bit = 7; bit >= 0; --bit)
                rng_shift(((value >> bit) & 1u) != 0);
            rng_sync();
        }
        return;
    case Tr0:
        bus_.storage(reg) = static_cast<std::uint8_t>(value & 0x01u);
        return;
    default:
        bus_.storage(reg) = value;
        return;
    }
}

std::uint64_t Adc::deadline() const { return current_ ? bus_.hf_deadline(done_hf_) : Never; }

void Adc::service(std::uint64_t) {
    while (current_ && bus_.hf_now() >= done_hf_)
        complete();
}

void Adc::power_mode_changed(PowerMode, PowerMode to) {
    // 12.2.1: TR0 has no retention in PM2/PM3. A conversion in progress
    // is frozen with the HF clock and finishes after wake (model-defined).
    if (to == PowerMode::Pm2 || to == PowerMode::Pm3)
        bus_.storage(Tr0) = 0;
    bus_.reschedule();
}

void Adc::set_temperature_celsius(double celsius) { temperature_c_ = celsius; }

void Adc::set_vdd_mv(std::uint32_t millivolts) {
    vdd_mv_ = millivolts;
    bus_.soc().comparator->inputs_changed(); // digital pad voltages follow VDD
}

void Adc::set_noise_seed(std::uint64_t seed) {
    noise_ = true;
    noise_state_ = seed;
}

void Adc::timer1_ch0_event() {
    if (((bus_.storage(Adccon1) >> 4) & 3u) == 2)
        request_sequence();
    bus_.reschedule();
}

void Adc::p2_0_rising() {
    if (((bus_.storage(Adccon1) >> 4) & 3u) == 0)
        request_sequence();
    bus_.reschedule();
}

// 12.2.4: "A sequence starts when this bit is set high, ADCCON1.STSEL is 11,
// and no conversion is currently running". Model-defined: a trigger during a
// running sequence is ignored; a trigger during a single conversion starts
// the sequence when that conversion ends (ST reads 1 meanwhile). The same
// rule applies to the P2_0 and Timer 1 triggers.
void Adc::request_sequence() {
    if (sequence_active_)
        return;
    if (current_) {
        sequence_requested_ = true;
        set_st_bit();
        return;
    }
    start_sequence(bus_.hf_now());
}

void Adc::start_sequence(std::uint64_t at) {
    const std::uint8_t adccon2 = bus_.storage(Adccon2);
    sequence_active_ = true;
    sequence_requested_ = false;
    // Model-defined: SREF/SDIV/SCH are latched when the sequence starts
    // (12.2.4 asks not to change them while a conversion runs).
    sequence_ = {sch_written_, static_cast<std::uint8_t>(adccon2 >> 6),
                 static_cast<std::uint8_t>((adccon2 >> 4) & 3u), false};
    // 12.2.2: SCH < 8: AIN0..SCH; 8..11: differential 8..SCH; >= 12: SCH only.
    sequence_next_ = sch_written_ < 8 ? 0 : sch_written_ < 12 ? 8 : sch_written_;
    set_st_bit();
    sequence_step(at);
}

void Adc::sequence_step(std::uint64_t at) {
    const std::uint8_t apcfg = bus_.storage(sfr_reg(ApcfgSfr));
    const std::uint8_t last = sequence_.channel;
    while (sequence_next_ <= last) {
        const std::uint8_t channel = sequence_next_++;
        // 12.2.2: channels whose pads are not analog in APCFG are skipped
        // (both pads of a differential pair).
        bool enabled = true;
        if (channel < 8)
            enabled = (apcfg >> channel) & 1u;
        else if (channel < 12)
            enabled = ((apcfg >> (2 * (channel - 8))) & 3u) == 3u;
        if (!enabled)
            continue;
        begin({channel, sequence_.reference, sequence_.decimation, false}, at);
        return;
    }
    // 12.2.5: after the sequence SCH reads one more than the last channel,
    // or the written value if that was 12 or more.
    sequence_active_ = false;
    std::uint8_t &adccon2 = bus_.storage(Adccon2);
    adccon2 = static_cast<std::uint8_t>((adccon2 & 0xf0u) | (last >= 12 ? last : last + 1));
    set_st_bit();
    start_pending(at);
}

void Adc::begin(Conversion conversion, std::uint64_t at) {
    current_ = conversion;
    done_hf_ = at + conversion_cycles(conversion.decimation);
    if (!conversion.single) {
        // 12.2.5: ADCCON2.SCH reads the channel being converted.
        std::uint8_t &adccon2 = bus_.storage(Adccon2);
        adccon2 = static_cast<std::uint8_t>((adccon2 & 0xf0u) | conversion.channel);
    }
}

// 12.2.7: Tconv = (decimation rate + 16) x 0.25 us, i.e. 8 cycles of the
// 32 MHz reference per 4 MHz sample clock period (SWRS081B lists 20/36/68/
// 132 us). Model-defined: with the 16 MHz RCOSC as system clock (which
// 12.2.7 rules out for the ADC) the fixed division doubles the time.
std::uint64_t Adc::conversion_cycles(std::uint8_t decimation) const {
    const std::uint64_t cycles = ((64u << decimation) + 16u) * 8u;
    return (bus_.clkconsta() & 0x40u) ? 2 * cycles : cycles;
}

void Adc::complete() {
    const Conversion conversion = *current_;
    const std::uint64_t at = done_hf_;
    current_.reset();
    done_hf_ = Never;

    // Model-defined: the input is sampled when the conversion completes.
    const std::int64_t input = input_value(conversion.channel, conversion.single ? Adccon3 : Adccon2);
    const std::int64_t reference = reference_value(conversion.reference);
    const unsigned bits = ResultBits[conversion.decimation];
    const std::int64_t half = std::int64_t{1} << (bits - 1);
    std::int64_t code;
    if (reference > 0)
        code = floor_div(input * half, reference);
    else // Model-defined: a non-positive reference saturates the result.
        code = input > 0 ? half - 1 : input < 0 ? -half : 0;
    if (noise_)
        code += static_cast<std::int64_t>(splitmix64(noise_state_) % 3) - 1; // +-1 LSB
    code = std::clamp(code, -half, half - 1);
    // 12.2.5: two's complement, MSB-aligned in ADCH:ADCL; ADCL[1:0] read 0.
    const auto aligned = static_cast<std::uint16_t>(
        (static_cast<std::uint64_t>(code) & ((std::uint64_t{1} << bits) - 1)) << (16 - bits));
    bus_.storage(Adch) = static_cast<std::uint8_t>(aligned >> 8);
    bus_.storage(Adcl) = static_cast<std::uint8_t>(aligned & 0xfcu);
    // EOC stays high if the previous result was not read (ADCCON1.EOC).
    bus_.storage(Adccon1) = static_cast<std::uint8_t>(bus_.storage(Adccon1) | Eoc);

    if (conversion.single) {
        // 12.2.8 / 12.2.9: interrupt only for the ADCCON3 conversion, no DMA.
        bus_.raise_irq(Irq::Adc);
        start_pending(at);
    } else {
        // 12.2.9: ADC_CHALL for every sequence conversion, ADC_CHsd for
        // the eight single-ended channels. Model-defined: differential
        // channels raise ADC_CHALL only.
        trigger_dma(trigger::AdcChAll);
        if (conversion.channel < 8)
            trigger_dma(trigger::AdcCh11 + conversion.channel);
        sequence_step(at);
    }
}

void Adc::start_pending(std::uint64_t at) {
    if (current_ || sequence_active_)
        return;
    if (single_pending_) {
        const std::uint8_t value = *single_pending_;
        single_pending_.reset();
        begin({static_cast<std::uint8_t>(value & 0x0fu), static_cast<std::uint8_t>(value >> 6),
               static_cast<std::uint8_t>((value >> 4) & 3u), true},
              at);
        return;
    }
    if (sequence_requested_) {
        start_sequence(at);
        return;
    }
    // STSEL = 01, "Full speed. Do not wait for triggers": the next sequence
    // starts at the end of the previous one (12.2.4).
    // An empty sequence (all channels skipped) would restart forever, so the
    // flag holds the restart until a conversion actually runs.
    if (((bus_.storage(Adccon1) >> 4) & 3u) == 1 && !full_speed_stalled_) {
        full_speed_stalled_ = true;
        start_sequence(at);
        if (current_)
            full_speed_stalled_ = false;
    }
}

std::int64_t Adc::input_value(std::uint8_t channel, Reg selected_by) {
    const Io &io = *bus_.soc().io;
    auto pad = [&](unsigned pin) { return std::int64_t{io.pad_mv(0, pin)} * 1000 * Scale; };
    if (channel < 8)
        return pad(channel);
    if (channel < 12)
        return pad(2 * (channel - 8)) - pad(2 * (channel - 8) + 1);
    switch (channel) {
    case 12:
        return 0; // GND
    case 13:
        bus_.note_unmodeled(selected_by, true); // reserved channel
        return 0;
    case 14: {
        // 12.2.1: TR0.ADCTM = 1 and ATEST.ATESTCTRL = 000001 connect the
        // sensor; model-defined: otherwise the input reads 0 V.
        if (!(bus_.storage(Tr0) & 0x01u) || (bus_.xdata_peek(Atest) & 0x3fu) != 1)
            return 0;
        // SWRS081B temperature sensor: 1480 (12-bit ADC, internal
        // reference) at 25 C and 3 V, 4.5 /C, 1 /0.1 V.
        const double counts = 1480.0 + 4.5 * (temperature_c_ - 25.0) +
                              (static_cast<double>(vdd_mv_) - 3000.0) / 100.0;
        // counts / 2048 * 1.15 V, in 1/2048 uV.
        return std::llround(counts * static_cast<double>(InternalReferenceUv));
    }
    default:
        return std::int64_t{vdd_mv_} * 1000 * Scale / 3; // AVDD5 / 3
    }
}

std::int64_t Adc::reference_value(std::uint8_t reference) const {
    const Io &io = *bus_.soc().io;
    auto pad = [&](unsigned pin) { return std::int64_t{io.pad_mv(0, pin)} * 1000 * Scale; };
    switch (reference) {
    case 0: return InternalReferenceUv * Scale;
    case 1: return pad(7);                                // AIN7
    case 2: return std::int64_t{vdd_mv_} * 1000 * Scale; // AVDD5
    default: return pad(6) - pad(7);                      // AIN6-AIN7
    }
}

void Adc::set_st_bit() {
    // Model-defined: ST reads 1 while a sequence is requested or running
    // ("Read as 1 until conversion has completed", whatever STSEL started it).
    std::uint8_t &adccon1 = bus_.storage(Adccon1);
    adccon1 = static_cast<std::uint8_t>((adccon1 & ~St) |
                                        (sequence_active_ || sequence_requested_ ? St : 0u));
}

// Figure 14-1: in_bit XOR bit 15 feeds bit 0 and the taps after bits 1 and
// 14 (X^16 + X^15 + X^2 + 1).
void Adc::rng_shift(bool in) {
    const bool feedback = ((lfsr_ >> 15) & 1u) != static_cast<unsigned>(in);
    lfsr_ = static_cast<std::uint16_t>(lfsr_ << 1);
    if (feedback)
        lfsr_ ^= 0x8005u;
}

// 14.2.1: one clocking = 13 shifts with feedback and no input.
void Adc::rng_clock() {
    for (int i = 0; i < 13; ++i)
        rng_shift(false);
    rng_sync();
}

void Adc::rng_sync() {
    bus_.storage(Rndl) = static_cast<std::uint8_t>(lfsr_);
    bus_.storage(Rndh) = static_cast<std::uint8_t>(lfsr_ >> 8);
}

void Adc::trigger_dma(unsigned trigger) {
    ++dma_triggers_[trigger];
    bus_.dma_trigger(trigger);
}

// ============================================================ comparator

AnalogComparator::AnalogComparator(Bus &bus) : Peripheral(bus) { bus.claim_xreg(Cmpctl, this); }

void AnalogComparator::reset(ResetCause) { bus_.storage(Cmpctl) = 0; }

std::uint8_t AnalogComparator::peek(Reg reg) const { return bus_.storage(reg); }

void AnalogComparator::write(Reg reg, std::uint8_t value) {
    // CMPCTL: EN R/W, OUTPUT read-only, bits 7:2 R0.
    std::uint8_t &cmpctl = bus_.storage(reg);
    const std::uint8_t before = cmpctl;
    cmpctl = static_cast<std::uint8_t>(value & 0x02u);
    const bool out = compute();
    cmpctl = static_cast<std::uint8_t>(cmpctl | (out ? 0x01u : 0u));
    if (cmpctl != before)
        bus_.soc().io->comparator_changed(out); // 19.1: feeds the P0_5 edge detector
}

void AnalogComparator::inputs_changed() {
    std::uint8_t &cmpctl = bus_.storage(Cmpctl);
    const bool out = compute();
    if (out == ((cmpctl & 0x01u) != 0))
        return;
    cmpctl = static_cast<std::uint8_t>((cmpctl & 0x02u) | (out ? 0x01u : 0u));
    bus_.soc().io->comparator_changed(out);
}

bool AnalogComparator::output() const { return (bus_.storage(Cmpctl) & 0x01u) != 0; }

// 19.1: positive input P0_5, negative input P0_4; the comparator stays active
// in PM2/PM3. Model-defined: ideal comparator (the SWRS081B 1 mV input offset
// is not modeled), OUTPUT reads 0 while disabled.
bool AnalogComparator::compute() const {
    if (!(bus_.storage(Cmpctl) & 0x02u))
        return false;
    const Io &io = *bus_.soc().io;
    return io.pad_mv(0, 5) > io.pad_mv(0, 4);
}

// ================================================================ op-amp

namespace {
// Model-defined calibration time (SWRU191F 18.2 does not give one): 4096
// cycles of the HF clock domain (128 us).
constexpr std::uint64_t CalibrationCycles = 4096;
// SWRS081B op-amp characteristics: output 0.07 V .. VDD - 0.07 V.
constexpr std::uint32_t OutputHeadroomMv = 70;
} // namespace

OpAmp::OpAmp(Bus &bus) : Peripheral(bus) {
    bus.claim_xreg(Opampc, this);
    bus.claim_xreg(Opamps, this);
    bus.claim_xreg(Opampmc, this);
}

void OpAmp::reset(ResetCause) {
    bus_.storage(Opampc) = 0;
    bus_.storage(Opamps) = 0;
    bus_.storage(Opampmc) = 0;
    calibration_done_hf_ = Never;
}

std::uint8_t OpAmp::peek(Reg reg) const { return bus_.storage(reg); }

void OpAmp::write(Reg reg, std::uint8_t value) {
    switch (reg) {
    case Opampc: {
        // EN R/W; CAL W1/R0, starts only if EN = 1 and no calibration runs
        // (18.2: "A new calibration is not accepted before CAL_BUSY goes low").
        bus_.storage(Opampc) = static_cast<std::uint8_t>(value & 0x01u);
        if (!(value & 0x01u)) {
            // Model-defined: disabling aborts a calibration in progress.
            calibration_done_hf_ = Never;
            bus_.storage(Opamps) = 0;
        } else if ((value & 0x02u) && !(bus_.storage(Opamps) & 0x01u)) {
            bus_.storage(Opamps) = 0x01; // CAL_BUSY
            calibration_done_hf_ = bus_.hf_now() + CalibrationCycles;
        }
        return;
    }
    case Opamps:
        return; // read-only
    default:
        // OPAMPMC.MODE selects chopping / offset behaviour, which the model
        // does not emulate (the output is offset-free).
        bus_.storage(reg) = value;
        bus_.note_unmodeled(reg, true);
        return;
    }
}

std::uint64_t OpAmp::deadline() const {
    return calibration_done_hf_ == Never ? Never : bus_.hf_deadline(calibration_done_hf_);
}

void OpAmp::service(std::uint64_t) {
    if (calibration_done_hf_ != Never && bus_.hf_now() >= calibration_done_hf_) {
        calibration_done_hf_ = Never;
        bus_.storage(Opamps) = 0;
    }
}

void OpAmp::power_mode_changed(PowerMode, PowerMode to) {
    // 18.1: shut down automatically when PM2 or PM3 is entered.
    if (to == PowerMode::Pm2 || to == PowerMode::Pm3) {
        bus_.storage(Opampc) = 0;
        bus_.storage(Opamps) = 0;
        calibration_done_hf_ = Never;
    }
    bus_.reschedule();
}

// 18.1: + input P0_0, - input P0_1, output P0_2. The feedback network is
// external to the chip; model-defined: the amplifier acts as a voltage
// follower of P0_0, limited to the SWRS081B output swing, and drives P0_2
// while enabled and not calibrating.
std::optional<std::uint32_t> OpAmp::output_mv() const {
    if (!(bus_.storage(Opampc) & 0x01u) || (bus_.storage(Opamps) & 0x01u))
        return std::nullopt;
    const Soc &soc = bus_.soc();
    const std::uint32_t vdd = soc.adc->vdd_mv_;
    const std::uint32_t high = vdd > OutputHeadroomMv ? vdd - OutputHeadroomMv : 0;
    return std::clamp(soc.io->pad_mv(0, 0), std::min(OutputHeadroomMv, high), high);
}

} // namespace mcs51emu::cc2530
