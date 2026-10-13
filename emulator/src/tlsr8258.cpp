#include "tc32emu/tlsr8258.hpp"
#include "tc32emu/radio.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>

namespace tc32emu {
namespace {

std::string address_error(std::uint32_t address, const char *operation) {
    std::ostringstream out;
    out << "TLSR8258 " << operation << " from unmapped address 0x" << std::hex
        << address;
    return out.str();
}

constexpr std::array<std::uint32_t, 5> gpio_port_offsets = {
    0x580, 0x588, 0x590, 0x598, 0x5a0,
};
constexpr std::uint32_t uart_data_first = 0x90;
constexpr std::uint32_t uart_data_last = 0x93;
constexpr std::size_t uart_fifo_capacity = 4;
constexpr std::uint8_t uart_tx_done_bit = 1u << 0;
constexpr std::uint8_t uart_tx_buffer_irq_bit = 1u << 1;
constexpr std::uint8_t uart_rx_buffer_irq_bit = 1u << 3;
constexpr std::uint8_t uart_irq_flag_bit = 1u << 3;
constexpr std::uint8_t uart_irq_mask_bit = 1u << 6;
constexpr std::uint8_t aes_data_feed_bit = 1u << 1;
constexpr std::uint8_t aes_codec_finished_bit = 1u << 2;
constexpr std::uint64_t aes_block_cycles = 1000;
constexpr std::uint32_t rf_frequency_low_offset = 0x1244;
constexpr std::uint32_t rf_frequency_high_offset = 0x1245;

bool decode_zigbee_channel(std::uint8_t low, std::uint8_t high,
                           std::uint8_t &channel) {
    // TC32 vendor rf_set_channel() stores four frequency units per MHz in
    // these synthesizer registers. The SDK sequence has a one-unit rounding
    // offset, so accept only values close to a Zigbee channel center.
    const int frequency_quarters =
        (static_cast<int>(high & 0x3fu) << 8) | static_cast<int>(low);
    const int channel_delta = frequency_quarters - 2405 * 4;
    const int nearest_channel_delta = (channel_delta + 10) / 20;
    const int encoded_center = 2405 * 4 + nearest_channel_delta * 20;
    const int error = frequency_quarters - encoded_center;
    if (nearest_channel_delta < 0 || nearest_channel_delta > 15 ||
        error < -3 || error > 3)
        return false;
    channel = static_cast<std::uint8_t>(11 + nearest_channel_delta);
    return true;
}

std::uint8_t aes_xtime(std::uint8_t value) {
    return static_cast<std::uint8_t>((value << 1) ^
        ((value & 0x80u) != 0 ? 0x1bu : 0u));
}

std::uint8_t aes_gf_multiply(std::uint8_t lhs, std::uint8_t rhs) {
    std::uint8_t result = 0;
    while (rhs != 0) {
        if ((rhs & 1u) != 0)
            result ^= lhs;
        lhs = aes_xtime(lhs);
        rhs >>= 1;
    }
    return result;
}

std::uint8_t aes_gf_inverse(std::uint8_t value) {
    if (value == 0)
        return 0;
    std::uint8_t result = 1;
    std::uint8_t power = value;
    unsigned exponent = 254;
    while (exponent != 0) {
        if ((exponent & 1u) != 0)
            result = aes_gf_multiply(result, power);
        power = aes_gf_multiply(power, power);
        exponent >>= 1;
    }
    return result;
}

std::uint8_t rotate_byte_left(std::uint8_t value, unsigned amount) {
    return static_cast<std::uint8_t>((value << amount) |
                                     (value >> (8u - amount)));
}

std::uint8_t aes_substitute(std::uint8_t value) {
    const std::uint8_t inverse = aes_gf_inverse(value);
    return static_cast<std::uint8_t>(inverse ^ rotate_byte_left(inverse, 1) ^
        rotate_byte_left(inverse, 2) ^ rotate_byte_left(inverse, 3) ^
        rotate_byte_left(inverse, 4) ^ 0x63u);
}

std::uint8_t aes_inverse_substitute(std::uint8_t value) {
    const std::uint8_t inverse_affine = static_cast<std::uint8_t>(
        rotate_byte_left(value, 1) ^ rotate_byte_left(value, 3) ^
        rotate_byte_left(value, 6) ^ 0x05u);
    return aes_gf_inverse(inverse_affine);
}

std::array<std::uint8_t, 176> aes_expand_key(
    const std::array<std::uint8_t, 16> &key) {
    std::array<std::uint8_t, 176> expanded{};
    std::copy(key.begin(), key.end(), expanded.begin());
    std::size_t generated = key.size();
    std::uint8_t round_constant = 1;
    while (generated < expanded.size()) {
        std::array<std::uint8_t, 4> word{};
        std::copy_n(expanded.begin() + generated - 4, 4, word.begin());
        if ((generated % key.size()) == 0) {
            std::rotate(word.begin(), word.begin() + 1, word.end());
            for (auto &byte : word)
                byte = aes_substitute(byte);
            word[0] ^= round_constant;
            round_constant = aes_xtime(round_constant);
        }
        for (unsigned i = 0; i != word.size(); ++i) {
            expanded[generated] =
                expanded[generated - key.size()] ^ word[i];
            ++generated;
        }
    }
    return expanded;
}

void aes_add_round_key(std::array<std::uint8_t, 16> &state,
                       const std::array<std::uint8_t, 176> &round_keys,
                       unsigned round) {
    for (unsigned i = 0; i != state.size(); ++i)
        state[i] ^= round_keys[round * 16u + i];
}

void aes_sub_bytes(std::array<std::uint8_t, 16> &state, bool inverse) {
    for (auto &byte : state)
        byte = inverse ? aes_inverse_substitute(byte) : aes_substitute(byte);
}

void aes_shift_rows(std::array<std::uint8_t, 16> &state, bool inverse) {
    const auto original = state;
    for (unsigned row = 1; row != 4; ++row) {
        for (unsigned column = 0; column != 4; ++column) {
            const unsigned source_column = inverse
                ? (column + 4u - row) % 4u
                : (column + row) % 4u;
            state[row + 4u * column] = original[row + 4u * source_column];
        }
    }
}

void aes_mix_columns(std::array<std::uint8_t, 16> &state, bool inverse) {
    for (unsigned column = 0; column != 4; ++column) {
        auto *bytes = state.data() + 4u * column;
        const std::uint8_t a = bytes[0];
        const std::uint8_t b = bytes[1];
        const std::uint8_t c = bytes[2];
        const std::uint8_t d = bytes[3];
        if (!inverse) {
            bytes[0] = static_cast<std::uint8_t>(aes_xtime(a) ^
                aes_xtime(b) ^ b ^ c ^ d);
            bytes[1] = static_cast<std::uint8_t>(a ^ aes_xtime(b) ^
                aes_xtime(c) ^ c ^ d);
            bytes[2] = static_cast<std::uint8_t>(a ^ b ^ aes_xtime(c) ^
                aes_xtime(d) ^ d);
            bytes[3] = static_cast<std::uint8_t>(aes_xtime(a) ^ a ^ b ^ c ^
                aes_xtime(d));
        } else {
            bytes[0] = static_cast<std::uint8_t>(aes_gf_multiply(a, 14) ^
                aes_gf_multiply(b, 11) ^ aes_gf_multiply(c, 13) ^
                aes_gf_multiply(d, 9));
            bytes[1] = static_cast<std::uint8_t>(aes_gf_multiply(a, 9) ^
                aes_gf_multiply(b, 14) ^ aes_gf_multiply(c, 11) ^
                aes_gf_multiply(d, 13));
            bytes[2] = static_cast<std::uint8_t>(aes_gf_multiply(a, 13) ^
                aes_gf_multiply(b, 9) ^ aes_gf_multiply(c, 14) ^
                aes_gf_multiply(d, 11));
            bytes[3] = static_cast<std::uint8_t>(aes_gf_multiply(a, 11) ^
                aes_gf_multiply(b, 13) ^ aes_gf_multiply(c, 9) ^
                aes_gf_multiply(d, 14));
        }
    }
}

std::array<std::uint8_t, 16> aes_process_block(
    const std::array<std::uint8_t, 16> &key,
    const std::array<std::uint8_t, 16> &input, bool decrypt) {
    const auto round_keys = aes_expand_key(key);
    auto state = input;
    if (!decrypt) {
        aes_add_round_key(state, round_keys, 0);
        for (unsigned round = 1; round != 10; ++round) {
            aes_sub_bytes(state, false);
            aes_shift_rows(state, false);
            aes_mix_columns(state, false);
            aes_add_round_key(state, round_keys, round);
        }
        aes_sub_bytes(state, false);
        aes_shift_rows(state, false);
        aes_add_round_key(state, round_keys, 10);
    } else {
        aes_add_round_key(state, round_keys, 10);
        for (unsigned round = 9; round != 0; --round) {
            aes_shift_rows(state, true);
            aes_sub_bytes(state, true);
            aes_add_round_key(state, round_keys, round);
            aes_mix_columns(state, true);
        }
        aes_shift_rows(state, true);
        aes_sub_bytes(state, true);
        aes_add_round_key(state, round_keys, 0);
    }
    return state;
}

} // namespace

BusFault::BusFault(std::uint32_t address, const char *operation)
    : std::runtime_error(address_error(address, operation)), address_(address) {}

std::uint16_t Bus::read16(std::uint32_t address) {
    const auto lo = read8(address);
    const auto hi = read8(address + 1u);
    return static_cast<std::uint16_t>(lo | (static_cast<std::uint16_t>(hi) << 8));
}

std::uint32_t Bus::read32(std::uint32_t address) {
    return static_cast<std::uint32_t>(read8(address)) |
           (static_cast<std::uint32_t>(read8(address + 1u)) << 8) |
           (static_cast<std::uint32_t>(read8(address + 2u)) << 16) |
           (static_cast<std::uint32_t>(read8(address + 3u)) << 24);
}

void Bus::write16(std::uint32_t address, std::uint16_t value) {
    write8(address, static_cast<std::uint8_t>(value));
    write8(address + 1u, static_cast<std::uint8_t>(value >> 8));
}

void Bus::write32(std::uint32_t address, std::uint32_t value) {
    write8(address, static_cast<std::uint8_t>(value));
    write8(address + 1u, static_cast<std::uint8_t>(value >> 8));
    write8(address + 2u, static_cast<std::uint8_t>(value >> 16));
    write8(address + 3u, static_cast<std::uint8_t>(value >> 24));
}

Tlsr8258::Tlsr8258(Tlsr8258Model model, std::uint32_t clock_hz)
    : clock_hz_(clock_hz),
      flash_(model == Tlsr8258Model::F512 ? 512u * 1024u : 1024u * 1024u,
             0xff) {
    if (clock_hz_ == 0)
        throw std::invalid_argument("TLSR8258 clock must be nonzero");
    registers_.fill(0);
    registers_[0x540] = aes_data_feed_bit; // Datasheet AES_CTRL reset value.
    // Datasheet SYS_TIMER_CTRL0 reset value enables 32 kHz calibration and
    // the system-timer subsystem.
    registers_[0x74c] = 0x90;
    analog_registers_.fill(0);
    // Datasheet AREG 0x05 reset state: the RC 32 kHz clock is powered, while
    // the optional 32 kHz crystal is powered down.
    analog_registers_[AnalogClockPower] = AnalogClock32kXtalPowerDown;
    // The 8258 SDK treats AREG 0x7f[0] == 1 as the cold-boot path. The
    // datasheet does not list this project-local PM status register's reset
    // value; this default is inferred from cpu_wakeup_init(), which otherwise
    // classifies POR as deep-retention wake and calls a null recovery hook.
    analog_registers_[0x7f] = 1;
    registers_[0x073] = 0x04; // Datasheet reset value: RC_32k selected.
    sram_.fill(0);
    // Board buttons on the TLSR8258 dongle are pulled high and become active
    // low. Start input pads in the idle state; tests and network stimuli can
    // drive individual port values explicitly.
    gpio_inputs_.fill(0xff);
    gpio_outputs_.fill(0);
    // Output-enable resets to 0xff (all pads are inputs).
    for (std::uint32_t base : gpio_port_offsets) {
        registers_[base + 2] = 0xff;
        registers_[base + 6] = 0xff; // all pads start as GPIO (ACT_AS_GPIO)
    }
    registers_[0xb0b] = 0x04; // DFIFO2 base address high byte
    dfifo2_write_pointer_ = 0;
    adc_next_sample_ns_ = 0;
    timer_last_tick_.fill(0);
    timer_fired_.fill(false);
    // UART register reset values follow datasheet table 7-9. The SDK's polling
    // API assumes TX_DONE is set while idle, despite the table listing 0 for
    // status register 0x9e; the model follows the SDK here pending a hardware trace.
    registers_[0x94] = 0xff;
    registers_[0x95] = 0x0f;
    registers_[0x96] = 0x0f;
    registers_[0x97] = 0x0e;
    registers_[0x98] = 0xa5;
    registers_[0x99] = 0x44;
    registers_[0x9a] = 0xc0;
    registers_[0x9b] = 0x01;
    registers_[0x9e] = uart_tx_done_bit;
    time_ns_ = 0;
}

void Tlsr8258::set_rc32k_characteristics(
    std::uint32_t frequency_hz, std::uint16_t system_timer_calibration) {
    if (frequency_hz == 0 || system_timer_calibration == 0)
        throw std::invalid_argument("RC 32 kHz characteristics must be nonzero");
    rc32k_frequency_hz_ = frequency_hz;
    rc32k_system_timer_calibration_ = system_timer_calibration;
}

void Tlsr8258::set_analog_trace_enabled(bool enabled, std::size_t limit) {
    analog_trace_enabled_ = enabled;
    analog_trace_limit_ = limit;
    analog_register_event_count_ = 0;
    analog_register_events_.clear();
}

std::uint32_t Tlsr8258::flash_size() const noexcept {
    return static_cast<std::uint32_t>(flash_.size());
}

std::uint32_t Tlsr8258::system_32k_compare() const noexcept {
    const std::uint32_t offset = System32kTickCompare - RegisterBase;
    return static_cast<std::uint32_t>(registers_[offset]) |
           (static_cast<std::uint32_t>(registers_[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(registers_[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(registers_[offset + 3]) << 24);
}

bool Tlsr8258::selected_32k_clock_running() const noexcept {
    const bool use_32k_xtal =
        (registers_[Clock32kSelect - RegisterBase] & Clock32kSelectXtal) != 0;
    const std::uint8_t power_down_mask = use_32k_xtal
        ? AnalogClock32kXtalPowerDown
        : AnalogClock32kRcPowerDown;
    return (analog_registers_[AnalogClockPower] & power_down_mask) == 0;
}

bool Tlsr8258::advance_sleep_to_wakeup() {
    if (power_state_ == Tlsr8258PowerState::Awake || !wake_timer_armed_ ||
        (analog_registers_[AnalogWakeSource] & AnalogWakeTimerEnable) == 0 ||
        !selected_32k_clock_running())
        return false;

    const std::uint32_t current = analog_32k_tick();
    const std::uint32_t target = system_32k_compare();
    const std::uint32_t delta = target - current;
    if (delta > 0x7fffffffu)
        return false; // Compare is already behind the counter; no match occurs.

    const bool use_32k_xtal =
        (registers_[Clock32kSelect - RegisterBase] & Clock32kSelectXtal) != 0;
    const std::uint64_t source_hz =
        use_32k_xtal ? 32768u : rc32k_frequency_hz_;
    std::uint32_t calibration =
        static_cast<std::uint32_t>(registers_[0x750]) |
        (static_cast<std::uint32_t>(registers_[0x751]) << 8);
    if (calibration == 0)
        calibration = use_32k_xtal ? 7812u : rc32k_system_timer_calibration_;
    const std::uint32_t previous_system_timer = system_timer_count_;
    system_timer_count_ += static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(delta) * calibration) / 16u);
    update_system_timer_irq(previous_system_timer);
    const std::uint64_t whole_seconds = delta / source_hz;
    const std::uint64_t remaining_ticks = delta % source_hz;
    const std::uint64_t max_time = std::numeric_limits<std::uint64_t>::max();
    const std::uint64_t whole_ns = whole_seconds * 1000000000u;
    const std::uint64_t fractional_ns =
        (remaining_ticks * 1000000000u + source_hz - 1u) / source_hz;
    if (whole_ns > max_time - time_ns_ ||
        fractional_ns > max_time - time_ns_ - whole_ns) {
        time_ns_ = max_time;
    } else {
        time_ns_ += whole_ns + fractional_ns;
    }
    time_fraction_ = 0;
    update_analog_calibration();
    analog_32k_tick_count_ += delta;
    analog_32k_fraction_ = 0;
    update_system_timer_calibration();
    analog_registers_[AnalogWakeStatus] |= AnalogWakeTimerStatus;
    wake_timer_armed_ = false;
    if (power_state_ == Tlsr8258PowerState::DeepSleep)
        deep_sleep_wakeup_pending_ = true;
    power_state_ = Tlsr8258PowerState::Awake;
    return true;
}

bool Tlsr8258::consume_deep_sleep_wakeup() noexcept {
    if (!deep_sleep_wakeup_pending_)
        return false;

    reset_volatile_state_after_deep_sleep();
    deep_sleep_wakeup_pending_ = false;
    return true;
}

void Tlsr8258::reset_volatile_state_after_deep_sleep() {
    registers_.fill(0);
    registers_[0x540] = aes_data_feed_bit;
    registers_[Clock32kSelect - RegisterBase] = 0x04;
    registers_[0x74c] = 0x90;
    registers_[0x94] = 0xff;
    registers_[0x95] = 0x0f;
    registers_[0x96] = 0x0f;
    registers_[0x97] = 0x0e;
    registers_[0x98] = 0xa5;
    registers_[0x99] = 0x44;
    registers_[0x9a] = 0xc0;
    registers_[0x9b] = 0x01;
    registers_[0x9e] = uart_tx_done_bit;
    system_timer_count_ = 0;
    system_timer_fraction_ = 0;
    system_timer_calibration_target_tick_ = 0;
    system_timer_calibration_pending_ = false;
    timer_last_tick_.fill(0);
    timer_fired_.fill(false);
    gpio_outputs_.fill(0);
    uart_rx_fifo_.clear();
    uart_tx_fifo_.clear();
    uart_tx_cycles_remaining_ = 0;
    uart_tx_done_ = true;
    uart_rx_error_ = false;
    uart_dma_tx_queue_.clear();
    uart_dma_tx_active_ = false;
    uart_dma_rx_count_ = 0;
    uart_dma_rx_idle_cycles_ = 0;
    aes_cycles_remaining_ = 0;
    aes_input_word_ = 0;
    aes_input_word_count_ = 0;
    aes_output_word_index_ = 0;
    mspi_active_ = false;
    mspi_command_seen_ = false;
    radio_tx_busy_ = false;
    wake_timer_armed_ = false;
    sleep_entry_pending_ = false;
    sleep_entry_cycles_remaining_ = 0;
    pending_sleep_state_ = Tlsr8258PowerState::Awake;
    power_state_ = Tlsr8258PowerState::Awake;
    // The TLSR8258 has 32 KB of SRAM retained in deep sleep at 0x840000..
    // 0x847fff and 32 KB that loses state at 0x848000..0x84ffff. Firmware
    // keeps its wake continuation in the retained range; startup clears its
    // normal BSS ranges after reset.
    constexpr std::size_t non_retained_offset =
        NonRetainedSramBase - RetainedSramBase;
    std::fill(sram_.begin() + non_retained_offset, sram_.end(), 0);
    // The PM status analog register also survives and distinguishes a timer
    // wake from the initial power-on path in the vendor SDK.
    analog_registers_[0x7f] = 0;
}

void Tlsr8258::enter_sleep() {
    const std::uint8_t mode = analog_registers_[0x7e];
    pending_sleep_state_ = (mode & 0x80u) != 0
        ? Tlsr8258PowerState::DeepSleep
        : Tlsr8258PowerState::Suspend;
    sleep_entry_cycles_remaining_ = SleepEntryDelayCycles;
    sleep_entry_pending_ = true;
}

void Tlsr8258::load_firmware(const std::vector<std::uint8_t> &image,
                            std::uint32_t offset) {
    if (offset > flash_.size() || image.size() > flash_.size() - offset)
        throw std::out_of_range("firmware image does not fit in TLSR8258 flash");
    std::copy(image.begin(), image.end(), flash_.begin() + offset);
}

std::uint8_t Tlsr8258::read8(std::uint32_t address) {
    if (address < flash_.size())
        return flash_[address];
    if (address >= RegisterBase && address < RegisterBase + RegisterSize)
        return read_register(address - RegisterBase);
    if (address >= VendorRegisterAliasBase &&
        address < VendorRegisterAliasBase + RegisterSize)
        return read_register(address - VendorRegisterAliasBase);
    if (address >= RetainedSramBase && address < RetainedSramBase + SramSize)
        return sram_[address - RetainedSramBase];
    throw BusFault(address, "read");
}

void Tlsr8258::write8(std::uint32_t address, std::uint8_t value) {
    if (address < flash_.size())
        throw BusFault(address, "write to read-only flash");
    if (address >= RegisterBase && address < RegisterBase + RegisterSize) {
        write_register(address - RegisterBase, value);
        return;
    }
    if (address >= VendorRegisterAliasBase &&
        address < VendorRegisterAliasBase + RegisterSize) {
        write_register(address - VendorRegisterAliasBase, value);
        return;
    }
    if (address >= RetainedSramBase && address < RetainedSramBase + SramSize) {
        sram_[address - RetainedSramBase] = value;
        return;
    }
    throw BusFault(address, "write");
}

void Tlsr8258::tick(std::uint32_t cycles) {
    constexpr std::uint64_t nanos_per_second = 1000000000u;
    const std::uint32_t previous_system_timer = system_timer_count_;
    const std::uint64_t system_timer_numerator =
        static_cast<std::uint64_t>(cycles) * SystemTimerClockHz +
        system_timer_fraction_;
    system_timer_count_ += static_cast<std::uint32_t>(
        system_timer_numerator / clock_hz_);
    system_timer_fraction_ = system_timer_numerator % clock_hz_;
    update_system_timer_irq(previous_system_timer);

    const bool use_32k_xtal =
        (registers_[0x073] & Clock32kSelectXtal) != 0;
    const std::uint8_t clock_power_down_mask = use_32k_xtal
        ? AnalogClock32kXtalPowerDown
        : AnalogClock32kRcPowerDown;
    if ((analog_registers_[AnalogClockPower] & clock_power_down_mask) == 0) {
        const std::uint64_t analog_clock_hz =
            use_32k_xtal ? 32768u : rc32k_frequency_hz_;
        const std::uint64_t analog_tick_numerator =
            static_cast<std::uint64_t>(cycles) * analog_clock_hz +
            analog_32k_fraction_;
        analog_32k_tick_count_ += analog_tick_numerator / clock_hz_;
        analog_32k_fraction_ = analog_tick_numerator % clock_hz_;
    } else {
        analog_32k_fraction_ = 0;
    }
    update_system_timer_calibration();

    const std::uint64_t whole_seconds = cycles / clock_hz_;
    const std::uint64_t remainder_cycles = cycles % clock_hz_;
    const std::uint64_t max_time = std::numeric_limits<std::uint64_t>::max();
    if (whole_seconds > (max_time - time_ns_) / nanos_per_second) {
        time_ns_ = max_time;
    } else {
        time_ns_ += whole_seconds * nanos_per_second;
        const std::uint64_t fractional_numerator =
            remainder_cycles * nanos_per_second + time_fraction_;
        const std::uint64_t fractional_ns = fractional_numerator / clock_hz_;
        if (fractional_ns > max_time - time_ns_)
            time_ns_ = max_time;
        else
            time_ns_ += fractional_ns;
        time_fraction_ = fractional_numerator % clock_hz_;
    }
    update_analog_calibration();
    i2c_advance(cycles);
    adc_advance();

    // Writing PWDN_CTRL requests sleep; it does not stop execution at the
    // store. The silicon's start_suspend trampoline runs 40 16-bit delay
    // instructions before reaching the instruction-cache +0x58 boundary.
    if (sleep_entry_pending_) {
        if (cycles >= sleep_entry_cycles_remaining_) {
            sleep_entry_cycles_remaining_ = 0;
            sleep_entry_pending_ = false;
            power_state_ = pending_sleep_state_;
            pending_sleep_state_ = Tlsr8258PowerState::Awake;
        } else {
            sleep_entry_cycles_remaining_ -= cycles;
        }
    }

    if (aes_cycles_remaining_ != 0) {
        if (cycles >= aes_cycles_remaining_) {
            aes_cycles_remaining_ = 0;
            aes_output_ = aes_process_block(aes_key_, aes_input_, aes_decrypt_);
            registers_[0x540] |= aes_codec_finished_bit;
        } else {
            aes_cycles_remaining_ -= cycles;
        }
    }

    const std::uint32_t control =
        static_cast<std::uint32_t>(registers_[0x620]) |
        (static_cast<std::uint32_t>(registers_[0x621]) << 8) |
        (static_cast<std::uint32_t>(registers_[0x622]) << 16) |
        (static_cast<std::uint32_t>(registers_[0x623]) << 24);
    for (unsigned timer = 0; timer != 3; ++timer) {
        const unsigned enable_bit = timer * 3u;
        if ((control & (1u << enable_bit)) == 0)
            continue;
        const std::uint32_t count = timer_tick(timer) + cycles;
        timer_last_tick_[timer] = count;
        const std::uint32_t capture =
            static_cast<std::uint32_t>(registers_[0x624 + timer * 4]) |
            (static_cast<std::uint32_t>(registers_[0x625 + timer * 4]) << 8) |
            (static_cast<std::uint32_t>(registers_[0x626 + timer * 4]) << 16) |
            (static_cast<std::uint32_t>(registers_[0x627 + timer * 4]) << 24);
        if (!timer_fired_[timer] && capture != 0 && count >= capture) {
            timer_fired_[timer] = true;
            registers_[0x623] |= static_cast<std::uint8_t>(1u << timer);
            registers_[0x648] |= static_cast<std::uint8_t>(1u << timer);
        }
    }

    std::uint64_t remaining = cycles;
    refill_uart_tx_from_dma();
    while (!uart_tx_fifo_.empty()) {
        start_uart_transmit();
        if (remaining < uart_tx_cycles_remaining_) {
            uart_tx_cycles_remaining_ -= remaining;
            remaining = 0;
            break;
        }
        remaining -= uart_tx_cycles_remaining_;
        const std::uint8_t sent = uart_tx_fifo_.front();
        uart_tx_fifo_.pop_front();
        uart_transmitted_.push_back(sent);
        uart_tx_cycles_remaining_ = 0;
        refill_uart_tx_from_dma();
        if (uart_tx_fifo_.empty()) {
            uart_tx_done_ = true;
            if (uart_dma_tx_active_) { // the DMA transfer has drained
                uart_dma_tx_active_ = false;
                registers_[0xc24] &= static_cast<std::uint8_t>(~0x02u);
                registers_[0xc26] |= 0x02u;
                update_dma_irq();
            }
        }
        if ((registers_[0x97] & 0x80u) != 0)
            inject_uart_rx(sent);
        if (remaining == 0)
            break;
    }
    if (uart_tx_fifo_.empty())
        uart_tx_cycles_remaining_ = 0;
    if (uart_dma_rx_idle_cycles_ != 0) {
        if (cycles >= uart_dma_rx_idle_cycles_)
            finish_uart_dma_receive();
        else
            uart_dma_rx_idle_cycles_ -= cycles;
    }
    update_uart_status();
    poll_radio();
}

bool Tlsr8258::interrupt_pending() const noexcept {
    if ((registers_[IrqGlobalEnable - RegisterBase] & 1u) == 0)
        return false;
    for (unsigned byte = 0; byte != 3; ++byte) {
        if ((registers_[0x640 + byte] & registers_[0x648 + byte]) != 0)
            return true;
    }
    return false;
}

void Tlsr8258::connect_radio(VirtualAir &air, std::string node_name) {
    if (node_name.empty())
        throw std::invalid_argument("virtual radio node must have a name");
    radio_air_ = &air;
    radio_node_name_ = std::move(node_name);
    radio_next_poll_ns_ = time_ns_;
    radio_last_poll_ns_ = time_ns_;
}

void Tlsr8258::set_radio_channel(std::uint8_t zigbee_channel) {
    if (zigbee_channel < 11 || zigbee_channel > 26)
        throw std::invalid_argument("Zigbee channel must be 11..26");
    radio_channel_ = zigbee_channel;
}

void Tlsr8258::set_gpio_input(unsigned port, std::uint8_t value) {
    if (port >= gpio_inputs_.size())
        throw std::out_of_range("GPIO port must be PA..PE (0..4)");
    gpio_inputs_[port] = value;
}

std::uint8_t Tlsr8258::gpio_oen(unsigned port) const {
    if (port >= gpio_port_offsets.size())
        throw std::out_of_range("GPIO port must be PA..PE (0..4)");
    return registers_[gpio_port_offsets[port] + 2];
}

bool Tlsr8258::gpio_is_gpio(unsigned port, unsigned bit) const {
    if (port >= gpio_port_offsets.size() || bit > 7)
        throw std::out_of_range("GPIO pad must be PA0..PE7");
    return (registers_[gpio_port_offsets[port] + 6] & (1u << bit)) != 0;
}

GpioPull Tlsr8258::gpio_pull(unsigned port, unsigned bit) const {
    if (port >= gpio_port_offsets.size() || bit > 7)
        throw std::out_of_range("GPIO pad must be PA0..PE7");
    // Two bits per pad in analog registers 0x0e + 2 * port + (bit >= 4):
    // 0 floating, 1 pull-up 1M, 2 pull-down 100K, 3 pull-up 10K.
    const std::uint8_t reg = analog_registers_[0x0e + 2 * port + (bit >= 4 ? 1 : 0)];
    switch ((reg >> ((bit & 3u) * 2u)) & 3u) {
    case 1:
    case 3: return GpioPull::Up;
    case 2: return GpioPull::Down;
    default: return GpioPull::None;
    }
}

std::uint8_t Tlsr8258::gpio_output(unsigned port) const {
    if (port >= gpio_outputs_.size())
        throw std::out_of_range("GPIO port must be PA..PE (0..4)");
    return gpio_outputs_[port];
}

std::uint8_t Tlsr8258::pwm_enable_mask() const noexcept {
    const std::uint32_t pwm_enable_offset = PwmEnable - RegisterBase;
    const std::uint32_t pwm0_enable_offset = Pwm0Enable - RegisterBase;
    return static_cast<std::uint8_t>(
        (registers_[pwm_enable_offset] & 0x3eu) |
        (registers_[pwm0_enable_offset] & 0x01u));
}

void Tlsr8258::record_output_event(OutputEvent::Kind kind, unsigned index,
                                  std::uint8_t value) {
    if (output_trace_enabled_)
        output_events_.push_back({kind, index, value, network_time_ns()});
}

void Tlsr8258::record_analog_register_event(
    AnalogRegisterEvent::Direction direction, std::uint8_t address,
    std::uint8_t value) {
    if (!analog_trace_enabled_)
        return;
    if (analog_register_event_count_ !=
        std::numeric_limits<std::uint64_t>::max())
        ++analog_register_event_count_;
    if (analog_register_events_.size() < analog_trace_limit_)
        analog_register_events_.push_back(
            {network_time_ns(), direction, address, value});
}

std::uint32_t Tlsr8258::dma_address(std::uint32_t low_offset,
                                   std::uint32_t high_offset) const noexcept {
    const std::uint32_t low =
        static_cast<std::uint32_t>(registers_[low_offset]) |
        (static_cast<std::uint32_t>(registers_[low_offset + 1]) << 8);
    std::uint32_t high = registers_[high_offset];
    if (high == 0)
        high = RetainedSramBase >> 16;
    else if ((high & 0x80u) == 0) {
        // The DMA high byte is relative to the 0x800000 memory window.
        high |= 0x80u;
    }
    return (high << 16) | low;
}

void Tlsr8258::start_radio_transmit() {
    const std::uint32_t address = dma_address(0x0c0c, 0x0c43);
    const std::uint8_t dma_length = read8(address + 4u);
    const std::size_t payload_size = dma_length >= 2u ? dma_length - 2u : 0u;
    std::vector<std::uint8_t> payload;
    payload.reserve(payload_size);
    for (std::size_t i = 0; i != payload_size; ++i)
        payload.push_back(read8(address + 5u + static_cast<std::uint32_t>(i)));

    if (radio_air_ && payload_size != 0 && payload_size <= 125u &&
        (registers_[0xf02] & 0x10u) != 0) {
        radio_air_->transmit(radio_node_name_, {}, RadioProtocol::Zigbee802154,
                             radio_channel_, network_time_ns(),
                             std::move(payload));
        radio_tx_finish_ns_ = radio_air_->frames().back().end_ns;
        radio_tx_busy_ = true;
        registers_[0xf20] &= static_cast<std::uint8_t>(~0x02u);
        return;
    }

    // Complete locally when no shared medium is attached or this is not a
    // supported Zigbee TX configuration, so firmware polling can still run.
    radio_tx_finish_ns_ = network_time_ns();
    radio_tx_busy_ = true;
}

void Tlsr8258::poll_radio() {
    if (radio_tx_busy_ && network_time_ns() >= radio_tx_finish_ns_) {
        radio_tx_busy_ = false;
        registers_[0xc24] &= static_cast<std::uint8_t>(~0x08u);
        registers_[0xf20] |= 0x02u; // RF_TX complete.
        update_radio_irq();
    }

    if (!radio_air_ || time_ns_ < radio_next_poll_ns_)
        return;
    const std::uint64_t previous_poll_local_ns = radio_last_poll_ns_;
    const std::uint64_t poll_lag_ns = time_ns_ - previous_poll_local_ns;
    const std::uint64_t current_network_ns = network_time_ns();
    const std::uint64_t previous_poll_network_ns =
        current_network_ns >= poll_lag_ns ? current_network_ns - poll_lag_ns : 0;
    radio_last_poll_ns_ = time_ns_;
    radio_next_poll_ns_ = time_ns_ + 10000u; // Poll at 10 us intervals.
    // RF mode and the DMA channel are separate enables on the 8258. 0x428
    // enables the RF receiver; DMA channel 2 (0xc20 bit 2) must also be on
    // before an arriving packet can reach the firmware's RX buffer.
    if ((registers_[0xf02] & 0x20u) == 0 ||
        (registers_[0x428] & 0x01u) == 0 ||
        (registers_[0xc20] & 0x04u) == 0)
        return;

    for (const RadioFrame &frame : radio_air_->receive(
             radio_node_name_, RadioProtocol::Zigbee802154,
             radio_channel_, current_network_ns, previous_poll_network_ns)) {
        const std::uint32_t address = dma_address(0x0c08, 0x0c42);
        // The 8258 DMA length includes the two on-air FCS bytes. The MAC
        // parser subtracts them before interpreting the frame, just as the
        // TX path removes the DMA-only FCS count before putting a packet on
        // the virtual air.
        const std::size_t required_size = frame.payload.size() + 15u;
        if (address < RetainedSramBase ||
            address >= RetainedSramBase + SramSize ||
            required_size > RetainedSramBase + SramSize - address ||
            frame.payload.size() > 125u)
            continue;

        const std::uint8_t packet_length =
            static_cast<std::uint8_t>(frame.payload.size() + 11u);
        write8(address, packet_length);
        write8(address + 1u, 0);
        write8(address + 2u, 0);
        write8(address + 3u, 0);
        write8(address + 4u,
               static_cast<std::uint8_t>(frame.payload.size() + 2u));
        for (std::size_t i = 0; i != frame.payload.size(); ++i)
            write8(address + 5u + static_cast<std::uint32_t>(i), frame.payload[i]);
        // VirtualAir carries the MAC frame without the generated FCS. Reserve
        // the two bytes present in the real DMA buffer; firmware excludes
        // them from the MAC PDU using p[4] - 2.
        write8(address + 5u + static_cast<std::uint32_t>(frame.payload.size()), 0);
        write8(address + 6u + static_cast<std::uint32_t>(frame.payload.size()), 0);
        const std::uint32_t timestamp_offset = packet_length - 4u;
        for (unsigned byte = 0; byte != 4; ++byte)
            write8(address + timestamp_offset + byte,
                   static_cast<std::uint8_t>(system_timer_count_ >> (byte * 8u)));
        // The SDK subtracts 110 from this raw byte to produce dBm, so the
        // medium's RSSI is encoded as dBm + 110 (-50 dBm in the ideal model).
        write8(address + packet_length + 2u,
               static_cast<std::uint8_t>(std::clamp(frame.rssi_dbm + 110, 0, 255)));
        write8(address + packet_length + 3u, 0x10u); // CRC valid
        registers_[0xf20] |= 0x01u; // RF_RX complete.
        update_radio_irq();
    }
}

void Tlsr8258::update_radio_irq() {
    const std::uint16_t mask = static_cast<std::uint16_t>(registers_[0xf1c]) |
        (static_cast<std::uint16_t>(registers_[0xf1d]) << 8);
    const std::uint16_t status = static_cast<std::uint16_t>(registers_[0xf20]) |
        (static_cast<std::uint16_t>(registers_[0xf21]) << 8);
    constexpr std::uint8_t zb_irq_bit = 1u << 5; // FLD_IRQ_ZB_RT_EN, IRQ 13.
    if ((mask & status) != 0)
        registers_[0x649] |= zb_irq_bit;
    else
        registers_[0x649] &= static_cast<std::uint8_t>(~zb_irq_bit);
}

bool Tlsr8258::uart_rx_dma_enabled() const noexcept {
    // ctrl0 bit 4 turns the receiver's DMA on, DMA channel 0 must be enabled
    // and have a buffer of at least one 16-byte unit.
    return (registers_[0x96] & 0x10u) != 0 && (registers_[0xc20] & 0x01u) != 0 &&
           registers_[0xc02] != 0;
}

void Tlsr8258::uart_dma_receive(std::uint8_t value) {
    const std::uint32_t address = dma_address(0x0c00, 0x0c40);
    const std::uint32_t capacity = static_cast<std::uint32_t>(registers_[0xc02]) * 16u;
    if (capacity <= 4u || uart_dma_rx_count_ + 4u >= capacity) {
        uart_rx_error_ = true; // no room: the byte is lost
        update_uart_status();
        return;
    }
    write8(address + 4u + uart_dma_rx_count_, value);
    ++uart_dma_rx_count_;
    // A packet ends after the line has been idle for the receive timeout:
    // 12 bit times, scaled by the multiplier in 0x9b[1:0].
    const std::uint64_t bit_cycles = std::max<std::uint64_t>(1u, uart_frame_cycles() / 10u);
    uart_dma_rx_idle_cycles_ = 12u * bit_cycles * ((registers_[0x9b] & 0x03u) + 1u);
    if (uart_dma_rx_count_ + 4u >= capacity)
        finish_uart_dma_receive();
}

void Tlsr8258::finish_uart_dma_receive() {
    const std::uint32_t address = dma_address(0x0c00, 0x0c40);
    for (unsigned i = 0; i != 4; ++i)
        write8(address + i, static_cast<std::uint8_t>(uart_dma_rx_count_ >> (8u * i)));
    uart_dma_rx_count_ = 0;
    uart_dma_rx_idle_cycles_ = 0;
    registers_[0xc26] |= 0x01u; // UART RX DMA done
    update_dma_irq();
}

void Tlsr8258::start_uart_dma_transmit() {
    const std::uint32_t address = dma_address(0x0c04, 0x0c41);
    std::uint32_t length = 0;
    for (unsigned i = 0; i != 4; ++i)
        length |= static_cast<std::uint32_t>(read8(address + i)) << (8u * i);
    constexpr std::uint32_t MaxDmaTransfer = 4096;
    length = std::min(length, MaxDmaTransfer);
    for (std::uint32_t i = 0; i != length; ++i)
        uart_dma_tx_queue_.push_back(read8(address + 4u + i));
    uart_dma_tx_active_ = true;
    uart_tx_done_ = length == 0 && uart_tx_fifo_.empty();
    if (length == 0) { // nothing to send: complete at once
        uart_dma_tx_active_ = false;
        registers_[0xc24] &= static_cast<std::uint8_t>(~0x02u);
        registers_[0xc26] |= 0x02u;
        update_dma_irq();
    }
    refill_uart_tx_from_dma();
    start_uart_transmit();
}

void Tlsr8258::refill_uart_tx_from_dma() {
    while (!uart_dma_tx_queue_.empty() && uart_tx_fifo_.size() < uart_fifo_capacity) {
        uart_tx_fifo_.push_back(uart_dma_tx_queue_.front());
        uart_dma_tx_queue_.pop_front();
    }
}

void Tlsr8258::update_dma_irq() noexcept {
    // The DMA interrupt (FLD_IRQ_DMA_EN, bit 4 of the source register) mirrors
    // the channel status gated by the channel interrupt mask.
    constexpr std::uint8_t dma_irq_bit = 1u << 4;
    if ((registers_[0xc26] & registers_[0xc21]) != 0)
        registers_[0x648] |= dma_irq_bit;
    else
        registers_[0x648] &= static_cast<std::uint8_t>(~dma_irq_bit);
}

bool Tlsr8258::inject_uart_rx(std::uint8_t value) {
    if (uart_rx_dma_enabled()) {
        uart_dma_receive(value);
        return true;
    }
    if (uart_rx_fifo_.size() == uart_fifo_capacity) {
        uart_rx_error_ = true;
        update_uart_status();
        return false;
    }
    uart_rx_fifo_.push_back(value);
    update_uart_status();
    return true;
}

std::vector<std::uint8_t> Tlsr8258::take_uart_transmitted() {
    std::vector<std::uint8_t> result;
    result.swap(uart_transmitted_);
    return result;
}

std::uint32_t Tlsr8258::timer_tick(unsigned index) const {
    return index < timer_last_tick_.size() ? timer_last_tick_[index] : 0;
}

std::uint32_t Tlsr8258::analog_32k_tick() const noexcept {
    // Firmware reads AREG 0x40..0x43 as a 32-bit 32 kHz clock. tick() uses
    // 32 kHz for the default RC source and 32.768 kHz for the crystal source.
    return static_cast<std::uint32_t>(analog_32k_tick_count_);
}

void Tlsr8258::update_analog_calibration() noexcept {
    if (!rc32k_calibration_pending_ ||
        time_ns_ < rc32k_calibration_ready_at_ns_)
        return;

    rc32k_calibration_pending_ = false;
    if ((analog_registers_[AnalogClockPower] &
         AnalogClock32kRcPowerDown) == 0)
        analog_registers_[AnalogRcCalibrationStatus] |=
            AnalogRc32kCalibrationReady;
}

std::uint8_t Tlsr8258::analog_register_read(std::uint8_t address) noexcept {
    update_analog_calibration();
    if (address >= 0x40 && address <= 0x43) {
        const unsigned byte = address - 0x40;
        return static_cast<std::uint8_t>(analog_32k_tick() >> (byte * 8u));
    }
    return analog_registers_[address];
}

std::uint8_t Tlsr8258::read_register(std::uint32_t offset) {
    if (offset == 0x002)
        return i2c_status();
    if (offset == 0xb1e)
        return static_cast<std::uint8_t>(dfifo2_write_pointer_ & 0xffu);
    if (offset == 0xb1f)
        return static_cast<std::uint8_t>(dfifo2_write_pointer_ >> 8);
    if (offset == 0x00c && mspi_active_ && mspi_command_seen_) {
        if (mspi_command_ == 0x9f) {
            const std::array<std::uint8_t, 3> jedec_id = {
                0xef, 0x40,
                static_cast<std::uint8_t>(flash_.size() == 512u * 1024u
                                              ? 0x13
                                              : 0x14),
            };
            if (mspi_response_index_ < jedec_id.size())
                return jedec_id[mspi_response_index_++];
            return 0xff;
        }
        if (mspi_command_ == 0x05)
            return 0; // Flash is ready: WIP=0.
    }
    if (offset >= 0x740 && offset <= 0x743) {
        const std::uint32_t count = system_timer_count_ & ~std::uint32_t{7};
        return static_cast<std::uint8_t>(count >> ((offset - 0x740) * 8u));
    }
    if (offset >= 0x548 && offset <= 0x54b &&
        (registers_[0x540] & aes_codec_finished_bit) != 0 &&
        aes_output_word_index_ < 4) {
        const unsigned byte = offset - 0x548;
        const std::uint8_t value =
            aes_output_[aes_output_word_index_ * 4u + byte];
        if (byte == 3 && ++aes_output_word_index_ == 4)
            registers_[0x540] &= static_cast<std::uint8_t>(
                ~aes_codec_finished_bit);
        return value;
    }
    for (unsigned port = 0; port != gpio_port_offsets.size(); ++port) {
        const std::uint32_t base = gpio_port_offsets[port];
        if (offset == base)
            return gpio_inputs_[port];
        if (offset == base + 3)
            return gpio_outputs_[port];
    }
    if (offset >= 0x630 && offset <= 0x63b) {
        const unsigned index = (offset - 0x630) / 4u;
        const unsigned byte = (offset - 0x630) % 4u;
        return static_cast<std::uint8_t>(timer_tick(index) >> (byte * 8u));
    }
    if (offset >= uart_data_first && offset <= uart_data_last) {
        if (uart_rx_fifo_.empty())
            return registers_[offset];
        const std::uint8_t value = uart_rx_fifo_.front();
        uart_rx_fifo_.pop_front();
        update_uart_status();
        return value;
    }
    if (offset == 0x9c || offset == 0x9d || offset == 0x9e)
        update_uart_status();
    return registers_[offset];
}

void Tlsr8258::update_system_timer_irq(
    std::uint32_t previous_count) noexcept {
    if ((registers_[SystemTimerMode - RegisterBase] &
         SystemTimerIrqEnable) == 0)
        return;

    const std::uint32_t compare =
        static_cast<std::uint32_t>(registers_[0x744]) |
        (static_cast<std::uint32_t>(registers_[0x745]) << 8) |
        (static_cast<std::uint32_t>(registers_[0x746]) << 16) |
        (static_cast<std::uint32_t>(registers_[0x747]) << 24);
    const bool before_compare =
        static_cast<std::int32_t>(previous_count - compare) < 0;
    const bool at_or_after_compare =
        static_cast<std::int32_t>(system_timer_count_ - compare) >= 0;
    if (before_compare && at_or_after_compare)
        registers_[0x64a] |= 1u << 4; // IRQ 20: system timer.
}

void Tlsr8258::update_system_timer_calibration() noexcept {
    if (!system_timer_calibration_pending_ ||
        analog_32k_tick_count_ < system_timer_calibration_target_tick_)
        return;

    system_timer_calibration_pending_ = false;
    const std::uint16_t calibration =
        (registers_[Clock32kSelect - RegisterBase] & Clock32kSelectXtal) != 0
            ? 7812u
            : rc32k_system_timer_calibration_;
    registers_[0x750] = static_cast<std::uint8_t>(calibration);
    registers_[0x751] = static_cast<std::uint8_t>(calibration >> 8);
}

void Tlsr8258::write_register(std::uint32_t offset, std::uint8_t value) {
    if (offset == 0x007) {
        registers_[offset] = value;
        i2c_start_command(value);
        return;
    }
    if (offset == 0x061) {
        registers_[offset] = value;
        if ((value & 0x20u) != 0) // FLD_RST1_ADC: restart the sample clock
            adc_next_sample_ns_ = 0;
        return;
    }
    if (offset == 0xb1e || offset == 0xb1f) {
        // reg_dfifo2_wptr: firmware clears it when it (re)configures the buffer.
        const unsigned shift = (offset - 0xb1e) * 8u;
        dfifo2_write_pointer_ = static_cast<std::uint16_t>(
            (dfifo2_write_pointer_ & ~(0xffu << shift)) | (value << shift));
        return;
    }
    if (offset >= 0x648 && offset <= 0x64a) {
        // IRQ source bits are write-one-to-clear; in particular Zephyr's
        // system timer ISR clears IRQ 20 with a 32-bit write at 0x800648.
        registers_[offset] &= static_cast<std::uint8_t>(~value);
        update_dma_irq(); // level sources stay asserted while pending
        return;
    }
    if (offset >= System32kTickCompare - RegisterBase &&
        offset < System32kTickCompare - RegisterBase + 4) {
        registers_[offset] = value;
        return;
    }
    if (offset == 0x74f) {
        // Bits 0, 1, and 3 are command strobes used for 32 kHz calibration,
        // timer setup, and compare programming. Hardware clears them after
        // accepting each command; retain the wake timer arm separately.
        wake_timer_armed_ = (value & 0x08u) != 0;
        registers_[offset] = value &
            static_cast<std::uint8_t>(~SystemTimerControlStrobeMask);
        if ((value & 0x01u) != 0 &&
            (registers_[0x74c] & SystemTimer32kCalibrationEnable) != 0) {
            // SYS_TIMER_CTRL1 starts a measurement over 16 cycles of the
            // selected 32 kHz source. Firmware can change other SYS_TIMER_CTRL0
            // bits while the measurement is in flight.
            registers_[0x750] = 0;
            registers_[0x751] = 0;
            if (selected_32k_clock_running()) {
                system_timer_calibration_target_tick_ =
                    analog_32k_tick_count_ + 16u;
                system_timer_calibration_pending_ = true;
            } else {
                system_timer_calibration_pending_ = false;
            }
        }
        return;
    }
    if (offset == PowerDownControl - RegisterBase) {
        registers_[offset] = value;
        if ((value & (PowerDownSleepEnable | PowerDownSleepTrigger)) ==
            (PowerDownSleepEnable | PowerDownSleepTrigger))
            enter_sleep();
        return;
    }
    if (offset == rf_frequency_high_offset) {
        registers_[offset] = value;
        std::uint8_t channel = 0;
        if (decode_zigbee_channel(registers_[rf_frequency_low_offset], value,
                                  channel))
            radio_channel_ = channel;
        return;
    }
    if (offset == 0x0c24) {
        const bool tx_ready_before = (registers_[offset] & 0x08u) != 0;
        const bool uart_tx_before = (registers_[offset] & 0x02u) != 0;
        registers_[offset] = value;
        if (!tx_ready_before && (value & 0x08u) != 0)
            start_radio_transmit();
        if (!uart_tx_before && (value & 0x02u) != 0)
            start_uart_dma_transmit();
        return;
    }
    if (offset == 0x0c26) { // DMA interrupt status: write one to clear
        registers_[offset] &= static_cast<std::uint8_t>(~value);
        update_dma_irq();
        return;
    }
    if (offset == 0x0c21) {
        registers_[offset] = value;
        update_dma_irq();
        return;
    }
    if (offset == 0x0f20 || offset == 0x0f21) {
        registers_[offset] &= static_cast<std::uint8_t>(~value);
        update_radio_irq();
        return;
    }
    if (offset == 0x0f1c || offset == 0x0f1d) {
        registers_[offset] = value;
        update_radio_irq();
        return;
    }
    if (offset == 0x00d) {
        registers_[offset] = value & static_cast<std::uint8_t>(~0x10u);
        if (value == 0) {
            mspi_active_ = true;
            mspi_command_seen_ = false;
            mspi_response_index_ = 0;
        } else if ((value & 1u) != 0) {
            mspi_active_ = false;
        }
        return;
    }
    if (offset == 0x00c) {
        registers_[offset] = value;
        if (mspi_active_ && !mspi_command_seen_) {
            mspi_command_ = value;
            mspi_command_seen_ = true;
            mspi_response_index_ = 0;
        }
        return;
    }
    if (offset == 0x660) {
        const std::uint32_t dividend =
            static_cast<std::uint32_t>(registers_[0x664]) |
            (static_cast<std::uint32_t>(registers_[0x665]) << 8) |
            (static_cast<std::uint32_t>(registers_[0x666]) << 16) |
            (static_cast<std::uint32_t>(registers_[0x667]) << 24);
        const std::uint32_t divisor =
            static_cast<std::uint32_t>(registers_[0x668]) |
            (static_cast<std::uint32_t>(registers_[0x669]) << 8) |
            (static_cast<std::uint32_t>(registers_[0x66a]) << 16) |
            (static_cast<std::uint32_t>(registers_[0x66b]) << 24);
        const bool signed_operation = (value & 1u) != 0;
        const bool remainder_operation = (value & 2u) != 0;
        std::uint32_t quotient = 0xffffffffu;
        std::uint32_t remainder = dividend;
        if (divisor != 0) {
            if (signed_operation) {
                const auto signed_dividend = static_cast<std::int32_t>(dividend);
                const auto signed_divisor = static_cast<std::int32_t>(divisor);
                if (signed_dividend == std::numeric_limits<std::int32_t>::min() &&
                    signed_divisor == -1) {
                    quotient = dividend;
                    remainder = 0;
                } else {
                    quotient = static_cast<std::uint32_t>(
                        signed_dividend / signed_divisor);
                    remainder = static_cast<std::uint32_t>(
                        signed_dividend % signed_divisor);
                }
            } else {
                quotient = dividend / divisor;
                remainder = dividend % divisor;
            }
        }
        for (unsigned byte = 0; byte != 4; ++byte) {
            registers_[0x664 + byte] =
                static_cast<std::uint8_t>(quotient >> (byte * 8u));
            registers_[0x668 + byte] =
                static_cast<std::uint8_t>(remainder >> (byte * 8u));
        }
        (void)remainder_operation; // Both results are latched; firmware selects one.
        // The vendor helper polls until this register clears to zero.
        registers_[offset] = 0; // Complete the modeled operation synchronously.
        return;
    }
    if (offset == 0x540) {
        registers_[offset] = static_cast<std::uint8_t>(
            (value & 0x81u) | aes_data_feed_bit);
        aes_decrypt_ = (value & 1u) != 0;
        aes_input_word_ = 0;
        aes_input_word_count_ = 0;
        aes_output_word_index_ = 0;
        aes_cycles_remaining_ = 0;
        return;
    }
    if (offset >= 0x550 && offset <= 0x55f) {
        registers_[offset] = value;
        aes_key_[offset - 0x550] = value;
        return;
    }
    if (offset >= 0x548 && offset <= 0x54b) {
        const unsigned byte = offset - 0x548;
        const std::uint32_t mask = 0xffu << (byte * 8u);
        aes_input_word_ = (aes_input_word_ & ~mask) |
                          (static_cast<std::uint32_t>(value) << (byte * 8u));
        if (byte == 3 && aes_input_word_count_ < 4) {
            const unsigned start = aes_input_word_count_ * 4u;
            for (unsigned i = 0; i != 4; ++i)
                aes_input_[start + i] = static_cast<std::uint8_t>(
                    aes_input_word_ >> (i * 8u));
            ++aes_input_word_count_;
            if (aes_input_word_count_ == 4) {
                registers_[0x540] &= static_cast<std::uint8_t>(
                    ~aes_data_feed_bit);
                registers_[0x540] &= static_cast<std::uint8_t>(
                    ~aes_codec_finished_bit);
                aes_cycles_remaining_ = aes_block_cycles;
            }
        }
        return;
    }
    if (offset >= 0x740 && offset <= 0x743) {
        const unsigned shift = (offset - 0x740) * 8u;
        const std::uint32_t mask = std::uint32_t{0xff} << shift;
        system_timer_count_ =
            (system_timer_count_ & ~mask) |
            (static_cast<std::uint32_t>(value) << shift);
        return;
    }
    if (offset == 0x0ba) {
        // AREG transactions complete synchronously in the model. CYC0 starts
        // the transfer; RW selects a write, while reads place the selected
        // analog register value in ANA_DATA. BUSY is therefore always clear.
        registers_[offset] = value & static_cast<std::uint8_t>(~1u);
        if ((value & (1u << 6)) != 0) {
            const std::uint8_t address = registers_[0x0b8];
            if ((value & (1u << 5)) != 0) {
                const std::uint8_t data = registers_[0x0b9];
                if (address < 0x40 || address > 0x43) {
                    if (address == AnalogWakeStatus) {
                        // AREG 0x44 wake status bits clear by writing one.
                        analog_registers_[address] &=
                            static_cast<std::uint8_t>(~data);
                    } else {
                        analog_registers_[address] = data;
                        if (address >= 0x0e && address <= 0x17)
                            notify_gpio_pad((address - 0x0eu) / 2u);
                    }
                    // cpu_wakeup_init's RC24M calibration writes 0x0e then
                    // 0x0f to AREG 0xc7 and polls AREG 0xcf[7]. Complete that
                    // hardware calibration synchronously in the virtual model.
                    if (address == 0xc7 && data == 0x0f)
                        analog_registers_[AnalogRcCalibrationStatus] |= 0x80;

                    // The vendor rc_32k_cal routine writes 0xf6 then 0xf7 to
                    // AREG 0xc6 and polls AREG 0xcf[6]. The datasheet gives a
                    // 3 ms RC 32 kHz calibration time but not these internal
                    // register semantics, so model the observed SDK handshake
                    // with a virtual-time completion event.
                    if (address == AnalogRc32kCalibrationControl) {
                        if (data == 0xf6) {
                            analog_registers_[AnalogRcCalibrationStatus] &=
                                static_cast<std::uint8_t>(
                                    ~AnalogRc32kCalibrationReady);
                            rc32k_calibration_pending_ = false;
                        } else if (data == 0xf7) {
                            analog_registers_[AnalogRcCalibrationStatus] &=
                                static_cast<std::uint8_t>(
                                    ~AnalogRc32kCalibrationReady);
                            if ((analog_registers_[AnalogClockPower] &
                                 AnalogClock32kRcPowerDown) == 0) {
                                const auto max_time =
                                    std::numeric_limits<std::uint64_t>::max();
                                rc32k_calibration_ready_at_ns_ =
                                    time_ns_ > max_time - Rc32kCalibrationTimeNs
                                        ? max_time
                                        : time_ns_ + Rc32kCalibrationTimeNs;
                                rc32k_calibration_pending_ = true;
                            } else {
                                rc32k_calibration_pending_ = false;
                            }
                        }
                    }
                }
                record_analog_register_event(
                    AnalogRegisterEvent::Direction::Write, address, data);
            } else {
                const std::uint8_t data = analog_register_read(address);
                registers_[0x0b9] = data;
                record_analog_register_event(
                    AnalogRegisterEvent::Direction::Read, address, data);
            }
        }
        return;
    }
    if (offset == 0x623) {
        registers_[offset] &= static_cast<std::uint8_t>(~value); // write one to clear
        registers_[0x648] &= static_cast<std::uint8_t>(~value);
        for (unsigned timer = 0; timer != 3; ++timer)
            if (value & (1u << timer))
                timer_fired_[timer] = false;
        return;
    }
    if (offset >= 0x648 && offset <= 0x64b) {
        registers_[offset] &= static_cast<std::uint8_t>(~value);
        return;
    }
    if (offset >= uart_data_first && offset <= uart_data_last) {
        if (uart_tx_fifo_.size() < uart_fifo_capacity) {
            uart_tx_fifo_.push_back(value);
            uart_tx_done_ = false;
            start_uart_transmit();
        }
        update_uart_status();
        return;
    }
    if (offset == 0x9d) {
        if (value & (1u << 6))
            uart_rx_error_ = false;
        update_uart_status();
        return;
    }
    if (offset == 0x9f) {
        if (value & 0x80u)
            uart_tx_done_ = false;
        registers_[offset] = value & 0x7fu;
        update_uart_status();
        return;
    }
    for (unsigned port = 0; port != gpio_port_offsets.size(); ++port) {
        const std::uint32_t base = gpio_port_offsets[port];
        if (offset == base)
            return; // GPIO input pins are driven by the external environment.
        if (offset == base + 3) {
            if (gpio_outputs_[port] != value) {
                gpio_outputs_[port] = value;
                record_output_event(OutputEvent::Kind::GpioPort, port, value);
            }
            registers_[offset] = value;
            if (gpio_pad_observer_)
                gpio_pad_observer_(port);
            return;
        }
        if (offset == base + 2 || offset == base + 6) {
            const bool changed = registers_[offset] != value;
            registers_[offset] = value;
            if (changed && gpio_pad_observer_)
                gpio_pad_observer_(port);
            return;
        }
    }
    if (offset >= 0x630 && offset <= 0x63b)
        return; // timer counters are read-only

    const bool pwm_enable_write =
        offset == PwmEnable - RegisterBase ||
        offset == Pwm0Enable - RegisterBase;
    const std::uint8_t previous_pwm_enable =
        pwm_enable_write ? pwm_enable_mask() : 0;
    registers_[offset] = value;
    if (pwm_enable_write) {
        const std::uint8_t current_pwm_enable = pwm_enable_mask();
        if (current_pwm_enable != previous_pwm_enable)
            record_output_event(OutputEvent::Kind::PwmEnableMask, 0,
                                current_pwm_enable);
    }
    if (offset == 0x620 || offset == 0x621 || offset == 0x622) {
        for (unsigned timer = 0; timer != 3; ++timer) {
            const unsigned enable_bit = timer * 3u;
            const bool enabled = (registers_[0x620 + enable_bit / 8u] &
                                  (1u << (enable_bit % 8u))) != 0;
            if (!enabled)
                timer_fired_[timer] = false;
        }
    }
    if (offset >= 0x94 && offset <= 0x99)
        update_uart_status();
}

std::uint64_t Tlsr8258::uart_frame_cycles() const {
    const std::uint32_t divider =
        static_cast<std::uint32_t>(registers_[0x94]) |
        (static_cast<std::uint32_t>(registers_[0x95] & 0x7fu) << 8);
    const std::uint64_t uart_clock_div =
        (registers_[0x95] & 0x80u) != 0 ? divider + 1u : 1u;
    const std::uint32_t control0 =
        static_cast<std::uint32_t>(registers_[0x96]) |
        (static_cast<std::uint32_t>(registers_[0x97]) << 8);
    const unsigned bit_width = (control0 & 0x0fu) + 1u;
    const unsigned control1 = registers_[0x97];
    const unsigned stop = (control1 >> 4) & 0x03u;
    const unsigned stop_half_bits = stop == 0 ? 2u : stop == 1 ? 3u : 4u;
    const unsigned parity_half_bits = (control1 & 0x04u) != 0 ? 2u : 0u;
    const unsigned frame_half_bits = 2u + 16u + parity_half_bits + stop_half_bits;
    const std::uint64_t numerator = uart_clock_div * bit_width * frame_half_bits;
    // Integer cycle stepping rounds a half-cycle frame up to the next core cycle.
    return std::max<std::uint64_t>(1u, (numerator + 1u) / 2u);
}

void Tlsr8258::start_uart_transmit() {
    if (!uart_tx_fifo_.empty() && uart_tx_cycles_remaining_ == 0)
        uart_tx_cycles_remaining_ = uart_frame_cycles();
}

void Tlsr8258::update_uart_status() {
    const auto rx_count = static_cast<std::uint8_t>(uart_rx_fifo_.size());
    const auto tx_count = static_cast<std::uint8_t>(uart_tx_fifo_.size());
    registers_[0x9c] = static_cast<std::uint8_t>((rx_count & 0x0fu) |
                                                 ((tx_count & 0x0fu) << 4));

    const unsigned rx_trigger = registers_[0x99] & 0x0fu;
    const unsigned tx_trigger = (registers_[0x99] >> 4) & 0x0fu;
    std::uint8_t status1 = uart_tx_done_ ? uart_tx_done_bit : 0;
    if (tx_count <= tx_trigger)
        status1 |= uart_tx_buffer_irq_bit;
    if (rx_trigger != 0 && rx_count >= rx_trigger)
        status1 |= uart_rx_buffer_irq_bit;
    registers_[0x9e] = status1;

    const unsigned ctrl0 = registers_[0x96];
    const bool irq = ((ctrl0 & 0x40u) != 0 &&
                      (status1 & uart_rx_buffer_irq_bit) != 0) ||
                     ((ctrl0 & 0x80u) != 0 &&
                      (status1 & uart_tx_buffer_irq_bit) != 0);
    registers_[0x9d] = static_cast<std::uint8_t>(
        (rx_count & 0x07u) | ((tx_count & 0x07u) << 4) |
        (irq ? uart_irq_flag_bit : 0) | (uart_rx_error_ ? 0x80u : 0));
    if (irq)
        registers_[0x648] |= uart_irq_mask_bit;
    else
        registers_[0x648] &= static_cast<std::uint8_t>(~uart_irq_mask_bit);
}

Tlsr8258Model parse_model(const std::string &name) {
    std::string normalized;
    normalized.reserve(name.size());
    for (char c : name)
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    if (normalized == "tlsr8258" || normalized == "tlsr8258f512" ||
        normalized == "f512")
        return Tlsr8258Model::F512;
    if (normalized == "tlsr8258f1k" || normalized == "f1k")
        return Tlsr8258Model::F1K;
    throw std::invalid_argument("unknown model; use TLSR8258F512 or TLSR8258F1K");
}

const char *model_name(Tlsr8258Model model) noexcept {
    return model == Tlsr8258Model::F512 ? "TLSR8258F512" : "TLSR8258F1K";
}

} // namespace tc32emu
