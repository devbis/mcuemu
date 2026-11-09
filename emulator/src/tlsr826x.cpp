#include "mcuemu/tlsr826x.hpp"
#include "mcuemu/aes.hpp"
#include "mcuemu/radio.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

// Evidence labels used in comments, as in tlsr8258.cpp:
//   SDK      platform/chip_826x/register.h and the Zigbee SDK sources
//   DISASM   the 826x driver objects (libdrivers_826x.a, drivers-8269/sdk/asm)
//   INFERRED same behaviour as the TLSR8258 model, not seen in 826x material
//   SIMPLIFIED modelled coarser than the silicon
namespace mcuemu {
namespace {

constexpr std::array<std::uint32_t, Tlsr826x::PortCount> gpio_port_offsets = {
    0x580, 0x588, 0x590, 0x598, 0x5a0, // SDK: in, ie, oen, out, pol, ds, gpio, irq_en per port
};
// SDK: reg_gpio_wakeup_irq (0x5b5) gates GPIO interrupts; IRQ 18 = source 0x64a bit 2.
constexpr std::uint32_t gpio_irq_source_offset = 0x64a;
constexpr std::uint8_t gpio_irq_source_bit = 1u << 2;
constexpr std::uint32_t gpio_core_irq_gate_offset = 0x5b5;
constexpr std::uint8_t gpio_core_irq_gate_bit = 1u << 3;
constexpr std::uint8_t irq_dma_bit = 1u << 4;   // SDK: FLD_IRQ_DMA_EN
constexpr std::uint8_t irq_uart_bit = 1u << 6;  // SDK: FLD_IRQ_UART_EN
constexpr std::uint8_t irq_zb_rt_bit = 1u << 5; // SDK: FLD_IRQ_ZB_RT_EN (bit 13, byte 0x649)
// DISASM: RF_TrxStateSet writes 2400 + channel offset (MHz) to the 16-bit 0x4d6.
constexpr std::uint32_t rf_frequency_low_offset = 0x4d6;
constexpr std::uint32_t rf_frequency_high_offset = 0x4d7;

bool decode_zigbee_channel(std::uint8_t low, std::uint8_t high, std::uint8_t &channel) {
    // Zigbee channel n (11..26) is 2405 + 5 * (n - 11) MHz, so the offset from
    // 2400 is 5 * (n - 10).
    const int mhz = ((static_cast<int>(high) << 8) | low) - 2400;
    if (mhz < 5 || mhz > 80 || mhz % 5 != 0)
        return false;
    channel = static_cast<std::uint8_t>(10 + mhz / 5);
    return true;
}

} // namespace

Tlsr826x::Tlsr826x(Tlsr826xModel model, std::uint32_t clock_hz)
    : model_(model),
      layout_{SramBase, model == Tlsr826xModel::T8269 ? 0x8000u : 0x4000u, RegisterBase,
              RegisterSize, 0},
      clock_hz_(clock_hz),
      flash_(FlashSize, 0xff) {
    if (clock_hz_ == 0)
        throw std::invalid_argument("TLSR826x clock must be nonzero");
    registers_.fill(0);
    analog_registers_.fill(0);
    sram_.fill(0);
    gpio_inputs_.fill(0xff);
    gpio_outputs_.fill(0);
    gpio_irq_active_.fill(0);
    for (std::uint32_t base : gpio_port_offsets) {
        registers_[base + 2] = 0xff; // INFERRED: OEN resets to "all inputs"
        registers_[base + 6] = 0xff; // INFERRED: every pad starts as GPIO
    }
    timer_last_tick_.fill(0);
    timer_fired_.fill(false);
    timer_stopped_.fill(false);
    // INFERRED: SYS_TIMER_CTRL0 resets to 0x90 as on the 8258 (32 kHz calibration enabled).
    registers_[0x74c] = 0x90;
    registers_[0x540] = 0x02; // data-feed ready, as the 8258 datasheet reset value
    // DISASM uart_Send: TX is allowed while 0x9e bit 0 is set, so it is set when idle.
    registers_[0x9e] = 1;
}

void Tlsr826x::load_firmware(const std::vector<std::uint8_t> &image, std::uint32_t offset) {
    if (offset > flash_.size() || image.size() > flash_.size() - offset)
        throw std::out_of_range("firmware image does not fit in TLSR826x flash");
    std::copy(image.begin(), image.end(), flash_.begin() + offset);
}

std::uint8_t Tlsr826x::read8(std::uint32_t address) {
    if (address < flash_.size())
        return flash_[address];
    if (address >= RegisterBase && address < RegisterBase + RegisterSize)
        return read_register(address - RegisterBase);
    if (address >= SramBase && address - SramBase < layout_.sram_size)
        return sram_[address - SramBase];
    throw BusFault(address, "read");
}

void Tlsr826x::write8(std::uint32_t address, std::uint8_t value) {
    if (address < flash_.size())
        throw BusFault(address, "write to read-only flash");
    if (address >= RegisterBase && address < RegisterBase + RegisterSize) {
        write_register(address - RegisterBase, value);
        return;
    }
    if (address >= SramBase && address - SramBase < layout_.sram_size) {
        sram_[address - SramBase] = value;
        return;
    }
    throw BusFault(address, "write");
}

bool Tlsr826x::take_flash_dirty(std::uint32_t &begin, std::uint32_t &end) noexcept {
    if (flash_dirty_end_ <= flash_dirty_begin_)
        return false;
    begin = flash_dirty_begin_;
    end = flash_dirty_end_;
    flash_dirty_begin_ = flash_dirty_end_ = 0;
    return true;
}

// ---- analog registers (0xb8 address, 0xb9 data, 0xba control) -------------

std::uint8_t Tlsr826x::analog_read(std::uint8_t address) const noexcept {
    return analog_registers_[address];
}

void Tlsr826x::analog_write(std::uint8_t address, std::uint8_t value) {
    if (address == 0x44) { // wake status: bits clear by writing one (DISASM: 0x0f clears)
        analog_registers_[address] &= static_cast<std::uint8_t>(~value);
        return;
    }
    analog_registers_[address] = value;
    // DISASM rc_32k_cal: writes 0x2e = 0xf6, then 0xf7, then polls AREG 0x44
    // bit 4 and copies AREG 0x45/0x46 into 0x31/0x32. SIMPLIFIED: the
    // calibration completes at once with a nominal result.
    if (address == 0x2e && value == 0xf7) {
        analog_registers_[0x44] |= 0x10;
        analog_registers_[0x45] = 0x00;
        analog_registers_[0x46] = 0x80;
    }
    if (address >= 0x0e && address <= 0x17)
        notify_gpio_pad((address - 0x0eu) / 2u); // INFERRED: pull layout as on the 8258
}

// ---- timing ----------------------------------------------------------------

void Tlsr826x::update_system_timer_irq(std::uint32_t previous_count) noexcept {
    if ((registers_[0x74c] & 0x02u) == 0) // SDK: FLD_SYSTEM_TICK_IRQ_EN
        return;
    const std::uint32_t compare = reg32(0x744);
    const bool before = static_cast<std::int32_t>(previous_count - compare) < 0;
    const bool after = static_cast<std::int32_t>(system_timer_count_ - compare) >= 0;
    if (before && after)
        registers_[0x64a] |= 1u << 4; // IRQ 20: system timer
}

std::uint32_t Tlsr826x::timer_capture(unsigned timer) const noexcept {
    return reg32(0x624 + timer * 4u);
}

void Tlsr826x::tick(std::uint32_t cycles) {
    constexpr std::uint64_t ns_per_s = 1000000000u;
    if (system_timer_started_) {
        const std::uint32_t previous = system_timer_count_;
        const std::uint64_t numerator =
            static_cast<std::uint64_t>(cycles) * SystemTimerClockHz + system_timer_fraction_;
        system_timer_count_ += static_cast<std::uint32_t>(numerator / clock_hz_);
        system_timer_fraction_ = numerator % clock_hz_;
        update_system_timer_irq(previous);
    }
    // 32 kHz tick read through 0x754 (DISASM PM_Get32kTick). INFERRED: RC 32 kHz.
    {
        const std::uint64_t numerator =
            static_cast<std::uint64_t>(cycles) * DefaultRc32kFrequencyHz + k32_fraction_;
        k32_ticks_ += numerator / clock_hz_;
        k32_fraction_ = numerator % clock_hz_;
        if (k32_calibration_pending_ && k32_ticks_ >= k32_calibration_target_) {
            // DISASM PM_WakeupInit: writes 0x750 = 1, starts the measurement with
            // 0x74f = 1 and polls 0x750 until it changes. The result is the
            // number of 16 MHz ticks in 16 cycles of the 32 kHz clock.
            k32_calibration_pending_ = false;
            const std::uint32_t result = static_cast<std::uint32_t>(
                16ull * SystemTimerClockHz / DefaultRc32kFrequencyHz);
            registers_[0x750] = static_cast<std::uint8_t>(result);
            registers_[0x751] = static_cast<std::uint8_t>(result >> 8);
        }
    }

    const std::uint64_t whole = cycles / clock_hz_;
    const std::uint64_t rest = cycles % clock_hz_;
    const std::uint64_t numerator = rest * ns_per_s + time_fraction_;
    time_ns_ += whole * ns_per_s + numerator / clock_hz_;
    time_fraction_ = numerator % clock_hz_;

    const std::uint32_t control = reg32(0x620);
    for (unsigned timer = 0; timer != 3; ++timer) {
        if ((control & (1u << (timer * 3u))) == 0 || timer_stopped_[timer])
            continue;
        const std::uint32_t capture = timer_capture(timer);
        std::uint32_t count = timer_last_tick_[timer] + cycles;
        if (capture != 0 && count >= capture && timer_last_tick_[timer] < capture)
            count = capture; // stops exactly at the match
        timer_last_tick_[timer] = count;
        if (!timer_fired_[timer] && capture != 0 && count >= capture) {
            timer_fired_[timer] = true;
            timer_stopped_[timer] = true;
            registers_[0x623] |= static_cast<std::uint8_t>(1u << timer);
            registers_[0x648] |= static_cast<std::uint8_t>(1u << timer);
        }
    }

    if (aes_cycles_remaining_ != 0) {
        if (cycles >= aes_cycles_remaining_) {
            aes_cycles_remaining_ = 0;
            aes_block(aes_key_.data(), 16, aes_input_.data(), aes_output_.data(), aes_decrypt_);
            registers_[0x540] |= 0x04; // codec finished
        } else {
            aes_cycles_remaining_ -= cycles;
        }
    }
    if (sleep_entry_pending_) {
        if (cycles >= sleep_entry_cycles_remaining_) {
            sleep_entry_pending_ = false;
            sleep_entry_cycles_remaining_ = 0;
            power_state_ = Tlsr8258PowerState::Suspend;
        } else {
            sleep_entry_cycles_remaining_ -= cycles;
        }
    }

    // UART transmit: DMA channel 1 drains the queue one byte per frame time.
    std::uint64_t remaining = cycles;
    while (!uart_dma_tx_queue_.empty()) {
        if (uart_tx_cycles_remaining_ == 0)
            uart_tx_cycles_remaining_ = uart_byte_cycles();
        if (remaining < uart_tx_cycles_remaining_) {
            uart_tx_cycles_remaining_ -= remaining;
            remaining = 0;
            break;
        }
        remaining -= uart_tx_cycles_remaining_;
        uart_tx_cycles_remaining_ = 0;
        const std::uint8_t sent = uart_dma_tx_queue_.front();
        uart_dma_tx_queue_.pop_front();
        uart_transmitted_.push_back(sent);
        if (uart_dma_tx_queue_.empty() && uart_dma_tx_active_) {
            uart_dma_tx_active_ = false;
            registers_[0x524] &= static_cast<std::uint8_t>(~0x02u);
            registers_[0x526] |= 0x02u; // channel 1 done
            update_dma_irq();
        }
        if ((registers_[0x97] & 0x80u) != 0) // INFERRED: loopback bit, as on the 8258
            inject_uart_rx(sent);
        if (remaining == 0)
            break;
    }
    if (uart_dma_rx_idle_cycles_ != 0) {
        if (cycles >= uart_dma_rx_idle_cycles_)
            finish_uart_dma_receive();
        else
            uart_dma_rx_idle_cycles_ -= cycles;
    }
    update_uart_status();
    poll_radio();
}

std::uint64_t Tlsr826x::cycles_until_event() const noexcept {
    constexpr std::uint64_t unlimited = std::numeric_limits<std::uint64_t>::max() / 4;
    std::uint64_t bound = unlimited;
    auto limit = [&bound](std::uint64_t cycles) { bound = std::min(bound, cycles); };
    auto ns_to_cycles = [this](std::uint64_t ns) -> std::uint64_t {
        if (ns <= 1)
            return 0;
        return std::min<std::uint64_t>(ns - 1, 4000000000ull) * clock_hz_ / 1000000000ull;
    };
    if (system_timer_started_ && (registers_[0x74c] & 0x02u) != 0) {
        const std::uint32_t distance = reg32(0x744) - system_timer_count_;
        if (static_cast<std::int32_t>(distance) > 0)
            limit((static_cast<std::uint64_t>(distance) - 1) * (clock_hz_ / SystemTimerClockHz));
    }
    const std::uint32_t control = reg32(0x620);
    for (unsigned timer = 0; timer != 3; ++timer) {
        if ((control & (1u << (timer * 3u))) == 0 || timer_fired_[timer] || timer_stopped_[timer])
            continue;
        const std::uint32_t capture = timer_capture(timer);
        if (capture == 0)
            continue;
        const std::uint32_t count = timer_last_tick_[timer];
        limit(count >= capture ? 0 : capture - count);
    }
    if (sleep_entry_pending_)
        limit(sleep_entry_cycles_remaining_);
    if (aes_cycles_remaining_ != 0)
        limit(aes_cycles_remaining_);
    if (!uart_dma_tx_queue_.empty())
        limit(0);
    if (k32_calibration_pending_) // 16 ticks of 32 kHz: one poll of the core
        limit(clock_hz_ / DefaultRc32kFrequencyHz * 4u);
    if (uart_dma_rx_idle_cycles_ != 0)
        limit(uart_dma_rx_idle_cycles_);
    if (radio_tx_busy_) {
        const std::uint64_t now = network_time_ns();
        limit(radio_tx_finish_ns_ > now ? ns_to_cycles(radio_tx_finish_ns_ - now) : 0);
    }
    if (radio_air_)
        limit(radio_next_poll_ns_ > time_ns_ ? ns_to_cycles(radio_next_poll_ns_ - time_ns_) : 0);
    return bound;
}

bool Tlsr826x::interrupt_pending() const noexcept {
    if ((registers_[0x643] & 1u) == 0) // SDK: reg_irq_en
        return false;
    for (unsigned byte = 0; byte != 3; ++byte)
        if ((registers_[0x640 + byte] & registers_[0x648 + byte]) != 0)
            return true;
    return false;
}


bool Tlsr826x::advance_sleep_to_wakeup() {
    if (power_state_ == Tlsr8258PowerState::Awake || !wake_timer_armed_)
        return false;
    // SIMPLIFIED: only the 32 kHz timer wakes the core (no pad, no deep sleep).
    const std::uint32_t current = static_cast<std::uint32_t>(k32_ticks_);
    std::uint32_t delta = reg32(0x754) - current;
    if (delta > 0x7fffffffu)
        delta = 0; // the compare is already behind: wake at once
    const std::uint64_t ns = (static_cast<std::uint64_t>(delta) * 1000000000ull +
                              DefaultRc32kFrequencyHz - 1) / DefaultRc32kFrequencyHz;
    time_ns_ += ns;
    time_fraction_ = 0;
    k32_ticks_ += delta;
    k32_fraction_ = 0;
    if (system_timer_started_) {
        const std::uint32_t previous = system_timer_count_;
        system_timer_count_ += static_cast<std::uint32_t>(
            static_cast<std::uint64_t>(delta) * (16ull * SystemTimerClockHz / DefaultRc32kFrequencyHz) / 16u);
        update_system_timer_irq(previous);
    }
    analog_registers_[0x44] |= 0x02; // timer wake status (DISASM pm_sleep_wakeup tests bit 1)
    wake_timer_armed_ = false;
    power_state_ = Tlsr8258PowerState::Awake;
    return true;
}

// ---- GPIO ------------------------------------------------------------------

void Tlsr826x::set_gpio_input(unsigned port, std::uint8_t value) {
    if (port >= PortCount)
        throw std::out_of_range("GPIO port must be PA..PE (0..4)");
    gpio_inputs_[port] = value;
    update_gpio_irq();
}

void Tlsr826x::update_gpio_irq() {
    const bool gate = (registers_[gpio_core_irq_gate_offset] & gpio_core_irq_gate_bit) != 0;
    for (unsigned port = 0; port != PortCount; ++port) {
        const std::uint32_t base = gpio_port_offsets[port];
        const std::uint8_t active =
            gate ? static_cast<std::uint8_t>((gpio_inputs_[port] ^ registers_[base + 4]) &
                                             registers_[base + 7])
                 : std::uint8_t{0};
        if ((active & static_cast<std::uint8_t>(~gpio_irq_active_[port])) != 0)
            registers_[gpio_irq_source_offset] |= gpio_irq_source_bit;
        gpio_irq_active_[port] = active;
    }
}

std::uint8_t Tlsr826x::gpio_output(unsigned port) const {
    if (port >= PortCount)
        throw std::out_of_range("GPIO port must be PA..PE (0..4)");
    return gpio_outputs_[port];
}

std::uint8_t Tlsr826x::gpio_oen(unsigned port) const {
    if (port >= PortCount)
        throw std::out_of_range("GPIO port must be PA..PE (0..4)");
    return registers_[gpio_port_offsets[port] + 2];
}

bool Tlsr826x::gpio_is_gpio(unsigned port, unsigned bit) const {
    if (port >= PortCount || bit > 7)
        throw std::out_of_range("GPIO pad must be PA0..PE7");
    return (registers_[gpio_port_offsets[port] + 6] & (1u << bit)) != 0;
}

GpioPull Tlsr826x::gpio_pull(unsigned port, unsigned bit) const {
    if (port >= PortCount || bit > 7)
        throw std::out_of_range("GPIO pad must be PA0..PE7");
    // INFERRED: the same two-bit pull fields in AREG 0x0e.. as the 8258.
    const std::uint8_t reg = analog_registers_[0x0e + 2 * port + (bit >= 4 ? 1 : 0)];
    switch ((reg >> ((bit & 3u) * 2u)) & 3u) {
    case 1:
    case 3: return GpioPull::Up;
    case 2: return GpioPull::Down;
    default: return GpioPull::None;
    }
}

// ---- UART (DMA only on the 826x) ---------------------------------------------

bool Tlsr826x::uart_rx_dma_enabled() const noexcept {
    // SDK FLD_UART_RX_DMA_EN = ctrl0 bit 4; DMA channel 0 enable (0x520 bit 0);
    // 0x502 holds the buffer size in 16-byte units (DISASM uart_RecBuffInit).
    return (registers_[0x96] & 0x10u) != 0 && (registers_[0x520] & 0x01u) != 0 &&
           registers_[0x502] != 0;
}

std::uint64_t Tlsr826x::uart_byte_cycles() const noexcept {
    // SDK: clk_div (bits 0..14) and bit width (ctrl0 bits 0..3); one 8N1 frame is
    // 10 bit times. INFERRED: parity (ctrl0 bit 10) adds one.
    const std::uint32_t divider = (registers_[0x94] | (registers_[0x95] << 8)) & 0x7fffu;
    const std::uint32_t bit_width = (registers_[0x96] & 0x0fu) + 1u;
    const std::uint32_t bits = 10u + ((registers_[0x97] & 0x04u) != 0 ? 1u : 0u);
    return std::max<std::uint64_t>(1u, static_cast<std::uint64_t>(divider + 1u) * bit_width * bits);
}

void Tlsr826x::uart_dma_receive(std::uint8_t value) {
    const std::uint32_t address = dma_address(0x500);
    const std::uint32_t capacity = static_cast<std::uint32_t>(registers_[0x502]) * 16u;
    if (capacity <= 4u || uart_dma_rx_count_ + 4u >= capacity) {
        uart_rx_error_ = true;
        update_uart_status();
        return;
    }
    write8(address + 4u + uart_dma_rx_count_, value);
    ++uart_dma_rx_count_;
    // SIMPLIFIED: a packet ends after 12 bit times of idle line, as on the 8258.
    uart_dma_rx_idle_cycles_ = 12u * std::max<std::uint64_t>(1u, uart_byte_cycles() / 10u);
    if (uart_dma_rx_count_ + 4u >= capacity)
        finish_uart_dma_receive();
}

void Tlsr826x::finish_uart_dma_receive() {
    const std::uint32_t address = dma_address(0x500);
    for (unsigned i = 0; i != 4; ++i)
        write8(address + i, static_cast<std::uint8_t>(uart_dma_rx_count_ >> (8u * i)));
    uart_dma_rx_count_ = 0;
    uart_dma_rx_idle_cycles_ = 0;
    registers_[0x526] |= 0x01u; // channel 0 done
    update_dma_irq();
}

void Tlsr826x::start_uart_dma_transmit() {
    const std::uint32_t address = dma_address(0x504);
    std::uint32_t length = 0;
    for (unsigned i = 0; i != 4; ++i)
        length |= static_cast<std::uint32_t>(read8(address + i)) << (8u * i);
    length = std::min<std::uint32_t>(length, 4096);
    for (std::uint32_t i = 0; i != length; ++i)
        uart_dma_tx_queue_.push_back(read8(address + 4u + i));
    uart_dma_tx_active_ = length != 0;
    if (length == 0) {
        registers_[0x524] &= static_cast<std::uint8_t>(~0x02u);
        registers_[0x526] |= 0x02u;
        update_dma_irq();
    }
}

bool Tlsr826x::inject_uart_rx(std::uint8_t value) {
    if (!uart_rx_dma_enabled()) {
        uart_rx_error_ = true;
        update_uart_status();
        return false;
    }
    uart_dma_receive(value);
    return true;
}

std::vector<std::uint8_t> Tlsr826x::take_uart_transmitted() {
    std::vector<std::uint8_t> result;
    result.swap(uart_transmitted_);
    return result;
}

void Tlsr826x::update_uart_status() {
    // DISASM uart_Send waits for 0x9e bit 0; uart_ErrorCLR tests 0x9d bit 7 and
    // writes bit 6 to clear it.
    const bool busy = uart_dma_tx_active_ || !uart_dma_tx_queue_.empty();
    registers_[0x9e] = static_cast<std::uint8_t>((registers_[0x9e] & ~1u) | (busy ? 0u : 1u));
    registers_[0x9d] = static_cast<std::uint8_t>(uart_rx_error_ ? 0x80u : 0u);
}

void Tlsr826x::update_dma_irq() noexcept {
    // DISASM DMA_IRQSourceGet: 0x526 holds the channel status (bit 0 UART RX,
    // 1 UART TX, 2 RF RX, 3 RF TX); 0x521 is the channel interrupt mask.
    const bool pending = (registers_[0x526] & registers_[0x521]) != 0;
    if (pending)
        registers_[0x648] |= irq_dma_bit;
    else
        registers_[0x648] &= static_cast<std::uint8_t>(~irq_dma_bit);
    (void)irq_uart_bit;
}

// ---- RF ----------------------------------------------------------------------

void Tlsr826x::connect_radio(VirtualAir &air, std::string node_name) {
    if (node_name.empty())
        throw std::invalid_argument("virtual radio node must have a name");
    radio_air_ = &air;
    radio_node_name_ = std::move(node_name);
    radio_next_poll_ns_ = time_ns_;
    radio_last_poll_ns_ = time_ns_;
}

void Tlsr826x::set_radio_channel(std::uint8_t zigbee_channel) {
    if (zigbee_channel < 11 || zigbee_channel > 26)
        throw std::invalid_argument("Zigbee channel must be 11..26");
    radio_channel_ = zigbee_channel;
}

std::uint32_t Tlsr826x::dma_address(std::uint32_t offset) const noexcept {
    // DISASM: the DMA address registers are 16 bits, the SRAM lies at 0x80xxxx.
    return 0x800000u | registers_[offset] | (static_cast<std::uint32_t>(registers_[offset + 1]) << 8);
}

// The TX buffer is "dma header (4 bytes, p[0] = length + 1), length byte p[4]
// = payload + 2 (FCS), payload": ZB_RADIO_DMA_HDR_BUILD and ZB_RADIO_TX_HDR_LEN
// are the same for the 826x and the 8258 in the Zigbee SDK (drv_radio.h).
void Tlsr826x::start_radio_transmit() {
    const std::uint32_t address = dma_address(0x50c); // DISASM RF_TxPkt: 0x50c
    const std::uint8_t dma_length = read8(address + 4u);
    const std::size_t payload_size = dma_length >= 2u ? dma_length - 2u : 0u;
    std::vector<std::uint8_t> payload;
    payload.reserve(payload_size);
    for (std::size_t i = 0; i != payload_size; ++i)
        payload.push_back(read8(address + 5u + static_cast<std::uint32_t>(i)));

    // DISASM RF_TrxStateSet: TX mode sets 0xf02 bit 4, RX mode sets bit 5 and 0x428 bit 0.
    if (radio_air_ && payload_size != 0 && payload_size <= 125u && (registers_[0xf02] & 0x10u) != 0) {
        const std::uint64_t id = radio_air_->transmit(
            radio_node_name_, {}, RadioProtocol::Zigbee802154, radio_channel_,
            network_time_ns(), std::move(payload));
        radio_tx_finish_ns_ = radio_air_->frames()[id - 1].end_ns;
        radio_tx_busy_ = true;
        registers_[0xf20] &= static_cast<std::uint8_t>(~0x02u);
        return;
    }
    radio_tx_finish_ns_ = network_time_ns();
    radio_tx_busy_ = true;
}

void Tlsr826x::poll_radio() {
    if (radio_tx_busy_ && network_time_ns() >= radio_tx_finish_ns_) {
        radio_tx_busy_ = false;
        registers_[0x524] &= static_cast<std::uint8_t>(~0x08u);
        registers_[0xf20] |= 0x02u; // FLD_RF_IRQ_TX
        update_radio_irq();
    }
    if (!radio_air_ || time_ns_ < radio_next_poll_ns_)
        return;
    const std::uint64_t lag = time_ns_ - radio_last_poll_ns_;
    const std::uint64_t now = network_time_ns();
    const std::uint64_t previous = now >= lag ? now - lag : 0;
    radio_last_poll_ns_ = time_ns_;
    radio_next_poll_ns_ = time_ns_ + 10000u;
    if ((registers_[0xf02] & 0x20u) == 0 || (registers_[0x428] & 0x01u) == 0 ||
        (registers_[0x520] & 0x04u) == 0)
        return;

    for (const RadioFrame &frame : radio_air_->receive(radio_node_name_, RadioProtocol::Zigbee802154,
                                                         radio_channel_, now, previous)) {
        const std::uint32_t address = dma_address(0x508); // DISASM RF_RxBufferSet: 0x508
        // RX buffer layout (SDK rf_drv.h macros, confirmed by the Zephyr 8269 driver):
        // p[0] = p[12] + 13, p[4] = RSSI (dBm + 110), p[8..11] = 32-bit timestamp,
        // p[12] = PHY length (payload + 2 FCS), the MAC frame from p[13], and the
        // status byte at p[p[0] + 3] with (status & 0x51) == 0x10 for a good CRC.
        // This differs from the 8258, where p[4] is the length and the MAC frame
        // starts at p[5].
        const std::size_t required = frame.payload.size() + 20u;
        if (address < SramBase || address >= SramBase + layout_.sram_size ||
            required > SramBase + layout_.sram_size - address || frame.payload.size() > 125u)
            continue;
        const std::uint8_t phy_length = static_cast<std::uint8_t>(frame.payload.size() + 2u);
        const std::uint8_t packet_length = static_cast<std::uint8_t>(phy_length + 13u);
        for (std::uint32_t i = 0; i != 13u; ++i)
            write8(address + i, 0);
        write8(address, packet_length);
        // SDK RF_GET_PKT_RSSI(p) = p[4]; INFERRED encoding dBm + 110 as on the 8258.
        write8(address + 4u, static_cast<std::uint8_t>(std::clamp(frame.rssi_dbm + 110, 0, 255)));
        for (unsigned byte = 0; byte != 4; ++byte)
            write8(address + 8u + byte, static_cast<std::uint8_t>(system_timer_count_ >> (byte * 8u)));
        write8(address + 12u, phy_length);
        for (std::size_t i = 0; i != frame.payload.size(); ++i)
            write8(address + 13u + static_cast<std::uint32_t>(i), frame.payload[i]);
        write8(address + 13u + static_cast<std::uint32_t>(frame.payload.size()), 0); // FCS
        write8(address + 14u + static_cast<std::uint32_t>(frame.payload.size()), 0);
        write8(address + packet_length + 3u, 0x10u); // CRC ok
        registers_[0xf20] |= 0x01u; // FLD_RF_IRQ_RX
        registers_[0x526] |= 0x04u; // DISASM RF_RxBufferRequest tests bit 2 of 0x526
        update_radio_irq();
    }
}

void Tlsr826x::update_radio_irq() {
    const std::uint16_t mask = registers_[0xf1c] | (static_cast<std::uint16_t>(registers_[0xf1d]) << 8);
    const std::uint16_t status = registers_[0xf20] | (static_cast<std::uint16_t>(registers_[0xf21]) << 8);
    if ((mask & status) != 0)
        registers_[0x649] |= irq_zb_rt_bit;
    else
        registers_[0x649] &= static_cast<std::uint8_t>(~irq_zb_rt_bit);
}

// ---- MSPI flash --------------------------------------------------------------
// Same 25-series command handling as the TLSR8258 model (see tlsr8258.cpp);
// SDK chip_826x/flash.c drives it through 0x0c/0x0d the same way.

void Tlsr826x::mspi_command_byte(std::uint8_t value) {
    mspi_command_ = value;
    mspi_command_seen_ = true;
    mspi_response_index_ = 0;
    mspi_write_count_ = 0;
    mspi_address_ = 0;
    mspi_read_pos_ = 0;
    if (value == 0x06)
        mspi_write_enable_ = true;
    else if (value == 0x04)
        mspi_write_enable_ = false;
    else if (value == 0xc7 && mspi_write_enable_)
        flash_erase(0, static_cast<std::uint32_t>(flash_.size()));
}

void Tlsr826x::mspi_data_byte(std::uint8_t value) {
    const unsigned n = ++mspi_write_count_;
    switch (mspi_command_) {
    case 0x03:
    case 0x0b:
    case 0x02:
    case 0x20:
    case 0x52:
    case 0xd8:
        if (n <= 3) {
            mspi_address_ = (mspi_address_ << 8) | value;
            if (n == 3) {
                mspi_address_ &= 0xffffffu;
                if (mspi_write_enable_) {
                    if (mspi_command_ == 0x20)
                        flash_erase(mspi_address_ & ~0xfffu, 0x1000);
                    else if (mspi_command_ == 0x52)
                        flash_erase(mspi_address_ & ~0x7fffu, 0x8000);
                    else if (mspi_command_ == 0xd8)
                        flash_erase(mspi_address_ & ~0xffffu, 0x10000);
                }
            }
        } else if (mspi_command_ == 0x02) {
            if (mspi_write_enable_) {
                flash_program(mspi_address_, value);
                mspi_address_ = (mspi_address_ & ~0xffu) | ((mspi_address_ + 1u) & 0xffu);
            }
        } else if ((mspi_command_ == 0x03 && n == 4) || (mspi_command_ == 0x0b && n == 5)) {
            mspi_read_pos_ = mspi_address_;
        }
        break;
    default:
        break;
    }
}

std::uint8_t Tlsr826x::mspi_read_byte() {
    switch (mspi_command_) {
    case 0x9f: {
        // SIMPLIFIED: a 4 Mbit (512 KB) part; manufacturer and type as the 8258 model.
        const std::array<std::uint8_t, 3> jedec_id = {0xef, 0x40, 0x13};
        if (mspi_response_index_ < jedec_id.size())
            return jedec_id[mspi_response_index_++];
        return 0xff;
    }
    case 0x05:
        return mspi_write_enable_ ? 0x02 : 0x00;
    case 0x03:
    case 0x0b: {
        const std::uint32_t pos = mspi_read_pos_++;
        return pos < flash_.size() ? flash_[pos] : 0xff;
    }
    case 0x4b:
        return static_cast<std::uint8_t>(0x3c + 0x1d * (mspi_read_pos_++ & 15u));
    default:
        return registers_[0x00c];
    }
}

void Tlsr826x::mspi_deselect() {
    switch (mspi_command_) {
    case 0x02:
    case 0x20:
    case 0x52:
    case 0xd8:
    case 0xc7:
        if (mspi_command_seen_)
            mspi_write_enable_ = false;
        break;
    default:
        break;
    }
    mspi_active_ = false;
}

void Tlsr826x::flash_program(std::uint32_t address, std::uint8_t value) {
    if (address >= flash_.size())
        return;
    flash_[address] &= value;
    if (flash_dirty_end_ <= flash_dirty_begin_) {
        flash_dirty_begin_ = address;
        flash_dirty_end_ = address + 1;
    } else {
        flash_dirty_begin_ = std::min(flash_dirty_begin_, address);
        flash_dirty_end_ = std::max(flash_dirty_end_, address + 1);
    }
}

void Tlsr826x::flash_erase(std::uint32_t address, std::uint32_t size) {
    if (address >= flash_.size())
        return;
    const std::uint32_t end = static_cast<std::uint32_t>(
        std::min<std::size_t>(flash_.size(), std::size_t{address} + size));
    std::fill(flash_.begin() + address, flash_.begin() + end, 0xff);
    if (flash_dirty_end_ <= flash_dirty_begin_) {
        flash_dirty_begin_ = address;
        flash_dirty_end_ = end;
    } else {
        flash_dirty_begin_ = std::min(flash_dirty_begin_, address);
        flash_dirty_end_ = std::max(flash_dirty_end_, end);
    }
}

// ---- register file -----------------------------------------------------------

std::uint8_t Tlsr826x::read_register(std::uint32_t offset) {
    if (offset == 0x00c && mspi_active_ && mspi_command_seen_)
        return mspi_read_byte();
    if (offset >= 0x740 && offset <= 0x743)
        return static_cast<std::uint8_t>((system_timer_count_ & ~std::uint32_t{7}) >> ((offset - 0x740) * 8u));
    if (offset >= 0x548 && offset <= 0x54b && (registers_[0x540] & 0x04u) != 0 && aes_output_words_ < 4) {
        const unsigned byte = offset - 0x548;
        const std::uint8_t value = aes_output_[aes_output_words_ * 4u + byte];
        if (byte == 3 && ++aes_output_words_ == 4)
            registers_[0x540] &= static_cast<std::uint8_t>(~0x04u);
        return value;
    }
    if (offset == 0x440) {
        // DISASM RF_GetRssi: dBm = 212 - table[reg 0x440 & 0x1f] when (0x446 & 0x3f) == 0 (AGC
        // settled). Index 8 selects table value 0x38, i.e. -100 dBm: a quiet channel, so
        // the MAC's clear channel assessment (CCA threshold) passes. SIMPLIFIED: no
        // dependence on frames on the air.
        return 8;
    }
    if (offset == 0x74f) // SDK: FLD_SYSTEM_TICK_RUNNING = bit 1
        return static_cast<std::uint8_t>((registers_[offset] & ~2u) | (system_timer_started_ ? 2u : 0u));
    if (offset >= 0x754 && offset <= 0x757) // DISASM PM_Get32kTick: 32 kHz tick
        return static_cast<std::uint8_t>(k32_ticks_ >> ((offset - 0x754) * 8u));
    for (unsigned port = 0; port != PortCount; ++port) {
        const std::uint32_t base = gpio_port_offsets[port];
        if (offset == base)
            return gpio_inputs_[port];
        if (offset == base + 3)
            return gpio_outputs_[port];
    }
    if (offset >= 0x630 && offset <= 0x63b) {
        const unsigned index = (offset - 0x630) / 4u;
        return static_cast<std::uint8_t>(timer_last_tick_[index] >> (((offset - 0x630) % 4u) * 8u));
    }
    if (offset == 0x9d || offset == 0x9e)
        update_uart_status();
    return registers_[offset];
}

void Tlsr826x::write_register(std::uint32_t offset, std::uint8_t value) {
    if (offset >= 0x648 && offset <= 0x64a) { // write one to clear
        registers_[offset] &= static_cast<std::uint8_t>(~value);
        update_dma_irq();
        return;
    }
    if (offset == 0x623) {
        registers_[offset] &= static_cast<std::uint8_t>(~value);
        registers_[0x648] &= static_cast<std::uint8_t>(~value);
        for (unsigned timer = 0; timer != 3; ++timer)
            if (value & (1u << timer))
                timer_fired_[timer] = false;
        return;
    }
    if (offset >= 0x630 && offset <= 0x63b) {
        const unsigned index = (offset - 0x630) / 4u;
        const unsigned byte = (offset - 0x630) % 4u;
        std::uint32_t &tick = timer_last_tick_[index];
        tick = (tick & ~(0xffu << (byte * 8u))) | (static_cast<std::uint32_t>(value) << (byte * 8u));
        timer_stopped_[index] = false;
        return;
    }
    if (offset >= 0x740 && offset <= 0x743) {
        const unsigned shift = (offset - 0x740) * 8u;
        system_timer_count_ = (system_timer_count_ & ~(0xffu << shift)) |
                              (static_cast<std::uint32_t>(value) << shift);
        return;
    }
    if (offset == 0x74f) {
        // SDK: bit 0 START, bit 1 STOP. DISASM PM_Get32kTick also writes bit 3
        // and polls it to clear: SIMPLIFIED, the latch completes at once.
        if ((value & 0x02u) != 0)
            system_timer_started_ = false;
        if ((value & 0x01u) != 0)
            system_timer_started_ = true;
        registers_[offset] = value & static_cast<std::uint8_t>(~0x0bu);
        // DISASM pm_sleep_wakeup: writes the 32 kHz wake compare to 0x754, then 0x74f = 8
        // and polls until the bit clears; that latches the compare, which arms the wake.
        if ((value & 0x08u) != 0)
            wake_timer_armed_ = true;
        if ((value & 0x01u) != 0 && (registers_[0x74c] & 0x80u) != 0) {
            registers_[0x750] = 0;
            registers_[0x751] = 0;
            k32_calibration_pending_ = true;
            k32_calibration_target_ = k32_ticks_ + 16u;
        }
        return;
    }
    if (offset == 0x540) {
        registers_[offset] = static_cast<std::uint8_t>((value & 0x81u) | 0x02u);
        aes_decrypt_ = (value & 1u) != 0;
        aes_input_words_ = 0;
        aes_output_words_ = 0;
        aes_cycles_remaining_ = 0;
        return;
    }
    if (offset >= 0x550 && offset <= 0x55f) {
        registers_[offset] = value;
        aes_key_[offset - 0x550] = value;
        return;
    }
    if (offset >= 0x548 && offset <= 0x54b) {
        registers_[offset] = value;
        if (offset == 0x54b && aes_input_words_ < 4) {
            for (unsigned i = 0; i != 4; ++i)
                aes_input_[aes_input_words_ * 4u + i] = registers_[0x548 + i];
            if (++aes_input_words_ == 4) {
                registers_[0x540] &= static_cast<std::uint8_t>(~0x06u);
                aes_cycles_remaining_ = 1000; // INFERRED: the 8258 datasheet's duration
            }
        }
        return;
    }
    if (offset == 0x6f) {
        // DISASM pm_start: 0x6f = 0x81 puts the core to sleep after a short delay of
        // instructions (the 8258 trampoline runs 40; INFERRED the same here).
        registers_[offset] = value;
        if ((value & 0x80u) != 0) {
            sleep_entry_pending_ = true;
            sleep_entry_cycles_remaining_ = 40u * 3u;
        }
        return;
    }
    if (offset == rf_frequency_high_offset) {
        registers_[offset] = value;
        std::uint8_t channel = 0;
        if (decode_zigbee_channel(registers_[rf_frequency_low_offset], value, channel))
            radio_channel_ = channel;
        return;
    }
    if (offset == 0x524) { // DMA TX ready: bit 1 UART, bit 3 RF (DISASM uart_Send, RF_TxPkt)
        const bool rf_before = (registers_[offset] & 0x08u) != 0;
        const bool uart_before = (registers_[offset] & 0x02u) != 0;
        registers_[offset] = value;
        if (!rf_before && (value & 0x08u) != 0)
            start_radio_transmit();
        if (!uart_before && (value & 0x02u) != 0)
            start_uart_dma_transmit();
        return;
    }
    if (offset == 0x526 || offset == 0x527) { // channel status, write one to clear
        // DISASM RF_RxBufferClearFlag ORs bit 2 into 0x526/0x527 to release a buffer.
        registers_[offset] &= static_cast<std::uint8_t>(~value);
        update_dma_irq();
        return;
    }
    if (offset == 0x521) {
        registers_[offset] = value;
        update_dma_irq();
        return;
    }
    if (offset == 0xf20 || offset == 0xf21) {
        registers_[offset] &= static_cast<std::uint8_t>(~value);
        update_radio_irq();
        return;
    }
    if (offset == 0xf1c || offset == 0xf1d) {
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
            mspi_deselect();
        }
        return;
    }
    if (offset == 0x00c) {
        registers_[offset] = value;
        if (mspi_active_) {
            if (!mspi_command_seen_)
                mspi_command_byte(value);
            else
                mspi_data_byte(value);
        }
        return;
    }
    if (offset == 0x660) {
        // The Zigbee SDK's platform/tc32/div_mod.S is part of the 826x build and drives
        // this block (control 0x660, dividend 0x664, divisor/remainder 0x668), so the
        // silicon has it although register.h does not list it. INFERRED: same
        // semantics as the 8258 model.
        const std::uint32_t dividend = reg32(0x664);
        const std::uint32_t divisor = reg32(0x668);
        std::uint32_t quotient = 0xffffffffu;
        std::uint32_t remainder = dividend;
        if (divisor != 0) {
            if ((value & 1u) != 0) {
                const auto a = static_cast<std::int32_t>(dividend);
                const auto b = static_cast<std::int32_t>(divisor);
                if (a == std::numeric_limits<std::int32_t>::min() && b == -1) {
                    quotient = dividend;
                    remainder = 0;
                } else {
                    quotient = static_cast<std::uint32_t>(a / b);
                    remainder = static_cast<std::uint32_t>(a % b);
                }
            } else {
                quotient = dividend / divisor;
                remainder = dividend % divisor;
            }
        }
        for (unsigned byte = 0; byte != 4; ++byte) {
            registers_[0x664 + byte] = static_cast<std::uint8_t>(quotient >> (byte * 8u));
            registers_[0x668 + byte] = static_cast<std::uint8_t>(remainder >> (byte * 8u));
        }
        registers_[offset] = 0; // completes at once; firmware polls until it reads 0
        return;
    }
    if (offset == 0xba) { // SDK: reg_ana_ctrl; transfers complete synchronously
        registers_[offset] = value & static_cast<std::uint8_t>(~1u);
        if ((value & (1u << 6)) != 0) {
            const std::uint8_t address = registers_[0xb8];
            if ((value & (1u << 5)) != 0)
                analog_write(address, registers_[0xb9]);
            else
                registers_[0xb9] = analog_read(address);
        }
        return;
    }
    if (offset == 0x9d) { // DISASM uart_ErrorCLR
        if (value & (1u << 6))
            uart_rx_error_ = false;
        update_uart_status();
        return;
    }
    for (unsigned port = 0; port != PortCount; ++port) {
        const std::uint32_t base = gpio_port_offsets[port];
        if (offset == base)
            return; // inputs are driven by the environment
        if (offset == base + 3) {
            if (gpio_outputs_[port] != value) {
                gpio_outputs_[port] = value;
                if (output_trace_enabled_)
                    output_events_.push_back({OutputEvent::Kind::GpioPort, port, value, network_time_ns()});
            }
            registers_[offset] = value;
            notify_gpio_pad(port);
            return;
        }
        if (offset == base + 2 || offset == base + 6) {
            const bool changed = registers_[offset] != value;
            registers_[offset] = value;
            if (changed)
                notify_gpio_pad(port);
            return;
        }
    }
    registers_[offset] = value;
    if (offset == gpio_core_irq_gate_offset)
        update_gpio_irq();
    else
        for (std::uint32_t base : gpio_port_offsets)
            if (offset == base + 4 || offset == base + 7)
                update_gpio_irq();
    if (offset >= 0x620 && offset <= 0x622) {
        for (unsigned timer = 0; timer != 3; ++timer) {
            const unsigned enable_bit = timer * 3u;
            if ((registers_[0x620 + enable_bit / 8u] & (1u << (enable_bit % 8u))) == 0) {
                timer_fired_[timer] = false;
                timer_stopped_[timer] = false;
            }
        }
    }
}

Tlsr826xModel parse_tlsr826x_model(const std::string &name) {
    std::string n;
    for (char c : name)
        n.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    if (n == "tlsr8269" || n == "8269" || n == "tlsr826x")
        return Tlsr826xModel::T8269;
    if (n == "tlsr8267" || n == "8267")
        return Tlsr826xModel::T8267;
    throw std::invalid_argument("unknown model; use TLSR8269 or TLSR8267");
}

const char *model_name(Tlsr826xModel model) noexcept {
    return model == Tlsr826xModel::T8269 ? "TLSR8269" : "TLSR8267";
}

} // namespace mcuemu
