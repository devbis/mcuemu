#pragma once

#include "mcuemu/tc32_device.hpp"
#include "mcuemu/tlsr8258.hpp" // OutputEvent, GpioPull

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace mcuemu {

class VirtualAir;

// TLSR826x family (TLSR8267 / TLSR8269), documented by the Telink SDK headers
// (platform/chip_826x/register.h) and its disassembled driver library; there
// is no datasheet in the project. docs/tlsr826x.md lists what each register
// block is based on.
enum class Tlsr826xModel {
    T8267, // 16 KB SRAM
    T8269, // 32 KB SRAM
};

class Tlsr826x final : public Tc32Device {
public:
    static constexpr std::uint32_t RegisterBase = 0x800000;
    // SRAM starts at 0x808000, so the register window ends there.
    static constexpr std::uint32_t RegisterSize = 0x8000;
    static constexpr std::uint32_t SramBase = 0x808000;
    static constexpr std::uint32_t FlashSize = 512u * 1024u;
    static constexpr std::uint32_t DefaultClockHz = 32000000;
    static constexpr std::uint32_t SystemTimerClockHz = 16000000;
    static constexpr std::uint32_t DefaultRc32kFrequencyHz = 32000;
    static constexpr std::size_t PortCount = 5;

    explicit Tlsr826x(Tlsr826xModel model = Tlsr826xModel::T8269,
                      std::uint32_t clock_hz = DefaultClockHz);

    const Tc32Layout &layout() const noexcept override { return layout_; }
    Tlsr826xModel model() const noexcept { return model_; }
    std::uint8_t read8(std::uint32_t address) override;
    void write8(std::uint32_t address, std::uint8_t value) override;
    void tick(std::uint32_t cycles) override;
    std::uint64_t cycles_until_event() const noexcept override;
    bool interrupt_pending() const noexcept override;
    Tlsr8258PowerState power_state() const noexcept override { return power_state_; }
    // Suspend ends at the 32 kHz compare armed through 0x754/0x74f (see docs/tlsr826x.md).
    bool advance_sleep_to_wakeup() override;
    bool consume_deep_sleep_wakeup() noexcept override { return false; }

    void load_firmware(const std::vector<std::uint8_t> &image, std::uint32_t offset = 0);
    std::uint32_t flash_size() const noexcept override {
        return static_cast<std::uint32_t>(flash_.size());
    }
    const std::vector<std::uint8_t> &flash() const noexcept override { return flash_; }
    std::uint8_t *sram_data() noexcept override { return sram_.data(); }
    // The SDK startup writes the RAM code size / 256 to 0x60c (tag RAM start).
    std::uint32_t ram_code_bytes() const noexcept override {
        return static_cast<std::uint32_t>(registers_[0x60c]) << 8;
    }
    bool take_flash_dirty(std::uint32_t &begin, std::uint32_t &end) noexcept override;
    std::uint64_t time_ns() const noexcept { return time_ns_; }
    std::uint32_t clock_hz() const noexcept override { return clock_hz_; }
    std::uint64_t network_time_ns() const noexcept override {
        if (radio_time_offset_ns_ > std::numeric_limits<std::uint64_t>::max() - time_ns_)
            return std::numeric_limits<std::uint64_t>::max();
        return time_ns_ + radio_time_offset_ns_;
    }

    void set_gpio_input(unsigned port, std::uint8_t value);
    std::uint8_t gpio_output(unsigned port) const;
    std::uint8_t gpio_oen(unsigned port) const;
    bool gpio_is_gpio(unsigned port, unsigned bit) const;
    GpioPull gpio_pull(unsigned port, unsigned bit) const;
    using GpioPadObserver = std::function<void(unsigned port)>;
    void set_gpio_pad_observer(GpioPadObserver observer) {
        gpio_pad_observer_ = std::move(observer);
    }
    void set_output_trace_enabled(bool enabled) noexcept { output_trace_enabled_ = enabled; }
    const std::vector<OutputEvent> &output_events() const noexcept { return output_events_; }

    void connect_radio(VirtualAir &air, std::string node_name);
    void set_radio_time_offset(std::uint64_t offset_ns) noexcept { radio_time_offset_ns_ = offset_ns; }
    void set_radio_channel(std::uint8_t zigbee_channel);
    std::uint8_t radio_channel() const noexcept { return radio_channel_; }

    bool inject_uart_rx(std::uint8_t value);
    std::vector<std::uint8_t> take_uart_transmitted();

private:
    Tlsr826xModel model_;
    Tc32Layout layout_;
    std::uint32_t clock_hz_;
    std::uint64_t time_ns_{0};
    std::uint64_t time_fraction_{0};
    std::uint32_t system_timer_count_{0};
    std::uint64_t system_timer_fraction_{0};
    bool system_timer_started_{true};
    std::uint64_t k32_ticks_{0};
    std::uint64_t k32_fraction_{0};
    Tlsr8258PowerState power_state_{Tlsr8258PowerState::Awake};
    bool sleep_entry_pending_{false};
    std::uint32_t sleep_entry_cycles_remaining_{0};
    bool wake_timer_armed_{false};
    // 32 kHz calibration measurement (0x750/0x751): done after 16 32 kHz ticks.
    bool k32_calibration_pending_{false};
    std::uint64_t k32_calibration_target_{0};
    std::vector<std::uint8_t> flash_;
    std::array<std::uint8_t, RegisterSize> registers_{};
    std::array<std::uint8_t, 256> analog_registers_{};
    alignas(4096) std::array<std::uint8_t, 0x8000> sram_{};

    // MSPI flash (registers 0x0c/0x0d), see tlsr8258.cpp for the command set.
    std::uint8_t mspi_command_{0};
    std::uint8_t mspi_response_index_{0};
    bool mspi_active_{false};
    bool mspi_command_seen_{false};
    unsigned mspi_write_count_{0};
    std::uint32_t mspi_address_{0};
    std::uint32_t mspi_read_pos_{0};
    bool mspi_write_enable_{false};
    std::uint32_t flash_dirty_begin_{0};
    std::uint32_t flash_dirty_end_{0};
    void mspi_command_byte(std::uint8_t value);
    void mspi_data_byte(std::uint8_t value);
    std::uint8_t mspi_read_byte();
    void mspi_deselect();
    void flash_program(std::uint32_t address, std::uint8_t value);
    void flash_erase(std::uint32_t address, std::uint32_t size);

    // AES-128 block engine (0x540 control, 0x548 data words, 0x550 key): DISASM
    // hwAes_encrypt_start has the same protocol as the TLSR8258 block.
    std::array<std::uint8_t, 16> aes_key_{};
    std::array<std::uint8_t, 16> aes_input_{};
    std::array<std::uint8_t, 16> aes_output_{};
    unsigned aes_input_words_{0};
    unsigned aes_output_words_{0};
    std::uint64_t aes_cycles_remaining_{0};
    bool aes_decrypt_{false};

    // GPIO
    std::array<std::uint8_t, PortCount> gpio_inputs_{};
    std::array<std::uint8_t, PortCount> gpio_outputs_{};
    std::array<std::uint8_t, PortCount> gpio_irq_active_{};
    GpioPadObserver gpio_pad_observer_;
    bool output_trace_enabled_{false};
    std::vector<OutputEvent> output_events_;
    void update_gpio_irq();

    // Timers 0..2
    std::array<std::uint32_t, 3> timer_last_tick_{};
    std::array<bool, 3> timer_fired_{};
    // Mode 0: the timer stops counting once its tick matches the capture value
    // (8258 datasheet 5.1.2; INFERRED the same on the 826x) until its tick is
    // written or it is disabled.
    std::array<bool, 3> timer_stopped_{};
    std::uint32_t timer_capture(unsigned timer) const noexcept;

    // UART: DMA only (channel 0 receives, channel 1 sends "u32 length + payload").
    std::deque<std::uint8_t> uart_dma_tx_queue_;
    bool uart_dma_tx_active_{false};
    std::vector<std::uint8_t> uart_transmitted_;
    std::uint64_t uart_tx_cycles_remaining_{0};
    bool uart_rx_error_{false};
    std::uint32_t uart_dma_rx_count_{0};
    std::uint64_t uart_dma_rx_idle_cycles_{0};
    bool uart_rx_dma_enabled() const noexcept;
    std::uint64_t uart_byte_cycles() const noexcept;
    void uart_dma_receive(std::uint8_t value);
    void finish_uart_dma_receive();
    void start_uart_dma_transmit();
    void update_uart_status();
    void update_dma_irq() noexcept;

    // RF (Zigbee over VirtualAir)
    VirtualAir *radio_air_{nullptr};
    std::string radio_node_name_;
    std::uint8_t radio_channel_{11};
    std::uint64_t radio_next_poll_ns_{0};
    std::uint64_t radio_last_poll_ns_{0};
    std::uint64_t radio_time_offset_ns_{0};
    std::uint64_t radio_tx_finish_ns_{0};
    bool radio_tx_busy_{false};
    std::uint32_t dma_address(std::uint32_t offset) const noexcept;
    void start_radio_transmit();
    void poll_radio();
    void update_radio_irq();

    std::uint8_t read_register(std::uint32_t offset);
    void write_register(std::uint32_t offset, std::uint8_t value);
    std::uint8_t analog_read(std::uint8_t address) const noexcept;
    void analog_write(std::uint8_t address, std::uint8_t value);
    void update_system_timer_irq(std::uint32_t previous_count) noexcept;
    std::uint32_t reg32(std::uint32_t offset) const noexcept {
        return static_cast<std::uint32_t>(registers_[offset]) |
               (static_cast<std::uint32_t>(registers_[offset + 1]) << 8) |
               (static_cast<std::uint32_t>(registers_[offset + 2]) << 16) |
               (static_cast<std::uint32_t>(registers_[offset + 3]) << 24);
    }
    void notify_gpio_pad(unsigned port) {
        if (gpio_pad_observer_)
            gpio_pad_observer_(port);
    }
};

Tlsr826xModel parse_tlsr826x_model(const std::string &name);
const char *model_name(Tlsr826xModel model) noexcept;

} // namespace mcuemu
