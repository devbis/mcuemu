#pragma once

#include "tc32emu/bus.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace tc32emu {

class VirtualAir;

enum class Tlsr8258Model {
    F512,
    F1K,
};

enum class Tlsr8258PowerState {
    Awake,
    Suspend,
    DeepSleep,
};

struct OutputEvent {
    enum class Kind {
        GpioPort,
        PwmEnableMask,
    };

    Kind kind;
    unsigned index;
    std::uint8_t value;
    std::uint64_t time_ns;
};

class Tlsr8258 final : public Bus {
public:
    static constexpr std::uint32_t RegisterBase = 0x800000;
    static constexpr std::uint32_t RegisterSize = 0x40000;
    // LoadTblCmdSet in the vendor TLSR8258 firmware builds register writes as
    // 0x1000000 + offset. The datasheet documents only RegisterBase; this
    // compatibility alias is inferred from the firmware's executed code.
    static constexpr std::uint32_t VendorRegisterAliasBase = 0x1000000;
    static constexpr std::uint32_t RetainedSramBase = 0x840000;
    static constexpr std::uint32_t NonRetainedSramBase = 0x848000;
    static constexpr std::uint32_t SramSize = 0x10000;
    static constexpr std::uint32_t GpioBase = 0x800580;
    static constexpr std::uint32_t PwmEnable = 0x800780;
    static constexpr std::uint32_t Pwm0Enable = 0x800781;
    static constexpr std::uint32_t TimerControl = 0x800620;
    static constexpr std::uint32_t TimerCapture0 = 0x800624;
    static constexpr std::uint32_t TimerTick0 = 0x800630;
    static constexpr std::uint32_t DividerControl = 0x800660;
    static constexpr std::uint32_t DividerDividend = 0x800664;
    static constexpr std::uint32_t DividerDivisor = 0x800668;
    static constexpr std::uint32_t RfRxDmaAddress = 0x800c08;
    static constexpr std::uint32_t RfTxDmaAddress = 0x800c0c;
    static constexpr std::uint32_t RfRxDmaAddressHigh = 0x800c42;
    static constexpr std::uint32_t RfTxDmaAddressHigh = 0x800c43;
    static constexpr std::uint32_t RfDmaControl = 0x800428;
    static constexpr std::uint32_t DmaChannelEnable = 0x800c20;
    static constexpr std::uint8_t DmaRfRxEnable = 1u << 2;
    static constexpr std::uint8_t DmaRfTxEnable = 1u << 3;
    static constexpr std::uint32_t DmaTxReady0 = 0x800c24;
    static constexpr std::uint32_t RfMode = 0x800f02;
    static constexpr std::uint32_t RfSynthFrequencyLow = 0x801244;
    static constexpr std::uint32_t RfSynthFrequencyHigh = 0x801245;
    static constexpr std::uint32_t RfIrqMask = 0x800f1c;
    static constexpr std::uint32_t RfIrqStatus = 0x800f20;
    static constexpr std::uint32_t SystemTimer = 0x800740;
    static constexpr std::uint32_t SystemTimerCompare = 0x800744;
    static constexpr std::uint32_t SystemTimerMode = 0x80074c;
    static constexpr std::uint8_t SystemTimerIrqEnable = 1u << 1;
    static constexpr std::uint8_t SystemTimer32kCalibrationEnable = 1u << 7;
    static constexpr std::uint32_t SystemTimerControl = 0x80074f;
    static constexpr std::uint8_t SystemTimerControlStrobeMask = 0x0b;
    static constexpr std::uint32_t IrqMask = 0x800640;
    static constexpr std::uint32_t IrqGlobalEnable = 0x800643;
    static constexpr std::uint32_t IrqSource = 0x800648;
    static constexpr std::uint32_t AnalogAddress = 0x8000b8;
    static constexpr std::uint32_t AnalogData = 0x8000b9;
    static constexpr std::uint32_t AnalogControl = 0x8000ba;
    static constexpr std::uint8_t AnalogClockPower = 0x05;
    static constexpr std::uint8_t AnalogClock32kRcPowerDown = 1u << 0;
    static constexpr std::uint8_t AnalogClock32kXtalPowerDown = 1u << 1;
    static constexpr std::uint8_t AnalogRc32kCalibrationControl = 0xc6;
    static constexpr std::uint8_t AnalogRcCalibrationStatus = 0xcf;
    static constexpr std::uint8_t AnalogRc32kCalibrationReady = 1u << 6;
    static constexpr std::uint32_t Rc32kCalibrationTimeNs = 3000000;
    static constexpr std::uint32_t AesControl = 0x800540;
    static constexpr std::uint32_t AesData = 0x800548;
    static constexpr std::uint32_t AesKey = 0x800550;
    static constexpr std::uint32_t MspiData = 0x80000c;
    static constexpr std::uint32_t MspiControl = 0x80000d;
    static constexpr std::uint32_t Clock32kSelect = 0x800073;
    static constexpr std::uint8_t Clock32kSelectXtal = 1u << 0;
    static constexpr std::uint32_t PowerDownControl = 0x80006f;
    static constexpr std::uint8_t PowerDownSleepEnable = 1u << 0;
    static constexpr std::uint8_t PowerDownSleepTrigger = 1u << 7;
    static constexpr std::uint32_t System32kTickCompare = 0x800754;
    static constexpr std::uint32_t AnalogWakeSource = 0x26;
    static constexpr std::uint8_t AnalogWakeTimerEnable = 1u << 6;
    static constexpr std::uint32_t AnalogWakeStatus = 0x44;
    static constexpr std::uint8_t AnalogWakeTimerStatus = 1u << 1;
    static constexpr std::uint32_t SystemTimerClockHz = 16000000;
    static constexpr std::uint32_t UartData = 0x800090;
    static constexpr std::uint32_t UartClockDivider = 0x800094;
    static constexpr std::uint32_t UartControl0 = 0x800096;
    static constexpr std::uint32_t UartControl1 = 0x800097;
    static constexpr std::uint32_t UartControl2 = 0x800098;
    static constexpr std::uint32_t UartControl3 = 0x800099;
    static constexpr std::uint32_t UartBufferCount = 0x80009c;
    static constexpr std::uint32_t UartStatus0 = 0x80009d;
    static constexpr std::uint32_t UartStatus1 = 0x80009e;
    static constexpr std::uint32_t UartState = 0x80009f;

    explicit Tlsr8258(Tlsr8258Model model = Tlsr8258Model::F512,
                     std::uint32_t clock_hz = 48000000);

    std::uint8_t read8(std::uint32_t address) override;
    void write8(std::uint32_t address, std::uint8_t value) override;
    void tick(std::uint32_t cycles) override;
    bool interrupt_pending() const noexcept override;
    Tlsr8258PowerState power_state() const noexcept { return power_state_; }
    bool advance_sleep_to_wakeup();
    bool consume_deep_sleep_wakeup() noexcept;

    void load_firmware(const std::vector<std::uint8_t> &image,
                       std::uint32_t offset = 0);
    std::uint32_t flash_size() const noexcept;
    std::uint64_t time_ns() const noexcept { return time_ns_; }
    std::uint32_t clock_hz() const noexcept { return clock_hz_; }
    std::uint8_t *sram_data() noexcept { return sram_.data(); }
    void set_gpio_input(unsigned port, std::uint8_t value);
    std::uint8_t gpio_output(unsigned port) const;
    void set_output_trace_enabled(bool enabled) noexcept {
        output_trace_enabled_ = enabled;
    }
    const std::vector<OutputEvent> &output_events() const noexcept {
        return output_events_;
    }
    void connect_radio(VirtualAir &air, std::string node_name);
    void set_radio_time_offset(std::uint64_t offset_ns) noexcept {
        radio_time_offset_ns_ = offset_ns;
    }
    std::uint64_t network_time_ns() const noexcept {
        if (radio_time_offset_ns_ >
            std::numeric_limits<std::uint64_t>::max() - time_ns_)
            return std::numeric_limits<std::uint64_t>::max();
        return time_ns_ + radio_time_offset_ns_;
    }
    void set_radio_channel(std::uint8_t zigbee_channel);
    bool inject_uart_rx(std::uint8_t value);
    std::vector<std::uint8_t> take_uart_transmitted();
    const std::vector<std::uint8_t> &flash() const noexcept { return flash_; }

private:
    std::uint32_t clock_hz_;
    std::uint64_t time_ns_{0};
    std::uint64_t time_fraction_{0};
    std::uint64_t rc32k_calibration_ready_at_ns_{0};
    bool rc32k_calibration_pending_{false};
    std::uint64_t analog_32k_tick_count_{0};
    std::uint64_t analog_32k_fraction_{0};
    std::uint64_t system_timer_calibration_target_tick_{0};
    bool system_timer_calibration_pending_{false};
    Tlsr8258PowerState power_state_{Tlsr8258PowerState::Awake};
    bool deep_sleep_wakeup_pending_{false};
    bool wake_timer_armed_{false};
    std::uint32_t system_timer_count_{0};
    std::uint64_t system_timer_fraction_{0};
    std::uint8_t mspi_command_{0};
    std::uint8_t mspi_response_index_{0};
    bool mspi_active_{false};
    bool mspi_command_seen_{false};
    std::array<std::uint8_t, 16> aes_key_{};
    std::array<std::uint8_t, 16> aes_input_{};
    std::array<std::uint8_t, 16> aes_output_{};
    std::uint32_t aes_input_word_{0};
    unsigned aes_input_word_count_{0};
    unsigned aes_output_word_index_{0};
    std::uint64_t aes_cycles_remaining_{0};
    bool aes_decrypt_{false};
    VirtualAir *radio_air_{nullptr};
    std::string radio_node_name_;
    std::uint8_t radio_channel_{11};
    std::uint64_t radio_next_poll_ns_{0};
    std::uint64_t radio_last_poll_ns_{0};
    std::uint64_t radio_time_offset_ns_{0};
    std::uint64_t radio_tx_finish_ns_{0};
    bool radio_tx_busy_{false};
    std::vector<std::uint8_t> flash_;
    std::array<std::uint8_t, RegisterSize> registers_{};
    std::array<std::uint8_t, 256> analog_registers_{};
    alignas(4096) std::array<std::uint8_t, SramSize> sram_{};
    std::array<std::uint8_t, 5> gpio_inputs_{};
    std::array<std::uint8_t, 5> gpio_outputs_{};
    bool output_trace_enabled_{false};
    std::vector<OutputEvent> output_events_;
    std::array<std::uint32_t, 3> timer_last_tick_{};
    std::array<bool, 3> timer_fired_{};
    std::deque<std::uint8_t> uart_rx_fifo_;
    std::deque<std::uint8_t> uart_tx_fifo_;
    std::vector<std::uint8_t> uart_transmitted_;
    std::uint64_t uart_tx_cycles_remaining_{0};
    bool uart_tx_done_{true};
    bool uart_rx_error_{false};

    std::uint8_t read_register(std::uint32_t offset);
    void write_register(std::uint32_t offset, std::uint8_t value);
    void update_system_timer_irq(std::uint32_t previous_count) noexcept;
    void update_system_timer_calibration() noexcept;
    void update_analog_calibration() noexcept;
    std::uint8_t pwm_enable_mask() const noexcept;
    void record_output_event(OutputEvent::Kind kind, unsigned index,
                             std::uint8_t value);
    std::uint32_t timer_tick(unsigned index) const;
    std::uint32_t analog_32k_tick() const noexcept;
    std::uint32_t system_32k_compare() const noexcept;
    bool selected_32k_clock_running() const noexcept;
    void enter_sleep();
    void reset_volatile_state_after_deep_sleep();
    std::uint8_t analog_register_read(std::uint8_t address) noexcept;
    std::uint32_t dma_address(std::uint32_t low_offset,
                              std::uint32_t high_offset) const noexcept;
    void start_radio_transmit();
    void poll_radio();
    void update_radio_irq();
    std::uint64_t uart_frame_cycles() const;
    void update_uart_status();
    void start_uart_transmit();
};

Tlsr8258Model parse_model(const std::string &name);
const char *model_name(Tlsr8258Model model) noexcept;

} // namespace tc32emu
