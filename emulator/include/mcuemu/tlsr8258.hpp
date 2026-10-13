#pragma once

#include "mcuemu/bus.hpp"

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

struct AnalogRegisterEvent {
    enum class Direction {
        Read,
        Write,
    };

    std::uint64_t time_ns;
    Direction direction;
    std::uint8_t address;
    std::uint8_t value;
};

enum class GpioPull {
    None,
    Up,
    Down,
};

// The pad pair currently used by the I2C master (SDA/SCL), if any.
struct I2cPads {
    int sda_port{-1};
    unsigned sda_bit{0};
    int scl_port{-1};
    unsigned scl_bit{0};
    bool valid() const noexcept { return sda_port >= 0; }
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
    // The TLSR8258 sleep trampoline runs 40 16-bit instructions after the
    // 0x6f write before the instruction-cache +0x58 sleep boundary. The CPU
    // model estimates three core cycles per instruction.
    static constexpr std::uint32_t SleepEntryDelayCycles = 40u * 3u;
    static constexpr std::uint32_t System32kTickCompare = 0x800754;
    static constexpr std::uint32_t AnalogWakeSource = 0x26;
    static constexpr std::uint8_t AnalogWakeTimerEnable = 1u << 6;
    static constexpr std::uint32_t AnalogWakeStatus = 0x44;
    static constexpr std::uint8_t AnalogWakeTimerStatus = 1u << 1;
    static constexpr std::uint32_t SystemTimerClockHz = 16000000;
    static constexpr std::uint32_t DefaultRc32kFrequencyHz = 32000;
    static constexpr std::uint16_t DefaultRc32kCalibration = 8000;
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
    void set_rc32k_characteristics(std::uint32_t frequency_hz,
                                   std::uint16_t system_timer_calibration);

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
    // Pad-level view for external wiring: called whenever the output latch
    // (OUT) or the output-enable (OEN, 0 = driven) of a port changes. The
    // callee reads pad_out()/pad_oen() and drives its nets.
    using GpioPadObserver = std::function<void(unsigned port)>;
    void set_gpio_pad_observer(GpioPadObserver observer) {
        gpio_pad_observer_ = std::move(observer);
    }
    std::uint8_t gpio_oen(unsigned port) const;
    // True while the pad belongs to GPIO (ACT_AS_GPIO = 1); a pad assigned to
    // another function (I2C, UART, PWM, ...) is not driven by OUT/OEN.
    bool gpio_is_gpio(unsigned port, unsigned bit) const;
    // Internal pull resistor selected through the analog registers 0x0e..0x17.
    GpioPull gpio_pull(unsigned port, unsigned bit) const;

    // I2C master (registers 0x00..0x07). The master drives SDA/SCL open-drain:
    // `sda_low`/`scl_low` tell which line it pulls low, otherwise it releases
    // the line. The callee applies this to the nets and feeds the line level
    // back through set_gpio_input() on the SDA pad.
    using I2cPadObserver =
        std::function<void(const I2cPads &pads, bool sda_low, bool scl_low)>;
    void set_i2c_pad_observer(I2cPadObserver observer) {
        i2c_pad_observer_ = std::move(observer);
    }

    // SAR ADC misc channel. The provider returns the voltage (millivolts) of a
    // pad: it is queried for every sample.
    using AdcInputProvider =
        std::function<std::uint32_t(unsigned port, unsigned bit)>;
    void set_adc_input_provider(AdcInputProvider provider) {
        adc_input_provider_ = std::move(provider);
    }
    // Actual 1.2 V reference voltage; the SDK default calibration is 1175 mV.
    void set_adc_reference_mv(std::uint32_t millivolts) noexcept {
        adc_reference_mv_ = millivolts;
    }
    // Supply voltage seen by the ADC's VBAT input (positive selector 0xf).
    void set_supply_mv(std::uint32_t millivolts) noexcept { supply_mv_ = millivolts; }
    std::uint32_t supply_mv() const noexcept { return supply_mv_; }
    void set_output_trace_enabled(bool enabled) noexcept {
        output_trace_enabled_ = enabled;
    }
    const std::vector<OutputEvent> &output_events() const noexcept {
        return output_events_;
    }
    void set_analog_trace_enabled(bool enabled, std::size_t limit);
    const std::vector<AnalogRegisterEvent> &analog_register_events() const noexcept {
        return analog_register_events_;
    }
    std::uint64_t analog_register_event_count() const noexcept {
        return analog_register_event_count_;
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
    // Flash bytes changed through the MSPI program/erase commands since the
    // last call; the CPU wrapper mirrors them into the engine's flash mapping.
    bool take_flash_dirty(std::uint32_t &begin, std::uint32_t &end) noexcept;

private:
    std::uint32_t clock_hz_;
    std::uint32_t rc32k_frequency_hz_{DefaultRc32kFrequencyHz};
    std::uint16_t rc32k_system_timer_calibration_{DefaultRc32kCalibration};
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
    bool sleep_entry_pending_{false};
    std::uint32_t sleep_entry_cycles_remaining_{0};
    Tlsr8258PowerState pending_sleep_state_{Tlsr8258PowerState::Awake};
    std::uint32_t system_timer_count_{0};
    std::uint64_t system_timer_fraction_{0};
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
    GpioPadObserver gpio_pad_observer_;
    I2cPadObserver i2c_pad_observer_;
    AdcInputProvider adc_input_provider_;
    std::uint32_t adc_reference_mv_{1175};
    std::uint32_t supply_mv_{3300};
    std::uint64_t adc_next_sample_ns_{0};
    std::uint16_t dfifo2_write_pointer_{0};

    struct I2cStep {
        enum class Kind {
            Drive,     // set the line states
            SampleBit, // shift SDA into the read byte
            SampleAck, // slave ACK of a written byte
            StoreRead, // read byte complete: copy into the data register
            End,       // command finished
        };
        std::uint64_t cycle;
        Kind kind;
        bool sda_low;
        bool scl_low;
    };
    std::deque<I2cStep> i2c_steps_;
    std::uint64_t i2c_cycles_{0};
    bool i2c_sda_low_{false};
    bool i2c_scl_low_{false};
    bool i2c_nak_{false};
    bool i2c_bus_busy_{false};
    bool i2c_cmd_stop_{false};
    I2cPads i2c_pads_;
    std::uint8_t i2c_read_shift_{0};
    bool output_trace_enabled_{false};
    std::vector<OutputEvent> output_events_;
    bool analog_trace_enabled_{false};
    std::size_t analog_trace_limit_{0};
    std::uint64_t analog_register_event_count_{0};
    std::vector<AnalogRegisterEvent> analog_register_events_;
    std::array<std::uint32_t, 3> timer_last_tick_{};
    std::array<bool, 3> timer_fired_{};
    std::deque<std::uint8_t> uart_rx_fifo_;
    std::deque<std::uint8_t> uart_tx_fifo_;
    std::vector<std::uint8_t> uart_transmitted_;
    std::uint64_t uart_tx_cycles_remaining_{0};
    bool uart_tx_done_{true};
    bool uart_rx_error_{false};
    // UART DMA (SDK drv_uart on the 8258): channel 0 receives into a buffer as
    // "u32 length + payload", closed by an idle timeout; channel 1 sends a
    // "u32 length + payload" buffer.
    std::deque<std::uint8_t> uart_dma_tx_queue_;
    bool uart_dma_tx_active_{false};
    std::uint32_t uart_dma_rx_count_{0};
    std::uint64_t uart_dma_rx_idle_cycles_{0};
    bool uart_rx_dma_enabled() const noexcept;
    void uart_dma_receive(std::uint8_t value);
    void finish_uart_dma_receive();
    void start_uart_dma_transmit();
    void refill_uart_tx_from_dma();
    void update_dma_irq() noexcept;

    std::uint8_t read_register(std::uint32_t offset);
    void write_register(std::uint32_t offset, std::uint8_t value);
    void update_system_timer_irq(std::uint32_t previous_count) noexcept;
    void update_system_timer_calibration() noexcept;
    void update_analog_calibration() noexcept;
    std::uint8_t pwm_enable_mask() const noexcept;
    void record_output_event(OutputEvent::Kind kind, unsigned index,
                             std::uint8_t value);
    void record_analog_register_event(AnalogRegisterEvent::Direction direction,
                                      std::uint8_t address,
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
    void notify_gpio_pad(unsigned port) {
        if (gpio_pad_observer_)
            gpio_pad_observer_(port);
    }
    // tlsr8258_i2c.cpp
    void i2c_start_command(std::uint8_t command);
    void i2c_advance(std::uint32_t cycles);
    void i2c_apply(bool sda_low, bool scl_low);
    I2cPads i2c_select_pads() const;
    std::uint8_t i2c_status() const;
    // tlsr8258_adc.cpp
    void adc_advance();
    std::uint16_t adc_sample() const;
    std::uint32_t adc_channel_mv(unsigned selector) const;
};

Tlsr8258Model parse_model(const std::string &name);
const char *model_name(Tlsr8258Model model) noexcept;

} // namespace mcuemu
