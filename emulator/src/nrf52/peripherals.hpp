#pragma once

#include "nrf52/peripheral.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "mcuemu/radio.hpp"

namespace nrf52emu {

constexpr std::uint32_t peripheral_address(unsigned id) {
    return Nrf52840::PeripheralBase + id * 0x1000u;
}

// POWER and CLOCK share one instance (id 0) and one interrupt.
class PowerClock final : public Peripheral {
public:
    static constexpr int Id = 0;
    explicit PowerClock(Nrf52840 &chip) : Peripheral(chip, peripheral_address(Id), Id) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;
    // RESETREAS survives a reset; the chip adds the cause before restarting.
    void add_reset_reason(std::uint32_t bits) { reset_reason_ |= bits; }

private:
    enum Bit : std::uint32_t {
        HfclkStarted = 1u << 0,
        LfclkStarted = 1u << 1,
        PofWarn = 1u << 2,
        Done = 1u << 3,
        Ctto = 1u << 4,
        SleepEnter = 1u << 5,
        SleepExit = 1u << 6,
        UsbDetected = 1u << 7,
        UsbRemoved = 1u << 8,
        UsbPwrRdy = 1u << 9,
        CtStarted = 1u << 10,
        CtStopped = 1u << 11,
    };
    void raise(std::uint32_t bit);
    void update_irq();
    std::uint32_t event_bit(std::uint32_t offset) const;

    std::uint32_t events_{0};
    std::uint32_t inten_{0};
    std::uint32_t reset_reason_{0};
    std::uint32_t gpregret_[2]{0, 0};
    bool hf_requested_{false}, hf_running_{false}, hf_xtal_{false};
    bool lf_requested_{false}, lf_running_{false};
    std::uint32_t lf_source_{0}, lf_source_running_{0};
    std::uint32_t hf_debounce_{0x10};
    std::uint32_t ctiv_{0};
    std::uint32_t dcdcen_{0}, dcdcen0_{0}, pofcon_{0};
    std::uint64_t hf_ready_at_{Never}, lf_ready_at_{Never}, cal_done_at_{Never};
    std::uint64_t ct_next_{Never};
    bool ct_running_{false};
};

// NVMC: flash and UICR programming. Operations complete instantly (READY stays
// 1); the real part takes up to 85 ms per page erase. Inferred: a write while
// CONFIG is neither WEN nor EEN is ignored without a fault.
class Nvmc final : public Peripheral {
public:
    static constexpr int Id = 30;
    explicit Nvmc(Nrf52840 &chip) : Peripheral(chip, peripheral_address(Id), Id) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void reset() override { config_ = 0; }
    // CPU store to non-writable memory (flash, UICR, FICR).
    bool on_store(std::uint32_t address, unsigned size, std::uint32_t value);

private:
    std::uint8_t *target(std::uint32_t address, unsigned size);
    std::uint32_t config_{0};
    std::uint32_t icache_{0};
};

// RTC0..2: 24-bit counter on the 32.768 kHz clock with a 12-bit prescaler.
class Rtc final : public Peripheral {
public:
    Rtc(Nrf52840 &chip, unsigned id, unsigned compare_channels)
        : Peripheral(chip, peripheral_address(id), static_cast<int>(id)), channels_(compare_channels) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    std::uint32_t counter_at(std::uint64_t cycle) const;
    std::uint64_t cycles_for_ticks(std::uint64_t ticks) const;
    void sync(std::uint64_t cycle);
    void rebase(std::uint64_t cycle, std::uint32_t counter);
    void raise(unsigned event_bit); // 0 tick, 1 overflow, 16.. compare
    void update_irq();
    std::uint64_t next_event_cycle() const;
    bool event_enabled(unsigned bit) const { return ((evten_ | inten_) >> bit) & 1u; }

    unsigned channels_;
    bool running_{false};
    std::uint32_t prescaler_{0};
    std::uint32_t cc_[4]{};
    std::uint32_t evten_{0}, inten_{0};
    std::uint32_t events_{0};
    // counter(cycle) = base_counter_ + ticks elapsed since base_cycle_
    std::uint64_t base_cycle_{0};
    std::uint32_t base_counter_{0};
    std::uint32_t synced_counter_{0};
    std::uint64_t synced_ticks_{0};
};

// TIMER0..4: 16 MHz base clock, programmable prescaler and bit width.
class Timer final : public Peripheral {
public:
    Timer(Nrf52840 &chip, unsigned id, unsigned channels)
        : Peripheral(chip, peripheral_address(id), static_cast<int>(id)), channels_(channels) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    std::uint64_t cycles_per_tick() const;
    std::uint32_t mask() const;
    std::uint32_t counter_at(std::uint64_t cycle) const;
    void freeze(std::uint64_t cycle);
    void restart_from(std::uint64_t cycle, std::uint32_t counter);
    void compare_hit(unsigned channel, std::uint64_t cycle);
    void update_irq();
    std::uint64_t next_event_cycle() const;

    unsigned channels_;
    bool running_{false};
    std::uint32_t mode_{0}, bitmode_{0}, prescaler_{4};
    std::uint32_t cc_[6]{};
    std::uint32_t shorts_{0}, inten_{0};
    std::uint32_t events_{0};
    std::uint64_t base_cycle_{0};
    std::uint32_t base_counter_{0};
    std::uint32_t last_counter_{0};
    std::uint64_t synced_ticks_{0}; // ticks since base_cycle_ already processed
};

class Rng final : public Peripheral {
public:
    static constexpr int Id = 13;
    explicit Rng(Nrf52840 &chip) : Peripheral(chip, peripheral_address(Id), Id) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;
    void seed(std::uint64_t value) { state_ = value ? value : 1; }

private:
    void update_irq();
    std::uint64_t next_random();

    bool running_{false};
    bool valrdy_{false};
    std::uint32_t shorts_{0}, inten_{0}, config_{0}, value_{0};
    std::uint64_t ready_at_{Never};
    std::uint64_t state_{0x9e3779b97f4a7c15ull};
};

class Temp final : public Peripheral {
public:
    static constexpr int Id = 12;
    explicit Temp(Nrf52840 &chip) : Peripheral(chip, peripheral_address(Id), Id) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;
    // Die temperature in units of 0.25 degrees Celsius.
    void set_quarter_degrees(std::int32_t value) { quarter_degrees_ = value; }

private:
    bool ready_{false};
    std::uint32_t inten_{0};
    std::int32_t quarter_degrees_{88};
    std::uint64_t done_at_{Never};
};

class Wdt final : public Peripheral {
public:
    static constexpr int Id = 16;
    explicit Wdt(Nrf52840 &chip) : Peripheral(chip, peripheral_address(Id), Id) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    void arm(std::uint64_t cycle);
    void update_irq();

    bool running_{false};
    bool timeout_{false};
    std::uint32_t inten_{0}, crv_{0xffffffffu}, rren_{1}, config_{1}, reload_pending_{0};
    std::uint64_t deadline_{Never}, reset_at_{Never};
};

// EGU0..5 (also the SWI interrupts): 16 software-triggered events.
class Egu final : public Peripheral {
public:
    Egu(Nrf52840 &chip, unsigned id) : Peripheral(chip, peripheral_address(id), static_cast<int>(id)) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    void reset() override;

private:
    void update_irq();
    std::uint32_t events_{0}, inten_{0};
};

// PPI: 20 programmable channels and 12 fixed ones, 6 channel groups.
class Ppi final : public Peripheral {
public:
    static constexpr int Id = 31;
    explicit Ppi(Nrf52840 &chip) : Peripheral(chip, peripheral_address(Id), -1) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void reset() override;
    // PPI tasks (TASKS_CHG[n].EN/DIS) can be the target of a channel or its fork.
    void task(std::uint32_t offset) override;
    // An event register at `address` was raised: trigger the connected tasks.
    void route(std::uint32_t address);

private:
    std::uint32_t eep(unsigned channel) const;
    std::uint32_t tep(unsigned channel) const;
    void set_group(unsigned group, bool enable);

    std::uint32_t chen_{0};
    std::uint32_t eep_[20]{}, tep_[20]{};
    std::uint32_t fork_[32]{};
    std::uint32_t chg_[6]{};
    int depth_{0};
};


// SAADC: eight channels, EasyDMA results. Inferred: a conversion takes the
// acquisition time plus 2 us; the ADC reference is exactly 0.6 V; calibration
// completes after 100 us and changes nothing.
class Saadc final : public Peripheral {
public:
    static constexpr int Id = 7;
    explicit Saadc(Nrf52840 &chip) : Peripheral(chip, peripheral_address(Id), Id) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    void raise(unsigned event_index);
    void update_irq();
    std::int16_t convert(unsigned channel);
    std::uint32_t input_mv(std::uint32_t psel) const;
    unsigned enabled_channels() const;
    void start_sample(std::uint64_t t);
    void finish_sample(std::uint64_t t);

    struct Channel {
        std::uint32_t pselp{0}, pseln{0}, config{0x00020000}, limit{0x7fff8000};
    };
    std::uint32_t events_{0}, inten_{0}, enable_{0};
    std::array<Channel, 8> channels_{};
    std::uint32_t resolution_{1}, oversample_{0}, samplerate_{0};
    std::uint32_t ptr_{0}, maxcnt_{0}, amount_{0};
    bool started_{false}, busy_{false}, running_{false}, stopped_pending_{false};
    unsigned next_channel_{0}, sample_total_{0};
    std::uint64_t done_at_{Never}, calibrate_at_{Never}, next_sample_at_{Never};
    std::uint32_t result_index_{0};
};


// ECB: AES-128 encryption of one block in RAM (key, cleartext, ciphertext, each
// 16 bytes, in FIPS-197 byte order). Inferred: a block takes 7 us.
class Ecb final : public Peripheral {
public:
    static constexpr int Id = 14;
    explicit Ecb(Nrf52840 &chip) : Peripheral(chip, peripheral_address(Id), Id) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    void update_irq();
    bool end_{false}, error_{false};
    std::uint32_t inten_{0}, ptr_{0};
    std::uint64_t done_at_{Never};
};


// UARTE0/1 with EasyDMA. Transmit takes ten bit times per byte; receive has no
// bit timing (the sender's byte rate spaces the bytes, as in the other models)
// and a four-byte FIFO in front of the DMA. The TXD pad idles high while the
// block is enabled; the bit stream itself is not put on the pad.
class Uarte final : public Peripheral {
public:
    Uarte(Nrf52840 &chip, unsigned id, unsigned index)
        : Peripheral(chip, peripheral_address(id), static_cast<int>(id)), index_(index) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;
    bool can_receive() const { return enable_ == 8 && rx_fifo_count_ < RxFifo; }
    bool receive(std::uint8_t byte);
    std::vector<std::uint8_t> take_tx() {
        std::vector<std::uint8_t> bytes;
        bytes.swap(tx_bytes_);
        return bytes;
    }

private:
    static constexpr unsigned RxFifo = 4;
    enum Event : unsigned {
        Cts = 0, Ncts = 1, RxDrdy = 2, EndRx = 4, TxDrdy = 7, EndTx = 8, Error = 9, RxTo = 17,
        RxStarted = 19, TxStarted = 20, TxStopped = 22,
    };
    void raise(unsigned bit);
    void update_irq();
    void apply_pads();
    std::uint64_t byte_cycles() const;
    void deliver_rx();
    static std::uint32_t event_register(unsigned bit);

    unsigned index_;
    std::uint32_t events_{0}, inten_{0}, shorts_{0}, enable_{0}, baud_{0x04000000};
    std::uint32_t psel_[4]{~0u, ~0u, ~0u, ~0u}; // RTS, TXD, CTS, RXD
    std::uint32_t rx_ptr_{0}, rx_max_{0}, rx_amount_{0};
    std::uint32_t tx_ptr_{0}, tx_max_{0}, tx_amount_{0};
    std::uint32_t config_{0}, errorsrc_{0};
    bool rx_active_{false}, tx_active_{false};
    std::uint64_t tx_next_{Never};
    std::array<std::uint8_t, RxFifo> rx_fifo_{};
    unsigned rx_fifo_count_{0};
    std::vector<std::uint8_t> tx_bytes_;
};

// TWIM0/1: I2C master with EasyDMA, driving the SCL and SDA pads bit by bit so
// slave models on the nets see a real waveform. Clock stretching is not modelled.
class Twim final : public Peripheral {
public:
    Twim(Nrf52840 &chip, unsigned id) : Peripheral(chip, peripheral_address(id), static_cast<int>(id)) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    enum Event : unsigned {
        Stopped = 1, Error = 9, Suspended = 18, RxStarted = 19, TxStarted = 20, LastRx = 23, LastTx = 24,
    };
    struct Step {
        std::uint64_t at;
        enum Kind { Drive, SampleAck, SampleBit, Raise, Finish } kind;
        bool sda_low;
        bool scl_low;
        unsigned arg;
    };
    void raise(unsigned bit);
    void update_irq();
    static std::uint32_t event_register(unsigned bit);
    void start_segment(bool write, std::uint64_t t);
    void abort_segment(std::uint64_t t, unsigned error_bit);
    void push_stop(std::uint64_t at);
    void apply_lines(bool sda_low, bool scl_low);
    void release_lines();
    void hand_back();
    std::uint64_t quarter_cycles() const;
    bool connected() const { return psel_[0] != ~0u && psel_[1] != ~0u && enable_ == 6; }
    std::uint64_t next_step() const { return steps_.empty() ? Never : steps_.front().at; }

    std::uint32_t events_{0}, inten_{0}, shorts_{0}, enable_{0}, frequency_{0x04000000}, errorsrc_{0};
    std::uint32_t psel_[2]{~0u, ~0u}; // SCL, SDA
    std::uint32_t rx_ptr_{0}, rx_max_{0}, rx_amount_{0}, tx_ptr_{0}, tx_max_{0}, tx_amount_{0};
    std::uint32_t address_{0};
    std::vector<Step> steps_;
    std::size_t step_index_{0};
    bool bus_started_{false};  // a START was issued and no STOP yet (repeated start next)
    bool sda_low_{false}, scl_low_{false};
    bool lines_driven_{false};
    unsigned read_bits_{0};
    std::uint8_t read_byte_{0};
    std::uint32_t read_count_{0};
    std::uint32_t pending_tx_ack_{0};
    bool segment_is_write_{false};
    bool suspended_{false};
};


// RADIO in IEEE 802.15.4 mode (MODE = 15), bridged to the shared `VirtualAir`.
//
// Evidence: nRF52840 Product Specification, chapter "RADIO" (task/event names,
// SHORTS bits, INTEN positions, state values) and the packet layout the Nordic
// 802.15.4 drivers use: PACKETPTR holds the length byte (PHR, counting the two
// FCS bytes), the MAC frame, and on reception the FCS and then one energy
// detect byte. Inferred, not documented: ramp-up 40 us (fast, MODECNF0.RU) or
// 130 us, disable 6 us; the receiver polls the medium every 16 us; CCA and ED
// treat any overlapping frame whose energy reaches the threshold as busy; the
// ED byte is clamp(RSSI + 92, 0, 63); the FCS written on reception is the real
// CRC-16 of the frame. Other modes (BLE) only run the state machine: no frame
// reaches the medium.
class Radio final : public Peripheral {
public:
    static constexpr int Id = 1;
    explicit Radio(Nrf52840 &chip) : Peripheral(chip, peripheral_address(Id), Id) {}
    std::uint32_t read(std::uint32_t offset) override;
    void write(std::uint32_t offset, std::uint32_t value) override;
    void task(std::uint32_t offset) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;
    void connect(mcuemu::VirtualAir &air, std::string node) {
        air_ = &air;
        node_ = std::move(node);
    }

private:
    enum class State : std::uint32_t {
        Disabled = 0, RxRu = 1, RxIdle = 2, Rx = 3, RxDisable = 4,
        TxRu = 9, TxIdle = 10, Tx = 11, TxDisable = 12,
    };
    void raise(unsigned event_index);
    void update_irq();
    void apply_shorts(unsigned event_index);
    void enable(bool transmit);
    void disable();
    void start();
    void begin_transmit(std::uint64_t ns);
    void finish_transmit();
    void poll_air(std::uint64_t ns);
    void deliver(const mcuemu::RadioFrame &frame, std::uint64_t ns);
    void start_cca();
    void finish_cca(std::uint64_t ns);
    void start_ed();
    std::uint8_t energy_at(std::uint64_t from_ns, std::uint64_t to_ns, bool strongest_only) const;
    std::uint64_t ramp_cycles() const;
    std::uint8_t channel() const;
    std::uint64_t to_ns(std::uint64_t cycles) const;
    std::uint64_t reschedule();

    mcuemu::VirtualAir *air_{nullptr};
    std::string node_;
    State state_{State::Disabled};
    std::uint32_t events_{0}, inten_{0}, shorts_{0};
    std::uint32_t packetptr_{0}, frequency_{0}, txpower_{0}, mode_{0}, pcnf0_{0}, pcnf1_{0};
    std::uint32_t crccnf_{0}, crcpoly_{0}, crcinit_{0}, modecnf0_{0}, sfd_{0xa7};
    std::uint32_t edcnt_{0}, edsample_{0}, ccactrl_{0}, crcstatus_{0}, rxcrc_{0}, rssisample_{0};
    std::uint32_t tifs_{0}, bcc_{0}, power_{1};
    std::uint64_t phase_end_{Never};       // ramp-up or ramp-down completes (cycles)
    std::uint64_t tx_end_ns_{0};
    bool tx_active_{false};
    std::uint64_t rx_start_ns_{0}, last_poll_ns_{0};
    std::uint64_t rx_lock_id_{0}, rx_lock_end_ns_{0}; // frame the receiver synced to before it ended
    std::uint64_t cca_end_{Never}, ed_end_{Never}, rssi_end_{Never};
    std::uint64_t cca_start_ns_{0};
    bool cca_active_{false}, ed_active_{false};
};

} // namespace nrf52emu
