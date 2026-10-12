#pragma once

#include "sim/part.hpp"

#include <array>
#include <deque>
#include <functional>

namespace sim {

// Maxim DS18B20 1-Wire temperature sensor. The slave works purely from
// timestamped edges on the net, so it behaves the same for a bit-banging MCU
// at any quantum: it samples master write slots 30 us after the falling edge
// (using the edge history, not "now"), answers read slots immediately, and
// signals presence 30 us after a reset pulse of at least 480 us.
//
// Not modeled: EEPROM persistence (Copy/Recall are accepted and ignored),
// parasite power, and slot timing violations (a slot is classified by its
// falling edge only).
class Ds18b20 final : public Part {
public:
    static constexpr std::uint64_t ResetMinNs = 480000;
    static constexpr std::uint64_t PresenceDelayNs = 30000;
    static constexpr std::uint64_t PresenceLengthNs = 120000;
    static constexpr std::uint64_t SampleDelayNs = 30000;
    static constexpr std::uint64_t ReadHoldNs = 40000;

    Ds18b20(std::string name, Scheduler &scheduler, Pin &dq,
            std::array<std::uint8_t, 8> rom, double temperature_c,
            unsigned resolution_bits);

    std::string type() const override { return "ds18b20"; }
    void register_endpoints(
        EndpointRegistry &registry,
        const std::function<std::uint64_t()> &now) override;

    void set_temperature_c(double celsius) { temperature_c_ = celsius; }
    double temperature_c() const noexcept { return temperature_c_; }
    const std::array<std::uint8_t, 8> &rom() const noexcept { return rom_; }

    static std::uint8_t crc8(const std::uint8_t *data, std::size_t length);
    // Parses "28-aa-bb-cc-dd-ee-ff" (7 bytes, CRC appended) or 8 bytes with a
    // valid CRC. Throws std::invalid_argument.
    static std::array<std::uint8_t, 8> parse_rom(const std::string &text);
    static std::array<std::uint8_t, 8> default_rom(const std::string &name);

private:
    enum class Mode {
        Idle,       // not addressed; waits for the next reset
        RxByte,     // collecting bits from the master
        TxBytes,    // sending bytes to the master
        Search,     // Search ROM / Alarm Search
        ConvStatus, // answering "busy/done" after Convert T
    };

    void on_change(const NetChange &change);
    void on_fall(std::uint64_t time_ns);
    void on_rise(std::uint64_t time_ns);
    void drive(Drive drive, std::uint64_t time_ns);
    void hold_low(std::uint64_t time_ns);
    bool level_at(std::uint64_t time_ns) const;

    void begin_rx(unsigned bytes, std::function<void()> done);
    void begin_tx(std::vector<std::uint8_t> bytes, std::function<void()> done);
    void rx_bit(bool bit);
    void tx_slot(std::uint64_t time_ns);
    void search_slot(std::uint64_t time_ns);
    void rom_command(std::uint8_t command);
    void function_command(std::uint8_t command);
    std::vector<std::uint8_t> scratchpad() const;
    bool alarm_active() const;
    std::uint64_t conversion_ns() const;

    Scheduler &scheduler_;
    Pin &pin_;
    std::array<std::uint8_t, 8> rom_;
    double temperature_c_;
    std::array<std::uint8_t, 2> temperature_reg_{0x50, 0x05}; // 85 C at power-on
    std::uint8_t th_{0x4b};
    std::uint8_t tl_{0x46};
    std::uint8_t config_;
    std::uint64_t busy_until_ns_{0};

    Mode mode_{Mode::Idle};
    bool in_own_drive_{false};
    std::uint64_t fall_ns_{0};
    // Edges up to this time belong to the presence window (ours and other
    // slaves'), not to the master.
    std::uint64_t ignore_until_ns_{0};
    std::deque<std::pair<std::uint64_t, bool>> edges_; // (time, level high)
    EventId release_event_{0};

    // RxByte state.
    unsigned rx_remaining_{0};
    std::vector<std::uint8_t> rx_bytes_;
    unsigned rx_bit_index_{0};
    std::uint8_t rx_shift_{0};
    std::function<void()> rx_done_;
    // TxBytes state.
    std::vector<std::uint8_t> tx_bytes_;
    std::size_t tx_index_{0};
    unsigned tx_bit_index_{0};
    std::function<void()> tx_done_;
    // Search state: phase 0 = send bit, 1 = send complement, 2 = master's pick.
    unsigned search_bit_{0};
    unsigned search_phase_{0};
};

} // namespace sim
