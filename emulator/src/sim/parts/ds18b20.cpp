#include "sim/parts/ds18b20.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace sim {

namespace {

constexpr std::uint8_t CmdSearchRom = 0xf0;
constexpr std::uint8_t CmdReadRom = 0x33;
constexpr std::uint8_t CmdMatchRom = 0x55;
constexpr std::uint8_t CmdSkipRom = 0xcc;
constexpr std::uint8_t CmdAlarmSearch = 0xec;
constexpr std::uint8_t CmdConvert = 0x44;
constexpr std::uint8_t CmdWriteScratchpad = 0x4e;
constexpr std::uint8_t CmdReadScratchpad = 0xbe;
constexpr std::uint8_t CmdCopyScratchpad = 0x48;
constexpr std::uint8_t CmdRecall = 0xb8;
constexpr std::uint8_t CmdReadPower = 0xb4;

} // namespace

std::uint8_t Ds18b20::crc8(const std::uint8_t *data, std::size_t length) {
    std::uint8_t crc = 0;
    for (std::size_t i = 0; i < length; ++i) {
        std::uint8_t byte = data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const bool mix = ((crc ^ byte) & 1u) != 0;
            crc >>= 1;
            if (mix)
                crc ^= 0x8c;
            byte >>= 1;
        }
    }
    return crc;
}

std::array<std::uint8_t, 8> Ds18b20::parse_rom(const std::string &text) {
    std::vector<std::uint8_t> bytes;
    std::stringstream stream(text);
    std::string token;
    while (std::getline(stream, token, '-')) {
        std::size_t used = 0;
        unsigned long value = 0;
        try {
            value = std::stoul(token, &used, 16);
        } catch (const std::exception &) {
            used = 0;
        }
        if (token.empty() || token.size() > 2 || used != token.size())
            throw std::invalid_argument("rom must look like 28-aa-bb-cc-dd-ee-ff, got '" +
                                        text + "'");
        bytes.push_back(static_cast<std::uint8_t>(value));
    }
    if (bytes.size() != 7 && bytes.size() != 8)
        throw std::invalid_argument("rom needs 7 bytes (CRC is appended) or 8, got " +
                                    std::to_string(bytes.size()));
    if (bytes[0] != 0x28)
        throw std::invalid_argument("DS18B20 family code must be 0x28");
    std::array<std::uint8_t, 8> rom{};
    std::copy(bytes.begin(), bytes.begin() + 7, rom.begin());
    const std::uint8_t crc = crc8(rom.data(), 7);
    if (bytes.size() == 8 && bytes[7] != crc)
        throw std::invalid_argument("rom CRC byte is wrong (expected " +
                                    std::to_string(crc) + ")");
    rom[7] = crc;
    return rom;
}

std::array<std::uint8_t, 8> Ds18b20::default_rom(const std::string &name) {
    // Deterministic per-name serial so two sensors on one bus differ.
    std::uint64_t hash = 1469598103934665603ull;
    for (char c : name)
        hash = (hash ^ static_cast<std::uint8_t>(c)) * 1099511628211ull;
    std::array<std::uint8_t, 8> rom{};
    rom[0] = 0x28;
    for (int i = 0; i < 6; ++i)
        rom[1 + i] = static_cast<std::uint8_t>(hash >> (8 * i));
    rom[7] = crc8(rom.data(), 7);
    return rom;
}

Ds18b20::Ds18b20(std::string name, Scheduler &scheduler, Pin &dq,
                 std::array<std::uint8_t, 8> rom, double temperature_c,
                 unsigned resolution_bits)
    : Part(std::move(name)), scheduler_(scheduler), pin_(dq), rom_(rom),
      temperature_c_(temperature_c),
      config_(static_cast<std::uint8_t>(0x1f | ((resolution_bits - 9) << 5))) {
    if (resolution_bits < 9 || resolution_bits > 12)
        throw std::invalid_argument("resolution must be 9..12 bits");
    edges_.emplace_back(0, pin_.net().level() != Level::Low);
    pin_.net().subscribe([this](const NetChange &c) { on_change(c); });
}

void Ds18b20::register_endpoints(EndpointRegistry &registry,
                                 const std::function<std::uint64_t()> &now) {
    registry.add(
        name() + ".temperature_c",
        std::make_unique<ParamEndpoint>(
            name() + ".temperature_c",
            [this] { return Value{temperature_c_}; },
            [this](const Value &v) { temperature_c_ = value_as_number(v); }, now));
}

bool Ds18b20::level_at(std::uint64_t time_ns) const {
    bool level = true;
    for (const auto &edge : edges_) {
        if (edge.first > time_ns)
            break;
        level = edge.second;
    }
    return level;
}

void Ds18b20::drive(Drive d, std::uint64_t time_ns) {
    // Edges produced by our own drive arrive synchronously; on_change ignores
    // them so a release is never mistaken for the end of a reset pulse.
    in_own_drive_ = true;
    pin_.set_drive(d, time_ns);
    in_own_drive_ = false;
}

void Ds18b20::hold_low(std::uint64_t time_ns) {
    drive(Drive::Low, time_ns);
    release_event_ = scheduler_.at(time_ns + ReadHoldNs, [this] {
        release_event_ = 0;
        drive(Drive::HiZ, scheduler_.now());
    });
}

void Ds18b20::on_change(const NetChange &change) {
    edges_.emplace_back(change.time_ns, change.level != Level::Low);
    while (edges_.size() > 64)
        edges_.pop_front();
    if (in_own_drive_ || change.time_ns <= ignore_until_ns_)
        return;
    if (change.level == Level::Low)
        on_fall(change.time_ns);
    else
        on_rise(change.time_ns);
}

void Ds18b20::on_fall(std::uint64_t time_ns) {
    fall_ns_ = time_ns;
    switch (mode_) {
    case Mode::RxByte: {
        // Master write slot: sample the line 30 us after the falling edge.
        const std::uint64_t sample_at = time_ns + SampleDelayNs;
        scheduler_.at(sample_at, [this, sample_at] {
            if (mode_ == Mode::RxByte)
                rx_bit(level_at(sample_at));
        });
        break;
    }
    case Mode::TxBytes:
        tx_slot(time_ns);
        break;
    case Mode::Search:
        search_slot(time_ns);
        break;
    case Mode::ConvStatus:
        if (time_ns < busy_until_ns_)
            hold_low(time_ns); // busy: the master reads 0
        break;
    case Mode::Idle:
        break;
    }
}

void Ds18b20::on_rise(std::uint64_t time_ns) {
    if (time_ns - fall_ns_ < ResetMinNs)
        return;
    // Reset pulse: abort whatever was in progress and announce presence.
    if (release_event_) {
        scheduler_.cancel(release_event_);
        release_event_ = 0;
    }
    mode_ = Mode::Idle;
    ignore_until_ns_ = time_ns + PresenceDelayNs + PresenceLengthNs;
    scheduler_.at(time_ns + PresenceDelayNs, [this, time_ns] {
        drive(Drive::Low, std::max(scheduler_.now(), time_ns + PresenceDelayNs));
        release_event_ = scheduler_.at(
            time_ns + PresenceDelayNs + PresenceLengthNs, [this] {
                release_event_ = 0;
                drive(Drive::HiZ, scheduler_.now());
                begin_rx(1, [this] { rom_command(rx_bytes_[0]); });
            });
    });
}

void Ds18b20::begin_rx(unsigned bytes, std::function<void()> done) {
    mode_ = Mode::RxByte;
    rx_remaining_ = bytes;
    rx_bytes_.clear();
    rx_bit_index_ = 0;
    rx_shift_ = 0;
    rx_done_ = std::move(done);
}

void Ds18b20::rx_bit(bool bit) {
    rx_shift_ = static_cast<std::uint8_t>(rx_shift_ | ((bit ? 1u : 0u) << rx_bit_index_));
    if (++rx_bit_index_ < 8)
        return;
    rx_bytes_.push_back(rx_shift_);
    rx_shift_ = 0;
    rx_bit_index_ = 0;
    if (--rx_remaining_ == 0) {
        mode_ = Mode::Idle;
        auto done = std::move(rx_done_);
        done();
    }
}

void Ds18b20::begin_tx(std::vector<std::uint8_t> bytes, std::function<void()> done) {
    mode_ = Mode::TxBytes;
    tx_bytes_ = std::move(bytes);
    tx_index_ = 0;
    tx_bit_index_ = 0;
    tx_done_ = std::move(done);
}

void Ds18b20::tx_slot(std::uint64_t time_ns) {
    const bool bit = ((tx_bytes_[tx_index_] >> tx_bit_index_) & 1u) != 0;
    if (!bit)
        hold_low(time_ns);
    if (++tx_bit_index_ == 8) {
        tx_bit_index_ = 0;
        if (++tx_index_ == tx_bytes_.size()) {
            mode_ = Mode::Idle;
            if (tx_done_) {
                auto done = std::move(tx_done_);
                done();
            }
        }
    }
}

void Ds18b20::search_slot(std::uint64_t time_ns) {
    const bool rom_bit = ((rom_[search_bit_ / 8] >> (search_bit_ % 8)) & 1u) != 0;
    if (search_phase_ < 2) {
        // Send the bit, then its complement; a 0 is a held-low slot.
        const bool value = search_phase_ == 0 ? rom_bit : !rom_bit;
        if (!value)
            hold_low(time_ns);
        ++search_phase_;
        return;
    }
    // Master write slot: its choice must match our bit or we drop out.
    const std::uint64_t sample_at = time_ns + SampleDelayNs;
    scheduler_.at(sample_at, [this, sample_at, rom_bit] {
        if (mode_ != Mode::Search)
            return;
        if (level_at(sample_at) != rom_bit) {
            mode_ = Mode::Idle;
            return;
        }
        search_phase_ = 0;
        if (++search_bit_ == 64) {
            mode_ = Mode::Idle;
            begin_rx(1, [this] { function_command(rx_bytes_[0]); });
        }
    });
}

void Ds18b20::rom_command(std::uint8_t command) {
    switch (command) {
    case CmdReadRom:
        begin_tx(std::vector<std::uint8_t>(rom_.begin(), rom_.end()), [this] {
            begin_rx(1, [this] { function_command(rx_bytes_[0]); });
        });
        break;
    case CmdSkipRom:
        begin_rx(1, [this] { function_command(rx_bytes_[0]); });
        break;
    case CmdMatchRom:
        begin_rx(8, [this] {
            if (std::equal(rx_bytes_.begin(), rx_bytes_.end(), rom_.begin()))
                begin_rx(1, [this] { function_command(rx_bytes_[0]); });
            else
                mode_ = Mode::Idle;
        });
        break;
    case CmdSearchRom:
    case CmdAlarmSearch:
        if (command == CmdAlarmSearch && !alarm_active()) {
            mode_ = Mode::Idle;
            break;
        }
        mode_ = Mode::Search;
        search_bit_ = 0;
        search_phase_ = 0;
        break;
    default:
        mode_ = Mode::Idle;
    }
}

std::vector<std::uint8_t> Ds18b20::scratchpad() const {
    std::vector<std::uint8_t> data = {temperature_reg_[0], temperature_reg_[1],
                                      th_, tl_, config_, 0xff, 0x0c, 0x10, 0};
    data[8] = crc8(data.data(), 8);
    return data;
}

bool Ds18b20::alarm_active() const {
    const int whole = static_cast<std::int16_t>(
                          temperature_reg_[0] | (temperature_reg_[1] << 8)) >> 4;
    return whole > static_cast<std::int8_t>(th_) ||
           whole <= static_cast<std::int8_t>(tl_);
}

std::uint64_t Ds18b20::conversion_ns() const {
    switch ((config_ >> 5) & 3u) {
    case 0: return 93750000;
    case 1: return 187500000;
    case 2: return 375000000;
    default: return 750000000;
    }
}

void Ds18b20::function_command(std::uint8_t command) {
    switch (command) {
    case CmdConvert: {
        const double clamped = std::clamp(temperature_c_, -55.0, 125.0);
        auto raw = static_cast<std::int16_t>(std::lround(clamped * 16.0));
        const unsigned resolution = ((config_ >> 5) & 3u) + 9u;
        raw = static_cast<std::int16_t>(raw & ~((1 << (12 - resolution)) - 1));
        temperature_reg_[0] = static_cast<std::uint8_t>(raw & 0xff);
        temperature_reg_[1] = static_cast<std::uint8_t>((raw >> 8) & 0xff);
        busy_until_ns_ = scheduler_.now() + conversion_ns();
        mode_ = Mode::ConvStatus;
        break;
    }
    case CmdReadScratchpad:
        begin_tx(scratchpad(), nullptr);
        break;
    case CmdWriteScratchpad:
        begin_rx(3, [this] {
            th_ = rx_bytes_[0];
            tl_ = rx_bytes_[1];
            config_ = static_cast<std::uint8_t>((rx_bytes_[2] & 0x60) | 0x1f);
            begin_rx(1, [this] { function_command(rx_bytes_[0]); });
        });
        break;
    case CmdCopyScratchpad:
    case CmdRecall:
        mode_ = Mode::Idle; // EEPROM is not modeled; the command is accepted
        break;
    case CmdReadPower:
        begin_tx({0xff}, nullptr); // 1 bit = externally powered
        break;
    default:
        mode_ = Mode::Idle;
    }
}

} // namespace sim
