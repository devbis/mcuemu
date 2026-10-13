// nRF52840 TWIM0/1: I2C master with EasyDMA.
//
// Evidence: nRF52840 Product Specification, chapter "TWIM" (task, event and
// SHORTS names, ERRORSRC bits, FREQUENCY values). The master puts a real
// START / address / data / ACK / STOP waveform on the SCL and SDA pads in
// quarter-bit steps, so slave models on the nets (I2C parts) see it and answer.
// Inferred, not documented: after an address or data NACK the master sets
// ERRORSRC, raises ERROR, issues a STOP and raises STOPPED; clock stretching
// is not modelled; an unconnected bus reads high (an external pull-up is
// assumed), so a transfer to an absent slave ends in ANACK.

#include "nrf52/gpio.hpp"
#include "nrf52/peripherals.hpp"

#include <algorithm>

namespace nrf52emu {

namespace {
constexpr unsigned NoPad = ~0u;
unsigned decode_pad(std::uint32_t psel) {
    if (psel == ~0u || (psel & 0x80000000u))
        return NoPad;
    const unsigned pad = ((psel >> 5) & 1u) * 32 + (psel & 31u);
    return pad < Nrf52840::PadCount ? pad : NoPad;
}
constexpr std::uint32_t ShortLastTxStartRx = 1u << 7, ShortLastTxSuspend = 1u << 8,
                        ShortLastTxStop = 1u << 9, ShortLastRxStartTx = 1u << 10,
                        ShortLastRxStop = 1u << 12;
} // namespace

std::uint32_t Twim::event_register(unsigned bit) {
    switch (bit) {
    case Stopped: return 0x104;
    case Error: return 0x124;
    case Suspended: return 0x148;
    case RxStarted: return 0x150;
    case TxStarted: return 0x154;
    case LastRx: return 0x15c;
    case LastTx: return 0x160;
    default: return 0;
    }
}

void Twim::reset() {
    hand_back();
    events_ = inten_ = shorts_ = enable_ = errorsrc_ = 0;
    frequency_ = 0x04000000;
    psel_[0] = psel_[1] = ~0u;
    rx_ptr_ = rx_max_ = rx_amount_ = tx_ptr_ = tx_max_ = tx_amount_ = address_ = 0;
    steps_.clear();
    step_index_ = 0;
    bus_started_ = false;
    suspended_ = false;
    update_irq();
}

void Twim::update_irq() { set_irq_level((events_ & inten_) != 0); }

void Twim::raise(unsigned bit) {
    events_ |= 1u << bit;
    event_raised(event_register(bit));
    update_irq();
}

std::uint64_t Twim::quarter_cycles() const {
    std::uint64_t hz = 100000;
    if (frequency_ >= 0x10000000u)
        hz = 1000000;
    else if (frequency_ >= 0x06400000u)
        hz = 400000;
    else if (frequency_ >= 0x04000000u)
        hz = 250000;
    return std::max<std::uint64_t>(clock_hz() / hz / 4, 1);
}

void Twim::apply_lines(bool sda_low, bool scl_low) {
    Gpio &gpio = chip_.gpio();
    const unsigned scl = decode_pad(psel_[0]), sda = decode_pad(psel_[1]);
    auto set = [&](unsigned pad, bool low) {
        if (pad != NoPad)
            gpio.set_override(pad, Gpio::Override{!low, true});
    };
    // Never move SDA while SCL is high: lower SCL first, settle SDA before
    // raising SCL.
    const bool scl_falls = scl_low && !scl_low_;
    if (scl_falls) {
        set(scl, scl_low);
        set(sda, sda_low);
    } else {
        set(sda, sda_low);
        set(scl, scl_low);
    }
    sda_low_ = sda_low;
    scl_low_ = scl_low;
    lines_driven_ = true;
}

// Gives the pads back to the GPIO registers.
void Twim::hand_back() {
    for (std::uint32_t psel : psel_)
        if (const unsigned pad = decode_pad(psel); pad != NoPad)
            chip_.gpio().set_override(pad, std::nullopt);
    lines_driven_ = false;
    sda_low_ = scl_low_ = false;
}

// While the block is enabled it owns the pads, whatever the GPIO registers say
// (the SDK leaves OUT at 0): an idle bus is both lines released (open-drain
// high). Disabling hands them back.
void Twim::release_lines() {
    Gpio &gpio = chip_.gpio();
    const bool owned = connected();
    for (std::uint32_t psel : psel_)
        if (const unsigned pad = decode_pad(psel); pad != NoPad)
            gpio.set_override(pad, owned ? std::optional<Gpio::Override>(Gpio::Override{true, true})
                                         : std::nullopt);
    lines_driven_ = owned;
    sda_low_ = scl_low_ = false;
}

void Twim::push_stop(std::uint64_t at) {
    const std::uint64_t q = quarter_cycles();
    steps_.push_back({at, Step::Drive, true, true, 0});
    steps_.push_back({at + q, Step::Drive, true, false, 0});
    steps_.push_back({at + 2 * q, Step::Drive, false, false, 0});
    steps_.push_back({at + 3 * q, Step::Raise, false, false, Stopped});
    steps_.push_back({at + 3 * q, Step::Finish, false, false, 0});
}

void Twim::start_segment(bool write, std::uint64_t t) {
    const std::uint64_t q = quarter_cycles();
    std::uint64_t at = t;
    steps_.clear();
    step_index_ = 0;
    segment_is_write_ = write;
    read_count_ = 0;
    auto drive = [&](std::uint64_t cycle, bool sda_low, bool scl_low) {
        steps_.push_back({cycle, Step::Drive, sda_low, scl_low, 0});
    };
    if (!bus_started_) {
        drive(at, true, false);
        drive(at + q, true, true);
        at += 2 * q;
    } else { // repeated START: SCL is low, SDA released, then the usual START
        drive(at, false, true);
        drive(at + q, false, false);
        drive(at + 2 * q, true, false);
        drive(at + 3 * q, true, true);
        at += 4 * q;
    }
    bus_started_ = true;
    auto write_byte = [&](std::uint8_t value, unsigned ack_arg) {
        for (int bit = 7; bit >= 0; --bit) {
            const bool low = ((value >> bit) & 1u) == 0;
            drive(at, low, true);
            drive(at + q, low, false);
            drive(at + 3 * q, low, true);
            at += 4 * q;
        }
        drive(at, false, true); // release SDA for the slave's ACK
        drive(at + q, false, false);
        steps_.push_back({at + 2 * q, Step::SampleAck, false, false, ack_arg});
        drive(at + 3 * q, false, true);
        at += 4 * q;
    };
    write_byte(static_cast<std::uint8_t>((address_ << 1) | (write ? 0u : 1u)), 1);
    if (write) {
        steps_.push_back({at, Step::Raise, false, false, TxStarted});
        for (std::uint32_t i = 0; i != tx_max_; ++i) {
            const std::uint32_t address = tx_ptr_ + i;
            std::uint8_t byte = 0;
            if (address >= Nrf52840::RamBase && address < Nrf52840::RamBase + Nrf52840::RamSize)
                byte = chip_.ram()[address - Nrf52840::RamBase];
            else if (address < Nrf52840::FlashSize)
                byte = chip_.flash()[address];
            write_byte(byte, 2);
        }
        tx_amount_ = tx_max_;
        steps_.push_back({at, Step::Raise, false, false, LastTx});
    } else {
        steps_.push_back({at, Step::Raise, false, false, RxStarted});
        for (std::uint32_t i = 0; i != rx_max_; ++i) {
            for (int bit = 7; bit >= 0; --bit) {
                drive(at, false, true);
                drive(at + q, false, false);
                steps_.push_back({at + 2 * q, Step::SampleBit, false, false, static_cast<unsigned>(bit)});
                drive(at + 3 * q, false, true);
                at += 4 * q;
            }
            const bool nack = (i + 1 == rx_max_);
            drive(at, !nack, true); // ACK (SDA low) for all but the last byte
            drive(at + q, !nack, false);
            drive(at + 3 * q, !nack, true);
            at += 4 * q;
            drive(at, false, true); // release SDA again
        }
        steps_.push_back({at, Step::Raise, false, false, LastRx});
    }
    request_service(steps_.front().at);
}

void Twim::abort_segment(std::uint64_t t, unsigned error_bit) {
    errorsrc_ |= error_bit;
    steps_.clear();
    step_index_ = 0;
    raise(Error);
    push_stop(t + quarter_cycles());
}

std::uint64_t Twim::service(std::uint64_t t) {
    Gpio &gpio = chip_.gpio();
    while (step_index_ < steps_.size() && steps_[step_index_].at <= t) {
        const Step step = steps_[step_index_++];
        switch (step.kind) {
        case Step::Drive:
            apply_lines(step.sda_low, step.scl_low);
            break;
        case Step::SampleAck: {
            const unsigned sda = decode_pad(psel_[1]);
            const bool ack = sda != NoPad && !gpio.level(sda);
            if (!ack) {
                abort_segment(step.at, step.arg == 1 ? 2u : 4u); // ANACK : DNACK
                step_index_ = 0;
            }
            break;
        }
        case Step::SampleBit: {
            const unsigned sda = decode_pad(psel_[1]);
            const bool bit = sda != NoPad && gpio.level(sda);
            read_byte_ = static_cast<std::uint8_t>((read_byte_ << 1) | (bit ? 1u : 0u));
            if (step.arg == 0) {
                const std::uint32_t address = rx_ptr_ + read_count_;
                if (address >= Nrf52840::RamBase && address < Nrf52840::RamBase + Nrf52840::RamSize)
                    chip_.ram()[address - Nrf52840::RamBase] = read_byte_;
                ++read_count_;
                rx_amount_ = read_count_;
            }
            break;
        }
        case Step::Raise: {
            raise(step.arg);
            if (step.arg == LastTx) {
                if (shorts_ & ShortLastTxStop) {
                    push_stop(step.at);
                } else if (shorts_ & ShortLastTxStartRx) {
                    start_segment(false, step.at);
                } else if (shorts_ & ShortLastTxSuspend) {
                    suspended_ = true;
                    raise(Suspended);
                }
            } else if (step.arg == LastRx) {
                if (shorts_ & ShortLastRxStop)
                    push_stop(step.at);
                else if (shorts_ & ShortLastRxStartTx)
                    start_segment(true, step.at);
            }
            break;
        }
        case Step::Finish:
            bus_started_ = false;
            release_lines();
            break;
        }
    }
    if (step_index_ >= steps_.size()) {
        steps_.clear();
        step_index_ = 0;
        return Never;
    }
    return steps_[step_index_].at;
}

void Twim::task(std::uint32_t offset) {
    if (!connected())
        return;
    const bool idle = steps_.empty();
    switch (offset) {
    case 0x008: // STARTTX
        if (idle)
            start_segment(true, now());
        break;
    case 0x000: // STARTRX
        if (idle)
            start_segment(false, now());
        break;
    case 0x014: // STOP
        if (idle && bus_started_) {
            push_stop(now());
            request_service(steps_.front().at);
        } else if (!idle) {
            steps_.clear();
            step_index_ = 0;
            push_stop(now());
            request_service(steps_.front().at);
        } else {
            raise(Stopped);
        }
        break;
    case 0x01c: // SUSPEND
        break;
    case 0x020: // RESUME
        suspended_ = false;
        break;
    default: break;
    }
}

std::uint32_t Twim::read(std::uint32_t offset) {
    for (unsigned bit : {1u, 9u, 18u, 19u, 20u, 23u, 24u})
        if (event_register(bit) == offset)
            return (events_ >> bit) & 1u;
    switch (offset) {
    case 0x200: return shorts_;
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x4c4: return errorsrc_;
    case 0x500: return enable_;
    case 0x508: return psel_[0];
    case 0x50c: return psel_[1];
    case 0x524: return frequency_;
    case 0x534: return rx_ptr_;
    case 0x538: return rx_max_;
    case 0x53c: return rx_amount_;
    case 0x544: return tx_ptr_;
    case 0x548: return tx_max_;
    case 0x54c: return tx_amount_;
    case 0x588: return address_;
    default: return 0;
    }
}

void Twim::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x100) {
        if (value & 1u)
            task(offset);
        return;
    }
    for (unsigned bit : {1u, 9u, 18u, 19u, 20u, 23u, 24u}) {
        if (event_register(bit) == offset) {
            if (value & 1u)
                events_ |= 1u << bit;
            else
                events_ &= ~(1u << bit);
            update_irq();
            return;
        }
    }
    switch (offset) {
    case 0x200: shorts_ = value; break;
    case 0x304: inten_ |= value; update_irq(); break;
    case 0x308: inten_ &= ~value; update_irq(); break;
    case 0x4c4: errorsrc_ &= ~value; break;
    case 0x500:
        enable_ = value & 0xfu;
        steps_.clear();
        step_index_ = 0;
        bus_started_ = false;
        if (enable_ == 6)
            release_lines();
        else
            hand_back();
        break;
    case 0x508:
        hand_back();
        psel_[0] = value;
        release_lines();
        break;
    case 0x50c:
        hand_back();
        psel_[1] = value;
        release_lines();
        break;
    case 0x524: frequency_ = value; break;
    case 0x534: rx_ptr_ = value; break;
    case 0x538: rx_max_ = value & 0x7fffu; break;
    case 0x544: tx_ptr_ = value; break;
    case 0x548: tx_max_ = value & 0x7fffu; break;
    case 0x588: address_ = value & 0x7fu; break;
    default: break;
    }
}

} // namespace nrf52emu
