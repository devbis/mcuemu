// TLSR8258 I2C master (registers 0x00..0x07).
//
// Evidence: datasheet section 7.3.4 and the SDK drivers (drivers/B85/i2c.c).
// Command register 0x07 bits: ID(0) ADDR(1) DO(2) DI(3) START(4) STOP(5)
// READ_ID(6) ACK(7). The master runs the enabled phases in the fixed order
// START, ID byte (0x01), ADR (0x04), DO (0x05), DI (0x06), STOP. With
// READ_ID set the DI phase receives a byte into 0x06 instead of sending it.
// Status 0x02: CMD_BUSY(0), BUS_BUSY(1), NAK(2). Bit period = 4 * reg 0x00
// system-clock cycles (datasheet: FI2C = system clock / (4 * speed)).
//
// Inferred, not documented: the ACK command bit makes the master release SDA
// (NACK) after a received byte, as the SDK sets it on the last byte of a read;
// NAK reports a missing slave ACK on any written byte of the command; the
// remaining phases still run after a NACK; clock stretching is not modeled.

#include "tc32emu/tlsr8258.hpp"

#include <algorithm>

namespace tc32emu {

namespace {

constexpr std::uint8_t CmdId = 1u << 0;
constexpr std::uint8_t CmdAddr = 1u << 1;
constexpr std::uint8_t CmdDo = 1u << 2;
constexpr std::uint8_t CmdDi = 1u << 3;
constexpr std::uint8_t CmdStart = 1u << 4;
constexpr std::uint8_t CmdStop = 1u << 5;
constexpr std::uint8_t CmdRead = 1u << 6;
constexpr std::uint8_t CmdAck = 1u << 7;

constexpr std::uint32_t RegSpeed = 0x00;
constexpr std::uint32_t RegId = 0x01;
constexpr std::uint32_t RegMode = 0x03;
constexpr std::uint32_t RegAdr = 0x04;
constexpr std::uint32_t RegDo = 0x05;
constexpr std::uint32_t RegDi = 0x06;
constexpr std::uint8_t ModeMasterEnable = 1u << 1;

struct PadPair {
    unsigned sda_port, sda_bit, scl_port, scl_bit;
};
// I2C_GPIO_GroupTypeDef order in the SDK: A3/A4, B6/D7, C0/C1, C2/C3.
constexpr PadPair pad_pairs[] = {
    {0, 3, 0, 4},
    {1, 6, 3, 7},
    {2, 0, 2, 1},
    {2, 2, 2, 3},
};

} // namespace

I2cPads Tlsr8258::i2c_select_pads() const {
    for (const PadPair &pair : pad_pairs) {
        if (!gpio_is_gpio(pair.sda_port, pair.sda_bit) &&
            !gpio_is_gpio(pair.scl_port, pair.scl_bit))
            return {static_cast<int>(pair.sda_port), pair.sda_bit,
                    static_cast<int>(pair.scl_port), pair.scl_bit};
    }
    return {};
}

std::uint8_t Tlsr8258::i2c_status() const {
    return static_cast<std::uint8_t>((i2c_steps_.empty() ? 0u : 1u) |
                                     (i2c_bus_busy_ ? 2u : 0u) |
                                     (i2c_nak_ ? 4u : 0u));
}

void Tlsr8258::i2c_apply(bool sda_low, bool scl_low) {
    if (sda_low == i2c_sda_low_ && scl_low == i2c_scl_low_)
        return;
    i2c_sda_low_ = sda_low;
    i2c_scl_low_ = scl_low;
    if (i2c_pad_observer_)
        i2c_pad_observer_(i2c_pads_, sda_low, scl_low);
}

void Tlsr8258::i2c_start_command(std::uint8_t command) {
    if ((registers_[RegMode] & ModeMasterEnable) == 0 || !i2c_steps_.empty())
        return;
    i2c_nak_ = false;
    i2c_cmd_stop_ = (command & CmdStop) != 0;
    i2c_pads_ = i2c_select_pads();

    const std::uint64_t quarter = std::max<std::uint64_t>(registers_[RegSpeed], 1u);
    std::uint64_t at = i2c_cycles_;
    using Kind = I2cStep::Kind;
    auto push = [&](std::uint64_t cycle, Kind kind, bool sda_low, bool scl_low) {
        i2c_steps_.push_back({cycle, kind, sda_low, scl_low});
    };
    auto drive = [&](std::uint64_t cycle, bool sda, bool scl) {
        push(cycle, Kind::Drive, sda, scl);
    };

    auto write_byte = [&](std::uint8_t value) {
        for (int bit = 7; bit >= 0; --bit) {
            const bool low = ((value >> bit) & 1u) == 0;
            drive(at, low, true);
            drive(at + quarter, low, false);
            drive(at + 3 * quarter, low, true);
            at += 4 * quarter;
        }
        drive(at, false, true); // release SDA for the slave's ACK
        drive(at + quarter, false, false);
        push(at + 2 * quarter, Kind::SampleAck, false, false);
        drive(at + 3 * quarter, false, true);
        at += 4 * quarter;
    };
    auto read_byte = [&](bool nack) {
        for (int bit = 7; bit >= 0; --bit) {
            drive(at, false, true);
            drive(at + quarter, false, false);
            push(at + 2 * quarter, Kind::SampleBit, false, false);
            drive(at + 3 * quarter, false, true);
            at += 4 * quarter;
        }
        push(at, Kind::StoreRead, false, false);
        drive(at, !nack, true); // ACK pulls SDA low
        drive(at + quarter, !nack, false);
        drive(at + 3 * quarter, false, true);
        at += 4 * quarter;
    };

    if (command & CmdStart) {
        // Also valid as a repeated START: SCL is low between commands.
        drive(at, false, i2c_scl_low_);
        drive(at + quarter, false, false);
        drive(at + 2 * quarter, true, false); // SDA falls while SCL is high
        drive(at + 3 * quarter, true, true);
        at += 4 * quarter;
    }
    if (command & CmdId)
        write_byte(registers_[RegId]);
    if (command & CmdAddr)
        write_byte(registers_[RegAdr]);
    if (command & CmdDo)
        write_byte(registers_[RegDo]);
    if (command & CmdDi) {
        if (command & CmdRead)
            read_byte((command & CmdAck) != 0);
        else
            write_byte(registers_[RegDi]);
    }
    if (command & CmdStop) {
        drive(at, true, true);
        drive(at + quarter, true, false);
        drive(at + 2 * quarter, false, false); // SDA rises while SCL is high
        at += 3 * quarter;
    }
    push(at, Kind::End, false, false);
    if (command & CmdStart)
        i2c_bus_busy_ = true;
}

void Tlsr8258::i2c_advance(std::uint32_t cycles) {
    i2c_cycles_ += cycles;
    using Kind = I2cStep::Kind;
    while (!i2c_steps_.empty() && i2c_steps_.front().cycle <= i2c_cycles_) {
        const I2cStep step = i2c_steps_.front();
        i2c_steps_.pop_front();
        const bool sda_high_on_pad =
            !i2c_pads_.valid() ||
            (gpio_inputs_[static_cast<unsigned>(i2c_pads_.sda_port)] &
             (1u << i2c_pads_.sda_bit)) != 0;
        switch (step.kind) {
        case Kind::Drive:
            i2c_apply(step.sda_low, step.scl_low);
            break;
        case Kind::SampleBit:
            i2c_read_shift_ = static_cast<std::uint8_t>(
                (i2c_read_shift_ << 1) | (sda_high_on_pad ? 1u : 0u));
            break;
        case Kind::SampleAck:
            // The pad level seen while the master released SDA: a slave ACKs
            // by pulling it low.
            if (sda_high_on_pad)
                i2c_nak_ = true;
            break;
        case Kind::StoreRead:
            registers_[RegDi] = i2c_read_shift_;
            i2c_read_shift_ = 0;
            break;
        case Kind::End:
            // Without STOP the master keeps SCL low and waits for the next
            // command; with STOP the lines were already released.
            if (i2c_cmd_stop_)
                i2c_bus_busy_ = false;
            break;
        }
    }
}

} // namespace tc32emu
