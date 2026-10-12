#include "sim/parts/i2c_slave.hpp"

namespace sim {

I2cSlave::I2cSlave(Pin &sda, Pin &scl, Handlers handlers)
    : sda_(sda), scl_(scl), handlers_(std::move(handlers)),
      last_sda_(sda.net().level() != Level::Low),
      last_scl_(scl.net().level() != Level::Low) {
    sda_.net().subscribe([this](const NetChange &c) { on_sda(c); });
    scl_.net().subscribe([this](const NetChange &c) { on_scl(c); });
}

void I2cSlave::drive_bit(bool bit, std::uint64_t t) {
    sda_.set_drive(bit ? Drive::HiZ : Drive::Low, t);
}

void I2cSlave::on_sda(const NetChange &change) {
    const bool level = change.level != Level::Low;
    const bool changed = level != last_sda_;
    last_sda_ = level;
    // SDA moving while SCL is high is a START (falling) or STOP (rising); data
    // only changes while SCL is low.
    if (!changed || !last_scl_)
        return;
    if (!level) {
        state_ = State::RecvBits;
        addressed_ = false;
        read_mode_ = false;
        first_byte_ = true;
        bit_count_ = 0;
        shift_ = 0;
        release(change.time_ns);
        if (handlers_.on_start)
            handlers_.on_start();
    } else {
        const bool was_active = state_ != State::Idle;
        state_ = State::Idle;
        addressed_ = false;
        if (was_active && handlers_.on_stop)
            handlers_.on_stop();
    }
}

void I2cSlave::on_scl(const NetChange &change) {
    const bool level = change.level != Level::Low;
    if (level == last_scl_)
        return;
    last_scl_ = level;
    if (level)
        scl_rising(change.time_ns);
    else
        scl_falling(change.time_ns);
}

void I2cSlave::scl_rising(std::uint64_t) {
    switch (state_) {
    case State::RecvBits:
        shift_ = static_cast<std::uint8_t>((shift_ << 1) | (sda() ? 1u : 0u));
        if (++bit_count_ == 8) {
            if (!addressed_) {
                read_mode_ = (shift_ & 1u) != 0;
                ack_ = handlers_.on_address &&
                       handlers_.on_address(static_cast<std::uint8_t>(shift_ >> 1),
                                            read_mode_);
            } else {
                ack_ = handlers_.on_write && handlers_.on_write(shift_);
            }
        }
        break;
    case State::SendAck:
        break; // master samples our ACK on this edge
    case State::RecvAck:
        master_acked_ = !sda(); // master pulled SDA low = ACK
        break;
    default:
        break;
    }
}

void I2cSlave::scl_falling(std::uint64_t t) {
    switch (state_) {
    case State::RecvBits:
        if (bit_count_ == 8) {
            // End of the 8th clock: present ACK (SDA low) or leave it high.
            if (ack_)
                sda_.set_drive(Drive::Low, t);
            state_ = State::SendAck;
        }
        break;
    case State::SendAck:
        // End of the ACK clock: release SDA and move on.
        release(t);
        bit_count_ = 0;
        shift_ = 0;
        if (!ack_) {
            state_ = State::Idle;
        } else if (!addressed_ && read_mode_) {
            addressed_ = true;
            shift_ = handlers_.on_read ? handlers_.on_read() : 0xff;
            drive_bit((shift_ & 0x80u) != 0, t);
            bit_count_ = 1;
            state_ = State::SendBits;
        } else {
            addressed_ = true;
            state_ = State::RecvBits;
        }
        break;
    case State::SendBits:
        if (bit_count_ < 8) {
            drive_bit(((shift_ >> (7 - bit_count_)) & 1u) != 0, t);
            ++bit_count_;
        } else {
            release(t); // let the master ACK/NACK the byte
            state_ = State::RecvAck;
        }
        break;
    case State::RecvAck:
        if (master_acked_) {
            shift_ = handlers_.on_read ? handlers_.on_read() : 0xff;
            drive_bit((shift_ & 0x80u) != 0, t);
            bit_count_ = 1;
            state_ = State::SendBits;
        } else {
            state_ = State::Idle; // NACK: the master ends the read
        }
        break;
    default:
        break;
    }
}

} // namespace sim
