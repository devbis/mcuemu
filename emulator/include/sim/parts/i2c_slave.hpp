#pragma once

#include "sim/net.hpp"

#include <cstdint>
#include <functional>

namespace sim {

// Bit-level I2C slave front end for a pair of open-drain nets. It decodes
// START/STOP, shifts bytes on SCL edges and drives SDA for ACKs and read data,
// so it works with both hardware controllers and bit-banging firmware. A
// device model supplies the protocol through the handlers.
class I2cSlave {
public:
    struct Handlers {
        // 7-bit address and R/W; return true to ACK.
        std::function<bool(std::uint8_t address, bool read)> on_address;
        // Byte written by the master; return true to ACK.
        std::function<bool(std::uint8_t byte)> on_write;
        // Next byte for a master read.
        std::function<std::uint8_t()> on_read;
        std::function<void()> on_start; // START or repeated START
        std::function<void()> on_stop;
    };

    I2cSlave(Pin &sda, Pin &scl, Handlers handlers);

    // SDA/SCL as seen on the wire (true = high).
    bool sda() const { return sda_.net().level() != Level::Low; }
    bool scl() const { return scl_.net().level() != Level::Low; }

private:
    enum class State {
        Idle,      // no transaction addressed to us
        RecvBits,  // receiving the 8 bits of an address or data byte
        SendAck,   // after bit 8: our ACK is on the wire
        SendBits,  // transmitting a byte to the master
        RecvAck,   // master's ACK of our byte
    };

    void on_sda(const NetChange &change);
    void on_scl(const NetChange &change);
    void scl_rising(std::uint64_t t);
    void scl_falling(std::uint64_t t);
    void release(std::uint64_t t) { sda_.set_drive(Drive::HiZ, t); }
    void drive_bit(bool bit, std::uint64_t t);

    Pin &sda_;
    Pin &scl_;
    Handlers handlers_;
    State state_{State::Idle};
    bool addressed_{false}; // address phase done and ACKed
    bool read_mode_{false};
    unsigned bit_count_{0};
    std::uint8_t shift_{0};
    bool ack_{false};
    bool master_acked_{false};
    bool first_byte_{true};
    bool last_sda_{true};
    bool last_scl_{true};
};

} // namespace sim
