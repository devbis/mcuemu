#include "blocks.hpp"

#include <algorithm>
#include <vector>

namespace bl702emu {

namespace {
constexpr mcuemu::RadioProtocol Zigbee = mcuemu::RadioProtocol::Zigbee802154;
// The MAC's time counter (0x40) ticks in 16 us symbols: the driver writes now_us >> 4 to it, and the library's
// alarms and ACK wait times are counted in the same units (54 symbols = 864 us, the 2.4 GHz ACK wait).
constexpr std::uint32_t CyclesPerSymbol = Bl702::ClockHz / 1000000 * 16;

// The channel is the frequency field of M154_CONTROL (bits 16..27), as the driver writes it: 0x965 for
// channel 11 (2405 MHz), 5 more per channel.
constexpr std::uint32_t ChannelFieldBase = 0x965, ChannelFieldStep = 5;
constexpr std::uint8_t FirstChannel = 11, LastChannel = 26;

// The M154 register offsets (from 0x4c000000), as the zigbee-rs driver uses them.
constexpr std::uint32_t M154Control = 0x000, M154Reset = 0x008, M154IrqClear = 0x024;
constexpr std::uint32_t M154IrqStatus = 0x030, M154Timer = 0x040, M154Busy = 0x0b0, M154TxControl = 0x0b4;
constexpr std::uint32_t M154State = 0x0bc, M154RxStatus = 0x0c4, M154RxControl = 0x0e4;
constexpr std::uint32_t M154TxBuffer = 0x200, M154TxBufferSize = 128, M154TxLength = 0x2fc;
constexpr std::uint32_t M154RxBuffer = 0x400, M154RxBufferSize = 128, M154RxMetadata = 0x4fc;
constexpr std::uint32_t M154ResetTx = 24;
constexpr std::uint32_t M154AckWait = 0x0fc, DefaultAckWaitTime = 0x360;
constexpr std::uint32_t M154IrqEnable = 0x020, M154TimerEnable = 0x020, M154TimerTarget = 0x064;
constexpr unsigned M154TimerChannels = 6;
constexpr std::uint32_t TimerChannelMask = (1u << M154TimerChannels) - 1;

// Bits of the registers the driver polls.
constexpr std::uint32_t BusyTx = 1u << 11, TxStart = 1u << 0, RxStart = 1u << 12, RxStop = 1u << 8;
constexpr std::uint32_t StateIdle = 1, StateMask = 0x7ff;

// The PHY registers (absolute) that the CCA and the RSSI use.
constexpr std::uint32_t PhyCcaEnable = 0x4000'182c, PhyCcaCtrl = 0x4000'1834, PhyRssi = 0x4000'1860;
constexpr std::uint32_t CcaEnableStart = 1u << 27;
constexpr std::uint32_t CcaDone = 1u << 2, CcaBusy = 1u << 7, CcaRawMask = 0x7fu << 8;

// The raw RSSI the hardware reports is 5 - dBm (the driver converts it back the same way).
int raw_rssi(int dbm) { return std::clamp(5 - dbm, 0, 127); }

std::uint32_t little_endian(const std::uint8_t *bytes, std::uint32_t available) {
    std::uint32_t value = 0;
    for (std::uint32_t b = 0; b < 4 && b < available; ++b)
        value |= static_cast<std::uint32_t>(bytes[b]) << (8 * b);
    return value;
}
} // namespace

// ---- RF / PHY -------------------------------------------------------------------------------------------

RfPhy::RfPhy(Bl702 &chip) : RegisterBlock(chip, 0x4000'1000, 0x1000) {}

void RfPhy::write_word(std::uint32_t offset, std::uint32_t value) {
    RegisterBlock::write_word(offset, value);
    // The CCA runs when the driver sets its start bit; the result is ready at once (inferred, the
    // driver polls for it).
    if (base() + offset == PhyCcaEnable && (value & CcaEnableStart) && mac_)
        store(PhyCcaCtrl, mac_->assess(load(PhyCcaCtrl)));
}

// ---- IEEE 802.15.4 MAC ----------------------------------------------------------------------------------

Ieee802154::Ieee802154(Bl702 &chip, RfPhy &phy) : RegisterBlock(chip, Bl702::M154Base, Bl702::M154Size), phy_(phy) {}

void Ieee802154::connect(mcuemu::VirtualAir &air, const std::string &node) {
    air_ = &air;
    node_ = node;
}

void Ieee802154::reset() {
    RegisterBlock::reset();
    // The ACK wait time (bits 16..25 of 0xFC) has a reset value: lmac154_init copies it into its own global, and an
    // interrupt-routine alarm of zero never ends (0x360 is the library's default; inferred from its data section).
    set_word(M154AckWait, DefaultAckWaitTime << 16);
    tx_busy_ = false;
    rx_on_ = false;
    tx_end_cycle_ = 0;
    irq_pending_ = 0;
    rx_phy_length_ = 0;
}

std::uint8_t Ieee802154::channel() const {
    const std::uint32_t field = (word(M154Control) >> 16) & 0xfff;
    if (field >= ChannelFieldBase && (field - ChannelFieldBase) % ChannelFieldStep == 0) {
        const std::uint32_t channel = FirstChannel + (field - ChannelFieldBase) / ChannelFieldStep;
        if (channel <= LastChannel)
            return static_cast<std::uint8_t>(channel);
    }
    return FirstChannel;
}

std::uint64_t Ieee802154::now_cycles() const { return chip_.core().cycles(); }

std::uint32_t Ieee802154::read_word(std::uint32_t offset) {
    switch (offset) {
    case M154Busy:
        return word(offset) | (tx_busy_ ? BusyTx : 0);
    case M154State:
        // The state field reads 1 when the MAC is idle (the driver's is_idle test); the rest is stored.
        return (word(offset) & ~StateMask) | (tx_busy_ ? 0 : StateIdle);
    case M154RxControl:
        return word(offset) & ~RxControlCommands;
    case M154TxControl:
        return word(offset) & ~TxStart;
    case M154IrqStatus:
        return irq_pending_;
    case M154Timer:
        return static_cast<std::uint32_t>(timer_ticks());
    case M154RxStatus:
        return 0; // no CRC error: the model never corrupts a frame
    case M154RxMetadata:
        return rx_phy_length_ << 24;
    default:
        break;
    }
    if (offset >= M154RxBuffer && offset < M154RxBuffer + M154RxBufferSize)
        return little_endian(rx_buffer_ + (offset - M154RxBuffer), M154RxBufferSize - (offset - M154RxBuffer));
    return RegisterBlock::read_word(offset);
}

void Ieee802154::write_word(std::uint32_t offset, std::uint32_t value) {
    if (offset >= M154TxBuffer && offset < M154TxBuffer + M154TxBufferSize) {
        for (std::uint32_t b = 0; b < 4 && offset - M154TxBuffer + b < M154TxBufferSize; ++b)
            tx_buffer_[offset - M154TxBuffer + b] = static_cast<std::uint8_t>(value >> (8 * b));
        return;
    }
    if (offset >= M154RxBuffer && offset < M154RxBuffer + M154RxBufferSize)
        return; // the receive buffer is read-only
    switch (offset) {
    case M154Reset:
        if (value == M154ResetTx)
            tx_busy_ = false; // the aborted frame is not taken back off the air (documented)
        return;
    case M154IrqClear:
        irq_pending_ &= ~value;
        return;
    case M154IrqStatus:
        return;
    case M154TxControl:
        RegisterBlock::write_word(offset, value & ~TxStart);
        if (value & TxStart)
            start_tx();
        return;
    case M154RxControl:
        // Bit 12 is the receiver's state as written (the library clears it to stop); bit 8 stops it too.
        rx_on_ = (value & RxStart) != 0 && (value & RxStop) == 0;
        if (rx_on_)
            chip_.core().request_service(now_cycles());
        RegisterBlock::write_word(offset, value & ~RxControlCommands);
        return;
    case M154Timer:
        time_offset_ = value - now_cycles() / CyclesPerSymbol;
        return;
    default:
        RegisterBlock::write_word(offset, value);
        if (offset == M154TimerEnable || (offset >= M154TimerTarget && offset < M154TimerTarget + 4 * M154TimerChannels))
            chip_.core().request_service(now_cycles());
    }
}

void Ieee802154::start_tx() {
    if (tx_busy_)
        return;
    // PHR counts the two FCS bytes the hardware appends, so the MAC frame is PHR - 2 bytes.
    const int length = static_cast<int>(word(M154TxLength) >> 24) - 2;
    if (length < 1 || length > 125)
        return;
    std::vector<std::uint8_t> payload(tx_buffer_, tx_buffer_ + length);
    tx_ack_requested_ = (payload[0] & 0x20) != 0; // the frame control's ACK request bit
    const std::uint64_t now = now_cycles();
    const std::uint64_t start_ns = chip_.core().cycles_to_ns(now);
    std::uint64_t end_ns = start_ns + mcuemu::VirtualAir::airtime_ns(Zigbee, payload.size());
    if (air_) {
        const std::uint64_t id = air_->transmit(node_, {}, Zigbee, channel(), start_ns, std::move(payload));
        for (const mcuemu::RadioFrame &frame : air_->frames())
            if (frame.id == id)
                end_ns = frame.end_ns;
    }
    tx_busy_ = true;
    tx_end_cycle_ = std::max(chip_.core().ns_to_cycles(end_ns), now + 1);
    chip_.core().request_service(tx_end_cycle_);
}

void Ieee802154::deliver_rx(std::uint64_t now_ns) {
    if (tx_busy_)
        return; // half duplex: a frame that ends during our transmission is not heard
    for (const mcuemu::RadioFrame &frame : air_->receive(node_, Zigbee, channel(), now_ns)) {
        if (frame.payload.empty() || frame.payload.size() > 125)
            continue;
        std::copy(frame.payload.begin(), frame.payload.end(), rx_buffer_);
        rx_phy_length_ = static_cast<std::uint32_t>(frame.payload.size()) + 2;
        phy_.store(PhyRssi, static_cast<std::uint32_t>(raw_rssi(frame.rssi_dbm)) << 8);
        // An acknowledgement raises RX_ACK (bit 22), which the library enables; a data frame raises RX_DONE and RX_CRC
        // (bit 19), which the library waits for.
        const bool acknowledgement = (frame.payload[0] & 7) == 2;
        irq_pending_ |= acknowledgement ? (IrqRxAck | IrqRxDone) : (IrqRxDone | IrqRxCrc);
        if (!acknowledgement && addressed_to_us(frame.payload))
            acknowledge(frame);
    }
}

// The hardware acknowledges a unicast frame that asks for it and is addressed to this radio's extended address
// (the EUI-64 the driver wrote with lmac154_setLongAddr, which lands in the words at 0xd0 and 0xd4, in frame byte
// order: inferred from the registers the library writes). Frames to other addresses are left alone.
bool Ieee802154::addressed_to_us(const std::vector<std::uint8_t> &frame) const {
    const std::uint32_t fcf = frame[0] | (std::uint32_t(frame[1]) << 8);
    const std::uint32_t destination_mode = (fcf >> 10) & 3;
    if (!(fcf & FcfAckRequest) || destination_mode != AddressModeExtended || frame.size() < 13)
        return false;
    const std::uint32_t low = frame[5] | (std::uint32_t(frame[6]) << 8) | (std::uint32_t(frame[7]) << 16) |
                              (std::uint32_t(frame[8]) << 24);
    const std::uint32_t high = frame[9] | (std::uint32_t(frame[10]) << 8) | (std::uint32_t(frame[11]) << 16) |
                               (std::uint32_t(frame[12]) << 24);
    return low == word(LongAddrLow) && high == word(LongAddrHigh);
}

void Ieee802154::acknowledge(const mcuemu::RadioFrame &frame) {
    // The acknowledgement is sent one turnaround time (aTurnaroundTime, 12 symbols) after the frame ends.
    const std::vector<std::uint8_t> ack = {0x02, 0x00, frame.payload[2]};
    const std::uint64_t start_ns = frame.end_ns + AckTurnaroundNs;
    air_->transmit(node_, {}, Zigbee, channel(), start_ns, ack);
}

std::uint64_t Ieee802154::service(std::uint64_t now) {
    if (tx_busy_ && now >= tx_end_cycle_) {
        tx_busy_ = false;
        irq_pending_ |= IrqTxFinished;
        // A frame that asks for an acknowledgement leaves the receiver on for it (inferred: the hardware turns the
        // radio around for the ACK; the library does not turn it on itself).
        if (tx_ack_requested_)
            rx_on_ = true;
    }
    std::uint64_t next = tx_busy_ ? tx_end_cycle_ : NoEvent;
    next = std::min(next, timers(now));
    if (!rx_on_ || !air_)
        return next;
    deliver_rx(chip_.core().cycles_to_ns(now));
    return std::min(next, now + PollPeriodCycles);
}

// The timer channels: channel n fires (status bit n) once the counter reaches its target, while its enable bit
// (bit n of IRQ_ENABLE, which the library also uses for its other enables) is set. Returns the cycle of the next
// compare, or NoEvent.
std::uint64_t Ieee802154::timers(std::uint64_t now) {
    const std::uint32_t enabled = word(M154IrqEnable) & TimerChannelMask;
    const std::uint32_t ticks = static_cast<std::uint32_t>(timer_ticks());
    std::uint64_t next = NoEvent;
    for (unsigned ch = 0; ch < M154TimerChannels; ++ch) {
        if (!(enabled & (1u << ch)))
            continue;
        const std::uint32_t target = word(M154TimerTarget + 4 * ch);
        const std::uint32_t ahead = target - ticks; // wraps like the counter
        if (ahead >= 0x8000'0000u) {
            irq_pending_ |= 1u << ch;
        } else {
            next = std::min(next, now + std::uint64_t(ahead) * CyclesPerSymbol);
        }
    }
    return next;
}

std::uint64_t Ieee802154::timer_ticks() const {
    return now_cycles() / CyclesPerSymbol + time_offset_;
}

std::uint32_t Ieee802154::assess(std::uint32_t cca_control) {
    // The threshold is the 7-bit field at bits 16..22 of PHY_CCA_CTRL, read as -dBm (inferred).
    const int threshold = -static_cast<int>((cca_control >> 16) & 0x7f);
    int strongest = -100; // the noise floor when nothing is on the air
    bool busy = false;
    if (air_) {
        for (const mcuemu::RadioFrame &frame :
             air_->in_flight(node_, Zigbee, channel(), chip_.core().cycles_to_ns(now_cycles()), 0)) {
            strongest = std::max(strongest, frame.rssi_dbm);
            busy = busy || frame.rssi_dbm >= threshold;
        }
    }
    const std::uint32_t kept = cca_control & ~(CcaDone | CcaBusy | CcaRawMask);
    return kept | CcaDone | (busy ? CcaBusy : 0) | (static_cast<std::uint32_t>(raw_rssi(strongest)) << 8);
}

} // namespace bl702emu
