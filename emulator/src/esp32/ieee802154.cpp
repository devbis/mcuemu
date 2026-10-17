#include "ieee802154.hpp"

#include "esp32emu/esp32c6_regs.hpp"

#include <algorithm>
#include <cstring>

namespace esp32emu {

namespace {
namespace r = esp32c6_regs::ieee802154::reg;

constexpr std::uint64_t PollUs = 16;
// Time from the command to the first bit on air: the TX-on delay the driver
// programs is 50 us and the hardware adds the PA ramp; 192 us (12 symbols) is the
// 802.15.4 turnaround and a round figure for both. Inferred, not measured.
constexpr std::uint64_t TxLeadNs = 192000;
constexpr std::uint64_t AckTurnaroundNs = 192000;
constexpr std::uint64_t RxOnNs = 50000;
constexpr std::uint64_t SfdNs = 160000; // 4 preamble bytes + SFD at 32 us
constexpr std::uint64_t EdDurationNs = 128000;

constexpr std::uint32_t EvTxDone = 1u << 0, EvRxDone = 1u << 1, EvAckTxDone = 1u << 2, EvAckRxDone = 1u << 3,
                        EvRxAbort = 1u << 4, EvTxAbort = 1u << 5, EvEdDone = 1u << 6, EvTxSfd = 1u << 11,
                        EvRxSfd = 1u << 12;
constexpr unsigned AckTxCtl = 0, AckRxCtl = 3, Promiscuous = 7, PanCoordinator = 6, DisVersionFilter = 11;
} // namespace

Ieee802154::Ieee802154(std::uint32_t base, std::uint32_t cpu_hz, unsigned source, CycleSource cycles, IrqSink irq, MemRead read,
                       MemWrite write, Request request_service)
    : Block(base, 0x200), cpu_hz_(cpu_hz), source_(source), cycles_(std::move(cycles)), irq_(std::move(irq)),
      mem_read_(std::move(read)), mem_write_(std::move(write)), request_(std::move(request_service)) {
    regs_[r::MAC_DATE / 4] = esp32c6_regs::ieee802154::reset::MAC_DATE;
    regs_[r::CTRL_CFG / 4] = esp32c6_regs::ieee802154::reset::CTRL_CFG;
}

void Ieee802154::connect(mcuemu::VirtualAir &air, const std::string &node) {
    air_ = &air;
    node_ = node;
}

// Split into whole seconds and a remainder so the product stays in 64 bits (no __int128 on MSVC).
std::uint64_t Ieee802154::to_ns(std::uint64_t cycles) const {
    return cycles / cpu_hz_ * 1000000000ull + cycles % cpu_hz_ * 1000000000ull / cpu_hz_;
}
std::uint64_t Ieee802154::to_cycles(std::uint64_t ns) const {
    return ns / 1000000000ull * cpu_hz_ + ns % 1000000000ull * cpu_hz_ / 1000000000ull;
}

std::uint8_t Ieee802154::channel() const {
    // HOP holds 3 + 5 * (channel - 11) (esp-hal channel_to_freq).
    const std::uint32_t hop = reg(r::CHANNEL) & 0x7f;
    return static_cast<std::uint8_t>(hop < 3 ? 11 : std::min<std::uint32_t>((hop - 3) / 5 + 11, 26));
}

void Ieee802154::update_irq() { irq_(source_, (events_ & reg(r::EVENT_EN)) != 0); }

void Ieee802154::raise(std::uint32_t event) {
    events_ |= event;
    update_irq();
}

void Ieee802154::tx_abort(unsigned code) {
    tx_abort_code_ = code;
    state_ = State::Idle;
    if (reg(r::TX_ABORT_INTERRUPT_CONTROL) & (1u << (code - 1)))
        raise(EvTxAbort);
}

void Ieee802154::rx_abort(unsigned code) {
    rx_abort_code_ = code;
    if (reg(r::RX_ABORT_INTR_CTRL) & (1u << (code - 1))) {
        state_ = State::Idle;
        raise(EvRxAbort);
    }
}

std::uint32_t Ieee802154::read(std::uint32_t offset, unsigned) {
    offset &= ~3u;
    if (offset >= 0x200)
        return 0;
    switch (offset) {
    case r::EVENT_STATUS: return events_;
    case r::RX_STATUS:
        // RX_STATE above 1 means a frame is being received (hal.rs is_current_rx_frame).
        return (rx_abort_code_ << 4) | ((state_ == State::Rx || state_ == State::RxAck ? 1u : 0u) << 16);
    case r::TX_STATUS: return tx_abort_code_ << 4;
    case r::TIME0_VALUE:
    case r::TIME1_VALUE: {
        const unsigned i = offset == r::TIME1_VALUE;
        return timer_running_[i] ? static_cast<std::uint32_t>((to_ns(cycles_()) - timer_start_ns_[i]) / 1000) : 0;
    }
    case r::ED_SCAN_CFG:
        // ED_RSS (bits 16..23) is a signed dBm figure; the model reports a quiet channel.
        return (reg(offset) & ~(0xffu << 16)) | (static_cast<std::uint32_t>(static_cast<std::uint8_t>(-100)) << 16);
    default: return regs_[offset / 4];
    }
}

void Ieee802154::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    const std::uint32_t aligned = offset & ~3u;
    if (aligned >= 0x200)
        return;
    if (size != 4) {
        const unsigned shift = (offset & 3u) * 8;
        const std::uint32_t mask = (size == 1 ? 0xffu : 0xffffu) << shift;
        value = (regs_[aligned / 4] & ~mask) | ((value << shift) & mask);
    }
    switch (aligned) {
    case r::EVENT_STATUS:
        // Write one to clear (the Rust driver writes status & mask).
        events_ &= ~value;
        update_irq();
        return;
    case r::COMMAND:
        regs_[aligned / 4] = value;
        command(static_cast<std::uint8_t>(value));
        return;
    case r::RX_STATUS:
    case r::TX_STATUS:
    case r::MAC_DATE: return;
    default: regs_[aligned / 4] = value;
    }
    if (aligned == r::EVENT_EN)
        update_irq();
    if (aligned == r::TIME0_THRESHOLD || aligned == r::TIME1_THRESHOLD)
        request_(cycles_());
}

void Ieee802154::command(std::uint8_t opcode) {
    const std::uint64_t ns = to_ns(cycles_());
    switch (opcode) {
    case 0x41: start_tx(false); break;
    case 0x43: start_tx(true); break;
    case 0x42: start_rx(); break;
    case 0x45: stop(); break;
    case 0x44:
        state_ = State::Ed;
        ed_end_ns_ = ns + EdDurationNs;
        break;
    case 0x4c:
    case 0x4e: {
        const unsigned i = opcode == 0x4e;
        timer_running_[i] = true;
        timer_start_ns_[i] = ns;
        break;
    }
    case 0x4d: timer_running_[0] = false; break;
    case 0x4f: timer_running_[1] = false; break;
    default: break; // DTM and unknown opcodes
    }
    request_(cycles_());
}

void Ieee802154::start_rx() {
    // esp-hal re-issues RxStart every few hundred us: a receiver that is listening keeps listening and
    // one that is busy sending a frame or an ACK (or waiting for one) ignores the command.
    if (state_ == State::Rx || state_ == State::Tx || state_ == State::TxAck || state_ == State::RxAck)
        return;
    const std::uint64_t ns = to_ns(cycles_());
    state_ = State::Rx;
    rx_start_ns_ = ns + RxOnNs;
    last_poll_ns_ = ns;
}

void Ieee802154::stop() {
    switch (state_) {
    case State::Rx: rx_abort_code_ = 1; if (reg(r::RX_ABORT_INTR_CTRL) & 1u) raise(EvRxAbort); break;
    case State::TxAck: rx_abort_code_ = 17; if (reg(r::RX_ABORT_INTR_CTRL) & (1u << 16)) raise(EvRxAbort); break;
    case State::Tx: tx_abort_code_ = 17; if (reg(r::TX_ABORT_INTERRUPT_CONTROL) & (1u << 16)) raise(EvTxAbort); break;
    case State::RxAck: tx_abort_code_ = 1; if (reg(r::TX_ABORT_INTERRUPT_CONTROL) & 1u) raise(EvTxAbort); break;
    default: break;
    }
    state_ = State::Idle;
    ack_due_ = false;
    ack_on_air_ = false;
}

void Ieee802154::start_tx(bool cca) {
    std::uint8_t phr = 0;
    if (!mem_read_(reg(r::TXDMA_ADDR), &phr, 1) || phr < 2) {
        tx_abort(19); // no usable frame: reported like the closest abort the driver handles
        return;
    }
    tx_frame_.assign(phr - 2u, 0);
    if (!tx_frame_.empty())
        mem_read_(reg(r::TXDMA_ADDR) + 1, tx_frame_.data(), tx_frame_.size());
    const std::uint64_t ns = to_ns(cycles_());
    const std::uint64_t start = ns + TxLeadNs;
    if (cca && air_ && !air_->in_flight(node_, mcuemu::RadioProtocol::Zigbee802154, channel(), ns, 0).empty()) {
        tx_abort(25); // CCA busy
        return;
    }
    std::uint64_t end = start + mcuemu::VirtualAir::airtime_ns(mcuemu::RadioProtocol::Zigbee802154, phr);
    if (air_ && !tx_frame_.empty() && tx_frame_.size() <= 125) {
        const std::uint64_t id = air_->transmit(node_, {}, mcuemu::RadioProtocol::Zigbee802154, channel(), start,
                                                tx_frame_);
        end = std::max(end, air_->frames()[id - 1].end_ns);
    }
    state_ = State::Tx;
    tx_start_ns_ = start;
    tx_end_ns_ = end;
    tx_sfd_done_ = false;
}

bool Ieee802154::accept(const std::vector<std::uint8_t> &frame, bool &ack_needed) const {
    ack_needed = false;
    if (frame.size() < 3)
        return false;
    const std::uint32_t cfg = reg(r::CTRL_CFG);
    const unsigned fcf = frame[0] | (frame[1] << 8);
    const unsigned type = fcf & 7, dest_mode = (fcf >> 10) & 3, version = (fcf >> 12) & 3;
    if (type == 2) {
        // An ACK nobody waits for is dropped (UnexpectedAck) while the hardware matches ACKs to its own
        // transmissions (HW_AUTO_ACK_RX_EN); with that off the frame reaches software like any other, which
        // is how zigbee-rs, running with it off, sees the ACKs of its frames. Inferred from that driver's
        // behaviour, not documented.
        return !(cfg >> AckRxCtl & 1u);
    }
    if (cfg & (1u << Promiscuous))
        return true;
    if (version == 3 && !(cfg & (1u << DisVersionFilter)))
        return false;
    if (dest_mode == 0)
        return type == 0 || ((type == 1 || type == 3) && (cfg & (1u << PanCoordinator)));
    if (dest_mode == 1)
        return false;
    const std::size_t addr_len = dest_mode == 2 ? 2 : 8;
    if (frame.size() < 5 + addr_len)
        return false;
    const unsigned pan = frame[3] | (frame[4] << 8);
    std::uint64_t address = 0;
    for (std::size_t i = 0; i < addr_len; ++i)
        address |= static_cast<std::uint64_t>(frame[5 + i]) << (8 * i);
    const bool broadcast = dest_mode == 2 && address == 0xffff;
    for (unsigned i = 0; i < 4; ++i) {
        // The SVD puts the interface enables at bits 28..31, but esp-hal writes its mask at bit 29
        // upward ("the REGS are garbage and the struct is right"), which its driver evidently works
        // with; either position enables the interface here (the stray enable of a neighbouring
        // interface only widens the filter).
        if (!(cfg >> (28 + i) & 1u) && !(i < 3 && cfg >> (29 + i) & 1u))
            continue;
        const std::uint32_t base = r::INF0_SHORT_ADDR + i * 0x10;
        const unsigned my_pan = reg(base + 4) & 0xffff;
        if (pan != 0xffff && pan != my_pan)
            continue;
        bool match = broadcast;
        if (dest_mode == 2)
            match = match || address == (reg(base) & 0xffff);
        else
            match = address == (static_cast<std::uint64_t>(reg(base + 12)) << 32 | reg(base + 8));
        if (match) {
            ack_needed = ((fcf >> 5) & 1) && !broadcast && (type == 1 || type == 3) && version <= 1;
            return true;
        }
    }
    return false;
}

void Ieee802154::store(const mcuemu::RadioFrame &frame) {
    const std::size_t length = frame.payload.size() + 2;
    std::vector<std::uint8_t> buffer;
    buffer.reserve(length + 2);
    buffer.push_back(static_cast<std::uint8_t>(length));
    buffer.insert(buffer.end(), frame.payload.begin(), frame.payload.end());
    buffer.push_back(static_cast<std::uint8_t>(static_cast<std::int8_t>(std::clamp(frame.rssi_dbm, -127, 0))));
    buffer.push_back(static_cast<std::uint8_t>(std::clamp((frame.rssi_dbm + 100) * 255 / 60, 0, 255)));
    mem_write_(reg(r::RXDMA_ADDR), buffer.data(), buffer.size());
}

void Ieee802154::deliver(const mcuemu::RadioFrame &frame, bool ack_wanted) {
    store(frame);
    raise(EvRxSfd | EvRxDone);
    if (ack_wanted && (reg(r::CTRL_CFG) & (1u << AckTxCtl))) {
        state_ = State::TxAck;
        ack_due_ = true;
        ack_on_air_ = false;
        ack_at_ns_ = frame.end_ns + AckTurnaroundNs;
        ack_seq_ = frame.payload[2];
    } else {
        state_ = State::Idle;
    }
}

void Ieee802154::send_ack(std::uint64_t at_ns) {
    // The pending bit is read when the ACK goes out, after the driver had its
    // interrupt (hal.rs ack_config_pending_bit sets it in the RX-done handler).
    const std::uint8_t pending = reg(r::ACK_FRAME_PENDING_EN) & 1u;
    std::vector<std::uint8_t> ack{static_cast<std::uint8_t>(0x02 | (pending << 4)), 0x00, ack_seq_};
    std::uint64_t end = at_ns + mcuemu::VirtualAir::airtime_ns(mcuemu::RadioProtocol::Zigbee802154, 5);
    if (air_) {
        const std::uint64_t id = air_->transmit(node_, {}, mcuemu::RadioProtocol::Zigbee802154, channel(), at_ns,
                                                std::move(ack));
        end = std::max(end, air_->frames()[id - 1].end_ns);
    }
    ack_on_air_ = true;
    ack_end_ns_ = end;
}

void Ieee802154::poll_air(std::uint64_t ns) {
    if (!air_ || (state_ != State::Rx && state_ != State::RxAck))
        return;
    const std::uint64_t after = last_poll_ns_;
    last_poll_ns_ = ns;
    for (const mcuemu::RadioFrame &frame :
         air_->receive(node_, mcuemu::RadioProtocol::Zigbee802154, channel(), ns, after)) {
        if (state_ != State::Rx && state_ != State::RxAck)
            break;
        const std::uint64_t listening = state_ == State::RxAck ? tx_end_ns_ : rx_start_ns_;
        if (frame.start_ns + SfdNs < listening || frame.payload.size() < 3)
            continue; // the preamble went by before the receiver was on
        const bool is_ack = (frame.payload[0] & 7) == 2;
        if (state_ == State::RxAck) {
            if (is_ack && frame.payload.size() == 3 && tx_frame_.size() >= 3 && frame.payload[2] == tx_frame_[2]) {
                // The buffer driven by RXDMA_ADDR now holds the ACK (the driver points it
                // at its ACK buffer in tx_init).
                store(frame);
                raise(EvRxSfd | EvAckRxDone);
                state_ = State::Idle;
            }
            continue;
        }
        if (is_ack && (reg(r::CTRL_CFG) >> AckRxCtl & 1u)) {
            rx_abort(8); // unexpected ACK
            continue;
        }
        bool ack_wanted = false;
        if (!accept(frame.payload, ack_wanted)) {
            rx_abort(5); // filter fail
            continue;
        }
        deliver(frame, ack_wanted);
    }
}

void Ieee802154::timers(std::uint64_t ns) {
    for (unsigned i = 0; i < 2; ++i) {
        if (!timer_running_[i])
            continue;
        const std::uint64_t threshold = reg(i == 0 ? r::TIME0_THRESHOLD : r::TIME1_THRESHOLD);
        if ((ns - timer_start_ns_[i]) / 1000 >= threshold) {
            timer_running_[i] = false;
            raise(1u << (8 + i));
        }
    }
}

std::uint64_t Ieee802154::service(std::uint64_t now) {
    const std::uint64_t ns = to_ns(now);
    if (state_ == State::Tx) {
        if (!tx_sfd_done_ && ns >= tx_start_ns_ + SfdNs) {
            tx_sfd_done_ = true;
            raise(EvTxSfd);
        }
        if (ns >= tx_end_ns_) {
            if (!tx_sfd_done_)
                raise(EvTxSfd);
            raise(EvTxDone);
            const bool ack_required = !tx_frame_.empty() && (tx_frame_[0] & 0x20);
            if (ack_required && (reg(r::CTRL_CFG) & (1u << AckRxCtl))) {
                state_ = State::RxAck;
                last_poll_ns_ = tx_end_ns_;
            } else {
                state_ = State::Idle;
            }
        }
    }
    if (state_ == State::TxAck) {
        if (!ack_on_air_ && ns >= ack_at_ns_)
            send_ack(ack_at_ns_);
        if (ack_on_air_ && ns >= ack_end_ns_) {
            ack_due_ = ack_on_air_ = false;
            state_ = State::Idle;
            raise(EvAckTxDone);
        }
    }
    if (state_ == State::Ed && ns >= ed_end_ns_) {
        state_ = State::Idle;
        raise(EvEdDone);
    }
    poll_air(ns);
    timers(ns);

    std::uint64_t next_ns = ~0ull;
    auto want = [&](std::uint64_t t) { next_ns = std::min(next_ns, std::max(t, ns + 1000)); };
    if (state_ == State::Tx)
        want(tx_sfd_done_ ? tx_end_ns_ : tx_start_ns_ + SfdNs);
    if (state_ == State::TxAck)
        want(ack_on_air_ ? ack_end_ns_ : ack_at_ns_);
    if (state_ == State::Ed)
        want(ed_end_ns_);
    if ((state_ == State::Rx || state_ == State::RxAck) && air_)
        want(ns + PollUs * 1000);
    for (unsigned i = 0; i < 2; ++i)
        if (timer_running_[i])
            want(timer_start_ns_[i] + std::uint64_t(reg(i == 0 ? r::TIME0_THRESHOLD : r::TIME1_THRESHOLD)) * 1000);
    return next_ns == ~0ull ? NoEvent : to_cycles(next_ns);
}

} // namespace esp32emu
