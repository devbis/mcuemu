// nRF52840 RADIO in IEEE 802.15.4 mode. See peripherals.hpp for what is
// documented and what is inferred.

#include "nrf52/peripherals.hpp"

#include <algorithm>
#include <cstring>
#include <ostream>

namespace nrf52emu {

namespace {

// Event indices double as INTEN bit positions.
enum Event : unsigned {
    Ready = 0, Address = 1, Payload = 2, End = 3, Disabled = 4, DevMatch = 5, DevMiss = 6,
    RssiEnd = 7, BcMatch = 10, CrcOk = 12, CrcError = 13, FrameStart = 14, EdEnd = 15,
    EdStopped = 16, CcaIdle = 17, CcaBusy = 18, CcaStopped = 19, RateBoost = 20, TxReady = 21,
    RxReady = 22, MhrMatch = 23, Sync = 26, PhyEnd = 27,
};

struct EventRegister {
    unsigned index;
    std::uint32_t offset;
};
constexpr EventRegister event_registers[] = {
    {Ready, 0x100}, {Address, 0x104}, {Payload, 0x108}, {End, 0x10c}, {Disabled, 0x110},
    {DevMatch, 0x114}, {DevMiss, 0x118}, {RssiEnd, 0x11c}, {BcMatch, 0x128}, {CrcOk, 0x130},
    {CrcError, 0x134}, {FrameStart, 0x138}, {EdEnd, 0x13c}, {EdStopped, 0x140}, {CcaIdle, 0x144},
    {CcaBusy, 0x148}, {CcaStopped, 0x14c}, {RateBoost, 0x150}, {TxReady, 0x154}, {RxReady, 0x158},
    {MhrMatch, 0x15c}, {Sync, 0x168}, {PhyEnd, 0x16c},
};

constexpr std::uint64_t DisableUs = 6, PollUs = 16, CcaUs = 128, EdUs = 128, SfdNs = 128000;
// Preamble (4 bytes) and SFD (1 byte): SYNC and ADDRESS come this long after the frame starts (FRAMESTART
// follows the PHR 32 us later; folded in). The driver's ACK timeout is stopped by ADDRESS, so it must not
// wait for the frame end.
constexpr std::uint64_t AddressNs = 160000;
constexpr std::uint32_t ModeIeee802154 = 15;

std::uint16_t fcs16(const std::uint8_t *data, std::size_t length) {
    // IEEE 802.15.4 FCS: CRC-16/KERMIT (x^16 + x^12 + x^5 + 1, LSB first, init 0).
    std::uint16_t crc = 0;
    for (std::size_t i = 0; i != length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit != 8; ++bit)
            crc = static_cast<std::uint16_t>((crc & 1u) ? (crc >> 1) ^ 0x8408u : crc >> 1);
    }
    return crc;
}

} // namespace

std::uint64_t Radio::to_ns(std::uint64_t cycles) const { return chip_.core().cycles_to_ns(cycles); }

std::uint64_t Radio::ramp_cycles() const {
    const std::uint64_t us = (modecnf0_ & 1u) ? 40 : 130;
    return us * (clock_hz() / 1000000u);
}

std::uint8_t Radio::channel() const {
    const std::uint32_t mhz = frequency_ & 0x7fu;
    if (mhz < 5)
        return 11;
    return static_cast<std::uint8_t>(std::min<std::uint32_t>(11 + (mhz - 5) / 5, 26));
}

void Radio::reset() {
    state_ = State::Disabled;
    events_ = inten_ = shorts_ = 0;
    packetptr_ = frequency_ = txpower_ = mode_ = pcnf0_ = pcnf1_ = 0;
    crccnf_ = crcpoly_ = crcinit_ = modecnf0_ = 0;
    sfd_ = 0xa7;
    edcnt_ = edsample_ = ccactrl_ = crcstatus_ = rxcrc_ = rssisample_ = 0;
    tifs_ = bcc_ = 0;
    power_ = 1;
    phase_end_ = cca_end_ = ed_end_ = rssi_end_ = Never;
    tx_active_ = cca_active_ = ed_active_ = false;
    update_irq();
}

void Radio::update_irq() { set_irq_level((events_ & inten_) != 0); }

void Radio::raise(unsigned index) {
    events_ |= 1u << index;
    for (const EventRegister &entry : event_registers)
        if (entry.index == index)
            event_raised(entry.offset);
    update_irq();
    apply_shorts(index);
}

void Radio::apply_shorts(unsigned index) {
    auto on = [&](unsigned bit) { return (shorts_ >> bit) & 1u; };
    switch (index) {
    case Ready:
        if (on(0)) start();
        if (on(15)) start_ed();
        break;
    case End:
        if (on(1)) disable();
        if (on(5)) start();
        break;
    case Disabled:
        if (on(2)) enable(true);
        if (on(3)) enable(false);
        break;
    case CcaIdle:
        if (on(12)) enable(true);
        if (on(17)) { cca_active_ = false; }
        break;
    case CcaBusy:
        if (on(13)) disable();
        break;
    case EdEnd:
        if (on(16)) disable();
        break;
    case TxReady:
        if (on(18)) start();
        break;
    case RxReady:
        if (on(11)) start_cca();
        if (on(19)) start();
        break;
    case PhyEnd:
        if (on(20)) disable();
        if (on(21)) start();
        break;
    default: break;
    }
}

void Radio::enable(bool transmit) {
    const std::uint64_t t = now();
    if (state_ == State::Disabled || state_ == State::RxIdle || state_ == State::Rx ||
        state_ == State::TxIdle || state_ == State::RxDisable || state_ == State::TxDisable) {
        cca_active_ = ed_active_ = false;
        cca_end_ = ed_end_ = Never;
        state_ = transmit ? State::TxRu : State::RxRu;
        phase_end_ = t + ramp_cycles();
        request_service(phase_end_);
    }
}

void Radio::disable() {
    const std::uint64_t t = now();
    cca_active_ = ed_active_ = false;
    cca_end_ = ed_end_ = Never;
    switch (state_) {
    case State::Disabled:
        raise(Disabled);
        return;
    case State::RxDisable:
    case State::TxDisable:
        return;
    case State::TxRu:
    case State::TxIdle:
    case State::Tx:
        state_ = State::TxDisable;
        break;
    default:
        state_ = State::RxDisable;
        break;
    }
    phase_end_ = t + DisableUs * (clock_hz() / 1000000u);
    request_service(phase_end_);
}

std::uint8_t Radio::energy_at(std::uint64_t from_ns, std::uint64_t to_ns, bool) const {
    if (!air_)
        return 0;
    int best = 0;
    const auto &frames = air_->frames();
    const bool ideal = air_->propagation().model == mcuemu::Propagation::Model::Ideal;
    for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
        if (it->end_ns + 20000000ull < from_ns)
            break;
        if (it->protocol != mcuemu::RadioProtocol::Zigbee802154 || it->channel != channel() ||
            it->source == node_ || it->start_ns >= to_ns || it->end_ns <= from_ns)
            continue;
        const double rssi = ideal ? -50.0 : it->tx_power_dbm - air_->path_loss_db(it->source, node_);
        best = std::max(best, std::clamp(static_cast<int>(rssi) + 92, 0, 63));
    }
    return static_cast<std::uint8_t>(best);
}

void Radio::start_cca() {
    if (state_ != State::RxIdle && state_ != State::Rx)
        return;
    cca_active_ = true;
    cca_start_ns_ = to_ns(now());
    cca_end_ = now() + CcaUs * (clock_hz() / 1000000u);
    request_service(cca_end_);
}

void Radio::finish_cca(std::uint64_t ns) {
    cca_active_ = false;
    cca_end_ = Never;
    const std::uint32_t mode = ccactrl_ & 7u;
    const std::uint8_t threshold = static_cast<std::uint8_t>((ccactrl_ >> 8) & 0xffu);
    const std::uint8_t energy = energy_at(cca_start_ns_, ns, false);
    bool busy = false;
    if (mode == 1 || mode == 3)
        busy = energy != 0; // carrier sense: any frame on the channel
    else
        busy = energy != 0 && energy >= threshold;
    raise(busy ? CcaBusy : CcaIdle);
}

void Radio::start_ed() {
    if (state_ != State::RxIdle && state_ != State::Rx)
        return;
    ed_active_ = true;
    cca_start_ns_ = to_ns(now());
    ed_end_ = now() + (static_cast<std::uint64_t>(edcnt_) + 1) * EdUs * (clock_hz() / 1000000u);
    request_service(ed_end_);
}

void Radio::start() {
    const std::uint64_t t = now();
    if (state_ == State::RxIdle) {
        state_ = State::Rx;
        rx_start_ns_ = last_poll_ns_ = to_ns(t);
        rx_lock_id_ = 0;
        if (chip_.trace())
            *chip_.trace() << "RADIO listening from " << rx_start_ns_ << " ch " << int(channel()) << '\n';
        request_service(t + PollUs * (clock_hz() / 1000000u));
    } else if (state_ == State::TxIdle) {
        begin_transmit(to_ns(t));
    }
}

void Radio::begin_transmit(std::uint64_t ns) {
    state_ = State::Tx;
    tx_active_ = true;
    std::vector<std::uint8_t> payload;
    if (packetptr_ >= Nrf52840::RamBase && packetptr_ + 1 <= Nrf52840::RamBase + Nrf52840::RamSize) {
        const std::uint8_t *buffer = chip_.ram() + (packetptr_ - Nrf52840::RamBase);
        const std::size_t room = Nrf52840::RamBase + Nrf52840::RamSize - packetptr_ - 1;
        const std::size_t length = buffer[0];
        // PHR counts the two FCS bytes the radio appends itself.
        const std::size_t mac = length >= 2 ? std::min(length - 2, room) : 0;
        payload.assign(buffer + 1, buffer + 1 + mac);
    }
    std::uint64_t end = ns + mcuemu::VirtualAir::airtime_ns(mcuemu::RadioProtocol::Zigbee802154,
                                                            payload.size() + 2);
    if (air_ && mode_ == ModeIeee802154 && !payload.empty() && payload.size() <= 125) {
        // The medium may answer with an ACK frame of its own, so look the frame
        // up by id instead of taking the last one.
        const std::uint64_t id = air_->transmit(node_, {}, mcuemu::RadioProtocol::Zigbee802154,
                                                channel(), ns, std::move(payload));
        end = std::max(end, air_->frames()[id - 1].end_ns);
    }
    tx_end_ns_ = end;
    raise(Address);
    raise(FrameStart);
    request_service(chip_.core().ns_to_cycles(tx_end_ns_));
}

void Radio::finish_transmit() {
    tx_active_ = false;
    state_ = State::TxIdle;
    raise(Payload);
    raise(End);
    raise(PhyEnd);
}

void Radio::deliver(const mcuemu::RadioFrame &frame, std::uint64_t) {
    const std::uint32_t address = packetptr_;
    const std::size_t length = frame.payload.size() + 2;
    if (address < Nrf52840::RamBase || length + 3 > Nrf52840::RamBase + Nrf52840::RamSize - address ||
        frame.payload.size() > 125)
        return;
    std::uint8_t *buffer = chip_.ram() + (address - Nrf52840::RamBase);
    buffer[0] = static_cast<std::uint8_t>(length);
    std::memcpy(buffer + 1, frame.payload.data(), frame.payload.size());
    const std::uint16_t crc = fcs16(frame.payload.data(), frame.payload.size());
    buffer[length - 1] = static_cast<std::uint8_t>(crc & 0xffu);
    buffer[length] = static_cast<std::uint8_t>(crc >> 8);
    buffer[length + 1] = static_cast<std::uint8_t>(std::clamp(frame.rssi_dbm + 92, 0, 63));
    crcstatus_ = 1;
    rxcrc_ = crc;
    state_ = State::RxIdle;
    // poll_air() raised these when the frame's preamble went by.
    if (rx_lock_id_ != frame.id) {
        raise(Sync);
        raise(Address);
        raise(FrameStart);
    }
    rx_lock_id_ = 0;
    raise(Payload);
    raise(End);
    raise(CrcOk);
    raise(PhyEnd);
}

void Radio::poll_air(std::uint64_t ns) {
    if (!air_ || mode_ != ModeIeee802154)
        return;
    const std::uint64_t after = last_poll_ns_;
    last_poll_ns_ = ns;
    if (rx_lock_id_ == 0) {
        for (const mcuemu::RadioFrame &frame :
             air_->in_flight(node_, mcuemu::RadioProtocol::Zigbee802154, channel(), ns, AddressNs)) {
            if (frame.start_ns + SfdNs < rx_start_ns_)
                continue;
            rx_lock_id_ = frame.id;
            rx_lock_end_ns_ = frame.end_ns;
            raise(Sync);
            raise(Address);
            raise(FrameStart);
            break;
        }
    }
    for (const mcuemu::RadioFrame &frame :
         air_->receive(node_, mcuemu::RadioProtocol::Zigbee802154, channel(), ns, after)) {
        if (chip_.trace())
            *chip_.trace() << "RADIO rx frame start=" << frame.start_ns << " end=" << frame.end_ns
                           << " rx_start=" << rx_start_ns_ << " len=" << frame.payload.size()
                           << (frame.start_ns + SfdNs < rx_start_ns_ ? " (missed: before rx start)" : "") << '\n';
        // The receiver locks onto the preamble, so it can still sync when it
        // starts listening before the start-of-frame delimiter (4 preamble
        // bytes = 128 us in). Inferred, not documented.
        if (frame.start_ns + SfdNs < rx_start_ns_)
            continue;
        deliver(frame, ns);
        break;
    }
    if (rx_lock_id_ != 0 && ns >= rx_lock_end_ns_)
        rx_lock_id_ = 0; // the frame ended without being received (collision)
}

std::uint64_t Radio::reschedule() {
    std::uint64_t next = std::min({phase_end_, cca_end_, ed_end_, rssi_end_});
    if (tx_active_)
        next = std::min(next, chip_.core().ns_to_cycles(tx_end_ns_));
    if (state_ == State::Rx && air_ && mode_ == ModeIeee802154)
        next = std::min(next, now() + PollUs * (clock_hz() / 1000000u));
    return next;
}

std::uint64_t Radio::service(std::uint64_t t) {
    const std::uint64_t ns = to_ns(t);
    for (int guard = 0; guard != 16; ++guard) {
        bool progressed = false;
        if (phase_end_ <= t) {
            phase_end_ = Never;
            progressed = true;
            switch (state_) {
            case State::RxRu:
                state_ = State::RxIdle;
                raise(Ready);
                raise(RxReady);
                break;
            case State::TxRu:
                state_ = State::TxIdle;
                raise(Ready);
                raise(TxReady);
                break;
            case State::RxDisable:
            case State::TxDisable:
                state_ = State::Disabled;
                raise(Disabled);
                break;
            default: break;
            }
        }
        if (tx_active_ && ns >= tx_end_ns_) {
            finish_transmit();
            progressed = true;
        }
        if (cca_active_ && cca_end_ <= t) {
            finish_cca(ns);
            progressed = true;
        }
        if (ed_active_ && ed_end_ <= t) {
            ed_active_ = false;
            ed_end_ = Never;
            edsample_ = energy_at(cca_start_ns_, ns, true);
            raise(EdEnd);
            progressed = true;
        }
        if (rssi_end_ <= t) {
            rssi_end_ = Never;
            const std::uint8_t energy = energy_at(ns > 128000 ? ns - 128000 : 0, ns, true);
            rssisample_ = energy == 0 ? 100 : static_cast<std::uint32_t>(92 - std::min<int>(energy, 63) + 0);
            raise(RssiEnd);
            progressed = true;
        }
        if (!progressed)
            break;
    }
    if (state_ == State::Rx)
        poll_air(ns);
    return reschedule();
}

void Radio::task(std::uint32_t offset) {
    switch (offset) {
    case 0x000: enable(true); break;
    case 0x004: enable(false); break;
    case 0x008: start(); break;
    case 0x00c:
        if (state_ == State::Rx)
            state_ = State::RxIdle;
        break;
    case 0x010: disable(); break;
    case 0x014:
        rssi_end_ = now() + 8u * (clock_hz() / 1000000u);
        request_service(rssi_end_);
        break;
    case 0x018: rssi_end_ = Never; break;
    case 0x024: start_ed(); break;
    case 0x028:
        if (ed_active_) {
            ed_active_ = false;
            ed_end_ = Never;
            raise(EdStopped);
        }
        break;
    case 0x02c: start_cca(); break;
    case 0x030:
        if (cca_active_) {
            cca_active_ = false;
            cca_end_ = Never;
            raise(CcaStopped);
        }
        break;
    default: break;
    }
    request_service(reschedule());
}

std::uint32_t Radio::read(std::uint32_t offset) {
    for (const EventRegister &entry : event_registers)
        if (entry.offset == offset)
            return (events_ >> entry.index) & 1u;
    switch (offset) {
    case 0x200: return shorts_;
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x400: return crcstatus_;
    case 0x40c: return rxcrc_;
    case 0x504: return packetptr_;
    case 0x508: return frequency_;
    case 0x50c: return txpower_;
    case 0x510: return mode_;
    case 0x514: return pcnf0_;
    case 0x518: return pcnf1_;
    case 0x534: return crccnf_;
    case 0x538: return crcpoly_;
    case 0x53c: return crcinit_;
    case 0x544: return tifs_;
    case 0x548: return rssisample_;
    case 0x550: return static_cast<std::uint32_t>(state_);
    case 0x560: return bcc_;
    case 0x650: return modecnf0_;
    case 0x660: return sfd_;
    case 0x664: return edcnt_;
    case 0x668: return edsample_;
    case 0x66c: return ccactrl_;
    case 0xffc: return power_;
    default: return 0;
    }
}

void Radio::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x100) {
        if (value & 1u)
            task(offset);
        return;
    }
    for (const EventRegister &entry : event_registers) {
        if (entry.offset == offset) {
            if (value & 1u)
                events_ |= 1u << entry.index;
            else
                events_ &= ~(1u << entry.index);
            update_irq();
            return;
        }
    }
    switch (offset) {
    case 0x200: shorts_ = value; break;
    case 0x304: inten_ |= value; update_irq(); break;
    case 0x308: inten_ &= ~value; update_irq(); break;
    case 0x504: packetptr_ = value; break;
    case 0x508: frequency_ = value; break;
    case 0x50c: txpower_ = value; break;
    case 0x510: mode_ = value; break;
    case 0x514: pcnf0_ = value; break;
    case 0x518: pcnf1_ = value; break;
    case 0x534: crccnf_ = value; break;
    case 0x538: crcpoly_ = value; break;
    case 0x53c: crcinit_ = value; break;
    case 0x544: tifs_ = value; break;
    case 0x560: bcc_ = value; break;
    case 0x650: modecnf0_ = value; break;
    case 0x660: sfd_ = value; break;
    case 0x664: edcnt_ = value; break;
    case 0x66c: ccactrl_ = value; break;
    case 0xffc:
        if (!(value & 1u)) {
            reset();
            power_ = 0;
        } else {
            power_ = 1;
        }
        break;
    default: break;
    }
}

} // namespace nrf52emu
