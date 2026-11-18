// nRF52840 RADIO in IEEE 802.15.4 and Bluetooth LE (1M/2M) modes. See
// peripherals.hpp for what is documented and what is inferred.

#include "nrf52/peripherals.hpp"

#include "ble/ll.hpp"

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
    base_[0] = base_[1] = prefix_[0] = prefix_[1] = txaddress_ = rxaddresses_ = 0;
    datawhiteiv_ = 0x40;
    std::fill(std::begin(dab_), std::end(dab_), 0u);
    std::fill(std::begin(dap_), std::end(dap_), 0u);
    dacnf_ = rxmatch_ = dai_ = 0;
    clear_schedule();
    rx_locked_ = last_end_valid_ = false;
    last_end_ns_ = 0;
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
        if (on(8) && ble_mode()) task(0x018); // DISABLED_RSSISTOP
        break;
    case Address:
        // ADDRESS_RSSISTART / ADDRESS_BCSTART. Only in BLE modes: the 802.15.4
        // path never honoured them and its driver timing depends on that.
        if (ble_mode()) {
            if (on(4)) task(0x014);
            if (on(6)) task(0x01c);
        }
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
    // A packet in progress is abandoned (a transmitted one stays on the air).
    clear_schedule();
    rx_locked_ = false;
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
        if (ble_mode())
            start_ble_tx();
        else
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
    if (!air_)
        return;
    if (ble_mode()) {
        poll_air_ble(ns);
        return;
    }
    if (mode_ != ModeIeee802154)
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
    if (!timeline_.empty())
        next = std::min(next, timeline_.front().cycle);
    if (state_ == State::Rx && air_ &&
        (mode_ == ModeIeee802154 || (ble_mode() && !rx_locked_)))
        next = std::min(next, now() + PollUs * (clock_hz() / 1000000u));
    return next;
}

std::uint64_t Radio::service(std::uint64_t t) {
    const std::uint64_t ns = to_ns(t);
    // Two passes: the air poll can schedule actions that are already due.
    for (int pass = 0; pass != 2; ++pass) {
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
            if (!timeline_.empty() && timeline_.front().cycle <= t) {
                // Pop first: the action may disable the radio and clear the timeline.
                auto action = std::move(timeline_.front().action);
                timeline_.erase(timeline_.begin());
                action();
                progressed = true;
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
                if (ble_mode()) {
                    rssisample_ = rx_locked_ ? static_cast<std::uint32_t>(std::clamp(-rx_rssi_dbm_, 0, 127)) : 100u;
                } else {
                    const std::uint8_t energy = energy_at(ns > 128000 ? ns - 128000 : 0, ns, true);
                    rssisample_ = energy == 0 ? 100 : static_cast<std::uint32_t>(92 - std::min<int>(energy, 63) + 0);
                }
                raise(RssiEnd);
                progressed = true;
            }
            if (!progressed)
                break;
        }
        if (pass == 0 && state_ == State::Rx)
            poll_air(ns);
    }
    return reschedule();
}

void Radio::task(std::uint32_t offset) {
    switch (offset) {
    case 0x000: enable(true); break;
    case 0x004: enable(false); break;
    case 0x008: start(); break;
    case 0x00c:
        if (state_ == State::Rx) {
            state_ = State::RxIdle;
            clear_schedule();
            rx_locked_ = false;
        }
        break;
    case 0x010: disable(); break;
    case 0x014:
        rssi_end_ = now() + 8u * (clock_hz() / 1000000u);
        request_service(rssi_end_);
        break;
    case 0x018: rssi_end_ = Never; break;
    case 0x01c:
        if (ble_mode())
            start_bit_counter();
        break;
    case 0x020: ++bc_generation_; break; // BCSTOP
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
    case 0x408: return rxmatch_;
    case 0x40c: return rxcrc_;
    case 0x410: return dai_;
    case 0x504: return packetptr_;
    case 0x508: return frequency_;
    case 0x50c: return txpower_;
    case 0x510: return mode_;
    case 0x514: return pcnf0_;
    case 0x518: return pcnf1_;
    case 0x51c: return base_[0];
    case 0x520: return base_[1];
    case 0x524: return prefix_[0];
    case 0x528: return prefix_[1];
    case 0x52c: return txaddress_;
    case 0x530: return rxaddresses_;
    case 0x534: return crccnf_;
    case 0x538: return crcpoly_;
    case 0x53c: return crcinit_;
    case 0x544: return tifs_;
    case 0x548: return rssisample_;
    case 0x550: return static_cast<std::uint32_t>(state_);
    case 0x554: return datawhiteiv_;
    case 0x560: return bcc_;
    case 0x640: return dacnf_;
    case 0x650: return modecnf0_;
    case 0x660: return sfd_;
    case 0x664: return edcnt_;
    case 0x668: return edsample_;
    case 0x66c: return ccactrl_;
    case 0xffc: return power_;
    default:
        if (offset >= 0x600 && offset < 0x620)
            return dab_[(offset - 0x600) / 4];
        if (offset >= 0x620 && offset < 0x640)
            return dap_[(offset - 0x620) / 4];
        return 0;
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
    case 0x51c: base_[0] = value; break;
    case 0x520: base_[1] = value; break;
    case 0x524: prefix_[0] = value; break;
    case 0x528: prefix_[1] = value; break;
    case 0x52c: txaddress_ = value & 7u; break;
    case 0x530: rxaddresses_ = value & 0xffu; break;
    case 0x534: crccnf_ = value; break;
    case 0x538: crcpoly_ = value; break;
    case 0x53c: crcinit_ = value; break;
    case 0x544: tifs_ = value; break;
    case 0x554: datawhiteiv_ = value & 0x7fu; break;
    case 0x560: bcc_ = value; break;
    case 0x640: dacnf_ = value & 0xffffu; break;
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
    default:
        if (offset >= 0x600 && offset < 0x620)
            dab_[(offset - 0x600) / 4] = value;
        else if (offset >= 0x620 && offset < 0x640)
            dap_[(offset - 0x620) / 4] = value & 0xffffu;
        break;
    }
}

// ---- Bluetooth LE ------------------------------------------------------------
//
// Documented (nRF52840 PS, RADIO): MODE 3/4, FREQUENCY (offset from 2400 MHz),
// the packet layout registers (PCNF0/PCNF1: S0, LENGTH, S1, MAXLEN, STATLEN),
// BASEn/PREFIXn/TXADDRESS/RXADDRESSES, CRCCNF/CRCINIT, TIFS, BCC, DAB/DAP/DACNF
// and the events ADDRESS (end of the access address), PAYLOAD (end of the
// payload), END (end of the CRC), CRCOK/CRCERROR, DEVMATCH/DEVMISS, BCMATCH.
// Inferred: the TIFS rule below (a START that would begin sooner than TIFS after
// the end of the previous packet waits until then), the device-address compare
// instant, the 8 us RSSI sample time, and that whitening is not modelled (the
// medium carries de-whitened PDUs; a whitening mismatch between two radios is
// not reproduced). Only BALEN 3 (a 32-bit access address) is meaningful, S0LEN
// is 0 or 1 byte, LFLEN at most 8 bits, S1 is skipped, and the long range PHYs
// (MODE 5/6) only run the state machine.

namespace {
constexpr unsigned BleCrcBits = 24;
} // namespace

int Radio::ble_channel() const {
    if (frequency_ & 0x100u)
        return -1; // MAP = 1 (2360 MHz base) is not a BLE channel plan
    const unsigned mhz = 2400 + (frequency_ & 0x7fu);
    if (mhz == 2402) return 37;
    if (mhz == 2426) return 38;
    if (mhz == 2480) return 39;
    if (mhz >= 2404 && mhz <= 2424 && !(mhz & 1u)) return static_cast<int>((mhz - 2404) / 2);
    if (mhz >= 2428 && mhz <= 2478 && !(mhz & 1u)) return static_cast<int>(11 + (mhz - 2428) / 2);
    return -1;
}

std::uint64_t Radio::ble_address_ns() const {
    const std::uint64_t preamble_bytes = mode_ == 4 ? 2 : 1;
    return (preamble_bytes + 4) * 8 * ble_bit_ns();
}

// Logical address n is one prefix byte (PREFIXn) plus the base: for the usual
// BALEN = 3 the access address is prefix << 24 | base >> 8.
std::uint32_t Radio::logical_access_address(unsigned logical) const {
    const std::uint32_t prefixes = prefix_[(logical >> 2) & 1u];
    const std::uint32_t prefix = (prefixes >> (8 * (logical & 3u))) & 0xffu;
    return prefix << 24 | base_[(logical >> 2) & 1u] >> 8;
}

int Radio::match_access_address(std::uint32_t access_address) const {
    for (unsigned logical = 0; logical != 8; ++logical)
        if ((rxaddresses_ >> logical & 1u) && logical_access_address(logical) == access_address)
            return static_cast<int>(logical);
    return -1;
}

std::uint64_t Radio::cycle_for_ns(std::uint64_t ns) const {
    std::uint64_t cycle = chip_.core().ns_to_cycles(ns);
    while (to_ns(cycle) < ns)
        ++cycle;
    return cycle;
}

void Radio::schedule(std::uint64_t ns, std::function<void()> action) {
    Timed item{cycle_for_ns(ns), ns, timeline_seq_++, std::move(action)};
    const auto at = std::upper_bound(timeline_.begin(), timeline_.end(), item,
                                     [](const Timed &a, const Timed &b) {
                                         return a.ns != b.ns ? a.ns < b.ns : a.seq < b.seq;
                                     });
    const std::uint64_t cycle = item.cycle;
    timeline_.insert(at, std::move(item));
    request_service(cycle);
}

void Radio::clear_schedule() {
    timeline_.clear();
    ++bc_generation_;
}

// RAM <-> PDU. The PDU on the medium is always S0 | LENGTH | payload; the
// register layout decides where those fields sit in RAM.
void Radio::begin_transmit_ble(std::uint64_t ns) {
    state_ = State::Tx;
    const unsigned s0len = pcnf0_ >> 8 & 1u, lflen = pcnf0_ & 0xfu;
    const bool s1 = (pcnf0_ >> 16 & 0xfu) != 0 || (pcnf0_ >> 20 & 1u);
    const std::size_t maxlen = pcnf1_ & 0xffu, statlen = pcnf1_ >> 8 & 0xffu;
    std::vector<std::uint8_t> pdu{0, 0};
    if (packetptr_ >= Nrf52840::RamBase && packetptr_ < Nrf52840::RamBase + Nrf52840::RamSize) {
        const std::uint8_t *buffer = chip_.ram() + (packetptr_ - Nrf52840::RamBase);
        const std::size_t room = Nrf52840::RamBase + Nrf52840::RamSize - packetptr_;
        std::size_t pos = 0;
        if (s0len && pos < room)
            pdu[0] = buffer[pos++];
        std::size_t length = 0;
        if (lflen && pos < room)
            length = buffer[pos++] & (lflen >= 8 ? 0xffu : (1u << lflen) - 1u);
        if (s1)
            ++pos;
        const std::size_t count = std::min(length, maxlen) + statlen;
        const std::size_t available = pos < room ? room - pos : 0;
        pdu[1] = static_cast<std::uint8_t>(lflen ? std::min(length, maxlen) : count);
        pdu.insert(pdu.end(), buffer + pos, buffer + pos + std::min(count, available));
    }
    const std::uint64_t bit = ble_bit_ns();
    std::uint64_t end = ns + mcuemu::VirtualAir::airtime_ns(mcuemu::RadioProtocol::Ble, pdu.size(),
                                                           mode_ == 4 ? 2000000u : 0u, true);
    const int channel = ble_channel();
    if (air_ && channel >= 0) {
        mcuemu::BleLinkInfo link{logical_access_address(txaddress_), crcinit_ & 0xffffffu, true};
        const std::uint64_t id = air_->transmit(node_, {}, mcuemu::RadioProtocol::Ble,
                                                static_cast<std::uint8_t>(channel), ns, std::move(pdu),
                                                mode_ == 4 ? 2000000u : 0u, link);
        end = air_->frames()[id - 1].end_ns;
    }
    const std::uint64_t address_at = ns + ble_address_ns();
    const std::uint64_t payload_at = std::max(address_at, end - BleCrcBits * bit);
    schedule(address_at, [this] { raise(Address); });
    schedule(payload_at, [this] { raise(Payload); });
    schedule(end, [this, end] {
        if (state_ == State::Tx)
            state_ = State::TxIdle;
        last_end_ns_ = end;
        last_end_valid_ = true;
        ++bc_generation_;
        raise(End);
    });
}

// TIFS: when the previous packet ended less than TIFS ago, the START waits so the
// new packet begins exactly TIFS after it.
void Radio::start_ble_tx() {
    const std::uint64_t ns = to_ns(now());
    std::uint64_t target = ns;
    if (tifs_ != 0 && last_end_valid_ && last_end_ns_ + tifs_ * 1000ull > ns)
        target = last_end_ns_ + tifs_ * 1000ull;
    if (target <= ns) {
        begin_transmit_ble(ns);
        return;
    }
    schedule(target, [this, target] {
        if (state_ == State::TxIdle)
            begin_transmit_ble(target);
    });
}

void Radio::poll_air_ble(std::uint64_t ns) {
    const int channel = ble_channel();
    if (rx_locked_ || channel < 0)
        return;
    for (const mcuemu::RadioFrame &frame : air_->in_flight(node_, mcuemu::RadioProtocol::Ble,
                                                           static_cast<std::uint8_t>(channel), ns, 0)) {
        // The receiver locks onto the preamble (8 bits); one that starts
        // listening later than that misses the frame.
        if (!frame.ble.ll_framed || frame.start_ns + 8 * ble_bit_ns() < rx_start_ns_)
            continue;
        const int logical = match_access_address(frame.ble.access_address);
        if (logical < 0)
            continue;
        rx_locked_ = true;
        rxmatch_ = static_cast<std::uint32_t>(logical);
        rx_rssi_dbm_ = frame.rssi_dbm;
        rx_dma_ptr_ = packetptr_;
        const std::uint64_t bit = ble_bit_ns();
        const std::uint64_t address_at = frame.start_ns + ble_address_ns();
        const std::uint64_t payload_at = std::max(address_at, frame.end_ns - BleCrcBits * bit);
        schedule(address_at, [this] { raise(Sync); raise(Address); });
        if (dacnf_ & 0xffu)
            schedule(address_at + 64 * bit, [this, frame] { ble_device_match(frame); });
        schedule(payload_at, [this, frame] { ble_rx_payload(frame); });
        schedule(frame.end_ns, [this, frame] { ble_rx_end(frame); });
        break;
    }
}

void Radio::ble_rx_payload(const mcuemu::RadioFrame &frame) {
    const unsigned s0len = pcnf0_ >> 8 & 1u, lflen = pcnf0_ & 0xfu;
    const bool s1 = (pcnf0_ >> 16 & 0xfu) != 0 || (pcnf0_ >> 20 & 1u);
    const std::size_t maxlen = pcnf1_ & 0xffu, statlen = pcnf1_ >> 8 & 0xffu;
    const std::vector<std::uint8_t> &pdu = frame.payload;
    if (pdu.size() >= 2 && rx_dma_ptr_ >= Nrf52840::RamBase &&
        rx_dma_ptr_ < Nrf52840::RamBase + Nrf52840::RamSize) {
        std::uint8_t *buffer = chip_.ram() + (rx_dma_ptr_ - Nrf52840::RamBase);
        const std::size_t room = Nrf52840::RamBase + Nrf52840::RamSize - rx_dma_ptr_;
        std::size_t pos = 0;
        auto put = [&](std::uint8_t value) {
            if (pos < room)
                buffer[pos] = value;
            ++pos;
        };
        if (s0len)
            put(pdu[0]);
        const std::size_t length = pdu[1];
        if (lflen)
            put(static_cast<std::uint8_t>(length & (lflen >= 8 ? 0xffu : (1u << lflen) - 1u)));
        if (s1)
            put(0);
        const std::size_t count = std::min(std::min(length, maxlen) + statlen, pdu.size() - 2);
        for (std::size_t i = 0; i != count; ++i)
            put(pdu[2 + i]);
    }
    raise(Payload);
}

void Radio::ble_rx_end(const mcuemu::RadioFrame &frame) {
    rx_locked_ = false;
    last_end_ns_ = frame.end_ns;
    last_end_valid_ = true;
    ++bc_generation_;
    const int channel = ble_channel();
    bool decoded = false;
    if (channel >= 0)
        for (const mcuemu::RadioFrame &heard :
             air_->receive(node_, mcuemu::RadioProtocol::Ble, static_cast<std::uint8_t>(channel),
                           frame.end_ns, frame.end_ns - 1))
            decoded = decoded || heard.id == frame.id;
    // The CRC only checks out when the receiver was seeded like the sender.
    const bool crc_enabled = (crccnf_ & 3u) == 3u;
    const bool crc_ok = decoded && (!crc_enabled || (crcinit_ & 0xffffffu) == frame.ble.crc_init);
    const auto crc = ble::crc24(frame.ble.crc_init, frame.payload);
    rxcrc_ = crc[0] | crc[1] << 8 | crc[2] << 16;
    crcstatus_ = crc_ok ? 1 : 0;
    if (state_ == State::Rx)
        state_ = State::RxIdle;
    raise(End);
    raise(crc_ok ? CrcOk : CrcError);
}

// DEVMATCH/DEVMISS: the first six bytes after the two header bytes against the
// DAB/DAP list, honouring each entry's TxAdd bit.
void Radio::ble_device_match(const mcuemu::RadioFrame &frame) {
    const std::vector<std::uint8_t> &pdu = frame.payload;
    if (pdu.size() >= 8) {
        const unsigned tx_add = pdu[0] >> 6 & 1u;
        for (unsigned i = 0; i != 8; ++i) {
            if (!(dacnf_ >> i & 1u) || (dacnf_ >> (8 + i) & 1u) != tx_add)
                continue;
            std::uint8_t address[6];
            for (int b = 0; b != 4; ++b)
                address[b] = static_cast<std::uint8_t>(dab_[i] >> (8 * b));
            address[4] = static_cast<std::uint8_t>(dap_[i]);
            address[5] = static_cast<std::uint8_t>(dap_[i] >> 8);
            if (std::memcmp(address, pdu.data() + 2, 6) == 0) {
                dai_ = i;
                raise(DevMatch);
                return;
            }
        }
    }
    raise(DevMiss);
}

// BCSTART: BCMATCH when BCC bits have passed, if the packet is still going.
void Radio::start_bit_counter() {
    if (!rx_locked_ && state_ != State::Tx)
        return;
    const std::uint64_t at = to_ns(now()) + static_cast<std::uint64_t>(bcc_) * ble_bit_ns();
    schedule(at, [this, generation = bc_generation_] {
        if (generation == bc_generation_)
            raise(BcMatch);
    });
}

} // namespace nrf52emu
