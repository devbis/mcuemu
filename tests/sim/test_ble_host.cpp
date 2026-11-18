// The virtual BLE host against a scripted peripheral and against the nRF52840 RADIO.

#include "../test_support.hpp"
#include "../nrf52/programs.hpp"

#include "ble/ll.hpp"
#include "mcuemu/radio.hpp"
#include "nrf52emu/nrf52840.hpp"
#include "sim/ble_host.hpp"
#include "sim/config.hpp"
#include "sim/h4.hpp"
#include "sim/scheduler.hpp"
#include "sim/sink.hpp"
#include "sim/socket.hpp"
#include "sim/system.hpp"

#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <optional>

namespace mcuemu_tests {

namespace {

using ble::Bytes;
using mcuemu::BleLinkInfo;
using mcuemu::RadioFrame;
using mcuemu::RadioProtocol;

constexpr std::uint64_t Ms = 1000000, Us = 1000;
const BleLinkInfo kAdvLink{ble::kAdvertisingAccessAddress, ble::kAdvertisingCrcInit, true};

// ---- HCI helpers ----------------------------------------------------------

Bytes h4_command(std::uint16_t op, const Bytes &params = {}) {
    Bytes p{0x01, static_cast<std::uint8_t>(op), static_cast<std::uint8_t>(op >> 8),
            static_cast<std::uint8_t>(params.size())};
    p.insert(p.end(), params.begin(), params.end());
    return p;
}

struct Hci {
    sim::BleHost &host;
    std::vector<Bytes> packets;
    explicit Hci(sim::BleHost &h) : host(h) {
        host.set_hci_sink([this](const Bytes &p) { packets.push_back(p); });
    }
    void command(std::uint16_t op, const Bytes &params = {}) { host.hci_send(h4_command(op, params)); }
    void acl(std::uint16_t handle, std::uint8_t pb, const Bytes &data) {
        Bytes p{0x02, static_cast<std::uint8_t>(handle), static_cast<std::uint8_t>((handle >> 8) | pb << 4),
                static_cast<std::uint8_t>(data.size()), static_cast<std::uint8_t>(data.size() >> 8)};
        p.insert(p.end(), data.begin(), data.end());
        host.hci_send(p);
    }
    // Events with this code (and LE meta subevent when `sub` >= 0), oldest first.
    std::vector<Bytes> events(std::uint8_t code, int sub = -1) const {
        std::vector<Bytes> found;
        for (const Bytes &p : packets)
            if (p.size() >= 3 && p[0] == 0x04 && p[1] == code && (sub < 0 || (p.size() > 3 && p[3] == sub)))
                found.push_back(p);
        return found;
    }
    std::vector<Bytes> acl_packets() const {
        std::vector<Bytes> found;
        for (const Bytes &p : packets)
            if (!p.empty() && p[0] == 0x02)
                found.push_back(p);
        return found;
    }
    // The status byte of the Command Complete/Status event for `op`, or -1.
    int status_of(std::uint16_t op) const {
        for (const Bytes &p : packets) {
            if (p.size() < 6 || p[0] != 0x04)
                continue;
            if (p[1] == 0x0E && p[4] == (op & 0xff) && p[5] == (op >> 8) && p.size() > 6)
                return p[6];
            if (p[1] == 0x0F && p[5] == (op & 0xff) && p[6] == (op >> 8))
                return p[3];
        }
        return -1;
    }
};

constexpr std::uint16_t OpScanParams = 0x200B, OpScanEnable = 0x200C, OpCreateConn = 0x200D,
                        OpConnUpdate = 0x2013, OpReadRemoteFeatures = 0x2016, OpDisconnect = 0x0406,
                        OpReadRemoteVersion = 0x041D, OpReset = 0x0C03, OpReadBdAddr = 0x1009,
                        OpReadLocalCommands = 0x1002, OpLeRand = 0x2018;

Bytes scan_params(bool active, std::uint16_t interval = 0x10, std::uint16_t window = 0x10) {
    return {static_cast<std::uint8_t>(active), static_cast<std::uint8_t>(interval), static_cast<std::uint8_t>(interval >> 8),
            static_cast<std::uint8_t>(window), static_cast<std::uint8_t>(window >> 8), 0, 0};
}

Bytes create_connection(const ble::Address &peer, std::uint8_t peer_type, std::uint16_t interval,
                        std::uint16_t latency = 0, std::uint16_t timeout = 200) {
    Bytes p{0x10, 0x00, 0x10, 0x00, 0x00, peer_type};
    p.insert(p.end(), peer.begin(), peer.end());
    for (std::uint16_t v : {std::uint16_t{0}, interval, interval, latency, timeout, std::uint16_t{0}, std::uint16_t{0}}) {
        if (v == 0 && p.size() == 12) { // own address type is one byte
            p.push_back(0);
            continue;
        }
        p.push_back(static_cast<std::uint8_t>(v));
        p.push_back(static_cast<std::uint8_t>(v >> 8));
    }
    return p;
}

// ---- a scripted peripheral ----------------------------------------------------

struct Peripheral {
    mcuemu::VirtualAir &air;
    sim::Scheduler &sched;
    std::string node{"periph"};
    ble::Address addr{0x11, 0x22, 0x33, 0x44, 0x55, 0xC6};
    Bytes adv_data{0x02, 0x01, 0x06, 0x05, 0x09, 'T', 'e', 's', 't'};
    Bytes scan_rsp_data{0x04, 0xFF, 0x59, 0x00, 0x01};
    bool mute{false};
    bool connected{false};
    ble::ConnectParams params;
    ble::SeqState seq;
    struct Item {
        ble::Llid llid;
        Bytes payload;
    };
    std::deque<Item> queue;
    std::optional<Item> inflight;
    std::vector<std::pair<std::uint64_t, std::uint8_t>> rx_log; // frame start, channel
    std::vector<ble::Llid> rx_llids;
    Bytes l2cap;
    std::vector<Bytes> l2cap_messages;
    bool terminated{false};
    bool acked_terminate{false};
    std::uint8_t terminate_code{0};
    std::uint64_t connect_ind_end{0};
    std::vector<std::uint64_t> scan_req_starts;

    Peripheral(mcuemu::VirtualAir &a, sim::Scheduler &s) : air(a), sched(s) {
        air.set_node(node, {});
        air.add_observer([this](const RadioFrame &frame) { on_frame(frame); });
    }

    void advertise(std::uint64_t t, std::uint8_t channel) {
        sched.at(t, [this, t, channel] {
            const Bytes pdu = ble::build_adv_pdu(ble::AdvType::AdvInd, false, false,
                                                 ble::adv_payload_with_address(addr, adv_data));
            air.transmit(node, {}, RadioProtocol::Ble, channel, t, pdu, 0, kAdvLink);
        });
    }

    void queue_l2cap(std::uint16_t cid, const Bytes &data) {
        Bytes pdu{static_cast<std::uint8_t>(data.size()), static_cast<std::uint8_t>(data.size() >> 8),
                  static_cast<std::uint8_t>(cid), static_cast<std::uint8_t>(cid >> 8)};
        pdu.insert(pdu.end(), data.begin(), data.end());
        queue.push_back({ble::Llid::DataStart, pdu});
    }

    void on_frame(const RadioFrame &frame) {
        if (frame.source == node || frame.protocol != RadioProtocol::Ble || !frame.ble.ll_framed)
            return;
        if (!connected && frame.ble.access_address == ble::kAdvertisingAccessAddress) {
            const auto header = ble::parse_adv_header(frame.payload);
            const auto payload = ble::adv_payload(frame.payload);
            if (!header || !payload)
                return;
            if (header->type == ble::AdvType::ScanReq && payload->size() == 12 &&
                std::equal(addr.begin(), addr.end(), payload->begin() + 6)) {
                scan_req_starts.push_back(frame.start_ns);
                const std::uint64_t at = frame.end_ns + ble::kInterFrameSpaceNs;
                const std::uint8_t channel = frame.channel;
                sched.at(at, [this, at, channel] {
                    const Bytes pdu = ble::build_adv_pdu(ble::AdvType::ScanRsp, false, false,
                                                         ble::adv_payload_with_address(addr, scan_rsp_data));
                    air.transmit(node, {}, RadioProtocol::Ble, channel, at, pdu, 0, kAdvLink);
                });
            } else if (header->type == ble::AdvType::ConnectInd) {
                const auto ind = ble::parse_connect_ind_payload(*payload);
                if (ind && ind->adv_a == addr) {
                    connected = true;
                    params = ind->params;
                    seq.reset();
                    connect_ind_end = frame.end_ns;
                }
            }
            return;
        }
        if (connected && frame.ble.access_address == params.access_address) {
            rx_log.emplace_back(frame.start_ns, frame.channel);
            const auto copy = frame;
            sched.at(frame.end_ns, [this, copy] { on_conn_frame(copy); });
        }
    }

    void on_conn_frame(const RadioFrame &frame) {
        const auto header = ble::parse_data_header(frame.payload);
        const auto payload = ble::data_payload(frame.payload);
        if (!header || !payload || frame.ble.crc_init != params.crc_init)
            return;
        const auto result = seq.on_receive(header->sn, header->nesn);
        if (result.peer_acked)
            inflight.reset();
        if (result.new_data)
            handle(header->llid, *payload);
        if (mute || (terminated && acked_terminate))
            return;
        if (terminated)
            acked_terminate = true; // one last reply acknowledges the host's TERMINATE_IND
        if (!inflight && !queue.empty()) {
            inflight = queue.front();
            queue.pop_front();
        }
        const Bytes pdu = inflight ? ble::build_data_pdu(inflight->llid, seq.nesn(), seq.sn(), !queue.empty(),
                                                         inflight->payload)
                                   : ble::build_data_pdu(ble::Llid::DataContinuation, seq.nesn(), seq.sn(), false, {});
        const std::uint64_t at = frame.end_ns + ble::kInterFrameSpaceNs;
        const std::uint8_t channel = frame.channel;
        sched.at(at, [this, pdu, at, channel] {
            air.transmit(node, {}, RadioProtocol::Ble, channel, at, pdu, 0,
                         {params.access_address, params.crc_init, true});
        });
    }

    void handle(ble::Llid llid, const Bytes &payload) {
        if (llid == ble::Llid::Control && !payload.empty()) {
            rx_llids.push_back(llid);
            if (payload[0] == ble::ctrl::FeatureReq)
                queue.push_back({ble::Llid::Control, ble::ctrl_feature(false, 0x0102030405060708ull)});
            else if (payload[0] == ble::ctrl::VersionInd)
                queue.push_back({ble::Llid::Control, ble::ctrl_version(0x09, 0x0059, 0x1234)});
            else if (payload[0] == ble::ctrl::TerminateInd) {
                terminated = true;
                terminate_code = payload.size() > 1 ? payload[1] : 0;
            }
            return;
        }
        if (payload.empty())
            return;
        rx_llids.push_back(llid);
        if (llid == ble::Llid::DataStart)
            l2cap.clear();
        l2cap.insert(l2cap.end(), payload.begin(), payload.end());
        if (l2cap.size() >= 4 && l2cap.size() == 4u + (l2cap[0] | l2cap[1] << 8)) {
            l2cap_messages.push_back(l2cap);
            // ATT Read Request (0x0A, handle) -> Read Response (0x0B, value).
            if (l2cap.size() >= 7 && l2cap[2] == 4 && l2cap[4] == 0x0A)
                queue_l2cap(4, {0x0B, 'h', 'e', 'l', 'l', 'o'});
        }
    }

    void send_terminate(std::uint8_t code) { queue.push_back({ble::Llid::Control, ble::ctrl_terminate(code)}); }
};

struct Rig {
    mcuemu::VirtualAir air;
    sim::Scheduler sched;
    sim::BleHost host;
    Hci hci;
    Peripheral peripheral;
    Rig() : host(air, sched, "host", {}), hci(host), peripheral(air, sched) {
        air.set_node("host", {});
    }
    void run_to(std::uint64_t ns) { sched.advance_to(ns); }
    // Advertises on channel 37 + k % 3 inside scan window k (10 ms each).
    void advertise_every_window(std::uint64_t until_ns) {
        for (std::uint64_t k = 0; k * 10 * Ms + Ms < until_ns; ++k)
            peripheral.advertise(k * 10 * Ms + Ms, static_cast<std::uint8_t>(37 + k % 3));
    }
};

} // namespace

void test_ble_host_hci_basics() {
    Rig rig;
    rig.hci.command(OpReadBdAddr);
    CHECK(rig.hci.status_of(OpReadBdAddr) == 0);
    const auto &addr_event = rig.hci.packets.back();
    CHECK(addr_event.size() == 13 && addr_event[7] == 0x01 && addr_event[12] == 0x00);
    rig.hci.command(OpReadLocalCommands);
    const auto &commands = rig.hci.packets.back();
    CHECK(commands.size() == 7 + 64);
    CHECK((commands[7 + 26] & 0x08) != 0); // LE Set Scan Enable
    CHECK((commands[7 + 26] & 0x10) != 0); // LE Create Connection
    CHECK((commands[7 + 5] & 0x80) != 0);  // Reset
    rig.hci.command(OpLeRand);
    CHECK(rig.hci.packets.back().size() == 7 + 8); // H4 + event header + 3 + status + 8 bytes
    // An opcode the controller does not know gets Command Status "unknown command".
    rig.hci.command(0xFC00);
    CHECK(rig.hci.status_of(0xFC00) == 0x01);
    // Scan parameters cannot change while scanning; window larger than interval is refused.
    rig.hci.command(OpScanParams, scan_params(false, 0x10, 0x20));
    CHECK(rig.hci.status_of(OpScanParams) == 0x12);
    rig.hci.packets.clear();
    rig.hci.command(OpScanParams, scan_params(false));
    CHECK(rig.hci.status_of(OpScanParams) == 0);
    rig.hci.command(OpScanEnable, {1, 0});
    CHECK(rig.host.scanning());
    rig.hci.packets.clear();
    rig.hci.command(OpScanParams, scan_params(true));
    CHECK(rig.hci.status_of(OpScanParams) == 0x0C);
    rig.hci.command(OpReset);
    CHECK(!rig.host.scanning());
}

void test_ble_host_passive_and_active_scan() {
    for (bool active : {false, true}) {
        Rig rig;
        rig.hci.command(OpScanParams, scan_params(active));
        rig.hci.command(OpScanEnable, {1, 0});
        rig.advertise_every_window(100 * Ms);
        rig.run_to(100 * Ms);
        const auto reports = rig.hci.events(0x3E, 0x02);
        // The advertiser is heard once per window; the first is ADV_IND on channel 37.
        CHECK(reports.size() >= (active ? 10u : 9u));
        if (reports.empty())
            continue;
        const Bytes &first = reports[0];
        // [04 3E len 02 | 01 evt addr_type addr(6) len data rssi]
        CHECK(first[4] == 1 && first[5] == 0 && first[6] == 0);
        CHECK(std::equal(rig.peripheral.addr.begin(), rig.peripheral.addr.end(), first.begin() + 7));
        CHECK(first[13] == rig.peripheral.adv_data.size());
        CHECK(std::equal(rig.peripheral.adv_data.begin(), rig.peripheral.adv_data.end(), first.begin() + 14));
        CHECK(static_cast<std::int8_t>(first.back()) == -50); // the ideal medium
        std::size_t responses = 0;
        for (const Bytes &r : reports) {
            if (r[5] == 4) {
                ++responses;
                CHECK(r[13] == rig.peripheral.scan_rsp_data.size());
                CHECK(std::equal(rig.peripheral.scan_rsp_data.begin(), rig.peripheral.scan_rsp_data.end(),
                                 r.begin() + 14));
            }
        }
        if (active) {
            CHECK(responses >= 9);
            // Every SCAN_REQ went out exactly T_IFS after the advertisement it answers.
            CHECK(!rig.peripheral.scan_req_starts.empty());
            const auto &frames = rig.air.frames();
            for (std::uint64_t start : rig.peripheral.scan_req_starts) {
                bool ok = false;
                for (const RadioFrame &f : frames)
                    ok = ok || (f.source == "periph" && f.end_ns + ble::kInterFrameSpaceNs == start);
                CHECK(ok);
            }
        } else {
            CHECK(responses == 0 && rig.peripheral.scan_req_starts.empty());
        }
    }
}

void test_ble_host_duplicate_filter_and_accept_list() {
    {
        Rig rig;
        rig.hci.command(OpScanParams, scan_params(false));
        rig.hci.command(OpScanEnable, {1, 1}); // filter duplicates
        rig.advertise_every_window(100 * Ms);
        rig.run_to(100 * Ms);
        CHECK(rig.hci.events(0x3E, 0x02).size() == 1);
    }
    {
        // The accept list excludes the advertiser: no report. With it listed: reports.
        Rig rig;
        Bytes params = scan_params(false);
        params[6] = 1; // filter policy: use the accept list
        rig.hci.command(OpScanParams, params);
        rig.hci.command(OpScanEnable, {1, 0});
        rig.advertise_every_window(50 * Ms);
        rig.run_to(50 * Ms);
        CHECK(rig.hci.events(0x3E, 0x02).empty());
        rig.hci.command(OpScanEnable, {0, 0});
        Bytes entry{0};
        entry.insert(entry.end(), rig.peripheral.addr.begin(), rig.peripheral.addr.end());
        rig.hci.command(0x2011, entry);
        CHECK(rig.hci.status_of(0x2011) == 0);
        rig.hci.command(OpScanEnable, {1, 0});
        // The scan restarted at 50 ms, so window j (10 ms each) listens on channel 37 + j % 3.
        for (std::uint64_t j = 0; j < 5; ++j)
            rig.peripheral.advertise(50 * Ms + j * 10 * Ms + Ms, static_cast<std::uint8_t>(37 + j % 3));
        rig.run_to(110 * Ms);
        CHECK(!rig.hci.events(0x3E, 0x02).empty());
    }
}

// Brings up a connection to the scripted peripheral and returns once the host reported it.
static void connect(Rig &rig, std::uint16_t interval = 24, std::uint16_t latency = 0, std::uint16_t timeout = 200) {
    rig.hci.command(OpCreateConn, create_connection(rig.peripheral.addr, 0, interval, latency, timeout));
    rig.peripheral.advertise(2 * Ms, 37);
    rig.peripheral.advertise(5 * Ms, 38);
    rig.run_to(6 * Ms);
}

void test_ble_host_connection_events_and_timing() {
    Rig rig;
    connect(rig);
    CHECK(rig.hci.status_of(OpCreateConn) == 0);
    const auto complete = rig.hci.events(0x3E, 0x01);
    CHECK(complete.size() == 1);
    CHECK(rig.peripheral.connected);
    // LE Connection Complete: status 0, handle, role 0 (central), peer, interval 24, latency 0, timeout 200.
    const Bytes &cc = complete[0];
    CHECK(cc[4] == 0 && cc[7] == 0 && cc[8] == 0);
    CHECK(cc[15] == 24 && cc[17] == 0 && cc[19] == 200);
    CHECK(rig.peripheral.params.interval == 24 && rig.peripheral.params.sca == 7);
    CHECK(ble::access_address_valid(rig.peripheral.params.access_address));
    // CONNECT_IND went out T_IFS after the advertisement that started it.
    std::optional<RadioFrame> connect_ind;
    std::optional<RadioFrame> advertisement;
    for (const RadioFrame &f : rig.air.frames()) {
        if (f.source == "host" && f.ble.access_address == ble::kAdvertisingAccessAddress)
            connect_ind = f;
        if (f.source == "periph" && !advertisement)
            advertisement = f;
    }
    CHECK(connect_ind && advertisement);
    if (connect_ind && advertisement) {
        CHECK(connect_ind->start_ns == advertisement->end_ns + ble::kInterFrameSpaceNs);
        CHECK(connect_ind->payload.size() == 2 + 34 && (connect_ind->payload[0] & 0x0F) == 5);
    }

    rig.run_to(400 * Ms);
    // The first packet is 1.25 ms after the end of CONNECT_IND; then every 30 ms exactly.
    const auto &log = rig.peripheral.rx_log;
    CHECK(log.size() >= 12);
    if (!log.empty() && connect_ind) {
        CHECK(log[0].first == connect_ind->end_ns + ble::kTransmitWindowDelayNs);
        for (std::size_t i = 1; i < log.size(); ++i)
            CHECK(log[i].first == log[0].first + i * 30 * Ms);
        // Channels follow the selection algorithm with the connection's hop increment.
        ble::ChannelSelection1 csa(rig.peripheral.params.hop, rig.peripheral.params.channel_map);
        for (const auto &entry : log)
            CHECK(entry.second == csa.next());
    }
    const auto info = rig.host.connection();
    CHECK(info && info->established && info->event_counter >= 12 && info->packets_received >= 12);
}

void test_ble_host_data_and_gatt_read() {
    Rig rig;
    connect(rig);
    rig.run_to(100 * Ms);
    const std::uint16_t handle = rig.host.connection()->handle;
    rig.hci.packets.clear();
    // ATT Read Request for handle 3 on the ATT channel (CID 4).
    rig.hci.acl(handle, 0, {0x03, 0x00, 0x04, 0x00, 0x0A, 0x03, 0x00});
    rig.run_to(200 * Ms);
    CHECK(rig.peripheral.l2cap_messages.size() == 1);
    CHECK((rig.peripheral.l2cap_messages.empty() ? Bytes{} : rig.peripheral.l2cap_messages[0]) ==
          (Bytes{0x03, 0x00, 0x04, 0x00, 0x0A, 0x03, 0x00}));
    // The response comes back as ACL data, first fragment flag set.
    const auto acl = rig.hci.acl_packets();
    CHECK(acl.size() == 1);
    if (!acl.empty()) {
        CHECK((acl[0][2] >> 4) == 0x2 && (acl[0][1] | (acl[0][2] & 0x0F) << 8) == handle);
        CHECK((Bytes(acl[0].begin() + 5, acl[0].end()) == Bytes{0x06, 0x00, 0x04, 0x00, 0x0B, 'h', 'e', 'l', 'l', 'o'}));
    }
    // The host stack gets a completed-packets credit for what it sent.
    const auto credits = rig.hci.events(0x13);
    CHECK(credits.size() == 1 && credits[0][3] == 1 && credits[0][6] == 1);
}

void test_ble_host_fragments_large_acl_packets() {
    Rig rig;
    connect(rig);
    rig.run_to(100 * Ms);
    const std::uint16_t handle = rig.host.connection()->handle;
    Bytes data{0x3C, 0x00, 0x40, 0x00}; // L2CAP: 60 bytes on CID 0x40
    for (int i = 0; i < 60; ++i)
        data.push_back(static_cast<std::uint8_t>(i));
    rig.hci.packets.clear();
    rig.hci.acl(handle, 0, data);
    rig.run_to(400 * Ms);
    // 64 bytes go out as 27 + 27 + 10 over three connection events.
    CHECK(rig.peripheral.l2cap_messages.size() == 1);
    CHECK((rig.peripheral.l2cap_messages.empty() ? Bytes{} : rig.peripheral.l2cap_messages[0]) == data);
    std::vector<ble::Llid> data_llids;
    for (ble::Llid l : rig.peripheral.rx_llids)
        data_llids.push_back(l);
    CHECK(data_llids.size() == 3 && data_llids[0] == ble::Llid::DataStart &&
          data_llids[1] == ble::Llid::DataContinuation && data_llids[2] == ble::Llid::DataContinuation);
    // One ACL packet, one credit (on the last fragment).
    CHECK(rig.hci.events(0x13).size() == 1);
}

void test_ble_host_remote_features_version_and_update() {
    Rig rig;
    connect(rig);
    rig.run_to(100 * Ms);
    const std::uint16_t handle = rig.host.connection()->handle;
    const Bytes h{static_cast<std::uint8_t>(handle), static_cast<std::uint8_t>(handle >> 8)};
    rig.hci.packets.clear();
    rig.hci.command(OpReadRemoteFeatures, h);
    rig.hci.command(OpReadRemoteVersion, h);
    rig.run_to(300 * Ms);
    const auto features = rig.hci.events(0x3E, 0x04);
    CHECK(features.size() == 1);
    if (!features.empty())
        CHECK(features[0][4] == 0 && features[0][7] == 0x08 && features[0][14] == 0x01);
    const auto version = rig.hci.events(0x0C);
    CHECK(version.size() == 1);
    if (!version.empty())
        CHECK(version[0][3] == 0 && version[0][6] == 0x09 && version[0][7] == 0x59 && version[0][9] == 0x34);

    // Connection update to 50 ms: the anchors move to the new interval after the instant.
    rig.hci.packets.clear();
    Bytes update = h;
    for (std::uint16_t v : {std::uint16_t{40}, std::uint16_t{40}, std::uint16_t{0}, std::uint16_t{200}, std::uint16_t{0}, std::uint16_t{0}}) {
        update.push_back(static_cast<std::uint8_t>(v));
        update.push_back(static_cast<std::uint8_t>(v >> 8));
    }
    rig.hci.command(OpConnUpdate, update);
    CHECK(rig.hci.status_of(OpConnUpdate) == 0);
    rig.run_to(1200 * Ms);
    const auto done = rig.hci.events(0x3E, 0x03);
    CHECK(done.size() == 1);
    if (!done.empty())
        CHECK(done[0][4] == 0 && done[0][7] == 40);
    const auto &log = rig.peripheral.rx_log;
    CHECK(log.size() > 20);
    bool saw_30 = false, saw_50 = false;
    for (std::size_t i = 1; i < log.size(); ++i) {
        const std::uint64_t gap = log[i].first - log[i - 1].first;
        if (gap == 30 * Ms) saw_30 = true;
        if (gap == 50 * Ms) saw_50 = true;
    }
    CHECK(saw_30 && saw_50);
    CHECK(log.back().first - log[log.size() - 2].first == 50 * Ms);
    CHECK(rig.host.connection() && rig.host.connection()->interval == 40);
}

void test_ble_host_disconnects() {
    // Host terminates.
    {
        Rig rig;
        connect(rig);
        rig.run_to(100 * Ms);
        const std::uint16_t handle = rig.host.connection()->handle;
        rig.hci.packets.clear();
        rig.hci.command(OpDisconnect, {static_cast<std::uint8_t>(handle), static_cast<std::uint8_t>(handle >> 8), 0x13});
        CHECK(rig.hci.status_of(OpDisconnect) == 0);
        rig.run_to(300 * Ms);
        CHECK(rig.peripheral.terminated && rig.peripheral.terminate_code == 0x13);
        const auto gone = rig.hci.events(0x05);
        CHECK(gone.size() == 1);
        if (!gone.empty())
            CHECK(gone[0][3] == 0 && gone[0][6] == 0x16); // connection terminated by local host
        CHECK(!rig.host.connection());
    }
    // The peripheral terminates.
    {
        Rig rig;
        connect(rig);
        rig.run_to(100 * Ms);
        rig.hci.packets.clear();
        rig.peripheral.send_terminate(0x13);
        rig.run_to(300 * Ms);
        const auto gone = rig.hci.events(0x05);
        CHECK(gone.size() == 1);
        if (!gone.empty())
            CHECK(gone[0][3] == 0 && gone[0][6] == 0x13);
        CHECK(!rig.host.connection());
    }
    // The peripheral goes silent: supervision timeout after 2 s (timeout 200 * 10 ms).
    {
        Rig rig;
        connect(rig);
        rig.run_to(100 * Ms);
        rig.hci.packets.clear();
        rig.peripheral.mute = true;
        rig.run_to(1500 * Ms);
        CHECK(rig.hci.events(0x05).empty());
        rig.run_to(2400 * Ms);
        const auto gone = rig.hci.events(0x05);
        CHECK(gone.size() == 1);
        if (!gone.empty())
            CHECK(gone[0][6] == 0x08);
    }
    // The peripheral never answers: the connection fails to establish.
    {
        Rig rig;
        rig.peripheral.mute = true;
        connect(rig);
        rig.run_to(400 * Ms);
        const auto gone = rig.hci.events(0x05);
        CHECK(gone.size() == 1);
        if (!gone.empty())
            CHECK(gone[0][6] == 0x3E);
    }
}

void test_ble_host_cancel_and_second_connection() {
    Rig rig;
    rig.hci.command(OpCreateConn, create_connection(rig.peripheral.addr, 0, 24));
    CHECK(rig.hci.status_of(OpCreateConn) == 0);
    rig.hci.packets.clear();
    rig.hci.command(OpCreateConn, create_connection(rig.peripheral.addr, 0, 24));
    CHECK(rig.hci.status_of(OpCreateConn) == 0x0C); // already creating
    rig.hci.packets.clear();
    rig.hci.command(0x200E);
    CHECK(rig.hci.status_of(0x200E) == 0);
    const auto cancelled = rig.hci.events(0x3E, 0x01);
    CHECK(cancelled.size() == 1 && cancelled[0][4] == 0x02);
    rig.peripheral.advertise(2 * Ms, 37);
    rig.run_to(20 * Ms);
    CHECK(!rig.peripheral.connected);
    // Invalid parameters are refused.
    rig.hci.packets.clear();
    rig.hci.command(OpCreateConn, create_connection(rig.peripheral.addr, 0, 24, 0, 5));
    CHECK(rig.hci.status_of(OpCreateConn) == 0x12);
    // With a connection up, a second Create Connection is refused with "connection limit exceeded".
    Rig other;
    connect(other);
    other.hci.packets.clear();
    other.hci.command(OpCreateConn, create_connection(other.peripheral.addr, 0, 24));
    CHECK(other.hci.status_of(OpCreateConn) == 0x09);
}

// The host against the nRF52840 RADIO: an advertiser built from register writes,
// the way the Zephyr controller drives it (TX, then RX for a SCAN_REQ, then TX of
// the SCAN_RSP, all chained by SHORTS and TIFS).
void test_ble_host_scans_an_nrf52840_advertiser() {
    constexpr std::uint32_t Radio = 0x40001000;
    constexpr std::uint64_t Cycles = 64;
    constexpr std::uint32_t READY_START = 1u << 0, END_DISABLE = 1u << 1, DISABLED_TXEN = 1u << 2,
                            DISABLED_RXEN = 1u << 3;
    mcuemu::VirtualAir air;
    sim::Scheduler sched;
    air.set_node("host", {});
    air.set_node("nrf", {});
    sim::BleHost host(air, sched, "host", {});
    Hci hci(host);
    nrf52emu::Nrf52840 chip;
    chip.load_firmware(nrf52_programs::idle);
    chip.reset();
    chip.connect_radio(air, "nrf");
    auto write = [&](std::uint32_t reg, std::uint32_t value) { chip.write(Radio + reg, value, 4); };
    auto read = [&](std::uint32_t reg) { return chip.read(Radio + reg, 4); };
    write(0x510, 3);
    write(0x508, 2); // 2402 MHz: channel 37
    write(0x514, 8 | (1u << 8));
    write(0x518, 255 | (3u << 16) | (1u << 25));
    write(0x51c, ble::kAdvertisingAccessAddress << 8);
    write(0x524, ble::kAdvertisingAccessAddress >> 24);
    write(0x530, 1);
    write(0x534, 3 | (1u << 8));
    write(0x538, 0x00065B);
    write(0x53c, 0x555555);
    write(0x544, 150);
    write(0x554, 37 | 0x40);

    const ble::Address nrf_addr{0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xC1};
    const Bytes adv_data{0x02, 0x01, 0x06, 0x03, 0x09, 'n', 'R'};
    const Bytes rsp_data{0x04, 0xFF, 0x59, 0x00, 0x7E};
    const Bytes adv = ble::build_adv_pdu(ble::AdvType::AdvInd, true, false, ble::adv_payload_with_address(nrf_addr, adv_data));
    const Bytes rsp = ble::build_adv_pdu(ble::AdvType::ScanRsp, true, false, ble::adv_payload_with_address(nrf_addr, rsp_data));
    std::memcpy(chip.ram() + 0x200, adv.data(), adv.size());
    std::memcpy(chip.ram() + 0x300, rsp.data(), rsp.size());

    hci.command(OpScanParams, scan_params(true, 0x4000, 0x4000)); // stay on channel 37 for the whole test
    hci.command(OpScanEnable, {1, 0});

    // TX the ADV_IND, then listen for a SCAN_REQ, then answer it.
    write(0x504, 0x20000200);
    write(0x200, READY_START | END_DISABLE | DISABLED_RXEN);
    write(0x000, 1); // TXEN
    bool armed_rx = false, armed_rsp = false;
    for (std::uint64_t t = 1; t <= 3000; ++t) {
        chip.run(t * Us / 1000 * Cycles);
        sched.advance_to(t * Us);
        if (!armed_rx && read(0x110)) { // DISABLED after the advertisement: the radio is now ramping up for RX
            write(0x110, 0);
            write(0x108, 0); // forget the PAYLOAD and END of our own transmission
            write(0x10c, 0);
            write(0x504, 0x20000400);                                  // receive the SCAN_REQ here
            write(0x200, READY_START | END_DISABLE | DISABLED_TXEN);   // then answer after T_IFS
            armed_rx = true;
        }
        if (armed_rx && !armed_rsp && read(0x108)) { // PAYLOAD of the SCAN_REQ is in RAM
            write(0x504, 0x20000300);                                  // transmit the SCAN_RSP from here
            armed_rsp = true;
        }
        if (armed_rsp && air.frames().size() >= 3) {
            write(0x200, END_DISABLE); // stop after the response
            armed_rsp = false;
            armed_rx = false;
        }
    }
    const auto reports = hci.events(0x3E, 0x02);
    CHECK(reports.size() >= 2);
    if (reports.size() >= 2) {
        CHECK(reports[0][5] == 0 && reports[0][6] == 1); // ADV_IND, random address
        CHECK(std::equal(nrf_addr.begin(), nrf_addr.end(), reports[0].begin() + 7));
        CHECK(reports[1][5] == 4 && reports[1][13] == rsp_data.size());
        CHECK(std::equal(rsp_data.begin(), rsp_data.end(), reports[1].begin() + 14));
    }
    // The nRF received the SCAN_REQ the host built for it.
    CHECK(chip.ram()[0x400] == (0x03 | 0x00 | 0x80)); // SCAN_REQ, TxAdd 0 (public host), RxAdd 1 (random nRF)
    CHECK(chip.ram()[0x401] == 12);
    CHECK(std::equal(nrf_addr.begin(), nrf_addr.end(), chip.ram() + 0x400 + 8));
    // Timing: SCAN_REQ T_IFS after the ADV_IND, SCAN_RSP T_IFS after the SCAN_REQ.
    if (air.frames().size() >= 3) {
        CHECK(air.frames()[1].start_ns == air.frames()[0].end_ns + ble::kInterFrameSpaceNs);
        CHECK(air.frames()[2].start_ns == air.frames()[1].end_ns + ble::kInterFrameSpaceNs);
    }
}

// ---- the ble_host part: HCI as an endpoint, as a TCP stream ----------------------

namespace {

std::string scratch_file(const std::string &name, const std::string &text) {
    const auto dir = std::filesystem::temp_directory_path() / "simcore-ble-tests";
    std::filesystem::create_directories(dir);
    const auto file = dir / name;
    std::ofstream(file) << text;
    return file.string();
}

// A device needs an MCU; an idle nRF52840 does.
std::string idle_image() {
    const auto dir = std::filesystem::temp_directory_path() / "simcore-ble-tests";
    std::filesystem::create_directories(dir);
    const auto file = dir / "idle.bin";
    std::ofstream out(file, std::ios::binary);
    out.write(reinterpret_cast<const char *>(nrf52_programs::idle.data()),
              static_cast<std::streamsize>(nrf52_programs::idle.size()));
    return file.string();
}

// A Lua advertiser: ADV_IND with a full link-layer PDU, one per 10 ms, hopping 37, 38, 39.
std::string host_system_yaml() {
    const std::string lua = scratch_file("beacon.lua", R"(
local k = 0
local pdu = string.char(0x00, 13, 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6, 0x02, 0x01, 0x06, 0x03, 0x09, 0x4C, 0x75)
local function beacon()
  radio.transmit{protocol = "ble", channel = 37 + k % 3, payload = pdu, ll = true}
  k = k + 1
  after(10 * MS, beacon)
end
after(1 * MS, beacon)
)");
    return "version: 1\nworld:\n  radio:\n    - { id: air0 }\n"
           "devices:\n  - id: d\n    mcus: [ {id: m, model: nrf52840, firmware: " + idle_image() + "} ]\n"
           "    parts:\n"
           "      - id: adv\n        type: script\n        script: " + lua + "\n"
           "        radio: { air: air0, protocols: [ble] }\n"
           "      - id: host\n        type: ble_host\n"
           "        radio: { air: air0, protocols: [ble] }\n"
           "        params: { address: \"AA:BB:CC:DD:EE:01\" }\n";
}

const Bytes kScanConfig = h4_command(0x200B, {0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00}); // passive, 10 ms
const Bytes kScanEnable = h4_command(0x200C, {0x01, 0x00});

// Advertising reports found in a stream of H4 events.
std::vector<Bytes> reports_in(const Bytes &stream) {
    std::vector<Bytes> reports;
    sim::H4Framer framer;
    framer.feed(stream.data(), stream.size(), [&](const Bytes &p) {
        if (p.size() > 4 && p[0] == 0x04 && p[1] == 0x3E && p[3] == 0x02)
            reports.push_back(p);
    });
    return reports;
}

} // namespace

void test_ble_h4_framer() {
    std::vector<Bytes> got;
    auto deliver = [&](const Bytes &p) { got.push_back(p); };
    sim::H4Framer framer;
    const Bytes a = h4_command(0x200C, {1, 0});
    const Bytes b{0x02, 0x40, 0x20, 0x03, 0x00, 7, 8, 9};      // ACL data, 3 bytes
    const Bytes c{0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00};   // an event
    Bytes all = a;
    all.insert(all.end(), b.begin(), b.end());
    all.insert(all.end(), c.begin(), c.end());
    // Merged in one write.
    framer.feed(all.data(), all.size(), deliver);
    CHECK(got.size() == 3 && got[0] == a && got[1] == b && got[2] == c && framer.pending() == 0);
    // Split byte by byte.
    got.clear();
    for (std::uint8_t byte : all)
        framer.feed(&byte, 1, deliver);
    CHECK(got.size() == 3 && got[0] == a && got[1] == b && got[2] == c);
    // A packet cut in the middle waits for the rest.
    got.clear();
    framer.feed(b.data(), 6, deliver);
    CHECK(got.empty() && framer.pending() == 6);
    framer.feed(b.data() + 6, b.size() - 6, deliver);
    CHECK(got.size() == 1 && got[0] == b);
    // Garbage cannot be resynchronised: it throws and the buffer is dropped.
    const std::uint8_t garbage[] = {0x7F, 1, 2};
    bool threw = false;
    try {
        framer.feed(garbage, 3, deliver);
    } catch (const std::invalid_argument &) {
        threw = true;
    }
    CHECK(threw && framer.pending() == 0);
}

void test_ble_host_part_endpoint() {
    const auto system = sim::build_system(sim::parse_system_config(host_system_yaml()));
    sim::World &world = system->world();
    sim::Endpoint &hci = world.endpoints().at("d.host.hci");
    CHECK(hci.kind() == "hci");
    Bytes stream;
    hci.subscribe([&](const sim::EndpointEvent &event) {
        const Bytes &packet = std::get<Bytes>(event.value);
        stream.insert(stream.end(), packet.begin(), packet.end());
    });
    // The two commands arrive glued together and the second one cut in two writes.
    Bytes both = kScanConfig;
    both.insert(both.end(), kScanEnable.begin(), kScanEnable.begin() + 2);
    hci.set(sim::Value{both});
    hci.set(sim::Value{Bytes(kScanEnable.begin() + 2, kScanEnable.end())});
    world.run_until(100 * Ms);
    const auto reports = reports_in(stream);
    CHECK(reports.size() >= 9);
    if (!reports.empty()) {
        const Bytes &first = reports[0];
        CHECK(first[5] == 0 && first[6] == 0); // ADV_IND, public address
        CHECK((Bytes(first.begin() + 7, first.begin() + 13) == Bytes{0x11, 0x22, 0x33, 0x44, 0x55, 0xC6}));
        CHECK(first[13] == 7 && first[14] == 0x02 && first[20] == 0x75);
    }
    CHECK(std::get<std::int64_t>(hci.get()) >= 2 + 9); // two command completes and the reports
    // The hci endpoint only takes bytes.
    bool refused = false;
    try {
        hci.set(sim::Value{std::string("x")});
    } catch (const std::invalid_argument &) {
        refused = true;
    }
    CHECK(refused);
    // An unknown H4 type is an error to the writer.
    bool bad_type = false;
    try {
        hci.set(sim::Value{Bytes{0x7F}});
    } catch (const std::invalid_argument &) {
        bad_type = true;
    }
    CHECK(bad_type);
}

void test_ble_host_part_over_tcp() {
    const auto system = sim::build_system(sim::parse_system_config(host_system_yaml()));
    sim::World &world = system->world();
    sim::Session &session = system->session();
    const auto id = session.attach("d.host.hci", "tcp:0");
    auto *tcp = dynamic_cast<sim::TcpSink *>(session.sink(id));
    CHECK(tcp != nullptr);
    if (!tcp)
        return;
    const sim::sock::socket_t client = sim::sock::open_tcp();
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(tcp->port());
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    CHECK(::connect(client, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0);
    sim::sock::set_nonblocking(client);
    session.poll(); // accept
    Bytes commands = kScanConfig;
    commands.insert(commands.end(), kScanEnable.begin(), kScanEnable.end());
    CHECK(sim::sock::send_some(client, reinterpret_cast<const char *>(commands.data()), commands.size()) ==
          static_cast<std::ptrdiff_t>(commands.size()));
    Bytes received;
    for (int i = 0; i < 200 && reports_in(received).size() < 3; ++i) {
        sim::sock::sleep_us(1000);
        session.poll();
        world.run_for(2 * Ms);
        std::uint8_t buffer[512];
        for (;;) {
            const std::ptrdiff_t n = sim::sock::recv_some(client, buffer, sizeof buffer);
            if (n <= 0)
                break;
            received.insert(received.end(), buffer, buffer + n);
        }
    }
    // The stream starts with the two Command Complete events, then reports.
    CHECK(received.size() > 14 && received[0] == 0x04 && received[1] == 0x0E);
    CHECK(reports_in(received).size() >= 3);
    sim::sock::close_socket(client);
    session.poll();
}

void test_ble_host_part_config_errors() {
    auto build = [](const std::string &host_part) {
        try {
            sim::build_system(sim::parse_system_config(
                "version: 1\nworld:\n  radio:\n    - { id: air0 }\n"
                "devices:\n  - id: d\n    mcus: [ {id: m, model: nrf52840, firmware: " + idle_image() + "} ]\n"
                "    parts:\n" + host_part));
        } catch (const std::exception &error) {
            return std::string(error.what());
        }
        return std::string();
    };
    auto contains = [](const std::string &text, const std::string &part) { return text.find(part) != std::string::npos; };
    CHECK(contains(build("      - { id: h, type: ble_host }\n"), "needs a 'radio:' section"));
    CHECK(contains(build("      - { id: h, type: ble_host, radio: { air: air0, protocols: [zigbee] } }\n"),
                   "needs 'ble' in its radio protocols"));
    CHECK(contains(build("      - { id: h, type: ble_host, radio: { air: air0, protocols: [ble] }, params: { address: nope } }\n"),
                   "param 'address'"));
    CHECK(contains(build("      - { id: h, type: ble_host, radio: { air: air0, protocols: [ble] }, params: { colour: red } }\n"),
                   "colour"));
    CHECK(build("      - { id: h, type: ble_host, radio: { air: air0, protocols: [ble] } }\n").empty());
}

} // namespace mcuemu_tests
