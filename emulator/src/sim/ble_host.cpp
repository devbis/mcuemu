#include "sim/ble_host.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <deque>
#include <stdexcept>

namespace sim {

namespace {

using ble::Bytes;

// ---- HCI constants (Core spec Vol 4 Part E) --------------------------------

constexpr std::uint8_t H4Command = 0x01, H4Acl = 0x02, H4Event = 0x04;

constexpr std::uint16_t opcode(unsigned ogf, unsigned ocf) {
    return static_cast<std::uint16_t>(ogf << 10 | ocf);
}
constexpr std::uint16_t OpDisconnect = opcode(1, 0x06);
constexpr std::uint16_t OpReadRemoteVersion = opcode(1, 0x1D);
constexpr std::uint16_t OpSetEventMask = opcode(3, 0x01);
constexpr std::uint16_t OpReset = opcode(3, 0x03);
constexpr std::uint16_t OpSetEventMaskPage2 = opcode(3, 0x63);
constexpr std::uint16_t OpReadLeHostSupport = opcode(3, 0x6C);
constexpr std::uint16_t OpWriteLeHostSupport = opcode(3, 0x6D);
constexpr std::uint16_t OpReadLocalVersion = opcode(4, 0x01);
constexpr std::uint16_t OpReadLocalCommands = opcode(4, 0x02);
constexpr std::uint16_t OpReadLocalFeatures = opcode(4, 0x03);
constexpr std::uint16_t OpReadLocalExtFeatures = opcode(4, 0x04);
constexpr std::uint16_t OpReadBufferSize = opcode(4, 0x05);
constexpr std::uint16_t OpReadBdAddr = opcode(4, 0x09);
constexpr std::uint16_t OpLeSetEventMask = opcode(8, 0x01);
constexpr std::uint16_t OpLeReadBufferSize = opcode(8, 0x02);
constexpr std::uint16_t OpLeReadLocalFeatures = opcode(8, 0x03);
constexpr std::uint16_t OpLeSetRandomAddress = opcode(8, 0x05);
constexpr std::uint16_t OpLeSetScanParameters = opcode(8, 0x0B);
constexpr std::uint16_t OpLeSetScanEnable = opcode(8, 0x0C);
constexpr std::uint16_t OpLeCreateConnection = opcode(8, 0x0D);
constexpr std::uint16_t OpLeCreateConnectionCancel = opcode(8, 0x0E);
constexpr std::uint16_t OpLeReadFilterAcceptListSize = opcode(8, 0x0F);
constexpr std::uint16_t OpLeClearFilterAcceptList = opcode(8, 0x10);
constexpr std::uint16_t OpLeAddToFilterAcceptList = opcode(8, 0x11);
constexpr std::uint16_t OpLeRemoveFromFilterAcceptList = opcode(8, 0x12);
constexpr std::uint16_t OpLeConnectionUpdate = opcode(8, 0x13);
constexpr std::uint16_t OpLeReadRemoteFeatures = opcode(8, 0x16);
constexpr std::uint16_t OpLeRand = opcode(8, 0x18);

constexpr std::uint8_t EvDisconnectionComplete = 0x05, EvReadRemoteVersionComplete = 0x0C,
                       EvCommandComplete = 0x0E, EvCommandStatus = 0x0F,
                       EvNumberOfCompletedPackets = 0x13, EvLeMeta = 0x3E;
constexpr std::uint8_t LeConnectionComplete = 0x01, LeAdvertisingReport = 0x02,
                       LeConnectionUpdateComplete = 0x03, LeReadRemoteFeaturesComplete = 0x04;

// Error codes used here.
constexpr std::uint8_t ErrUnknownCommand = 0x01, ErrUnknownConnection = 0x02, ErrCommandDisallowed = 0x0C,
                       ErrInvalidParameters = 0x12,
                       ErrRemoteUserTerminated = 0x13, ErrLocalHostTerminated = 0x16,
                       ErrConnectionTimeout = 0x08, ErrConnectionLimit = 0x09,
                       ErrUnsupportedRemoteFeature = 0x1A, ErrNotEstablished = 0x3E,
                       ErrOperationCancelled = 0x02;

constexpr std::uint16_t LeAclLength = 251;
constexpr std::uint8_t LeAclPackets = 8;
constexpr std::size_t LlPayloadMax = 27; // no data length extension
constexpr std::uint64_t IfsNs = ble::kInterFrameSpaceNs;
// How far from T_IFS a reply may start and still be taken as the answer: the
// specification allows +-2 us; the margin covers a peer that is late by an
// instruction or two of its core.
constexpr std::uint64_t ReplyToleranceNs = 16000;
constexpr std::uint64_t EstablishEvents = 6;
// Supported LL features: peripheral-initiated feature exchange, LE ping.
constexpr std::uint64_t LlFeatures = 0x18;

std::uint16_t get16(const std::uint8_t *p) { return static_cast<std::uint16_t>(p[0] | p[1] << 8); }

} // namespace

struct BleHost::Impl {
    mcuemu::VirtualAir &air;
    Scheduler &sched;
    std::string node;
    Config config;
    HciSink sink;
    std::shared_ptr<bool> alive{std::make_shared<bool>(true)};
    std::uint64_t rng_state;

    // ---- controller state ----
    ble::Address random_address{};
    bool random_address_set{false};
    std::vector<std::pair<std::uint8_t, ble::Address>> accept_list;

    struct Listen {
        bool on{false};
        std::uint64_t start_ns{0};
        std::uint64_t interval_ns{10000000};
        std::uint64_t window_ns{10000000};
    };
    struct ScanParams {
        bool active{false};
        std::uint16_t interval{0x10}, window{0x10};
        std::uint8_t own_type{0}, filter_policy{0};
    } scan_params;
    Listen scan;
    bool filter_duplicates{false};
    std::vector<std::pair<ble::Address, std::uint8_t>> reported; // (address, event type) already reported

    struct Initiating {
        bool on{false};
        std::uint8_t peer_type{0};
        ble::Address peer{};
        std::uint8_t own_type{0}, filter_policy{0};
        std::uint16_t interval_min{0}, interval_max{0}, latency{0}, timeout{0};
        Listen listen;
    } init;

    // A SCAN_REQ went out; the SCAN_RSP is due at `from`.
    struct Awaiting {
        bool on{false};
        ble::Address adv{};
        std::uint8_t channel{0};
        std::uint64_t from{0};
    } awaiting;
    std::vector<ble::Address> scan_responded;

    // ---- connection (one at a time) ----
    struct TxItem {
        Bytes payload;
        ble::Llid llid{ble::Llid::DataContinuation};
        bool completes_acl{false};            // the last fragment of an HCI ACL packet
        std::function<void()> on_acked;       // control procedures that end with an ack
    };
    struct Conn {
        std::uint16_t handle{0x0040};
        ble::Address peer{};
        std::uint8_t peer_type{0};
        std::uint32_t aa{0}, crc_init{0};
        std::uint16_t interval{0}, latency{0}, timeout{0};
        std::array<std::uint8_t, 5> channel_map{};
        std::unique_ptr<ble::ChannelSelection1> csa;
        std::uint64_t anchor_ns{0};      // start of the current event
        std::uint32_t event_counter{0};  // counter of the current event
        std::uint8_t channel{0};         // channel of the current event
        ble::SeqState seq;
        std::deque<TxItem> queue;
        std::optional<TxItem> inflight;
        std::uint64_t generation{0};     // invalidates scheduled work of an old connection/event
        std::uint64_t exchange{0};       // invalidates the "no reply" timer of an exchange
        bool established{false};
        std::uint64_t last_rx_ns{0};
        std::uint64_t tx_end_ns{0};
        bool waiting_reply{false};
        std::uint64_t reply_from{0}, reply_to{0};
        bool closing{false};             // drop after the next transmission (peer terminated)
        std::uint8_t closing_reason{0};
        std::uint64_t sent{0}, received{0};
        // Pending connection update.
        struct Update {
            ble::ConnectionUpdate params;
        };
        std::optional<Update> update;
        // Remote procedures asked for by the host.
        bool features_pending{false}, version_pending{false}, version_sent{false};
    };
    std::optional<Conn> conn;
    std::uint64_t generation_counter{0};

    Impl(mcuemu::VirtualAir &a, Scheduler &s, std::string n, Config c)
        : air(a), sched(s), node(std::move(n)), config(c), rng_state(c.seed * 0x9E3779B97F4A7C15ull + 1) {
        std::weak_ptr<bool> token = alive;
        Impl *self = this;
        air.add_observer([token, self](const mcuemu::RadioFrame &frame) {
            if (auto live = token.lock(); live && *live)
                self->on_frame(frame);
        });
    }

    // ---- small helpers ----

    std::uint64_t random64() {
        std::uint64_t z = (rng_state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    void at(std::uint64_t ns, std::function<void()> fn) {
        std::weak_ptr<bool> token = alive;
        sched.at(ns, [token, fn = std::move(fn)] {
            if (auto live = token.lock(); live && *live)
                fn();
        });
    }

    const ble::Address &own_address(std::uint8_t type) const {
        return type == 1 && random_address_set ? random_address : config.address;
    }

    // ---- HCI out ----

    void emit(const Bytes &packet) {
        if (sink)
            sink(packet);
    }
    void event(std::uint8_t code, const Bytes &params) {
        Bytes p{H4Event, code, static_cast<std::uint8_t>(params.size())};
        p.insert(p.end(), params.begin(), params.end());
        emit(p);
    }
    void le_event(std::uint8_t subevent, const Bytes &params) {
        Bytes body{subevent};
        body.insert(body.end(), params.begin(), params.end());
        event(EvLeMeta, body);
    }
    void command_complete(std::uint16_t op, const Bytes &return_params) {
        Bytes p{1, static_cast<std::uint8_t>(op), static_cast<std::uint8_t>(op >> 8)};
        p.insert(p.end(), return_params.begin(), return_params.end());
        event(EvCommandComplete, p);
    }
    void status_only(std::uint16_t op, std::uint8_t status) { command_complete(op, {status}); }
    void command_status(std::uint16_t op, std::uint8_t status) {
        event(EvCommandStatus, {status, 1, static_cast<std::uint8_t>(op), static_cast<std::uint8_t>(op >> 8)});
    }

    // ---- HCI in ----

    void hci_send(const std::vector<std::uint8_t> &packet) {
        if (packet.empty())
            return;
        if (packet[0] == H4Command && packet.size() >= 4) {
            const std::size_t length = packet[3];
            if (packet.size() < 4 + length)
                return;
            handle_command(get16(&packet[1]), packet.data() + 4, length);
        } else if (packet[0] == H4Acl && packet.size() >= 5) {
            const std::size_t length = get16(&packet[3]);
            if (packet.size() < 5 + length)
                return;
            handle_acl(get16(&packet[1]), packet.data() + 5, length);
        }
    }

    static Bytes with_status(std::uint8_t status, const Bytes &rest) {
        Bytes out{status};
        out.insert(out.end(), rest.begin(), rest.end());
        return out;
    }

    Bytes supported_commands() const {
        Bytes mask(64, 0);
        auto set = [&](unsigned octet, unsigned bit) { mask[octet] |= static_cast<std::uint8_t>(1u << bit); };
        set(0, 5);                                              // Disconnect
        set(5, 6); set(5, 7);                                   // Set Event Mask, Reset
        set(14, 3); set(14, 4); set(14, 5); set(14, 6); set(14, 7); // version, commands, features, ext features, buffer size
        set(15, 1);                                             // Read BD_ADDR
        set(22, 2);                                             // Set Event Mask Page 2
        set(24, 5); set(24, 6);                                 // Read/Write LE Host Supported
        set(25, 0); set(25, 1); set(25, 2); set(25, 4);         // LE event mask, buffer size, features, random address
        set(26, 2); set(26, 3); set(26, 4); set(26, 5); set(26, 6); set(26, 7); // scan, create connection, accept list
        set(27, 0); set(27, 1); set(27, 2); set(27, 5); set(27, 7); // add/remove, connection update, remote features, rand
        return mask;
    }

    void handle_command(std::uint16_t op, const std::uint8_t *p, std::size_t n) {
        switch (op) {
        case OpReset:
            reset();
            status_only(op, 0);
            return;
        case OpSetEventMask:
        case OpSetEventMaskPage2:
        case OpLeSetEventMask:
        case OpWriteLeHostSupport:
            status_only(op, n >= 1 ? 0 : ErrInvalidParameters);
            return;
        case OpReadLeHostSupport:
            command_complete(op, {0, 1, 0});
            return;
        case OpReadLocalVersion:
            // HCI/LMP version 5.1 (0x0A), manufacturer 0xFFFF (test).
            command_complete(op, {0, 0x0A, 0x00, 0x00, 0x0A, 0xFF, 0xFF, 0x00, 0x00});
            return;
        case OpReadLocalCommands:
            command_complete(op, with_status(0, supported_commands()));
            return;
        case OpReadLocalFeatures:
            // LE only: BR/EDR not supported (byte 4 bit 5), LE supported (bit 6).
            command_complete(op, {0, 0, 0, 0, 0, 0x60, 0, 0, 0});
            return;
        case OpReadLocalExtFeatures:
            if (n >= 1 && p[0] == 0)
                command_complete(op, {0, 0, 0, 0, 0, 0, 0x60, 0, 0, 0, 0});
            else
                status_only(op, ErrInvalidParameters);
            return;
        case OpReadBufferSize:
            command_complete(op, {0, static_cast<std::uint8_t>(LeAclLength), static_cast<std::uint8_t>(LeAclLength >> 8),
                                  0, LeAclPackets, 0, 0, 0});
            return;
        case OpReadBdAddr:
            command_complete(op, with_status(0, Bytes(config.address.begin(), config.address.end())));
            return;
        case OpLeReadBufferSize:
            command_complete(op, {0, static_cast<std::uint8_t>(LeAclLength), static_cast<std::uint8_t>(LeAclLength >> 8),
                                  LeAclPackets});
            return;
        case OpLeReadLocalFeatures:
            command_complete(op, {0, 0, 0, 0, 0, 0, 0, 0, 0});
            return;
        case OpLeSetRandomAddress:
            if (n != 6) {
                status_only(op, ErrInvalidParameters);
            } else if (scan.on || init.on) {
                status_only(op, ErrCommandDisallowed);
            } else {
                std::memcpy(random_address.data(), p, 6);
                random_address_set = true;
                status_only(op, 0);
            }
            return;
        case OpLeSetScanParameters: set_scan_parameters(op, p, n); return;
        case OpLeSetScanEnable: set_scan_enable(op, p, n); return;
        case OpLeCreateConnection: create_connection(op, p, n); return;
        case OpLeCreateConnectionCancel:
            if (!init.on) {
                status_only(op, ErrCommandDisallowed);
                return;
            }
            init.on = false;
            init.listen.on = false;
            status_only(op, 0);
            {
                // The pending create ends with LE Connection Complete, status 0x02.
                Bytes cancelled(18, 0);
                cancelled[0] = ErrOperationCancelled;
                le_event(LeConnectionComplete, cancelled);
            }
            return;
        case OpLeReadFilterAcceptListSize: command_complete(op, {0, 16}); return;
        case OpLeClearFilterAcceptList:
            accept_list.clear();
            status_only(op, 0);
            return;
        case OpLeAddToFilterAcceptList:
        case OpLeRemoveFromFilterAcceptList: {
            if (n != 7) {
                status_only(op, ErrInvalidParameters);
                return;
            }
            ble::Address address;
            std::memcpy(address.data(), p + 1, 6);
            const auto same = [&](const auto &entry) { return entry.first == p[0] && entry.second == address; };
            if (op == OpLeAddToFilterAcceptList) {
                if (std::none_of(accept_list.begin(), accept_list.end(), same))
                    accept_list.emplace_back(p[0], address);
            } else {
                accept_list.erase(std::remove_if(accept_list.begin(), accept_list.end(), same), accept_list.end());
            }
            status_only(op, 0);
            return;
        }
        case OpLeConnectionUpdate: connection_update(op, p, n); return;
        case OpLeReadRemoteFeatures: read_remote_features(op, p, n); return;
        case OpLeRand: {
            const std::uint64_t value = random64();
            Bytes out(8);
            std::memcpy(out.data(), &value, 8);
            command_complete(op, with_status(0, out));
            return;
        }
        case OpDisconnect: disconnect(op, p, n); return;
        case OpReadRemoteVersion: read_remote_version(op, p, n); return;
        default:
            command_status(op, ErrUnknownCommand);
            return;
        }
    }

    void reset() {
        scan.on = false;
        init.on = false;
        awaiting.on = false;
        accept_list.clear();
        random_address_set = false;
        scan_params = ScanParams{};
        filter_duplicates = false;
        reported.clear();
        scan_responded.clear();
        if (conn) {
            ++generation_counter;
            conn.reset();
        }
    }

    // ---- scanning and initiating ----

    void set_scan_parameters(std::uint16_t op, const std::uint8_t *p, std::size_t n) {
        if (n != 7) {
            status_only(op, ErrInvalidParameters);
            return;
        }
        if (scan.on) {
            status_only(op, ErrCommandDisallowed);
            return;
        }
        ScanParams params;
        params.active = p[0] == 1;
        params.interval = get16(p + 1);
        params.window = get16(p + 3);
        params.own_type = p[5];
        params.filter_policy = p[6];
        if (p[0] > 1 || params.interval < 4 || params.window < 4 || params.window > params.interval ||
            p[5] > 1 || p[6] > 3) {
            status_only(op, ErrInvalidParameters);
            return;
        }
        scan_params = params;
        status_only(op, 0);
    }

    void set_scan_enable(std::uint16_t op, const std::uint8_t *p, std::size_t n) {
        if (n != 2 || p[0] > 1) {
            status_only(op, ErrInvalidParameters);
            return;
        }
        if (p[0] == 1) {
            if (!scan.on) {
                scan.on = true;
                scan.start_ns = sched.now();
                scan.interval_ns = scan_params.interval * 625000ull;
                scan.window_ns = scan_params.window * 625000ull;
                reported.clear();
                scan_responded.clear();
            }
            filter_duplicates = p[1] != 0;
        } else {
            scan.on = false;
            awaiting.on = false;
        }
        status_only(op, 0);
    }

    void create_connection(std::uint16_t op, const std::uint8_t *p, std::size_t n) {
        if (n != 25) {
            status_only(op, ErrInvalidParameters);
            return;
        }
        if (init.on || conn) {
            command_status(op, conn ? ErrConnectionLimit : ErrCommandDisallowed);
            return;
        }
        Initiating next;
        next.listen.interval_ns = get16(p) * 625000ull;
        next.listen.window_ns = get16(p + 2) * 625000ull;
        next.filter_policy = p[4];
        next.peer_type = p[5];
        std::memcpy(next.peer.data(), p + 6, 6);
        next.own_type = p[12];
        next.interval_min = get16(p + 13);
        next.interval_max = get16(p + 15);
        next.latency = get16(p + 17);
        next.timeout = get16(p + 19);
        ble::ConnectParams check;
        check.access_address = 0x71764129u;
        check.interval = next.interval_max;
        check.latency = next.latency;
        check.timeout = next.timeout;
        if (get16(p) < 4 || get16(p + 2) < 4 || get16(p + 2) > get16(p) || next.interval_min < 6 ||
            next.interval_min > next.interval_max || next.interval_max > 3200 || next.peer_type > 1 ||
            next.own_type > 1 || next.filter_policy > 1 || !ble::connect_params_valid(check)) {
            command_status(op, ErrInvalidParameters);
            return;
        }
        next.on = true;
        next.listen.on = true;
        next.listen.start_ns = sched.now();
        init = next;
        command_status(op, 0);
    }

    static bool in_window(const Listen &l, std::uint64_t ns, std::uint8_t channel) {
        if (!l.on || ns < l.start_ns)
            return false;
        const std::uint64_t k = (ns - l.start_ns) / l.interval_ns;
        return ble::kAdvertisingChannels[k % 3] == channel && (ns - l.start_ns) % l.interval_ns < l.window_ns;
    }

    bool accepted(std::uint8_t type, const ble::Address &address) const {
        return std::any_of(accept_list.begin(), accept_list.end(),
                           [&](const auto &entry) { return entry.first == type && entry.second == address; });
    }

    // ---- air: every frame passes the observer once, when it is registered ----

    void on_frame(const mcuemu::RadioFrame &frame) {
        if (frame.protocol != mcuemu::RadioProtocol::Ble || !frame.ble.ll_framed || frame.source == node)
            return;
        if (conn) {
            on_conn_frame(frame);
            return;
        }
        if (frame.ble.access_address != ble::kAdvertisingAccessAddress)
            return;
        const bool awaited = awaiting.on && frame.channel == awaiting.channel &&
                             frame.start_ns + ReplyToleranceNs >= awaiting.from &&
                             frame.start_ns <= awaiting.from + ReplyToleranceNs;
        if (!awaited && !in_window(scan, frame.start_ns, frame.channel) &&
            !in_window(init.listen, frame.start_ns, frame.channel))
            return;
        const auto copy = frame;
        at(frame.end_ns, [this, copy, awaited] { advertising_frame_ended(copy, awaited); });
    }

    // The frame survived the medium (collisions, sensitivity) and its CRC seed matched.
    bool decoded(const mcuemu::RadioFrame &frame, std::uint32_t crc_init) {
        if (frame.ble.crc_init != crc_init)
            return false;
        for (const mcuemu::RadioFrame &heard :
             air.receive(node, mcuemu::RadioProtocol::Ble, frame.channel, frame.end_ns, frame.end_ns - 1))
            if (heard.id == frame.id)
                return true;
        return false;
    }

    void advertising_frame_ended(const mcuemu::RadioFrame &frame, bool awaited) {
        if (conn || !decoded(frame, ble::kAdvertisingCrcInit))
            return;
        const auto header = ble::parse_adv_header(frame.payload);
        const auto payload = ble::adv_payload(frame.payload);
        if (!header || !payload)
            return;
        const std::uint8_t type = static_cast<std::uint8_t>(header->type);
        if (awaited && awaiting.on) {
            awaiting.on = false;
            if (header->type == ble::AdvType::ScanRsp && payload->size() >= 6 &&
                std::memcmp(payload->data(), awaiting.adv.data(), 6) == 0) {
                scan_responded.push_back(awaiting.adv);
                report(4, header->tx_add, *payload, frame.rssi_dbm);
            }
            return;
        }
        ble::Address adv{};
        switch (header->type) {
        case ble::AdvType::AdvInd:
        case ble::AdvType::AdvNonconnInd:
        case ble::AdvType::AdvScanInd:
            if (payload->size() < 6)
                return;
            std::memcpy(adv.data(), payload->data(), 6);
            break;
        case ble::AdvType::AdvDirectInd: {
            if (payload->size() != 12)
                return;
            std::memcpy(adv.data(), payload->data(), 6);
            // Directed at somebody else: not for this scanner.
            const ble::Address &target = own_address(header->rx_add ? 1 : 0);
            if (std::memcmp(payload->data() + 6, target.data(), 6) != 0 ||
                header->rx_add != (random_address_set && scan_params.own_type == 1))
                return;
            break;
        }
        default:
            return;
        }
        const std::uint8_t adv_type = header->tx_add ? 1 : 0;

        // Initiating: a connectable advertisement from the peer starts a connection.
        if (init.on && frame.channel >= 37 &&
            (header->type == ble::AdvType::AdvInd || header->type == ble::AdvType::AdvDirectInd)) {
            const bool target = init.filter_policy == 1 ? accepted(adv_type, adv)
                                                        : (init.peer_type == adv_type && init.peer == adv);
            if (target) {
                begin_connection(frame, adv, adv_type);
                return;
            }
        }
        if (!scan.on || !in_window(scan, frame.start_ns, frame.channel))
            return;
        if (scan_params.filter_policy & 1u && !accepted(adv_type, adv))
            return;
        const bool duplicate =
            filter_duplicates &&
            std::any_of(reported.begin(), reported.end(),
                        [&](const auto &entry) { return entry.first == adv && entry.second == type; });
        if (!duplicate) {
            if (filter_duplicates)
                reported.emplace_back(adv, type);
            report(type, header->tx_add, *payload, frame.rssi_dbm);
        }
        // Active scanning asks scannable advertisers for their scan response.
        if (scan_params.active &&
            (header->type == ble::AdvType::AdvInd || header->type == ble::AdvType::AdvScanInd) &&
            !(filter_duplicates &&
              std::any_of(scan_responded.begin(), scan_responded.end(), [&](const auto &a) { return a == adv; }))) {
            const std::uint8_t own_type = scan_params.own_type;
            const Bytes request = ble::build_adv_pdu(
                ble::AdvType::ScanReq, own_type == 1, header->tx_add,
                ble::two_address_payload(own_address(own_type), adv));
            const std::uint64_t at_ns = frame.end_ns + IfsNs;
            const std::uint8_t channel = frame.channel;
            at(at_ns, [this, request, adv, channel, at_ns] {
                if (conn || !scan.on)
                    return;
                const std::uint64_t id = air.transmit(node, {}, mcuemu::RadioProtocol::Ble, channel, at_ns, request,
                                                      0, {ble::kAdvertisingAccessAddress, ble::kAdvertisingCrcInit, true});
                awaiting = {true, adv, channel, air.frames()[id - 1].end_ns + IfsNs};
            });
        }
    }

    // LE Advertising Report: `payload` is the PDU body (AdvA first).
    void report(std::uint8_t event_type, bool tx_add, const Bytes &payload, int rssi) {
        if (payload.size() < 6)
            return;
        std::size_t skip = 6;
        if (event_type == 1)
            skip = 12; // ADV_DIRECT_IND: AdvA, InitA; no data
        Bytes data(payload.begin() + std::min(skip, payload.size()), payload.end());
        Bytes p{1, event_type, static_cast<std::uint8_t>(tx_add ? 1 : 0)};
        p.insert(p.end(), payload.begin(), payload.begin() + 6);
        p.push_back(static_cast<std::uint8_t>(data.size()));
        p.insert(p.end(), data.begin(), data.end());
        p.push_back(static_cast<std::uint8_t>(static_cast<std::int8_t>(std::clamp(rssi, -127, 20))));
        le_event(LeAdvertisingReport, p);
    }

    // ---- connection setup ----

    void begin_connection(const mcuemu::RadioFrame &adv_frame, const ble::Address &adv, std::uint8_t adv_type) {
        ble::ConnectInd ind;
        ind.init_a = own_address(init.own_type);
        ind.adv_a = adv;
        ble::ConnectParams &params = ind.params;
        do
            params.access_address = static_cast<std::uint32_t>(random64());
        while (!ble::access_address_valid(params.access_address));
        params.crc_init = static_cast<std::uint32_t>(random64()) & 0xFFFFFFu;
        params.win_size = 2;
        params.win_offset = 0;
        params.interval = init.interval_max;
        params.latency = init.latency;
        params.timeout = init.timeout;
        params.channel_map = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
        params.hop = static_cast<std::uint8_t>(5 + random64() % 12);
        params.sca = 7; // this clock is exact: 0..20 ppm
        const Bytes pdu = ble::build_adv_pdu(ble::AdvType::ConnectInd, init.own_type == 1, adv_type == 1,
                                             ble::build_connect_ind_payload(ind));
        const std::uint64_t send_at = adv_frame.end_ns + IfsNs;
        const std::uint8_t channel = adv_frame.channel;
        init.on = false;
        init.listen.on = false;
        const std::uint64_t generation = ++generation_counter;
        at(send_at, [this, pdu, ind, channel, send_at, adv_type, generation] {
            if (generation != generation_counter)
                return;
            const std::uint64_t id = air.transmit(node, {}, mcuemu::RadioProtocol::Ble, channel, send_at, pdu, 0,
                                                  {ble::kAdvertisingAccessAddress, ble::kAdvertisingCrcInit, true});
            const std::uint64_t end = air.frames()[id - 1].end_ns;
            Conn c;
            c.peer = ind.adv_a;
            c.peer_type = adv_type;
            c.aa = ind.params.access_address;
            c.crc_init = ind.params.crc_init;
            c.interval = ind.params.interval;
            c.latency = ind.params.latency;
            c.timeout = ind.params.timeout;
            c.channel_map = ind.params.channel_map;
            c.csa = std::make_unique<ble::ChannelSelection1>(ind.params.hop, ind.params.channel_map);
            // The first packet goes out at the start of the transmit window.
            c.anchor_ns = end + ble::kTransmitWindowDelayNs + ind.params.win_offset * ble::kUnit1250Ns;
            c.generation = generation;
            c.last_rx_ns = c.anchor_ns;
            conn = std::move(c);
            connection_complete(0);
            const std::uint64_t anchor = conn->anchor_ns;
            at(anchor, [this, generation] { run_event(generation); });
        });
    }

    void connection_complete(std::uint8_t status) {
        Bytes p(18, 0);
        p[0] = status;
        if (conn) {
            p[1] = static_cast<std::uint8_t>(conn->handle);
            p[2] = static_cast<std::uint8_t>(conn->handle >> 8);
            p[3] = 0; // central
            p[4] = conn->peer_type;
            std::memcpy(&p[5], conn->peer.data(), 6);
            p[11] = static_cast<std::uint8_t>(conn->interval);
            p[12] = static_cast<std::uint8_t>(conn->interval >> 8);
            p[13] = static_cast<std::uint8_t>(conn->latency);
            p[14] = static_cast<std::uint8_t>(conn->latency >> 8);
            p[15] = static_cast<std::uint8_t>(conn->timeout);
            p[16] = static_cast<std::uint8_t>(conn->timeout >> 8);
            p[17] = 0;
        }
        le_event(LeConnectionComplete, p);
    }

    // ---- connection events ----

    void drop(std::uint8_t reason) {
        if (!conn)
            return;
        const std::uint16_t handle = conn->handle;
        ++generation_counter;
        conn.reset();
        event(EvDisconnectionComplete, {0, static_cast<std::uint8_t>(handle), static_cast<std::uint8_t>(handle >> 8), reason});
    }

    void run_event(std::uint64_t generation) {
        if (!conn || conn->generation != generation)
            return;
        Conn &c = *conn;
        const std::uint64_t now = sched.now();
        c.anchor_ns = now;
        // Supervision: nothing heard for the timeout, or never answered at all.
        if (!c.established && c.event_counter >= EstablishEvents) {
            drop(ErrNotEstablished);
            return;
        }
        if (c.established && now - c.last_rx_ns > static_cast<std::uint64_t>(c.timeout) * 10000000ull) {
            drop(ErrConnectionTimeout);
            return;
        }
        if (c.update && c.update->params.instant == static_cast<std::uint16_t>(c.event_counter)) {
            // The new timing starts at this event (the anchor was already moved
            // by the transmit window offset when the event was scheduled).
            c.interval = c.update->params.interval;
            c.latency = c.update->params.latency;
            c.timeout = c.update->params.timeout;
            c.update.reset();
            Bytes p{0, static_cast<std::uint8_t>(c.handle), static_cast<std::uint8_t>(c.handle >> 8),
                    static_cast<std::uint8_t>(c.interval), static_cast<std::uint8_t>(c.interval >> 8),
                    static_cast<std::uint8_t>(c.latency), static_cast<std::uint8_t>(c.latency >> 8),
                    static_cast<std::uint8_t>(c.timeout), static_cast<std::uint8_t>(c.timeout >> 8)};
            le_event(LeConnectionUpdateComplete, p);
        }
        c.channel = c.csa->next();
        send_pdu();
    }

    void schedule_next_event() {
        Conn &c = *conn;
        const std::uint64_t generation = c.generation;
        std::uint64_t next = c.anchor_ns + static_cast<std::uint64_t>(c.interval) * ble::kUnit1250Ns;
        ++c.event_counter;
        if (c.update && c.update->params.instant == static_cast<std::uint16_t>(c.event_counter))
            next = c.anchor_ns + static_cast<std::uint64_t>(c.interval) * ble::kUnit1250Ns +
                   static_cast<std::uint64_t>(c.update->params.win_offset) * ble::kUnit1250Ns;
        at(next, [this, generation] { run_event(generation); });
    }

    // Transmits the next PDU of the event (a retransmission when the last one was not acknowledged).
    void send_pdu() {
        Conn &c = *conn;
        const std::uint64_t start = sched.now();
        if (!c.inflight && !c.queue.empty() && !c.closing) {
            c.inflight = std::move(c.queue.front());
            c.queue.pop_front();
        }
        Bytes pdu;
        if (c.inflight)
            pdu = ble::build_data_pdu(c.inflight->llid, c.seq.nesn(), c.seq.sn(), !c.queue.empty(), c.inflight->payload);
        else
            pdu = ble::build_data_pdu(ble::Llid::DataContinuation, c.seq.nesn(), c.seq.sn(), false, {});
        const std::uint64_t id = air.transmit(node, {}, mcuemu::RadioProtocol::Ble, c.channel, start, pdu, 0,
                                              {c.aa, c.crc_init, true});
        c.tx_end_ns = air.frames()[id - 1].end_ns;
        ++c.sent;
        const std::uint64_t exchange_closing = c.generation;
        if (c.closing) {
            // The peer terminated: this PDU only acknowledges it. The link is
            // gone once it has been sent; whatever the peer answers is moot.
            c.waiting_reply = false;
            ++c.exchange;
            const std::uint8_t reason = c.closing_reason;
            at(c.tx_end_ns, [this, exchange_closing, reason] {
                if (conn && conn->generation == exchange_closing)
                    drop(reason);
            });
            return;
        }
        c.waiting_reply = true;
        c.reply_from = c.tx_end_ns + IfsNs - ReplyToleranceNs;
        c.reply_to = c.tx_end_ns + IfsNs + ReplyToleranceNs;
        const std::uint64_t exchange = ++c.exchange, generation = c.generation;
        // No reply started in time: the event is over (the unacknowledged PDU stays in flight).
        at(c.reply_to + 1, [this, exchange, generation] {
            if (!conn || conn->generation != generation || conn->exchange != exchange || !conn->waiting_reply)
                return;
            conn->waiting_reply = false;
            schedule_next_event();
        });
    }

    void on_conn_frame(const mcuemu::RadioFrame &frame) {
        Conn &c = *conn;
        if (!c.waiting_reply || frame.ble.access_address != c.aa || frame.channel != c.channel ||
            frame.start_ns < c.reply_from || frame.start_ns > c.reply_to)
            return;
        c.waiting_reply = false;
        ++c.exchange; // cancels the no-reply timer
        const std::uint64_t generation = c.generation;
        const auto copy = frame;
        at(frame.end_ns, [this, copy, generation] { reply_ended(copy, generation); });
    }

    void reply_ended(const mcuemu::RadioFrame &frame, std::uint64_t generation) {
        if (!conn || conn->generation != generation)
            return;
        Conn &c = *conn;
        const auto header = ble::parse_data_header(frame.payload);
        const auto payload = ble::data_payload(frame.payload);
        if (!decoded(frame, c.crc_init) || !header || !payload) {
            // A bad packet ends the event; the peer's data will be sent again.
            if (c.closing) {
                drop(c.closing_reason);
                return;
            }
            schedule_next_event();
            return;
        }
        c.established = true;
        c.last_rx_ns = frame.end_ns;
        ++c.received;
        const ble::SeqState::Result result = c.seq.on_receive(header->sn, header->nesn);
        if (result.peer_acked && c.inflight) {
            TxItem done = std::move(*c.inflight);
            c.inflight.reset();
            if (done.completes_acl) {
                const std::uint16_t handle = c.handle;
                event(EvNumberOfCompletedPackets, {1, static_cast<std::uint8_t>(handle), static_cast<std::uint8_t>(handle >> 8), 1, 0});
            }
            if (done.on_acked)
                done.on_acked();
            if (!conn || conn->generation != generation)
                return; // the acknowledged procedure ended the connection
        }
        if (result.new_data)
            receive_pdu(header->llid, *payload);
        if (!conn || conn->generation != generation)
            return;
        // Another exchange in this event when either side has more, if it fits before the next anchor.
        const std::uint64_t next_tx = frame.end_ns + IfsNs;
        const std::uint64_t next_anchor = c.anchor_ns + static_cast<std::uint64_t>(c.interval) * ble::kUnit1250Ns;
        const bool more = header->md || !c.queue.empty() || (c.inflight && !result.peer_acked);
        if (c.closing) {
            // Acknowledge the peer's TERMINATE_IND, then drop.
            at(next_tx, [this, generation] {
                if (!conn || conn->generation != generation)
                    return;
                send_pdu();
            });
            return;
        }
        if (more && next_tx + 400000 <= next_anchor) {
            at(next_tx, [this, generation] {
                if (!conn || conn->generation != generation)
                    return;
                send_pdu();
            });
            return;
        }
        schedule_next_event();
    }

    // ---- received PDUs ----

    void queue_control(Bytes payload, std::function<void()> on_acked = {}) {
        TxItem item;
        item.llid = ble::Llid::Control;
        item.payload = std::move(payload);
        item.on_acked = std::move(on_acked);
        conn->queue.push_back(std::move(item));
    }

    void receive_pdu(ble::Llid llid, const Bytes &payload) {
        Conn &c = *conn;
        if (llid == ble::Llid::DataStart || llid == ble::Llid::DataContinuation) {
            if (payload.empty())
                return;
            Bytes acl{H4Acl};
            const std::uint16_t flags = static_cast<std::uint16_t>(
                c.handle | (llid == ble::Llid::DataStart ? 0x2000 : 0x1000));
            acl.push_back(static_cast<std::uint8_t>(flags));
            acl.push_back(static_cast<std::uint8_t>(flags >> 8));
            acl.push_back(static_cast<std::uint8_t>(payload.size()));
            acl.push_back(static_cast<std::uint8_t>(payload.size() >> 8));
            acl.insert(acl.end(), payload.begin(), payload.end());
            emit(acl);
            return;
        }
        if (llid != ble::Llid::Control || payload.empty())
            return;
        switch (payload[0]) {
        case ble::ctrl::TerminateInd:
            c.closing = true;
            c.closing_reason = payload.size() > 1 ? payload[1] : ErrRemoteUserTerminated;
            break;
        case ble::ctrl::VersionInd:
            if (!c.version_sent) {
                c.version_sent = true;
                queue_control(ble::ctrl_version(0x0A, 0xFFFF, 0));
            }
            if (c.version_pending && payload.size() >= 6) {
                c.version_pending = false;
                event(EvReadRemoteVersionComplete,
                      {0, static_cast<std::uint8_t>(c.handle), static_cast<std::uint8_t>(c.handle >> 8), payload[1],
                       payload[2], payload[3], payload[4], payload[5]});
            }
            break;
        case ble::ctrl::FeatureReq:
        case ble::ctrl::PeripheralFeatureReq:
            queue_control(ble::ctrl_feature(false, LlFeatures));
            break;
        case ble::ctrl::FeatureRsp:
            if (c.features_pending && payload.size() >= 9) {
                c.features_pending = false;
                Bytes p{0, static_cast<std::uint8_t>(c.handle), static_cast<std::uint8_t>(c.handle >> 8)};
                p.insert(p.end(), payload.begin() + 1, payload.begin() + 9);
                le_event(LeReadRemoteFeaturesComplete, p);
            }
            break;
        case ble::ctrl::PingReq:
            queue_control({ble::ctrl::PingRsp});
            break;
        case ble::ctrl::UnknownRsp:
        case ble::ctrl::RejectExtInd:
            if (c.features_pending) {
                c.features_pending = false;
                Bytes p{ErrUnsupportedRemoteFeature, static_cast<std::uint8_t>(c.handle),
                        static_cast<std::uint8_t>(c.handle >> 8)};
                p.resize(11, 0);
                le_event(LeReadRemoteFeaturesComplete, p);
            }
            break;
        case ble::ctrl::PingRsp:
            break;
        default:
            // Connection parameter request, length, PHY and anything else we do not do.
            queue_control(ble::ctrl_unknown_rsp(payload[0]));
            break;
        }
    }

    // ---- ACL data from the host ----

    void handle_acl(std::uint16_t handle_flags, const std::uint8_t *data, std::size_t length) {
        if (!conn || conn->handle != (handle_flags & 0x0FFF) || length == 0)
            return;
        const bool start = ((handle_flags >> 12) & 3u) != 1;
        std::size_t offset = 0;
        bool first = true;
        while (offset < length) {
            const std::size_t piece = std::min(LlPayloadMax, length - offset);
            TxItem item;
            item.llid = first && start ? ble::Llid::DataStart : ble::Llid::DataContinuation;
            item.payload.assign(data + offset, data + offset + piece);
            offset += piece;
            item.completes_acl = offset == length;
            conn->queue.push_back(std::move(item));
            first = false;
        }
    }

    // ---- host-initiated procedures ----

    void disconnect(std::uint16_t op, const std::uint8_t *p, std::size_t n) {
        if (n != 3) {
            command_status(op, ErrInvalidParameters);
            return;
        }
        if (!conn || conn->handle != (get16(p) & 0x0FFF)) {
            command_status(op, ErrUnknownConnection);
            return;
        }
        command_status(op, 0);
        const std::uint8_t reason = p[2];
        const std::uint64_t generation = conn->generation;
        queue_control(ble::ctrl_terminate(reason), [this, generation] {
            if (conn && conn->generation == generation)
                drop(ErrLocalHostTerminated);
        });
    }

    void read_remote_version(std::uint16_t op, const std::uint8_t *p, std::size_t n) {
        if (n != 2 || !conn || conn->handle != (get16(p) & 0x0FFF)) {
            command_status(op, n != 2 ? ErrInvalidParameters : ErrUnknownConnection);
            return;
        }
        command_status(op, 0);
        conn->version_pending = true;
        if (!conn->version_sent) {
            conn->version_sent = true;
            queue_control(ble::ctrl_version(0x0A, 0xFFFF, 0));
        }
    }

    void read_remote_features(std::uint16_t op, const std::uint8_t *p, std::size_t n) {
        if (n != 2 || !conn || conn->handle != (get16(p) & 0x0FFF)) {
            command_status(op, n != 2 ? ErrInvalidParameters : ErrUnknownConnection);
            return;
        }
        command_status(op, 0);
        conn->features_pending = true;
        queue_control(ble::ctrl_feature(true, LlFeatures));
    }

    void connection_update(std::uint16_t op, const std::uint8_t *p, std::size_t n) {
        if (n != 14 || !conn || conn->handle != (get16(p) & 0x0FFF)) {
            command_status(op, n != 14 ? ErrInvalidParameters : ErrUnknownConnection);
            return;
        }
        ble::ConnectParams check;
        check.access_address = 0x71764129u;
        const std::uint16_t min = get16(p + 2), max = get16(p + 4);
        check.interval = max;
        check.latency = get16(p + 6);
        check.timeout = get16(p + 8);
        if (conn->update || min < 6 || min > max || max > 3200 || !ble::connect_params_valid(check)) {
            command_status(op, conn->update ? ErrCommandDisallowed : ErrInvalidParameters);
            return;
        }
        command_status(op, 0);
        ble::ConnectionUpdate update;
        update.win_size = 1;
        update.win_offset = 0;
        update.interval = max;
        update.latency = check.latency;
        update.timeout = check.timeout;
        // The instant is at least six events ahead of the PDU going out; the
        // current event counter is the one being (or about to be) transmitted.
        update.instant = static_cast<std::uint16_t>(conn->event_counter + 8);
        conn->update = Conn::Update{update};
        queue_control(ble::ctrl_connection_update(update));
    }
};

BleHost::BleHost(mcuemu::VirtualAir &air, Scheduler &scheduler, std::string node, Config config)
    : impl_(std::make_shared<Impl>(air, scheduler, std::move(node), config)) {}

BleHost::~BleHost() {
    if (impl_)
        *impl_->alive = false;
}

void BleHost::set_hci_sink(HciSink sink) { impl_->sink = std::move(sink); }
void BleHost::hci_send(const std::vector<std::uint8_t> &packet) { impl_->hci_send(packet); }
const std::string &BleHost::node() const noexcept { return impl_->node; }
bool BleHost::scanning() const { return impl_->scan.on; }

std::optional<BleHost::ConnectionInfo> BleHost::connection() const {
    if (!impl_->conn)
        return std::nullopt;
    const Impl::Conn &c = *impl_->conn;
    ConnectionInfo info;
    info.handle = c.handle;
    info.peer = c.peer;
    info.access_address = c.aa;
    info.crc_init = c.crc_init;
    info.interval = c.interval;
    info.latency = c.latency;
    info.timeout = c.timeout;
    info.event_counter = c.event_counter;
    info.established = c.established;
    info.packets_sent = c.sent;
    info.packets_received = c.received;
    return info;
}

} // namespace sim
