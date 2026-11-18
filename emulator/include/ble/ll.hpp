#pragma once

// Bluetooth LE link layer building blocks (Core spec v5.x, Vol 6 Part B), LE 1M
// PHY, legacy advertising and data channel PDUs. No chip knowledge and no time:
// the nRF52840 RADIO and the virtual host both build on it.
//
// Everything here is written from the specification text; the sections are
// named next to each function. The test vectors are in tests/ble/.

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace ble {

using Bytes = std::vector<std::uint8_t>;
using Address = std::array<std::uint8_t, 6>; // on-air order (least significant byte first)

// ---- Constants ---------------------------------------------------------

constexpr std::uint32_t kAdvertisingAccessAddress = 0x8E89BED6u;
constexpr std::uint32_t kAdvertisingCrcInit = 0x555555u;
constexpr std::uint64_t kInterFrameSpaceNs = 150000;      // T_IFS, 4.1.1
constexpr std::uint64_t kUnit1250Ns = 1250000;            // connection timing unit
constexpr std::uint64_t kTransmitWindowDelayNs = 1250000; // 4.5.3, legacy CONNECT_IND
constexpr std::uint8_t kAdvertisingChannels[3] = {37, 38, 39};

// ---- CRC and whitening (3.1.1, 3.2) -------------------------------------

// CRC-24 of `data` (x^24+x^10+x^9+x^6+x^4+x^3+x+1) seeded with `init`, the
// CRCInit field of CONNECT_IND as a little-endian integer (0x555555 for the
// advertising channels). Returns the three bytes as they go on air, in order.
std::array<std::uint8_t, 3> crc24(std::uint32_t init, const std::uint8_t *data,
                                  std::size_t size);
inline std::array<std::uint8_t, 3> crc24(std::uint32_t init, const Bytes &data) {
    return crc24(init, data.data(), data.size());
}

// Whitening (x^7+x^4+1) seeded from the channel index. The same call
// whitens and de-whitens; it applies to the PDU and the CRC.
void whiten(std::uint8_t channel, std::uint8_t *data, std::size_t size);
inline void whiten(std::uint8_t channel, Bytes &data) {
    whiten(channel, data.data(), data.size());
}

// Access address validity rules of 2.1.2 that a controller can check without
// the air: not the advertising address, not one bit away from it, no more than
// six equal bits in a row, not all four bytes equal, no more than 24 bit
// transitions, at least two transitions in the top six bits.
bool access_address_valid(std::uint32_t access_address);

// ---- Advertising channel PDUs (2.3) -------------------------------------

enum class AdvType : std::uint8_t {
    AdvInd = 0,
    AdvDirectInd = 1,
    AdvNonconnInd = 2,
    ScanReq = 3,
    ScanRsp = 4,
    ConnectInd = 5,
    AdvScanInd = 6,
};

struct AdvHeader {
    AdvType type{AdvType::AdvInd};
    bool ch_sel{false}; // ChSel (bit 5): CSA #2 supported/used
    bool tx_add{false}; // TxAdd: the advertiser's (or initiator's) address is random
    bool rx_add{false}; // RxAdd: the target address is random
    std::uint8_t length{0};
};

// The 2-byte header followed by `payload`; length is taken from the payload.
Bytes build_adv_pdu(AdvType type, bool tx_add, bool rx_add, const Bytes &payload,
                    bool ch_sel = false);
std::optional<AdvHeader> parse_adv_header(const Bytes &pdu);
// The payload bytes of a PDU whose header length matches its size.
std::optional<Bytes> adv_payload(const Bytes &pdu);

// Advertising data helpers: AdvA (6) + data for ADV_IND/ADV_NONCONN_IND/ADV_SCAN_IND/SCAN_RSP.
Bytes adv_payload_with_address(const Address &adv_a, const Bytes &data);
// SCAN_REQ and ADV_DIRECT_IND carry two addresses (ScanA|AdvA, AdvA|InitA).
Bytes two_address_payload(const Address &first, const Address &second);

// LLData of CONNECT_IND (2.3.3.1).
struct ConnectParams {
    std::uint32_t access_address{0};
    std::uint32_t crc_init{0};
    std::uint8_t win_size{1};    // units of 1.25 ms
    std::uint16_t win_offset{0}; // units of 1.25 ms
    std::uint16_t interval{24};  // units of 1.25 ms (6..3200)
    std::uint16_t latency{0};    // connection events the peripheral may skip
    std::uint16_t timeout{200};  // units of 10 ms
    std::array<std::uint8_t, 5> channel_map{0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
    std::uint8_t hop{5};         // 5..16
    std::uint8_t sca{0};         // index into the sleep clock accuracy table
};

struct ConnectInd {
    Address init_a{};
    Address adv_a{};
    ConnectParams params;
};

// 34-byte CONNECT_IND payload. The PDU's TxAdd/RxAdd bits are the caller's.
Bytes build_connect_ind_payload(const ConnectInd &ind);
std::optional<ConnectInd> parse_connect_ind_payload(const Bytes &payload);

// The ranges the specification requires of a connection request: hop 5..16,
// interval 6..3200, timeout 10..3200, a valid access address, at least two used
// channels, and supervision timeout longer than the (latency + 1) intervals.
bool connect_params_valid(const ConnectParams &params);

// Master sleep clock accuracy ranges (2.3.3.1 table): the worst-case ppm of an
// SCA field value 0..7.
std::uint32_t sca_ppm(std::uint8_t sca);

// ---- Data channel PDUs (2.4) --------------------------------------------

enum class Llid : std::uint8_t {
    Reserved = 0,
    DataContinuation = 1, // continuation fragment of an L2CAP message, or empty
    DataStart = 2,        // start of an L2CAP message (or a complete one)
    Control = 3,
};

struct DataHeader {
    Llid llid{Llid::DataContinuation};
    bool nesn{false};
    bool sn{false};
    bool md{false};
    std::uint8_t length{0};
};

Bytes build_data_pdu(Llid llid, bool nesn, bool sn, bool md, const Bytes &payload);
std::optional<DataHeader> parse_data_header(const Bytes &pdu);
std::optional<Bytes> data_payload(const Bytes &pdu);
// Rewrite NESN/SN/MD in place (a retransmission keeps SN but a new NESN).
void set_data_header_bits(Bytes &pdu, bool nesn, bool sn, bool md);

// ---- Acknowledgement and flow control (4.5.9) ----------------------------

// One side of a connection's SN/NESN state. Feed every received data PDU's
// header to on_receive(); header_bits() gives the NESN/SN to send.
class SeqState {
public:
    struct Result {
        bool peer_acked{false}; // our last PDU was acknowledged (send a new one)
        bool new_data{false};   // the PDU is new; a repeat must be ignored
    };
    Result on_receive(bool rx_sn, bool rx_nesn);
    bool sn() const noexcept { return sn_; }
    bool nesn() const noexcept { return nesn_; }
    // A new connection starts with SN = NESN = 0 on both sides.
    void reset() noexcept { sn_ = false; nesn_ = false; }

private:
    bool sn_{false};   // SN of the PDU we send (or are retransmitting)
    bool nesn_{false}; // SN we expect next from the peer
};

// ---- Channel selection algorithm #1 (4.5.8.2) -----------------------------

// Bitmap of the used data channels (bit n = channel n) from the 5-byte map.
std::uint64_t channel_map_bits(const std::array<std::uint8_t, 5> &map);
std::uint8_t channel_map_count(const std::array<std::uint8_t, 5> &map);

class ChannelSelection1 {
public:
    ChannelSelection1(std::uint8_t hop, const std::array<std::uint8_t, 5> &map)
        : hop_(hop), map_(map) {}
    // The channel of the next connection event.
    std::uint8_t next();
    void set_map(const std::array<std::uint8_t, 5> &map) { map_ = map; }
    std::uint8_t last_unmapped() const noexcept { return last_unmapped_; }

private:
    std::uint8_t hop_;
    std::array<std::uint8_t, 5> map_;
    std::uint8_t last_unmapped_{0};
};

// ---- Connection timing (4.5.7) ---------------------------------------------

// Window widening for a listener `elapsed_ns` after the last anchor point,
// given both sides' clock accuracy in ppm, plus 16 us for the transmitter's
// active clock jitter (4.2.2).
std::uint64_t window_widening_ns(std::uint32_t master_ppm, std::uint32_t slave_ppm,
                                 std::uint64_t elapsed_ns);

// ---- Link layer control PDUs (2.4.2) ------------------------------------------

namespace ctrl {
constexpr std::uint8_t ConnectionUpdateInd = 0x00;
constexpr std::uint8_t ChannelMapInd = 0x01;
constexpr std::uint8_t TerminateInd = 0x02;
constexpr std::uint8_t UnknownRsp = 0x07;
constexpr std::uint8_t FeatureReq = 0x08;
constexpr std::uint8_t FeatureRsp = 0x09;
constexpr std::uint8_t VersionInd = 0x0C;
constexpr std::uint8_t RejectInd = 0x0D;
constexpr std::uint8_t PeripheralFeatureReq = 0x0E;
constexpr std::uint8_t ConnectionParamReq = 0x0F;
constexpr std::uint8_t ConnectionParamRsp = 0x10;
constexpr std::uint8_t RejectExtInd = 0x11;
constexpr std::uint8_t PingReq = 0x12;
constexpr std::uint8_t PingRsp = 0x13;
constexpr std::uint8_t LengthReq = 0x14;
constexpr std::uint8_t LengthRsp = 0x15;
constexpr std::uint8_t PhyReq = 0x16;
constexpr std::uint8_t PhyRsp = 0x17;
constexpr std::uint8_t PhyUpdateInd = 0x18;
} // namespace ctrl

struct ConnectionUpdate {
    std::uint8_t win_size{1};
    std::uint16_t win_offset{0};
    std::uint16_t interval{24};
    std::uint16_t latency{0};
    std::uint16_t timeout{200};
    std::uint16_t instant{0};
};

// Control PDU payloads (opcode first); wrap them with build_data_pdu(Llid::Control, ...).
Bytes ctrl_terminate(std::uint8_t error_code);
Bytes ctrl_feature(bool request, std::uint64_t features);
Bytes ctrl_version(std::uint8_t version, std::uint16_t company, std::uint16_t subversion);
Bytes ctrl_unknown_rsp(std::uint8_t unknown_opcode);
Bytes ctrl_connection_update(const ConnectionUpdate &update);
Bytes ctrl_channel_map(const std::array<std::uint8_t, 5> &map, std::uint16_t instant);
Bytes ctrl_length(bool request, std::uint16_t rx_octets, std::uint16_t rx_time,
                  std::uint16_t tx_octets, std::uint16_t tx_time);
std::optional<ConnectionUpdate> parse_connection_update(const Bytes &payload);
std::optional<std::pair<std::array<std::uint8_t, 5>, std::uint16_t>> parse_channel_map(
    const Bytes &payload);

// ---- Frame helpers --------------------------------------------------------------

// The complete on-air bytes after the preamble: access address, PDU and CRC
// (whitened with `channel` when `do_whiten`). Used by the register-level radios.
Bytes air_packet(std::uint32_t access_address, std::uint32_t crc_init,
                 std::uint8_t channel, const Bytes &pdu, bool do_whiten = true);

} // namespace ble
