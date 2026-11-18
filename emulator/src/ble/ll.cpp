#include "ble/ll.hpp"

#include <algorithm>
#include <bit>

namespace ble {

namespace {

std::uint8_t reverse8(std::uint8_t v) {
    v = static_cast<std::uint8_t>((v & 0xF0) >> 4 | (v & 0x0F) << 4);
    v = static_cast<std::uint8_t>((v & 0xCC) >> 2 | (v & 0x33) << 2);
    v = static_cast<std::uint8_t>((v & 0xAA) >> 1 | (v & 0x55) << 1);
    return v;
}

void put16(Bytes &b, std::uint16_t v) {
    b.push_back(static_cast<std::uint8_t>(v));
    b.push_back(static_cast<std::uint8_t>(v >> 8));
}

void put32(Bytes &b, std::uint32_t v) {
    for (int shift = 0; shift < 32; shift += 8)
        b.push_back(static_cast<std::uint8_t>(v >> shift));
}

std::uint16_t get16(const std::uint8_t *p) {
    return static_cast<std::uint16_t>(p[0] | p[1] << 8);
}

std::uint32_t get32(const std::uint8_t *p) {
    return static_cast<std::uint32_t>(p[0]) | static_cast<std::uint32_t>(p[1]) << 8 |
           static_cast<std::uint32_t>(p[2]) << 16 | static_cast<std::uint32_t>(p[3]) << 24;
}

} // namespace

// 3.1.1: the shift register has positions 0..23 and the data bit is XORed with
// position 23 on its way in. Bits go through least significant first and the CRC
// leaves starting at position 23.
std::array<std::uint8_t, 3> crc24(std::uint32_t init, const std::uint8_t *data,
                                  std::size_t size) {
    constexpr std::uint32_t taps = 0x00065B; // positions 0,1,3,4,6,9,10
    // `init` is the CRCInit field read as a little-endian integer (byte 0 is the
    // one sent first in CONNECT_IND). The register takes its bytes in the other
    // order, as in Wireshark's and Ubertooth's CRC and scapy's BTLE.compute_crc
    // (the advertising value 0x555555 reads the same either way).
    std::uint32_t reg = (init & 0xFF) << 16 | (init & 0xFF00) | (init >> 16 & 0xFF);
    for (std::size_t i = 0; i < size; ++i) {
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t feedback = ((data[i] >> bit) ^ (reg >> 23)) & 1u;
            reg = (reg << 1) & 0xFFFFFF;
            if (feedback)
                reg ^= taps;
        }
    }
    return {reverse8(static_cast<std::uint8_t>(reg >> 16)),
            reverse8(static_cast<std::uint8_t>(reg >> 8)),
            reverse8(static_cast<std::uint8_t>(reg))};
}

// 3.2: position 0 is 1 and positions 1..6 hold the channel index, most
// significant bit first. Position 6 is the output. Mirrored into a byte (position
// k is bit 6-k) one step is a right shift that feeds the output back into the top
// and into position 4.
void whiten(std::uint8_t channel, std::uint8_t *data, std::size_t size) {
    std::uint8_t reg = static_cast<std::uint8_t>(0x40 | (channel & 0x3F));
    for (std::size_t i = 0; i < size; ++i) {
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint8_t out = reg & 1u;
            reg = static_cast<std::uint8_t>(reg >> 1);
            if (out) {
                reg ^= 0x44;
                data[i] ^= static_cast<std::uint8_t>(1u << bit);
            }
        }
    }
}

bool access_address_valid(std::uint32_t aa) {
    if (aa == kAdvertisingAccessAddress ||
        std::popcount(aa ^ kAdvertisingAccessAddress) <= 1)
        return false;
    const std::uint8_t b0 = static_cast<std::uint8_t>(aa);
    if (b0 == static_cast<std::uint8_t>(aa >> 8) &&
        b0 == static_cast<std::uint8_t>(aa >> 16) &&
        b0 == static_cast<std::uint8_t>(aa >> 24))
        return false;
    int run = 1, longest = 1, transitions = 0, top_transitions = 0;
    for (int i = 1; i < 32; ++i) {
        const bool a = (aa >> (i - 1)) & 1u, b = (aa >> i) & 1u;
        if (a == b) {
            longest = std::max(longest, ++run);
        } else {
            run = 1;
            ++transitions;
            if (i >= 27) // transitions among the six most significant bits (26..31)
                ++top_transitions;
        }
    }
    return longest <= 6 && transitions <= 24 && top_transitions >= 2;
}

// ---- Advertising PDUs -----------------------------------------------------

Bytes build_adv_pdu(AdvType type, bool tx_add, bool rx_add, const Bytes &payload,
                    bool ch_sel) {
    Bytes pdu;
    pdu.push_back(static_cast<std::uint8_t>(static_cast<std::uint8_t>(type) & 0x0F) |
                  (ch_sel ? 0x20 : 0) | (tx_add ? 0x40 : 0) | (rx_add ? 0x80 : 0));
    pdu.push_back(static_cast<std::uint8_t>(payload.size()));
    pdu.insert(pdu.end(), payload.begin(), payload.end());
    return pdu;
}

std::optional<AdvHeader> parse_adv_header(const Bytes &pdu) {
    if (pdu.size() < 2)
        return std::nullopt;
    AdvHeader h;
    h.type = static_cast<AdvType>(pdu[0] & 0x0F);
    h.ch_sel = pdu[0] & 0x20;
    h.tx_add = pdu[0] & 0x40;
    h.rx_add = pdu[0] & 0x80;
    h.length = pdu[1] & 0x3F; // legacy PDUs have six length bits (up to 37)
    return h;
}

std::optional<Bytes> adv_payload(const Bytes &pdu) {
    const auto h = parse_adv_header(pdu);
    if (!h || pdu.size() != 2u + h->length)
        return std::nullopt;
    return Bytes(pdu.begin() + 2, pdu.end());
}

Bytes adv_payload_with_address(const Address &adv_a, const Bytes &data) {
    Bytes out(adv_a.begin(), adv_a.end());
    out.insert(out.end(), data.begin(), data.end());
    return out;
}

Bytes two_address_payload(const Address &first, const Address &second) {
    Bytes out(first.begin(), first.end());
    out.insert(out.end(), second.begin(), second.end());
    return out;
}

Bytes build_connect_ind_payload(const ConnectInd &ind) {
    Bytes b(ind.init_a.begin(), ind.init_a.end());
    b.insert(b.end(), ind.adv_a.begin(), ind.adv_a.end());
    const ConnectParams &p = ind.params;
    put32(b, p.access_address);
    b.push_back(static_cast<std::uint8_t>(p.crc_init));
    b.push_back(static_cast<std::uint8_t>(p.crc_init >> 8));
    b.push_back(static_cast<std::uint8_t>(p.crc_init >> 16));
    b.push_back(p.win_size);
    put16(b, p.win_offset);
    put16(b, p.interval);
    put16(b, p.latency);
    put16(b, p.timeout);
    b.insert(b.end(), p.channel_map.begin(), p.channel_map.end());
    b.push_back(static_cast<std::uint8_t>((p.hop & 0x1F) | (p.sca & 7) << 5));
    return b;
}

std::optional<ConnectInd> parse_connect_ind_payload(const Bytes &payload) {
    if (payload.size() != 34)
        return std::nullopt;
    ConnectInd ind;
    std::copy_n(payload.begin(), 6, ind.init_a.begin());
    std::copy_n(payload.begin() + 6, 6, ind.adv_a.begin());
    const std::uint8_t *d = payload.data() + 12;
    ConnectParams &p = ind.params;
    p.access_address = get32(d);
    p.crc_init = static_cast<std::uint32_t>(d[4]) | static_cast<std::uint32_t>(d[5]) << 8 |
                 static_cast<std::uint32_t>(d[6]) << 16;
    p.win_size = d[7];
    p.win_offset = get16(d + 8);
    p.interval = get16(d + 10);
    p.latency = get16(d + 12);
    p.timeout = get16(d + 14);
    std::copy_n(d + 16, 5, p.channel_map.begin());
    p.hop = d[21] & 0x1F;
    p.sca = d[21] >> 5;
    return ind;
}

bool connect_params_valid(const ConnectParams &p) {
    if (!access_address_valid(p.access_address) || p.hop < 5 || p.hop > 16 ||
        p.interval < 6 || p.interval > 3200 || p.timeout < 10 || p.timeout > 3200 ||
        channel_map_count(p.channel_map) < 2 || (p.channel_map[4] & 0xE0) != 0)
        return false;
    // 4.5.2: the supervision timeout must exceed (1 + latency) * interval * 2
    // (interval in 1.25 ms, timeout in 10 ms).
    return static_cast<std::uint64_t>(p.timeout) * 10 >
           static_cast<std::uint64_t>(1 + p.latency) * p.interval * 2 * 1250 / 1000;
}

std::uint32_t sca_ppm(std::uint8_t sca) {
    static constexpr std::uint32_t ppm[8] = {500, 250, 150, 100, 75, 50, 30, 20};
    return ppm[sca & 7];
}

// ---- Data PDUs ---------------------------------------------------------------

Bytes build_data_pdu(Llid llid, bool nesn, bool sn, bool md, const Bytes &payload) {
    Bytes pdu;
    pdu.push_back(static_cast<std::uint8_t>(static_cast<std::uint8_t>(llid) & 3) |
                  (nesn ? 0x04 : 0) | (sn ? 0x08 : 0) | (md ? 0x10 : 0));
    pdu.push_back(static_cast<std::uint8_t>(payload.size()));
    pdu.insert(pdu.end(), payload.begin(), payload.end());
    return pdu;
}

std::optional<DataHeader> parse_data_header(const Bytes &pdu) {
    if (pdu.size() < 2)
        return std::nullopt;
    DataHeader h;
    h.llid = static_cast<Llid>(pdu[0] & 3);
    h.nesn = pdu[0] & 0x04;
    h.sn = pdu[0] & 0x08;
    h.md = pdu[0] & 0x10;
    h.length = pdu[1];
    return h;
}

std::optional<Bytes> data_payload(const Bytes &pdu) {
    const auto h = parse_data_header(pdu);
    if (!h || pdu.size() != 2u + h->length)
        return std::nullopt;
    return Bytes(pdu.begin() + 2, pdu.end());
}

void set_data_header_bits(Bytes &pdu, bool nesn, bool sn, bool md) {
    if (pdu.empty())
        return;
    pdu[0] = static_cast<std::uint8_t>((pdu[0] & ~0x1C) | (nesn ? 0x04 : 0) |
                                       (sn ? 0x08 : 0) | (md ? 0x10 : 0));
}

SeqState::Result SeqState::on_receive(bool rx_sn, bool rx_nesn) {
    Result r;
    if (rx_nesn != sn_) { // the peer expects the PDU after ours: ours arrived
        sn_ = !sn_;
        r.peer_acked = true;
    }
    if (rx_sn == nesn_) { // the sequence number we were waiting for
        nesn_ = !nesn_;
        r.new_data = true;
    }
    return r;
}

// ---- Channel selection #1 ------------------------------------------------------

std::uint64_t channel_map_bits(const std::array<std::uint8_t, 5> &map) {
    std::uint64_t bits = 0;
    for (int i = 0; i < 5; ++i)
        bits |= static_cast<std::uint64_t>(map[i]) << (8 * i);
    return bits & ((1ull << 37) - 1);
}

std::uint8_t channel_map_count(const std::array<std::uint8_t, 5> &map) {
    return static_cast<std::uint8_t>(std::popcount(channel_map_bits(map)));
}

std::uint8_t ChannelSelection1::next() {
    last_unmapped_ = static_cast<std::uint8_t>((last_unmapped_ + hop_) % 37);
    const std::uint64_t bits = channel_map_bits(map_);
    if (bits >> last_unmapped_ & 1u)
        return last_unmapped_;
    const unsigned used = static_cast<unsigned>(std::popcount(bits));
    if (used == 0)
        return last_unmapped_;
    unsigned index = last_unmapped_ % used;
    for (std::uint8_t channel = 0; channel < 37; ++channel) {
        if (!(bits >> channel & 1u))
            continue;
        if (index-- == 0)
            return channel;
    }
    return last_unmapped_;
}

std::uint64_t window_widening_ns(std::uint32_t master_ppm, std::uint32_t slave_ppm,
                                 std::uint64_t elapsed_ns) {
    constexpr std::uint64_t jitter_ns = 16000;
    const std::uint64_t drift =
        (elapsed_ns * (static_cast<std::uint64_t>(master_ppm) + slave_ppm) + 999999) /
        1000000;
    return drift + jitter_ns;
}

// ---- Control PDUs ----------------------------------------------------------------

Bytes ctrl_terminate(std::uint8_t error_code) { return {ctrl::TerminateInd, error_code}; }

Bytes ctrl_feature(bool request, std::uint64_t features) {
    Bytes b{request ? ctrl::FeatureReq : ctrl::FeatureRsp};
    for (int i = 0; i < 8; ++i)
        b.push_back(static_cast<std::uint8_t>(features >> (8 * i)));
    return b;
}

Bytes ctrl_version(std::uint8_t version, std::uint16_t company, std::uint16_t subversion) {
    Bytes b{ctrl::VersionInd, version};
    put16(b, company);
    put16(b, subversion);
    return b;
}

Bytes ctrl_unknown_rsp(std::uint8_t unknown_opcode) {
    return {ctrl::UnknownRsp, unknown_opcode};
}

Bytes ctrl_connection_update(const ConnectionUpdate &u) {
    Bytes b{ctrl::ConnectionUpdateInd, u.win_size};
    put16(b, u.win_offset);
    put16(b, u.interval);
    put16(b, u.latency);
    put16(b, u.timeout);
    put16(b, u.instant);
    return b;
}

Bytes ctrl_channel_map(const std::array<std::uint8_t, 5> &map, std::uint16_t instant) {
    Bytes b{ctrl::ChannelMapInd};
    b.insert(b.end(), map.begin(), map.end());
    put16(b, instant);
    return b;
}

Bytes ctrl_length(bool request, std::uint16_t rx_octets, std::uint16_t rx_time,
                  std::uint16_t tx_octets, std::uint16_t tx_time) {
    Bytes b{request ? ctrl::LengthReq : ctrl::LengthRsp};
    put16(b, rx_octets);
    put16(b, rx_time);
    put16(b, tx_octets);
    put16(b, tx_time);
    return b;
}

std::optional<ConnectionUpdate> parse_connection_update(const Bytes &payload) {
    if (payload.size() != 12 || payload[0] != ctrl::ConnectionUpdateInd)
        return std::nullopt;
    ConnectionUpdate u;
    u.win_size = payload[1];
    u.win_offset = get16(&payload[2]);
    u.interval = get16(&payload[4]);
    u.latency = get16(&payload[6]);
    u.timeout = get16(&payload[8]);
    u.instant = get16(&payload[10]);
    return u;
}

std::optional<std::pair<std::array<std::uint8_t, 5>, std::uint16_t>> parse_channel_map(
    const Bytes &payload) {
    if (payload.size() != 8 || payload[0] != ctrl::ChannelMapInd)
        return std::nullopt;
    std::array<std::uint8_t, 5> map{};
    std::copy_n(payload.begin() + 1, 5, map.begin());
    return std::make_pair(map, get16(&payload[6]));
}

Bytes air_packet(std::uint32_t access_address, std::uint32_t crc_init,
                 std::uint8_t channel, const Bytes &pdu, bool do_whiten) {
    Bytes body = pdu;
    const auto crc = crc24(crc_init, pdu);
    body.insert(body.end(), crc.begin(), crc.end());
    if (do_whiten)
        whiten(channel, body);
    Bytes out;
    put32(out, access_address);
    out.insert(out.end(), body.begin(), body.end());
    return out;
}

} // namespace ble
