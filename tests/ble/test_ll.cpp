#include "../test_support.hpp"

#include "ble/ll.hpp"

#include <array>
#include <cstdint>

namespace mcuemu_tests {

namespace {

using namespace ble;

// A second CRC implementation in the usual reflected form (0xDA6000 is the
// polynomial 0x00065B mirrored), to check crc24()'s literal shift register.
std::array<std::uint8_t, 3> crc24_reflected(std::uint32_t init, const Bytes &data) {
    // The mirrored register holds position 0 in its top bit; the CRCInit bytes
    // are loaded in the other order (see crc24()).
    init = (init & 0xFF) << 16 | (init & 0xFF00) | (init >> 16 & 0xFF);
    std::uint32_t reg = 0;
    for (int i = 0; i < 24; ++i)
        if (init >> i & 1u)
            reg |= 1u << (23 - i);
    for (std::uint8_t byte : data) {
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t fb = ((byte >> bit) ^ reg) & 1u;
            reg >>= 1;
            if (fb)
                reg ^= 0xDA6000u;
        }
    }
    // Bit 0 of the mirrored register is position 23: it goes first, LSB first.
    return {static_cast<std::uint8_t>(reg), static_cast<std::uint8_t>(reg >> 8),
            static_cast<std::uint8_t>(reg >> 16)};
}

// The whitening shift register exactly as drawn in the specification: seven
// positions, 0 = 1, 1..6 = channel (MSB at 1), output from position 6.
Bytes whiten_by_positions(std::uint8_t channel, Bytes data) {
    bool pos[7];
    pos[0] = true;
    for (int i = 1; i <= 6; ++i)
        pos[i] = channel >> (6 - i) & 1u;
    for (auto &byte : data) {
        for (int bit = 0; bit < 8; ++bit) {
            const bool out = pos[6];
            for (int i = 6; i >= 1; --i)
                pos[i] = pos[i - 1];
            pos[0] = out;
            if (out) {
                pos[4] = !pos[4];
                byte ^= static_cast<std::uint8_t>(1u << bit);
            }
        }
    }
    return data;
}

Bytes pattern(std::size_t size, std::uint32_t seed) {
    Bytes b;
    for (std::size_t i = 0; i < size; ++i) {
        seed = seed * 1664525u + 1013904223u;
        b.push_back(static_cast<std::uint8_t>(seed >> 24));
    }
    return b;
}

} // namespace

void test_ble_crc24() {
    for (std::uint32_t init : {0x555555u, 0x000000u, 0xFFFFFFu, 0xA1B2C3u}) {
        for (std::size_t size : {0u, 1u, 2u, 17u, 39u, 257u}) {
            const Bytes data = pattern(size, init + static_cast<std::uint32_t>(size));
            CHECK(crc24(init, data) == crc24_reflected(init, data));
            // Feeding the transmitted CRC back through the register leaves zero.
            Bytes with_crc = data;
            const auto crc = crc24(init, data);
            with_crc.insert(with_crc.end(), crc.begin(), crc.end());
            CHECK((crc24(init, with_crc) == std::array<std::uint8_t, 3>{0, 0, 0}));
        }
    }
    // Vectors from scapy's BTLE.compute_crc (an independent implementation) for
    // an ADV_IND-shaped PDU, with the advertising seed and a connection seed.
    const Bytes adv{0x40, 0x24, 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6, 2, 1, 6, 3, 3, 0x0F, 0x18,
                    9,    9,    9,    9,    9,    9,    9,    9,    9, 9, 9, 9, 9, 9,    9};
    CHECK((crc24(0x555555, adv) == std::array<std::uint8_t, 3>{0xE3, 0xC9, 0xF9}));
    CHECK((crc24(0xA1B2C3, adv) == std::array<std::uint8_t, 3>{0xC0, 0x77, 0x90}));
    // Any single flipped bit changes the CRC.
    Bytes data = pattern(20, 7);
    const auto base = crc24(kAdvertisingCrcInit, data);
    for (int i = 0; i < 20 * 8; ++i) {
        Bytes bad = data;
        bad[i / 8] ^= static_cast<std::uint8_t>(1u << (i % 8));
        CHECK(crc24(kAdvertisingCrcInit, bad) != base);
    }
}

void test_ble_whitening() {
    for (std::uint8_t channel : {0, 1, 12, 36, 37, 38, 39}) {
        const Bytes data = pattern(48, channel);
        Bytes whitened = data;
        whiten(channel, whitened);
        CHECK(whitened != data);
        CHECK(whitened == whiten_by_positions(channel, data));
        whiten(channel, whitened); // involution
        CHECK(whitened == data);
    }
    // The whitening sequence of advertising channel 37 (the register starts as
    // 1100101): the well-known 8D D2 57 A1 3D A7 66 B0 ...
    Bytes zeros(8, 0);
    whiten(37, zeros);
    CHECK((zeros == Bytes{0x8D, 0xD2, 0x57, 0xA1, 0x3D, 0xA7, 0x66, 0xB0}));
    // The register has period 127: the sequence repeats after 127 bits.
    Bytes seq(40, 0);
    whiten(5, seq);
    bool periodic = true;
    for (int i = 0; i + 127 < 40 * 8; ++i)
        periodic = periodic && ((seq[i / 8] >> (i % 8) & 1) == (seq[(i + 127) / 8] >> ((i + 127) % 8) & 1));
    CHECK(periodic);
}

void test_ble_access_address_rules() {
    CHECK(!access_address_valid(kAdvertisingAccessAddress));
    CHECK(!access_address_valid(kAdvertisingAccessAddress ^ 0x00000100u)); // one bit away
    CHECK(!access_address_valid(0x00000000u));
    CHECK(!access_address_valid(0x0F0F0F0Fu)); // four equal bytes
    CHECK(!access_address_valid(0x1FFFFF12u)); // more than six ones in a row
    CHECK(!access_address_valid(0x03A5C3D2u)); // no transition in the top six bits
    CHECK(access_address_valid(0x71764129u));
}

void test_ble_adv_pdus_and_connect_ind() {
    const Address adv_a{0x11, 0x22, 0x33, 0x44, 0x55, 0xC6};
    const Bytes data{0x02, 0x01, 0x06, 0x03, 0x03, 0x0F, 0x18};
    const Bytes pdu = build_adv_pdu(AdvType::AdvInd, true, false,
                                    adv_payload_with_address(adv_a, data), true);
    CHECK(pdu.size() == 2 + 6 + data.size());
    CHECK(pdu[0] == (0x00 | 0x20 | 0x40) && pdu[1] == 13);
    const auto header = parse_adv_header(pdu);
    CHECK(header && header->type == AdvType::AdvInd && header->tx_add &&
          !header->rx_add && header->ch_sel && header->length == 13);
    const auto payload = adv_payload(pdu);
    CHECK(payload && payload->size() == 13 && (*payload)[0] == 0x11 && (*payload)[6] == 0x02);
    Bytes truncated = pdu;
    truncated.pop_back();
    CHECK(!adv_payload(truncated));
    CHECK(!parse_adv_header(Bytes{0x00}));
    const Address scan_a{1, 2, 3, 4, 5, 6};
    CHECK(two_address_payload(scan_a, adv_a).size() == 12);

    ConnectInd ind;
    ind.init_a = scan_a;
    ind.adv_a = adv_a;
    ind.params.access_address = 0x71764129u;
    ind.params.crc_init = 0xA1B2C3u;
    ind.params.win_size = 2;
    ind.params.win_offset = 3;
    ind.params.interval = 40;
    ind.params.latency = 4;
    ind.params.timeout = 300;
    ind.params.channel_map = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
    ind.params.hop = 9;
    ind.params.sca = 5;
    const Bytes raw = build_connect_ind_payload(ind);
    CHECK(raw.size() == 34);
    CHECK(raw[12] == 0x29 && raw[15] == 0x71);          // access address, little endian
    CHECK(raw[16] == 0xC3 && raw[18] == 0xA1);          // CRC init
    CHECK(raw[33] == (9 | 5 << 5));                      // hop and SCA share a byte
    const auto parsed = parse_connect_ind_payload(raw);
    CHECK(parsed && parsed->init_a == scan_a && parsed->adv_a == adv_a);
    CHECK(parsed && parsed->params.access_address == 0x71764129u &&
          parsed->params.crc_init == 0xA1B2C3u && parsed->params.win_size == 2 &&
          parsed->params.win_offset == 3 && parsed->params.interval == 40 &&
          parsed->params.latency == 4 && parsed->params.timeout == 300 &&
          parsed->params.channel_map == ind.params.channel_map &&
          parsed->params.hop == 9 && parsed->params.sca == 5);
    CHECK(!parse_connect_ind_payload(Bytes(33, 0)));
    CHECK(connect_params_valid(ind.params));
    ConnectParams bad = ind.params;
    bad.hop = 4;
    CHECK(!connect_params_valid(bad));
    bad = ind.params;
    bad.interval = 5;
    CHECK(!connect_params_valid(bad));
    bad = ind.params;
    bad.channel_map = {0x01, 0, 0, 0, 0};
    CHECK(!connect_params_valid(bad));
    bad = ind.params;
    bad.timeout = 10; // 100 ms against 5 intervals of 50 ms times two
    CHECK(!connect_params_valid(bad));
    bad = ind.params;
    bad.access_address = kAdvertisingAccessAddress;
    CHECK(!connect_params_valid(bad));
    CHECK(sca_ppm(0) == 500 && sca_ppm(7) == 20);
}

void test_ble_data_pdus_and_sequence_numbers() {
    const Bytes payload{0xDE, 0xAD};
    Bytes pdu = build_data_pdu(Llid::DataStart, true, false, true, payload);
    CHECK(pdu == (Bytes{0x02 | 0x04 | 0x10, 2, 0xDE, 0xAD}));
    auto h = parse_data_header(pdu);
    CHECK(h && h->llid == Llid::DataStart && h->nesn && !h->sn && h->md && h->length == 2);
    CHECK(data_payload(pdu) == payload);
    set_data_header_bits(pdu, false, true, false);
    h = parse_data_header(pdu);
    CHECK(h && h->llid == Llid::DataStart && !h->nesn && h->sn && !h->md);
    CHECK(!data_payload(Bytes{0x01, 3, 0}));

    // Two ends exchanging PDUs; a reply is lost and the exchange repeats.
    SeqState central, peripheral;
    // Event 1: the central sends SN 0 / NESN 0. The data is new, nothing to ack yet.
    auto r = peripheral.on_receive(central.sn(), central.nesn());
    CHECK(r.new_data && !r.peer_acked);
    // The peripheral's reply carries NESN 1: it acknowledges the central's PDU.
    CHECK(!peripheral.sn() && peripheral.nesn());
    r = central.on_receive(peripheral.sn(), peripheral.nesn());
    CHECK(r.new_data && r.peer_acked);
    // Event 2: the central sends its next PDU (SN 1, NESN 1); the reply is lost.
    CHECK(central.sn() && central.nesn());
    r = peripheral.on_receive(central.sn(), central.nesn());
    CHECK(r.new_data && r.peer_acked);
    // Event 3: the central got no acknowledgement and repeats SN 1.
    r = peripheral.on_receive(central.sn(), central.nesn());
    CHECK(!r.new_data && !r.peer_acked); // a duplicate, ignored, and the reply repeats
    r = central.on_receive(peripheral.sn(), peripheral.nesn());
    CHECK(r.new_data && r.peer_acked);
    CHECK(!central.sn()); // the central moves on to SN 0
    central.reset();
    CHECK(!central.sn() && !central.nesn());
}

void test_ble_channel_selection_1() {
    const std::array<std::uint8_t, 5> all{0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
    ChannelSelection1 plain(5, all);
    const std::uint8_t expect[] = {5, 10, 15, 20, 25, 30, 35, 3, 8, 13};
    for (std::uint8_t channel : expect)
        CHECK(plain.next() == channel);

    // Channel 7 unused: the event that lands on it is remapped by index into the
    // used channels in ascending order: 7 % 36 = 7 -> the eighth used channel, 8.
    std::array<std::uint8_t, 5> map = all;
    map[0] &= static_cast<std::uint8_t>(~(1u << 7));
    CHECK(channel_map_count(map) == 36);
    ChannelSelection1 remap(7, map);
    CHECK(remap.next() == 8);
    CHECK(remap.last_unmapped() == 7);
    CHECK(remap.next() == 14); // unmapped 14 is used

    // Only two channels in use: every event lands on one of them, and the
    // unmapped sequence still advances by the hop.
    const std::array<std::uint8_t, 5> two{0x00, 0x00, 0x00, 0x00, 0x18}; // 35 and 36
    ChannelSelection1 sparse(11, two);
    std::uint8_t unmapped = 0;
    for (int i = 0; i < 74; ++i) {
        const std::uint8_t channel = sparse.next();
        unmapped = static_cast<std::uint8_t>((unmapped + 11) % 37);
        CHECK(sparse.last_unmapped() == unmapped);
        CHECK(channel == 35 || channel == 36);
        if (unmapped == 35 || unmapped == 36)
            CHECK(channel == unmapped);
    }
    CHECK(channel_map_bits(all) == (1ull << 37) - 1);
}

void test_ble_timing_and_air_packet() {
    CHECK(window_widening_ns(500, 500, 1000000000ull) == 1000000u + 16000u);
    CHECK(window_widening_ns(50, 50, 0) == 16000u);
    CHECK(window_widening_ns(20, 20, 37500000) == 1500u + 16000u);
    CHECK(kInterFrameSpaceNs == 150000 && kUnit1250Ns == 1250000);

    // The packet as it leaves the radio: access address, whitened PDU + CRC.
    const Bytes pdu = build_data_pdu(Llid::DataStart, false, false, false, Bytes{1, 2, 3});
    const std::uint32_t aa = 0x71764129u, init = 0xA1B2C3u;
    const Bytes air = air_packet(aa, init, 9, pdu);
    CHECK(air.size() == 4 + pdu.size() + 3);
    CHECK(air[0] == 0x29 && air[3] == 0x71);
    Bytes body(air.begin() + 4, air.end());
    whiten(9, body); // a receiver de-whitens, then the CRC residue is zero
    CHECK(Bytes(body.begin(), body.end() - 3) == pdu);
    CHECK((crc24(init, body) == std::array<std::uint8_t, 3>{0, 0, 0}));
    const Bytes clear = air_packet(aa, init, 9, pdu, false);
    CHECK(Bytes(clear.begin() + 4, clear.begin() + 4 + static_cast<long>(pdu.size())) == pdu);
}

void test_ble_control_pdus() {
    CHECK(ctrl_terminate(0x13) == (Bytes{0x02, 0x13}));
    CHECK(ctrl_unknown_rsp(0x20) == (Bytes{0x07, 0x20}));
    const Bytes features = ctrl_feature(true, 0x0102030405060708ull);
    CHECK(features.size() == 9 && features[0] == ctrl::FeatureReq && features[1] == 0x08 &&
          features[8] == 0x01);
    CHECK(ctrl_feature(false, 0)[0] == ctrl::FeatureRsp);
    CHECK((ctrl_version(9, 0x0059, 0x1234) == Bytes{0x0C, 9, 0x59, 0x00, 0x34, 0x12}));
    const Bytes len = ctrl_length(true, 251, 2120, 27, 328);
    CHECK(len.size() == 9 && len[0] == ctrl::LengthReq && len[1] == 251 && len[2] == 0);

    ConnectionUpdate u;
    u.win_size = 2;
    u.win_offset = 1;
    u.interval = 80;
    u.latency = 3;
    u.timeout = 500;
    u.instant = 0x1234;
    const Bytes raw = ctrl_connection_update(u);
    CHECK(raw.size() == 12 && raw[0] == ctrl::ConnectionUpdateInd);
    const auto back = parse_connection_update(raw);
    CHECK(back && back->win_size == 2 && back->win_offset == 1 && back->interval == 80 &&
          back->latency == 3 && back->timeout == 500 && back->instant == 0x1234);
    CHECK(!parse_connection_update(Bytes(11, 0)));

    const std::array<std::uint8_t, 5> map{0xFF, 0x0F, 0x00, 0xF0, 0x1F};
    const Bytes cm = ctrl_channel_map(map, 77);
    const auto cm_back = parse_channel_map(cm);
    CHECK(cm.size() == 8 && cm_back && cm_back->first == map && cm_back->second == 77);
    CHECK(!parse_channel_map(Bytes{ctrl::TerminateInd, 1, 2, 3, 4, 5, 6, 7}));

    // Wrapped as a PDU the control LLID is 3.
    const Bytes pdu = build_data_pdu(Llid::Control, false, false, false, ctrl_terminate(0x16));
    CHECK(pdu[0] == 0x03 && pdu[1] == 2);
}

} // namespace mcuemu_tests
