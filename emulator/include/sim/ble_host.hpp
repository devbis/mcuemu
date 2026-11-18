#pragma once

// A virtual Bluetooth LE host: an HCI controller plus the link layer of a
// scanner/initiator/central, living on the shared radio medium as one node.
// Firmware under test is a peripheral (advertising, connectable); this is the
// phone. Whatever drives it speaks HCI over H4 packets (type byte, then the
// packet), so BlueZ or Bumble can sit on top through a transport, and tests can
// drive it directly.
//
// Scope (v1): LE 1M, legacy advertising, one connection at a time, no LL
// encryption, no data length extension, no 2M/Coded PHY, no extended or
// periodic advertising, no channel map update, no privacy. Scanning pauses
// while a connection is up. Behaviour that the specification leaves to the
// controller is documented in docs/ble.md.

#include "ble/ll.hpp"
#include "mcuemu/radio.hpp"
#include "sim/part.hpp"
#include "sim/scheduler.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim {

class BleHost {
public:
    struct Config {
        // Public device address (BD_ADDR), least significant byte first, as it
        // appears on air.
        ble::Address address{0x01, 0x00, 0x00, 0xDE, 0xC0, 0x00};
        std::uint64_t seed{1}; // access addresses, CRC seeds, hop increments, LE Rand
    };

    BleHost(mcuemu::VirtualAir &air, Scheduler &scheduler, std::string node, Config config);
    ~BleHost();
    BleHost(const BleHost &) = delete;
    BleHost &operator=(const BleHost &) = delete;

    // Packets for the host stack (H4: 0x04 event, 0x02 ACL data).
    using HciSink = std::function<void(const std::vector<std::uint8_t> &)>;
    void set_hci_sink(HciSink sink);
    // A packet from the host stack (H4: 0x01 command, 0x02 ACL data). Replies
    // and events go to the sink, synchronously for command replies.
    void hci_send(const std::vector<std::uint8_t> &packet);

    const std::string &node() const noexcept;

    struct ConnectionInfo {
        std::uint16_t handle{0};
        ble::Address peer{};
        std::uint32_t access_address{0};
        std::uint32_t crc_init{0};
        std::uint16_t interval{0}; // units of 1.25 ms
        std::uint16_t latency{0};
        std::uint16_t timeout{0}; // units of 10 ms
        std::uint32_t event_counter{0};
        bool established{false}; // the peripheral has answered at least once
        std::uint64_t packets_sent{0};
        std::uint64_t packets_received{0};
    };
    std::optional<ConnectionInfo> connection() const;
    bool scanning() const;

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

// The `ble_host` part type (parts/ble_host_part.cpp).
std::unique_ptr<Part> make_ble_host_part(PartContext &context);

} // namespace sim
