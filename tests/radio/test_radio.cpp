#include "../test_support.hpp"
#include <array>
#include <cstdint>
#include <vector>
#include "mcuemu/radio.hpp"
#include "mcuemu/tlsr8258.hpp"

namespace mcuemu_tests {

void test_virtual_air() {
    mcuemu::VirtualAir air;
    mcuemu::VirtualRadioNode switch_node(air, "switch");
    mcuemu::VirtualRadioNode light_node(air, "light");
    const auto id = switch_node.transmit(
        "light", mcuemu::RadioProtocol::Zigbee802154, 15, 0,
        {0x01, 0x02, 0x03});
    CHECK(id == 1);
    const std::uint64_t frame_end =
        (std::uint64_t{3} + 8u) * 8u * 1000000000u / 250000u; // + preamble, SFD, PHR, FCS
    CHECK(light_node.receive(mcuemu::RadioProtocol::Zigbee802154, 15,
                             frame_end).size() == 1);
    CHECK(air.receptions().size() == 1);
    CHECK(air.receptions()[0].frame_id == id);
    CHECK(air.receptions()[0].receiver == "light");
    CHECK(light_node.receive(mcuemu::RadioProtocol::Zigbee802154, 15,
                             frame_end).empty());
    CHECK(light_node.receive(mcuemu::RadioProtocol::Zigbee802154, 14,
                             frame_end).empty());

    mcuemu::VirtualAir collided;
    collided.transmit("a", "c", mcuemu::RadioProtocol::Ble, 37, 0, {1}, 1000000);
    collided.transmit("b", "c", mcuemu::RadioProtocol::Ble, 37, 0, {2}, 1000000);
    CHECK(collided.receive("c", mcuemu::RadioProtocol::Ble, 37, 1000000).empty());
    CHECK(collided.receptions().empty());

    mcuemu::VirtualAir delayed_start;
    delayed_start.transmit("coordinator", "", mcuemu::RadioProtocol::Zigbee802154,
                           15, 100000, {0x07});
    CHECK(delayed_start.receive("late-node", mcuemu::RadioProtocol::Zigbee802154,
                                15, 1000000, 500000).empty());
    CHECK(delayed_start.receptions().empty());
}

void test_tlsr_radio_dma_packet_exchange() {
    mcuemu::VirtualAir air;
    mcuemu::Tlsr8258 sender;
    mcuemu::Tlsr8258 receiver;
    sender.connect_radio(air, "sender");
    receiver.connect_radio(air, "receiver");
    sender.set_radio_time_offset(5000000000u);
    receiver.set_radio_time_offset(5000000000u);
    sender.set_radio_channel(11);
    receiver.set_radio_channel(11);
    // Firmware rf_set_channel() programs the synthesizer directly. A channel
    // 20 setting must override the host's initial channel until firmware
    // switches back to channel 11.
    sender.write8(mcuemu::Tlsr8258::RfSynthFrequencyLow, 0x49);
    sender.write8(mcuemu::Tlsr8258::RfSynthFrequencyHigh, 0x26);

    constexpr std::uint32_t tx_buffer =
        mcuemu::Tlsr8258::RetainedSramBase + 0x100;
    constexpr std::uint32_t rx_buffer =
        mcuemu::Tlsr8258::RetainedSramBase + 0x200;
    const std::array<std::uint8_t, 6> mac_frame{
        0x61, 0x88, 0x01, 0x00, 0x00, 0x00,
    };
    sender.write8(tx_buffer, static_cast<std::uint8_t>(mac_frame.size() + 1));
    sender.write8(tx_buffer + 4, static_cast<std::uint8_t>(mac_frame.size() + 2));
    for (std::size_t i = 0; i != mac_frame.size(); ++i)
        sender.write8(tx_buffer + 5 + static_cast<std::uint32_t>(i), mac_frame[i]);

    const auto set_dma_pointer = [](mcuemu::Tlsr8258 &device,
                                    std::uint32_t low_register,
                                    std::uint32_t high_register,
                                    std::uint32_t address) {
        device.write8(low_register, static_cast<std::uint8_t>(address));
        device.write8(low_register + 1,
                     static_cast<std::uint8_t>(address >> 8));
        device.write8(high_register,
                     static_cast<std::uint8_t>((address >> 16) & 0x7fu));
    };
    set_dma_pointer(sender, mcuemu::Tlsr8258::RfTxDmaAddress,
                    mcuemu::Tlsr8258::RfTxDmaAddressHigh, tx_buffer);
    set_dma_pointer(receiver, mcuemu::Tlsr8258::RfRxDmaAddress,
                    mcuemu::Tlsr8258::RfRxDmaAddressHigh, rx_buffer);

    receiver.write8(mcuemu::Tlsr8258::RfDmaControl, 0x01);
    receiver.write8(mcuemu::Tlsr8258::DmaChannelEnable,
                    mcuemu::Tlsr8258::DmaRfRxEnable);
    receiver.write8(mcuemu::Tlsr8258::RfMode, 0x65); // RF RX enable.
    receiver.write8(mcuemu::Tlsr8258::RfIrqMask, 0x03);
    sender.write8(mcuemu::Tlsr8258::RfIrqMask, 0x03);
    sender.write8(mcuemu::Tlsr8258::RfMode, 0x55); // RF TX enable.
    sender.write8(mcuemu::Tlsr8258::DmaTxReady0, 0x08);

    sender.tick(30000);
    for (unsigned poll = 0; poll != 54; ++poll)
        receiver.tick(480); // Poll at 10 us intervals through the full frame.
    CHECK(air.frames().size() == 1);
    CHECK(air.frames()[0].channel == 20);
    CHECK((receiver.read8(mcuemu::Tlsr8258::RfIrqStatus) & 0x01u) == 0);

    sender.write8(mcuemu::Tlsr8258::RfSynthFrequencyLow, 0x95);
    sender.write8(mcuemu::Tlsr8258::RfSynthFrequencyHigh, 0x25);
    sender.write8(mcuemu::Tlsr8258::DmaTxReady0, 0x08);
    sender.tick(30000);
    for (unsigned poll = 0; poll != 54; ++poll)
        receiver.tick(480);
    CHECK(air.frames().size() == 2);
    CHECK(air.frames()[1].channel == 11);
    CHECK(air.frames()[0].payload ==
          std::vector<std::uint8_t>(mac_frame.begin(), mac_frame.end()));
    CHECK((sender.read8(mcuemu::Tlsr8258::RfIrqStatus) & 0x02u) != 0);
    CHECK((receiver.read8(mcuemu::Tlsr8258::RfIrqStatus) & 0x01u) != 0);
    CHECK((sender.read8(mcuemu::Tlsr8258::DmaTxReady0) & 0x08u) == 0);
    CHECK(sender.read8(0x800649) & 0x20u);
    CHECK(receiver.read8(0x800649) & 0x20u);
    CHECK(receiver.read8(rx_buffer) == mac_frame.size() + 11);
    CHECK(receiver.read8(rx_buffer + 4) == mac_frame.size() + 2);
    for (std::size_t i = 0; i != mac_frame.size(); ++i)
        CHECK(receiver.read8(rx_buffer + 5 + static_cast<std::uint32_t>(i)) ==
              mac_frame[i]);
    sender.write8(mcuemu::Tlsr8258::RfIrqStatus, 0x02);
    receiver.write8(mcuemu::Tlsr8258::RfIrqStatus, 0x01);
    CHECK((sender.read8(0x800649) & 0x20u) == 0);
    CHECK((receiver.read8(0x800649) & 0x20u) == 0);
}

} // namespace mcuemu_tests
