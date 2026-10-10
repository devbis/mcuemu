#include "../test_support.hpp"
#include <array>
#include <cstdint>
#include <vector>
#include "tc32emu/radio.hpp"
#include "tc32emu/tlsr8258.hpp"

namespace tc32emu_tests {

void test_virtual_air() {
    tc32emu::VirtualAir air;
    tc32emu::VirtualRadioNode switch_node(air, "switch");
    tc32emu::VirtualRadioNode light_node(air, "light");
    const auto id = switch_node.transmit(
        "light", tc32emu::RadioProtocol::Zigbee802154, 15, 0,
        {0x01, 0x02, 0x03});
    CHECK(id == 1);
    const std::uint64_t frame_end =
        (std::uint64_t{3} + 6u) * 8u * 1000000000u / 250000u;
    CHECK(light_node.receive(tc32emu::RadioProtocol::Zigbee802154, 15,
                             frame_end).size() == 1);
    CHECK(light_node.receive(tc32emu::RadioProtocol::Zigbee802154, 15,
                             frame_end).empty());
    CHECK(light_node.receive(tc32emu::RadioProtocol::Zigbee802154, 14,
                             frame_end).empty());

    tc32emu::VirtualAir collided;
    collided.transmit("a", "c", tc32emu::RadioProtocol::Ble, 37, 0, {1}, 1000000);
    collided.transmit("b", "c", tc32emu::RadioProtocol::Ble, 37, 0, {2}, 1000000);
    CHECK(collided.receive("c", tc32emu::RadioProtocol::Ble, 37, 1000000).empty());
}

void test_tlsr_radio_dma_packet_exchange() {
    tc32emu::VirtualAir air;
    tc32emu::Tlsr8258 sender;
    tc32emu::Tlsr8258 receiver;
    sender.connect_radio(air, "sender");
    receiver.connect_radio(air, "receiver");
    sender.set_radio_channel(11);
    receiver.set_radio_channel(11);
    // Firmware rf_set_channel() programs the synthesizer directly. A channel
    // 20 setting must override the host's initial channel until firmware
    // switches back to channel 11.
    sender.write8(tc32emu::Tlsr8258::RfSynthFrequencyLow, 0x49);
    sender.write8(tc32emu::Tlsr8258::RfSynthFrequencyHigh, 0x26);

    constexpr std::uint32_t tx_buffer =
        tc32emu::Tlsr8258::RetainedSramBase + 0x100;
    constexpr std::uint32_t rx_buffer =
        tc32emu::Tlsr8258::RetainedSramBase + 0x200;
    const std::array<std::uint8_t, 6> mac_frame{
        0x61, 0x88, 0x01, 0x00, 0x00, 0x00,
    };
    sender.write8(tx_buffer, static_cast<std::uint8_t>(mac_frame.size() + 1));
    sender.write8(tx_buffer + 4, static_cast<std::uint8_t>(mac_frame.size() + 2));
    for (std::size_t i = 0; i != mac_frame.size(); ++i)
        sender.write8(tx_buffer + 5 + static_cast<std::uint32_t>(i), mac_frame[i]);

    const auto set_dma_pointer = [](tc32emu::Tlsr8258 &device,
                                    std::uint32_t low_register,
                                    std::uint32_t high_register,
                                    std::uint32_t address) {
        device.write8(low_register, static_cast<std::uint8_t>(address));
        device.write8(low_register + 1,
                     static_cast<std::uint8_t>(address >> 8));
        device.write8(high_register,
                     static_cast<std::uint8_t>((address >> 16) & 0x7fu));
    };
    set_dma_pointer(sender, tc32emu::Tlsr8258::RfTxDmaAddress,
                    tc32emu::Tlsr8258::RfTxDmaAddressHigh, tx_buffer);
    set_dma_pointer(receiver, tc32emu::Tlsr8258::RfRxDmaAddress,
                    tc32emu::Tlsr8258::RfRxDmaAddressHigh, rx_buffer);

    receiver.write8(tc32emu::Tlsr8258::RfDmaControl, 0x01);
    receiver.write8(tc32emu::Tlsr8258::RfMode, 0x65); // RF RX enable.
    receiver.write8(tc32emu::Tlsr8258::RfIrqMask, 0x03);
    sender.write8(tc32emu::Tlsr8258::RfIrqMask, 0x03);
    sender.write8(tc32emu::Tlsr8258::RfMode, 0x55); // RF TX enable.
    sender.write8(tc32emu::Tlsr8258::DmaTxReady0, 0x08);

    sender.tick(20000);
    receiver.tick(20000);
    CHECK(air.frames().size() == 1);
    CHECK(air.frames()[0].channel == 20);
    CHECK((receiver.read8(tc32emu::Tlsr8258::RfIrqStatus) & 0x01u) == 0);

    sender.write8(tc32emu::Tlsr8258::RfSynthFrequencyLow, 0x95);
    sender.write8(tc32emu::Tlsr8258::RfSynthFrequencyHigh, 0x25);
    sender.write8(tc32emu::Tlsr8258::DmaTxReady0, 0x08);
    sender.tick(20000);
    receiver.tick(20000);
    CHECK(air.frames().size() == 2);
    CHECK(air.frames()[1].channel == 11);
    CHECK(air.frames()[0].payload ==
          std::vector<std::uint8_t>(mac_frame.begin(), mac_frame.end()));
    CHECK((sender.read8(tc32emu::Tlsr8258::RfIrqStatus) & 0x02u) != 0);
    CHECK((receiver.read8(tc32emu::Tlsr8258::RfIrqStatus) & 0x01u) != 0);
    CHECK((sender.read8(tc32emu::Tlsr8258::DmaTxReady0) & 0x08u) == 0);
    CHECK(sender.read8(0x800649) & 0x20u);
    CHECK(receiver.read8(0x800649) & 0x20u);
    CHECK(receiver.read8(rx_buffer) == mac_frame.size() + 9);
    CHECK(receiver.read8(rx_buffer + 4) == mac_frame.size());
    for (std::size_t i = 0; i != mac_frame.size(); ++i)
        CHECK(receiver.read8(rx_buffer + 5 + static_cast<std::uint32_t>(i)) ==
              mac_frame[i]);
    sender.write8(tc32emu::Tlsr8258::RfIrqStatus, 0x02);
    receiver.write8(tc32emu::Tlsr8258::RfIrqStatus, 0x01);
    CHECK((sender.read8(0x800649) & 0x20u) == 0);
    CHECK((receiver.read8(0x800649) & 0x20u) == 0);
}

} // namespace tc32emu_tests
