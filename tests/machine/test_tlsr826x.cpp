#include "../test_support.hpp"
#include <array>
#include <cstdint>
#include <vector>
#include "mcuemu/cpu.hpp"
#include "mcuemu/radio.hpp"
#include "mcuemu/tlsr826x.hpp"

namespace mcuemu_tests {
namespace {
using mcuemu::Tlsr826x;
constexpr std::uint32_t reg(std::uint32_t offset) { return Tlsr826x::RegisterBase + offset; }
} // namespace

void test_tlsr826x_memory_map() {
    Tlsr826x chip(mcuemu::Tlsr826xModel::T8269);
    CHECK(chip.flash_size() == 512u * 1024u);
    CHECK(chip.layout().sram_base == 0x808000u);
    CHECK(chip.layout().sram_size == 0x8000u);
    chip.write32(0x808900, 0xaabbccddu);
    CHECK(chip.read32(0x808900) == 0xaabbccddu);
    chip.write8(0x80ffff, 0x5a); // last SRAM byte of the 8269
    CHECK(chip.read8(0x80ffff) == 0x5a);
    chip.write8(reg(0x10), 0x3c);
    CHECK(chip.read8(reg(0x10)) == 0x3c);

    Tlsr826x small(mcuemu::Tlsr826xModel::T8267);
    CHECK(small.layout().sram_size == 0x4000u);
    small.write8(0x80bfff, 1);
    bool fault = false;
    try {
        small.write8(0x80c000, 1);
    } catch (const mcuemu::BusFault &) {
        fault = true;
    }
    CHECK(fault);
    fault = false;
    try {
        chip.write8(0, 0);
    } catch (const mcuemu::BusFault &) {
        fault = true;
    }
    CHECK(fault);
    CHECK(mcuemu::parse_tlsr826x_model("TLSR8267") == mcuemu::Tlsr826xModel::T8267);
    CHECK(mcuemu::parse_tlsr826x_model("tlsr8269") == mcuemu::Tlsr826xModel::T8269);
}

void test_tlsr826x_cpu_runs_and_sees_sram() {
    Tlsr826x chip;
    chip.load_firmware(words({
        0x4008, // strb r0, [r1, #0]
        0xcf00, // tserv #0
    }));
    mcuemu::Cpu cpu(chip);
    cpu.set_reg(0, 0x77);
    cpu.set_reg(1, 0x808123);
    const auto result = cpu.run(4);
    CHECK(result.reason == mcuemu::StopReason::Halted);
    CHECK(chip.read8(0x808123) == 0x77);

    // MMIO through the CPU, and no vendor alias on this chip.
    Tlsr826x second;
    second.load_firmware(words({0x4008, 0xcf00}));
    mcuemu::Cpu cpu2(second);
    cpu2.set_reg(0, 0x5a);
    cpu2.set_reg(1, reg(0x12d2));
    CHECK(cpu2.run(4).reason == mcuemu::StopReason::Halted);
    CHECK(second.read8(reg(0x12d2)) == 0x5a);
}

void test_tlsr826x_timer_and_system_timer_irq() {
    Tlsr826x chip(mcuemu::Tlsr826xModel::T8269, 32000000);
    chip.write32(reg(0x624), 100);
    chip.write32(reg(0x620), 1); // timer 0 enabled
    chip.write8(reg(0x640), 0x01);
    chip.write8(reg(0x643), 0x01);
    chip.tick(99);
    CHECK(!chip.interrupt_pending());
    chip.tick(1);
    CHECK(chip.read32(reg(0x630)) == 100);
    CHECK(chip.interrupt_pending());
    chip.write8(reg(0x623), 0x01);
    CHECK(!chip.interrupt_pending());

    chip.write32(reg(0x620), 0); // timer 0 off
    chip.write8(reg(0x648), 0x01);
    // System timer: 16 MHz counter on a 32 MHz core, compare IRQ is IRQ 20.
    chip.write32(reg(0x744), 0x40 + (chip.read32(reg(0x740)) & ~7u));
    chip.write8(reg(0x74c), 0x02); // FLD_SYSTEM_TICK_IRQ_EN
    chip.write8(reg(0x642), 0x10); // mask bit 20
    chip.tick(200);
    CHECK((chip.read8(reg(0x64a)) & 0x10u) != 0);
    CHECK(chip.interrupt_pending());
    chip.write8(reg(0x64a), 0x10);
    CHECK(!chip.interrupt_pending());
    // STOP freezes it, START resumes.
    chip.write8(reg(0x74f), 0x02);
    const std::uint32_t frozen = chip.read32(reg(0x740));
    chip.tick(100);
    CHECK(chip.read32(reg(0x740)) == frozen);
    chip.write8(reg(0x74f), 0x01);
    chip.tick(100);
    CHECK(chip.read32(reg(0x740)) != frozen);
}

void test_tlsr826x_gpio_and_interrupt() {
    Tlsr826x chip;
    chip.set_output_trace_enabled(true);
    chip.write8(reg(0x582), 0x00); // PA output enable (0 = driven)
    chip.write8(reg(0x583), 0x81);
    CHECK(chip.gpio_output(0) == 0x81);
    CHECK(chip.output_events().size() == 1);
    chip.set_gpio_input(1, 0x02);
    CHECK(chip.read8(reg(0x588)) == 0x02);

    // GPIO interrupt: pin source enable + core gate, IRQ 18.
    chip.write8(reg(0x5b5), 0x08);
    chip.write8(reg(0x58f), 0x01); // PB irq enable bit 0
    chip.write8(reg(0x58c), 0x00); // polarity: active high
    chip.set_gpio_input(1, 0x03);
    CHECK((chip.read8(reg(0x64a)) & 0x04u) != 0);
}

void test_tlsr826x_mspi_flash() {
    Tlsr826x chip;
    auto command = [&](std::initializer_list<std::uint8_t> bytes) {
        chip.write8(reg(0x0d), 0x00);
        for (auto b : bytes)
            chip.write8(reg(0x0c), b);
    };
    auto release = [&] { chip.write8(reg(0x0d), 0x01); };
    command({0x9f, 0x00});
    CHECK(chip.read8(reg(0x0c)) == 0xef);
    CHECK(chip.read8(reg(0x0c)) == 0x40);
    CHECK(chip.read8(reg(0x0c)) == 0x13);
    release();

    command({0x06});
    release();
    command({0x02, 0x00, 0x10, 0x00, 0xa5, 0x5a});
    release();
    CHECK(chip.flash()[0x1000] == 0xa5);
    CHECK(chip.flash()[0x1001] == 0x5a);
    std::uint32_t begin = 0, end = 0;
    CHECK(chip.take_flash_dirty(begin, end));
    CHECK(begin == 0x1000 && end == 0x1002);

    command({0x06});
    release();
    command({0x20, 0x00, 0x10, 0x00});
    release();
    CHECK(chip.flash()[0x1000] == 0xff);
}

void test_tlsr826x_uart_dma() {
    Tlsr826x chip(mcuemu::Tlsr826xModel::T8269, 32000000);
    constexpr std::uint32_t tx = 0x808200;
    constexpr std::uint32_t rx = 0x808300;
    // DISASM uart_Send: 16-bit buffer address in 0x504, then 0x524 bit 1.
    chip.write32(tx, 3);
    chip.write8(tx + 4, 'a');
    chip.write8(tx + 5, 'b');
    chip.write8(tx + 6, 'c');
    chip.write16(reg(0x94), 0x8000 | 277); // divider enabled
    chip.write16(reg(0x96), 0x0009);       // bit width 10
    CHECK((chip.read8(reg(0x9e)) & 1u) != 0);
    chip.write16(reg(0x504), static_cast<std::uint16_t>(tx));
    chip.write8(reg(0x524), 0x02);
    CHECK((chip.read8(reg(0x9e)) & 1u) == 0); // busy
    chip.tick(32000000 / 10); // plenty of time
    CHECK(chip.take_uart_transmitted() == std::vector<std::uint8_t>({'a', 'b', 'c'}));
    CHECK((chip.read8(reg(0x9e)) & 1u) != 0);
    CHECK((chip.read8(reg(0x526)) & 0x02u) != 0);

    // RX: DMA channel 0, buffer size in 16-byte units at 0x502.
    chip.write16(reg(0x500), static_cast<std::uint16_t>(rx));
    chip.write8(reg(0x502), 4);
    chip.write8(reg(0x520), 0x01);
    chip.write8(reg(0x96), 0x19); // RX DMA enable
    chip.write8(reg(0x521), 0x01);
    CHECK(chip.inject_uart_rx('x'));
    CHECK(chip.inject_uart_rx('y'));
    chip.tick(32000000 / 10); // idle timeout closes the packet
    CHECK(chip.read32(rx) == 2);
    CHECK(chip.read8(rx + 4) == 'x');
    CHECK(chip.read8(rx + 5) == 'y');
    CHECK((chip.read8(reg(0x526)) & 0x01u) != 0);
    CHECK((chip.read8(reg(0x648)) & 0x10u) != 0); // DMA IRQ
    chip.write8(reg(0x526), 0x01);
    CHECK((chip.read8(reg(0x648)) & 0x10u) == 0);
}

void test_tlsr826x_analog_and_rc_calibration() {
    Tlsr826x chip;
    auto analog_write = [&](std::uint8_t address, std::uint8_t value) {
        chip.write8(reg(0xb8), address);
        chip.write8(reg(0xb9), value);
        chip.write8(reg(0xba), 0x60);
    };
    auto analog_read = [&](std::uint8_t address) {
        chip.write8(reg(0xb8), address);
        chip.write8(reg(0xba), 0x40);
        return chip.read8(reg(0xb9));
    };
    analog_write(0x3a, 0x12);
    CHECK(analog_read(0x3a) == 0x12);
    // DISASM rc_32k_cal: 0xf6 then 0xf7 into AREG 0x2e, ready flag is AREG 0x44 bit 4.
    analog_write(0x2e, 0xf6);
    analog_write(0x2e, 0xf7);
    CHECK((analog_read(0x44) & 0x10u) != 0);
}

void test_tlsr826x_divider_aes_and_suspend() {
    Tlsr826x chip(mcuemu::Tlsr826xModel::T8269, 32000000);
    // Hardware divider (div_mod.S): dividend 0x664, divisor 0x668, control 0x660.
    chip.write32(reg(0x664), 100);
    chip.write32(reg(0x668), 7);
    chip.write8(reg(0x660), 0);
    CHECK(chip.read32(reg(0x664)) == 14);
    CHECK(chip.read32(reg(0x668)) == 2);
    CHECK(chip.read8(reg(0x660)) == 0);
    chip.write32(reg(0x664), static_cast<std::uint32_t>(-100));
    chip.write32(reg(0x668), 7);
    chip.write8(reg(0x660), 1); // signed
    CHECK(static_cast<std::int32_t>(chip.read32(reg(0x664))) == -14);

    // AES-128 (FIPS-197 appendix B vector) through the 0x540 block, as hwAes_encrypt_start does.
    const std::array<std::uint8_t, 16> key{0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                                           0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
    const std::array<std::uint8_t, 16> plain{0x32, 0x43, 0xf6, 0xa8, 0x88, 0x5a, 0x30, 0x8d,
                                             0x31, 0x31, 0x98, 0xa2, 0xe0, 0x37, 0x07, 0x34};
    const std::array<std::uint8_t, 16> cipher{0x39, 0x25, 0x84, 0x1d, 0x02, 0xdc, 0x09, 0xfb,
                                              0xdc, 0x11, 0x85, 0x97, 0x19, 0x6a, 0x0b, 0x32};
    chip.write8(reg(0x540), 0);
    for (unsigned i = 0; i != 16; ++i)
        chip.write8(reg(0x550 + i), key[i]);
    for (unsigned i = 0; i != 16; ++i)
        chip.write8(reg(0x548 + (i % 4)), plain[i]);
    CHECK((chip.read8(reg(0x540)) & 0x04u) == 0);
    chip.tick(1000);
    CHECK((chip.read8(reg(0x540)) & 0x04u) != 0);
    for (unsigned i = 0; i != 16; ++i)
        CHECK(chip.read8(reg(0x548 + (i % 4))) == cipher[i]);
    CHECK((chip.read8(reg(0x540)) & 0x04u) == 0);

    // Suspend: 0x6f = 0x81 sleeps, the 32 kHz compare armed through 0x74f bit 3 wakes it.
    chip.write8(reg(0x74c), 0x80); // 32 kHz calibration enable off, timer irq off
    chip.write32(reg(0x754), 3200); // 100 ms of 32 kHz ticks
    chip.write8(reg(0x74f), 0x08);
    chip.write8(reg(0x6f), 0x81);
    CHECK(chip.power_state() == mcuemu::Tlsr8258PowerState::Awake);
    chip.tick(200);
    CHECK(chip.power_state() == mcuemu::Tlsr8258PowerState::Suspend);
    const std::uint64_t before = chip.time_ns();
    CHECK(chip.advance_sleep_to_wakeup());
    CHECK(chip.power_state() == mcuemu::Tlsr8258PowerState::Awake);
    CHECK(chip.time_ns() - before >= 99000000ull && chip.time_ns() - before <= 101000000ull);
    // Timer wake status in AREG 0x44 bit 1, cleared by writing one.
    chip.write8(reg(0xb8), 0x44);
    chip.write8(reg(0xba), 0x40);
    CHECK((chip.read8(reg(0xb9)) & 0x02u) != 0);
    chip.write8(reg(0xb9), 0x0f);
    chip.write8(reg(0xba), 0x60);
    chip.write8(reg(0xb8), 0x44);
    chip.write8(reg(0xba), 0x40);
    CHECK((chip.read8(reg(0xb9)) & 0x02u) == 0);
    // Nothing armed: a sleep with no wake source never ends.
    chip.write8(reg(0x6f), 0x81);
    chip.tick(200);
    CHECK(!chip.advance_sleep_to_wakeup());
}

void test_tlsr826x_radio_dma_packet_exchange() {
    mcuemu::VirtualAir air;
    Tlsr826x sender;
    Tlsr826x receiver;
    sender.connect_radio(air, "sender");
    receiver.connect_radio(air, "receiver");
    sender.set_radio_time_offset(5000000000u);
    receiver.set_radio_time_offset(5000000000u);

    // DISASM RF_TrxStateSet: 2400 + 5 * (channel - 10) MHz into 0x4d6, 16 bits.
    sender.write16(reg(0x4d6), 2400 + 5 * (20 - 10));
    CHECK(sender.radio_channel() == 20);
    receiver.write16(reg(0x4d6), 2400 + 5 * (20 - 10));

    constexpr std::uint32_t tx_buffer = 0x808100;
    constexpr std::uint32_t rx_buffer = 0x808200;
    const std::array<std::uint8_t, 6> mac_frame{0x61, 0x88, 0x01, 0x00, 0x00, 0x00};
    sender.write8(tx_buffer, static_cast<std::uint8_t>(mac_frame.size() + 1));
    sender.write8(tx_buffer + 4, static_cast<std::uint8_t>(mac_frame.size() + 2));
    for (std::size_t i = 0; i != mac_frame.size(); ++i)
        sender.write8(tx_buffer + 5 + static_cast<std::uint32_t>(i), mac_frame[i]);

    // RF_RxBufferSet: address 0x508, size in 16-byte units 0x50a, mode 0x50b.
    receiver.write16(reg(0x508), static_cast<std::uint16_t>(rx_buffer));
    receiver.write8(reg(0x50a), 8);
    receiver.write8(reg(0x50b), 1);
    receiver.write8(reg(0x520), 0x04); // RFDMA_RX_ENABLE
    receiver.write8(reg(0x428), 0x01); // RF RX on
    receiver.write8(reg(0xf02), 0x20);
    receiver.write16(reg(0xf1c), 0x0003);
    sender.write16(reg(0xf1c), 0x0003);
    sender.write8(reg(0xf02), 0x10); // TX mode
    // RF_TxPkt: address to 0x50c, then bit 3 of 0x524.
    sender.write16(reg(0x50c), static_cast<std::uint16_t>(tx_buffer));
    sender.write8(reg(0x524), 0x08);

    sender.tick(30000);
    for (unsigned poll = 0; poll != 100; ++poll)
        receiver.tick(320);
    CHECK(air.frames().size() == 1);
    CHECK(air.frames()[0].channel == 20);
    CHECK(air.frames()[0].payload == std::vector<std::uint8_t>(mac_frame.begin(), mac_frame.end()));
    CHECK((sender.read8(reg(0xf20)) & 0x02u) != 0);
    CHECK((sender.read8(reg(0x524)) & 0x08u) == 0);
    CHECK((receiver.read8(reg(0xf20)) & 0x01u) != 0);
    CHECK((receiver.read8(reg(0x526)) & 0x04u) != 0);
    CHECK((sender.read8(reg(0x649)) & 0x20u) != 0);
    CHECK((receiver.read8(reg(0x649)) & 0x20u) != 0);
    // SDK layout: p[0] = p[12] + 13, p[12] = payload + 2, payload at p[13], RSSI at p[4].
    CHECK(receiver.read8(rx_buffer + 12) == mac_frame.size() + 2);
    CHECK(receiver.read8(rx_buffer) == receiver.read8(rx_buffer + 12) + 13);
    CHECK(receiver.read8(rx_buffer + 4) != 0);
    for (std::size_t i = 0; i != mac_frame.size(); ++i)
        CHECK(receiver.read8(rx_buffer + 13 + static_cast<std::uint32_t>(i)) == mac_frame[i]);
    // SDK RF_PACKET_CRC_OK: (p[p[0] + 3] & 0x51) == 0x10
    CHECK((receiver.read8(rx_buffer + receiver.read8(rx_buffer) + 3) & 0x51u) == 0x10u);
    sender.write8(reg(0xf20), 0x02);
    receiver.write8(reg(0xf20), 0x01);
    CHECK((sender.read8(reg(0x649)) & 0x20u) == 0);
    CHECK((receiver.read8(reg(0x649)) & 0x20u) == 0);
}

} // namespace mcuemu_tests
