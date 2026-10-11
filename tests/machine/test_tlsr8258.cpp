#include "../test_support.hpp"
#include <array>
#include <cstdint>
#include <vector>
#include "tc32emu/cpu.hpp"
#include "tc32emu/tlsr8258.hpp"

namespace tc32emu_tests {

void test_tlsr_memory_gpio_and_timer() {
    tc32emu::Tlsr8258 device(tc32emu::Tlsr8258Model::F512);
    device.set_output_trace_enabled(true);
    CHECK(device.flash_size() == 512u * 1024u);
    device.write8(tc32emu::Tlsr8258::RegisterBase + 0x10, 0x5a);
    CHECK(device.read8(tc32emu::Tlsr8258::RegisterBase + 0x10) == 0x5a);
    device.write8(tc32emu::Tlsr8258::VendorRegisterAliasBase + 0x12d2, 0x6b);
    CHECK(device.read8(tc32emu::Tlsr8258::RegisterBase + 0x12d2) == 0x6b);
    device.write32(tc32emu::Tlsr8258::RetainedSramBase + 0x24, 0xaabbccddu);
    CHECK(device.read32(tc32emu::Tlsr8258::RetainedSramBase + 0x24) == 0xaabbccddu);
    CHECK(device.read8(tc32emu::Tlsr8258::GpioBase) == 0xff);

    device.set_gpio_input(0, 0x05);
    CHECK(device.read8(tc32emu::Tlsr8258::GpioBase) == 0x05);
    device.write8(tc32emu::Tlsr8258::GpioBase + 3, 0x81);
    CHECK(device.gpio_output(0) == 0x81);
    CHECK(device.output_events().size() == 1);
    CHECK(device.output_events()[0].kind ==
          tc32emu::OutputEvent::Kind::GpioPort);
    CHECK(device.output_events()[0].index == 0);
    CHECK(device.output_events()[0].value == 0x81);

    device.write8(tc32emu::Tlsr8258::PwmEnable, 0x06);
    device.write8(tc32emu::Tlsr8258::PwmEnable, 0x00);
    CHECK(device.output_events().size() == 3);
    CHECK(device.output_events()[1].kind ==
          tc32emu::OutputEvent::Kind::PwmEnableMask);
    CHECK(device.output_events()[1].value == 0x06);
    CHECK(device.output_events()[2].value == 0x00);

    device.write32(tc32emu::Tlsr8258::TimerCapture0, 3);
    device.write32(tc32emu::Tlsr8258::TimerControl, 1);
    device.tick(3);
    CHECK(device.read32(tc32emu::Tlsr8258::TimerTick0) == 3);
    CHECK((device.read8(tc32emu::Tlsr8258::TimerControl + 3) & 1u) != 0);
    CHECK(device.time_ns() == 62);
    device.write8(tc32emu::Tlsr8258::TimerControl + 3, 1);
    CHECK((device.read8(tc32emu::Tlsr8258::TimerControl + 3) & 1u) == 0);

    bool write_fault = false;
    try {
        device.write8(0, 0);
    } catch (const tc32emu::BusFault &) {
        write_fault = true;
    }
    CHECK(write_fault);

    tc32emu::Tlsr8258 one_megabyte(tc32emu::Tlsr8258Model::F1K);
    CHECK(one_megabyte.flash_size() == 1024u * 1024u);
    one_megabyte.load_firmware({0x3c}, one_megabyte.flash_size() - 1u);
    CHECK(one_megabyte.read8(one_megabyte.flash_size() - 1u) == 0x3c);
}

void test_tlsr_analog_registers_and_32k_tick() {
    tc32emu::Tlsr8258 device;
    device.write8(tc32emu::Tlsr8258::AnalogAddress, 0x7f);
    device.write8(tc32emu::Tlsr8258::AnalogControl, 0x40);
    CHECK(device.read8(tc32emu::Tlsr8258::AnalogData) == 1);
    device.write8(tc32emu::Tlsr8258::AnalogAddress,
                  tc32emu::Tlsr8258::AnalogClockPower);
    device.write8(tc32emu::Tlsr8258::AnalogControl, 0x40);
    CHECK(device.read8(tc32emu::Tlsr8258::AnalogData) ==
          tc32emu::Tlsr8258::AnalogClock32kXtalPowerDown);

    device.write8(tc32emu::Tlsr8258::AnalogAddress, 0x22);
    device.write8(tc32emu::Tlsr8258::AnalogData, 0xa5);
    device.write8(tc32emu::Tlsr8258::AnalogControl, 0x60); // start AREG write
    CHECK((device.read8(tc32emu::Tlsr8258::AnalogControl) & 1u) == 0);

    device.write8(tc32emu::Tlsr8258::AnalogAddress, 0x22);
    device.write8(tc32emu::Tlsr8258::AnalogControl, 0x40); // start AREG read
    CHECK(device.read8(tc32emu::Tlsr8258::AnalogData) == 0xa5);

    device.write8(tc32emu::Tlsr8258::AnalogAddress, 0xc7);
    device.write8(tc32emu::Tlsr8258::AnalogData, 0x0e);
    device.write8(tc32emu::Tlsr8258::AnalogControl, 0x60);
    device.write8(tc32emu::Tlsr8258::AnalogAddress, 0xcf);
    device.write8(tc32emu::Tlsr8258::AnalogControl, 0x40);
    CHECK((device.read8(tc32emu::Tlsr8258::AnalogData) & 0x80u) == 0);
    device.write8(tc32emu::Tlsr8258::AnalogAddress, 0xc7);
    device.write8(tc32emu::Tlsr8258::AnalogData, 0x0f);
    device.write8(tc32emu::Tlsr8258::AnalogControl, 0x60);
    device.write8(tc32emu::Tlsr8258::AnalogAddress, 0xcf);
    device.write8(tc32emu::Tlsr8258::AnalogControl, 0x40);
    CHECK((device.read8(tc32emu::Tlsr8258::AnalogData) & 0x80u) != 0);

    auto read_tick = [&device] {
        std::uint32_t tick = 0;
        for (unsigned byte = 0; byte != 4; ++byte) {
            device.write8(tc32emu::Tlsr8258::AnalogAddress,
                          static_cast<std::uint8_t>(0x40 + byte));
            device.write8(tc32emu::Tlsr8258::AnalogControl, 0x40);
            tick |= static_cast<std::uint32_t>(
                        device.read8(tc32emu::Tlsr8258::AnalogData))
                    << (byte * 8u);
        }
        return tick;
    };
    auto write_analog = [&device](std::uint8_t address, std::uint8_t value) {
        device.write8(tc32emu::Tlsr8258::AnalogAddress, address);
        device.write8(tc32emu::Tlsr8258::AnalogData, value);
        device.write8(tc32emu::Tlsr8258::AnalogControl, 0x60);
    };

    device.tick(48000); // 1 ms at 48 MHz; 32 ticks from the RC source.
    CHECK(read_tick() == 32);
    device.tick(47952000);
    CHECK(read_tick() == 32000);

    write_analog(tc32emu::Tlsr8258::AnalogClockPower,
                 tc32emu::Tlsr8258::AnalogClock32kRcPowerDown |
                     tc32emu::Tlsr8258::AnalogClock32kXtalPowerDown);
    device.tick(48000000);
    CHECK(read_tick() == 32000);

    write_analog(tc32emu::Tlsr8258::AnalogClockPower,
                 tc32emu::Tlsr8258::AnalogClock32kXtalPowerDown);
    device.tick(48000000);
    CHECK(read_tick() == 64000);

    device.write8(tc32emu::Tlsr8258::Clock32kSelect,
                  tc32emu::Tlsr8258::Clock32kSelectXtal);
    device.tick(48000000);
    CHECK(read_tick() == 64000); // XTAL remains powered down.

    write_analog(tc32emu::Tlsr8258::AnalogClockPower, 0);
    device.tick(48000000);
    CHECK(read_tick() == 96768);

    write_analog(tc32emu::Tlsr8258::AnalogClockPower,
                 tc32emu::Tlsr8258::AnalogClock32kXtalPowerDown);
    device.tick(48000000);
    CHECK(read_tick() == 96768);
}

void test_tlsr_system_timer() {
    tc32emu::Tlsr8258 device;
    CHECK(device.read32(tc32emu::Tlsr8258::SystemTimer) == 0);

    device.tick(48); // 1 us at 48 MHz; 16 MHz timer, 3 low bits read as zero.
    CHECK(device.read32(tc32emu::Tlsr8258::SystemTimer) == 16);
    device.tick(48);
    CHECK(device.read32(tc32emu::Tlsr8258::SystemTimer) == 32);

    device.write32(tc32emu::Tlsr8258::SystemTimer, 0x123);
    CHECK(device.read32(tc32emu::Tlsr8258::SystemTimer) == 0x120);
    device.tick(48);
    CHECK(device.read32(tc32emu::Tlsr8258::SystemTimer) == 0x130);
}

void test_tlsr_mspi_jedec_id() {
    auto read_id = [](tc32emu::Tlsr8258 &device) {
        device.write8(tc32emu::Tlsr8258::MspiControl, 1); // deselect
        device.write8(tc32emu::Tlsr8258::MspiControl, 0); // select
        device.write8(tc32emu::Tlsr8258::MspiData, 0x9f);
        device.write8(tc32emu::Tlsr8258::MspiData, 0x00); // issue clocks
        device.write8(tc32emu::Tlsr8258::MspiControl, 0x0a); // auto read
        const std::array<std::uint8_t, 3> id = {
            device.read8(tc32emu::Tlsr8258::MspiData),
            device.read8(tc32emu::Tlsr8258::MspiData),
            device.read8(tc32emu::Tlsr8258::MspiData),
        };
        device.write8(tc32emu::Tlsr8258::MspiControl, 1); // deselect
        return id;
    };

    tc32emu::Tlsr8258 f512;
    CHECK(read_id(f512) == (std::array<std::uint8_t, 3>{0xef, 0x40, 0x13}));
    tc32emu::Tlsr8258 f1k(tc32emu::Tlsr8258Model::F1K);
    CHECK(read_id(f1k) == (std::array<std::uint8_t, 3>{0xef, 0x40, 0x14}));
}

void test_tlsr_aes128_encrypt_decrypt() {
    tc32emu::Tlsr8258 device;
    const std::array<std::uint8_t, 16> key = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    };
    const std::array<std::uint8_t, 16> plaintext = {
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    };
    const std::array<std::uint8_t, 16> ciphertext = {
        0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
        0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
    };
    auto run_block = [&device, &key](
                         std::uint8_t mode,
                         const std::array<std::uint8_t, 16> &input) {
        device.write8(tc32emu::Tlsr8258::AesControl, mode);
        for (unsigned i = 0; i != key.size(); ++i)
            device.write8(tc32emu::Tlsr8258::AesKey + i, key[i]);
        for (unsigned word = 0; word != 4; ++word) {
            std::uint32_t value = 0;
            for (unsigned byte = 0; byte != 4; ++byte)
                value |= static_cast<std::uint32_t>(input[word * 4u + byte])
                         << (byte * 8u);
            device.write32(tc32emu::Tlsr8258::AesData, value);
        }
        CHECK((device.read8(tc32emu::Tlsr8258::AesControl) & 0x02u) == 0);
        CHECK((device.read8(tc32emu::Tlsr8258::AesControl) & 0x04u) == 0);
        device.tick(999);
        CHECK((device.read8(tc32emu::Tlsr8258::AesControl) & 0x04u) == 0);
        device.tick(1);
        CHECK((device.read8(tc32emu::Tlsr8258::AesControl) & 0x04u) != 0);
        std::array<std::uint8_t, 16> output{};
        for (unsigned word = 0; word != 4; ++word) {
            const std::uint32_t value =
                device.read32(tc32emu::Tlsr8258::AesData);
            for (unsigned byte = 0; byte != 4; ++byte)
                output[word * 4u + byte] =
                    static_cast<std::uint8_t>(value >> (byte * 8u));
        }
        return output;
    };
    CHECK(run_block(0, plaintext) == ciphertext);
    CHECK(run_block(1, ciphertext) == plaintext);
}

void test_tlsr_divider_registers() {
    tc32emu::Tlsr8258 device;
    device.write32(tc32emu::Tlsr8258::DividerDividend, 100);
    device.write32(tc32emu::Tlsr8258::DividerDivisor, 7);
    device.write8(tc32emu::Tlsr8258::DividerControl, 0); // unsigned divide
    CHECK(device.read8(tc32emu::Tlsr8258::DividerControl) == 0);
    CHECK(device.read32(tc32emu::Tlsr8258::DividerDividend) == 14);
    CHECK(device.read32(tc32emu::Tlsr8258::DividerDivisor) == 2);

    device.write32(tc32emu::Tlsr8258::DividerDividend,
                   static_cast<std::uint32_t>(-100));
    device.write32(tc32emu::Tlsr8258::DividerDivisor, 7);
    device.write8(tc32emu::Tlsr8258::DividerControl, 1); // signed divide
    CHECK(device.read32(tc32emu::Tlsr8258::DividerDividend) ==
          static_cast<std::uint32_t>(-14));
    CHECK(device.read32(tc32emu::Tlsr8258::DividerDivisor) ==
          static_cast<std::uint32_t>(-2));
}

void test_tlsr_uart_fifo_and_virtual_timing() {
    tc32emu::Tlsr8258 device;
    device.write8(tc32emu::Tlsr8258::UartControl1, 0); // 8N1 framing.
    CHECK(device.read8(tc32emu::Tlsr8258::UartBufferCount) == 0);
    CHECK((device.read8(tc32emu::Tlsr8258::UartStatus1) & 1u) != 0);
    device.write8(tc32emu::Tlsr8258::UartState, 0x80); // clear TX done.
    device.tick(10);
    CHECK((device.read8(tc32emu::Tlsr8258::UartStatus1) & 1u) == 0);

    for (std::uint8_t value : {0x11u, 0x22u, 0x33u, 0x44u})
        device.write8(tc32emu::Tlsr8258::UartData, value);
    CHECK(device.read8(tc32emu::Tlsr8258::UartBufferCount) == 0x40);
    device.write8(tc32emu::Tlsr8258::UartData, 0x55); // FIFO is full.
    device.tick(159);
    CHECK(device.take_uart_transmitted().empty());
    CHECK((device.read8(tc32emu::Tlsr8258::UartStatus1) & 1u) == 0);
    device.tick(1);
    CHECK(device.take_uart_transmitted() == std::vector<std::uint8_t>{0x11});
    CHECK(device.read8(tc32emu::Tlsr8258::UartBufferCount) == 0x30);
    device.tick(480);
    CHECK(device.take_uart_transmitted() ==
          (std::vector<std::uint8_t>{0x22, 0x33, 0x44}));
    CHECK(device.read8(tc32emu::Tlsr8258::UartBufferCount) == 0);
    CHECK((device.read8(tc32emu::Tlsr8258::UartStatus1) & 1u) != 0);

    device.write8(tc32emu::Tlsr8258::UartControl0, 0x4f); // RX interrupt enable.
    device.write8(tc32emu::Tlsr8258::UartControl3, 0x01); // RX threshold = 1.
    CHECK(device.inject_uart_rx(0xa5));
    CHECK(device.read8(tc32emu::Tlsr8258::UartBufferCount) == 1);
    CHECK((device.read8(tc32emu::Tlsr8258::UartStatus1) & 0x08u) != 0);
    CHECK((device.read8(tc32emu::Tlsr8258::UartStatus0) & 0x08u) != 0);
    CHECK((device.read8(tc32emu::Tlsr8258::IrqSource) & 0x40u) != 0);
    CHECK(device.read8(tc32emu::Tlsr8258::UartData + 3) == 0xa5);
    CHECK(device.read8(tc32emu::Tlsr8258::UartBufferCount) == 0);
    CHECK((device.read8(tc32emu::Tlsr8258::UartStatus0) & 0x08u) == 0);
    CHECK((device.read8(tc32emu::Tlsr8258::IrqSource) & 0x40u) == 0);

    device.write8(tc32emu::Tlsr8258::UartControl1, 0x80); // internal loopback.
    device.write8(tc32emu::Tlsr8258::UartData + 2, 0x7e);
    device.tick(160);
    CHECK(device.read8(tc32emu::Tlsr8258::UartData) == 0x7e);
    CHECK(device.take_uart_transmitted() == std::vector<std::uint8_t>{0x7e});

    tc32emu::Tlsr8258 divided_clock;
    divided_clock.write16(tc32emu::Tlsr8258::UartClockDivider, 0x8003);
    divided_clock.write8(tc32emu::Tlsr8258::UartControl0, 0);
    divided_clock.write8(tc32emu::Tlsr8258::UartControl1, 0);
    divided_clock.write8(tc32emu::Tlsr8258::UartData, 0x5a);
    divided_clock.tick(39);
    CHECK(divided_clock.take_uart_transmitted().empty());
    divided_clock.tick(1);
    CHECK(divided_clock.take_uart_transmitted() == std::vector<std::uint8_t>{0x5a});
}

void test_tlsr_uart_irq_enters_and_returns_from_irq_mode() {
    tc32emu::Tlsr8258 device;
    device.load_firmware(words({
        0xcf00, // thread entry: stop after the pending interrupt returns
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
        0x8006, // vector at 0x10 -> handler at 0x20
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
        0x6500, // handler: push IRQ LR
        0x1c08, // ldrb r0, [r1, r0] to drain the RX FIFO and ack its level IRQ
        0x6900, // treti {pc}
    }));
    device.write8(tc32emu::Tlsr8258::UartControl0, 0x4f);
    device.write8(tc32emu::Tlsr8258::UartControl3, 0x01);
    device.write8(tc32emu::Tlsr8258::IrqMask, 0x40);
    device.write8(tc32emu::Tlsr8258::IrqGlobalEnable, 1);
    CHECK(device.inject_uart_rx(0xa6));
    CHECK(device.interrupt_pending());

    tc32emu::Cpu cpu(device);
    cpu.set_reg(1, tc32emu::Tlsr8258::UartData);
    cpu.set_reg(13, tc32emu::Tlsr8258::RetainedSramBase + 0x100);
    cpu.set_reg(14, 0x1234);
    cpu.set_cpsr(0x12);
    cpu.set_reg(13, tc32emu::Tlsr8258::RetainedSramBase + 0x200);
    cpu.set_cpsr(0x13);

    const auto result = cpu.run(16);
    CHECK(result.reason == tc32emu::StopReason::Halted);
    CHECK(cpu.reg(0) == 0xa6);
    CHECK((cpu.cpsr() & 0x1fu) == 0x13u);
    CHECK(cpu.reg(13) == tc32emu::Tlsr8258::RetainedSramBase + 0x100);
    CHECK(cpu.reg(14) == 0x1234);
    CHECK(!device.interrupt_pending());
}

} // namespace tc32emu_tests
