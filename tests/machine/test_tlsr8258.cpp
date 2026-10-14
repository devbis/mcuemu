#include "../test_support.hpp"
#include <array>
#include <cstdint>
#include <vector>
#include "mcuemu/cpu.hpp"
#include "mcuemu/tlsr8258.hpp"

namespace mcuemu_tests {

void test_tlsr_memory_gpio_and_timer() {
    mcuemu::Tlsr8258 device(mcuemu::Tlsr8258Model::F512);
    device.set_output_trace_enabled(true);
    CHECK(device.flash_size() == 512u * 1024u);
    device.write8(mcuemu::Tlsr8258::RegisterBase + 0x10, 0x5a);
    CHECK(device.read8(mcuemu::Tlsr8258::RegisterBase + 0x10) == 0x5a);
    device.write8(mcuemu::Tlsr8258::VendorRegisterAliasBase + 0x12d2, 0x6b);
    CHECK(device.read8(mcuemu::Tlsr8258::RegisterBase + 0x12d2) == 0x6b);
    device.write32(mcuemu::Tlsr8258::RetainedSramBase + 0x24, 0xaabbccddu);
    CHECK(device.read32(mcuemu::Tlsr8258::RetainedSramBase + 0x24) == 0xaabbccddu);
    CHECK(device.read8(mcuemu::Tlsr8258::GpioBase) == 0xff);

    device.set_gpio_input(0, 0x05);
    CHECK(device.read8(mcuemu::Tlsr8258::GpioBase) == 0x05);
    device.write8(mcuemu::Tlsr8258::GpioBase + 3, 0x81);
    CHECK(device.gpio_output(0) == 0x81);
    CHECK(device.output_events().size() == 1);
    CHECK(device.output_events()[0].kind ==
          mcuemu::OutputEvent::Kind::GpioPort);
    CHECK(device.output_events()[0].index == 0);
    CHECK(device.output_events()[0].value == 0x81);

    device.write8(mcuemu::Tlsr8258::PwmEnable, 0x06);
    device.write8(mcuemu::Tlsr8258::PwmEnable, 0x00);
    CHECK(device.output_events().size() == 3);
    CHECK(device.output_events()[1].kind ==
          mcuemu::OutputEvent::Kind::PwmEnableMask);
    CHECK(device.output_events()[1].value == 0x06);
    CHECK(device.output_events()[2].value == 0x00);

    device.write32(mcuemu::Tlsr8258::TimerCapture0, 3);
    device.write32(mcuemu::Tlsr8258::TimerControl, 1);
    device.tick(3);
    CHECK(device.read32(mcuemu::Tlsr8258::TimerTick0) == 3);
    CHECK((device.read8(mcuemu::Tlsr8258::TimerControl + 3) & 1u) != 0);
    CHECK(device.time_ns() == 62);
    device.write8(mcuemu::Tlsr8258::TimerControl + 3, 1);
    CHECK((device.read8(mcuemu::Tlsr8258::TimerControl + 3) & 1u) == 0);

    bool write_fault = false;
    try {
        device.write8(0, 0);
    } catch (const mcuemu::BusFault &) {
        write_fault = true;
    }
    CHECK(write_fault);

    mcuemu::Tlsr8258 one_megabyte(mcuemu::Tlsr8258Model::F1K);
    CHECK(one_megabyte.flash_size() == 1024u * 1024u);
    one_megabyte.load_firmware({0x3c}, one_megabyte.flash_size() - 1u);
    CHECK(one_megabyte.read8(one_megabyte.flash_size() - 1u) == 0x3c);
}

void test_tlsr_analog_registers_and_32k_tick() {
    mcuemu::Tlsr8258 device;
    device.write8(mcuemu::Tlsr8258::AnalogAddress, 0x7f);
    device.write8(mcuemu::Tlsr8258::AnalogControl, 0x40);
    CHECK(device.read8(mcuemu::Tlsr8258::AnalogData) == 1);
    device.write8(mcuemu::Tlsr8258::AnalogAddress,
                  mcuemu::Tlsr8258::AnalogClockPower);
    device.write8(mcuemu::Tlsr8258::AnalogControl, 0x40);
    CHECK(device.read8(mcuemu::Tlsr8258::AnalogData) ==
          mcuemu::Tlsr8258::AnalogClock32kXtalPowerDown);

    device.write8(mcuemu::Tlsr8258::AnalogAddress, 0x22);
    device.write8(mcuemu::Tlsr8258::AnalogData, 0xa5);
    device.write8(mcuemu::Tlsr8258::AnalogControl, 0x60); // start AREG write
    CHECK((device.read8(mcuemu::Tlsr8258::AnalogControl) & 1u) == 0);

    device.write8(mcuemu::Tlsr8258::AnalogAddress, 0x22);
    device.write8(mcuemu::Tlsr8258::AnalogControl, 0x40); // start AREG read
    CHECK(device.read8(mcuemu::Tlsr8258::AnalogData) == 0xa5);

    device.write8(mcuemu::Tlsr8258::AnalogAddress, 0xc7);
    device.write8(mcuemu::Tlsr8258::AnalogData, 0x0e);
    device.write8(mcuemu::Tlsr8258::AnalogControl, 0x60);
    device.write8(mcuemu::Tlsr8258::AnalogAddress, 0xcf);
    device.write8(mcuemu::Tlsr8258::AnalogControl, 0x40);
    CHECK((device.read8(mcuemu::Tlsr8258::AnalogData) & 0x80u) == 0);
    device.write8(mcuemu::Tlsr8258::AnalogAddress, 0xc7);
    device.write8(mcuemu::Tlsr8258::AnalogData, 0x0f);
    device.write8(mcuemu::Tlsr8258::AnalogControl, 0x60);
    device.write8(mcuemu::Tlsr8258::AnalogAddress, 0xcf);
    device.write8(mcuemu::Tlsr8258::AnalogControl, 0x40);
    CHECK((device.read8(mcuemu::Tlsr8258::AnalogData) & 0x80u) != 0);

    auto read_tick = [&device] {
        std::uint32_t tick = 0;
        for (unsigned byte = 0; byte != 4; ++byte) {
            device.write8(mcuemu::Tlsr8258::AnalogAddress,
                          static_cast<std::uint8_t>(0x40 + byte));
            device.write8(mcuemu::Tlsr8258::AnalogControl, 0x40);
            tick |= static_cast<std::uint32_t>(
                        device.read8(mcuemu::Tlsr8258::AnalogData))
                    << (byte * 8u);
        }
        return tick;
    };
    auto write_analog = [&device](std::uint8_t address, std::uint8_t value) {
        device.write8(mcuemu::Tlsr8258::AnalogAddress, address);
        device.write8(mcuemu::Tlsr8258::AnalogData, value);
        device.write8(mcuemu::Tlsr8258::AnalogControl, 0x60);
    };

    device.tick(48000); // 1 ms at 48 MHz; 32 ticks from the RC source.
    CHECK(read_tick() == 32);
    device.tick(47952000);
    CHECK(read_tick() == 32000);

    write_analog(mcuemu::Tlsr8258::AnalogClockPower,
                 mcuemu::Tlsr8258::AnalogClock32kRcPowerDown |
                     mcuemu::Tlsr8258::AnalogClock32kXtalPowerDown);
    device.tick(48000000);
    CHECK(read_tick() == 32000);

    write_analog(mcuemu::Tlsr8258::AnalogClockPower,
                 mcuemu::Tlsr8258::AnalogClock32kXtalPowerDown);
    device.tick(48000000);
    CHECK(read_tick() == 64000);

    device.write8(mcuemu::Tlsr8258::Clock32kSelect,
                  mcuemu::Tlsr8258::Clock32kSelectXtal);
    device.tick(48000000);
    CHECK(read_tick() == 64000); // XTAL remains powered down.

    write_analog(mcuemu::Tlsr8258::AnalogClockPower, 0);
    device.tick(48000000);
    CHECK(read_tick() == 96768);

    write_analog(mcuemu::Tlsr8258::AnalogClockPower,
                 mcuemu::Tlsr8258::AnalogClock32kXtalPowerDown);
    device.tick(48000000);
    CHECK(read_tick() == 96768);
}

void test_tlsr_rc32k_calibration_ready_after_virtual_time() {
    mcuemu::Tlsr8258 device;
    auto write_analog = [&device](std::uint8_t address, std::uint8_t value) {
        device.write8(mcuemu::Tlsr8258::AnalogAddress, address);
        device.write8(mcuemu::Tlsr8258::AnalogData, value);
        device.write8(mcuemu::Tlsr8258::AnalogControl, 0x60);
    };
    auto read_analog = [&device](std::uint8_t address) {
        device.write8(mcuemu::Tlsr8258::AnalogAddress, address);
        device.write8(mcuemu::Tlsr8258::AnalogControl, 0x40);
        return device.read8(mcuemu::Tlsr8258::AnalogData);
    };

    write_analog(mcuemu::Tlsr8258::AnalogRc32kCalibrationControl, 0xf6);
    write_analog(mcuemu::Tlsr8258::AnalogRc32kCalibrationControl, 0xf7);
    CHECK((read_analog(mcuemu::Tlsr8258::AnalogRcCalibrationStatus) &
           mcuemu::Tlsr8258::AnalogRc32kCalibrationReady) == 0);

    device.tick(mcuemu::Tlsr8258::Rc32kCalibrationTimeNs * 48 / 1000 - 1);
    CHECK((read_analog(mcuemu::Tlsr8258::AnalogRcCalibrationStatus) &
           mcuemu::Tlsr8258::AnalogRc32kCalibrationReady) == 0);
    device.tick(1);
    CHECK((read_analog(mcuemu::Tlsr8258::AnalogRcCalibrationStatus) &
           mcuemu::Tlsr8258::AnalogRc32kCalibrationReady) != 0);
    CHECK(device.time_ns() == mcuemu::Tlsr8258::Rc32kCalibrationTimeNs);

    write_analog(mcuemu::Tlsr8258::AnalogRc32kCalibrationControl, 0xf6);
    CHECK((read_analog(mcuemu::Tlsr8258::AnalogRcCalibrationStatus) &
           mcuemu::Tlsr8258::AnalogRc32kCalibrationReady) == 0);
}

void test_tlsr_deep_sleep_sram_retention() {
    mcuemu::Tlsr8258 device;
    device.write32(mcuemu::Tlsr8258::RetainedSramBase + 0x100,
                   0x12345678);
    device.write32(mcuemu::Tlsr8258::NonRetainedSramBase, 0x9abcdef0);

    auto write_analog = [&device](std::uint8_t address, std::uint8_t value) {
        device.write8(mcuemu::Tlsr8258::AnalogAddress, address);
        device.write8(mcuemu::Tlsr8258::AnalogData, value);
        device.write8(mcuemu::Tlsr8258::AnalogControl, 0x60);
    };
    write_analog(mcuemu::Tlsr8258::AnalogWakeSource,
                 mcuemu::Tlsr8258::AnalogWakeTimerEnable);
    write_analog(0x7e, 0x80); // Deep-sleep mode.
    device.write32(mcuemu::Tlsr8258::System32kTickCompare, 1);
    device.write8(mcuemu::Tlsr8258::SystemTimerControl, 0x08);
    device.write8(mcuemu::Tlsr8258::PowerDownControl,
                  mcuemu::Tlsr8258::PowerDownSleepEnable |
                      mcuemu::Tlsr8258::PowerDownSleepTrigger);
    CHECK(device.power_state() == mcuemu::Tlsr8258PowerState::Awake);
    device.tick(mcuemu::Tlsr8258::SleepEntryDelayCycles - 1);
    CHECK(device.power_state() == mcuemu::Tlsr8258PowerState::Awake);
    device.tick(1);

    CHECK(device.power_state() == mcuemu::Tlsr8258PowerState::DeepSleep);
    CHECK(device.advance_sleep_to_wakeup());
    CHECK(device.consume_deep_sleep_wakeup());
    CHECK(device.read32(mcuemu::Tlsr8258::RetainedSramBase + 0x100) ==
          0x12345678);
    CHECK(device.read32(mcuemu::Tlsr8258::NonRetainedSramBase) == 0);
}

// The vendor deep-sleep sequence writes AREG 0x2c = 0xde for a timer wake; a
// Zephyr build that wrote 0xd7 (digital timer bit) never woke on the board.
void test_tlsr_deep_sleep_wake_gate() {
    auto sleep_with_gate = [](std::uint8_t mode, std::uint8_t gate) {
        mcuemu::Tlsr8258 device;
        auto write_analog = [&device](std::uint8_t address, std::uint8_t value) {
            device.write8(mcuemu::Tlsr8258::AnalogAddress, address);
            device.write8(mcuemu::Tlsr8258::AnalogData, value);
            device.write8(mcuemu::Tlsr8258::AnalogControl, 0x60);
        };
        write_analog(mcuemu::Tlsr8258::AnalogWakeSource,
                     mcuemu::Tlsr8258::AnalogWakeTimerEnable);
        write_analog(mcuemu::Tlsr8258::AnalogWakeGate, gate);
        write_analog(0x7e, mode);
        device.write32(mcuemu::Tlsr8258::System32kTickCompare, 1);
        device.write8(mcuemu::Tlsr8258::SystemTimerControl, 0x08);
        device.write8(mcuemu::Tlsr8258::PowerDownControl,
                      mcuemu::Tlsr8258::PowerDownSleepEnable |
                          mcuemu::Tlsr8258::PowerDownSleepTrigger);
        device.tick(mcuemu::Tlsr8258::SleepEntryDelayCycles);
        return device.advance_sleep_to_wakeup();
    };
    CHECK(sleep_with_gate(0x80, 0xde));  // deep sleep, analog timer bit
    CHECK(!sleep_with_gate(0x80, 0xd7)); // deep sleep, digital timer bit only
    CHECK(sleep_with_gate(0x00, 0x97));  // suspend, digital timer bit
}

// Zephyr's stimer driver (TB03F): STOP, wait for RUNNING to clear, select the
// 16 MHz clock, START. A late handler that wrote 0x74c = 0 and 0x74f = START
// left the counter at a constant value while the CPU ran, reading 0xf4.
void test_tlsr_system_timer_start_stop_and_clock_select() {
    mcuemu::Tlsr8258 device;
    device.tick(48);
    CHECK(device.read32(mcuemu::Tlsr8258::SystemTimer) == 16);
    CHECK((device.read8(mcuemu::Tlsr8258::SystemTimerControl) & 2) != 0); // running

    device.write8(mcuemu::Tlsr8258::SystemTimerControl, 2);               // STOP
    CHECK((device.read8(mcuemu::Tlsr8258::SystemTimerControl) & 2) == 0);
    device.tick(48 * 1000);
    CHECK(device.read32(mcuemu::Tlsr8258::SystemTimer) == 16);
    CHECK(device.cycles_until_event() > 1000000);

    device.write32(mcuemu::Tlsr8258::SystemTimer, 0);
    device.write8(mcuemu::Tlsr8258::SystemTimerMode, 0);
    device.write8(mcuemu::Tlsr8258::SystemTimerControl, 1);               // START, no clock
    device.tick(48 * 1000);
    CHECK(device.read32(mcuemu::Tlsr8258::SystemTimer) == 0);
    CHECK((device.read8(mcuemu::Tlsr8258::SystemTimerControl) & 2) == 0); // not running

    device.write8(mcuemu::Tlsr8258::SystemTimerMode, mcuemu::Tlsr8258::SystemTimerEnable);
    device.tick(48);
    CHECK(device.read32(mcuemu::Tlsr8258::SystemTimer) == 16);
    CHECK((device.read8(mcuemu::Tlsr8258::SystemTimerControl) & 2) != 0);
}

void test_tlsr_system_timer() {
    mcuemu::Tlsr8258 device;
    CHECK(device.read32(mcuemu::Tlsr8258::SystemTimer) == 0);

    device.tick(48); // 1 us at 48 MHz; 16 MHz timer, 3 low bits read as zero.
    CHECK(device.read32(mcuemu::Tlsr8258::SystemTimer) == 16);
    device.tick(48);
    CHECK(device.read32(mcuemu::Tlsr8258::SystemTimer) == 32);

    device.write32(mcuemu::Tlsr8258::SystemTimer, 0x123);
    CHECK(device.read32(mcuemu::Tlsr8258::SystemTimer) == 0x120);
    device.tick(48);
    CHECK(device.read32(mcuemu::Tlsr8258::SystemTimer) == 0x130);

    device.write8(mcuemu::Tlsr8258::SystemTimerMode, 0x92);
    device.write8(mcuemu::Tlsr8258::SystemTimerControl, 0x01);
    CHECK(device.read16(0x800750) == 0);
    device.write8(mcuemu::Tlsr8258::SystemTimerControl, 0x02);
    CHECK(device.read8(mcuemu::Tlsr8258::SystemTimerControl) == 0); // STOP: strobes clear, not running
    device.write8(mcuemu::Tlsr8258::SystemTimerControl, 0x01);       // START
    CHECK(device.read8(mcuemu::Tlsr8258::SystemTimerControl) == 0x02);

    device.write32(mcuemu::Tlsr8258::SystemTimerCompare, 0x150);
    device.write8(mcuemu::Tlsr8258::SystemTimerMode,
                  mcuemu::Tlsr8258::SystemTimerEnable |
                      mcuemu::Tlsr8258::SystemTimerIrqEnable);
    device.write8(mcuemu::Tlsr8258::IrqMask + 2, 1u << 4); // IRQ 20.
    device.write8(mcuemu::Tlsr8258::IrqGlobalEnable, 1);
    device.tick(48); // 0x140; compare has not fired yet.
    CHECK(!device.interrupt_pending());
    device.tick(48); // 0x150; IRQ 20 becomes pending.
    CHECK(device.interrupt_pending());
    CHECK((device.read8(mcuemu::Tlsr8258::IrqSource + 2) & (1u << 4)) != 0);
    device.write32(mcuemu::Tlsr8258::IrqSource, 1u << 20);
    CHECK(!device.interrupt_pending());
    device.tick(48); // A one-shot compare does not reassert after its target.
    CHECK(!device.interrupt_pending());

    mcuemu::Tlsr8258 calibration_device;
    CHECK(calibration_device.read8(mcuemu::Tlsr8258::SystemTimerMode) == 0x90);
    calibration_device.write8(mcuemu::Tlsr8258::SystemTimerControl, 0x01);
    CHECK(calibration_device.read16(0x800750) == 0);
    calibration_device.tick(23999); // Just short of 16 RC 32 kHz ticks.
    CHECK(calibration_device.read16(0x800750) == 0);
    calibration_device.tick(1);
    CHECK(calibration_device.read16(0x800750) == 8000);

    mcuemu::Tlsr8258 measured_rc_device;
    measured_rc_device.set_rc32k_characteristics(29182, 8778);
    auto read_measured_rc_tick = [&measured_rc_device] {
        std::uint32_t tick = 0;
        for (unsigned byte = 0; byte != 4; ++byte) {
            measured_rc_device.write8(
                mcuemu::Tlsr8258::AnalogAddress,
                static_cast<std::uint8_t>(0x40 + byte));
            measured_rc_device.write8(mcuemu::Tlsr8258::AnalogControl, 0x40);
            tick |= static_cast<std::uint32_t>(measured_rc_device.read8(
                        mcuemu::Tlsr8258::AnalogData))
                    << (byte * 8u);
        }
        return tick;
    };
    measured_rc_device.tick(48000000);
    CHECK(read_measured_rc_tick() == 29182);
    measured_rc_device.write8(mcuemu::Tlsr8258::SystemTimerControl, 0x01);
    const std::uint64_t cycles_for_16_rc_ticks =
        (static_cast<std::uint64_t>(measured_rc_device.clock_hz()) * 16u +
         29182u - 1u) /
        29182u;
    measured_rc_device.tick(static_cast<std::uint32_t>(cycles_for_16_rc_ticks));
    CHECK(measured_rc_device.read16(0x800750) == 8778);
}

void test_tlsr_mspi_jedec_id() {
    auto read_id = [](mcuemu::Tlsr8258 &device) {
        device.write8(mcuemu::Tlsr8258::MspiControl, 1); // deselect
        device.write8(mcuemu::Tlsr8258::MspiControl, 0); // select
        device.write8(mcuemu::Tlsr8258::MspiData, 0x9f);
        device.write8(mcuemu::Tlsr8258::MspiData, 0x00); // issue clocks
        device.write8(mcuemu::Tlsr8258::MspiControl, 0x0a); // auto read
        const std::array<std::uint8_t, 3> id = {
            device.read8(mcuemu::Tlsr8258::MspiData),
            device.read8(mcuemu::Tlsr8258::MspiData),
            device.read8(mcuemu::Tlsr8258::MspiData),
        };
        device.write8(mcuemu::Tlsr8258::MspiControl, 1); // deselect
        return id;
    };

    mcuemu::Tlsr8258 f512;
    CHECK(read_id(f512) == (std::array<std::uint8_t, 3>{0xef, 0x40, 0x13}));
    mcuemu::Tlsr8258 f1k(mcuemu::Tlsr8258Model::F1K);
    CHECK(read_id(f1k) == (std::array<std::uint8_t, 3>{0xef, 0x40, 0x14}));
}

void test_tlsr_mspi_flash_read_program_erase() {
    using mcuemu::Tlsr8258;
    Tlsr8258 device;
    auto cmd = [&](std::uint8_t command, std::uint32_t address, bool with_address,
                   const std::vector<std::uint8_t> &data = {}) {
        device.write8(Tlsr8258::MspiControl, 1);
        device.write8(Tlsr8258::MspiControl, 0);
        device.write8(Tlsr8258::MspiData, command);
        if (with_address) {
            device.write8(Tlsr8258::MspiData, static_cast<std::uint8_t>(address >> 16));
            device.write8(Tlsr8258::MspiData, static_cast<std::uint8_t>(address >> 8));
            device.write8(Tlsr8258::MspiData, static_cast<std::uint8_t>(address));
        }
        for (std::uint8_t byte : data)
            device.write8(Tlsr8258::MspiData, byte);
    };
    auto read = [&](std::uint32_t address, unsigned count) {
        cmd(0x03, address, true);
        device.write8(Tlsr8258::MspiData, 0x00); // clock the first byte
        device.write8(Tlsr8258::MspiControl, 0x0a);
        std::vector<std::uint8_t> out;
        for (unsigned i = 0; i < count; ++i)
            out.push_back(device.read8(Tlsr8258::MspiData));
        device.write8(Tlsr8258::MspiControl, 1);
        return out;
    };
    auto status = [&] {
        cmd(0x05, 0, false);
        device.write8(Tlsr8258::MspiData, 0x00);
        device.write8(Tlsr8258::MspiControl, 0x0a);
        const std::uint8_t value = device.read8(Tlsr8258::MspiData);
        device.write8(Tlsr8258::MspiControl, 1);
        return value;
    };

    device.load_firmware({0x11, 0x22, 0x33, 0x44}, 0x1000);
    CHECK((read(0x1000, 4) == std::vector<std::uint8_t>{0x11, 0x22, 0x33, 0x44}));

    // Without write enable a program is ignored.
    cmd(0x02, 0x1000, true, {0x00});
    device.write8(Tlsr8258::MspiControl, 1);
    CHECK(read(0x1000, 1)[0] == 0x11);

    cmd(0x06, 0, false);
    CHECK(status() == 0x02); // WEL set
    cmd(0x02, 0x1000, true, {0x0f, 0xf0});
    device.write8(Tlsr8258::MspiControl, 1);
    CHECK(status() == 0x00); // WEL cleared at the end of the operation
    CHECK((read(0x1000, 3) == std::vector<std::uint8_t>{0x01, 0x20, 0x33}));
    std::uint32_t begin = 0, end = 0;
    CHECK(device.take_flash_dirty(begin, end) && begin == 0x1000 && end == 0x1002);
    CHECK(!device.take_flash_dirty(begin, end));

    // A sector erase needs write enable and sets the whole 4 KiB to 0xff.
    cmd(0x20, 0x1000, true);
    device.write8(Tlsr8258::MspiControl, 1);
    CHECK(read(0x1000, 1)[0] == 0x01);
    cmd(0x06, 0, false);
    cmd(0x20, 0x1234, true);
    device.write8(Tlsr8258::MspiControl, 1);
    CHECK((read(0x1000, 2) == std::vector<std::uint8_t>{0xff, 0xff}));
    CHECK(device.take_flash_dirty(begin, end) && begin == 0x1000 && end == 0x2000);

    // The unique id is stable and is neither blank nor all zero.
    auto uid = [&] {
        cmd(0x4b, 0, false, {0, 0, 0, 0, 0});
        device.write8(Tlsr8258::MspiControl, 0x0a);
        std::vector<std::uint8_t> out;
        for (int i = 0; i < 16; ++i)
            out.push_back(device.read8(Tlsr8258::MspiData));
        device.write8(Tlsr8258::MspiControl, 1);
        return out;
    };
    const auto first = uid();
    CHECK(first == uid());
    CHECK(first != std::vector<std::uint8_t>(16, 0xff));
    CHECK(first != std::vector<std::uint8_t>(16, 0x00));
}

void test_tlsr_aes128_encrypt_decrypt() {
    mcuemu::Tlsr8258 device;
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
        device.write8(mcuemu::Tlsr8258::AesControl, mode);
        for (unsigned i = 0; i != key.size(); ++i)
            device.write8(mcuemu::Tlsr8258::AesKey + i, key[i]);
        for (unsigned word = 0; word != 4; ++word) {
            std::uint32_t value = 0;
            for (unsigned byte = 0; byte != 4; ++byte)
                value |= static_cast<std::uint32_t>(input[word * 4u + byte])
                         << (byte * 8u);
            device.write32(mcuemu::Tlsr8258::AesData, value);
        }
        CHECK((device.read8(mcuemu::Tlsr8258::AesControl) & 0x02u) == 0);
        CHECK((device.read8(mcuemu::Tlsr8258::AesControl) & 0x04u) == 0);
        device.tick(999);
        CHECK((device.read8(mcuemu::Tlsr8258::AesControl) & 0x04u) == 0);
        device.tick(1);
        CHECK((device.read8(mcuemu::Tlsr8258::AesControl) & 0x04u) != 0);
        std::array<std::uint8_t, 16> output{};
        for (unsigned word = 0; word != 4; ++word) {
            const std::uint32_t value =
                device.read32(mcuemu::Tlsr8258::AesData);
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
    mcuemu::Tlsr8258 device;
    device.write32(mcuemu::Tlsr8258::DividerDividend, 100);
    device.write32(mcuemu::Tlsr8258::DividerDivisor, 7);
    device.write8(mcuemu::Tlsr8258::DividerControl, 0); // unsigned divide
    CHECK(device.read8(mcuemu::Tlsr8258::DividerControl) == 0);
    CHECK(device.read32(mcuemu::Tlsr8258::DividerDividend) == 14);
    CHECK(device.read32(mcuemu::Tlsr8258::DividerDivisor) == 2);

    device.write32(mcuemu::Tlsr8258::DividerDividend,
                   static_cast<std::uint32_t>(-100));
    device.write32(mcuemu::Tlsr8258::DividerDivisor, 7);
    device.write8(mcuemu::Tlsr8258::DividerControl, 1); // signed divide
    CHECK(device.read32(mcuemu::Tlsr8258::DividerDividend) ==
          static_cast<std::uint32_t>(-14));
    CHECK(device.read32(mcuemu::Tlsr8258::DividerDivisor) ==
          static_cast<std::uint32_t>(-2));
}

void test_tlsr_uart_fifo_and_virtual_timing() {
    mcuemu::Tlsr8258 device;
    device.write8(mcuemu::Tlsr8258::UartControl1, 0); // 8N1 framing.
    CHECK(device.read8(mcuemu::Tlsr8258::UartBufferCount) == 0);
    CHECK((device.read8(mcuemu::Tlsr8258::UartStatus1) & 1u) != 0);
    device.write8(mcuemu::Tlsr8258::UartState, 0x80); // clear TX done.
    device.tick(10);
    CHECK((device.read8(mcuemu::Tlsr8258::UartStatus1) & 1u) == 0);

    // The transmit FIFO holds 8 bytes: the SDK's uart_ndma_send_byte() waits
    // while the TX count is above 7.
    for (std::uint8_t value : {0x11u, 0x22u, 0x33u, 0x44u, 0x55u, 0x66u, 0x77u, 0x88u})
        device.write8(mcuemu::Tlsr8258::UartData, value);
    CHECK(device.read8(mcuemu::Tlsr8258::UartBufferCount) == 0x80);
    device.write8(mcuemu::Tlsr8258::UartData, 0x99); // FIFO is full.
    device.tick(159);
    CHECK(device.take_uart_transmitted().empty());
    CHECK((device.read8(mcuemu::Tlsr8258::UartStatus1) & 1u) == 0);
    device.tick(1);
    CHECK(device.take_uart_transmitted() == std::vector<std::uint8_t>{0x11});
    CHECK(device.read8(mcuemu::Tlsr8258::UartBufferCount) == 0x70);
    device.tick(7 * 160);
    CHECK(device.take_uart_transmitted() ==
          (std::vector<std::uint8_t>{0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}));
    CHECK(device.read8(mcuemu::Tlsr8258::UartBufferCount) == 0);
    CHECK((device.read8(mcuemu::Tlsr8258::UartStatus1) & 1u) != 0);

    device.write8(mcuemu::Tlsr8258::UartControl0, 0x4f); // RX interrupt enable.
    device.write8(mcuemu::Tlsr8258::UartControl3, 0x01); // RX threshold = 1.
    CHECK(device.inject_uart_rx(0xa5));
    CHECK(device.read8(mcuemu::Tlsr8258::UartBufferCount) == 1);
    CHECK((device.read8(mcuemu::Tlsr8258::UartStatus1) & 0x08u) != 0);
    CHECK((device.read8(mcuemu::Tlsr8258::UartStatus0) & 0x08u) != 0);
    CHECK((device.read8(mcuemu::Tlsr8258::IrqSource) & 0x40u) != 0);
    CHECK(device.read8(mcuemu::Tlsr8258::UartData + 3) == 0xa5);
    CHECK(device.read8(mcuemu::Tlsr8258::UartBufferCount) == 0);
    CHECK((device.read8(mcuemu::Tlsr8258::UartStatus0) & 0x08u) == 0);
    CHECK((device.read8(mcuemu::Tlsr8258::IrqSource) & 0x40u) == 0);

    device.write8(mcuemu::Tlsr8258::UartControl1, 0x80); // internal loopback.
    device.write8(mcuemu::Tlsr8258::UartData + 2, 0x7e);
    device.tick(160);
    CHECK(device.read8(mcuemu::Tlsr8258::UartData) == 0x7e);
    CHECK(device.take_uart_transmitted() == std::vector<std::uint8_t>{0x7e});

    mcuemu::Tlsr8258 divided_clock;
    divided_clock.write16(mcuemu::Tlsr8258::UartClockDivider, 0x8003);
    divided_clock.write8(mcuemu::Tlsr8258::UartControl0, 0);
    divided_clock.write8(mcuemu::Tlsr8258::UartControl1, 0);
    divided_clock.write8(mcuemu::Tlsr8258::UartData, 0x5a);
    divided_clock.tick(39);
    CHECK(divided_clock.take_uart_transmitted().empty());
    divided_clock.tick(1);
    CHECK(divided_clock.take_uart_transmitted() == std::vector<std::uint8_t>{0x5a});
}

void test_tlsr_uart_irq_enters_and_returns_from_irq_mode() {
    mcuemu::Tlsr8258 device;
    device.load_firmware(words({
        0xcf00, // thread entry: stop after the pending interrupt returns
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
        0x8006, // vector at 0x10 -> handler at 0x20
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
        0x6500, // handler: push IRQ LR
        0x1c08, // ldrb r0, [r1, r0] to drain the RX FIFO and ack its level IRQ
        0x6900, // treti {pc}
    }));
    device.write8(mcuemu::Tlsr8258::UartControl0, 0x4f);
    device.write8(mcuemu::Tlsr8258::UartControl3, 0x01);
    device.write8(mcuemu::Tlsr8258::IrqMask, 0x40);
    device.write8(mcuemu::Tlsr8258::IrqGlobalEnable, 1);
    CHECK(device.inject_uart_rx(0xa6));
    CHECK(device.interrupt_pending());

    mcuemu::Cpu cpu(device);
    cpu.set_reg(1, mcuemu::Tlsr8258::UartData);
    cpu.set_reg(13, mcuemu::Tlsr8258::RetainedSramBase + 0x100);
    cpu.set_reg(14, 0x1234);
    cpu.set_cpsr(0x12);
    cpu.set_reg(13, mcuemu::Tlsr8258::RetainedSramBase + 0x200);
    cpu.set_cpsr(0x13);

    const auto result = cpu.run(16);
    CHECK(result.reason == mcuemu::StopReason::Halted);
    CHECK(cpu.reg(0) == 0xa6);
    CHECK((cpu.cpsr() & 0x1fu) == 0x13u);
    CHECK(cpu.reg(13) == mcuemu::Tlsr8258::RetainedSramBase + 0x100);
    CHECK(cpu.reg(14) == 0x1234);
    CHECK(!device.interrupt_pending());
}

// Flash timing: a fetch from flash that misses the 128-line, 16-byte-line
// instruction cache stalls for 120 cycles; SRAM code and warm lines do not.
void test_tlsr_flash_timing_charges_cache_misses() {
    std::vector<std::uint8_t> bytes;
    for (int i = 0; i != 300; ++i) {
        bytes.push_back(0x00);
        bytes.push_back(0x00);
    }
    bytes.push_back(0x00);
    bytes.push_back(0xcf); // stop marker
    const std::uint64_t instructions = 301;
    const std::uint64_t lines = (bytes.size() + 15) / 16;

    auto run = [&](bool timing) {
        mcuemu::Tlsr8258 device;
        device.load_firmware(bytes);
        mcuemu::Cpu cpu(device);
        cpu.set_flash_timing(timing);
        const auto result = cpu.run(100000);
        CHECK(result.reason == mcuemu::StopReason::Halted);
        return device.time_ns();
    };
    const std::uint64_t plain = run(false);
    const std::uint64_t timed = run(true);
    // 3 cycles per instruction at 48 MHz, plus a 120-cycle fill per new line.
    CHECK(plain == instructions * 3 * 1000000000ull / 48000000ull);
    CHECK(timed == (instructions * 3 + lines * 120) * 1000000000ull / 48000000ull);

    // The SDK startup writes the RAM code size (in 256-byte units) to 0x60c; code below it
    // is fetched from SRAM, so a firmware that sets it never stalls on those lines.
    mcuemu::Tlsr8258 device;
    device.load_firmware(bytes);
    device.write8(0x80060c, 3); // 768 bytes, covers the whole program
    CHECK(device.ram_code_bytes() == 768);
    mcuemu::Cpu cpu(device);
    cpu.set_flash_timing(true);
    const auto result = cpu.run(100000);
    CHECK(result.reason == mcuemu::StopReason::Halted);
    CHECK(device.time_ns() == plain);
}

void test_tlsr_batched_run_matches_single_steps() {
    std::vector<std::uint16_t> program(300, 0x0000);
    program.push_back(0xcf00); // stop marker
    auto make = [&](mcuemu::Tlsr8258 &device) {
        std::vector<std::uint8_t> bytes;
        for (std::uint16_t value : program) {
            bytes.push_back(static_cast<std::uint8_t>(value));
            bytes.push_back(static_cast<std::uint8_t>(value >> 8));
        }
        device.load_firmware(bytes);
    };

    mcuemu::Tlsr8258 stepped_device, batched_device, bounded_device;
    make(stepped_device);
    make(batched_device);
    make(bounded_device);
    mcuemu::Cpu stepped(stepped_device), batched(batched_device),
        bounded(bounded_device);

    std::size_t stepped_count = 0;
    mcuemu::RunResult result;
    do {
        result = stepped.run(1);
        stepped_count += result.instructions;
    } while (result.reason == mcuemu::StopReason::StepLimit);
    const auto batch = batched.run(100000);

    CHECK(result.reason == mcuemu::StopReason::Halted);
    CHECK(batch.reason == mcuemu::StopReason::Halted);
    CHECK(batch.instructions == stepped_count);
    CHECK(batched.pc() == stepped.pc());
    CHECK(batched_device.time_ns() == stepped_device.time_ns());

    // A batch also stops at its instruction limit and resumes where it left off.
    const auto first = bounded.run(100);
    CHECK(first.reason == mcuemu::StopReason::StepLimit);
    CHECK(first.instructions == 100);
    const auto rest = bounded.run(100000);
    CHECK(rest.reason == mcuemu::StopReason::Halted);
    CHECK(first.instructions + rest.instructions == stepped_count);
    CHECK(bounded_device.time_ns() == stepped_device.time_ns());

    // A time deadline ends the run once the device clock reaches it, at most
    // one instruction late.
    mcuemu::Tlsr8258 timed_device;
    make(timed_device);
    mcuemu::Cpu timed(timed_device);
    const std::uint64_t deadline = 2000;
    const auto timed_result = timed.run(100000, deadline);
    CHECK(timed_result.reason == mcuemu::StopReason::StepLimit);
    CHECK(timed_device.network_time_ns() >= deadline);
    CHECK(timed_device.network_time_ns() < deadline + 200);
    CHECK(timed_result.instructions < stepped_count);
}

} // namespace mcuemu_tests
