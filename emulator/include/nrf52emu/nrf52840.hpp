#pragma once

#include "cortexm/core.hpp"
#include "mcuemu/radio.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

namespace nrf52emu {

class Peripheral;
class Gpio;
class Gpiote;

// What a pad does to its net, as seen from outside the chip.
enum class PadDrive { HiZ, Low, High, PullUp, PullDown };

// Nordic nRF52840: Cortex-M4F at 64 MHz, 1 MiB flash, 256 KiB RAM.
//
// Peripheral instances sit at 0x40000000 + 0x1000 * id, and the id is also the
// interrupt number. Registers follow the nRF52840 product specification; where
// the model fills a gap with an inferred behaviour the peripheral says so.
class Nrf52840 final : public cortexm::Bus {
public:
    static constexpr std::uint32_t FlashSize = 0x100000;
    static constexpr std::uint32_t RamSize = 0x40000;
    static constexpr std::uint32_t RamBase = 0x20000000;
    static constexpr std::uint32_t FicrBase = 0x10000000;
    static constexpr std::uint32_t UicrBase = 0x10001000;
    static constexpr std::uint32_t PeripheralBase = 0x40000000;
    static constexpr std::uint32_t GpioBase = 0x50000000;
    static constexpr std::uint32_t ClockHz = 64000000;
    // P0.00..P0.31 are pads 0..31, P1.00..P1.15 are pads 32..47.
    static constexpr unsigned PadCount = 48;

    Nrf52840();
    ~Nrf52840() override;
    Nrf52840(const Nrf52840 &) = delete;
    Nrf52840 &operator=(const Nrf52840 &) = delete;

    // Flash image placed at `offset` (the model has no bootloader of its own).
    void load_firmware(const std::vector<std::uint8_t> &image, std::uint32_t offset = 0);
    // (Re)starts the CPU from the vector table; RAM and flash are kept.
    void reset();

    cortexm::Core &core() noexcept { return *core_; }
    const cortexm::Core &core() const noexcept { return *core_; }
    std::uint64_t time_ns() const { return core_->cycles_to_ns(core_->cycles()); }

    // Logs every peripheral access (address, value, pc) to `out`.
    void set_trace(std::ostream *out) noexcept { trace_ = out; }

    // Supply voltage seen by the ADC's VDD input.
    void set_supply_mv(std::uint32_t millivolts) noexcept { supply_mv_ = millivolts; }
    std::uint32_t supply_mv() const noexcept { return supply_mv_; }

    // ---- pads ------------------------------------------------------------
    using PadObserver = std::function<void(unsigned pad)>;
    // Called whenever what a pad drives changes.
    void set_pad_observer(PadObserver observer) { pad_observer_ = std::move(observer); }
    PadDrive pad_drive(unsigned pad) const;
    // The level an external circuit holds on the pad (true = high).
    void set_pad_input(unsigned pad, bool high);
    // Pads are named like the datasheet: "P0.13".
    static bool parse_pad(const std::string &name, unsigned &pad);

    // Voltage on an analog-capable pad, in millivolts, for the SAADC. The
    // default reads a pad driven high as the supply and anything else as 0.
    using AnalogProvider = std::function<std::uint32_t(unsigned pad)>;
    void set_analog_provider(AnalogProvider provider) { analog_provider_ = std::move(provider); }
    std::uint32_t analog_input_mv(unsigned pad) const;

    // ---- memory ----------------------------------------------------------
    std::uint8_t *flash() noexcept { return flash_; }
    std::uint8_t *ram() noexcept { return ram_; }
    std::uint8_t *ficr() noexcept { return ficr_; }
    std::uint8_t *uicr() noexcept { return uicr_; }

    // cortexm::Bus
    std::uint32_t read(std::uint32_t address, unsigned size) override;
    void write(std::uint32_t address, std::uint32_t value, unsigned size) override;

    // ---- used by the peripherals ----------------------------------------
    Peripheral *peripheral(unsigned id) noexcept { return id < peripherals_.size() ? peripherals_[id].get() : nullptr; }
    Gpio &gpio() noexcept { return *gpio_; }
    // PPI: an event register at `address` went to 1 / a task is triggered.
    void route_event(std::uint32_t address);
    void trigger_task(std::uint32_t address);
    // Asks for a service call no later than `at_cycle`.
    void request_service(std::uint64_t at_cycle) { core_->request_service(at_cycle); }
    // The pad drive may have changed: tell the observer.
    void notify_pad(unsigned pad);
    // Watchdog, system reset request or SYSTEMOFF wake: stops the CPU so the
    // chip restarts; `reason_bits` land in POWER.RESETREAS.
    void request_reset(std::uint32_t reason_bits);
    // POWER.SYSTEMOFF: nothing runs until a pin with SENSE matches or a reset.
    void request_system_off();
    void wake_from_system_off();
    bool system_off() const noexcept { return system_off_; }
    // Flash below this address holds the program image (translated code);
    // writes above it do not invalidate the translation cache.
    std::uint32_t code_end() const noexcept { return code_end_; }
    std::ostream *trace() noexcept { return trace_; }

    // Runs until `until_cycle`, restarting after resets. Stops early on a
    // breakpoint or an engine fault, which `result` reports.
    cortexm::RunResult run(std::uint64_t until_cycle);

    // UARTE0 and UARTE1: bytes to receive and bytes the chip transmitted.
    static constexpr unsigned UartCount = 2;
    bool uart_can_receive(unsigned index);
    bool uart_receive(unsigned index, std::uint8_t byte);
    std::vector<std::uint8_t> uart_take_tx(unsigned index);

    // Connects the RADIO to a shared medium under the node name `node`
    // (IEEE 802.15.4 frames; the node's position and power are the medium's).
    void connect_radio(mcuemu::VirtualAir &air, const std::string &node);

    // Identity: DEVICEID, DEVICEADDR and the encryption/identity roots in FICR
    // derive from `id`.
    void set_device_id(std::uint64_t id);

private:
    std::uint64_t service(std::uint64_t now);
    void build_peripherals();

    std::unique_ptr<cortexm::Core> core_;
    std::uint8_t *flash_{nullptr};
    std::uint8_t *ram_{nullptr};
    std::uint8_t *ficr_{nullptr};
    std::uint8_t *uicr_{nullptr};
    std::ostream *trace_{nullptr};
    std::uint32_t supply_mv_{3300};
    bool system_off_{false};
    std::uint32_t code_end_{0};
    PadObserver pad_observer_;
    AnalogProvider analog_provider_;
    std::array<std::unique_ptr<Peripheral>, 128> peripherals_;
    std::unique_ptr<Gpio> gpio_;
    std::vector<unsigned> serviced_;
};

} // namespace nrf52emu
