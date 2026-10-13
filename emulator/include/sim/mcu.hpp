#pragma once

#include "sim/net.hpp"
#include "tc32emu/radio.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace sim {

enum class McuState {
    Running,
    Sleeping, // asleep with no wake event; will not run again by itself
    Halted,
    Fault,
};

// One piece of a flash image: `size` bytes of a file (all of it when 0),
// starting `skip` bytes in, placed at `offset` in the flash. Slicing one file
// into several pieces lets an image linked for a bootloader layout be placed
// the way the chip sees it.
struct FirmwareImage {
    std::string file;
    std::uint32_t offset{0};
    std::uint32_t skip{0};
    std::uint32_t size{0};
};

// Chip-independent view of one emulated MCU as seen by the simulation world.
// Implementations wrap a concrete chip model (CPU core + peripherals).
class Mcu {
public:
    explicit Mcu(std::string name) : name_(std::move(name)) {}
    virtual ~Mcu() = default;
    Mcu(const Mcu &) = delete;
    Mcu &operator=(const Mcu &) = delete;

    // Instance id, e.g. "sensor_node.main".
    const std::string &name() const noexcept { return name_; }
    virtual std::string_view model() const = 0;

    virtual void load_firmware(const std::string &path) = 0;
    // Loads several images (e.g. a bootloader at 0 and the application at
    // 0x8000). The default accepts only the single image at offset 0.
    virtual void load_images(const std::vector<FirmwareImage> &images) {
        if (images.size() != 1 || images[0].offset != 0)
            throw std::invalid_argument(std::string(model()) +
                                        " loads a single firmware image at offset 0");
        load_firmware(images[0].file);
    }
    virtual void reset() = 0;

    // Power-on delay: the MCU's clock starts at `offset_ns` of world time.
    // Must be set before the first run_until().
    virtual void set_start_offset(std::uint64_t offset_ns) = 0;
    // Current position of this MCU on the world timeline.
    virtual std::uint64_t time_ns() const = 0;
    // Executes until time_ns() >= `target_ns` or the MCU stops. A single
    // instruction (or a sleep jump) may overshoot the target slightly.
    virtual McuState run_until(std::uint64_t target_ns) = 0;
    virtual McuState state() const noexcept = 0;
    virtual const std::string &fault_message() const noexcept = 0;

    virtual std::vector<std::string> pin_names() const { return {}; }
    // Wires the named MCU pad to `pin` (already attached to a net). Throws
    // std::invalid_argument for an unknown or unsupported pad.
    virtual void connect_pin(std::string_view pad, Pin &pin) = 0;

    // Connects the MCU's radio to a medium under the node name `node`.
    // `protocols` lists what the firmware will use ("zigbee", "ble"). Throws
    // std::invalid_argument when the model cannot do that.
    virtual void attach_radio(tc32emu::VirtualAir &air, const std::string &node,
                              const std::vector<std::string> &protocols) {
        (void)air;
        (void)node;
        (void)protocols;
        throw std::invalid_argument(std::string(model()) +
                                    " has no radio model");
    }

    // Target memory as the CPU sees it (used by RTT and debugging). Throws
    // std::runtime_error for an unmapped address and std::invalid_argument
    // when the model does not expose memory.
    virtual void read_memory(std::uint32_t address, std::uint8_t *out,
                             std::size_t length) {
        (void)address;
        (void)out;
        (void)length;
        throw std::invalid_argument(std::string(model()) + " does not expose memory");
    }
    virtual void write_memory(std::uint32_t address, const std::uint8_t *data,
                              std::size_t length) {
        (void)address;
        (void)data;
        (void)length;
        throw std::invalid_argument(std::string(model()) + " does not expose memory");
    }

    // Number of modeled UARTs, and whether UART `index` can accept bytes.
    virtual unsigned uart_count() const { return 0; }
    virtual bool uart_can_receive(unsigned index) const {
        (void)index;
        return false;
    }
    // Byte stream access to UART `index`; unsupported ports return false/empty.
    virtual bool uart_rx(unsigned index, std::uint8_t byte) {
        (void)index;
        (void)byte;
        return false;
    }
    virtual std::vector<std::uint8_t> uart_take_tx(unsigned index) {
        (void)index;
        return {};
    }

private:
    std::string name_;
};

} // namespace sim
