#include <limits>
#include "sim/tlsr826x_mcu.hpp"

#include <fstream>
#include <iterator>
#include <stdexcept>

namespace sim {

namespace {

bool parse_pad(std::string_view pad, unsigned &port, unsigned &bit) {
    if (pad.size() != 3 || pad[0] != 'P' || pad[1] < 'A' ||
        pad[1] >= 'A' + static_cast<int>(Tlsr826xMcu::PortCount) ||
        pad[2] < '0' || pad[2] > '7')
        return false;
    port = static_cast<unsigned>(pad[1] - 'A');
    bit = static_cast<unsigned>(pad[2] - '0');
    return true;
}

} // namespace

Tlsr826xMcu::Tlsr826xMcu(std::string name, mcuemu::Tlsr826xModel model,
                         std::uint32_t clock_hz)
    : Mcu(std::move(name)), device_(model, clock_hz), cpu_(device_) {
    device_.set_gpio_pad_observer([this](unsigned port) { drive_port(port); });
}

void Tlsr826xMcu::load_firmware(const std::string &path) {
    load_images({{path, 0}});
}

void Tlsr826xMcu::load_images(const std::vector<FirmwareImage> &images) {
    for (const FirmwareImage &entry : images) {
        std::ifstream input(entry.file, std::ios::binary);
        if (!input)
            throw std::runtime_error("cannot open firmware image: " + entry.file);
        std::vector<std::uint8_t> image(
            (std::istreambuf_iterator<char>(input)),
            std::istreambuf_iterator<char>());
        if (entry.skip > image.size())
            throw std::runtime_error("firmware image " + entry.file + " is shorter than 'from'");
        image.erase(image.begin(), image.begin() + entry.skip);
        if (entry.size != 0) {
            if (entry.size > image.size())
                throw std::runtime_error("firmware image " + entry.file + " is shorter than 'from' + 'size'");
            image.resize(entry.size);
        }
        device_.load_firmware(image, entry.offset);
    }
    cpu_.sync_flash();
    cpu_.reset();
}

void Tlsr826xMcu::reset() {
    cpu_.reset();
    state_ = McuState::Running;
    fault_.clear();
}

void Tlsr826xMcu::set_start_offset(std::uint64_t offset_ns) {
    device_.set_radio_time_offset(offset_ns);
}

McuState Tlsr826xMcu::run_until(std::uint64_t target_ns) {
    // The CPU runs instructions in batches up to the target time. Pad changes
    // are stamped with the device clock when the write happens, so batching
    // does not blur them; the target can be overshot by one instruction.
    while (state_ == McuState::Running && device_.network_time_ns() < target_ns) {
        const mcuemu::RunResult result =
            cpu_.run(std::numeric_limits<std::size_t>::max(), target_ns);
        switch (result.reason) {
        case mcuemu::StopReason::StepLimit:
            break;
        case mcuemu::StopReason::Halted:
            state_ = McuState::Halted;
            break;
        case mcuemu::StopReason::Sleeping:
            state_ = McuState::Sleeping;
            fault_ = result.message;
            break;
        case mcuemu::StopReason::Fault:
            state_ = McuState::Fault;
            fault_ = result.message;
            break;
        }
    }
    return state_;
}

std::vector<std::string> Tlsr826xMcu::pin_names() const {
    std::vector<std::string> names;
    for (unsigned port = 0; port != PortCount; ++port)
        for (unsigned bit = 0; bit != 8; ++bit)
            names.push_back(std::string{'P', static_cast<char>('A' + port),
                                        static_cast<char>('0' + bit)});
    return names;
}

void Tlsr826xMcu::connect_pin(std::string_view pad, Pin &pin) {
    unsigned port = 0, bit = 0;
    if (!parse_pad(pad, port, bit))
        throw std::invalid_argument("TLSR826x has no pad named '" +
                                    std::string(pad) + "' (expected PA0..PE7)");
    if (pins_[port][bit])
        throw std::invalid_argument("pad " + std::string(pad) +
                                    " is already connected");
    pins_[port][bit] = &pin;
    // Net -> pad: track the line level in the GPIO input register. A floating
    // line keeps the idle-high default of an unconnected input.
    pin.net().subscribe([this, port, bit](const NetChange &change) {
        sample_pin(port, bit, change.level);
    });
    sample_pin(port, bit, pin.net().level());
    // Pad -> net: apply the current OUT/OEN state.
    drive_port(port);
}

void Tlsr826xMcu::sample_pin(unsigned port, unsigned bit, Level level) {
    std::uint8_t value = device_.read8(mcuemu::Tlsr826x::RegisterBase + 0x580u + port * 8u);
    const std::uint8_t mask = static_cast<std::uint8_t>(1u << bit);
    if (level == Level::Low)
        value = static_cast<std::uint8_t>(value & ~mask);
    else
        value = static_cast<std::uint8_t>(value | mask);
    device_.set_gpio_input(port, value);
}

void Tlsr826xMcu::drive_port(unsigned port) {
    for (unsigned bit = 0; bit != 8; ++bit)
        drive_pad(port, bit);
}

void Tlsr826xMcu::drive_pad(unsigned port, unsigned bit) {
    Pin *pin = pins_[port][bit];
    if (!pin)
        return;
    const std::uint8_t mask = static_cast<std::uint8_t>(1u << bit);
    Drive drive;
    if (device_.gpio_is_gpio(port, bit) &&
               (device_.gpio_oen(port) & mask) == 0) {
        drive = (device_.gpio_output(port) & mask) ? Drive::High : Drive::Low;
    } else {
        // Input, or a pad owned by another function: only the internal pull
        // resistor acts on the line.
        switch (device_.gpio_pull(port, bit)) {
        case mcuemu::GpioPull::Up: drive = Drive::PullUp; break;
        case mcuemu::GpioPull::Down: drive = Drive::PullDown; break;
        default: drive = Drive::HiZ; break;
        }
    }
    pin->set_drive(drive, device_.network_time_ns());
}

void Tlsr826xMcu::attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                               const std::vector<std::string> &protocols) {
    for (const std::string &protocol : protocols)
        if (protocol != "zigbee")
            throw std::invalid_argument(
                "tlsr826x: '" + protocol +
                "' is not connected to the chip's RF registers yet (only zigbee)");
    device_.connect_radio(air, node);
}

void Tlsr826xMcu::read_memory(std::uint32_t address, std::uint8_t *out,
                              std::size_t length) {
    for (std::size_t i = 0; i < length; ++i)
        out[i] = device_.read8(address + static_cast<std::uint32_t>(i));
}

void Tlsr826xMcu::write_memory(std::uint32_t address, const std::uint8_t *data,
                               std::size_t length) {
    for (std::size_t i = 0; i < length; ++i)
        device_.write8(address + static_cast<std::uint32_t>(i), data[i]);
}

bool Tlsr826xMcu::uart_rx(unsigned index, std::uint8_t byte) {
    return index == 0 && device_.inject_uart_rx(byte);
}

std::vector<std::uint8_t> Tlsr826xMcu::uart_take_tx(unsigned index) {
    return index == 0 ? device_.take_uart_transmitted()
                      : std::vector<std::uint8_t>{};
}

} // namespace sim
