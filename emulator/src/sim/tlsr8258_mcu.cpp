#include "sim/tlsr8258_mcu.hpp"

#include <fstream>
#include <iterator>
#include <stdexcept>

namespace sim {

namespace {

bool parse_pad(std::string_view pad, unsigned &port, unsigned &bit) {
    if (pad.size() != 3 || pad[0] != 'P' || pad[1] < 'A' ||
        pad[1] >= 'A' + static_cast<int>(Tlsr8258Mcu::PortCount) ||
        pad[2] < '0' || pad[2] > '7')
        return false;
    port = static_cast<unsigned>(pad[1] - 'A');
    bit = static_cast<unsigned>(pad[2] - '0');
    return true;
}

} // namespace

Tlsr8258Mcu::Tlsr8258Mcu(std::string name, tc32emu::Tlsr8258Model model,
                         std::uint32_t clock_hz)
    : Mcu(std::move(name)), device_(model, clock_hz), cpu_(device_) {
    device_.set_gpio_pad_observer([this](unsigned port) { drive_port(port); });
    device_.set_i2c_pad_observer(
        [this](const tc32emu::I2cPads &pads, bool sda_low, bool scl_low) {
            on_i2c_pads(pads, sda_low, scl_low);
        });
    // The ADC reads the voltage of whatever net is wired to the pad. A pad the
    // chip itself drives sits at the rail or at ground: the SDK's VBAT mode
    // measures the supply that way, through a pad driven high.
    device_.set_adc_input_provider([this](unsigned port, unsigned bit) {
        const std::uint8_t mask = static_cast<std::uint8_t>(1u << bit);
        if (device_.gpio_is_gpio(port, bit) && (device_.gpio_oen(port) & mask) == 0)
            return (device_.gpio_output(port) & mask) ? device_.supply_mv() : 0u;
        Pin *pin = pins_[port][bit];
        return pin ? pin->net().millivolts() : 0u;
    });
}

void Tlsr8258Mcu::load_firmware(const std::string &path) {
    load_images({{path, 0}});
}

void Tlsr8258Mcu::load_images(const std::vector<FirmwareImage> &images) {
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

void Tlsr8258Mcu::reset() {
    cpu_.reset();
    state_ = McuState::Running;
    fault_.clear();
}

void Tlsr8258Mcu::set_start_offset(std::uint64_t offset_ns) {
    device_.set_radio_time_offset(offset_ns);
}

McuState Tlsr8258Mcu::run_until(std::uint64_t target_ns) {
    // One instruction per call keeps pad changes timestamped to within one
    // instruction (~60 ns at 48 MHz) of the executing code.
    while (state_ == McuState::Running && device_.network_time_ns() < target_ns) {
        const tc32emu::RunResult result = cpu_.run(1);
        switch (result.reason) {
        case tc32emu::StopReason::StepLimit:
            break;
        case tc32emu::StopReason::Halted:
            state_ = McuState::Halted;
            break;
        case tc32emu::StopReason::Sleeping:
            state_ = McuState::Sleeping;
            fault_ = result.message;
            break;
        case tc32emu::StopReason::Fault:
            state_ = McuState::Fault;
            fault_ = result.message;
            break;
        }
    }
    return state_;
}

std::vector<std::string> Tlsr8258Mcu::pin_names() const {
    std::vector<std::string> names;
    for (unsigned port = 0; port != PortCount; ++port)
        for (unsigned bit = 0; bit != 8; ++bit)
            names.push_back(std::string{'P', static_cast<char>('A' + port),
                                        static_cast<char>('0' + bit)});
    return names;
}

void Tlsr8258Mcu::connect_pin(std::string_view pad, Pin &pin) {
    unsigned port = 0, bit = 0;
    if (!parse_pad(pad, port, bit))
        throw std::invalid_argument("TLSR8258 has no pad named '" +
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

void Tlsr8258Mcu::sample_pin(unsigned port, unsigned bit, Level level) {
    std::uint8_t value = device_.read8(tc32emu::Tlsr8258::GpioBase + port * 8u);
    const std::uint8_t mask = static_cast<std::uint8_t>(1u << bit);
    if (level == Level::Low)
        value = static_cast<std::uint8_t>(value & ~mask);
    else
        value = static_cast<std::uint8_t>(value | mask);
    device_.set_gpio_input(port, value);
}

void Tlsr8258Mcu::drive_port(unsigned port) {
    for (unsigned bit = 0; bit != 8; ++bit)
        drive_pad(port, bit);
}

void Tlsr8258Mcu::drive_pad(unsigned port, unsigned bit) {
    Pin *pin = pins_[port][bit];
    if (!pin)
        return;
    const std::uint8_t mask = static_cast<std::uint8_t>(1u << bit);
    Drive drive;
    if (i2c_low_[port][bit]) {
        drive = Drive::Low; // open-drain I2C line pulled low by the master
    } else if (device_.gpio_is_gpio(port, bit) &&
               (device_.gpio_oen(port) & mask) == 0) {
        drive = (device_.gpio_output(port) & mask) ? Drive::High : Drive::Low;
    } else {
        // Input, or a pad owned by another function: only the internal pull
        // resistor acts on the line.
        switch (device_.gpio_pull(port, bit)) {
        case tc32emu::GpioPull::Up: drive = Drive::PullUp; break;
        case tc32emu::GpioPull::Down: drive = Drive::PullDown; break;
        default: drive = Drive::HiZ; break;
        }
    }
    pin->set_drive(drive, device_.network_time_ns());
}

void Tlsr8258Mcu::on_i2c_pads(const tc32emu::I2cPads &pads, bool sda_low,
                              bool scl_low) {
    if (!pads.valid())
        return;
    const unsigned sda_port = static_cast<unsigned>(pads.sda_port);
    const unsigned scl_port = static_cast<unsigned>(pads.scl_port);
    // When both lines change at once, SDA must never move while SCL is high:
    // lower SCL before changing SDA, and settle SDA before raising SCL.
    const bool scl_falls = scl_low && !i2c_low_[scl_port][pads.scl_bit];
    i2c_low_[sda_port][pads.sda_bit] = sda_low;
    i2c_low_[scl_port][pads.scl_bit] = scl_low;
    if (scl_falls) {
        drive_pad(scl_port, pads.scl_bit);
        drive_pad(sda_port, pads.sda_bit);
    } else {
        drive_pad(sda_port, pads.sda_bit);
        drive_pad(scl_port, pads.scl_bit);
    }
}

void Tlsr8258Mcu::attach_radio(tc32emu::VirtualAir &air, const std::string &node,
                               const std::vector<std::string> &protocols) {
    for (const std::string &protocol : protocols)
        if (protocol != "zigbee")
            throw std::invalid_argument(
                "tlsr8258: '" + protocol +
                "' is not connected to the chip's RF registers yet (only zigbee)");
    device_.connect_radio(air, node);
}

void Tlsr8258Mcu::read_memory(std::uint32_t address, std::uint8_t *out,
                              std::size_t length) {
    for (std::size_t i = 0; i < length; ++i)
        out[i] = device_.read8(address + static_cast<std::uint32_t>(i));
}

void Tlsr8258Mcu::write_memory(std::uint32_t address, const std::uint8_t *data,
                               std::size_t length) {
    for (std::size_t i = 0; i < length; ++i)
        device_.write8(address + static_cast<std::uint32_t>(i), data[i]);
}

bool Tlsr8258Mcu::uart_rx(unsigned index, std::uint8_t byte) {
    return index == 0 && device_.inject_uart_rx(byte);
}

std::vector<std::uint8_t> Tlsr8258Mcu::uart_take_tx(unsigned index) {
    return index == 0 ? device_.take_uart_transmitted()
                      : std::vector<std::uint8_t>{};
}

} // namespace sim
