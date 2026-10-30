#include "sim/phy6252_mcu.hpp"
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace sim {
namespace {
Drive to_drive(phy6252emu::GpioDrive drive) {
    switch (drive) {
    case phy6252emu::GpioDrive::Low: return Drive::Low;
    case phy6252emu::GpioDrive::High: return Drive::High;
    case phy6252emu::GpioDrive::PullUp: return Drive::PullUp;
    case phy6252emu::GpioDrive::PullDown: return Drive::PullDown;
    default: return Drive::HiZ;
    }
}
} // namespace

std::vector<std::string> Phy6252Mcu::pin_names() const {
    std::vector<std::string> names;
    for (unsigned pin = 0; pin < GpioCount; ++pin)
        names.push_back("GPIO" + std::to_string(pin));
    return names;
}

void Phy6252Mcu::load_images(const std::vector<FirmwareImage> &images) {
    if (images.size() != 1 || images[0].offset || images[0].skip || images[0].size)
        throw std::invalid_argument("phy6252 expects one complete ROM package");
    load_firmware(images[0].file);
}
void Phy6252Mcu::load_firmware(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open firmware image: " + path);
    std::vector<std::uint8_t> image((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    chip_.load_firmware(image);
    reset();
}
void Phy6252Mcu::reset() { chip_.reset(); state_ = McuState::Running; fault_.clear(); }
McuState Phy6252Mcu::run_until(std::uint64_t target_ns) {
    if (state_ != McuState::Running) return state_;
    const auto result = chip_.run(chip_.core().ns_to_cycles(target_ns));
    if (result.reason == cortexm::StopReason::Fault || result.reason == cortexm::StopReason::Halted) {
        state_ = result.reason == cortexm::StopReason::Fault ? McuState::Fault : McuState::Halted;
        fault_ = result.message;
    } else if (result.reason == cortexm::StopReason::Reset) {
        reset();
    }
    return state_;
}
void Phy6252Mcu::connect_pin(std::string_view pad_text, Pin &pin) {
    const std::string text(pad_text);
    bool valid = text.rfind("GPIO", 0) == 0 && text.size() > 4 && text.size() <= 6;
    for (std::size_t i = 4; valid && i < text.size(); ++i)
        valid = text[i] >= '0' && text[i] <= '9';
    unsigned pad = 0;
    if (valid)
        pad = static_cast<unsigned>(std::stoul(text.substr(4)));
    if (!valid || pad >= GpioCount)
        throw std::invalid_argument("phy6252 has no pad named '" + text + "' (expected GPIO0..GPIO22)");
    if (pins_[pad])
        throw std::invalid_argument("pad " + text + " is already connected");
    pins_[pad] = &pin;
    pin.net().subscribe([this, pad](const NetChange &change) {
        chip_.set_gpio_input(pad, change.level == Level::High);
    });
    chip_.set_gpio_input(pad, pin.net().level() == Level::High);
    drive_gpio(pad);
}

void Phy6252Mcu::drive_gpio(unsigned pad) {
    if (pad < pins_.size() && pins_[pad])
        pins_[pad]->set_drive(to_drive(chip_.gpio_drive(pad)), time_ns());
}
void Phy6252Mcu::attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                              const std::vector<std::string> &protocols) {
    for (const std::string &protocol : protocols)
        if (protocol != "zigbee")
            throw std::invalid_argument("phy6252: '" + protocol + "' is not connected to the radio (only zigbee)");
    chip_.connect_radio(air, node);
}
void Phy6252Mcu::read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) {
    if (!chip_.core().read_memory(address, out, length)) throw std::runtime_error("unmapped PHY6252 memory");
}
void Phy6252Mcu::write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) {
    if (!chip_.core().write_memory(address, data, length)) throw std::runtime_error("unmapped PHY6252 memory");
}
bool Phy6252Mcu::uart_rx(unsigned index, std::uint8_t byte) {
    if (index != 0) return false;
    chip_.uart_receive(byte);
    return true;
}
std::vector<std::uint8_t> Phy6252Mcu::uart_take_tx(unsigned index) {
    return index == 0 ? chip_.take_uart_tx() : std::vector<std::uint8_t>{};
}
} // namespace sim
