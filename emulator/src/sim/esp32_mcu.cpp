#include "sim/esp32_mcu.hpp"

#include <fstream>
#include <iterator>
#include <stdexcept>

namespace sim {

namespace {
esp32emu::Esp32::Config config_for(esp32emu::Variant variant) {
    esp32emu::Esp32::Config config;
    config.variant = variant;
    return config;
}
} // namespace

Esp32Mcu::Esp32Mcu(std::string name, esp32emu::Variant variant)
    : Mcu(std::move(name)), variant_(variant), chip_(config_for(variant)) {
    chip_.set_pad_observer([this](unsigned pad) { drive_pad(pad); });
    chip_.on_uart_tx = [this](std::uint8_t byte) { uart_tx_.push_back(byte); };
}

void Esp32Mcu::load_firmware(const std::string &path) { load_images({{path, 0}}); }

void Esp32Mcu::load_images(const std::vector<FirmwareImage> &images) {
    for (const FirmwareImage &entry : images) {
        std::ifstream input(entry.file, std::ios::binary);
        if (!input)
            throw std::runtime_error("cannot open firmware image: " + entry.file);
        std::vector<std::uint8_t> image((std::istreambuf_iterator<char>(input)),
                                        std::istreambuf_iterator<char>());
        if (entry.skip > image.size())
            throw std::runtime_error("firmware image " + entry.file + " is shorter than 'from'");
        image.erase(image.begin(), image.begin() + entry.skip);
        if (entry.size != 0) {
            if (entry.size > image.size())
                throw std::runtime_error("firmware image " + entry.file + " is shorter than 'from' + 'size'");
            image.resize(entry.size);
        }
        chip_.load_flash(image, entry.offset);
    }
    chip_.reset();
}

void Esp32Mcu::reset() {
    chip_.reset();
    state_ = McuState::Running;
    fault_.clear();
}

void Esp32Mcu::set_start_offset(std::uint64_t offset_ns) {
    chip_.core().advance_to(chip_.core().ns_to_cycles(offset_ns));
}

McuState Esp32Mcu::run_until(std::uint64_t target_ns) {
    while (state_ == McuState::Running && chip_.time_ns() < target_ns) {
        const riscv::RunResult result = chip_.run_until(target_ns);
        switch (result.reason) {
        case riscv::StopReason::Deadline:
        case riscv::StopReason::StepLimit:
            break;
        case riscv::StopReason::Halted:
            state_ = McuState::Halted;
            fault_ = result.message;
            break;
        case riscv::StopReason::Fault:
            state_ = McuState::Fault;
            fault_ = result.message;
            break;
        case riscv::StopReason::Reset:
            chip_.reset(); // a software reset: the ROM loader starts the image again
            break;
        }
    }
    return state_;
}

namespace {
Drive to_drive(esp32emu::PadDrive drive) {
    switch (drive) {
    case esp32emu::PadDrive::Low: return Drive::Low;
    case esp32emu::PadDrive::High: return Drive::High;
    case esp32emu::PadDrive::PullUp: return Drive::PullUp;
    case esp32emu::PadDrive::PullDown: return Drive::PullDown;
    default: return Drive::HiZ;
    }
}
} // namespace

std::vector<std::string> Esp32Mcu::pin_names() const {
    std::vector<std::string> names;
    for (unsigned pad = 0; pad != chip_.gpio_pins(); ++pad)
        names.push_back("GPIO" + std::to_string(pad));
    return names;
}

void Esp32Mcu::connect_pin(std::string_view pad_text, Pin &pin) {
    unsigned pad = 0;
    const std::string text(pad_text);
    bool ok = text.rfind("GPIO", 0) == 0 && text.size() > 4 && text.size() <= 6;
    if (ok) {
        for (std::size_t i = 4; i < text.size(); ++i)
            ok = ok && text[i] >= '0' && text[i] <= '9';
        if (ok)
            pad = static_cast<unsigned>(std::stoul(text.substr(4)));
    }
    if (!ok || pad >= chip_.gpio_pins())
        throw std::invalid_argument(std::string(model()) + " has no pad named '" + text + "' (expected GPIO0..GPIO" +
                                    std::to_string(chip_.gpio_pins() - 1) + ")");
    if (pins_[pad])
        throw std::invalid_argument("pad " + text + " is already connected");
    pins_[pad] = &pin;
    pin.net().subscribe([this, pad](const NetChange &change) { chip_.set_pad_input(pad, change.level == Level::High); });
    chip_.set_pad_input(pad, pin.net().level() == Level::High);
    drive_pad(pad);
}

void Esp32Mcu::drive_pad(unsigned pad) {
    if (pad < pins_.size() && pins_[pad])
        pins_[pad]->set_drive(to_drive(chip_.pad_drive(pad)), time_ns());
}

void Esp32Mcu::attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                              const std::vector<std::string> &protocols) {
    for (const std::string &protocol : protocols)
        if (protocol != "zigbee")
            throw std::invalid_argument(std::string(model()) + ": '" + protocol + "' is not connected to the radio (only zigbee)");
    chip_.connect_radio(air, node);
}

void Esp32Mcu::read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) {
    if (!chip_.core().read_memory(address, out, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

void Esp32Mcu::write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) {
    if (!chip_.core().write_memory(address, data, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

std::vector<std::uint8_t> Esp32Mcu::uart_take_tx(unsigned index) {
    if (index != 0)
        return {};
    std::vector<std::uint8_t> out;
    out.swap(uart_tx_);
    return out;
}

} // namespace sim
