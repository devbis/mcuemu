#include "sim/efr32mg21_mcu.hpp"

#include <fstream>
#include <iterator>
#include <stdexcept>

namespace sim {

namespace {

Drive to_drive(efr32emu::PadDrive drive) {
    switch (drive) {
    case efr32emu::PadDrive::Low: return Drive::Low;
    case efr32emu::PadDrive::High: return Drive::High;
    case efr32emu::PadDrive::PullUp: return Drive::PullUp;
    case efr32emu::PadDrive::PullDown: return Drive::PullDown;
    default: return Drive::HiZ;
    }
}

} // namespace

Efr32mg21Mcu::Efr32mg21Mcu(std::string name) : Mcu(std::move(name)) {
    chip_.set_pad_observer([this](unsigned pad) { drive_pad(pad); });
}

void Efr32mg21Mcu::load_firmware(const std::string &path) { load_images({{path, 0}}); }

void Efr32mg21Mcu::load_images(const std::vector<FirmwareImage> &images) {
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
                throw std::runtime_error("firmware image " + entry.file +
                                         " is shorter than 'from' + 'size'");
            image.resize(entry.size);
        }
        chip_.load_firmware(image, entry.offset);
    }
    chip_.reset();
}

void Efr32mg21Mcu::reset() {
    chip_.reset();
    state_ = McuState::Running;
    fault_.clear();
}

void Efr32mg21Mcu::set_start_offset(std::uint64_t offset_ns) {
    chip_.core().advance_to(chip_.core().ns_to_cycles(offset_ns));
}

McuState Efr32mg21Mcu::run_until(std::uint64_t target_ns) {
    const std::uint64_t target = chip_.core().ns_to_cycles(target_ns);
    while (state_ == McuState::Running && chip_.core().cycles() < target) {
        const cortexm::RunResult result = chip_.run(target);
        switch (result.reason) {
        case cortexm::StopReason::Deadline:
        case cortexm::StopReason::StepLimit:
            break;
        case cortexm::StopReason::Halted:
            state_ = McuState::Halted;
            fault_ = result.message;
            break;
        case cortexm::StopReason::Fault:
            state_ = McuState::Fault;
            fault_ = result.message;
            break;
        case cortexm::StopReason::Reset:
            break; // Nrf52840::run restarts after resets
        }
    }
    return state_;
}

std::vector<std::string> Efr32mg21Mcu::pin_names() const {
    std::vector<std::string> names;
    for (unsigned pad = 0; pad != efr32emu::Efr32mg21::PadCount; ++pad)
        names.push_back(efr32emu::Efr32mg21::pad_name(pad));
    return names;
}

void Efr32mg21Mcu::connect_pin(std::string_view pad_text, Pin &pin) {
    unsigned pad = 0;
    if (!efr32emu::Efr32mg21::parse_pad(std::string(pad_text), pad))
        throw std::invalid_argument("EFR32MG21 has no pad named '" + std::string(pad_text) +
                                    "' (expected PA0..PD15)");
    if (pins_[pad])
        throw std::invalid_argument("pad " + std::string(pad_text) + " is already connected");
    pins_[pad] = &pin;
    // Net -> pad: the line level as the input buffer sees it.
    pin.net().subscribe([this, pad](const NetChange &change) {
        chip_.set_pad_input(pad, change.level == Level::High);
    });
    chip_.set_pad_input(pad, pin.net().level() == Level::High);
    // Pad -> net: whatever the pad drives now.
    drive_pad(pad);
}

void Efr32mg21Mcu::drive_pad(unsigned pad) {
    if (Pin *pin = pins_[pad])
        pin->set_drive(to_drive(chip_.pad_drive(pad)), time_ns());
}

void Efr32mg21Mcu::attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                               const std::vector<std::string> &protocols) {
    for (const std::string &protocol : protocols)
        if (protocol != "zigbee")
            throw std::invalid_argument("efr32mg21: '" + protocol +
                                        "' is not connected to the RADIO (only zigbee)");
    chip_.connect_radio(air, node);
}

void Efr32mg21Mcu::read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) {
    if (!chip_.core().read_memory(address, out, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

std::vector<std::uint8_t> Efr32mg21Mcu::uart_take_tx(unsigned index) {
    if (index != 0)
        return {};
    std::string &all = chip_.uart0_output();
    std::vector<std::uint8_t> bytes(all.begin(), all.end());
    all.clear();
    return bytes;
}

void Efr32mg21Mcu::write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) {
    if (!chip_.core().write_memory(address, data, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

} // namespace sim
