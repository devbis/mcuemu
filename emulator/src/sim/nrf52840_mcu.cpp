#include "sim/nrf52840_mcu.hpp"

#include <fstream>
#include <iterator>
#include <stdexcept>

namespace sim {

namespace {

std::string pad_name(unsigned pad) {
    const unsigned port = pad / 32, pin = pad % 32;
    return "P" + std::to_string(port) + "." + (pin < 10 ? "0" : "") + std::to_string(pin);
}

Drive to_drive(nrf52emu::PadDrive drive) {
    switch (drive) {
    case nrf52emu::PadDrive::Low: return Drive::Low;
    case nrf52emu::PadDrive::High: return Drive::High;
    case nrf52emu::PadDrive::PullUp: return Drive::PullUp;
    case nrf52emu::PadDrive::PullDown: return Drive::PullDown;
    default: return Drive::HiZ;
    }
}

} // namespace

Nrf52840Mcu::Nrf52840Mcu(std::string name) : Mcu(std::move(name)) {
    chip_.set_pad_observer([this](unsigned pad) { drive_pad(pad); });
    // The SAADC reads the voltage of whatever net is wired to the pad; an
    // unwired pad reads its own drive (high is the supply).
    chip_.set_analog_provider([this](unsigned pad) -> std::uint32_t {
        if (Pin *pin = pins_[pad])
            return pin->net().millivolts();
        return chip_.pad_drive(pad) == nrf52emu::PadDrive::High ? chip_.supply_mv() : 0;
    });
}

void Nrf52840Mcu::load_firmware(const std::string &path) { load_images({{path, 0}}); }

void Nrf52840Mcu::load_images(const std::vector<FirmwareImage> &images) {
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

void Nrf52840Mcu::reset() {
    chip_.reset();
    state_ = McuState::Running;
    fault_.clear();
}

void Nrf52840Mcu::set_start_offset(std::uint64_t offset_ns) {
    chip_.core().advance_to(chip_.core().ns_to_cycles(offset_ns));
}

McuState Nrf52840Mcu::run_until(std::uint64_t target_ns) {
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
        if (chip_.system_off()) {
            // Nothing runs until a pin wakes the chip; the world can still
            // drive the pads, so keep time moving.
            chip_.core().advance_to(target);
            break;
        }
    }
    return state_;
}

std::vector<std::string> Nrf52840Mcu::pin_names() const {
    std::vector<std::string> names;
    for (unsigned pad = 0; pad != nrf52emu::Nrf52840::PadCount; ++pad)
        names.push_back(pad_name(pad));
    return names;
}

void Nrf52840Mcu::connect_pin(std::string_view pad_text, Pin &pin) {
    unsigned pad = 0;
    if (!nrf52emu::Nrf52840::parse_pad(std::string(pad_text), pad))
        throw std::invalid_argument("nRF52840 has no pad named '" + std::string(pad_text) +
                                    "' (expected P0.00..P0.31 or P1.00..P1.15)");
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

void Nrf52840Mcu::drive_pad(unsigned pad) {
    if (Pin *pin = pins_[pad])
        pin->set_drive(to_drive(chip_.pad_drive(pad)), time_ns());
}

void Nrf52840Mcu::read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) {
    if (!chip_.core().read_memory(address, out, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

void Nrf52840Mcu::write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) {
    if (!chip_.core().write_memory(address, data, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

} // namespace sim
