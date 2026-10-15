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

void Esp32Mcu::connect_pin(std::string_view pad, Pin &) {
    throw std::invalid_argument(std::string(model()) + " has no GPIO model yet, cannot wire pad '" + std::string(pad) + "'");
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
