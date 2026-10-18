#include "sim/bl702_mcu.hpp"

#include <fstream>
#include <iterator>
#include <stdexcept>

namespace sim {

namespace {
// The EUI-64 the eFuse reports: derived from the instance name, so two BL702 nodes on one air never share an
// address (FNV-1a of the name).
bl702emu::Bl702::Config config_for(const std::string &name) {
    std::uint64_t hash = 14695981039346656037ull;
    for (unsigned char c : name)
        hash = (hash ^ c) * 1099511628211ull;
    bl702emu::Bl702::Config config;
    for (unsigned i = 0; i < config.mac.size(); ++i)
        config.mac[i] = static_cast<std::uint8_t>(hash >> (8 * i));
    return config;
}
} // namespace

Bl702Mcu::Bl702Mcu(std::string name) : Mcu(name), chip_(config_for(name)) {
    chip_.on_uart_tx = [this](std::uint8_t byte) { uart_tx_.push_back(byte); };
}

void Bl702Mcu::load_firmware(const std::string &path) { load_images({{path, 0}}); }

void Bl702Mcu::load_images(const std::vector<FirmwareImage> &images) {
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
    reset();
}

void Bl702Mcu::reset() {
    chip_.reset(); // the boot ROM: throws when the flash holds no BFNP header
    state_ = McuState::Running;
    fault_.clear();
}

void Bl702Mcu::set_start_offset(std::uint64_t offset_ns) {
    chip_.core().advance_to(chip_.core().ns_to_cycles(offset_ns));
}

McuState Bl702Mcu::run_until(std::uint64_t target_ns) {
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
            chip_.reset(); // a software reset: the boot ROM starts the image again
            break;
        }
    }
    return state_;
}

void Bl702Mcu::connect_pin(std::string_view pad, Pin &) {
    throw std::invalid_argument("bl702 has no modelled pads (asked for '" + std::string(pad) + "')");
}

void Bl702Mcu::attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                            const std::vector<std::string> &protocols) {
    for (const std::string &protocol : protocols)
        if (protocol != "zigbee")
            throw std::invalid_argument("bl702: '" + protocol + "' is not connected to the radio (only zigbee)");
    chip_.connect_radio(air, node);
}

void Bl702Mcu::read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) {
    if (!chip_.core().read_memory(address, out, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

void Bl702Mcu::write_memory(std::uint32_t address, const std::uint8_t *data, std::size_t length) {
    if (!chip_.core().write_memory(address, data, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

std::vector<std::uint8_t> Bl702Mcu::uart_take_tx(unsigned index) {
    if (index != 0)
        return {};
    std::vector<std::uint8_t> out;
    out.swap(uart_tx_);
    return out;
}

} // namespace sim
