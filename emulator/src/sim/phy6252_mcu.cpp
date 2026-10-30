#include "sim/phy6252_mcu.hpp"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace sim {
namespace {
Drive to_drive(phy6252emu::GpioDrive drive) {
    switch (drive) {
    case phy6252emu::GpioDrive::Low: return Drive::Low;
    case phy6252emu::GpioDrive::High: return Drive::High;
    case phy6252emu::GpioDrive::PullUp: return Drive::PullUp;
    case phy6252emu::GpioDrive::PullUpStrong: return Drive::PullUpStrong;
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
namespace {
// Debug aid: MCUEMU_PHY6252_TRACE_PC="FROM_NS:TO_NS" prints every executed
// instruction (cycle count, address) of every phy6252 MCU in that window to stderr;
// MCUEMU_PHY6252_TRACE_RANGE="FIRST:LAST" limits it to those addresses.
bool trace_window(std::uint64_t &from, std::uint64_t &to) {
    static const char *text = std::getenv("MCUEMU_PHY6252_TRACE_PC");
    if (!text) return false;
    char *end = nullptr;
    from = std::strtoull(text, &end, 0);
    if (*end != ':') return false;
    to = std::strtoull(end + 1, nullptr, 0);
    return true;
}
} // namespace

McuState Phy6252Mcu::run_until(std::uint64_t target_ns) {
    if (state_ != McuState::Running) return state_;
    // Debug aid: MCUEMU_PHY6252_DUMP_SRAM="NS:PATH" writes the 64 KiB SRAM once the clock passes NS.
    static bool dumped = false;
    if (!dumped) {
        if (const char *text = std::getenv("MCUEMU_PHY6252_DUMP_SRAM")) {
            char *end = nullptr;
            const std::uint64_t at = std::strtoull(text, &end, 0);
            if (*end == ':' && time_ns() >= at) {
                dumped = true;
                std::vector<std::uint8_t> sram(0x10000);
                if (chip_.core().read_memory(0x1fff0000u, sram.data(), sram.size()))
                    std::ofstream(end + 1, std::ios::binary).write(reinterpret_cast<const char *>(sram.data()), sram.size());
            }
        }
    }
    std::uint64_t trace_from = 0, trace_to = 0;
    if (trace_window(trace_from, trace_to))
    {
        std::uint32_t first = 0, last = 0xffffffffu;
        if (const char *range = std::getenv("MCUEMU_PHY6252_TRACE_RANGE")) {
            char *end = nullptr;
            first = static_cast<std::uint32_t>(std::strtoull(range, &end, 0));
            if (*end == ':') last = static_cast<std::uint32_t>(std::strtoull(end + 1, nullptr, 0));
        }
        chip_.core().set_pc_trace(time_ns() >= trace_from && time_ns() < trace_to ? &std::cerr : nullptr,
                                  first, last, false);
        std::uint32_t mmio_first = 0x40030000u, mmio_last = 0x40031fffu;
        if (const char *range = std::getenv("MCUEMU_PHY6252_TRACE_MMIO")) {
            char *end = nullptr;
            mmio_first = static_cast<std::uint32_t>(std::strtoull(range, &end, 0));
            if (*end == ':') mmio_last = static_cast<std::uint32_t>(std::strtoull(end + 1, nullptr, 0));
        }
        chip_.set_mmio_trace(std::getenv("MCUEMU_PHY6252_TRACE_MMIO") && time_ns() >= trace_from &&
                                     time_ns() < trace_to ? &std::cerr : nullptr,
                             mmio_first, mmio_last);
    }
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
