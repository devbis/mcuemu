#include "efr32emu/efr32mg21.hpp"

#include "efr32/blocks.hpp"

#include <algorithm>
#include <cstring>
#include <ostream>
#include <stdexcept>

namespace efr32emu {

namespace {

constexpr std::uint32_t RadioBase = 0x40090000, RadioEnd = 0x40096000;
// Peripheral blocks are 16 KiB apart; slot = (offset from 0x40000000) / 0x4000.
constexpr unsigned CmuSlot = 2, HfxoSlot = 3, MscSlot = 12, GpioSlot = 15;

} // namespace

Efr32mg21::Efr32mg21() {
    cortexm::Core::Config config;
    config.model = cortexm::Model::M33;
    config.clock_hz = ClockHz;
    config.irq_count = IrqCount;
    config.priority_bits = 4;
    core_ = std::make_unique<cortexm::Core>(config);
    flash_ = core_->map_memory(0, FlashSize, false);
    ram_ = core_->map_memory(RamBase, RamSize, true);
    userdata_ = core_->map_memory(UserDataBase, 0x1000, false);
    devinfo_ = core_->map_memory(DevInfoBase, 0x1000, false);
    std::memset(ram_, 0, RamSize);
    std::memset(userdata_, 0xff, 0x1000);
    std::memset(devinfo_, 0xff, 0x1000);
    set_device_id(0x0123456789abcdefull);

    auto cmu = std::make_unique<Cmu>(*this);
    cmu_ = cmu.get();
    blocks_[CmuSlot] = std::move(cmu);
    blocks_[HfxoSlot] = std::make_unique<Hfxo>(*this, *cmu_);
    blocks_[MscSlot] = std::make_unique<Msc>(*this);
    auto gpio = std::make_unique<Gpio>(*this);
    gpio_ = gpio.get();
    blocks_[GpioSlot] = std::move(gpio);
    radio_ = std::make_unique<Radio>(*this);

    core_->map_mmio(PeripheralBase, PeripheralSize, *this);
    core_->set_service_handler([this](std::uint64_t now) { return service(now); });
    // Flash is written through the MSC; a CPU store to it is a bus fault.
    core_->set_write_protect_handler([](std::uint32_t, unsigned, std::uint32_t) { return false; });
}

Efr32mg21::~Efr32mg21() = default;

void Efr32mg21::set_device_id(std::uint64_t id) {
    // DEVINFO.EUI64L/H: Silicon Labs OUI 84:FD:27 with the id in the low bytes.
    const std::uint8_t eui[8] = {static_cast<std::uint8_t>(id), static_cast<std::uint8_t>(id >> 8),
                                 static_cast<std::uint8_t>(id >> 16), 0xfe, 0xff, 0x27, 0xfd, 0x84};
    std::memcpy(devinfo_ + 0x1a0, eui, 8);
}

Block &Efr32mg21::block_at(std::uint32_t slot) {
    if (!blocks_[slot])
        blocks_[slot] = std::make_unique<RegBlock>(*this);
    return *blocks_[slot];
}

void Efr32mg21::load_firmware(const std::vector<std::uint8_t> &image, std::uint32_t offset) {
    if (image.size() >= 8 && offset == 0) {
        std::uint32_t vector[2];
        std::memcpy(vector, image.data(), 8);
        // An application-only image has its vector table in the application slot.
        if ((vector[0] >> 24) == 0x20 && (vector[1] & ~1u) >= AppBase && (vector[1] & ~1u) < FlashSize)
            offset = AppBase;
    }
    if (offset > FlashSize || image.size() > FlashSize - offset)
        throw std::runtime_error("firmware does not fit in the 512 KiB flash");
    std::copy(image.begin(), image.end(), flash_ + offset);
    code_end_ = std::max<std::uint32_t>(code_end_, offset + static_cast<std::uint32_t>(image.size()));
    if (offset == AppBase)
        boot_vector_ = AppBase;
    core_->invalidate_code();
}

void Efr32mg21::reset() {
    gpio_->reset();
    for (auto &block : blocks_)
        if (block)
            block->reset();
    radio_->reset();
    core_->reset(boot_vector_);
}

cortexm::RunResult Efr32mg21::run(std::uint64_t until_cycle) {
    for (;;) {
        cortexm::RunResult result = core_->run(until_cycle);
        if (result.reason != cortexm::StopReason::Reset)
            return result;
        reset();
    }
}

std::uint64_t Efr32mg21::service(std::uint64_t now) { return radio_->service(now); }

void Efr32mg21::connect_radio(mcuemu::VirtualAir &air, const std::string &node) { radio_->connect(air, node); }

void Efr32mg21::flash_changed(std::uint32_t address, std::uint32_t length) {
    (void)length;
    if (address < code_end_)
        core_->invalidate_code();
}

PadDrive Efr32mg21::pad_drive(unsigned pad) const { return pad < PadCount ? gpio_->drive(pad) : PadDrive::HiZ; }

void Efr32mg21::set_pad_input(unsigned pad, bool high) {
    if (pad < PadCount)
        gpio_->set_external(pad, high);
}

void Efr32mg21::notify_pad(unsigned pad) {
    if (pad_observer_)
        pad_observer_(pad);
}

bool Efr32mg21::parse_pad(const std::string &name, unsigned &pad) {
    // "PA0" .. "PD15"
    if (name.size() < 3 || name.size() > 4 || name[0] != 'P' || name[1] < 'A' || name[1] > 'D')
        return false;
    unsigned pin = 0;
    for (std::size_t i = 2; i != name.size(); ++i) {
        if (name[i] < '0' || name[i] > '9')
            return false;
        pin = pin * 10 + static_cast<unsigned>(name[i] - '0');
    }
    if (pin >= 16 || (name.size() == 4 && name[2] == '0'))
        return false;
    pad = static_cast<unsigned>(name[1] - 'A') * 16 + pin;
    return true;
}

std::string Efr32mg21::pad_name(unsigned pad) {
    return std::string("P") + static_cast<char>('A' + pad / 16) + std::to_string(pad % 16);
}

std::uint32_t Efr32mg21::read(std::uint32_t address, unsigned size) {
    std::uint32_t value = 0;
    if (address >= RadioBase && address < RadioEnd) {
        value = radio_->read(address - RadioBase);
    } else {
        const std::uint32_t offset = address - PeripheralBase;
        value = block_at(offset >> 14).read(offset & 0xffc);
    }
    if (trace_)
        *trace_ << "R 0x" << std::hex << address << " -> 0x" << value << " pc=0x" << core_->pc() << std::dec
                << " t=" << time_ns() << '\n';
    if (size < 4)
        value = (value >> ((address & 3u) * 8)) & (size == 1 ? 0xffu : 0xffffu);
    return value;
}

void Efr32mg21::write(std::uint32_t address, std::uint32_t value, unsigned size) {
    if (trace_)
        *trace_ << "W 0x" << std::hex << address << " <- 0x" << value << " pc=0x" << core_->pc() << std::dec
                << " t=" << time_ns() << '\n';
    if (size < 4)
        value <<= (address & 3u) * 8;
    if (address >= RadioBase && address < RadioEnd) {
        radio_->write(address - RadioBase, value);
        return;
    }
    const std::uint32_t offset = address - PeripheralBase;
    Block &block = block_at(offset >> 14);
    const std::uint32_t reg = offset & 0xffc;
    // Series 2 aliases: +0x1000 sets, +0x2000 clears, +0x3000 toggles the written bits.
    switch ((offset >> 12) & 3u) {
    case 1: block.write(reg, block.peek(reg) | value); break;
    case 2: block.write(reg, block.peek(reg) & ~value); break;
    case 3: block.write(reg, block.peek(reg) ^ value); break;
    default: block.write(reg, value); break;
    }
}

} // namespace efr32emu
