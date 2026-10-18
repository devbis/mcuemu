#include "bl702emu/bl702.hpp"

#include "blocks.hpp"
#include "rom.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace bl702emu {

namespace {
// The boot header of a flash image (176 bytes; bflb-mcu-tool's bootheader_cfg_keys.py). Each segment is a
// 16-byte header (load address, length, data CRC, header CRC) and its data.
constexpr std::uint32_t BootMagic = 0x504e4642; // "BFNP", little-endian
constexpr std::uint32_t BootHeaderLength = 176, SegmentHeaderLength = 16;
constexpr std::uint32_t BootFlagsOffset = 116, SegmentCountOffset = 120, BootEntryOffset = 124;
constexpr std::uint32_t NoSegmentBit = 1u << 8; // no_segment in the boot config word

std::uint32_t little_endian(const std::uint8_t *bytes) {
    std::uint32_t value;
    std::memcpy(&value, bytes, 4);
    return value;
}

std::uint32_t lane_mask(unsigned size) { return size == 1 ? 0xffu : size == 2 ? 0xffffu : 0xffffffffu; }

constexpr unsigned M154Irq = 64; // the 802.15.4 interrupt (the SoC's dts)
} // namespace

Bl702::Bl702(const Config &config)
    : config_(config), core_(riscv::Core::Config{ClockHz, true}), flash_(config.flash_size, 0xff) {
    window_ = core_.map_memory(FlashBase, config_.flash_size, false);
    sync_window();
    core_.map_memory(RomBase, RomSize, false);
    core_.map_memory(SramBase, SramSize, true);
    core_.map_memory(ItcmBase, ItcmSize, true);
    core_.map_mmio(PeriphBase, PeriphSize, *this);
    core_.map_mmio(M154Base, M154Size, *this);
    core_.map_mmio(ClintBase, 0x0100'0000, *this); // CLINT and CLIC; the rest of the window reads back

    auto phy = std::make_unique<RfPhy>(*this);
    auto mac = std::make_unique<Ieee802154>(*this, *phy);
    phy->connect(*mac);
    mac_ = mac.get();
    blocks_.push_back(std::make_unique<Uart0>(*this));
    blocks_.push_back(std::make_unique<Timer0>(*this));
    blocks_.push_back(std::make_unique<Efuse>(*this, config_.mac));
    blocks_.push_back(std::make_unique<Aes>(*this));
    blocks_.push_back(std::make_unique<Trng>(*this));
    blocks_.push_back(std::make_unique<L1c>(*this));
    blocks_.push_back(std::make_unique<SfCtrl>(*this));
    blocks_.push_back(std::move(phy));
    blocks_.push_back(std::move(mac));
    auto clic = std::make_unique<Clic>(*this);
    clic_ = clic.get();
    blocks_.push_back(std::move(clic));
    blocks_.push_back(std::make_unique<Clint>(*this, *clic_));
    core_.set_clic([this](std::uint32_t mil, std::uint32_t thresh, std::uint32_t &level, bool claim) {
        return clic_->pick(mil, thresh, level, claim);
    });

    rom_ = std::make_unique<Rom>(*this);
    core_.set_service_handler([this](std::uint64_t now) { return service(now); });
}

Bl702::~Bl702() = default;

void Bl702::load_flash(const std::vector<std::uint8_t> &data, std::uint32_t offset) {
    if (static_cast<std::uint64_t>(offset) + data.size() > config_.flash_size)
        throw std::invalid_argument("image does not fit the flash");
    std::memcpy(flash_.data() + offset, data.data(), data.size());
    flash_modified();
}

void Bl702::reset() {
    for (auto &block : blocks_)
        block->reset();
    scratch_.clear();
    std::uint8_t *sram = core_.host_pointer(SramBase, SramSize);
    std::memset(sram, 0, SramSize);
    window_shift_ = 0;

    // The boot ROM: the header at flash offset 0, then its segments. A segment that loads into SRAM is copied
    // there; one whose load address is in the XIP window maps its data (in the flash, right after its header)
    // at that address, as bflb-mcu-tool lays out a linked XIP image.
    const std::uint8_t *flash = flash_.data();
    if (little_endian(flash) != BootMagic)
        throw std::runtime_error("no BL702 boot header ('BFNP') at flash offset 0");
    bool xip_mapped = false;
    if ((little_endian(flash + BootFlagsOffset) & NoSegmentBit) == 0) {
        const std::uint32_t count = little_endian(flash + SegmentCountOffset);
        std::uint64_t offset = BootHeaderLength;
        for (std::uint32_t i = 0; i < count; ++i) {
            if (offset + SegmentHeaderLength > config_.flash_size)
                throw std::runtime_error("boot header segment list runs past the flash");
            const std::uint32_t load = little_endian(flash + offset);
            const std::uint32_t length = little_endian(flash + offset + 4);
            const std::uint64_t data = offset + SegmentHeaderLength;
            if (data + length > config_.flash_size)
                throw std::runtime_error("boot header segment runs past the flash");
            if (load >= SramBase && load - SramBase <= SramSize && length <= SramSize - (load - SramBase)) {
                std::memcpy(sram + (load - SramBase), flash + data, length);
            } else if (load >= FlashBase && load - FlashBase < config_.flash_size && !xip_mapped) {
                window_shift_ = static_cast<std::int64_t>(data) - (load - FlashBase);
                xip_mapped = true;
            }
            offset = data + length;
        }
    }
    sync_window();
    sf_offset_reg_ = (sf_offset_reg_ & 0xff00'0000) | (static_cast<std::uint32_t>(window_shift_) & 0x00ff'ffff);
    core_.reset(little_endian(flash + BootEntryOffset));
}

riscv::RunResult Bl702::run_until(std::uint64_t target_ns) {
    const std::uint64_t target = core_.ns_to_cycles(target_ns);
    for (;;) {
        riscv::RunResult result = core_.run(target);
        if (result.reason == riscv::StopReason::Halted && rom_->call(core_.pc()))
            continue;
        return result;
    }
}

std::uint64_t Bl702::time_ns() const { return core_.cycles_to_ns(core_.cycles()); }

void Bl702::connect_radio(mcuemu::VirtualAir &air, const std::string &node) { mac_->connect(air, node); }

void Bl702::uart_tx(std::uint8_t byte) {
    if (on_uart_tx)
        on_uart_tx(byte);
}

void Bl702::sync_interrupts() {
    clic_->set_line(M154Irq, mac_->interrupt_asserted());
    std::uint32_t level = 0;
    // The core only needs to know that something is pending (it wakes from WFI); it takes the interrupt
    // itself, when the CLIC lets it through.
    core_.set_interrupt(clic_->pick(0, 0, level, false));
}

void Bl702::flash_modified() {
    sync_window();
    core_.invalidate_code();
}

void Bl702::set_xip_offset_register(std::uint32_t value) {
    sf_offset_reg_ = value;
    window_shift_ = static_cast<std::int32_t>(value << 8) >> 8; // the offset is the low 24 bits, signed
    sync_window();
}

void Bl702::sync_window() {
    const std::int64_t size = config_.flash_size;
    for (std::int64_t i = 0; i < size; ++i) {
        const std::int64_t source = i + window_shift_;
        window_[i] = source >= 0 && source < size ? flash_[source] : 0xff;
    }
}

std::uint64_t Bl702::service(std::uint64_t now) {
    std::uint64_t next = NoEvent;
    for (auto &block : blocks_)
        next = std::min(next, block->service(now));
    sync_interrupts();
    return next;
}

Block *Bl702::find_block(std::uint32_t address) const {
    for (const auto &block : blocks_)
        if (address >= block->base() && address - block->base() < block->size())
            return block.get();
    return nullptr;
}

std::uint32_t Bl702::read(std::uint32_t address, unsigned size) {
    if (Block *block = find_block(address))
        return block->read(address - block->base(), size);
    // Unclaimed registers read back what was written.
    const auto it = scratch_.find(address & ~3u);
    const std::uint32_t word = it == scratch_.end() ? 0 : it->second;
    return (word >> ((address & 3u) * 8)) & lane_mask(size);
}

void Bl702::write(std::uint32_t address, std::uint32_t value, unsigned size) {
    if (Block *block = find_block(address)) {
        block->write(address - block->base(), value, size);
        sync_interrupts();
        return;
    }
    const std::uint32_t aligned = address & ~3u;
    const unsigned shift = (address & 3u) * 8;
    const std::uint32_t mask = lane_mask(size) << shift;
    std::uint32_t &word = scratch_[aligned];
    word = (word & ~mask) | ((value << shift) & mask);
}

} // namespace bl702emu
