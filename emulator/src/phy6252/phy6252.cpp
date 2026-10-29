#include "phy6252emu/phy6252.hpp"
#include "crypto/aes128.hpp"
#include <unicorn/arm.h>
#include <unicorn/unicorn.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cstdio>
#include <limits>
#include <span>
#include <stdexcept>
#include <utility>

namespace phy6252emu {
namespace {
cortexm::Core::Config config() {
    cortexm::Core::Config c;
    c.model = cortexm::Model::M0;
    c.clock_hz = 16000000;
    c.irq_count = 32;
    c.priority_bits = 2;
    c.vtor_present = false;
    c.track_last_instruction_pc = true;
    return c;
}
constexpr std::uint32_t RomSize = 0x18000, UartBase = 0x40004000;
constexpr std::uint32_t RomNmiHandler = 0x8481;
constexpr std::uint32_t RomHardFaultHandler = 0x28f1;
constexpr std::uint32_t RomPendSvHandler = 0x84cd;
constexpr std::uint32_t TimerBase = 0x40001000;
constexpr unsigned TimerCount = 6;
constexpr std::uint32_t TimerRegisterStride = 0x14;
constexpr std::uint32_t WatchdogBase = 0x40002000;
constexpr std::uint32_t WatchdogControl = WatchdogBase;
constexpr std::uint32_t WatchdogTimeout = WatchdogBase + 0x04;
constexpr std::uint32_t WatchdogIrq = 10;
constexpr std::uint32_t GpioBase = 0x40008000;
constexpr std::uint32_t DmaBase = 0x40010000;
constexpr std::uint32_t DmaChannel3Sar = DmaBase + 0x108;
constexpr std::uint32_t DmaChannel3Dar = DmaBase + 0x110;
constexpr std::uint32_t DmaChannel3CtlHigh = DmaBase + 0x124;
constexpr std::uint32_t DmaChannel3Config = DmaBase + 0x148;
constexpr std::uint32_t DmaRawTransfer = DmaBase + 0x2c0;
constexpr std::uint32_t DmaClearTransfer = DmaBase + 0x338;
constexpr std::uint32_t DmaConfig = DmaBase + 0x398;
constexpr std::uint32_t DmaChannelEnable = DmaBase + 0x3a0;
constexpr std::uint32_t DmaChannel3Mask = 1u << 3;
constexpr std::uint32_t FlashControlBase = 0x4000c000;
constexpr std::uint32_t PcrBase = 0x40000000;
constexpr std::uint32_t SpifConfig = FlashControlBase + 0x800;
constexpr std::uint32_t SpifReadInstruction = FlashControlBase + 0x804;
constexpr std::uint32_t SpifWriteInstruction = FlashControlBase + 0x808;
constexpr std::uint32_t SpifDelay = FlashControlBase + 0x80c;
constexpr std::uint32_t SpifReadDataCapture = FlashControlBase + 0x810;
constexpr std::uint32_t SpifDeviceSize = FlashControlBase + 0x814;
constexpr std::uint32_t SpifSramPartition = FlashControlBase + 0x818;
constexpr std::uint32_t SpifRemap = FlashControlBase + 0x824;
constexpr std::uint32_t SpifModeBit = FlashControlBase + 0x828;
constexpr std::uint32_t SpifWriteCompletionControl = FlashControlBase + 0x838;
constexpr std::uint32_t SpifCommand = FlashControlBase + 0x890;
constexpr std::uint32_t SpifCommandAddress = FlashControlBase + 0x894;
constexpr std::uint32_t SpifReadData = FlashControlBase + 0x8a0;
constexpr std::uint32_t SpifReadDataHigh = FlashControlBase + 0x8a4;
constexpr std::uint32_t SpifWriteData = FlashControlBase + 0x8a8;
constexpr std::uint32_t SpifWriteDataHigh = FlashControlBase + 0x8ac;
constexpr std::uint32_t SpifPollStatus = FlashControlBase + 0x8b0;
constexpr std::uint32_t FlashSectorSize = 4096;
constexpr std::uint32_t FlashReady = 0x80000000;
constexpr std::uint32_t SpifReadStatus = 0x05900001;
constexpr std::uint32_t PcrReset = PcrBase;
constexpr std::uint32_t PcrReset1 = PcrBase + 0x04;
constexpr std::uint32_t PcrClock = PcrBase + 0x08;
constexpr std::uint32_t PcrReset2 = PcrBase + 0x0c;
constexpr std::uint32_t PcrReset3 = PcrBase + 0x10;
constexpr std::uint32_t PcrClock1 = PcrBase + 0x14;
constexpr std::uint32_t PcrApbClock = PcrBase + 0x18;
constexpr std::uint32_t PcrApbClockUpdate = PcrBase + 0x1c;
constexpr std::uint32_t PcrCacheClockGate = PcrBase + 0x20;
constexpr std::uint32_t PcrCacheReset = PcrBase + 0x24;
constexpr std::uint32_t PcrCacheBypass = PcrBase + 0x28;
constexpr std::uint32_t CacheControl0 = FlashControlBase + 0x40;
constexpr std::uint32_t CacheControl1 = FlashControlBase + 0x44;
constexpr std::uint32_t CacheRemapTable = FlashControlBase + 0x7c;
constexpr std::uint32_t CacheRemapControlBase = FlashControlBase + 0x80;
constexpr std::uint32_t MuxEnable = 0x4000380c;
constexpr std::uint32_t MuxAnalog = 0x40003800;
constexpr std::uint32_t MuxPadEnable = 0x40003814;
constexpr std::uint32_t MuxSelectBase = 0x40003818;
constexpr std::uint32_t AonPullBase = 0x4000f008;
constexpr std::uint32_t AonPullEnd = 0x4000f014;
constexpr std::uint32_t AonPmctl2_0 = 0x4000f01c;
constexpr std::uint32_t AonRtcControl = 0x4000f024;
constexpr std::uint32_t AonRtcCount = 0x4000f028;
constexpr std::uint32_t AonRtcClock = 0x4000f03c;
constexpr std::uint32_t AonRtcCompare0 = 0x4000f02c;
constexpr std::uint32_t AonRtcCompare1 = 0x4000f030;
constexpr std::uint32_t AonRtcCompare2 = 0x4000f034;
constexpr std::uint32_t AonRtcFlag = 0x4000f038;
constexpr std::uint32_t AonWakeMask0 = 0x4000f0a0;
constexpr std::uint32_t AonWakeMask1 = 0x4000f0a4;
constexpr std::uint32_t AonWakeSource0 = 0x4000f0b0;
constexpr std::uint32_t AonWakeSource1 = 0x4000f0b4;
constexpr std::uint32_t RadioClockControl = 0x4000f040;
constexpr std::uint32_t RadioClockConfig = 0x4000f044;
constexpr std::uint32_t AonEfuseConfig = 0x4000f054;
constexpr std::uint32_t RfTpCal0 = 0x1fff0a44;
constexpr std::uint32_t RfTpCal1 = 0x1fff0a45;
constexpr std::uint8_t ObservedRfTpCal0 = 0x2b;
constexpr std::uint8_t ObservedRfTpCal1 = 0x23;
constexpr std::uint32_t AonEfuseProgram0 = 0x4000f140;
constexpr std::uint32_t AonEfuseProgram1 = 0x4000f144;
constexpr std::uint32_t AonXtal16Control = 0x4000f0bc;
constexpr std::uint32_t AonSleepR0 = 0x4000f0c0;
constexpr std::uint32_t RadioRfBase = 0x40030000;
constexpr std::uint32_t RadioRfSettle = RadioRfBase + 0xbc;
constexpr std::uint32_t RadioLlBase = 0x40031000;
constexpr std::uint32_t RadioTxFifo = 0x40031400;
constexpr std::uint32_t RadioRxFifo = 0x40031c00;
constexpr std::uint32_t RadioTxFifoSize = 0x800;
constexpr std::uint32_t RadioRxFifoSize = 0x800;
constexpr std::uint32_t RadioBasebandMemoryEnd = RadioRxFifo + RadioRxFifoSize;
constexpr unsigned RadioIrq = 4;
constexpr std::uint32_t RadioIrqMd = 1u << 0;
constexpr std::uint32_t RadioIrqCok = 1u << 10;
// The PB-03F Zephyr driver reads the LL status register directly and uses a
// compact five-bit IRQ layout, while the SDK ROM API exposes the BLE LL bits.
constexpr std::uint32_t RadioIrqCokZephyr = 1u << 1;
constexpr std::uint32_t RadioIrqRto = 1u << 2;
constexpr std::uint32_t RadioIrqRtoZephyr = 1u << 3;
constexpr std::uint32_t RadioIrqMask = 0x3fffu;
constexpr std::uint32_t RadioModeStx = 0x00;
constexpr std::uint32_t RadioModeSrx = 0x01;
constexpr std::uint32_t RadioModeTrx = 0x10;
constexpr std::uint32_t RadioModeRtx = 0x12;
constexpr std::uint32_t RomLlIrqHandler = 0x4e25;
constexpr std::uint32_t RomGpioIrqHandler = 0x112d;
constexpr std::uint32_t RomTimerIrqDispatch = 0x17ff1; // Private host trap; no exported ROM symbol.
constexpr std::uint32_t JumpTableBase = BootImage::RamBase;
constexpr std::uint32_t BasebandIrqJumpSlot = 228;
constexpr std::uint32_t GpioIrqJumpSlot = 240;
constexpr std::uint32_t Timer1IrqJumpSlot = 244;
constexpr std::uint64_t RadioPollCycles = 256; // 16 us at the 16 MHz core clock
constexpr std::uint32_t GpioMask = 0x007fffff;
constexpr std::uint32_t TimerEnable = 1u << 0;
constexpr std::uint32_t TimerUserMode = 1u << 1;
constexpr std::uint32_t TimerInterruptMask = 1u << 2;
constexpr std::uint32_t TimerClockHz = 4000000; // SDK hal_timer_set_loadtimer documents a 4 MHz source

std::uint64_t timer_cycles_per_tick(std::uint32_t core_clock_hz) {
    return std::max<std::uint64_t>(1, core_clock_hz / TimerClockHz);
}

bool mux_select_register(std::uint32_t address) {
    return address >= MuxSelectBase && address <= MuxSelectBase + 5 * 4 &&
           ((address - MuxSelectBase) & 3u) == 0;
}

constexpr std::array<std::uint8_t, 23> GpioPullRegister = {
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3};
constexpr std::array<std::uint8_t, 23> GpioPullShift = {
    1, 4, 7, 10, 22, 28, 1, 4, 13, 16, 19, 22, 25, 1, 10, 13, 16, 19, 22, 4, 7, 10, 13};

bool phy_rf_register(std::uint32_t offset) {
    switch (offset) {
    case 0x00: case 0x0c: case 0x40: case 0x48: case 0x4c:
    case 0x80: case 0x90: case 0x94: case 0xb0: case 0xb4: case 0xb8: case 0xbc:
    case 0xcc: case 0xd0: case 0xd4: case 0xd8: case 0xdc: case 0xe0:
    case 0xe4: case 0xe8: case 0x104: case 0x108: case 0x10c:
        return true;
    default:
        return false;
    }
}

bool phy_ll_register(std::uint32_t offset) {
    switch (offset) {
    case 0x00: case 0x04: case 0x08: case 0x0c: case 0x10: case 0x14:
    case 0x18: case 0x1c: case 0x28: case 0x34: case 0x38: case 0x40:
        return true;
    default:
        return false;
    }
}

std::uint32_t lane_read(std::uint32_t value, std::uint32_t address, unsigned size) {
    if (size == 4)
        return value;
    const unsigned shift = (address & 3u) * 8;
    return (value >> shift) & (size == 1 ? 0xffu : 0xffffu);
}

std::uint32_t lane_write(std::uint32_t old, std::uint32_t value,
                         std::uint32_t address, unsigned size) {
    if (size == 4)
        return value;
    const unsigned shift = (address & 3u) * 8;
    const std::uint32_t mask = (size == 1 ? 0xffu : 0xffffu) << shift;
    return (old & ~mask) | ((value << shift) & mask);
}

std::uint16_t ieee802154_fcs(std::span<const std::uint8_t> payload) {
    // The radio validates the received frame and exposes its FCS in the FIFO;
    // VirtualAir intentionally carries only the MAC bytes between devices.
    std::uint16_t crc = 0;
    for (const std::uint8_t byte : payload) {
        crc ^= byte;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint16_t>((crc & 1u) ? (crc >> 1) ^ 0x8408u : crc >> 1);
    }
    return crc;
}

struct PhySecResult {
    std::vector<std::uint8_t> data;
    std::array<std::uint8_t, 4> mic{};
};

bool phy_sec_process(const crypto::Block &key, std::span<const std::uint8_t, 13> iv,
                     std::span<const std::uint8_t> input, bool encrypt,
                     PhySecResult &result) {
    if (input.size() > 0xffffu * 16u)
        return false;

    crypto::Block mac{};
    mac[0] = 0x62;
    std::copy(iv.begin(), iv.end(), mac.begin() + 1);
    const auto blocks = static_cast<std::uint16_t>(input.size() / 16);
    mac[14] = static_cast<std::uint8_t>(blocks >> 8);
    mac[15] = static_cast<std::uint8_t>(blocks);

    crypto::Block counter{};
    counter[0] = 0x02;
    std::copy(iv.begin(), iv.end(), counter.begin() + 1);
    result.data.resize(input.size());
    std::uint16_t counter_value = 0;
    for (std::size_t offset = 0; offset < input.size(); offset += 16) {
        counter_value = static_cast<std::uint16_t>(counter_value + 13);
        counter[14] = static_cast<std::uint8_t>(counter_value >> 8);
        counter[15] = static_cast<std::uint8_t>(counter_value);
        const crypto::Block stream = crypto::aes128_encrypt(key, counter);
        crypto::Block plain{};
        const std::size_t count = std::min<std::size_t>(16, input.size() - offset);
        for (std::size_t i = 0; i < count; ++i) {
            plain[i] = encrypt ? input[offset + i] : input[offset + i] ^ stream[i];
            result.data[offset + i] = encrypt ? plain[i] ^ stream[i] : plain[i];
        }
        crypto::Block next_mac = crypto::aes128_encrypt(key, mac);
        for (std::size_t i = 0; i < next_mac.size(); ++i)
            next_mac[i] ^= plain[i];
        mac = next_mac;
    }

    const crypto::Block digest = crypto::aes128_encrypt(key, mac);
    counter[14] = counter[15] = 0;
    const crypto::Block zero_stream = crypto::aes128_encrypt(key, counter);
    for (std::size_t i = 0; i < result.mic.size(); ++i)
        result.mic[i] = digest[i] ^ zero_stream[i];
    return true;
}

std::uint16_t crc16_arc(std::span<const std::uint8_t> data) {
    std::uint16_t crc = 0;
    for (const std::uint8_t byte : data) {
        crc ^= byte;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint16_t>((crc >> 1) ^ ((crc & 1) ? 0xa001 : 0));
    }
    return crc;
}

bool phy_sec_provisioned(const std::uint8_t *flash) {
    constexpr std::size_t SecureMicOffset = 0x2920;
    return !std::all_of(flash + SecureMicOffset, flash + SecureMicOffset + 16,
                        [](std::uint8_t byte) { return byte == 0xff; });
}

void phy_sec_crc_result(std::span<const std::uint8_t> input, PhySecResult &result) {
    result.data.assign(input.begin(), input.end());
    const std::uint16_t crc = crc16_arc(input);
    result.mic[0] = static_cast<std::uint8_t>(crc);
    result.mic[1] = static_cast<std::uint8_t>(crc >> 8);
    result.mic[2] = result.mic[3] = 0;
}

int timer_index(std::uint32_t aligned) {
    if (aligned < TimerBase)
        return -1;
    const std::uint32_t offset = aligned - TimerBase;
    const unsigned timer = offset / TimerRegisterStride;
    if (timer >= TimerCount || offset % TimerRegisterStride > 0x10)
        return -1;
    return static_cast<int>(timer);
}

std::uint32_t timer_base(unsigned timer) {
    return TimerBase + timer * TimerRegisterStride;
}
}

Phy6252::Phy6252() : core_(config()) {
    auto *low_rom = core_.map_memory(0, RomSize, false);
    vector_alias_ = low_rom;
    auto *high_rom = core_.map_memory(0x10000000, RomSize, false);
    // Unknown ROM entries stop at their first halfword. The low 0x100 bytes
    // are a writable exception-vector alias on this Cortex-M0 configuration.
    for (std::uint32_t i = 0; i < RomSize; i += 2) {
        low_rom[i] = 0x00;
        low_rom[i + 1] = 0xbe;
    }
    for (std::uint32_t i = 0; i < RomSize; i += 2) {
        high_rom[i] = 0x00;
        high_rom[i + 1] = 0xbe;
    }
    core_.set_write_protect_handler([](std::uint32_t address, unsigned size, std::uint32_t) {
        return address < 0x100 && static_cast<std::uint64_t>(address) + size <= 0x100;
    });
    flash_ = core_.map_memory(BootImage::FlashBase, BootImage::FlashSize, false);
    ram_ = core_.map_memory(BootImage::RamBase, BootImage::RamSize, true);
    core_.map_mmio(TimerBase, 0x1000, *this);
    core_.map_mmio(WatchdogBase, 0x1000, *this);
    core_.map_mmio(UartBase, 0x1000, *this);
    core_.map_mmio(FlashControlBase, 0x1000, *this);
    core_.map_mmio(PcrBase, 0x1000, *this);
    core_.map_mmio(0x40003000, 0x1000, *this);
    core_.map_mmio(GpioBase, 0x1000, *this);
    core_.map_mmio(DmaBase, 0x1000, *this);
    core_.map_mmio(0x4000f000, 0x1000, *this);
    core_.map_mmio(RadioRfBase, 0x1000, *this);
    core_.map_mmio(RadioLlBase, 0x1000, *this);
    // PHY62xx has a 4 KiB baseband SRAM shared by the 2 KiB TX/RX FIFOs.
    // The RFIFO starts at +0xc00, so its last 1 KiB is above the LL register page.
    core_.map_mmio(RadioLlBase + 0x1000, RadioBasebandMemoryEnd - (RadioLlBase + 0x1000), *this);
    core_.set_service_handler([this](std::uint64_t now) { return service_peripherals(now); });
}

void Phy6252::load_firmware(const std::vector<std::uint8_t> &image) {
    BootImage boot = load_boot_image(image);
    std::copy(boot.flash.begin(), boot.flash.end(), flash_);
    std::copy(boot.ram.begin(), boot.ram.end(), ram_);
    boot_ram_ = std::move(boot.ram);
    run_address_ = boot.run_address;
}

void Phy6252::load_rom_dump(const std::vector<std::uint8_t> &dump) {
    constexpr std::size_t FullDumpSize = 0x20000;
    if (run_address_ != 0)
        throw std::invalid_argument("PHY6252 ROM dump must be loaded before firmware");
    if (dump.size() != RomSize && dump.size() != FullDumpSize)
        throw std::invalid_argument("PHY6252 ROM dump must be 96 KiB or 128 KiB");
    if (dump.size() == FullDumpSize &&
        !std::all_of(dump.begin() + RomSize, dump.end(),
                     [](std::uint8_t byte) { return byte == 0xff; }))
        throw std::invalid_argument("PHY6252 128 KiB ROM dump must have an erased 32 KiB tail");
    rom_dump_.assign(dump.begin(), dump.begin() + RomSize);
    std::copy(rom_dump_.begin(), rom_dump_.end(), vector_alias_);
    std::copy(rom_dump_.begin(), rom_dump_.end(),
              core_.host_pointer(0x10000000, RomSize));
}

void Phy6252::load_efuse_dump(const std::vector<std::uint8_t> &dump) {
    constexpr std::size_t BlockSize = 8;
    constexpr std::size_t BlockCount = 4;
    if (dump.size() != BlockSize * BlockCount)
        throw std::invalid_argument("PHY6252 eFuse dump must be exactly 32 bytes");
    for (std::size_t block = 0; block < BlockCount; ++block) {
        std::copy_n(dump.begin() + block * BlockSize, BlockSize,
                    efuse_blocks_[block].begin());
        efuse_block_present_[block] = true;
    }
}

void Phy6252::reset() {
    if (!run_address_)
        throw std::runtime_error("PHY6252 firmware has not been loaded");
    if (!rom_dump_.empty()) {
        std::copy(rom_dump_.begin(), rom_dump_.end(), vector_alias_);
        std::copy(rom_dump_.begin(), rom_dump_.end(),
                  core_.host_pointer(0x10000000, RomSize));
    } else {
        std::fill_n(vector_alias_, 0x100, 0);
        const std::array<std::pair<unsigned, std::uint32_t>, 3> rom_vectors = {{
            {2, RomNmiHandler}, {3, RomHardFaultHandler}, {14, RomPendSvHandler},
        }};
        for (const auto &[index, handler] : rom_vectors)
            std::memcpy(vector_alias_ + index * sizeof(std::uint32_t), &handler, sizeof handler);
        const std::array<std::pair<unsigned, std::uint32_t>, 2> irq_vectors = {{
            {16 + 4, RomLlIrqHandler}, {16 + 16, RomGpioIrqHandler},
        }};
        for (const auto &[index, handler] : irq_vectors)
            std::memcpy(vector_alias_ + index * sizeof(std::uint32_t), &handler, sizeof handler);
        for (unsigned timer = 0; timer < TimerCount; ++timer) {
            const std::uint32_t handler = RomTimerIrqDispatch;
            const unsigned vector = 16 + 20 + timer;
            std::memcpy(vector_alias_ + vector * sizeof handler, &handler, sizeof handler);
        }
    }
    std::copy(boot_ram_.begin(), boot_ram_.end(), ram_);
    // The board ROM's RF startup calibration runs before the image's reset
    // entry. Direct image boot skips it; these two outputs are observed in
    // the supplied PHY6252 TS0001 boot log and at the SDK ROM data symbols.
    ram_[RfTpCal0 - BootImage::RamBase] = ObservedRfTpCal0;
    ram_[RfTpCal1 - BootImage::RamBase] = ObservedRfTpCal1;
    lcr_ = ier_ = dll_ = dlm_ = mcr_ = scratch_ = 0;
    rx_.clear();
    tx_.clear();
    control_registers_.clear();
    rf_registers_.clear();
    ll_registers_.clear();
    tx_fifo_.clear();
    rx_fifo_.fill(0);
    rx_fifo_read_ = 0;
    radio_tx_end_cycle_ = 0;
    radio_rx_after_ns_ = 0;
    radio_rx_deadline_cycle_ = 0;
    radio_rx_timeout_us_ = 0;
    radio_tx_active_ = false;
    radio_rx_active_ = false;
    radio_rx_rssi_ = -100;
    timers_ = {};
    for (TimerState &timer : timers_)
        timer.last_cycle = core_.cycles();
    // The SDK's WaitUs() polls AP_TIM3->CurrentCount continuously. ROM board
    // init enables this down-counter before handing control to the image, so
    // seed the state direct boot skips; otherwise WaitUs() never returns.
    TimerState &wait_timer = timers_[2];
    wait_timer.load = std::numeric_limits<std::uint32_t>::max();
    wait_timer.current = wait_timer.load;
    wait_timer.control = TimerEnable | TimerInterruptMask;
    gpio_data_ = gpio_direction_ = gpio_interrupt_enable_ = 0;
    gpio_interrupt_mask_ = GpioMask;
    gpio_interrupt_type_ = gpio_interrupt_polarity_ = 0;
    gpio_interrupt_status_ = gpio_input_ = 0;
    spif_config_ = FlashReady;
    spif_command_ = 0;
    spif_read_data_ = 0;
    dma_raw_transfer_status_ = dma_channel_enable_ = 0;
    watchdog_deadline_ns_ = 0;
    watchdog_interrupt_pending_ = false;
    sleep_allowed_ = false;
    sleep_mode_ = 0;
    unexported_init_hook_warned_ = false;
    pendsv_dispatch_warning_logged_ = false;
    ll_rand_warning_logged_ = false;
    baseband_irq_dispatch_warning_logged_ = false;
    gpio_irq_dispatch_warning_logged_ = false;
    timer_irq_dispatch_warning_logged_.fill(false);
    ll_context_warning_logged_ = false;
    rf_sample_e8_warning_logged_ = false;
    heap_start_ = heap_size_ = 0;
    random_state_ = 0x6d2b79f5u;
    free_heap_blocks_.clear();
    heap_allocations_.clear();
    pending_fault_.clear();
    core_.set_clock_hz(16000000);
    core_.reset(run_address_);
    core_.set_vtor(0);
    rtc_epoch_ns_ = core_.cycles_to_ns(core_.cycles());
    rtc_epoch_count_ = 0;
    // The SDK ROM board init starts the always-on RTC before handing control
    // to this image. Direct image boot skips that path; retain the observed
    // running state required by the early RF startup wait loop.
    control_registers_[AonRtcControl] = 1;
    update_radio_irq();
    update_gpio_irq();
    core_.set_irq_line(WatchdogIrq, false);
    for (unsigned timer = 0; timer < timers_.size(); ++timer)
        update_timer_irq(timer);
    notify_gpio(GpioMask);
}

cortexm::RunResult Phy6252::run(std::uint64_t until_cycle) {
    for (;;) {
        auto result = core_.run(until_cycle);
        const auto pc = core_.pc();
        if (result.reason == cortexm::StopReason::Reset && !pending_fault_.empty()) {
            std::uint32_t vtor = 0;
            std::array<std::uint32_t, 16> vectors{};
            const bool have_vector_table = core_.read_memory(0xe000ed08, &vtor, sizeof vtor) &&
                vtor >= BootImage::RamBase &&
                static_cast<std::uint64_t>(vtor) + sizeof vectors <=
                    static_cast<std::uint64_t>(BootImage::RamBase) + BootImage::RamSize &&
                core_.read_memory(vtor, vectors.data(), sizeof vectors);
            if (have_vector_table) {
                char context[192];
                std::snprintf(context, sizeof context,
                              " VTOR=0x%08x initial_sp=0x%08x hardfault=0x%08x pendsv=0x%08x",
                              vtor, vectors[0], vectors[3], vectors[14]);
                pending_fault_ += context;
            } else if (core_.read_memory(0xe000ed08, &vtor, sizeof vtor)) {
                char context[40];
                std::snprintf(context, sizeof context, " VTOR=0x%08x", vtor);
                pending_fault_ += context;
            }
            if (const std::uint8_t *handler = core_.host_pointer(pc & ~1u, 4)) {
                char context[48];
                std::snprintf(context, sizeof context, " handler_bytes=%02x%02x%02x%02x",
                              handler[0], handler[1], handler[2], handler[3]);
                pending_fault_ += context;
            }
            result.reason = cortexm::StopReason::Fault;
            result.message = std::move(pending_fault_);
            pending_fault_.clear();
            return result;
        }
        if (result.reason != cortexm::StopReason::Halted ||
            !((pc < RomSize) || (pc >= 0x10000000 && pc < 0x10000000 + RomSize))) {
            if (result.reason == cortexm::StopReason::Fault) {
                char context[128];
                std::snprintf(context, sizeof context,
                              " (r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x sp=0x%08x lr=0x%08x)",
                              core_.reg(0), core_.reg(1), core_.reg(2), core_.reg(3),
                              core_.reg(13), core_.reg(14));
                result.message += context;
            }
            return result;
        }
        const std::uint32_t address = pc >= 0x10000000 ? pc - 0x10000000 : pc;
        bool rom_branch = false;
        if (address == 0xa5ec) {
            constexpr std::array<std::uint32_t, 8> SystemClockHz = {
                32000000, 32000000, 16000000, 48000000,
                64000000, 96000000, 8000000, 4000000};
            const std::uint32_t selection = core_.reg(0);
            if (selection < SystemClockHz.size()) {
                const std::uint64_t old_cycles_per_tick = timer_cycles_per_tick(core_.clock_hz());
                const std::uint64_t new_cycles_per_tick = timer_cycles_per_tick(SystemClockHz[selection]);
                for (unsigned timer = 0; timer < timers_.size(); ++timer) {
                    sync_timer(timer, core_.cycles());
                    TimerState &state = timers_[timer];
                    state.cycle_remainder = state.cycle_remainder * new_cycles_per_tick /
                        old_cycles_per_tick;
                }
                core_.set_clock_hz(SystemClockHz[selection]);
                core_.set_reg(0, 0);
            } else {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            }
        } else if (address == 0xa5d0) {
            core_.set_reg(0, core_.clock_hz());
        } else if (address == 0xa9c8) {
            // SDK drv_irq_init() leaves the core able to accept interrupts.
            core_.set_interrupts_masked(false);
        } else if (address == 0xa974) {
            const bool was_masked = core_.interrupts_masked();
            core_.set_interrupts_masked(true);
            core_.set_reg(0, was_masked ? 1u : 0u);
        } else if (address == 0xa99c) {
            core_.set_interrupts_masked(false);
            core_.set_reg(0, 0);
        } else if (address == 0x0e08 || address == 0x0e34) {
            // SDK ROM symbols __aeabi_uidiv[mod] and __aeabi_idiv[mod] share
            // these entries. EABI returns quotient in r0 and remainder in r1.
            const std::uint32_t numerator = core_.reg(0);
            const std::uint32_t denominator = core_.reg(1);
            if (denominator != 0) {
                if (address == 0x0e08) {
                    core_.set_reg(0, numerator / denominator);
                    core_.set_reg(1, numerator % denominator);
                } else {
                    const std::int32_t dividend = static_cast<std::int32_t>(numerator);
                    const std::int32_t divisor = static_cast<std::int32_t>(denominator);
                    if (dividend == std::numeric_limits<std::int32_t>::min() && divisor == -1) {
                        core_.set_reg(0, numerator);
                        core_.set_reg(1, 0);
                    } else {
                        core_.set_reg(0, static_cast<std::uint32_t>(dividend / divisor));
                        core_.set_reg(1, static_cast<std::uint32_t>(dividend % divisor));
                    }
                }
            } else {
                char message[256];
                std::snprintf(message, sizeof message,
                              "PHY6252 divide helper at 0x%08x called with zero divisor (r0=0x%08x lr=0x%08x)",
                              pc, numerator, core_.reg(14));
                return {cortexm::StopReason::Fault, message};
            }
        } else if (address == 0x0f74) {
            // ARM runtime __aeabi_uread4 performs an unaligned little-endian
            // 32-bit load. The M0 core uses this helper for such accesses.
            std::array<std::uint8_t, 4> bytes{};
            if (!core_.read_memory(core_.reg(0), bytes.data(), bytes.size()))
                return {cortexm::StopReason::Fault,
                        "invalid pointer passed to PHY6252 __aeabi_uread4"};
            core_.set_reg(0, static_cast<std::uint32_t>(bytes[0]) |
                             (static_cast<std::uint32_t>(bytes[1]) << 8) |
                             (static_cast<std::uint32_t>(bytes[2]) << 16) |
                             (static_cast<std::uint32_t>(bytes[3]) << 24));
        } else if (address == 0x16b14) {
            // SDK ROM rtc_get_counter() exposes the AON 32.768 kHz counter.
            core_.set_reg(0, rtc_count(core_.cycles_to_ns(core_.cycles())));
        } else if (address == 0x16c2c) {
            // SDK declares set_timer(AP_TIM_TypeDef *, int). The exported body
            // is absent; match the load/control sequence in timer.c's
            // hal_timer_set_loadtimer() for the observed Timer1 call path.
            const std::uint32_t base = core_.reg(0);
            const std::int32_t time = static_cast<std::int32_t>(core_.reg(1));
            const int timer = timer_index(base);
            if (timer < 0 || base != timer_base(static_cast<unsigned>(timer)) ||
                time > std::numeric_limits<std::int32_t>::max() / 4) {
                char message[160];
                std::snprintf(message, sizeof message,
                              "invalid PHY6252 set_timer(base=0x%08x, time=%d)", base, time);
                return {cortexm::StopReason::Fault, message};
            }
            write_register(base + 0x08, 0, 4);
            if (time > 0) {
                write_register(base + 0x08, TimerUserMode, 4);
                write_register(base, static_cast<std::uint32_t>(time) * 4u, 4);
                write_register(base + 0x08, TimerEnable | TimerUserMode, 4);
            }
        } else if (address == 0x10adc) {
            // ll_hw_clr_irq() writes the SDK's 14-bit IRQ mask to the W1C
            // status register (same sequence as phy_hw_go() in PHY+).
            write_register(RadioLlBase + 0x14, 0x3fffu, 4);
        } else if (address == 0x10cf8 || address == 0x11420) {
            // Both exported entries start the mode configured in LL_HW+0x04.
            write_register(RadioLlBase, 1, 4);
        } else if (address == 0x11174) {
            // ll_hw_set_irq(mask) maps to LL_HW+0x0c, as the SDK's PHY+
            // implementation documents with its direct register setup.
            write_register(RadioLlBase + 0x0c, core_.reg(0), 4);
        } else if (address == 0x10f50) {
            // ll_hw_read_rfifo_zb(rxPkt, pktLen, pktFoot0, pktFoot1) copies
            // the FIFO's length byte followed by the PSDU and its FCS.
            const std::uint32_t packet = core_.reg(0);
            const std::uint32_t length_out = core_.reg(1);
            const std::uint32_t foot0_out = core_.reg(2);
            const std::uint32_t foot1_out = core_.reg(3);
            const std::uint16_t packet_length = rx_fifo_[0];
            const std::uint32_t foot0 =
                (static_cast<std::uint32_t>(static_cast<std::uint8_t>(radio_rx_rssi_)) << 24) | 2u;
            constexpr std::uint32_t foot1 = 0;
            const std::size_t fifo_bytes = static_cast<std::size_t>(packet_length) + 1;
            const bool valid = packet_length >= 2 && packet_length <= 127 &&
                packet != 0 && length_out != 0 && foot0_out != 0 && foot1_out != 0 &&
                core_.write_memory(packet, rx_fifo_.data(), fifo_bytes) &&
                core_.write_memory(length_out, &packet_length, sizeof packet_length) &&
                core_.write_memory(foot0_out, &foot0, sizeof foot0) &&
                core_.write_memory(foot1_out, &foot1, sizeof foot1);
            if (valid) {
                rx_fifo_read_ = fifo_bytes;
                core_.set_reg(0, 0); // LL_STATUS_SUCCESS
            } else {
                core_.set_reg(0, 1);
            }
        } else if (address == 0x10bbc) {
            // ll_hw_get_irq_status() returns LL_HW+0x10, whose bit layout is
            // listed in ll_hw_drv.h (MD=0, COK=10, RTO=2, and so on).
            core_.set_reg(0, ll_registers_[RadioLlBase + 0x10]);
        } else if (address == 0x11264 || address == 0x112a4 ||
                   address == 0x112b8 || address == 0x113c8) {
            // These generic PHY helpers select the modes declared in
            // ll_hw_drv.h. PHY+ uses them for Zigbee as well as BLE.
            const std::uint32_t mode = address == 0x11264 ? RadioModeRtx :
                address == 0x112a4 ? RadioModeSrx :
                address == 0x112b8 ? RadioModeStx : RadioModeTrx;
            ll_registers_[RadioLlBase + 0x04] = mode;
        } else if (address == 0x110b0) {
            rx_fifo_.fill(0);
            rx_fifo_read_ = 0;
        } else if (address == 0x110e8) {
            tx_fifo_.clear();
        } else if (address == 0x11278 || address == 0x11284) {
            // The PHY+ SDK wrapper names this parameter rxTimeOutUs. The
            // timeout is applied to an active receive and reused by the next.
            radio_rx_timeout_us_ = core_.reg(0);
            if (radio_rx_active_)
                schedule_radio_rx_timeout();
        } else if (address == 0x113dc) {
            // The related PHY62xx SDK driver packs these values into BB_HW+0xbc.
            // Preserve the register contract; PHY6252 timing remains unverified.
            const std::uint32_t packed = ((core_.reg(0) & 0xffu) << 16) |
                ((core_.reg(1) & 0xffu) << 8) | (core_.reg(2) & 0xffu);
            write_register(RadioRfSettle, packed, 4);
        } else if (address == 0x10cdc) {
            // Return the current LL_HW operation mode; the symbol is exported
            // by the SDK ROM map but its declaration is omitted from headers.
            core_.set_reg(0, ll_registers_[RadioLlBase + 0x04]);
        } else if (address == 0x15cc8) {
            // timer.h wraps fine-time deltas at BASE_TIME_UNITS. Core cycles
            // are the only exposed 16 MHz clock in this model.
            core_.set_reg(0, static_cast<std::uint32_t>(core_.cycles()) & 0x3fffffu);
        } else if (address == 0xace0) {
            const std::uint32_t block = core_.reg(0);
            const std::uint32_t destination = core_.reg(1);
            // The PHY6222 SDK declares this same PHY62xx API as
            // efuse_read(block, uint32_t *buf): two words per block. Its error
            // enum defines INVALID_PARAM=6 and UNINITIALIZED=24. The PHY6252
            // SDK exports the ROM symbol but omits its declaration.
            if (block >= efuse_blocks_.size()) {
                core_.set_reg(0, 6); // PPlus_ERR_INVALID_PARAM
            } else if (!efuse_block_present_[block]) {
                if (!efuse_missing_warning_logged_[block]) {
                    std::fprintf(stderr,
                                 "PHY6252: efuse_read block %u has no supplied device data; returning PPlus_ERR_UNINITIALIZED (24)\n",
                                 block);
                    efuse_missing_warning_logged_[block] = true;
                }
                core_.set_reg(0, 24); // PPlus_ERR_UNINITIALIZED
            } else if (destination < BootImage::RamBase ||
                       static_cast<std::uint64_t>(destination) + efuse_blocks_[block].size() >
                           static_cast<std::uint64_t>(BootImage::RamBase) + BootImage::RamSize ||
                       !core_.write_memory(destination, efuse_blocks_[block].data(),
                                           efuse_blocks_[block].size())) {
                core_.set_reg(0, 6); // PPlus_ERR_INVALID_PARAM
            } else {
                core_.set_reg(0, 0); // PPlus_SUCCESS
            }
        } else if (address == 0x3fdc) {
            // SDK ROM symbol LL_ENC_AES128_Encrypt0(input, key, output).
            crypto::Block input{}, key{};
            crypto::Block output{};
            if (!core_.read_memory(core_.reg(0), input.data(), input.size()) ||
                !core_.read_memory(core_.reg(1), key.data(), key.size())) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                output = crypto::aes128_encrypt(key, input);
                if (!core_.write_memory(core_.reg(2), output.data(), output.size()))
                    core_.set_reg(0, static_cast<std::uint32_t>(-22));
                else
                    core_.set_reg(0, 0);
            }
        } else if (address == 0x14ccc) {
            // SDK ROM osal_memcmp(left, right, size), with standard C semantics.
            const std::uint32_t size = core_.reg(2);
            if (size > BootImage::RamSize + BootImage::FlashSize) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                std::vector<std::uint8_t> left(size), right(size);
                if (size != 0 &&
                    (!core_.read_memory(core_.reg(0), left.data(), size) ||
                     !core_.read_memory(core_.reg(1), right.data(), size))) {
                    core_.set_reg(0, static_cast<std::uint32_t>(-22));
                } else {
                    int comparison = 0;
                    for (std::size_t i = 0; i < size; ++i) {
                        if (left[i] != right[i]) {
                            comparison = static_cast<int>(left[i]) - right[i];
                            break;
                        }
                    }
                    core_.set_reg(0, static_cast<std::uint32_t>(comparison));
                }
            }
        } else if (address == 0x14ce8 || address == 0x0e80) {
            // SDK osal_memcpy and ARM runtime __aeabi_memcpy share this ABI.
            const std::uint32_t size = core_.reg(2);
            if (size > BootImage::RamSize + BootImage::FlashSize) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                std::vector<std::uint8_t> data(size);
                if (size != 0 &&
                    (!core_.read_memory(core_.reg(1), data.data(), size) ||
                     !core_.write_memory(core_.reg(0), data.data(), size)))
                    core_.set_reg(0, static_cast<std::uint32_t>(-22));
            }
        } else if (address == 0x0ea4 || address == 0x0eb2 || address == 0x0eb6 ||
                   address == 0x14d14) {
            // ARM runtime memset/memclr use (dst, size, value); C/OSAL memset
            // use (dst, value, size).
            const bool aeabi_memset = address == 0x0ea4;
            const bool clear = address == 0x0eb2;
            const bool c_memset = address == 0x0eb6 || address == 0x14d14;
            const std::uint32_t destination = core_.reg(0);
            const std::uint32_t size = aeabi_memset || clear ? core_.reg(1) : core_.reg(2);
            const std::uint8_t fill = clear ? 0 : static_cast<std::uint8_t>(
                aeabi_memset ? core_.reg(2) : core_.reg(1));
            if (size > BootImage::RamSize + BootImage::FlashSize) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                std::vector<std::uint8_t> data(size, fill);
                if (size != 0 && !core_.write_memory(destination, data.data(), size))
                    core_.set_reg(0, static_cast<std::uint32_t>(-22));
                else
                    core_.set_reg(0, c_memset ? destination : 0);
            }
        } else if (address == 0x0ec8) {
            // ARM runtime strlen(pointer).
            const std::uint32_t source = core_.reg(0);
            std::uint32_t length = 0;
            for (; length < BootImage::RamSize + BootImage::FlashSize; ++length) {
                std::uint8_t byte = 0;
                if (!core_.read_memory(source + length, &byte, 1) || byte == 0)
                    break;
            }
            core_.set_reg(0, length);
        } else if (address == 0x0ed6 || address == 0x0ef2 || address == 0x0f0c) {
            // Standard strcmp/memcmp/strncmp, using unsigned-byte ordering.
            const std::uint32_t limit = address == 0x0ef2 ? core_.reg(2) :
                                       address == 0x0f0c ? core_.reg(2) :
                                       BootImage::RamSize + BootImage::FlashSize;
            if (limit > BootImage::RamSize + BootImage::FlashSize) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                int comparison = 0;
                for (std::uint32_t i = 0; i < limit; ++i) {
                    std::uint8_t left = 0, right = 0;
                    if (!core_.read_memory(core_.reg(0) + i, &left, 1) ||
                        !core_.read_memory(core_.reg(1) + i, &right, 1)) {
                        comparison = -22;
                        break;
                    }
                    if (left != right) {
                        comparison = static_cast<int>(left) - right;
                        break;
                    }
                    if (left == 0 && address != 0x0ef2)
                        break;
                }
                core_.set_reg(0, static_cast<std::uint32_t>(comparison));
            }
        } else if (address == 0x14cb4) {
            // SDK ROM osal_mem_set_heap(start, size).
            const std::uint32_t start = core_.reg(0);
            const std::uint32_t size = core_.reg(1);
            if (start < BootImage::RamBase ||
                static_cast<std::uint64_t>(start) + size > BootImage::RamBase + BootImage::RamSize) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                heap_start_ = start;
                heap_size_ = size;
                free_heap_blocks_.clear();
                heap_allocations_.clear();
                if (size != 0)
                    free_heap_blocks_.emplace(start, size);
                core_.set_reg(0, 0);
            }
        } else if (address == 0x14c24) {
            // osal_mem_init resets the free list within the configured heap.
            free_heap_blocks_.clear();
            heap_allocations_.clear();
            if (heap_size_ != 0)
                free_heap_blocks_.emplace(heap_start_, heap_size_);
            core_.set_reg(0, 0);
        } else if (address == 0x14b3c) {
            // osal_mem_alloc(size) returns a pointer into the firmware's heap.
            core_.set_reg(0, heap_allocate(core_.reg(0)));
        } else if (address == 0x14c00) {
            // osal_mem_free(pointer).
            heap_release(core_.reg(0));
            core_.set_reg(0, 0);
        } else if (address == 0x14c8c) {
            // osal_mem_kick only retries deferred allocation work in the SDK.
            core_.set_reg(0, 0);
        } else if (address == 0x14cf8) {
            // osal_memdup(source, size).
            const std::uint32_t source = core_.reg(0);
            const std::uint32_t size = core_.reg(1);
            if (size > BootImage::RamSize + BootImage::FlashSize) {
                core_.set_reg(0, 0);
            } else {
                std::vector<std::uint8_t> data(size);
                if (size != 0 && !core_.read_memory(source, data.data(), size)) {
                    core_.set_reg(0, 0);
                } else {
                    const std::uint32_t destination = heap_allocate(size);
                    if (destination != 0 && size != 0 &&
                        !core_.write_memory(destination, data.data(), size)) {
                        heap_release(destination);
                        core_.set_reg(0, 0);
                    } else {
                        core_.set_reg(0, destination);
                    }
                }
            }
        } else if (address == 0x5044) {
            // SDK LL_InitConnectContext is a BLE connection initializer. The
            // TS0001 app calls it before using its separate Zigbee MAC path;
            // the supplied model does not implement BLE connection state.
            std::uint8_t byte = 0;
            if (!core_.read_memory(core_.reg(0), &byte, 1) ||
                !core_.read_memory(core_.reg(1), &byte, 1)) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                if (!ll_context_warning_logged_) {
                    std::fprintf(stderr,
                                 "PHY6252: LL_InitConnectContext reached; BLE connection state is not modeled\n");
                    ll_context_warning_logged_ = true;
                }
                core_.set_reg(0, 0); // LL_STATUS_SUCCESS for unused BLE context setup.
            }
        } else if (address == 0x5e04) {
            // SDK LL_Rand(buffer, length) accepts lengths from 1 to 255 and
            // returns an LL status. Use a deterministic PRNG for repeatable
            // simulation; this is not a cryptographic random source.
            const std::uint32_t destination = core_.reg(0);
            const std::uint32_t length = core_.reg(1);
            if (length == 0 || length > 255) {
                core_.set_reg(0, 0x12); // LL_STATUS_ERROR_BAD_PARAMETER
            } else {
                std::vector<std::uint8_t> bytes(length);
                for (std::uint8_t &byte : bytes) {
                    random_state_ ^= random_state_ << 13;
                    random_state_ ^= random_state_ >> 17;
                    random_state_ ^= random_state_ << 5;
                    byte = static_cast<std::uint8_t>(random_state_ >> 24);
                }
                if (!core_.write_memory(destination, bytes.data(), bytes.size())) {
                    core_.set_reg(0, 0x12);
                } else {
                    if (!ll_rand_warning_logged_) {
                        std::fprintf(stderr,
                                     "PHY6252: LL_Rand uses deterministic non-cryptographic bytes\n");
                        ll_rand_warning_logged_ = true;
                    }
                    core_.set_reg(0, 0); // LL_STATUS_SUCCESS
                }
            }
        } else if (address == 0xa5c0) {
            // The SDK ROM exports clear_timer(AP_TIM_TypeDef *). Its body is
            // absent; match the public hal_timer_stop_counter() sequence.
            const std::uint32_t base = core_.reg(0);
            const int timer = timer_index(base);
            if (timer < 0 || base != timer_base(static_cast<unsigned>(timer))) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                write_register(base + 0x08, 0, 4);
                write_register(base, 0, 4);
                write_register(base + 0x04, 0, 4);
                core_.set_reg(0, 0);
            }
        } else if (address == 0x13ae4) {
            rom_log_printf();
        } else if (address == 0xa2e0) {
            // This image calls an unexported entry inside the SDK's
            // ble_crc24_gen symbol range as a zero-argument init hook. Its ROM
            // body is not in the SDK package, so this visible shim only returns.
            if (!unexported_init_hook_warned_) {
                std::fprintf(stderr,
                             "PHY6252: unverified ROM entry 0xa2e0 (inside ble_crc24_gen) treated as return-only\n");
                unexported_init_hook_warned_ = true;
            }
            core_.set_reg(0, 0);
        } else if (address == (RomPendSvHandler & ~1u)) {
            // The ROM body is unavailable, but this image installs FreeRTOS
            // handlers in the observed exception callback slots at 0x1fff0374.
            // Dispatch PendSV to its image-resident handler and expose its
            // registered SysTick handler through the fixed vector alias.
            std::array<std::uint32_t, 3> handlers{};
            if (!core_.read_memory(0x1fff0374, handlers.data(), sizeof handlers) ||
                (handlers[1] & 1u) == 0 || (handlers[2] & 1u) == 0 ||
                (handlers[1] & ~1u) < BootImage::RamBase ||
                (handlers[1] & ~1u) >= BootImage::RamBase + BootImage::RamSize ||
                (handlers[2] & ~1u) < BootImage::RamBase ||
                (handlers[2] & ~1u) >= BootImage::RamBase + BootImage::RamSize) {
                return {cortexm::StopReason::Fault,
                        "PHY6252 PendSV_Handler has no valid image callback"};
            }
            if (!pendsv_dispatch_warning_logged_) {
                std::fprintf(stderr,
                             "PHY6252: ROM PendSV body unavailable; dispatching registered image handlers PendSV=0x%08x SysTick=0x%08x (inferred callback slots)\n",
                             handlers[1], handlers[2]);
                pendsv_dispatch_warning_logged_ = true;
            }
            std::memcpy(vector_alias_ + 15 * sizeof(std::uint32_t),
                        &handlers[2], sizeof handlers[2]);
            core_.set_reg(15, handlers[1] & ~1u);
            rom_branch = true;
        } else if (address == (RomLlIrqHandler & ~1u) ||
                   address == (RomGpioIrqHandler & ~1u)) {
            const bool gpio_irq = address == (RomGpioIrqHandler & ~1u);
            const std::uint32_t slot = gpio_irq ? GpioIrqJumpSlot : BasebandIrqJumpSlot;
            const std::uint32_t jump_address = JumpTableBase + slot * sizeof(std::uint32_t);
            std::uint32_t handler = 0;
            const bool have_handler = core_.read_memory(jump_address, &handler, sizeof handler);
            const std::uint32_t handler_address = handler & ~1u;
            const bool handler_in_ram = handler_address >= BootImage::RamBase &&
                handler_address < BootImage::RamBase + BootImage::RamSize;
            const bool handler_in_flash = handler_address >= BootImage::FlashBase &&
                handler_address < BootImage::FlashBase + BootImage::FlashSize;
            if (!have_handler || (handler & 1u) == 0 ||
                (!handler_in_ram && !handler_in_flash)) {
                char message[160];
                std::snprintf(message, sizeof message,
                              "PHY6252 ROM %s IRQ vector has no valid JUMP_FUNCTION(%u) callback (0x%08x)",
                              gpio_irq ? "GPIO" : "baseband", slot, handler);
                return {cortexm::StopReason::Fault, message};
            }
            bool &logged = gpio_irq ? gpio_irq_dispatch_warning_logged_ :
                                      baseband_irq_dispatch_warning_logged_;
            if (!logged) {
                std::fprintf(stderr,
                             "PHY6252: ROM %s IRQ dispatches JUMP_FUNCTION(%u)=0x%08x\n",
                             gpio_irq ? "GPIO" : "baseband", slot, handler);
                logged = true;
            }
            core_.set_reg(15, handler & ~1u);
            rom_branch = true;
        } else if (address == (RomTimerIrqDispatch & ~1u)) {
            std::uint32_t ipsr = 0;
            uc_reg_read(core_.engine(), UC_ARM_REG_IPSR, &ipsr);
            const unsigned irq = ipsr >= 16 ? ipsr - 16 : 0;
            if (irq < 20 || irq >= 20 + TimerCount)
                return {cortexm::StopReason::Fault,
                        "PHY6252 internal timer IRQ dispatch reached outside IRQ20..IRQ25"};
            const unsigned timer = irq - 20;
            const std::uint32_t slot = Timer1IrqJumpSlot + timer;
            const std::uint32_t jump_address = JumpTableBase + slot * sizeof(std::uint32_t);
            std::uint32_t handler = 0;
            const bool have_handler = core_.read_memory(jump_address, &handler, sizeof handler);
            const std::uint32_t handler_address = handler & ~1u;
            const bool handler_in_ram = handler_address >= BootImage::RamBase &&
                handler_address < BootImage::RamBase + BootImage::RamSize;
            const bool handler_in_flash = handler_address >= BootImage::FlashBase &&
                handler_address < BootImage::FlashBase + BootImage::FlashSize;
            if (!have_handler || (handler & 1u) == 0 ||
                (!handler_in_ram && !handler_in_flash)) {
                char message[160];
                std::snprintf(message, sizeof message,
                              "PHY6252 Timer%u IRQ has no valid JUMP_FUNCTION(%u) callback (0x%08x)",
                              timer + 1, slot, handler);
                return {cortexm::StopReason::Fault, message};
            }
            bool &logged = timer_irq_dispatch_warning_logged_[timer];
            if (!logged) {
                std::fprintf(stderr,
                             "PHY6252: Timer%u IRQ dispatches JUMP_FUNCTION(%u)=0x%08x (inferred ROM vector)\n",
                             timer + 1, slot, handler);
                logged = true;
            }
            core_.set_reg(15, handler_address);
            rom_branch = true;
        } else if (address == 0xa920) {
            sleep_allowed_ = false; // SDK disableSleep() only changes power-manager policy.
        } else if (address == 0xaeac) {
            sleep_allowed_ = true; // SDK enableSleep() allows later power-conserve entry.
        } else if (address == 0x16b44) {
            sleep_mode_ = core_.reg(0); // SDK setSleepMode(enum Sleep_Mode) stores the policy.
        } else if (address == 0x16d48) {
            // SDK ROM spif_cmd(op, addrlen, rdlen, wrlen, mode_bits, dummy).
            std::uint32_t mode_bits = 0, dummy_cycles = 0;
            const std::uint32_t stack = core_.reg(13);
            if (!core_.read_memory(stack, &mode_bits, sizeof mode_bits) ||
                !core_.read_memory(stack + 4, &dummy_cycles, sizeof dummy_cycles)) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                const std::uint32_t op = core_.reg(0) & 0xffu;
                const std::uint32_t address_length = core_.reg(1) & 0xffu;
                const std::uint32_t read_length = core_.reg(2) & 0xffu;
                const std::uint32_t write_length = core_.reg(3) & 0xffu;
                spif_command_ = op | (address_length << 8) |
                    (read_length << 16) | (write_length << 24);
                control_registers_[SpifModeBit] = mode_bits & 0xffu;
                control_registers_[SpifDelay] = dummy_cycles & 0xffu;
                // Flash status commands return ready with WIP clear in this model.
                if (op == 0x05u)
                    spif_read_data_ = 0;
                core_.set_reg(0, 0);
            }
        } else if (address == 0x1713c) {
            // SDK ROM spif_rddata(destination, length). The model completes
            // controller commands synchronously and currently supplies the
            // ready/idle status bytes for the status-register command.
            const std::uint32_t destination = core_.reg(0);
            const std::uint32_t length = core_.reg(1);
            if (length > 8) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                std::array<std::uint8_t, 8> data{};
                const std::uint32_t words[] = {
                    spif_read_data_, control_registers_[SpifReadDataHigh]};
                for (std::size_t i = 0; i < length; ++i)
                    data[i] = static_cast<std::uint8_t>(words[i / 4] >> ((i % 4) * 8));
                if (length != 0 && !core_.write_memory(destination, data.data(), length))
                    core_.set_reg(0, static_cast<std::uint32_t>(-22));
                else
                    core_.set_reg(0, 0);
            }
        } else if (address == 0x8960) {
            // ARM Compiler's __ARM_common_switch8 uses an inline byte table
            // immediately after its caller's BL. LR-1 stores the last index;
            // each table byte is a halfword offset from LR to a case body.
            const std::uint32_t lr = core_.reg(14);
            std::uint8_t last_index = 0;
            if (lr == 0 || !core_.read_memory(lr - 1, &last_index, 1))
                return {cortexm::StopReason::Fault,
                        "invalid inline table for PHY6252 __ARM_common_switch8"};
            const std::uint32_t selector = core_.reg(3);
            const std::uint32_t index = std::min<std::uint32_t>(selector, last_index);
            std::uint8_t offset = 0;
            if (!core_.read_memory(lr + index, &offset, 1))
                return {cortexm::StopReason::Fault,
                        "unreadable inline table for PHY6252 __ARM_common_switch8"};
            core_.set_reg(15, (lr + static_cast<std::uint32_t>(offset) * 2) & ~1u);
            rom_branch = true;
        } else if (address == 0x16dc4) {
            // SDK ROM spif_config(ref_clk, div, rd_instr, mode_bit, QE).
            // The flash remains memory-mapped in this model; preserve the
            // documented controller configuration fields used by the caller.
            std::uint32_t qe = 0;
            const std::uint32_t ref_clock = core_.reg(0);
            const std::uint32_t divider = core_.reg(1);
            if (core_.read_memory(core_.reg(13), &qe, sizeof qe) &&
                ref_clock < 8 && divider != 0) {
                control_registers_[SpifReadInstruction] = core_.reg(2);
                control_registers_[SpifModeBit] = core_.reg(3) & 0xffu;
                control_registers_[SpifConfig] = FlashReady | (divider & 0xffu) |
                    ((ref_clock & 7u) << 8) | ((qe & 1u) << 16);
                spif_config_ = control_registers_[SpifConfig];
                core_.set_reg(0, 0);
            } else {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            }
        } else if (address == 0x17394 || address == 0x1744c) {
            // SDK ROM spif_write()/spif_write_dma() share the same destination,
            // RAM source and byte-count contract.
            rom_flash_write();
        } else if (address == 0x16fa8) {
            rom_flash_erase();
        } else if (address == 0x1531c) {
            std::uint32_t stack_arg0 = 0, stack_arg1 = 0;
            const bool args_read = core_.read_memory(core_.reg(13), &stack_arg0, sizeof stack_arg0) &&
                core_.read_memory(core_.reg(13) + 4, &stack_arg1, sizeof stack_arg1);
            const std::uint32_t source = core_.reg(2), length = core_.reg(3);
            const std::uint64_t source_end = static_cast<std::uint64_t>(source) + length;
            const std::uint64_t output_end = static_cast<std::uint64_t>(stack_arg1) + length;
            const bool source_valid = length <= BootImage::FlashSize &&
                ((source >= BootImage::FlashBase &&
                  source_end <= static_cast<std::uint64_t>(BootImage::FlashBase) + BootImage::FlashSize) ||
                 (source >= BootImage::RamBase &&
                  source_end <= static_cast<std::uint64_t>(BootImage::RamBase) + BootImage::RamSize));
            const bool output_valid = stack_arg1 == 0 ||
                (stack_arg1 >= BootImage::RamBase &&
                 output_end <= static_cast<std::uint64_t>(BootImage::RamBase) + BootImage::RamSize);
            std::array<std::uint8_t, 4> stored_mic{};
            std::vector<std::uint8_t> data(length);
            PhySecResult transformed;
            const bool inputs_read = args_read && source_valid && output_valid &&
                core_.read_memory(source, data.data(), data.size()) &&
                (stack_arg0 == 0 || core_.read_memory(stack_arg0, stored_mic.data(), stored_mic.size()));
            bool mic_valid = false;
            bool decrypted = false;
            if (inputs_read && !phy_sec_provisioned(flash_)) {
                phy_sec_crc_result(data, transformed);
                const std::uint16_t expected = static_cast<std::uint16_t>(stored_mic[0] |
                    (static_cast<std::uint16_t>(stored_mic[1]) << 8));
                mic_valid = stack_arg0 == 0 || expected ==
                    static_cast<std::uint16_t>(transformed.mic[0] |
                    (static_cast<std::uint16_t>(transformed.mic[1]) << 8));
                decrypted = mic_valid;
            } else if (inputs_read) {
                crypto::Block key{};
                std::array<std::uint8_t, 13> iv{};
                if (core_.read_memory(core_.reg(0), key.data(), key.size()) &&
                    core_.read_memory(core_.reg(1), iv.data(), iv.size()) &&
                    phy_sec_process(key, std::span<const std::uint8_t, 13>(iv),
                                    data, false, transformed)) {
                    mic_valid = stack_arg0 == 0 || transformed.mic == stored_mic;
                    decrypted = mic_valid;
                }
            }
            if (!decrypted) {
                core_.set_reg(0, static_cast<std::uint32_t>(inputs_read ? -1 : -22));
                std::fprintf(stderr,
                             "PHY6252: phy_sec_decrypt rejected block data=0x%08x size=0x%x (%s)\n",
                             source, length, inputs_read ? "partition MIC mismatch" : "invalid arguments");
            } else if (stack_arg1 != 0 &&
                       !core_.write_memory(stack_arg1, transformed.data.data(), transformed.data.size())) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
            } else {
                if (stack_arg1 != 0 && !transformed.data.empty())
                    core_.invalidate_code();
                std::fprintf(stderr,
                             "PHY6252: PHY+ partition %s data=0x%08x size=0x%x dst=0x%08x\n",
                             phy_sec_provisioned(flash_) ? "authenticated/decrypted" : "CRC-checked/copied",
                             source, length, stack_arg1);
                core_.set_reg(0, 0);
            }
        } else if (address == 0x15338) {
            std::uint32_t stack_arg0 = 0, stack_arg1 = 0;
            const bool args_read = core_.read_memory(core_.reg(13), &stack_arg0, sizeof stack_arg0) &&
                core_.read_memory(core_.reg(13) + 4, &stack_arg1, sizeof stack_arg1);
            const std::uint32_t source = core_.reg(2), length = core_.reg(3);
            const std::uint64_t source_end = static_cast<std::uint64_t>(source) + length;
            const bool source_valid = length <= BootImage::FlashSize &&
                ((source >= BootImage::FlashBase &&
                  source_end <= static_cast<std::uint64_t>(BootImage::FlashBase) + BootImage::FlashSize) ||
                 (source >= BootImage::RamBase &&
                  source_end <= static_cast<std::uint64_t>(BootImage::RamBase) + BootImage::RamSize));
            const std::uint64_t output_end = static_cast<std::uint64_t>(stack_arg0) + length;
            const bool output_valid = stack_arg0 == 0 ||
                (stack_arg0 >= BootImage::RamBase &&
                 output_end <= static_cast<std::uint64_t>(BootImage::RamBase) + BootImage::RamSize);
            const bool mic_valid = stack_arg1 == 0 ||
                (stack_arg1 >= BootImage::RamBase &&
                 static_cast<std::uint64_t>(stack_arg1) + 4 <=
                     static_cast<std::uint64_t>(BootImage::RamBase) + BootImage::RamSize);
            std::vector<std::uint8_t> data(length);
            PhySecResult transformed;
            const bool inputs_read = args_read && source_valid && output_valid && mic_valid &&
                core_.read_memory(source, data.data(), data.size());
            std::uint32_t previous_mic = 0xffffffffu;
            if (stack_arg1 != 0)
                core_.read_memory(stack_arg1, &previous_mic, sizeof previous_mic);
            bool transformed_ok = false;
            if (inputs_read && !phy_sec_provisioned(flash_)) {
                phy_sec_crc_result(data, transformed);
                transformed_ok = true;
            } else if (inputs_read) {
                crypto::Block key{};
                std::array<std::uint8_t, 13> iv{};
                transformed_ok = core_.read_memory(core_.reg(0), key.data(), key.size()) &&
                    core_.read_memory(core_.reg(1), iv.data(), iv.size()) &&
                    phy_sec_process(key, std::span<const std::uint8_t, 13>(iv),
                                    data, true, transformed);
            }
            if (!transformed_ok ||
                (stack_arg0 != 0 &&
                 !core_.write_memory(stack_arg0, transformed.data.data(), transformed.data.size())) ||
                (stack_arg1 != 0 &&
                 !core_.write_memory(stack_arg1, transformed.mic.data(), transformed.mic.size()))) {
                core_.set_reg(0, static_cast<std::uint32_t>(-22));
                std::fprintf(stderr,
                             "PHY6252: phy_sec_encrypt rejected block data=0x%08x size=0x%x\n",
                             source, length);
            } else {
                if (stack_arg1 != 0)
                    std::fprintf(stderr,
                                 "PHY6252: PHY+ partition %s generated data=0x%08x size=0x%x mic=%02x%02x%02x%02x previous=0x%08x\n",
                                 phy_sec_provisioned(flash_) ? "AES-CCM MIC" : "CRC16",
                                 source, length, transformed.mic[0], transformed.mic[1],
                                 transformed.mic[2], transformed.mic[3], previous_mic);
                core_.set_reg(0, 0);
            }
        } else {
            char message[512];
            std::uint32_t stack_args[2]{};
            std::uint32_t msp = 0, psp = 0, control = 0, ipsr = 0;
            std::uint32_t cfsr = 0, hfsr = 0, bfar = 0, mmfar = 0;
            core_.read_memory(core_.reg(13), stack_args, sizeof stack_args);
            core_.read_memory(0xe000ed28, &cfsr, sizeof cfsr);
            core_.read_memory(0xe000ed2c, &hfsr, sizeof hfsr);
            core_.read_memory(0xe000ed34, &mmfar, sizeof mmfar);
            core_.read_memory(0xe000ed38, &bfar, sizeof bfar);
            uc_reg_read(core_.engine(), UC_ARM_REG_MSP, &msp);
            uc_reg_read(core_.engine(), UC_ARM_REG_PSP, &psp);
            uc_reg_read(core_.engine(), UC_ARM_REG_CONTROL, &control);
            uc_reg_read(core_.engine(), UC_ARM_REG_IPSR, &ipsr);
            const std::uint32_t lr = core_.reg(14);
            const std::uint32_t exception_sp = (lr & 4u) != 0 ? psp : msp;
            std::array<std::uint32_t, 8> exception_frame{};
            const bool have_exception_frame = ipsr != 0 &&
                core_.read_memory(exception_sp, exception_frame.data(), sizeof exception_frame);
            const char *symbol = address == (RomPendSvHandler & ~1u) ? "PendSV_Handler at " :
                                 address == (RomNmiHandler & ~1u) ? "NMI_Handler at " :
                                 address == (RomHardFaultHandler & ~1u) ? "HardFault_Handler at " :
                                 nullptr;
            std::snprintf(message, sizeof message,
                          "unimplemented PHY6252 ROM call %s0x%08x last_pc=0x%08x previous_pc=0x%08x (r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x sp=0x%08x msp=0x%08x psp=0x%08x control=0x%08x ipsr=0x%08x cfsr=0x%08x hfsr=0x%08x bfar=0x%08x mmfar=0x%08x stack=[0x%08x,0x%08x] frame_pc=0x%08x frame_lr=0x%08x xpsr=0x%08x lr=0x%08x)",
                          symbol ? symbol : "", pc, core_.last_instruction_pc(),
                          core_.previous_instruction_pc(),
                          core_.reg(0), core_.reg(1), core_.reg(2), core_.reg(3),
                          core_.reg(13), msp, psp, control, ipsr,
                          cfsr, hfsr, bfar, mmfar,
                          stack_args[0], stack_args[1],
                          have_exception_frame ? exception_frame[6] : 0,
                          have_exception_frame ? exception_frame[5] : 0,
                          have_exception_frame ? exception_frame[7] : 0, lr);
            return {cortexm::StopReason::Fault, message};
        }
        if (rom_branch)
            continue;
        core_.set_reg(15, core_.reg(14) & ~1u);
    }
}

void Phy6252::rom_log_printf() {
    constexpr std::size_t MaxFormat = 2048;
    constexpr std::size_t MaxField = 256;
    std::string format;
    for (std::size_t i = 0; i < MaxFormat; ++i) {
        std::uint8_t byte = 0;
        if (!core_.read_memory(core_.reg(0) + static_cast<std::uint32_t>(i), &byte, 1))
            break;
        if (byte == 0)
            break;
        format.push_back(static_cast<char>(byte));
    }

    const std::array<std::uint32_t, 3> register_arguments = {
        core_.reg(1), core_.reg(2), core_.reg(3)};
    std::size_t register_argument = 0;
    std::uint32_t stack_argument = core_.reg(13);
    auto next_argument = [&]() {
        if (register_argument < register_arguments.size())
            return register_arguments[register_argument++];
        std::uint32_t value = 0;
        core_.read_memory(stack_argument, &value, sizeof value);
        stack_argument += sizeof value;
        return value;
    };
    auto read_string = [&](std::uint32_t address, std::size_t limit) {
        std::string value;
        for (std::size_t i = 0; i < limit && address + i >= address; ++i) {
            std::uint8_t byte = 0;
            if (!core_.read_memory(address + static_cast<std::uint32_t>(i), &byte, 1) || byte == 0)
                break;
            value.push_back(static_cast<char>(byte));
        }
        return value;
    };

    std::string output;
    for (std::size_t i = 0; i < format.size();) {
        if (format[i] != '%') {
            output.push_back(format[i++]);
            continue;
        }
        ++i;
        if (i < format.size() && format[i] == '%') {
            output.push_back('%');
            ++i;
            continue;
        }
        bool left = false, zero = false, plus = false, alternate = false, space = false;
        while (i < format.size()) {
            switch (format[i]) {
            case '-': left = true; ++i; continue;
            case '0': zero = true; ++i; continue;
            case '+': plus = true; ++i; continue;
            case '#': alternate = true; ++i; continue;
            case ' ': space = true; ++i; continue;
            default: break;
            }
            break;
        }
        unsigned width = 0;
        while (i < format.size() && format[i] >= '0' && format[i] <= '9') {
            width = std::min<unsigned>(MaxField, width * 10 + (format[i++] - '0'));
        }
        int precision = -1;
        if (i < format.size() && format[i] == '.') {
            ++i;
            precision = 0;
            while (i < format.size() && format[i] >= '0' && format[i] <= '9')
                precision = std::min<int>(MaxField, precision * 10 + (format[i++] - '0'));
        }
        if (i < format.size() && (format[i] == 'h' || format[i] == 'l' ||
                                  format[i] == 'z' || format[i] == 't')) {
            const char length = format[i++];
            if (i < format.size() && format[i] == length)
                ++i;
        }
        if (i >= format.size()) {
            output.push_back('%');
            break;
        }
        const char specifier = format[i++];
        std::string field;
        if (specifier == 's') {
            field = read_string(next_argument(), precision < 0 ? MaxFormat :
                                static_cast<std::size_t>(precision));
        } else if (specifier == 'c') {
            field.push_back(static_cast<char>(next_argument()));
        } else if (specifier == 'd' || specifier == 'i' || specifier == 'u' ||
                   specifier == 'x' || specifier == 'X' || specifier == 'o' ||
                   specifier == 'p') {
            const std::uint32_t raw = next_argument();
            const bool signed_value = specifier == 'd' || specifier == 'i';
            const unsigned base = specifier == 'x' || specifier == 'X' || specifier == 'p' ? 16 :
                                  specifier == 'o' ? 8 : 10;
            const bool negative = signed_value && static_cast<std::int32_t>(raw) < 0;
            const std::uint64_t magnitude = negative ?
                static_cast<std::uint64_t>(-static_cast<std::int64_t>(static_cast<std::int32_t>(raw))) : raw;
            char digits[32];
            const auto converted = std::to_chars(digits, digits + sizeof digits, magnitude, base);
            if (converted.ec == std::errc{})
                field.assign(digits, converted.ptr);
            if (specifier == 'X')
                std::transform(field.begin(), field.end(), field.begin(),
                               [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            if (precision == 0 && magnitude == 0)
                field.clear();
            if (precision > 0 && field.size() < static_cast<std::size_t>(precision))
                field.insert(0, static_cast<std::size_t>(precision) - field.size(), '0');
            if (negative)
                field.insert(field.begin(), '-');
            else if (plus && signed_value)
                field.insert(field.begin(), '+');
            else if (space && signed_value)
                field.insert(field.begin(), ' ');
            if (alternate && base == 16 && magnitude != 0)
                field.insert(0, specifier == 'X' ? "0X" : "0x");
            if (specifier == 'p')
                field.insert(0, "0x");
        } else {
            field.push_back('%');
            field.push_back(specifier);
        }
        if (field.size() < width) {
            const std::size_t padding = width - field.size();
            if (left) {
                field.append(padding, ' ');
            } else if (zero && precision < 0 && !field.empty() &&
                       (field[0] == '-' || field[0] == '+' || field[0] == ' ')) {
                field.insert(1, padding, '0');
            } else {
                field.insert(0, padding, zero && precision < 0 ? '0' : ' ');
            }
        }
        output += field;
    }
    tx_.insert(tx_.end(), output.begin(), output.end());
    core_.set_reg(0, static_cast<std::uint32_t>(output.size()));
}

std::uint32_t Phy6252::heap_allocate(std::uint32_t size) {
    if (size == 0 || heap_size_ == 0 || size > std::numeric_limits<std::uint32_t>::max() - 3)
        return 0;
    const std::uint32_t aligned_size = (size + 3) & ~3u;
    for (auto block = free_heap_blocks_.begin(); block != free_heap_blocks_.end(); ++block) {
        if (block->second < aligned_size)
            continue;
        const std::uint32_t address = block->first;
        const std::uint32_t remaining = block->second - aligned_size;
        free_heap_blocks_.erase(block);
        if (remaining != 0)
            free_heap_blocks_.emplace(address + aligned_size, remaining);
        heap_allocations_.emplace(address, aligned_size);
        return address;
    }
    return 0;
}

void Phy6252::heap_release(std::uint32_t address) {
    const auto allocation = heap_allocations_.find(address);
    if (allocation == heap_allocations_.end())
        return;
    std::uint32_t begin = address;
    std::uint32_t size = allocation->second;
    heap_allocations_.erase(allocation);

    auto next = free_heap_blocks_.lower_bound(begin);
    if (next != free_heap_blocks_.begin()) {
        auto previous = std::prev(next);
        if (static_cast<std::uint64_t>(previous->first) + previous->second == begin) {
            begin = previous->first;
            size += previous->second;
            free_heap_blocks_.erase(previous);
        }
    }
    next = free_heap_blocks_.lower_bound(begin);
    if (next != free_heap_blocks_.end() &&
        static_cast<std::uint64_t>(begin) + size == next->first) {
        size += next->second;
        free_heap_blocks_.erase(next);
    }
    free_heap_blocks_.emplace(begin, size);
}

void Phy6252::dma_copy_channel3() {
    const std::uint32_t source = control_registers_[DmaChannel3Sar];
    const std::uint32_t destination = control_registers_[DmaChannel3Dar];
    const std::uint32_t words = control_registers_[DmaChannel3CtlHigh];
    const std::uint64_t byte_count = static_cast<std::uint64_t>(words) * 4;
    if (byte_count > BootImage::FlashSize + BootImage::RamSize) {
        unsupported_register(DmaChannel3CtlHigh, "DMA transfer with invalid block size");
        return;
    }

    std::uint32_t mapped_source = source;
    if ((spif_config_ & (1u << 16)) != 0 &&
        source >= BootImage::FlashBase &&
        source < BootImage::FlashBase + BootImage::FlashSize) {
        const std::uint64_t remapped_offset =
            static_cast<std::uint64_t>(control_registers_[SpifRemap]) +
            (source - BootImage::FlashBase);
        if (remapped_offset + byte_count > BootImage::FlashSize) {
            unsupported_register(DmaChannel3Sar, "DMA source outside remapped flash");
            return;
        }
        mapped_source = BootImage::FlashBase + static_cast<std::uint32_t>(remapped_offset);
    }

    std::vector<std::uint8_t> buffer(static_cast<std::size_t>(byte_count));
    if ((byte_count != 0 &&
         (!core_.read_memory(mapped_source, buffer.data(), buffer.size()) ||
          !core_.write_memory(destination, buffer.data(), buffer.size())))) {
        unsupported_register(DmaChannel3Sar, "DMA transfer with unmapped memory");
        return;
    }
    dma_raw_transfer_status_ |= DmaChannel3Mask;
}

std::uint32_t Phy6252::read(std::uint32_t address, unsigned size) {
    if (address >= UartBase && address < UartBase + 0x1000) {
        const auto reg = (address - UartBase) / 4;
        switch (reg) {
        case 0:
            if (lcr_ & 0x80) return dll_;
            if (rx_.empty()) return 0;
            { auto byte = rx_.front(); rx_.pop_front(); return byte; }
        case 1: return (lcr_ & 0x80) ? dlm_ : ier_;
        case 2: return 1; // no interrupt pending
        case 3: return lcr_;
        case 4: return mcr_;
        case 5: return 0x60 | (rx_.empty() ? 0 : 1);
        case 7: return scratch_;
        default: return 0;
        }
    }
    if (address >= RadioTxFifo && address < RadioTxFifo + RadioTxFifoSize) {
        if (size != 1 && size != 2 && size != 4) {
            unsupported_register(address, "read", 0, size);
            return 0;
        }
        std::uint32_t value = 0;
        for (unsigned i = 0; i < size; ++i) {
            const std::size_t index = address - RadioTxFifo + i;
            if (index < tx_fifo_.size())
                value |= static_cast<std::uint32_t>(tx_fifo_[index]) << (8 * i);
        }
        return value;
    }
    if (address >= RadioRxFifo && address < RadioRxFifo + RadioRxFifoSize) {
        if (size != 1 && size != 2 && size != 4) {
            unsupported_register(address, "read", 0, size);
            return 0;
        }
        std::uint32_t value = 0;
        for (unsigned i = 0; i < size; ++i) {
            const std::size_t index = address - RadioRxFifo + i;
            if (index < rx_fifo_.size()) {
                value |= static_cast<std::uint32_t>(rx_fifo_[index]) << (8 * i);
                if (index >= 1 && index <= rx_fifo_[0] && rx_fifo_read_ <= index)
                    rx_fifo_read_ = index + 1;
            }
        }
        return value;
    }
    if ((address >= RadioRfBase && address < RadioRfBase + 0x1000) ||
        (address >= RadioLlBase && address < RadioLlBase + 0x1000))
        return read_register(address, size);
    if ((address >= FlashControlBase && address < FlashControlBase + 0x1000) ||
        (address >= PcrBase && address < PcrBase + 0x1000) ||
        (address >= TimerBase && address < TimerBase + 0x1000) ||
        (address >= WatchdogBase && address < WatchdogBase + 0x1000) ||
        (address >= GpioBase && address < GpioBase + 0x1000) ||
        (address >= DmaBase && address < DmaBase + 0x1000) ||
        (address >= 0x40003000 && address < 0x40004000) ||
        (address >= 0x4000f000 && address < 0x40010000))
        return read_register(address, size);
    unsupported_register(address, "read", 0, size);
    return 0;
}

void Phy6252::write(std::uint32_t address, std::uint32_t value, unsigned size) {
    if (address >= UartBase && address < UartBase + 0x1000) {
        const auto byte = static_cast<std::uint8_t>(value);
        switch ((address - UartBase) / 4) {
        case 0: if (lcr_ & 0x80) dll_ = byte; else tx_.push_back(byte); break;
        case 1: if (lcr_ & 0x80) dlm_ = byte; else ier_ = byte; break;
        case 3: lcr_ = byte; break;
        case 4: mcr_ = byte; break;
        case 7: scratch_ = byte; break;
        default: break;
        }
        return;
    }
    if (address >= RadioTxFifo && address < RadioTxFifo + RadioTxFifoSize) {
        if (size != 1 && size != 2 && size != 4) {
            unsupported_register(address, "write");
            return;
        }
        const std::size_t offset = address - RadioTxFifo;
        if (offset + size > RadioTxFifoSize) {
            unsupported_register(address, "write outside TX FIFO");
            return;
        }
        if (tx_fifo_.size() < offset + size)
            tx_fifo_.resize(offset + size);
        for (unsigned i = 0; i < size; ++i)
            tx_fifo_[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
        return;
    }
    if (address >= RadioRxFifo && address < RadioRxFifo + RadioRxFifoSize) {
        if (size != 1 && size != 2 && size != 4) {
            unsupported_register(address, "write");
            return;
        }
        const std::size_t offset = address - RadioRxFifo;
        if (offset + size > rx_fifo_.size()) {
            unsupported_register(address, "write outside RX FIFO");
            return;
        }
        for (unsigned i = 0; i < size; ++i)
            rx_fifo_[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
        return;
    }
    if ((address >= RadioRfBase && address < RadioRfBase + 0x1000) ||
        (address >= RadioLlBase && address < RadioLlBase + 0x1000)) {
        write_register(address, value, size);
        return;
    }
    if ((address >= FlashControlBase && address < FlashControlBase + 0x1000) ||
        (address >= PcrBase && address < PcrBase + 0x1000) ||
        (address >= TimerBase && address < TimerBase + 0x1000) ||
        (address >= WatchdogBase && address < WatchdogBase + 0x1000) ||
        (address >= GpioBase && address < GpioBase + 0x1000) ||
        (address >= DmaBase && address < DmaBase + 0x1000) ||
        (address >= 0x40003000 && address < 0x40004000) ||
        (address >= 0x4000f000 && address < 0x40010000)) {
        write_register(address, value, size);
        return;
    }
    unsupported_register(address, "write", value, size);
}

std::uint32_t Phy6252::read_register(std::uint32_t address, unsigned size) {
    if (size != 1 && size != 2 && size != 4) {
        unsupported_register(address, "read", 0, size);
        return 0;
    }
    const std::uint32_t aligned = address & ~3u;
    std::uint32_t value;
    const int timer = timer_index(aligned);
    if (timer >= 0) {
        sync_timer(static_cast<unsigned>(timer), core_.cycles());
        TimerState &state = timers_[static_cast<unsigned>(timer)];
        switch (aligned - timer_base(static_cast<unsigned>(timer))) {
        case 0x00: value = state.load; break;
        case 0x04: value = state.current; break;
        case 0x08: value = state.control; break;
        case 0x0c:
            value = state.interrupt_pending ? 1u : 0u;
            state.interrupt_pending = false;
            update_timer_irq(static_cast<unsigned>(timer));
            break;
        case 0x10: value = state.interrupt_pending ? 1u : 0u; break;
        default:
            unsupported_register(address, "read", 0, size);
            return 0;
        }
        return lane_read(value, address, size);
    }
    if (aligned >= GpioBase && aligned < GpioBase + 0x1000) {
        update_gpio_irq();
        switch (aligned - GpioBase) {
        case 0x00: value = gpio_data_; break;
        case 0x04: value = gpio_direction_; break;
        case 0x30: value = gpio_interrupt_enable_; break;
        case 0x34: value = gpio_interrupt_mask_; break;
        case 0x38: value = gpio_interrupt_type_; break;
        case 0x3c: value = gpio_interrupt_polarity_; break;
        case 0x40: value = gpio_interrupt_status_; break;
        case 0x4c: value = gpio_interrupt_status_; break;
        case 0x50: value = gpio_input_data(); break;
        default:
            unsupported_register(address, "read", 0, size);
            return 0;
        }
        return lane_read(value, address, size);
    }
    if (aligned >= DmaBase && aligned < DmaBase + 0x1000) {
        if (aligned == DmaRawTransfer)
            value = dma_raw_transfer_status_;
        else if (aligned == DmaChannelEnable)
            value = dma_channel_enable_;
        else if (aligned == DmaClearTransfer)
            value = 0;
        else if (aligned >= DmaChannel3Sar && aligned <= DmaChannel3Config + 4)
            value = control_registers_[aligned];
        else if (aligned == DmaConfig)
            value = control_registers_[aligned];
        else {
            unsupported_register(address, "read", 0, size);
            return 0;
        }
        return lane_read(value, address, size);
    }
    if (aligned == CacheControl0 || aligned == CacheControl1 || aligned == CacheRemapTable ||
        (aligned >= CacheRemapControlBase && aligned < CacheRemapControlBase + 0x80)) {
        value = control_registers_[aligned];
        return lane_read(value, address, size);
    }
    if (aligned >= WatchdogBase && aligned < WatchdogBase + 0x1000) {
        switch (aligned - WatchdogBase) {
        case 0x00:
            value = control_registers_[WatchdogControl] & 0xffu;
            break;
        case 0x04:
            value = control_registers_[WatchdogTimeout] & 0xffu;
            break;
        case 0x08: {
            const std::uint32_t control = control_registers_[WatchdogControl];
            if ((control & 1u) == 0 || watchdog_deadline_ns_ == 0) {
                value = 0;
            } else {
                constexpr std::uint64_t NanosPerSecond = 1000000000ull;
                const std::uint32_t range = control_registers_[WatchdogTimeout] & 7u;
                const std::uint64_t timeout_ns = (2ull << range) * NanosPerSecond;
                const std::uint64_t now_ns = core_.cycles_to_ns(core_.cycles());
                const std::uint64_t remaining = watchdog_deadline_ns_ > now_ns ?
                    watchdog_deadline_ns_ - now_ns : 0;
                value = static_cast<std::uint32_t>(
                    (remaining * 65536ull + timeout_ns - 1) / timeout_ns);
            }
            break;
        }
        case 0x0c:
            value = 0;
            break;
        case 0x10:
            value = watchdog_interrupt_pending_ ? 1u : 0u;
            break;
        case 0x14:
            value = watchdog_interrupt_pending_ ? 1u : 0u;
            watchdog_interrupt_pending_ = false;
            core_.set_irq_line(WatchdogIrq, false);
            break;
        default:
            unsupported_register(address, "read", 0, size);
            return 0;
        }
        return lane_read(value, address, size);
    }
    if (aligned >= RadioRfBase && aligned < RadioRfBase + 0x1000) {
        const std::uint32_t offset = aligned - RadioRfBase;
        if (!phy_rf_register(offset)) {
            unsupported_register(address, "read", 0, size);
            return 0;
        }
        if (offset == 0xe4 || offset == 0xe8) {
            if (offset == 0xe8 && !rf_sample_e8_warning_logged_) {
                std::fprintf(stderr,
                             "PHY6252: RF register 0x400300e8 is read by TS0001 calibration code; returning the virtual-air RSSI sample (inferred)\n");
                rf_sample_e8_warning_logged_ = true;
            }
            const std::int8_t rssi = radio_rx_rssi_ > -100 ? radio_rx_rssi_ :
                                     radio_energy_dbm(core_.cycles_to_ns(core_.cycles()));
            value = rf_registers_[aligned] & 0x00ffffffu;
            value |= static_cast<std::uint32_t>(static_cast<std::uint8_t>(rssi)) << 24;
        } else if (offset == 0x94 && rf_registers_.find(aligned) == rf_registers_.end()) {
            value = 0x20; // calibration capacitor value used by the Zephyr driver's fallback range
        } else {
            value = rf_registers_[aligned];
        }
        return lane_read(value, address, size);
    }
    if (aligned >= RadioLlBase && aligned < RadioLlBase + 0x1000) {
        const std::uint32_t offset = aligned - RadioLlBase;
        if (!phy_ll_register(offset)) {
            unsupported_register(address, "read", 0, size);
            return 0;
        }
        if (offset == 0x00)
            value = (radio_tx_active_ || radio_rx_active_) ? 1u : 0u;
        else
            value = ll_registers_[aligned];
        return lane_read(value, address, size);
    }
    switch (aligned) {
    case FlashControlBase: value = control_registers_[aligned]; break;
    case SpifConfig: value = spif_config_; break;
    case SpifReadInstruction:
    case SpifWriteInstruction:
    case SpifDelay:
    case SpifReadDataCapture:
    case SpifDeviceSize:
    case SpifSramPartition:
    case SpifRemap: value = control_registers_[aligned]; break;
    case SpifModeBit: value = control_registers_[aligned]; break;
    case SpifWriteCompletionControl: value = control_registers_[aligned]; break;
    case SpifCommand: value = spif_command_ & ~2u; break; // commands complete immediately
    case SpifCommandAddress: value = control_registers_[aligned]; break;
    case SpifReadData: value = spif_read_data_; break;
    case SpifReadDataHigh:
    case SpifWriteData:
    case SpifWriteDataHigh: value = control_registers_[aligned]; break;
    case SpifPollStatus: value = control_registers_[aligned]; break;
    case PcrCacheReset:
    case PcrCacheBypass: value = control_registers_[aligned]; break;
    case PcrCacheClockGate: value = control_registers_[aligned]; break;
    case PcrReset:
    case PcrReset1:
    case PcrClock:
    case PcrReset2:
    case PcrReset3:
    case PcrClock1:
    case PcrApbClock:
    case PcrApbClockUpdate:
    case MuxAnalog:
    case MuxEnable:
    case MuxPadEnable:
    case RadioClockControl:
    case RadioClockConfig:
    case AonRtcClock:
    case AonPmctl2_0:
    case AonRtcControl:
    case AonEfuseConfig:
    case AonEfuseProgram0:
    case AonEfuseProgram1:
    case AonXtal16Control:
    case AonWakeMask0:
    case AonWakeMask1:
    case AonWakeSource0:
    case AonWakeSource1:
    case AonRtcCompare0:
    case AonRtcCompare1:
    case AonRtcCompare2:
    case AonRtcFlag:
    case AonSleepR0:
    case AonSleepR0 + 4:
    case AonSleepR0 + 8:
    case AonSleepR0 + 0xc:
        value = control_registers_[aligned];
        break;
    case AonRtcCount:
        value = rtc_count(core_.cycles_to_ns(core_.cycles()));
        break;
    default:
        if (mux_select_register(aligned))
            value = control_registers_[aligned];
        else if (aligned >= AonPullBase && aligned <= AonPullEnd && ((aligned - AonPullBase) % 4) == 0)
            value = control_registers_[aligned];
        else {
            unsupported_register(address, "read", 0, size);
            return 0;
        }
    }
    return lane_read(value, address, size);
}

void Phy6252::write_register(std::uint32_t address, std::uint32_t value, unsigned size) {
    if (size != 1 && size != 2 && size != 4) {
        unsupported_register(address, "write", value, size);
        return;
    }
    const std::uint32_t aligned = address & ~3u;
    const int timer = timer_index(aligned);
    if (timer >= 0) {
        const unsigned index = static_cast<unsigned>(timer);
        TimerState &state = timers_[index];
        sync_timer(index, core_.cycles());
        const std::uint32_t offset = aligned - timer_base(index);
        switch (offset) {
        case 0x00:
            state.load = lane_write(state.load, value, address, size);
            if ((state.control & TimerEnable) == 0)
                state.current = state.load;
            state.cycle_remainder = 0;
            state.last_cycle = core_.cycles();
            break;
        case 0x04:
            state.current = lane_write(state.current, value, address, size);
            state.cycle_remainder = 0;
            state.last_cycle = core_.cycles();
            break;
        case 0x08: {
            const std::uint32_t old_control = state.control;
            state.control = lane_write(state.control, value, address, size) & 7u;
            if ((old_control & TimerEnable) == 0 && (state.control & TimerEnable) != 0) {
                state.current = state.load;
                state.cycle_remainder = 0;
            }
            state.last_cycle = core_.cycles();
            update_timer_irq(index);
            core_.request_service(core_.cycles() + 1);
            break;
        }
        case 0x0c:
            if (lane_write(0, value, address, size) & 1u)
                state.interrupt_pending = false;
            update_timer_irq(index);
            break;
        case 0x10:
            // Interrupt status is read-only on the DesignWare timer.
            break;
        default:
            unsupported_register(address, "write", value, size);
            return;
        }
        return;
    }
    if (aligned >= GpioBase && aligned < GpioBase + 0x1000) {
        const std::uint32_t offset = aligned - GpioBase;
        switch (offset) {
        case 0x00: {
            const std::uint32_t updated = lane_write(gpio_data_, value, address, size) & GpioMask;
            const std::uint32_t changed = gpio_data_ ^ updated;
            gpio_data_ = updated;
            notify_gpio(changed);
            update_gpio_irq();
            break;
        }
        case 0x04: {
            const std::uint32_t updated = lane_write(gpio_direction_, value, address, size) & GpioMask;
            const std::uint32_t changed = gpio_direction_ ^ updated;
            gpio_direction_ = updated;
            notify_gpio(changed);
            update_gpio_irq();
            break;
        }
        case 0x30:
            gpio_interrupt_enable_ = lane_write(gpio_interrupt_enable_, value, address, size) & GpioMask;
            update_gpio_irq();
            break;
        case 0x34:
            gpio_interrupt_mask_ = lane_write(gpio_interrupt_mask_, value, address, size) & GpioMask;
            update_gpio_irq();
            break;
        case 0x38:
        {
            const std::uint32_t updated =
                lane_write(gpio_interrupt_type_, value, address, size) & GpioMask;
            // Switching an active-low level input to edge mode does not
            // synthesize an edge; the SDK configures edge mode before it
            // enables the pin and installs its ROM callback.
            gpio_interrupt_status_ &= ~(updated & ~gpio_interrupt_type_);
            gpio_interrupt_type_ = updated;
            update_gpio_irq();
            break;
        }
        case 0x3c:
            gpio_interrupt_polarity_ = lane_write(gpio_interrupt_polarity_, value, address, size) & GpioMask;
            update_gpio_irq();
            break;
        case 0x4c:
            gpio_interrupt_status_ &= ~(lane_write(0, value, address, size) & GpioMask);
            update_gpio_irq();
            break;
        case 0x40:
        case 0x50:
            unsupported_register(address, "write to read-only GPIO register");
            return;
        default:
            unsupported_register(address, "write");
            return;
        }
        return;
    }
    if (aligned >= DmaBase && aligned < DmaBase + 0x1000) {
        const std::uint32_t written = lane_write(control_registers_[aligned], value, address, size);
        if (aligned == DmaRawTransfer) {
            unsupported_register(address, "write to read-only DMA status");
        } else if (aligned == DmaClearTransfer) {
            dma_raw_transfer_status_ &= ~written;
        } else if (aligned == DmaChannelEnable) {
            dma_channel_enable_ = written;
            control_registers_[aligned] = written;
            if (written & DmaChannel3Mask) {
                dma_copy_channel3();
                dma_channel_enable_ &= ~DmaChannel3Mask;
                control_registers_[aligned] = dma_channel_enable_;
            }
        } else if (aligned == DmaConfig ||
                   (aligned >= DmaChannel3Sar && aligned <= DmaChannel3Config + 4)) {
            control_registers_[aligned] = written;
        } else {
            unsupported_register(address, "write");
        }
        return;
    }
    if (aligned == CacheControl0 || aligned == CacheControl1 || aligned == CacheRemapTable ||
        (aligned >= CacheRemapControlBase && aligned < CacheRemapControlBase + 0x80)) {
        control_registers_[aligned] = lane_write(control_registers_[aligned], value, address, size);
        return;
    }
    if (aligned >= WatchdogBase && aligned < WatchdogBase + 0x1000) {
        switch (aligned - WatchdogBase) {
        case 0x00: {
            const std::uint32_t old_control = control_registers_[WatchdogControl];
            const std::uint32_t updated = lane_write(old_control, value, address, size) & 0x1fu;
            control_registers_[WatchdogControl] = updated;
            if ((updated & 1u) == 0) {
                watchdog_deadline_ns_ = 0;
                watchdog_interrupt_pending_ = false;
                core_.set_irq_line(WatchdogIrq, false);
            } else if ((old_control & 1u) == 0) {
                watchdog_interrupt_pending_ = false;
                watchdog_restart();
            }
            break;
        }
        case 0x04:
            control_registers_[WatchdogTimeout] =
                lane_write(control_registers_[WatchdogTimeout], value, address, size) & 0xffu;
            if (control_registers_[WatchdogControl] & 1u)
                watchdog_restart();
            break;
        case 0x08:
        case 0x10:
        case 0x14:
            unsupported_register(address, "write to read-only watchdog register");
            return;
        case 0x0c:
            if (lane_write(0, value, address, size) == 0x76u)
                watchdog_restart();
            break;
        default:
            unsupported_register(address, "write");
            return;
        }
        return;
    }
    if (aligned >= RadioRfBase && aligned < RadioRfBase + 0x1000) {
        const std::uint32_t offset = aligned - RadioRfBase;
        if (!phy_rf_register(offset)) {
            unsupported_register(address, "write");
            return;
        }
        rf_registers_[aligned] = lane_write(rf_registers_[aligned], value, address, size);
        return;
    }
    if (aligned >= RadioLlBase && aligned < RadioLlBase + 0x1000) {
        const std::uint32_t offset = aligned - RadioLlBase;
        if (!phy_ll_register(offset)) {
            unsupported_register(address, "write", value, size);
            return;
        }
        const std::uint32_t written = lane_write(ll_registers_[aligned], value, address, size);
        switch (offset) {
        case 0x00:
            ll_registers_[aligned] = written & ~1u;
            if ((written & 1u) == 0) {
                radio_rx_active_ = false;
                radio_rx_deadline_cycle_ = 0;
                // PHY_ABORT cancels controller completion but cannot retract a frame already on air.
                radio_tx_active_ = false;
            } else if ((ll_registers_[RadioLlBase + 0x04] & 7u) == 0) {
                start_radio_tx();
            } else if ((ll_registers_[RadioLlBase + 0x04] & 7u) == 1) {
                radio_rx_active_ = true;
                radio_rx_after_ns_ = core_.cycles_to_ns(core_.cycles());
                schedule_radio_rx_timeout();
                if (air_)
                    core_.request_service(core_.cycles() + 1);
            }
            break;
        case 0x10:
            // Interrupt status is hardware-owned and write-one-to-clear at +0x14.
            break;
        case 0x14:
            ll_registers_[RadioLlBase + 0x10] &= ~written;
            if ((written & (RadioIrqCok | RadioIrqCokZephyr)) != 0)
                ll_registers_[RadioLlBase + 0x10] &= ~(RadioIrqCok | RadioIrqCokZephyr);
            if ((written & (RadioIrqRto | RadioIrqRtoZephyr)) != 0)
                ll_registers_[RadioLlBase + 0x10] &= ~(RadioIrqRto | RadioIrqRtoZephyr);
            ll_registers_[aligned] = written;
            if ((ll_registers_[RadioLlBase + 0x10] &
                 (RadioIrqMd | RadioIrqCok | RadioIrqCokZephyr)) == 0)
                radio_rx_rssi_ = -100;
            update_radio_irq();
            break;
        default:
            ll_registers_[aligned] = written;
            break;
        }
        if (offset == 0x0c || offset == 0x10 || offset == 0x14)
            update_radio_irq();
        return;
    }
    switch (aligned) {
    case FlashControlBase:
    case SpifReadInstruction:
    case SpifWriteInstruction:
    case SpifDelay:
    case SpifReadDataCapture:
    case SpifDeviceSize:
    case SpifSramPartition:
    case SpifRemap:
    case SpifModeBit:
    case SpifWriteCompletionControl:
    case SpifCommandAddress:
    case SpifWriteData:
    case SpifWriteDataHigh:
    case SpifPollStatus:
    case PcrCacheReset:
    case PcrCacheBypass:
    case PcrCacheClockGate:
    case PcrReset:
    case PcrReset1:
    case PcrClock:
    case PcrReset2:
    case PcrReset3:
    case PcrClock1:
    case PcrApbClock:
    case PcrApbClockUpdate:
    case MuxAnalog:
    case MuxEnable:
    case MuxPadEnable:
    case RadioClockControl:
    case RadioClockConfig:
    case AonPmctl2_0:
    case AonEfuseConfig:
    case AonEfuseProgram0:
    case AonEfuseProgram1:
    case AonXtal16Control:
    case AonWakeMask0:
    case AonWakeMask1:
    case AonSleepR0:
    case AonSleepR0 + 4:
    case AonSleepR0 + 8:
    case AonSleepR0 + 0xc:
        control_registers_[aligned] = lane_write(control_registers_[aligned], value, address, size);
        if (aligned == MuxEnable || aligned == MuxAnalog || aligned == MuxPadEnable ||
            mux_select_register(aligned))
            notify_gpio(GpioMask);
        break;
    case AonRtcControl: {
        const std::uint64_t now_ns = core_.cycles_to_ns(core_.cycles());
        const std::uint32_t old_control = control_registers_[AonRtcControl];
        const std::uint32_t updated = lane_write(old_control, value, address, size);
        if (old_control & 1u)
            rtc_epoch_count_ = rtc_count(now_ns);
        if (updated & 2u)
            rtc_epoch_count_ = 0;
        control_registers_[AonRtcControl] = updated & ~2u;
        rtc_epoch_ns_ = now_ns;
        break;
    }
    case AonRtcCompare0:
    case AonRtcCompare1:
    case AonRtcCompare2:
    case AonRtcClock:
        control_registers_[aligned] = lane_write(control_registers_[aligned], value, address, size);
        break;
    case AonRtcFlag:
        control_registers_[AonRtcFlag] &= ~lane_write(0, value, address, size);
        break;
    case SpifConfig:
        // Bit 31 is exposed as the hardware-owned idle/ready indication.
        spif_config_ = lane_write(spif_config_, value, address, size) | FlashReady;
        control_registers_[aligned] = spif_config_;
        break;
    case SpifCommand: {
        const std::uint32_t command = lane_write(spif_command_, value, address, size);
        if (command != SpifReadStatus) {
            unsupported_register(address, "unsupported SPIF command write");
            return;
        }
        spif_command_ = command;
        spif_read_data_ = 0; // status register has WIP clear in this non-busy model
        break;
    }
    case SpifReadData:
        unsupported_register(address, "write");
        break;
    default:
        if (mux_select_register(aligned)) {
            control_registers_[aligned] = lane_write(control_registers_[aligned], value, address, size);
            notify_gpio(GpioMask);
        } else if (aligned >= AonPullBase && aligned <= AonPullEnd && ((aligned - AonPullBase) % 4) == 0)
        {
            control_registers_[aligned] = lane_write(control_registers_[aligned], value, address, size);
            notify_gpio(GpioMask);
        }
        else
            unsupported_register(address, "write", value, size);
        break;
    }
}

void Phy6252::unsupported_register(std::uint32_t address, const char *operation,
                                  std::uint32_t value, unsigned size) {
    if (pending_fault_.empty()) {
        std::uint32_t msp = 0, psp = 0, control = 0, ipsr = 0;
        uc_reg_read(core_.engine(), UC_ARM_REG_MSP, &msp);
        uc_reg_read(core_.engine(), UC_ARM_REG_PSP, &psp);
        uc_reg_read(core_.engine(), UC_ARM_REG_CONTROL, &control);
        uc_reg_read(core_.engine(), UC_ARM_REG_IPSR, &ipsr);
        char message[560];
        if (size != 0) {
            std::snprintf(message, sizeof message,
                          "unimplemented PHY6252 MMIO %s at 0x%08x value=0x%08x size=%u pc=0x%08x last_pc=0x%08x r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x sp=0x%08x msp=0x%08x psp=0x%08x control=0x%08x ipsr=0x%08x lr=0x%08x",
                          operation, address, value, size, core_.pc(),
                          core_.last_instruction_pc(), core_.reg(0),
                          core_.reg(1), core_.reg(2), core_.reg(3), core_.reg(13),
                          msp, psp, control, ipsr, core_.reg(14));
        } else {
            std::snprintf(message, sizeof message, "unimplemented PHY6252 MMIO %s at 0x%08x",
                          operation, address);
        }
        pending_fault_ = message;
    }
    core_.request_reset();
}

void Phy6252::connect_radio(mcuemu::VirtualAir &air, std::string node) {
    air_ = &air;
    radio_node_ = std::move(node);
    if (radio_rx_active_)
        core_.request_service(core_.cycles() + 1);
}

void Phy6252::set_gpio_observer(std::function<void(unsigned)> observer) {
    gpio_observer_ = std::move(observer);
    notify_gpio(GpioMask);
}

void Phy6252::set_gpio_input(unsigned pin, bool high) {
    if (pin >= 23)
        return;
    const std::uint32_t bit = 1u << pin;
    const bool previous = (gpio_input_ & bit) != 0;
    if (high)
        gpio_input_ |= bit;
    else
        gpio_input_ &= ~bit;
    if (previous != high && (gpio_interrupt_type_ & bit) != 0 &&
        high == ((gpio_interrupt_polarity_ & bit) != 0))
        gpio_interrupt_status_ |= bit;
    update_gpio_irq();
}

GpioDrive Phy6252::gpio_drive(unsigned pin) const noexcept {
    if (pin >= 23)
        return GpioDrive::HiZ;
    const std::uint32_t bit = 1u << pin;
    if ((control_registers_.contains(MuxEnable) && (control_registers_.at(MuxEnable) & bit)) != 0)
        return GpioDrive::HiZ;
    if (gpio_direction_ & bit)
        return gpio_data_ & bit ? GpioDrive::High : GpioDrive::Low;
    const std::uint32_t pull_address = AonPullBase + GpioPullRegister[pin] * 4;
    const auto it = control_registers_.find(pull_address);
    const std::uint32_t pull_value = it == control_registers_.end() ? 0 : it->second;
    const std::uint32_t pull = (pull_value >> GpioPullShift[pin]) & 3u;
    if (pull == 2)
        return GpioDrive::PullUp;
    if (pull == 3)
        return GpioDrive::PullDown;
    return GpioDrive::HiZ;
}

std::uint32_t Phy6252::gpio_input_data() const noexcept {
    return ((gpio_input_ & ~gpio_direction_) | (gpio_data_ & gpio_direction_)) & GpioMask;
}

void Phy6252::update_gpio_irq() {
    const std::uint32_t inputs = gpio_input_data();
    const std::uint32_t level_mask = ~gpio_interrupt_type_ & GpioMask;
    const std::uint32_t active_levels = (inputs & gpio_interrupt_polarity_) |
        (~inputs & ~gpio_interrupt_polarity_);
    gpio_interrupt_status_ = (gpio_interrupt_status_ & gpio_interrupt_type_) |
        (active_levels & level_mask);
    core_.set_irq_line(16, (gpio_interrupt_status_ & gpio_interrupt_enable_ &
                            ~gpio_interrupt_mask_ & GpioMask) != 0);
}

void Phy6252::notify_gpio(std::uint32_t changed) {
    if (!gpio_observer_)
        return;
    for (unsigned pin = 0; pin < 23; ++pin)
        if (changed & (1u << pin))
            gpio_observer_(pin);
}

bool Phy6252::radio_receiving() const noexcept { return radio_rx_active_; }

std::uint8_t Phy6252::radio_channel() const noexcept {
    const auto it = rf_registers_.find(RadioRfBase + 0xb4);
    const std::uint32_t setting = it == rf_registers_.end() ? 5u : it->second & 0xffu;
    if (setting < 5)
        return 11;
    return static_cast<std::uint8_t>(std::clamp<std::uint32_t>(11 + (setting - 5) / 5, 11, 26));
}

void Phy6252::receive_radio_frame(const mcuemu::RadioFrame &frame) {
    if (!radio_rx_active_ || frame.payload.size() + 2 > 127 || frame.channel != radio_channel() ||
        (radio_rx_deadline_cycle_ != 0 &&
         frame.end_ns > core_.cycles_to_ns(radio_rx_deadline_cycle_)))
        return;
    rx_fifo_.fill(0);
    rx_fifo_[0] = static_cast<std::uint8_t>(frame.payload.size() + 2);
    std::copy(frame.payload.begin(), frame.payload.end(), rx_fifo_.begin() + 1);
    const std::uint16_t fcs = ieee802154_fcs(frame.payload);
    rx_fifo_[frame.payload.size() + 1] = static_cast<std::uint8_t>(fcs);
    rx_fifo_[frame.payload.size() + 2] = static_cast<std::uint8_t>(fcs >> 8);
    rx_fifo_read_ = 0;
    radio_rx_rssi_ = static_cast<std::int8_t>(std::clamp(frame.rssi_dbm, -127, 0));
    radio_rx_active_ = false;
    radio_rx_deadline_cycle_ = 0;
    ll_registers_[RadioLlBase + 0x10] |= RadioIrqMd | RadioIrqCok | RadioIrqCokZephyr;
    update_radio_irq();
}

void Phy6252::schedule_radio_rx_timeout() {
    radio_rx_deadline_cycle_ = 0;
    if (radio_rx_timeout_us_ == 0)
        return;
    const std::uint64_t duration_cycles = std::max<std::uint64_t>(1,
        static_cast<std::uint64_t>(radio_rx_timeout_us_) * core_.clock_hz() / 1'000'000);
    radio_rx_deadline_cycle_ = core_.cycles() + duration_cycles;
    core_.request_service(radio_rx_deadline_cycle_);
}

std::int8_t Phy6252::radio_energy_dbm(std::uint64_t at_ns) const {
    if (!air_)
        return -100;
    double strongest = -100;
    const bool ideal = air_->propagation().model == mcuemu::Propagation::Model::Ideal;
    for (const mcuemu::RadioFrame &frame : air_->frames()) {
        if (frame.protocol != mcuemu::RadioProtocol::Zigbee802154 ||
            frame.channel != radio_channel() || frame.source == radio_node_ ||
            frame.start_ns > at_ns || frame.end_ns + 128000 < at_ns)
            continue;
        const double power = ideal ? -50.0 :
            frame.tx_power_dbm - air_->path_loss_db(frame.source, radio_node_);
        strongest = std::max(strongest, power);
    }
    return static_cast<std::int8_t>(std::clamp(strongest, -127.0, 0.0));
}

void Phy6252::start_radio_tx() {
    radio_rx_active_ = false;
    radio_rx_deadline_cycle_ = 0;
    const std::uint32_t length = tx_fifo_.empty() ? 0 : tx_fifo_[0];
    if (length < 5 || length > 127 || tx_fifo_.size() < length - 1) {
        std::fprintf(stderr, "PHY6252: ignored radio TX command without a valid FIFO frame (length=%u, bytes=%zu)\n",
                     length, tx_fifo_.size());
        return;
    }
    radio_tx_active_ = true;
    const std::uint64_t start_ns = core_.cycles_to_ns(core_.cycles());
    std::vector<std::uint8_t> payload;
    payload.assign(tx_fifo_.begin() + 1, tx_fifo_.begin() + (length - 1));
    std::uint64_t duration_ns = mcuemu::VirtualAir::airtime_ns(
        mcuemu::RadioProtocol::Zigbee802154, payload.size());
    if (air_ && !payload.empty()) {
        air_->transmit(radio_node_, {}, mcuemu::RadioProtocol::Zigbee802154,
                       radio_channel(), start_ns, std::move(payload));
    }
    radio_tx_end_cycle_ = std::max(core_.cycles() + 1,
        core_.ns_to_cycles(start_ns + duration_ns));
    core_.request_service(radio_tx_end_cycle_);
}

void Phy6252::update_radio_irq() {
    const std::uint32_t status = ll_registers_[RadioLlBase + 0x10];
    const std::uint32_t enabled = ll_registers_[RadioLlBase + 0x0c];
    core_.set_irq_line(RadioIrq, (status & enabled & RadioIrqMask) != 0);
}

std::uint64_t Phy6252::service_peripherals(std::uint64_t now) {
    std::uint64_t next = service_radio(now);
    const std::uint32_t watchdog_control = control_registers_[WatchdogControl];
    if ((watchdog_control & 1u) != 0 && watchdog_deadline_ns_ != 0) {
        const std::uint64_t now_ns = core_.cycles_to_ns(now);
        if (now_ns >= watchdog_deadline_ns_) {
            if ((watchdog_control & 2u) != 0 && !watchdog_interrupt_pending_) {
                watchdog_interrupt_pending_ = true;
                core_.set_irq_line(WatchdogIrq, true);
                watchdog_restart();
            } else {
                core_.request_reset();
                watchdog_deadline_ns_ = 0;
            }
        }
        if (watchdog_deadline_ns_ != 0)
            next = std::min(next, core_.ns_to_cycles(watchdog_deadline_ns_));
    }
    for (unsigned timer = 0; timer < timers_.size(); ++timer) {
        sync_timer(timer, now);
        const TimerState &state = timers_[timer];
        if (state.control & TimerEnable) {
            const std::uint64_t ticks = state.current == 0 ? 1 : state.current;
            const std::uint64_t cycles = ticks * timer_cycles_per_tick(core_.clock_hz()) -
                state.cycle_remainder;
            next = std::min(next, now + std::max<std::uint64_t>(1, cycles));
        }
    }
    return next;
}

void Phy6252::watchdog_restart() {
    constexpr std::uint64_t NanosPerSecond = 1000000000ull;
    const std::uint32_t range = control_registers_[WatchdogTimeout] & 7u;
    const std::uint64_t timeout_ns = (2ull << range) * NanosPerSecond;
    watchdog_deadline_ns_ = core_.cycles_to_ns(core_.cycles()) + timeout_ns;
    core_.request_service(core_.ns_to_cycles(watchdog_deadline_ns_));
}

std::uint32_t Phy6252::rtc_count(std::uint64_t at_ns) const noexcept {
    if ((control_registers_.contains(AonRtcControl) &&
         (control_registers_.at(AonRtcControl) & 1u) != 0) && at_ns > rtc_epoch_ns_) {
        const std::uint64_t elapsed_ns = at_ns - rtc_epoch_ns_;
        const std::uint64_t ticks = elapsed_ns / 1000000000ull * 32768ull +
            elapsed_ns % 1000000000ull * 32768ull / 1000000000ull;
        return rtc_epoch_count_ + static_cast<std::uint32_t>(ticks);
    }
    return rtc_epoch_count_;
}

void Phy6252::sync_timer(unsigned timer, std::uint64_t now) {
    TimerState &state = timers_[timer];
    if (now <= state.last_cycle) {
        state.last_cycle = now;
        return;
    }
    const std::uint64_t elapsed = now - state.last_cycle;
    state.last_cycle = now;
    if ((state.control & TimerEnable) == 0)
        return;
    const std::uint64_t accumulated = elapsed + state.cycle_remainder;
    const std::uint64_t cycles_per_tick = timer_cycles_per_tick(core_.clock_hz());
    const std::uint64_t ticks = accumulated / cycles_per_tick;
    state.cycle_remainder = accumulated % cycles_per_tick;
    if (ticks == 0)
        return;

    if (state.control & TimerUserMode) {
        const std::uint64_t until_zero = state.current == 0 ? 1 : state.current;
        if (ticks >= until_zero) {
            state.current = 0;
            state.control &= ~TimerEnable;
            state.cycle_remainder = 0;
            state.interrupt_pending = true;
        } else {
            state.current -= static_cast<std::uint32_t>(ticks);
        }
    } else {
        const std::uint64_t until_zero = state.current == 0 ? 1 : state.current;
        if (ticks >= until_zero) {
            const std::uint64_t after_zero = ticks - until_zero;
            state.current = after_zero == 0 ? 0 :
                0xffffffffu - static_cast<std::uint32_t>(after_zero - 1);
            state.interrupt_pending = true;
        } else {
            state.current -= static_cast<std::uint32_t>(ticks);
        }
    }
    update_timer_irq(timer);
}

void Phy6252::update_timer_irq(unsigned timer) {
    const TimerState &state = timers_[timer];
    const unsigned irq = 20 + timer;
    const bool asserted = state.interrupt_pending && (state.control & TimerInterruptMask) == 0;
    core_.set_irq_line(irq, asserted);
}

std::uint64_t Phy6252::service_radio(std::uint64_t now) {
    if (radio_tx_active_ && now >= radio_tx_end_cycle_) {
        radio_tx_active_ = false;
        ll_registers_[RadioLlBase + 0x10] |= RadioIrqMd;
        update_radio_irq();
    }

    std::uint64_t next = radio_tx_active_ ? radio_tx_end_cycle_ :
        std::numeric_limits<std::uint64_t>::max();
    if (radio_rx_active_ && air_) {
        const std::uint64_t now_ns = core_.cycles_to_ns(now);
        for (const mcuemu::RadioFrame &frame : air_->receive(
                 radio_node_, mcuemu::RadioProtocol::Zigbee802154,
                 radio_channel(), now_ns, radio_rx_after_ns_)) {
            if ((ll_registers_[RadioLlBase + 0x10] & RadioIrqMd) != 0)
                break;
            receive_radio_frame(frame);
            if (!radio_rx_active_)
                break;
        }
        radio_rx_after_ns_ = now_ns;
        if (radio_rx_active_)
            next = std::min(next, now + RadioPollCycles);
    }
    if (radio_rx_active_ && radio_rx_deadline_cycle_ != 0) {
        if (now >= radio_rx_deadline_cycle_) {
            radio_rx_active_ = false;
            radio_rx_deadline_cycle_ = 0;
            // RTO identifies the cause; MD signals that the radio operation completed.
            ll_registers_[RadioLlBase + 0x10] |=
                RadioIrqMd | RadioIrqRto | RadioIrqRtoZephyr;
            update_radio_irq();
        } else {
            next = std::min(next, radio_rx_deadline_cycle_);
        }
    }
    return next;
}

bool Phy6252::rom_flash_write() {
    const std::uint32_t address = core_.reg(0);
    const std::uint32_t source = core_.reg(1);
    const std::uint32_t length = core_.reg(2);
    const std::uint64_t end = static_cast<std::uint64_t>(address) + length;
    if (address < BootImage::FlashBase ||
        end > static_cast<std::uint64_t>(BootImage::FlashBase) + BootImage::FlashSize ||
        (length != 0 && length > BootImage::FlashSize)) {
        core_.set_reg(0, static_cast<std::uint32_t>(-22)); // EINVAL
        return false;
    }
    if (length == 0) {
        core_.set_reg(0, 0);
        return true;
    }
    std::vector<std::uint8_t> data(length);
    if (!core_.read_memory(source, data.data(), data.size())) {
        core_.set_reg(0, static_cast<std::uint32_t>(-22));
        return false;
    }
    const std::uint32_t offset = address - BootImage::FlashBase;
    for (std::uint32_t i = 0; i < length; ++i)
        flash_[offset + i] &= data[i];
    core_.set_reg(0, 0);
    // ROM execution has returned to the host between translated blocks here.
    core_.invalidate_code();
    return true;
}

bool Phy6252::rom_flash_erase() {
    const std::uint32_t address = core_.reg(0);
    if (address < BootImage::FlashBase ||
        static_cast<std::uint64_t>(address) >=
            static_cast<std::uint64_t>(BootImage::FlashBase) + BootImage::FlashSize ||
        ((address - BootImage::FlashBase) % FlashSectorSize) != 0) {
        core_.set_reg(0, static_cast<std::uint32_t>(-22)); // EINVAL
        return false;
    }
    const std::uint32_t offset = address - BootImage::FlashBase;
    std::fill_n(flash_ + offset, FlashSectorSize, 0xff);
    core_.set_reg(0, 0);
    core_.invalidate_code();
    return true;
}

void Phy6252::uart_receive(std::uint8_t byte) { rx_.push_back(byte); }
std::vector<std::uint8_t> Phy6252::take_uart_tx() {
    std::vector<std::uint8_t> output;
    output.swap(tx_);
    return output;
}
} // namespace phy6252emu
