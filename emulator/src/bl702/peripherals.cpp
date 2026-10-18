#include "blocks.hpp"

#include "mcuemu/aes.hpp"

#include <algorithm>
#include <cstring>

namespace bl702emu {

namespace {
std::uint32_t lane_mask(unsigned size) {
    return size == 1 ? 0xffu : size == 2 ? 0xffffu : 0xffffffffu;
}

std::uint32_t little_endian(const std::uint8_t *bytes) {
    return bytes[0] | bytes[1] << 8 | bytes[2] << 16 | static_cast<std::uint32_t>(bytes[3]) << 24;
}
} // namespace

std::uint32_t RegisterBlock::read(std::uint32_t offset, unsigned size) {
    return (read_word(offset & ~3u) >> ((offset & 3u) * 8)) & lane_mask(size);
}

void RegisterBlock::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    const std::uint32_t aligned = offset & ~3u;
    if (size == 4) {
        write_word(aligned, value);
        return;
    }
    const unsigned shift = (offset & 3u) * 8;
    const std::uint32_t mask = lane_mask(size) << shift;
    write_word(aligned, (read_word(aligned) & ~mask) | ((value << shift) & mask));
}

std::uint32_t RegisterBlock::read_word(std::uint32_t offset) { return word(offset); }

void RegisterBlock::write_word(std::uint32_t offset, std::uint32_t value) { set_word(offset, value); }

std::uint32_t RegisterBlock::word(std::uint32_t offset) const {
    auto it = words_.find(offset);
    return it == words_.end() ? 0 : it->second;
}

void RegisterBlock::set_word(std::uint32_t offset, std::uint32_t value) { words_[offset] = value; }

// ---- UART0 ----------------------------------------------------------------------------------------------
// Offsets from the SDK's uart driver: FIFO_CONFIG_1 (0x84) carries the free space of the transmit FIFO in
// its low byte, FIFO_WDATA (0x88) takes the byte to send.

namespace {
constexpr std::uint32_t UartFifoConfig1 = 0x84, UartFifoWrite = 0x88;
// The transmit FIFO never holds a byte: it reports room for this many.
constexpr std::uint32_t UartTxFifoFree = 0x10;
} // namespace

Uart0::Uart0(Bl702 &chip) : RegisterBlock(chip, 0x4000'a000, 0x100) {}

std::uint32_t Uart0::read_word(std::uint32_t offset) {
    if (offset == UartFifoConfig1)
        return (word(offset) & ~0xffu) | UartTxFifoFree;
    return RegisterBlock::read_word(offset);
}

void Uart0::write_word(std::uint32_t offset, std::uint32_t value) {
    if (offset == UartFifoWrite) {
        chip_.uart_tx(static_cast<std::uint8_t>(value));
        return;
    }
    RegisterBlock::write_word(offset, value);
}

// ---- TIMER0 ---------------------------------------------------------------------------------------------

namespace {
constexpr std::uint32_t TimerCounter = 0x2c, TimerEnable = 0x84, TimerClockDivider = 0xbc;
constexpr std::uint32_t TimerEnableBit = 1u << 1;
} // namespace

Timer0::Timer0(Bl702 &chip) : RegisterBlock(chip, 0x4000'a500, 0x100) {}

void Timer0::reset() {
    RegisterBlock::reset();
    base_cycle_ = 0;
    base_count_ = 0;
    running_ = false;
}

std::uint32_t Timer0::counter() const {
    if (!running_)
        return base_count_;
    // One tick every (divider + 1) core cycles: the SDK divides the 32 MHz clock by 32 for 1 MHz.
    const std::uint64_t period = ((word(TimerClockDivider) >> 8) & 0xff) + 1;
    return base_count_ + static_cast<std::uint32_t>((chip_.core().cycles() - base_cycle_) / period);
}

std::uint32_t Timer0::read_word(std::uint32_t offset) {
    if (offset == TimerCounter)
        return counter();
    return RegisterBlock::read_word(offset);
}

void Timer0::write_word(std::uint32_t offset, std::uint32_t value) {
    const std::uint64_t now = chip_.core().cycles();
    if (offset == TimerCounter) {
        base_count_ = value;
        base_cycle_ = now;
        return;
    }
    if (offset == TimerEnable) {
        const bool run = (value & TimerEnableBit) != 0;
        if (run && !running_) {
            base_cycle_ = now;
            running_ = true;
        } else if (!run && running_) {
            base_count_ = counter();
            running_ = false;
        }
    }
    RegisterBlock::write_word(offset, value);
}

// ---- L1 cache ------------------------------------------------------------------------------------------

namespace {
constexpr std::uint32_t L1cOperationDone = 1u << 3;
} // namespace

L1c::L1c(Bl702 &chip) : RegisterBlock(chip, 0x4000'9000, 0x100) {}

std::uint32_t L1c::read_word(std::uint32_t offset) {
    return RegisterBlock::read_word(offset) | (offset == 0 ? L1cOperationDone : 0);
}

// ---- flash controller -----------------------------------------------------------------------------------

namespace {
constexpr std::uint32_t SfId0Offset = 0x434; // SF_CTRL_SF_ID0_OFFSET of bl70x (bank 1; bank 2 is 0x43c)
} // namespace

namespace {
// IF_SAHB_0 (bank 1, at +0x8 of the interface): the command word, and the command words at +0xc and +0x10.
constexpr std::uint32_t SahbConfig = 0x8, SahbCmd0 = 0xc, SahbCmd1 = 0x10;
constexpr std::uint32_t SahbBusy = 1u << 0, SahbTrig = 1u << 1;
constexpr unsigned SahbDataBytePos = 2, SahbAddrBytePos = 17;
constexpr std::uint32_t SahbAddrEn = 1u << 26, SahbDataEn = 1u << 24, SahbDataRw = 1u << 23;
constexpr std::uint32_t SfBuffer = 0x700; // SF_CTRL_BUF_BASE is 0x4000B700

// Commands of the SPI NOR flash the XT-ZB1 carries (KH25V80, JEDEC 0xC22014).
constexpr std::uint8_t CmdWriteDisable = 0x04, CmdReadStatus = 0x05, CmdWriteEnable = 0x06, CmdPageProgram = 0x02;
constexpr std::uint8_t CmdQuadPageProgram = 0x32, CmdReadId = 0x9f, CmdSectorErase = 0x20, CmdBlock32Erase = 0x52;
constexpr std::uint8_t CmdBlock64Erase = 0xd8, CmdChipErase = 0xc7, CmdChipErase2 = 0x60;
constexpr std::uint8_t CmdReadStatus2 = 0x35, CmdWriteStatus = 0x01;
constexpr std::uint8_t JedecId[3] = {0xc2, 0x20, 0x14};
constexpr std::uint32_t Sector = 4096;
} // namespace

SfCtrl::SfCtrl(Bl702 &chip) : RegisterBlock(chip, 0x4000'b000, 0x1000) {}

std::uint8_t SfCtrl::buffer(std::uint32_t index) { return static_cast<std::uint8_t>(RegisterBlock::read(SfBuffer + index, 1)); }

void SfCtrl::set_buffer(std::uint32_t index, std::uint8_t value) { RegisterBlock::write(SfBuffer + index, value, 1); }

std::uint32_t SfCtrl::read_word(std::uint32_t offset) {
    if (offset == SfId0Offset)
        return chip_.xip_offset_register();
    if (offset == SahbConfig)
        return word(offset) & ~(SahbBusy | SahbTrig); // done at once: never busy, the trigger clears
    return RegisterBlock::read_word(offset);
}

void SfCtrl::write_word(std::uint32_t offset, std::uint32_t value) {
    if (offset == SfId0Offset) {
        chip_.set_xip_offset_register(value);
        return;
    }
    if (offset == SahbConfig) {
        RegisterBlock::write_word(offset, value & ~SahbTrig);
        if (value & SahbTrig)
            run_command();
        return;
    }
    RegisterBlock::write_word(offset, value);
}

void SfCtrl::run_command() {
    const std::uint32_t config = word(SahbConfig);
    const std::uint8_t command = static_cast<std::uint8_t>(word(SahbCmd0) >> 24);
    const std::uint32_t addr_bytes = (config & SahbAddrEn) ? ((config >> SahbAddrBytePos) & 7) + 1 : 0;
    const std::uint32_t address = addr_bytes ? word(SahbCmd1) >> (32 - 8 * addr_bytes) : 0;
    const std::uint32_t length = (config & SahbDataEn) ? ((config >> SahbDataBytePos) & 0x3ff) + 1 : 0;
    const bool host_writes = (config & SahbDataRw) != 0;
    const std::int64_t size = chip_.flash_size();
    std::uint8_t *flash = chip_.flash();
    bool changed = false;

    switch (command) {
    case CmdReadId:
        for (std::uint32_t i = 0; i < length; ++i)
            set_buffer(i, i < 3 ? JedecId[i] : 0xff);
        break;
    case CmdReadStatus:
        set_buffer(0, write_enabled_ ? 2 : 0);
        break;
    case CmdReadStatus2:
        set_buffer(0, 0);
        break;
    case CmdWriteEnable:
        write_enabled_ = true;
        break;
    case CmdWriteDisable:
        write_enabled_ = false;
        break;
    case CmdWriteStatus:
        write_enabled_ = false; // the status register itself is not modelled
        break;
    case CmdPageProgram:
    case CmdQuadPageProgram:
        if (write_enabled_ && host_writes)
            for (std::uint32_t i = 0; i < length && address + i < size; ++i)
                flash[address + i] &= buffer(i); // NOR flash: programming only clears bits
        write_enabled_ = false;
        changed = true;
        break;
    case CmdSectorErase:
    case CmdBlock32Erase:
    case CmdBlock64Erase: {
        const std::uint32_t span = command == CmdSectorErase ? Sector : command == CmdBlock32Erase ? 32 * 1024 : 64 * 1024;
        if (write_enabled_ && address < size)
            std::memset(flash + (address & ~(span - 1)), 0xff, std::min<std::int64_t>(span, size - (address & ~(span - 1))));
        write_enabled_ = false;
        changed = true;
        break;
    }
    case CmdChipErase:
    case CmdChipErase2:
        if (write_enabled_)
            std::memset(flash, 0xff, size);
        write_enabled_ = false;
        changed = true;
        break;
    default:
        // Reads (0x03, 0x0b, 0x3b, 0x6b, 0xbb, 0xeb, ...) copy the array into the buffer; any other command
        // (reset, power-down release) has no visible effect here.
        if (!host_writes && length && address < size)
            for (std::uint32_t i = 0; i < length; ++i)
                set_buffer(i, address + i < size ? flash[address + i] : 0xff);
        break;
    }
    if (changed)
        chip_.flash_modified();
}

// ---- eFuse ----------------------------------------------------------------------------------------------
// The SDK reads the MAC from WIFI_MAC_LOW (0x14) and WIFI_MAC_HIGH (0x18), each little-endian: the chip ID is
// the eight bytes in that order (efuse.rs, decode_chip_id).

Efuse::Efuse(Bl702 &chip, const std::array<std::uint8_t, 8> &mac) : RegisterBlock(chip, 0x4000'7000, 0x1000) {
    set_word(0x14, little_endian(mac.data()));
    set_word(0x18, little_endian(mac.data() + 4));
}

// IF_CTRL_0 (0x800): AUTOLOAD_DONE is bit 1 and BUSY bit 2 (ef_ctrl_reg.h). The auto-load is instant here,
// so the controller reads as done and idle.
std::uint32_t Efuse::read_word(std::uint32_t offset) {
    constexpr std::uint32_t IfCtrl0 = 0x800, AutoloadDone = 1u << 1, Busy = 1u << 2;
    if (offset == IfCtrl0)
        return (RegisterBlock::read_word(offset) | AutoloadDone) & ~Busy;
    return RegisterBlock::read_word(offset);
}

// ---- TRNG (SEC_ENG) ------------------------------------------------------------------------------------
// Offsets relative to the block at 0x40004200 (SE_TRNG_0 at SEC_ENG + 0x200): CTRL 0x00, DOUT 0x08..0x24. The
// bits are the SDK's SEC_ENG_SE_TRNG_0_* masks (sec_eng_reg.h): TRIG bit 1, BUSY bit 0 (never set here).

namespace {
constexpr std::uint32_t TrngCtrl = 0x00, TrngDout = 0x08;
constexpr std::uint32_t TrngTrigger = 1u << 1;
} // namespace

Trng::Trng(Bl702 &chip) : RegisterBlock(chip, 0x4000'4200, 0x100) {}

std::uint32_t Trng::next() {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 17;
    state_ ^= state_ << 5;
    return state_;
}

void Trng::write_word(std::uint32_t offset, std::uint32_t value) {
    RegisterBlock::write_word(offset, value);
    if (offset == TrngCtrl && (value & TrngTrigger)) {
        for (std::uint32_t i = 0; i < 8; ++i)
            set_word(TrngDout + 4 * i, next());
    }
}

// ---- AES (SEC_ENG) --------------------------------------------------------------------------------------
// The register offsets are relative to the AES block at 0x40004100 (SEC_ENG + 0x100): CTRL 0x00, MSA 0x04
// (source), MDA 0x08 (destination), KEY 0x20..0x2c, CTRL_PROT 0xfc (0x400041fc).

namespace {
constexpr std::uint32_t AesCtrl = 0x00, AesSource = 0x04, AesDest = 0x08, AesKey = 0x20;
constexpr std::uint32_t AesTrigger = 1u << 1, AesEnable = 1u << 2, AesDecrypt = 1u << 5;
constexpr unsigned AesBlockBytes = 16;
} // namespace

Aes::Aes(Bl702 &chip) : RegisterBlock(chip, 0x4000'4100, 0x100) {}

std::uint32_t Aes::read_word(std::uint32_t offset) {
    if (offset == AesCtrl)
        return word(offset) & ~AesTrigger; // the trigger bit clears itself; BUSY stays 0
    return RegisterBlock::read_word(offset);
}

void Aes::write_word(std::uint32_t offset, std::uint32_t value) {
    if (offset != AesCtrl) {
        RegisterBlock::write_word(offset, value);
        return;
    }
    RegisterBlock::write_word(offset, value & ~AesTrigger);
    if ((value & AesTrigger) && (value & AesEnable))
        run(value);
}

void Aes::run(std::uint32_t ctrl) {
    const std::uint32_t blocks = (ctrl >> 16) & 0xffff;
    std::uint8_t key[16];
    for (unsigned i = 0; i < 4; ++i) {
        const std::uint32_t value = word(AesKey + 4 * i);
        for (unsigned b = 0; b < 4; ++b)
            key[4 * i + b] = static_cast<std::uint8_t>(value >> (8 * b));
    }
    const bool decrypt = (ctrl & AesDecrypt) != 0;
    for (std::uint32_t n = 0; n < blocks; ++n) {
        std::uint8_t in[AesBlockBytes], out[AesBlockBytes];
        if (!chip_.core().read_memory(word(AesSource) + n * AesBlockBytes, in, AesBlockBytes))
            return;
        mcuemu::aes_block(key, sizeof key, in, out, decrypt);
        if (!chip_.core().write_memory(word(AesDest) + n * AesBlockBytes, out, AesBlockBytes))
            return;
    }
}

} // namespace bl702emu
