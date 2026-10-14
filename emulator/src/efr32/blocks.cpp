// CMU, HFXO, MSC and GPIO of the EFR32MG21 (Series 2). Sources: the EFR32xG21
// reference manual register layouts as used by the zigbee-rs HAL
// (efr32mg21-hal: clock.rs, flash.rs, gpio.rs). What the manual does not
// settle is marked "inferred".

#include "efr32/blocks.hpp"

#include <algorithm>
#include <cstring>

namespace efr32emu {

std::uint64_t Block::now() const { return chip_.core().cycles(); }
std::uint64_t Block::clock_hz() const { return chip_.core().clock_hz(); }
void Block::set_irq_level(unsigned irq, bool level) { chip_.core().set_irq_line(irq, level); }

// ---- CMU, HFXO -----------------------------------------------------------------

void Cmu::reset() {
    RegBlock::reset();
    regs_[Sysclkctrl >> 2] = 1; // FSRCO selected out of reset
}

namespace {
constexpr std::uint32_t HfxoCtrl = 0x028, HfxoXtalctrl = 0x018, HfxoStatus = 0x058;
constexpr std::uint32_t ClkselHfxo = 3;
} // namespace

std::uint32_t Hfxo::read(std::uint32_t reg) {
    if (reg != HfxoStatus)
        return RegBlock::read(reg);
    // CTRL.FORCEEN keeps the oscillator on; otherwise it runs on demand while
    // it is the system clock unless DISONDEMAND is set. Start-up takes no time
    // (inferred). FSMLOCK is the first-lock optimisation: on while the
    // oscillator runs with DISONDEMAND clear and the core-bias optimisation
    // has not been skipped (inferred from how the HAL waits on it).
    const std::uint32_t ctrl = regs_[HfxoCtrl >> 2];
    const bool force = ctrl & 1u, disondemand = ctrl & 2u;
    const bool enabled = force || (!disondemand && cmu_.clock_select() == ClkselHfxo);
    std::uint32_t status = 0;
    if (enabled)
        status |= (1u << 0) | (1u << 1) | (1u << 16);
    if (enabled && !disondemand && !(regs_[HfxoXtalctrl >> 2] >> 31))
        status |= 1u << 30;
    return status;
}

// ---- MSC -------------------------------------------------------------------------

namespace {
constexpr std::uint32_t MscWritectrl = 0x00c, MscWritecmd = 0x010, MscAddrb = 0x014, MscWdata = 0x018,
                        MscStatus = 0x01c, MscLock = 0x03c;
constexpr std::uint32_t PageSize = 8192;
} // namespace

void Msc::reset() {
    RegBlock::reset();
    unlocked_ = invalid_address_ = false;
    address_ = 0;
}

std::uint32_t Msc::read(std::uint32_t reg) {
    if (reg == MscStatus) {
        std::uint32_t status = 0;
        if (invalid_address_)
            status |= 1u << 2;
        if (unlocked_ && (regs_[MscWritectrl >> 2] & 1u))
            status |= 1u << 3; // WDATAREADY: programming is instantaneous
        if (!unlocked_)
            status |= 1u << 16; // REGLOCK
        return status;
    }
    if (reg == MscLock)
        return unlocked_ ? 0 : 1;
    return RegBlock::read(reg);
}

void Msc::write(std::uint32_t reg, std::uint32_t value) {
    switch (reg) {
    case MscLock:
        unlocked_ = value == 0x1b71;
        return;
    case MscAddrb:
        if (!unlocked_)
            return;
        address_ = value;
        invalid_address_ = value >= Efr32mg21::FlashSize;
        return;
    case MscWritecmd:
        if (!unlocked_ || !(regs_[MscWritectrl >> 2] & 1u))
            return;
        if ((value & 2u) && !invalid_address_) { // ERASEPAGE
            const std::uint32_t page = address_ & ~(PageSize - 1);
            std::memset(chip_.flash() + page, 0xff, PageSize);
            chip_.flash_changed(page, PageSize);
        }
        return;
    case MscWdata:
        if (!unlocked_ || !(regs_[MscWritectrl >> 2] & 1u) || invalid_address_)
            return;
        // Flash programming can only clear bits.
        std::uint32_t old;
        std::memcpy(&old, chip_.flash() + address_, 4);
        old &= value;
        std::memcpy(chip_.flash() + address_, &old, 4);
        chip_.flash_changed(address_, 4);
        address_ += 4;
        invalid_address_ = address_ >= Efr32mg21::FlashSize;
        return;
    default:
        RegBlock::write(reg, value);
    }
}

// ---- GPIO ------------------------------------------------------------------------

namespace {
constexpr std::uint32_t PortStride = 0x30;
constexpr std::uint32_t RegCtrl = 0x00, RegModel = 0x04, RegModeh = 0x0c, RegDout = 0x10, RegDin = 0x14;
constexpr std::uint32_t RegLock = 0x300, RegExtipsel = 0x400, RegExtipinsel = 0x408, RegExtirise = 0x410,
                        RegExtifall = 0x414, RegIf = 0x420, RegIen = 0x424;
constexpr unsigned ExtiLines = 8, GpioEvenIrq = 3, GpioOddIrq = 13;
} // namespace

void Gpio::reset() {
    modes_ = {};
    dout_ = {};
    ctrl_ = {};
    extipsel_ = extipinsel_ = extirise_ = extifall_ = if_ = ien_ = 0;
    for (unsigned pad = 0; pad != 64; ++pad)
        last_[pad] = level(pad);
    update_irq();
}

PadDrive Gpio::drive(unsigned pad) const {
    const bool high = dout(pad);
    // Modes: 0 disabled, 1 input, 2/3 input with pull (DOUT picks the
    // direction), 4/5 push-pull, 6 wired-or, 7 wired-or with pull-down,
    // 8..13 wired-and (open drain; 10/11 with pull-up).
    switch (mode(pad)) {
    case 2:
    case 3: return high ? PadDrive::PullUp : PadDrive::PullDown;
    case 4:
    case 5: return high ? PadDrive::High : PadDrive::Low;
    case 6: return high ? PadDrive::High : PadDrive::HiZ;
    case 7: return high ? PadDrive::High : PadDrive::PullDown;
    case 8:
    case 9:
    case 12:
    case 13: return high ? PadDrive::HiZ : PadDrive::Low;
    case 10:
    case 11: return high ? PadDrive::PullUp : PadDrive::Low;
    default: return PadDrive::HiZ;
    }
}

bool Gpio::level(unsigned pad) const {
    if (pad >= 64)
        return false;
    if (external_valid_[pad])
        return external_[pad];
    switch (drive(pad)) {
    case PadDrive::High:
    case PadDrive::PullUp: return true;
    default: return false;
    }
}

void Gpio::set_external(unsigned pad, bool high) {
    if (pad >= 64)
        return;
    external_valid_[pad] = true;
    external_[pad] = high;
    refresh_exti(pad, last_[pad], level(pad));
    last_[pad] = level(pad);
}

void Gpio::changed(unsigned pad) {
    chip_.notify_pad(pad);
    const bool after = level(pad);
    refresh_exti(pad, last_[pad], after);
    last_[pad] = after;
}

void Gpio::refresh_exti(unsigned pad, bool before, bool after) {
    if (before == after)
        return;
    for (unsigned line = 0; line != ExtiLines; ++line) {
        const unsigned port = (extipsel_ >> (4 * line)) & 3u;
        // Inferred: pin select is a 2-bit offset inside the line's group of
        // four pins (lines 0..3 reach pins 0..3, lines 4..7 reach pins 4..7).
        const unsigned selected = ((extipinsel_ >> (4 * line)) & 3u) + 4 * (line / 4);
        if (port * 16 + selected != pad)
            continue;
        if ((after && ((extirise_ >> line) & 1u)) || (!after && ((extifall_ >> line) & 1u)))
            if_ |= 1u << line;
    }
    update_irq();
}

void Gpio::update_irq() {
    const std::uint32_t pending = if_ & ien_ & ((1u << ExtiLines) - 1);
    set_irq_level(GpioEvenIrq, (pending & 0x55u) != 0);
    set_irq_level(GpioOddIrq, (pending & 0xaau) != 0);
}

std::uint32_t Gpio::read(std::uint32_t reg) {
    if (reg < PortCount * PortStride) {
        const unsigned port = reg / PortStride;
        switch (reg % PortStride) {
        case RegCtrl: return ctrl_[port];
        case RegModel: return modes_[port][0];
        case RegModeh: return modes_[port][1];
        case RegDout: return dout_[port];
        case RegDin: {
            std::uint32_t din = 0;
            for (unsigned pin = 0; pin != 16; ++pin)
                if (mode(port * 16 + pin) != 0 && level(port * 16 + pin))
                    din |= 1u << pin;
            return din;
        }
        default: return 0;
        }
    }
    switch (reg) {
    case RegLock: return 0;
    case RegExtipsel: return extipsel_;
    case RegExtipinsel: return extipinsel_;
    case RegExtirise: return extirise_;
    case RegExtifall: return extifall_;
    case RegIf: return if_;
    case RegIen: return ien_;
    default: return 0;
    }
}

void Gpio::write(std::uint32_t reg, std::uint32_t value) {
    if (reg < PortCount * PortStride) {
        const unsigned port = reg / PortStride;
        switch (reg % PortStride) {
        case RegCtrl: ctrl_[port] = value; break;
        case RegModel:
        case RegModeh: {
            std::uint32_t &word = modes_[port][(reg % PortStride) == RegModel ? 0 : 1];
            const std::uint32_t before = word;
            word = value;
            for (unsigned pin = 0; pin != 8; ++pin)
                if (((before ^ value) >> (4 * pin)) & 0xfu)
                    changed(port * 16 + pin + ((reg % PortStride) == RegModel ? 0 : 8));
            break;
        }
        case RegDout: {
            const std::uint32_t before = dout_[port];
            dout_[port] = value & 0xffffu;
            for (unsigned pin = 0; pin != 16; ++pin)
                if (((before ^ dout_[port]) >> pin) & 1u)
                    changed(port * 16 + pin);
            break;
        }
        default: break;
        }
        return;
    }
    switch (reg) {
    case RegExtipsel: extipsel_ = value; break;
    case RegExtipinsel: extipinsel_ = value; break;
    case RegExtirise: extirise_ = value; break;
    case RegExtifall: extifall_ = value; break;
    case RegIf:
        if_ = value;
        update_irq();
        break;
    case RegIen:
        ien_ = value;
        update_irq();
        break;
    default: break; // LOCK and the rest
    }
}

} // namespace efr32emu
