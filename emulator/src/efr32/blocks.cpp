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

// ---- DPLL ------------------------------------------------------------------------

namespace {
constexpr std::uint32_t DpllEn = 0x004, DpllIf = 0x010, DpllStatus = 0x018;
} // namespace

std::uint32_t Dpll::read(std::uint32_t reg) {
    const bool enabled = regs_[DpllEn >> 2] & 1u;
    if (reg == DpllIf)
        return RegBlock::read(reg) | (enabled ? 1u : 0u);
    if (reg == DpllStatus)
        return enabled ? (1u | 2u | (1u << 31)) : 0u; // RDY, ENS, LOCK
    return RegBlock::read(reg);
}

// ---- USART -----------------------------------------------------------------------

namespace {
constexpr std::uint32_t UsartStatus = 0x018, UsartTxdataX = 0x038, UsartTxdata = 0x03c;
constexpr std::uint32_t UsartStatusTxc = 1u << 5, UsartStatusTxbl = 1u << 6, UsartStatusTxidle = 1u << 13;
} // namespace

std::uint32_t Usart::read(std::uint32_t reg) {
    if (reg == UsartStatus)
        return UsartStatusTxc | UsartStatusTxbl | UsartStatusTxidle;
    return RegBlock::read(reg);
}

void Usart::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == UsartTxdata || reg == UsartTxdataX) {
        chip().uart0_output().push_back(static_cast<char>(value & 0xffu));
        return;
    }
    RegBlock::write(reg, value);
}

// ---- SEMAILBOX host --------------------------------------------------------------

namespace {
constexpr std::uint32_t SeFifo = 0x000, SeTxStatus = 0x040, SeRxStatus = 0x044, SeTxHeader = 0x050,
                        SeRxHeader = 0x054;
constexpr std::uint32_t SeTxint = 1u << 20, SeRxint = 1u << 20;
} // namespace

std::uint32_t SeMailbox::read(std::uint32_t reg) {
    if (reg == SeTxStatus)
        return SeTxint; // room in the TX FIFO
    return RegBlock::read(reg);
}

void SeMailbox::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == SeFifo) {
        const std::uint32_t words = regs_[SeTxHeader >> 2] / 4;
        // The header counts the command's words, including one that is not
        // stored (the handle word), so the command is complete one word early.
        if (++words_ >= (words > 1 ? words - 1 : 1)) {
            words_ = 0;
            regs_[SeRxHeader >> 2] = 0; // SLI_SE_RESPONSE_OK
            regs_[SeRxStatus >> 2] |= SeRxint;
        }
        return;
    }
    if (reg == SeTxHeader)
        words_ = 0;
    RegBlock::write(reg, value);
}

void SeMailbox::reset() {
    RegBlock::reset();
    words_ = 0;
}

// ---- RAIL-driven radio registers -----------------------------------------------

namespace {
// RAC + 0x0C, state code in [27:24]. Inferred: 0 is idle (the loop after the RAC
// enable waits for it to be 0), and 7..9 and 12 are the active states that
// rfhal_rac accepts. No hook yet: the state follows what is stored.
constexpr std::uint32_t RacStatus = 0xA802000C;
// SEQRAM + 0x1EA0, polled with masks 0x800, 0x20000 and 0x40000. Inferred: these
// are "operation done" bits the sequencer would set. Forced set on every read
// for now, to see which waits they release.
constexpr std::uint32_t SeqStatus = 0xA0001EA0;
constexpr std::uint32_t SeqReadyBits = (1u << 11) | (1u << 17);

// RDMEM (0xA0000000) is RAM: no set/clear/toggle aliases. The register blocks
// (RAC, BUFC, ...) use the Series 2 aliases: +0x1000 set, +0x2000 clear, +0x3000 toggle.
bool is_rdmem(std::uint32_t address) { return address >= 0xA0000000u && address < 0xA0004000u; }
std::uint32_t register_base(std::uint32_t address) { return is_rdmem(address) ? address : address & ~0x3000u; }
} // namespace

std::uint32_t RailRegs::read(std::uint32_t address) {
    const std::uint32_t base = register_base(address);
    const auto it = words_.find(base);
    std::uint32_t value = it != words_.end() ? it->second : 0;
    if (address == SeqStatus)
        value |= SeqReadyBits;
    // RAC STATUS.STATE (bits 27:24): code 1 is RX (RAIL state 2), 0 is idle.
    if (base == RacStatus && rx_enabled())
        value = (value & ~(0xfu << 24)) | ((rx_frame_ ? 3u : 1u) << 24);
    return value;
}

std::uint32_t RailRegs::peek(std::uint32_t address) const {
    const auto it = words_.find(address);
    return it != words_.end() ? it->second : 0;
}

void RailRegs::set_bits(std::uint32_t address, std::uint32_t mask) { words_[address] |= mask; }

void RailRegs::clear_bits(std::uint32_t address, std::uint32_t mask) { words_[address] &= ~mask; }

void RailRegs::write(std::uint32_t address, std::uint32_t value, unsigned) {
    if (is_rdmem(address)) {
        words_[address] = value;
        return;
    }
    // PROTIMER CMD (image layout: header +0x1C moved to +0x0C): START (bit 0) and
    // RTCSYNCSTART (bit 1) set STATUS.RUNNING (+0x14 bit 0), STOP (bit 2) clears it.
    // The transmit path refuses to start while RUNNING is clear (sli_rail_3d0930...).
    if (register_base(address) == 0xA801C00Cu) {
        if (value & 0x3u)
            words_[0xA801C014u] |= 1u;
        if (value & 0x4u)
            words_[0xA801C014u] &= ~1u;
    }
    std::uint32_t &word = words_[register_base(address)];
    switch ((address >> 12) & 3u) {
    case 1: word |= value; break;
    case 2: word &= ~value; break;
    case 3: word ^= value; break;
    default: word = value; break;
    }
    // Inferred: RAC+0x008 bit 5 is a request that the hardware clears when the
    // operation is done. RAIL sets it and polls it to zero (rfhal_rac), so the
    // operation is treated as completing at once.
    if (register_base(address) == 0xA8020008u)
        word &= ~(1u << 5);
    // RAC CMD.TXEN starts the transmitter. The frame then ends on air after its
    // duration (the chip models that end, see Efr32mg21::service).
    if (register_base(address) == 0xA8020010u && (value & 1u))
        tx_started_ = true;
}

bool RailRegs::rx_enabled() const {
    const auto it = words_.find(0xA8020008u);
    return it != words_.end() && (it->second & 2u) != 0;
}

bool RailRegs::take_tx_start() {
    const bool started = tx_started_;
    tx_started_ = false;
    return started;
}

// ---- RTCC ------------------------------------------------------------------------

namespace {
constexpr std::uint32_t RtccEn = 0x004, RtccCfg = 0x008, RtccCmd = 0x00c, RtccIf = 0x014, RtccIen = 0x018,
                        RtccCnt = 0x020, RtccCombcnt = 0x024, RtccCc1Ctrl = 0x03c, RtccCc1Ocvalue = 0x040;
constexpr std::uint32_t RtccCmdStart = 1u << 0, RtccCmdStop = 1u << 1, RtccIfCc1 = 1u << 3;
constexpr std::uint64_t RtccWrap = 1ull << 32, RtccTickHz = 32768;
// CNTPRESC above DIV64 is not a valid setting; clamping keeps the cycle arithmetic in 64 bits.
constexpr unsigned RtccMaxPresc = 6;
} // namespace

bool Rtcc::counting() const { return started_ && (regs_[RtccEn >> 2] & 1u); }

std::uint64_t Rtcc::ticks(std::uint64_t at) const {
    if (!counting() || at <= cycle_at_)
        return ticks_at_;
    const unsigned presc = std::min<unsigned>((regs_[RtccCfg >> 2] >> 4) & 0xfu, RtccMaxPresc);
    return ticks_at_ + (((at - cycle_at_) * RtccTickHz / clock_hz()) >> presc);
}

// The first core cycle at which the counter reaches `tick` (rounded up, so the
// counter is there when the service runs).
std::uint64_t Rtcc::cycle_for(std::uint64_t tick) const {
    const unsigned presc = std::min<unsigned>((regs_[RtccCfg >> 2] >> 4) & 0xfu, RtccMaxPresc);
    const std::uint64_t cycles = ((tick - ticks_at_) * (clock_hz() << presc) + RtccTickHz - 1) / RtccTickHz;
    return cycle_at_ + cycles;
}

// Moves the base to the cycle where the counter reached its current value. The base
// used to move to `now`, so the fraction of a tick in progress was dropped at every
// freeze: the service runs every few thousand cycles and the counter lost about 40%.
void Rtcc::freeze() {
    const std::uint64_t n = now();
    const std::uint64_t t = ticks(n);
    const std::uint64_t anchor = std::min(cycle_for(t), n);
    ticks_at_ = t;
    cycle_at_ = anchor;
}

void Rtcc::arm() {
    if (!counting() || (regs_[RtccCc1Ctrl >> 2] & 3u) != 2u) { // OUTPUTCOMPARE
        armed_tick_ = Never;
        return;
    }
    // Match when the counter reaches OCVALUE + 1 (32-bit). A target equal to the
    // current count is taken as a full wrap away, not as an immediate match.
    const auto target = static_cast<std::uint32_t>(regs_[RtccCc1Ocvalue >> 2] + 1u);
    std::uint64_t delta = static_cast<std::uint32_t>(target - static_cast<std::uint32_t>(ticks_at_));
    if (delta == 0)
        delta = RtccWrap;
    armed_tick_ = ticks_at_ + delta;
    chip().request_service(cycle_for(armed_tick_));
}

void Rtcc::update_irq() { set_irq_level(Irq, (regs_[RtccIf >> 2] & regs_[RtccIen >> 2]) != 0); }

std::uint32_t Rtcc::read(std::uint32_t reg) {
    if (reg == RtccCnt || reg == RtccCombcnt)
        return static_cast<std::uint32_t>(ticks(now()));
    return RegBlock::read(reg);
}

void Rtcc::write(std::uint32_t reg, std::uint32_t value) {
    switch (reg) {
    case RtccCmd:
        freeze();
        if (value & RtccCmdStart)
            started_ = true;
        if (value & RtccCmdStop)
            started_ = false;
        arm();
        return;
    case RtccCnt:
        freeze();
        ticks_at_ = value;
        arm();
        return;
    case RtccEn:
    case RtccCfg:
    case RtccCc1Ctrl:
    case RtccCc1Ocvalue:
        freeze();
        RegBlock::write(reg, value);
        arm();
        return;
    case RtccIf:
    case RtccIen:
        RegBlock::write(reg, value);
        update_irq();
        return;
    default:
        RegBlock::write(reg, value);
        return;
    }
}

std::uint64_t Rtcc::service(std::uint64_t) {
    if (!counting() || armed_tick_ == Never)
        return Never;
    freeze();
    if (ticks_at_ >= armed_tick_) {
        regs_[RtccIf >> 2] |= RtccIfCc1;
        do {
            armed_tick_ += RtccWrap;
        } while (armed_tick_ <= ticks_at_);
        update_irq();
    }
    return cycle_for(armed_tick_);
}

void Rtcc::reset() {
    RegBlock::reset();
    ticks_at_ = 0;
    cycle_at_ = 0;
    armed_tick_ = Never;
    started_ = false;
    update_irq();
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
