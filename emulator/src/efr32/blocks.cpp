// CMU, HFXO, MSC and GPIO of the EFR32MG21 (Series 2). Sources: the EFR32xG21
// reference manual register layouts as used by the zigbee-rs HAL
// (efr32mg21-hal: clock.rs, flash.rs, gpio.rs). What the manual does not
// settle is marked "inferred".

#include "efr32/blocks.hpp"
#include "crypto/aes128.hpp"
#include <algorithm>
#include "efr32/rail_map.hpp"

#include <algorithm>
#include <cstring>

namespace efr32emu {

std::uint64_t Block::now() const { return chip_.core().cycles(); }
std::uint64_t Block::clock_hz() const { return chip_.core().clock_hz(); }
void Block::set_irq_level(unsigned irq, bool level) { chip_.core().set_irq_line(irq, level); }

// ---- CMU, HFXO -----------------------------------------------------------------

namespace {
constexpr std::uint32_t LdmaChen = 0x24, LdmaChdis = 0x28, LdmaChstatus = 0x2c, LdmaChdone = 0x34,
                        LdmaReqdis = 0x40, LdmaLinkload = 0x48, LdmaIf = 0x50, LdmaIen = 0x54;
constexpr std::uint32_t ChCtrl = 0x08, ChSrc = 0x0c, ChDst = 0x10, ChLink = 0x14;
// RXDATA (+0x24) and RXDATAX (+0x20) of USART0 in the non-secure and secure views.
bool is_usart0_rxdata(std::uint32_t address) {
    const std::uint32_t reg = address & 0xfffu;
    return (address & ~0xfffu) % 0x10000000u == 0x58000u && (reg == 0x24 || reg == 0x20);
}
} // namespace

void Ldma::reset() {
    RegBlock::reset();
    enabled_ = 0;
    update_irq();
}

std::uint32_t Ldma::read(std::uint32_t reg) {
    if (reg == LdmaChstatus)
        return enabled_;
    return RegBlock::read(reg);
}

void Ldma::update_irq() { set_irq_level(Irq, (regs_[LdmaIf >> 2] & regs_[LdmaIen >> 2]) != 0); }

void Ldma::load_descriptor(unsigned ch, std::uint32_t address) {
    std::uint32_t words[4] = {0, 0, 0, 0};
    if (!chip().core().read_memory(address, words, sizeof words))
        return;
    desc_addr_[ch] = address;
    set_channel_reg(ch, ChCtrl, words[0]);
    set_channel_reg(ch, ChSrc, words[1]);
    set_channel_reg(ch, ChDst, words[2]);
    set_channel_reg(ch, ChLink, words[3]);
}

void Ldma::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == LdmaChdis) {
        enabled_ &= ~value;
        return;
    }
    if (reg == LdmaChen) {
        enabled_ |= value;
        return;
    }
    if (reg == LdmaLinkload) {
        for (unsigned ch = 0; ch < Channels; ++ch) {
            if (!(value & (1u << ch)))
                continue;
            // The NCP's UART driver parks the receive channel on a "dummy" descriptor (source and
            // destination the same RAM byte, linked back to the receive descriptor) to freeze the
            // write pointer, and tells "ring empty" by DST == that RAM byte. So a link load takes
            // effect at once, even for an enabled channel. LINK: bit 1 = link valid, bit 0 =
            // relative address (xG21 reference manual, CHx_LINK; the firmware tests bit 1).
            load_descriptor(ch, channel_reg(ch, ChLink) & ~3u);
            enabled_ |= 1u << ch;
        }
        return;
    }
    RegBlock::write(reg, value);
    if (reg == LdmaIf || reg == LdmaIen)
        update_irq();
}

// One transfer of the descriptor on `ch` has happened; completes it when the count runs out.
void Ldma::transfer_done(unsigned ch) {
    std::uint32_t ctrl = channel_reg(ch, ChCtrl);
    const std::uint32_t count = (ctrl >> 4) & 0x7ffu;
    if (count != 0) {
        set_channel_reg(ch, ChCtrl, (ctrl & ~(0x7ffu << 4)) | ((count - 1) << 4));
        return;
    }
    if (ctrl & (1u << 20))
        regs_[LdmaIf >> 2] |= 1u << ch;
    const std::uint32_t link = channel_reg(ch, ChLink);
    if (link & 2u) {
        std::uint32_t address = link & ~3u;
        if (link & 1u)
            address += desc_addr_[ch];
        load_descriptor(ch, address);
    } else {
        // CHDONE means the whole chain is finished, not one descriptor: the NCP's UART driver
        // rebuilds "the rest of the running descriptor" from XFERCNT unless CHDONE is set, and a
        // CHDONE raised by the parking descriptor made it build a 2048-byte one that overran the ring.
        regs_[LdmaChdone >> 2] |= 1u << ch;
        enabled_ &= ~(1u << ch);
    }
    update_irq();
}

bool Ldma::usart0_rx(std::uint8_t byte) {
    for (unsigned ch = 0; ch < Channels; ++ch) {
        // REQDIS masks a channel's peripheral requests: the NCP's driver sets it around its
        // read-pointer snapshot, and the byte waits in the USART until it is cleared again.
        if (!(enabled_ & (1u << ch)) || (regs_[LdmaReqdis >> 2] & (1u << ch)))
            continue;
        // A memory-to-memory no-op descriptor (the parking one) takes the request without
        // touching RXDATA; the byte stays pending for the descriptor it links to.
        for (unsigned guard = 0; guard < 4 && (enabled_ & (1u << ch)) &&
                                 channel_reg(ch, ChSrc) == channel_reg(ch, ChDst);
             ++guard)
            transfer_done(ch);
        if (!(enabled_ & (1u << ch)) || !is_usart0_rxdata(channel_reg(ch, ChSrc)))
            continue;
        const std::uint32_t ctrl = channel_reg(ch, ChCtrl);
        const std::uint32_t dst = channel_reg(ch, ChDst);
        chip().core().write_memory(dst, &byte, 1);
        if (((ctrl >> 28) & 3u) == 0) // DSTINC: one byte
            set_channel_reg(ch, ChDst, dst + 1);
        transfer_done(ch);
        return true;
    }
    return false;
}

void Emu::reset() {
    RegBlock::reset();
    regs_[Rstcause >> 2] = chip().soft_reset() ? SystemRequestReset : PowerOnReset;
}

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
constexpr std::size_t UartOutputLimit = 1u << 20;
constexpr std::uint32_t UsartStatusTxc = 1u << 5, UsartStatusTxbl = 1u << 6, UsartStatusTxidle = 1u << 13;
} // namespace

std::uint32_t Usart::read(std::uint32_t reg) {
    if (reg == UsartStatus)
        return UsartStatusTxc | UsartStatusTxbl | UsartStatusTxidle;
    return RegBlock::read(reg);
}

void Usart::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == UsartTxdata || reg == UsartTxdataX) {
        // A run that nobody drains stops recording instead of growing without bound.
        if (chip().uart0_output().size() < UartOutputLimit)
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
        fifo_.push_back(value);
        // The header counts the command's words, including one that is not
        // stored (the handle word), so the command is complete one word early.
        if (++words_ >= (words > 1 ? words - 1 : 1)) {
            words_ = 0;
            execute();
            fifo_.clear();
            regs_[SeRxHeader >> 2] = 0; // SLI_SE_RESPONSE_OK
            regs_[SeRxStatus >> 2] |= SeRxint;
        }
        return;
    }
    if (reg == SeTxHeader) {
        words_ = 0;
        fifo_.clear();
    }
    RegBlock::write(reg, value);
}

// The command stream is [command, data_in chain, data_out chain, parameters...] (sli_se_manager_mailbox.c).
// A chain element is {address, next, length}; next == 1 ends it (SLI_SE_DATATRANSFER_STOP, inferred from the
// NCP's descriptors) and the length word carries flag bits above bit 27. AES-ECB (0x04000100 encrypt, 0x04010100 decrypt, inferred from the NCP) takes
// the plain key as the first input element and the data as the last, and writes the result to the output.
void SeMailbox::execute() {
    if (fifo_.size() < 3 || (fifo_[0] != 0x04000100u))
        return; // only AES-ECB encryption is modelled; other commands just succeed
    struct Element {
        std::uint32_t address, length;
    };
    auto chain = [&](std::uint32_t head) {
        std::vector<Element> elements;
        for (unsigned guard = 0; guard < 6 && head != 0; ++guard) {
            std::uint32_t d[3] = {};
            if (!chip().core().read_memory(head, d, sizeof d))
                break;
            elements.push_back({d[0], d[2] & 0x0fffffffu});
            if ((d[1] & 1u) != 0 || d[1] == 0) // next == 1 is SLI_SE_DATATRANSFER_STOP
                break;
            head = d[1];
        }
        return elements;
    };
    const auto in = chain(fifo_[1]);
    const auto out = chain(fifo_[2]);
    if (in.size() < 2 || out.empty() || in.back().length == 0 || in.back().length % 16 != 0)
        return;
    const Element &key = in[in.size() - 2];
    if (key.length != 16)
        return;
    std::uint8_t key_bytes[16];
    std::vector<std::uint8_t> data(in.back().length);
    if (!chip().core().read_memory(key.address, key_bytes, 16) ||
        !chip().core().read_memory(in.back().address, data.data(), data.size()))
        return;
    crypto::Block k;
    std::copy(key_bytes, key_bytes + 16, k.begin());
    for (std::size_t offset = 0; offset < data.size(); offset += 16) {
        crypto::Block block;
        std::copy(data.begin() + offset, data.begin() + offset + 16, block.begin());
        const crypto::Block encrypted = crypto::aes128_encrypt(k, block);
        std::copy(encrypted.begin(), encrypted.end(), data.begin() + offset);
    }
    chip().core().write_memory(out[0].address, data.data(), std::min<std::size_t>(data.size(), out[0].length));
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
using rail::RacStatus;
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

namespace {
constexpr std::uint32_t IadcCmd = 0x0c, IadcStatus = 0x14, IadcIf = 0x24, IadcSingleFifoData = 0x74,
                        IadcSingleData = 0x7c, IadcScanFifoData = 0x84, IadcScanData = 0x8c;
constexpr std::uint32_t IadcResult = 0x8000;
} // namespace

std::uint32_t Iadc::peek(std::uint32_t reg) {
    if (reg == IadcStatus)
        return RegBlock::read(reg) | (single_dv_ ? 0x100u : 0u) | (scan_dv_ ? 0x200u : 0u);
    return RegBlock::read(reg);
}

std::uint32_t Iadc::read(std::uint32_t reg) {
    switch (reg) {
    case IadcSingleFifoData:
        single_dv_ = false;
        return IadcResult;
    case IadcScanFifoData:
        scan_dv_ = false;
        return IadcResult;
    case IadcSingleData:
    case IadcScanData:
        return IadcResult;
    default: return peek(reg);
    }
}

void Iadc::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == IadcCmd) {
        if (value & 1u) {
            single_dv_ = true;
            regs_[IadcIf >> 2] |= (1u << 9) | 1u;
        }
        if (value & 8u) {
            scan_dv_ = true;
            regs_[IadcIf >> 2] |= (1u << 8) | 2u;
        }
        return;
    }
    if (reg == IadcStatus)
        return;
    RegBlock::write(reg, value);
}

std::uint32_t RailRegs::read(std::uint32_t address) {
    const std::uint32_t base = register_base(address);
    const auto it = words_.find(base);
    std::uint32_t value = it != words_.end() ? it->second : 0;
    if (address == SeqStatus)
        value |= SeqReadyBits;
    // BUFC BUF0 STATUS (+0x28): the byte count in bits 12:0, WRITEOFFSET - READOFFSET. INFERRED from the
    // Tuya ZS3L image, whose RAIL peeks byte 3 or 4 of the TX frame in BUF0 before it starts a transmit and
    // fails with EMBER_PHY_TX_BUSY (0x8C) when the count is zero: the frame is copied into the buffer RAM and
    // WRITEOFFSET set to its length, so the count has to follow the offsets.
    if (address == rail::Buf0Status && it == words_.end()) {
        const auto count = [&](std::uint32_t reg) {
            const auto word = words_.find(reg);
            return word != words_.end() ? word->second : 0u;
        };
        value = (count(rail::Buf0WriteOffset) - count(rail::Buf0ReadOffset)) & 0x1fffu;
    }
    // INFERRED: a Tuya EFR32MG21 image (ZS3L thermometer) reads 0xA801803C before the radio configuration
    // table is applied and multiplies its three 3-bit fields (the synthesizer divider); a product of zero
    // makes it assert (udf at 0xef3c). The NCP writes 1 to this word, so an unwritten word reads as 1.
    if (address == 0xA801803C && it == words_.end())
        value = 1;
    // INFERRED stand-ins, not the silicon: the RAIL init in the Silicon Labs Zigbee NCP 8.2.2 image
    // (pc 0x2e2f4) polls FRC IF (0xA8004068) for bit 14 and FRC+0x08 for bit 8 after it starts a
    // calibration that the RAC sequencer would run (this model does not run it). Reporting both as
    // set lets the init finish; what the bits mean on the silicon is not known here.
    if (address == 0xA8004068)
        value |= 0x4000;
    if (address == 0xA8004008)
        value |= 0x100;
    // RAC STATUS.STATE (bits 27:24): code 1 is RX (RAIL state 2), 0 is idle.
    // INFERRED from the NCP's RAIL_GetRssi (energy scan, scan RX): once the receiver is in RXSEARCH or
    // RXFRAME it waits for a nonzero AGC STATUS1.RSSISTATE (bits 17:15) and then reads AGC RSSI
    // (RSSIINT bits 15:8, signed dBm). The model reports a valid state (3 is a guess; any nonzero
    // passes the test) and a quiet -100 dBm channel.
    if (base == rail::AgcStatus1 && (rx_ready_ || rx_frame_))
        value |= 3u << 15;
    if (base == rail::AgcRssi)
        value = (rx_ready_ || rx_frame_) ? (0x9cu << 8) : (0x80u << 8);
    if (base == RacStatus && (rx_enabled() || rx_frame_hold_))
        value = (value & ~(0xfu << 24)) | (((rx_frame_ || rx_frame_hold_) ? 3u : rx_ready_ ? 2u : 1u) << 24);
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
    if (register_base(address) == rail::ProtimerCmd) {
        if (value & 0x3u)
            words_[rail::ProtimerStatus] |= 1u;
        if (value & 0x4u)
            words_[rail::ProtimerStatus] &= ~1u;
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
    if (register_base(address) == rail::RacRxEnable)
        word &= ~(1u << 5);
    // RAC CMD.TXEN starts the transmitter. The frame then ends on air after its
    // duration (the chip models that end, see Efr32mg21::service).
    if (register_base(address) == rail::RacCmd && (value & 1u))
        tx_started_ = true;
}

bool RailRegs::rx_enabled() const {
    const auto it = words_.find(rail::RacRxEnable);
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
constexpr unsigned ExtiLines = 8, GpioEvenIrq = 26, GpioOddIrq = 25; // GPIO_EVEN_IRQn / GPIO_ODD_IRQn in efr32mg21*.h
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
