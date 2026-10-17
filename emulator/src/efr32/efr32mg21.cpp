#include "efr32emu/efr32mg21.hpp"

#include "efr32/blocks.hpp"
#include "efr32/rail_map.hpp"

#include <algorithm>
#include <cstring>
#include <ostream>
#include <stdexcept>

namespace efr32emu {

namespace {

constexpr std::uint32_t RadioBase = 0x40090000, RadioEnd = 0x40096000;
// Peripheral blocks are 16 KiB apart; slot = (offset from 0x40000000) / 0x4000.
constexpr unsigned CmuSlot = 2, HfxoSlot = 3, DpllSlot = 7, UsartSlot = 22, MscSlot = 12, GpioSlot = 15;
constexpr std::uint32_t RtccBase = 0x48000000, RtccSize = 0x4000;
constexpr unsigned ProtimerIrq = 36; // PROTIMER_IRQn
constexpr unsigned FrcIrq = 34, RacSeqIrq = 38; // FRC_IRQn, RAC_SEQ_IRQn
// FRC IF bit 15: TX_PACKET_SENT; RAC IF bit 22: RX_ACK_TIMEOUT.
constexpr std::uint32_t FrcTxPacketSent = 1u << 15, RacRxAckTimeout = 1u << 22;
// BUFC IF/IEN: IRQ 32 (BUFC_IRQn) follows IF & IEN; the BUF2 threshold is bit 18.
constexpr unsigned BufcIrq = 32;
using namespace rail;
// The RX poll rate: 100 us at 38.4 MHz, like the PROTIMER compare check.
constexpr std::uint64_t RailRxPollCycles = 3840;
// Inferred: the 802.15.4 ACK wait of 54 symbols (864 us).
constexpr std::uint64_t AckWaitCycles = 33178;
constexpr std::uint32_t SeBase = 0x4C000000, SeSize = 0x1000;

// 0x5xxxxxxx (non-secure alias) addresses the same register as 0x4xxxxxxx.
std::uint32_t secure_alias(std::uint32_t address) {
    // 0x5xxxxxxx and 0xBxxxxxxx (RDMEM, PRORTC/RAC) are the non-secure views of 0x4/0xA.
    const std::uint32_t top = address & 0xf0000000u;
    return (top == 0x50000000u || top == 0xB0000000u) ? address - 0x10000000u : address;
}

// Secure (0xA0, 0xA8, 0x44) addresses after secure_alias(): RDMEM, RAC and BUFC_S.
bool is_rail_register(std::uint32_t address) {
    return (address >= 0xA0000000u && address < 0xA0010000u) || (address >= 0xA8000000u && address < 0xA8040000u) ||
           (address >= 0x44000000u && address < 0x44010000u);
}

// `offset` is the address inside the block, with the Series 2 alias bits:
// +0x1000 sets, +0x2000 clears, +0x3000 toggles the bits written.
void write_register(Block &block, std::uint32_t offset, std::uint32_t value) {
    const std::uint32_t reg = offset & 0xffc;
    switch ((offset >> 12) & 3u) {
    case 1: block.write(reg, block.peek(reg) | value); break;
    case 2: block.write(reg, block.peek(reg) & ~value); break;
    case 3: block.write(reg, block.peek(reg) ^ value); break;
    default: block.write(reg, value); break;
    }
}

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
    // Peripherals the model does not implement, read and written as plain storage
    // (the emulator has one security state, so the secure-only blocks need no
    // separate behaviour):
    //   0x5A014000  HFRCOEM23_NS (written by CMU_HFRCOEM23BandSet)
    struct StandIn {
        std::uint32_t base, size;
    };
    for (const StandIn &stand_in : {StandIn{0x5A014000, 0x4000}}) {
        std::uint8_t *storage = core_->map_memory(stand_in.base, stand_in.size, true);
        std::memset(storage, 0, stand_in.size);
    }
    // The RAIL radio blocks (BUFC, RAC, sequencer/FRC RAM) go through RailRegs.
    rail_regs_ = std::make_unique<RailRegs>();
    std::memset(ram_, 0, RamSize);
    std::memset(userdata_, 0xff, 0x1000);
    std::memset(devinfo_, 0xff, 0x1000);
    set_device_id(0x0123456789abcdefull);

    auto cmu = std::make_unique<Cmu>(*this);
    cmu_ = cmu.get();
    blocks_[CmuSlot] = std::move(cmu);
    blocks_[HfxoSlot] = std::make_unique<Hfxo>(*this, *cmu_);
    blocks_[DpllSlot] = std::make_unique<Dpll>(*this);
    blocks_[UsartSlot] = std::make_unique<Usart>(*this);
    blocks_[MscSlot] = std::make_unique<Msc>(*this);
    auto gpio = std::make_unique<Gpio>(*this);
    gpio_ = gpio.get();
    blocks_[GpioSlot] = std::move(gpio);
    radio_ = std::make_unique<Radio>(*this);
    rtcc_ = std::make_unique<Rtcc>(*this);
    se_mailbox_ = std::make_unique<SeMailbox>(*this);

    core_->map_mmio(PeripheralBase, PeripheralSize, *this);
    // RTCC_S and RTCC_NS sit outside the peripheral window.
    core_->map_mmio(RtccBase, RtccSize, *this);
    core_->map_mmio(0x58000000, RtccSize, *this);
    // SEMAILBOX host, secure and non-secure (the NS alias folds onto the secure one).
    core_->map_mmio(SeBase, SeSize, *this);
    core_->map_mmio(0x5C000000, SeSize, *this);
    core_->map_mmio(0x44000000, 0x10000, *this);
    core_->map_mmio(0x54000000, 0x10000, *this);
    core_->map_mmio(0xB0000000, 0x10000, *this);
    core_->map_mmio(0xB8000000, 0x40000, *this);
    // The non-secure alias (CMU_NS 0x50008000, SYSCFG_NS 0x5007C000, ...) is the
    // same register block as the 0x4xxxxxxx one; the offset mask in read/write
    // folds both onto it.
    core_->map_mmio(0x50000000, PeripheralSize, *this);
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
    // DEVINFO.UNIQUEL/H (+0x48, +0x4C) are what SYSTEM_GetUnique returns. Zephyr's hwinfo
    // device id, and with it the 802.15.4 driver's address, comes from these words; with
    // them erased the router's address was ff..ff and the coordinator's ACKs went to it.
    for (unsigned i = 0; i < 8; ++i)
        devinfo_[0x48 + i] = static_cast<std::uint8_t>(id >> (8 * i));
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
        throw std::runtime_error("firmware does not fit in the 1 MiB flash");
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
    rtcc_->reset();
    se_mailbox_->reset();
    rail_regs_->reset();
    tx_done_at_ = ack_timeout_at_ = NoEvent;
    // short_address_known_ is deliberately kept: the driver's short address and PAN live in RAIL's
    // RAM handle, which the model does not read, so the copy learned from the association response
    // stands in for them. A firmware that resets after joining (the Zephyr ED does, at about 10 s)
    // restores its address from non-volatile storage and would otherwise lose every ACK.
    rail_rx_poll_at_ = 0;
    rx_context_ = 0;
    next_protimer_ = 0;
    std::fill(std::begin(protimer_last_wrap_), std::end(protimer_last_wrap_), 0);
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

// The frame RAIL sent is in the BUF0 buffer RAM (BUF0 ADDR, +0x0C) from the start
// up to PHR + 1 bytes. The firmware's FIFO writer stores the first bytes through
// WRITEDATA and copies the rest into the same RAM (see tx_store).
std::vector<std::uint8_t> Efr32mg21::tx_frame() {
    const std::uint32_t buffer = rail_regs_->peek(Buf0Addr);
    std::uint8_t phr = 0;
    if (!core_->read_memory(buffer, &phr, 1) || phr == 0)
        return {};
    std::vector<std::uint8_t> frame(1u + phr);
    if (!core_->read_memory(buffer, frame.data(), frame.size()))
        return {};
    return frame;
}

// A WRITEDATA store writes one byte at ADDR + WRITEOFFSET and advances WRITEOFFSET
// (inferred from the FIFO writer, which reads WRITEOFFSET after its four stores).
void Efr32mg21::tx_store(std::uint8_t byte) {
    const std::uint32_t offset = rail_regs_->peek(Buf0WriteOffset);
    core_->write_memory(rail_regs_->peek(Buf0Addr) + offset, &byte, 1);
    rail_regs_->write(Buf0WriteOffset, offset + 1);
}

// The RF channel is the SYNTH channel field (0xB8018044), 11..26 for 802.15.4.
std::uint8_t Efr32mg21::rail_channel() const {
    const std::uint32_t synth_channel = rail_regs_->peek(SynthChannel);
    return static_cast<std::uint8_t>(synth_channel >= 11 && synth_channel <= 26 ? synth_channel : 11);
}

// RAIL's RX context (the BUF2 descriptor ring state) is a RAM structure whose address differs
// between images, so it is found by what it holds: the buffer RAM addresses of BUF2, BUF1 and
// BUF0 in consecutive words at +0x18, +0x1C and +0x20 (observed in the Zephyr router and ED
// images; the ring size sits at +0x3A). The result is cached while it still matches.
std::uint32_t Efr32mg21::rx_context() {
    const std::uint32_t wanted[3] = {rail_regs_->peek(Buf2Addr), rail_regs_->peek(Buf1Addr), rail_regs_->peek(Buf0Addr)};
    auto matches = [&](std::uint32_t context) {
        std::uint32_t words[3] = {};
        return core_->read_memory(context + 0x18, words, sizeof(words)) && std::equal(words, words + 3, wanted);
    };
    if (rx_context_ != 0 && matches(rx_context_))
        return rx_context_;
    rx_context_ = 0;
    if (wanted[0] == 0 || wanted[1] == 0 || wanted[2] == 0)
        return 0;
    std::vector<std::uint8_t> ram(RamSize);
    if (!core_->read_memory(RamBase, ram.data(), ram.size()))
        return 0;
    for (std::uint32_t offset = 0; offset + 12 <= RamSize; offset += 4) {
        if (std::memcmp(ram.data() + offset, wanted, sizeof(wanted)) == 0 && offset >= 0x18) {
            rx_context_ = RamBase + offset - 0x18;
            return rx_context_;
        }
    }
    return 0;
}

// A received frame goes to the BUF1 buffer RAM and its descriptor to the BUF2 ring, then
// the BUF2 threshold flag (BUFC IF bit 18) is raised for the firmware's handler.
void Efr32mg21::deliver_rail_rx(const std::vector<std::uint8_t> &fifo) {
    // The frame goes to the BUF1 buffer RAM (BUF1 ADDR, +0x3C) with eight bytes the hardware
    // appends after it; RAIL takes those eight bytes off the count for statuses 6 and 7 (the
    // trailing data option), so the end offset includes them.
    std::vector<std::uint8_t> stored = fifo;
    stored.insert(stored.end(), 8, 0x00);
    // Each frame goes after the previous one (BUF1 WRITEOFFSET), so a frame that is still
    // unread is not overwritten. The read offset is set to the frame start.
    const std::uint32_t frame_at = rail_regs_->peek(Buf1WriteOffset);
    const std::uint32_t buf1 = rail_regs_->peek(Buf1Addr) + frame_at;
    if (!core_->write_memory(buf1, stored.data(), stored.size()))
        return;
    rail_regs_->write(Buf1WriteOffset, frame_at + static_cast<std::uint32_t>(stored.size()));
    rail_regs_->write(Buf1ReadOffset, frame_at);
    // The descriptor is written in the BUF2 ring where the threshold handler reads it: base
    // at context +0x18, size at +0x3A, write and read indexes at +0x3C and +0x3E (see
    // the RX section above). Word 0 is the end offset in BUF1; byte 2 low bits are the
    // packet status, 7 = READY_SUCCESS. BUF2 STATUS counts the descriptor bytes pending.
    std::uint32_t base = 0;
    std::uint16_t size = 0, write = 0, read = 0;
    std::uint8_t ring[0x42] = {};
    const std::uint32_t context = rx_context();
    if (context == 0 || !core_->read_memory(context, ring, sizeof(ring)))
        return;
    base = ring[0x18] | (ring[0x19] << 8) | (ring[0x1A] << 16) | (static_cast<std::uint32_t>(ring[0x1B]) << 24);
    size = static_cast<std::uint16_t>(ring[0x3A] | (ring[0x3B] << 8));
    write = static_cast<std::uint16_t>(ring[0x3C] | (ring[0x3D] << 8));
    read = static_cast<std::uint16_t>(ring[0x3E] | (ring[0x3F] << 8));
    if (size == 0)
        return;
    const std::uint32_t position = static_cast<std::uint32_t>((write + read - 7) & (size - 1)) & ~7u;
    // Word 0 is the frame's end offset in BUF1 (frame start plus stored length).
    const std::uint32_t end = frame_at + static_cast<std::uint32_t>(stored.size());
    // Byte 2 bit 4 is the ACK flag: RAIL reports it as isAck in the packet details (byte 13
    // when the status is 6 or 7, see sli_rail_e331ba91 at 0x1D9C8). The driver only matches
    // a short frame against its pending transmit when that flag is set, so an ACK (frame type
    // 2 in the FC byte after the PHR) must carry it or it is dropped as a 3-byte frame.
    const bool ack = fifo.size() >= 2 && (fifo[1] & 7u) == 2u;
    std::uint8_t descriptor[8] = {static_cast<std::uint8_t>(end), static_cast<std::uint8_t>(end >> 8),
                                  static_cast<std::uint8_t>(7u | (ack ? 0x10u : 0u)), 0, 0, 0, 0, 0};
    if (!core_->write_memory(base + position, descriptor, sizeof(descriptor)))
        return;
    rail_regs_->write(Buf2Status, 8);
    rail_regs_->set_bits(BufcIf, Buf2Threshold);
    rail_regs_->set_rx_frame(true);
}

void Efr32mg21::acknowledge(const std::vector<std::uint8_t> &fifo) {
    // fifo is the PHR followed by the frame: FC (2 bytes), sequence, then the destination PAN
    // (2 bytes, the only one with PAN compression) and the destination address. The hardware
    // sends the ACK about 12 symbols after the frame; the model sends it when the frame is
    // delivered. The stack's extended address is the unique id (DEVINFO +0x48, see
    // set_device_id), not the EUI-64 at +0x1A0.
    if (fifo.size() < 14)
        return;
    const std::uint8_t fc0 = fifo[1], fc1 = fifo[2];
    if ((fc0 & 7u) == 2u)
        return; // an ACK is never acknowledged
    const std::uint8_t dst_mode = (fc1 >> 2) & 3u, src_mode = (fc1 >> 6) & 3u;
    // Only PAN-compressed frames are handled: the offsets below assume a single PAN ID.
    if ((fc0 & 0x40u) == 0)
        return;
    bool mine = false;
    if (dst_mode == 3u)
        mine = std::memcmp(fifo.data() + 6, devinfo_ + 0x48, 8) == 0;
    else if (dst_mode == 2u && short_address_known_)
        mine = fifo[4] == pan_id_[0] && fifo[5] == pan_id_[1] && fifo[6] == short_address_[0] &&
               fifo[7] == short_address_[1];
    // A command frame in the association response (command 0x02): it carries the short
    // address the coordinator assigns, after the 8-byte source address.
    if (mine && (fc0 & 7u) == 3u && dst_mode == 3u && src_mode == 3u && fifo.size() >= 26 && fifo[22] == 0x02u) {
        pan_id_[0] = fifo[4];
        pan_id_[1] = fifo[5];
        short_address_[0] = fifo[23];
        short_address_[1] = fifo[24];
        short_address_known_ = true;
    }
    if ((fc0 & 0x20u) == 0 || !mine)
        return;
    const std::vector<std::uint8_t> ack = {5, 0x02, 0x00, fifo[3]};
    radio_->transmit_fifo(ack, rail_channel());
    // The sequencer ends the ACK with TXAFTERFRAMEDONE (FRC IF bit 1, enabled in the RAIL
    // image). The FRC handler reports it as TXACK_PACKET_SENT (SDK bit 25, 0x2000000) when
    // RAC SR0 bits 13..15 are clear; the driver clears its ack_tx state on that event.
    rail_regs_->set_bits(FrcIf, 1u << 1);
}

// PROTIMER (IRQ 36), layout from recovery/reconstructed.c (the image's, not the
// v5.8.0 header): WRAPCNT at +0x20 counts once per PRECNTTOP+0x100 core cycles;
// compare channel n is at +0x100+16n (CTRL, WRAP at +0x0C). A channel in compare
// mode (CTRL & 3 == 1) sets IF bit 8+n when WRAPCNT passes its WRAP value. IF is
// at +0x64 (IF_SET/IF_CLR through the aliases), IEN at +0x70, and the line is
// IF & IEN.
std::uint64_t Efr32mg21::service(std::uint64_t now) {
    static constexpr std::uint64_t ProtimerPeriod = 3840; // 100 us at 38.4 MHz; the compare check rate
    if (now >= next_protimer_) {
        const std::uint64_t divisor = static_cast<std::uint64_t>(rail_regs_->peek(ProtimerPrecntTop)) + 0x100u;
        const std::uint64_t wrap_now = divisor != 0 ? now / divisor : 0;
        const std::uint32_t enabled = rail_regs_->peek(ProtimerIen);
        for (unsigned channel = 0; channel < 8; ++channel) {
            const std::uint32_t cc = ProtimerBase + 0x100u + 16u * channel;
            const std::uint32_t bit = 1u << (8 + channel);
            const std::uint64_t previous = protimer_last_wrap_[channel];
            protimer_last_wrap_[channel] = wrap_now;
            if (!(enabled & bit) || (rail_regs_->peek(cc) & 3u) != 1u)
                continue;
            const std::uint64_t compare = rail_regs_->peek(cc + 0x0Cu);
            if (previous < compare && compare <= wrap_now)
                rail_regs_->set_bits(ProtimerIf, bit);
        }
        core_->set_irq_line(ProtimerIrq, (rail_regs_->peek(ProtimerIf) & enabled) != 0);
        next_protimer_ = now + ProtimerPeriod;
    }
    // The end of the frame sets FRC TX_PACKET_SENT; the ACK wait then runs out
    // unless an ACK arrives, which the RAIL model does not receive from the air.
    if (now >= tx_done_at_) {
        tx_done_at_ = NoEvent;
        rail_regs_->set_bits(FrcIf, FrcTxPacketSent);
        // The driver sets tx_transitions.success = RX: a successful transmit leaves the radio
        // receiving (RXENSRCEN bit 1, the RX request this model tracks).
        rail_regs_->set_bits(RacRxEnable, RacRxenRequest);
        ack_timeout_at_ = now + AckWaitCycles;
    }
    if (now >= ack_timeout_at_) {
        ack_timeout_at_ = NoEvent;
        rail_regs_->set_bits(RacIf, RacRxAckTimeout);
    }
    core_->set_irq_line(FrcIrq, (rail_regs_->peek(FrcIf) & rail_regs_->peek(FrcIen)) != 0);
    core_->set_irq_line(RacSeqIrq, (rail_regs_->peek(RacIf) & rail_regs_->peek(RacIen)) != 0);
    core_->set_irq_line(BufcIrq, (rail_regs_->peek(BufcIf) & rail_regs_->peek(BufcIen)) != 0);
    std::uint64_t rx_poll = NoEvent;
    if (rail_regs_->rx_enabled()) {
        if (now >= rail_rx_poll_at_) {
            rail_rx_poll_at_ = now + RailRxPollCycles;
            for (const auto &fifo : radio_->receive_fifo(rail_channel(), core_->cycles_to_ns(now))) {
                deliver_rail_rx(fifo);
                acknowledge(fifo);
            }
        }
        rx_poll = rail_rx_poll_at_;
    }
    return std::min({radio_->service(now), rtcc_->service(now), next_protimer_, tx_done_at_, ack_timeout_at_, rx_poll});
}

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
    address = secure_alias(address);
    if (address >= RadioBase && address < RadioEnd) {
        value = radio_->read(address - RadioBase);
    } else if (address - RtccBase < RtccSize) {
        value = rtcc_->read((address - RtccBase) & 0xffc);
    } else if (address - SeBase < SeSize) {
        value = se_mailbox_->read((address - SeBase) & 0xffc);
    } else if (address == ProtimerWrapCount && rail_regs_->peek(ProtimerStatus) != 0) {
        // PROTIMER WRAPCNT (image layout +0x20): counts pre-counter overflows, one per
        // PRECNTTOP+0x100 core cycles while RUNNING (+0x14 bit 0).
        const std::uint64_t divisor = static_cast<std::uint64_t>(rail_regs_->peek(ProtimerPrecntTop)) + 0x100u;
        value = static_cast<std::uint32_t>(core_->cycles() / divisor);
    } else if (is_rail_register(address)) {
        value = rail_regs_->read(address);
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
    address = secure_alias(address);
    if (address >= RadioBase && address < RadioEnd) {
        radio_->write(address - RadioBase, value);
        return;
    }
    if (address - RtccBase < RtccSize) {
        write_register(*rtcc_, address - RtccBase, value);
        return;
    }
    if (address - SeBase < SeSize) {
        write_register(*se_mailbox_, address - SeBase, value);
        return;
    }
    if (is_rail_register(address)) {
        if (address == Buf0WriteData) {
            tx_store(static_cast<std::uint8_t>(value));
            return;
        }
        // The firmware acknowledges the BUF2 threshold by clearing BUFC IF bit 18: the
        // frame is consumed and RAC leaves RXFRAME.
        if (address == BufcIfClear && (value & Buf2Threshold))
            rail_regs_->set_rx_frame(false);
        if (address == Buf0Cmd && (value & 1u))
            rail_regs_->write(Buf0WriteOffset, 0); // CMD.CLEAR empties the buffer
        rail_regs_->write(address, value, size);
        if (rail_regs_->take_tx_start()) {
            const std::uint64_t airtime_ns = radio_->transmit_fifo(tx_frame(), rail_channel());
            tx_done_at_ = core_->cycles() + airtime_ns * 384 / 10000; // 38.4 cycles per us
            request_service(tx_done_at_);
        }
        return;
    }
    const std::uint32_t offset = address - PeripheralBase;
    write_register(block_at(offset >> 14), offset, value);
}

} // namespace efr32emu
