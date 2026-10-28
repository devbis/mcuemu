// CC2530 USART0/USART1 (SWRU191F Chapter 17).
//
// UART: host bytes are logical frames (start/stop polarity is not rendered
// on pads). Model-defined bit order: a host byte is the byte as an LSB-first
// receiver/transmitter sees it on the wire, so UxGCR.ORDER = 1 (MSB first)
// bit-reverses it. Timing follows the baud-rate generator exactly
// (equation 3, 17.4): one bit lasts 2^28 * (32 MHz / f) / ((256 + BAUD_M) *
// 2^BAUD_E) cycles of the 32 MHz reference, f = 32 MHz (XOSC) or 16 MHz
// (RCOSC); a frame has 1 start bit, 8 data bits, the optional 9th bit (BIT9)
// and SPB ? 2 : 1 stop bits. The baud rate is latched at the start of each
// frame (17.4: a change "may take up to one bit period" to apply).
//   TX (17.1.1, 17.6, 17.7): writing UxDBUF starts the frame at once (or
//   after the frame in progress: UxDBUF is double-buffered); UTXxIF and the
//   UTXx DMA trigger fire when the frame starts ("immediately after the
//   transmission has been started"), UxCSR.TX_BYTE is set and the byte is
//   handed to the host when the frame ends. With FLOW, a frame starts only
//   while the CTS pad is low (17.1.3).
//   RX (17.1.2, 17.1.4): queued host bytes follow each other back to back
//   while UxCSR.RE = 1. A byte is complete at the end of its first stop bit
//   ("the RX interrupt is set when the first stop bit is checked OK"):
//   RX_BYTE, URXxIF and the URXx DMA trigger. With FLOW the host only starts
//   a frame while RTS is low (RE = 1 and RX_BYTE = 0, 17.1.3).
//   Model-defined: without FLOW, a byte completed while RX_BYTE = 1 waits in
//   the shift register (17.1.2 note) and moves to UxDBUF when UxDBUF is
//   read; a further byte overwrites it (overrun: SWRU191F documents no
//   overrun flag, ERR is the parity error only, so nothing is flagged).
//   9th bit (17.1.4, UxUCR): with BIT9 = 1 a frame carries a 9th bit after
//   the data bits. PARITY = 1: the parity of the 8 data bits, D9 = 0 odd
//   (data and parity bits hold an odd number of ones), D9 = 1 even
//   (SWRU191F UxUCR; the DN112 design note SWRA222B labels D9 the other way
//   round, so this needs hardware evidence). PARITY = 0: the 9th bit is
//   D9 itself (DN112 "fixed level"; SWRU191F does not say). The bit is
//   independent of ORDER. TX computes it from UxUCR when the frame starts;
//   with frame capture on (host) each transmitted frame is also recorded
//   with its 9th bit, the byte stream is unchanged.
//   Errors: a host frame (receive(Cc2530UartFrame)) may carry an explicit
//   9th bit, a wrong parity bit, wrong stop-bit levels or a break; a plain
//   host byte is a well-formed frame in the receiver's format, so it never
//   sets FE or ERR. The frame is rendered in the receiver's UxUCR, latched
//   at its start bit (model-defined, like the baud rate): ERR (17.1.4) when
//   BIT9 = PARITY = 1 and the received 9th bit is not the parity of the
//   data per D9; FE when the first stop bit is not at UxUCR.STOP, and with
//   SPB = 1 also when the second one is not ("the receiver checks both
//   stop bits when UxUCR.SPB is set"). Model-defined: a frame with ERR or a
//   bad first stop bit is still received normally at the end of its first
//   stop bit (UxDBUF, RX_BYTE, URXxIF, the URXx DMA trigger, or the pending
//   shift register on overrun: UxCSR "byte received with parity error /
//   incorrect stop-bit level"), ERR/FE being set at that instant before the
//   interrupt; this needs hardware evidence ("the RX interrupt is set when
//   the first stop bit is checked OK"). A bad second stop bit sets FE one
//   bit later, at the end of that bit (17.1.4: "a delay in setting the
//   framing error bit ... (bit duration)"), a scheduled event of its own;
//   FLUSH, RE = 0, a mode change or a reset cancel it. A frame discarded
//   by FLUSH sets no flag. The received 9th bit cannot be read back
//   (SWRU191F has no register for it). A 9th bit sent to a receiver with
//   BIT9 = 0 falls into its first stop-bit slot (FE if that level is not
//   STOP) and the sender's first stop bit into its second slot; a frame
//   always occupies the receiver's frame time on the wire. Break
//   (model-defined, SWRU191F describes none): the line stays at the
//   start-bit level for one frame time, i.e. data, 9th and stop bits at
//   the START level, so FE (and ERR with odd parity) are set and the byte
//   is START ? 0xff : 0x00.
// SPI (17.2, UxGCR), pad level. SCK idles at CPOL. A leading edge goes
// from CPOL to CPOL inverted, a trailing edge back to CPOL. CPHA = 0: data
// is output on trailing edges and sampled on leading edges; CPHA = 1:
// output on leading edges, sampled on trailing edges. ORDER = 1 sends bit
// 7 first, ORDER = 0 bit 0 first. Pads follow Table 7-1 (MI, MO, SS, C).
//   Master (17.2.1): writing UxDBUF starts the byte (double buffered, like
//   UART TX). The baud-rate generator (17.4, equation 3) gives the SCK
//   period of one bit; the 16 edges of a byte are half a bit apart, the
//   first half a bit after the start, so the last edge ends the byte 8 bit
//   periods after its start (the byte timing used before pad-level SPI).
//   Each edge is its own scheduled event at its HF cycle (2 events per bit,
//   16 per byte); pad changes reach Io and the host listener stamped with
//   that cycle. MOSI and SCK are driven; MISO is read from the pad (Io
//   level: the external/net level, else the pad's own drive or pull).
//   UTXxIF and the UTXx DMA trigger at the start (17.2.1, 17.6, 17.7); at
//   the end TX_BYTE, the received byte in UxDBUF, URXxIF and the URXx DMA
//   trigger. RX_BYTE stays 0 (UxCSR: "UART mode and SPI slave mode").
//   Model-defined: UxGCR is latched when the byte starts; with CPHA = 0 the
//   first bit is on MOSI when the byte starts (before the first, sampling,
//   edge); MOSI keeps the last bit after a byte and is low after reset or a
//   mode change; edge times are rounded up to the next 32 MHz cycle like
//   UART bits; MISO is sampled when the edge event is serviced, at the
//   instruction boundary at or after the edge cycle (exact for levels the
//   chip drives itself, e.g. a net loop from MOSI); FLUSH or a mode change
//   returns SCK to idle at once.
//   With a host device (set_spi_device) a master byte is exchanged at its
//   end and the pads do not toggle: the device gets the MOSI byte and
//   returns MISO, both as MSB-first wire bytes (ORDER = 0 bit-reverses
//   them, model-defined); CPOL/CPHA do not change the data.
//   Slave (17.2.2, 17.3): SCK, SSN and MOSI are pad inputs; Io reports pad
//   level changes. While SSN is low, SCK edges sample MOSI and drive MISO
//   from the transmit shift register; while SSN is high edges are ignored
//   and MISO is 3-state. After 8 samples: the byte in UxDBUF, RX_BYTE,
//   URXxIF and the URXx DMA trigger (17.2.2). TX_BYTE stays 0 (UxCSR: "UART
//   mode and SPI master mode"). UTXxIF and the UTXx DMA trigger when a byte
//   written to UxDBUF enters the shift register (17.2.2: "at the start of
//   the operation"). ACTIVE = 1 while SSN is low (UxCSR: "In SPI slave
//   mode, this bit equals slave select").
//   Model-defined: UxGCR is read at each edge. A byte written to UxDBUF
//   enters the shift register at once when no bit of the next byte has been
//   shifted and no written byte is loaded for it yet, otherwise when the
//   current byte completes (double buffering as in 17.1.1); with CPHA = 0
//   its first bit is driven at once if SCK is idle, else at the next output
//   edge. Without a new UxDBUF byte the previous one is sent again (0x00
//   after reset). A complete byte overwrites an unread UxDBUF (no overrun
//   flag is documented). SSN released mid-byte keeps the bits already
//   shifted, so the next byte completes after the remaining bits only (17.3:
//   "not received properly"); FLUSH clears them (17.3, 17.5). SSN and SCK
//   changing at the same instant: SSN first. Changing SLAVE in SPI mode
//   aborts like a mode change. In PM1-PM3 (no HF clock) SCK edges and
//   loads are ignored; MISO still follows SSN.
//   Not modeled: corruption of back-to-back DMA transmits with CPHA = 1
//   (17.2.1 note), the maximum SPI rate of the data sheet (17.4).
// The HF clock stops in PM1-PM3 (all timing uses hf_now()): frames in
// progress freeze and host bytes queued while asleep arrive after wake.

#include "cc2530_usart.hpp"

#include "cc2530_io.hpp"

#include <algorithm>
#include <bit>
#include <utility>

namespace mcs51emu::cc2530 {
namespace {

struct Registers {
    std::uint8_t csr, dbuf, baud, ucr, gcr;
};
constexpr Registers UsartRegs[2] = {{0x86, 0xc1, 0xc2, 0xc4, 0xc5}, {0xf8, 0xf9, 0xfa, 0xfb, 0xfc}};

constexpr std::uint8_t CsrMode = 0x80, CsrRe = 0x40, CsrSlave = 0x20, CsrFe = 0x10, CsrErr = 0x08,
                       CsrRxByte = 0x04, CsrTxByte = 0x02, CsrActive = 0x01;
constexpr std::uint8_t UcrFlush = 0x80, UcrFlow = 0x40, UcrD9 = 0x20, UcrBit9 = 0x10, UcrParity = 0x08,
                       UcrSpb = 0x04, UcrStop = 0x02, UcrStart = 0x01;
constexpr std::uint8_t GcrCpol = 0x80, GcrCpha = 0x40, GcrOrder = 0x20;
// Model-defined bound of the host receive queue.
constexpr std::size_t RxQueueCapacity = 4096;

std::uint8_t reverse_bits(std::uint8_t byte) {
    std::uint8_t out = 0;
    for (int i = 0; i < 8; ++i)
        if (byte & (1u << i))
            out = static_cast<std::uint8_t>(out | (0x80u >> i));
    return out;
}

// Bits from the start bit to the end of the first stop bit (RX complete).
unsigned bits_to_first_stop_end(std::uint8_t ucr) { return 1 + 8 + ((ucr & UcrBit9) ? 1 : 0) + 1; }

// The 9th bit a UART with `ucr` sends after the LSB-first wire byte `data`
// (17.1.4, UxUCR D9/PARITY): parity (D9 = 1 even, 0 odd) or D9 itself.
bool ninth_bit(std::uint8_t ucr, std::uint8_t data) {
    if (!(ucr & UcrParity))
        return (ucr & UcrD9) != 0;
    const bool odd_ones = (std::popcount(data) & 1) != 0;
    return (ucr & UcrD9) ? odd_ones : !odd_ones;
}

using PadUse = std::optional<std::optional<bool>>;

// Mask of data bit `k` (0 = first on the wire) of a byte (UxGCR.ORDER).
std::uint8_t wire_bit(unsigned k, bool msb_first) {
    return static_cast<std::uint8_t>(msb_first ? 0x80u >> k : 1u << k);
}

} // namespace

std::uint64_t Usart::BitClock::time_after(unsigned bits) const {
    return anchor + (frac + bits * num + den - 1) / den;
}

std::uint64_t Usart::BitClock::time_after_halves(unsigned halves) const {
    return anchor + (2 * frac + halves * num + 2 * den - 1) / (2 * den);
}

void Usart::BitClock::advance(unsigned bits) {
    const std::uint64_t total = frac + bits * num;
    anchor += total / den;
    frac = total % den;
}

void Usart::BitClock::restart(std::uint64_t at, std::uint64_t n, std::uint64_t d) {
    anchor = at;
    frac = 0;
    num = n;
    den = d;
}

Usart::Usart(Bus &bus, unsigned index) : Peripheral(bus), index_(index) {
    const Registers &r = UsartRegs[index];
    bus.claim_sfr(r.csr, this, true);  // ACTIVE is derived from time
    bus.claim_sfr(r.dbuf, this, true); // reading clears RX_BYTE
    bus.claim_sfr(r.baud, this, false);
    bus.claim_sfr(r.ucr, this, false);
    bus.claim_sfr(r.gcr, this, false);
}

void Usart::reset(ResetCause) {
    const Registers &r = UsartRegs[index_];
    bus_.storage(sfr_reg(r.csr)) = 0x00;
    bus_.storage(sfr_reg(r.dbuf)) = 0x00;
    bus_.storage(sfr_reg(r.baud)) = 0x00;
    bus_.storage(sfr_reg(r.ucr)) = 0x02;
    bus_.storage(sfr_reg(r.gcr)) = 0x00;
    tx_hold_.reset();
    tx_busy_ = false;
    tx_end_hf_ = Never;
    rx_queue_.clear();
    rx_active_ = false;
    rx_discard_ = false;
    rx_done_hf_ = Never;
    rx_idle_hf_ = 0;
    rx_fe2_hf_ = Never;
    rx_data_ = 0;
    rx_pending_.reset();
    spi_pads_ = false;
    spi_edge_ = 0;
    sck_out_ = false;
    mosi_out_ = false;
    slave_ssn_ = true;
    slave_sck_ = false;
    slave_tx_ = 0;
    miso_out_ = false;
    spi_slave_reset(false);
    // The host side (spi_device_, bytes and frames already transmitted,
    // frame capture) is kept.
}

std::uint8_t Usart::read(Reg reg) {
    const Registers &r = UsartRegs[index_];
    if (reg == sfr_reg(r.csr)) {
        const std::uint8_t value = peek(reg);
        // FE and ERR clear on a read of UxCSR "or bits in the UxCSR
        // register" (17.8): byte, bit (U1CSR) and read-modify-write reads
        // all come through here.
        bus_.storage(reg) = static_cast<std::uint8_t>(bus_.storage(reg) & ~(CsrFe | CsrErr));
        return value;
    }
    if (reg == sfr_reg(r.dbuf)) {
        const std::uint8_t value = rx_data_;
        std::uint8_t &csr = bus_.storage(sfr_reg(r.csr));
        if (csr & CsrRxByte) {
            csr = static_cast<std::uint8_t>(csr & ~CsrRxByte); // 17.1.2
            if (rx_pending_) {
                const std::uint8_t next = *rx_pending_;
                rx_pending_.reset();
                deliver(next);
            } else {
                try_start_rx(bus_.hf_now()); // RTS asserted again (FLOW)
            }
            outputs_changed();
            bus_.reschedule();
        }
        return value;
    }
    return peek(reg);
}

std::uint8_t Usart::peek(Reg reg) const {
    const Registers &r = UsartRegs[index_];
    if (reg == sfr_reg(r.csr)) {
        // ACTIVE: transmit or receive in progress (17.1.1, 17.1.2, 17.2.1);
        // the slave select in SPI slave mode (UxCSR).
        const std::uint64_t hf = bus_.hf_now();
        const bool active = spi_slave() ? !slave_ssn_
                                        : (tx_busy_ && hf < tx_end_hf_) ||
                                              (rx_active_ && !rx_discard_ && hf >= rx_start_hf_ &&
                                               hf < rx_done_hf_);
        return static_cast<std::uint8_t>((bus_.storage(reg) & ~CsrActive) | (active ? CsrActive : 0u));
    }
    if (reg == sfr_reg(r.dbuf))
        return rx_data_;
    return bus_.storage(reg);
}

void Usart::write(Reg reg, std::uint8_t value) {
    const Registers &r = UsartRegs[index_];
    if (reg == sfr_reg(r.csr)) {
        std::uint8_t &csr = bus_.storage(reg);
        const std::uint8_t old = csr;
        // MODE/RE/SLAVE R/W; FE, ERR, RX_BYTE, TX_BYTE R/W0; ACTIVE R.
        csr = static_cast<std::uint8_t>((value & (CsrMode | CsrRe | CsrSlave)) |
                                        (old & value & (CsrFe | CsrErr | CsrRxByte | CsrTxByte)));
        // Model-defined: changing the mode (UART, SPI master, SPI slave)
        // aborts everything.
        if (((old ^ csr) & CsrMode) || (!(csr & CsrMode) && ((old ^ csr) & CsrSlave))) {
            abort_all(true);
            mosi_out_ = false;
            spi_slave_reset(true);
        } else if ((old & CsrRe) && !(csr & CsrRe)) {
            // Model-defined: disabling the receiver aborts the frame in
            // progress and drops the host bytes not yet received.
            rx_active_ = false;
            rx_discard_ = false;
            rx_done_hf_ = Never;
            rx_fe2_hf_ = Never;
            rx_queue_.clear();
        }
        if ((old & CsrRxByte) && !(csr & CsrRxByte))
            rx_pending_.reset(); // "effectively discards the data" (17.8)
        try_start_rx(bus_.hf_now());
        try_start_tx(bus_.hf_now(), false);
        outputs_changed();
        return;
    }
    if (reg == sfr_reg(r.dbuf)) {
        bus_.storage(reg) = value;
        // Double buffering (17.1.1): the byte waits in the data buffer while
        // the shift register is busy; a second write replaces it.
        tx_hold_ = value;
        if (spi_slave()) {
            const bool was_loaded = slave_loaded_;
            spi_slave_load();
            // CPHA = 0 with SCK idle: the new first bit is due on MISO now.
            const std::uint8_t gcr = bus_.storage(sfr_reg(r.gcr));
            if (slave_loaded_ && !was_loaded && !(gcr & GcrCpha) &&
                slave_sck_ == ((gcr & GcrCpol) != 0)) {
                miso_out_ = (slave_tx_ & wire_bit(0, gcr & GcrOrder)) != 0;
                outputs_changed();
            }
            return;
        }
        try_start_tx(bus_.hf_now(), false);
        return;
    }
    if (reg == sfr_reg(r.ucr)) {
        bus_.storage(reg) = static_cast<std::uint8_t>(value & ~UcrFlush); // FLUSH reads 0
        if (value & UcrFlush)
            abort_all(false);
        try_start_rx(bus_.hf_now()); // FLOW may have been cleared
        try_start_tx(bus_.hf_now(), false);
        outputs_changed();
        return;
    }
    bus_.storage(reg) = value; // UxBAUD, UxGCR: used from the next frame/byte
    if (reg == sfr_reg(r.gcr))
        outputs_changed(); // CPOL is the idle SCK level
}

std::uint64_t Usart::deadline() const {
    const std::uint64_t tx = tx_busy_ ? bus_.hf_deadline(spi_pads_ ? spi_edge_hf() : tx_end_hf_) : Never;
    const std::uint64_t rx = rx_active_ ? bus_.hf_deadline(rx_done_hf_) : Never;
    const std::uint64_t fe2 = rx_fe2_hf_ != Never ? bus_.hf_deadline(rx_fe2_hf_) : Never;
    return std::min({tx, rx, fe2});
}

void Usart::service(std::uint64_t) {
    for (;;) {
        const std::uint64_t tx = tx_busy_ ? (spi_pads_ ? spi_edge_hf() : tx_end_hf_) : Never;
        const std::uint64_t rx = rx_active_ ? rx_done_hf_ : Never;
        const std::uint64_t next = std::min({tx, rx, rx_fe2_hf_});
        if (next == Never || next > bus_.hf_now())
            return;
        if (rx_fe2_hf_ == next)
            rx_second_stop();
        else if (tx > rx)
            rx_complete();
        else if (spi_pads_)
            spi_master_edge();
        else
            tx_complete();
    }
}

void Usart::power_mode_changed(PowerMode, PowerMode) { bus_.reschedule(); }

// ---- host ----

bool Usart::can_receive() const {
    return uart() && (bus_.storage(sfr_reg(UsartRegs[index_].csr)) & CsrRe) &&
           rx_queue_.size() < RxQueueCapacity;
}

void Usart::receive(std::uint8_t byte) { receive(Cc2530UartFrame{.data = byte}); }

void Usart::receive(const Cc2530UartFrame &frame) {
    if (!can_receive())
        return; // receiver disabled or queue full: the frame is lost
    rx_queue_.push_back(frame);
    try_start_rx(bus_.hf_now());
    bus_.reschedule();
}

std::vector<std::uint8_t> Usart::take_transmitted() { return std::exchange(transmitted_, {}); }

void Usart::set_frame_capture(bool on) {
    capture_frames_ = on;
    if (!on)
        transmitted_frames_.clear();
}

std::vector<Cc2530UartFrame> Usart::take_transmitted_frames() {
    return std::exchange(transmitted_frames_, {});
}

void Usart::set_spi_device(std::function<std::uint8_t(std::uint8_t)> device) {
    spi_device_ = std::move(device);
}

// ---- Io ----

std::optional<std::optional<bool>> Usart::pad_use(unsigned role) const {
    const Registers &r = UsartRegs[index_];
    const std::uint8_t csr = bus_.storage(sfr_reg(r.csr));
    const std::uint8_t ucr = bus_.storage(sfr_reg(r.ucr));
    if (csr & CsrMode) {
        const bool flow = ucr & UcrFlow;
        switch (role) {
        case 0: return PadUse(std::in_place); // RXD
        case 1: return PadUse(std::in_place, !(ucr & UcrStart)); // TXD idles opposite the start bit
        case 2: return flow ? PadUse(std::in_place) : PadUse(); // CTS
        default:
            // RTS low while the receive register is empty and RE = 1 (17.1.3).
            if (!flow)
                return PadUse();
            return PadUse(std::in_place, !((csr & CsrRe) && !(csr & CsrRxByte)));
        }
    }
    if (csr & CsrSlave) {
        // 17.3: MISO is driven while SSN is low, 3-state while SSN is high;
        // MOSI, SSN and SCK are inputs.
        if (role == 0 && !slave_ssn_)
            return PadUse(std::in_place, miso_out_);
        return PadUse(std::in_place);
    }
    switch (role) {
    case 0: return PadUse(std::in_place); // MISO
    case 1: return PadUse(std::in_place, mosi_out_); // MOSI
    case 2: return PadUse(); // SSN unused in master mode (17.3)
    default:
        // SCK: the edges of the byte in progress, else idle at CPOL.
        if (tx_busy_ && spi_pads_)
            return PadUse(std::in_place, sck_out_);
        return PadUse(std::in_place, (bus_.storage(sfr_reg(r.gcr)) & GcrCpol) != 0);
    }
}

void Usart::pad_levels_changed() {
    if (uart() && (bus_.storage(sfr_reg(UsartRegs[index_].ucr)) & UcrFlow) && tx_hold_ && !tx_busy_) {
        try_start_tx(bus_.hf_now(), false);
        bus_.reschedule();
    }
    if (spi_slave())
        spi_slave_pads();
}

// ---- implementation ----

bool Usart::uart() const { return bus_.storage(sfr_reg(UsartRegs[index_].csr)) & CsrMode; }

bool Usart::spi_master() const {
    return !(bus_.storage(sfr_reg(UsartRegs[index_].csr)) & (CsrMode | CsrSlave));
}

bool Usart::spi_slave() const {
    return (bus_.storage(sfr_reg(UsartRegs[index_].csr)) & (CsrMode | CsrSlave)) == CsrSlave;
}

// Equation 3: rate = (256 + BAUD_M) * 2^BAUD_E / 2^28 * f. In cycles of the
// 32 MHz reference one bit lasts num / den.
void Usart::bit_rate(std::uint64_t &num, std::uint64_t &den) const {
    const Registers &r = UsartRegs[index_];
    const std::uint64_t mantissa = bus_.storage(sfr_reg(r.baud));
    const unsigned exponent = bus_.storage(sfr_reg(r.gcr)) & 0x1fu;
    const bool rcosc = (bus_.clkconsta() & 0x40u) != 0; // CLKCONSTA.OSC: 16 MHz RCOSC
    num = (std::uint64_t{1} << 28) * (rcosc ? 2 : 1);
    den = (256 + mantissa) << exponent;
}

unsigned Usart::frame_bits() const {
    const std::uint8_t ucr = bus_.storage(sfr_reg(UsartRegs[index_].ucr));
    return 1 + 8 + ((ucr & UcrBit9) ? 1 : 0) + ((ucr & UcrSpb) ? 2 : 1);
}

void Usart::try_start_tx(std::uint64_t at, bool chained) {
    if (tx_busy_ || !tx_hold_ || !(uart() || spi_master()))
        return;
    const bool is_uart = uart();
    if (is_uart && (bus_.storage(sfr_reg(UsartRegs[index_].ucr)) & UcrFlow) &&
        bus_.soc().io->usart_pad_level(index_, 2))
        return; // 17.1.3: wait for CTS low
    std::uint64_t num, den;
    bit_rate(num, den);
    if (!chained || num != tx_clock_.num || den != tx_clock_.den)
        tx_clock_.restart(at, num, den);
    tx_byte_ = *tx_hold_;
    tx_hold_.reset();
    tx_busy_ = true;
    tx_bits_ = is_uart ? frame_bits() : 8;
    if (is_uart) {
        // The 9th bit follows UxUCR at the start of the frame, like the
        // frame length; the parity is over the data bits, whatever ORDER.
        const std::uint8_t ucr = bus_.storage(sfr_reg(UsartRegs[index_].ucr));
        tx_ninth_ = !(ucr & UcrBit9)        ? Cc2530UartFrame::Ninth::Expected
                    : ninth_bit(ucr, tx_byte_) ? Cc2530UartFrame::Ninth::One
                                               : Cc2530UartFrame::Ninth::Zero;
    }
    tx_end_hf_ = tx_clock_.time_after(tx_bits_);
    // SPI master on the pads unless a host device takes the byte.
    spi_pads_ = !is_uart && !spi_device_;
    if (spi_pads_) {
        spi_gcr_ = bus_.storage(sfr_reg(UsartRegs[index_].gcr));
        spi_edge_ = 0;
        spi_rx_ = 0;
        sck_out_ = (spi_gcr_ & GcrCpol) != 0;
        if (!(spi_gcr_ & GcrCpha))
            mosi_out_ = (tx_byte_ & wire_bit(0, spi_gcr_ & GcrOrder)) != 0;
        publish_pads(at);
    }
    // 17.1.1 / 17.2.1 / 17.6 / 17.7: the data buffer is free again.
    bus_.raise_irq(index_ == 0 ? Irq::Utx0 : Irq::Utx1);
    trigger_dma(index_ == 0 ? trigger::Utx0 : trigger::Utx1);
    bus_.reschedule();
}

void Usart::tx_complete() {
    const std::uint64_t at = tx_end_hf_;
    tx_busy_ = false;
    tx_end_hf_ = Never;
    tx_clock_.advance(tx_bits_);
    const bool msb_first = bus_.storage(sfr_reg(UsartRegs[index_].gcr)) & GcrOrder;
    set_csr_bits(CsrTxByte, true);
    if (uart()) {
        const std::uint8_t wire = msb_first ? reverse_bits(tx_byte_) : tx_byte_;
        transmitted_.push_back(wire);
        // Stop bits are always sent at UxUCR.STOP: no error to report.
        if (capture_frames_)
            transmitted_frames_.push_back(Cc2530UartFrame{.data = wire, .ninth = tx_ninth_});
    } else if (spi_pads_) {
        rx_data_ = spi_rx_;
        bus_.storage(sfr_reg(UsartRegs[index_].dbuf)) = rx_data_;
        bus_.raise_irq(index_ == 0 ? Irq::Urx0 : Irq::Urx1);
        trigger_dma(index_ == 0 ? trigger::Urx0 : trigger::Urx1);
    } else {
        const std::uint8_t mosi = msb_first ? tx_byte_ : reverse_bits(tx_byte_);
        const std::uint8_t miso = spi_device_ ? spi_device_(mosi) : 0xff;
        rx_data_ = msb_first ? miso : reverse_bits(miso);
        bus_.storage(sfr_reg(UsartRegs[index_].dbuf)) = rx_data_;
        bus_.raise_irq(index_ == 0 ? Irq::Urx0 : Irq::Urx1);
        trigger_dma(index_ == 0 ? trigger::Urx0 : trigger::Urx1);
    }
    try_start_tx(at, true);
}

std::uint64_t Usart::spi_edge_hf() const { return tx_clock_.time_after_halves(spi_edge_ + 1); }

// One SCK edge of the master byte in progress (17.2.1, UxGCR CPOL/CPHA/ORDER).
void Usart::spi_master_edge() {
    const unsigned edge = spi_edge_++;
    const std::uint64_t at = tx_clock_.time_after_halves(edge + 1);
    const bool cpol = spi_gcr_ & GcrCpol, cpha = spi_gcr_ & GcrCpha, msb_first = spi_gcr_ & GcrOrder;
    const bool leading = (edge & 1u) == 0; // CPOL -> CPOL inverted
    const unsigned bit = edge / 2;
    if (leading != cpha) {
        // Sampling edge: MISO before this edge's own pad changes.
        if (bus_.soc().io->usart_pad_level(index_, 0))
            spi_rx_ = static_cast<std::uint8_t>(spi_rx_ | wire_bit(bit, msb_first));
    } else {
        // Output edge: CPHA = 1 outputs bit `bit`, CPHA = 0 the next bit.
        const unsigned next = cpha ? bit : bit + 1;
        if (next < 8)
            mosi_out_ = (tx_byte_ & wire_bit(next, msb_first)) != 0;
    }
    sck_out_ = leading != cpol;
    if (spi_edge_ == 16)
        tx_complete(); // the last edge ends the byte (tx_end_hf_)
    publish_pads(at);
}

// SCK/SSN pad levels of the slave may have changed (17.2.2, 17.3, UxGCR).
void Usart::spi_slave_pads() {
    Io &io = *bus_.soc().io;
    const bool ssn = io.usart_pad_level(index_, 2);
    const bool sck = io.usart_pad_level(index_, 3);
    const bool ssn_changed = ssn != slave_ssn_;
    const bool sck_changed = sck != slave_sck_;
    if (!ssn_changed && !sck_changed)
        return;
    slave_ssn_ = ssn;
    slave_sck_ = sck;
    const std::uint8_t gcr = bus_.storage(sfr_reg(UsartRegs[index_].gcr));
    const bool cpol = gcr & GcrCpol, cpha = gcr & GcrCpha, msb_first = gcr & GcrOrder;
    // CPHA = 0: the current bit is on MISO before its (sampling) edge.
    if (ssn_changed && !ssn && !cpha)
        miso_out_ = (slave_tx_ & wire_bit(slave_bits_, msb_first)) != 0;
    const PowerMode mode = bus_.power_mode();
    if (sck_changed && !ssn && (mode == PowerMode::Active || mode == PowerMode::Idle)) {
        const bool leading = sck != cpol; // CPOL -> CPOL inverted
        if (leading != cpha) {
            if (io.usart_pad_level(index_, 1)) // MOSI
                slave_rx_ = static_cast<std::uint8_t>(slave_rx_ | wire_bit(slave_bits_, msb_first));
            if (++slave_bits_ == 8) {
                rx_data_ = slave_rx_;
                bus_.storage(sfr_reg(UsartRegs[index_].dbuf)) = rx_data_;
                set_csr_bits(CsrRxByte, true);
                bus_.raise_irq(index_ == 0 ? Irq::Urx0 : Irq::Urx1);
                trigger_dma(index_ == 0 ? trigger::Urx0 : trigger::Urx1);
                spi_slave_reset(false);
                spi_slave_load(); // the next byte, if one waits in UxDBUF
            }
        } else {
            miso_out_ = (slave_tx_ & wire_bit(slave_bits_, msb_first)) != 0;
        }
    }
    outputs_changed(); // MISO
}

// A byte waiting in UxDBUF enters the transmit shift register at a byte
// boundary (bit count 0) unless one already has: UTXxIF and the UTXx DMA
// trigger (17.2.2, 17.6, 17.7). Without one the previous byte is sent again.
void Usart::spi_slave_load() {
    if (!tx_hold_ || slave_bits_ != 0 || slave_loaded_)
        return;
    slave_tx_ = *tx_hold_;
    tx_hold_.reset();
    slave_loaded_ = true;
    bus_.raise_irq(index_ == 0 ? Irq::Utx0 : Irq::Utx1);
    trigger_dma(index_ == 0 ? trigger::Utx0 : trigger::Utx1);
}

void Usart::spi_slave_reset(bool snapshot) {
    slave_bits_ = 0;
    slave_rx_ = 0;
    slave_loaded_ = false;
    if (snapshot && spi_slave()) {
        // Entering slave mode: the current pad levels are not edges; MISO
        // presents the first bit of the shift register.
        const Io &io = *bus_.soc().io;
        slave_ssn_ = io.usart_pad_level(index_, 2);
        slave_sck_ = io.usart_pad_level(index_, 3);
        const bool msb_first = bus_.storage(sfr_reg(UsartRegs[index_].gcr)) & GcrOrder;
        miso_out_ = (slave_tx_ & wire_bit(0, msb_first)) != 0;
    }
}

void Usart::publish_pads(std::uint64_t hf_at) {
    const std::uint64_t hf = bus_.hf_now(), now = bus_.now();
    const std::uint64_t behind = hf > hf_at ? hf - hf_at : 0;
    bus_.soc().io->update(now > behind ? now - behind : 0);
}

void Usart::try_start_rx(std::uint64_t at) {
    const std::uint8_t csr = bus_.storage(sfr_reg(UsartRegs[index_].csr));
    const std::uint8_t ucr = bus_.storage(sfr_reg(UsartRegs[index_].ucr));
    if (rx_active_ || rx_queue_.empty() || !uart() || !(csr & CsrRe))
        return;
    if ((ucr & UcrFlow) && (csr & CsrRxByte))
        return; // RTS high: the host holds the next byte back (17.1.3)
    std::uint64_t num, den;
    bit_rate(num, den);
    const unsigned bits = frame_bits();
    // Exact back-to-back chaining when the next frame directly follows the
    // previous one at the same rate and format (no rounding drift).
    if (at <= rx_idle_hf_ && num == rx_clock_.num && den == rx_clock_.den && bits == rx_bits_ &&
        rx_clock_.time_after(rx_bits_) == rx_idle_hf_)
        rx_clock_.advance(rx_bits_);
    else
        rx_clock_.restart(std::max(at, rx_idle_hf_), num, den);
    rx_frame_ = rx_queue_.front();
    rx_queue_.pop_front();
    rx_ucr_ = ucr;
    rx_active_ = true;
    rx_discard_ = false;
    rx_bits_ = bits;
    rx_start_hf_ = rx_clock_.time_after(0);
    rx_done_hf_ = rx_clock_.time_after(bits_to_first_stop_end(ucr));
    rx_idle_hf_ = rx_clock_.time_after(bits);
}

void Usart::rx_complete() {
    const bool discard = rx_discard_;
    const std::uint64_t at = rx_done_hf_;
    const std::uint64_t frame_end = rx_idle_hf_;
    rx_active_ = false;
    rx_discard_ = false;
    rx_done_hf_ = Never;
    if (!discard) {
        // Render the host frame in the format latched at its start bit and
        // check it (17.1.4); see the header comment.
        using Ninth = Cc2530UartFrame::Ninth;
        const std::uint8_t ucr = rx_ucr_;
        const bool stop = ucr & UcrStop;
        std::uint8_t wire = rx_frame_.data;
        bool parity_error = false;
        bool slot1 = rx_frame_.stop1_error ? !stop : stop; // stop-bit slot levels
        bool slot2 = rx_frame_.stop2_error ? !stop : stop;
        if (rx_frame_.line_break) {
            const bool start = ucr & UcrStart;
            wire = start ? 0xff : 0x00;
            slot1 = slot2 = start;
            parity_error = (ucr & UcrBit9) && (ucr & UcrParity) && start != ninth_bit(ucr, wire);
        } else if (ucr & UcrBit9) {
            const bool expected = ninth_bit(ucr, wire);
            const bool ninth = rx_frame_.ninth == Ninth::Zero  ? false
                               : rx_frame_.ninth == Ninth::One ? true
                               : rx_frame_.ninth == Ninth::BadParity ? !expected
                                                                     : expected;
            parity_error = (ucr & UcrParity) && ninth != expected;
        } else if (rx_frame_.ninth == Ninth::Zero || rx_frame_.ninth == Ninth::One) {
            // An 8-bit receiver samples the 9th bit as its first stop bit.
            slot2 = slot1;
            slot1 = rx_frame_.ninth == Ninth::One;
        }
        const bool framing_error = slot1 != stop;
        if (parity_error || framing_error)
            set_csr_bits(static_cast<std::uint8_t>((parity_error ? CsrErr : 0u) | (framing_error ? CsrFe : 0u)),
                         true);
        if ((ucr & UcrSpb) && slot2 != stop)
            rx_fe2_hf_ = frame_end; // checked at the end of the second stop bit
        const bool msb_first = bus_.storage(sfr_reg(UsartRegs[index_].gcr)) & GcrOrder;
        const std::uint8_t byte = msb_first ? reverse_bits(wire) : wire;
        if (bus_.storage(sfr_reg(UsartRegs[index_].csr)) & CsrRxByte)
            rx_pending_ = byte; // overrun replaces an older pending byte
        else
            deliver(byte);
    }
    // The next host byte follows on the wire right after this frame.
    try_start_rx(at);
}

// 17.1.4: a bad second stop bit sets FE one bit after the byte completed.
void Usart::rx_second_stop() {
    rx_fe2_hf_ = Never;
    set_csr_bits(CsrFe, true);
}

void Usart::deliver(std::uint8_t byte) {
    rx_data_ = byte;
    bus_.storage(sfr_reg(UsartRegs[index_].dbuf)) = byte;
    set_csr_bits(CsrRxByte, true);
    bus_.raise_irq(index_ == 0 ? Irq::Urx0 : Irq::Urx1);
    trigger_dma(index_ == 0 ? trigger::Urx0 : trigger::Urx1);
    outputs_changed(); // RTS
}

// FLUSH (17.5, drop_queue = false) or a mode change (drop_queue = true).
// Model-defined: frames in progress stop at once (no sub-bit delay), the
// TX data buffer and the pending RX byte are cleared, status flags are
// kept and a pending second stop-bit check is dropped. After FLUSH a host
// frame being received still occupies the wire until its end and is
// discarded; later host bytes follow normally.
void Usart::abort_all(bool drop_queue) {
    tx_hold_.reset();
    tx_busy_ = false;
    tx_end_hf_ = Never;
    spi_pads_ = false;
    slave_bits_ = 0; // 17.3: a flush removes a partly shifted slave byte
    slave_rx_ = 0;
    slave_loaded_ = false;
    rx_pending_.reset();
    rx_fe2_hf_ = Never;
    if (drop_queue) {
        rx_queue_.clear();
        rx_active_ = false;
        rx_discard_ = false;
        rx_done_hf_ = Never;
    } else if (rx_active_) {
        rx_discard_ = true;
    }
}

void Usart::set_csr_bits(std::uint8_t mask, bool set) {
    std::uint8_t &csr = bus_.storage(sfr_reg(UsartRegs[index_].csr));
    csr = static_cast<std::uint8_t>(set ? (csr | mask) : (csr & ~mask));
}

void Usart::outputs_changed() { bus_.soc().io->peripheral_outputs_changed(); }

void Usart::trigger_dma(unsigned trigger) {
    ++dma_triggers_[trigger];
    bus_.dma_trigger(trigger);
}

} // namespace mcs51emu::cc2530
