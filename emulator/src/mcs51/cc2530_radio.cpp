// CC2530 IEEE 802.15.4 radio, SWRU191F Chapter 23 (section numbers below
// refer to SWRU191F unless stated otherwise).
//
// Time base: every radio event is kept in the HF clock domain
// (Bus::hf_now, 32 cycles = 1 us, frozen in PM1-PM3) and processed in time
// order together with the MAC timer by advance_rf_domain(). One IEEE
// 802.15.4 byte (2 O-QPSK symbols at 62.5 ksymbol/s) lasts 32 us = 1024
// cycles; one symbol 16 us = 512 cycles; 12 symbols (192 us) = 6144 cycles.
//
// Air interface (mcuemu::VirtualAir): air time ns = ns_offset + now-cycles
// * 1000 / 32. The air payload is the MPDU without the 2 FCS bytes; the
// air frame starts at the first preamble byte, so the SFD of a frame with
// the IEEE 802.15.4 4-byte preamble completes 160 us after start_ns.

#include "cc2530_radio.hpp"

#include "cc2530_mactimer.hpp"
#include "cc2530_timers.hpp"
#include "mcuemu/radio.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace mcs51emu::cc2530 {

namespace {

// ---- SFRs (23.15 / Table 23-5 and the SFR map) ----
constexpr std::uint8_t SfrRfirqf1 = 0x91, SfrRferrf = 0xBF, SfrRfd = 0xD9, SfrRfst = 0xE1,
                       SfrRfirqf0 = 0xE9;

// ---- XREGs (23.15.3, Table 23-5) ----
constexpr std::uint16_t RamBase = 0x6000, RamEnd = 0x6180; // 23.4: 384 bytes
constexpr std::uint16_t FRMFILT0 = 0x6180, FRMFILT1 = 0x6181, SRCMATCH = 0x6182,
                        SRCSHORTEN0 = 0x6183, SRCSHORTEN1 = 0x6184, SRCSHORTEN2 = 0x6185,
                        SRCEXTEN0 = 0x6186, SRCEXTEN1 = 0x6187, SRCEXTEN2 = 0x6188,
                        FRMCTRL0 = 0x6189, FRMCTRL1 = 0x618A, RXENABLE = 0x618B,
                        RXMASKSET = 0x618C, RXMASKCLR = 0x618D, FREQTUNE = 0x618E,
                        FREQCTRL = 0x618F, TXPOWER = 0x6190, TXCTRL = 0x6191,
                        FSMSTAT0 = 0x6192, FSMSTAT1 = 0x6193, FIFOPCTRL = 0x6194,
                        FSMCTRL = 0x6195, CCACTRL0 = 0x6196, CCACTRL1 = 0x6197, RSSI = 0x6198,
                        RSSISTAT = 0x6199, RXFIRST = 0x619A, RXFIFOCNT = 0x619B,
                        TXFIFOCNT = 0x619C, RXFIRST_PTR = 0x619D, RXLAST_PTR = 0x619E,
                        RXP1_PTR = 0x619F, TXFIRST_PTR = 0x61A1, TXLAST_PTR = 0x61A2,
                        RFIRQM0 = 0x61A3, RFIRQM1 = 0x61A4, RFERRM = 0x61A5, RFRND = 0x61A7,
                        MDMCTRL0 = 0x61A8, MDMCTRL1 = 0x61A9, FREQEST = 0x61AA,
                        RXCTRL = 0x61AB, FSCTRL = 0x61AC, FSCAL1 = 0x61AE, FSCAL2 = 0x61AF,
                        FSCAL3 = 0x61B0, AGCCTRL0 = 0x61B1, AGCCTRL1 = 0x61B2,
                        AGCCTRL2 = 0x61B3, AGCCTRL3 = 0x61B4, ADCTEST0 = 0x61B5,
                        ADCTEST1 = 0x61B6, ADCTEST2 = 0x61B7, MDMTEST0 = 0x61B8,
                        MDMTEST1 = 0x61B9, DACTEST0 = 0x61BA, DACTEST1 = 0x61BB,
                        DACTEST2 = 0x61BC, ATEST = 0x61BD, PTEST0 = 0x61BE, PTEST1 = 0x61BF,
                        CSPPROG0 = 0x61C0, CSPCTRL = 0x61E0, CSPSTAT = 0x61E1, CSPX = 0x61E2,
                        CSPY = 0x61E3, CSPZ = 0x61E4, CSPT = 0x61E5, RFC_OBS_CTRL0 = 0x61EB,
                        RFC_OBS_CTRL1 = 0x61EC, RFC_OBS_CTRL2 = 0x61ED, TXFILTCFG = 0x61FA;

// ---- radio RAM (23.4, Table 23-1), offsets from 0x6000 ----
constexpr unsigned RxFifoOff = 0x00, TxFifoOff = 0x80, SrcTableOff = 0x100,
                   SrcResMaskOff = 0x160, SrcResIndexOff = 0x163, SrcExtPendEnOff = 0x164,
                   SrcShortPendEnOff = 0x167, ExtAddrOff = 0x16A, PanIdOff = 0x172,
                   ShortAddrOff = 0x174;
constexpr unsigned FifoSize = 128;

// Storage-only registers: documented reset value and writable-bit mask.
// The analog/test registers among them have no modeled effect (listed in
// the report); their documented bits are stored and read back.
struct PlainReg {
    std::uint16_t addr;
    std::uint8_t reset;
    std::uint8_t wmask;
};
constexpr PlainReg PlainRegs[] = {
    {FRMFILT0, 0x0D, 0xFF},  {FRMFILT1, 0x78, 0xFF},  {SRCMATCH, 0x07, 0xFF},
    {SRCSHORTEN0, 0x00, 0xFF}, {SRCSHORTEN1, 0x00, 0xFF}, {SRCSHORTEN2, 0x00, 0xFF},
    {FRMCTRL0, 0x40, 0xFF},  {FRMCTRL1, 0x01, 0x07},  {FREQTUNE, 0x0F, 0x0F},
    {FREQCTRL, 0x0B, 0x7F},  {TXPOWER, 0xF5, 0xFF},   {TXCTRL, 0x69, 0x7F},
    {FIFOPCTRL, 0x40, 0x7F}, {FSMCTRL, 0x01, 0x03},   {CCACTRL0, 0xE0, 0xFF},
    {CCACTRL1, 0x1A, 0x1F},  {MDMCTRL0, 0x85, 0xFF},  {MDMCTRL1, 0x14, 0x3F},
    {RXCTRL, 0x3F, 0x3F},    {FSCTRL, 0x5A, 0xFF},    {FSCAL2, 0x20, 0x7F},
    {FSCAL3, 0x2A, 0x7F},    {AGCCTRL0, 0x5F, 0x7F},  {AGCCTRL1, 0x11, 0x3F},
    {AGCCTRL2, 0x00, 0xFF},  {AGCCTRL3, 0x2E, 0x7F},  {ADCTEST0, 0x10, 0xFF},
    {ADCTEST1, 0x0E, 0xFF},  {ADCTEST2, 0x03, 0x7F},  {MDMTEST0, 0x75, 0xFF},
    {MDMTEST1, 0x08, 0x1F},  {DACTEST0, 0x00, 0x7F},  {DACTEST1, 0x00, 0x7F},
    {DACTEST2, 0x00, 0x07},  {ATEST, 0x00, 0x3F},     {PTEST0, 0x00, 0xFF},
    {PTEST1, 0x00, 0x0F},    {RFC_OBS_CTRL0, 0x00, 0x7F}, {RFC_OBS_CTRL1, 0x00, 0x7F},
    {RFC_OBS_CTRL2, 0x00, 0x7F}, {TXFILTCFG, 0x0F, 0x0F},
};

const PlainReg *plain_reg(std::uint16_t addr) {
    for (const PlainReg &r : PlainRegs)
        if (r.addr == addr)
            return &r;
    return nullptr;
}

// ---- interrupt flag bits (23.1.2) ----
constexpr std::uint8_t F0RxMaskZero = 0x80, F0RxPktDone = 0x40, F0FrameAccepted = 0x20,
                       F0SrcMatchFound = 0x10, F0SrcMatchDone = 0x08, F0Fifop = 0x04,
                       F0Sfd = 0x02;
constexpr std::uint8_t F1CspWait = 0x20, F1CspStop = 0x10, F1CspManInt = 0x08, F1RfIdle = 0x04,
                       F1TxDone = 0x02, F1TxAckDone = 0x01;
constexpr std::uint8_t ErrStrobe = 0x40, ErrTxUnderf = 0x20, ErrTxOverf = 0x10,
                       ErrRxUnderf = 0x08, ErrRxOverf = 0x04, ErrRxAbo = 0x02, ErrNlock = 0x01;
constexpr std::uint8_t Irqf0Mask = 0xFF, Irqf1Mask = 0x3F, ErrfMask = 0x7F;

// ---- FSM state numbers (Table 23-3). Model-defined: states with a range of
// numbers (SFD wait 3-6, RX 7-13, TX 34-38, ACK 49-54) report the first
// number of the range; the internal sub-states are not documented.
constexpr std::uint8_t StIdle = 0, StRxCal = 2, StSfdWait = 3, StRx = 7, StRxWait = 14,
                       StRxOverflow = 17, StTxCal = 32, StTx = 34, StTxFinal = 39,
                       StTransit = 40, StAckCal = 48, StAck = 49, StTxUnderflow = 56;

// ---- timing in 32 MHz cycles ----
constexpr std::uint64_t ByteCycles = 1024;   // 32 us per byte (250 kbit/s)
constexpr std::uint64_t SymbolCycles = 512;  // 16 us per symbol
constexpr std::uint64_t TurnCycles = 6144;   // 12 symbols = 192 us (23.8.2, 23.9.2)
constexpr std::uint64_t RampCycles = 64;     // 2 us modulator ramp-down (23.8.2)
constexpr std::uint64_t RssiValidCycles = 4096; // 8 symbols (23.10.3, RSSISTAT)
constexpr std::uint64_t CcaDelayCycles = 4;  // CCA updated 4 cycles after RSSI_VALID (23.8.12)
// Model-defined: while the receiver searches for an SFD, the shared air is
// polled every 10 symbol periods (160 us, the synchronization header of an
// IEEE 802.15.4 frame) for frames of other nodes, so a frame registered on
// the air before it starts is always found before its SFD completes.
constexpr std::uint64_t PollCycles = 10 * SymbolCycles;
// Model-defined: a CSP busy loop waiting on CCA is re-evaluated at least
// every 512 cycles (in addition to every frame boundary on the channel).
constexpr std::uint64_t ParkCycles = 512;
// RSSI offset (23.10.3 example and CC2530 data sheet SWRS081: 73 dB).
constexpr int RssiOffset = 73;
// Model-defined: correlation value appended to every received frame (23.10.4:
// "approximately 110 indicates a maximum-quality frame").
constexpr std::uint8_t Correlation = 110;
// Air frame start to the end of the SFD with the standard 4-byte preamble.
constexpr std::uint64_t AirSfdNs = 160000;

std::uint64_t mix64(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

// IEEE 802.15.4 FCS: ITU-T CRC-16, x^16 + x^12 + x^5 + 1, initial value 0,
// LSB first (IEEE 802.15.4-2006 7.2.1.9; SWRU191F 23.8.10 / 23.9.7);
// transmitted low byte first.
std::uint16_t fcs16(const std::uint8_t *data, std::size_t size) {
    std::uint16_t crc = 0;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint16_t>((crc & 1u) ? (crc >> 1) ^ 0x8408u : crc >> 1);
    }
    return crc;
}

std::int8_t clamp_s8(int v) { return static_cast<std::int8_t>(std::clamp(v, -128, 127)); }

bool in_sleep(PowerMode mode) {
    return mode == PowerMode::Pm1 || mode == PowerMode::Pm2 || mode == PowerMode::Pm3;
}

// Parsed MAC header layout (IEEE 802.15.4-2006 7.2.1), offsets in the MPDU.
struct Mhr {
    unsigned type{0}, dst_mode{0}, src_mode{0}, version{0};
    bool security{false}, ack_request{false}, compression{false};
    unsigned reserved{0}; // FCF[9:7]
    unsigned dst_pan_off{0}, dst_addr_off{0}, src_pan_off{0}, src_addr_off{0};
    bool has_dst_pan{false}, has_src_pan{false};
    unsigned dst_len{0}, src_len{0};
    unsigned fields_end{0}; // FCF + seq + dst PAN + dst addr + src PAN
    unsigned src_end{0};    // fields_end + source address
};

unsigned addr_len(unsigned mode) { return mode == 2 ? 2 : mode == 3 ? 8 : 0; }

Mhr parse_mhr(std::uint16_t fcf) {
    Mhr m;
    m.type = fcf & 7u;
    m.security = fcf & 0x0008u;
    m.ack_request = fcf & 0x0020u;
    m.compression = fcf & 0x0040u;
    m.reserved = (fcf >> 7) & 7u;
    m.dst_mode = (fcf >> 10) & 3u;
    m.version = (fcf >> 12) & 3u;
    m.src_mode = (fcf >> 14) & 3u;
    m.dst_len = addr_len(m.dst_mode);
    m.src_len = addr_len(m.src_mode);
    unsigned off = 3;
    if (m.dst_mode != 0) {
        m.has_dst_pan = true;
        m.dst_pan_off = off;
        off += 2;
        m.dst_addr_off = off;
        off += m.dst_len;
    }
    // Model-defined: the source PAN ID is present when a source address is
    // present unless PAN ID compression is set and a destination address is
    // also present (IEEE 802.15.4-2006 7.2.1.1.5).
    if (m.src_mode != 0 && !(m.compression && m.dst_mode != 0)) {
        m.has_src_pan = true;
        m.src_pan_off = off;
        off += 2;
    }
    m.fields_end = off;
    m.src_addr_off = off;
    m.src_end = off + m.src_len;
    return m;
}

} // namespace


namespace {

// ---- register fields (23.15.3) ----
// FRMCTRL0
constexpr std::uint8_t Ctrl0AppendData = 0x80, Ctrl0AutoCrc = 0x40, Ctrl0AutoAck = 0x20,
                       Ctrl0EnergyScan = 0x10;
// FRMCTRL1
constexpr std::uint8_t Ctrl1PendingOr = 0x04, Ctrl1IgnoreTxUnderf = 0x02,
                       Ctrl1SetRxEnMaskOnTx = 0x01;
// FRMFILT0
constexpr std::uint8_t Filt0PanCoordinator = 0x02, Filt0Enable = 0x01;
// SRCMATCH
constexpr std::uint8_t SrcPendDataReqOnly = 0x04, SrcAutoPend = 0x02, SrcMatchEnable = 0x01;
// FSMCTRL
constexpr std::uint8_t FsmSlottedAck = 0x02, FsmRx2RxTimeOff = 0x01;
// MDMTEST1
constexpr std::uint8_t MdmTest1ModulationMode = 0x02;
// RXENABLE bits written by strobes (23.9.1, RXENABLE register)
constexpr std::uint8_t RxEnSrxon = 0x80, RxEnStxon = 0x40, RxEnMaskBit = 0x20;

enum class CspWait : std::uint8_t { None, Overflows, WaitX, Event1, Event2 };

// SNOP and the undefined opcodes, which behave as SNOP (23.14.8).
bool csp_is_nop(std::uint8_t op) {
    return op == 0xD0 || op == 0xD1 || op == 0xDC || (op >= 0xB0 && op <= 0xB7) || op == 0xBF ||
           op == 0xC6 || op == 0xC7 || op == 0xE0 || op == 0xEC || (op >= 0xF0 && op <= 0xFE);
}

// Immediate command strobes (Table 23-4): executed when written to RFST,
// never stored in the instruction memory.
bool csp_is_immediate(std::uint8_t op) {
    return (op >= 0xE1 && op <= 0xEB) || op == 0xED || op == 0xEE || op == 0xEF || op == 0xFF;
}

} // namespace

struct Radio::State {
    // A transmission in progress: a data frame from the TXFIFO or an ACK.
    struct Tx {
        bool active{false};
        bool ack{false};
        std::uint64_t start_hf{0}; // first preamble byte
        std::uint64_t sfd_hf{0};   // SFD completely sent
        std::uint64_t end_hf{0};   // last MPDU bit sent
        bool sfd_done{false};
        unsigned base{0};      // TXFIFO index of the length byte
        unsigned length{0};    // PHR frame length (MPDU bytes incl. FCS)
        unsigned fetches{0};   // TXFIFO bytes the frame reads (length byte included)
        unsigned checked{0};   // fetches known to have succeeded
        bool length_known{false};
        bool autocrc{true};
        bool flushed{false};   // SFLUSHTX during the frame
        bool registered{false};
        std::vector<std::uint8_t> ack_mpdu; // ACK MPDU without FCS
    };
    // The frame the receiver is locked on.
    struct Rx {
        bool active{false};
        std::uint64_t id{0};
        std::uint8_t channel{11};
        std::uint64_t end_ns{0};
        std::vector<std::uint8_t> mpdu; // without FCS
        unsigned length{0};             // L = MPDU bytes incl. FCS
        std::uint64_t sfd_hf{0};
        std::uint64_t end_hf{0};
        int rssi_dbm{0};
        unsigned next{0}; // next PHY byte (0 = length byte, L = last FCS byte)
        bool rejected{false};
        bool accepted{false}; // passed frame filtering (filtering enabled)
        bool filtering{false};
        bool autocrc{true};
        bool append{false};
        bool match_enabled{false};
        bool verdict_done{false};
        bool crc_ok{false};
        bool have_mhr{false};
        unsigned ftype{0}; // frame type after MODIFY_FT_FILTER
        Mhr mhr{};
        unsigned ack_strobe{0}; // 0 none, 6 SACK, 7 SACKPEND, 8 SNACK
        bool matched{false};    // source matching ran for this frame
        bool autopend{false};
        std::uint8_t srcresindex{0x3F};
    };
    // A frame (complete or in reception) stored in the RXFIFO.
    struct Record {
        unsigned size{0};     // bytes written
        unsigned consumed{0}; // bytes read through RFD
        bool complete{false};
    };

    State(Radio &radio, Bus &b) : self(radio), bus(b) { prog.fill(0xD0); }

    Radio &self;
    Bus &bus;

    // ---- host wiring and chip-external state: kept across chip resets ----
    mcuemu::VirtualAir *air{nullptr};
    std::string node;
    std::uint64_t ns_offset{0};
    std::uint64_t seed{0};
    std::uint64_t dma_triggers{0};
    // Radio RAM (23.4). Model-defined: cleared only by a power-on reset
    // (23.4: preserved in sleep modes; SWRU191F gives no reset value).
    std::array<std::uint8_t, RamEnd - RamBase> ram{};

    // ---- interrupt flags RFIRQF0, RFIRQF1, RFERRF (23.1.2) ----
    std::array<std::uint8_t, 3> flags{};
    std::array<std::uint64_t, 3> rmw_time{Never, Never, Never};
    std::array<std::uint8_t, 3> rmw_snapshot{};

    std::uint8_t rxenable{0};

    // ---- radio-control FSM (23.11) ----
    std::uint8_t st{StIdle};
    std::uint64_t fsm_hf{Never}; // timeout of the current FSM state
    bool sfd{false};
    bool sampled_cca{false};
    bool cca_memory{false};
    std::uint64_t rx_on{Never};     // receiver started (RSSI_VALID reference)
    std::uint64_t search_start{0};  // SFD search (re)started
    std::uint64_t poll_hf{Never};   // next air poll in SFD wait
    std::uint64_t lock_hf{Never};   // scheduled SFD lock
    std::uint64_t lock_id{0};
    std::uint64_t energy_scan_start{0};

    // ---- TXFIFO (23.8.3) ----
    unsigned tx_first{0}, tx_last{0};
    bool tx_overflow{false}, tx_underflow{false};
    Tx tx;

    // ---- RXFIFO (23.10) ----
    unsigned rx_first{0}, rx_count{0};
    bool rx_overflow{false};
    std::deque<Record> records;
    bool fifop_end{false};
    bool fifop_level{false};
    Rx rx;
    std::set<std::uint64_t> handled; // air frames already locked or missed

    // ---- command strobe processor (23.14) ----
    std::array<std::uint8_t, 24> prog{};
    unsigned wp{0}, pc{0};
    bool running{false};
    int label{-1};
    std::uint8_t csp_x{0}, csp_y{0}, csp_z{0}, csp_t{0xFF}, csp_ctrl{0};
    std::uint64_t csp_hf{Never}; // next instruction
    CspWait wait{CspWait::None};
    unsigned wait_count{0};
    bool parked{false};
    std::uint64_t park_anchor{0}, park_period{1};
    std::uint64_t park_wake{Never}; // earliest time the condition may change
    std::uint64_t tx_stop_hf{0};    // end of the last transmission (TX final timing)
    std::uint64_t rand_hf{Never};
    std::uint8_t rand_value{0};

    // ==================================================================
    // Registers and helpers
    // ==================================================================
    std::uint8_t reg(std::uint16_t addr) const { return bus.storage(addr); }
    std::uint8_t ram_at(unsigned off) const { return ram[off]; }

    unsigned preamble_bytes() const {
        // MDMCTRL0.PREAMBLE_LENGTH: 0000 = 2 bytes ... (value + 2) bytes.
        return ((reg(MDMCTRL0) >> 1) & 0x0Fu) + 2u;
    }
    unsigned zero_symbols() const {
        // MDMCTRL0.DEM_NUM_ZEROS. Model-defined: the reserved value 00 is
        // treated as 1 zero symbol.
        const unsigned v = (reg(MDMCTRL0) >> 6) & 3u;
        return v == 0 ? 1u : v;
    }
    // IEEE 802.15.4 channel of FREQCTRL.FREQ (23.5, FREQCTRL: f = 2394 +
    // FREQ MHz; channel k <=> FREQ = 11 + 5 (k - 11)), or -1 when the
    // frequency is not an IEEE 802.15.4 channel.
    int channel() const {
        const unsigned freq = reg(FREQCTRL) & 0x7Fu;
        if (freq < 11 || (freq - 11) % 5 != 0 || (freq - 11) / 5 > 15)
            return -1;
        return static_cast<int>(11 + (freq - 11) / 5);
    }
    // Model-defined: frames reach the shared air only on an IEEE 802.15.4
    // channel in the IEEE 802.15.4 compliant modulation mode (MDMTEST1.
    // MODULATION_MODE = 0, 23.9.5 last note).
    bool on_air() const {
        return air && channel() >= 0 && !(reg(MDMTEST1) & MdmTest1ModulationMode);
    }

    // ---- time conversion ----
    std::uint64_t now_of(std::uint64_t hf) const {
        const std::uint64_t hn = bus.hf_now(), n = bus.now();
        return hf <= hn ? n - (hn - hf) : n + (hf - hn);
    }
    std::uint64_t air_ns(std::uint64_t hf) const {
        const std::uint64_t c = now_of(hf);
        return ns_offset + c / 32 * 1000 + c % 32 * 1000 / 32;
    }
    // First HF time whose air time is >= ns (exact inverse of air_ns).
    std::uint64_t hf_of_ns(std::uint64_t ns) const {
        const std::uint64_t d = ns > ns_offset ? ns - ns_offset : 0;
        const std::uint64_t c = d / 1000 * 32 + (d % 1000 * 32 + 999) / 1000;
        const std::uint64_t sleep = bus.now() - bus.hf_now();
        return c > sleep ? c - sleep : 0;
    }

    // ---- interrupt flags ----
    static Reg flag_reg(unsigned i) {
        return sfr_reg(i == 0 ? SfrRfirqf0 : i == 1 ? SfrRfirqf1 : SfrRferrf);
    }
    static std::uint8_t flag_mask(unsigned i) {
        return i == 0 ? Irqf0Mask : i == 1 ? Irqf1Mask : ErrfMask;
    }
    std::uint8_t irq_mask(unsigned i) const {
        return reg(i == 0 ? RFIRQM0 : i == 1 ? RFIRQM1 : RFERRM);
    }
    void raise_cpu(unsigned i) {
        bus.raise_irq(i == 2 ? Irq::RfErr : Irq::Rf);
        if (i < 2)
            rf_capture_edge(true);
    }
    // The RF interrupt (interrupt 16) line of Figure 2-4: an unmasked
    // RFIRQF0/RFIRQF1 flag stands.
    bool rf_line() const {
        return (flags[0] & irq_mask(0)) != 0 || (flags[1] & irq_mask(1)) != 0;
    }
    // Timer 1 channels with T1CCTLn.RFIRQ = 1 capture on the RF interrupt
    // (SWRU191F 9.12; edge rules in Timer1::rf_interrupt_edge).
    void rf_capture_edge(bool rising) {
        if (Timer1 *timer1 = bus.soc().timer1)
            timer1->rf_interrupt_edge(rising);
    }
    void set_flags(unsigned i, std::uint8_t bits) {
        bits &= flag_mask(i);
        const std::uint8_t rising = static_cast<std::uint8_t>(bits & ~flags[i]);
        flags[i] = static_cast<std::uint8_t>(flags[i] | bits);
        bus.storage(flag_reg(i)) = flags[i];
        // 23.1.1: the RF interrupts are rising-edge triggered; flags are
        // updated regardless of the masks (23.1.2). Model-defined: a flag
        // that is already 1 does not request again.
        if (rising & irq_mask(i))
            raise_cpu(i);
    }
    void f0(std::uint8_t bits) { set_flags(0, bits); }
    void f1(std::uint8_t bits) { set_flags(1, bits); }
    void ferr(std::uint8_t bits) { set_flags(2, bits); }

    // DMA trigger 19 (23.3). Model-defined: triggers raised while the radio
    // processes an event or register access are delivered to the DMA
    // controller after that access/event (flush_dma), so a DMA transfer
    // reading RFD never re-enters the radio.
    void trigger_dma() {
        ++dma_triggers;
        ++dma_pending;
    }

    // ==================================================================
    // RXFIFO (23.10)
    // ==================================================================
    // Header bytes of a frame in reception that are not valid for FIFOP
    // until the frame is accepted (23.10.1).
    unsigned invalid_bytes() const {
        if (!rx.active || rx.rejected || !rx.filtering || rx.accepted || records.empty() ||
            records.back().complete)
            return 0;
        const Record &r = records.back();
        return r.size - std::min(r.size, r.consumed);
    }
    bool fifop() const {
        // FSMSTAT1.FIFOP (23.10.1/23.10.2): high during RX overflow, when
        // more than FIFOP_THR valid bytes are stored, and after the last
        // byte of a new frame until the next RXFIFO read.
        if (rx_overflow || fifop_end)
            return true;
        return rx_count - invalid_bytes() > (reg(FIFOPCTRL) & 0x7Fu);
    }
    void update_fifop() {
        const bool level = fifop();
        if (level && !fifop_level)
            f0(F0Fifop);
        fifop_level = level;
    }
    unsigned rx_last() const { return (rx_first + rx_count) % FifoSize; }
    unsigned rxp1() const {
        // Model-defined: RXP1_PTR is the start of the first stored frame of
        // which no byte has been read, or RXLAST_PTR when there is none.
        unsigned pos = rx_first;
        for (const Record &r : records) {
            if (r.consumed == 0)
                return pos;
            pos = (pos + (r.size - r.consumed)) % FifoSize;
        }
        return rx_last();
    }
    // Stores one received byte; false on RX overflow (23.10.2).
    bool rx_push(std::uint8_t byte) {
        if (rx_count >= FifoSize) {
            rx_overflow = true;
            ferr(ErrRxOverf);
            update_fifop();
            return false;
        }
        ram[RxFifoOff + rx_last()] = byte;
        ++rx_count;
        records.back().size++;
        if (rx_count == 1) // 23.3: RXFIFO empty -> nonempty
            trigger_dma();
        update_fifop();
        return true;
    }
    // Removes the unread bytes of the frame in reception (rejected or
    // aborted frame). Model-defined for aborts (23.9.5 documents it for
    // rejected frames, Figure 23-19).
    void rx_drop_current() {
        if (records.empty() || records.back().complete)
            return;
        const Record r = records.back();
        records.pop_back();
        rx_count -= r.size - std::min(r.size, r.consumed);
        update_fifop();
    }
    std::uint8_t rfd_read() {
        if (rx_count == 0) {
            // 23.10.2: RX underflow. Model-defined: returns the byte at
            // RXFIRST_PTR without moving the pointer.
            ferr(ErrRxUnderf);
            return ram[RxFifoOff + rx_first];
        }
        const std::uint8_t v = ram[RxFifoOff + rx_first];
        rx_first = (rx_first + 1) % FifoSize;
        --rx_count;
        bool finished_head = false;
        while (!records.empty()) {
            Record &h = records.front();
            if (h.consumed < h.size) {
                ++h.consumed;
                if (h.complete && h.consumed == h.size) {
                    records.pop_front();
                    finished_head = true;
                }
                break;
            }
            if (!h.complete)
                break;
            records.pop_front();
        }
        // 23.10.1: FIFOP set by a completed frame goes low at the next
        // read; RFIRQF0.FIFOP is raised again when a frame has been read
        // out completely and more complete frames are available.
        const bool more = std::any_of(records.begin(), records.end(),
                                      [](const Record &r) { return r.complete; });
        fifop_end = finished_head && more;
        if (fifop_end)
            f0(F0Fifop);
        if (rx_count > 0) // 23.3: read with more data available
            trigger_dma();
        update_fifop();
        return v;
    }
    void flush_rx(std::uint64_t hf) {
        // 23.10: SFLUSHRX resets the RXFIFO, all pointers, counters, status
        // signals and sticky errors; RXABO if a frame is being received.
        if (rx.active && !rx.rejected)
            ferr(ErrRxAbo);
        abort_rx();
        rx_first = rx_count = 0;
        rx_overflow = false;
        records.clear();
        fifop_end = false;
        fifop_level = false;
        (void)hf;
    }
    void abort_rx() {
        if (rx.active) {
            if (!rx.rejected)
                rx_drop_current();
            handled.insert(rx.id);
            rx.active = false;
            sfd = false;
        }
        lock_hf = Never;
    }

    // ==================================================================
    // RSSI and CCA (23.10.3, 23.8.12)
    // ==================================================================
    bool rx_state() const { return st == StSfdWait || st == StRx || st == StRxWait; }
    std::uint64_t rssi_valid_hf() const {
        return rx_on == Never || !rx_state() ? Never : rx_on + RssiValidCycles;
    }
    bool rssi_valid(std::uint64_t hf) const {
        const std::uint64_t v = rssi_valid_hf();
        return v != Never && hf >= v;
    }
    int noise_floor_dbm() const {
        // Model-defined: an unattached radio hears -100 dBm (the VirtualAir
        // default noise floor).
        return air ? static_cast<int>(std::lround(air->propagation().noise_floor_dbm)) : -100;
    }
    // Strongest signal on the channel at `hf` (dBm). Model-defined: the
    // instantaneous maximum of the frames in flight and the noise floor
    // (the 8-symbol average of 23.10.3 is not modeled).
    int signal_dbm_at_ns(std::uint64_t ns) const {
        int best = noise_floor_dbm();
        if (!on_air())
            return best;
        for (const mcuemu::RadioFrame &f :
             air->in_flight(node, mcuemu::RadioProtocol::Zigbee802154,
                            static_cast<std::uint8_t>(channel()), ns, 0))
            best = std::max(best, f.rssi_dbm);
        return best;
    }
    // RSSI in register units (dBm + offset). With `scan` the ENERGY_SCAN
    // setting applies; CCA always uses the current value (model-defined).
    int rssi_raw(std::uint64_t hf, bool scan) const {
        int dbm = signal_dbm_at_ns(air_ns(hf));
        if (scan && (reg(FRMCTRL0) & Ctrl0EnergyScan)) {
            // 23.10.3: peak since the energy scan was enabled. Evaluated at
            // the window start and at every frame start inside the window.
            const std::uint64_t from = std::max(energy_scan_start, rssi_valid_hf());
            if (from <= hf) {
                const std::uint64_t from_ns = air_ns(from), to_ns = air_ns(hf);
                dbm = std::max(dbm, signal_dbm_at_ns(from_ns));
                if (on_air())
                    for (const mcuemu::RadioFrame &f : air->frames())
                        if (f.start_ns > from_ns && f.start_ns <= to_ns)
                            dbm = std::max(dbm, signal_dbm_at_ns(f.start_ns));
            }
        }
        return dbm + RssiOffset;
    }
    std::uint8_t rssi_reg(std::uint64_t hf) const {
        // RSSI register: -128 (0x80) while not valid.
        if (!rssi_valid(hf))
            return 0x80;
        return static_cast<std::uint8_t>(clamp_s8(rssi_raw(hf, true)));
    }
    // CCA (CCACTRL1.CCA_MODE). `update` stores the hysteresis state.
    bool cca(std::uint64_t hf, bool update) {
        const unsigned mode = (reg(CCACTRL1) >> 3) & 3u;
        if (mode == 0)
            return true;
        // 23.8.12 / Model-defined: CCA is 0 until 4 cycles after RSSI_VALID.
        const std::uint64_t v = rssi_valid_hf();
        if (v == Never || hf < v + CcaDelayCycles)
            return false;
        const bool receiving = st == StRx;
        bool level = true;
        if (mode == 1 || mode == 3) {
            const int r = clamp_s8(rssi_raw(hf, false));
            const int thr = static_cast<std::int8_t>(reg(CCACTRL0));
            const int hyst = reg(CCACTRL1) & 7;
            if (r < thr - hyst)
                level = true;
            else if (r >= thr)
                level = false;
            else // Model-defined: inside the hysteresis band keep the last value.
                level = cca_memory;
            if (update)
                cca_memory = level;
        }
        if (mode == 2 || mode == 3)
            level = level && !receiving;
        return level;
    }
    bool cca_peek(std::uint64_t hf) const { return const_cast<State *>(this)->cca(hf, false); }

    void capture_sfd(std::uint64_t hf) {
        // 22.1.5: SFD (transmitted or received) captures the MAC timer.
        bus.soc().mac_timer->sfd_capture_hf(hf);
    }
    void wake_csp(std::uint64_t hf) {
        if (parked)
            park_wake = std::min(park_wake, hf);
    }

    // ==================================================================
    // Radio-control FSM (23.11, Figure 23-20, Table 23-3)
    // ==================================================================
    void stop_receiver() {
        rx_on = Never;
        poll_hf = Never;
        lock_hf = Never;
    }
    void go_idle() {
        const bool was_idle = st == StIdle;
        st = StIdle;
        fsm_hf = Never;
        sfd = false;
        tx.active = false;
        stop_receiver();
        if (!was_idle) // 23.1.2 RFIDLE: the FSM has entered the idle state
            f1(F1RfIdle);
    }
    void start_rx_cal(std::uint64_t hf) {
        // Figure 23-20: RX calibration (state 2), 192 us, then SFD wait.
        st = StRxCal;
        fsm_hf = hf + TurnCycles;
        sfd = false;
        tx.active = false;
        stop_receiver();
    }
    void enter_sfd_wait(std::uint64_t hf, bool receiver_start) {
        st = StSfdWait;
        fsm_hf = Never;
        sfd = false;
        if (receiver_start || rx_on == Never) {
            rx_on = hf;
            // Model-defined: the CCA hysteresis state starts "busy".
            cca_memory = false;
        }
        search_start = hf;
        poll(hf);
    }
    // Aborts reception and transmission (strobes SRXON, STXON, STXONCCA,
    // SRFOFF, 23.8.1). A frame already registered on the virtual air stays
    // there (VirtualAir cannot withdraw frames).
    void abort_activity() {
        if (rx.active && !rx.rejected) // 23.1.2 RXABO
            ferr(ErrRxAbo);
        abort_rx();
        tx.active = false;
        sfd = false;
    }
    void set_rxenable(std::uint8_t value, std::uint64_t hf, bool fsm_effect) {
        const std::uint8_t old = rxenable;
        rxenable = value;
        bus.storage(RXENABLE) = rxenable;
        if (old != 0 && value == 0)
            f0(F0RxMaskZero); // 23.1.2 RXMASKZERO
        if (!fsm_effect)
            return;
        // RXENABLE register: a nonzero value enables the receiver when in
        // idle; Figure 23-20: rxenable = 0 returns the receiver to idle.
        // Model-defined: an ongoing frame reception, transmission or ACK is
        // completed first (the FSM checks RXENABLE when it ends).
        if (old == 0 && value != 0 && st == StIdle)
            start_rx_cal(hf);
        else if (old != 0 && value == 0 &&
                 (st == StRxCal || st == StSfdWait || st == StRxWait || st == StRxOverflow))
            go_idle();
    }

    // ==================================================================
    // Command strobes (23.14.9, low nibble of Sxxx / ISxxx)
    // ==================================================================
    void strobe(unsigned code, std::uint64_t hf) {
        switch (code) {
        case 0x3: // SRXON: RXENABLE[7], abort, RX calibration (23.8.1)
            set_rxenable(rxenable | RxEnSrxon, hf, false);
            abort_activity();
            start_rx_cal(hf);
            break;
        case 0x4: // SRXMASKBITSET
            set_rxenable(rxenable | RxEnMaskBit, hf, true);
            break;
        case 0x5: // SRXMASKBITCLR
            set_rxenable(static_cast<std::uint8_t>(rxenable & ~RxEnMaskBit), hf, true);
            break;
        case 0x6: // SACK
        case 0x7: // SACKPEND
        case 0x8: // SNACK
            // 23.9.8 Manual control: only during frame reception (Figure
            // 23-18, accepted or rejected frame); the last strobe wins.
            if (st == StRx && rx.active)
                rx.ack_strobe = code;
            else
                ferr(ErrStrobe);
            break;
        case 0x9: // STXON (23.8.1): SAMPLED_CCA not updated
            if (reg(FRMCTRL1) & Ctrl1SetRxEnMaskOnTx)
                set_rxenable(rxenable | RxEnStxon, hf, false);
            start_tx_cal(hf);
            break;
        case 0xA: { // STXONCCA: SAMPLED_CCA updated, TX only if CCA = 1
            const bool clear = cca(hf, true);
            sampled_cca = clear;
            if (clear)
                start_tx_cal(hf);
            break;
        }
        case 0xB: // SSAMPLECCA
            sampled_cca = cca(hf, true);
            break;
        case 0xD: // SFLUSHRX (23.10)
            flush_rx(hf);
            // Figure 23-20: SFLUSHRX in any RX state -> RX calibration.
            if (st == StRxCal || st == StSfdWait || st == StRx || st == StRxWait ||
                st == StRxOverflow) {
                if (rxenable)
                    start_rx_cal(hf);
                else
                    go_idle();
            }
            break;
        case 0xE: // SFLUSHTX (23.8.3/23.8.5)
            if (tx.active && !tx.ack) {
                tx.checked = fetches_done(hf);
                tx.flushed = true;
            }
            tx_first = tx_last = 0;
            tx_overflow = tx_underflow = false;
            break;
        case 0xF: // SRFOFF (23.8.1, RFERRF.STROBEERR)
            if (st == StIdle)
                ferr(ErrStrobe);
            set_rxenable(0, hf, false);
            abort_activity();
            go_idle();
            break;
        default: // 0x0/0x1/0xC: SNOP; 0x2 (SSTOP) is handled by the CSP
            break;
        }
    }

    // ==================================================================
    // Transmission (23.8)
    // ==================================================================
    bool ignore_underflow() const { return reg(FRMCTRL1) & Ctrl1IgnoreTxUnderf; }
    unsigned tx_avail() const { return tx_last > tx.base ? tx_last - tx.base : 0; }
    // TXFIFO bytes a frame of `length` reads (Figure 23-5).
    static unsigned frame_fetches(unsigned length, bool autocrc) {
        if (autocrc)
            return 1 + (length >= 2 ? length - 2 : 0);
        return 1 + length;
    }
    // Number of TXFIFO fetches done at `hf`. Model-defined: byte j of the
    // PHY payload (j = 0 is the length byte) is fetched when its
    // transmission starts, at SFD + j * 32 us.
    unsigned fetches_done(std::uint64_t hf) const {
        if (!tx.active || !tx.sfd_done || tx.ack || hf < tx.sfd_hf)
            return 0;
        const std::uint64_t n = (hf - tx.sfd_hf) / ByteCycles + 1;
        return static_cast<unsigned>(std::min<std::uint64_t>(n, tx.fetches));
    }
    std::uint8_t tx_ram(unsigned index) const { return ram[TxFifoOff + index % FifoSize]; }
    void fix_length() {
        if (tx.length_known)
            return;
        // PHR: bit 7 of the length byte is reserved (Model-defined: ignored).
        tx.length = tx_ram(tx.base) & 0x7Fu;
        // Model-defined: FRMCTRL0.AUTOCRC is sampled with the length byte.
        tx.autocrc = reg(FRMCTRL0) & Ctrl0AutoCrc;
        tx.fetches = frame_fetches(tx.length, tx.autocrc);
        tx.end_hf = tx.sfd_hf + ByteCycles * (1 + tx.length);
        tx.length_known = true;
    }
    void start_tx_cal(std::uint64_t hf) {
        // 23.8.2: preamble starts 192 us after STXON/STXONCCA.
        abort_activity();
        st = StTxCal;
        fsm_hf = hf + TurnCycles;
        stop_receiver();
    }
    void start_tx(std::uint64_t hf) {
        st = StTx;
        fsm_hf = Never;
        tx = Tx{};
        tx.active = true;
        tx.start_hf = hf;
        tx.base = tx_first;
        tx.sfd_hf = hf + ByteCycles * (preamble_bytes() + 1);
        try_register();
    }
    void start_ack(std::uint64_t hf, std::vector<std::uint8_t> mpdu) {
        st = StAck;
        fsm_hf = Never;
        tx = Tx{};
        tx.active = true;
        tx.ack = true;
        tx.start_hf = hf;
        tx.sfd_hf = hf + ByteCycles * (preamble_bytes() + 1);
        tx.length = 5;
        tx.length_known = true;
        tx.end_hf = tx.sfd_hf + ByteCycles * (1 + tx.length);
        tx.ack_mpdu = std::move(mpdu);
        if (on_air())
            air_transmit(tx.sfd_hf, tx.ack_mpdu);
        tx.registered = true;
    }
    void air_transmit(std::uint64_t sfd_hf, std::vector<std::uint8_t> payload) {
        // The air frame starts 160 us (4-byte preamble + SFD) before the
        // SFD end, whatever MDMCTRL0.PREAMBLE_LENGTH is, so that receivers
        // see the SFD at the correct time.
        const std::uint64_t sfd_ns = air_ns(sfd_hf);
        const std::uint64_t start = sfd_ns >= AirSfdNs ? sfd_ns - AirSfdNs : 0;
        air->transmit(node, "", mcuemu::RadioProtocol::Zigbee802154,
                      static_cast<std::uint8_t>(channel()), start, std::move(payload));
    }
    // Registers the TXFIFO frame on the air once its content is known:
    // when all bytes the frame needs are in the TXFIFO, or immediately with
    // IGNORE_TX_UNDERF (Model-defined: the RAM content at that time).
    void try_register() {
        if (!tx.active || tx.ack || tx.registered || st != StTx)
            return;
        if (!ignore_underflow()) {
            if (tx_underflow || tx_avail() == 0)
                return;
            const unsigned length = tx_ram(tx.base) & 0x7Fu;
            const bool autocrc = reg(FRMCTRL0) & Ctrl0AutoCrc;
            if (tx_avail() < frame_fetches(length, autocrc))
                return;
        }
        fix_length();
        tx.registered = true;
        // Model-defined: frames with LEN < 2 (no room for the FCS), TX test
        // modes (FRMCTRL0.TX_MODE != 0) and off-channel frequencies are not
        // placed on the IEEE 802.15.4 air.
        if (tx.length < 2 || (reg(FRMCTRL0) & 3u) != 0 || !on_air())
            return;
        std::vector<std::uint8_t> payload;
        for (unsigned i = 1; i + 1 < tx.length; ++i)
            payload.push_back(tx_ram(tx.base + i));
        if (!tx.autocrc) {
            // AUTOCRC = 0: the frame carries the FCS written by software.
            // The virtual air cannot carry a corrupt FCS: a wrong FCS is
            // reported as an unmodeled feature and the frame is sent intact.
            const std::uint16_t fcs = fcs16(payload.data(), payload.size());
            if (tx_ram(tx.base + tx.length - 1) != (fcs & 0xFF) ||
                tx_ram(tx.base + tx.length) != (fcs >> 8))
                bus.note_unmodeled(sfr_reg(SfrRfd), true);
        }
        air_transmit(tx.sfd_hf, std::move(payload));
    }
    std::uint64_t tx_underflow_hf() const {
        if (!tx.active || tx.ack || !tx.sfd_done || ignore_underflow())
            return Never;
        const unsigned j = std::max(tx.flushed ? 0u : tx_avail(), tx.checked);
        if (j >= tx.fetches)
            return Never;
        return tx.sfd_hf + ByteCycles * j;
    }
    std::uint64_t tx_event_hf() const {
        if (!tx.active)
            return Never;
        if (!tx.sfd_done)
            return tx.sfd_hf;
        return std::min(tx_underflow_hf(), tx.end_hf);
    }
    void tx_event(std::uint64_t hf) {
        if (!tx.sfd_done) {
            tx_sfd(hf);
            return;
        }
        const std::uint64_t u = tx_underflow_hf();
        if (u == hf) {
            tx_underflow_at(hf, std::max(tx.flushed ? 0u : tx_avail(), tx.checked));
            return;
        }
        tx_end(hf);
    }
    void tx_sfd(std::uint64_t hf) {
        tx.sfd_done = true;
        sfd = true;
        f0(F0Sfd);
        capture_sfd(hf);
        if (tx.ack)
            return;
        // Fetch 0 (length byte) at the SFD.
        if (!ignore_underflow() && (tx_underflow || tx_avail() == 0)) {
            tx_underflow_at(hf, 0);
            return;
        }
        fix_length();
        tx.checked = 1;
    }
    void tx_underflow_at(std::uint64_t hf, unsigned fetched) {
        // 23.8.5: TX underflow aborts the transmission; 23.8.2: 2 us ramp.
        ferr(ErrTxUnderf);
        tx_underflow = true;
        if (!tx.flushed)
            tx_first = tx.base + fetched;
        tx.active = false;
        st = StTxUnderflow;
        fsm_hf = hf + RampCycles;
        tx_stop_hf = hf;
    }
    void tx_end(std::uint64_t hf) {
        tx.active = false;
        if (tx.ack) {
            f1(F1TxAckDone);
        } else {
            if (!tx.registered && !tx.flushed) { // IGNORE_TX_UNDERF set late
                tx.active = true;
                st = StTx;
                try_register();
                tx.active = false;
            }
            f1(F1TxDone);
            // 23.8.4: the TXFIFO content is kept for retransmission.
            // Model-defined: TXFIRST_PTR returns to the frame start.
            if (!tx.flushed)
                tx_first = tx.base;
        }
        // 23.8.2: SFD stretched by 2 us, FSM leaves TX 2 us later.
        st = StTxFinal;
        fsm_hf = hf + RampCycles;
        tx_stop_hf = hf;
    }
    unsigned tx_first_reg(std::uint64_t hf) const {
        if (tx.active && !tx.ack && st == StTx && tx.sfd_done && !tx.flushed)
            return tx.base + fetches_done(hf);
        return tx_first;
    }

    // ==================================================================
    // Reception (23.9)
    // ==================================================================
    bool can_search() const {
        // FRMCTRL0.RX_MODE = 11: symbol search disabled (23.9.5 tips).
        return st == StSfdWait && on_air() && ((reg(FRMCTRL0) >> 2) & 3u) != 3u && !rx_overflow;
    }
    std::uint64_t first_lockable_sfd() const {
        // MDMCTRL0.DEM_NUM_ZEROS zero symbols plus the 2-symbol SFD must be
        // received after the search started.
        return search_start + (2 + zero_symbols()) * SymbolCycles;
    }
    void poll(std::uint64_t hf) {
        poll_hf = st == StSfdWait ? hf + PollCycles : Never;
        if (!can_search() || lock_hf != Never)
            return;
        const std::uint8_t ch = static_cast<std::uint8_t>(channel());
        const std::vector<mcuemu::RadioFrame> frames =
            air->in_flight(node, mcuemu::RadioProtocol::Zigbee802154, ch, air_ns(hf), 0);
        const mcuemu::RadioFrame *best = nullptr;
        std::uint64_t best_s = Never;
        for (const mcuemu::RadioFrame &f : frames) {
            if (handled.count(f.id))
                continue;
            const std::uint64_t s = hf_of_ns(f.start_ns + AirSfdNs);
            // aMaxPHYPacketSize = 127 (PHR); SFD before the search: missed.
            if (f.payload.size() + 2 > 127 || s < first_lockable_sfd()) {
                handled.insert(f.id);
                continue;
            }
            if (s < best_s || (s == best_s && best && f.id < best->id)) {
                best = &f;
                best_s = s;
            }
        }
        if (handled.size() > 64) {
            // Frames no longer in flight have ended and never come back.
            std::set<std::uint64_t> live;
            for (const mcuemu::RadioFrame &f : frames)
                if (handled.count(f.id))
                    live.insert(f.id);
            handled.swap(live);
        }
        if (!best)
            return;
        if (best_s > hf) {
            lock_hf = best_s;
            lock_id = best->id;
        } else {
            lock(*best, best_s);
        }
    }
    void lock_event(std::uint64_t hf) {
        const std::uint64_t id = lock_id;
        lock_hf = Never;
        if (!can_search())
            return;
        for (const mcuemu::RadioFrame &f :
             air->in_flight(node, mcuemu::RadioProtocol::Zigbee802154,
                            static_cast<std::uint8_t>(channel()), air_ns(hf), 0))
            if (f.id == id) {
                lock(f, hf);
                return;
            }
        handled.insert(id);
        poll(hf);
    }
    void lock(const mcuemu::RadioFrame &f, std::uint64_t s) {
        handled.insert(f.id);
        lock_hf = Never;
        rx = Rx{};
        rx.active = true;
        rx.id = f.id;
        rx.channel = f.channel;
        rx.end_ns = f.end_ns;
        rx.mpdu = f.payload;
        rx.length = static_cast<unsigned>(f.payload.size()) + 2;
        rx.sfd_hf = s;
        rx.end_hf = s + ByteCycles * (rx.length + 1);
        rx.rssi_dbm = f.rssi_dbm;
        // Model-defined: the frame processing settings are sampled at the SFD.
        rx.filtering = reg(FRMFILT0) & Filt0Enable;
        rx.autocrc = reg(FRMCTRL0) & Ctrl0AutoCrc;
        rx.append = reg(FRMCTRL0) & Ctrl0AppendData;
        st = StRx;
        sfd = true;
        f0(F0Sfd);
        capture_sfd(s);
        records.push_back(Record{});
    }
    // A scheduled lock (SFD wait) and a locked frame never coexist.
    std::uint64_t rx_event_hf() const {
        if (lock_hf != Never)
            return lock_hf;
        if (!rx.active)
            return Never;
        return rx.rejected ? rx.end_hf : rx.sfd_hf + ByteCycles * (rx.next + 1);
    }
    void rx_event(std::uint64_t hf) {
        if (lock_hf != Never) {
            lock_event(hf);
            return;
        }
        if (rx.rejected) {
            // 23.9.5: the search resumes after the rejected frame.
            rx.active = false;
            if (rxenable)
                enter_sfd_wait(hf, false);
            else
                go_idle();
            return;
        }
        rx_byte(hf);
    }
    std::uint16_t le16(unsigned off) const {
        return static_cast<std::uint16_t>(rx.mpdu[off] | rx.mpdu[off + 1] << 8);
    }
    std::uint16_t ram16(unsigned off) const {
        return static_cast<std::uint16_t>(ram[off] | ram[off + 1] << 8);
    }
    bool verdict() {
        // Collision/SINR outcome of the virtual air at the frame end.
        for (const mcuemu::RadioFrame &f :
             air->receive(node, mcuemu::RadioProtocol::Zigbee802154, rx.channel, rx.end_ns,
                          rx.end_ns - 1))
            if (f.id == rx.id)
                return true;
        return false;
    }
    void rx_byte(std::uint64_t hf) {
        const unsigned i = rx.next++;
        const unsigned len = rx.length;
        std::uint8_t b = 0;
        if (i == 0) {
            b = static_cast<std::uint8_t>(len);
        } else if (i + 1 < len) {
            b = rx.mpdu[i - 1];
        } else {
            if (!rx.verdict_done && (!rx.autocrc || i == len)) {
                rx.crc_ok = verdict();
                rx.verdict_done = true;
            }
            if (rx.autocrc) {
                // Figure 23-15: RSSI (signed), then CRC_OK | correlation or
                // SRCRESINDEX (APPEND_DATA_MODE).
                if (i + 1 == len) {
                    b = static_cast<std::uint8_t>(clamp_s8(rx.rssi_dbm + RssiOffset));
                } else {
                    const std::uint8_t low = rx.append ? rx.srcresindex : Correlation;
                    b = static_cast<std::uint8_t>((rx.crc_ok ? 0x80 : 0x00) | (low & 0x7F));
                }
            } else {
                // AUTOCRC = 0: the FCS itself. Model-defined: a frame lost on
                // the air (collision/SINR) delivers an inverted FCS.
                const std::uint16_t fcs = fcs16(rx.mpdu.data(), rx.mpdu.size());
                b = static_cast<std::uint8_t>(i + 1 == len ? fcs & 0xFF : fcs >> 8);
                if (!rx.crc_ok)
                    b = static_cast<std::uint8_t>(b ^ 0xFF);
            }
        }
        if (!rx_push(b)) {
            // 23.10.2: RX overflow halts reception (state 17).
            rx.active = false;
            sfd = false;
            st = StRxOverflow;
            fsm_hf = Never;
            stop_receiver();
            return;
        }
        if (i == 2 && len >= 3) {
            rx.mhr = parse_mhr(le16(0));
            rx.have_mhr = true;
        }
        if (rx.filtering && !rx.accepted) {
            if ((i == 0 && len < 3) || (i == 2 && !filter_fcf())) {
                reject();
                return;
            }
            if (rx.have_mhr && i == rx.mhr.fields_end) {
                if (!filter_addresses()) {
                    reject();
                    return;
                }
                accept();
            }
        }
        if (rx.accepted && rx.match_enabled && rx.mhr.src_mode != 0 && i == rx.mhr.src_end)
            source_match();
        if (i == len)
            rx_complete(hf);
    }
    // 23.9.5 first stage, once the FCF is received.
    bool filter_fcf() {
        const Mhr &m = rx.mhr;
        const std::uint8_t f0r = reg(FRMFILT0), f1r = reg(FRMFILT1);
        unsigned type = m.type;
        switch ((f1r >> 1) & 3u) { // MODIFY_FT_FILTER on the type MSB
        case 1: type ^= 4u; break;
        case 2: type &= 3u; break;
        case 3: type |= 4u; break;
        default: break;
        }
        rx.ftype = type;
        const unsigned len = rx.length;
        if (m.reserved & ((f0r >> 4) & 7u))
            return false;
        if (m.version > ((f0r >> 2) & 3u))
            return false;
        if (m.dst_mode == 1 || m.src_mode == 1)
            return false;
        if (len < m.src_end + 2) // minimum length from the address fields
            return false;
        switch (type) {
        case 0: // beacon
            return (f1r & 0x08) && len >= 9 && m.dst_mode == 0 && m.src_mode >= 2;
        case 1: // data
        case 3: // MAC command
            if (!(f1r & (type == 1 ? 0x10 : 0x40)) || len < 9)
                return false;
            if (m.dst_mode == 0 && m.src_mode == 0)
                return false;
            return m.dst_mode != 0 || (f0r & Filt0PanCoordinator);
        case 2: // acknowledgment
            return (f1r & 0x20) && len == 5;
        default: // reserved types 4-7
            return (f1r & 0x80) && len >= 9;
        }
    }
    // 23.9.5 second stage, once FCF + SEQ + destination + source PAN ID
    // have been received.
    bool filter_addresses() const {
        const Mhr &m = rx.mhr;
        const std::uint16_t pan = ram16(PanIdOff);
        if (m.has_dst_pan) {
            const std::uint16_t dpan = le16(m.dst_pan_off);
            if (dpan != pan && dpan != 0xFFFF)
                return false;
        }
        if (m.dst_mode == 2) {
            const std::uint16_t dst = le16(m.dst_addr_off);
            if (dst != ram16(ShortAddrOff) && dst != 0xFFFF)
                return false;
        } else if (m.dst_mode == 3) {
            for (unsigned k = 0; k < 8; ++k)
                if (rx.mpdu[m.dst_addr_off + k] != ram[ExtAddrOff + k])
                    return false;
        }
        if (rx.ftype == 0 && m.has_src_pan) {
            const std::uint16_t span = le16(m.src_pan_off);
            if (span != pan && pan != 0xFFFF)
                return false;
        }
        if ((rx.ftype == 1 || rx.ftype == 3) && m.dst_mode == 0 && m.has_src_pan &&
            le16(m.src_pan_off) != pan)
            return false;
        return true;
    }
    void reject() {
        // 23.9.5 / Figure 23-19: SFD goes low, the frame is removed from
        // the RXFIFO, the search resumes after the frame.
        rx.rejected = true;
        sfd = false;
        rx_drop_current();
    }
    void accept() {
        rx.accepted = true;
        f0(F0FrameAccepted);
        update_fifop(); // header bytes become valid (23.10.1)
        // 23.9.6: source matching runs only on accepted frames.
        rx.match_enabled = reg(SRCMATCH) & SrcMatchEnable;
        if (rx.match_enabled && rx.mhr.src_mode == 0)
            source_match();
    }
    void source_match() {
        // 23.9.6, Figure 23-13.
        const Mhr &m = rx.mhr;
        std::uint32_t resmask = 0;
        std::uint8_t index = 0x3F;
        bool pend = false;
        if (m.src_mode == 2) {
            // srcPanid: the source PAN ID, or the destination PAN ID with
            // PAN ID compression (IEEE 802.15.4-2006 7.2.1.1.5).
            const std::uint16_t span =
                m.has_src_pan ? le16(m.src_pan_off) : m.has_dst_pan ? le16(m.dst_pan_off) : 0;
            const std::uint16_t sshort = le16(m.src_addr_off);
            const std::uint32_t en = reg(SRCSHORTEN0) | reg(SRCSHORTEN1) << 8 |
                                     static_cast<std::uint32_t>(reg(SRCSHORTEN2)) << 16;
            const std::uint32_t pend_en = ram[SrcShortPendEnOff] |
                                          ram[SrcShortPendEnOff + 1] << 8 |
                                          static_cast<std::uint32_t>(ram[SrcShortPendEnOff + 2]) << 16;
            for (unsigned n = 0; n < 24; ++n) {
                const std::uint32_t bit = 1u << n;
                if (!(en & bit))
                    continue;
                if (ram16(SrcTableOff + 4 * n) == span && ram16(SrcTableOff + 4 * n + 2) == sshort) {
                    resmask |= bit;
                    if (index == 0x3F)
                        index = static_cast<std::uint8_t>(n);
                    if (pend_en & bit)
                        pend = true;
                }
            }
        } else if (m.src_mode == 3) {
            const std::uint32_t en = reg(SRCEXTEN0) | reg(SRCEXTEN1) << 8 |
                                     static_cast<std::uint32_t>(reg(SRCEXTEN2)) << 16;
            const std::uint32_t pend_en = ram[SrcExtPendEnOff] | ram[SrcExtPendEnOff + 1] << 8 |
                                          static_cast<std::uint32_t>(ram[SrcExtPendEnOff + 2]) << 16;
            for (unsigned n = 0; n < 12; ++n) {
                const std::uint32_t bit = 1u << (2 * n); // EXT_ADDR_EN[2n]
                if (!(en & bit))
                    continue;
                bool equal = true;
                for (unsigned k = 0; k < 8 && equal; ++k)
                    equal = rx.mpdu[m.src_addr_off + k] == ram[SrcTableOff + 8 * n + k];
                if (equal) {
                    resmask |= 3u << (2 * n);
                    if (index == 0x3F)
                        index = static_cast<std::uint8_t>(n | 0x20);
                    if (pend_en & bit) // SRCEXTPENDEN[2n]
                        pend = true;
                }
            }
        }
        // 23.9.8 AUTOPEND. PEND_DATAREQ_ONLY: an unsecured MAC command frame
        // with command identifier 0x04 (data request, IEEE 802.15.4-2006
        // 7.3). Model-defined: the command identifier, which follows the
        // source address, is taken from the frame when matching completes.
        const std::uint8_t srcmatch = reg(SRCMATCH);
        bool autopend = (srcmatch & SrcAutoPend) && pend;
        if (autopend && (srcmatch & SrcPendDataReqOnly))
            autopend = m.type == 3 && !m.security && rx.mpdu.size() > m.src_end &&
                       rx.mpdu[m.src_end] == 0x04;
        if (autopend)
            index = static_cast<std::uint8_t>(index | 0x40);
        ram[SrcResMaskOff] = static_cast<std::uint8_t>(resmask);
        ram[SrcResMaskOff + 1] = static_cast<std::uint8_t>(resmask >> 8);
        ram[SrcResMaskOff + 2] = static_cast<std::uint8_t>(resmask >> 16);
        ram[SrcResIndexOff] = index;
        rx.srcresindex = index;
        rx.autopend = autopend;
        rx.matched = true;
        if (resmask)
            f0(F0SrcMatchFound);
        f0(F0SrcMatchDone);
    }
    void rx_complete(std::uint64_t hf) {
        // 23.9.4 / 23.10.1: last byte received.
        rx.active = false;
        sfd = false;
        records.back().complete = true;
        fifop_end = true;
        f0(F0Fifop);
        update_fifop();
        f0(F0RxPktDone);
        // 23.9.8: acknowledgment decision.
        bool ack = false;
        if (rx.crc_ok && rx.mpdu.size() >= 3) {
            if (rx.ack_strobe == 0x6 || rx.ack_strobe == 0x7)
                ack = true;
            else if (rx.ack_strobe == 0)
                ack = rx.filtering && rx.accepted && (reg(FRMCTRL0) & Ctrl0AutoAck) &&
                      rx.have_mhr && rx.mhr.ack_request && rx.mhr.type != 0 && rx.mhr.type != 2;
        }
        if (ack) {
            const bool pending =
                rx.ack_strobe == 0x7 || (reg(FRMCTRL1) & Ctrl1PendingOr) || rx.autopend;
            // Figure 23-16: FCF (frame type 2, frame pending), DSN, FCS.
            // Model-defined: the remaining FCF bits (frame version 0) are 0.
            pending_ack = {static_cast<std::uint8_t>(pending ? 0x12 : 0x02), 0x00, rx.mpdu[2]};
            st = StAckCal;
            if (reg(FSMCTRL) & FsmSlottedAck) {
                // FSMCTRL.SLOTTED_ACK = 1 (23.15 FSMCTRL, Figure 23-17): the
                // ACK is sent at the first backoff-slot boundary 12 symbol
                // periods or more after the frame. Model-defined: a boundary
                // exactly 12 symbol periods after the frame qualifies
                // (Figure 23-17 "12-31 symbol periods" and IEEE 802.15.4
                // aTurnaroundTime; the FSMCTRL text says "more than"); the
                // backoff-slot boundary is the MAC timer overflow (t2_per
                // wrap, Chapter 22: the time base for time-slotted
                // protocols); the bit is sampled at the frame end; while the
                // MAC timer does not run there is no boundary and the FSM
                // stays in ACK calibration until a strobe ends it.
                ack_slot_from = hf + TurnCycles;
                fsm_hf = Never;
            } else {
                fsm_hf = hf + TurnCycles; // unslotted: 12 symbols after the frame
            }
            stop_receiver();
            return;
        }
        if (!rxenable) {
            go_idle();
        } else if (reg(FSMCTRL) & FsmRx2RxTimeOff) {
            st = StRxWait; // FSMCTRL.RX2RX_TIME_OFF: 12-symbol time-out
            fsm_hf = hf + TurnCycles;
            poll_hf = Never;
        } else {
            enter_sfd_wait(hf, false);
        }
    }
    std::vector<std::uint8_t> pending_ack;
    // Slotted ACK: earliest boundary time, Never when not waiting.
    std::uint64_t ack_slot_from{Never};

    // ==================================================================
    // Command strobe processor (23.14)
    // ==================================================================
    // Model-defined: the CSP executes one instruction per 32 MHz cycle;
    // ISSTART at t executes instruction 0 at t + 1; a command strobe
    // instruction takes effect when it executes.
    unsigned park_from{0}; // first instruction of a parked loop
    unsigned dma_pending{0};
    bool dma_busy{false};

    bool csp_condition(unsigned c, std::uint64_t hf) {
        switch (c & 7u) { // 23.14.9.16/17 condition codes
        case 0: return cca(hf, true);
        case 1: return sfd;
        case 2: return csp_ctrl & 1u;
        case 3: return false; // Model-defined: the reserved condition is false
        case 4: return csp_x == 0;
        case 5: return csp_y == 0;
        case 6: return csp_z == 0;
        default: return rssi_valid(hf);
        }
    }
    void csp_stop(bool irq) {
        // 23.14.3: execution stops; SSTOP/ISSTOP invalidate the label.
        running = false;
        parked = false;
        csp_hf = Never;
        wait = CspWait::None;
        label = -1;
        if (irq) // 23.14.4 IRQ_CSP_STOP
            f1(F1CspStop);
    }
    void csp_start(std::uint64_t hf) {
        // 23.14.9.33 ISSTART: PC = 0, start execution. "Do not issue an
        // ISSTART instruction if CSP is already running": Model-defined, the
        // program restarts and the access is counted as unmodeled.
        if (running)
            bus.note_unmodeled(sfr_reg(SfrRfst), true);
        running = true;
        parked = false;
        pc = 0;
        label = -1;
        wait = CspWait::None;
        csp_hf = hf + 1;
        // 23.14.3: execution stops when CSPT is zero (Model-defined: also
        // when CSPT is already zero at ISSTART).
        if (csp_t == 0)
            csp_stop(true);
    }
    // Continue after WAIT/WAITX/WEVENTn at the next cycle.
    void csp_continue(std::uint64_t hf, bool wait_irq) {
        wait = CspWait::None;
        if (wait_irq) // 23.14.4 IRQ_CSP_WT
            f1(F1CspWait);
        csp_advance(pc + 1, hf + 1);
    }
    void csp_advance(unsigned next, std::uint64_t at) {
        if (next > 23) { // 23.14.3: last instruction executed / skipped past the end
            csp_stop(true);
            return;
        }
        pc = next;
        csp_hf = at;
    }
    bool loop_body_is_nop(unsigned from, unsigned to) const {
        for (unsigned k = from; k < to; ++k)
            if (!csp_is_nop(prog[k]))
                return false;
        return true;
    }
    // Model-defined: a busy loop whose body has no effect (SKIP with S = 0,
    // or RPT jumping back over SNOPs only) is "parked": its condition is
    // re-evaluated at the first loop iteration at or after the earliest time
    // the condition can change, which gives the same execution times as
    // stepping every cycle.
    void csp_park(unsigned at, unsigned from, unsigned cond, std::uint64_t hf) {
        parked = true;
        pc = at;
        park_from = from;
        park_anchor = hf;
        park_period = at - from + 1;
        park_wake = csp_cond_change_hf(cond, hf);
        csp_hf = Never;
    }
    std::uint64_t csp_cond_change_hf(unsigned cond, std::uint64_t hf) const {
        const std::uint64_t valid = rssi_valid_hf();
        switch (cond & 7u) {
        case 0: { // CCA: RSSI_VALID, frame boundaries on the channel
            std::uint64_t w = hf + ParkCycles;
            if (valid != Never && valid + CcaDelayCycles > hf)
                w = std::min(w, valid + CcaDelayCycles);
            if (on_air()) {
                const std::uint64_t now_ns = air_ns(hf);
                const auto ch = static_cast<std::uint8_t>(channel());
                for (const mcuemu::RadioFrame &f : air->frames()) {
                    if (f.source == node || f.channel != ch ||
                        f.protocol != mcuemu::RadioProtocol::Zigbee802154)
                        continue;
                    if (f.start_ns > now_ns)
                        w = std::min(w, hf_of_ns(f.start_ns));
                    if (f.end_ns > now_ns)
                        w = std::min(w, hf_of_ns(f.end_ns));
                }
            }
            return std::max(w, hf + 1);
        }
        case 7: // RSSI_VALID
            return valid != Never && valid > hf ? valid : Never;
        default: // changed only by radio events or CPU writes (wake_csp)
            return Never;
        }
    }
    std::uint64_t csp_align(std::uint64_t x) const {
        if (x <= park_anchor)
            return park_anchor + park_period;
        const std::uint64_t k = (x - park_anchor + park_period - 1) / park_period;
        return park_anchor + k * park_period;
    }
    std::uint64_t csp_next_hf() const {
        if (!running)
            return Never;
        if (parked)
            return park_wake == Never ? Never : csp_align(std::max(park_wake, park_anchor + 1));
        if (wait != CspWait::None)
            return Never;
        return csp_hf;
    }
    // Index of the next instruction to execute (CSPSTAT.CSP_PC, RFST read).
    unsigned pc_at(std::uint64_t hf) const {
        if (!running || !parked || park_period == 1 || hf < park_anchor)
            return pc;
        const std::uint64_t d = (hf - park_anchor) % park_period;
        return (d + 1) % park_period == 0 ? pc : park_from + static_cast<unsigned>(d);
    }
    void randxy(std::uint64_t hf) {
        // 23.14.9.8: X[Y-1:0] = random, X[7:Y] = 0; Y = 0 or Y > 7: 8 bits.
        // A RANDXY within 13 cycles of the previous one uses the same value.
        if (rand_hf == Never || hf - rand_hf >= 13) {
            rand_value = static_cast<std::uint8_t>(mix64(seed ^ mix64(hf ^ 0x52414E44ull)));
            rand_hf = hf;
        }
        const unsigned y = csp_y;
        const std::uint8_t mask = (y == 0 || y > 7) ? 0xFF : static_cast<std::uint8_t>((1u << y) - 1u);
        csp_x = rand_value & mask;
    }
    void csp_step(std::uint64_t hf) {
        parked = false;
        const unsigned at = pc;
        const std::uint8_t op = prog[at];
        unsigned next = at + 1;
        if (op < 0x80) { // SKIP S, C (23.14.9.17)
            const unsigned s = (op >> 4) & 7u;
            if (csp_condition(op & 7u, hf) != ((op & 8u) != 0)) {
                if (s == 0) {
                    // Table 23-4: S = 0 re-executes the SKIP (busy loop).
                    csp_park(at, at, op & 7u, hf);
                    return;
                }
                next = at + s + 1;
            }
        } else if (op < 0xA0) { // WAIT W (23.14.9.12)
            wait = CspWait::Overflows;
            wait_count = (op & 0x1Fu) ? (op & 0x1Fu) : 32u;
            csp_hf = Never;
            return;
        } else if (op < 0xB0) { // RPT C (23.14.9.16)
            if (label >= 0 && csp_condition(op & 7u, hf) != ((op & 8u) != 0)) {
                const unsigned from = static_cast<unsigned>(label);
                if (from <= at && loop_body_is_nop(from, at)) {
                    csp_park(at, from, op & 7u, hf);
                    return;
                }
                next = from;
            }
        } else {
            switch (op) {
            case 0xB8: // WEVENT1 (23.14.9.13)
                wait = CspWait::Event1;
                csp_hf = Never;
                return;
            case 0xB9: // WEVENT2
                wait = CspWait::Event2;
                csp_hf = Never;
                return;
            case 0xBA: // INT (23.14.9.9)
                f1(F1CspManInt);
                break;
            case 0xBB: // LABEL (23.14.9.15)
                label = static_cast<int>(at == 23 ? 23 : at + 1);
                break;
            case 0xBC: // WAITX (23.14.9.10)
                if (csp_x != 0) {
                    wait = CspWait::WaitX;
                    csp_hf = Never;
                    return;
                }
                f1(F1CspWait);
                break;
            case 0xBD: randxy(hf); break;
            case 0xBE: // SETCMP1 (23.14.9.11)
                bus.soc().mac_timer->set_compare1_from_timer(hf);
                break;
            case 0xC0: ++csp_x; break;
            case 0xC1: ++csp_y; break;
            case 0xC2: ++csp_z; break;
            case 0xC3: --csp_x; break;
            case 0xC4: --csp_y; break;
            case 0xC5: --csp_z; break;
            case 0xD2: // SSTOP: stop, IRQ_CSP_STOP, write pointer = 0 (23.14.1)
                wp = 0;
                csp_stop(true);
                return;
            default:
                if (op >= 0xC8 && op <= 0xCF) { // INCMAXY M (23.14.9.7)
                    csp_y = static_cast<std::uint8_t>(std::min<unsigned>(csp_y + 1u, op & 7u));
                } else if (op >= 0xD0 && op <= 0xDF && !csp_is_nop(op)) {
                    strobe(op & 0x0Fu, hf);
                }
                break;
            }
        }
        if (running)
            csp_advance(next, hf + 1);
    }
    void csp_event(std::uint64_t hf) { csp_step(hf); }
    void csp_write(std::uint8_t v, std::uint64_t hf) {
        if (csp_is_immediate(v)) {
            switch (v) {
            case 0xE1: csp_start(hf); break;
            case 0xE2: // ISSTOP (23.14.9.32, 23.14.1)
                wp = 0;
                if (running)
                    csp_stop(true);
                else
                    f1(F1CspStop);
                break;
            case 0xFF: // ISCLEAR (23.14.9.46): no stop interrupt
                csp_stop(false);
                prog.fill(0xD0);
                pc = 0;
                // Model-defined: one ISCLEAR also resets the write pointer
                // (the "issued twice" remark is not modeled).
                wp = 0;
                break;
            default: strobe(v & 0x0Fu, hf); break; // ISxxx
            }
            return;
        }
        // 23.14.1: no instruction writes while a program runs.
        if (running)
            bus.note_unmodeled(sfr_reg(SfrRfst), true);
        if (wp >= prog.size()) {
            // Model-defined: the write pointer stopped at the end of memory;
            // the last location is overwritten.
            bus.note_unmodeled(sfr_reg(SfrRfst), true);
            prog[prog.size() - 1] = v;
            return;
        }
        prog[wp++] = v;
    }
    void timer_overflow(std::uint64_t hf) {
        if (st == StAckCal && fsm_hf == Never && ack_slot_from != Never && hf >= ack_slot_from) {
            ack_slot_from = Never;
            fsm_hf = hf; // slotted ACK starts at this boundary
        }
        if (!running)
            return;
        // 23.14.2: CSPT decremented on each MAC timer overflow; 0xFF holds.
        if (csp_t != 0xFF && csp_t != 0) {
            --csp_t;
            if (csp_t == 0) {
                csp_stop(true);
                return;
            }
        }
        if (wait == CspWait::Overflows) {
            if (--wait_count == 0)
                csp_continue(hf, true);
        } else if (wait == CspWait::WaitX) {
            --csp_x;
            if (csp_x == 0)
                csp_continue(hf, true);
        }
    }
    void timer_event(unsigned which, std::uint64_t hf) {
        if (running && ((which == 1 && wait == CspWait::Event1) ||
                        (which == 2 && wait == CspWait::Event2)))
            csp_continue(hf, false);
    }

    // ==================================================================
    // Event dispatch. Ties: FSM, TX, RX, air poll, CSP.
    // ==================================================================
    std::uint64_t next_event() const {
        return std::min({fsm_hf, tx_event_hf(), rx_event_hf(), poll_hf, csp_next_hf()});
    }
    void process_event() {
        const std::uint64_t f = fsm_hf, t = tx_event_hf(), r = rx_event_hf(), p = poll_hf,
                            c = csp_next_hf();
        const std::uint64_t m = std::min({f, t, r, p, c});
        if (m == Never)
            return;
        if (f == m)
            fsm_event(m);
        else if (t == m)
            tx_event(m);
        else if (r == m)
            rx_event(m);
        else if (p == m)
            poll(m);
        else
            csp_event(m);
        if (c != m || f == m || t == m || r == m || p == m)
            wake_csp(m);
    }
    void fsm_event(std::uint64_t hf) {
        fsm_hf = Never;
        switch (st) {
        case StRxCal: // RX calibration done (Figure 23-20)
        case StTransit:
            enter_sfd_wait(hf, true);
            break;
        case StRxWait: // RX2RX time-out over, receiver stays on
            enter_sfd_wait(hf, false);
            break;
        case StTxCal:
            start_tx(hf);
            break;
        case StAckCal:
            start_ack(hf, std::move(pending_ack));
            pending_ack.clear();
            break;
        case StTxFinal:
        case StTxUnderflow:
            // Figure 23-20: after TX the FSM enables the receiver if
            // RXENABLE != 0 (TX/RX transit, 192 us after the transmission
            // ended, 23.8.2), otherwise it returns to idle.
            sfd = false;
            if (rxenable) {
                st = StTransit;
                fsm_hf = std::max(tx_stop_hf + TurnCycles, hf);
            } else {
                go_idle();
            }
            break;
        default:
            break;
        }
    }
    void flush_dma() {
        if (dma_busy)
            return;
        dma_busy = true;
        while (dma_pending) {
            --dma_pending;
            bus.dma_trigger(trigger::Radio);
        }
        dma_busy = false;
    }

    // ==================================================================
    // Register access (23.15)
    // ==================================================================
    static unsigned flag_index(std::uint8_t sfr) {
        return sfr == SfrRfirqf0 ? 0u : sfr == SfrRfirqf1 ? 1u : 2u;
    }
    bool tx_active_state() const {
        return st == StTxCal || st == StTx || st == StTxFinal || st == StTransit ||
               st == StAckCal || st == StAck || st == StTxUnderflow;
    }
    bool rx_active_state() const {
        return st == StRxCal || st == StSfdWait || st == StRx || st == StRxWait;
    }
    std::uint8_t fsmstat0() const {
        // CAL_RUNNING. Model-defined: high for the whole calibration state.
        const bool cal = st == StRxCal || st == StTxCal || st == StAckCal;
        return static_cast<std::uint8_t>((cal ? 0x40 : 0x00) | (st & 0x3F));
    }
    std::uint8_t fsmstat1(std::uint64_t hf, bool side) {
        std::uint8_t v = 0;
        if (rx_count > 0 && !rx_overflow)
            v |= 0x80; // FIFO
        if (fifop())
            v |= 0x40;
        if (sfd)
            v |= 0x20;
        if (side ? cca(hf, true) : cca_peek(hf))
            v |= 0x10;
        if (sampled_cca)
            v |= 0x08;
        // LOCK_STATUS. Model-defined: the PLL is in lock in every state
        // with the synthesizer on except the calibration states.
        if (st != StIdle && st != StRxCal && st != StTxCal && st != StAckCal)
            v |= 0x04;
        if (tx_active_state())
            v |= 0x02;
        if (rx_active_state())
            v |= 0x01;
        return v;
    }
    std::uint8_t value(Reg r, bool side) {
        const std::uint64_t hf = bus.hf_now();
        if (is_sfr(r)) {
            switch (sfr_of(r)) {
            case SfrRfd:
                return side ? rfd_read() : ram[RxFifoOff + rx_first];
            case SfrRfst: // 23.14.3: instruction being executed, else 0xD0
                return running ? prog[pc_at(hf)] : 0xD0;
            case SfrRfirqf0:
            case SfrRfirqf1:
            case SfrRferrf: {
                const unsigned i = flag_index(sfr_of(r));
                if (side && bus.rmw_access()) {
                    rmw_time[i] = bus.now();
                    rmw_snapshot[i] = flags[i];
                }
                return flags[i];
            }
            default:
                return bus.storage(r);
            }
        }
        if (r >= RamBase && r < RamEnd)
            return ram[r - RamBase];
        if (r >= CSPPROG0 && r < CSPPROG0 + 24)
            return prog[r - CSPPROG0];
        switch (r) {
        case SRCEXTEN0:
        case SRCEXTEN1:
        case SRCEXTEN2: { // EXT_ADDR_EN[2n + 1] reads as EXT_ADDR_EN[2n]
            const std::uint8_t s = reg(r) & 0x55u;
            return static_cast<std::uint8_t>(s | s << 1);
        }
        case RXENABLE: return rxenable;
        case RXMASKSET:
        case RXMASKCLR: return 0; // R0/W
        case FSMSTAT0: return fsmstat0();
        case FSMSTAT1: return fsmstat1(hf, side);
        case RSSI: return rssi_reg(hf);
        case RSSISTAT: return rssi_valid(hf) ? 0x01 : 0x00;
        case RXFIRST: return ram[RxFifoOff + rx_first];
        case RXFIFOCNT: return static_cast<std::uint8_t>(rx_count);
        case TXFIFOCNT: {
            const unsigned first = tx_first_reg(hf);
            return static_cast<std::uint8_t>(tx_last > first ? tx_last - first : 0);
        }
        case RXFIRST_PTR: return static_cast<std::uint8_t>(rx_first);
        case RXLAST_PTR: return static_cast<std::uint8_t>(rx_last());
        case RXP1_PTR: return static_cast<std::uint8_t>(rxp1());
        case TXFIRST_PTR: return static_cast<std::uint8_t>(tx_first_reg(hf));
        case TXLAST_PTR: return static_cast<std::uint8_t>(tx_last);
        case RFRND:
            // 23.12: valid in RX after RSSI_VALID. Model-defined: a seeded
            // deterministic random source; reads outside that condition are
            // counted as unmodeled.
            if (side && !(rx_active_state() && rssi_valid(hf)))
                bus.note_unmodeled(r, false);
            return static_cast<std::uint8_t>(mix64(seed ^ mix64(hf)) & 3u);
        case FREQEST:
            return 0; // Model-defined: transmitters have no frequency offset
        case CSPCTRL: return csp_ctrl;
        case CSPSTAT:
            return static_cast<std::uint8_t>((running ? 0x20 : 0x00) | (pc_at(hf) & 0x1Fu));
        case CSPX: return csp_x;
        case CSPY: return csp_y;
        case CSPZ: return csp_z;
        case CSPT: return csp_t;
        default:
            return reg(r);
        }
    }
    void write_flags(unsigned i, std::uint8_t v) {
        // R/W0 (23.1.2): writing 0 clears; on the write half of a
        // read-modify-write instruction only flags that were 1 at the read
        // can be cleared.
        std::uint8_t clear = static_cast<std::uint8_t>(~v & flag_mask(i));
        if (rmw_time[i] == bus.now())
            clear &= rmw_snapshot[i];
        rmw_time[i] = Never;
        const bool line = rf_line();
        const bool cleared = (flags[i] & clear) != 0;
        flags[i] = static_cast<std::uint8_t>(flags[i] & ~clear);
        bus.storage(flag_reg(i)) = flags[i];
        // 23.1.2: "If a flag is cleared in the RF Core and there are other
        // unmasked flags standing, another interrupt is generated."
        if (cleared && (flags[i] & irq_mask(i)))
            raise_cpu(i);
        if (line && !rf_line())
            rf_capture_edge(false);
    }
    void write_mask(Reg r, unsigned i, std::uint8_t v) {
        const std::uint8_t old = reg(r);
        const bool line = rf_line();
        bus.storage(r) = static_cast<std::uint8_t>(v & flag_mask(i));
        // Model-defined: unmasking a standing flag requests the interrupt.
        if (flags[i] & v & ~old & flag_mask(i))
            raise_cpu(i);
        if (line && !rf_line())
            rf_capture_edge(false);
    }
    void rfd_write(std::uint8_t v) {
        // 23.8.3/23.8.5: TXFIFO write; TX overflow when full. Model-defined:
        // writes are ignored after an overflow until SFLUSHTX.
        if (tx_overflow)
            return;
        if (tx_last >= FifoSize) {
            tx_overflow = true;
            ferr(ErrTxOverf);
            return;
        }
        ram[TxFifoOff + tx_last] = v;
        ++tx_last;
        if (st == StTx && tx.active && !tx.ack)
            try_register();
    }
    void write_reg(Reg r, std::uint8_t v) {
        const std::uint64_t hf = bus.hf_now();
        if (is_sfr(r)) {
            switch (sfr_of(r)) {
            case SfrRfd: rfd_write(v); break;
            case SfrRfst: csp_write(v, hf); break;
            case SfrRfirqf0:
            case SfrRfirqf1:
            case SfrRferrf: write_flags(flag_index(sfr_of(r)), v); break;
            default: break;
            }
            return;
        }
        if (r >= RamBase && r < RamEnd) {
            ram[r - RamBase] = v;
            return;
        }
        if (const PlainReg *p = plain_reg(r)) {
            const std::uint8_t old = reg(r);
            bus.storage(r) = static_cast<std::uint8_t>(v & p->wmask);
            plain_written(r, old, bus.storage(r), hf);
            return;
        }
        switch (r) {
        case SRCEXTEN0:
        case SRCEXTEN1:
        case SRCEXTEN2:
            bus.storage(r) = v & 0x55u; // only EXT_ADDR_EN[2n] is writable
            break;
        case RXMASKSET: set_rxenable(rxenable | v, hf, true); break;
        case RXMASKCLR: set_rxenable(static_cast<std::uint8_t>(rxenable & ~v), hf, true); break;
        case RFIRQM0: write_mask(r, 0, v); break;
        case RFIRQM1: write_mask(r, 1, v); break;
        case RFERRM: write_mask(r, 2, v); break;
        case FSCAL1:
            // Bits 7:2 R/W0 (Table 0-2: only 0 can be written), 1:0 R/W.
            bus.storage(r) = static_cast<std::uint8_t>((reg(r) & v & 0xFC) | (v & 0x03));
            break;
        case CSPCTRL: csp_ctrl = v & 0x01u; break;
        case CSPX:
            csp_x = v;
            // 23.14.9.10: WAITX continues as soon as X = 0.
            if (running && wait == CspWait::WaitX && csp_x == 0)
                csp_continue(hf, true);
            break;
        case CSPY: csp_y = v; break;
        case CSPZ: csp_z = v; break;
        case CSPT:
            csp_t = v;
            // Model-defined: a CPU write of 0 while running stops the program
            // (23.14.3 "the CSPT data register content is zero").
            if (running && csp_t == 0)
                csp_stop(true);
            break;
        default:
            // Read-only registers (FSMSTAT0/1, RSSI, RSSISTAT, RXFIRST,
            // FIFO counters and pointers, RXENABLE, RFRND, FREQEST,
            // CSPSTAT, CSPPROGn): writes are ignored.
            break;
        }
    }
    void plain_written(Reg r, std::uint8_t old, std::uint8_t v, std::uint64_t hf) {
        switch (r) {
        case FRMCTRL0:
            // 23.10.3: ENERGY_SCAN = 1 keeps the peak since it was set.
            if ((v & Ctrl0EnergyScan) && !(old & Ctrl0EnergyScan))
                energy_scan_start = hf;
            // RX_MODE 01 (reserved) / 10 (RXFIFO looping) and TX_MODE != 0
            // (TXFIFO looping, PN9 test data) are test modes: not modeled.
            if (((v >> 2) & 3u) == 1u || ((v >> 2) & 3u) == 2u || (v & 3u) != 0)
                bus.note_unmodeled(r, true);
            break;
        case FRMCTRL1:
            if (st == StTx && tx.active && !tx.ack)
                try_register();
            break;
        case MDMTEST1:
            if (v & MdmTest1ModulationMode) // non-IEEE modulation: off the air
                bus.note_unmodeled(r, true);
            break;
        case FSCAL2:
            if (v & 0x40) // VCO_CAPARR_OE calibration override
                bus.note_unmodeled(r, true);
            break;
        case ATEST:
        case PTEST0:
        case PTEST1:
        case DACTEST0:
        case DACTEST1:
        case DACTEST2:
        case RFC_OBS_CTRL0:
        case RFC_OBS_CTRL1:
        case RFC_OBS_CTRL2:
            // Analog test, power-down override, DAC override and observation
            // outputs: no modeled effect beyond the stored value.
            if (v != 0)
                bus.note_unmodeled(r, true);
            break;
        default:
            break;
        }
    }

    // ==================================================================
    // Reset
    // ==================================================================
    void reset(ResetCause cause) {
        // Host wiring (air, node, ns_offset), the noise seed and the DMA
        // trigger statistics are not chip state and are kept.
        // 23.4.3: the RAM values are "unknown after reset". Model-defined:
        // a power-on reset clears the radio RAM to 0; other resets keep it.
        if (cause == ResetCause::PowerOn)
            ram.fill(0);
        flags = {};
        rmw_time = {Never, Never, Never};
        rmw_snapshot = {};
        rxenable = 0;
        st = StIdle;
        fsm_hf = Never;
        sfd = sampled_cca = cca_memory = false;
        rx_on = Never;
        search_start = 0;
        poll_hf = lock_hf = Never;
        lock_id = 0;
        energy_scan_start = 0;
        tx_first = tx_last = 0;
        tx_overflow = tx_underflow = false;
        tx = Tx{};
        rx_first = rx_count = 0;
        rx_overflow = false;
        records.clear();
        fifop_end = fifop_level = false;
        rx = Rx{};
        handled.clear();
        pending_ack.clear();
        ack_slot_from = Never;
        prog.fill(0xD0); // 23.14.1 / CSPPROGn reset 0xD0
        wp = pc = 0;
        running = false;
        label = -1;
        csp_x = csp_y = csp_z = 0;
        csp_t = 0xFF;
        csp_ctrl = 0;
        csp_hf = Never;
        wait = CspWait::None;
        wait_count = 0;
        parked = false;
        park_anchor = 0;
        park_period = 1;
        park_wake = Never;
        park_from = 0;
        tx_stop_hf = 0;
        rand_hf = Never;
        rand_value = 0;
        dma_pending = 0;
        for (const PlainReg &p : PlainRegs)
            bus.storage(p.addr) = p.reset;
        bus.storage(FSCAL1) = 0x2B;
        bus.storage(sfr_reg(SfrRfst)) = 0xD0;
    }
    bool active() const { return st != StIdle || running; }
};

Radio::Radio(Bus &bus) : Peripheral(bus), state_(std::make_unique<State>(*this, bus)) {
    bus_.claim_sfr(SfrRfd, this, true);
    bus_.claim_sfr(SfrRfst, this, true);
    bus_.claim_sfr(SfrRfirqf0, this, true); // RMW snapshot (R/W0 rule)
    bus_.claim_sfr(SfrRfirqf1, this, true);
    bus_.claim_sfr(SfrRferrf, this, true);
    // Table 23-5 / 23.15: RAM 0x6000-0x617F and the documented registers.
    // 0x61A0, 0x61A6 (MONMUX), 0x61AD and 0x61D8-0x61DF, 0x61E6-0x61EA,
    // 0x61EE-0x61F9, 0x61FB-0x61FF are not radio registers of this model.
    auto claim = [&](std::uint16_t from, std::uint16_t to) {
        for (std::uint32_t a = from; a <= to; ++a)
            bus_.claim_xreg(static_cast<std::uint16_t>(a), this);
    };
    claim(RamBase, RXP1_PTR);
    claim(TXFIRST_PTR, RFERRM);
    claim(RFRND, FSCTRL);
    claim(FSCAL1, PTEST1);
    claim(CSPPROG0, CSPPROG0 + 23);
    claim(CSPCTRL, CSPT);
    claim(RFC_OBS_CTRL0, RFC_OBS_CTRL2);
    claim(TXFILTCFG, TXFILTCFG);
}

Radio::~Radio() = default;

void Radio::reset(ResetCause cause) { state_->reset(cause); }

std::uint8_t Radio::read(Reg reg) {
    advance_rf_domain(bus_.soc(), bus_.hf_now());
    const std::uint8_t v = state_->value(reg, true);
    state_->flush_dma();
    return v;
}

std::uint8_t Radio::peek(Reg reg) const { return state_->value(reg, false); }

void Radio::write(Reg reg, std::uint8_t value) {
    advance_rf_domain(bus_.soc(), bus_.hf_now());
    state_->write_reg(reg, value);
    // A CPU write can change any CSP loop condition (CSPCTRL, X/Y/Z, FSM).
    state_->wake_csp(bus_.hf_now());
    state_->flush_dma();
}

std::uint64_t Radio::deadline() const { return bus_.hf_deadline(next_event_hf()); }

void Radio::service(std::uint64_t) {
    advance_rf_domain(bus_.soc(), bus_.hf_now());
    state_->flush_dma();
}

void Radio::power_mode_changed(PowerMode, PowerMode to) {
    advance_rf_domain(bus_.soc(), bus_.hf_now());
    state_->flush_dma();
    // 23.4: configuration and FIFOs are preserved in sleep modes. The HF
    // clock stops in PM1-PM3, so an active radio or CSP is frozen; that
    // situation is not modeled further and is counted.
    if (in_sleep(to) && state_->active())
        bus_.note_unmodeled(FSMSTAT0, false);
}

void Radio::clock_changed() {
    // 23.14.3: the system clock must be 32 MHz (XOSC) for radio operation.
    // Model-defined: radio timing always uses the HF time base; running the
    // radio or the CSP on the 16 MHz RCOSC is counted as unmodeled.
    if ((bus_.clkconsta() & 0x40) && state_->active())
        bus_.note_unmodeled(FSMSTAT0, false);
}

bool Radio::may_interrupt() const { return next_event_hf() != Never || state_->running; }

void Radio::attach_air(mcuemu::VirtualAir &air, std::string node, std::uint64_t ns_offset) {
    state_->air = &air;
    state_->node = std::move(node);
    state_->ns_offset = ns_offset;
    bus_.reschedule();
}

void Radio::set_noise_seed(std::uint64_t seed) { state_->seed = seed; }

std::uint64_t Radio::next_event_hf() const { return state_->next_event(); }

void Radio::process_next_event() { state_->process_event(); }

void Radio::mac_timer_overflow(std::uint64_t hf) { state_->timer_overflow(hf); }

void Radio::mac_timer_event(unsigned which, std::uint64_t hf) { state_->timer_event(which, hf); }

std::uint64_t Radio::dma_trigger_count() const { return state_->dma_triggers; }

unsigned Radio::fsm_state() const { return state_->st; }

} // namespace mcs51emu::cc2530
