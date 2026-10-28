// CC2530 IEEE 802.15.4 radio tests (SWRU191F Chapter 23). Programs are
// hand-assembled MCS-51 machine code, or host strobes and FIFO writes
// through the documented SFR side effects; frames travel through the shared
// mcuemu::VirtualAir. Expected values come from SWRU191F and IEEE 802.15.4
// (2.4 GHz O-QPSK, 250 kbit/s: 32 us = 1024 cycles per octet, 16 us = 512
// cycles per symbol at 32 MHz). Checks that depend on a model-defined rule
// (documented with "Model-defined:" in cc2530_radio.cpp) say so.
#include "cc2530_periph_support.hpp"

#include "cc2530_mactimer.hpp"
#include "cc2530_radio.hpp"
#include "mcuemu/radio.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

using namespace cc2530test;
using mcuemu::RadioFrame;
using mcuemu::RadioProtocol;
using mcuemu::VirtualAir;
using mcuemu::VirtualRadioNode;

namespace {

// ---- SFRs ----
constexpr std::uint8_t RFIRQF1 = 0x91, RFERRF = 0xBF, RFD = 0xD9, RFST = 0xE1, RFIRQF0 = 0xE9,
                       S1CON = 0x9B, IEN2 = 0x9A, TCON = 0x88, T2CTRL = 0x94, T2EVTCFG = 0x9C,
                       T2M0 = 0xA2, T2M1 = 0xA3, T2MSEL = 0xC3;
constexpr std::uint8_t BitEa = 0xAF, BitRferrie = 0xA8;

// ---- XREGs (Table 23-5) ----
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
                        RFIRQM0 = 0x61A3, RFIRQM1 = 0x61A4, RFERRM = 0x61A5,
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
// ---- radio RAM (23.4, Table 23-1) ----
constexpr std::uint16_t RxFifo = 0x6000, TxFifo = 0x6080, SrcTable = 0x6100,
                        SRCRESMASK0 = 0x6160, SRCRESINDEX = 0x6163, SRCSHORTPENDEN0 = 0x6167,
                        PAN_ID0 = 0x6172, SHORT_ADDR0 = 0x6174;

// ---- command strobes (23.14.9) ----
constexpr std::uint8_t IsStart = 0xE1, IsStop = 0xE2, IsRxOn = 0xE3, IsTxOn = 0xE9,
                       IsTxOnCca = 0xEA, IsSampleCca = 0xEB, IsFlushRx = 0xED,
                       IsFlushTx = 0xEE, IsRfOff = 0xEF;

// ---- flags (23.1.2) ----
constexpr std::uint8_t F0RxMaskZero = 0x80, F0RxPktDone = 0x40, F0FrameAccepted = 0x20,
                       F0SrcMatchFound = 0x10, F0SrcMatchDone = 0x08, F0Fifop = 0x04,
                       F0Sfd = 0x02;
constexpr std::uint8_t F1CspWait = 0x20, F1CspStop = 0x10, F1CspManInt = 0x08, F1RfIdle = 0x04,
                       F1TxDone = 0x02, F1TxAckDone = 0x01;
constexpr std::uint8_t ErrStrobe = 0x40, ErrTxUnderf = 0x20, ErrTxOverf = 0x10,
                       ErrRxUnderf = 0x08;
// FSMSTAT1 bits
constexpr std::uint8_t S1Fifo = 0x80, S1Fifop = 0x40, S1Sfd = 0x20, S1Cca = 0x10,
                       S1SampledCca = 0x08, S1TxActive = 0x02, S1RxActive = 0x01;

// ---- timing (cycles at 32 MHz) ----
constexpr std::uint64_t Octet = 1024;   // 32 us per octet at 250 kbit/s
constexpr std::uint64_t Turn = 6144;    // 12 symbols = 192 us (23.8.2, 23.9.2)
constexpr std::uint64_t Ramp = 64;      // 2 us modulator ramp-down (23.8.2)
constexpr std::uint64_t Shr = 5 * Octet; // 4-octet preamble + SFD (IEEE 802.15.4 PPDU)
constexpr std::uint64_t RssiValid = 4096; // 8 symbol periods (RSSISTAT, 23.10.3)

constexpr std::uint16_t Main = 0x0100, RfIsr = 0x0090, ErrIsr = 0x00C0;

std::vector<std::uint8_t> mov(std::uint8_t dir, std::uint8_t imm) { return mov_dir_imm(dir, imm); }
// MOV dst,src
std::vector<std::uint8_t> mov_dd(std::uint8_t dst, std::uint8_t src) { return {0x85, src, dst}; }

struct XWrite {
    std::uint16_t addr;
    std::uint8_t value;
};

// Vectors: reset -> Main, RFERR (0x03) -> ErrIsr, RF (0x83) -> RfIsr.
// RF ISR: counts entries in 0x30, ORs RFIRQF0/RFIRQF1 into 0x31/0x32, clears
// the RF core flags and then S1CON (23.1.2). RFERR ISR: counts in 0x33, ORs
// RFERRF into 0x34 and clears it (TCON.RFERRIF is cleared on vectoring).
// Main executes the XREG writes in `config`, then enables the requested
// interrupts. The caller appends code and the sled.
Flash program(std::initializer_list<XWrite> config, bool rf_irq = false, bool err_irq = false) {
    Flash flash;
    flash.at(0x0000).op({0x02, hi(Main), lo(Main)});
    flash.at(0x0003).op({0x02, hi(ErrIsr), lo(ErrIsr)});
    flash.at(0x0083).op({0x02, hi(RfIsr), lo(RfIsr)});
    flash.at(RfIsr)
        .op({0xC0, 0xE0, 0x05, 0x30, 0xE5, RFIRQF0, 0x42, 0x31, 0xE5, RFIRQF1, 0x42, 0x32})
        .op(mov(RFIRQF0, 0x00))
        .op(mov(RFIRQF1, 0x00))
        .op(mov(S1CON, 0x00))
        .op({0xD0, 0xE0, 0x32});
    flash.at(ErrIsr)
        .op({0xC0, 0xE0, 0x05, 0x33, 0xE5, RFERRF, 0x42, 0x34})
        .op(mov(RFERRF, 0x00))
        .op({0xD0, 0xE0, 0x32});
    flash.at(Main);
    for (const XWrite &w : config)
        flash.op(movx_write(w.addr, w.value));
    if (rf_irq)
        flash.op(mov(IEN2, 0x01)); // RFIE
    if (err_irq)
        flash.op({0xD2, BitRferrie});
    if (rf_irq || err_irq)
        flash.op({0xD2, BitEa});
    return flash;
}
// One-cycle NOPs up to 0x7FF0, then LJMP back: every cycle is an
// instruction boundary, so events are serviced at their exact time.
void sled(Flash &flash) {
    const std::uint16_t start = flash.here();
    while (flash.here() < 0x7FF0)
        flash.op({0x00});
    flash.op({0x02, hi(start), lo(start)});
}

std::uint64_t run_to_pc(Cc2530F256 &device, std::uint16_t pc) {
    device.set_breakpoint(pc);
    const Cc2530RunResult r = device.run(10'000'000);
    device.set_breakpoint(std::nullopt);
    CHECK(r.error.empty());
    CHECK(r.break_hit);
    return device.registers().cycles;
}
// Exact on the sled while no ISR runs.
void run_to(Cc2530F256 &device, std::uint64_t cycle) {
    const Cc2530RunResult r = device.run(10'000'000, cycle);
    CHECK(r.error.empty());
    CHECK_EQ(device.registers().cycles, cycle);
}
// Lenient variant for runs with ISRs (multi-cycle instructions).
void run_until(Cc2530F256 &device, std::uint64_t cycle) {
    const Cc2530RunResult r = device.run(10'000'000, cycle);
    CHECK(r.error.empty());
    CHECK(device.registers().cycles >= cycle);
}

std::uint8_t x(const Cc2530F256 &device, std::uint16_t addr) { return device.peek_xdata(addr); }
unsigned state(Cc2530F256 &device) { return device.peripherals().radio->fsm_state(); }
// Air time of a cycle count (attach_air: ns = ns_offset + cycles * 1000 / 32).
std::uint64_t ns(std::uint64_t cycle) { return cycle * 1000 / 32; }

// IEEE 802.15.4 MPDU without the FCS: FCF, sequence number, destination PAN,
// destination and source short addresses (PAN ID compression), then `tail`.
std::vector<std::uint8_t> mpdu(std::uint16_t fcf, std::uint8_t seq, std::uint16_t pan,
                               std::uint16_t dst, std::uint16_t src,
                               std::vector<std::uint8_t> tail) {
    std::vector<std::uint8_t> m{lo(fcf), hi(fcf), seq, lo(pan), hi(pan),
                                lo(dst), hi(dst), lo(src), hi(src)};
    m.insert(m.end(), tail.begin(), tail.end());
    return m;
}
// FCF: data frame, ACK request, PAN ID compression, short/short addressing.
constexpr std::uint16_t FcfDataAck = 0x8861;
// Same without ACK request.
constexpr std::uint16_t FcfData = 0x8841;
// MAC command frame, ACK request, PAN ID compression, short/short.
constexpr std::uint16_t FcfCmdAck = 0x8863;

// Writes a frame for AUTOCRC = 1 through RFD: LEN (MPDU + 2-octet FCS) and
// LEN - 2 octets (Figure 23-5).
void fill_txfifo(Cc2530F256 &device, const std::vector<std::uint8_t> &m) {
    device.write_sfr(RFD, static_cast<std::uint8_t>(m.size() + 2));
    for (const std::uint8_t b : m)
        device.write_sfr(RFD, b);
}
std::vector<RadioFrame> frames_from(const VirtualAir &air, const std::string &source) {
    std::vector<RadioFrame> out;
    for (const RadioFrame &f : air.frames())
        if (f.source == source)
            out.push_back(f);
    return out;
}

// Local address configuration in radio RAM (Table 23-1): PAN 0x1234,
// short address `self`; source table entry 3 = PAN 0x1234 / short 0x0002,
// enabled and with the pending bit enabled.
std::vector<XWrite> address_config(std::uint16_t self) {
    return {{PAN_ID0, 0x34},
            {PAN_ID0 + 1, 0x12},
            {SHORT_ADDR0, lo(self)},
            {SHORT_ADDR0 + 1, hi(self)},
            {SrcTable + 12, 0x34},
            {SrcTable + 13, 0x12},
            {SrcTable + 14, 0x02},
            {SrcTable + 15, 0x00},
            {SRCSHORTEN0, 0x08},
            {SRCSHORTPENDEN0, 0x08}};
}
Flash program_with(const std::vector<XWrite> &config, bool rf_irq = false, bool err_irq = false) {
    Flash flash = program({}, false, false);
    for (const XWrite &w : config)
        flash.op(movx_write(w.addr, w.value));
    if (rf_irq)
        flash.op(mov(IEN2, 0x01));
    if (err_irq)
        flash.op({0xD2, BitRferrie});
    if (rf_irq || err_irq)
        flash.op({0xD2, BitEa});
    return flash;
}

// ======================================================================

void test_reset_values_and_write_masks() {
    struct Reg {
        std::uint16_t addr;
        std::uint8_t reset;    // register description reset value (23.15)
        std::uint8_t after_ff; // value after writing 0xFF (R0 bits, read-only)
    };
    // Reset values and R/W/R0 columns of the register descriptions in
    // 23.15 (and 23.14.7 for the CSP registers).
    const Reg regs[] = {
        {FRMFILT0, 0x0D, 0xFF},  {FRMFILT1, 0x78, 0xFF},     {SRCMATCH, 0x07, 0xFF},
        {SRCSHORTEN0, 0x00, 0xFF}, {SRCSHORTEN1, 0x00, 0xFF}, {SRCSHORTEN2, 0x00, 0xFF},
        {SRCEXTEN0, 0x00, 0xFF}, {SRCEXTEN1, 0x00, 0xFF},    {SRCEXTEN2, 0x00, 0xFF},
        {FRMCTRL0, 0x40, 0xFF},  {FRMCTRL1, 0x01, 0x07},     {RXENABLE, 0x00, 0x00},
        {RXMASKSET, 0x00, 0x00}, {RXMASKCLR, 0x00, 0x00},    {FREQTUNE, 0x0F, 0x0F},
        {FREQCTRL, 0x0B, 0x7F},  {TXPOWER, 0xF5, 0xFF},      {TXCTRL, 0x69, 0x7F},
        {FSMSTAT0, 0x00, 0x00},  {FSMSTAT1, 0x00, 0x00},     {FIFOPCTRL, 0x40, 0x7F},
        {FSMCTRL, 0x01, 0x03},   {CCACTRL0, 0xE0, 0xFF},     {CCACTRL1, 0x1A, 0x1F},
        {RSSI, 0x80, 0x80},      {RSSISTAT, 0x00, 0x00},     {RXFIRST, 0x00, 0x00},
        {RXFIFOCNT, 0x00, 0x00}, {TXFIFOCNT, 0x00, 0x00},    {RXFIRST_PTR, 0x00, 0x00},
        {RXLAST_PTR, 0x00, 0x00}, {RXP1_PTR, 0x00, 0x00},    {TXFIRST_PTR, 0x00, 0x00},
        {TXLAST_PTR, 0x00, 0x00}, {RFIRQM0, 0x00, 0xFF},     {RFIRQM1, 0x00, 0x3F},
        {RFERRM, 0x00, 0x7F},    {MDMCTRL0, 0x85, 0xFF},     {MDMCTRL1, 0x14, 0x3F},
        {FREQEST, 0x00, 0x00},   {RXCTRL, 0x3F, 0x3F},       {FSCTRL, 0x5A, 0xFF},
        {FSCAL2, 0x20, 0x7F},    {FSCAL3, 0x2A, 0x7F},       {AGCCTRL0, 0x5F, 0x7F},
        {AGCCTRL1, 0x11, 0x3F},  {AGCCTRL2, 0x00, 0xFF},     {AGCCTRL3, 0x2E, 0x7F},
        {ADCTEST0, 0x10, 0xFF},  {ADCTEST1, 0x0E, 0xFF},     {ADCTEST2, 0x03, 0x7F},
        {MDMTEST0, 0x75, 0xFF},  {MDMTEST1, 0x08, 0x1F},     {DACTEST0, 0x00, 0x7F},
        {DACTEST1, 0x00, 0x7F},  {ATEST, 0x00, 0x3F},        {PTEST0, 0x00, 0xFF},
        {PTEST1, 0x00, 0x0F},    {CSPPROG0, 0xD0, 0xD0},     {CSPPROG0 + 23, 0xD0, 0xD0},
        {CSPCTRL, 0x00, 0x01},   {CSPSTAT, 0x00, 0x00},      {CSPX, 0x00, 0xFF},
        {CSPY, 0x00, 0xFF},      {CSPZ, 0x00, 0xFF},         {CSPT, 0xFF, 0xFF},
        {RFC_OBS_CTRL0, 0x00, 0x7F}, {RFC_OBS_CTRL1, 0x00, 0x7F},
        {RFC_OBS_CTRL2, 0x00, 0x7F}, {TXFILTCFG, 0x0F, 0x0F},
    };
    Flash flash = program({});
    // FSCAL1: bits 7:2 R/W0 (only 0 can be written), 1:0 R/W.
    flash.op(movx_write(FSCAL1, 0xFF)).op(movx_read_to(FSCAL1, 0x40));
    flash.op(movx_write(FSCAL1, 0x00)).op(movx_read_to(FSCAL1, 0x41));
    flash.op(movx_write(FSCAL1, 0xFF)).op(movx_read_to(FSCAL1, 0x42));
    // SRCEXTEN0: EXT_ADDR_EN[2n + 1] are read-only and read as EXT_ADDR_EN[2n].
    flash.op(movx_write(SRCEXTEN0, 0xAA)).op(movx_read_to(SRCEXTEN0, 0x43));
    flash.op(movx_write(SRCEXTEN0, 0x01)).op(movx_read_to(SRCEXTEN0, 0x44));
    flash.op(movx_read_to(RXMASKSET, 0x45));
    for (const Reg &r : regs)
        if (r.addr != RXMASKSET) // a nonzero RXENMASK would start the receiver
            flash.op(movx_write(r.addr, 0xFF));
    flash.op(movx_write(DACTEST2, 0xFF));
    flash.op({0x80, 0xFE});
    Cc2530F256 device;
    load(device, flash);
    // Power-on reset values.
    for (const Reg &r : regs)
        if (x(device, r.addr) != r.reset) {
            std::cerr << "reset value of 0x" << std::hex << r.addr << ": 0x" << +x(device, r.addr)
                      << ", want 0x" << +r.reset << std::dec << '\n';
            ++failures;
        }
    CHECK_EQ(x(device, FSCAL1), 0x2B);
    // DACTEST2: bits 2:0 reset 000 (bits 7:3 are documented both as reset
    // 00101 and as R0; see the report).
    CHECK_EQ(x(device, DACTEST2) & 0x07, 0x00);
    CHECK_EQ(device.read_sfr(RFST), 0xD0); // 23.14 RFST reset 0xD0
    CHECK_EQ(device.read_sfr(RFIRQF0), 0x00);
    CHECK_EQ(device.read_sfr(RFIRQF1), 0x00);
    CHECK_EQ(device.read_sfr(RFERRF), 0x00);
    CHECK_EQ(state(device), 0u);
    CHECK(device.run(10'000).error.empty());
    for (const Reg &r : regs)
        if (x(device, r.addr) != r.after_ff) {
            std::cerr << "0xFF write to 0x" << std::hex << r.addr << ": 0x" << +x(device, r.addr)
                      << ", want 0x" << +r.after_ff << std::dec << '\n';
            ++failures;
        }
    CHECK_EQ(x(device, DACTEST2) & 0x07, 0x07);
    CHECK_EQ(device.read_idata(0x40), 0x2B);
    CHECK_EQ(device.read_idata(0x41), 0x00);
    CHECK_EQ(device.read_idata(0x42), 0x03);
    CHECK_EQ(device.read_idata(0x43), 0x00);
    CHECK_EQ(device.read_idata(0x44), 0x03);
    CHECK_EQ(device.read_idata(0x45), 0x00);
    CHECK_EQ(state(device), 0u);
}

void test_fifo_and_rxenable_access() {
    Flash flash = program({});
    // 23.4.1/23.4.2: the FIFO RAM areas are accessible as XREG memory;
    // RAM writes are not FIFO operations.
    flash.op(movx_write(RxFifo, 0x5A)).op(movx_write(RxFifo + 0x7F, 0xC3));
    flash.op(movx_read_to(RXFIFOCNT, 0x40)).op(movx_read_to(RxFifo + 0x7F, 0x41));
    // RXMASKSET/RXMASKCLR (23.15): OR / AND-NOT into RXENMASK; a nonzero
    // RXENABLE enables the receiver when idle (RX calibration, Table 23-3
    // state 2); returning to 0 sets RXMASKZERO (23.1.2) and the receiver
    // returns to idle (Figure 23-20).
    flash.op(movx_write(RXMASKSET, 0x21)).op(movx_read_to(RXENABLE, 0x42));
    flash.op(movx_read_to(FSMSTAT0, 0x43)).op(movx_read_to(FSMSTAT1, 0x44));
    flash.op(movx_write(RXMASKCLR, 0x01)).op(movx_read_to(RXENABLE, 0x45));
    flash.op(movx_read_to(FSMSTAT0, 0x46)).op(mov_dd(0x47, RFIRQF0));
    flash.op(movx_write(RXMASKCLR, 0xFF)).op(movx_read_to(RXENABLE, 0x48));
    flash.op(movx_read_to(FSMSTAT0, 0x49)).op(mov_dd(0x4A, RFIRQF0)).op(mov_dd(0x4B, RFIRQF1));
    const std::uint16_t ready = flash.here();
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    run_to_pc(device, ready);
    CHECK_EQ(device.read_idata(0x40), 0x00);
    CHECK_EQ(device.read_idata(0x41), 0xC3);
    CHECK_EQ(x(device, RxFifo), 0x5A);
    CHECK_EQ(device.read_idata(0x42), 0x21);
    CHECK_EQ(device.read_idata(0x43) & 0x3F, 2);
    CHECK_EQ(device.read_idata(0x44) & (S1RxActive | S1TxActive), S1RxActive);
    CHECK_EQ(device.read_idata(0x45), 0x20);
    CHECK_EQ(device.read_idata(0x46) & 0x3F, 2);
    CHECK_EQ(device.read_idata(0x47) & F0RxMaskZero, 0);
    CHECK_EQ(device.read_idata(0x48), 0x00);
    CHECK_EQ(device.read_idata(0x49), 0x00);
    CHECK_EQ(device.read_idata(0x4A), F0RxMaskZero);
    CHECK_EQ(device.read_idata(0x4B), F1RfIdle); // 23.1.2 RFIDLE

    // TXFIFO through RFD (23.8.3): TXFIFOCNT, TXLAST_PTR, RAM at 0x6080.
    for (unsigned i = 0; i < 10; ++i)
        device.write_sfr(RFD, static_cast<std::uint8_t>(0xB0 + i));
    CHECK_EQ(x(device, TXFIFOCNT), 10);
    CHECK_EQ(x(device, TXLAST_PTR), 10);
    CHECK_EQ(x(device, TXFIRST_PTR), 0);
    for (unsigned i = 0; i < 10; ++i)
        CHECK_EQ(x(device, static_cast<std::uint16_t>(TxFifo + i)), 0xB0 + i);
    // SFLUSHTX empties the TXFIFO (23.8.3).
    device.write_sfr(RFST, IsFlushTx);
    CHECK_EQ(x(device, TXFIFOCNT), 0);
    CHECK_EQ(x(device, TXLAST_PTR), 0);
    // 128 bytes fit; the 129th write overflows: TX_OVERFLOW, byte lost
    // (23.8.5).
    for (unsigned i = 0; i < 128; ++i)
        device.write_sfr(RFD, static_cast<std::uint8_t>(i));
    CHECK_EQ(x(device, TXFIFOCNT), 128);
    CHECK_EQ(device.read_sfr(RFERRF), 0x00);
    device.write_sfr(RFD, 0xEE);
    CHECK_EQ(device.read_sfr(RFERRF), ErrTxOverf);
    CHECK_EQ(x(device, TXFIFOCNT), 128);
    CHECK_EQ(x(device, TxFifo), 0x00);
    // The condition is cleared with SFLUSHTX; the flag stays until cleared.
    device.write_sfr(RFST, IsFlushTx);
    CHECK_EQ(x(device, TXFIFOCNT), 0);
    device.write_sfr(RFD, 0x77);
    CHECK_EQ(x(device, TXFIFOCNT), 1);
    CHECK_EQ(x(device, TxFifo), 0x77);
    CHECK_EQ(device.read_sfr(RFERRF), ErrTxOverf);
    device.write_sfr(RFERRF, 0x00);
    CHECK_EQ(device.read_sfr(RFERRF), 0x00);
}

void test_transmit() {
    VirtualAir air;
    Flash flash = program({});
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    device.attach_air(air, "dut");
    const std::uint64_t ts = 4096, t = 8192;
    run_to(device, ts);
    device.write_sfr(T2CTRL, 0x01); // MAC timer runs (period 0: 16-bit roll-over)
    const auto m = mpdu(FcfDataAck, 0x07, 0x1234, 0x0002, 0x0001, {0xAA, 0xBB});
    const unsigned len = static_cast<unsigned>(m.size() + 2); // 13
    fill_txfifo(device, m);
    CHECK_EQ(x(device, TXFIFOCNT), len - 1);
    run_to(device, t);
    device.write_sfr(RFST, IsTxOn);
    // 23.8.1 / Table 23-3: TX calibration; FRMCTRL1.SET_RXENMASK_ON_TX = 1
    // (reset): STXON sets RXENABLE bit 6.
    CHECK_EQ(state(device), 32u);
    CHECK_EQ(x(device, FSMSTAT1) & (S1TxActive | S1RxActive), S1TxActive);
    CHECK_EQ(x(device, RXENABLE), 0x40);
    // 23.8.2: the preamble starts 192 us after STXON.
    run_to(device, t + Turn - 1);
    CHECK_EQ(state(device), 32u);
    CHECK(air.frames().empty());
    run_to(device, t + Turn);
    CHECK_EQ(state(device), 34u);
    CHECK_EQ(air.frames().size(), 1u);
    if (air.frames().size() == 1) {
        const RadioFrame &f = air.frames()[0];
        CHECK(f.source == "dut");
        CHECK(f.protocol == RadioProtocol::Zigbee802154);
        CHECK_EQ(f.channel, 11); // FREQCTRL 0x0B: 2405 MHz = channel 11
        CHECK(f.payload == m);
        CHECK_EQ(f.start_ns, ns(t + Turn));
        // PPDU: preamble 4 + SFD 1 + PHR 1 + MPDU (with the 2-octet FCS).
        CHECK_EQ(f.end_ns - f.start_ns, (m.size() + 8) * 32'000ull);
    }
    // SFD sent after the 4-octet preamble and the SFD octet: RFIRQF0.SFD,
    // FSMSTAT1.SFD and the MAC timer capture (22.1.10).
    const std::uint64_t sfd = t + Turn + Shr;
    run_to(device, sfd - 1);
    CHECK_EQ(device.read_sfr(RFIRQF0), 0x00);
    CHECK_EQ(x(device, FSMSTAT1) & S1Sfd, 0);
    run_to(device, sfd);
    CHECK_EQ(device.read_sfr(RFIRQF0), F0Sfd);
    CHECK_EQ(x(device, FSMSTAT1) & S1Sfd, S1Sfd);
    device.write_sfr(T2MSEL, 0x01);
    CHECK_EQ(device.read_sfr(T2M0), lo(static_cast<std::uint16_t>(sfd - ts)));
    CHECK_EQ(device.read_sfr(T2M1), hi(static_cast<std::uint16_t>(sfd - ts)));
    // End of the PHR + MPDU (LEN + 1 octets after the SFD): TXDONE; the SFD
    // signal is stretched and the FSM leaves TX 2 us later (23.8.2).
    const std::uint64_t end = sfd + (1 + len) * Octet;
    run_to(device, end - 1);
    CHECK_EQ(device.read_sfr(RFIRQF1), 0x00);
    CHECK_EQ(state(device), 34u);
    run_to(device, end);
    CHECK_EQ(device.read_sfr(RFIRQF1), F1TxDone);
    CHECK_EQ(state(device), 39u);
    run_to(device, end + Ramp - 1);
    CHECK_EQ(x(device, FSMSTAT1) & S1Sfd, S1Sfd);
    CHECK_EQ(state(device), 39u);
    run_to(device, end + Ramp);
    CHECK_EQ(x(device, FSMSTAT1) & S1Sfd, 0);
    // RXENABLE != 0: TX -> RX transit (state 40), 192 us to SFD wait
    // (23.8.2 "equal delay when returning to receive mode"; Model-defined:
    // measured from the end of the frame).
    CHECK_EQ(state(device), 40u);
    run_to(device, end + Turn - 1);
    CHECK_EQ(state(device), 40u);
    run_to(device, end + Turn);
    CHECK_EQ(state(device), 3u);
    CHECK_EQ(x(device, FSMSTAT1) & (S1TxActive | S1RxActive), S1RxActive);
    // 23.8.4: the TXFIFO content is left unchanged; STXON retransmits.
    CHECK_EQ(x(device, TxFifo), len);
    for (unsigned i = 0; i < m.size(); ++i)
        CHECK_EQ(x(device, static_cast<std::uint16_t>(TxFifo + 1 + i)), m[i]);
    const std::uint64_t t2 = end + Turn + 1024;
    run_to(device, t2);
    device.write_sfr(RFST, IsTxOn);
    run_to(device, t2 + Turn);
    CHECK_EQ(air.frames().size(), 2u);
    if (air.frames().size() == 2) {
        CHECK(air.frames()[1].payload == m);
        CHECK_EQ(air.frames()[1].start_ns, ns(t2 + Turn));
    }
    const std::uint64_t end2 = t2 + Turn + Shr + (1 + len) * Octet;
    run_to(device, end2 + Turn);
    CHECK_EQ(state(device), 3u);
    // SRFOFF: RXENABLE cleared (RXMASKZERO), the FSM goes idle (RFIDLE).
    device.write_sfr(RFIRQF0, 0x00);
    device.write_sfr(RFIRQF1, 0x00);
    device.write_sfr(RFST, IsRfOff);
    CHECK_EQ(state(device), 0u);
    CHECK_EQ(x(device, RXENABLE), 0x00);
    CHECK_EQ(device.read_sfr(RFIRQF0), F0RxMaskZero);
    CHECK_EQ(device.read_sfr(RFIRQF1), F1RfIdle);
    CHECK_EQ(device.read_sfr(RFERRF), 0x00);
    // SRFOFF while idle: RFERRF.STROBEERR (command strobe in a state where
    // it cannot be used).
    device.write_sfr(RFST, IsRfOff);
    CHECK_EQ(device.read_sfr(RFERRF), ErrStrobe);
}

void test_transmit_to_idle_and_underflow() {
    // FRMCTRL1.SET_RXENMASK_ON_TX = 0, no air attached: after TX the FSM
    // returns to idle 2 us after the frame (23.8.2), raising RFIDLE.
    Flash flash = program({{FRMCTRL1, 0x00}});
    const std::uint16_t ready = flash.here();
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    run_to_pc(device, ready);
    const auto m = mpdu(FcfData, 0x01, 0xFFFF, 0xFFFF, 0x0001, {0x10, 0x20, 0x30});
    const unsigned len = static_cast<unsigned>(m.size() + 2);
    fill_txfifo(device, m);
    const std::uint64_t t = 4096;
    run_to(device, t);
    device.write_sfr(RFST, IsTxOn);
    CHECK_EQ(x(device, RXENABLE), 0x00);
    const std::uint64_t end = t + Turn + Shr + (1 + len) * Octet;
    run_to(device, end - 1);
    CHECK_EQ(device.read_sfr(RFIRQF1), 0x00);
    run_to(device, end);
    CHECK_EQ(device.read_sfr(RFIRQF1), F1TxDone);
    CHECK_EQ(state(device), 39u);
    run_to(device, end + Ramp - 1);
    CHECK_EQ(state(device), 39u);
    run_to(device, end + Ramp);
    CHECK_EQ(state(device), 0u);
    CHECK_EQ(device.read_sfr(RFIRQF1), F1TxDone | F1RfIdle);

    // TX underflow (23.8.5): STXON with an empty TXFIFO. Model-defined: the
    // length octet is fetched when its transmission starts, right after the
    // SFD, so the underflow is flagged at the SFD time. The aborted frame is
    // not placed on the air (Model-defined).
    VirtualAir air;
    Cc2530F256 second;
    load(second, flash);
    second.attach_air(air, "dut");
    run_to_pc(second, ready);
    run_to(second, t);
    second.write_sfr(RFST, IsTxOn);
    const std::uint64_t sfd = t + Turn + Shr;
    run_to(second, sfd - 1);
    CHECK_EQ(second.read_sfr(RFERRF), 0x00);
    run_to(second, sfd);
    CHECK_EQ(second.read_sfr(RFERRF), ErrTxUnderf);
    CHECK_EQ(state(second), 56u); // Table 23-3: TX underflow
    CHECK_EQ(second.read_sfr(RFIRQF1) & F1TxDone, 0);
    run_to(second, sfd + Ramp);
    CHECK_EQ(state(second), 0u);
    CHECK(air.frames().empty());
    second.write_sfr(RFST, IsFlushTx);
    CHECK_EQ(x(second, TXFIFOCNT), 0);
}

// Checks the RXFIFO content of one received frame at `at` (23.10.3,
// AUTOCRC = 1): LEN, MPDU, then RSSI and CRC_OK | correlation in place of
// the FCS. RSSI: -50 dBm (VirtualAir ideal propagation) + 73 dB offset;
// correlation 110 is Model-defined ("approximately 110" for a
// maximum-quality frame, 23.10.4).
void check_rx_frame(const Cc2530F256 &device, std::uint16_t at,
                    const std::vector<std::uint8_t> &m, bool crc_ok = true) {
    CHECK_EQ(x(device, static_cast<std::uint16_t>(RxFifo + at)), m.size() + 2);
    for (unsigned i = 0; i < m.size(); ++i)
        CHECK_EQ(x(device, static_cast<std::uint16_t>(RxFifo + at + 1 + i)), m[i]);
    CHECK_EQ(x(device, static_cast<std::uint16_t>(RxFifo + at + 1 + m.size())), 0x17);
    CHECK_EQ(x(device, static_cast<std::uint16_t>(RxFifo + at + 2 + m.size())),
             crc_ok ? 0x80 | 110 : 110);
}

void test_receive_filter_source_match_and_ack() {
    VirtualAir air;
    VirtualRadioNode peer(air, "peer");
    auto cfg = address_config(0x0001);
    cfg.push_back({FRMCTRL0, 0x60}); // AUTOCRC | AUTOACK
    Flash flash = program_with(cfg);
    const std::uint16_t ready = flash.here();
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    device.attach_air(air, "dut");
    run_to_pc(device, ready);
    const std::uint64_t t = 8192;
    run_to(device, t);
    device.write_sfr(RFST, IsRxOn);
    // 23.9.2/Figure 23-20: RX calibration (192 us), then SFD search.
    CHECK_EQ(x(device, RXENABLE), 0x80);
    CHECK_EQ(state(device), 2u);
    run_to(device, t + Turn - 1);
    CHECK_EQ(state(device), 2u);
    run_to(device, t + Turn);
    CHECK_EQ(state(device), 3u);
    CHECK_EQ(x(device, FSMSTAT1) & (S1TxActive | S1RxActive), S1RxActive);
    // RSSI valid 8 symbol periods after RX is enabled (RSSISTAT, 23.10.3);
    // RSSI reads -128 until then. The noise level (-100 dBm, the VirtualAir
    // noise floor) is Model-defined: -100 + 73 = -27.
    run_to(device, t + Turn + RssiValid - 1);
    CHECK_EQ(x(device, RSSISTAT), 0x00);
    CHECK_EQ(x(device, RSSI), 0x80);
    run_to(device, t + Turn + RssiValid);
    CHECK_EQ(x(device, RSSISTAT), 0x01);
    CHECK_EQ(x(device, RSSI), 0xE5);

    // ---- frame 1: data frame to 0x0001 from 0x0002, ACK requested ----
    const auto m1 = mpdu(FcfDataAck, 0x05, 0x1234, 0x0001, 0x0002, {0x11, 0x22, 0x33});
    const std::uint64_t p1 = t + 16384;
    peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(p1), m1);
    // The 8-symbol RSSI average covers only the frame 128 us after its start.
    run_to(device, p1 + RssiValid);
    CHECK_EQ(x(device, RSSI), 0x17);
    // SFD 160 us (preamble + SFD) after the frame start (23.9.4).
    const std::uint64_t s1 = p1 + Shr;
    run_to(device, s1 - 1);
    CHECK_EQ(device.read_sfr(RFIRQF0), 0x00);
    CHECK_EQ(state(device), 3u);
    run_to(device, s1);
    CHECK_EQ(device.read_sfr(RFIRQF0), F0Sfd);
    CHECK_EQ(x(device, FSMSTAT1) & S1Sfd, S1Sfd);
    CHECK_EQ(state(device), 7u); // Table 23-3: RX
    // Octet i of the PHY payload (0 = length) is complete 32 us after the
    // previous one.
    run_to(device, s1 + Octet - 1);
    CHECK_EQ(x(device, RXFIFOCNT), 0);
    CHECK_EQ(x(device, FSMSTAT1) & S1Fifo, 0);
    run_to(device, s1 + Octet);
    CHECK_EQ(x(device, RXFIFOCNT), 1);
    CHECK_EQ(x(device, RXFIRST), m1.size() + 2);
    CHECK_EQ(x(device, FSMSTAT1) & S1Fifo, S1Fifo); // 23.10.1
    run_to(device, s1 + 4 * Octet);
    CHECK_EQ(x(device, RXFIFOCNT), 4);
    // Figure 23-12: the filtering result is known once FCF, sequence
    // number, destination PAN/address (and source PAN) are received; with
    // PAN ID compression that is octet 7 (destination address MSB).
    run_to(device, s1 + 8 * Octet - 1);
    CHECK_EQ(device.read_sfr(RFIRQF0) & F0FrameAccepted, 0);
    run_to(device, s1 + 8 * Octet);
    CHECK_EQ(device.read_sfr(RFIRQF0) & F0FrameAccepted, F0FrameAccepted);
    // Figure 23-14: source matching completes with the source address
    // (octet 9). Entry 3 matches: SRCRESMASK bit 3, SRCRESINDEX 3; no
    // pending bit (data frame, SRCMATCH.PEND_DATAREQ_ONLY = 1).
    run_to(device, s1 + 10 * Octet - 1);
    CHECK_EQ(device.read_sfr(RFIRQF0) & (F0SrcMatchDone | F0SrcMatchFound), 0);
    run_to(device, s1 + 10 * Octet);
    CHECK_EQ(device.read_sfr(RFIRQF0) & (F0SrcMatchDone | F0SrcMatchFound),
             F0SrcMatchDone | F0SrcMatchFound);
    CHECK_EQ(x(device, SRCRESMASK0), 0x08);
    CHECK_EQ(x(device, SRCRESMASK0 + 1), 0x00);
    CHECK_EQ(x(device, SRCRESMASK0 + 2), 0x00);
    CHECK_EQ(x(device, SRCRESINDEX), 0x03);
    // Frame end: LEN + 1 octets after the SFD. RX_PKTDONE, FIFOP (last
    // octet of a frame, 23.10.1), the complete frame in the RXFIFO.
    const unsigned len1 = static_cast<unsigned>(m1.size() + 2);
    const std::uint64_t e1 = s1 + (1 + len1) * Octet;
    run_to(device, e1 - 1);
    CHECK_EQ(device.read_sfr(RFIRQF0) & F0RxPktDone, 0);
    CHECK_EQ(x(device, FSMSTAT1) & S1Sfd, S1Sfd);
    run_to(device, e1);
    CHECK_EQ(device.read_sfr(RFIRQF0), F0Sfd | F0FrameAccepted | F0SrcMatchFound |
                                           F0SrcMatchDone | F0RxPktDone | F0Fifop);
    CHECK_EQ(x(device, FSMSTAT1) & (S1Fifo | S1Fifop | S1Sfd), S1Fifo | S1Fifop);
    CHECK_EQ(x(device, RXFIFOCNT), len1 + 1);
    CHECK_EQ(x(device, RXFIRST_PTR), 0);
    CHECK_EQ(x(device, RXLAST_PTR), len1 + 1);
    check_rx_frame(device, 0, m1);
    // 23.9.7: ACK 12 symbol periods (192 us) after the frame; ACK
    // calibration, then the ACK frame: FCF 0x0002, sequence number 5.
    CHECK_EQ(state(device), 48u);
    run_to(device, e1 + Turn - 1);
    CHECK_EQ(state(device), 48u);
    CHECK_EQ(frames_from(air, "dut").size(), 0u);
    run_to(device, e1 + Turn);
    CHECK_EQ(state(device), 49u);
    auto acks = frames_from(air, "dut");
    CHECK_EQ(acks.size(), 1u);
    if (acks.size() == 1) {
        CHECK((acks[0].payload == std::vector<std::uint8_t>{0x02, 0x00, 0x05}));
        CHECK_EQ(acks[0].start_ns, ns(e1 + Turn));
        CHECK_EQ(acks[0].end_ns - acks[0].start_ns, (3 + 8) * 32'000ull);
    }
    // TX_ACK_DONE after the 11-octet ACK PPDU, then back to RX (RXENABLE
    // bit 7 still set) through TX final / transit.
    device.write_sfr(RFIRQF0, 0x00);
    const std::uint64_t a1 = e1 + Turn + 11 * Octet;
    run_to(device, a1 - 1);
    CHECK_EQ(device.read_sfr(RFIRQF1), 0x00);
    run_to(device, a1);
    CHECK_EQ(device.read_sfr(RFIRQF1), F1TxAckDone);
    CHECK_EQ(state(device), 39u);
    run_to(device, a1 + Ramp);
    CHECK_EQ(state(device), 40u);
    run_to(device, a1 + Turn);
    CHECK_EQ(state(device), 3u);
    device.write_sfr(RFIRQF0, 0x00);
    device.write_sfr(RFIRQF1, 0x00);

    // ---- frame 2: MAC data request from 0x0002 (pending data) ----
    const auto m2 = mpdu(FcfCmdAck, 0x06, 0x1234, 0x0001, 0x0002, {0x04});
    const std::uint64_t p2 = t + 65536;
    peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(p2), m2);
    const std::uint64_t s2 = p2 + Shr;
    const unsigned len2 = static_cast<unsigned>(m2.size() + 2);
    const std::uint64_t e2 = s2 + (1 + len2) * Octet;
    run_to(device, e2);
    CHECK_EQ(device.read_sfr(RFIRQF0), F0Sfd | F0FrameAccepted | F0SrcMatchFound |
                                           F0SrcMatchDone | F0RxPktDone | F0Fifop);
    // 23.9.6: SRCRESINDEX bit 6 = pending bit for the matched entry
    // (SRCSHORTPENDEN bit 3, AUTOPEND, data request command).
    CHECK_EQ(x(device, SRCRESINDEX), 0x43);
    CHECK_EQ(x(device, RXFIFOCNT), len1 + 1 + len2 + 1);
    CHECK_EQ(x(device, RXP1_PTR), 0);
    check_rx_frame(device, static_cast<std::uint16_t>(len1 + 1), m2);
    run_to(device, e2 + Turn);
    acks = frames_from(air, "dut");
    CHECK_EQ(acks.size(), 2u);
    if (acks.size() == 2) {
        // Frame pending bit set (FCF 0x0012).
        CHECK((acks[1].payload == std::vector<std::uint8_t>{0x12, 0x00, 0x06}));
        CHECK_EQ(acks[1].start_ns, ns(e2 + Turn));
    }
    const std::uint64_t a2 = e2 + Turn + 11 * Octet;
    run_to(device, a2 + Turn);
    CHECK_EQ(state(device), 3u);
    device.write_sfr(RFIRQF0, 0x00);
    device.write_sfr(RFIRQF1, 0x00);

    // ---- frame 3: other PAN (0x4321): rejected (23.9.5) ----
    const auto m3 = mpdu(FcfDataAck, 0x07, 0x4321, 0x0001, 0x0002, {0x44});
    const std::uint64_t p3 = t + 7 * 16384;
    peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(p3), m3);
    const std::uint64_t s3 = p3 + Shr;
    const unsigned len3 = static_cast<unsigned>(m3.size() + 2);
    const std::uint64_t e3 = s3 + (1 + len3) * Octet;
    const unsigned stored = len1 + 1 + len2 + 1;
    run_to(device, s3 + 8 * Octet - 1);
    CHECK_EQ(device.read_sfr(RFIRQF0), F0Sfd);
    CHECK_EQ(x(device, RXFIFOCNT), stored + 7);
    CHECK_EQ(x(device, FSMSTAT1) & S1Sfd, S1Sfd);
    // Rejected: no RX_FRM_ACCEPTED, the frame is removed from the RXFIFO
    // and FSMSTAT1.SFD goes low (Figures 23-11, 23-12, 23-19).
    run_to(device, s3 + 8 * Octet);
    CHECK_EQ(device.read_sfr(RFIRQF0), F0Sfd);
    CHECK_EQ(x(device, RXFIFOCNT), stored);
    CHECK_EQ(x(device, FSMSTAT1) & S1Sfd, 0);
    // The search resumes after the complete rejected frame; no ACK.
    run_to(device, e3);
    CHECK_EQ(device.read_sfr(RFIRQF0), F0Sfd);
    CHECK_EQ(state(device), 3u);
    run_to(device, e3 + Turn + 11 * Octet);
    CHECK_EQ(frames_from(air, "dut").size(), 2u);
    CHECK_EQ(x(device, RXFIFOCNT), stored);

    // SFLUSHRX (23.10): RXFIFO emptied; in RX the FSM recalibrates
    // (Figure 23-20).
    device.write_sfr(RFST, IsFlushRx);
    CHECK_EQ(x(device, RXFIFOCNT), 0);
    CHECK_EQ(x(device, RXFIRST_PTR), 0);
    CHECK_EQ(x(device, RXLAST_PTR), 0);
    CHECK_EQ(x(device, FSMSTAT1) & (S1Fifo | S1Fifop), 0);
    CHECK_EQ(state(device), 2u);
}

void test_cca() {
    VirtualAir air;
    VirtualRadioNode peer(air, "peer");
    // Reset CCA_THR (-32): "the CCA signal never indicates a clear channel"
    // (CCACTRL0 note); 0xF8 is the recommended value.
    for (const bool recommended : {false, true}) {
        Flash flash = recommended ? program({{CCACTRL0, 0xF8}}) : program({});
        const std::uint16_t ready = flash.here();
        sled(flash);
        Cc2530F256 device;
        load(device, flash);
        device.attach_air(air, recommended ? "dut" : "dut-default");
        run_to_pc(device, ready);
        const std::uint64_t t = 4096;
        run_to(device, t);
        device.write_sfr(RFST, IsRxOn);
        const std::uint64_t valid = t + Turn + RssiValid;
        // 23.8.12: CCA is updated four clock cycles after RSSI_VALID.
        // Model-defined: it reads 0 before that.
        run_to(device, valid + 3);
        CHECK_EQ(x(device, FSMSTAT1) & S1Cca, 0);
        run_to(device, valid + 4);
        if (!recommended) {
            CHECK_EQ(x(device, FSMSTAT1) & S1Cca, 0);
            continue;
        }
        // Noise (Model-defined -100 dBm, RSSI -27) < -8 - CCA_HYST (2).
        CHECK_EQ(x(device, FSMSTAT1) & S1Cca, S1Cca);
        CHECK_EQ(x(device, FSMSTAT1) & S1SampledCca, 0);
        device.write_sfr(RFST, IsSampleCca);
        CHECK_EQ(x(device, FSMSTAT1) & S1SampledCca, S1SampledCca);
        const auto m = mpdu(FcfData, 0x09, 0xFFFF, 0xFFFF, 0x0002, {0x01});
        const auto own = mpdu(FcfData, 0x0A, 0xFFFF, 0xFFFF, 0x0001, {0x02});
        fill_txfifo(device, own);
        const std::uint64_t p = 32768;
        peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(p), m);
        // A -50 dBm frame (RSSI 23) on the channel: CCA = 0 (RSSI >= CCA_THR).
        run_to(device, p + RssiValid);
        CHECK_EQ(x(device, RSSI), 0x17);
        CHECK_EQ(x(device, FSMSTAT1) & S1Cca, 0);
        device.write_sfr(RFST, IsSampleCca);
        CHECK_EQ(x(device, FSMSTAT1) & S1SampledCca, 0);
        CHECK_EQ(device.read_sfr(RFIRQF1), 0x00);
        device.write_sfr(RFIRQF1, 0x00);
        // STXONCCA with CCA = 0: no transmission, SAMPLED_CCA = 0 (23.8.1).
        device.write_sfr(RFST, IsTxOnCca);
        CHECK_EQ(state(device), 3u);
        CHECK_EQ(x(device, FSMSTAT1) & S1SampledCca, 0);
        // While the frame is received CCA stays 0 (CCA_MODE 11).
        const std::uint64_t e = p + Shr + (1 + m.size() + 2) * Octet;
        run_to(device, p + Shr + 2 * Octet);
        CHECK_EQ(state(device), 7u);
        CHECK_EQ(x(device, FSMSTAT1) & S1Cca, 0);
        // 8 symbol periods after the frame the channel is clear again.
        run_to(device, e + 2 * RssiValid);
        CHECK_EQ(x(device, RSSI), 0xE5);
        CHECK_EQ(x(device, FSMSTAT1) & S1Cca, S1Cca);
        CHECK(frames_from(air, "dut").empty());
        const std::uint64_t tx = e + 2 * RssiValid;
        device.write_sfr(RFST, IsTxOnCca);
        CHECK_EQ(state(device), 32u);
        CHECK_EQ(x(device, FSMSTAT1) & S1SampledCca, S1SampledCca);
        run_to(device, tx + Turn);
        const auto sent = frames_from(air, "dut");
        CHECK_EQ(sent.size(), 1u);
        if (sent.size() == 1) {
            CHECK(sent[0].payload == own);
            CHECK_EQ(sent[0].start_ns, ns(tx + Turn));
        }
    }
}

void test_collision_fails_crc_and_suppresses_ack() {
    VirtualAir air;
    VirtualRadioNode peer1(air, "peer1"), peer2(air, "peer2");
    auto cfg = address_config(0x0001);
    cfg.push_back({FRMCTRL0, 0x60});
    Flash flash = program_with(cfg);
    const std::uint16_t ready = flash.here();
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    device.attach_air(air, "dut");
    run_to_pc(device, ready);
    run_to(device, 4096);
    device.write_sfr(RFST, IsRxOn);
    const auto m1 = mpdu(FcfDataAck, 0x21, 0x1234, 0x0001, 0x0002, {0xA1, 0xA2});
    const auto m2 = mpdu(FcfData, 0x22, 0x1234, 0xFFFF, 0x0003, {0xB1, 0xB2, 0xB3, 0xB4});
    const std::uint64_t p = 32768;
    peer1.transmit("", RadioProtocol::Zigbee802154, 11, ns(p), m1);
    // Overlapping frame of another node (VirtualAir ideal model: any
    // overlap on the channel is a collision).
    peer2.transmit("", RadioProtocol::Zigbee802154, 11, ns(p + 4 * Octet), m2);
    const unsigned len = static_cast<unsigned>(m1.size() + 2);
    const std::uint64_t e = p + Shr + (1 + len) * Octet;
    run_to(device, e);
    // RX_PKTDONE is set for every completed frame; the FCS check fails:
    // CRC_OK = 0 in the last appended octet (23.10.3).
    CHECK_EQ(device.read_sfr(RFIRQF0) & F0RxPktDone, F0RxPktDone);
    CHECK_EQ(x(device, RXFIFOCNT), len + 1);
    CHECK_EQ(x(device, static_cast<std::uint16_t>(RxFifo + len)) & 0x80, 0);
    // 23.9.7: no acknowledgment for a frame with a bad FCS.
    run_to(device, e + 2 * Turn + 11 * Octet);
    CHECK(frames_from(air, "dut").empty());
    CHECK_EQ(device.read_sfr(RFIRQF1) & F1TxAckDone, 0);
}

void test_two_devices_data_and_ack() {
    VirtualAir air;
    auto cfg_a = address_config(0x0001);
    auto cfg_b = address_config(0x0002);
    cfg_b.push_back({FRMCTRL0, 0x60}); // B acknowledges automatically
    Flash flash_a = program_with(cfg_a), flash_b = program_with(cfg_b);
    const std::uint16_t ready_a = flash_a.here(), ready_b = flash_b.here();
    sled(flash_a);
    sled(flash_b);
    Cc2530F256 a, b;
    load(a, flash_a);
    load(b, flash_b);
    a.attach_air(air, "A");
    b.attach_air(air, "B");
    run_to_pc(a, ready_a);
    run_to_pc(b, ready_b);
    run_to(b, 4096);
    b.write_sfr(RFST, IsRxOn);
    const auto m = mpdu(FcfDataAck, 0x33, 0x1234, 0x0002, 0x0001, {0xC0, 0xFF, 0xEE});
    fill_txfifo(a, m);
    const std::uint64_t ta = 16384;
    run_to(a, ta);
    a.write_sfr(RFST, IsTxOn);
    const unsigned len = static_cast<unsigned>(m.size() + 2);
    const std::uint64_t e = ta + Turn + Shr + (1 + len) * Octet;
    const std::uint64_t ack_end = e + Turn + 11 * Octet;
    // Lockstep in 4096-cycle quanta (less than the 160 us synchronization
    // header), A before B.
    for (std::uint64_t q = ta + 4096; q < ack_end + 8192; q += 4096) {
        run_to(a, q);
        run_to(b, q);
    }
    // B received the frame and acknowledged it.
    CHECK_EQ(x(b, RXFIFOCNT), len + 1);
    check_rx_frame(b, 0, m);
    const auto acks = frames_from(air, "B");
    CHECK_EQ(acks.size(), 1u);
    if (acks.size() == 1) {
        CHECK((acks[0].payload == std::vector<std::uint8_t>{0x02, 0x00, 0x33}));
        CHECK_EQ(acks[0].start_ns, ns(e + Turn));
    }
    // A returned to RX after TX (SET_RXENMASK_ON_TX) and received the ACK:
    // LEN 5, FCF 0x0002, sequence number, RSSI, CRC_OK | correlation.
    CHECK_EQ(x(a, RXFIFOCNT), 6);
    check_rx_frame(a, 0, {0x02, 0x00, 0x33});
    CHECK_EQ(a.read_sfr(RFIRQF0) & F0RxPktDone, F0RxPktDone);
    CHECK_EQ(a.read_sfr(RFIRQF1) & F1TxDone, F1TxDone);
    CHECK_EQ(b.read_sfr(RFIRQF1) & F1TxAckDone, F1TxAckDone);
}

// MAC timer with period `period`, started at `ts` (SYNC = 0: immediately).
void start_mac_timer(Cc2530F256 &device, std::uint16_t period, std::uint64_t ts) {
    device.write_sfr(T2MSEL, 0x02);
    device.write_sfr(T2M0, lo(period));
    device.write_sfr(T2M1, hi(period));
    run_to(device, ts);
    device.write_sfr(T2CTRL, 0x01);
}

void test_csp_programs() {
    constexpr std::uint16_t P = 1000;
    // ---- program 1: WAIT 2; INT; WAITX; LABEL; INCY; DECZ; RPT !(Z = 0); SSTOP
    {
        Flash flash = program({{CSPX, 3}, {CSPZ, 3}});
        const std::uint16_t ready = flash.here();
        sled(flash);
        Cc2530F256 device;
        load(device, flash);
        run_to_pc(device, ready);
        const std::uint64_t ts = 4096;
        start_mac_timer(device, P, ts);
        const std::uint8_t prog[] = {0x82, 0xBA, 0xBC, 0xBB, 0xC1, 0xC5, 0xAE, 0xD2};
        for (const std::uint8_t op : prog)
            device.write_sfr(RFST, op);
        // 23.14.1: sequential writes from location 0; the rest keeps SNOP.
        for (unsigned i = 0; i < sizeof prog; ++i)
            CHECK_EQ(x(device, static_cast<std::uint16_t>(CSPPROG0 + i)), prog[i]);
        CHECK_EQ(x(device, CSPPROG0 + 8), 0xD0);
        CHECK_EQ(x(device, CSPSTAT), 0x00);
        const std::uint64_t t0 = ts + 100;
        run_to(device, t0);
        device.write_sfr(RFST, IsStart);
        // 23.14.3/23.14.7: RFST returns the instruction being executed,
        // CSPSTAT the running bit and the program counter.
        run_to(device, t0 + 50);
        CHECK_EQ(device.read_sfr(RFST), 0x82);
        CHECK_EQ(x(device, CSPSTAT), 0x20);
        // WAIT 2 continues at the second MAC timer overflow (Ts + 2P) with
        // IRQ_CSP_WT; INT follows. Model-defined: one instruction per
        // 32 MHz cycle, the instruction after a wait executes in the next
        // cycle.
        const std::uint64_t o2 = ts + 2 * P;
        run_to(device, o2 - 1);
        CHECK_EQ(device.read_sfr(RFIRQF1), 0x00);
        run_to(device, o2);
        CHECK_EQ(device.read_sfr(RFIRQF1), F1CspWait);
        run_to(device, o2 + 1);
        CHECK_EQ(device.read_sfr(RFIRQF1), F1CspWait | F1CspManInt);
        device.write_sfr(RFIRQF1, 0x00);
        // WAITX: X decremented at each overflow (23.14.9.10).
        run_to(device, o2 + 100);
        CHECK_EQ(device.read_sfr(RFST), 0xBC);
        CHECK_EQ(x(device, CSPSTAT), 0x22);
        CHECK_EQ(x(device, CSPX), 3);
        run_to(device, ts + 3 * P);
        CHECK_EQ(x(device, CSPX), 2);
        run_to(device, ts + 4 * P);
        CHECK_EQ(x(device, CSPX), 1);
        const std::uint64_t o5 = ts + 5 * P;
        run_to(device, o5 - 1);
        CHECK_EQ(device.read_sfr(RFIRQF1), 0x00);
        run_to(device, o5);
        CHECK_EQ(device.read_sfr(RFIRQF1), F1CspWait);
        CHECK_EQ(x(device, CSPX), 0);
        // LABEL sets the loop start to INCY; RPT repeats while Z != 0:
        // 1 + 3 * 3 instructions, then SSTOP (IRQ_CSP_STOP).
        run_to(device, o5 + 10);
        CHECK_EQ(device.read_sfr(RFIRQF1), F1CspWait);
        CHECK_EQ(x(device, CSPSTAT) & 0x20, 0x20);
        run_to(device, o5 + 11);
        CHECK_EQ(device.read_sfr(RFIRQF1), F1CspWait | F1CspStop);
        CHECK_EQ(x(device, CSPSTAT) & 0x20, 0x00);
        CHECK_EQ(device.read_sfr(RFST), 0xD0);
        CHECK_EQ(x(device, CSPY), 3);
        CHECK_EQ(x(device, CSPZ), 0);
        // SSTOP reset the write pointer to 0 (23.14.1); the program stays.
        device.write_sfr(RFST, 0xC0);
        CHECK_EQ(x(device, CSPPROG0), 0xC0);
        CHECK_EQ(x(device, CSPPROG0 + 1), 0xBA);

        // ---- program 3: ISSTOP while waiting; immediate strobes run
        // while the program executes (23.14.3).
        device.write_sfr(RFST, IsStop); // write pointer to 0
        device.write_sfr(RFST, 0xFF);   // ISCLEAR: all SNOP
        CHECK_EQ(x(device, CSPPROG0), 0xD0);
        CHECK_EQ(x(device, CSPPROG0 + 1), 0xD0);
        device.write_sfr(RFST, 0x85); // WAIT 5
        device.write_sfr(RFIRQF1, 0x00);
        const std::uint64_t t3 = ts + 20 * P + 10;
        run_to(device, t3);
        device.write_sfr(RFST, IsStart);
        run_to(device, t3 + 500);
        CHECK_EQ(x(device, CSPSTAT), 0x20);
        device.write_sfr(RFST, IsRxOn);
        CHECK_EQ(state(device), 2u);
        CHECK_EQ(x(device, CSPSTAT), 0x20);
        run_to(device, t3 + 600);
        device.write_sfr(RFST, IsStop);
        CHECK_EQ(x(device, CSPSTAT) & 0x20, 0x00);
        CHECK_EQ(device.read_sfr(RFIRQF1), F1CspStop);
        run_to(device, t3 + 6 * P);
        CHECK_EQ(device.read_sfr(RFIRQF1), F1CspStop);
    }
    // ---- program 2: SKIP 1 if X = 0; INCY; INCZ; SRXON; WEVENT1; INT; WAIT 0
    // with CSPT = 3 (23.14.2: decremented at each MAC timer overflow, the
    // program stops at 0 with IRQ_CSP_STOP).
    {
        Flash flash = program({{CSPT, 3}});
        const std::uint16_t ready = flash.here();
        sled(flash);
        Cc2530F256 device;
        load(device, flash);
        run_to_pc(device, ready);
        constexpr std::uint16_t Cmp1 = 300;
        device.write_sfr(T2MSEL, 0x03);
        device.write_sfr(T2M0, lo(Cmp1));
        device.write_sfr(T2M1, hi(Cmp1));
        device.write_sfr(T2EVTCFG, 0x71); // EVENT1 = t2_cmp1, EVENT2 none (22.5)
        const std::uint64_t ts = 4096;
        start_mac_timer(device, P, ts);
        for (const std::uint8_t op : {0x14, 0xC1, 0xC2, 0xD3, 0xB8, 0xBA, 0x80})
            device.write_sfr(RFST, op);
        const std::uint64_t t0 = ts + 100;
        run_to(device, t0);
        device.write_sfr(RFST, IsStart);
        // Model-defined instruction timing: SKIP at t0 + 1, INCZ at t0 + 2,
        // SRXON at t0 + 3.
        run_to(device, t0 + 2);
        CHECK_EQ(state(device), 0u);
        run_to(device, t0 + 3);
        CHECK_EQ(state(device), 2u);
        CHECK_EQ(x(device, RXENABLE), 0x80);
        CHECK_EQ(x(device, CSPY), 0);
        CHECK_EQ(x(device, CSPZ), 1);
        // WEVENT1 continues at the compare-1 event (Ts + 300) without
        // IRQ_CSP_WT (23.14.4); INT in the next cycle.
        run_to(device, ts + Cmp1);
        CHECK_EQ(device.read_sfr(RFIRQF1), 0x00);
        run_to(device, ts + Cmp1 + 1);
        CHECK_EQ(device.read_sfr(RFIRQF1), F1CspManInt);
        run_to(device, ts + P);
        CHECK_EQ(x(device, CSPT), 2);
        run_to(device, ts + 3 * P - 1);
        CHECK_EQ(x(device, CSPT), 1);
        CHECK_EQ(x(device, CSPSTAT) & 0x20, 0x20);
        run_to(device, ts + 3 * P);
        CHECK_EQ(x(device, CSPT), 0);
        CHECK_EQ(x(device, CSPSTAT) & 0x20, 0x00);
        CHECK_EQ(device.read_sfr(RFIRQF1), F1CspManInt | F1CspStop);
    }
}

void test_interrupts_through_vectors() {
    VirtualAir air;
    Flash flash = program({{RFIRQM0, F0Sfd}, {RFIRQM1, F1TxDone}, {RFERRM, ErrStrobe}}, true, true);
    const std::uint16_t ready = flash.here();
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    device.attach_air(air, "dut");
    run_to_pc(device, ready);
    const auto m = mpdu(FcfData, 0x44, 0xFFFF, 0xFFFF, 0x0001, {0x55});
    const unsigned len = static_cast<unsigned>(m.size() + 2);
    fill_txfifo(device, m);
    const std::uint64_t t = 4096;
    run_to(device, t);
    device.write_sfr(RFST, IsTxOn);
    // 23.1: RF interrupt (vector 0x83) when an unmasked flag is set; the
    // CPU enters the ISR two cycles after the flag (LCALL to the vector).
    const std::uint64_t sfd = t + Turn + Shr;
    CHECK_EQ(run_to_pc(device, 0x0083), sfd + 2);
    const std::uint64_t end = sfd + (1 + len) * Octet;
    CHECK_EQ(run_to_pc(device, 0x0083), end + 2);
    run_until(device, end + 200);
    CHECK_EQ(device.read_idata(0x30), 2);
    CHECK_EQ(device.read_idata(0x31), F0Sfd);
    CHECK_EQ(device.read_idata(0x32) & F1TxDone, F1TxDone);
    CHECK_EQ(device.read_sfr(S1CON), 0x00);
    CHECK_EQ(device.read_sfr(RFIRQF0), 0x00);
    // SRFOFF in RX (no error), then SRFOFF while idle: STROBEERR -> RFERR
    // (vector 0x03); TCON.RFERRIF is cleared by hardware on entry.
    const std::uint64_t r = end + Turn + 1024;
    run_until(device, r);
    device.write_sfr(RFST, IsRfOff);
    CHECK_EQ(device.read_sfr(RFERRF), 0x00);
    const std::uint64_t err = device.registers().cycles + 100;
    run_to(device, err);
    device.write_sfr(RFST, IsRfOff);
    CHECK_EQ(device.read_sfr(RFERRF), ErrStrobe);
    CHECK_EQ(device.read_sfr(TCON) & 0x02, 0x02);
    CHECK_EQ(run_to_pc(device, 0x0003), err + 2);
    CHECK_EQ(device.read_sfr(TCON) & 0x02, 0x00);
    run_until(device, err + 100);
    CHECK_EQ(device.read_idata(0x33), 1);
    CHECK_EQ(device.read_idata(0x34), ErrStrobe);
    CHECK_EQ(device.read_sfr(RFERRF), 0x00);
}

// FIFOP threshold boundary (23.10.1; FSMSTAT1/FIFOPCTRL in 23.15): FIFOP is
// high when *more than* FIFOP_THR valid bytes are in the RXFIFO, and goes low
// again when a read leaves FIFOP_THR bytes. With frame filtering enabled the
// header bytes count only after the frame is accepted (Figure 23-19).
void test_fifop_threshold() {
    {
        // Frame filtering off (FRMFILT0.FRM_FILTER_EN = 0): every byte is valid.
        Flash flash = program_with({{FRMFILT0, 0x0C}, {FIFOPCTRL, 0x00}});
        const std::uint16_t ready = flash.here();
        flash.op(movx_write(FIFOPCTRL, 0x05));
        // MOV DPTR,#FSMSTAT1; poll: MOVX A,@DPTR; JNB ACC.6,poll
        flash.op({0x90, hi(FSMSTAT1), lo(FSMSTAT1), 0xE0, 0x30, 0xE6, 0xFC});
        // MOV A,RFD; MOV 0x40,A; MOVX A,@DPTR; MOV 0x41,A; RXFIFOCNT -> 0x42
        flash.op({0xE5, RFD, 0xF5, 0x40, 0xE0, 0xF5, 0x41}).op(movx_read_to(RXFIFOCNT, 0x42));
        sled(flash);
        VirtualAir air;
        VirtualRadioNode peer(air, "peer");
        Cc2530F256 device;
        load(device, flash);
        device.attach_air(air, "dut");
        run_to_pc(device, ready);
        // Empty RXFIFO, FIFOP_THR = 0: 0 bytes is not more than 0.
        CHECK_EQ(x(device, FIFOPCTRL), 0x00);
        CHECK_EQ(x(device, RXFIFOCNT), 0);
        CHECK_EQ(x(device, FSMSTAT1) & (S1Fifo | S1Fifop), 0);
        CHECK_EQ(device.read_sfr(RFIRQF0) & F0Fifop, 0);
        const std::uint64_t t = 8192;
        run_until(device, t);
        CHECK_EQ(x(device, FIFOPCTRL), 0x05);
        device.write_sfr(RFST, IsRxOn);
        const auto m = mpdu(FcfData, 0x21, 0x1234, 0xFFFF, 0x0002, {1, 2, 3, 4, 5, 6});
        const std::uint64_t s = t + 16384 + Shr;
        peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(t + 16384), m);
        // Octet k of the PHY payload (0 = LEN) is stored at s + (k + 1) * Octet.
        // 5 bytes = FIFOP_THR: FIFOP low and no FIFOP event.
        run_until(device, s + 5 * Octet);
        CHECK_EQ(x(device, RXFIFOCNT), 5);
        CHECK_EQ(x(device, FSMSTAT1) & (S1Fifo | S1Fifop), S1Fifo);
        CHECK_EQ(device.read_sfr(RFIRQF0) & F0Fifop, 0);
        // 6 bytes: FIFOP high (RFIRQF0.FIFOP event); the firmware reads one
        // byte, which leaves FIFOP_THR = 5 bytes: FIFOP low again.
        run_until(device, s + 6 * Octet + 128);
        CHECK_EQ(device.read_idata(0x40), m.size() + 2);
        CHECK_EQ(device.read_idata(0x41) & (S1Fifo | S1Fifop), S1Fifo);
        CHECK_EQ(device.read_idata(0x42), 5);
        CHECK_EQ(device.read_sfr(RFIRQF0) & F0Fifop, F0Fifop);
        CHECK_EQ(x(device, FSMSTAT1) & S1Fifop, 0);
        // 7 received, 1 read: 6 bytes > 5, FIFOP high.
        run_until(device, s + 7 * Octet);
        CHECK_EQ(x(device, RXFIFOCNT), 6);
        CHECK_EQ(x(device, FSMSTAT1) & S1Fifop, S1Fifop);
    }
    {
        // Frame filtering on, FIFOP_THR = 2: the 7 header bytes stored before
        // the filtering result (octet 7, see test_receive_filter_...) are not
        // valid, so FIFOP stays low; it goes high when the frame is accepted.
        auto cfg = address_config(0x0001);
        cfg.push_back({FIFOPCTRL, 0x02});
        Flash flash = program_with(cfg);
        const std::uint16_t ready = flash.here();
        sled(flash);
        VirtualAir air;
        VirtualRadioNode peer(air, "peer");
        Cc2530F256 device;
        load(device, flash);
        device.attach_air(air, "dut");
        run_to_pc(device, ready);
        const std::uint64_t t = 8192;
        run_to(device, t);
        device.write_sfr(RFST, IsRxOn);
        const auto m = mpdu(FcfData, 0x22, 0x1234, 0x0001, 0x0002, {0x11, 0x22, 0x33});
        const std::uint64_t s = t + 16384 + Shr;
        peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(t + 16384), m);
        run_to(device, s + 7 * Octet);
        CHECK_EQ(x(device, RXFIFOCNT), 7);
        CHECK_EQ(x(device, FSMSTAT1) & (S1Fifo | S1Fifop), S1Fifo);
        CHECK_EQ(device.read_sfr(RFIRQF0) & (F0FrameAccepted | F0Fifop), 0);
        run_to(device, s + 8 * Octet);
        CHECK_EQ(device.read_sfr(RFIRQF0) & (F0FrameAccepted | F0Fifop),
                 F0FrameAccepted | F0Fifop);
        CHECK_EQ(x(device, FSMSTAT1) & S1Fifop, S1Fifop);
    }
}

void test_rfd_read_dma_trigger_and_underflow() {
    VirtualAir air;
    VirtualRadioNode peer(air, "peer");
    Flash flash = program({});
    const std::uint16_t poll = flash.here();
    // poll: MOV A,RFIRQF0; JNB ACC.6,poll (RX_PKTDONE)
    flash.op({0xE5, RFIRQF0, 0x30, 0xE6, 0xFB});
    // MOV R0,#0x40; MOV R7,#15; loop: MOV A,RFD; MOV @R0,A; INC R0; DJNZ R7,loop
    flash.op({0x78, 0x40, 0x7F, 0x0F, 0xE5, RFD, 0xF6, 0x08, 0xDF, 0xFA});
    flash.op(movx_read_to(FSMSTAT1, 0x50)).op(movx_read_to(RXFIFOCNT, 0x51));
    // One read too many: RX underflow (23.10.2).
    flash.op({0xE5, RFD, 0xF5, 0x52});
    const std::uint16_t done = flash.here();
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    device.attach_air(air, "dut");
    auto &radio = *device.peripherals().radio;
    run_to_pc(device, poll);
    device.write_sfr(RFST, IsRxOn);
    const auto m = mpdu(FcfData, 0x66, 0xFFFF, 0xFFFF, 0x0002, {0x01, 0x02, 0x03});
    peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(16384), m);
    const std::uint64_t before = radio.dma_trigger_count();
    run_to_pc(device, done);
    run_until(device, device.registers().cycles + 10);
    for (unsigned i = 0; i < m.size() + 3; ++i) {
        const std::uint8_t want = i == 0 ? static_cast<std::uint8_t>(m.size() + 2)
                                  : i <= m.size() ? m[i - 1]
                                  : i == m.size() + 1 ? 0x17
                                                      : 0x80 | 110;
        CHECK_EQ(device.read_idata(static_cast<std::uint8_t>(0x40 + i)), want);
    }
    // 23.10.1: FIFO low when empty; FIFOP back to low at the next read.
    CHECK_EQ(device.read_idata(0x50) & (S1Fifo | S1Fifop), 0);
    CHECK_EQ(device.read_idata(0x51), 0);
    // 23.3: one trigger when the RXFIFO went non-empty, one for each of the
    // 14 reads that left data in the RXFIFO.
    CHECK_EQ(radio.dma_trigger_count() - before, 15u);
    CHECK_EQ(device.read_sfr(RFERRF) & ErrRxUnderf, ErrRxUnderf);
}

void test_reset_keeps_air_attachment() {
    VirtualAir air;
    Flash flash = program({{TXPOWER, 0xD5}});
    const std::uint16_t ready = flash.here();
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    constexpr std::uint64_t Offset = 1'000'000;
    device.attach_air(air, "dut", Offset);
    run_to_pc(device, ready);
    CHECK_EQ(x(device, TXPOWER), 0xD5);
    fill_txfifo(device, {0x01, 0x02, 0x03});
    device.write_sfr(RFST, IsRxOn);
    run_to(device, 8192);
    CHECK_EQ(state(device), 3u);
    // External reset (same Peripheral::reset path as the watchdog reset):
    // all radio chip state back to reset values.
    device.reset();
    CHECK_EQ(state(device), 0u);
    CHECK_EQ(x(device, TXPOWER), 0xF5);
    CHECK_EQ(x(device, RXENABLE), 0x00);
    CHECK_EQ(x(device, TXFIFOCNT), 0);
    CHECK_EQ(x(device, RSSI), 0x80);
    CHECK_EQ(device.read_sfr(RFIRQF0), 0x00);
    // 23.4.3: RAM "unknown after reset"; Model-defined: kept unless power-on.
    CHECK_EQ(x(device, TxFifo), 0x05); // LEN = 3 + 2 (FCS)
    CHECK_EQ(x(device, TxFifo + 1), 0x01);
    // The air attachment, node name and time offset are host wiring.
    const auto m = mpdu(FcfData, 0x77, 0xFFFF, 0xFFFF, 0x0001, {0xAB});
    for (const bool power_on : {false, true}) {
        if (power_on) {
            device.power_on_reset();
            CHECK_EQ(device.registers().cycles, 0u);
            CHECK_EQ(x(device, TxFifo), 0x00); // Model-defined: cleared by power-on
            CHECK_EQ(state(device), 0u);
        }
        run_to_pc(device, ready);
        CHECK_EQ(x(device, TXPOWER), 0xD5);
        fill_txfifo(device, m);
        const std::uint64_t t = (device.registers().cycles / 1024 + 4) * 1024;
        run_to(device, t);
        device.write_sfr(RFST, IsTxOn);
        run_to(device, t + Turn);
        const auto sent = frames_from(air, "dut");
        CHECK_EQ(sent.size(), power_on ? 2u : 1u);
        if (!sent.empty()) {
            CHECK(sent.back().payload == m);
            CHECK_EQ(sent.back().start_ns, Offset + ns(t + Turn));
        }
        run_to(device, t + 2 * Turn + Shr + (m.size() + 3) * Octet);
        CHECK_EQ(device.read_sfr(RFIRQF1) & F1TxDone, F1TxDone);
    }
}

void test_rx_ack_tx_all_modes() {
    auto cfg = address_config(0x0001);
    cfg.push_back({FRMCTRL0, 0x60});
    cfg.push_back({RFIRQM0, F0RxPktDone});
    cfg.push_back({RFIRQM1, F1TxAckDone | F1TxDone});
    Flash flash = program_with(cfg, true);
    flash.op(mov(RFST, IsRxOn));
    // wait: MOV A,0x30; CJNE A,#2,wait (RX_PKTDONE and TX_ACK_DONE seen)
    flash.op({0xE5, 0x30, 0xB4, 0x02, 0xFB});
    const auto reply = mpdu(FcfData, 0x02, 0x1234, 0xFFFF, 0x0001, {0x99, 0x88});
    flash.op(mov(RFD, static_cast<std::uint8_t>(reply.size() + 2)));
    for (const std::uint8_t b : reply)
        flash.op(mov(RFD, b));
    flash.op(mov(RFST, IsTxOn));
    sled(flash);
    const auto m = mpdu(FcfDataAck, 0x5C, 0x1234, 0x0001, 0x0002, {0x10, 0x20});
    const std::uint64_t p = 16384;
    const std::uint64_t e = p + Shr + (1 + m.size() + 2) * Octet;
    const std::uint64_t limit = e + 3 * Turn + (11 + 30) * Octet + 4096;
    std::unique_ptr<VirtualAir> air;
    for_each_mode(
        flash, limit,
        [&](Cc2530F256 &device) {
            air = std::make_unique<VirtualAir>();
            device.attach_air(*air, "dut");
            VirtualRadioNode(*air, "peer").transmit("", RadioProtocol::Zigbee802154, 11, ns(p),
                                                    m);
        },
        [&](Cc2530F256 &device, Mode mode) {
            const auto sent = frames_from(*air, "dut");
            if (sent.size() != 2) {
                std::cerr << "rx/ack/tx [" << mode_name(mode) << "]: " << sent.size()
                          << " frames\n";
                ++failures;
                return;
            }
            CHECK((sent[0].payload == std::vector<std::uint8_t>{0x02, 0x00, 0x5C}));
            CHECK_EQ(sent[0].start_ns, ns(e + Turn));
            CHECK(sent[1].payload == reply);
            check_rx_frame(device, 0, m);
            CHECK_EQ(device.read_idata(0x30), 3);
            // The ISR records all of RFIRQF0: the frame also set SFD, FIFOP,
            // the source-match flags (entry 3) and FRAME_ACCEPTED.
            CHECK_EQ(device.read_idata(0x31), F0RxPktDone | F0FrameAccepted | F0SrcMatchFound |
                                                  F0SrcMatchDone | F0Fifop | F0Sfd);
            CHECK_EQ(device.read_idata(0x32), F1TxAckDone | F1TxDone);
        },
        "radio rx/ack/tx");
}

// FSMCTRL.SLOTTED_ACK = 1 (23.9.8, Figure 23-17): the ACK starts on a
// backoff-slot boundary 12-31 symbol periods after the frame. Boundaries are
// MAC timer overflows (Model-defined) with the IEEE 802.15.4 backoff period
// aUnitBackoffPeriod = 20 symbols = 320 us = 10240 cycles.
void test_slotted_ack() {
    VirtualAir air;
    VirtualRadioNode peer(air, "peer");
    auto cfg = address_config(0x0001);
    cfg.push_back({FRMCTRL0, 0x60});
    cfg.push_back({FSMCTRL, 0x03}); // SLOTTED_ACK | RX2RX_TIME_OFF
    Flash flash = program_with(cfg);
    const std::uint16_t ready = flash.here();
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    device.attach_air(air, "dut");
    run_to_pc(device, ready);
    constexpr std::uint64_t P = 10240;
    const std::uint64_t ts = 4096;
    start_mac_timer(device, static_cast<std::uint16_t>(P), ts);
    device.write_sfr(RFST, IsRxOn);
    const auto m = mpdu(FcfDataAck, 0x40, 0x1234, 0x0001, 0x0002, {0x01});
    const std::uint64_t frame = Shr + (1 + m.size() + 2) * Octet; // start to end
    const std::uint64_t ack_air = 11 * Octet; // SHR, PHR, FCF, DSN, FCS
    // Frame 1: a boundary (overflow 4) exactly 12 symbol periods after the
    // frame end: the ACK starts there (Figure 23-17: 12-31 symbol periods).
    const std::uint64_t b1 = ts + 4 * P;
    const std::uint64_t e1 = b1 - Turn;
    peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(e1 - frame), m);
    run_to(device, b1 - 1);
    CHECK_EQ(state(device), 48u);
    run_to(device, b1);
    CHECK_EQ(state(device), 49u);
    auto acks = frames_from(air, "dut");
    CHECK_EQ(acks.size(), 1u);
    if (acks.size() == 1) {
        CHECK((acks[0].payload == std::vector<std::uint8_t>{0x02, 0x00, 0x40}));
        CHECK_EQ(acks[0].start_ns, ns(b1));
    }
    // Frame 2: the last boundary before 12 symbol periods is 1 cycle early:
    // the ACK waits for the next one (31.99 symbol periods after the frame).
    const std::uint64_t b2 = ts + 9 * P;
    const std::uint64_t e2 = b2 - Turn + 1;
    CHECK(b1 + ack_air + Turn < e2 - frame); // receiver back in SFD search
    const auto m2 = mpdu(FcfDataAck, 0x41, 0x1234, 0x0001, 0x0002, {0x02});
    peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(e2 - frame), m2);
    run_to(device, e2);
    CHECK_EQ(device.read_sfr(RFIRQF0) & F0RxPktDone, F0RxPktDone);
    run_to(device, b2 + P - 1);
    CHECK_EQ(state(device), 48u);
    CHECK_EQ(frames_from(air, "dut").size(), 1u);
    run_to(device, b2 + P);
    CHECK_EQ(state(device), 49u);
    acks = frames_from(air, "dut");
    CHECK_EQ(acks.size(), 2u);
    if (acks.size() == 2) {
        CHECK((acks[1].payload == std::vector<std::uint8_t>{0x02, 0x00, 0x41}));
        CHECK_EQ(acks[1].start_ns, ns(b2 + P));
    }
    // Frame 3 with the MAC timer stopped: no boundary, no ACK; SRFOFF ends
    // the ACK calibration (23.11: SRFOFF aborts, RFIDLE).
    const std::uint64_t r3 = b2 + P + ack_air + Turn;
    run_to(device, r3);
    device.write_sfr(T2CTRL, 0x00);
    CHECK_EQ(state(device), 3u);
    const std::uint64_t p3 = r3 + 4096;
    const auto m3 = mpdu(FcfDataAck, 0x42, 0x1234, 0x0001, 0x0002, {0x03});
    peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(p3), m3);
    run_to(device, p3 + frame + 3 * P);
    CHECK_EQ(state(device), 48u);
    CHECK_EQ(frames_from(air, "dut").size(), 2u);
    device.write_sfr(RFIRQF1, 0x00);
    device.write_sfr(RFST, IsRfOff);
    CHECK_EQ(state(device), 0u);
    CHECK_EQ(device.read_sfr(RFIRQF1) & F1RfIdle, F1RfIdle);
    run_to(device, p3 + frame + 6 * P);
    CHECK_EQ(frames_from(air, "dut").size(), 2u);
}

} // namespace

int main() {
    test_reset_values_and_write_masks();
    test_fifo_and_rxenable_access();
    test_transmit();
    test_transmit_to_idle_and_underflow();
    test_receive_filter_source_match_and_ack();
    test_slotted_ack();
    test_cca();
    test_collision_fails_crc_and_suppresses_ack();
    test_two_devices_data_and_ack();
    test_csp_programs();
    test_interrupts_through_vectors();
    test_fifop_threshold();
    test_rfd_read_dma_trigger_and_underflow();
    test_reset_keeps_air_attachment();
    test_rx_ack_tx_all_modes();
    return finish("cc2530 radio");
}
