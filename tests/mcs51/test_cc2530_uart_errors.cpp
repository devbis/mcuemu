// CC2530F256 USART UART 9th bit, parity error (UxCSR.ERR), framing error
// (UxCSR.FE) and host error frames (SWRU191F 17.1.4, 17.8). Expectations
// come from SWRU191F or from a rule cc2530_usart.cpp documents as
// "Model-defined"; programs are hand-encoded MCS-51 machine code.
//
// Timing: 32 MHz XOSC, 115200 baud (UxBAUD = 216, UxGCR.BAUD_E = 11, Table
// 17-1). Equation 3 (17.4) gives one bit = 2^28 / ((256 + 216) * 2^11)
// cycles (277.69); bit n of a frame ends at the first cycle >= n bit times
// after the frame start.

#include "cc2530_periph_support.hpp"

#include "cc2530_soc.hpp"
#include "cc2530_usart.hpp"

#include <bit>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace mcs51emu::cc2530 {
// White-box access to the URXn/UTXn DMA trigger counters.
struct UartErrorsTestAccess {
    static std::uint64_t usart_dma(Cc2530F256 &device, unsigned index, unsigned trigger) {
        return device.peripherals().usart[index]->dma_triggers_[trigger];
    }
};
} // namespace mcs51emu::cc2530

namespace {

using namespace cc2530test;
using Access = mcs51emu::cc2530::UartErrorsTestAccess;
using Frame = mcs51emu::Cc2530UartFrame;
using Ninth = Frame::Ninth;

// SFRs (SWRU191F Table 2-1).
constexpr std::uint8_t SP = 0x81, U0CSR = 0x86, PCON = 0x87, TCON = 0x88, IEN0 = 0xa8,
                       U0DBUF = 0xc1, U0BAUD = 0xc2, U0UCR = 0xc4, U0GCR = 0xc5,
                       CLKCONCMD = 0xc6, U1CSR = 0xf8, U1DBUF = 0xf9, U1BAUD = 0xfa,
                       U1UCR = 0xfb, U1GCR = 0xfc;
// UxCSR and UxUCR bits (17.8).
constexpr std::uint8_t CSR_FE = 0x10, CSR_ERR = 0x08, CSR_RX_BYTE = 0x04, CSR_ACTIVE = 0x01;
constexpr std::uint8_t UCR_FLUSH = 0x80, UCR_D9 = 0x20, UCR_BIT9 = 0x10, UCR_PARITY = 0x08,
                       UCR_SPB = 0x04;
// UART mode, receiver enabled.
constexpr std::uint8_t CSR_UART_RX = 0xc0;
// TCON.URX0IF/URX1IF; DMA triggers URX0/URX1 (Table 8-1).
constexpr std::uint8_t TCON_URX0IF = 0x08, TCON_URX1IF = 0x80;
constexpr unsigned DMA_URX0 = 14, DMA_URX1 = 16;
// Bit addresses of U1CSR.FE and U1CSR.ERR (U1CSR = 0xF8 is bit addressable).
constexpr std::uint8_t BIT_U1CSR_FE = 0xfc, BIT_U1CSR_ERR = 0xfb;

constexpr std::uint64_t bit_end(unsigned n) {
    constexpr std::uint64_t num = std::uint64_t{1} << 28, den = (256 + 216) * 2048;
    return (n * num + den - 1) / den;
}
static_assert(bit_end(10) == 2777 && bit_end(11) == 3055 && bit_end(12) == 3333);

// 17.1.4: frame layout of a UxUCR value.
unsigned first_stop_end(std::uint8_t ucr) { return (ucr & UCR_BIT9) ? 11 : 10; }
unsigned frame_bits(std::uint8_t ucr) { return first_stop_end(ucr) + ((ucr & UCR_SPB) ? 1 : 0); }

// UxUCR.D9 with PARITY = 1 (17.8): "0: Odd parity, 1: Even parity". The
// parity bit makes the number of ones in data + parity odd or even.
bool parity_bit(std::uint8_t data, bool even) {
    const bool ones_odd = (std::popcount(data) & 1) != 0;
    return even ? ones_odd : !ones_odd;
}

struct Usart {
    std::uint8_t csr, dbuf, baud, ucr, gcr, urxif;
    unsigned dma_rx;
};
constexpr Usart Usarts[2] = {{U0CSR, U0DBUF, U0BAUD, U0UCR, U0GCR, TCON_URX0IF, DMA_URX0},
                             {U1CSR, U1DBUF, U1BAUD, U1UCR, U1GCR, TCON_URX1IF, DMA_URX1}};

// MOV PCON,#1; SJMP back: wait in Idle (SLEEPCMD.MODE = 00 at reset).
const std::vector<std::uint8_t> IdleLoop = {0x75, PCON, 0x01, 0x80, 0xfb};

std::uint64_t cycles(const Cc2530F256 &device) { return device.registers().cycles; }

void run_to(Cc2530F256 &device, std::uint64_t cycle) {
    const Cc2530RunResult result = device.run(10'000'000, cycle);
    CHECK(result.error.empty());
}

// USART `index` in UART mode at 115200 with `ucr` and `gcr`, configured by
// the host while the CPU waits in Idle; `csr` selects the receiver. Returns
// the current cycle (host frames start there).
std::uint64_t uart_device(Cc2530F256 &device, unsigned index, std::uint8_t ucr,
                          std::uint8_t gcr = 11, std::uint8_t csr = CSR_UART_RX) {
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80}).op(IdleLoop);
    load(device, flash);
    device.run(100);
    CHECK(device.sleeping());
    const Usart &u = Usarts[index];
    device.write_sfr(u.baud, 216);
    device.write_sfr(u.gcr, gcr);
    device.write_sfr(u.ucr, ucr);
    device.write_sfr(u.csr, csr);
    return cycles(device);
}

// Reports the failing case of a group of checks.
struct Case {
    std::string name;
    unsigned before = failures;
    ~Case() {
        if (failures != before)
            std::cerr << "  in case: " << name << '\n';
    }
};

std::string hex(unsigned value) {
    static const char *digits = "0123456789abcdef";
    return std::string("0x") + digits[(value >> 4) & 15] + digits[value & 15];
}

// ============================================================ transmit

// 17.1.4 / UxUCR: BIT9 = 1 adds a 9th bit, "given by D9" or the parity
// (PARITY = 1, which needs BIT9 too) of the data bits. The captured frame
// holds the wire byte and that bit; the byte stream is unchanged and the
// frame ends after start + 8 (+ 1) + SPB ? 2 : 1 bits.
void test_tx_ninth_bit() {
    std::vector<std::uint8_t> bytes{0x00, 0xff, 0x01, 0x80, 0xa7, 0x5a, 0x7f, 0x03};
    std::uint32_t seed = 0x2530;
    for (int i = 0; i < 24; ++i) {
        seed = seed * 1103515245u + 12345u;
        bytes.push_back(static_cast<std::uint8_t>(seed >> 16));
    }
    // PARITY without BIT9 (0x0a) sends no 9th bit ("one must set BIT9 in
    // addition").
    for (const std::uint8_t ucr : {0x02, 0x12, 0x32, 0x1a, 0x3a, 0x0a, 0x3e, 0x16}) {
        Cc2530F256 device;
        uart_device(device, 0, ucr, 11, 0x80);
        device.uart_capture_frames(0, true);
        for (const std::uint8_t byte : bytes) {
            Case scope{"tx ucr " + hex(ucr) + " byte " + hex(byte)};
            const std::uint64_t c = cycles(device);
            device.write_sfr(U0DBUF, byte);
            run_to(device, c + bit_end(frame_bits(ucr)) - 1);
            CHECK(device.uart_take_transmitted_frames(0).empty());
            CHECK(device.uart_take_transmitted(0).empty());
            run_to(device, c + bit_end(frame_bits(ucr)));
            Ninth want = Ninth::Expected;
            if (ucr & UCR_BIT9) {
                const bool ninth =
                    (ucr & UCR_PARITY) ? parity_bit(byte, ucr & UCR_D9) : (ucr & UCR_D9) != 0;
                want = ninth ? Ninth::One : Ninth::Zero;
            }
            CHECK(device.uart_take_transmitted_frames(0) ==
                  (std::vector<Frame>{Frame{.data = byte, .ninth = want}}));
            CHECK(device.uart_take_transmitted(0) == std::vector<std::uint8_t>{byte});
        }
    }
}

// ORDER = 1 sends the MSB first; frames hold the LSB-first wire byte
// (model-defined, like plain host bytes). The parity counts the same ones.
void test_tx_order_and_latching() {
    {
        Cc2530F256 device;
        uart_device(device, 0, 0x3a, 0x20 | 11, 0x80); // even parity
        device.uart_capture_frames(0, true);
        const std::uint64_t c = cycles(device);
        device.write_sfr(U0DBUF, 0x01); // one 1: even parity bit 1
        device.write_sfr(U0DBUF, 0x03); // waits in the data buffer
        // UxUCR is read when a frame starts (like its length): the first
        // frame keeps even parity, the second one uses odd parity.
        device.write_sfr(U0UCR, 0x1a);
        run_to(device, c + bit_end(22));
        CHECK(device.uart_take_transmitted_frames(0) ==
              (std::vector<Frame>{Frame{.data = 0x80, .ninth = Ninth::One},
                                  Frame{.data = 0xc0, .ninth = Ninth::One}}));
        CHECK(device.uart_take_transmitted(0) == (std::vector<std::uint8_t>{0x80, 0xc0}));
    }
    {
        // Capture is off by default; turning it off drops unread frames.
        Cc2530F256 device;
        const std::uint64_t c = uart_device(device, 0, 0x3a, 11, 0x80);
        device.write_sfr(U0DBUF, 0x55);
        run_to(device, c + bit_end(11));
        CHECK(device.uart_take_transmitted_frames(0).empty());
        CHECK(device.uart_take_transmitted(0) == std::vector<std::uint8_t>{0x55});
        device.uart_capture_frames(0, true);
        device.write_sfr(U0DBUF, 0x56);
        run_to(device, c + bit_end(11) + bit_end(11));
        device.uart_capture_frames(0, false);
        CHECK(device.uart_take_transmitted_frames(0).empty());
        device.uart_capture_frames(0, true);
        device.write_sfr(U0DBUF, 0x57); // five ones: even parity bit 1
        run_to(device, c + 3 * bit_end(11));
        CHECK(device.uart_take_transmitted_frames(0) ==
              (std::vector<Frame>{Frame{.data = 0x57, .ninth = Ninth::One}}));
        CHECK(device.uart_take_transmitted(0) == (std::vector<std::uint8_t>{0x56, 0x57}));
    }
}

// ============================================================ receive

// Receives `frame` alone and checks that RX_BYTE, URXxIF, the URXx DMA
// trigger, UxDBUF and the error flags `flags` all appear at the end of the
// first stop bit (17.1.2, 17.1.4), none of them one cycle before, and that
// nothing changes during the rest of the frame and after it.
void check_rx(unsigned index, std::uint8_t ucr, std::uint8_t gcr, const Frame &frame,
              std::uint8_t data, std::uint8_t flags, const std::string &name) {
    Case scope{name};
    const Usart &u = Usarts[index];
    Cc2530F256 device;
    const std::uint64_t c = uart_device(device, index, ucr, gcr);
    device.uart_receive_frame(index, frame);
    const std::uint64_t done = c + bit_end(first_stop_end(ucr));
    run_to(device, done - 1);
    CHECK_EQ(device.read_sfr(u.csr), CSR_UART_RX | CSR_ACTIVE);
    CHECK_EQ(device.read_sfr(TCON) & u.urxif, 0);
    CHECK_EQ(Access::usart_dma(device, index, u.dma_rx), 0u);
    run_to(device, done);
    CHECK_EQ(device.read_sfr(u.csr), CSR_UART_RX | CSR_RX_BYTE | flags);
    CHECK_EQ(device.read_sfr(TCON) & u.urxif, u.urxif);
    CHECK_EQ(Access::usart_dma(device, index, u.dma_rx), 1u);
    CHECK_EQ(device.read_sfr(u.dbuf), data);
    run_to(device, c + bit_end(frame_bits(ucr) + 4));
    CHECK_EQ(device.read_sfr(u.csr), CSR_UART_RX | CSR_RX_BYTE | flags);
    CHECK_EQ(Access::usart_dma(device, index, u.dma_rx), 1u);
}

// 17.1.4: "If parity is enabled, then parity is checked ... ERR is set if
// the parity is wrong". Without PARITY the 9th bit is data and never
// checked.
void test_rx_parity() {
    const std::vector<std::uint8_t> bytes{0x00, 0xff, 0x01, 0x80, 0xa7, 0x5a, 0x3c};
    for (const std::uint8_t ucr : {0x3a, 0x1a}) {
        const bool even = ucr & UCR_D9;
        for (const std::uint8_t byte : bytes) {
            const std::string name = std::string(even ? "even " : "odd ") + hex(byte);
            const bool p = parity_bit(byte, even);
            check_rx(0, ucr, 11, Frame{.data = byte}, byte, 0, name + " plain");
            check_rx(0, ucr, 11, Frame{.data = byte, .ninth = Ninth::BadParity}, byte, CSR_ERR,
                     name + " perr");
            check_rx(0, ucr, 11, Frame{.data = byte, .ninth = p ? Ninth::One : Ninth::Zero}, byte,
                     0, name + " right 9th");
            check_rx(0, ucr, 11, Frame{.data = byte, .ninth = p ? Ninth::Zero : Ninth::One}, byte,
                     CSR_ERR, name + " wrong 9th");
        }
    }
    // BIT9 without PARITY: the 9th bit is data, never an error.
    for (const std::uint8_t ucr : {0x12, 0x32})
        for (const Ninth ninth : {Ninth::Expected, Ninth::Zero, Ninth::One, Ninth::BadParity})
            check_rx(0, ucr, 11, Frame{.data = 0xa7, .ninth = ninth}, 0xa7, 0,
                     "9-bit data ucr " + hex(ucr) + " ninth " +
                         std::to_string(static_cast<int>(ninth)));
    // Model-defined: "perr" means the inverse of the expected 9th bit; a
    // receiver without a 9th bit (BIT9 = 0, also with PARITY alone) expects
    // none, so the frame is plain.
    check_rx(0, 0x0a, 11, Frame{.data = 0xa7, .ninth = Ninth::BadParity}, 0xa7, 0,
             "parity without BIT9");
    check_rx(0, 0x02, 11, Frame{.data = 0xa7, .ninth = Ninth::BadParity}, 0xa7, 0,
             "perr at 8-bit receiver");
    // Model-defined bit order: UxDBUF is the wire byte bit-reversed with
    // ORDER = 1; the parity counts the same ones.
    check_rx(0, 0x3a, 0x20 | 11, Frame{.data = 0x01, .ninth = Ninth::One}, 0x80, 0,
             "order=1 right parity");
    check_rx(0, 0x3a, 0x20 | 11, Frame{.data = 0x01, .ninth = Ninth::Zero}, 0x80, CSR_ERR,
             "order=1 wrong parity");
    check_rx(1, 0x1a, 11, Frame{.data = 0x5a, .ninth = Ninth::BadParity}, 0x5a, CSR_ERR,
             "usart1 perr");
}

// UxCSR.FE: "byte received with incorrect stop-bit level"; the stop-bit
// level is UxUCR.STOP. The first stop bit is always checked.
void test_rx_framing() {
    check_rx(0, 0x02, 11, Frame{.data = 0x41, .stop1_error = true}, 0x41, CSR_FE, "8n1 fe");
    check_rx(0, 0x01, 11, Frame{.data = 0x41}, 0x41, 0, "start=1 stop=0 plain");
    check_rx(0, 0x01, 11, Frame{.data = 0x41, .stop1_error = true}, 0x41, CSR_FE,
             "start=1 stop=0 fe");
    check_rx(0, 0x06, 11, Frame{.data = 0x41, .stop1_error = true}, 0x41, CSR_FE, "spb fe");
    check_rx(0, 0x3a, 11, Frame{.data = 0x41, .stop1_error = true}, 0x41, CSR_FE, "9-bit fe");
    check_rx(0, 0x3a, 11, Frame{.data = 0x41, .ninth = Ninth::BadParity, .stop1_error = true},
             0x41, CSR_FE | CSR_ERR, "9-bit fe perr");
    // Only one stop bit with SPB = 0: a "second stop bit" error is idle line.
    check_rx(0, 0x02, 11, Frame{.data = 0x41, .stop2_error = true}, 0x41, 0, "fe2 with spb=0");
    check_rx(1, 0x01, 11, Frame{.data = 0x7e, .stop1_error = true}, 0x7e, CSR_FE, "usart1 fe");
}

// 17.1.4: "The receiver always checks for one stop bit ... the framing
// error ... checks both stop bits when UxUCR.SPB is set. The RX interrupt
// is set when the first stop bit is checked OK. If second stop bit is not
// OK, there is a delay in setting the framing error bit, dependent on baud
// rate (bit duration)": the byte completes at the end of the first stop
// bit, FE follows at the end of the second.
void check_rx_late_fe(std::uint8_t ucr, const Frame &frame, std::uint8_t flags,
                      const std::string &name) {
    Case scope{name};
    Cc2530F256 device;
    const std::uint64_t c = uart_device(device, 0, ucr);
    device.uart_receive_frame(0, frame);
    const std::uint64_t done = c + bit_end(first_stop_end(ucr));
    const std::uint64_t end = c + bit_end(frame_bits(ucr));
    run_to(device, done - 1);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_ACTIVE);
    run_to(device, done);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | flags);
    CHECK_EQ(device.read_sfr(TCON) & TCON_URX0IF, TCON_URX0IF);
    CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 1u);
    CHECK_EQ(device.read_sfr(U0DBUF), frame.data);
    run_to(device, end - 1);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | flags);
    run_to(device, end);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | flags | CSR_FE);
    run_to(device, end + 2000);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | flags | CSR_FE);
    CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 1u);
}

void test_rx_second_stop_bit() {
    check_rx_late_fe(0x06, Frame{.data = 0x41, .stop2_error = true}, 0, "spb stop=1 fe2");
    check_rx_late_fe(0x05, Frame{.data = 0x41, .stop2_error = true}, 0, "spb stop=0 fe2");
    check_rx_late_fe(0x16, Frame{.data = 0x41, .ninth = Ninth::One, .stop2_error = true}, 0,
                     "9-bit spb fe2");
    check_rx_late_fe(0x3e, Frame{.data = 0x41, .ninth = Ninth::BadParity, .stop2_error = true},
                     CSR_ERR, "even spb perr fe2");
    {
        // Each bad stop bit sets FE at its check: FE cleared by firmware
        // after the first one is set again by the second.
        Case scope{"fe then fe2"};
        Cc2530F256 device;
        const std::uint64_t c = uart_device(device, 0, 0x06);
        device.uart_receive_frame(0, Frame{.data = 0x41, .stop1_error = true, .stop2_error = true});
        run_to(device, c + bit_end(10));
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_FE);
        device.write_sfr(U0CSR, CSR_UART_RX | CSR_RX_BYTE); // R/W0: FE written 0
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE);
        run_to(device, c + bit_end(11));
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_FE);
    }
}

// Model-defined: a 9th bit sent to a receiver with BIT9 = 0 is sampled as
// its first stop bit and the sender's first stop bit as its second one.
void test_ninth_bit_into_8bit_receiver() {
    check_rx(0, 0x02, 11, Frame{.data = 0x41, .ninth = Ninth::Zero}, 0x41, CSR_FE, "9=0 at 8n1");
    check_rx(0, 0x02, 11, Frame{.data = 0x41, .ninth = Ninth::One}, 0x41, 0, "9=1 at 8n1");
    check_rx(0, 0x01, 11, Frame{.data = 0x41, .ninth = Ninth::One}, 0x41, CSR_FE,
             "9=1 at stop=0");
    check_rx(0, 0x01, 11, Frame{.data = 0x41, .ninth = Ninth::Zero}, 0x41, 0, "9=0 at stop=0");
    check_rx(0, 0x06, 11, Frame{.data = 0x41, .ninth = Ninth::Zero}, 0x41, CSR_FE,
             "9=0 at 8n2");
    check_rx_late_fe(0x06, Frame{.data = 0x41, .ninth = Ninth::One, .stop1_error = true}, 0,
                     "9=1 fe at 8n2");
}

// Model-defined break (SWRU191F describes none): the line stays at the
// start-bit level for one frame, so data, 9th and stop bits are at the
// UxUCR.START level.
void test_rx_break() {
    const Frame brk{.line_break = true};
    check_rx(0, 0x02, 11, brk, 0x00, CSR_FE, "break 8n1");
    check_rx(0, 0x3a, 11, brk, 0x00, CSR_FE, "break even (parity of 0x00 is 0)");
    check_rx(0, 0x1a, 11, brk, 0x00, CSR_FE | CSR_ERR, "break odd");
    check_rx(0, 0x01, 11, brk, 0xff, CSR_FE, "break start=1");
    check_rx(0, 0x01, 0x20 | 11, brk, 0xff, CSR_FE, "break start=1 order=1");
    // The fields besides line_break are ignored.
    check_rx(0, 0x02, 11, Frame{.data = 0x55, .ninth = Ninth::One, .line_break = true}, 0x00,
             CSR_FE, "break ignores data");
    {
        Case scope{"break 8n2"};
        Cc2530F256 device;
        const std::uint64_t c = uart_device(device, 0, 0x06);
        device.uart_receive_frame(0, brk);
        run_to(device, c + bit_end(10));
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_FE);
        device.write_sfr(U0CSR, CSR_UART_RX | CSR_RX_BYTE);
        run_to(device, c + bit_end(11));
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_FE);
    }
}

// Errored frames take exactly the time of plain frames back to back (the
// frame time is the receiver's), and each sets its flags as it completes.
// RX_BYTE, FE and ERR are cleared by an R/W0 write after each byte so the
// next one is delivered (17.8: writing 0 "effectively discards the data").
void test_rx_back_to_back() {
    for (const std::uint8_t ucr : {0x02, 0x06, 0x3a, 0x3e}) {
        Case scope{"back to back ucr " + hex(ucr)};
        const bool spb = ucr & UCR_SPB;
        const std::uint8_t perr = (ucr & UCR_PARITY) ? CSR_ERR : 0;
        // 0x12 has two ones: its even parity bit (D9 = 1 here) is 0. At an
        // 8-bit receiver a 9th bit of 1 is a good first stop bit (STOP = 1).
        // The break leaves data and 9th bit at 0 (START = 0): even parity
        // holds.
        const std::vector<std::pair<Frame, std::uint8_t>> frames{
            {Frame{.data = 0x10}, 0},
            {Frame{.data = 0x11, .stop1_error = true}, CSR_FE},
            {Frame{.data = 0x12, .ninth = Ninth::One}, perr},
            {Frame{.line_break = true}, CSR_FE},
            {Frame{.data = 0x14, .ninth = Ninth::BadParity}, perr},
            {Frame{.data = 0x15}, 0}};
        Cc2530F256 device;
        const std::uint64_t c = uart_device(device, 0, ucr);
        for (const auto &entry : frames)
            device.uart_receive_frame(0, entry.first);
        const unsigned bits = frame_bits(ucr);
        for (unsigned k = 0; k < frames.size(); ++k) {
            // With SPB the break's second stop bit (at the START level) sets
            // FE again at its end, before the next byte.
            const std::uint8_t late = (k > 0 && spb && frames[k - 1].first.line_break) ? CSR_FE : 0;
            const std::uint64_t done = c + bit_end(k * bits + first_stop_end(ucr));
            run_to(device, done - 1);
            CHECK_EQ(device.read_sfr(U0CSR) & (CSR_RX_BYTE | CSR_FE | CSR_ERR), late);
            CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), k);
            run_to(device, done);
            const Frame &frame = frames[k].first;
            CHECK_EQ(device.read_sfr(U0CSR) & (CSR_RX_BYTE | CSR_FE | CSR_ERR),
                     CSR_RX_BYTE | late | frames[k].second);
            CHECK_EQ(device.read_sfr(U0DBUF), frame.line_break ? 0x00 : frame.data);
            CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), k + 1);
            device.write_sfr(U0CSR, CSR_UART_RX);
        }
    }
}

// Overrun (model-defined: the byte waits in the shift register while
// RX_BYTE = 1): an errored byte still sets its flags when it completes,
// without URXxIF or a DMA trigger; FLUSH and RE = 0 drop the frame or the
// pending second stop-bit check.
void test_rx_overrun_flush_disable() {
    {
        Case scope{"overrun"};
        Cc2530F256 device;
        const std::uint64_t c = uart_device(device, 0, 0x3a);
        device.uart_receive_frame(0, Frame{.data = 0x11});
        device.uart_receive_frame(0, Frame{.data = 0x22, .ninth = Ninth::BadParity});
        run_to(device, c + bit_end(11));
        CHECK_EQ(device.read_sfr(U0CSR) & ~CSR_ACTIVE, CSR_UART_RX | CSR_RX_BYTE);
        device.write_sfr(TCON, 0x00);
        run_to(device, c + bit_end(22) - 1);
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_ACTIVE);
        run_to(device, c + bit_end(22));
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_ERR);
        CHECK_EQ(device.read_sfr(U0DBUF), 0x11);
        CHECK_EQ(device.read_sfr(TCON) & TCON_URX0IF, 0);
        CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 1u);
    }
    {
        // FLUSH (17.5) during an errored frame: it is discarded without
        // flags; the next host frame follows after its frame time.
        Case scope{"flush discards"};
        Cc2530F256 device;
        const std::uint64_t c = uart_device(device, 0, 0x3a);
        device.uart_receive_frame(
            0, Frame{.data = 0x22, .ninth = Ninth::BadParity, .stop1_error = true});
        device.uart_receive_frame(0, Frame{.data = 0x33});
        run_to(device, c + 1000);
        device.write_sfr(U0UCR, UCR_FLUSH | 0x3a);
        run_to(device, c + bit_end(22) - 1);
        CHECK_EQ(device.read_sfr(U0CSR) & ~CSR_ACTIVE, CSR_UART_RX);
        CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 0u);
        run_to(device, c + bit_end(22));
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE);
        CHECK_EQ(device.read_sfr(U0DBUF), 0x33);
    }
    for (const std::uint8_t cancel : {0, 1, 2}) {
        // FLUSH, RE = 0 or leaving UART mode between the byte and its bad
        // second stop bit drop the late FE.
        Case scope{"late fe cancelled " + std::to_string(cancel)};
        Cc2530F256 device;
        const std::uint64_t c = uart_device(device, 0, 0x06);
        device.uart_receive_frame(0, Frame{.data = 0x41, .stop2_error = true});
        run_to(device, c + bit_end(10) + 10);
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE);
        if (cancel == 0)
            device.write_sfr(U0UCR, UCR_FLUSH | 0x06);
        else if (cancel == 1)
            device.write_sfr(U0CSR, 0x80 | CSR_RX_BYTE);
        else
            device.write_sfr(U0CSR, 0x40 | CSR_RX_BYTE);
        run_to(device, c + bit_end(11) + 2000);
        CHECK_EQ(device.read_sfr(U0CSR) & CSR_FE, 0);
        CHECK_EQ(device.read_sfr(U0CSR) & CSR_RX_BYTE, CSR_RX_BYTE);
    }
    {
        // UxUCR is latched at the start bit: a change during the frame does
        // not change its length or its check.
        Case scope{"format latched"};
        Cc2530F256 device;
        const std::uint64_t c = uart_device(device, 0, 0x3a);
        device.uart_receive_frame(0, Frame{.data = 0x22, .ninth = Ninth::BadParity});
        run_to(device, c + 1000);
        device.write_sfr(U0UCR, 0x02);
        run_to(device, c + bit_end(11) - 1);
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_ACTIVE);
        run_to(device, c + bit_end(11));
        CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_ERR);
    }
}

// UxCSR.FE/ERR are R/W0 (17.8): writing 0 clears them, writing 1 keeps
// them and cannot set them.
void test_rw0_writes() {
    Cc2530F256 device;
    const std::uint64_t c = uart_device(device, 0, 0x3a);
    device.uart_receive_frame(0, Frame{.data = 0x22, .ninth = Ninth::BadParity, .stop1_error = true});
    run_to(device, c + bit_end(11));
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_FE | CSR_ERR);
    device.write_sfr(U0CSR, CSR_UART_RX | CSR_RX_BYTE | CSR_FE);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_FE);
    device.write_sfr(U0CSR, CSR_UART_RX | CSR_RX_BYTE | CSR_FE | CSR_ERR);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_FE);
    device.write_sfr(U0CSR, CSR_UART_RX | CSR_RX_BYTE);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE);
    device.write_sfr(U0CSR, CSR_UART_RX | CSR_RX_BYTE | CSR_FE | CSR_ERR);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE);
}

// Configures USART `index` (UART, 115200, `ucr`, RE = 1), waits until the
// host sets idata 0x50, runs `body` and stops in SJMP $.
Flash polling_program(unsigned index, std::uint8_t ucr, const std::vector<std::uint8_t> &body) {
    const Usart &u = Usarts[index];
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80, 0x75, u.baud, 216, 0x75, u.gcr, 11, 0x75, u.ucr,
                         ucr, 0x75, u.csr, CSR_UART_RX,
                         0xe5, 0x50, 0x60, 0xfc}); // wait: MOV A,0x50; JZ wait
    flash.op(body).op({0x80, 0xfe});
    return flash;
}

std::uint64_t start_polling(Cc2530F256 &device, const Flash &flash) {
    load(device, flash);
    device.run(1000, 20);
    return cycles(device);
}

void release(Cc2530F256 &device) {
    device.write_sram(Cc2530F256::IdataXdataAlias + 0x50, 1);
    run_to(device, cycles(device) + 200);
}

// FE and ERR clear on a CPU read of UxCSR (17.8: "cleared on a read of the
// UxCSR register"). An errored byte waiting during an overrun moves into
// UxDBUF when UxDBUF is read; its flags were set when it completed.
void test_cpu_read_clears() {
    Case scope{"cpu read"};
    const Flash flash = polling_program(0, 0x3a,
                                        {0x85, U0CSR, 0x30,   // MOV 0x30,U0CSR
                                         0x85, U0CSR, 0x31,   // MOV 0x31,U0CSR
                                         0x85, U0DBUF, 0x32,  // MOV 0x32,U0DBUF
                                         0xe5, U0CSR, 0xf5, 0x33, // MOV A,U0CSR; MOV 0x33,A
                                         0x85, U0DBUF, 0x34,  // MOV 0x34,U0DBUF
                                         0x85, U0CSR, 0x35}); // MOV 0x35,U0CSR
    Cc2530F256 device;
    const std::uint64_t c = start_polling(device, flash);
    device.uart_receive_frame(0, Frame{.data = 0x11});
    device.uart_receive_frame(0, Frame{.data = 0x22, .ninth = Ninth::BadParity});
    device.uart_receive_frame(0, Frame{.data = 0x33, .stop1_error = true});
    run_to(device, c + bit_end(33) + 10);
    CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_FE | CSR_ERR);
    release(device);
    CHECK_EQ(device.read_idata(0x30), CSR_UART_RX | CSR_RX_BYTE | CSR_FE | CSR_ERR);
    CHECK_EQ(device.read_idata(0x31), CSR_UART_RX | CSR_RX_BYTE);
    CHECK_EQ(device.read_idata(0x32), 0x11);
    CHECK_EQ(device.read_idata(0x33), CSR_UART_RX | CSR_RX_BYTE); // 0x33 moved in
    CHECK_EQ(device.read_idata(0x34), 0x33);
    CHECK_EQ(device.read_idata(0x35), CSR_UART_RX);
    CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 2u);
}

// "Cleared on a read of the UxCSR register or bits in the UxCSR register"
// (17.8): a bit read (MOV C,bit) of U1CSR reads the register, so it clears
// FE and ERR (model-defined: both, as for a byte read; the 8051 core reads
// the whole SFR byte for a bit access).
void test_cpu_bit_read_clears() {
    Case scope{"cpu bit read"};
    const Flash flash = polling_program(1, 0x3a,
                                        {0xa2, BIT_U1CSR_FE,  // MOV C,U1CSR.4 (FE)
                                         0x92, 0x00,          // MOV 0x20.0,C
                                         0xa2, BIT_U1CSR_ERR, // MOV C,U1CSR.3 (ERR)
                                         0x92, 0x01,          // MOV 0x20.1,C
                                         0x85, U1CSR, 0x31}); // MOV 0x31,U1CSR
    Cc2530F256 device;
    const std::uint64_t c = start_polling(device, flash);
    device.uart_receive_frame(1, Frame{.data = 0x22, .ninth = Ninth::BadParity, .stop1_error = true});
    run_to(device, c + bit_end(11) + 10);
    CHECK_EQ(device.read_sfr(U1CSR), CSR_UART_RX | CSR_RX_BYTE | CSR_FE | CSR_ERR);
    release(device);
    CHECK_EQ(device.read_idata(0x20) & 0x03, 0x01);
    CHECK_EQ(device.read_idata(0x31), CSR_UART_RX | CSR_RX_BYTE);
}

// A read-modify-write of UxCSR reads it first (model-defined, as above):
// ANL U0CSR,#~RX_BYTE clears FE and ERR together with RX_BYTE.
void test_cpu_rmw_clears() {
    Case scope{"cpu rmw"};
    const Flash flash = polling_program(0, 0x3a,
                                        {0x53, U0CSR, 0xfb,   // ANL U0CSR,#0xfb
                                         0x85, U0CSR, 0x31}); // MOV 0x31,U0CSR
    Cc2530F256 device;
    const std::uint64_t c = start_polling(device, flash);
    device.uart_receive_frame(0, Frame{.data = 0x22, .ninth = Ninth::BadParity, .stop1_error = true});
    run_to(device, c + bit_end(11) + 10);
    release(device);
    CHECK_EQ(device.read_idata(0x31), CSR_UART_RX);
}

// Plain frames (only `data` set) are exactly host bytes in every format:
// same state at every point, no FE or ERR. So is a 9th bit that equals the
// expected one.
void test_plain_frames_unchanged() {
    const std::vector<std::uint8_t> bytes{0x00, 0xff, 0x41, 0xa7, 0x5a};
    for (const std::uint8_t ucr : {0x02, 0x06, 0x12, 0x32, 0x1a, 0x3a, 0x3e, 0x01, 0x05})
        for (const std::uint8_t gcr : {std::uint8_t{11}, std::uint8_t{0x20 | 11}})
            for (const bool explicit_ninth : {false, true}) {
                Case scope{"plain ucr " + hex(ucr) + " gcr " + hex(gcr) +
                           (explicit_ninth ? " explicit 9th" : "")};
                Cc2530F256 bytes_device, frames_device;
                const std::uint64_t c = uart_device(bytes_device, 0, ucr, gcr);
                CHECK_EQ(uart_device(frames_device, 0, ucr, gcr), c);
                for (const std::uint8_t byte : bytes) {
                    bytes_device.uart_receive(0, byte);
                    Frame frame{.data = byte};
                    if (explicit_ninth && (ucr & UCR_BIT9)) {
                        const bool ninth = (ucr & UCR_PARITY) ? parity_bit(byte, ucr & UCR_D9)
                                                              : (ucr & UCR_D9) != 0;
                        frame.ninth = ninth ? Ninth::One : Ninth::Zero;
                    }
                    frames_device.uart_receive_frame(0, frame);
                }
                const unsigned bits = frame_bits(ucr);
                for (unsigned k = 0; k < bytes.size(); ++k)
                    for (const std::uint64_t at :
                         {c + bit_end(k * bits + first_stop_end(ucr)) - 1,
                          c + bit_end(k * bits + first_stop_end(ucr)),
                          c + bit_end((k + 1) * bits)}) {
                        run_to(bytes_device, at);
                        run_to(frames_device, at);
                        CHECK(snapshot(bytes_device) == snapshot(frames_device));
                        CHECK_EQ(frames_device.read_sfr(U0CSR) & (CSR_FE | CSR_ERR), 0);
                    }
            }
}

// End to end in fast, step and trace modes: the URX0 ISR (vector 0x13)
// stores UxCSR (clearing FE/ERR) and UxDBUF for each byte. Even parity, two
// stop bits: the late FE of 'D' is seen when 'E' arrives, and the break's
// late FE stays set.
void test_modes_isr() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x0013).op({0xa6, U0CSR, 0x08, 0xa6, U0DBUF, 0x08, 0x32});
    flash.at(0x0100).op({0x75, SP, 0x70, 0x78, 0x30, 0x75, IEN0, 0x84}).op(IdleLoop);
    constexpr std::uint8_t ucr = 0x3e;
    for_each_mode(
        flash, bit_end(6 * 12) + 1000,
        [](Cc2530F256 &device) {
            device.write_sfr(CLKCONCMD, 0x80);
            device.write_sfr(U0BAUD, 216);
            device.write_sfr(U0GCR, 11);
            device.write_sfr(U0UCR, ucr);
            device.write_sfr(U0CSR, CSR_UART_RX);
            device.uart_receive_frame(0, Frame{.data = 'A'});
            device.uart_receive_frame(0, Frame{.data = 'B', .ninth = Ninth::BadParity});
            device.uart_receive_frame(0, Frame{.data = 'C', .stop1_error = true});
            device.uart_receive_frame(0, Frame{.data = 'D', .stop2_error = true});
            device.uart_receive_frame(0, Frame{.data = 'E'});
            device.uart_receive_frame(0, Frame{.line_break = true});
        },
        [](Cc2530F256 &device, Mode mode) {
            Case scope{std::string("isr modes ") + mode_name(mode)};
            const std::uint8_t want[] = {0xc4, 'A', 0xcc, 'B', 0xd4, 'C',
                                         0xc4, 'D', 0xd4, 'E', 0xd4, 0x00};
            for (unsigned i = 0; i < sizeof want; ++i) {
                const std::uint8_t got = device.read_idata(static_cast<std::uint8_t>(0x30 + i));
                CHECK_EQ(i % 2 == 0 ? got & ~CSR_ACTIVE : got, want[i]);
            }
            CHECK_EQ(device.read_idata(0x00), 0x3c); // R0
            CHECK_EQ(device.read_sfr(U0CSR), CSR_UART_RX | CSR_FE);
            CHECK_EQ(device.read_sfr(TCON) & TCON_URX0IF, 0);
            CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 6u);
        },
        "isr modes");
}

} // namespace

int main() {
    test_tx_ninth_bit();
    test_tx_order_and_latching();
    test_rx_parity();
    test_rx_framing();
    test_rx_second_stop_bit();
    test_ninth_bit_into_8bit_receiver();
    test_rx_break();
    test_rx_back_to_back();
    test_rx_overrun_flush_disable();
    test_rw0_writes();
    test_cpu_read_clears();
    test_cpu_bit_read_clears();
    test_cpu_rmw_clears();
    test_plain_frames_unchanged();
    test_modes_isr();
    return finish("cc2530 uart errors");
}
