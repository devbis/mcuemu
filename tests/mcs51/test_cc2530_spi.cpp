// CC2530F256 USART SPI master and slave at pad level. Expectations come
// from SWRU191F: 17.2 (SPI mode), 17.2.1 (master), 17.2.2 (slave), 17.3
// (SSN), 17.4 (equation 3), 17.5 (flush), 17.6/17.7 (interrupts, DMA
// triggers), the UxCSR/UxGCR/UxDBUF register descriptions, Table 7-1 (pads)
// and P2DIR.PRIP0 (7.6.4). Rules SWRU191F leaves open are the ones
// cc2530_usart.cpp documents as "Model-defined"; checks relying on them say so.
//
// UxGCR: CPHA = 0 outputs data when SCK goes from CPOL inverted to CPOL
// (trailing edge) and samples when SCK goes from CPOL to CPOL inverted
// (leading edge); CPHA = 1 the reverse. ORDER = 1: MSB first.
//
// Pads: USART1 alternative 1 (PERCFG.U1CFG = 0, Table 7-1): MI P0_5,
// MO P0_4, C P0_3, SS P0_2. P2DIR.PRIP0 = 01 gives USART1 port 0 priority
// over USART0 (whose alternative 1 uses the same pins).
//
// Timing: 32 MHz XOSC; virtual time counts 32 MHz cycles. With BAUD_M = 0,
// BAUD_E = 17 equation 3 gives 256 * 2^17 / 2^28 * 32 MHz = 4 MHz: one SCK
// period (bit) is 8 cycles, one half period 4 cycles. The CPU waits in Idle
// so that run() stops exactly at a requested cycle.

#include "cc2530_periph_support.hpp"

#include "cc2530_soc.hpp"
#include "cc2530_usart.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

namespace mcs51emu::cc2530 {
struct SpiTestAccess {
    static std::uint64_t usart_dma(Cc2530F256 &device, unsigned index, unsigned trigger) {
        return device.peripherals().usart[index]->dma_triggers_[trigger];
    }
};
} // namespace mcs51emu::cc2530

namespace {

using namespace cc2530test;
using Access = mcs51emu::cc2530::SpiTestAccess;
using Drive = mcs51emu::Cc2530PinDrive;

// SFRs (SWRU191F Table 2-1).
constexpr std::uint8_t PCON = 0x87, TCON = 0x88, CLKCONSTA = 0x9e, CLKCONCMD = 0xc6,
                       IRCON2 = 0xe8, PERCFG = 0xf1, P0SEL = 0xf3, U1CSR = 0xf8, U1DBUF = 0xf9,
                       U1BAUD = 0xfa, U1UCR = 0xfb, U1GCR = 0xfc, P2DIR = 0xff;
// Flag masks (2.5): TCON.URX1IF, IRCON2.UTX1IF. UxCSR bits.
constexpr std::uint8_t TCON_URX1IF = 0x80, IRCON2_UTX1IF = 0x04;
constexpr std::uint8_t CSR_SLAVE = 0x20, CSR_RX_BYTE = 0x04, CSR_TX_BYTE = 0x02, CSR_ACTIVE = 0x01;
// DMA triggers (Table 8-1).
constexpr unsigned DMA_URX1 = 16, DMA_UTX1 = 17;
// USART1 alternative 1 pads (Table 7-1), port 0.
constexpr unsigned MI = 5, MO = 4, C = 3, SS = 2;

// MOV PCON,#1; SJMP back: Idle (SLEEPCMD.MODE = 00 at reset).
const std::vector<std::uint8_t> IdleLoop = {0x75, PCON, 0x01, 0x80, 0xfb};

std::uint64_t cycles(const Cc2530F256 &device) { return device.registers().cycles; }

void run_to(Cc2530F256 &device, std::uint64_t cycle) {
    if (cycles(device) >= cycle)
        return;
    const Cc2530RunResult result = device.run(10'000'000, cycle);
    CHECK(result.error.empty());
    CHECK_EQ(cycles(device), cycle);
}

std::optional<bool> level_of(Drive drive) {
    if (drive == Drive::High)
        return true;
    if (drive == Drive::Low)
        return false;
    return std::nullopt;
}

std::uint8_t wire_mask(unsigned k, bool msb_first) {
    return static_cast<std::uint8_t>(msb_first ? 0x80u >> k : 1u << k);
}

std::uint8_t gcr_of(bool cpol, bool cpha, bool msb_first, std::uint8_t baud_e) {
    return static_cast<std::uint8_t>((cpol ? 0x80 : 0) | (cpha ? 0x40 : 0) | (msb_first ? 0x20 : 0) |
                                     baud_e);
}

// 32 MHz XOSC, CPU in Idle, USART1 on alternative 1 with port 0 priority.
void idle_device(Cc2530F256 &device) {
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80}).op(IdleLoop);
    load(device, flash);
    device.run(100);
    CHECK(device.sleeping());
    CHECK_EQ(device.read_sfr(CLKCONSTA) & 0x40, 0); // XOSC
    device.write_sfr(PERCFG, 0x00);
    device.write_sfr(P2DIR, 0x40); // PRIP0 = 01
}

// SPI master (UxCSR.MODE = 0, SLAVE = 0) with C, MO, MI as peripheral pads.
void master_config(Cc2530F256 &device, std::uint8_t baud_m, std::uint8_t gcr) {
    device.write_sfr(U1BAUD, baud_m);
    device.write_sfr(U1GCR, gcr);
    device.write_sfr(U1CSR, 0x00);
    device.write_sfr(P0SEL, 0x38);
}

// The net of the emut firmware: MO (P0_4) wired to MI (P0_5).
void loop_mosi_to_miso(Cc2530F256 &device) {
    device.set_pin_listener([&device](unsigned port, unsigned pin, Drive drive, std::uint64_t) {
        if (port == 0 && pin == MO)
            device.set_pin_input(0, MI, level_of(drive));
    });
    device.set_pin_input(0, MI, level_of(device.pin_drive(0, MO)));
}

struct PadEvent {
    unsigned pin;
    Drive drive;
    std::uint64_t cycle;
    bool operator==(const PadEvent &other) const {
        return pin == other.pin && drive == other.drive && cycle == other.cycle;
    }
    bool operator<(const PadEvent &other) const {
        return std::tie(cycle, pin) < std::tie(other.cycle, other.pin);
    }
};

void check_events(std::vector<PadEvent> actual, std::vector<PadEvent> expected, const char *name) {
    std::sort(actual.begin(), actual.end());
    std::sort(expected.begin(), expected.end());
    if (actual == expected)
        return;
    std::cerr << name << ": pad events differ\n  got:";
    for (const PadEvent &e : actual)
        std::cerr << " P0_" << e.pin << '=' << static_cast<int>(e.drive) << '@' << e.cycle;
    std::cerr << "\n  want:";
    for (const PadEvent &e : expected)
        std::cerr << " P0_" << e.pin << '=' << static_cast<int>(e.drive) << '@' << e.cycle;
    std::cerr << '\n';
    ++failures;
}

// Pad events of one master byte started at t0, derived from 17.2.1, 17.4
// and UxGCR: SCK edge j (0-15) is half a bit period after edge j - 1, the
// first half a bit after the start: t0 + ceil((j + 1) * T / 2), T = one bit
// in 32 MHz cycles = 2^28 / ((256 + BAUD_M) * 2^BAUD_E) (equation 3;
// rounding up to the next cycle is the documented model rule). Even edges
// are leading (CPOL -> CPOL inverted). CPHA = 1: bit k goes out on leading
// edge k; CPHA = 0: bit k + 1 on trailing edge k and bit 0 before the first
// (sampling) edge, i.e. at t0 (model-defined instant). Only changes are
// reported.
std::vector<PadEvent> master_events(std::uint8_t byte, std::uint8_t gcr, std::uint8_t baud_m,
                                    std::uint64_t t0, bool mosi_before) {
    const bool cpol = gcr & 0x80, cpha = gcr & 0x40, msb = gcr & 0x20;
    const std::uint64_t num = std::uint64_t{1} << 28, den = std::uint64_t{256u + baud_m} << (gcr & 0x1f);
    std::vector<PadEvent> events;
    bool mosi = mosi_before;
    const auto put_mosi = [&](unsigned k, std::uint64_t at) {
        const bool level = (byte & wire_mask(k, msb)) != 0;
        if (level != mosi)
            events.push_back({MO, level ? Drive::High : Drive::Low, at});
        mosi = level;
    };
    if (!cpha)
        put_mosi(0, t0);
    for (unsigned j = 0; j < 16; ++j) {
        const std::uint64_t at = t0 + ((j + 1) * num + 2 * den - 1) / (2 * den);
        const bool leading = (j % 2) == 0;
        const bool sck = leading ? !cpol : cpol;
        events.push_back({C, sck ? Drive::High : Drive::Low, at});
        if (cpha && leading)
            put_mosi(j / 2, at);
        if (!cpha && !leading && j / 2 + 1 < 8)
            put_mosi(j / 2 + 1, at);
    }
    return events;
}

// ===================================================== SPI master (17.2.1)

// Explicit edge list for CPOL = 0, CPHA = 0, MSB first, 0xa5 = 1010 0101.
void test_master_edge_times_explicit() {
    Cc2530F256 device;
    idle_device(device);
    master_config(device, 0, gcr_of(false, false, true, 17));
    CHECK(device.pin_drive(0, C) == Drive::Low);  // SCK idles at CPOL
    CHECK(device.pin_drive(0, MO) == Drive::Low); // model-defined: MOSI low after reset
    CHECK(device.pin_drive(0, MI) != Drive::Low && device.pin_drive(0, MI) != Drive::High);
    std::vector<PadEvent> events;
    device.set_pin_listener([&](unsigned port, unsigned pin, Drive drive, std::uint64_t at) {
        if (port == 0)
            events.push_back({pin, drive, at});
    });
    const std::uint64_t t = cycles(device);
    device.write_sfr(U1DBUF, 0xa5);
    run_to(device, t + 100);
    const Drive H = Drive::High, L = Drive::Low;
    check_events(events,
                 {{MO, H, t},                                    // bit 7 = 1 before edge 0
                  {C, H, t + 4},  {C, L, t + 8},  {MO, L, t + 8},  // bit 6 = 0
                  {C, H, t + 12}, {C, L, t + 16}, {MO, H, t + 16}, // bit 5 = 1
                  {C, H, t + 20}, {C, L, t + 24}, {MO, L, t + 24}, // bit 4 = 0
                  {C, H, t + 28}, {C, L, t + 32},                  // bit 3 = 0
                  {C, H, t + 36}, {C, L, t + 40}, {MO, H, t + 40}, // bit 2 = 1
                  {C, H, t + 44}, {C, L, t + 48}, {MO, L, t + 48}, // bit 1 = 0
                  {C, H, t + 52}, {C, L, t + 56}, {MO, H, t + 56}, // bit 0 = 1
                  {C, H, t + 60}, {C, L, t + 64}},
                 "explicit edges");
    CHECK(device.pin_drive(0, C) == Drive::Low);
}

// Edge times for every CPOL/CPHA/ORDER at an integer rate (4 MHz) and at a
// non-integer one (BAUD_M = 216, BAUD_E = 11: T = 2^17 / 472 = 277.69
// cycles, last edge = byte end at ceil(8 T) = 2222 cycles).
void test_master_edge_times_all_modes() {
    for (const auto &[baud_m, baud_e] : {std::pair<std::uint8_t, std::uint8_t>{0, 17}, {216, 11}}) {
        for (unsigned mode = 0; mode < 8; ++mode) {
            const std::uint8_t gcr = gcr_of(mode & 4, mode & 2, mode & 1, baud_e);
            Cc2530F256 device;
            idle_device(device);
            master_config(device, baud_m, gcr);
            CHECK(device.pin_drive(0, C) == ((gcr & 0x80) ? Drive::High : Drive::Low));
            std::vector<PadEvent> events;
            device.set_pin_listener([&](unsigned port, unsigned pin, Drive drive, std::uint64_t at) {
                if (port == 0)
                    events.push_back({pin, drive, at});
            });
            for (const std::uint8_t byte : {std::uint8_t{0x96}, std::uint8_t{0x0f}}) {
                const bool mosi_before = device.pin_drive(0, MO) == Drive::High;
                events.clear();
                const std::uint64_t t = cycles(device);
                device.write_sfr(U1DBUF, byte);
                const std::uint64_t end = baud_e == 17 ? t + 64 : t + 2222;
                run_to(device, end - 1);
                CHECK_EQ(device.read_sfr(TCON) & TCON_URX1IF, 0);
                CHECK_EQ(device.read_sfr(U1CSR) & (CSR_ACTIVE | CSR_TX_BYTE), CSR_ACTIVE);
                run_to(device, end);
                CHECK_EQ(device.read_sfr(TCON) & TCON_URX1IF, TCON_URX1IF);
                CHECK_EQ(device.read_sfr(U1CSR) & (CSR_ACTIVE | CSR_TX_BYTE), CSR_TX_BYTE);
                run_to(device, end + 300);
                check_events(events, master_events(byte, gcr, baud_m, t, mosi_before),
                             "edges all modes");
                CHECK(events.back().cycle == end);
                device.write_sfr(TCON, 0x00);
                device.write_sfr(U1CSR, 0x00);
            }
        }
    }
}

// MOSI wired to MISO (the emut `spi_loop` net): every received byte equals
// the transmitted byte for all CPOL/CPHA/ORDER (17.2.1: MOSI shifts out
// while MISO shifts in, sampled on the edge opposite to the output edge).
void test_master_loopback_all_modes() {
    for (unsigned mode = 0; mode < 8; ++mode) {
        const std::uint8_t gcr = gcr_of(mode & 4, mode & 2, mode & 1, 17);
        Cc2530F256 device;
        idle_device(device);
        master_config(device, 0, gcr);
        loop_mosi_to_miso(device);
        for (const std::uint8_t byte : {0xa5, 0x5a, 0x01, 0x80, 0xff, 0x00, 0x3c, 0xe7, 0x96}) {
            const std::uint64_t t = cycles(device);
            device.write_sfr(U1DBUF, byte);
            run_to(device, t + 64);
            CHECK_EQ(device.read_sfr(U1DBUF), byte);
        }
        // Without the wire the MISO pad floats (a peripheral input has no
        // pull, 7.6) and a floating pad reads 1 (cc2530f256.hpp contract).
        device.set_pin_listener({});
        device.set_pin_input(0, MI, std::nullopt);
        const std::uint64_t t = cycles(device);
        device.write_sfr(U1DBUF, 0x00);
        run_to(device, t + 64);
        CHECK_EQ(device.read_sfr(U1DBUF), 0xff);
        device.set_pin_input(0, MI, false); // MISO held low externally
        const std::uint64_t t2 = cycles(device);
        device.write_sfr(U1DBUF, 0xff);
        run_to(device, t2 + 64);
        CHECK_EQ(device.read_sfr(U1DBUF), 0x00);
    }
}

// MISO driven by the test at the sampling edges: the received byte follows
// the pad at those edges only (CPHA selects the edge; ORDER the bit).
void test_master_samples_miso_pad() {
    for (unsigned mode = 0; mode < 8; ++mode) {
        const bool cpha = mode & 2, msb = mode & 1;
        const std::uint8_t gcr = gcr_of(mode & 4, cpha, msb, 17);
        Cc2530F256 device;
        idle_device(device);
        master_config(device, 0, gcr);
        const std::uint8_t data = 0xb2; // 1011 0010
        const std::uint64_t t = cycles(device);
        device.write_sfr(U1DBUF, 0x00);
        // Bit k is sampled on edge 2k (CPHA = 0, leading) or 2k + 1 (CPHA =
        // 1, trailing). The pad shows bit k from 1 cycle after the previous
        // edge to 1 cycle after its sampling edge and the inverted bit in
        // between, so only a sample at the right edge gives `data`.
        for (unsigned k = 0; k < 8; ++k) {
            const std::uint64_t edge = t + 4 * ((cpha ? 2 * k + 1 : 2 * k) + 1);
            const bool bit = (data & wire_mask(k, msb)) != 0;
            run_to(device, edge - 3);
            device.set_pin_input(0, MI, bit);
            run_to(device, edge + 1);
            device.set_pin_input(0, MI, !bit);
        }
        run_to(device, t + 64);
        CHECK_EQ(device.read_sfr(U1DBUF), data);
    }
}

// Flags and DMA triggers (17.2.1, 17.6, 17.7, UxCSR): UTX1IF and the UTX1
// trigger when the byte starts (the data buffer is free again); at the end
// TX_BYTE, URX1IF and the URX1 trigger; RX_BYTE is "UART mode and SPI slave
// mode" only. Double buffering: a byte written during a transfer starts
// when the current one ends (back-to-back, CPHA = 0).
void test_master_flags_dma_double_buffer() {
    Cc2530F256 device;
    idle_device(device);
    master_config(device, 0, gcr_of(false, false, true, 17));
    loop_mosi_to_miso(device);
    const std::uint64_t t = cycles(device);
    device.write_sfr(U1DBUF, 0x3c);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_UTX1IF, IRCON2_UTX1IF);
    CHECK_EQ(Access::usart_dma(device, 1, DMA_UTX1), 1u);
    CHECK_EQ(device.read_sfr(U1CSR), CSR_ACTIVE);
    device.write_sfr(IRCON2, 0x00);
    run_to(device, t + 10);
    device.write_sfr(U1DBUF, 0xc3); // waits in the data buffer
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_UTX1IF, 0);
    CHECK_EQ(Access::usart_dma(device, 1, DMA_UTX1), 1u);
    run_to(device, t + 63);
    CHECK_EQ(device.read_sfr(TCON) & TCON_URX1IF, 0);
    CHECK_EQ(Access::usart_dma(device, 1, DMA_URX1), 0u);
    CHECK_EQ(device.read_sfr(U1CSR), CSR_ACTIVE);
    run_to(device, t + 64);
    CHECK_EQ(device.read_sfr(TCON) & TCON_URX1IF, TCON_URX1IF);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_UTX1IF, IRCON2_UTX1IF); // second byte started
    CHECK_EQ(Access::usart_dma(device, 1, DMA_URX1), 1u);
    CHECK_EQ(Access::usart_dma(device, 1, DMA_UTX1), 2u);
    CHECK_EQ(device.read_sfr(U1DBUF), 0x3c);
    CHECK_EQ(device.read_sfr(U1CSR) & (CSR_TX_BYTE | CSR_RX_BYTE), CSR_TX_BYTE);
    CHECK_EQ(device.read_sfr(U1CSR) & CSR_ACTIVE, CSR_ACTIVE);
    run_to(device, t + 128);
    CHECK_EQ(device.read_sfr(U1DBUF), 0xc3);
    CHECK_EQ(Access::usart_dma(device, 1, DMA_URX1), 2u);
    CHECK_EQ(device.read_sfr(U1CSR) & (CSR_ACTIVE | CSR_RX_BYTE), 0);
    CHECK(device.pin_drive(0, C) == Drive::Low);
}

// The host device path (set_spi_device) stays byte level: no pad toggles.
void test_master_host_device_no_pad_edges() {
    Cc2530F256 device;
    idle_device(device);
    master_config(device, 0, gcr_of(false, false, true, 17));
    std::vector<PadEvent> events;
    device.set_pin_listener([&](unsigned port, unsigned pin, Drive drive, std::uint64_t at) {
        if (port == 0)
            events.push_back({pin, drive, at});
    });
    device.set_spi_device(1, [](std::uint8_t byte) { return static_cast<std::uint8_t>(~byte); });
    const std::uint64_t t = cycles(device);
    device.write_sfr(U1DBUF, 0x5a);
    run_to(device, t + 64);
    CHECK_EQ(device.read_sfr(U1DBUF), 0xa5);
    CHECK(events.empty());
}

// Fast, step and trace runs of firmware polling URX1IF give identical state.
void test_master_modes() {
    for (const std::uint8_t gcr : {gcr_of(false, false, true, 17), gcr_of(true, true, false, 17)}) {
        Flash flash;
        flash.at(0x0000).op({0x75, CLKCONCMD, 0x80, 0x75, P2DIR, 0x40, 0x75, P0SEL, 0x38,
                             0x75, U1BAUD, 0x00, 0x75, U1GCR, gcr,
                             0x78, 0x30,             // MOV R0,#0x30
                             0x90, 0x01, 0x00});     // MOV DPTR,#table
        const std::uint16_t loop = flash.here();
        flash.op({0xe4, 0x93,                        // CLR A; MOVC A,@A+DPTR
                  0xf5, U1DBUF,                      // MOV U1DBUF,A
                  0x30, 0x8f, 0xfd,                  // JNB TCON.7 (URX1IF),$
                  0xc2, 0x8f,                        // CLR TCON.7
                  0xa6, U1DBUF,                      // MOV @R0,U1DBUF
                  0xa3, 0x08,                        // INC DPTR; INC R0
                  0xb8, 0x34});                      // CJNE R0,#0x34,loop
        flash.op({static_cast<std::uint8_t>(loop - (flash.here() + 1))}).op(IdleLoop);
        flash.at(0x0100).op({0xa5, 0x3c, 0x01, 0x80});
        for_each_mode(
            flash, 2000,
            [](Cc2530F256 &device) { loop_mosi_to_miso(device); },
            [](Cc2530F256 &device, Mode) {
                CHECK_EQ(device.read_idata(0x30), 0xa5);
                CHECK_EQ(device.read_idata(0x31), 0x3c);
                CHECK_EQ(device.read_idata(0x32), 0x01);
                CHECK_EQ(device.read_idata(0x33), 0x80);
                CHECK_EQ(Access::usart_dma(device, 1, DMA_UTX1), 4u);
                CHECK_EQ(Access::usart_dma(device, 1, DMA_URX1), 4u);
            },
            "spi master loopback modes");
    }
}

// ===================================================== SPI slave (17.2.2)

void slave_config(Cc2530F256 &device, std::uint8_t gcr) {
    device.set_pin_input(0, SS, true);              // deselected
    device.set_pin_input(0, C, (gcr & 0x80) != 0); // SCK idle at CPOL
    device.set_pin_input(0, MO, false);
    device.write_sfr(U1GCR, gcr);
    device.write_sfr(U1CSR, CSR_SLAVE);
    device.write_sfr(P0SEL, 0x3c);
}

// The test as the SPI master of a slave device: SCK edges every 4 cycles,
// MOSI changed on output edges, MISO read just before sampling edges
// (UxGCR CPOL/CPHA/ORDER).
struct Master {
    Cc2530F256 &device;
    bool cpol, cpha, msb;
    std::uint64_t t;
    std::uint8_t mosi = 0, miso = 0;
    unsigned edge = 0; // edges of the current byte
    bool miso_driven = true;

    void begin(std::uint8_t byte) {
        mosi = byte;
        miso = 0;
        edge = 0;
        if (!cpha)
            device.set_pin_input(0, MO, (mosi & wire_mask(0, msb)) != 0);
    }
    void step() {
        t += 4;
        run_to(device, t);
        const bool leading = (edge % 2) == 0;
        const unsigned k = edge / 2;
        if (leading != cpha) { // sampling edge
            const Drive d = device.pin_drive(0, MI);
            miso_driven = miso_driven && (d == Drive::High || d == Drive::Low);
            if (d == Drive::High)
                miso = static_cast<std::uint8_t>(miso | wire_mask(k, msb));
            device.set_pin_input(0, C, leading ? !cpol : cpol);
        } else {
            device.set_pin_input(0, C, leading ? !cpol : cpol);
            const unsigned next = cpha ? k : k + 1;
            if (next < 8)
                device.set_pin_input(0, MO, (mosi & wire_mask(next, msb)) != 0);
        }
        ++edge;
    }
    void steps(unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            step();
    }
    void select(bool selected) {
        t += 4;
        run_to(device, t);
        device.set_pin_input(0, SS, !selected);
    }
};

// Number of edges up to and including the 8th sampling edge: 15 (CPHA = 0,
// leading edge 7) or 16 (CPHA = 1, trailing edge 7).
unsigned edges_to_last_sample(bool cpha) { return cpha ? 16 : 15; }

// 17.2.2 / 17.3: with SSN low the slave samples MOSI and drives MISO from
// the transmit register; at the end of the byte RX_BYTE, URX1IF, URX1 DMA
// trigger and the byte in UxDBUF. UTX1IF "at the start of the operation"
// (model-defined: when the written byte enters the shift register, at once
// if the slave is between bytes). ACTIVE equals the (asserted) slave select.
// MISO is 3-state while SSN is high. TX_BYTE is "UART mode and SPI master
// mode" only.
void test_slave_receive_transmit_all_modes() {
    for (unsigned mode = 0; mode < 8; ++mode) {
        const bool cpol = mode & 4, cpha = mode & 2, msb = mode & 1;
        const std::uint8_t gcr = gcr_of(cpol, cpha, msb, 0);
        Cc2530F256 device;
        idle_device(device);
        slave_config(device, gcr);
        CHECK(device.pin_drive(0, MI) == Drive::HiZ);
        CHECK(device.pin_drive(0, MO) == Drive::HiZ); // inputs (17.2.2, 7.6: no pull)
        CHECK(device.pin_drive(0, C) == Drive::HiZ);
        CHECK(device.pin_drive(0, SS) == Drive::HiZ);
        CHECK_EQ(device.read_sfr(U1CSR), CSR_SLAVE);
        device.write_sfr(U1DBUF, 0x6d); // 0110 1101
        CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_UTX1IF, IRCON2_UTX1IF);
        CHECK_EQ(Access::usart_dma(device, 1, DMA_UTX1), 1u);
        CHECK(device.pin_drive(0, MI) == Drive::HiZ);
        Master master{device, cpol, cpha, msb, cycles(device)};
        master.select(true);
        CHECK_EQ(device.read_sfr(U1CSR), CSR_SLAVE | CSR_ACTIVE);
        CHECK(device.pin_drive(0, MI) == Drive::High || device.pin_drive(0, MI) == Drive::Low);
        if (!cpha) // bit 0 must be on MISO before the first (sampling) edge
            CHECK(device.pin_drive(0, MI) ==
                  ((0x6d & wire_mask(0, msb)) ? Drive::High : Drive::Low));
        master.begin(0xc6);
        master.steps(edges_to_last_sample(cpha) - 1);
        CHECK_EQ(device.read_sfr(U1CSR) & CSR_RX_BYTE, 0);
        CHECK_EQ(device.read_sfr(TCON) & TCON_URX1IF, 0);
        CHECK_EQ(Access::usart_dma(device, 1, DMA_URX1), 0u);
        master.step(); // 8th sampling edge
        CHECK_EQ(device.read_sfr(U1CSR), CSR_SLAVE | CSR_RX_BYTE | CSR_ACTIVE);
        CHECK_EQ(device.read_sfr(TCON) & TCON_URX1IF, TCON_URX1IF);
        CHECK_EQ(Access::usart_dma(device, 1, DMA_URX1), 1u);
        master.steps(16 - edges_to_last_sample(cpha));
        CHECK(master.miso_driven);
        CHECK_EQ(master.miso, 0x6d);
        CHECK_EQ(device.read_sfr(U1DBUF), 0xc6);
        device.write_sfr(U1CSR, CSR_SLAVE); // RX_BYTE is R/W0
        CHECK_EQ(device.read_sfr(U1CSR), CSR_SLAVE | CSR_ACTIVE);
        master.select(false);
        CHECK(device.pin_drive(0, MI) == Drive::HiZ);
        CHECK_EQ(device.read_sfr(U1CSR), CSR_SLAVE);
    }
}

// Transmit double buffering in slave mode: a byte written during a transfer
// is loaded when the current byte ends (UTX1IF then, model-defined); without
// a new byte the previous one is sent again (model-defined).
void test_slave_transmit_buffering() {
    for (const bool cpha : {false, true}) {
        const std::uint8_t gcr = gcr_of(false, cpha, true, 0);
        Cc2530F256 device;
        idle_device(device);
        slave_config(device, gcr);
        device.write_sfr(U1DBUF, 0x81);
        device.write_sfr(IRCON2, 0x00);
        Master master{device, false, cpha, true, cycles(device)};
        master.select(true);
        master.begin(0x11);
        master.steps(6);
        device.write_sfr(U1DBUF, 0x7e); // during the byte: waits
        CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_UTX1IF, 0);
        CHECK_EQ(Access::usart_dma(device, 1, DMA_UTX1), 1u);
        master.steps(edges_to_last_sample(cpha) - 6);
        CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_UTX1IF, IRCON2_UTX1IF);
        CHECK_EQ(Access::usart_dma(device, 1, DMA_UTX1), 2u);
        master.steps(16 - edges_to_last_sample(cpha));
        CHECK_EQ(master.miso, 0x81);
        CHECK_EQ(device.read_sfr(U1DBUF), 0x11);
        master.begin(0x22);
        master.steps(16);
        CHECK_EQ(master.miso, 0x7e);
        CHECK_EQ(device.read_sfr(U1DBUF), 0x22);
        master.begin(0x33);
        master.steps(16);
        CHECK_EQ(master.miso, 0x7e); // sent again
        CHECK_EQ(device.read_sfr(U1DBUF), 0x33);
        CHECK_EQ(Access::usart_dma(device, 1, DMA_UTX1), 2u);
        CHECK_EQ(Access::usart_dma(device, 1, DMA_URX1), 3u);
        CHECK_EQ(device.read_sfr(U1CSR) & CSR_TX_BYTE, 0);
    }
}

// 17.3: while SSN is high the slave "does not receive data": SCK edges are
// ignored. Released during a byte, "the next received byte is not received
// properly"; a flush removes the partial byte (17.3, 17.5).
void test_slave_ssn() {
    for (unsigned mode = 0; mode < 4; ++mode) {
        const bool cpol = mode & 2, cpha = mode & 1;
        Cc2530F256 device;
        idle_device(device);
        slave_config(device, gcr_of(cpol, cpha, true, 0));
        device.write_sfr(U1DBUF, 0x00);
        Master master{device, cpol, cpha, true, cycles(device)};

        // Deselected: 20 edges (more than a byte) change nothing.
        master.begin(0xff);
        master.steps(20);
        CHECK_EQ(device.read_sfr(U1CSR), CSR_SLAVE);
        CHECK_EQ(device.read_sfr(TCON) & TCON_URX1IF, 0);
        CHECK(device.pin_drive(0, MI) == Drive::HiZ);
        if (master.edge % 2) // back to idle SCK
            master.step();
        master.select(true);
        master.begin(0x5a);
        master.steps(16);
        CHECK_EQ(device.read_sfr(U1CSR) & CSR_RX_BYTE, CSR_RX_BYTE);
        CHECK_EQ(device.read_sfr(U1DBUF), 0x5a);
        device.write_sfr(U1CSR, CSR_SLAVE); // clear RX_BYTE (R/W0)

        // Released after 3 bits, flushed, reselected: the next byte is exact.
        master.begin(0xff);
        master.steps(6);
        master.select(false);
        CHECK_EQ(device.read_sfr(U1CSR) & CSR_ACTIVE, 0);
        CHECK(device.pin_drive(0, MI) == Drive::HiZ);
        device.write_sfr(U1UCR, 0x82); // FLUSH (STOP keeps its reset value)
        master.select(true);
        master.begin(0x96);
        master.steps(edges_to_last_sample(cpha) - 1);
        CHECK_EQ(device.read_sfr(U1CSR) & CSR_RX_BYTE, 0);
        master.steps(16 - (edges_to_last_sample(cpha) - 1));
        CHECK_EQ(device.read_sfr(U1CSR) & CSR_RX_BYTE, CSR_RX_BYTE);
        CHECK_EQ(device.read_sfr(U1DBUF), 0x96);
        device.write_sfr(U1CSR, CSR_SLAVE);

        // Released after 3 bits without a flush: model-defined, the three
        // bits stay, so the next byte completes after 5 bits (MSB first:
        // 111 from 0xff, then 00000 from 0x00 = 0xe0).
        master.begin(0xff);
        master.steps(6);
        master.select(false);
        master.select(true);
        master.begin(0x00);
        const unsigned five = cpha ? 10 : 9; // through the 5th sampling edge
        master.steps(five - 1);
        CHECK_EQ(device.read_sfr(U1CSR) & CSR_RX_BYTE, 0);
        master.step();
        CHECK_EQ(device.read_sfr(U1CSR) & CSR_RX_BYTE, CSR_RX_BYTE);
        CHECK_EQ(device.read_sfr(U1DBUF), 0xe0);
    }
}

// Slave SCK polarity: with CPOL = 1, SCK idles high and a byte clocked with
// CPOL = 0 edges (one inverted SCK) is not received as sent.
void test_slave_cpol() {
    Cc2530F256 device;
    idle_device(device);
    slave_config(device, gcr_of(true, false, true, 0));
    device.write_sfr(U1DBUF, 0xa3);
    Master master{device, true, false, true, cycles(device)};
    master.select(true);
    master.begin(0x4b);
    master.steps(16);
    CHECK_EQ(device.read_sfr(U1DBUF), 0x4b);
    CHECK_EQ(master.miso, 0xa3);
    // The same SCK waveform for CPOL = 0 (idle low) after re-selecting.
    master.select(false);
    device.write_sfr(U1GCR, gcr_of(false, false, true, 0));
    device.set_pin_input(0, C, false);
    master.select(true);
    Master low{device, false, false, true, master.t};
    low.begin(0x4b);
    low.steps(16);
    CHECK_EQ(device.read_sfr(U1DBUF), 0x4b);
}

// ===================================================== master -> slave

// Two devices in lockstep: device A is the master, device B the slave
// (both USART1 alternative 1). A's C and MO drive B's C and MO, B's MI
// drives A's MI, B's SSN is held low. Both receive the other's byte; B's
// byte completes at its 8th sampling edge: A's edge 14 (CPHA = 0, t + 60)
// or edge 15 (CPHA = 1, t + 64).
void test_master_to_slave_two_devices() {
    for (unsigned mode = 0; mode < 8; ++mode) {
        const bool cpol = mode & 4, cpha = mode & 2, msb = mode & 1;
        const std::uint8_t gcr = gcr_of(cpol, cpha, msb, 17);
        Cc2530F256 a, b;
        idle_device(a);
        idle_device(b);
        run_to(a, cycles(b));
        run_to(b, cycles(a));
        struct Ev {
            unsigned pin;
            Drive drive;
            std::uint64_t at;
        };
        std::vector<Ev> from_a, from_b;
        a.set_pin_listener([&](unsigned port, unsigned pin, Drive drive, std::uint64_t at) {
            if (port == 0 && (pin == C || pin == MO))
                from_a.push_back({pin, drive, at});
        });
        b.set_pin_listener([&](unsigned port, unsigned pin, Drive drive, std::uint64_t at) {
            if (port == 0 && pin == MI)
                from_b.push_back({pin, drive, at});
        });
        master_config(a, 0, gcr);
        b.set_pin_input(0, SS, false);
        b.set_pin_input(0, C, level_of(a.pin_drive(0, C)));
        b.set_pin_input(0, MO, level_of(a.pin_drive(0, MO)));
        b.write_sfr(U1GCR, gcr);
        b.write_sfr(U1CSR, CSR_SLAVE);
        b.write_sfr(P0SEL, 0x3c);
        from_a.clear();
        const auto sync = [&](std::uint64_t until) {
            run_to(a, until);
            for (const Ev &e : from_a) {
                run_to(b, e.at);
                b.set_pin_input(0, e.pin, level_of(e.drive));
            }
            from_a.clear();
            run_to(b, until);
            for (const Ev &e : from_b)
                a.set_pin_input(0, MI, level_of(e.drive));
            from_b.clear();
        };
        b.write_sfr(U1DBUF, 0x9c);
        sync(cycles(a));
        const std::uint64_t t = cycles(a);
        a.write_sfr(U1DBUF, 0x35);
        std::uint64_t slave_rx_at = 0;
        for (std::uint64_t now = t; now <= t + 70; ++now) {
            sync(now);
            if (!slave_rx_at && (b.read_sfr(U1CSR) & CSR_RX_BYTE))
                slave_rx_at = now;
        }
        CHECK_EQ(a.read_sfr(U1DBUF), 0x9c);
        CHECK_EQ(b.read_sfr(U1DBUF), 0x35);
        CHECK_EQ(slave_rx_at, t + (cpha ? 64 : 60));
        CHECK_EQ(a.read_sfr(TCON) & TCON_URX1IF, TCON_URX1IF);
        CHECK_EQ(b.read_sfr(TCON) & TCON_URX1IF, TCON_URX1IF);
    }
}

} // namespace

int main() {
    test_master_edge_times_explicit();
    test_master_edge_times_all_modes();
    test_master_loopback_all_modes();
    test_master_samples_miso_pad();
    test_master_flags_dma_double_buffer();
    test_master_host_device_no_pad_edges();
    test_master_modes();
    test_slave_receive_transmit_all_modes();
    test_slave_transmit_buffering();
    test_slave_ssn();
    test_slave_cpol();
    test_master_to_slave_two_devices();
    return finish("cc2530 spi");
}
