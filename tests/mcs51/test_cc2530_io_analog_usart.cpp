// CC2530F256 I/O controller, ADC + RNG, analog comparator, op-amp and
// USART0/USART1 tests. Every expectation is derived from SWRU191F (chapter
// and section cited), the SWRS081B datasheet or a rule the models document
// as "Model-defined"; programs are hand-encoded MCS-51 machine code.
//
// Timing: virtual time counts 32 MHz cycles. Programs select the 32 MHz
// XOSC (CLKCONCMD = 0x80) unless a test says otherwise, and wait in Idle
// (SLEEPCMD.MODE = 00 at reset, PCON.IDLE = 1) so that run() stops exactly
// at a requested cycle.

#include "cc2530_periph_support.hpp"

#include "cc2530_analog.hpp"
#include "cc2530_io.hpp"
#include "cc2530_soc.hpp"
#include "cc2530_usart.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace mcs51emu::cc2530 {
// White-box access to the DMA trigger counters each model keeps (the DMA
// controller is another work package; its trigger input is a no-op here).
struct IoAnalogUsartTestAccess {
    static std::uint64_t io_dma(Cc2530F256 &device, unsigned trigger) {
        return device.peripherals().io->dma_triggers_[trigger];
    }
    static std::uint64_t adc_dma(Cc2530F256 &device, unsigned trigger) {
        return device.peripherals().adc->dma_triggers_[trigger];
    }
    static std::uint64_t usart_dma(Cc2530F256 &device, unsigned index, unsigned trigger) {
        return device.peripherals().usart[index]->dma_triggers_[trigger];
    }
};
} // namespace mcs51emu::cc2530

namespace {

using namespace cc2530test;
using Access = mcs51emu::cc2530::IoAnalogUsartTestAccess;
using Drive = mcs51emu::Cc2530PinDrive;

// SFRs (SWRU191F Table 2-1).
constexpr std::uint8_t P0 = 0x80, SP = 0x81, U0CSR = 0x86, PCON = 0x87, TCON = 0x88,
                       P0IFG = 0x89, P1IFG = 0x8a, P2IFG = 0x8b, PICTL = 0x8c, P1IEN = 0x8d,
                       P0INP = 0x8f, P1 = 0x90, IEN2 = 0x9a, CLKCONSTA = 0x9e, P2 = 0xa0,
                       IEN0 = 0xa8, P0IEN = 0xab, P2IEN = 0xac, PMUX = 0xae, ADCCON1 = 0xb4,
                       ADCCON2 = 0xb5, ADCCON3 = 0xb6, IEN1 = 0xb8, ADCL = 0xba, ADCH = 0xbb,
                       RNDL = 0xbc, RNDH = 0xbd, SLEEPCMD = 0xbe, IRCON = 0xc0, U0DBUF = 0xc1,
                       U0BAUD = 0xc2, U0UCR = 0xc4, U0GCR = 0xc5, CLKCONCMD = 0xc6,
                       IRCON2 = 0xe8, PERCFG = 0xf1, APCFG = 0xf2, P0SEL = 0xf3, P1SEL = 0xf4,
                       P2SEL = 0xf5, P1INP = 0xf6, P2INP = 0xf7, U1CSR = 0xf8, U1DBUF = 0xf9,
                       U1BAUD = 0xfa, U1UCR = 0xfb, U1GCR = 0xfc, P0DIR = 0xfd, P1DIR = 0xfe,
                       P2DIR = 0xff;
constexpr std::uint8_t T1CCTL0 = 0xe5;
constexpr std::uint8_t T1STAT = 0xaf, T1CC0L = 0xda, T1CC0H = 0xdb, T1CTL = 0xe4;
// XREGs.
constexpr std::uint16_t OBSSEL0 = 0x6243, TR0 = 0x624b, CMPCTL = 0x62d0, OPAMPC = 0x62c0,
                        OPAMPS = 0x62c1, OPAMPMC = 0x61a6, ATEST = 0x61bd;
// Bit addresses.
constexpr std::uint8_t BIT_EA = 0xaf, BIT_P0IF = 0xc5, BIT_P2IF = 0xe8, BIT_UTX0IF = 0xe9,
                       BIT_P1IF = 0xeb, BIT_P1_0 = 0x90;
// Flag masks.
constexpr std::uint8_t TCON_URX1IF = 0x80, TCON_ADCIF = 0x20, TCON_URX0IF = 0x08;
constexpr std::uint8_t IRCON_P0IF = 0x20;
constexpr std::uint8_t IRCON2_P1IF = 0x08, IRCON2_UTX1IF = 0x04, IRCON2_UTX0IF = 0x02,
                       IRCON2_P2IF = 0x01;
// DMA triggers (SWRU191F Table 8-1).
constexpr unsigned DMA_IOC_0 = 12, DMA_IOC_1 = 13, DMA_URX0 = 14, DMA_UTX0 = 15, DMA_URX1 = 16,
                   DMA_UTX1 = 17, DMA_ADC_CHALL = 20, DMA_ADC_CH11 = 21;

// UART timing from SWRU191F 17.4, equation 3: baud = (256 + BAUD_M) *
// 2^BAUD_E / 2^28 * F, F = 32 MHz. One bit lasts 2^28 / ((256 + M) * 2^E)
// cycles of 32 MHz; the frame ends at the first cycle >= n bit times.
//   115200 (Table 17-1: M = 216, E = 11): 277.69 cycles per bit.
//   9600 (Table 17-1: M = 59, E = 8): 3328.81 cycles per bit.
constexpr std::uint64_t B115200_10 = 2777, B115200_12 = 3333, B115200_20 = 5554,
                        B115200_24 = 6665, B115200_30 = 8331;
constexpr std::uint64_t B9600_10 = 33289, B9600_20 = 66577;

// MOV PCON,#1; SJMP back: wait in the power mode SLEEPCMD selects.
const std::vector<std::uint8_t> IdleLoop = {0x75, PCON, 0x01, 0x80, 0xfb};

std::uint64_t cycles(const Cc2530F256 &device) { return device.registers().cycles; }

// Runs until the cycle counter reaches `cycle`. In a power mode run() stops
// exactly there.
void run_to(Cc2530F256 &device, std::uint64_t cycle) {
    const Cc2530RunResult result = device.run(10'000'000, cycle);
    CHECK(result.error.empty());
}

// Stops before the instruction at `pc` and returns its start cycle.
std::uint64_t run_to_pc(Cc2530F256 &device, std::uint16_t pc) {
    device.set_breakpoint(pc);
    const Cc2530RunResult result = device.run(1'000'000);
    device.set_breakpoint(std::nullopt);
    CHECK(result.error.empty());
    CHECK(result.break_hit);
    CHECK_EQ(device.pc(), pc);
    return cycles(device);
}

struct PadEvent {
    unsigned port, pin;
    Drive drive;
    std::uint64_t cycle;
    bool operator==(const PadEvent &other) const {
        return port == other.port && pin == other.pin && drive == other.drive &&
               cycle == other.cycle;
    }
};

void record_pads(Cc2530F256 &device, std::vector<PadEvent> &events) {
    device.set_pin_listener([&events](unsigned port, unsigned pin, Drive drive, std::uint64_t at) {
        events.push_back({port, pin, drive, at});
    });
}

void check_events(const std::vector<PadEvent> &actual, const std::vector<PadEvent> &expected,
                  const char *name) {
    if (actual == expected)
        return;
    std::cerr << name << ": pad events differ\n  got:";
    for (const PadEvent &e : actual)
        std::cerr << " P" << e.port << '_' << e.pin << '=' << static_cast<int>(e.drive) << '@'
                  << e.cycle;
    std::cerr << "\n  want:";
    for (const PadEvent &e : expected)
        std::cerr << " P" << e.port << '_' << e.pin << '=' << static_cast<int>(e.drive) << '@'
                  << e.cycle;
    std::cerr << '\n';
    ++failures;
}

// ============================================================== GPIO (7.3)

// SWRU191F 7.3: PxDIR selects output (the latch drives the pad) or input;
// inputs get the pull of PxINP/P2INP.PDUPx (P1_0/P1_1 have none); reading
// Px returns the pin levels, read-modify-write instructions use the latch.
// The pin listener reports every drive change at the start cycle of the
// instruction that caused it.
void test_gpio_direction_pulls_and_listener() {
    Flash flash;
    flash.at(0x0000).op({0x75, P1DIR, 0x0c,  // t=0  P1_2, P1_3 outputs (latch 1)
                         0x75, P1, 0xf3,     // t=2  latch bits 3:2 = 0
                         0x75, P2INP, 0x40,  // t=4  PDUP1: port 1 pulldowns
                         0x75, P1INP, 0x80,  // t=6  MDP1_7: P1_7 3-state
                         0x75, P0INP, 0x01,  // t=8  MDP0_0: P0_0 3-state
                         0x75, P2INP, 0x61,  // t=10 PDUP0+PDUP1, MDP2_0
                         0xe5, P1,           // t=12 MOV A,P1 (pin levels)
                         0xf5, 0x30,         // t=13 MOV 0x30,A
                         0x53, P1, 0xff,     // t=14 ANL P1,#0xFF (latch read)
                         0x75, P1DIR, 0x1c,  // t=16 P1_4 output: drives latch bit 4
                         0x80, 0xfe});

    Cc2530F256 device;
    load(device, flash);
    // Reset (7.11: PxDIR = 0, PxINP = 0, P2INP = 0): every input pulls up,
    // except P1_0/P1_1 (no pull); a floating pad reads 1.
    CHECK(device.pin_drive(1, 0) == Drive::HiZ);
    CHECK(device.pin_drive(1, 1) == Drive::HiZ);
    for (unsigned pin = 0; pin < 8; ++pin)
        CHECK(device.pin_drive(0, pin) == Drive::PullUp);
    for (unsigned pin = 2; pin < 8; ++pin)
        CHECK(device.pin_drive(1, pin) == Drive::PullUp);
    for (unsigned pin = 0; pin < 5; ++pin)
        CHECK(device.pin_drive(2, pin) == Drive::PullUp);
    CHECK_EQ(device.read_sfr(P0), 0xff);
    CHECK_EQ(device.read_sfr(P1), 0xff);
    CHECK_EQ(device.read_sfr(P2), 0x1f); // P2 bits 7:5 read 0

    std::vector<PadEvent> events;
    record_pads(device, events);
    const Cc2530RunResult result = device.run(10);
    CHECK(result.error.empty());
    CHECK_EQ(cycles(device), 18u);

    std::vector<PadEvent> expected = {
        {1, 2, Drive::High, 0},     {1, 3, Drive::High, 0},     {1, 2, Drive::Low, 2},
        {1, 3, Drive::Low, 2},      {1, 4, Drive::PullDown, 4}, {1, 5, Drive::PullDown, 4},
        {1, 6, Drive::PullDown, 4}, {1, 7, Drive::PullDown, 4}, {1, 7, Drive::HiZ, 6},
        {0, 0, Drive::HiZ, 8}};
    for (unsigned pin = 1; pin < 8; ++pin)
        expected.push_back({0, pin, Drive::PullDown, 10});
    expected.push_back({2, 0, Drive::HiZ, 10});
    // Only if ANL read the latch (bit 4 = 1) does P1_4 drive high.
    expected.push_back({1, 4, Drive::High, 16});
    check_events(events, expected, "gpio");

    // P1: P1_0/P1_1/P1_7 floating (1), P1_2/P1_3 low, P1_5/P1_6 pulled down,
    // P1_4 driven high; at t=12 P1_4 was still pulled down.
    CHECK_EQ(device.read_idata(0x30), 0x83);
    CHECK_EQ(device.read_sfr(P1), 0x93);
    CHECK_EQ(device.read_sfr(P0), 0x01);
    CHECK_EQ(device.read_sfr(P2), 0x1f); // P2_1-P2_4 still pull up (PDUP2 = 0)
    // 7.4: edges on outputs and pulls set the flags whatever P1IEN is:
    // P1_7 rose at t=6 (pulldown -> 3-state), P1_4 at t=16 (output high).
    CHECK_EQ(device.read_sfr(P1IFG), 0x90);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_P1IF, 0);

    // The host's external level overrides a pull.
    events.clear();
    device.set_pin_input(1, 5, true);
    CHECK(device.pin_level(1, 5));
    CHECK(device.pin_drive(1, 5) == Drive::PullDown);
    CHECK_EQ(device.read_sfr(P1), 0xb3);
    CHECK_EQ(device.read_sfr(P1IFG), 0xb0);
    device.set_pin_input(1, 5, std::nullopt);
    CHECK_EQ(device.read_sfr(P1), 0x93);
    CHECK(events.empty()); // the chip's own drive did not change

    // 7.8: with CLKCONCMD.OSC32K = 0, P2_3/P2_4 belong to the 32 kHz XOSC.
    device.write_sfr(CLKCONCMD, 0x00);
    CHECK(device.pin_drive(2, 3) == Drive::HiZ);
    CHECK(device.pin_drive(2, 4) == Drive::HiZ);
    CHECK(device.pin_drive(2, 1) == Drive::PullUp);
    CHECK_EQ(events.size(), 2u);
}

// PMUX (7.11): CKOEN puts the 32 kHz clock on P0[CKOPIN], overriding the
// pin configuration; DREGSTA shows the digital regulator status on
// P1[DREGSTAPIN] (1 while the regulator is on).
void test_pmux_clock_and_regulator_outputs() {
    Flash flash;
    flash.at(0x0000).op({0x75, PMUX, 0xe0}).op(IdleLoop); // CKOEN, CKOPIN = 6
    Cc2530F256 device;
    load(device, flash);
    std::vector<PadEvent> events;
    record_pads(device, events);
    run_to(device, 2000);
    CHECK_EQ(cycles(device), 2000u);
    // 32.768 kHz: a half period lasts 32e6 / 65536 = 488.28125 cycles; the
    // clock is low at time 0 and changes at the first cycle >= k * 488.28125.
    check_events(events,
                 {{0, 6, Drive::Low, 0},
                  {0, 6, Drive::High, 489},
                  {0, 6, Drive::Low, 977},
                  {0, 6, Drive::High, 1465},
                  {0, 6, Drive::Low, 1954}},
                 "cko");
    events.clear();
    device.write_sfr(PMUX, 0x0b); // DREGSTA on P1_3
    CHECK(device.pin_drive(1, 3) == Drive::High);
    CHECK(device.pin_drive(0, 6) == Drive::PullUp);
    run_to(device, 3000);
    CHECK_EQ(events.size(), 2u); // no further clock edges
}

// =================================================== port interrupts (7.4)

// ISR (P0INT, vector 0x6B): INC 0x40; MOV 0x41,P0IFG; MOV P0IFG,#0;
// CLR P0IF; RETI. The flag is cleared before the CPU flag as 7.4 requires.
std::vector<std::uint8_t> port0_isr() {
    return {0x05, 0x40, 0x85, P0IFG, 0x41, 0x75, P0IFG, 0x00, 0xc2, BIT_P0IF, 0x32};
}

void test_port0_interrupt_edges_and_mask() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x006b).op(port0_isr());
    flash.at(0x0100)
        .op({0x75, SP, 0x70, 0x75, P0IEN, 0x10, 0x75, IEN1, 0x20, 0xd2, BIT_EA})
        .op(IdleLoop);

    Cc2530F256 device;
    load(device, flash);
    Cc2530RunResult result = device.run(100);
    CHECK(result.sleeping);
    const std::uint64_t c = cycles(device);

    // PICTL.P0ICON = 0: rising edges only.
    device.set_pin_input(0, 4, false);
    CHECK_EQ(device.read_sfr(P0IFG), 0x00);
    run_to(device, c + 100);
    CHECK_EQ(device.read_idata(0x40), 0);
    device.set_pin_input(0, 4, true);
    CHECK_EQ(device.read_sfr(P0IFG), 0x10);
    CHECK_EQ(device.read_sfr(IRCON) & IRCON_P0IF, IRCON_P0IF);
    run_to(device, c + 200);
    CHECK_EQ(device.read_idata(0x40), 1);
    CHECK_EQ(device.read_idata(0x41), 0x10);
    CHECK_EQ(device.read_sfr(P0IFG), 0x00);
    CHECK_EQ(device.read_sfr(IRCON) & IRCON_P0IF, 0);

    // P0IEN masks the CPU interrupt, not the status flag (7.4, 7.11 P0IFG).
    device.set_pin_input(0, 3, false);
    device.set_pin_input(0, 3, true);
    CHECK_EQ(device.read_sfr(P0IFG), 0x08);
    CHECK_EQ(device.read_sfr(IRCON) & IRCON_P0IF, 0);
    run_to(device, c + 300);
    CHECK_EQ(device.read_idata(0x40), 1);
    // Enabling the pin later delivers the pending request.
    device.write_sfr(P0IEN, 0x18);
    CHECK_EQ(device.read_sfr(IRCON) & IRCON_P0IF, IRCON_P0IF);
    run_to(device, c + 400);
    CHECK_EQ(device.read_idata(0x40), 2);
    CHECK_EQ(device.read_idata(0x41), 0x08);

    // PICTL.P0ICON = 1: falling edges.
    device.write_sfr(PICTL, 0x01);
    device.set_pin_input(0, 4, false);
    CHECK_EQ(device.read_sfr(P0IFG), 0x10);
    run_to(device, c + 500);
    CHECK_EQ(device.read_idata(0x40), 3);
    CHECK_EQ(device.read_idata(0x41), 0x10);
    // 7.5 / Table 8-1: IOC_0 on interrupts of interrupt-enabled P0 pins
    // (the masked P0_3 edge does not count).
    CHECK_EQ(Access::io_dma(device, DMA_IOC_0), 2u);
    CHECK_EQ(Access::io_dma(device, DMA_IOC_1), 0u);
}

// 7.4: "This flag must be cleared prior to clearing the CPU port interrupt
// flag" (2.5.1: port interrupts are level sources). An ISR that clears
// IRCON.P0IF first sees the request again and runs a second time.
void test_port0_flag_clearing_order() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x006b).op({0x05, 0x40,                // INC 0x40
                         0xc2, BIT_P0IF,            // CLR P0IF (too early)
                         0x75, P0IFG, 0x00, 0x32}); // MOV P0IFG,#0; RETI
    flash.at(0x0100)
        .op({0x75, SP, 0x70, 0x75, P0IEN, 0x01, 0x75, IEN1, 0x20, 0xd2, BIT_EA})
        .op(IdleLoop);
    Cc2530F256 device;
    load(device, flash);
    device.run(100);
    const std::uint64_t c = cycles(device);
    device.set_pin_input(0, 0, false);
    device.set_pin_input(0, 0, true);
    run_to(device, c + 200);
    CHECK_EQ(device.read_idata(0x40), 2);
    CHECK_EQ(device.read_sfr(P0IFG), 0x00);
    CHECK_EQ(device.read_sfr(IRCON) & IRCON_P0IF, 0);
}

// PICTL.P1ICONL/P1ICONH select the edge per half of port 1; P1IEN masks;
// the CPU flag follows P1IFG & P1IEN.
void test_port1_half_port_edges_and_order() {
    Flash flash;
    flash.at(0x0000).op({0x80, 0xfe});
    Cc2530F256 device;
    load(device, flash);
    device.write_sfr(PICTL, 0x02); // P1_3-P1_0 falling, P1_7-P1_4 rising
    device.set_pin_input(1, 2, false); // falling, low half: flag
    device.set_pin_input(1, 6, false); // falling, high half: no flag
    device.set_pin_input(1, 6, true);  // rising, high half: flag
    device.set_pin_input(1, 2, true);  // rising, low half: no flag
    CHECK_EQ(device.read_sfr(P1IFG), 0x44);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_P1IF, 0);
    CHECK_EQ(Access::io_dma(device, DMA_IOC_1), 0u); // no enabled pin
    device.write_sfr(P1IEN, 0x04);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_P1IF, IRCON2_P1IF);
    // Wrong order: clearing P1IF while P1IFG[2] is still set does not stick.
    device.write_sfr(IRCON2, 0x00);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_P1IF, IRCON2_P1IF);
    // Right order: P1IFG (R/W0, write 0 to clear) first, then P1IF.
    device.write_sfr(P1IFG, 0xfb);
    CHECK_EQ(device.read_sfr(P1IFG), 0x40);
    device.write_sfr(IRCON2, 0x00);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_P1IF, 0);
    // Writing 1 to a PxIFG bit has no effect.
    device.write_sfr(P1IFG, 0xff);
    CHECK_EQ(device.read_sfr(P1IFG), 0x40);
    device.set_pin_input(1, 2, false);
    CHECK_EQ(Access::io_dma(device, DMA_IOC_1), 1u);
}

// Port 2 interrupt (P2INT, vector 0x33, IEN2.P2IE, IRCON2.P2IF).
void test_port2_interrupt() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x0033).op({0x05, 0x43, 0x85, P2IFG, 0x44, 0x75, P2IFG, 0x00, 0xc2, BIT_P2IF, 0x32});
    flash.at(0x0100)
        .op({0x75, SP, 0x70, 0x75, PICTL, 0x08, // P2ICON: falling
             0x75, P2IEN, 0x01, 0x75, IEN2, 0x02, 0xd2, BIT_EA})
        .op(IdleLoop);
    Cc2530F256 device;
    load(device, flash);
    device.run(100);
    const std::uint64_t c = cycles(device);
    device.set_pin_input(2, 4, false); // P2IEN[4] = 0: flag only
    CHECK_EQ(device.read_sfr(P2IFG), 0x10);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_P2IF, 0);
    run_to(device, c + 100);
    CHECK_EQ(device.read_idata(0x43), 0);
    device.set_pin_input(2, 0, false);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_P2IF, IRCON2_P2IF);
    run_to(device, c + 200);
    CHECK_EQ(device.read_idata(0x43), 1);
    CHECK_EQ(device.read_idata(0x44), 0x11);
    CHECK_EQ(device.read_sfr(P2IFG), 0x00);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_P2IF, 0);
}

// SWRU191F 4.1 / Table 4-1: PM1-PM3 end on an I/O port interrupt. A pad
// edge injected while the CPU sleeps sets P0IFG and wakes it at that cycle.
// Also: DREGSTA is 0 in PM2/PM3 (regulator off); OBSSEL0-5 (PMUX note),
// TR0 (12.2.10) and the op-amp (18.1) do not survive PM2/PM3.
void test_port_wake_from_power_modes() {
    for (const auto &[sleepcmd, mode] :
         {std::pair{std::uint8_t{0xfd}, Cc2530PowerMode::Pm1},
          std::pair{std::uint8_t{0xfe}, Cc2530PowerMode::Pm2},
          std::pair{std::uint8_t{0xff}, Cc2530PowerMode::Pm3}}) {
        const bool deep = mode != Cc2530PowerMode::Pm1;
        Flash flash;
        flash.at(0x0000).op({0x02, 0x01, 0x00});
        flash.at(0x006b).op(port0_isr());
        flash.at(0x0100)
            .op({0x75, SP, 0x70, 0x75, PMUX, 0x0b}) // DREGSTA on P1_3
            .op(movx_write(OBSSEL0, 0x80))
            .op(movx_write(TR0, 0x01))
            .op(movx_write(OPAMPC, 0x01))
            .op({0x75, P0IEN, 0x02, 0x75, IEN1, 0x20, 0xd2, BIT_EA, 0x75, SLEEPCMD, sleepcmd,
                 0x75, PCON, 0x01, // enter the power mode
                 0x05, 0x42,       // INC 0x42 (first instruction after wake)
                 0x80, 0xfe});
        Cc2530F256 device;
        load(device, flash);
        std::vector<PadEvent> events;
        Cc2530RunResult result = device.run(100);
        CHECK(result.sleeping);
        CHECK(device.power_mode() == mode);
        const std::uint64_t c = cycles(device);
        CHECK(device.pin_drive(1, 3) == (deep ? Drive::Low : Drive::High));
        CHECK(device.pin_drive(1, 0) == (deep ? Drive::HiZ : Drive::Low));
        CHECK_EQ(device.peek_xdata(OBSSEL0), deep ? 0x00 : 0x80);
        CHECK_EQ(device.peek_xdata(TR0), deep ? 0x00 : 0x01);
        CHECK_EQ(device.peek_xdata(OPAMPC), deep ? 0x00 : 0x01);
        record_pads(device, events);

        run_to(device, c + 20000);
        CHECK_EQ(cycles(device), c + 20000);
        device.set_pin_input(0, 1, false); // falling: not selected, no wake
        result = device.run(100, c + 30000);
        CHECK(result.sleeping && result.instructions == 0);
        CHECK_EQ(cycles(device), c + 30000);
        CHECK_EQ(device.read_sfr(P0IFG), 0x00);
        device.set_pin_input(0, 1, true); // rising edge at c + 30000
        CHECK_EQ(device.read_sfr(P0IFG), 0x02);
        result = device.run(100, c + 30100);
        CHECK(result.error.empty() && !result.sleeping);
        CHECK(device.power_mode() == Cc2530PowerMode::Active);
        CHECK_EQ(device.read_idata(0x42), 1);
        CHECK_EQ(device.read_idata(0x40), 1);
        CHECK_EQ(device.read_idata(0x41), 0x02);
        CHECK_EQ(device.diagnostics().power_mode_cycles_by_mode[static_cast<unsigned>(mode)],
                 30000u);
        CHECK(device.pin_drive(1, 3) == Drive::High);
        if (deep)
            check_events(events, {{1, 3, Drive::High, c + 30000}}, "dregsta wake");
        else
            CHECK(events.empty());
    }
}

// ============================================ peripheral pin mapping (7.6)

void test_peripheral_pin_mapping() {
    Flash flash;
    flash.at(0x0000).op({0x80, 0xfe});
    Cc2530F256 device;
    load(device, flash);

    // GPIO output low on P0_3, then USART0 (alternative 1, Table 7-1: RX
    // P0_2, TX P0_3, CT P0_4, RT P0_5) claims it through P0SEL.
    device.write_sfr(P0DIR, 0x08);
    device.write_sfr(P0, 0xf7);
    CHECK(device.pin_drive(0, 3) == Drive::Low);
    device.write_sfr(U0CSR, 0x80); // UART mode
    device.write_sfr(P0SEL, 0x08);
    // 17.1: UxUCR.START = 0 -> start bit low, idle line high.
    CHECK(device.pin_drive(0, 3) == Drive::High);
    device.write_sfr(U0UCR, 0x03); // START = 1: idle line low
    CHECK(device.pin_drive(0, 3) == Drive::Low);
    device.write_sfr(P0SEL, 0x0c); // RX is an input: 3-state, no pull (7.6)
    CHECK(device.pin_drive(0, 2) == Drive::HiZ);
    // APCFG makes the pad analog whatever P0SEL says (7.6.6).
    device.write_sfr(APCFG, 0x08);
    CHECK(device.pin_drive(0, 3) == Drive::HiZ);
    device.write_sfr(APCFG, 0x00);
    CHECK(device.pin_drive(0, 3) == Drive::Low);

    // Alternative 2 (PERCFG.U0CFG = 1): TX on P1_5. P0_3 now maps USART1
    // alternative 1 (SPI master at reset, UxCSR.MODE = 0: SCK idles at
    // CPOL = 0) ahead of Timer 1 channel 1 (PRIP0 = 00); P0_2 maps the
    // unused master SSN and Timer 1 channel 0 (input capture): 3-state.
    device.write_sfr(PERCFG, 0x01);
    CHECK(device.pin_drive(0, 3) == Drive::Low);
    CHECK(device.pin_drive(0, 2) == Drive::HiZ);
    CHECK(device.pin_drive(1, 5) == Drive::PullUp);
    device.write_sfr(P1SEL, 0x20);
    CHECK(device.pin_drive(1, 5) == Drive::Low);
    device.write_sfr(U0UCR, 0x02);
    CHECK(device.pin_drive(1, 5) == Drive::High);

    // Port 0 conflicts, P2DIR.PRIP0 (7.6.4): USART0 and USART1 at
    // alternative 1. P0_4 = U0 CT (input, with FLOW) / U1 TX; P0_5 = U0 RT
    // (output, high while RE = 0) / U1 RX.
    device.write_sfr(P1SEL, 0x00);
    device.write_sfr(PERCFG, 0x00);
    device.write_sfr(U0UCR, 0x42); // FLOW
    device.write_sfr(U1CSR, 0x80);
    device.write_sfr(U1UCR, 0x03); // U1 TX idles low
    device.write_sfr(P0SEL, 0x30);
    CHECK(device.pin_drive(0, 4) == Drive::HiZ); // PRIP0 = 00: USART0 first
    CHECK(device.pin_drive(0, 5) == Drive::High);
    device.write_sfr(P2DIR, 0x40); // PRIP0 = 01: USART1 first
    CHECK(device.pin_drive(0, 4) == Drive::Low);
    CHECK(device.pin_drive(0, 5) == Drive::HiZ);
    // Without FLOW USART0 has no CT/RT pads; USART1 owns them at PRIP0 = 00.
    device.write_sfr(P2DIR, 0x00);
    device.write_sfr(U0UCR, 0x02);
    CHECK(device.pin_drive(0, 4) == Drive::Low);
    // PRIP0 = 10: Timer 1 channels 0-1 first. T1 channel 0 is P0_2; in
    // compare mode (T1CCTL0.MODE = 1) it drives the pad over USART0 RX.
    device.write_sfr(P0SEL, 0x04);
    CHECK(device.pin_drive(0, 2) == Drive::HiZ);
    device.write_sfr(T1CCTL0, 0x04);
    CHECK(device.pin_drive(0, 2) == Drive::HiZ); // USART0 RX still first
    device.write_sfr(P2DIR, 0x80);
    CHECK(device.pin_drive(0, 2) != Drive::HiZ);
    CHECK(device.pin_drive(0, 2) != Drive::PullUp);
    device.write_sfr(T1CCTL0, 0x00);
    device.write_sfr(P2DIR, 0x00);
    device.write_sfr(P0SEL, 0x00);

    // Port 1 conflict, P2SEL.PRI3P1 (7.6.2): alternative 2 of both USARTs,
    // P1_5 = U0 TX (low, START = 1) / U1 RT (high, FLOW, RE = 0).
    device.write_sfr(PERCFG, 0x03);
    device.write_sfr(U0UCR, 0x03);
    device.write_sfr(U1UCR, 0x42);
    device.write_sfr(P1SEL, 0x20);
    CHECK(device.pin_drive(1, 5) == Drive::Low);  // PRI3P1 = 0: USART0
    device.write_sfr(P2SEL, 0x40);
    CHECK(device.pin_drive(1, 5) == Drive::High); // PRI3P1 = 1: USART1
    device.write_sfr(P2SEL, 0x00);

    // SPI master (17.2, Table 7-1 USART1 alt 2: MI P1_7, MO P1_6, C P1_5):
    // SCK idles at CPOL; MISO is an input. Model-defined: MOSI idles low.
    device.write_sfr(PERCFG, 0x02);
    device.write_sfr(U1CSR, 0x00);
    device.write_sfr(P1SEL, 0xe0);
    CHECK(device.pin_drive(1, 5) == Drive::Low);
    device.write_sfr(U1GCR, 0x80); // CPOL = 1
    CHECK(device.pin_drive(1, 5) == Drive::High);
    CHECK(device.pin_drive(1, 6) == Drive::Low);
    CHECK(device.pin_drive(1, 7) == Drive::HiZ);
}

// ============================================================ ADC (12)

struct AdcCase {
    const char *name;
    std::uint8_t adccon3;
    std::uint64_t cycles;   // conversion time
    std::uint16_t expected; // ADCH:ADCL
    std::function<void(Cc2530F256 &)> host;
    bool temperature_sensor = false;
    bool xosc = true;
};

// 12.2.3: writing ADCCON3 starts one conversion; at its end ADCH:ADCL hold
// the result, ADCCON1.EOC = 1 and TCON.ADCIF requests the ADC interrupt
// (12.2.8). The ISR reads ADCL and ADCH (reading ADCH clears EOC); ADCIF is
// cleared by hardware when the CPU vectors (Table 2-5 note).
void run_adc_single(const AdcCase &test) {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x000b).op({0x05, 0x40, 0x85, ADCL, 0x41, 0x85, ADCH, 0x42, 0x32});
    flash.at(0x0100).op({0x75, SP, 0x70});
    if (test.xosc)
        flash.op({0x75, CLKCONCMD, 0x80});
    flash.op({0x75, APCFG, 0xff});
    if (test.temperature_sensor)
        flash.op(movx_write(TR0, 0x01)).op(movx_write(ATEST, 0x01));
    flash.op({0x75, IEN0, 0x82});
    const std::uint16_t start = flash.here();
    flash.op({0x75, ADCCON3, test.adccon3}).op(IdleLoop);

    Cc2530F256 device;
    load(device, flash);
    if (test.host)
        test.host(device);
    const std::uint64_t t0 = run_to_pc(device, start);
    run_to(device, t0 + test.cycles - 1);
    const bool early = (device.read_sfr(ADCCON1) & 0x80) != 0 ||
                       (device.read_sfr(TCON) & TCON_ADCIF) != 0;
    run_to(device, t0 + test.cycles);
    const std::uint16_t result =
        static_cast<std::uint16_t>(device.read_sfr(ADCH) << 8 | device.read_sfr(ADCL));
    const bool done = (device.read_sfr(ADCCON1) & 0x80) && (device.read_sfr(TCON) & TCON_ADCIF);
    run_to(device, t0 + test.cycles + 100);
    const bool isr_ok = device.read_idata(0x40) == 1 &&
                        device.read_idata(0x41) == static_cast<std::uint8_t>(test.expected) &&
                        device.read_idata(0x42) == static_cast<std::uint8_t>(test.expected >> 8) &&
                        (device.read_sfr(TCON) & TCON_ADCIF) == 0 &&
                        (device.read_sfr(ADCCON1) & 0x80) == 0;
    if (early || !done || result != test.expected || !isr_ok) {
        std::cerr << "adc " << test.name << ": early=" << early << " done=" << done
                  << " result=0x" << std::hex << result << " want=0x" << test.expected
                  << std::dec << " isr_ok=" << isr_ok << '\n';
        ++failures;
    }
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 0u); // 12.2.9: no DMA for ADCCON3
}

// Expected codes: two's complement, MSB-aligned in ADCH:ADCL (12.2.5),
// code = floor(Vin / Vref * 2^(bits-1)) with 7/9/10/12 bits for decimation
// 64/128/256/512 (ADCCON3.EDIV). Conversion time 12.2.7: (decimation + 16)
// * 0.25 us = 20/36/68/132 us = 640/1152/2176/4224 cycles. Internal
// reference 1.15 V (SWRS081B).
void test_adc_single_conversions() {
    auto ain = [](std::initializer_list<std::pair<unsigned, std::uint32_t>> pins) {
        std::vector<std::pair<unsigned, std::uint32_t>> list(pins);
        return [list](Cc2530F256 &device) {
            for (const auto &[pin, mv] : list)
                device.set_pin_analog_mv(0, pin, mv);
        };
    };
    const AdcCase cases[] = {
        // 1000 mV / 1150 mV: 7 bits floor(55.65) = 55 -> 0x37 << 9.
        {"int ref 7 bit", 0x00, 640, 0x6e00, ain({{0, 1000}})},
        // 9 bits: floor(222.6) = 222 = 0xDE << 7.
        {"int ref 9 bit", 0x10, 1152, 0x6f00, ain({{0, 1000}})},
        // 10 bits: floor(445.2) = 445 = 0x1BD << 6.
        {"int ref 10 bit", 0x20, 2176, 0x6f40, ain({{0, 1000}})},
        // 12 bits: floor(1780.9) = 1780 = 0x6F4 << 4.
        {"int ref 12 bit", 0x30, 4224, 0x6f40, ain({{0, 1000}})},
        // AVDD5 reference (EREF = 10): 1650 / 3300 * 2048 = 1024.
        {"avdd5 ref", 0xb1, 4224, 0x4000,
         [](Cc2530F256 &device) {
             device.set_vdd_mv(3300);
             device.set_pin_analog_mv(0, 1, 1650);
         }},
        // External reference AIN7 (EREF = 01): 500 / 2000 * 2048 = 512.
        {"ain7 ref", 0x70, 4224, 0x2000, ain({{0, 500}, {7, 2000}})},
        // Differential reference AIN6-AIN7 (EREF = 11): 1500 / 2000 -> 1536.
        {"ain6-ain7 ref", 0xf0, 4224, 0x6000, ain({{0, 1500}, {6, 2500}, {7, 500}})},
        // Channel 8 = AIN0-AIN1: -575 / 1150 * 2048 = -1024 = 0xC00.
        {"differential", 0x38, 4224, 0xc000, ain({{0, 0}, {1, 575}})},
        // Above the reference: saturates at +2047.
        {"clamp", 0x30, 4224, 0x7ff0, ain({{0, 3000}})},
        // Channel 12 = GND.
        {"gnd", 0x3c, 4224, 0x0000, {}},
        // Channel 15 = VDD/3: 3000 mV / 3 = 1000 mV.
        {"vdd/3", 0x3f, 4224, 0x6f40, {}},
        // Channel 14 = temperature sensor (TR0.ADCTM = 1, ATEST = 1, 12.2.1).
        // SWRS081B: 1480 (12-bit) at 25 C, 4.5 / C, 1 / 0.1 V of VDD.
        {"temp 25C", 0x3e, 4224, 0x5c80, {}, true},
        {"temp 35C", 0x3e, 4224, 0x5f50,
         [](Cc2530F256 &device) { device.set_temperature_celsius(35.0); }, true},
        {"temp 25C 3.3V", 0x3e, 4224, 0x5cb0,
         [](Cc2530F256 &device) { device.set_vdd_mv(3300); }, true},
        // Model-defined: without TR0/ATEST the sensor is disconnected (0 V).
        {"temp disconnected", 0x3e, 4224, 0x0000, {}, false},
        // Model-defined: the ADC needs the 32 MHz XOSC (12.2.7); on the
        // 16 MHz RCOSC its timing doubles.
        {"rcosc", 0x00, 1280, 0x6e00, ain({{0, 1000}}), false, false},
    };
    for (const AdcCase &test : cases)
        run_adc_single(test);
}

// 12.2.4 / 12.2.5 / 12.2.9: a sequence converts AIN0..SCH (pads enabled in
// APCFG only), ADCCON2.SCH reads the channel being converted and one more
// than the last at the end; each conversion raises ADC_CHALL and ADC_CHxy,
// no ADC interrupt (that is for ADCCON3 conversions only).
void test_adc_sequence() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x000b).op({0x05, 0x40, 0x32}); // ADC ISR: INC 0x40 (must not run)
    flash.at(0x0100).op({0x75, SP, 0x70, 0x75, CLKCONCMD, 0x80,
                         0x75, APCFG, 0x05,     // AIN0, AIN2
                         0x75, ADCCON2, 0x02,   // internal ref, 7 bit, SCH = 2
                         0x75, IEN0, 0x82});
    const std::uint16_t start = flash.here();
    flash.op({0x75, ADCCON1, 0x73}).op(IdleLoop); // ST = 1, STSEL = 11

    Cc2530F256 device;
    load(device, flash);
    device.set_pin_analog_mv(0, 0, 1000);
    device.set_pin_analog_mv(0, 2, 575);
    const std::uint64_t t0 = run_to_pc(device, start);
    run_to(device, t0 + 639);
    CHECK_EQ(device.read_sfr(ADCCON1), 0x73); // ST reads 1 while running
    CHECK_EQ(device.read_sfr(ADCCON2) & 0x0f, 0);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 0u);
    run_to(device, t0 + 640);
    CHECK_EQ(device.read_sfr(ADCH), 0x6e);
    CHECK_EQ(device.read_sfr(ADCL), 0x00);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 1u);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CH11), 1u);
    CHECK_EQ(device.read_sfr(ADCCON2) & 0x0f, 2); // AIN1 skipped (APCFG[1] = 0)
    run_to(device, t0 + 1279);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 1u);
    run_to(device, t0 + 1280);
    // 575 / 1150 * 64 = 32 = 0x20 << 9.
    CHECK_EQ(device.read_sfr(ADCH), 0x40);
    CHECK_EQ(device.read_sfr(ADCL), 0x00);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 2u);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CH11 + 1), 0u);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CH11 + 2), 1u);
    CHECK_EQ(device.read_sfr(ADCCON2), 0x03);
    CHECK_EQ(device.read_sfr(ADCCON1), 0xb3); // EOC, ST back to 0
    run_to(device, t0 + 3000);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 2u);
    CHECK_EQ(device.read_sfr(TCON) & TCON_ADCIF, 0);
    CHECK_EQ(device.read_idata(0x40), 0);

    // STSEL = 01, full speed: sequences follow each other without trigger.
    const std::uint64_t t1 = cycles(device);
    device.write_sfr(ADCCON1, 0x13);
    run_to(device, t1 + 3 * 1280);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 2u + 6u);
    device.write_sfr(ADCCON1, 0x33); // back to manual: the running sequence ends
    run_to(device, t1 + 6 * 1280);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 2u + 8u);
    CHECK_EQ(device.read_sfr(ADCCON1) & 0x40, 0);

    // STSEL = 00: rising edge on P2_0 (ADCCON1.STSEL).
    device.write_sfr(ADCCON2, 0x00); // SCH = 0: AIN0 only
    device.write_sfr(ADCCON1, 0x03);
    const std::uint64_t t2 = cycles(device);
    device.set_pin_input(2, 0, false); // falling: no start
    run_to(device, t2 + 1000);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 10u);
    device.set_pin_input(2, 0, true);
    run_to(device, t2 + 1000 + 639);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 10u);
    run_to(device, t2 + 1000 + 640);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 11u);

    // STSEL = 10: Timer 1 channel 0 compare event (Timer1 calls
    // Adc::timer1_ch0_event()).
    device.write_sfr(ADCCON1, 0x23);
    const std::uint64_t t3 = cycles(device);
    device.peripherals().adc->timer1_ch0_event();
    run_to(device, t3 + 639);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 11u);
    run_to(device, t3 + 640);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 12u);
    device.write_sfr(ADCCON1, 0x03); // other trigger selected: ignored
    device.peripherals().adc->timer1_ch0_event();
    run_to(device, t3 + 2000);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 12u);
}

// Cross-peripheral: a real Timer 1 channel 0 compare event (SWRU191F 9.8)
// starts the ADC sequence when ADCCON1.STSEL = 10 (12.2.4, ADCCON1). Timer 1
// free-running at TICKSPD = 0, DIV = 1 ticks once per 32 MHz cycle; the model
// starts counting at the instruction that writes T1CTL, so the count reaches
// T1CC0 = 100 at t + 100 and again 65536 ticks later. A 7-bit conversion of
// AIN0 takes (64 + 16) * 8 = 640 cycles (12.2.6).
void test_adc_timer1_compare_trigger() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x0100).op({0x75, SP, 0x70, 0x75, CLKCONCMD, 0x80,
                         0x75, APCFG, 0x01,     // AIN0
                         0x75, ADCCON2, 0x00,   // internal ref, 7 bit, SCH = 0
                         0x75, ADCCON1, 0x23,   // STSEL = 10
                         0x75, T1CC0L, 100, 0x75, T1CC0H, 0x00,
                         0x75, T1CCTL0, 0x04}); // compare mode, IM = 0
    const std::uint16_t start = flash.here();
    flash.op({0x75, T1CTL, 0x01}).op(IdleLoop); // free-running, DIV = 1

    Cc2530F256 device;
    load(device, flash);
    device.set_pin_analog_mv(0, 0, 1000);
    const std::uint64_t t = run_to_pc(device, start);
    run_to(device, t + 99);
    CHECK_EQ(device.read_sfr(T1STAT) & 0x01, 0);
    run_to(device, t + 100);
    CHECK_EQ(device.read_sfr(T1STAT) & 0x01, 1);
    CHECK_EQ(device.read_sfr(ADCCON1) & 0x40, 0x40); // sequence running
    run_to(device, t + 100 + 639);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 0u);
    run_to(device, t + 100 + 640);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 1u);
    CHECK_EQ(device.read_sfr(ADCH), 0x6e); // 1000 / 1150 * 64 = 55 -> 0x6e
    CHECK_EQ(device.read_sfr(ADCCON1) & 0xc0, 0x80); // EOC, ST back to 0
    // Next compare after one full 16-bit period.
    run_to(device, t + 100 + 65536 + 639);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 1u);
    run_to(device, t + 100 + 65536 + 640);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 2u);
    // Another start select: the compare events no longer start sequences.
    device.write_sfr(ADCCON1, 0x33);
    run_to(device, t + 100 + 3 * 65536 + 640);
    CHECK_EQ(Access::adc_dma(device, DMA_ADC_CHALL), 2u);
    CHECK_EQ(device.read_sfr(T1STAT) & 0x01, 1);
}

// ======================================================== RNG (Chapter 14)

// Figure 14-1: 16-bit LFSR X^16 + X^15 + X^2 + 1 (0x8005). Two RNDL writes
// seed it (14.2.2); ADCCON1.RCTRL = 01 clocks it once (13 shifts, 14.2.1)
// and reads 00 afterwards; RNDH writes run the CRC16 of the byte, MSB first
// (14.2.3). Hand computation: 0x0001 << 13 = 0x2000; 0x0008 << 12 = 0x8000,
// the 13th shift feeds back: 0x0000 ^ 0x8005; 0x8003 and 0x0000 are fixed
// points (14.2.1). CRC-16 (poly 0x8005, MSB first) of "123456789": 0xFEE8
// from 0x0000 (CRC-16/BUYPASS check value), 0xAEE7 from 0xFFFF (CRC-16/CMS).
void test_rng() {
    Flash flash;
    flash.at(0x0000);
    auto seed_and_clock = [&](std::uint8_t high, std::uint8_t low, std::uint8_t to) {
        flash.op({0x75, RNDL, high, 0x75, RNDL, low, 0x75, ADCCON1, 0x37, // RCTRL = 01
                  0x85, RNDL, to, 0x85, RNDH, static_cast<std::uint8_t>(to + 1)});
    };
    seed_and_clock(0x00, 0x01, 0x30);
    flash.op({0x85, ADCCON1, 0x32});
    seed_and_clock(0x00, 0x08, 0x33);
    seed_and_clock(0x80, 0x03, 0x35);
    seed_and_clock(0x00, 0x00, 0x37);
    auto crc = [&](std::uint8_t seed, std::uint8_t to) {
        flash.op({0x75, RNDL, seed, 0x75, RNDL, seed});
        for (const char ch : std::string("123456789"))
            flash.op({0x75, RNDH, static_cast<std::uint8_t>(ch)});
        flash.op({0x85, RNDL, to, 0x85, RNDH, static_cast<std::uint8_t>(to + 1)});
    };
    crc(0x00, 0x39);
    crc(0xff, 0x3b);
    // RCTRL = 11: generator stopped. Model-defined: RNDH/RNDL writes ignored.
    flash.op({0x75, ADCCON1, 0x3f, 0x75, RNDH, 0x55, 0x85, RNDL, 0x3d, 0x85, RNDH, 0x3e, 0x85,
              ADCCON1, 0x3f, 0x80, 0xfe});

    Cc2530F256 device;
    load(device, flash);
    CHECK_EQ(device.read_sfr(RNDL), 0xff); // reset 0xFFFF
    CHECK_EQ(device.read_sfr(RNDH), 0xff);
    device.run(200);
    const std::uint8_t expected[] = {0x00, 0x20, 0x33, 0x05, 0x80, 0x03, 0x80, 0x00,
                                     0x00, 0xe8, 0xfe, 0xe7, 0xae, 0xe7, 0xae, 0x3f};
    for (unsigned i = 0; i < sizeof expected; ++i)
        if (device.read_idata(static_cast<std::uint8_t>(0x30 + i)) != expected[i]) {
            std::cerr << "rng: IRAM 0x" << std::hex << 0x30 + i << " = 0x"
                      << +device.read_idata(static_cast<std::uint8_t>(0x30 + i)) << ", want 0x"
                      << +expected[i] << std::dec << '\n';
            ++failures;
        }
}

// ================================================= comparator (Chapter 19)

// 19.1: + input P0_5, - input P0_4; CMPCTL.OUTPUT; the output feeds the
// P0_5 port-interrupt edge detector and can wake the chip from PM2/PM3.
void test_comparator() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x006b).op(port0_isr());
    flash.at(0x0100)
        .op({0x75, SP, 0x70, 0x75, APCFG, 0x30})
        .op(movx_write(CMPCTL, 0x02)) // EN
        .op({0x75, P0IEN, 0x20, 0x75, IEN1, 0x20, 0xd2, BIT_EA, 0x75, SLEEPCMD, 0xff, // PM3
             0x75, PCON, 0x01, 0x05, 0x42, 0x80, 0xfe});
    Cc2530F256 device;
    load(device, flash);
    CHECK_EQ(device.peek_xdata(CMPCTL), 0x00);
    device.set_pin_analog_mv(0, 4, 1000);
    device.set_pin_analog_mv(0, 5, 800);
    Cc2530RunResult result = device.run(100);
    CHECK(result.sleeping);
    CHECK(device.power_mode() == Cc2530PowerMode::Pm3);
    CHECK_EQ(device.peek_xdata(CMPCTL), 0x02);
    CHECK_EQ(device.read_sfr(P0IFG), 0x00);
    const std::uint64_t c = cycles(device);
    run_to(device, c + 5000);
    device.set_pin_analog_mv(0, 5, 1200); // P0_5 > P0_4: OUTPUT rises
    CHECK_EQ(device.peek_xdata(CMPCTL), 0x03);
    CHECK_EQ(device.read_sfr(P0IFG), 0x20);
    result = device.run(100, c + 5100);
    CHECK(!result.sleeping);
    CHECK_EQ(device.read_idata(0x42), 1);
    CHECK_EQ(device.read_idata(0x40), 1);
    CHECK_EQ(device.read_idata(0x41), 0x20);
    CHECK_EQ(device.diagnostics().power_mode_cycles_by_mode[4], 5000u);
    device.set_pin_analog_mv(0, 5, 999); // falling: not selected
    CHECK_EQ(device.peek_xdata(CMPCTL), 0x02);
    CHECK_EQ(device.read_sfr(P0IFG), 0x00);
    device.set_pin_analog_mv(0, 4, 900);
    CHECK_EQ(device.peek_xdata(CMPCTL), 0x03);
}

// ===================================================== op-amp (Chapter 18)

// 18.2: OPAMPC.CAL starts a calibration, OPAMPS.CAL_BUSY is 1 until it
// completes. Model-defined: calibration lasts 4096 cycles; the amplifier is
// a voltage follower of P0_0 limited to 0.07 V .. VDD - 0.07 V (SWRS081B),
// driving P0_2 (AIN2) while enabled and not calibrating.
void test_opamp() {
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80, 0x75, APCFG, 0x07}).op(movx_write(OPAMPMC, 0x01));
    const std::uint16_t cal = static_cast<std::uint16_t>(flash.here() + 5); // the MOVX
    flash.op(movx_write(OPAMPC, 0x03)).op(IdleLoop);
    Cc2530F256 device;
    load(device, flash);
    auto &opamp = *device.peripherals().opamp;
    device.set_pin_analog_mv(0, 0, 500);
    const std::uint64_t t0 = run_to_pc(device, cal);
    CHECK(!opamp.output_mv());
    CHECK_EQ(device.diagnostics().unmodeled_feature_writes.count(OPAMPMC), 1u);
    device.run(1);
    CHECK_EQ(device.peek_xdata(OPAMPC), 0x01); // CAL reads 0
    CHECK_EQ(device.peek_xdata(OPAMPS), 0x01);
    CHECK(!opamp.output_mv());
    run_to(device, t0 + 4095);
    CHECK_EQ(device.peek_xdata(OPAMPS), 0x01);
    run_to(device, t0 + 4096);
    CHECK_EQ(device.peek_xdata(OPAMPS), 0x00);
    CHECK(opamp.output_mv() == std::optional<std::uint32_t>(500));

    // The ADC sees the output on AIN2: 500 / 1150 * 2048 = 890 = 0x37A.
    device.write_sfr(ADCCON3, 0x32);
    const std::uint64_t t1 = cycles(device);
    run_to(device, t1 + 4224);
    CHECK_EQ(device.read_sfr(ADCH), 0x37);
    CHECK_EQ(device.read_sfr(ADCL), 0xa0);

    device.set_pin_analog_mv(0, 0, 20);
    CHECK(opamp.output_mv() == std::optional<std::uint32_t>(70));
    device.set_pin_analog_mv(0, 0, 3000);
    CHECK(opamp.output_mv() == std::optional<std::uint32_t>(2930));
}

// ============================================== USART UART (Chapter 17)

// 17.1.1: writing UxDBUF starts the frame; UxDBUF is double-buffered, so a
// second byte waits and follows directly. 17.6/17.7: UTXxIF and the UTXx
// DMA trigger when the byte moves to the shift register ("immediately
// after the transmission has been started"); TX_BYTE when it is out.
void run_uart_tx(std::uint8_t baud, std::uint8_t gcr, std::uint8_t ucr, std::uint64_t frame1,
                 std::uint64_t frame2, const char *name) {
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80, 0x75, U0CSR, 0x80, 0x75, U0BAUD, baud, 0x75,
                         U0GCR, gcr, 0x75, U0UCR, ucr});
    const std::uint16_t tx = flash.here();
    flash.op({0x75, U0DBUF, 0x55, 0x75, U0DBUF, 0xa3}).op(IdleLoop);
    Cc2530F256 device;
    load(device, flash);
    const std::uint64_t t0 = run_to_pc(device, tx);
    device.run(1);
    const bool started = (device.read_sfr(IRCON2) & IRCON2_UTX0IF) &&
                         device.read_sfr(U0CSR) == 0x81 &&
                         Access::usart_dma(device, 0, DMA_UTX0) == 1;
    device.write_sfr(IRCON2, 0x00);
    run_to(device, t0 + frame1 - 1);
    const bool early = !device.uart_take_transmitted(0).empty() ||
                       (device.read_sfr(IRCON2) & IRCON2_UTX0IF) || device.read_sfr(U0CSR) != 0x81;
    run_to(device, t0 + frame1);
    const auto first = device.uart_take_transmitted(0);
    const bool second_started = (device.read_sfr(IRCON2) & IRCON2_UTX0IF) &&
                                device.read_sfr(U0CSR) == 0x83 &&
                                Access::usart_dma(device, 0, DMA_UTX0) == 2;
    run_to(device, t0 + frame2 - 1);
    const bool early2 = !device.uart_take_transmitted(0).empty();
    run_to(device, t0 + frame2);
    const auto second = device.uart_take_transmitted(0);
    if (!started || early || first != std::vector<std::uint8_t>{0x55} || !second_started ||
        early2 || second != std::vector<std::uint8_t>{0xa3} || device.read_sfr(U0CSR) != 0x82) {
        std::cerr << "uart tx " << name << ": started=" << started << " early=" << early
                  << " first=" << first.size() << " second_started=" << second_started
                  << " early2=" << early2 << " second=" << second.size() << " csr=0x" << std::hex
                  << +device.read_sfr(U0CSR) << std::dec << '\n';
        ++failures;
    }
}

void test_uart_tx_timing() {
    run_uart_tx(216, 11, 0x02, B115200_10, B115200_20, "115200 8N1");
    run_uart_tx(59, 8, 0x02, B9600_10, B9600_20, "9600 8N1");
    // BIT9 + PARITY + SPB: 1 + 9 + 2 = 12 bits per frame.
    run_uart_tx(216, 11, 0x1e, B115200_12, B115200_24, "115200 9-bit 2 stop");
}

// Host-side UART configuration with the CPU waiting in Idle.
void idle_uart_device(Cc2530F256 &device, std::uint8_t baud, std::uint8_t gcr) {
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80}).op(IdleLoop);
    load(device, flash);
    device.run(100);
    CHECK(device.sleeping());
    device.write_sfr(U0BAUD, baud);
    device.write_sfr(U0GCR, gcr);
}

// 17.1.2: with UxCSR.RE = 1 a received byte sets RX_BYTE and URXxIF/URXx
// at the end of the first stop bit; ACTIVE while receiving. Host bytes are
// received back to back.
void test_uart_rx_timing() {
    for (const auto &[baud, gcr, frame] : {std::tuple{std::uint8_t{216}, std::uint8_t{11}, B115200_10},
                                           std::tuple{std::uint8_t{59}, std::uint8_t{8}, B9600_10}}) {
        Cc2530F256 device;
        idle_uart_device(device, baud, gcr);
        CHECK(!device.uart_can_receive(0)); // RE = 0
        device.write_sfr(U0CSR, 0xc0);
        CHECK(device.uart_can_receive(0));
        const std::uint64_t c = cycles(device);
        device.uart_receive(0, 0x41);
        device.uart_receive(0, 0x42);
        run_to(device, c + frame - 1);
        CHECK_EQ(device.read_sfr(U0CSR), 0xc1); // ACTIVE
        CHECK_EQ(device.read_sfr(TCON) & TCON_URX0IF, 0);
        run_to(device, c + frame);
        CHECK_EQ(device.read_sfr(U0CSR) & 0x04, 0x04);
        CHECK_EQ(device.read_sfr(TCON) & TCON_URX0IF, TCON_URX0IF);
        CHECK_EQ(device.read_sfr(U0DBUF), 0x41);
        CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 1u);
        run_to(device, c + 2 * frame + 10);
        CHECK_EQ(device.read_sfr(U0CSR), 0xc4);
    }
}

// Overrun: the second byte completes while RX_BYTE = 1 and waits in the
// shift register; the third replaces it (model-defined; SWRU191F defines no
// overrun flag). Reading UxDBUF clears RX_BYTE (17.1.2) and moves the
// waiting byte in.
void test_uart_rx_overrun() {
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80, 0x75, U0BAUD, 216, 0x75, U0GCR, 11, 0x75, U0CSR,
                         0xc0,
                         0xe5, 0x50, 0x60, 0xfc,  // wait: MOV A,0x50; JZ wait
                         0xe5, U0DBUF, 0xf5, 0x30, 0x85, U0CSR, 0x31,
                         0xe5, U0DBUF, 0xf5, 0x32, 0x85, U0CSR, 0x33, 0x80, 0xfe});
    Cc2530F256 device;
    load(device, flash);
    device.run(1000, 20);
    const std::uint64_t c = cycles(device);
    for (const std::uint8_t byte : {0x11, 0x22, 0x33})
        device.uart_receive(0, byte);
    run_to(device, c + B115200_30 + 10);
    CHECK_EQ(device.read_sfr(U0DBUF), 0x11);
    CHECK_EQ(device.read_sfr(U0CSR) & 0x1c, 0x04); // RX_BYTE, no FE/ERR
    device.write_sram(Cc2530F256::IdataXdataAlias + 0x50, 1);
    run_to(device, cycles(device) + 100);
    CHECK_EQ(device.read_idata(0x30), 0x11);
    CHECK_EQ(device.read_idata(0x31), 0xc4);
    CHECK_EQ(device.read_idata(0x32), 0x33);
    CHECK_EQ(device.read_idata(0x33), 0xc0);
    CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 2u);
}

// URX0 ISR (vector 0x13) stores the received bytes; TCON.URX0IF is cleared
// by hardware on vector entry (Table 2-5 note).
void test_uart_rx_interrupt() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x0013).op({0xa6, U0DBUF, 0x08, 0x32}); // MOV @R0,U0DBUF; INC R0; RETI
    flash.at(0x0100)
        .op({0x75, SP, 0x70, 0x75, CLKCONCMD, 0x80, 0x75, U0BAUD, 216, 0x75, U0GCR, 11, 0x75,
             U0CSR, 0xc0, 0x78, 0x30, 0x75, IEN0, 0x84})
        .op(IdleLoop);
    Cc2530F256 device;
    load(device, flash);
    device.run(100);
    CHECK(device.sleeping());
    const std::uint64_t c = cycles(device);
    for (const char ch : std::string("ABC"))
        device.uart_receive(0, static_cast<std::uint8_t>(ch));
    run_to(device, c + B115200_10 - 1);
    CHECK_EQ(device.read_idata(0x30), 0);
    run_to(device, c + B115200_10);
    CHECK_EQ(device.read_sfr(TCON) & TCON_URX0IF, TCON_URX0IF);
    run_to(device, c + B115200_30 + 100);
    CHECK_EQ(device.read_idata(0x30), 'A');
    CHECK_EQ(device.read_idata(0x31), 'B');
    CHECK_EQ(device.read_idata(0x32), 'C');
    CHECK_EQ(device.read_idata(0x00), 0x33); // R0
    CHECK_EQ(device.read_sfr(TCON) & TCON_URX0IF, 0);
    CHECK_EQ(device.read_sfr(U0CSR), 0xc0);
    CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 3u);
}

// UTX0 ISR (vector 0x3B) feeding the next byte: it runs when a byte moves
// into the shift register, so the next one can be written at once.
void test_uart_tx_interrupt_chain() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x003b).op({0x02, 0x02, 0x00});
    flash.at(0x0200).op({0xc2, BIT_UTX0IF,       // CLR UTX0IF
                         0x05, 0x51,             // INC 0x51
                         0xd5, 0x50, 0x01,       // DJNZ 0x50,send
                         0x32,                   // RETI
                         0x85, 0x52, U0DBUF,     // send: MOV U0DBUF,0x52
                         0x05, 0x52, 0x32});     // INC 0x52; RETI
    flash.at(0x0100).op({0x75, SP, 0x70, 0x75, CLKCONCMD, 0x80, 0x75, U0CSR, 0x80, 0x75, U0BAUD,
                         216, 0x75, U0GCR, 11, 0x75, 0x50, 0x02, 0x75, 0x52, 0x61, 0x75, IEN2,
                         0x04, 0xd2, BIT_EA});
    const std::uint16_t tx = flash.here();
    flash.op({0x75, U0DBUF, 0x60}).op(IdleLoop);
    Cc2530F256 device;
    load(device, flash);
    const std::uint64_t t0 = run_to_pc(device, tx);
    run_to(device, t0 + B115200_10);
    CHECK(device.uart_take_transmitted(0) == std::vector<std::uint8_t>{0x60});
    run_to(device, t0 + B115200_20 - 1);
    CHECK(device.uart_take_transmitted(0).empty());
    run_to(device, t0 + B115200_20);
    CHECK(device.uart_take_transmitted(0) == std::vector<std::uint8_t>{0x61});
    run_to(device, t0 + 3 * B115200_10 + 1000);
    CHECK(device.uart_take_transmitted(0).empty());
    CHECK_EQ(device.read_idata(0x51), 2);
    CHECK_EQ(device.read_idata(0x52), 0x62);
}

// 17.1.3 flow control: TX waits for CTS low; RTS is low while the receive
// register is empty and RE = 1, and the host sends only while RTS is low.
void test_uart_flow_control() {
    Cc2530F256 device;
    idle_uart_device(device, 216, 11);
    device.write_sfr(U0UCR, 0x42); // FLOW, 1 stop bit
    device.write_sfr(U0CSR, 0xc0);
    device.write_sfr(P0SEL, 0x3c); // alt 1: RX P0_2, TX P0_3, CT P0_4, RT P0_5
    CHECK(device.pin_drive(0, 5) == Drive::Low);  // RTS asserted
    CHECK(device.pin_drive(0, 4) == Drive::HiZ);  // CTS input, floating = 1
    const std::uint64_t c = cycles(device);
    device.write_sfr(U0DBUF, 0x77);
    run_to(device, c + 5000);
    CHECK(device.uart_take_transmitted(0).empty());
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_UTX0IF, 0);
    device.set_pin_input(0, 4, false); // CTS low at c + 5000
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_UTX0IF, IRCON2_UTX0IF);
    run_to(device, c + 5000 + B115200_10 - 1);
    CHECK(device.uart_take_transmitted(0).empty());
    run_to(device, c + 5000 + B115200_10);
    CHECK(device.uart_take_transmitted(0) == std::vector<std::uint8_t>{0x77});

    const std::uint64_t r = cycles(device);
    device.uart_receive(0, 0x01);
    device.uart_receive(0, 0x02);
    run_to(device, r + B115200_10);
    CHECK_EQ(device.read_sfr(U0DBUF), 0x01);
    CHECK(device.pin_drive(0, 5) == Drive::High); // RTS deasserted
    run_to(device, r + 20000);
    CHECK_EQ(device.read_sfr(U0DBUF), 0x01); // the host holds byte 2 back
    CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 1u);
    device.write_sfr(U0CSR, 0xc0); // clear RX_BYTE
    CHECK(device.pin_drive(0, 5) == Drive::Low);
    run_to(device, r + 20000 + B115200_10 - 1);
    CHECK_EQ(device.read_sfr(U0CSR) & 0x04, 0);
    run_to(device, r + 20000 + B115200_10);
    CHECK_EQ(device.read_sfr(U0DBUF), 0x02);
    CHECK(device.pin_drive(0, 5) == Drive::High);
}

// The HF clock stops in PM1-PM3 (Table 4-1): a host byte queued while the
// CPU sleeps in PM1 is received after wake, one frame of running HF time
// later.
void test_uart_frozen_in_pm1() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x006b).op({0x75, SLEEPCMD, 0xfc, 0x75, P0IFG, 0x00, 0xc2, BIT_P0IF, 0x32});
    flash.at(0x0100).op({0x75, SP, 0x70, 0x75, CLKCONCMD, 0x80, 0x75, U0BAUD, 216, 0x75, U0GCR,
                         11, 0x75, U0CSR, 0xc0, 0x75, P0IEN, 0x02, 0x75, IEN1, 0x20, 0xd2,
                         BIT_EA, 0x75, SLEEPCMD, 0xfd})
        .op(IdleLoop);
    Cc2530F256 device;
    load(device, flash);
    device.run(100);
    CHECK(device.power_mode() == Cc2530PowerMode::Pm1);
    const std::uint64_t c = cycles(device);
    CHECK(device.uart_can_receive(0));
    device.uart_receive(0, 0x5a);
    run_to(device, c + 10000);
    CHECK_EQ(cycles(device), c + 10000);
    CHECK_EQ(device.read_sfr(U0CSR) & 0x04, 0);
    device.set_pin_input(0, 1, false);
    device.set_pin_input(0, 1, true); // wake at c + 10000; the ISR selects Idle
    run_to(device, c + 10000 + B115200_10 - 1);
    CHECK(device.power_mode() == Cc2530PowerMode::Idle);
    CHECK_EQ(device.read_sfr(U0CSR) & 0x04, 0);
    run_to(device, c + 10000 + B115200_10);
    CHECK_EQ(device.read_sfr(U0CSR) & 0x04, 0x04);
    CHECK_EQ(device.read_sfr(U0DBUF), 0x5a);
    CHECK_EQ(device.diagnostics().power_mode_cycles_by_mode[2], 10000u);
}

// FLUSH (17.5), ORDER (UxGCR), receiver enable and unmodeled SPI slave.
void test_uart_misc() {
    {
        Cc2530F256 device;
        idle_uart_device(device, 216, 11);
        device.write_sfr(U0CSR, 0x80);
        const std::uint64_t c = cycles(device);
        device.write_sfr(U0DBUF, 0x12);
        run_to(device, c + 1000);
        CHECK_EQ(device.read_sfr(U0CSR), 0x81);
        device.write_sfr(U0UCR, 0x82); // FLUSH: abort
        CHECK_EQ(device.read_sfr(U0UCR), 0x02);
        CHECK_EQ(device.read_sfr(U0CSR), 0x80);
        run_to(device, c + 10000);
        CHECK(device.uart_take_transmitted(0).empty());
        CHECK_EQ(device.read_sfr(U0CSR), 0x80);
    }
    {
        // Model-defined: host bytes are LSB-first wire bytes; ORDER = 1
        // (MSB first) bit-reverses them.
        Cc2530F256 device;
        idle_uart_device(device, 216, 0x20 | 11);
        device.write_sfr(U0CSR, 0xc0);
        const std::uint64_t c = cycles(device);
        device.write_sfr(U0DBUF, 0x01);
        device.uart_receive(0, 0x03);
        run_to(device, c + B115200_10);
        CHECK(device.uart_take_transmitted(0) == std::vector<std::uint8_t>{0x80});
        CHECK_EQ(device.read_sfr(U0DBUF), 0xc0);
    }
    {
        // A byte offered while RE = 0 (or in SPI mode) is lost.
        Cc2530F256 device;
        idle_uart_device(device, 216, 11);
        device.write_sfr(U0CSR, 0x80);
        CHECK(!device.uart_can_receive(0));
        device.uart_receive(0, 0x99);
        device.write_sfr(U0CSR, 0xc0);
        const std::uint64_t c = cycles(device);
        run_to(device, c + 5000);
        CHECK_EQ(device.read_sfr(U0CSR), 0xc0);
        device.write_sfr(U0CSR, 0x40); // SPI master
        CHECK(!device.uart_can_receive(0));
        // SPI slave mode is modeled (test_cc2530_spi.cpp): no unmodeled note.
        device.write_sfr(U0CSR, 0x20);
        CHECK(device.diagnostics().unmodeled_feature_writes.count(0x7000 | U0CSR) == 0);
    }
}

// =============================================== USART SPI master (17.2)

// 17.2.1: writing UxDBUF starts the transfer; SCK = baud-rate generator
// (BAUD_M = 0, BAUD_E = 17: 256 * 2^17 / 2^28 * 32 MHz = 4 MHz, 8 cycles
// per bit, 64 per byte). The received byte is in UxDBUF at the end with
// TX_BYTE and URXxIF (17.6); UTXxIF when the byte enters the shift register.
void test_spi_master() {
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80, 0x75, U1BAUD, 0x00, 0x75, U1GCR, 0x31});
    const std::uint16_t xfer = flash.here();
    flash.op({0x75, U1DBUF, 0xa5}).op(IdleLoop);
    Cc2530F256 device;
    load(device, flash);
    std::vector<std::uint8_t> mosi;
    std::uint8_t miso = 0x3c;
    device.set_spi_device(1, [&](std::uint8_t byte) {
        mosi.push_back(byte);
        return miso;
    });
    const std::uint64_t t0 = run_to_pc(device, xfer);
    device.run(1);
    CHECK_EQ(device.read_sfr(IRCON2) & IRCON2_UTX1IF, IRCON2_UTX1IF);
    CHECK_EQ(device.read_sfr(U1CSR), 0x01); // ACTIVE
    run_to(device, t0 + 63);
    CHECK(mosi.empty());
    CHECK_EQ(device.read_sfr(TCON) & TCON_URX1IF, 0);
    run_to(device, t0 + 64);
    CHECK(mosi == std::vector<std::uint8_t>{0xa5});
    CHECK_EQ(device.read_sfr(U1DBUF), 0x3c);
    CHECK_EQ(device.read_sfr(U1CSR), 0x02); // TX_BYTE
    CHECK_EQ(device.read_sfr(TCON) & TCON_URX1IF, TCON_URX1IF);
    CHECK_EQ(Access::usart_dma(device, 1, DMA_UTX1), 1u);
    CHECK_EQ(Access::usart_dma(device, 1, DMA_URX1), 1u);

    // ORDER = 0: LSB first. The device sees the wire order (first bit = MSB
    // of the device byte, model-defined convention).
    mosi.clear();
    miso = 0xc0;
    device.write_sfr(U1GCR, 0x11);
    const std::uint64_t t1 = cycles(device);
    device.write_sfr(U1DBUF, 0x01);
    run_to(device, t1 + 64);
    CHECK(mosi == std::vector<std::uint8_t>{0x80});
    CHECK_EQ(device.read_sfr(U1DBUF), 0x03);

    // No device: MISO idles high.
    device.set_spi_device(1, {});
    device.write_sfr(U1GCR, 0x31);
    const std::uint64_t t2 = cycles(device);
    device.write_sfr(U1DBUF, 0x00);
    run_to(device, t2 + 64);
    CHECK_EQ(device.read_sfr(U1DBUF), 0xff);
    CHECK(device.uart_take_transmitted(1).empty()); // SPI bytes are not UART output
}

// ============================================ fast / step / trace modes

void test_modes_port_interrupt() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x007b).op({0x05, 0x40, 0x75, P1IFG, 0x00, 0xc2, BIT_P1IF, 0x32});
    flash.at(0x0100).op({0x75, SP, 0x70, 0x75, P1DIR, 0x01, 0x75, P1IEN, 0x01, 0x75, IEN2, 0x10,
                         0xd2, BIT_EA, 0x75, 0x50, 0x04,
                         0xb2, BIT_P1_0,          // loop: CPL P1.0
                         0xd5, 0x50, 0xfb,        // DJNZ 0x50,loop
                         0x80, 0xfe});
    for_each_mode(
        flash, 2000, [](Cc2530F256 &) {},
        [](Cc2530F256 &device, Mode) {
            CHECK_EQ(device.read_idata(0x40), 2); // two rising edges
            CHECK_EQ(Access::io_dma(device, DMA_IOC_1), 2u);
        },
        "port interrupt modes");
}

void test_modes_uart_rx() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x0013).op({0xa6, U0DBUF, 0x08, 0x32});
    flash.at(0x0100).op({0x75, SP, 0x70, 0x78, 0x30, 0x75, IEN0, 0x84, 0x80, 0xfe});
    for_each_mode(
        flash, 6000,
        [](Cc2530F256 &device) {
            device.write_sfr(CLKCONCMD, 0x80);
            device.write_sfr(U0BAUD, 216);
            device.write_sfr(U0GCR, 11);
            device.write_sfr(U0CSR, 0xc0);
            device.uart_receive(0, 'x');
            device.uart_receive(0, 'y');
        },
        [](Cc2530F256 &device, Mode) {
            CHECK_EQ(device.read_sfr(CLKCONSTA) & 0x40, 0);
            CHECK_EQ(device.read_idata(0x30), 'x');
            CHECK_EQ(device.read_idata(0x31), 'y');
            CHECK_EQ(Access::usart_dma(device, 0, DMA_URX0), 2u);
        },
        "uart rx modes");
}

void test_modes_adc() {
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80, 0x75, ADCCON3, 0x30,
                         0xe5, ADCCON1, 0x30, 0xe7, 0xfb, // poll: MOV A,ADCCON1; JNB ACC.7
                         0x85, ADCL, 0x30, 0x85, ADCH, 0x31, 0x85, ADCCON1, 0x32, 0x80, 0xfe});
    for_each_mode(
        flash, 6000, [](Cc2530F256 &device) { device.set_pin_analog_mv(0, 0, 1000); },
        [](Cc2530F256 &device, Mode) {
            CHECK_EQ(device.read_idata(0x30), 0x40);
            CHECK_EQ(device.read_idata(0x31), 0x6f);
            CHECK_EQ(device.read_idata(0x32), 0x33); // EOC cleared by the ADCH read
        },
        "adc modes");
}

} // namespace

int main() {
    test_gpio_direction_pulls_and_listener();
    test_pmux_clock_and_regulator_outputs();
    test_port0_interrupt_edges_and_mask();
    test_port0_flag_clearing_order();
    test_port1_half_port_edges_and_order();
    test_port2_interrupt();
    test_port_wake_from_power_modes();
    test_peripheral_pin_mapping();
    test_adc_single_conversions();
    test_adc_sequence();
    test_adc_timer1_compare_trigger();
    test_rng();
    test_comparator();
    test_opamp();
    test_uart_tx_timing();
    test_uart_rx_timing();
    test_uart_rx_overrun();
    test_uart_rx_interrupt();
    test_uart_tx_interrupt_chain();
    test_uart_flow_control();
    test_uart_frozen_in_pm1();
    test_uart_misc();
    test_spi_master();
    test_modes_port_interrupt();
    test_modes_uart_rx();
    test_modes_adc();
    return finish("cc2530 io_analog_usart");
}
